#include "CollisionTerrain.hh"
#include "PersistentTerrain.hh"
#include "TerrainTypes.hh"
#include "TileStore.hh"
#include "GuiTerrain.hh"

#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SphericalCoordinates.hh>

#include <sdf/Root.hh>

#include <curl/curl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dynamic_terrain
{
    namespace
    {
        struct VisualRequest
        {
            TileKey center;
            std::uint64_t generation{0};
        };

        struct RefinementRequest
        {
            std::shared_ptr<TerrainSnapshot> snapshot;
            std::uint64_t generation{0};
        };

        struct CollisionRequest
        {
            TileKey center;
            std::uint64_t generation{0};
        };

        struct CollisionResult
        {
            CollisionRequest request;
            std::optional<CollisionPatch> patch;
            std::string error;
        };

        std::int64_t steadyMilliseconds()
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
        }

        class DynamicTerrainSystem final
            : public gz::sim::System,
              public gz::sim::ISystemConfigure,
              public gz::sim::ISystemPreUpdate
        {
        public:
            DynamicTerrainSystem()
            {
                static std::once_flag curlInitFlag;
                std::call_once(curlInitFlag, []
                               { curl_global_init(CURL_GLOBAL_DEFAULT); });
            }

            ~DynamicTerrainSystem() override
            {
                stop_.store(true, std::memory_order_relaxed);
                visualCv_.notify_all();
                refineCv_.notify_all();
                collisionCv_.notify_all();
                if (visualThread_.joinable())
                    visualThread_.join();
                if (refineThread_.joinable())
                    refineThread_.join();
                if (collisionThread_.joinable())
                    collisionThread_.join();
                renderer_.reset();
            }

            void Configure(const gz::sim::Entity &entity,
                           const std::shared_ptr<const sdf::Element> &,
                           gz::sim::EntityComponentManager &ecm,
                           gz::sim::EventManager &eventManager) override
            {
                worldEntity_ = entity;
                eventManager_ = &eventManager;
                const auto *sphericalComponent =
                    ecm.Component<gz::sim::components::SphericalCoordinates>(worldEntity_);
                if (!sphericalComponent)
                {
                    logError("[DynamicTerrain][STARTUP] world has no <spherical_coordinates>; plugin disabled");
                    disabled_ = true;
                    return;
                }
                spherical_ = sphericalComponent->Data();
                logInfo("[DynamicTerrain][STARTUP] v0.19.0 dynamic terrain renderer enabled",
                        " worldEntity=", worldEntity_,
                        " elevation_ref=", spherical_->ElevationReference());
                logInfo("[DynamicTerrain][STARTUP] visual terrain will NOT create SDF heightmap tiles");
                logInfo("[DynamicTerrain][STARTUP] waiting for model-side custom::DynamicTerrainConfig");
            }

            void PreUpdate(const gz::sim::UpdateInfo &info,
                           gz::sim::EntityComponentManager &ecm) override
            {
                if (disabled_ || !eventManager_ || !spherical_)
                    return;

                const double simTime = std::chrono::duration<double>(info.simTime).count();
                ensureStartupSafetyGround(ecm);

                if (!configured_)
                {
                    if (!tryBindModelConfiguration(ecm))
                    {
                        periodicStatus("waiting-for-model-config");
                        return;
                    }
                }

                applyCollisionResult(simTime, ecm);
                retireOldCollision(simTime, ecm);
                updateStartupSafetyGround(simTime, ecm);

                if (info.paused)
                    return;
                if (simTime - lastUpdateTime_ < cfg_.updatePeriodSec)
                    return;
                lastUpdateTime_ = simTime;

                if (trackedEntity_ == gz::sim::kNullEntity ||
                    !ecm.Component<gz::sim::components::Model>(trackedEntity_))
                {
                    periodicStatus("tracked-model-missing");
                    return;
                }

                const auto pose = gz::sim::worldPose(trackedEntity_, ecm);
                const auto geo = gz::sim::sphericalCoordinates(trackedEntity_, ecm);
                if (!geo)
                {
                    periodicStatus("spherical-position-unavailable");
                    return;
                }

                updateVisualRequest(pose.Pos(), geo->X(), geo->Y());
                updateCollisionRequest(pose.Pos().Z(), geo->X(), geo->Y());
                periodicStatus("running");
            }

        private:
            bool tryBindModelConfiguration(gz::sim::EntityComponentManager &ecm)
            {
                for (auto registration : registeredModelConfigs())
                {
                    if (!ecm.Component<gz::sim::components::Model>(registration.entity))
                        continue;

                    trackedEntity_ = registration.entity;
                    cfg_ = std::move(registration.config);
                    if (const auto *name = ecm.Component<gz::sim::components::Name>(trackedEntity_))
                        cfg_.modelName = name->Data();
                    if (cfg_.modelName.empty())
                        cfg_.modelName = "entity_" + std::to_string(trackedEntity_);

                    store_ = std::make_shared<TileStore>(cfg_, *spherical_);
                    visualBuilder_ = std::make_unique<PersistentTerrainBuilder>(store_);
                    collisionBuilder_ = std::make_unique<CollisionTerrainBuilder>(store_);
                    renderer_ = std::make_unique<PersistentTerrainRenderer>(cfg_, *eventManager_);
                    if (cfg_.visualGui)
                    {
                        const auto *worldName = ecm.Component<gz::sim::components::Name>(worldEntity_);
                        const std::string service = "/world/" +
                                                    (worldName ? worldName->Data() : std::to_string(worldEntity_)) +
                                                    "/model/" + cfg_.modelName + "/terrain_gui";
                        try
                        {
                            guiSource_ = std::make_unique<GuiTerrainSource>(service);
                            logInfo("[DynamicTerrain][GUI] on-demand preview enabled service=", service);
                        }
                        catch (const std::exception &e)
                        {
                            logError("[DynamicTerrain][GUI] preview unavailable: ", e.what());
                        }
                    }

                    visualThread_ = std::thread([this]
                                                { visualWorker(); });
                    refineThread_ = std::thread([this]
                                                { refinementWorker(); });
                    collisionThread_ = std::thread([this]
                                                   { collisionWorker(); });
                    configured_ = true;

                    if (!cfg_.startupSafetyGround && startupSafetyEntity_ != gz::sim::kNullEntity)
                    {
                        gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                        creator.RequestRemoveEntity(startupSafetyEntity_, true);
                        startupSafetyEntity_ = gz::sim::kNullEntity;
                        startupSafetyRemoved_ = true;
                    }

                    const auto provider = resolveImageryProvider(cfg_);
                    logInfo("[DynamicTerrain][CONFIG] bound model='", cfg_.modelName,
                            "' entity=", trackedEntity_,
                            " imagery=", provider.name,
                            " elevation=", cfg_.elevationProvider);
                    logInfo("[DynamicTerrain][CONFIG] persistent visual radius=", cfg_.visualRadiusM,
                            "m geometry_z=", cfg_.visualGeometryZoom,
                            " dem_z=", cfg_.visualElevationZoom,
                            " cells_per_tile=", cfg_.visualMeshCellsPerTile,
                            " page_texture_max=", cfg_.visualPageTextureMaxSize,
                            " page_cache_mb=", cfg_.visualPageCacheMb,
                            " bootstrap_z=", cfg_.visualBootstrapImageryZoom,
                            " recenter=", cfg_.visualRecenterDistanceM, "m",
                            " frustum_eviction=",
                            cfg_.visualFrustumEviction ? "on" : "off",
                            " offscreen_frames=", cfg_.visualOffscreenFrames,
                            " texture_guard=", cfg_.visualTextureGuardM, "m",
                            " detail_mode=", cfg_.visualDetailMode,
                            " detail_camera=", cfg_.visualDetailCameraName,
                            " detail_radius=", cfg_.visualDetailRadiusM, "m",
                            " detail_zoom=", cfg_.visualDetailZoom,
                            " recenter_ready_z=", cfg_.visualRecenterReadyZoom,
                            " refine_batch_source_tiles=",
                            cfg_.visualRefineMaxSourceTilesPerBatch,
                            " lighting=", cfg_.visualLightingEnabled ? "on" : "off");
                    logInfo("[DynamicTerrain][CONFIG] downloader concurrency=",
                            cfg_.downloadConcurrency,
                            " per_host=", cfg_.downloadPerHost,
                            " retries=", cfg_.downloadRetries,
                            " no_started_request_cancellation=true");

                    if (cfg_.startupPreload)
                    {
                        if (const auto geo = gz::sim::sphericalCoordinates(trackedEntity_, ecm))
                        {
                            const auto pose = gz::sim::worldPose(trackedEntity_, ecm);
                            updateVisualRequest(pose.Pos(), geo->X(), geo->Y(), true);
                            updateCollisionRequest(pose.Pos().Z(), geo->X(), geo->Y(), true);
                        }
                        else
                        {
                            const double lat = spherical_->LatitudeReference().Degree();
                            const double lon = spherical_->LongitudeReference().Degree();
                            updateVisualRequest(gz::math::Vector3d::Zero, lat, lon, true);
                            updateCollisionRequest(0.0, lat, lon, true);
                        }
                    }
                    return true;
                }
                return false;
            }

            void updateVisualRequest(const gz::math::Vector3d &vehicleWorld,
                                     double latDeg, double lonDeg,
                                     bool force = false)
            {
                if (!visualBuilder_)
                    return;
                const auto retryAfter = visualRetryAfterMs_.load(std::memory_order_relaxed);
                if (!force && retryAfter > 0 && steadyMilliseconds() >= retryAfter)
                {
                    force = true;
                    visualRetryAfterMs_.store(0, std::memory_order_relaxed);
                }
                const TileKey center = visualBuilder_->SnappedCenter(latDeg, lonDeg);
                bool need = force;
                if (!need)
                {
                    const auto activeCenter = renderer_ ? renderer_->ActiveCenterTile() : std::nullopt;
                    if (!activeCenter)
                    {
                        need = !lastVisualRequestedCenter_.has_value();
                    }
                    else
                    {
                        const double activeLat = tileYToLat(activeCenter->y + 0.5, activeCenter->z);
                        const double activeLon = tileXToLon(activeCenter->x + 0.5, activeCenter->z);
                        const auto activeLocal = localFromGeodetic(
                            *spherical_, activeLat, activeLon,
                            spherical_->ElevationReference());
                        const double dx = vehicleWorld.X() - activeLocal.X();
                        const double dy = vehicleWorld.Y() - activeLocal.Y();
                        need = std::sqrt(dx * dx + dy * dy) >= cfg_.visualRecenterDistanceM;
                    }
                }
                if (!need)
                    return;
                if (lastVisualRequestedCenter_ && *lastVisualRequestedCenter_ == center && !force)
                    return;

                VisualRequest request{center, ++visualGeneration_};
                {
                    std::lock_guard<std::mutex> lock(visualMutex_);
                    visualRequest_ = request;
                }
                latestVisualGeneration_.store(request.generation, std::memory_order_relaxed);
                lastVisualRequestedCenter_ = center;
                visualCv_.notify_one();
                logInfo("[DynamicTerrain][VISUAL] requested generation=", request.generation,
                        " center=", tileText(center),
                        " vehicle_xy=", vehicleWorld.X(), ",", vehicleWorld.Y());
            }

            void queueRefinement(const std::shared_ptr<TerrainSnapshot> &snapshot)
            {
                if (!snapshot || !cfg_.visualRefineTexture)
                    return;
                std::lock_guard<std::mutex> lock(refineMutex_);
                if (!refineRequest_ || snapshot->generation >= refineRequest_->generation)
                    refineRequest_ = RefinementRequest{snapshot, snapshot->generation};
                refineCv_.notify_one();
            }

            void visualWorker()
            {
                while (!stop_.load(std::memory_order_relaxed))
                {
                    VisualRequest request;
                    {
                        std::unique_lock<std::mutex> lock(visualMutex_);
                        visualCv_.wait(lock, [&]
                                       { return stop_.load(std::memory_order_relaxed) || visualRequest_.has_value(); });
                        if (stop_.load(std::memory_order_relaxed))
                            return;
                        request = *visualRequest_;
                        visualRequest_.reset();
                        visualBuilding_.store(true, std::memory_order_relaxed);
                    }

                    auto isStale = [&]()
                    {
                        return request.generation <
                               latestVisualGeneration_.load(std::memory_order_relaxed);
                    };

                    std::string error;
                    auto snapshot = visualBuilder_->BuildBootstrap(
                        request.center, request.generation, error);
                    if (!snapshot)
                    {
                        logError("[DynamicTerrain][VISUAL] generation=", request.generation,
                                 " failed: ", error,
                                 "; currently active terrain remains untouched");
                        visualRetryAfterMs_.store(
                            steadyMilliseconds() + static_cast<std::int64_t>(cfg_.retryDelaySec * 1000.0),
                            std::memory_order_relaxed);
                        visualBuilding_.store(false, std::memory_order_relaxed);
                        continue;
                    }

                    if (isStale())
                    {
                        logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                                " superseded during bootstrap; cache kept, mesh not queued");
                        visualBuilding_.store(false, std::memory_order_relaxed);
                        continue;
                    }

                    const bool hadActiveTerrain = renderer_->HasActiveTerrain();
                    if (hadActiveTerrain && cfg_.visualRefineTexture)
                    {
                        const int geometryZoom = snapshot->geometryRect.zoom;
                        const int readyZoom = std::max(
                            geometryZoom, cfg_.visualRecenterReadyZoom);
                        std::vector<int> pageZoom(snapshot->pages.size(),
                                                  cfg_.visualBootstrapImageryZoom);
                        for (std::size_t i = 0; i < snapshot->pages.size(); ++i)
                            pageZoom[i] = snapshot->pages[i].imageryZoom;

                        auto pageOrder = visualBuilder_->ProgressivePageOrder(*snapshot);
                        for (int zoom = geometryZoom; zoom <= readyZoom && !isStale(); ++zoom)
                        {
                            std::vector<std::size_t> candidates;
                            for (const auto i : pageOrder)
                            {
                                if (i >= pageZoom.size() || pageZoom[i] >= zoom)
                                    continue;
                                if (visualBuilder_->TargetZoomForPage(*snapshot, i) < zoom)
                                    continue;
                                candidates.push_back(i);
                            }

                            const std::size_t batchSize = std::max<std::size_t>(
                                1u, static_cast<std::size_t>(cfg_.visualRefineMaxSourceTilesPerBatch));
                            for (std::size_t offset = 0;
                                 offset < candidates.size() && !isStale();
                                 offset += batchSize)
                            {
                                const auto last = std::min(candidates.size(), offset + batchSize);
                                std::vector<std::size_t> batch(candidates.begin() + offset,
                                                               candidates.begin() + last);
                                std::string stageError;
                                auto update = visualBuilder_->BuildTextureStage(
                                    *snapshot, batch, zoom, stageError);
                                if (update)
                                {
                                    visualBuilder_->ApplyTextureUpdate(*snapshot, *update);
                                    for (const auto &u : update->pages)
                                        if (u.pageIndex < pageZoom.size())
                                            pageZoom[u.pageIndex] = std::max(
                                                pageZoom[u.pageIndex], u.imageryZoom);
                                }
                                if (!stageError.empty())
                                    logInfo("[DynamicTerrain][TEXTURE] recenter base generation=",
                                            request.generation, " z=", zoom,
                                            " pages=", batch.size(), " partial: ", stageError);
                            }
                        }

                        if (isStale())
                        {
                            logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                                    " superseded while preparing base imagery; not queued");
                            visualBuilding_.store(false, std::memory_order_relaxed);
                            continue;
                        }

                        if (cfg_.visualDetailMode == "bottom_camera_only")
                        {
                            std::vector<std::size_t> detailPages;
                            for (const auto i : pageOrder)
                            {
                                if (i >= pageZoom.size())
                                    continue;
                                const int target = visualBuilder_->TargetZoomForPage(*snapshot, i);
                                if (target > geometryZoom && pageZoom[i] < target)
                                    detailPages.push_back(i);
                            }

                            for (const auto pageIndex : detailPages)
                            {
                                if (isStale())
                                    break;
                                const int target = visualBuilder_->TargetZoomForPage(
                                    *snapshot, pageIndex);
                                std::string detailError;
                                auto update = visualBuilder_->BuildTextureStage(
                                    *snapshot, {pageIndex}, target, detailError);
                                if (update)
                                {
                                    visualBuilder_->ApplyTextureUpdate(*snapshot, *update);
                                    for (const auto &u : update->pages)
                                        if (u.pageIndex < pageZoom.size())
                                            pageZoom[u.pageIndex] = std::max(
                                                pageZoom[u.pageIndex], u.imageryZoom);
                                }
                                if (!detailError.empty())
                                    logInfo("[DynamicTerrain][TEXTURE] recenter preload generation=",
                                            request.generation, " page=", pageIndex,
                                            " z=", target, " partial: ", detailError);
                            }
                        }

                        if (isStale())
                        {
                            logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                                    " superseded during detail preload; not queued");
                            visualBuilding_.store(false, std::memory_order_relaxed);
                            continue;
                        }

                        bool baseReady = true;
                        for (std::size_t i = 0; i < pageZoom.size(); ++i)
                        {
                            const int target = visualBuilder_->TargetZoomForPage(*snapshot, i);
                            const int required = cfg_.visualDetailMode == "bottom_camera_only"
                                                     ? target
                                                     : std::min(readyZoom, target);
                            if (pageZoom[i] < required)
                            {
                                baseReady = false;
                                break;
                            }
                        }
                        if (!baseReady)
                        {
                            logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                                    " base imagery incomplete; keeping current active terrain and retrying");
                            visualRetryAfterMs_.store(
                                steadyMilliseconds() + static_cast<std::int64_t>(
                                                           cfg_.retryDelaySec * 1000.0),
                                std::memory_order_relaxed);
                            visualBuilding_.store(false, std::memory_order_relaxed);
                            continue;
                        }
                    }

                    renderer_->QueueSnapshot(snapshot);
                    if (guiSource_)
                        guiSource_->SetSnapshot(snapshot);
                    logInfo("[DynamicTerrain][VISUAL] coverage queued generation=",
                            request.generation,
                            " center=", tileText(request.center),
                            " priority=HIGH; refinement delegated");

                    const auto activationDeadline =
                        std::chrono::steady_clock::now() + std::chrono::seconds(5);
                    while (!isStale() &&
                           renderer_->ActiveGeneration() < request.generation &&
                           std::chrono::steady_clock::now() < activationDeadline &&
                           !stop_.load(std::memory_order_relaxed))
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    if (!isStale() && renderer_->ActiveGeneration() == request.generation)
                    {
                        queueRefinement(snapshot);
                        logInfo("[DynamicTerrain][VISUAL] active generation=", request.generation,
                                " center=", tileText(request.center),
                                " refinement_worker=queued");
                    }
                    else if (isStale())
                    {
                        logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                                " superseded before activation");
                    }
                    else
                    {
                        logInfo("[DynamicTerrain][VISUAL] generation=", request.generation,
                                " activation timeout; active=", renderer_->ActiveGeneration(),
                                "; scheduling retry so this center cannot become permanently blocked");
                        visualRetryAfterMs_.store(
                            steadyMilliseconds() + static_cast<std::int64_t>(
                                                       std::max(5.0, cfg_.retryDelaySec) * 1000.0),
                            std::memory_order_relaxed);
                    }

                    visualBuilding_.store(false, std::memory_order_relaxed);
                }
            }

            void refinementWorker()
            {
                while (!stop_.load(std::memory_order_relaxed))
                {
                    RefinementRequest request;
                    {
                        std::unique_lock<std::mutex> lock(refineMutex_);
                        refineCv_.wait(lock, [&]
                                       { return stop_.load(std::memory_order_relaxed) || refineRequest_.has_value(); });
                        if (stop_.load(std::memory_order_relaxed))
                            return;
                        request = *refineRequest_;
                        refineRequest_.reset();
                        refinementActive_.store(true, std::memory_order_relaxed);
                    }

                    auto stale = [&]()
                    {
                        return request.generation !=
                                   latestVisualGeneration_.load(std::memory_order_relaxed) ||
                               renderer_->ActiveGeneration() != request.generation;
                    };
                    const auto activationDeadline =
                        std::chrono::steady_clock::now() + std::chrono::seconds(2);
                    while (!stop_.load(std::memory_order_relaxed) &&
                           request.generation == latestVisualGeneration_.load(std::memory_order_relaxed) &&
                           renderer_->ActiveGeneration() < request.generation &&
                           std::chrono::steady_clock::now() < activationDeadline)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    if (!request.snapshot || stale())
                    {
                        refinementActive_.store(false, std::memory_order_relaxed);
                        continue;
                    }

                    auto &snapshot = *request.snapshot;
                    const int geometryZoom = snapshot.geometryRect.zoom;
                    auto pageOrder = visualBuilder_->ProgressivePageOrder(snapshot);
                    std::vector<int> pageZoom(snapshot.pages.size(), cfg_.visualBootstrapImageryZoom);
                    std::size_t changedPages = 0;
                    std::unordered_map<std::size_t, TexturePageUpdate> finalPageUpdates;
                    for (std::size_t i = 0; i < snapshot.pages.size(); ++i)
                        pageZoom[i] = snapshot.pages[i].imageryZoom;

                    int maxTargetZoom = geometryZoom;
                    for (std::size_t i = 0; i < snapshot.pages.size(); ++i)
                        maxTargetZoom = std::max(
                            maxTargetZoom, visualBuilder_->TargetZoomForPage(snapshot, i));

                    std::vector<int> refinementLevels;
                    refinementLevels.push_back(geometryZoom);
                    if (cfg_.visualDetailMode == "bottom_camera_only")
                    {
                        if (cfg_.visualDetailZoom > geometryZoom)
                            refinementLevels.push_back(std::min(maxTargetZoom, cfg_.visualDetailZoom));
                    }
                    else
                    {
                        for (int z = geometryZoom + 1; z <= maxTargetZoom; ++z)
                            refinementLevels.push_back(z);
                    }
                    std::sort(refinementLevels.begin(), refinementLevels.end());
                    refinementLevels.erase(
                        std::unique(refinementLevels.begin(), refinementLevels.end()),
                        refinementLevels.end());

                    for (const int zoom : refinementLevels)
                    {
                        if (stale())
                            break;
                        std::vector<std::size_t> candidates;
                        for (const auto pageIndex : pageOrder)
                        {
                            if (pageIndex >= pageZoom.size() || pageZoom[pageIndex] >= zoom)
                                continue;
                            if (visualBuilder_->TargetZoomForPage(snapshot, pageIndex) < zoom)
                                continue;
                            candidates.push_back(pageIndex);
                        }
                        if (candidates.empty())
                            continue;

                        const int delta = std::max(0, zoom - geometryZoom);
                        std::size_t tilesPerPage = 1u;
                        for (int i = 0; i < delta; ++i)
                            tilesPerPage *= 4u;
                        const std::size_t batchSize = std::max<std::size_t>(
                            1u, static_cast<std::size_t>(cfg_.visualRefineMaxSourceTilesPerBatch) /
                                    std::max<std::size_t>(1u, tilesPerPage));

                        logInfo("[DynamicTerrain][TEXTURE] low-priority level generation=",
                                request.generation, " z=", zoom,
                                " pages=", candidates.size(),
                                " pages_per_batch=", batchSize);

                        for (std::size_t offset = 0;
                             offset < candidates.size() && !stale();
                             offset += batchSize)
                        {
                            const auto last = std::min(candidates.size(), offset + batchSize);
                            std::vector<std::size_t> batch(candidates.begin() + offset,
                                                           candidates.begin() + last);
                            std::string stageError;
                            auto update = visualBuilder_->BuildTextureStage(
                                snapshot, batch, zoom, stageError);
                            if (stale())
                                break;
                            if (update)
                            {
                                visualBuilder_->ApplyTextureUpdate(snapshot, *update);
                                changedPages += update->pages.size();
                                for (auto &u : update->pages)
                                {
                                    if (u.pageIndex < pageZoom.size())
                                        pageZoom[u.pageIndex] = std::max(
                                            pageZoom[u.pageIndex], u.imageryZoom);
                                    auto existing = finalPageUpdates.find(u.pageIndex);
                                    if (existing == finalPageUpdates.end() ||
                                        u.imageryZoom >= existing->second.imageryZoom)
                                        finalPageUpdates[u.pageIndex] = std::move(u);
                                }
                            }
                            if (!stageError.empty())
                                logInfo("[DynamicTerrain][TEXTURE] low-priority generation=",
                                        request.generation, " z=", zoom,
                                        " pages=", batch.size(), " partial: ", stageError);
                        }
                    }

                    if (stale())
                    {
                        logInfo("[DynamicTerrain][TEXTURE] refinement preempted generation=",
                                request.generation, " by latest_generation=",
                                latestVisualGeneration_.load(std::memory_order_relaxed));
                    }
                    else
                    {
                        if (!finalPageUpdates.empty())
                        {
                            TextureUpdate finalUpdate;
                            finalUpdate.generation = request.generation;
                            finalUpdate.pages.reserve(finalPageUpdates.size());
                            for (auto &entry : finalPageUpdates)
                                finalUpdate.pages.push_back(std::move(entry.second));
                            finalUpdate.changedPageCount = finalUpdate.pages.size();
                            if (!stale())
                            {
                                if (guiSource_)
                                    guiSource_->ApplyTexture(finalUpdate);
                                renderer_->QueueTexture(std::move(finalUpdate));
                                logInfo("[DynamicTerrain][TEXTURE] coalesced page uploads queued generation=",
                                        request.generation, " changed_pages=",
                                        finalPageUpdates.size());
                            }
                        }
                        logInfo("[DynamicTerrain][TEXTURE] refinement complete generation=",
                                request.generation, " changed_pages=", changedPages,
                                " gpu_page_uploads=", finalPageUpdates.size());
                    }
                    refinementActive_.store(false, std::memory_order_relaxed);
                }
            }

            void updateCollisionRequest(double altitude, double latDeg, double lonDeg,
                                        bool force = false)
            {
                if (!cfg_.enableCollision || !collisionBuilder_)
                    return;
                const auto retryAfter = collisionRetryAfterMs_.load(std::memory_order_relaxed);
                if (!force && retryAfter > 0 && steadyMilliseconds() >= retryAfter)
                {
                    force = true;
                    collisionRetryAfterMs_.store(0, std::memory_order_relaxed);
                }
                const int zoom = zoomForAltitude(cfg_, altitude);
                const TileKey aircraft = latLonToTile(latDeg, lonDeg, zoom);

                bool need = force || !lastCollisionRequestedCenter_ ||
                            lastCollisionRequestedCenter_->z != zoom;
                if (!need)
                {
                    const int threshold = std::max(1, cfg_.radiusTiles);
                    const int dx = std::abs(aircraft.x - lastCollisionRequestedCenter_->x);
                    const int dy = std::abs(aircraft.y - lastCollisionRequestedCenter_->y);
                    need = dx >= threshold || dy >= threshold;
                }
                if (!need)
                    return;

                CollisionRequest request{aircraft, ++collisionGeneration_};
                {
                    std::lock_guard<std::mutex> lock(collisionMutex_);
                    collisionRequest_ = request;
                }
                latestCollisionGeneration_.store(request.generation, std::memory_order_relaxed);
                lastCollisionRequestedCenter_ = aircraft;
                collisionCv_.notify_one();
                if (cfg_.diagnostics)
                    logInfo("[DynamicTerrain][COLLISION] requested generation=", request.generation,
                            " center=", tileText(aircraft), " altitude=", altitude);
            }

            void collisionWorker()
            {
                while (!stop_.load(std::memory_order_relaxed))
                {
                    CollisionRequest request;
                    {
                        std::unique_lock<std::mutex> lock(collisionMutex_);
                        collisionCv_.wait(lock, [&]
                                          { return stop_.load(std::memory_order_relaxed) || collisionRequest_.has_value(); });
                        if (stop_.load(std::memory_order_relaxed))
                            return;
                        request = *collisionRequest_;
                        collisionRequest_.reset();
                    }
                    CollisionResult result;
                    result.request = request;
                    result.patch = collisionBuilder_->Build(request.center, result.error);
                    {
                        std::lock_guard<std::mutex> lock(collisionResultMutex_);
                        if (!collisionResult_ || request.generation >= collisionResult_->request.generation)
                            collisionResult_ = std::move(result);
                    }
                }
            }

            void applyCollisionResult(double simTime, gz::sim::EntityComponentManager &ecm)
            {
                std::optional<CollisionResult> result;
                {
                    std::lock_guard<std::mutex> lock(collisionResultMutex_);
                    if (collisionResult_)
                    {
                        result = std::move(collisionResult_);
                        collisionResult_.reset();
                    }
                }
                if (!result)
                    return;
                if (!result->patch)
                {
                    logError("[DynamicTerrain][COLLISION] generation=",
                             result->request.generation, " failed: ", result->error,
                             "; old collision remains active");
                    collisionRetryAfterMs_.store(
                        steadyMilliseconds() + static_cast<std::int64_t>(cfg_.retryDelaySec * 1000.0),
                        std::memory_order_relaxed);
                    return;
                }

                const bool stale = result->request.generation <
                                   latestCollisionGeneration_.load(std::memory_order_relaxed);
                if (stale && currentCollisionEntity_ != gz::sim::kNullEntity)
                {
                    if (cfg_.diagnostics)
                        logInfo("[DynamicTerrain][COLLISION] skipping stale generated patch generation=",
                                result->request.generation);
                    return;
                }

                sdf::Root root;
                const std::string sdfText = collisionBuilder_->Sdf(*result->patch, ++collisionSerial_);
                const auto errors = root.LoadSdfString(sdfText);
                if (!errors.empty() || !root.Model())
                {
                    logError("[DynamicTerrain][COLLISION] generated SDF failed to parse");
                    return;
                }
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                const auto newEntity = creator.CreateEntities(root.Model());
                if (newEntity == gz::sim::kNullEntity)
                {
                    logError("[DynamicTerrain][COLLISION] CreateEntities failed; old collision retained");
                    return;
                }
                creator.SetParent(newEntity, worldEntity_);

                if (currentCollisionEntity_ != gz::sim::kNullEntity)
                {
                    retiringCollisionEntity_ = currentCollisionEntity_;
                    retireCollisionAtSimTime_ = simTime + 0.10;
                }
                currentCollisionEntity_ = newEntity;
                currentCollisionGeneration_ = result->request.generation;
                currentCollisionCenter_ = result->request.center;
                logInfo("[DynamicTerrain][COLLISION] new patch entity=", newEntity,
                        " generation=", currentCollisionGeneration_,
                        " center=", tileText(*currentCollisionCenter_),
                        " old retained for make-before-break=0.10s");
            }

            void retireOldCollision(double simTime, gz::sim::EntityComponentManager &ecm)
            {
                if (retiringCollisionEntity_ == gz::sim::kNullEntity ||
                    simTime < retireCollisionAtSimTime_)
                    return;
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                creator.RequestRemoveEntity(retiringCollisionEntity_, true);
                if (cfg_.diagnostics)
                    logInfo("[DynamicTerrain][COLLISION] retired old entity=",
                            retiringCollisionEntity_);
                retiringCollisionEntity_ = gz::sim::kNullEntity;
                retireCollisionAtSimTime_ = -1.0;
            }

            void ensureStartupSafetyGround(gz::sim::EntityComponentManager &ecm)
            {
                if (!cfg_.startupSafetyGround || startupSafetyCreated_ ||
                    startupSafetyRemoved_ || !eventManager_)
                    return;

                const double centerZ = cfg_.startupSafetyTopZ -
                                       0.5 * cfg_.startupSafetyThicknessM;
                std::ostringstream sdfText;
                sdfText << std::setprecision(17)
                        << "<sdf version='1.9'><model name='dynamic_terrain_startup_safety'>"
                        << "<static>true</static><link name='safety_ground'>"
                        << "<collision name='collision'><pose>0 0 " << centerZ << " 0 0 0</pose>"
                        << "<geometry><box><size>" << cfg_.startupSafetySizeM << ' '
                        << cfg_.startupSafetySizeM << ' ' << cfg_.startupSafetyThicknessM
                        << "</size></box></geometry></collision></link></model></sdf>";
                sdf::Root root;
                const auto errors = root.LoadSdfString(sdfText.str());
                if (!errors.empty() || !root.Model())
                    return;
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                startupSafetyEntity_ = creator.CreateEntities(root.Model());
                if (startupSafetyEntity_ == gz::sim::kNullEntity)
                    return;
                creator.SetParent(startupSafetyEntity_, worldEntity_);
                startupSafetyCreated_ = true;
                logInfo("[DynamicTerrain][SAFETY] startup collision live entity=",
                        startupSafetyEntity_);
            }

            void updateStartupSafetyGround(double simTime,
                                           gz::sim::EntityComponentManager &ecm)
            {
                if (!cfg_.startupSafetyGround || startupSafetyRemoved_ ||
                    startupSafetyEntity_ == gz::sim::kNullEntity)
                    return;
                if (trackedEntity_ == gz::sim::kNullEntity ||
                    currentCollisionEntity_ == gz::sim::kNullEntity)
                {
                    realTerrainReadySince_ = -1.0;
                    return;
                }
                const auto pose = gz::sim::worldPose(trackedEntity_, ecm);
                if (pose.Pos().Z() < cfg_.startupSafetyTopZ - 0.5)
                {
                    realTerrainReadySince_ = -1.0;
                    return;
                }
                if (realTerrainReadySince_ < 0.0)
                {
                    realTerrainReadySince_ = simTime;
                    return;
                }
                if (simTime - realTerrainReadySince_ < cfg_.startupSafetyRemoveDelaySec)
                    return;
                gz::sim::SdfEntityCreator creator(ecm, *eventManager_);
                creator.RequestRemoveEntity(startupSafetyEntity_, true);
                logInfo("[DynamicTerrain][SAFETY] removed startup collision after real collision became live");
                startupSafetyEntity_ = gz::sim::kNullEntity;
                startupSafetyRemoved_ = true;
            }

            void periodicStatus(const std::string &state)
            {
                if (!cfg_.diagnostics)
                    return;
                const auto now = std::chrono::steady_clock::now();
                if (lastStatusWall_.time_since_epoch().count() != 0)
                {
                    const double elapsed = std::chrono::duration<double>(now - lastStatusWall_).count();
                    if (elapsed < cfg_.statusPeriodSec)
                        return;
                }
                lastStatusWall_ = now;
                logInfo("[DynamicTerrain][STATUS] ", state,
                        " active_visual_generation=", renderer_ ? renderer_->ActiveGeneration() : 0,
                        " visual_building=", visualBuilding_.load(std::memory_order_relaxed) ? "true" : "false",
                        " refinement_active=", refinementActive_.load(std::memory_order_relaxed) ? "true" : "false",
                        " latest_visual_request=", latestVisualGeneration_.load(std::memory_order_relaxed),
                        " collision_entity=", currentCollisionEntity_,
                        " collision_generation=", currentCollisionGeneration_);
            }

        private:
            gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
            gz::sim::Entity trackedEntity_{gz::sim::kNullEntity};
            gz::sim::EventManager *eventManager_{nullptr};
            std::optional<gz::math::SphericalCoordinates> spherical_;
            Config cfg_;
            bool disabled_{false};
            bool configured_{false};

            std::shared_ptr<TileStore> store_;
            std::unique_ptr<PersistentTerrainBuilder> visualBuilder_;
            std::unique_ptr<GuiTerrainSource> guiSource_;
            std::unique_ptr<CollisionTerrainBuilder> collisionBuilder_;
            std::unique_ptr<PersistentTerrainRenderer> renderer_;

            std::atomic<bool> stop_{false};

            std::thread visualThread_;
            std::thread refineThread_;
            std::mutex visualMutex_;
            std::condition_variable visualCv_;
            std::optional<VisualRequest> visualRequest_;
            std::atomic<bool> visualBuilding_{false};
            std::atomic<std::uint64_t> latestVisualGeneration_{0};
            std::atomic<std::int64_t> visualRetryAfterMs_{0};
            std::uint64_t visualGeneration_{0};
            std::optional<TileKey> lastVisualRequestedCenter_;

            std::mutex refineMutex_;
            std::condition_variable refineCv_;
            std::optional<RefinementRequest> refineRequest_;
            std::atomic<bool> refinementActive_{false};

            std::thread collisionThread_;
            std::mutex collisionMutex_;
            std::condition_variable collisionCv_;
            std::optional<CollisionRequest> collisionRequest_;
            std::atomic<std::uint64_t> latestCollisionGeneration_{0};
            std::atomic<std::int64_t> collisionRetryAfterMs_{0};
            std::uint64_t collisionGeneration_{0};
            std::optional<TileKey> lastCollisionRequestedCenter_;

            std::mutex collisionResultMutex_;
            std::optional<CollisionResult> collisionResult_;
            gz::sim::Entity currentCollisionEntity_{gz::sim::kNullEntity};
            gz::sim::Entity retiringCollisionEntity_{gz::sim::kNullEntity};
            double retireCollisionAtSimTime_{-1.0};
            std::uint64_t currentCollisionGeneration_{0};
            std::uint64_t collisionSerial_{0};
            std::optional<TileKey> currentCollisionCenter_;

            gz::sim::Entity startupSafetyEntity_{gz::sim::kNullEntity};
            bool startupSafetyCreated_{false};
            bool startupSafetyRemoved_{false};
            double realTerrainReadySince_{-1.0};

            double lastUpdateTime_{-1e9};
            std::chrono::steady_clock::time_point lastStatusWall_{};
        };

        class DynamicTerrainConfig final
            : public gz::sim::System,
              public gz::sim::ISystemConfigure
        {
        public:
            ~DynamicTerrainConfig() override
            {
                if (entity_ != gz::sim::kNullEntity)
                    unregisterModelConfig(entity_);
            }

            void Configure(const gz::sim::Entity &entity,
                           const std::shared_ptr<const sdf::Element> &sdf,
                           gz::sim::EntityComponentManager &ecm,
                           gz::sim::EventManager &) override
            {
                if (!ecm.Component<gz::sim::components::Model>(entity))
                {
                    logError("[DynamicTerrain][CONFIG] custom::DynamicTerrainConfig must be attached to a <model>");
                    return;
                }
                entity_ = entity;
                Config cfg = parseTerrainConfig(sdf);
                if (const auto *name = ecm.Component<gz::sim::components::Name>(entity))
                    cfg.modelName = name->Data();
                registerModelConfig(entity, cfg);
                logInfo("[DynamicTerrain][CONFIG] registered v0.19.0 model configuration model=",
                        cfg.modelName.empty() ? std::to_string(entity) : cfg.modelName,
                        " entity=", entity);
            }

        private:
            gz::sim::Entity entity_{gz::sim::kNullEntity};
        };

    }
}

GZ_ADD_PLUGIN(dynamic_terrain::DynamicTerrainSystem, gz::sim::System, dynamic_terrain::DynamicTerrainSystem::ISystemConfigure, dynamic_terrain::DynamicTerrainSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainSystem, "dynamic_terrain::DynamicTerrainSystem")
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainSystem, "custom::DynamicTerrainSystem")

GZ_ADD_PLUGIN(dynamic_terrain::DynamicTerrainConfig, gz::sim::System, dynamic_terrain::DynamicTerrainConfig::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainConfig, "dynamic_terrain::DynamicTerrainConfig")
GZ_ADD_PLUGIN_ALIAS(dynamic_terrain::DynamicTerrainConfig, "custom::DynamicTerrainConfig")
