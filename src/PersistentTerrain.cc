#include "PersistentTerrain.hh"
#include "Ogre2ResourceCleanup.hh"

#include <gz/common/SubMesh.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Color.hh>
#include <gz/math/Frustum.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/WideAngleCamera.hh>
#include <gz/sim/rendering/Events.hh>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace dynamic_terrain
{
namespace
{
std::string terrainPageSubmeshName(const TileKey &key)
{
    return "terrain_page_z" + std::to_string(key.z) +
           "_x" + std::to_string(key.x) +
           "_y" + std::to_string(key.y);
}

double horizontalDistance(const gz::math::Vector3d &a,
                          const gz::math::Vector3d &b)
{
    const double dx = a.X() - b.X();
    const double dy = a.Y() - b.Y();
    return std::sqrt(dx * dx + dy * dy);
}

int floorLog2Positive(int value)
{
    int result = 0;
    while (value > 1)
    {
        value >>= 1;
        ++result;
    }
    return result;
}

std::atomic<std::uint64_t> &rendererInstanceCounter()
{
    static std::atomic<std::uint64_t> counter{0};
    return counter;
}

bool scopedNameMatches(const std::string &candidate,
                       const std::string &requested)
{
    if (requested.empty() || candidate == requested)
        return candidate == requested;
    if (candidate.size() < requested.size() ||
        candidate.compare(candidate.size() - requested.size(),
                          requested.size(), requested) != 0)
        return false;
    const std::size_t prefixEnd = candidate.size() - requested.size();
    return prefixEnd == 0u || candidate[prefixEnd - 1u] == ':' ||
           candidate[prefixEnd - 1u] == '/';
}

bool finiteVector(const gz::math::Vector3d &value)
{
    return std::isfinite(value.X()) && std::isfinite(value.Y()) &&
           std::isfinite(value.Z());
}
}

PersistentTerrainBuilder::PersistentTerrainBuilder(std::shared_ptr<TileStore> store)
    : store_(std::move(store))
{
}

std::optional<PersistentTerrainBuilder::CachedPageTexture>
PersistentTerrainBuilder::CachedTextureForPage(const TileKey &key) const
{
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    const auto it = pageTextureCache_.find(key);
    if (it == pageTextureCache_.end() || !it->second.texture ||
        !it->second.texture->Valid())
        return std::nullopt;
    it->second.touch = ++pageTextureCacheTouch_;
    return it->second;
}

void PersistentTerrainBuilder::RememberPageTexture(
    const TileKey &key, int imageryZoom, int textureSize,
    const std::shared_ptr<gz::common::Image> &texture,
    const std::string &textureName) const
{
    if (!texture || !texture->Valid())
        return;

    if (imageryZoom <= store_->GetConfig().visualGeometryZoom)
        return;

    const std::size_t limit = store_->GetConfig().visualPageCacheMb * 1024u * 1024u;
    const std::size_t bytes = static_cast<std::size_t>(texture->Pitch()) *
                              texture->Height();
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    if (bytes > limit || limit == 0u)
    {
        const auto old = pageTextureCache_.find(key);
        if (old != pageTextureCache_.end())
        {
            pageTextureCacheBytes_ -= old->second.bytes;
            pageTextureCache_.erase(old);
        }
        return;
    }
    auto &entry = pageTextureCache_[key];
    if (!entry.texture || imageryZoom >= entry.imageryZoom)
    {
        entry.imageryZoom = imageryZoom;
        entry.textureSize = textureSize;
        pageTextureCacheBytes_ -= entry.bytes;
        entry.bytes = bytes;
        pageTextureCacheBytes_ += entry.bytes;
        entry.texture = texture;
        entry.textureName = textureName;
    }
    entry.touch = ++pageTextureCacheTouch_;

    while (pageTextureCache_.size() > kPageTextureCacheLimit ||
           pageTextureCacheBytes_ > limit)
    {
        auto victim = pageTextureCache_.end();
        for (auto it = pageTextureCache_.begin(); it != pageTextureCache_.end(); ++it)
        {
            if (victim == pageTextureCache_.end() ||
                it->second.touch < victim->second.touch)
                victim = it;
        }
        if (victim == pageTextureCache_.end())
            break;
        pageTextureCacheBytes_ -= victim->second.bytes;
        pageTextureCache_.erase(victim);
    }
}

PersistentTerrainBuilder::PageCacheStats
PersistentTerrainBuilder::CachedPageStats() const
{
    std::lock_guard<std::mutex> lock(pageTextureCacheMutex_);
    return {pageTextureCache_.size(), pageTextureCacheBytes_,
            store_->GetConfig().visualPageCacheMb * 1024u * 1024u};
}

TileKey PersistentTerrainBuilder::SnappedCenter(double latDeg, double lonDeg) const
{
    return latLonToTile(latDeg, lonDeg, store_->GetConfig().visualGeometryZoom);
}

int PersistentTerrainBuilder::RadiusTiles(const TileKey &center) const
{
    const auto &cfg = store_->GetConfig();
    const double reference = store_->Spherical().ElevationReference();
    const double centerLat = tileYToLat(center.y + 0.5, center.z);
    const double centerLon = tileXToLon(center.x + 0.5, center.z);
    const auto centerLocal = localFromGeodetic(
        store_->Spherical(), centerLat, centerLon, reference);

    for (int radius = 1; radius <= 32; ++radius)
    {
        const int count = 1 << center.z;
        const TileRect rect{
            clampValue(center.x - radius, 0, count - 1),
            clampValue(center.y - radius, 0, count - 1),
            clampValue(center.x + radius, 0, count - 1),
            clampValue(center.y + radius, 0, count - 1), center.z};
        const TileBounds b = tileRectBounds(rect);
        const auto west = localFromGeodetic(
            store_->Spherical(), centerLat, b.west, reference);
        const auto east = localFromGeodetic(
            store_->Spherical(), centerLat, b.east, reference);
        const auto north = localFromGeodetic(
            store_->Spherical(), b.north, centerLon, reference);
        const auto south = localFromGeodetic(
            store_->Spherical(), b.south, centerLon, reference);
        const double halfX = std::min(horizontalDistance(centerLocal, west),
                                      horizontalDistance(centerLocal, east));
        const double halfY = std::min(horizontalDistance(centerLocal, north),
                                      horizontalDistance(centerLocal, south));
        if (halfX >= cfg.visualRadiusM && halfY >= cfg.visualRadiusM)
            return radius;
    }
    return 32;
}

int PersistentTerrainBuilder::EffectiveCellsPerTile(const TileRect &rect) const
{
    const auto &cfg = store_->GetConfig();
    const int maxSideTiles = std::max(rect.Width(), rect.Height());
    int cells = cfg.visualMeshCellsPerTile;
    if (maxSideTiles * cells > cfg.visualMaxMeshCells)
    {
        cells = std::max(8, cfg.visualMaxMeshCells / std::max(1, maxSideTiles));
        if (cfg.diagnostics)
            logInfo("[DynamicTerrain][VISUAL] mesh cells reduced per_tile=",
                    cfg.visualMeshCellsPerTile, " -> ", cells,
                    " to stay within visual_max_mesh_cells=",
                    cfg.visualMaxMeshCells);
    }
    return cells;
}

std::vector<TileKey> PersistentTerrainBuilder::PageSourceKeys(
    const TileKey &pageKey, int sourceZoom) const
{
    std::vector<TileKey> result;
    if (sourceZoom <= pageKey.z)
    {
        const int shift = pageKey.z - sourceZoom;
        result.push_back({pageKey.x >> shift, pageKey.y >> shift, sourceZoom});
        return result;
    }

    const int delta = sourceZoom - pageKey.z;
    const int factor = 1 << delta;
    result.reserve(static_cast<std::size_t>(factor) * factor);
    const int baseX = pageKey.x * factor;
    const int baseY = pageKey.y * factor;
    for (int y = 0; y < factor; ++y)
        for (int x = 0; x < factor; ++x)
            result.push_back({baseX + x, baseY + y, sourceZoom});
    return result;
}

cv::Mat PersistentTerrainBuilder::ComposeBootstrapPage(
    const TileKey &pageKey, int geometryZoom, int bootstrapZoom,
    int outputSize) const
{
    outputSize = std::max(256, outputSize);
    cv::Mat output(outputSize, outputSize, CV_8UC3, cv::Scalar(96, 96, 96));

    if (bootstrapZoom > geometryZoom)
        bootstrapZoom = geometryZoom;
    const int shift = geometryZoom - bootstrapZoom;
    const TileKey source{pageKey.x >> shift, pageKey.y >> shift, bootstrapZoom};
    cv::Mat tile = store_->LoadImagery(source);
    if (tile.empty())
        return output;

    if (shift == 0)
    {
        cv::resize(tile, output, output.size(), 0.0, 0.0,
                   tile.cols > outputSize || tile.rows > outputSize ?
                       cv::INTER_AREA : cv::INTER_LINEAR);
        return output;
    }

    const int factor = 1 << shift;
    const int localX = pageKey.x - (source.x << shift);
    const int localY = pageKey.y - (source.y << shift);
    const int sx0 = localX * tile.cols / factor;
    const int sx1 = (localX + 1) * tile.cols / factor;
    const int sy0 = localY * tile.rows / factor;
    const int sy1 = (localY + 1) * tile.rows / factor;
    const int x0 = clampValue(sx0, 0, tile.cols - 1);
    const int x1 = clampValue(sx1, x0 + 1, tile.cols);
    const int y0 = clampValue(sy0, 0, tile.rows - 1);
    const int y1 = clampValue(sy1, y0 + 1, tile.rows);
    const cv::Rect src{x0, y0, x1 - x0, y1 - y0};
    cv::resize(tile(src), output, output.size(), 0.0, 0.0, cv::INTER_LINEAR);
    return output;
}

int PersistentTerrainBuilder::PageTargetZoom(
    const TerrainSnapshot &snapshot, const TileKey &pageKey) const
{
    const auto &cfg = store_->GetConfig();
    const double reference = store_->Spherical().ElevationReference();
    const double lat = tileYToLat(pageKey.y + 0.5, pageKey.z);
    const double lon = tileXToLon(pageKey.x + 0.5, pageKey.z);
    const auto local = localFromGeodetic(
        store_->Spherical(), lat, lon, reference);
    const TileBounds tb = tileRectBounds(
        {pageKey.x, pageKey.y, pageKey.x, pageKey.y, pageKey.z});
    const auto corner = localFromGeodetic(
        store_->Spherical(), tb.north, tb.west, reference);
    const double pagePad = horizontalDistance(local, corner);
    const double distance = horizontalDistance(local, snapshot.patchCenterLocal);

    int target = snapshot.geometryRect.zoom;
    if (cfg.visualDetailMode == "bottom_camera_only")
    {
        if (distance <= cfg.visualDetailRadiusM + pagePad)
            target = cfg.visualDetailZoom;
    }
    else
    {
        for (const auto &[ringDistance, zoom] : cfg.visualImageryLodTable)
        {
            const double protectedDistance = ringDistance +
                cfg.visualRecenterDistanceM + cfg.visualTextureGuardM + pagePad;
            if (distance <= protectedDistance)
            {
                target = zoom;
                break;
            }
        }
    }

    target = std::max(target, snapshot.geometryRect.zoom);
    const int maxScale = std::max(1, cfg.visualPageTextureMaxSize / 256);
    const int maxDelta = floorLog2Positive(maxScale);
    target = std::min(target, snapshot.geometryRect.zoom + maxDelta);
    return clampValue(target, snapshot.geometryRect.zoom, 20);
}

bool PersistentTerrainBuilder::SourceTileWanted(
    const TerrainSnapshot &snapshot, const TileKey &key) const
{
    const auto &cfg = store_->GetConfig();
    if (cfg.visualDetailMode != "bottom_camera_only" ||
        key.z <= snapshot.geometryRect.zoom)
        return true;

    const int delta = key.z - snapshot.geometryRect.zoom;
    const TileKey parent{key.x >> delta, key.y >> delta,
                         snapshot.geometryRect.zoom};
    return PageTargetZoom(snapshot, parent) >= key.z;
}

cv::Mat PersistentTerrainBuilder::ComposeRefinedPage(
    const TileKey &pageKey,
    int geometryZoom,
    int sourceZoom,
    int outputSize,
    const std::unordered_set<TileKey, TileKeyHash> &failed) const
{
    sourceZoom = std::max(sourceZoom, geometryZoom);
    outputSize = std::max(256, outputSize);

    cv::Mat output = ComposeBootstrapPage(
        pageKey, geometryZoom,
        store_->GetConfig().visualBootstrapImageryZoom,
        outputSize);

    for (int zoom = geometryZoom; zoom <= sourceZoom; ++zoom)
    {
        const int levelDelta = zoom - geometryZoom;
        const int factor = 1 << levelDelta;
        if (outputSize % factor != 0)
            continue;
        const int regionPixels = outputSize / factor;
        const int baseX = pageKey.x * factor;
        const int baseY = pageKey.y * factor;
        for (int y = 0; y < factor; ++y)
        {
            for (int x = 0; x < factor; ++x)
            {
                const TileKey key{baseX + x, baseY + y, zoom};
                if (zoom == sourceZoom && failed.count(key))
                    continue;
                cv::Mat tile = store_->LoadImagery(key);
                if (tile.empty())
                    continue;

                cv::Mat normalized;
                if (tile.cols == regionPixels && tile.rows == regionPixels)
                    normalized = tile;
                else
                    cv::resize(tile, normalized,
                               cv::Size(regionPixels, regionPixels), 0.0, 0.0,
                               tile.cols > regionPixels || tile.rows > regionPixels ?
                                   cv::INTER_AREA : cv::INTER_LINEAR);
                normalized.copyTo(output(cv::Rect(
                    x * regionPixels, y * regionPixels,
                    regionPixels, regionPixels)));
            }
        }
    }
    return output;
}

std::shared_ptr<TerrainSnapshot> PersistentTerrainBuilder::BuildBootstrap(
    const TileKey &center, std::uint64_t generation, std::string &error)
{
    const auto &cfg = store_->GetConfig();
    const auto started = std::chrono::steady_clock::now();
    const int radius = RadiusTiles(center);
    const int count = 1 << center.z;
    TileRect rect{
        clampValue(center.x - radius, 0, count - 1),
        clampValue(center.y - radius, 0, count - 1),
        clampValue(center.x + radius, 0, count - 1),
        clampValue(center.y + radius, 0, count - 1), center.z};
    const TileBounds bounds = tileRectBounds(rect);

    if (cfg.diagnostics)
        logInfo("[DynamicTerrain][VISUAL] build generation=", generation,
                " center=", tileText(center),
                " rect=", rect.Width(), "x", rect.Height(),
                " geometry_zoom=", center.z,
                " elevation_zoom=", cfg.visualElevationZoom,
                " radius~=", cfg.visualRadiusM, "m mode=paged-textures");

    auto dem = store_->BuildElevationMosaic(
        bounds, cfg.visualElevationZoom, 1, error);
    if (!dem)
        return {};
    const double elevationOffset =
        store_->ElevationAlignmentOffset(cfg.visualElevationZoom) + cfg.zOffsetM;

    auto snapshot = std::make_shared<TerrainSnapshot>();
    snapshot->generation = generation;
    snapshot->centerTile = center;
    snapshot->geometryRect = rect;
    snapshot->bounds = bounds;
    snapshot->cellsPerTile = EffectiveCellsPerTile(rect);
    snapshot->cellsX = rect.Width() * snapshot->cellsPerTile;
    snapshot->cellsY = rect.Height() * snapshot->cellsPerTile;

    snapshot->resourcePrefix = store_->ResourcePrefix();

    const double centerLat = tileYToLat(center.y + 0.5, center.z);
    const double centerLon = tileXToLon(center.x + 0.5, center.z);
    snapshot->patchCenterLocal = localFromGeodetic(
        store_->Spherical(), centerLat, centerLon,
        store_->Spherical().ElevationReference());

    const int vertexCols = snapshot->cellsX + 1;
    const int vertexRows = snapshot->cellsY + 1;
    const std::size_t vertexCount = static_cast<std::size_t>(vertexCols) * vertexRows;
    if (vertexCount > 4'000'000u)
    {
        error = "visual mesh exceeds 4 million vertices; reduce radius or mesh cells";
        return {};
    }

    std::vector<gz::math::Vector3d> vertices(vertexCount);
    for (int y = 0; y < vertexRows; ++y)
    {
        const double tileY = static_cast<double>(rect.minY) +
            static_cast<double>(y) / snapshot->cellsPerTile;
        const double lat = tileYToLat(tileY, rect.zoom);
        for (int x = 0; x < vertexCols; ++x)
        {
            const double tileX = static_cast<double>(rect.minX) +
                static_cast<double>(x) / snapshot->cellsPerTile;
            const double lon = tileXToLon(tileX, rect.zoom);
            const double elevation = dem->Sample(lat, lon) + elevationOffset;
            vertices[static_cast<std::size_t>(y) * vertexCols + x] =
                localFromGeodetic(store_->Spherical(), lat, lon, elevation);
        }
    }

    auto vertexAt = [&](int x, int y) -> const gz::math::Vector3d &
    {
        return vertices[static_cast<std::size_t>(y) * vertexCols + x];
    };
    std::vector<gz::math::Vector3d> normals(vertexCount);
    for (int y = 0; y < vertexRows; ++y)
    {
        for (int x = 0; x < vertexCols; ++x)
        {
            const auto &left = vertexAt(std::max(0, x - 1), y);
            const auto &right = vertexAt(std::min(vertexCols - 1, x + 1), y);
            const auto &north = vertexAt(x, std::max(0, y - 1));
            const auto &south = vertexAt(x, std::min(vertexRows - 1, y + 1));
            gz::math::Vector3d eastTangent = right - left;
            gz::math::Vector3d northTangent = north - south;
            gz::math::Vector3d normal = eastTangent.Cross(northTangent);
            if (normal.Length() < 1e-9)
                normal = gz::math::Vector3d::UnitZ;
            else
                normal.Normalize();
            if (normal.Z() < 0.0)
                normal = -normal;
            normals[static_cast<std::size_t>(y) * vertexCols + x] = normal;
        }
    }

    const int bootstrapZoom = std::min(cfg.visualBootstrapImageryZoom, rect.zoom);
    std::unordered_set<TileKey, TileKeyHash> bootstrapSet;
    for (int ty = rect.minY; ty <= rect.maxY; ++ty)
        for (int tx = rect.minX; tx <= rect.maxX; ++tx)
            for (const auto &key : PageSourceKeys({tx, ty, rect.zoom}, bootstrapZoom))
                bootstrapSet.insert(key);
    std::vector<TileKey> bootstrapKeys(bootstrapSet.begin(), bootstrapSet.end());
    std::vector<TileKey> bootstrapFailed;
    store_->EnsureImagery(bootstrapKeys, &bootstrapFailed);

    snapshot->pages.reserve(static_cast<std::size_t>(rect.Width()) * rect.Height());
    std::size_t carriedPageTextures = 0;
    std::size_t pageIndex = 0;
    for (int ty = rect.minY; ty <= rect.maxY; ++ty)
    {
        for (int tx = rect.minX; tx <= rect.maxX; ++tx, ++pageIndex)
        {
            const TileKey pageKey{tx, ty, rect.zoom};
            const std::string submeshName = terrainPageSubmeshName(pageKey);

            TerrainPage page;
            page.key = pageKey;
            page.index = pageIndex;
            page.submeshName = submeshName;
            page.imageryZoom = bootstrapZoom;
            page.targetImageryZoom = PageTargetZoom(*snapshot, page.key);

            const auto cached = CachedTextureForPage(page.key);
            if (cached)
                page.targetImageryZoom = std::max(page.targetImageryZoom,
                                                  cached->imageryZoom);

            page.textureSize = 256 <<
                (page.targetImageryZoom - snapshot->geometryRect.zoom);
            page.textureSize = std::min(page.textureSize,
                                        cfg.visualPageTextureMaxSize);

            if (cached && cached->textureSize == page.textureSize &&
                cached->imageryZoom >= bootstrapZoom)
            {
                page.imageryZoom = cached->imageryZoom;
                page.texture = cached->texture;
                page.textureName = cached->textureName;
                ++carriedPageTextures;
            }
            else
            {
                const auto raster = ComposeBootstrapPage(
                    page.key, rect.zoom, bootstrapZoom, page.textureSize);
                page.texture = ToImage(raster);
                page.textureName = snapshot->resourcePrefix + "_page_z" +
                    std::to_string(page.key.z) + "_x" +
                    std::to_string(page.key.x) + "_y" +
                    std::to_string(page.key.y) + "_q" +
                    std::to_string(bootstrapZoom) + "_s" +
                    std::to_string(page.textureSize);
            }
            if (!page.texture || !page.texture->Valid())
            {
                error = "failed to create bootstrap page texture";
                return {};
            }

            page.mesh = std::make_shared<gz::common::Mesh>();
            page.mesh->SetName(snapshot->resourcePrefix + "_mesh_g" +
                std::to_string(generation) + "_z" +
                std::to_string(page.key.z) + "_x" +
                std::to_string(page.key.x) + "_y" +
                std::to_string(page.key.y));
            auto pageSurface = std::make_unique<gz::common::SubMesh>(submeshName);
            pageSurface->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
            const int baseGridX = (tx - rect.minX) * snapshot->cellsPerTile;
            const int baseGridY = (ty - rect.minY) * snapshot->cellsPerTile;
            const int pageVertexCols = snapshot->cellsPerTile + 1;
            for (int py = 0; py <= snapshot->cellsPerTile; ++py)
            {
                for (int px = 0; px <= snapshot->cellsPerTile; ++px)
                {
                    const int gx = baseGridX + px;
                    const int gy = baseGridY + py;
                    const std::size_t globalIndex =
                        static_cast<std::size_t>(gy) * vertexCols + gx;
                    pageSurface->AddVertex(vertices[globalIndex]);
                    pageSurface->AddNormal(normals[globalIndex]);
                    pageSurface->AddTexCoord(
                        static_cast<double>(px) / snapshot->cellsPerTile,
                        static_cast<double>(py) / snapshot->cellsPerTile);
                }
            }
            for (int py = 0; py < snapshot->cellsPerTile; ++py)
            {
                for (int px = 0; px < snapshot->cellsPerTile; ++px)
                {
                    const unsigned int a = static_cast<unsigned int>(
                        py * pageVertexCols + px);
                    const unsigned int b = a + 1u;
                    const unsigned int c = static_cast<unsigned int>(
                        (py + 1) * pageVertexCols + px);
                    const unsigned int d = c + 1u;
                    pageSurface->AddIndex(a);
                    pageSurface->AddIndex(c);
                    pageSurface->AddIndex(b);
                    pageSurface->AddIndex(b);
                    pageSurface->AddIndex(c);
                    pageSurface->AddIndex(d);
                }
            }
            page.mesh->AddSubMesh(std::move(pageSurface));
            snapshot->estimatedTextureBytes += mipmappedRgbaBytes(
                page.textureSize, page.textureSize);
            snapshot->pages.push_back(std::move(page));
        }
    }

    if (cfg.diagnostics)
    {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        logInfo("[DynamicTerrain][VISUAL] bootstrap ready generation=", generation,
                " vertices=", vertexCount,
                " triangles=", static_cast<std::size_t>(snapshot->cellsX) *
                    snapshot->cellsY * 2u,
                " pages=", snapshot->pages.size(),
                " bootstrap_z=", bootstrapZoom,
                " texture_grid=stable-per-page",
                " max_page_texture=", cfg.visualPageTextureMaxSize,
                " gpu_layout=independent-geographic-pages",
                " estimated_resident_texture_mib=",
                static_cast<double>(snapshot->estimatedTextureBytes) /
                    (1024.0 * 1024.0),
                " carried_exact_pages=", carriedPageTextures,
                " time=", elapsed, "s");
    }
    return snapshot;
}

std::vector<std::size_t> PersistentTerrainBuilder::ProgressivePageOrder(
    const TerrainSnapshot &snapshot) const
{
    std::vector<std::size_t> order(snapshot.pages.size());
    for (std::size_t i = 0; i < order.size(); ++i)
        order[i] = i;

    const double reference = store_->Spherical().ElevationReference();
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b)
    {
        auto distanceFor = [&](std::size_t index)
        {
            const auto &page = snapshot.pages[index];
            const double lat = tileYToLat(page.key.y + 0.5, page.key.z);
            const double lon = tileXToLon(page.key.x + 0.5, page.key.z);
            const auto local = localFromGeodetic(
                store_->Spherical(), lat, lon, reference);
            return horizontalDistance(local, snapshot.patchCenterLocal);
        };
        const double da = distanceFor(a);
        const double db = distanceFor(b);
        if (std::abs(da - db) > 1e-6)
            return da < db;
        return a < b;
    });
    return order;
}

int PersistentTerrainBuilder::TargetZoomForPage(
    const TerrainSnapshot &snapshot, std::size_t pageIndex) const
{
    if (pageIndex >= snapshot.pages.size())
        return snapshot.geometryRect.zoom;
    return PageTargetZoom(snapshot, snapshot.pages[pageIndex].key);
}

std::optional<TextureUpdate> PersistentTerrainBuilder::BuildTextureStage(
    const TerrainSnapshot &snapshot,
    const std::vector<std::size_t> &pageIndices,
    int sourceZoom,
    std::string &error)
{
    if (!store_->GetConfig().visualRefineTexture || pageIndices.empty())
        return std::nullopt;

    sourceZoom = clampValue(sourceZoom, snapshot.geometryRect.zoom, 20);
    std::unordered_set<TileKey, TileKeyHash> requestSet;
    struct Plan
    {
        std::size_t pageIndex{0};
        std::vector<TileKey> keys;
    };
    std::vector<Plan> plans;
    plans.reserve(pageIndices.size());

    for (const std::size_t pageIndex : pageIndices)
    {
        if (pageIndex >= snapshot.pages.size())
            continue;
        const auto &page = snapshot.pages[pageIndex];
        if (sourceZoom > PageTargetZoom(snapshot, page.key))
            continue;
        Plan plan;
        plan.pageIndex = pageIndex;
        for (const auto &key : PageSourceKeys(page.key, sourceZoom))
        {
            if (SourceTileWanted(snapshot, key))
            {
                plan.keys.push_back(key);
                requestSet.insert(key);
            }
        }
        if (!plan.keys.empty())
            plans.push_back(std::move(plan));
    }
    if (plans.empty())
        return std::nullopt;

    std::vector<TileKey> requests(requestSet.begin(), requestSet.end());
    std::vector<TileKey> failed;
    store_->EnsureImagery(requests, &failed);
    const std::unordered_set<TileKey, TileKeyHash> failedSet(
        failed.begin(), failed.end());

    TextureUpdate update;
    update.generation = snapshot.generation;
    update.pages.reserve(plans.size());
    std::size_t partialPages = 0;
    std::size_t fullyFailedPages = 0;

    for (const auto &plan : plans)
    {
        std::size_t failedKeys = 0;
        for (const auto &key : plan.keys)
            if (failedSet.count(key))
                ++failedKeys;

        if (failedKeys == plan.keys.size() && !plan.keys.empty())
        {
            ++fullyFailedPages;
            continue;
        }
        if (failedKeys > 0)
        {
            ++partialPages;
            continue;
        }

        const auto &page = snapshot.pages[plan.pageIndex];
        cv::Mat texture = ComposeRefinedPage(
            page.key, snapshot.geometryRect.zoom, sourceZoom,
            page.textureSize, failedSet);
        auto image = ToImage(texture);
        if (!image || !image->Valid())
            continue;

        TexturePageUpdate pageUpdate;
        pageUpdate.pageIndex = plan.pageIndex;
        pageUpdate.pageKey = page.key;
        pageUpdate.submeshName = page.submeshName;
        pageUpdate.imageryZoom = sourceZoom;
        pageUpdate.textureSize = page.textureSize;
        pageUpdate.texture = std::move(image);
        pageUpdate.textureName = snapshot.resourcePrefix + "_page_z" +
            std::to_string(page.key.z) + "_x" + std::to_string(page.key.x) +
            "_y" + std::to_string(page.key.y) + "_q" +
            std::to_string(sourceZoom) + "_s" + std::to_string(page.textureSize);
        RememberPageTexture(page.key, sourceZoom, page.textureSize,
                            pageUpdate.texture,
                            pageUpdate.textureName);
        update.pages.push_back(std::move(pageUpdate));
    }

    if (partialPages > 0 || fullyFailedPages > 0)
    {
        error = std::to_string(partialPages) + " partial pages, " +
            std::to_string(fullyFailedPages) +
            " pages kept at previous quality";
    }

    if (store_->GetConfig().diagnostics)
    {
        const auto cache = CachedPageStats();
        logInfo("[DynamicTerrain][TEXTURE] progressive stage generation=",
                snapshot.generation,
                " z=", sourceZoom,
                " pages_requested=", plans.size(),
                " pages_ready=", update.pages.size(),
                " source_tiles=", requests.size(),
                " failed_tiles=", failed.size(),
                " partial_pages=", partialPages,
                " kept_previous=", fullyFailedPages,
                " cpu_cache_pages=", cache.pages,
                " cpu_cache_bytes=", cache.bytes,
                " cpu_cache_limit_bytes=", cache.limitBytes);
    }

    if (update.pages.empty())
        return std::nullopt;
    update.changedPageCount = update.pages.size();
    return update;
}

void PersistentTerrainBuilder::ApplyTextureUpdate(
    TerrainSnapshot &snapshot, TextureUpdate &update) const
{
    if (snapshot.generation != update.generation)
        return;
    for (const auto &pageUpdate : update.pages)
    {
        if (pageUpdate.pageIndex >= snapshot.pages.size())
            continue;
        auto &page = snapshot.pages[pageUpdate.pageIndex];
        if (page.key != pageUpdate.pageKey ||
            page.submeshName != pageUpdate.submeshName)
        {
            logError("[DynamicTerrain][MAPPING] rejected snapshot texture update index=",
                     pageUpdate.pageIndex, " expected=", tileText(page.key),
                     " incoming=", tileText(pageUpdate.pageKey),
                     " expected_submesh=", page.submeshName,
                     " incoming_submesh=", pageUpdate.submeshName);
            continue;
        }
        if (pageUpdate.imageryZoom < page.imageryZoom)
            continue;
        if (pageUpdate.textureSize != page.textureSize ||
            !pageUpdate.texture || !pageUpdate.texture->Valid())
            continue;
        page.imageryZoom = pageUpdate.imageryZoom;
        page.texture = pageUpdate.texture;
        page.textureName = pageUpdate.textureName;
    }
}

std::shared_ptr<gz::common::Image> PersistentTerrainBuilder::ToImage(
    const cv::Mat &bgr) const
{
    if (bgr.empty() || bgr.type() != CV_8UC3)
        return {};

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    if (!rgb.isContinuous())
        rgb = rgb.clone();

    auto image = std::make_shared<gz::common::Image>();
    image->SetFromData(rgb.data,
                       static_cast<unsigned int>(rgb.cols),
                       static_cast<unsigned int>(rgb.rows),
                       gz::common::Image::RGB_INT8);
    return image;
}

PersistentTerrainRenderer::PersistentTerrainRenderer(
    Config config, gz::sim::EventManager &events)
    : cfg_(std::move(config)),
      rendererId_(rendererInstanceCounter().fetch_add(
          1, std::memory_order_relaxed) + 1)
{
    preRenderConnection_ = events.Connect<gz::sim::events::PreRender>(
        std::bind(&PersistentTerrainRenderer::OnPreRender, this));
    postRenderConnection_ = events.Connect<gz::sim::events::PostRender>(
        std::bind(&PersistentTerrainRenderer::OnPostRender, this));
    teardownConnection_ = events.Connect<gz::sim::events::RenderTeardown>(
        std::bind(&PersistentTerrainRenderer::OnRenderTeardown, this));
}

PersistentTerrainRenderer::~PersistentTerrainRenderer()
{
    renderShuttingDown_.store(true, std::memory_order_release);
    preRenderConnection_.reset();
    postRenderConnection_.reset();
    teardownConnection_.reset();
}

void PersistentTerrainRenderer::QueueSnapshot(
    std::shared_ptr<TerrainSnapshot> snapshot)
{
    if (!snapshot || renderShuttingDown_.load(std::memory_order_acquire))
        return;
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (renderShuttingDown_.load(std::memory_order_relaxed))
        return;
    if (pendingTexture_ && pendingTexture_->generation < snapshot->generation)
        pendingTexture_.reset();
    if (!pendingSnapshot_ || snapshot->generation >= pendingSnapshot_->generation)
        pendingSnapshot_ = std::move(snapshot);
}

void PersistentTerrainRenderer::QueueTexture(TextureUpdate update)
{
    if (update.pages.empty() ||
        renderShuttingDown_.load(std::memory_order_acquire))
        return;
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (renderShuttingDown_.load(std::memory_order_relaxed))
        return;
    if (!pendingTexture_ || update.generation > pendingTexture_->generation)
    {
        pendingTexture_ = std::move(update);
        return;
    }
    if (update.generation < pendingTexture_->generation)
        return;

    for (auto &incoming : update.pages)
    {
        auto it = std::find_if(
            pendingTexture_->pages.begin(), pendingTexture_->pages.end(),
            [&](const TexturePageUpdate &queued)
            {
                return queued.pageKey == incoming.pageKey;
            });
        if (it == pendingTexture_->pages.end())
            pendingTexture_->pages.push_back(std::move(incoming));
        else if (incoming.imageryZoom >= it->imageryZoom)
            *it = std::move(incoming);
    }
    pendingTexture_->changedPageCount = pendingTexture_->pages.size();
}

std::uint64_t PersistentTerrainRenderer::ActiveGeneration() const
{
    return activeGeneration_.load(std::memory_order_relaxed);
}

bool PersistentTerrainRenderer::HasActiveTerrain() const
{
    return hasActive_.load(std::memory_order_relaxed);
}

std::optional<TileKey> PersistentTerrainRenderer::ActiveCenterTile() const
{
    if (!hasActive_.load(std::memory_order_acquire))
        return std::nullopt;
    return TileKey{activeCenterX_.load(std::memory_order_relaxed),
                   activeCenterY_.load(std::memory_order_relaxed),
                   activeCenterZ_.load(std::memory_order_relaxed)};
}

void PersistentTerrainRenderer::FindScene()
{
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene || !scene->IsInitialized() || !scene->RootVisual())
        return;

    scene_ = scene;
    const std::string rootName = "dynamic_terrain_persistent_root_i" +
        std::to_string(rendererId_);
    root_ = scene_->CreateVisual(rootName);
    if (!root_)
    {
        auto node = scene_->RootVisual()->ChildByName(rootName);
        root_ = std::dynamic_pointer_cast<gz::rendering::Visual>(node);
    }
    if (root_ && !root_->HasParent())
        scene_->RootVisual()->AddChild(root_);
    if (root_)
    {
        root_->SetVisible(true);
        logInfo("[DynamicTerrain][RENDER] persistent server-scene root ready scene=",
                scene_->Name(), " mode=evictable-geographic-pages");
    }
}

void PersistentTerrainRenderer::ConfigureMaterial(
    const gz::rendering::MaterialPtr &material) const
{
    if (!material)
        return;
    material->SetDiffuse(1.0, 1.0, 1.0, 1.0);
    material->SetAmbient(1.0, 1.0, 1.0, 1.0);
    material->SetSpecular(0.0, 0.0, 0.0, 1.0);
    material->SetLightingEnabled(cfg_.visualLightingEnabled);
    material->SetCastShadows(cfg_.visualLightingEnabled &&
                             cfg_.visualCastShadows);
    material->SetReceiveShadows(cfg_.visualLightingEnabled &&
                                cfg_.visualReceiveShadows);
}

std::optional<PersistentTerrainRenderer::PageSlot>
PersistentTerrainRenderer::CreatePage(
    const TerrainSnapshot &snapshot, const TerrainPage &page)
{
    if (!page.mesh || !page.texture || !page.texture->Valid() ||
        page.textureName.empty())
        return std::nullopt;

    PageSlot result;
    result.key = page.key;
    result.pageIndex = page.index;
    result.geographicName = page.submeshName;
    result.sourceMesh = page.mesh;
    result.sourceTexture = page.texture;
    result.sourceTextureName = page.textureName;
    result.sourceTextureSize = page.textureSize;
    result.boundsMin = page.mesh->Min();
    result.boundsMax = page.mesh->Max();
    if (!MakePageResident(snapshot.resourcePrefix, snapshot.generation,
                          result, false))
    {
        DestroyPage(result);
        return std::nullopt;
    }
    return result;
}

bool PersistentTerrainRenderer::MakePageResident(
    const std::string &resourcePrefix, std::uint64_t generation,
    PageSlot &page, bool visible)
{
    if (page.gpuResident && page.geometry && page.visual && page.submesh &&
        page.material && page.meshReferenceHeld &&
        page.textureReferenceHeld)
    {
        page.visual->SetVisible(visible);
        return true;
    }

    if (page.geometry || page.visual || page.submesh || page.material ||
        page.meshReferenceHeld || page.textureReferenceHeld)
    {
        if (!UnloadPageGpu(page))
            return false;
    }
    if (!page.sourceMesh || !page.sourceTexture ||
        !page.sourceTexture->Valid() || page.sourceTextureName.empty())
        return false;

    const auto serial = ++renderSerial_;
    page.meshName = page.sourceMesh->Name();
    page.textureName = page.sourceTextureName;
    page.textureBytes = mipmappedRgbaBytes(
        page.sourceTextureSize, page.sourceTextureSize);
    try
    {
        page.geometry = scene_->CreateMesh(page.sourceMesh.get());
        if (!page.geometry)
            throw std::runtime_error(
                "CreateMesh failed for " + tileText(page.key));
        meshReferences_.Acquire(page.meshName);
        page.meshReferenceHeld = true;
        deferredMeshDeletes_.erase(page.meshName);
        if (page.geometry->SubMeshCount() != 1u)
            throw std::runtime_error(
                "CreateMesh did not return one geographic submesh for " +
                tileText(page.key));
        page.submesh = page.geometry->SubMeshByIndex(0);
        if (!page.submesh)
            throw std::runtime_error(
                "SubMeshByIndex failed for " + tileText(page.key));

        const std::string suffix = "_g" + std::to_string(generation) +
            "_p" + std::to_string(page.pageIndex) + "_r" +
            std::to_string(serial);
        page.visual = scene_->CreateVisual(
            resourcePrefix + "_visual" + suffix);
        if (!page.visual)
            throw std::runtime_error(
                "CreateVisual failed for " + tileText(page.key));
        page.visual->SetVisible(false);

        page.material = scene_->CreateMaterial(
            resourcePrefix + "_material" + suffix);
        if (!page.material)
            throw std::runtime_error(
                "CreateMaterial failed for " + tileText(page.key));

        textureReferences_.Acquire(page.textureName);
        page.textureReferenceHeld = true;
        deferredTextureDeletes_.erase(page.textureName);
        page.material->SetTexture(page.textureName, page.sourceTexture);
        ConfigureMaterial(page.material);
        page.submesh->SetMaterial(page.material, false);
        page.visual->AddGeometry(page.geometry);
        root_->AddChild(page.visual);
        page.visual->SetVisible(visible);
        page.offscreenFrames = 0;
        page.gpuResident = true;
        return true;
    }
    catch (const std::exception &e)
    {
        UnloadPageGpu(page);
        logError("[DynamicTerrain][RENDER] failed to make page resident page=",
                 tileText(page.key), " error=", e.what());
        return false;
    }
    catch (...)
    {
        UnloadPageGpu(page);
        logError("[DynamicTerrain][RENDER] failed to make page resident page=",
                 tileText(page.key), " unknown rendering exception");
        return false;
    }
}

PersistentTerrainRenderer::ResidencyCameraSet
PersistentTerrainRenderer::ResidencyCameras() const
{
    ResidencyCameraSet result;
    if (!scene_)
        return result;

    std::vector<bool> matched(cfg_.cameraNames.size(), false);
    std::unordered_set<const gz::rendering::Camera *> seen;
    try
    {
        for (unsigned int i = 0; i < scene_->SensorCount(); ++i)
        {
            auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
                scene_->SensorByIndex(i));
            if (!camera)
                continue;
            bool selected = cfg_.cameraNames.empty();
            for (std::size_t requestedIndex = 0;
                 requestedIndex < cfg_.cameraNames.size(); ++requestedIndex)
            {
                if (!scopedNameMatches(camera->Name(),
                                       cfg_.cameraNames[requestedIndex]))
                    continue;
                matched[requestedIndex] = true;
                selected = true;
            }
            if (selected && seen.insert(camera.get()).second)
                result.cameras.push_back(std::move(camera));
        }
    }
    catch (...)
    {
        result.cameras.clear();
        return result;
    }

    result.complete = !result.cameras.empty() &&
        (cfg_.cameraNames.empty() ||
         std::all_of(matched.begin(), matched.end(),
                     [](bool value) { return value; }));
    return result;
}

PersistentTerrainRenderer::CameraObservation
PersistentTerrainRenderer::ObservePageFromCamera(
    const PageSlot &page, const gz::rendering::CameraPtr &camera) const
{
    if (!camera)
        return CameraObservation::Unknown;

    if (std::dynamic_pointer_cast<gz::rendering::GpuRays>(camera) ||
        std::dynamic_pointer_cast<gz::rendering::WideAngleCamera>(camera) ||
        camera->ProjectionType() != gz::rendering::CPT_PERSPECTIVE)
        return CameraObservation::Unknown;

    const double nearClip = camera->NearClipPlane();
    const double farClip = camera->FarClipPlane();
    const double hfov = camera->HFOV().Radian();
    double aspect = camera->AspectRatio();
    if ((!std::isfinite(aspect) || aspect <= 0.0) &&
        camera->ImageHeight() != 0u)
    {
        aspect = static_cast<double>(camera->ImageWidth()) /
                 static_cast<double>(camera->ImageHeight());
    }
    const auto pose = camera->WorldPose();
    const auto rotation = pose.Rot();
    if (!std::isfinite(nearClip) || !std::isfinite(farClip) ||
        !std::isfinite(hfov) || !std::isfinite(aspect) ||
        nearClip < 0.0 || farClip <= nearClip || aspect <= 0.0 ||
        hfov <= 0.0 || hfov >= kPi || !finiteVector(pose.Pos()) ||
        !std::isfinite(rotation.W()) || !std::isfinite(rotation.X()) ||
        !std::isfinite(rotation.Y()) || !std::isfinite(rotation.Z()) ||
        !finiteVector(page.boundsMin) || !finiteVector(page.boundsMax))
        return CameraObservation::Unknown;

    const double margin = cfg_.visualTextureGuardM;
    const gz::math::Vector3d minimum = page.boundsMin -
        gz::math::Vector3d(margin, margin, margin);
    const gz::math::Vector3d maximum = page.boundsMax +
        gz::math::Vector3d(margin, margin, margin);
    try
    {
        const gz::math::Frustum frustum(
            nearClip, farClip, camera->HFOV(), aspect, pose);
        return frustum.Contains(gz::math::AxisAlignedBox(minimum, maximum))
            ? CameraObservation::Visible
            : CameraObservation::Offscreen;
    }
    catch (...)
    {
        return CameraObservation::Unknown;
    }
}

void PersistentTerrainRenderer::UpdateActivePageResidency()
{
    if (!cfg_.visualFrustumEviction || !active_)
        return;
    const auto cameraSet = ResidencyCameras();
    if (!cameraSet.complete)
    {
        for (auto &page : active_->pages)
            page.offscreenFrames = 0;
        if (!warnedNoResidencyCameras_)
        {
            logInfo("[DynamicTerrain][GPU-RESIDENCY] selected camera set is empty or incomplete; ",
                    "keeping the current page working set");
            warnedNoResidencyCameras_ = true;
        }
        return;
    }
    warnedNoResidencyCameras_ = false;

    std::size_t loaded = 0;
    std::size_t unloaded = 0;
    for (auto &page : active_->pages)
    {
        bool visible = false;
        bool unknown = false;
        for (const auto &camera : cameraSet.cameras)
        {
            const auto observation = ObservePageFromCamera(page, camera);
            visible = visible || observation == CameraObservation::Visible;
            unknown = unknown || observation == CameraObservation::Unknown;
        }
        const bool resident = page.gpuResident && page.geometry &&
                              page.visual && page.submesh && page.material &&
                              page.meshReferenceHeld &&
                              page.textureReferenceHeld;
        const bool hasGpuState = page.geometry || page.visual || page.submesh ||
                                 page.material || page.meshReferenceHeld ||
                                 page.textureReferenceHeld;
        if (visible)
        {
            page.offscreenFrames = 0;
            if (!resident)
            {
                if (MakePageResident(active_->resourcePrefix,
                                     active_->generation, page, true))
                    ++loaded;
            }
            else if (page.visual)
            {
                page.visual->SetVisible(true);
            }
            continue;
        }

        if (unknown)
        {
            page.offscreenFrames = 0;
            continue;
        }

        page.offscreenFrames = std::min(
            cfg_.visualOffscreenFrames, page.offscreenFrames + 1);
        if (hasGpuState &&
            page.offscreenFrames >= cfg_.visualOffscreenFrames)
        {
            if (UnloadPageGpu(page))
                ++unloaded;
        }
    }

    if (loaded != 0u || unloaded != 0u)
    {
        const auto residentPages = static_cast<std::size_t>(std::count_if(
            active_->pages.begin(), active_->pages.end(),
            [](const PageSlot &page) { return page.gpuResident; }));
        logInfo("[DynamicTerrain][GPU-RESIDENCY] generation=",
                active_->generation, " loaded=", loaded,
                " unloaded=", unloaded,
                " resident_pages=", residentPages,
                " total_pages=", active_->pages.size(),
                " named_textures=", textureReferences_.ResourceCount());
    }
}

std::optional<PersistentTerrainRenderer::Slot>
PersistentTerrainRenderer::CreateSlot(
    const std::shared_ptr<TerrainSnapshot> &snapshot,
    bool *resourcePressureCandidate)
{
    if (resourcePressureCandidate)
        *resourcePressureCandidate = false;
    if (!scene_ || !root_ || !snapshot || snapshot->pages.empty())
        return std::nullopt;

    Slot slot;
    slot.generation = snapshot->generation;
    slot.centerTile = snapshot->centerTile;
    slot.resourcePrefix = snapshot->resourcePrefix;
    slot.warmupFrames = cfg_.visualWarmupFrames;
    slot.estimatedTextureBytes = snapshot->estimatedTextureBytes;
    slot.pages.reserve(snapshot->pages.size());

    try
    {
        if (resourcePressureCandidate)
            *resourcePressureCandidate = true;
        for (const auto &page : snapshot->pages)
        {
            auto uploaded = CreatePage(*snapshot, page);
            if (!uploaded)
                throw std::runtime_error(
                    "invalid geographic page " + tileText(page.key));
            slot.pages.push_back(std::move(*uploaded));
        }

        if (resourcePressureCandidate)
            *resourcePressureCandidate = false;
        logInfo("[DynamicTerrain][RENDER] staging page set uploaded generation=",
                slot.generation,
                " pages=", slot.pages.size(),
                " estimated_resident_texture_mib=",
                static_cast<double>(slot.estimatedTextureBytes) /
                    (1024.0 * 1024.0),
                " unique_gpu_textures=", textureReferences_.ResourceCount(),
                " warmup_frames=", slot.warmupFrames,
                " lighting=", cfg_.visualLightingEnabled ? "on" : "off",
                " mapping=one-geographic-page-per-visual",
                " page_uv=west-east/north-south-direct");
        return slot;
    }
    catch (const std::exception &e)
    {
        DestroySlot(slot);
        logError("[DynamicTerrain][RENDER] failed to create staging page set generation=",
                 snapshot->generation, " error=", e.what());
        return std::nullopt;
    }
    catch (...)
    {
        DestroySlot(slot);
        logError("[DynamicTerrain][RENDER] failed to create staging page set generation=",
                 snapshot->generation, " unknown rendering exception");
        return std::nullopt;
    }
}

void PersistentTerrainRenderer::ClearActiveState()
{
    hasActive_.store(false, std::memory_order_release);
    activeGeneration_.store(0, std::memory_order_release);
    activeCenterX_.store(0, std::memory_order_relaxed);
    activeCenterY_.store(0, std::memory_order_relaxed);
    activeCenterZ_.store(0, std::memory_order_relaxed);
}

std::optional<PersistentTerrainRenderer::Slot>
PersistentTerrainRenderer::CreateSlotWithRecovery(
    const std::shared_ptr<TerrainSnapshot> &snapshot)
{
    bool pressureCandidate = false;
    auto slot = CreateSlot(snapshot, &pressureCandidate);
    if (slot || !pressureCandidate || !active_)
        return slot;

    const auto oldGeneration = active_->generation;
    logError("[DynamicTerrain][VRAM-RECOVERY] staging allocation failed for generation=",
             snapshot ? snapshot->generation : 0,
             "; releasing active generation=", oldGeneration,
             " before one low-memory retry");
    DestroySlot(*active_);
    active_.reset();
    ClearActiveState();

    bool retryPressure = false;
    slot = CreateSlot(snapshot, &retryPressure);
    if (slot)
    {
        slot->warmupFrames = 0;
        logInfo("[DynamicTerrain][VRAM-RECOVERY] low-memory staging upload succeeded generation=",
                slot->generation, " old_generation_released=", oldGeneration);
    }
    else
    {
        logError("[DynamicTerrain][VRAM-RECOVERY] low-memory staging retry also failed generation=",
                 snapshot ? snapshot->generation : 0,
                 "; renderer remains without an active terrain until retry");
    }
    return slot;
}

void PersistentTerrainRenderer::ApplyPendingTexture()
{
    std::optional<TextureUpdate> update;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (pendingTexture_)
        {
            update = std::move(pendingTexture_);
            pendingTexture_.reset();
        }
    }
    if (!update || update->pages.empty())
        return;

    Slot *target = nullptr;
    if (staging_ && staging_->generation == update->generation)
        target = &*staging_;
    else if (active_ && active_->generation == update->generation)
        target = &*active_;
    if (!target)
    {
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][RENDER] dropped stale page update generation=",
                    update->generation,
                    " active=", active_ ? active_->generation : 0,
                    " staging=", staging_ ? staging_->generation : 0);
        return;
    }

    std::size_t applied = 0;
    std::size_t rejected = 0;
    for (const auto &pageUpdate : update->pages)
    {
        auto pageIt = std::find_if(
            target->pages.begin(), target->pages.end(),
            [&](const PageSlot &page) { return page.key == pageUpdate.pageKey; });
        if (pageIt == target->pages.end() ||
            pageIt->geographicName != pageUpdate.submeshName ||
            !pageUpdate.texture || !pageUpdate.texture->Valid() ||
            pageUpdate.textureName.empty())
        {
            ++rejected;
            continue;
        }
        if (pageIt->sourceTextureName == pageUpdate.textureName)
            continue;

        if (!pageIt->gpuResident || !pageIt->geometry || !pageIt->submesh ||
            !pageIt->material || !pageIt->textureReferenceHeld)
        {
            pageIt->sourceTexture = pageUpdate.texture;
            pageIt->sourceTextureName = pageUpdate.textureName;
            pageIt->sourceTextureSize = pageUpdate.textureSize;
            ++applied;
            continue;
        }

        std::string preparedSourceName = pageUpdate.textureName;
        auto preparedSourceTexture = pageUpdate.texture;
        PageSlot replacementResource;
        replacementResource.textureName = pageUpdate.textureName;
        replacementResource.textureBytes = mipmappedRgbaBytes(
            pageUpdate.textureSize, pageUpdate.textureSize);
        replacementResource.material = scene_->CreateMaterial(
            target->resourcePrefix + "_material_g" +
            std::to_string(update->generation) + "_u" +
            std::to_string(++materialSerial_));
        if (!replacementResource.material)
        {
            ++rejected;
            continue;
        }

        try
        {
            textureReferences_.Acquire(replacementResource.textureName);
            replacementResource.textureReferenceHeld = true;
            deferredTextureDeletes_.erase(replacementResource.textureName);
            replacementResource.material->SetTexture(
                replacementResource.textureName, pageUpdate.texture);
            ConfigureMaterial(replacementResource.material);
            pageIt->submesh->SetMaterial(replacementResource.material, false);
            pageIt->material.swap(replacementResource.material);
            pageIt->textureName.swap(replacementResource.textureName);
            std::swap(pageIt->textureBytes,
                      replacementResource.textureBytes);
            std::swap(pageIt->textureReferenceHeld,
                      replacementResource.textureReferenceHeld);
            pageIt->sourceTextureName.swap(preparedSourceName);
            pageIt->sourceTexture.swap(preparedSourceTexture);
            pageIt->sourceTextureSize = pageUpdate.textureSize;

            DestroyPageMaterial(replacementResource);
            ++applied;
        }
        catch (const std::exception &e)
        {
            DestroyPage(replacementResource);
            ++rejected;
            logError("[DynamicTerrain][GPU-GC] page replacement failed page=",
                     tileText(pageUpdate.pageKey), " error=", e.what());
        }
        catch (...)
        {
            DestroyPage(replacementResource);
            ++rejected;
            logError("[DynamicTerrain][GPU-GC] page replacement failed page=",
                     tileText(pageUpdate.pageKey), " unknown rendering exception");
        }
    }

    logInfo("[DynamicTerrain][RENDER] progressive page textures applied generation=",
            update->generation, " applied=", applied,
            " rejected=", rejected,
            " resident_named_textures=", textureReferences_.ResourceCount(),
            " destroyed_textures=", destroyedTextures_);
}

void PersistentTerrainRenderer::OnPreRender()
{
    if (renderShuttingDown_.load(std::memory_order_acquire))
        return;
    if (!scene_ || !root_)
        FindScene();
    if (!scene_ || !root_)
        return;

    DrainDeferredPageReleases();
    DrainDeferredTextureReleases();
    DrainDeferredTextureDeletes();
    DrainDeferredMeshDeletes();

    if (retired_)
    {
        const auto generation = retired_->generation;
        DestroySlot(*retired_);
        retired_.reset();
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][GPU-GC] pre-render safety cleanup generation=",
                    generation);
    }

    if (staging_)
    {
        std::shared_ptr<TerrainSnapshot> newer;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (pendingSnapshot_ &&
                pendingSnapshot_->generation > staging_->generation)
            {
                newer = std::move(pendingSnapshot_);
                pendingSnapshot_.reset();
            }
        }
        if (newer)
        {
            const auto oldGeneration = staging_->generation;
            DestroySlot(*staging_);
            staging_.reset();
            staging_ = CreateSlotWithRecovery(newer);
            if (cfg_.diagnostics)
                logInfo("[DynamicTerrain][RENDER] superseded hidden staging generation=",
                        oldGeneration, " by generation=", newer->generation);
        }
    }

    if (!staging_)
    {
        std::shared_ptr<TerrainSnapshot> pending;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (pendingSnapshot_ &&
                (!active_ || pendingSnapshot_->generation > active_->generation))
            {
                pending = std::move(pendingSnapshot_);
                pendingSnapshot_.reset();
            }
        }
        if (pending)
            staging_ = CreateSlotWithRecovery(pending);
    }

    ApplyPendingTexture();

    if (staging_)
    {
        if (staging_->warmupFrames > 0)
        {
            --staging_->warmupFrames;
        }
        else
        {
            if (active_)
                for (auto &page : active_->pages)
                    if (page.visual)
                        page.visual->SetVisible(false);
            for (auto &page : staging_->pages)
                if (page.visual)
                    page.visual->SetVisible(true);
            if (active_)
                retired_ = std::move(active_);
            active_ = std::move(staging_);
            staging_.reset();
            activeCenterX_.store(active_->centerTile.x, std::memory_order_relaxed);
            activeCenterY_.store(active_->centerTile.y, std::memory_order_relaxed);
            activeCenterZ_.store(active_->centerTile.z, std::memory_order_relaxed);
            activeGeneration_.store(active_->generation, std::memory_order_release);
            hasActive_.store(true, std::memory_order_release);
            logInfo("[DynamicTerrain][RENDER] atomic A/B swap generation=",
                    active_->generation,
                    " pages=", active_->pages.size(),
                    " resident_named_textures=",
                    textureReferences_.ResourceCount());
        }
    }

    UpdateActivePageResidency();
}

void PersistentTerrainRenderer::ReleaseTextureReference(PageSlot &page)
{
    if (!page.textureReferenceHeld || page.textureName.empty())
        return;
    const std::string textureName = page.textureName;
    page.textureReferenceHeld = false;
    if (!textureReferences_.Release(textureName))
        return;
    if (destroyUnreferencedOgre2Texture(scene_, textureName))
    {
        ++destroyedTextures_;
        deferredTextureDeletes_.erase(textureName);
    }
    else if (ogre2TextureExists(scene_, textureName))
    {
        deferredTextureDeletes_.insert(textureName);
        logError("[DynamicTerrain][GPU-GC] Ogre2 texture deletion deferred name=",
                 textureName);
    }
}

bool PersistentTerrainRenderer::DetachAndDestroyMaterial(
    gz::rendering::MaterialPtr &material)
{
    if (!material)
        return true;

    try
    {
        material->ClearTexture();
    }
    catch (...)
    {
        return false;
    }

    try
    {
        scene_->DestroyMaterial(material);
        ++destroyedMaterials_;
    }
    catch (...)
    {
    }
    material.reset();
    return true;
}

void PersistentTerrainRenderer::DestroyPageMaterial(PageSlot &page)
{
    if (DetachAndDestroyMaterial(page.material))
    {
        ReleaseTextureReference(page);
        return;
    }

    PageSlot deferred;
    deferred.material = std::move(page.material);
    deferred.textureName = std::move(page.textureName);
    deferred.textureReferenceHeld = page.textureReferenceHeld;
    page.textureReferenceHeld = false;
    deferredTextureReleases_.push_back(std::move(deferred));
    logError("[DynamicTerrain][GPU-GC] ClearTexture failed; cleanup deferred");
}

void PersistentTerrainRenderer::DrainDeferredTextureReleases()
{
    auto it = deferredTextureReleases_.begin();
    while (it != deferredTextureReleases_.end())
    {
        if (!DetachAndDestroyMaterial(it->material))
        {
            ++it;
            continue;
        }
        ReleaseTextureReference(*it);
        it = deferredTextureReleases_.erase(it);
    }
}

void PersistentTerrainRenderer::DrainDeferredTextureDeletes()
{
    auto it = deferredTextureDeletes_.begin();
    while (it != deferredTextureDeletes_.end())
    {
        if (textureReferences_.References(*it) != 0u)
        {
            ++it;
            continue;
        }
        if (!ogre2TextureExists(scene_, *it))
        {
            it = deferredTextureDeletes_.erase(it);
            continue;
        }
        if (destroyUnreferencedOgre2Texture(scene_, *it))
        {
            ++destroyedTextures_;
            it = deferredTextureDeletes_.erase(it);
            continue;
        }
        ++it;
    }
}

void PersistentTerrainRenderer::ReleaseMeshResource(PageSlot &page)
{
    if (page.meshName.empty())
        return;
    const std::string meshName = std::move(page.meshName);
    if (page.meshReferenceHeld)
    {
        page.meshReferenceHeld = false;
        if (!meshReferences_.Release(meshName))
            return;
    }
    else if (meshReferences_.References(meshName) != 0u)
    {
        return;
    }
    if (destroyUnreferencedOgre2Mesh(scene_, meshName))
    {
        ++destroyedMeshes_;
        deferredMeshDeletes_.erase(meshName);
    }
    else if (ogre2MeshExists(scene_, meshName))
    {
        deferredMeshDeletes_.insert(meshName);
        logError("[DynamicTerrain][GPU-GC] Ogre2 mesh deletion deferred name=",
                 meshName);
    }
}

void PersistentTerrainRenderer::DrainDeferredMeshDeletes()
{
    auto it = deferredMeshDeletes_.begin();
    while (it != deferredMeshDeletes_.end())
    {
        if (meshReferences_.References(*it) != 0u)
        {
            ++it;
            continue;
        }
        if (!ogre2MeshExists(scene_, *it))
        {
            it = deferredMeshDeletes_.erase(it);
            continue;
        }
        if (destroyUnreferencedOgre2Mesh(scene_, *it))
        {
            ++destroyedMeshes_;
            it = deferredMeshDeletes_.erase(it);
            continue;
        }
        ++it;
    }
}

bool PersistentTerrainRenderer::UnloadPageGpu(PageSlot &page)
{
    page.gpuResident = false;
    if (!scene_)
        return !page.geometry && !page.visual && !page.material;

    if (page.visual)
    {
        try
        {
            page.visual->SetVisible(false);
            page.visual->RemoveGeometries();
        }
        catch (...) {}
    }

    bool geometryDestroyed = !page.geometry;
    if (page.geometry)
    {
        try
        {
            page.geometry->Destroy();
            geometryDestroyed = true;
        }
        catch (...) {}
    }

    if (!geometryDestroyed)
        return false;
    page.submesh.reset();
    page.geometry.reset();
    if (page.visual)
    {
        try { scene_->DestroyVisual(page.visual, true); } catch (...) {}
        page.visual.reset();
    }

    ReleaseMeshResource(page);

    DestroyPageMaterial(page);
    page.textureName.clear();
    page.textureBytes = 0u;
    return true;
}

void PersistentTerrainRenderer::DestroyPage(PageSlot &page)
{
    if (!UnloadPageGpu(page))
    {
        deferredPageReleases_.push_back(std::move(page));
        logError("[DynamicTerrain][GPU-GC] geometry destruction failed; page cleanup deferred");
        return;
    }
    page.sourceMesh.reset();
    page.sourceTexture.reset();
    page.sourceTextureName.clear();
    page.geographicName.clear();
}

void PersistentTerrainRenderer::DrainDeferredPageReleases()
{
    if (deferredPageReleases_.empty())
        return;
    auto deferred = std::move(deferredPageReleases_);
    deferredPageReleases_.clear();
    for (auto &page : deferred)
        DestroyPage(page);
}

void PersistentTerrainRenderer::DestroySlot(Slot &slot)
{
    if (!scene_)
        return;

    const auto materialCountBefore = destroyedMaterials_;
    const auto textureCountBefore = destroyedTextures_;
    const auto meshCountBefore = destroyedMeshes_;
    for (auto &page : slot.pages)
        DestroyPage(page);
    slot.pages.clear();

    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][GPU-GC] destroyed terrain slot generation=",
                slot.generation,
                " materials=", destroyedMaterials_ - materialCountBefore,
                " textures=", destroyedTextures_ - textureCountBefore,
                " meshes=", destroyedMeshes_ - meshCountBefore,
                " remaining_named_textures=",
                textureReferences_.ResourceCount(),
                " total_textures_destroyed=", destroyedTextures_);
}

void PersistentTerrainRenderer::OnPostRender()
{
    if (retired_)
    {
        const auto generation = retired_->generation;
        DestroySlot(*retired_);
        retired_.reset();
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][RENDER] retired old mesh after completed frame generation=",
                    generation);
    }
}

void PersistentTerrainRenderer::OnRenderTeardown()
{
    if (renderShuttingDown_.exchange(true, std::memory_order_acq_rel))
        return;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        pendingSnapshot_.reset();
        pendingTexture_.reset();
    }
    if (retired_)
        DestroySlot(*retired_);
    if (staging_)
        DestroySlot(*staging_);
    if (active_)
        DestroySlot(*active_);
    DrainDeferredPageReleases();
    DrainDeferredTextureReleases();
    DrainDeferredTextureDeletes();
    DrainDeferredMeshDeletes();
    retired_.reset();
    staging_.reset();
    active_.reset();
    if (!deferredPageReleases_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredPageReleases_.size(),
                 " pages whose native geometry could not be destroyed");
    if (!deferredTextureReleases_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredTextureReleases_.size(),
                 " materials whose texture binding could not be cleared");
    if (!deferredTextureDeletes_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredTextureDeletes_.size(),
                 " Ogre2 texture deletions for backend teardown");
    if (!deferredMeshDeletes_.empty())
        logError("[DynamicTerrain][GPU-GC] render teardown retained ",
                 deferredMeshDeletes_.size(),
                 " Ogre2 mesh deletions for backend teardown");
    ClearActiveState();
    if (textureReferences_.ResourceCount() != 0u)
        logError("[DynamicTerrain][GPU-GC] render teardown left ",
                 textureReferences_.ResourceCount(),
                 " named texture references pending");
    if (meshReferences_.ResourceCount() != 0u)
        logError("[DynamicTerrain][GPU-GC] render teardown left ",
                 meshReferences_.ResourceCount(),
                 " named mesh references pending");
    if (scene_ && root_)
    {
        try { scene_->DestroyVisual(root_, true); } catch (...) {}
    }
    root_.reset();
    scene_.reset();
}

}
