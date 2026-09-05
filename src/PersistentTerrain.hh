#pragma once

#include "TerrainTypes.hh"
#include "TileStore.hh"

#include <gz/common/Event.hh>
#include <gz/common/Image.hh>
#include <gz/common/Mesh.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/EventManager.hh>

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dynamic_terrain
{

struct TerrainPage
{
    TileKey key;
    std::size_t index{0};
    std::string submeshName;
    int imageryZoom{0};
    int textureSize{256};
    int targetImageryZoom{0};
    std::shared_ptr<gz::common::Image> texture;
    std::string textureName;
    std::shared_ptr<gz::common::Mesh> mesh;
};

struct TerrainSnapshot
{
    std::uint64_t generation{0};
    TileKey centerTile;
    TileRect geometryRect;
    TileBounds bounds;
    gz::math::Vector3d patchCenterLocal;
    std::string resourcePrefix;
    int cellsPerTile{0};
    int cellsX{0};
    int cellsY{0};
    std::vector<TerrainPage> pages;
    std::size_t estimatedTextureBytes{0};
};

struct TexturePageUpdate
{
    std::size_t pageIndex{0};
    TileKey pageKey;
    std::string submeshName;
    int imageryZoom{0};
    int textureSize{256};
    std::shared_ptr<gz::common::Image> texture;
    std::string textureName;
};

struct TextureUpdate
{
    std::uint64_t generation{0};
    std::size_t changedPageCount{0};
    std::vector<TexturePageUpdate> pages;
};

class PersistentTerrainBuilder
{
public:
    explicit PersistentTerrainBuilder(std::shared_ptr<TileStore> store);

    std::shared_ptr<TerrainSnapshot> BuildBootstrap(const TileKey &center,
                                                    std::uint64_t generation,
                                                    std::string &error);
    std::optional<TextureUpdate> BuildTextureStage(
        const TerrainSnapshot &snapshot,
        const std::vector<std::size_t> &pageIndices,
        int sourceZoom,
        std::string &error);

    std::vector<std::size_t> ProgressivePageOrder(
        const TerrainSnapshot &snapshot) const;
    int TargetZoomForPage(const TerrainSnapshot &snapshot,
                          std::size_t pageIndex) const;

    void ApplyTextureUpdate(TerrainSnapshot &snapshot,
                            TextureUpdate &update) const;

    TileKey SnappedCenter(double latDeg, double lonDeg) const;

    struct PageCacheStats
    {
        std::size_t pages{0};
        std::size_t bytes{0};
        std::size_t limitBytes{0};
    };
    PageCacheStats CachedPageStats() const;

private:
    struct CachedPageTexture
    {
        int imageryZoom{0};
        int textureSize{256};
        std::shared_ptr<gz::common::Image> texture;
        std::string textureName;
        std::uint64_t touch{0};
        std::size_t bytes{0};
    };

    std::optional<CachedPageTexture> CachedTextureForPage(
        const TileKey &key) const;
    void RememberPageTexture(const TileKey &key, int imageryZoom,
                             int textureSize,
                             const std::shared_ptr<gz::common::Image> &texture,
                             const std::string &textureName) const;

    int RadiusTiles(const TileKey &center) const;
    int EffectiveCellsPerTile(const TileRect &rect) const;
    int PageTargetZoom(const TerrainSnapshot &snapshot,
                       const TileKey &pageKey) const;
    bool SourceTileWanted(const TerrainSnapshot &snapshot,
                          const TileKey &key) const;
    std::vector<TileKey> PageSourceKeys(const TileKey &pageKey,
                                        int sourceZoom) const;
    cv::Mat ComposeBootstrapPage(const TileKey &pageKey,
                                 int geometryZoom,
                                 int bootstrapZoom,
                                 int outputSize) const;
    cv::Mat ComposeRefinedPage(const TileKey &pageKey,
                               int geometryZoom,
                               int sourceZoom,
                               int outputSize,
                               const std::unordered_set<TileKey, TileKeyHash> &failed) const;
    std::shared_ptr<gz::common::Image> ToImage(const cv::Mat &bgr) const;

    std::shared_ptr<TileStore> store_;

    mutable std::mutex pageTextureCacheMutex_;
    mutable std::unordered_map<TileKey, CachedPageTexture, TileKeyHash>
        pageTextureCache_;
    mutable std::uint64_t pageTextureCacheTouch_{0};
    mutable std::size_t pageTextureCacheBytes_{0};
    static constexpr std::size_t kPageTextureCacheLimit = 24u;
};

class PersistentTerrainRenderer
{
public:
    PersistentTerrainRenderer(Config config, gz::sim::EventManager &events);
    ~PersistentTerrainRenderer();

    void QueueSnapshot(std::shared_ptr<TerrainSnapshot> snapshot);
    void QueueTexture(TextureUpdate update);
    std::uint64_t ActiveGeneration() const;
    bool HasActiveTerrain() const;
    std::optional<TileKey> ActiveCenterTile() const;

private:
    enum class CameraObservation
    {
        Visible,
        Offscreen,
        Unknown
    };

    struct ResidencyCameraSet
    {
        std::vector<gz::rendering::CameraPtr> cameras;
        bool complete{false};
    };

    struct PageSlot
    {
        TileKey key;
        std::size_t pageIndex{0};
        std::string geographicName;
        std::shared_ptr<gz::common::Mesh> sourceMesh;
        std::shared_ptr<gz::common::Image> sourceTexture;
        std::string sourceTextureName;
        int sourceTextureSize{256};
        gz::math::Vector3d boundsMin;
        gz::math::Vector3d boundsMax;
        int offscreenFrames{0};
        gz::rendering::VisualPtr visual;
        gz::rendering::MeshPtr geometry;
        gz::rendering::SubMeshPtr submesh;
        gz::rendering::MaterialPtr material;
        std::string meshName;
        bool meshReferenceHeld{false};
        std::string textureName;
        bool textureReferenceHeld{false};
        std::size_t textureBytes{0};
        bool gpuResident{false};
    };

    struct Slot
    {
        std::uint64_t generation{0};
        TileKey centerTile;
        std::string resourcePrefix;
        std::vector<PageSlot> pages;
        std::size_t estimatedTextureBytes{0};
        int warmupFrames{0};
    };

    void FindScene();
    void OnPreRender();
    void OnPostRender();
    void OnRenderTeardown();
    std::optional<Slot> CreateSlot(
        const std::shared_ptr<TerrainSnapshot> &snapshot,
        bool *resourcePressureCandidate = nullptr);
    std::optional<Slot> CreateSlotWithRecovery(
        const std::shared_ptr<TerrainSnapshot> &snapshot);
    std::optional<PageSlot> CreatePage(
        const TerrainSnapshot &snapshot, const TerrainPage &page);
    bool MakePageResident(const std::string &resourcePrefix,
                          std::uint64_t generation,
                          PageSlot &page, bool visible);
    bool UnloadPageGpu(PageSlot &page);
    ResidencyCameraSet ResidencyCameras() const;
    CameraObservation ObservePageFromCamera(
        const PageSlot &page,
        const gz::rendering::CameraPtr &camera) const;
    void UpdateActivePageResidency();
    void ConfigureMaterial(const gz::rendering::MaterialPtr &material) const;
    void DestroyPage(PageSlot &page);
    void DrainDeferredPageReleases();
    bool DetachAndDestroyMaterial(gz::rendering::MaterialPtr &material);
    void DestroyPageMaterial(PageSlot &page);
    void DrainDeferredTextureReleases();
    void DrainDeferredTextureDeletes();
    void ReleaseMeshResource(PageSlot &page);
    void DrainDeferredMeshDeletes();
    void ReleaseTextureReference(PageSlot &page);
    void ClearActiveState();
    void DestroySlot(Slot &slot);
    void ApplyPendingTexture();

    Config cfg_;
    mutable std::mutex queueMutex_;
    std::shared_ptr<TerrainSnapshot> pendingSnapshot_;
    std::optional<TextureUpdate> pendingTexture_;

    gz::rendering::ScenePtr scene_;
    gz::rendering::VisualPtr root_;
    std::optional<Slot> active_;
    std::optional<Slot> staging_;
    std::optional<Slot> retired_;

    std::atomic<std::uint64_t> activeGeneration_{0};
    std::atomic<bool> hasActive_{false};
    std::atomic<int> activeCenterX_{0};
    std::atomic<int> activeCenterY_{0};
    std::atomic<int> activeCenterZ_{0};
    std::atomic<std::uint64_t> renderSerial_{0};
    std::atomic<std::uint64_t> materialSerial_{0};
    std::atomic<bool> renderShuttingDown_{false};
    std::uint64_t rendererId_{0};
    ResourceReferenceCounter textureReferences_;
    ResourceReferenceCounter meshReferences_;
    std::vector<PageSlot> deferredPageReleases_;
    std::vector<PageSlot> deferredTextureReleases_;
    std::unordered_set<std::string> deferredTextureDeletes_;
    std::unordered_set<std::string> deferredMeshDeletes_;
    std::uint64_t destroyedMaterials_{0};
    std::uint64_t destroyedTextures_{0};
    std::uint64_t destroyedMeshes_{0};
    bool warnedNoResidencyCameras_{false};

    gz::common::ConnectionPtr preRenderConnection_;
    gz::common::ConnectionPtr postRenderConnection_;
    gz::common::ConnectionPtr teardownConnection_;
};

}
