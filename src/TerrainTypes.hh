#pragma once

#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/Entity.hh>

#include <sdf/Element.hh>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dynamic_terrain
{
namespace fs = std::filesystem;

constexpr double kPi = 3.14159265358979323846;
constexpr double kMercatorLatLimit = 85.05112878;

struct TileKey
{
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const TileKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator!=(const TileKey &other) const { return !(*this == other); }
};

struct TileKeyHash
{
    std::size_t operator()(const TileKey &key) const noexcept;
};

struct TileCoordF
{
    double x{0.0};
    double y{0.0};
};

struct TileBounds
{
    double north{0.0};
    double south{0.0};
    double west{0.0};
    double east{0.0};
};

struct TileRect
{
    int minX{0};
    int minY{0};
    int maxX{0};
    int maxY{0};
    int zoom{0};

    int Width() const { return maxX - minX + 1; }
    int Height() const { return maxY - minY + 1; }
    bool Contains(const TileKey &key) const
    {
        return key.z == zoom && key.x >= minX && key.x <= maxX &&
               key.y >= minY && key.y <= maxY;
    }
};

struct Provider
{
    std::string name;
    std::string url;
    std::string extension;
    bool quadKey{false};
};

struct Config
{
    std::string modelName;

    std::string imageryProvider{"google_satellite"};
    std::string imageryUrl;
    std::string imageryExtension;
    std::string imageryToken;

    std::string elevationProvider{"terrarium"};
    std::string elevationUrl;
    std::string elevationToken;
    int elevationMaxZoom{15};
    double visualRadiusM{7500.0};
    bool visualGui{false};
    int visualGeometryZoom{14};
    int visualElevationZoom{13};
    int visualMeshCellsPerTile{64};
    int visualMaxMeshCells{768};
    int visualTextureSize{4096};
    int visualPageTextureMaxSize{2048};
    std::size_t visualPageCacheMb{128};
    int visualAtlasPagePixels{1280};
    int visualAtlasMaxSize{16384};
    double visualTextureGuardM{300.0};
    std::string visualDetailMode{"all"};
    std::string visualDetailCameraName{"camera_down"};
    double visualDetailRadiusM{2500.0};
    int visualDetailZoom{17};
    int visualBootstrapImageryZoom{12};
    std::vector<std::pair<double, int>> visualImageryLodTable{
        {1000.0, 17},
        {2500.0, 16},
        {5500.0, 15},
        {1000000.0, 14},
    };
    double visualRecenterDistanceM{1800.0};
    int visualWarmupFrames{1};
    bool visualFrustumEviction{true};
    int visualOffscreenFrames{30};
    bool visualRefineTexture{true};
    int visualRecenterReadyZoom{14};
    int visualRefineMaxSourceTilesPerBatch{32};
    bool visualLightingEnabled{false};
    bool visualCastShadows{false};
    bool visualReceiveShadows{false};
    int downloadConcurrency{4};
    int downloadPerHost{1};
    int downloadRetries{3};
    unsigned int httpTimeoutMs{5000};
    std::string userAgent{"gz-dynamic-terrain/0.19.0"};
    std::string cacheDir{"~/.cache/gz_dynamic_terrain"};
    std::size_t decodedDemCacheMb{256};
    bool dynamicZoom{true};
    int staticZoom{17};
    int minZoom{12};
    int maxZoom{18};
    std::vector<std::pair<double, int>> zoomTable{
        {80.0, 18},
        {180.0, 17},
        {400.0, 16},
        {900.0, 15},
        {1800.0, 14},
        {4000.0, 13},
        {1000000.0, 12},
    };
    int radiusTiles{1};
    int meshCells{32};
    int heightmapSize{257};
    bool enableCollision{true};
    double collisionOverlapM{0.75};
    double collisionRecenterFraction{0.35};

    bool alignOriginToGround{true};
    double zOffsetM{0.0};

    bool startupPreload{true};
    bool startupSafetyGround{true};
    double startupSafetySizeM{800.0};
    double startupSafetyThicknessM{0.20};
    double startupSafetyTopZ{0.0};
    double startupSafetyRemoveDelaySec{2.0};

    double updatePeriodSec{0.20};
    double retryDelaySec{1.5};
    bool diagnostics{true};
    double statusPeriodSec{2.0};
    std::string coverageMode{"camera_projection"};
    std::vector<std::string> cameraNames;
};

class ResourceReferenceCounter
{
public:
    std::size_t Acquire(const std::string &name);
    bool Release(const std::string &name);
    std::size_t References(const std::string &name) const;
    std::size_t ResourceCount() const { return references_.size(); }

private:
    std::unordered_map<std::string, std::size_t> references_;
};

struct RegisteredModelConfig
{
    gz::sim::Entity entity{gz::sim::kNullEntity};
    Config config;
};

template <typename T>
T clampValue(T value, T lo, T hi)
{
    return std::max(lo, std::min(value, hi));
}

std::mutex &diagnosticLogMutex();

template <typename... Args>
void logInfo(Args &&...args)
{
    std::lock_guard<std::mutex> lock(diagnosticLogMutex());
    (std::cout << ... << std::forward<Args>(args)) << std::endl;
}

template <typename... Args>
void logError(Args &&...args)
{
    std::lock_guard<std::mutex> lock(diagnosticLogMutex());
    (std::cerr << ... << std::forward<Args>(args)) << std::endl;
}

std::string lower(std::string value);
std::string providerKey(std::string value);
std::string expandHome(std::string path);
std::string xmlEscape(std::string value);
std::string tileText(const TileKey &key);
std::string redactUrl(std::string url);

TileKey latLonToTile(double latDeg, double lonDeg, int zoom);
TileCoordF latLonToTileFraction(double latDeg, double lonDeg, int zoom);
double tileXToLon(double x, int zoom);
double tileYToLat(double y, int zoom);
TileBounds tileRectBounds(const TileRect &rect);
TileRect boundsToTileRect(const TileBounds &bounds, int zoom, int halo = 0);
Provider resolveImageryProvider(const Config &cfg);
std::string buildUrl(const Provider &provider, const TileKey &key, const std::string &token);
int zoomForAltitude(const Config &cfg, double altitude);
int validHeightmapSize(int requested);

gz::math::Vector3d localFromGeodetic(const gz::math::SphericalCoordinates &spherical, double latDeg, double lonDeg, double elevationM);
gz::math::Vector3d geodeticFromLocal(const gz::math::SphericalCoordinates &spherical, const gz::math::Vector3d &local);

std::size_t mipmappedRgbaBytes(int width, int height);

Config parseTerrainConfig(const std::shared_ptr<const sdf::Element> &sdf);
void normalizeConfig(Config &cfg);

void registerModelConfig(gz::sim::Entity entity, const Config &cfg);
void unregisterModelConfig(gz::sim::Entity entity);
std::vector<RegisteredModelConfig> registeredModelConfigs();

}
