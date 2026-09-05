#include "TerrainTypes.hh"

#include <gz/math/Angle.hh>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace dynamic_terrain
{
namespace
{
template <typename T>
void readSdf(const std::shared_ptr<const sdf::Element> &sdf,
             const std::string &name, T &value)
{
    if (sdf && sdf->HasElement(name))
        value = sdf->Get<T>(name);
}

std::vector<std::string> splitCommaList(const std::string &text)
{
    std::vector<std::string> result;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        const auto first = token.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
            continue;
        const auto last = token.find_last_not_of(" \t\r\n");
        result.emplace_back(token.substr(first, last - first + 1));
    }
    return result;
}

std::vector<std::pair<double, int>> parseZoomTable(const std::string &text)
{
    std::vector<std::pair<double, int>> result;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        const auto colon = token.find(':');
        if (colon == std::string::npos)
            continue;
        try
        {
            result.emplace_back(std::stod(token.substr(0, colon)),
                                std::stoi(token.substr(colon + 1)));
        }
        catch (...)
        {
        }
    }
    std::sort(result.begin(), result.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    return result;
}

std::string quadKey(int tileX, int tileY, int zoom)
{
    std::string out;
    out.reserve(static_cast<std::size_t>(zoom));
    for (int i = zoom; i > 0; --i)
    {
        char digit = '0';
        const int mask = 1 << (i - 1);
        if (tileX & mask)
            ++digit;
        if (tileY & mask)
            digit += 2;
        out.push_back(digit);
    }
    return out;
}

void replaceAll(std::string &value, const std::string &needle,
                const std::string &replacement)
{
    if (needle.empty())
        return;
    std::size_t pos = 0;
    while ((pos = value.find(needle, pos)) != std::string::npos)
    {
        value.replace(pos, needle.size(), replacement);
        pos += replacement.size();
    }
}

Provider builtinImageryProvider(const std::string &requested)
{
    const std::string name = providerKey(requested);
    if (name == "google_street")
        return {"google_street",
                "https://mt{s}.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "google_terrain")
        return {"google_terrain",
                "https://mt{s}.google.com/vt/v=t,r&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "google_hybrid")
        return {"google_hybrid",
                "https://mt{s}.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "google_labels")
        return {"google_labels",
                "https://mt{s}.google.com/vt/lyrs=h&hl=en&x={x}&y={y}&z={z}",
                "png", false};
    if (name == "bing_road")
        return {"bing_road",
                "https://ecn.t{s}.tiles.virtualearth.net/tiles/r{q}.png?g=2981&mkt=en",
                "png", true};
    if (name == "bing_satellite")
        return {"bing_satellite",
                "https://ecn.t{s}.tiles.virtualearth.net/tiles/a{q}.jpg?g=2981&mkt=en",
                "jpg", true};
    if (name == "bing_hybrid")
        return {"bing_hybrid",
                "https://ecn.t{s}.tiles.virtualearth.net/tiles/h{q}.jpg?g=2981&mkt=en",
                "jpg", true};
    return {"google_satellite",
            "https://mt{s}.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}",
            "jpg", false};
}

std::unordered_map<gz::sim::Entity, Config> &modelConfigRegistry()
{
    static std::unordered_map<gz::sim::Entity, Config> registry;
    return registry;
}

std::mutex &modelConfigRegistryMutex()
{
    static std::mutex mutex;
    return mutex;
}
} // namespace

std::size_t TileKeyHash::operator()(const TileKey &key) const noexcept
{
    std::size_t h = std::hash<int>{}(key.z);
    h ^= std::hash<int>{}(key.x) + 0x9e3779b9u + (h << 6u) + (h >> 2u);
    h ^= std::hash<int>{}(key.y) + 0x9e3779b9u + (h << 6u) + (h >> 2u);
    return h;
}

std::size_t ResourceReferenceCounter::Acquire(const std::string &name)
{
    if (name.empty())
        return 0;
    return ++references_[name];
}

bool ResourceReferenceCounter::Release(const std::string &name)
{
    const auto it = references_.find(name);
    if (it == references_.end())
        return false;
    if (it->second > 1)
    {
        --it->second;
        return false;
    }
    references_.erase(it);
    return true;
}

std::size_t ResourceReferenceCounter::References(const std::string &name) const
{
    const auto it = references_.find(name);
    return it == references_.end() ? 0u : it->second;
}

std::mutex &diagnosticLogMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::string lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string providerKey(std::string value)
{
    value = lower(std::move(value));
    for (char &c : value)
        if (c == ' ' || c == '-')
            c = '_';
    return value;
}

std::string expandHome(std::string path)
{
    if (!path.empty() && path.front() == '~')
    {
        if (const char *home = std::getenv("HOME"))
            path.replace(0, 1, home);
    }
    return path;
}

std::string xmlEscape(std::string value)
{
    replaceAll(value, "&", "&amp;");
    replaceAll(value, "<", "&lt;");
    replaceAll(value, ">", "&gt;");
    replaceAll(value, "\"", "&quot;");
    replaceAll(value, "'", "&apos;");
    return value;
}

std::string tileText(const TileKey &key)
{
    return std::to_string(key.z) + "/" + std::to_string(key.x) + "/" +
           std::to_string(key.y);
}

std::string redactUrl(std::string url)
{
    const std::vector<std::string> keys{"access_token=", "token=", "api_key=", "key="};
    for (const auto &key : keys)
    {
        std::size_t pos = 0;
        while ((pos = url.find(key, pos)) != std::string::npos)
        {
            const std::size_t start = pos + key.size();
            std::size_t end = url.find('&', start);
            if (end == std::string::npos)
                end = url.size();
            url.replace(start, end - start, "<redacted>");
            pos = start + 10;
        }
    }
    return url;
}

TileKey latLonToTile(double latDeg, double lonDeg, int zoom)
{
    latDeg = clampValue(latDeg, -kMercatorLatLimit, kMercatorLatLimit);
    lonDeg = clampValue(lonDeg, -180.0, 180.0);
    const int count = 1 << zoom;
    const double latRad = latDeg * kPi / 180.0;
    const int x = clampValue(static_cast<int>(std::floor((lonDeg + 180.0) / 360.0 * count)), 0, count - 1);
    const int y = clampValue(static_cast<int>(std::floor((1.0 - std::asinh(std::tan(latRad)) / kPi) * 0.5 * count)), 0, count - 1);
    return {x, y, zoom};
}

TileCoordF latLonToTileFraction(double latDeg, double lonDeg, int zoom)
{
    latDeg = clampValue(latDeg, -kMercatorLatLimit, kMercatorLatLimit);
    lonDeg = clampValue(lonDeg, -180.0, 180.0);
    const double n = static_cast<double>(1u << zoom);
    const double latRad = latDeg * kPi / 180.0;
    return {(lonDeg + 180.0) / 360.0 * n,
            (1.0 - std::asinh(std::tan(latRad)) / kPi) * 0.5 * n};
}

double tileXToLon(double x, int zoom)
{
    return x / static_cast<double>(1u << zoom) * 360.0 - 180.0;
}

double tileYToLat(double y, int zoom)
{
    const double n = kPi - 2.0 * kPi * y / static_cast<double>(1u << zoom);
    return std::atan(std::sinh(n)) * 180.0 / kPi;
}

TileBounds tileRectBounds(const TileRect &rect)
{
    return {tileYToLat(rect.minY, rect.zoom),
            tileYToLat(rect.maxY + 1.0, rect.zoom),
            tileXToLon(rect.minX, rect.zoom),
            tileXToLon(rect.maxX + 1.0, rect.zoom)};
}

TileRect boundsToTileRect(const TileBounds &bounds, int zoom, int halo)
{
    const int count = 1 << zoom;
    const TileCoordF nw = latLonToTileFraction(bounds.north, bounds.west, zoom);
    const TileCoordF se = latLonToTileFraction(bounds.south, bounds.east, zoom);
    int minX = static_cast<int>(std::floor(std::min(nw.x, se.x))) - halo;
    int maxX = static_cast<int>(std::floor(std::max(nw.x, se.x) - 1e-10)) + halo;
    int minY = static_cast<int>(std::floor(std::min(nw.y, se.y))) - halo;
    int maxY = static_cast<int>(std::floor(std::max(nw.y, se.y) - 1e-10)) + halo;
    minX = clampValue(minX, 0, count - 1);
    maxX = clampValue(maxX, 0, count - 1);
    minY = clampValue(minY, 0, count - 1);
    maxY = clampValue(maxY, 0, count - 1);
    return {minX, minY, maxX, maxY, zoom};
}

Provider resolveImageryProvider(const Config &cfg)
{
    Provider provider = builtinImageryProvider(cfg.imageryProvider);
    if (!cfg.imageryUrl.empty())
    {
        provider.name = providerKey(cfg.imageryProvider);
        if (provider.name.empty())
            provider.name = "custom";
        provider.url = cfg.imageryUrl;
        provider.quadKey = provider.url.find("{q}") != std::string::npos;
    }
    if (!cfg.imageryExtension.empty())
        provider.extension = cfg.imageryExtension;
    return provider;
}

std::string buildUrl(const Provider &provider, const TileKey &key,
                     const std::string &token)
{
    std::string url = provider.url;
    const int server = (key.x + 2 * key.y) % 4;
    replaceAll(url, "{s}", std::to_string(server));
    replaceAll(url, "{s4}", std::to_string(server + 1));
    replaceAll(url, "{x}", std::to_string(key.x));
    replaceAll(url, "{y}", std::to_string(key.y));
    replaceAll(url, "{z}", std::to_string(key.z));
    replaceAll(url, "{q}", quadKey(key.x, key.y, key.z));
    replaceAll(url, "{token}", token);
    return url;
}

int zoomForAltitude(const Config &cfg, double altitude)
{
    if (!cfg.dynamicZoom)
        return clampValue(cfg.staticZoom, cfg.minZoom, cfg.maxZoom);
    altitude = std::max(0.0, altitude);
    for (const auto &[maxAltitude, zoom] : cfg.zoomTable)
        if (altitude <= maxAltitude)
            return clampValue(zoom, cfg.minZoom, cfg.maxZoom);
    return cfg.minZoom;
}

int validHeightmapSize(int requested)
{
    constexpr int sizes[] = {129, 257, 513, 1025, 2049, 4097};
    int best = sizes[0];
    int distance = std::abs(requested - best);
    for (const int candidate : sizes)
    {
        const int d = std::abs(requested - candidate);
        if (d < distance)
        {
            best = candidate;
            distance = d;
        }
    }
    return best;
}

gz::math::Vector3d localFromGeodetic(
    const gz::math::SphericalCoordinates &spherical,
    double latDeg, double lonDeg, double elevationM)
{
    const gz::math::Vector3d radians{
        latDeg * kPi / 180.0,
        lonDeg * kPi / 180.0,
        elevationM};
    return spherical.PositionTransform(
        radians,
        gz::math::SphericalCoordinates::SPHERICAL,
        gz::math::SphericalCoordinates::LOCAL2);
}

gz::math::Vector3d geodeticFromLocal(
    const gz::math::SphericalCoordinates &spherical,
    const gz::math::Vector3d &local)
{
    auto result = spherical.PositionTransform(
        local,
        gz::math::SphericalCoordinates::LOCAL2,
        gz::math::SphericalCoordinates::SPHERICAL);
    result.X(result.X() * 180.0 / kPi);
    result.Y(result.Y() * 180.0 / kPi);
    return result;
}

std::size_t mipmappedRgbaBytes(int width, int height)
{
    if (width <= 0 || height <= 0)
        return 0;
    std::size_t pixels = 0;
    while (true)
    {
        pixels += static_cast<std::size_t>(width) *
                  static_cast<std::size_t>(height);
        if (width == 1 && height == 1)
            break;
        width = std::max(1, width / 2);
        height = std::max(1, height / 2);
    }
    return pixels * 4u;
}

void normalizeConfig(Config &cfg)
{
    cfg.imageryProvider = providerKey(cfg.imageryProvider);
    cfg.elevationProvider = providerKey(cfg.elevationProvider);

    cfg.minZoom = clampValue(cfg.minZoom, 1, 20);
    cfg.maxZoom = clampValue(cfg.maxZoom, cfg.minZoom, 20);
    cfg.staticZoom = clampValue(cfg.staticZoom, cfg.minZoom, cfg.maxZoom);
    cfg.elevationMaxZoom = clampValue(cfg.elevationMaxZoom, 1, 20);

    cfg.visualRadiusM = clampValue(cfg.visualRadiusM, 1000.0, 30000.0);
    cfg.visualGeometryZoom = clampValue(cfg.visualGeometryZoom, 8, 18);
    cfg.visualElevationZoom = clampValue(cfg.visualElevationZoom, 8, cfg.elevationMaxZoom);
    cfg.visualMeshCellsPerTile = clampValue(cfg.visualMeshCellsPerTile, 8, 128);
    cfg.visualMaxMeshCells = clampValue(cfg.visualMaxMeshCells, 128, 1536);
    cfg.visualTextureSize = clampValue(cfg.visualTextureSize, 1024, 16384);
    cfg.visualPageTextureMaxSize = clampValue(cfg.visualPageTextureMaxSize, 256, 4096);
    cfg.visualPageCacheMb = std::min<std::size_t>(cfg.visualPageCacheMb, 2048u);
    cfg.visualAtlasPagePixels = clampValue(cfg.visualAtlasPagePixels, 256, 2048);
    cfg.visualAtlasMaxSize = clampValue(cfg.visualAtlasMaxSize, 4096, 16384);
    if (cfg.visualPageTextureMaxSize < 512) cfg.visualPageTextureMaxSize = 256;
    else if (cfg.visualPageTextureMaxSize < 1024) cfg.visualPageTextureMaxSize = 512;
    else if (cfg.visualPageTextureMaxSize < 2048) cfg.visualPageTextureMaxSize = 1024;
    else if (cfg.visualPageTextureMaxSize < 4096) cfg.visualPageTextureMaxSize = 2048;
    else cfg.visualPageTextureMaxSize = 4096;
    cfg.visualTextureGuardM = clampValue(cfg.visualTextureGuardM, 0.0, 5000.0);
    cfg.visualDetailMode = lower(cfg.visualDetailMode);
    if (cfg.visualDetailMode != "bottom_camera_only" &&
        cfg.visualDetailMode != "all")
        cfg.visualDetailMode = "all";
    cfg.visualDetailRadiusM = clampValue(cfg.visualDetailRadiusM, 100.0,
                                         cfg.visualRadiusM);
    cfg.visualDetailZoom = clampValue(cfg.visualDetailZoom,
                                      cfg.visualGeometryZoom, 20);
    cfg.visualBootstrapImageryZoom = clampValue(cfg.visualBootstrapImageryZoom, 8, cfg.visualGeometryZoom);
    cfg.visualRecenterDistanceM = clampValue(cfg.visualRecenterDistanceM, 100.0,
                                             std::max(100.0, cfg.visualRadiusM * 0.5));
    cfg.visualWarmupFrames = clampValue(cfg.visualWarmupFrames, 1, 10);
    cfg.visualOffscreenFrames = clampValue(cfg.visualOffscreenFrames, 1, 600);
    cfg.visualRecenterReadyZoom = clampValue(
        cfg.visualRecenterReadyZoom, cfg.visualGeometryZoom, 20);
    cfg.visualRefineMaxSourceTilesPerBatch = clampValue(
        cfg.visualRefineMaxSourceTilesPerBatch, 1, 256);
    for (auto &entry : cfg.visualImageryLodTable)
        entry.second = clampValue(entry.second, 1, 20);
    std::sort(cfg.visualImageryLodTable.begin(), cfg.visualImageryLodTable.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });

    cfg.downloadConcurrency = clampValue(cfg.downloadConcurrency, 1, 16);
    cfg.downloadPerHost = clampValue(cfg.downloadPerHost, 1,
                                     cfg.downloadConcurrency);
    cfg.downloadRetries = clampValue(cfg.downloadRetries, 0, 5);
    cfg.httpTimeoutMs = clampValue(cfg.httpTimeoutMs, 1000u, 30000u);
    cfg.decodedDemCacheMb = clampValue<std::size_t>(cfg.decodedDemCacheMb, 32u, 2048u);

    cfg.radiusTiles = clampValue(cfg.radiusTiles, 0, 8);
    cfg.meshCells = clampValue(cfg.meshCells, 4, 128);
    cfg.heightmapSize = validHeightmapSize(cfg.heightmapSize);
    cfg.collisionOverlapM = clampValue(cfg.collisionOverlapM, 0.0, 10.0);
    cfg.collisionRecenterFraction = clampValue(cfg.collisionRecenterFraction, 0.10, 0.49);
    cfg.startupSafetySizeM = clampValue(cfg.startupSafetySizeM, 10.0, 10000.0);
    cfg.startupSafetyThicknessM = clampValue(cfg.startupSafetyThicknessM, 0.02, 10.0);
    cfg.startupSafetyRemoveDelaySec = clampValue(cfg.startupSafetyRemoveDelaySec, 0.0, 10.0);
    cfg.updatePeriodSec = std::max(0.02, cfg.updatePeriodSec);
    cfg.retryDelaySec = std::max(0.2, cfg.retryDelaySec);
    cfg.statusPeriodSec = std::max(0.5, cfg.statusPeriodSec);
}

Config parseTerrainConfig(const std::shared_ptr<const sdf::Element> &sdf)
{
    Config cfg;
    readSdf(sdf, "imagery_provider", cfg.imageryProvider);
    readSdf(sdf, "imagery_url", cfg.imageryUrl);
    readSdf(sdf, "imagery_extension", cfg.imageryExtension);
    readSdf(sdf, "imagery_token", cfg.imageryToken);
    readSdf(sdf, "elevation_provider", cfg.elevationProvider);
    readSdf(sdf, "elevation_url", cfg.elevationUrl);
    readSdf(sdf, "elevation_token", cfg.elevationToken);
    readSdf(sdf, "elevation_max_zoom", cfg.elevationMaxZoom);

    readSdf(sdf, "visual_radius_m", cfg.visualRadiusM);
    readSdf(sdf, "visual_gui", cfg.visualGui);
    readSdf(sdf, "visual_geometry_zoom", cfg.visualGeometryZoom);
    readSdf(sdf, "visual_elevation_zoom", cfg.visualElevationZoom);
    readSdf(sdf, "visual_mesh_cells_per_tile", cfg.visualMeshCellsPerTile);
    readSdf(sdf, "visual_max_mesh_cells", cfg.visualMaxMeshCells);
    readSdf(sdf, "visual_texture_size", cfg.visualTextureSize);
    readSdf(sdf, "visual_page_texture_max_size", cfg.visualPageTextureMaxSize);
    int visualPageCacheMb = static_cast<int>(cfg.visualPageCacheMb);
    readSdf(sdf, "visual_page_cache_mb", visualPageCacheMb);
    cfg.visualPageCacheMb = static_cast<std::size_t>(std::max(0, visualPageCacheMb));
    readSdf(sdf, "visual_atlas_page_pixels", cfg.visualAtlasPagePixels);
    readSdf(sdf, "visual_atlas_max_size", cfg.visualAtlasMaxSize);
    readSdf(sdf, "visual_texture_guard_m", cfg.visualTextureGuardM);
    readSdf(sdf, "visual_detail_mode", cfg.visualDetailMode);
    readSdf(sdf, "visual_detail_camera_name", cfg.visualDetailCameraName);
    readSdf(sdf, "visual_detail_radius_m", cfg.visualDetailRadiusM);
    readSdf(sdf, "visual_detail_zoom", cfg.visualDetailZoom);
    readSdf(sdf, "visual_bootstrap_imagery_zoom", cfg.visualBootstrapImageryZoom);
    readSdf(sdf, "visual_recenter_distance_m", cfg.visualRecenterDistanceM);
    readSdf(sdf, "visual_warmup_frames", cfg.visualWarmupFrames);
    readSdf(sdf, "visual_frustum_eviction", cfg.visualFrustumEviction);
    readSdf(sdf, "visual_offscreen_frames", cfg.visualOffscreenFrames);
    readSdf(sdf, "visual_refine_texture", cfg.visualRefineTexture);
    readSdf(sdf, "visual_recenter_ready_zoom", cfg.visualRecenterReadyZoom);
    readSdf(sdf, "visual_refine_max_source_tiles_per_batch",
            cfg.visualRefineMaxSourceTilesPerBatch);
    readSdf(sdf, "visual_lighting_enabled", cfg.visualLightingEnabled);
    readSdf(sdf, "visual_cast_shadows", cfg.visualCastShadows);
    readSdf(sdf, "visual_receive_shadows", cfg.visualReceiveShadows);
    std::string visualLod;
    readSdf(sdf, "visual_imagery_lod_table", visualLod);
    if (!visualLod.empty())
    {
        auto parsed = parseZoomTable(visualLod);
        if (!parsed.empty())
            cfg.visualImageryLodTable = std::move(parsed);
    }

    readSdf(sdf, "download_concurrency", cfg.downloadConcurrency);
    readSdf(sdf, "download_per_host", cfg.downloadPerHost);
    readSdf(sdf, "download_retries", cfg.downloadRetries);
    readSdf(sdf, "http_timeout_ms", cfg.httpTimeoutMs);
    readSdf(sdf, "user_agent", cfg.userAgent);
    readSdf(sdf, "cache_dir", cfg.cacheDir);
    int decodedDemCacheMb = static_cast<int>(cfg.decodedDemCacheMb);
    readSdf(sdf, "decoded_dem_cache_mb", decodedDemCacheMb);
    cfg.decodedDemCacheMb = static_cast<std::size_t>(std::max(1, decodedDemCacheMb));

    readSdf(sdf, "dynamic_zoom", cfg.dynamicZoom);
    readSdf(sdf, "static_zoom", cfg.staticZoom);
    readSdf(sdf, "min_zoom", cfg.minZoom);
    readSdf(sdf, "max_zoom", cfg.maxZoom);
    std::string zoomTable;
    readSdf(sdf, "zoom_table", zoomTable);
    if (!zoomTable.empty())
    {
        auto parsed = parseZoomTable(zoomTable);
        if (!parsed.empty())
            cfg.zoomTable = std::move(parsed);
    }
    readSdf(sdf, "radius_tiles", cfg.radiusTiles);
    readSdf(sdf, "mesh_cells", cfg.meshCells);
    readSdf(sdf, "heightmap_size", cfg.heightmapSize);
    readSdf(sdf, "enable_collision", cfg.enableCollision);
    readSdf(sdf, "collision_overlap_m", cfg.collisionOverlapM);
    readSdf(sdf, "collision_recenter_fraction", cfg.collisionRecenterFraction);

    readSdf(sdf, "align_origin_to_ground", cfg.alignOriginToGround);
    readSdf(sdf, "z_offset_m", cfg.zOffsetM);
    readSdf(sdf, "startup_preload", cfg.startupPreload);
    readSdf(sdf, "startup_safety_ground", cfg.startupSafetyGround);
    readSdf(sdf, "startup_safety_size_m", cfg.startupSafetySizeM);
    readSdf(sdf, "startup_safety_thickness_m", cfg.startupSafetyThicknessM);
    readSdf(sdf, "startup_safety_top_z", cfg.startupSafetyTopZ);
    readSdf(sdf, "startup_safety_remove_delay_sec", cfg.startupSafetyRemoveDelaySec);
    readSdf(sdf, "update_period_sec", cfg.updatePeriodSec);
    readSdf(sdf, "retry_delay_sec", cfg.retryDelaySec);
    readSdf(sdf, "diagnostics", cfg.diagnostics);
    readSdf(sdf, "status_period_sec", cfg.statusPeriodSec);
    readSdf(sdf, "coverage_mode", cfg.coverageMode);
    std::string cameraNames;
    readSdf(sdf, "camera_names", cameraNames);
    cfg.cameraNames = splitCommaList(cameraNames);

    normalizeConfig(cfg);
    return cfg;
}

void registerModelConfig(gz::sim::Entity entity, const Config &cfg)
{
    std::lock_guard<std::mutex> lock(modelConfigRegistryMutex());
    modelConfigRegistry()[entity] = cfg;
}

void unregisterModelConfig(gz::sim::Entity entity)
{
    std::lock_guard<std::mutex> lock(modelConfigRegistryMutex());
    modelConfigRegistry().erase(entity);
}

std::vector<RegisteredModelConfig> registeredModelConfigs()
{
    std::vector<RegisteredModelConfig> result;
    std::lock_guard<std::mutex> lock(modelConfigRegistryMutex());
    result.reserve(modelConfigRegistry().size());
    for (const auto &[entity, cfg] : modelConfigRegistry())
        result.push_back({entity, cfg});
    std::sort(result.begin(), result.end(),
              [](const auto &a, const auto &b) { return a.entity < b.entity; });
    return result;
}

}
