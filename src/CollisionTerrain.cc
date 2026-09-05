#include "CollisionTerrain.hh"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace dynamic_terrain
{
namespace
{
bool encodeHeightmap(const cv::Mat &heights, const fs::path &path,
                     double &baseZ, double &sizeZ, std::string &error)
{
    double minZ = 0.0;
    double maxZ = 0.0;
    cv::minMaxLoc(heights, &minZ, &maxZ);
    const double realRange = maxZ - minZ;
    const double encodedRange = std::max(realRange, 0.10);
    baseZ = minZ;
    sizeZ = encodedRange;
    cv::Mat encoded(heights.rows, heights.cols, CV_16UC1);
    if (realRange <= 1e-9)
        encoded.setTo(cv::Scalar(0));
    else
    {
        for (int row = 0; row < heights.rows; ++row)
        {
            const float *src = heights.ptr<float>(row);
            auto *dst = encoded.ptr<std::uint16_t>(row);
            for (int col = 0; col < heights.cols; ++col)
            {
                const double normalized = clampValue(
                    (static_cast<double>(src[col]) - minZ) / encodedRange,
                    0.0, 1.0);
                dst[col] = static_cast<std::uint16_t>(
                    std::llround(normalized * 65535.0));
            }
        }
    }
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    if (!cv::imwrite(path.string(), encoded))
    {
        error = "cannot write collision heightmap " + path.string();
        return false;
    }
    return true;
}
}

CollisionTerrainBuilder::CollisionTerrainBuilder(std::shared_ptr<TileStore> store)
    : store_(std::move(store))
{
}

std::optional<CollisionPatch> CollisionTerrainBuilder::Build(
    const TileKey &center, std::string &error)
{
    const auto &cfg = store_->GetConfig();
    const int count = 1 << center.z;
    const int radius = cfg.radiusTiles;
    const TileRect rect{
        clampValue(center.x - radius, 0, count - 1),
        clampValue(center.y - radius, 0, count - 1),
        clampValue(center.x + radius, 0, count - 1),
        clampValue(center.y + radius, 0, count - 1), center.z};
    const TileBounds bounds = tileRectBounds(rect);

    auto dem = store_->BuildElevationMosaic(
        bounds, cfg.elevationMaxZoom, 1, error);
    if (!dem)
    {
        if (error.empty())
            error = "collision DEM is incomplete";
        return std::nullopt;
    }
    const double elevationOffset =
        store_->ElevationAlignmentOffset(cfg.elevationMaxZoom) + cfg.zOffsetM;

    const int size = validHeightmapSize(cfg.heightmapSize);
    cv::Mat heights(size, size, CV_32FC1);
    const int cells = size - 1;
    for (int row = 0; row < size; ++row)
    {
        const double v = static_cast<double>(row) / cells;
        const double tileY = rect.minY + (rect.maxY + 1.0 - rect.minY) * v;
        const double lat = tileYToLat(tileY, rect.zoom);
        for (int col = 0; col < size; ++col)
        {
            const double u = static_cast<double>(col) / cells;
            const double tileX = rect.minX + (rect.maxX + 1.0 - rect.minX) * u;
            const double lon = tileXToLon(tileX, rect.zoom);
            const double elevation = dem->Sample(lat, lon) + elevationOffset;
            heights.at<float>(row, col) = static_cast<float>(
                localFromGeodetic(
                    store_->Spherical(), lat, lon, elevation).Z());
        }
    }

    const double centerLat = tileYToLat((rect.minY + rect.maxY + 1.0) * 0.5, rect.zoom);
    const double centerLon = tileXToLon((rect.minX + rect.maxX + 1.0) * 0.5, rect.zoom);
    const double reference = store_->Spherical().ElevationReference();
    const auto west = localFromGeodetic(
        store_->Spherical(), centerLat, bounds.west, reference);
    const auto east = localFromGeodetic(
        store_->Spherical(), centerLat, bounds.east, reference);
    const auto north = localFromGeodetic(
        store_->Spherical(), bounds.north, centerLon, reference);
    const auto south = localFromGeodetic(
        store_->Spherical(), bounds.south, centerLon, reference);
    const auto patchCenter = localFromGeodetic(
        store_->Spherical(), centerLat, centerLon, reference);
    const gz::math::Vector3d eastAxis = east - west;
    const gz::math::Vector3d northAxis = north - south;

    CollisionPatch patch;
    patch.center = center;
    patch.radius = radius;
    patch.centerX = patchCenter.X();
    patch.centerY = patchCenter.Y();
    patch.sizeX = std::max(0.1, std::hypot(eastAxis.X(), eastAxis.Y())) +
                  2.0 * cfg.collisionOverlapM;
    patch.sizeY = std::max(0.1, std::hypot(northAxis.X(), northAxis.Y())) +
                  2.0 * cfg.collisionOverlapM;
    patch.yaw = std::atan2(eastAxis.Y(), eastAxis.X());

    const fs::path root = store_->CacheRoot() / "collision_v18" /
        std::to_string(center.z) /
        (std::to_string(center.x) + "_" + std::to_string(center.y) +
         "_r" + std::to_string(radius) + "_s" + std::to_string(size));
    patch.heightmap = root / "height.png";
    if (!encodeHeightmap(heights, patch.heightmap,
                         patch.baseZ, patch.sizeZ, error))
        return std::nullopt;

    if (cfg.diagnostics)
        logInfo("[DynamicTerrain][COLLISION] prepared center=", tileText(center),
                " size=", patch.sizeX, "x", patch.sizeY,
                " yaw=", patch.yaw,
                " raster=", size, "x", size,
                " z=[", patch.baseZ, ",", patch.baseZ + patch.sizeZ, "]");
    return patch;
}

std::string CollisionTerrainBuilder::Sdf(const CollisionPatch &patch,
                                          std::uint64_t serial) const
{
    std::ostringstream out;
    out << std::setprecision(17)
        << "<sdf version='1.9'>"
        << "<model name='dynamic_terrain_collision_" << serial << "'>"
        << "<static>true</static><link name='ground'>"
        << "<collision name='collision'><pose>"
        << patch.centerX << ' ' << patch.centerY << ' ' << patch.baseZ
        << " 0 0 " << patch.yaw << "</pose><geometry><heightmap><uri>"
        << xmlEscape("file://" + patch.heightmap.string())
        << "</uri><size>" << patch.sizeX << ' ' << patch.sizeY << ' '
        << patch.sizeZ
        << "</size><pos>0 0 0</pos><sampling>1</sampling>"
        << "</heightmap></geometry></collision>"
        << "</link></model></sdf>";
    return out.str();
}

}
