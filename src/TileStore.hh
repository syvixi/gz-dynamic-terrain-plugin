#pragma once

#include "TerrainTypes.hh"

#include <opencv2/core.hpp>

#include <array>
#include <filesystem>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dynamic_terrain
{

struct DownloadSpec
{
    std::string label;
    std::string url;
    fs::path path;
};


struct ElevationMosaic
{
    int zoom{0};
    int tileSize{256};
    TileRect rect;
    cv::Mat heights;

    bool Valid() const { return !heights.empty() && heights.type() == CV_32FC1; }
    double Sample(double latDeg, double lonDeg) const;
};

struct DownloadStats
{
    std::size_t requested{0};
    std::size_t cached{0};
    std::size_t downloaded{0};
    std::size_t failed{0};
    double elapsedSec{0.0};
};

class BatchDownloader
{
public:
    explicit BatchDownloader(Config config);
    DownloadStats Ensure(const std::vector<DownloadSpec> &requests,
                         std::vector<fs::path> *failedPaths = nullptr) const;

private:
    Config cfg_;
};

class TileStore
{
public:
    TileStore(Config config, gz::math::SphericalCoordinates spherical);

    const Config &GetConfig() const { return cfg_; }
    const Provider &ImageryProvider() const { return imagery_; }
    const gz::math::SphericalCoordinates &Spherical() const { return spherical_; }
    const fs::path &CacheRoot() const { return cacheRoot_; }
    const std::string &ResourcePrefix() const { return resourcePrefix_; }

    double OriginLatDeg() const { return originLatDeg_; }
    double OriginLonDeg() const { return originLonDeg_; }

    fs::path ImageryPath(const TileKey &key) const;
    fs::path ElevationPath(const TileKey &key) const;
    std::string ImageryUrl(const TileKey &key) const;
    std::string ElevationUrl(const TileKey &key) const;

    bool EnsureImagery(const std::vector<TileKey> &keys,
                       std::vector<TileKey> *failed = nullptr);
    bool EnsureElevation(const std::vector<TileKey> &keys,
                         std::vector<TileKey> *failed = nullptr);
    bool EnsureElevationForBounds(const TileBounds &bounds, int zoom,
                                  int halo = 1);
    std::optional<ElevationMosaic> BuildElevationMosaic(
        const TileBounds &bounds, int zoom, int halo, std::string &error);
    double ElevationAlignmentOffset(int zoom);

    cv::Mat LoadImagery(const TileKey &key) const;
    std::shared_ptr<cv::Mat> LoadElevationRaster(const TileKey &key);

    double RawElevation(double latDeg, double lonDeg, int zoom);
    double SampleElevation(double latDeg, double lonDeg, int zoom);
    gz::math::Vector3d LocalPoint(double latDeg, double lonDeg, int zoom);

private:
    double DecodeHeight(const cv::Vec3b &bgr) const;
    std::optional<double> GlobalPixelHeight(std::int64_t globalX,
                                             std::int64_t globalY,
                                             int zoom,
                                             int tileWidth,
                                             int tileHeight);
    double SampleInsideRaster(const cv::Mat &raster, double u, double v) const;
    double OriginAlignmentOffset(int zoom);
    void TouchDemLru(const TileKey &key, std::size_t bytes);
    void EvictDemLruIfNeeded();

    Config cfg_;
    gz::math::SphericalCoordinates spherical_;
    fs::path cacheRoot_;
    Provider imagery_;
    BatchDownloader downloader_;
    std::string resourcePrefix_;
    double originLatDeg_{0.0};
    double originLonDeg_{0.0};

    mutable std::mutex demMutex_;
    std::unordered_map<TileKey, std::shared_ptr<cv::Mat>, TileKeyHash> demMemory_;
    std::unordered_map<TileKey, std::list<TileKey>::iterator, TileKeyHash> demLruPos_;
    std::unordered_map<TileKey, std::size_t, TileKeyHash> demBytes_;
    std::list<TileKey> demLru_;
    std::size_t demMemoryBytes_{0};
    std::array<std::mutex, 64> demFileMutexes_;

    std::mutex originMutex_;
    std::unordered_map<int, double> originOffsets_;
};

std::vector<TileKey> keysForRect(const TileRect &rect);

}
