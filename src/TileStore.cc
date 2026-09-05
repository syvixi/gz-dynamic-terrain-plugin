#include "TileStore.hh"

#include <curl/curl.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cmath>
#include <fstream>
#include <set>
#include <thread>
#include <unordered_set>

namespace dynamic_terrain
{
namespace
{
size_t curlWrite(void *data, size_t size, size_t count, void *user)
{
    auto *bytes = static_cast<std::vector<unsigned char> *>(user);
    const std::size_t total = size * count;
    const auto *begin = static_cast<unsigned char *>(data);
    bytes->insert(bytes->end(), begin, begin + total);
    return total;
}

bool fileReady(const fs::path &path)
{
    std::error_code ec;
    return fs::exists(path, ec) && fs::is_regular_file(path, ec) &&
           fs::file_size(path, ec) > 0;
}

bool writeBinaryAtomic(const fs::path &path,
                       const std::vector<unsigned char> &bytes)
{
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    const fs::path temp = path.string() + ".part." +
        std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) +
        "." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        if (!out)
            return false;
        out.write(reinterpret_cast<const char *>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
        if (!out.good())
            return false;
    }
    fs::rename(temp, path, ec);
    if (!ec)
        return true;
    if (fileReady(path))
    {
        fs::remove(temp, ec);
        return true;
    }
    fs::remove(temp, ec);
    return false;
}

struct Transfer
{
    DownloadSpec spec;
    CURL *easy{nullptr};
    std::vector<unsigned char> bytes;
    char errorBuffer[CURL_ERROR_SIZE]{};
    int attempt{0};
};

void configureEasy(Transfer &transfer, const Config &cfg)
{
    curl_easy_reset(transfer.easy);
    transfer.bytes.clear();
    transfer.errorBuffer[0] = '\0';
    curl_easy_setopt(transfer.easy, CURLOPT_URL, transfer.spec.url.c_str());
    curl_easy_setopt(transfer.easy, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(transfer.easy, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(transfer.easy, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(transfer.easy, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(transfer.easy, CURLOPT_TIMEOUT_MS,
                     static_cast<long>(cfg.httpTimeoutMs));
    curl_easy_setopt(transfer.easy, CURLOPT_CONNECTTIMEOUT_MS,
                     static_cast<long>(std::min(cfg.httpTimeoutMs, 3000u)));
    curl_easy_setopt(transfer.easy, CURLOPT_USERAGENT, cfg.userAgent.c_str());
    curl_easy_setopt(transfer.easy, CURLOPT_WRITEFUNCTION, curlWrite);
    curl_easy_setopt(transfer.easy, CURLOPT_WRITEDATA, &transfer.bytes);
    curl_easy_setopt(transfer.easy, CURLOPT_ERRORBUFFER, transfer.errorBuffer);
#if LIBCURL_VERSION_NUM >= 0x072F00
    curl_easy_setopt(transfer.easy, CURLOPT_PIPEWAIT, 1L);
#endif
}

std::string safeName(std::string value)
{
    for (char &c : value)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-'))
            c = '_';
    return value;
}

std::int64_t positiveModulo(std::int64_t value, std::int64_t modulus)
{
    const std::int64_t result = value % modulus;
    return result < 0 ? result + modulus : result;
}

struct DownloadCoordinator
{
    std::mutex mutex;
    std::condition_variable cv;
    std::unordered_set<std::string> inFlight;
};

DownloadCoordinator &downloadCoordinator()
{
    static DownloadCoordinator coordinator;
    return coordinator;
}

std::atomic<std::uint64_t> &terrainInstanceCounter()
{
    static std::atomic<std::uint64_t> counter{0};
    return counter;
}
}


double ElevationMosaic::Sample(double latDeg, double lonDeg) const
{
    if (!Valid() || tileSize <= 0)
        return 0.0;

    const TileCoordF tile = latLonToTileFraction(latDeg, lonDeg, zoom);
    const double globalPx = tile.x * tileSize - 0.5;
    const double globalPy = tile.y * tileSize - 0.5;
    const double localPx = globalPx - static_cast<double>(rect.minX * tileSize);
    const double localPy = globalPy - static_cast<double>(rect.minY * tileSize);

    const double px = clampValue(localPx, 0.0,
                                 static_cast<double>(heights.cols - 1));
    const double py = clampValue(localPy, 0.0,
                                 static_cast<double>(heights.rows - 1));
    const int x0 = clampValue(static_cast<int>(std::floor(px)), 0,
                              heights.cols - 1);
    const int y0 = clampValue(static_cast<int>(std::floor(py)), 0,
                              heights.rows - 1);
    const int x1 = std::min(x0 + 1, heights.cols - 1);
    const int y1 = std::min(y0 + 1, heights.rows - 1);
    const double tx = px - x0;
    const double ty = py - y0;
    const double h00 = heights.at<float>(y0, x0);
    const double h10 = heights.at<float>(y0, x1);
    const double h01 = heights.at<float>(y1, x0);
    const double h11 = heights.at<float>(y1, x1);
    const double top = h00 * (1.0 - tx) + h10 * tx;
    const double bottom = h01 * (1.0 - tx) + h11 * tx;
    return top * (1.0 - ty) + bottom * ty;
}

BatchDownloader::BatchDownloader(Config config) : cfg_(std::move(config)) {}

DownloadStats BatchDownloader::Ensure(const std::vector<DownloadSpec> &requests,
                                      std::vector<fs::path> *failedPaths) const
{
    DownloadStats stats;
    const auto started = std::chrono::steady_clock::now();

    std::unordered_set<std::string> seen;
    std::vector<DownloadSpec> candidates;
    candidates.reserve(requests.size());
    for (const auto &request : requests)
    {
        ++stats.requested;
        const std::string key = request.path.string();
        if (!seen.insert(key).second)
            continue;
        if (fileReady(request.path))
        {
            ++stats.cached;
            continue;
        }
        candidates.push_back(request);
    }

    if (candidates.empty())
    {
        stats.elapsedSec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        return stats;
    }

    auto &coordinator = downloadCoordinator();
    std::vector<DownloadSpec> owned;
    std::vector<DownloadSpec> waiting;
    {
        std::lock_guard<std::mutex> lock(coordinator.mutex);
        for (const auto &request : candidates)
        {
            const std::string key = request.path.string();
            if (coordinator.inFlight.insert(key).second)
                owned.push_back(request);
            else
                waiting.push_back(request);
        }
    }

    auto releaseOwned = [&]
    {
        std::lock_guard<std::mutex> lock(coordinator.mutex);
        for (const auto &request : owned)
            coordinator.inFlight.erase(request.path.string());
        coordinator.cv.notify_all();
    };

    std::vector<DownloadSpec> round = owned;
    for (int attempt = 0; attempt <= cfg_.downloadRetries && !round.empty(); ++attempt)
    {
        CURLM *multi = curl_multi_init();
        if (!multi)
        {
            if (attempt < cfg_.downloadRetries)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(150 * (attempt + 1)));
                continue;
            }
            stats.failed += round.size();
            if (failedPaths)
                for (const auto &request : round)
                    failedPaths->push_back(request.path);
            break;
        }
#if LIBCURL_VERSION_NUM >= 0x071E00
        curl_multi_setopt(multi, CURLMOPT_MAX_TOTAL_CONNECTIONS,
                          static_cast<long>(cfg_.downloadConcurrency));
        curl_multi_setopt(multi, CURLMOPT_MAX_HOST_CONNECTIONS,
                          static_cast<long>(cfg_.downloadPerHost));
#endif

        std::unordered_map<CURL *, std::unique_ptr<Transfer>> active;
        std::vector<DownloadSpec> retry;
        std::size_t nextIndex = 0;
        bool multiHealthy = true;

        auto startMore = [&]()
        {
            while (multiHealthy &&
                   active.size() < static_cast<std::size_t>(cfg_.downloadConcurrency) &&
                   nextIndex < round.size())
            {
                const auto spec = round[nextIndex++];
                auto transfer = std::make_unique<Transfer>();
                transfer->spec = spec;
                transfer->attempt = attempt;
                transfer->easy = curl_easy_init();
                if (!transfer->easy)
                {
                    retry.push_back(spec);
                    continue;
                }
                configureEasy(*transfer, cfg_);
                CURL *handle = transfer->easy;
                const CURLMcode addCode = curl_multi_add_handle(multi, handle);
                if (addCode != CURLM_OK)
                {
                    curl_easy_cleanup(handle);
                    transfer->easy = nullptr;
                    retry.push_back(spec);
                    continue;
                }
                active.emplace(handle, std::move(transfer));
            }
        };

        startMore();
        if (cfg_.diagnostics)
            logInfo("[DynamicTerrain][HTTP] batch start files=", round.size(),
                    " active_window=", cfg_.downloadConcurrency,
                    " per_host=", cfg_.downloadPerHost,
                    " attempt=", attempt + 1);

        int stillRunning = 0;
        CURLMcode multiCode = curl_multi_perform(multi, &stillRunning);
        if (multiCode != CURLM_OK)
            multiHealthy = false;

        while (multiHealthy && (!active.empty() || nextIndex < round.size()))
        {
            if (stillRunning > 0)
            {
                int numfds = 0;
#if LIBCURL_VERSION_NUM >= 0x074200
                multiCode = curl_multi_poll(multi, nullptr, 0, 500, &numfds);
#else
                multiCode = curl_multi_wait(multi, nullptr, 0, 500, &numfds);
#endif
                if (multiCode != CURLM_OK)
                {
                    multiHealthy = false;
                    break;
                }
            }

            multiCode = curl_multi_perform(multi, &stillRunning);
            if (multiCode != CURLM_OK)
            {
                multiHealthy = false;
                break;
            }

            int messages = 0;
            while (CURLMsg *message = curl_multi_info_read(multi, &messages))
            {
                if (message->msg != CURLMSG_DONE)
                    continue;
                auto it = active.find(message->easy_handle);
                if (it == active.end())
                    continue;

                Transfer &transfer = *it->second;
                long status = 0;
                curl_easy_getinfo(message->easy_handle,
                                  CURLINFO_RESPONSE_CODE, &status);
                const bool ok = message->data.result == CURLE_OK &&
                                status >= 200 && status < 400 &&
                                !transfer.bytes.empty() &&
                                writeBinaryAtomic(transfer.spec.path,
                                                  transfer.bytes);
                if (ok)
                {
                    ++stats.downloaded;
                    if (cfg_.diagnostics)
                        logInfo("[DynamicTerrain][HTTP] ready ",
                                transfer.spec.label,
                                " bytes=", transfer.bytes.size());
                }
                else
                {
                    retry.push_back(transfer.spec);
                    if (cfg_.diagnostics)
                    {
                        const char *curlText =
                            curl_easy_strerror(message->data.result);
                        logInfo("[DynamicTerrain][HTTP] retry candidate ",
                                transfer.spec.label,
                                " status=", status,
                                " curl=",
                                static_cast<int>(message->data.result),
                                " error=", (transfer.errorBuffer[0] ?
                                    transfer.errorBuffer : curlText));
                    }
                }

                curl_multi_remove_handle(multi, message->easy_handle);
                curl_easy_cleanup(message->easy_handle);
                transfer.easy = nullptr;
                active.erase(it);
            }

            startMore();
            if (!active.empty())
            {
                multiCode = curl_multi_perform(multi, &stillRunning);
                if (multiCode != CURLM_OK)
                    multiHealthy = false;
            }
        }

        if (!multiHealthy)
        {
            for (auto &entry : active)
                retry.push_back(entry.second->spec);
            while (nextIndex < round.size())
                retry.push_back(round[nextIndex++]);
        }

        for (auto &entry : active)
        {
            CURL *handle = entry.first;
            curl_multi_remove_handle(multi, handle);
            curl_easy_cleanup(handle);
            entry.second->easy = nullptr;
        }
        active.clear();
        curl_multi_cleanup(multi);

        std::sort(retry.begin(), retry.end(), [](const auto &a, const auto &b)
        {
            return a.path.string() < b.path.string();
        });
        retry.erase(std::unique(retry.begin(), retry.end(), [](const auto &a, const auto &b)
        {
            return a.path == b.path;
        }), retry.end());

        if (attempt == cfg_.downloadRetries && !retry.empty())
        {
            for (const auto &request : retry)
            {
                if (fileReady(request.path))
                    continue;
                ++stats.failed;
                if (failedPaths)
                    failedPaths->push_back(request.path);
                logError("[DynamicTerrain][HTTP] exhausted retries ",
                         request.label, " url=", redactUrl(request.url));
            }
            retry.clear();
        }
        else if (!retry.empty())
        {
            const int backoffMs = std::min(2000, 250 * (1 << attempt));
            std::this_thread::sleep_for(std::chrono::milliseconds(backoffMs));
        }
        round = std::move(retry);
    }

    releaseOwned();

    if (!waiting.empty())
    {
        std::unique_lock<std::mutex> lock(coordinator.mutex);
        coordinator.cv.wait(lock, [&]
        {
            for (const auto &request : waiting)
                if (coordinator.inFlight.count(request.path.string()) != 0)
                    return false;
            return true;
        });
        lock.unlock();

        for (const auto &request : waiting)
        {
            if (fileReady(request.path))
                ++stats.cached;
            else
            {
                ++stats.failed;
                if (failedPaths)
                    failedPaths->push_back(request.path);
            }
        }
    }

    stats.elapsedSec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][HTTP] batch done requested=", stats.requested,
                " cached=", stats.cached,
                " downloaded=", stats.downloaded,
                " failed=", stats.failed,
                " time=", stats.elapsedSec, "s");
    return stats;
}

TileStore::TileStore(Config config, gz::math::SphericalCoordinates spherical)
    : cfg_(std::move(config)),
      spherical_(std::move(spherical)),
      cacheRoot_(expandHome(cfg_.cacheDir)),
      imagery_(resolveImageryProvider(cfg_)),
      downloader_(cfg_)
{
    originLatDeg_ = spherical_.LatitudeReference().Degree();
    originLonDeg_ = spherical_.LongitudeReference().Degree();
    const auto instance = terrainInstanceCounter().fetch_add(
        1, std::memory_order_relaxed) + 1;
    resourcePrefix_ = "dynamic_terrain_" +
        safeName(cfg_.modelName.empty() ? "model" : cfg_.modelName) + "_i" +
        std::to_string(instance);
}

fs::path TileStore::ImageryPath(const TileKey &key) const
{
    return cacheRoot_ / "imagery" / safeName(imagery_.name) /
           std::to_string(key.z) / std::to_string(key.x) /
           (std::to_string(key.y) + "." + imagery_.extension);
}

fs::path TileStore::ElevationPath(const TileKey &key) const
{
    const std::string provider = lower(cfg_.elevationProvider);
    const std::string ext = provider == "terrarium" ? "png" : "webp";
    return cacheRoot_ / "elevation" / safeName(provider) /
           std::to_string(key.z) / std::to_string(key.x) /
           (std::to_string(key.y) + "." + ext);
}

std::string TileStore::ImageryUrl(const TileKey &key) const
{
    return buildUrl(imagery_, key, cfg_.imageryToken);
}

std::string TileStore::ElevationUrl(const TileKey &key) const
{
    const std::string provider = lower(cfg_.elevationProvider);
    std::string templ = cfg_.elevationUrl;
    if (templ.empty())
    {
        if (provider == "terrarium")
            templ = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
        else
            templ = "https://api.mapbox.com/raster/v1/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}.webp?sku=101CUGorpzzyK&access_token={token}";
    }
    Provider elevation{provider, templ, "", false};
    return buildUrl(elevation, key, cfg_.elevationToken);
}

bool TileStore::EnsureImagery(const std::vector<TileKey> &keys,
                              std::vector<TileKey> *failed)
{
    std::vector<DownloadSpec> requests;
    requests.reserve(keys.size());
    for (const auto &key : keys)
        requests.push_back({"IMG " + tileText(key), ImageryUrl(key), ImageryPath(key)});
    const auto stats = downloader_.Ensure(requests);
    if (failed)
    {
        for (const auto &key : keys)
            if (!fileReady(ImageryPath(key)))
                failed->push_back(key);
    }
    return stats.failed == 0;
}

bool TileStore::EnsureElevation(const std::vector<TileKey> &keys,
                                std::vector<TileKey> *failed)
{
    const std::string provider = lower(cfg_.elevationProvider);
    if (provider == "none" || provider == "flat")
        return true;
    std::vector<DownloadSpec> requests;
    requests.reserve(keys.size());
    for (const auto &key : keys)
        requests.push_back({"DEM " + tileText(key), ElevationUrl(key), ElevationPath(key)});
    const auto stats = downloader_.Ensure(requests);
    if (failed)
    {
        for (const auto &key : keys)
            if (!fileReady(ElevationPath(key)))
                failed->push_back(key);
    }
    return stats.failed == 0;
}

bool TileStore::EnsureElevationForBounds(const TileBounds &bounds, int zoom,
                                         int halo)
{
    return EnsureElevation(keysForRect(boundsToTileRect(bounds, zoom, halo)));
}

std::optional<ElevationMosaic> TileStore::BuildElevationMosaic(
    const TileBounds &bounds, int zoom, int halo, std::string &error)
{
    zoom = clampValue(zoom, 1, cfg_.elevationMaxZoom);
    const TileRect rect = boundsToTileRect(bounds, zoom, halo);
    const std::string provider = lower(cfg_.elevationProvider);

    if (provider == "none" || provider == "flat")
    {
        ElevationMosaic mosaic;
        mosaic.zoom = zoom;
        mosaic.tileSize = 256;
        mosaic.rect = rect;
        mosaic.heights = cv::Mat(rect.Height() * mosaic.tileSize,
                                 rect.Width() * mosaic.tileSize,
                                 CV_32FC1,
                                 cv::Scalar(static_cast<float>(spherical_.ElevationReference())));
        return mosaic;
    }

    const auto keys = keysForRect(rect);
    std::vector<TileKey> failed;
    EnsureElevation(keys, &failed);
    if (!failed.empty())
    {
        error = std::to_string(failed.size()) +
                " elevation tiles unavailable for mosaic at z" +
                std::to_string(zoom);
        return std::nullopt;
    }

    int tileWidth = 0;
    int tileHeight = 0;
    std::vector<std::pair<TileKey, std::shared_ptr<cv::Mat>>> rasters;
    rasters.reserve(keys.size());
    for (const auto &key : keys)
    {
        auto raster = LoadElevationRaster(key);
        if (!raster || raster->empty())
        {
            error = "failed to decode elevation tile " + tileText(key);
            return std::nullopt;
        }
        if (tileWidth == 0)
        {
            tileWidth = raster->cols;
            tileHeight = raster->rows;
        }
        if (raster->cols != tileWidth || raster->rows != tileHeight)
        {
            error = "elevation tile dimensions are inconsistent";
            return std::nullopt;
        }
        rasters.emplace_back(key, std::move(raster));
    }
    if (tileWidth <= 0 || tileHeight <= 0 || tileWidth != tileHeight)
    {
        error = "invalid elevation tile dimensions";
        return std::nullopt;
    }

    ElevationMosaic mosaic;
    mosaic.zoom = zoom;
    mosaic.tileSize = tileWidth;
    mosaic.rect = rect;
    mosaic.heights = cv::Mat(rect.Height() * tileHeight,
                             rect.Width() * tileWidth,
                             CV_32FC1);

    for (const auto &[key, raster] : rasters)
    {
        const int ox = (key.x - rect.minX) * tileWidth;
        const int oy = (key.y - rect.minY) * tileHeight;
        cv::Mat dst = mosaic.heights(cv::Rect(ox, oy, tileWidth, tileHeight));
        for (int row = 0; row < tileHeight; ++row)
        {
            const auto *src = raster->ptr<cv::Vec3b>(row);
            float *out = dst.ptr<float>(row);
            for (int col = 0; col < tileWidth; ++col)
                out[col] = static_cast<float>(DecodeHeight(src[col]));
        }
    }

    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][DEM] mosaic ready z=", zoom,
                " tiles=", rect.Width(), "x", rect.Height(),
                " raster=", mosaic.heights.cols, "x", mosaic.heights.rows);
    return mosaic;
}

double TileStore::ElevationAlignmentOffset(int zoom)
{
    return OriginAlignmentOffset(zoom);
}

cv::Mat TileStore::LoadImagery(const TileKey &key) const
{
    const auto path = ImageryPath(key);
    if (!fileReady(path))
        return {};
    return cv::imread(path.string(), cv::IMREAD_COLOR);
}

void TileStore::TouchDemLru(const TileKey &key, std::size_t bytes)
{
    auto pos = demLruPos_.find(key);
    if (pos != demLruPos_.end())
        demLru_.erase(pos->second);
    else
    {
        demMemoryBytes_ += bytes;
        demBytes_[key] = bytes;
    }
    demLru_.push_front(key);
    demLruPos_[key] = demLru_.begin();
}

void TileStore::EvictDemLruIfNeeded()
{
    const std::size_t limit = cfg_.decodedDemCacheMb * 1024ull * 1024ull;
    while (demMemoryBytes_ > limit && demLru_.size() > 1)
    {
        const TileKey victim = demLru_.back();
        demLru_.pop_back();
        demLruPos_.erase(victim);
        auto bytesIt = demBytes_.find(victim);
        if (bytesIt != demBytes_.end())
        {
            demMemoryBytes_ -= std::min(demMemoryBytes_, bytesIt->second);
            demBytes_.erase(bytesIt);
        }
        demMemory_.erase(victim);
    }
}

std::shared_ptr<cv::Mat> TileStore::LoadElevationRaster(const TileKey &key)
{
    {
        std::lock_guard<std::mutex> lock(demMutex_);
        auto it = demMemory_.find(key);
        if (it != demMemory_.end())
        {
            const std::size_t bytes = it->second->total() * it->second->elemSize();
            TouchDemLru(key, bytes);
            return it->second;
        }
    }

    auto &stripe = demFileMutexes_[TileKeyHash{}(key) % demFileMutexes_.size()];
    std::lock_guard<std::mutex> stripeLock(stripe);
    {
        std::lock_guard<std::mutex> lock(demMutex_);
        auto it = demMemory_.find(key);
        if (it != demMemory_.end())
            return it->second;
    }

    if (!fileReady(ElevationPath(key)))
    {
        if (!EnsureElevation({key}))
            return {};
    }

    cv::Mat decoded = cv::imread(ElevationPath(key).string(), cv::IMREAD_COLOR);
    if (decoded.empty())
    {
        logError("[DynamicTerrain][DEM] decode failed ", tileText(key),
                 " path=", ElevationPath(key).string());
        return {};
    }
    auto raster = std::make_shared<cv::Mat>(std::move(decoded));
    const std::size_t bytes = raster->total() * raster->elemSize();
    {
        std::lock_guard<std::mutex> lock(demMutex_);
        demMemory_[key] = raster;
        TouchDemLru(key, bytes);
        EvictDemLruIfNeeded();
    }
    return raster;
}

double TileStore::DecodeHeight(const cv::Vec3b &bgr) const
{
    const double b = bgr[0];
    const double g = bgr[1];
    const double r = bgr[2];
    if (lower(cfg_.elevationProvider) == "terrarium")
        return r * 256.0 + g + b / 256.0 - 32768.0;
    return ((r * 256.0 * 256.0 + g * 256.0 + b) * 0.1) - 10000.0;
}

double TileStore::SampleInsideRaster(const cv::Mat &raster, double u, double v) const
{
    u = clampValue(u, 0.0, 1.0);
    v = clampValue(v, 0.0, 1.0);
    const double px = u * static_cast<double>(raster.cols - 1);
    const double py = v * static_cast<double>(raster.rows - 1);
    const int x0 = clampValue(static_cast<int>(std::floor(px)), 0, raster.cols - 1);
    const int y0 = clampValue(static_cast<int>(std::floor(py)), 0, raster.rows - 1);
    const int x1 = std::min(x0 + 1, raster.cols - 1);
    const int y1 = std::min(y0 + 1, raster.rows - 1);
    const double tx = px - x0;
    const double ty = py - y0;
    auto h = [&](int x, int y) { return DecodeHeight(raster.at<cv::Vec3b>(y, x)); };
    const double top = h(x0, y0) * (1.0 - tx) + h(x1, y0) * tx;
    const double bottom = h(x0, y1) * (1.0 - tx) + h(x1, y1) * tx;
    return top * (1.0 - ty) + bottom * ty;
}

std::optional<double> TileStore::GlobalPixelHeight(std::int64_t globalX,
                                                    std::int64_t globalY,
                                                    int zoom,
                                                    int tileWidth,
                                                    int tileHeight)
{
    const std::int64_t tileCount = std::int64_t{1} << zoom;
    const std::int64_t globalWidth = tileCount * tileWidth;
    const std::int64_t globalHeight = tileCount * tileHeight;
    globalX = positiveModulo(globalX, globalWidth);
    globalY = clampValue<std::int64_t>(globalY, 0, globalHeight - 1);
    const int tileX = static_cast<int>(globalX / tileWidth);
    const int tileY = static_cast<int>(globalY / tileHeight);
    const int pixelX = static_cast<int>(globalX % tileWidth);
    const int pixelY = static_cast<int>(globalY % tileHeight);
    auto raster = LoadElevationRaster({tileX, tileY, zoom});
    if (!raster || raster->empty())
        return std::nullopt;
    const int sx = raster->cols == tileWidth ? pixelX : clampValue(
        static_cast<int>(std::llround(static_cast<double>(pixelX) *
                                     (raster->cols - 1) /
                                     std::max(1, tileWidth - 1))),
        0, raster->cols - 1);
    const int sy = raster->rows == tileHeight ? pixelY : clampValue(
        static_cast<int>(std::llround(static_cast<double>(pixelY) *
                                     (raster->rows - 1) /
                                     std::max(1, tileHeight - 1))),
        0, raster->rows - 1);
    return DecodeHeight(raster->at<cv::Vec3b>(sy, sx));
}

double TileStore::RawElevation(double latDeg, double lonDeg, int zoom)
{
    const std::string provider = lower(cfg_.elevationProvider);
    if (provider == "flat" || provider == "none")
        return spherical_.ElevationReference();

    zoom = clampValue(zoom, 1, cfg_.elevationMaxZoom);
    const TileKey key = latLonToTile(latDeg, lonDeg, zoom);
    auto referenceRaster = LoadElevationRaster(key);
    if (!referenceRaster || referenceRaster->empty())
        return spherical_.ElevationReference();

    const double n = static_cast<double>(1u << zoom);
    const double latRad = clampValue(latDeg, -kMercatorLatLimit, kMercatorLatLimit) * kPi / 180.0;
    const double x = (lonDeg + 180.0) / 360.0 * n;
    const double y = (1.0 - std::asinh(std::tan(latRad)) / kPi) * 0.5 * n;
    const int tileWidth = referenceRaster->cols;
    const int tileHeight = referenceRaster->rows;
    const double globalPx = x * tileWidth - 0.5;
    const double globalPy = y * tileHeight - 0.5;
    const std::int64_t gx0 = static_cast<std::int64_t>(std::floor(globalPx));
    const std::int64_t gy0 = static_cast<std::int64_t>(std::floor(globalPy));
    const std::int64_t gx1 = gx0 + 1;
    const std::int64_t gy1 = gy0 + 1;
    const double tx = globalPx - gx0;
    const double ty = globalPy - gy0;
    const auto h00 = GlobalPixelHeight(gx0, gy0, zoom, tileWidth, tileHeight);
    const auto h10 = GlobalPixelHeight(gx1, gy0, zoom, tileWidth, tileHeight);
    const auto h01 = GlobalPixelHeight(gx0, gy1, zoom, tileWidth, tileHeight);
    const auto h11 = GlobalPixelHeight(gx1, gy1, zoom, tileWidth, tileHeight);
    if (!h00 || !h10 || !h01 || !h11)
        return SampleInsideRaster(*referenceRaster, x - std::floor(x), y - std::floor(y));
    const double top = *h00 * (1.0 - tx) + *h10 * tx;
    const double bottom = *h01 * (1.0 - tx) + *h11 * tx;
    return top * (1.0 - ty) + bottom * ty;
}

double TileStore::OriginAlignmentOffset(int zoom)
{
    if (!cfg_.alignOriginToGround)
        return 0.0;
    {
        std::lock_guard<std::mutex> lock(originMutex_);
        auto it = originOffsets_.find(zoom);
        if (it != originOffsets_.end())
            return it->second;
    }
    const double rawOrigin = RawElevation(originLatDeg_, originLonDeg_, zoom);
    const double offset = spherical_.ElevationReference() - rawOrigin;
    {
        std::lock_guard<std::mutex> lock(originMutex_);
        originOffsets_[zoom] = offset;
    }
    if (cfg_.diagnostics)
        logInfo("[DynamicTerrain][DEM] origin alignment zoom=", zoom,
                " raw=", rawOrigin,
                " reference=", spherical_.ElevationReference(),
                " offset=", offset);
    return offset;
}

double TileStore::SampleElevation(double latDeg, double lonDeg, int zoom)
{
    return RawElevation(latDeg, lonDeg, zoom) + OriginAlignmentOffset(zoom) + cfg_.zOffsetM;
}

gz::math::Vector3d TileStore::LocalPoint(double latDeg, double lonDeg, int zoom)
{
    const double elevation = SampleElevation(latDeg, lonDeg, zoom);
    return localFromGeodetic(spherical_, latDeg, lonDeg, elevation);
}

std::vector<TileKey> keysForRect(const TileRect &rect)
{
    std::vector<TileKey> keys;
    keys.reserve(static_cast<std::size_t>(std::max(0, rect.Width())) *
                 static_cast<std::size_t>(std::max(0, rect.Height())));
    for (int y = rect.minY; y <= rect.maxY; ++y)
        for (int x = rect.minX; x <= rect.maxX; ++x)
            keys.push_back({x, y, rect.zoom});
    return keys;
}

}
