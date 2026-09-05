#include "CollisionTerrain.hh"
#include "PersistentTerrain.hh"
#include "TerrainTypes.hh"
#include "TileStore.hh"

#include <gz/common/SubMesh.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SphericalCoordinates.hh>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
namespace fs = std::filesystem;

class TestFailure : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

void check(bool condition, const char *expression, const char *file, int line,
           const std::string &detail = {})
{
    if (condition)
        return;
    std::ostringstream out;
    out << file << ':' << line << ": CHECK failed: " << expression;
    if (!detail.empty())
        out << " (" << detail << ')';
    throw TestFailure(out.str());
}

void checkNear(double actual, double expected, double tolerance,
               const char *expression, const char *file, int line)
{
    if (std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <= tolerance)
        return;
    std::ostringstream out;
    out << std::setprecision(17) << file << ':' << line
        << ": CHECK_NEAR failed: " << expression << " (actual=" << actual
        << ", expected=" << expected << ", tolerance=" << tolerance << ')';
    throw TestFailure(out.str());
}

#define CHECK(expression) \
    check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)
#define CHECK_DETAIL(expression, detail) \
    check(static_cast<bool>(expression), #expression, __FILE__, __LINE__, detail)
#define CHECK_NEAR(actual, expected, tolerance) \
    checkNear((actual), (expected), (tolerance), \
              #actual " ~= " #expected, __FILE__, __LINE__)

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        const auto nonce = std::chrono::high_resolution_clock::now()
            .time_since_epoch().count();
        path_ = fs::temp_directory_path() /
            ("gz_dynamic_terrain_builder_test_" + std::to_string(nonce));
        std::error_code error;
        const bool created = fs::create_directories(path_, error);
        CHECK_DETAIL(created && !error, error.message());
    }

    ~TemporaryDirectory()
    {
        std::error_code error;
        fs::remove_all(path_, error);
    }

    const fs::path &Path() const
    {
        return path_;
    }

private:
    fs::path path_;
};

gz::math::Angle degrees(double value)
{
    gz::math::Angle result;
    result.SetDegree(value);
    return result;
}

gz::math::SphericalCoordinates makeSphericalCoordinates(
    double headingDeg = 37.0)
{
    return {
        gz::math::SphericalCoordinates::EARTH_WGS84,
        degrees(52.2297), degrees(21.0122), 123.4, degrees(headingDeg)};
}

dynamic_terrain::Config makeConfig(const fs::path &cacheRoot)
{
    dynamic_terrain::Config config;
    config.modelName = "paged_mesh_regression";
    config.cacheDir = cacheRoot.string();
    config.imageryProvider = "synthetic";
    config.imageryUrl =
        "file:///this-test-must-not-download/{z}/{x}/{y}.png";
    config.imageryExtension = "png";
    config.elevationProvider = "flat";
    config.elevationMaxZoom = 14;
    config.visualRadiusM = 1000.0;
    config.visualGeometryZoom = 14;
    config.visualElevationZoom = 14;
    config.visualMeshCellsPerTile = 8;
    config.visualMaxMeshCells = 128;
    config.visualPageTextureMaxSize = 256;
    config.visualBootstrapImageryZoom = 14;
    config.visualImageryLodTable = {{1000000.0, 14}};
    config.visualRefineTexture = false;
    config.alignOriginToGround = false;
    config.downloadRetries = 0;
    config.httpTimeoutMs = 50;
    config.diagnostics = false;
    dynamic_terrain::normalizeConfig(config);
    return config;
}

void seedImagery(const dynamic_terrain::TileStore &store,
                 const std::unordered_set<dynamic_terrain::TileKey,
                                          dynamic_terrain::TileKeyHash> &keys)
{
    for (const auto &key : keys)
    {
        cv::Mat image(256, 256, CV_8UC3);
        for (int row = 0; row < image.rows; ++row)
        {
            auto *pixels = image.ptr<cv::Vec3b>(row);
            for (int col = 0; col < image.cols; ++col)
            {
                pixels[col] = cv::Vec3b(
                    static_cast<unsigned char>(col),
                    static_cast<unsigned char>(row),
                    static_cast<unsigned char>((key.x + key.y) & 0xff));
            }
        }

        const fs::path path = store.ImageryPath(key);
        std::error_code error;
        fs::create_directories(path.parent_path(), error);
        CHECK_DETAIL(!error, error.message());
        CHECK_DETAIL(cv::imwrite(path.string(), image), path.string());
    }
}

std::unordered_set<dynamic_terrain::TileKey, dynamic_terrain::TileKeyHash>
imageryUnion(const dynamic_terrain::TileKey &firstCenter,
             const dynamic_terrain::TileKey &secondCenter)
{
    std::unordered_set<dynamic_terrain::TileKey,
                       dynamic_terrain::TileKeyHash> result;
    for (const auto &center : {firstCenter, secondCenter})
    {
        for (int y = center.y - 1; y <= center.y + 1; ++y)
            for (int x = center.x - 1; x <= center.x + 1; ++x)
                result.insert({x, y, center.z});
    }
    return result;
}

std::string geographicToken(const dynamic_terrain::TileKey &key)
{
    return "_z" + std::to_string(key.z) + "_x" +
           std::to_string(key.x) + "_y" + std::to_string(key.y);
}

using PageMap = std::unordered_map<dynamic_terrain::TileKey,
                                   const dynamic_terrain::TerrainPage *,
                                   dynamic_terrain::TileKeyHash>;

PageMap pagesByKey(const dynamic_terrain::TerrainSnapshot &snapshot)
{
    PageMap result;
    for (const auto &page : snapshot.pages)
        CHECK(result.emplace(page.key, &page).second);
    return result;
}

void validateSnapshot(const dynamic_terrain::TerrainSnapshot &snapshot,
                      const dynamic_terrain::TileStore &store,
                      std::uint64_t expectedGeneration)
{
    CHECK(snapshot.generation == expectedGeneration);
    CHECK(snapshot.geometryRect.Width() == 3);
    CHECK(snapshot.geometryRect.Height() == 3);
    CHECK(snapshot.pages.size() == 9u);
    CHECK(snapshot.cellsPerTile == 8);
    CHECK(!snapshot.resourcePrefix.empty());

    std::unordered_set<std::string> meshNames;
    std::unordered_set<std::string> textureNames;
    std::size_t expectedTextureBytes = 0;

    for (const auto &page : snapshot.pages)
    {
        CHECK(page.mesh);
        CHECK(page.texture);
        CHECK(page.texture->Valid());
        CHECK(page.textureSize == 256);
        CHECK(page.mesh->SubMeshCount() == 1u);

        const std::string expectedMeshName = snapshot.resourcePrefix +
            "_mesh_g" + std::to_string(snapshot.generation) +
            geographicToken(page.key);
        CHECK(page.mesh->Name() == expectedMeshName);
        CHECK(page.mesh->Name() != "unknown");
        CHECK(meshNames.insert(page.mesh->Name()).second);
        CHECK(textureNames.insert(page.textureName).second);
        CHECK(page.textureName.rfind(snapshot.resourcePrefix + "_page", 0) == 0u);
        CHECK(page.textureName.find(geographicToken(page.key)) !=
              std::string::npos);
        const auto rgb = page.texture->RGBData();
        const std::size_t pixel = (37u * 256u + 91u) * 3u;
        CHECK(rgb[pixel + 2u] == 91u);
        CHECK(rgb[pixel + 1u] == 37u);
        CHECK(rgb[pixel] ==
              static_cast<unsigned char>((page.key.x + page.key.y) & 0xff));

        auto surface = page.mesh->SubMeshByIndex(0).lock();
        CHECK(surface);
        CHECK(surface->Name() == page.submeshName);
        CHECK(page.submeshName == "terrain_page" + geographicToken(page.key));

        const int cells = snapshot.cellsPerTile;
        const unsigned int vertexSide = static_cast<unsigned int>(cells + 1);
        const unsigned int expectedVertices = vertexSide * vertexSide;
        CHECK(surface->VertexCount() == expectedVertices);
        CHECK(surface->NormalCount() == expectedVertices);
        CHECK(surface->TexCoordCount() == expectedVertices);
        CHECK(surface->IndexCount() ==
              static_cast<unsigned int>(cells * cells * 6));

        for (int py = 0; py <= cells; ++py)
        {
            const double v = static_cast<double>(py) / cells;
            for (int px = 0; px <= cells; ++px)
            {
                const double u = static_cast<double>(px) / cells;
                const unsigned int index = static_cast<unsigned int>(
                    py * (cells + 1) + px);
                const auto uv = surface->TexCoord(index);
                CHECK_NEAR(uv.X(), u, 1e-12);
                CHECK_NEAR(uv.Y(), v, 1e-12);
                const auto geodetic = dynamic_terrain::geodeticFromLocal(
                    store.Spherical(), surface->Vertex(index));
                CHECK_NEAR(geodetic.X(),
                           dynamic_terrain::tileYToLat(page.key.y + v,
                                                       page.key.z),
                           2e-9);
                CHECK_NEAR(geodetic.Y(),
                           dynamic_terrain::tileXToLon(page.key.x + u,
                                                       page.key.z),
                           2e-9);
            }
        }

        for (unsigned int i = 0; i < surface->IndexCount(); ++i)
        {
            CHECK(surface->Index(i) >= 0);
            CHECK(static_cast<unsigned int>(surface->Index(i)) <
                  surface->VertexCount());
        }
        expectedTextureBytes += dynamic_terrain::mipmappedRgbaBytes(
            page.textureSize, page.textureSize);
    }

    CHECK(meshNames.size() == snapshot.pages.size());
    CHECK(textureNames.size() == snapshot.pages.size());
    CHECK(snapshot.estimatedTextureBytes == expectedTextureBytes);
}

void testPagedMeshesAndDirectUvs()
{
    TemporaryDirectory temp;
    const auto config = makeConfig(temp.Path());
    auto store = std::make_shared<dynamic_terrain::TileStore>(
        config, makeSphericalCoordinates());
    dynamic_terrain::PersistentTerrainBuilder builder(store);

    const auto firstCenter = dynamic_terrain::latLonToTile(
        store->OriginLatDeg(), store->OriginLonDeg(),
        config.visualGeometryZoom);
    const dynamic_terrain::TileKey secondCenter{
        firstCenter.x, firstCenter.y + 1, firstCenter.z};
    seedImagery(*store, imageryUnion(firstCenter, secondCenter));

    std::string error;
    auto first = builder.BuildBootstrap(firstCenter, 7u, error);
    CHECK_DETAIL(first, error);
    error.clear();
    auto second = builder.BuildBootstrap(secondCenter, 8u, error);
    CHECK_DETAIL(second, error);

    validateSnapshot(*first, *store, 7u);
    validateSnapshot(*second, *store, 8u);
    CHECK(first->resourcePrefix == second->resourcePrefix);

    const auto firstPages = pagesByKey(*first);
    const auto secondPages = pagesByKey(*second);
    std::size_t overlap = 0;
    for (const auto &[key, firstPage] : firstPages)
    {
        const auto it = secondPages.find(key);
        if (it == secondPages.end())
            continue;
        ++overlap;
        const auto *secondPage = it->second;
        CHECK(firstPage->mesh->Name() != secondPage->mesh->Name());
        CHECK(firstPage->textureName == secondPage->textureName);
        CHECK(firstPage->texture->RGBData() == secondPage->texture->RGBData());

        auto firstSurface = firstPage->mesh->SubMeshByIndex(0).lock();
        auto secondSurface = secondPage->mesh->SubMeshByIndex(0).lock();
        CHECK(firstSurface);
        CHECK(secondSurface);
        CHECK(firstSurface->VertexCount() == secondSurface->VertexCount());
        for (unsigned int i = 0; i < firstSurface->VertexCount(); ++i)
            CHECK(firstSurface->Vertex(i).Distance(secondSurface->Vertex(i)) <
                  1e-7);
    }
    CHECK(overlap == 6u);
    auto secondStore = std::make_shared<dynamic_terrain::TileStore>(
        config, makeSphericalCoordinates());
    dynamic_terrain::PersistentTerrainBuilder secondBuilder(secondStore);
    error.clear();
    auto otherInstance = secondBuilder.BuildBootstrap(firstCenter, 7u, error);
    CHECK_DETAIL(otherInstance, error);
    validateSnapshot(*otherInstance, *secondStore, 7u);
    CHECK(first->resourcePrefix != otherInstance->resourcePrefix);

    const auto otherPages = pagesByKey(*otherInstance);
    for (const auto &[key, firstPage] : firstPages)
    {
        const auto it = otherPages.find(key);
        CHECK(it != otherPages.end());
        CHECK(firstPage->mesh->Name() != it->second->mesh->Name());
        CHECK(firstPage->textureName != it->second->textureName);
    }
}

void testRefinedPageCache(std::size_t budgetMb, int textureSize,
                          std::size_t expectedCapacity)
{
    TemporaryDirectory temp;
    auto config = makeConfig(temp.Path());
    config.visualRefineTexture = true;
    config.visualPageTextureMaxSize = std::max(1024, textureSize);
    config.visualImageryLodTable = {{1000000.0, 15}};
    config.visualPageCacheMb = budgetMb;
    dynamic_terrain::normalizeConfig(config);
    auto store = std::make_shared<dynamic_terrain::TileStore>(
        config, makeSphericalCoordinates());
    auto builder = std::make_unique<dynamic_terrain::PersistentTerrainBuilder>(store);
    const auto center = dynamic_terrain::latLonToTile(
        store->OriginLatDeg(), store->OriginLonDeg(), 14);
    std::vector<std::weak_ptr<gz::common::Image>> images;
    const std::size_t imageBytes = static_cast<std::size_t>(textureSize) *
                                   textureSize * 3u;

    for (int step = 0; step < 40; ++step)
    {
        const dynamic_terrain::TileKey key{center.x + step, center.y, 14};
        seedImagery(*store, {{key.x * 2, key.y * 2, 15},
                             {key.x * 2 + 1, key.y * 2, 15},
                             {key.x * 2, key.y * 2 + 1, 15},
                             {key.x * 2 + 1, key.y * 2 + 1, 15}});
        dynamic_terrain::TerrainSnapshot snapshot;
        snapshot.generation = step + 1;
        snapshot.geometryRect.zoom = 14;
        snapshot.resourcePrefix = "ram_regression";
        dynamic_terrain::TerrainPage page;
        page.key = key;
        page.submeshName = "terrain_page" + geographicToken(key);
        page.textureSize = textureSize;
        snapshot.pages.push_back(std::move(page));
        std::string error;
        auto update = builder->BuildTextureStage(snapshot, {0u}, 15, error);
        CHECK_DETAIL(update && update->pages.size() == 1u, error);
        builder->ApplyTextureUpdate(snapshot, *update);
        std::weak_ptr<gz::common::Image> replaced = update->pages[0].texture;
        update.reset();
        CHECK(!replaced.expired());
        update = builder->BuildTextureStage(snapshot, {0u}, 15, error);
        CHECK_DETAIL(update && update->pages.size() == 1u, error);
        builder->ApplyTextureUpdate(snapshot, *update);
        CHECK(replaced.expired());
        images.push_back(update->pages[0].texture);
        update.reset();
        CHECK(!images.back().expired());
        snapshot.pages.clear();

        const auto stats = builder->CachedPageStats();
        const auto expectedPages = std::min(images.size(), expectedCapacity);
        CHECK(stats.limitBytes == budgetMb * 1024u * 1024u);
        CHECK(stats.pages == expectedPages);
        CHECK(stats.bytes == expectedPages * imageBytes);
        CHECK(stats.bytes <= stats.limitBytes);
        for (std::size_t i = 0; i < images.size(); ++i)
            CHECK(images[i].expired() == (i < images.size() - expectedPages));
    }
    builder.reset();
    for (const auto &image : images)
        CHECK(image.expired());
}

void testRefinedPixelsSurviveRecenter()
{
    TemporaryDirectory temp;
    auto config = makeConfig(temp.Path());
    config.visualRefineTexture = true;
    config.visualPageTextureMaxSize = 512;
    config.visualImageryLodTable = {{1000000.0, 15}};
    config.visualPageCacheMb = 2;
    dynamic_terrain::normalizeConfig(config);
    auto store = std::make_shared<dynamic_terrain::TileStore>(
        config, makeSphericalCoordinates());
    dynamic_terrain::PersistentTerrainBuilder builder(store);
    const auto center = dynamic_terrain::latLonToTile(
        store->OriginLatDeg(), store->OriginLonDeg(), 14);
    const dynamic_terrain::TileKey next{center.x + 1, center.y, 14};
    seedImagery(*store, imageryUnion(center, next));
    seedImagery(*store, {{center.x * 2, center.y * 2, 15},
                         {center.x * 2 + 1, center.y * 2, 15},
                         {center.x * 2, center.y * 2 + 1, 15},
                         {center.x * 2 + 1, center.y * 2 + 1, 15}});
    std::string error;
    auto first = builder.BuildBootstrap(center, 1u, error);
    CHECK_DETAIL(first, error);
    const auto *page = pagesByKey(*first).at(center);
    auto update = builder.BuildTextureStage(*first, {page->index}, 15, error);
    CHECK_DETAIL(update && update->pages.size() == 1u, error);
    builder.ApplyTextureUpdate(*first, *update);
    auto second = builder.BuildBootstrap(next, 2u, error);
    CHECK_DETAIL(second, error);
    const auto *carried = pagesByKey(*second).at(center);
    CHECK(carried->texture == page->texture);
    CHECK(carried->textureName == page->textureName);
    CHECK(carried->imageryZoom == 15);
    CHECK(carried->textureSize == page->textureSize);
    CHECK(carried->texture->RGBData() == page->texture->RGBData());
}

void testCollisionPatchFollowsLocal2Heading()
{
    TemporaryDirectory temp;
    auto config = makeConfig(temp.Path());
    config.radiusTiles = 1;
    config.heightmapSize = 65;
    config.collisionOverlapM = 0.0;
    dynamic_terrain::normalizeConfig(config);
    for (const double headingDeg :
         std::array<double, 4>{{0.0, 37.0, 90.0, -73.0}})
    {
        auto store = std::make_shared<dynamic_terrain::TileStore>(
            config, makeSphericalCoordinates(headingDeg));
        dynamic_terrain::CollisionTerrainBuilder builder(store);

        const auto center = dynamic_terrain::latLonToTile(
            store->OriginLatDeg(), store->OriginLonDeg(), config.staticZoom);
        std::string error;
        const auto patch = builder.Build(center, error);
        CHECK_DETAIL(patch.has_value(), error);
        CHECK(fs::exists(patch->heightmap));

        const int tileCount = 1 << center.z;
        const dynamic_terrain::TileRect rect{
            dynamic_terrain::clampValue(center.x - config.radiusTiles,
                                        0, tileCount - 1),
            dynamic_terrain::clampValue(center.y - config.radiusTiles,
                                        0, tileCount - 1),
            dynamic_terrain::clampValue(center.x + config.radiusTiles,
                                        0, tileCount - 1),
            dynamic_terrain::clampValue(center.y + config.radiusTiles,
                                        0, tileCount - 1), center.z};
        const auto bounds = dynamic_terrain::tileRectBounds(rect);
        const double centerLat = dynamic_terrain::tileYToLat(
            (rect.minY + rect.maxY + 1.0) * 0.5, rect.zoom);
        const double centerLon = dynamic_terrain::tileXToLon(
            (rect.minX + rect.maxX + 1.0) * 0.5, rect.zoom);
        const double reference = store->Spherical().ElevationReference();
        const auto west = dynamic_terrain::localFromGeodetic(
            store->Spherical(), centerLat, bounds.west, reference);
        const auto east = dynamic_terrain::localFromGeodetic(
            store->Spherical(), centerLat, bounds.east, reference);
        const auto north = dynamic_terrain::localFromGeodetic(
            store->Spherical(), bounds.north, centerLon, reference);
        const auto south = dynamic_terrain::localFromGeodetic(
            store->Spherical(), bounds.south, centerLon, reference);
        const auto expectedCenter = dynamic_terrain::localFromGeodetic(
            store->Spherical(), centerLat, centerLon, reference);
        const auto eastAxis = east - west;
        const auto northAxis = north - south;

        CHECK_NEAR(patch->centerX, expectedCenter.X(), 1e-8);
        CHECK_NEAR(patch->centerY, expectedCenter.Y(), 1e-8);
        CHECK_NEAR(patch->sizeX,
                   std::hypot(eastAxis.X(), eastAxis.Y()), 1e-8);
        CHECK_NEAR(patch->sizeY,
                   std::hypot(northAxis.X(), northAxis.Y()), 1e-8);
        CHECK(patch->sizeX > 100.0);
        CHECK(patch->sizeY > 100.0);

        const gz::math::Vector3d localX{
            std::cos(patch->yaw), std::sin(patch->yaw), 0.0};
        const gz::math::Vector3d localY{
            -std::sin(patch->yaw), std::cos(patch->yaw), 0.0};
        CHECK(localX.Dot(eastAxis.Normalized()) > 0.999999);
        CHECK(localY.Dot(northAxis.Normalized()) > 0.999999);

        std::ostringstream yawText;
        yawText << std::setprecision(17) << patch->yaw;
        const std::string collisionSdf = builder.Sdf(*patch, 42u);
        CHECK(collisionSdf.find(" 0 0 " + yawText.str() + "</pose>") !=
              std::string::npos);
    }
}
}

int main()
{
    try
    {
        testPagedMeshesAndDirectUvs();
        testRefinedPageCache(2u, 512, 2u);
        testRefinedPageCache(64u, 256, 24u);
        testRefinedPageCache(0u, 512, 0u);
        testRefinedPageCache(1u, 1024, 0u);
        testRefinedPageCache(128u, 2048, 10u);
        testRefinedPixelsSurviveRecenter();
        testCollisionPatchFollowsLocal2Heading();
        std::cout << "PersistentTerrainBuilder tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
