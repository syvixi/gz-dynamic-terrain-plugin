#include "TerrainTypes.hh"

#include <gz/math/Angle.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
class TestFailure : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

void check(bool condition, const char *expression, const char *file, int line)
{
    if (condition)
        return;
    std::ostringstream out;
    out << file << ':' << line << ": CHECK failed: " << expression;
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
#define CHECK_NEAR(actual, expected, tolerance) \
    checkNear((actual), (expected), (tolerance), \
              #actual " ~= " #expected, __FILE__, __LINE__)

gz::math::Angle degrees(double value)
{
    gz::math::Angle result;
    result.SetDegree(value);
    return result;
}

void testLocal2RoundTrip()
{
    struct Reference
    {
        double latitudeDeg;
        double longitudeDeg;
        double elevationM;
    };

    const std::array<Reference, 2> references{{
        {52.2297, 21.0122, 123.4},
        {78.2232, 15.6469, 37.0},
    }};
    const std::array<double, 4> headingsDeg{{0.0, 37.0, 90.0, -73.0}};
    const std::array<gz::math::Vector3d, 7> localPoints{{
        {0.0, 0.0, 0.0},
        {1000.0, 0.0, 50.0},
        {0.0, 10000.0, -20.0},
        {60000.0, 80000.0, 250.0},
        {100000.0, 0.0, 1000.0},
        {-60000.0, 80000.0, -100.0},
        {-100000.0, 0.0, 5000.0},
    }};

    for (const auto &reference : references)
    {
        for (const double headingDeg : headingsDeg)
        {
            const gz::math::SphericalCoordinates spherical(
                gz::math::SphericalCoordinates::EARTH_WGS84,
                degrees(reference.latitudeDeg),
                degrees(reference.longitudeDeg),
                reference.elevationM, degrees(headingDeg));

            for (const auto &local : localPoints)
            {
                const auto expectedRadians = spherical.PositionTransform(
                    local,
                    gz::math::SphericalCoordinates::LOCAL2,
                    gz::math::SphericalCoordinates::SPHERICAL);
                const auto geodetic = dynamic_terrain::geodeticFromLocal(
                    spherical, local);

                CHECK_NEAR(geodetic.X(),
                           expectedRadians.X() * 180.0 / dynamic_terrain::kPi,
                           1e-10);
                CHECK_NEAR(geodetic.Y(),
                           expectedRadians.Y() * 180.0 / dynamic_terrain::kPi,
                           1e-10);
                CHECK_NEAR(geodetic.Z(), expectedRadians.Z(), 1e-6);

                const auto recovered = dynamic_terrain::localFromGeodetic(
                    spherical, geodetic.X(), geodetic.Y(), geodetic.Z());
                CHECK(recovered.Distance(local) < 1e-3);

                const gz::math::Vector3d geodeticRadians{
                    geodetic.X() * dynamic_terrain::kPi / 180.0,
                    geodetic.Y() * dynamic_terrain::kPi / 180.0,
                    geodetic.Z()};
                const auto expectedLocal = spherical.PositionTransform(
                    geodeticRadians,
                    gz::math::SphericalCoordinates::SPHERICAL,
                    gz::math::SphericalCoordinates::LOCAL2);
                CHECK(recovered.Distance(expectedLocal) < 1e-7);
            }
        }
    }
}

std::string textureName(int x, int y)
{
    return "dynamic_terrain_test_page_z14_x" + std::to_string(x) +
           "_y" + std::to_string(y) + "_q17_s2048";
}

std::vector<std::string> textureWindow(int centerX)
{
    std::vector<std::string> result;
    result.reserve(9);
    for (int y = 40; y <= 42; ++y)
        for (int x = centerX - 1; x <= centerX + 1; ++x)
            result.push_back(textureName(x, y));
    return result;
}

void checkReferenceModel(
    const dynamic_terrain::ResourceReferenceCounter &counter,
    const std::vector<std::string> &active,
    const std::vector<std::string> &staging)
{
    std::unordered_map<std::string, std::size_t> expected;
    for (const auto &name : active)
        ++expected[name];
    for (const auto &name : staging)
        ++expected[name];

    CHECK(counter.ResourceCount() == expected.size());
    for (const auto &[name, references] : expected)
        CHECK(counter.References(name) == references);
}

void testReferenceCounterAcrossRecenters()
{
    dynamic_terrain::ResourceReferenceCounter counter;
    CHECK(counter.Acquire("") == 0u);
    CHECK(!counter.Release(""));
    CHECK(!counter.Release("never-acquired"));
    CHECK(counter.ResourceCount() == 0u);

    std::vector<std::string> active;
    std::unordered_set<std::string> allNames;
    std::unordered_map<std::string, std::size_t> deletionCount;
    std::size_t acquiredReferences = 0;
    std::size_t releasedReferences = 0;

    for (int cycle = 0; cycle < 100; ++cycle)
    {
        auto staging = textureWindow(cycle);
        for (const auto &name : staging)
        {
            counter.Acquire(name);
            allNames.insert(name);
            ++acquiredReferences;
        }

        checkReferenceModel(counter, active, staging);
        CHECK(counter.ResourceCount() == (active.empty() ? 9u : 12u));

        for (const auto &name : active)
        {
            const std::size_t before = counter.References(name);
            CHECK(before > 0u);
            const bool finalReference = counter.Release(name);
            ++releasedReferences;
            CHECK(finalReference == (before == 1u));
            if (finalReference)
                ++deletionCount[name];
        }

        active = std::move(staging);
        checkReferenceModel(counter, active, {});
        CHECK(counter.ResourceCount() == 9u);
        for (const auto &name : active)
            CHECK(counter.References(name) == 1u);
    }

    for (const auto &name : active)
    {
        CHECK(counter.Release(name));
        ++releasedReferences;
        ++deletionCount[name];
    }
    active.clear();

    CHECK(acquiredReferences == 900u);
    CHECK(releasedReferences == acquiredReferences);
    CHECK(allNames.size() == 306u);
    CHECK(deletionCount.size() == allNames.size());
    for (const auto &name : allNames)
        CHECK(deletionCount[name] == 1u);
    CHECK(counter.ResourceCount() == 0u);
    CHECK(!counter.Release(textureName(99, 41)));
}

void testCpuPageCacheConfig()
{
    dynamic_terrain::Config config;
    CHECK(config.visualPageCacheMb == 128u);
    config.visualPageCacheMb = 0u;
    dynamic_terrain::normalizeConfig(config);
    CHECK(config.visualPageCacheMb == 0u);
    config.visualPageCacheMb = 4096u;
    dynamic_terrain::normalizeConfig(config);
    CHECK(config.visualPageCacheMb == 2048u);
}

void testMipmappedByteAccounting()
{
    CHECK(dynamic_terrain::mipmappedRgbaBytes(0, 64) == 0u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(64, 0) == 0u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(-1, 64) == 0u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(1, 1) == 4u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(2, 1) == 12u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(3, 5) == 72u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(64, 64) == 21844u);
    CHECK(dynamic_terrain::mipmappedRgbaBytes(256, 256) == 349524u);
}
}

int main()
{
    try
    {
        testLocal2RoundTrip();
        testReferenceCounterAcrossRecenters();
        testMipmappedByteAccounting();
        testCpuPageCacheConfig();
        std::cout << "TerrainTypes tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
