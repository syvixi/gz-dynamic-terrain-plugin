#pragma once

#include "TerrainTypes.hh"
#include "TileStore.hh"

#include <filesystem>
#include <memory>
#include <string>

namespace dynamic_terrain
{

struct CollisionPatch
{
    TileKey center;
    int radius{0};
    fs::path heightmap;
    double centerX{0.0};
    double centerY{0.0};
    double baseZ{0.0};
    double sizeX{1.0};
    double sizeY{1.0};
    double sizeZ{0.1};
    double yaw{0.0};
};

class CollisionTerrainBuilder
{
public:
    explicit CollisionTerrainBuilder(std::shared_ptr<TileStore> store);
    std::optional<CollisionPatch> Build(const TileKey &center,
                                        std::string &error);
    std::string Sdf(const CollisionPatch &patch, std::uint64_t serial) const;

private:
    std::shared_ptr<TileStore> store_;
};

}
