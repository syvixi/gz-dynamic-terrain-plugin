#pragma once
#include "PersistentTerrain.hh"
#include "GuiTerrain.pb.h"
#include <gz/transport/Node.hh>

namespace dynamic_terrain
{
class GuiTerrainSource
{
public:
    explicit GuiTerrainSource(const std::string &service);
    void SetSnapshot(const std::shared_ptr<TerrainSnapshot> &snapshot);
    void ApplyTexture(const TextureUpdate &update);
private:
    struct State;
    std::shared_ptr<State> state_;
    gz::transport::Node node_;
};

preview::PageInfo guiPageInfo(const TerrainPage &page);
preview::Page encodeGuiPage(const TerrainPage &page);
TerrainPage decodeGuiPage(const preview::Page &packet);
constexpr int kGuiTextureSize = 512;
}
