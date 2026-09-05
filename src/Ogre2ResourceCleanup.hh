#pragma once

#include <gz/rendering/Scene.hh>

#include <cstddef>
#include <string>

namespace dynamic_terrain
{

bool destroyUnreferencedOgre2Texture(
    const gz::rendering::ScenePtr &scene,
    const std::string &textureName) noexcept;
bool ogre2TextureExists(
    const gz::rendering::ScenePtr &scene,
    const std::string &textureName) noexcept;
std::size_t ogre2TextureBytes(
    const gz::rendering::ScenePtr &scene,
    const std::string &textureName) noexcept;
bool destroyUnreferencedOgre2Mesh(
    const gz::rendering::ScenePtr &scene,
    const std::string &commonMeshName) noexcept;
bool ogre2MeshExists(
    const gz::rendering::ScenePtr &scene,
    const std::string &commonMeshName) noexcept;

}
