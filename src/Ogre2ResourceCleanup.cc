#include "Ogre2ResourceCleanup.hh"

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>

#include <OgreRenderSystem.h>
#include <OgreRoot.h>
#include <OgreMeshManager.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreMeshManager2.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>

#include <unordered_set>

namespace dynamic_terrain
{
namespace
{
Ogre::TextureGpuManager *textureManager(
    const gz::rendering::ScenePtr &scene)
{
    if (!scene || !scene->Engine() || scene->Engine()->Name() != "ogre2")
        return nullptr;
    auto *engine = dynamic_cast<gz::rendering::Ogre2RenderEngine *>(
        scene->Engine());
    if (!engine || !engine->OgreRoot() ||
        !engine->OgreRoot()->getRenderSystem())
        return nullptr;
    return engine->OgreRoot()->getRenderSystem()->getTextureGpuManager();
}

bool ogre2Scene(const gz::rendering::ScenePtr &scene)
{
    return scene && scene->Engine() && scene->Engine()->Name() == "ogre2" &&
           dynamic_cast<gz::rendering::Ogre2RenderEngine *>(scene->Engine());
}

std::string generatedOgreMeshName(const std::string &commonMeshName)
{
    return commonMeshName + "::::ORIGINAL";
}
}

bool destroyUnreferencedOgre2Texture(
    const gz::rendering::ScenePtr &scene,
    const std::string &textureName) noexcept
{
    try
    {
        auto *manager = textureManager(scene);
        if (!manager || textureName.empty())
            return false;
        auto *texture = manager->findTextureNoThrow(textureName);
        if (!texture)
            return false;
        manager->destroyTexture(texture);
        return manager->findTextureNoThrow(textureName) == nullptr;
    }
    catch (...)
    {
        return false;
    }
}

bool ogre2TextureExists(
    const gz::rendering::ScenePtr &scene,
    const std::string &textureName) noexcept
{
    try
    {
        auto *manager = textureManager(scene);
        return manager && !textureName.empty() &&
               manager->findTextureNoThrow(textureName) != nullptr;
    }
    catch (...)
    {
        return false;
    }
}

std::size_t ogre2TextureBytes(
    const gz::rendering::ScenePtr &scene,
    const std::string &textureName) noexcept
{
    try
    {
        auto *manager = textureManager(scene);
        if (!manager || textureName.empty())
            return 0;
        const auto *texture = manager->findTextureNoThrow(textureName);
        return texture ? texture->getSizeBytes() : 0u;
    }
    catch (...)
    {
        return 0;
    }
}

bool destroyUnreferencedOgre2Mesh(
    const gz::rendering::ScenePtr &scene,
    const std::string &commonMeshName) noexcept
{
    try
    {
        if (!ogre2Scene(scene) || commonMeshName.empty())
            return false;
        const std::string name = generatedOgreMeshName(commonMeshName);
        std::unordered_set<std::string> importedMaterials;
        if (auto *v1 = Ogre::v1::MeshManager::getSingletonPtr())
        {
            const auto mesh = v1->getByName(name);
            if (mesh)
                for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
                    importedMaterials.insert(mesh->getSubMesh(i)->getMaterialName());
        }
        for (const auto &materialName : importedMaterials)
            if (auto material = scene->Material(materialName))
            {
                material->ClearTexture();
                scene->DestroyMaterial(material);
            }
        if (auto *v2 = Ogre::MeshManager::getSingletonPtr())
            if (v2->resourceExists(name))
                v2->remove(name);
        if (auto *v1 = Ogre::v1::MeshManager::getSingletonPtr())
            if (v1->resourceExists(name))
                v1->remove(name);
        return !ogre2MeshExists(scene, commonMeshName);
    }
    catch (...)
    {
        return false;
    }
}

bool ogre2MeshExists(
    const gz::rendering::ScenePtr &scene,
    const std::string &commonMeshName) noexcept
{
    try
    {
        if (!ogre2Scene(scene) || commonMeshName.empty())
            return false;
        const std::string name = generatedOgreMeshName(commonMeshName);
        auto *v2 = Ogre::MeshManager::getSingletonPtr();
        auto *v1 = Ogre::v1::MeshManager::getSingletonPtr();
        return (v2 && v2->resourceExists(name)) ||
               (v1 && v1->resourceExists(name));
    }
    catch (...)
    {
        return false;
    }
}

}
