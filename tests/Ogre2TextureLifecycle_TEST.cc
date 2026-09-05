#include "Ogre2ResourceCleanup.hh"
#include "PersistentTerrain.hh"
#include "TerrainTypes.hh"

#include <gz/common/Image.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>

#include <OgreRenderSystem.h>
#include <OgreRoot.h>
#include <OgreMeshManager.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
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

#define CHECK(expression) \
    check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

void destroyMaterial(const gz::rendering::ScenePtr &scene,
                     gz::rendering::MaterialPtr &material)
{
    if (scene && material)
    {
        material->ClearTexture();
        scene->DestroyMaterial(material);
    }
    material.reset();
}

struct RenderCleanup
{
    ~RenderCleanup()
    {
        try
        {
            if (visual)
                visual->RemoveGeometries();
            if (geometry)
                geometry->Destroy();
            geometry.reset();
            if (scene && visual)
                scene->DestroyVisual(visual, true);
            visual.reset();
            if (scene && !meshName.empty())
                dynamic_terrain::destroyUnreferencedOgre2Mesh(
                    scene, meshName);
        }
        catch (...)
        {
            geometry.reset();
            visual.reset();
        }

        try
        {
            destroyMaterial(scene, materialA);
            destroyMaterial(scene, materialB);
        }
        catch (...)
        {
            materialA.reset();
            materialB.reset();
        }

        if (textureManager && !textureName.empty())
        {
            try
            {
                if (auto *texture =
                        textureManager->findTextureNoThrow(textureName))
                    textureManager->destroyTexture(texture);
            }
            catch (...)
            {
            }
        }

        if (engine && scene)
        {
            try
            {
                engine->DestroyScene(scene);
            }
            catch (...)
            {
            }
        }
        scene.reset();
    }

    gz::rendering::RenderEngine *engine{nullptr};
    gz::rendering::ScenePtr scene;
    gz::rendering::MaterialPtr materialA;
    gz::rendering::MaterialPtr materialB;
    gz::rendering::MeshPtr geometry;
    gz::rendering::VisualPtr visual;
    Ogre::TextureGpuManager *textureManager{nullptr};
    std::string textureName;
    std::string meshName;
};

std::shared_ptr<gz::common::Image> solidImage(std::uint8_t value)
{
    constexpr unsigned int side = 64;
    std::vector<std::uint8_t> rgb(side * side * 3u, value);
    auto image = std::make_shared<gz::common::Image>();
    image->SetFromData(rgb.data(), side, side,
                       gz::common::Image::RGB_INT8);
    CHECK(image->Valid());
    return image;
}

dynamic_terrain::TerrainPage makeTerrainPage(
    const std::string &prefix, const dynamic_terrain::TileKey &key,
    std::size_t index, double minimumX, double maximumX,
    std::uint8_t imageValue)
{
    dynamic_terrain::TerrainPage page;
    page.key = key;
    page.index = index;
    page.submeshName = "terrain_page_z" + std::to_string(key.z) +
        "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y);
    page.textureSize = 64;
    page.texture = solidImage(imageValue);
    page.textureName = prefix + "_page_z" + std::to_string(key.z) +
        "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y) +
        "_q14_s64";
    page.mesh = std::make_shared<gz::common::Mesh>();
    page.mesh->SetName(prefix + "_mesh_g1_z" + std::to_string(key.z) +
        "_x" + std::to_string(key.x) + "_y" + std::to_string(key.y));
    auto surface = std::make_unique<gz::common::SubMesh>(page.submeshName);
    surface->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
    const std::array<gz::math::Vector3d, 4> vertices{{
        {minimumX, -4.0, 0.0}, {maximumX, -4.0, 0.0},
        {minimumX, 4.0, 0.0}, {maximumX, 4.0, 0.0}}};
    const std::array<gz::math::Vector2d, 4> uvs{{
        {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}}};
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        surface->AddVertex(vertices[i]);
        surface->AddNormal(0.0, 0.0, 1.0);
        surface->AddTexCoord(uvs[i]);
    }
    for (const unsigned int indexValue : {0u, 2u, 1u, 1u, 2u, 3u})
        surface->AddIndex(indexValue);
    page.mesh->AddSubMesh(std::move(surface));
    return page;
}

void testRendererRecenters(const gz::rendering::ScenePtr &scene,
                           std::int64_t nonce)
{
    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = false;
    config.diagnostics = false;

    const std::string prefix =
        "dynamic_terrain_recenter_" + std::to_string(nonce);
    const dynamic_terrain::TileKey key{100, 100, 14};
    std::string previousMesh;
    std::string sharedTexture;
    std::string previousImportedMaterial;
    std::weak_ptr<gz::common::Image> previousCpuImage;
    std::weak_ptr<gz::common::Mesh> previousCpuMesh;

    gz::sim::EventManager events;
    {
        dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
        for (std::uint64_t generation = 1u; generation <= 20u; ++generation)
        {
            auto snapshot =
                std::make_shared<dynamic_terrain::TerrainSnapshot>();
            snapshot->generation = generation;
            snapshot->resourcePrefix = prefix;
            snapshot->centerTile = key;
            auto page = makeTerrainPage(prefix, key, 0u, 10.0, 20.0, 60u);
            page.mesh->SetName(prefix + "_mesh_g" +
                               std::to_string(generation) + "_z14_x100_y100");
            sharedTexture = page.textureName;
            const std::string nextMesh = page.mesh->Name();
            snapshot->estimatedTextureBytes =
                dynamic_terrain::mipmappedRgbaBytes(
                    page.textureSize, page.textureSize);
            snapshot->pages.push_back(std::move(page));

            renderer.QueueSnapshot(snapshot);
            events.Emit<gz::sim::events::PreRender>();
            CHECK(dynamic_terrain::ogre2MeshExists(scene, nextMesh));
            const auto imported = Ogre::v1::MeshManager::getSingleton().getByName(
                nextMesh + "::::ORIGINAL");
            CHECK(imported);
            const auto importedMaterial = imported->getSubMesh(0)->getMaterialName();
            CHECK(scene->Material(importedMaterial));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, sharedTexture));
            if (!previousMesh.empty())
                CHECK(dynamic_terrain::ogre2MeshExists(scene, previousMesh));

            events.Emit<gz::sim::events::PreRender>();
            events.Emit<gz::sim::events::PostRender>();
            CHECK(renderer.ActiveGeneration() == generation);
            CHECK(dynamic_terrain::ogre2MeshExists(scene, nextMesh));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, sharedTexture));
            if (!previousMesh.empty())
            {
                CHECK(!dynamic_terrain::ogre2MeshExists(scene, previousMesh));
                CHECK(previousCpuImage.expired());
                CHECK(previousCpuMesh.expired());
                CHECK(!scene->Material(previousImportedMaterial));
            }
            previousMesh = nextMesh;
            previousImportedMaterial = importedMaterial;
            previousCpuImage = snapshot->pages[0].texture;
            previousCpuMesh = snapshot->pages[0].mesh;
        }

        events.Emit<gz::sim::events::RenderTeardown>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, previousMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, sharedTexture));
        CHECK(previousCpuImage.expired());
        CHECK(previousCpuMesh.expired());
        CHECK(!scene->Material(previousImportedMaterial));
    }
}

void testMountainCameraFrames(const gz::rendering::ScenePtr &scene,
                              std::int64_t nonce)
{
    const auto background = scene->BackgroundColor();
    const auto ambient = scene->AmbientLight();
    scene->SetAmbientLight(gz::math::Color(1, 1, 1));
    scene->SetBackgroundColor(gz::math::Color(0, 0, 1));
    auto camera = scene->CreateCamera("mountain_camera_" + std::to_string(nonce));
    CHECK(camera);
    scene->RootVisual()->AddChild(camera);
    camera->SetImageWidth(160);
    camera->SetImageHeight(90);
    camera->SetAspectRatio(16.0 / 9.0);
    camera->SetHFOV(gz::math::Angle(1.0));
    camera->SetNearClipPlane(0.05);
    camera->SetFarClipPlane(6000);
    camera->SetWorldPose(gz::math::Pose3d(0, 0, 1800, 0, GZ_PI / 2, 0));
    auto image = camera->CreateImage();
    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = true;
    config.cameraNames = {camera->Name()};
    config.visualLightingEnabled = false;
    config.diagnostics = false;
    gz::sim::EventManager events;
    dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
    const std::string prefix = "mountain_pixels_" + std::to_string(nonce);
    for (std::uint64_t generation = 1; generation <= 20; ++generation)
    {
        auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
        snapshot->generation = generation;
        snapshot->resourcePrefix = prefix;
        snapshot->centerTile = {100, 100, 14};
        for (int side = 0; side < 2; ++side)
        {
            auto page = makeTerrainPage(prefix, {100 + side, 100, 14}, side,
                                        0, 1, 100);
            page.mesh = std::make_shared<gz::common::Mesh>();
            page.mesh->SetName(prefix + "_g" + std::to_string(generation) +
                               "_p" + std::to_string(side));
            auto surface = std::make_unique<gz::common::SubMesh>(page.submeshName);
            surface->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
            constexpr int cells = 32;
            for (int y = 0; y <= cells; ++y)
                for (int x = 0; x <= cells; ++x)
                {
                    const double wx = (side - 1) * 3000.0 + 3000.0 * x / cells;
                    const double wy = -3000.0 + 6000.0 * y / cells;
                    const double wz = 400 + 200 * std::sin(wx / 300) *
                                                std::cos(wy / 400);
                    surface->AddVertex(wx, wy, wz);
                    surface->AddNormal(0, 0, 1);
                    surface->AddTexCoord(double(x) / cells, double(y) / cells);
                }
            for (int y = 0; y < cells; ++y)
                for (int x = 0; x < cells; ++x)
                {
                    const unsigned int a = y * (cells + 1) + x;
                    const unsigned int b = a + 1, c = a + cells + 1, d = c + 1;
                    for (auto i : {a, b, c, b, d, c})
                        surface->AddIndex(i);
                }
            page.mesh->AddSubMesh(std::move(surface));
            snapshot->pages.push_back(std::move(page));
        }
        renderer.QueueSnapshot(snapshot);
        for (int frame = 0; frame < 2; ++frame)
        {
            events.Emit<gz::sim::events::PreRender>();
            scene->PreRender();
            camera->PreRender();
            camera->Render();
            camera->PostRender();
            scene->PostRender();
            camera->Copy(image);
            events.Emit<gz::sim::events::PostRender>();
            if (generation == 1 && frame == 0)
                continue;
            const auto *pixels = image.Data<unsigned char>();
            CHECK(image.Depth() == 3);
            for (unsigned int i = 0; i < image.Width() * image.Height(); ++i)
            {
                CHECK(pixels[i * 3] > 20);
                CHECK(std::abs(int(pixels[i * 3]) - pixels[i * 3 + 2]) <= 2);
            }
        }
    }
    events.Emit<gz::sim::events::RenderTeardown>();
    scene->DestroySensor(camera, true);
    scene->SetBackgroundColor(background);
    scene->SetAmbientLight(ambient);
}

void testCameraResidency(const gz::rendering::ScenePtr &scene,
                         std::int64_t nonce)
{
    const std::string cameraLeaf =
        "dynamic_terrain_residency_camera_" + std::to_string(nonce);
    const std::string scopedCameraName =
        "vehicle::link::" + cameraLeaf;
    auto similarlyNamedCamera = scene->CreateCamera(cameraLeaf + "_aux");
    CHECK(similarlyNamedCamera);

    dynamic_terrain::Config config;
    config.visualWarmupFrames = 1;
    config.visualFrustumEviction = true;
    config.visualOffscreenFrames = 2;
    config.visualTextureGuardM = 0.0;
    config.cameraNames = {cameraLeaf};
    config.diagnostics = false;

    auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
    snapshot->generation = 1u;
    snapshot->resourcePrefix =
        "dynamic_terrain_residency_" + std::to_string(nonce);
    snapshot->centerTile = {100, 100, 14};
    snapshot->pages.push_back(makeTerrainPage(
        snapshot->resourcePrefix, {100, 100, 14}, 0u, 10.0, 20.0, 40u));
    snapshot->pages.push_back(makeTerrainPage(
        snapshot->resourcePrefix, {99, 100, 14}, 1u, -20.0, -10.0, 80u));
    for (const auto &page : snapshot->pages)
        snapshot->estimatedTextureBytes +=
            dynamic_terrain::mipmappedRgbaBytes(
                page.textureSize, page.textureSize);

    const std::string frontMesh = snapshot->pages[0].mesh->Name();
    const std::string backMesh = snapshot->pages[1].mesh->Name();
    const std::string frontTexture = snapshot->pages[0].textureName;
    const std::string backTexture = snapshot->pages[1].textureName;

    gz::sim::EventManager events;
    {
        dynamic_terrain::PersistentTerrainRenderer renderer(config, events);
        renderer.QueueSnapshot(snapshot);
        events.Emit<gz::sim::events::PreRender>();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(renderer.HasActiveTerrain());
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture));
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(dynamic_terrain::ogre2TextureExists(scene, backTexture));

        auto camera = scene->CreateCamera(scopedCameraName);
        CHECK(camera);
        CHECK(camera->Name() == scopedCameraName);
        camera->SetImageWidth(640u);
        camera->SetImageHeight(480u);
        camera->SetAspectRatio(4.0 / 3.0);
        gz::math::Angle hfov;
        hfov.SetDegree(90.0);
        camera->SetHFOV(hfov);
        camera->SetNearClipPlane(0.1);
        camera->SetFarClipPlane(100.0);
        camera->SetWorldPose(gz::math::Pose3d(0.0, 0.0, 5.0,
                                              0.0, 0.0, 0.0));

        camera->SetProjectionType(gz::rendering::CPT_ORTHOGRAPHIC);
        events.Emit<gz::sim::events::PreRender>();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        camera->SetProjectionType(gz::rendering::CPT_PERSPECTIVE);
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        try { scene->DestroySensor(camera, true); } catch (...) {}
        camera.reset();
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));

        camera = scene->CreateCamera(scopedCameraName);
        CHECK(camera);
        camera->SetImageWidth(640u);
        camera->SetImageHeight(480u);
        camera->SetAspectRatio(4.0 / 3.0);
        camera->SetHFOV(hfov);
        camera->SetNearClipPlane(0.1);
        camera->SetFarClipPlane(100.0);
        camera->SetWorldPose(gz::math::Pose3d(0.0, 0.0, 5.0,
                                              0.0, 0.0, 0.0));
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
        events.Emit<gz::sim::events::PreRender>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, backTexture));

        camera->SetWorldPose(gz::math::Pose3d(
            15.0, 0.0, 10.0, 0.0, dynamic_terrain::kPi * 0.5, 0.0));
        events.Emit<gz::sim::events::PreRender>();
        CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture));
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, backTexture));

        for (int turn = 0; turn < 20; ++turn)
        {
            const bool frontVisible = (turn % 2) != 0;
            camera->SetWorldPose(gz::math::Pose3d(
                0.0, 0.0, 5.0, 0.0, 0.0,
                frontVisible ? 0.0 : dynamic_terrain::kPi));
            events.Emit<gz::sim::events::PreRender>();
            CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture));
            CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh));
            CHECK(dynamic_terrain::ogre2TextureExists(scene, backTexture));

            events.Emit<gz::sim::events::PreRender>();
            CHECK(dynamic_terrain::ogre2MeshExists(scene, frontMesh) ==
                  frontVisible);
            CHECK(dynamic_terrain::ogre2TextureExists(scene, frontTexture) ==
                  frontVisible);
            CHECK(dynamic_terrain::ogre2MeshExists(scene, backMesh) ==
                  !frontVisible);
            CHECK(dynamic_terrain::ogre2TextureExists(scene, backTexture) ==
                  !frontVisible);
        }

        events.Emit<gz::sim::events::RenderTeardown>();
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, frontMesh));
        CHECK(!dynamic_terrain::ogre2MeshExists(scene, backMesh));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, frontTexture));
        CHECK(!dynamic_terrain::ogre2TextureExists(scene, backTexture));

        try { scene->DestroySensor(camera, true); } catch (...) {}
    }

    try { scene->DestroySensor(similarlyNamedCamera, true); } catch (...) {}
}

int runTest()
{
    std::map<std::string, std::string> parameters;
    auto *engine = gz::rendering::engine("ogre2", parameters);
    if (!engine)
    {
        std::cerr << "SKIP: Ogre2 render engine is unavailable\n";
        return 77;
    }

    const auto nonce = std::chrono::high_resolution_clock::now()
        .time_since_epoch().count();
    RenderCleanup cleanup;
    cleanup.engine = engine;
    cleanup.scene = engine->CreateScene(
        "dynamic_terrain_texture_lifecycle_scene_" +
        std::to_string(nonce));
    if (!cleanup.scene)
    {
        std::cerr << "SKIP: Ogre2 could not create a headless test scene\n";
        return 77;
    }

    auto *ogreEngine = gz::rendering::Ogre2RenderEngine::Instance();
    CHECK(ogreEngine);
    CHECK(ogreEngine->OgreRoot());
    CHECK(ogreEngine->OgreRoot()->getRenderSystem());
    cleanup.textureManager = ogreEngine->OgreRoot()
        ->getRenderSystem()->getTextureGpuManager();
    CHECK(cleanup.textureManager);

    constexpr unsigned int width = 64;
    constexpr unsigned int height = 64;
    std::vector<std::uint8_t> rgb(width * height * 3u, 127u);
    auto image = std::make_shared<gz::common::Image>();
    image->SetFromData(rgb.data(), width, height,
                       gz::common::Image::RGB_INT8);
    CHECK(image->Valid());

    cleanup.textureName =
        "dynamic_terrain_ctest_unique_texture_" + std::to_string(nonce);
    CHECK(!cleanup.textureManager->findTextureNoThrow(cleanup.textureName));

    cleanup.materialA = cleanup.scene->CreateMaterial(
        "dynamic_terrain_ctest_material_a_" + std::to_string(nonce));
    cleanup.materialB = cleanup.scene->CreateMaterial(
        "dynamic_terrain_ctest_material_b_" + std::to_string(nonce));
    CHECK(cleanup.materialA);
    CHECK(cleanup.materialB);

    dynamic_terrain::ResourceReferenceCounter references;
    cleanup.materialA->SetTexture(cleanup.textureName, image);
    CHECK(references.Acquire(cleanup.textureName) == 1u);
    cleanup.materialB->SetTexture(cleanup.textureName, image);
    CHECK(references.Acquire(cleanup.textureName) == 2u);

    auto *texture = cleanup.textureManager->findTextureNoThrow(
        cleanup.textureName);
    CHECK(texture);
    CHECK(texture->getSizeBytes() ==
          dynamic_terrain::mipmappedRgbaBytes(width, height));

    destroyMaterial(cleanup.scene, cleanup.materialA);
    CHECK(!references.Release(cleanup.textureName));
    CHECK(references.References(cleanup.textureName) == 1u);
    CHECK(cleanup.textureManager->findTextureNoThrow(cleanup.textureName));

    destroyMaterial(cleanup.scene, cleanup.materialB);
    CHECK(references.Release(cleanup.textureName));
    CHECK(references.ResourceCount() == 0u);
    texture = cleanup.textureManager->findTextureNoThrow(cleanup.textureName);
    CHECK(texture);
    CHECK(dynamic_terrain::ogre2TextureExists(
        cleanup.scene, cleanup.textureName));
    CHECK(dynamic_terrain::ogre2TextureBytes(
        cleanup.scene, cleanup.textureName) ==
        dynamic_terrain::mipmappedRgbaBytes(width, height));
    CHECK(dynamic_terrain::destroyUnreferencedOgre2Texture(
        cleanup.scene, cleanup.textureName));
    CHECK(!cleanup.textureManager->findTextureNoThrow(cleanup.textureName));

    cleanup.meshName =
        "dynamic_terrain_ctest_unique_mesh_" + std::to_string(nonce);
    gz::common::Mesh commonMesh;
    commonMesh.SetName(cleanup.meshName);
    auto surface = std::make_unique<gz::common::SubMesh>("terrain_test_page");
    surface->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
    surface->AddVertex(0.0, 0.0, 0.0);
    surface->AddNormal(0.0, 0.0, 1.0);
    surface->AddTexCoord(0.0, 0.0);
    surface->AddVertex(1.0, 0.0, 0.0);
    surface->AddNormal(0.0, 0.0, 1.0);
    surface->AddTexCoord(1.0, 0.0);
    surface->AddVertex(0.0, 1.0, 0.0);
    surface->AddNormal(0.0, 0.0, 1.0);
    surface->AddTexCoord(0.0, 1.0);
    surface->AddIndex(0u);
    surface->AddIndex(1u);
    surface->AddIndex(2u);
    commonMesh.AddSubMesh(std::move(surface));

    cleanup.geometry = cleanup.scene->CreateMesh(&commonMesh);
    cleanup.visual = cleanup.scene->CreateVisual(
        "dynamic_terrain_ctest_mesh_visual_" + std::to_string(nonce));
    CHECK(cleanup.geometry);
    CHECK(cleanup.visual);
    cleanup.visual->AddGeometry(cleanup.geometry);
    cleanup.scene->RootVisual()->AddChild(cleanup.visual);
    CHECK(dynamic_terrain::ogre2MeshExists(
        cleanup.scene, cleanup.meshName));
    const auto importedMesh = Ogre::v1::MeshManager::getSingleton().getByName(
        cleanup.meshName + "::::ORIGINAL");
    CHECK(importedMesh);
    const std::string importedMaterialName =
        importedMesh->getSubMesh(0)->getMaterialName();
    CHECK(cleanup.scene->Material(importedMaterialName));

    cleanup.visual->RemoveGeometries();
    cleanup.geometry->Destroy();
    cleanup.geometry.reset();
    cleanup.scene->DestroyVisual(cleanup.visual, true);
    cleanup.visual.reset();
    CHECK(dynamic_terrain::ogre2MeshExists(
        cleanup.scene, cleanup.meshName));
    CHECK(dynamic_terrain::destroyUnreferencedOgre2Mesh(
        cleanup.scene, cleanup.meshName));
    CHECK(!dynamic_terrain::ogre2MeshExists(
        cleanup.scene, cleanup.meshName));
    CHECK(!cleanup.scene->Material(importedMaterialName));

    testRendererRecenters(cleanup.scene, nonce);
    testCameraResidency(cleanup.scene, nonce);
    testMountainCameraFrames(cleanup.scene, nonce);

    std::cout << "Ogre2 texture, mesh, recenter and camera-residency tests passed\n";
    return 0;
}
}

int main()
{
    try
    {
        return runTest();
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
