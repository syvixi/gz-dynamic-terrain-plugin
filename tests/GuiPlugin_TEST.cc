#include "GuiTerrain.hh"
#include <gz/common/SubMesh.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <QQuickItem>
#include <QQuickWindow>
#include <thread>
#include <iostream>

#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

int main(int argc, char **argv)
{
    try
    {
        CHECK(argc == 2);
        const std::string libraryPath = argv[1];
        gz::gui::Application app(argc, argv);
        app.AddPluginPath(libraryPath);
        auto *main = app.findChild<gz::gui::MainWindow *>();
        CHECK(main);
        auto *engine = gz::rendering::engine("ogre2");
        CHECK(engine);
        auto scene = engine->CreateScene("gui_preview_test_scene");
        CHECK(scene);
        const auto baseVisuals = scene->VisualCount();
        const auto service = "/gui_plugin_test_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        dynamic_terrain::GuiTerrainSource source(service);
        auto snapshot = std::make_shared<dynamic_terrain::TerrainSnapshot>();
        snapshot->generation = 1; snapshot->resourcePrefix = "gui_plugin_fixture";
        dynamic_terrain::TerrainPage page;
        page.key = {1, 2, 14}; page.submeshName = "page";
        page.textureName = "gui_plugin_texture"; page.textureSize = 64;
        std::vector<unsigned char> pixels(64 * 64 * 3, 128);
        page.texture = std::make_shared<gz::common::Image>();
        page.texture->SetFromData(pixels.data(), 64, 64, gz::common::Image::RGB_INT8);
        page.mesh = std::make_shared<gz::common::Mesh>();
        page.mesh->SetName("gui_plugin_mesh");
        auto mesh = std::make_unique<gz::common::SubMesh>(page.submeshName);
        for (int i = 0; i < 3; ++i)
        {
            mesh->AddVertex(i == 1 ? 1 : 0, i == 2 ? 1 : 0, 0);
            mesh->AddNormal(0, 0, 1); mesh->AddTexCoord(0, 0); mesh->AddIndex(i);
        }
        page.mesh->AddSubMesh(std::move(mesh));
        snapshot->pages.push_back(std::move(page));
        source.SetSnapshot(snapshot);
        tinyxml2::XMLDocument config;
        config.Parse(("<plugin filename='DynamicTerrainGui'><service>" + service +
                      "</service></plugin>").c_str());
        CHECK(app.LoadPlugin("DynamicTerrainGui", config.RootElement()));
        std::string pluginName;
        for (auto *plugin : main->findChildren<gz::gui::Plugin *>())
            if (plugin->Title() == "Terrain GUI preview")
                pluginName = plugin->CardItem()->objectName().toStdString();
        CHECK(!pluginName.empty());
        bool loaded = false;
        for (int i = 0; i < 150; ++i)
        {
            app.processEvents();
            gz::gui::events::PreRender pre;
            gz::gui::events::Render post;
            QCoreApplication::sendEvent(main, &pre);
            QCoreApplication::sendEvent(main, &post);
            if (scene->VisualCount() >= baseVisuals + 2) { loaded = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        CHECK(loaded);
        for (int i = 0; i < 2; ++i)
        {
            gz::gui::events::PreRender pre;
            gz::gui::events::Render post;
            QCoreApplication::sendEvent(main, &pre);
            QCoreApplication::sendEvent(main, &post);
        }
        bool pageGeometry = false;
        for (unsigned int i = 0; i < scene->VisualCount(); ++i)
        {
            const auto visual = scene->VisualByIndex(i);
            if (visual->Name().find("gui_plugin_fixture") != std::string::npos &&
                visual->GeometryCount() == 1)
                pageGeometry = true;
        }
        CHECK(pageGeometry);
        CHECK(QMetaObject::invokeMethod(main->QuickWindow(), "sceneGraphInvalidated",
                                       Qt::DirectConnection));
        CHECK(scene->VisualCount() == baseVisuals);
        CHECK(app.RemovePlugin(pluginName));
        engine->DestroyScene(scene);
        std::cout << "GUI plugin load, transport, render events and cleanup passed\n";
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
