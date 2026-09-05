#include "GuiTerrain.hh"
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/bytes.pb.h>
#include <QQuickWindow>
#include <QRunnable>
#include <QPointer>
#include <condition_variable>
#include <thread>
#include <map>

namespace dynamic_terrain
{
class DynamicTerrainGui final : public gz::gui::Plugin
{
public:
    ~DynamicTerrainGui() override
    {
        stop_ = true;
        wake_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (main_) main_->removeEventFilter(this);
        if (window_ && render_)
        {
            struct Cleanup : QRunnable
            {
                std::shared_ptr<RenderState> state;
                explicit Cleanup(std::shared_ptr<RenderState> s) : state(std::move(s)) {}
                void run() override { state->Clear(); }
            };
            window_->scheduleRenderJob(new Cleanup(render_), QQuickWindow::AfterRenderingStage);
            window_->update();
        }
    }

protected:
    void LoadConfig(const tinyxml2::XMLElement *element) override
    {
        title = "Terrain GUI preview";
        if (element)
            if (const auto *service = element->FirstChildElement("service"))
                if (service->GetText()) service_ = service->GetText();
        main_ = gz::gui::App()->findChild<gz::gui::MainWindow *>();
        if (!main_) return;
        window_ = main_->QuickWindow();
        render_ = std::make_shared<RenderState>();
        QObject::connect(window_, &QQuickWindow::sceneGraphInvalidated, this,
            [this, state = render_] { state->Clear(); refresh_ = true; }, Qt::DirectConnection);
        main_->installEventFilter(this);
        worker_ = std::thread([this] { Poll(); });
    }

    bool eventFilter(QObject *object, QEvent *event) override
    {
        if (event->type() == gz::gui::events::PreRender::kType)
        {
            std::shared_ptr<TerrainSnapshot> incoming;
            bool clear;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                incoming = std::move(pending_);
                clear = clear_;
                clear_ = false;
            }
            if (clear) render_->Clear();
            if (incoming)
            {
                if (!render_->renderer)
                {
                    Config cfg;
                    cfg.cameraNames.clear();
                    cfg.visualWarmupFrames = 1;
                    cfg.diagnostics = false;
                    render_->renderer = std::make_unique<PersistentTerrainRenderer>(cfg, render_->events);
                }
                if (!render_->sent || render_->sent->generation != incoming->generation)
                    render_->renderer->QueueSnapshot(incoming);
                else
                {
                    TextureUpdate update;
                    update.generation = incoming->generation;
                    for (std::size_t i = 0; i < incoming->pages.size(); ++i)
                    {
                        const auto &page = incoming->pages[i];
                        if (page.textureName == render_->sent->pages[i].textureName) continue;
                        TexturePageUpdate item;
                        item.pageIndex = i; item.pageKey = page.key;
                        item.submeshName = page.submeshName;
                        item.texture = page.texture; item.textureName = page.textureName;
                        item.imageryZoom = page.imageryZoom; item.textureSize = page.textureSize;
                        update.pages.push_back(std::move(item));
                    }
                    render_->renderer->QueueTexture(std::move(update));
                }
                render_->sent = std::move(incoming);
            }
            render_->events.Emit<gz::sim::events::PreRender>();
        }
        else if (event->type() == gz::gui::events::Render::kType)
            render_->events.Emit<gz::sim::events::PostRender>();
        return gz::gui::Plugin::eventFilter(object, event);
    }

private:
    struct RenderState
    {
        gz::sim::EventManager events;
        std::unique_ptr<PersistentTerrainRenderer> renderer;
        std::shared_ptr<TerrainSnapshot> sent;
        void Clear()
        {
            events.Emit<gz::sim::events::RenderTeardown>();
            renderer.reset(); sent.reset();
        }
    };

    bool Request(gz::transport::Node &node, const std::string &service,
                 const std::string &query, gz::msgs::Bytes &reply)
    {
        gz::msgs::StringMsg request;
        request.set_data(query);
        bool result = false;
        return !stop_ && node.Request(service, request, 1000u, reply, result) && result &&
               reply.data().size() <= 32u * 1024u * 1024u;
    }

    void Poll()
    {
        gz::transport::Node node;
        std::map<std::string, TerrainPage> cache;
        std::shared_ptr<TerrainSnapshot> previous;
        std::string previousRevision;
        std::uint64_t generation = 0;
        unsigned failures = 0;
        const std::string viewerSuffix = "_viewer" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        while (!stop_)
        {
            try
            {
                if (refresh_.exchange(false)) previousRevision.clear();
                std::string service = service_;
                if (service.empty())
                {
                    std::vector<std::string> services;
                    node.ServiceList(services);
                    for (const auto &candidate : services)
                        if (candidate.size() >= 12 &&
                            candidate.compare(candidate.size() - 12, 12, "/terrain_gui") == 0)
                        {
                            if (!service.empty()) { service.clear(); break; }
                            service = candidate;
                        }
                }
                gz::msgs::Bytes reply;
                preview::Manifest manifest;
                if (service.empty() || !Request(node, service, "", reply) ||
                    !manifest.ParseFromString(reply.data()) || manifest.version() != 1 ||
                    manifest.pages_size() == 0 || manifest.pages_size() > 4225)
                    throw std::runtime_error("terrain GUI source unavailable");
                failures = 0;
                const auto revision = manifest.prefix() + ":" + std::to_string(manifest.revision());
                if (revision != previousRevision)
                {
                    std::map<std::string, TerrainPage> retained;
                    for (const auto &info : manifest.pages())
                    {
                        const auto key = info.mesh_name() + "|" + info.texture_name();
                        auto it = cache.find(key);
                        if (it != cache.end()) retained.emplace(key, it->second);
                    }
                    cache = std::move(retained);
                    auto snapshot = std::make_shared<TerrainSnapshot>();
                    snapshot->resourcePrefix = manifest.prefix() + viewerSuffix;
                    bool sameGeometry = previous &&
                        previous->resourcePrefix == snapshot->resourcePrefix &&
                        previous->pages.size() == static_cast<std::size_t>(manifest.pages_size());
                    for (int i = 0; i < manifest.pages_size(); ++i)
                    {
                        const auto &info = manifest.pages(i);
                        if (info.index() != static_cast<unsigned>(i))
                            throw std::runtime_error("invalid terrain GUI page order");
                        const auto key = info.mesh_name() + "|" + info.texture_name();
                        if (!cache.count(key))
                        {
                            preview::Page packet;
                            if (!Request(node, service, std::to_string(manifest.revision()) +
                                " " + std::to_string(i), reply) ||
                                !packet.ParseFromString(reply.data()) ||
                                packet.info().SerializeAsString() != info.SerializeAsString())
                                throw std::runtime_error("terrain GUI page superseded");
                            auto page = decodeGuiPage(packet);
                            page.mesh->SetName(info.mesh_name() + viewerSuffix);
                            page.textureName += viewerSuffix;
                            if (previous && i < static_cast<int>(previous->pages.size()) &&
                                previous->pages[i].mesh->Name() == info.mesh_name() + viewerSuffix)
                                page.mesh = previous->pages[i].mesh;
                            cache.emplace(key, std::move(page));
                        }
                        snapshot->pages.push_back(cache.at(key));
                        sameGeometry = sameGeometry && previous->pages[i].mesh->Name() == info.mesh_name() + viewerSuffix;
                        snapshot->estimatedTextureBytes += mipmappedRgbaBytes(
                            info.texture_size(), info.texture_size());
                    }
                    snapshot->generation = sameGeometry ? previous->generation : ++generation;
                    snapshot->centerTile = snapshot->pages[snapshot->pages.size()/2].key;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        pending_ = snapshot;
                    }
                    previous = std::move(snapshot);
                    previousRevision = revision;
                }
            }
            catch (const std::exception &)
            {
                if (++failures >= 3)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    clear_ = true; pending_.reset();
                    previous.reset(); cache.clear(); previousRevision.clear();
                }
            }
            std::unique_lock<std::mutex> lock(sleepMutex_);
            wake_.wait_for(lock, std::chrono::seconds(1), [this] { return stop_.load(); });
        }
    }

    QPointer<gz::gui::MainWindow> main_;
    QPointer<QQuickWindow> window_;
    std::shared_ptr<RenderState> render_;
    std::string service_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> refresh_{false};
    std::thread worker_;
    std::mutex mutex_, sleepMutex_;
    std::condition_variable wake_;
    std::shared_ptr<TerrainSnapshot> pending_;
    bool clear_{false};
};
}
GZ_ADD_PLUGIN(dynamic_terrain::DynamicTerrainGui, gz::gui::Plugin)
