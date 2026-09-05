#include "GuiTerrain.hh"
#include <gz/common/SubMesh.hh>
#include <gz/msgs/bytes.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <cmath>

namespace dynamic_terrain
{
preview::PageInfo guiPageInfo(const TerrainPage &page)
{
    preview::PageInfo info;
    info.set_x(page.key.x); info.set_y(page.key.y); info.set_z(page.key.z);
    info.set_index(page.index);
    info.set_mesh_name(page.mesh->Name());
    info.set_texture_name(page.textureName + "_gui512");
    info.set_submesh_name(page.submeshName);
    info.set_texture_size(std::min(page.textureSize, kGuiTextureSize));
    info.set_imagery_zoom(page.imageryZoom);
    return info;
}

preview::Page encodeGuiPage(const TerrainPage &page)
{
    preview::Page packet;
    *packet.mutable_info() = guiPageInfo(page);
    const auto surface = page.mesh->SubMeshByIndex(0).lock();
    if (!surface || !page.texture || !page.texture->Valid())
        throw std::runtime_error("invalid GUI terrain source");
    for (unsigned int i = 0; i < surface->VertexCount(); ++i)
    {
        const auto v = surface->Vertex(i), n = surface->Normal(i);
        const auto uv = surface->TexCoord(i);
        for (const auto value : {v.X(), v.Y(), v.Z()}) packet.add_vertices(value);
        for (const auto value : {n.X(), n.Y(), n.Z()}) packet.add_normals(value);
        packet.add_uvs(uv.X()); packet.add_uvs(uv.Y());
    }
    for (unsigned int i = 0; i < surface->IndexCount(); ++i)
        packet.add_indices(surface->Index(i));
    auto rgb = page.texture->RGBData();
    cv::Mat pixels(page.texture->Height(), page.texture->Width(), CV_8UC3, rgb.data());
    cv::Mat small, bgr;
    const int side = packet.info().texture_size();
    cv::resize(pixels, small, cv::Size(side, side), 0, 0, cv::INTER_AREA);
    cv::cvtColor(small, bgr, cv::COLOR_RGB2BGR);
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".png", bgr, encoded, {cv::IMWRITE_PNG_COMPRESSION, 1}))
        throw std::runtime_error("GUI terrain PNG encoding failed");
    packet.set_png(encoded.data(), encoded.size());
    return packet;
}

TerrainPage decodeGuiPage(const preview::Page &packet)
{
    const auto &info = packet.info();
    const int vertices = packet.vertices_size() / 3;
    if (vertices < 3 || vertices > 100000 || packet.vertices_size() % 3 ||
        packet.normals_size() != vertices * 3 || packet.uvs_size() != vertices * 2 ||
        packet.indices_size() == 0 || packet.indices_size() % 3 ||
        packet.indices_size() > 600000 || info.texture_size() == 0 ||
        info.texture_size() > kGuiTextureSize || info.mesh_name().empty() ||
        info.texture_name().empty() || packet.png().size() < 24 ||
        packet.png().size() > 2 * 1024 * 1024)
        throw std::runtime_error("invalid GUI terrain packet");
    const auto *png = reinterpret_cast<const unsigned char *>(packet.png().data());
    const auto dimension = [&](int offset) {
        return (unsigned(png[offset]) << 24) | (unsigned(png[offset+1]) << 16) |
               (unsigned(png[offset+2]) << 8) | unsigned(png[offset+3]);
    };
    if (dimension(16) != info.texture_size() || dimension(20) != info.texture_size())
        throw std::runtime_error("unexpected GUI PNG dimensions");
    TerrainPage page;
    page.key = {info.x(), info.y(), info.z()}; page.index = info.index();
    page.submeshName = info.submesh_name(); page.textureName = info.texture_name();
    page.textureSize = info.texture_size(); page.imageryZoom = info.imagery_zoom();
    page.mesh = std::make_shared<gz::common::Mesh>();
    page.mesh->SetName(info.mesh_name());
    auto mesh = std::make_unique<gz::common::SubMesh>(page.submeshName);
    mesh->SetPrimitiveType(gz::common::SubMesh::TRIANGLES);
    for (double value : packet.vertices()) if (!std::isfinite(value))
        throw std::runtime_error("nonfinite GUI vertex");
    for (double value : packet.normals()) if (!std::isfinite(value))
        throw std::runtime_error("nonfinite GUI normal");
    for (double value : packet.uvs()) if (!std::isfinite(value))
        throw std::runtime_error("nonfinite GUI UV");
    for (int i = 0; i < vertices; ++i)
    {
        mesh->AddVertex(packet.vertices(i*3), packet.vertices(i*3+1), packet.vertices(i*3+2));
        mesh->AddNormal(packet.normals(i*3), packet.normals(i*3+1), packet.normals(i*3+2));
        mesh->AddTexCoord(packet.uvs(i*2), packet.uvs(i*2+1));
    }
    for (auto index : packet.indices())
    {
        if (index >= static_cast<unsigned>(vertices))
            throw std::runtime_error("invalid GUI mesh index");
        mesh->AddIndex(index);
    }
    page.mesh->AddSubMesh(std::move(mesh));
    const std::vector<unsigned char> bytes(packet.png().begin(), packet.png().end());
    cv::Mat bgr = cv::imdecode(bytes, cv::IMREAD_COLOR), rgb;
    if (bgr.empty()) throw std::runtime_error("invalid GUI PNG");
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    page.texture = std::make_shared<gz::common::Image>();
    page.texture->SetFromData(rgb.data, rgb.cols, rgb.rows, gz::common::Image::RGB_INT8);
    return page;
}

struct GuiTerrainSource::State
{
    std::mutex mutex, encodeMutex;
    std::shared_ptr<TerrainSnapshot> snapshot;
    std::uint64_t revision{0};
};

GuiTerrainSource::GuiTerrainSource(const std::string &service)
    : state_(std::make_shared<State>())
{
    std::function<bool(const gz::msgs::StringMsg &, gz::msgs::Bytes &)> callback =
        [state = state_](const auto &request, auto &response) {
        try
        {
            if (request.data().empty())
            {
                preview::Manifest manifest;
                std::lock_guard<std::mutex> lock(state->mutex);
                manifest.set_version(1);
                manifest.set_revision(state->revision);
                if (state->snapshot)
                {
                    manifest.set_prefix(state->snapshot->resourcePrefix);
                    for (const auto &page : state->snapshot->pages)
                        *manifest.add_pages() = guiPageInfo(page);
                }
                return manifest.SerializeToString(response.mutable_data());
            }
            std::uint64_t revision;
            std::size_t index;
            std::istringstream input(request.data());
            if (!(input >> revision >> index) || !(input >> std::ws).eof()) return false;
            std::lock_guard<std::mutex> encoding(state->encodeMutex);
            TerrainPage page;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (!state->snapshot || revision != state->revision ||
                    index >= state->snapshot->pages.size()) return false;
                page = state->snapshot->pages[index];
            }
            return encodeGuiPage(page).SerializeToString(response.mutable_data());
        }
        catch (const std::exception &) { return false; }
    };
    if (!node_.Advertise(service, callback))
        throw std::runtime_error("cannot advertise terrain GUI service: " + service);
}

void GuiTerrainSource::SetSnapshot(const std::shared_ptr<TerrainSnapshot> &snapshot)
{
    if (!snapshot) return;
    auto copy = std::make_shared<TerrainSnapshot>(*snapshot);
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->snapshot = std::move(copy);
    ++state_->revision;
}

void GuiTerrainSource::ApplyTexture(const TextureUpdate &update)
{
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->snapshot || update.generation != state_->snapshot->generation) return;
    bool changed = false;
    for (const auto &incoming : update.pages)
    {
        if (incoming.pageIndex >= state_->snapshot->pages.size()) continue;
        auto &page = state_->snapshot->pages[incoming.pageIndex];
        if (page.key != incoming.pageKey || page.submeshName != incoming.submeshName ||
            incoming.imageryZoom < page.imageryZoom || !incoming.texture) continue;
        page.texture = incoming.texture; page.textureName = incoming.textureName;
        page.imageryZoom = incoming.imageryZoom; page.textureSize = incoming.textureSize;
        changed = true;
    }
    if (changed) ++state_->revision;
}
}
