/* Textures embedded in a binary glTF (.glb) file: the mesh loader keeps the
image (geometry::MeshAsset::embedded_textures) and the viewer's texture
loader decodes it */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <QBuffer>
#include <QImage>
#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/geometry/mesh.h"
#include "shift/gui/scene.h"

static void Append(std::vector<uint8_t>& out, const void* data, size_t size) {
  const auto* p = static_cast<const uint8_t*>(data);
  out.insert(out.end(), p, p + size);
}

static void AppendU32(std::vector<uint8_t>& out, uint32_t value) {
  Append(out, &value, 4); // glb is little endian, as x86
}

/* one triangle whose base color is a 4 x 4 PNG of color, stored in the
binary buffer of the file */
static void WriteGlb(const std::string& path, QRgb color) {
  QImage image(4, 4, QImage::Format_RGBA8888);
  image.fill(QColor::fromRgb(color));
  QByteArray png;
  QBuffer buffer(&png);
  buffer.open(QIODevice::WriteOnly);
  ASSERT_TRUE(image.save(&buffer, "PNG"));

  std::vector<uint8_t> bin;
  const float positions[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
  const float uvs[6] = {0, 0, 1, 0, 0, 1};
  Append(bin, positions, sizeof(positions));
  Append(bin, uvs, sizeof(uvs));
  Append(bin, png.constData(), size_t(png.size()));
  while(bin.size() % 4) {
    bin.push_back(0);
  }
  std::ostringstream json;
  json << R"({"asset": {"version": "2.0"}, "scene": 0,
  "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
  "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "TEXCOORD_0": 1},
                              "material": 0}]}],
  "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
  "textures": [{"source": 0}],
  "images": [{"bufferView": 2, "mimeType": "image/png"}],
  "accessors": [
    {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3",
     "min": [0, 0, 0], "max": [1, 1, 0]},
    {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC2"}],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
    {"buffer": 0, "byteOffset": 36, "byteLength": 24},
    {"buffer": 0, "byteOffset": 60, "byteLength": )"
       << png.size() << R"(}],
  "buffers": [{"byteLength": )"
       << bin.size() << "}]}";
  std::string text = json.str();
  while(text.size() % 4) {
    text += ' ';
  }
  std::vector<uint8_t> glb;
  AppendU32(glb, 0x46546C67); // "glTF"
  AppendU32(glb, 2);
  AppendU32(glb, uint32_t(12 + 8 + text.size() + 8 + bin.size()));
  AppendU32(glb, uint32_t(text.size()));
  AppendU32(glb, 0x4E4F534A); // "JSON"
  Append(glb, text.data(), text.size());
  AppendU32(glb, uint32_t(bin.size()));
  AppendU32(glb, 0x004E4942); // "BIN"
  Append(glb, bin.data(), bin.size());
  std::ofstream file(path, std::ios::binary);
  file.write(reinterpret_cast<const char*>(glb.data()), long(glb.size()));
}

TEST(EmbeddedTexture, GlbBaseColor) {
  const std::string dir = testing::TempDir();
  const std::string glb = dir + "/embedded_texture.glb";
  const std::string model = dir + "/embedded_texture.json";
  const QRgb color = qRgb(200, 40, 10);
  WriteGlb(glb, color);
  {
    std::ofstream file(model);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
 "assemblies": [{"name": "a",
  "bodies": [{"name": "plate",
              "geometry": [{"type": "mesh",
                            "filename": ["embedded_texture.glb"]}],
              "center_of_mass": [0, 0, 0],
              "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                          "Ixy": 0, "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "J", "type": "revolute",
              "parent": [["fixed"], "origin"], "child": [["plate"], "origin"],
              "axis": [0, 0, 1]}]}]})";
  }
  shift::core::Context context;
  context.LoadJSON(model);
  std::remove(model.c_str());
  std::remove(glb.c_str());

  const shift::core::geometry::MeshAsset* asset = nullptr;
  for(auto body : context.get_active_articulated_bodies()) {
    for(auto& geometry : body->get_geometry()) {
      if(auto mesh =
           dynamic_cast<shift::core::geometry::Mesh*>(geometry.get())) {
        asset = mesh->get_asset();
      }
    }
  }
  ASSERT_NE(asset, nullptr);
  ASSERT_EQ(asset->mesh_data.size(), 1u);
  const int material = asset->mesh_data[0].material_index;
  ASSERT_GE(material, 0);
  const std::string& key = asset->material_data[material].base_color_map;
  ASSERT_EQ(asset->embedded_textures.count(key), 1u) << key;
  EXPECT_EQ(asset->embedded_textures.at(key).height, 0u); // encoded PNG

  shift::gui::Scene scene;
  shift::gui::TextureLoader textures(scene);
  textures.AddEmbedded(asset->embedded_textures);
  const int layer = textures.Load(key);
  ASSERT_EQ(layer, 0);
  const QImage& image = scene.texture_layers[0];
  const QColor texel = image.pixelColor(image.width() / 2, image.height() / 2);
  EXPECT_EQ(texel.rgb(), color);
}
