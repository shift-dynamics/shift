#include "shift/gui/scene.h"

#include <QtDebug>

#include <algorithm>
#include <cmath>

void shift::gui::SceneBounds(const Scene& scene, const Poses& poses,
                             QVector3D& box_min, QVector3D& box_max) {
  box_min = QVector3D(1e30f, 1e30f, 1e30f);
  box_max = -box_min;
  for(const auto& instance : scene.instances) {
    const SceneMesh& mesh = scene.meshes[instance.mesh_index];
    if(mesh.terrain >= 0) {
      continue; // frame the model, not the ground it stands on
    }
    const QMatrix4x4& pose = poses.bodies.at(instance.body_index);
    for(int corner = 0; corner < 8; corner++) {
      const QVector3D local((corner & 1) ? mesh.bounds_max.x() : mesh.bounds_min.x(),
                            (corner & 2) ? mesh.bounds_max.y() : mesh.bounds_min.y(),
                            (corner & 4) ? mesh.bounds_max.z() : mesh.bounds_min.z());
      const QVector3D world = pose.map(local);
      for(int k = 0; k < 3; k++) {
        box_min[k] = std::min(box_min[k], world[k]);
        box_max[k] = std::max(box_max[k], world[k]);
      }
    }
  }
  for(const auto& cad : scene.cads) {
    if(cad.patches.empty()) {
      continue;
    }
    for(int body : cad.bodies) {
      const QMatrix4x4& pose = poses.bodies.at(body);
      for(int corner = 0; corner < 8; corner++) {
        const QVector3D local(
          (corner & 1) ? cad.bounds_max.x() : cad.bounds_min.x(),
          (corner & 2) ? cad.bounds_max.y() : cad.bounds_min.y(),
          (corner & 4) ? cad.bounds_max.z() : cad.bounds_min.z());
        const QVector3D world = pose.map(local);
        for(int k = 0; k < 3; k++) {
          box_min[k] = std::min(box_min[k], world[k]);
          box_max[k] = std::max(box_max[k], world[k]);
        }
      }
    }
  }
  if(box_min.x() > box_max.x()) {
    box_min = QVector3D(-1.f, -1.f, -1.f);
    box_max = QVector3D(1.f, 1.f, 1.f);
  }
}

void shift::gui::TextureLoader::AddEmbedded(
  const std::map<std::string, EmbeddedTexture>& images) {
  for(const auto& [key, image] : images) {
    embedded_[key] = &image;
  }
}

QImage shift::gui::TextureLoader::Read(const std::string& path) {
  QImage image;
  auto embedded = embedded_.find(path);
  if(embedded == embedded_.end()) {
    // qrc:/textures/... is a texture built into the viewer, which Qt names
    // :/textures/...
    const bool resource = path.rfind("qrc:/", 0) == 0;
    image.load(QString::fromStdString(resource ? path.substr(3) : path));
  } else if(embedded->second->height == 0) {
    // an encoded image file
    const auto& data = embedded->second->data;
    image.loadFromData(data.data(), static_cast<int>(data.size()));
  } else {
    // B G R A texels, as Qt's ARGB32 on little endian machines
    const EmbeddedTexture& t = *embedded->second;
    image = QImage(t.data.data(), int(t.width), int(t.height),
                   int(4 * t.width), QImage::Format_ARGB32)
              .copy();
  }
  if(image.isNull()) {
    qWarning("Cannot read texture %s", path.c_str());
    return image;
  }
  const int size = Scene::kTextureSize;
  if(image.width() != size || image.height() != size) {
    image = image.scaled(size, size, Qt::IgnoreAspectRatio,
                         Qt::SmoothTransformation);
  }
  return image.convertToFormat(QImage::Format_RGBA8888);
}

int shift::gui::TextureLoader::Load(const std::string& path) {
  if(path.empty()) {
    return -1;
  }
  auto it = layers_.find(path);
  if(it != layers_.end()) {
    return it->second;
  }
  QImage image = Read(path);
  int layer = -1;
  if(!image.isNull()) {
    layer = static_cast<int>(scene_.texture_layers.size());
    scene_.texture_layers.push_back(std::move(image));
  }
  layers_[path] = layer;
  return layer;
}

int shift::gui::TextureLoader::LoadMetallicRoughness(
  const std::string& packed, const std::string& metallic,
  const std::string& roughness) {
  if(!packed.empty()) {
    return Load(packed);
  }
  if(metallic.empty() && roughness.empty()) {
    return -1;
  }
  const std::string key = "metallic:" + metallic + "|roughness:" + roughness;
  auto it = layers_.find(key);
  if(it != layers_.end()) {
    return it->second;
  }
  // channels without an image are 1, so the factor alone applies
  const int size = Scene::kTextureSize;
  QImage combined(size, size, QImage::Format_RGBA8888);
  combined.fill(Qt::white);
  auto copy_channel = [&](const std::string& path, int channel) {
    if(path.empty()) {
      return;
    }
    QImage image = Read(path);
    if(image.isNull()) {
      return;
    }
    // single channel images: red equals the gray value
    for(int y = 0; y < size; y++) {
      const uchar* in = image.constScanLine(y);
      uchar* out = combined.scanLine(y);
      for(int x = 0; x < size; x++) {
        out[4 * x + channel] = in[4 * x];
      }
    }
  };
  copy_channel(roughness, 1);
  copy_channel(metallic, 2);
  const int layer = static_cast<int>(scene_.texture_layers.size());
  scene_.texture_layers.push_back(std::move(combined));
  layers_[key] = layer;
  return layer;
}

/* the noise of surfaces.glsl (CellRandom, ValueNoise, Fbm), in single
precision as the shaders evaluate it */
static float CellRandom(int32_t x, int32_t y, uint32_t seed) {
  uint32_t h = static_cast<uint32_t>(x) * 0x8da6b343u ^
               static_cast<uint32_t>(y) * 0xd8163841u ^ seed * 0xcb1ab31fu;
  h ^= h >> 13;
  h *= 0x5bd1e995u;
  h ^= h >> 15;
  return float(h & 0xffffffu) / 16777215.f;
}

static float ValueNoise(float x, float y, uint32_t seed) {
  const float ix = std::floor(x), iy = std::floor(y);
  const float fx = x - ix, fy = y - iy;
  const float ux = fx * fx * (3.f - 2.f * fx);
  const float uy = fy * fy * (3.f - 2.f * fy);
  const int32_t cx = int32_t(ix), cy = int32_t(iy);
  const float a = CellRandom(cx, cy, seed);
  const float b = CellRandom(cx + 1, cy, seed);
  const float d = CellRandom(cx, cy + 1, seed);
  const float e = CellRandom(cx + 1, cy + 1, seed);
  const float bottom = a + (b - a) * ux;
  const float top = d + (e - d) * ux;
  return bottom + (top - bottom) * uy;
}

static float Fbm(float x, float y, uint32_t seed) {
  float sum = 0.f;
  float amplitude = 0.5f;
  for(uint32_t k = 0; k < 4; k++) {
    sum += amplitude * ValueNoise(x, y, seed + k);
    x = x * 2.f + 17.31f;
    y = y * 2.f + 9.13f;
    amplitude *= 0.5f;
  }
  return sum / 0.9375f;
}

float shift::gui::SurfaceNoiseLevel(int surface, double coverage) {
  if(coverage <= 0.0) {
    return 2.f; // above any noise: not drawn
  }
  if(coverage >= 1.0) {
    return -1.f;
  }
  /* the quantile of the noise over many patches (seed as in SurfaceMask) */
  constexpr int kSide = 256;
  std::vector<float> samples;
  samples.reserve(kSide * kSide);
  const uint32_t seed = 101u + 17u * uint32_t(surface);
  for(int j = 0; j < kSide; j++) {
    for(int i = 0; i < kSide; i++) {
      samples.push_back(Fbm(0.37f * i + 0.11f, 0.41f * j + 0.07f, seed));
    }
  }
  const size_t k = std::min(samples.size() - 1,
                            size_t((1.0 - coverage) * samples.size()));
  std::nth_element(samples.begin(), samples.begin() + k, samples.end());
  return samples[k];
}
