#include "shift/gui/scene.h"

#include <QtDebug>

#include <algorithm>
#include <cmath>

#include "ksl/matrix.h"

/* grow the box by the corners of a body's box transformed by its pose */
static void Grow(const ksl_SE3f_t& pose, const ksl_vec3f_t& local_min,
                 const ksl_vec3f_t& local_max, ksl_vec3f_t& box_min,
                 ksl_vec3f_t& box_max) {
  for(int corner = 0; corner < 8; corner++) {
    const ksl_vec3f_t local =
      ksl_vec3f((corner & 1) ? local_max.x : local_min.x,
                (corner & 2) ? local_max.y : local_min.y,
                (corner & 4) ? local_max.z : local_min.z);
    ksl_vec3f_t world;
    ksl_product_dvf(&pose, &local, &world);
    for(int k = 0; k < 3; k++) {
      box_min.at[k] = std::min(box_min.at[k], world.at[k]);
      box_max.at[k] = std::max(box_max.at[k], world.at[k]);
    }
  }
}

void shift::gui::SceneBounds(const Scene& scene, const Poses& poses,
                             ksl_vec3f_t& box_min, ksl_vec3f_t& box_max) {
  box_min = ksl_vec3f(1e30f, 1e30f, 1e30f);
  box_max = ksl_vec3f(-1e30f, -1e30f, -1e30f);
  for(const auto& instance : scene.instances) {
    const SceneMesh& mesh = scene.meshes[instance.mesh_index];
    if(mesh.terrain >= 0) {
      continue; // frame the model, not the ground it stands on
    }
    Grow(poses.bodies.at(instance.body_index), mesh.bounds_min,
         mesh.bounds_max, box_min, box_max);
  }
  for(const auto& cad : scene.cads) {
    if(cad.patches.empty()) {
      continue;
    }
    for(int body : cad.bodies) {
      Grow(poses.bodies.at(body), cad.bounds_min, cad.bounds_max, box_min,
           box_max);
    }
  }
  if(box_min.x > box_max.x) {
    box_min = ksl_vec3f(-1.f, -1.f, -1.f);
    box_max = ksl_vec3f(1.f, 1.f, 1.f);
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

void shift::gui::ComputeNormals(std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices) {
  for(auto& vertex : vertices) {
    std::fill(vertex.normal, vertex.normal + 3, 0.f);
  }
  for(size_t i = 0; i + 2 < indices.size(); i += 3) {
    shift::gui::Vertex* v[3] = {&vertices[indices[i]],
                                &vertices[indices[i + 1]],
                                &vertices[indices[i + 2]]};
    ksl_vec3f_t p[3];
    for(int k = 0; k < 3; k++) {
      p[k] = ksl_vec3f(v[k]->position[0], v[k]->position[1],
                       v[k]->position[2]);
    }
    ksl_vec3f_t e1, e2, n;
    ksl_subtract_vvf(&p[1], &p[0], &e1);
    ksl_subtract_vvf(&p[2], &p[0], &e2);
    ksl_cross_vvf(&e1, &e2, &n);
    for(auto vertex : v) {
      for(int k = 0; k < 3; k++) {
        vertex->normal[k] += n.at[k];
      }
    }
  }
  /* divided by the length, however small the triangles; a vertex of no
  area keeps its zero normal */
  for(auto& vertex : vertices) {
    ksl_vec3f_t n = ksl_vec3f(vertex.normal[0], vertex.normal[1],
                              vertex.normal[2]);
    const float length = ksl_vec3f_l2norm(&n);
    if(length > 0.f) {
      ksl_vec3f_scale(&n, 1.f / length);
    }
    for(int k = 0; k < 3; k++) {
      vertex.normal[k] = n.at[k];
    }
  }
}

void shift::gui::DeformFlexMesh(const SceneMesh& mesh, const FlexUpdate& update,
                                std::vector<Vertex>& vertices) {
  vertices = mesh.vertices;
  const size_t columns = mesh.flex_columns;
  if(update.coefficients.size() != columns ||
     mesh.flex_basis.size() != 3 * columns * vertices.size()) {
    return;
  }
  for(size_t v = 0; v < vertices.size(); v++) {
    const ksl_vec3f_t x0 = ksl_vec3f(vertices[v].position[0],
                                     vertices[v].position[1],
                                     vertices[v].position[2]);
    ksl_vec3f_t d, x;
    ksl_subtract_vvf(&x0, &mesh.flex_centroid, &d);
    ksl_cross_vvf(&update.rotation, &d, &x);
    ksl_add_vvf(&x, &update.translation, &x);
    ksl_add_vvf(&x, &x0, &x);
    const float* s = mesh.flex_basis.data() + 3 * columns * v;
    for(size_t j = 0; j < columns; j++) {
      const float a = update.coefficients[j];
      for(int c = 0; c < 3; c++) {
        x.at[c] += a * s[3 * j + c];
      }
    }
    for(int c = 0; c < 3; c++) {
      vertices[v].position[c] = x.at[c];
    }
  }
  ComputeNormals(vertices, mesh.indices);
}
