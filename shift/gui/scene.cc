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

float shift::gui::FlexVonMises(const SceneMesh& mesh,
                               const FlexUpdate& update, size_t vertex) {
  const size_t columns = mesh.flex_columns;
  if(update.stress_coefficients.size() != columns ||
     mesh.flex_stress_basis.size() != 6 * columns * mesh.vertices.size()) {
    return 0.f;
  }
  float s[6] = {};
  const float* b = mesh.flex_stress_basis.data() + 6 * columns * vertex;
  for(size_t j = 0; j < columns; j++) {
    const float a = update.stress_coefficients[j];
    for(int c = 0; c < 6; c++) {
      s[c] += a * b[6 * j + c];
    }
  }
  const float d0 = s[0] - s[1], d1 = s[1] - s[2], d2 = s[2] - s[0];
  return std::sqrt(0.5f * (d0 * d0 + d1 * d1 + d2 * d2) +
                   3.f * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]));
}

float shift::gui::DeformFlexMesh(const SceneMesh& mesh,
                                 const FlexUpdate& update,
                                 std::vector<Vertex>& vertices) {
  vertices = mesh.vertices;
  const size_t columns = mesh.flex_columns;
  if(update.coefficients.size() != columns ||
     mesh.flex_basis.size() != 3 * columns * vertices.size()) {
    return 0.f;
  }
  /* the stress, mapped over its range */
  float largest = 0.f;
  if(update.stress != FlexUpdate::Stress::kNone) {
    std::vector<float> stress(vertices.size());
    for(size_t v = 0; v < vertices.size(); v++) {
      stress[v] = FlexVonMises(mesh, update, v);
      largest = std::max(largest, stress[v]);
    }
    const float low = update.stress == FlexUpdate::Stress::kAutomatic
                        ? 0.f
                        : update.stress_range[0];
    const float high = update.stress == FlexUpdate::Stress::kAutomatic
                         ? largest
                         : update.stress_range[1];
    for(size_t v = 0; v < vertices.size(); v++) {
      vertices[v].uv[0] =
        high > low ? std::clamp((stress[v] - low) / (high - low), 0.f, 1.f)
                   : 0.f;
    }
  }
  for(size_t v = 0; v < vertices.size(); v++) {
    const ksl_vec3f_t x0 = ksl_vec3f(vertices[v].position[0],
                                     vertices[v].position[1],
                                     vertices[v].position[2]);
    ksl_vec3f_t d, x;
    ksl_subtract_vvf(&x0, &mesh.flex_centroid, &d);
    ksl_cross_vvf(&update.rotation, &d, &x);
    ksl_axpy_vvf(1.f, &update.translation, &x);
    ksl_axpy_vvf(1.f, &x0, &x);
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
  return largest;
}

void shift::gui::FlexCadField(const SceneCad& cad, const SceneMesh& mesh,
                              const FlexUpdate& update,
                              std::vector<float>& field) {
  const size_t samples = cad.sample_points.size() / 4;
  const size_t columns = mesh.flex_columns;
  field.assign(4 * samples, 0.f);
  if(update.coefficients.size() != columns ||
     cad.sample_basis.size() != 3 * columns * samples) {
    return;
  }
  for(size_t k = 0; k < samples; k++) {
    const float* x = cad.sample_points.data() + 4 * k;
    const ksl_vec3f_t d =
      ksl_vec3f(x[0] - mesh.flex_centroid.x, x[1] - mesh.flex_centroid.y,
                x[2] - mesh.flex_centroid.z);
    ksl_vec3f_t u;
    ksl_cross_vvf(&update.rotation, &d, &u);
    ksl_axpy_vvf(1.f, &update.translation, &u);
    const float* b = cad.sample_basis.data() + 3 * columns * k;
    for(size_t j = 0; j < columns; j++) {
      for(int c = 0; c < 3; c++) {
        u.at[c] += update.coefficients[j] * b[3 * j + c];
      }
    }
    std::copy(u.at, u.at + 3, field.data() + 4 * k);
  }
}

/* Catmull-Rom weights of the points before, at, after and two after t in
[0, 1], and their derivatives along t */
static void CatmullRom(float t, float w[4], float dw[4]) {
  const float t2 = t * t, t3 = t2 * t;
  w[0] = 0.5f * (-t + 2.f * t2 - t3);
  w[1] = 0.5f * (2.f - 5.f * t2 + 3.f * t3);
  w[2] = 0.5f * (t + 4.f * t2 - 3.f * t3);
  w[3] = 0.5f * (-t2 + t3);
  dw[0] = 0.5f * (-1.f + 4.f * t - 3.f * t2);
  dw[1] = 0.5f * (-10.f * t + 9.f * t2);
  dw[2] = 0.5f * (1.f + 8.f * t - 9.f * t2);
  dw[3] = 0.5f * (-2.f * t + 3.f * t2);
}

ksl_vec3f_t shift::gui::FlexCadInterpolate(const SceneCad& cad,
                                           const std::vector<float>& values,
                                           int patch, float u, float v,
                                           ksl_vec3f_t* along_u,
                                           ksl_vec3f_t* along_v) {
  const CadGpuSampleGrid& g = cad.sample_grids[size_t(patch)];
  const int n[2] = {g.grid[0], g.grid[1]};
  float s[2] = {(u - g.uv[0]) / (g.uv[2] - g.uv[0]) * float(n[0]),
                (v - g.uv[1]) / (g.uv[3] - g.uv[1]) * float(n[1])};
  int i[2];
  float w[2][4], dw[2][4];
  for(int a = 0; a < 2; a++) {
    s[a] = std::clamp(s[a], 0.f, float(n[a]));
    i[a] = std::min(int(std::floor(s[a])), n[a] - 1);
    CatmullRom(s[a] - float(i[a]), w[a], dw[a]);
  }
  // a sample, the grid extended linearly beyond its ends
  auto at = [&](int a, int b, int c) {
    auto value = [&](int p, int q) {
      return values[4 * (size_t(g.grid[2]) + size_t(q) * size_t(n[0] + 1) +
                         size_t(p)) +
                    size_t(c)];
    };
    auto along_v_line = [&](int p) {
      if(b < 0) {
        return 2.f * value(p, 0) - value(p, 1);
      }
      if(b > n[1]) {
        return 2.f * value(p, n[1]) - value(p, n[1] - 1);
      }
      return value(p, b);
    };
    if(a < 0) {
      return 2.f * along_v_line(0) - along_v_line(1);
    }
    if(a > n[0]) {
      return 2.f * along_v_line(n[0]) - along_v_line(n[0] - 1);
    }
    return along_v_line(a);
  };
  ksl_vec3f_t x = ksl_vec3f(0.f, 0.f, 0.f), xu = x, xv = x;
  for(int q = 0; q < 4; q++) {
    for(int p = 0; p < 4; p++) {
      for(int c = 0; c < 3; c++) {
        const float value = at(i[0] - 1 + p, i[1] - 1 + q, c);
        x.at[c] += w[0][p] * w[1][q] * value;
        xu.at[c] += dw[0][p] * w[1][q] * value;
        xv.at[c] += w[0][p] * dw[1][q] * value;
      }
    }
  }
  // per grid interval to per parameter
  ksl_vec3f_scale(&xu, float(n[0]) / (g.uv[2] - g.uv[0]));
  ksl_vec3f_scale(&xv, float(n[1]) / (g.uv[3] - g.uv[1]));
  if(along_u) {
    *along_u = xu;
  }
  if(along_v) {
    *along_v = xv;
  }
  return x;
}
