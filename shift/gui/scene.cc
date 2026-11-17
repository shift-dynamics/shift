#include "shift/gui/scene.h"

#include <QtDebug>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <string_view>

#include "shift/core/geometry/simplify.h"

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
    if(mesh.terrain >= 0 || instance.is_placed()) {
      continue; // frame the model, not the ground and scenery about it
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

QImage shift::gui::TextureLoader::ReadImage(const std::string& path) {
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
  }
  return image;
}

QImage shift::gui::TextureLoader::Fit(QImage image, int size) {
  if(image.width() != size || image.height() != size) {
    image = image.scaled(size, size, Qt::IgnoreAspectRatio,
                         Qt::SmoothTransformation);
  }
  return image.convertToFormat(QImage::Format_RGBA8888);
}

QImage shift::gui::TextureLoader::Read(const std::string& path) {
  QImage image = ReadImage(path);
  return image.isNull() ? image : Fit(std::move(image), Scene::kTextureSize);
}

int shift::gui::TextureLoader::Load(const std::string& path) {
  if(path.empty()) {
    return -1;
  }
  auto it = layers_.find(path);
  if(it != layers_.end()) {
    return it->second;
  }
  // an embedded image is loaded once for all the mesh files that hold the
  // same data
  std::string content;
  auto embedded = embedded_.find(path);
  if(embedded != embedded_.end()) {
    const auto& data = embedded->second->data;
    content = "data:" + std::to_string(data.size()) + ":" +
              std::to_string(embedded->second->width) + ":" +
              std::to_string(std::hash<std::string_view>()(std::string_view(
                reinterpret_cast<const char*>(data.data()), data.size())));
    auto same = layers_.find(content);
    if(same != layers_.end()) {
      layers_[path] = same->second;
      return same->second;
    }
  }
  QImage image = ReadImage(path);
  int layer = -1;
  if(!image.isNull()) {
    if(std::max(image.width(), image.height()) > Scene::kTextureSize) {
      layer = Scene::kLargeLayer +
              static_cast<int>(scene_.large_texture_layers.size());
      scene_.large_texture_layers.push_back(
        Fit(std::move(image), Scene::kLargeTextureSize));
    } else {
      layer = static_cast<int>(scene_.texture_layers.size());
      scene_.texture_layers.push_back(
        Fit(std::move(image), Scene::kTextureSize));
    }
  }
  layers_[path] = layer;
  if(!content.empty()) {
    layers_[content] = layer;
  }
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

shift::gui::SceneMeshBvh shift::gui::BuildSceneMeshBvh(const SceneMesh& mesh,
                                                       bool tagged) {
  SceneMeshBvh bvh;
  auto& triangles = bvh.triangles;
  triangles.resize(mesh.indices.size() / 3);
  for(size_t i = 0; i < triangles.size(); i++) {
    float* positions[3] = {triangles[i].v0, triangles[i].v1, triangles[i].v2};
    float* normals[3] = {triangles[i].n0, triangles[i].n1, triangles[i].n2};
    for(int j = 0; j < 3; j++) {
      const Vertex& v = mesh.vertices[mesh.indices[3 * i + j]];
      std::copy(v.position, v.position + 3, positions[j]);
      std::copy(v.normal, v.normal + 3, normals[j]);
      // texture coordinates in the spare components
      positions[j][3] = v.uv[0];
      normals[j][3] = v.uv[1];
    }
  }
  bvh.faces.assign(triangles.size(), -1);
  if(tagged) {
    // the BVH reorders the tags with the triangles: where each went
    for(size_t i = 0; i < bvh.faces.size(); i++) {
      bvh.faces[i] = static_cast<int32_t>(i);
    }
  }
  bvh.nodes = BuildBvh(triangles, &bvh.faces);
  return bvh;
}

/* a level of detail of mesh from faces into its vertices: the vertices
those use, at positions (x, y, z each) when not null, else at their own */
static shift::gui::SceneMeshLod
SceneMeshLodOf(const shift::gui::SceneMesh& mesh, std::vector<uint32_t> faces,
               const std::vector<float>* positions, float max_pixels) {
  shift::gui::SceneMeshLod lod;
  const std::vector<uint32_t> old_index =
    shift::core::geometry::CompactVertices(faces, mesh.vertices.size());
  lod.vertices.reserve(old_index.size());
  for(uint32_t v : old_index) {
    shift::gui::Vertex vertex = mesh.vertices[v];
    if(positions) {
      for(int k = 0; k < 3; k++) {
        vertex.position[k] = (*positions)[3 * v + k];
      }
    }
    lod.vertices.push_back(vertex);
  }
  lod.indices = std::move(faces);
  lod.max_pixels = max_pixels;
  return lod;
}

void shift::gui::BuildSceneMeshLods(SceneMesh& mesh) {
  // meshes of fewer triangles gain nothing; each level has a quarter of
  // the triangles of the one before; levels below kFewest triangles are not
  // made
  constexpr size_t kLeast = 2000;
  constexpr size_t kFewest = 128;
  constexpr int kLevels = 3;
  // a level is drawn where its error (relative to the size of the mesh)
  // spans fewer pixels than this, the mesh spanning max_pixels
  constexpr double kPixelError = SceneMeshLod::kPixelError;
  // a level that stops above this share of its target is not used
  constexpr double kShortfall = 1.5;
  // the weights of the normal and of the texture coordinates in the error
  constexpr float kNormalWeight = 0.1f;
  constexpr float kUvWeight = 0.1f;
  constexpr double kMaxError = 0.5;
  // the parts of foliage kept are the same at every level, a share of
  // those of the level before
  constexpr uint32_t kSeed = 1;

  mesh.lods.clear();
  const size_t face_count = mesh.indices.size() / 3;
  if(face_count < kLeast || mesh.terrain >= 0) {
    return;
  }
  std::vector<float> positions;
  std::vector<float> attributes;
  positions.reserve(3 * mesh.vertices.size());
  attributes.reserve(5 * mesh.vertices.size());
  for(const Vertex& v : mesh.vertices) {
    positions.insert(positions.end(), v.position, v.position + 3);
    attributes.insert(attributes.end(), v.normal, v.normal + 3);
    attributes.insert(attributes.end(), v.uv, v.uv + 2);
  }
  const std::vector<uint32_t> faces(mesh.indices.begin(), mesh.indices.end());
  shift::core::geometry::SimplifyOptions options;
  options.max_error = kMaxError;
  options.attribute_weights = {kNormalWeight, kNormalWeight, kNormalWeight,
                               kUvWeight, kUvWeight};
  // the switch of each level: no finer than the level before
  double max_pixels = std::numeric_limits<double>::max();
  double most_error = 0.0;
  auto add = [&](std::vector<uint32_t> level_faces,
                 const std::vector<float>* level_positions, double error) {
    most_error = std::max(most_error, error);
    max_pixels =
      std::min(max_pixels, kPixelError / std::max(most_error, 1e-6));
    mesh.lods.push_back(SceneMeshLodOf(mesh, std::move(level_faces),
                                       level_positions, float(max_pixels)));
    mesh.lods.back().error = float(most_error);
  };
  for(int level = 1; level <= kLevels; level++) {
    const double share = std::pow(0.25, level);
    const size_t target = size_t(share * double(face_count));
    if(target < kFewest) {
      break;
    }
    auto short_of = [&](const std::vector<uint32_t>& f) {
      return double(f.size() / 3) > kShortfall * double(target);
    };
    double error = 0.0;
    if(mesh.alpha_cutoff > 0.f) {
      std::vector<float> scaled;
      std::vector<uint32_t> kept = shift::core::geometry::PruneComponents(
        positions, faces, share, kSeed, &scaled, &error);
      if(!short_of(kept)) {
        add(std::move(kept), &scaled, error);
        continue;
      }
    }
    options.target_face_count = target;
    std::vector<uint32_t> simplified = shift::core::geometry::SimplifyQuadric(
      positions, attributes, 5, faces, options, &error);
    if(short_of(simplified)) {
      simplified =
        shift::core::geometry::SimplifyGrid(positions, faces, target, &error);
    }
    if(simplified.empty() || short_of(simplified)) {
      break;
    }
    add(std::move(simplified), nullptr, error);
  }
}

/* the texel of image (RGBA8888) at the texture coordinates u, v, repeated
as the samplers repeat them */
static const uchar* SceneImpostorTexel(const QImage& image, float u, float v) {
  const int x = std::clamp(int((u - std::floor(u)) * float(image.width())), 0,
                           image.width() - 1);
  const int y = std::clamp(int((v - std::floor(v)) * float(image.height())),
                           0, image.height() - 1);
  return image.constScanLine(y) + 4 * x;
}

bool shift::gui::BakeSceneMeshImpostor(const Scene& scene,
                                       const SceneMesh& mesh,
                                       SceneMeshImpostor& impostor) {
  if(mesh.alpha_cutoff <= 0.f || mesh.base_color_layer < 0 ||
     mesh.lods.empty() || mesh.indices.empty() || mesh.triplanar) {
    return false;
  }
  const QImage* source = nullptr;
  if(mesh.base_color_layer >= Scene::kLargeLayer) {
    const size_t k = size_t(mesh.base_color_layer - Scene::kLargeLayer);
    if(k < scene.large_texture_layers.size()) {
      source = &scene.large_texture_layers[k];
    }
  } else if(size_t(mesh.base_color_layer) < scene.texture_layers.size()) {
    source = &scene.texture_layers[size_t(mesh.base_color_layer)];
  }
  if(!source || source->isNull() ||
     source->format() != QImage::Format_RGBA8888) {
    return false;
  }
  const float lo[3] = {mesh.bounds_min.x, mesh.bounds_min.y,
                       mesh.bounds_min.z};
  const float hi[3] = {mesh.bounds_max.x, mesh.bounds_max.y,
                       mesh.bounds_max.z};
  float sides[3];
  for(int k = 0; k < 3; k++) {
    sides[k] = std::max(hi[k] - lo[k], 0.f);
  }
  float sorted[3] = {sides[0], sides[1], sides[2]};
  std::sort(sorted, sorted + 3);
  if(sorted[2] <= 0.f) {
    return false;
  }
  // each view in a quarter of the texture, with transparent texels about
  // it so that the coarse mipmaps of one do not reach the next; a texel is
  // covered where any of kSub x kSub samples in it meets the mesh, so that
  // twigs thinner than a texel do not vanish
  constexpr int kSize = Scene::kTextureSize;
  constexpr int kCell = kSize / 2;
  constexpr int kMargin = 8;
  constexpr int kInside = kCell - 2 * kMargin;
  constexpr int kSub = 2;
  constexpr int kSamples = kInside * kSub;
  const int cutoff = int(std::ceil(mesh.alpha_cutoff * 255.f));
  impostor.color = QImage(kSize, kSize, QImage::Format_RGBA8888);
  impostor.color.fill(Qt::transparent);
  impostor.normal = QImage(kSize, kSize, QImage::Format_RGBA8888);
  impostor.normal.fill(QColor(128, 128, 255, 255));
  // texels written, which the filling below spreads from
  std::vector<uint8_t> filled(size_t(kSize) * kSize, 0);
  impostor.lod = SceneMeshLod();
  std::vector<float> depth(size_t(kSamples) * kSamples);
  std::vector<int32_t> nearest(size_t(kSamples) * kSamples);
  std::vector<float> weights(2 * size_t(kSamples) * kSamples);
  const size_t triangle_count = mesh.indices.size() / 3;
  for(int a = 0; a < 3; a++) {
    // seen along a from +a: b across (u), c down (v)
    const int b = (a + 1) % 3, c = (a + 2) % 3;
    const int x0 = (a % 2) * kCell + kMargin, y0 = (a / 2) * kCell + kMargin;
    const float span = std::max(sides[b], sides[c]);
    if(span <= 0.f) {
      continue;
    }
    // the same texels per length along u and v, as the tangent frames of
    // the shaders take them
    const float per_length = float(kInside) / span;
    const float sample_per_length = per_length * float(kSub);
    std::fill(depth.begin(), depth.end(),
              -std::numeric_limits<float>::infinity());
    std::fill(nearest.begin(), nearest.end(), -1);
    for(size_t t = 0; t < triangle_count; t++) {
      const Vertex* v[3];
      float x[3], y[3];
      for(int j = 0; j < 3; j++) {
        v[j] = &mesh.vertices[mesh.indices[3 * t + size_t(j)]];
        x[j] = (v[j]->position[b] - lo[b]) * sample_per_length;
        y[j] = (v[j]->position[c] - lo[c]) * sample_per_length;
      }
      const float area =
        (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
      if(std::fabs(area) < 1e-12f) {
        continue;
      }
      const int i_lo = std::max(
        int(std::floor(std::min({x[0], x[1], x[2]}) - 0.5f)), 0);
      const int i_hi = std::min(
        int(std::ceil(std::max({x[0], x[1], x[2]}) - 0.5f)), kSamples - 1);
      const int j_lo = std::max(
        int(std::floor(std::min({y[0], y[1], y[2]}) - 0.5f)), 0);
      const int j_hi = std::min(
        int(std::ceil(std::max({y[0], y[1], y[2]}) - 0.5f)), kSamples - 1);
      for(int j = j_lo; j <= j_hi; j++) {
        const float py = float(j) + 0.5f;
        for(int i = i_lo; i <= i_hi; i++) {
          const float px = float(i) + 0.5f;
          const float w1 =
            ((px - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (py - y[0])) / area;
          const float w2 =
            ((x[1] - x[0]) * (py - y[0]) - (px - x[0]) * (y[1] - y[0])) / area;
          const float w0 = 1.f - w1 - w2;
          if(w0 < 0.f || w1 < 0.f || w2 < 0.f) {
            continue;
          }
          const size_t s = size_t(j) * kSamples + size_t(i);
          const float z = w0 * v[0]->position[a] + w1 * v[1]->position[a] +
                          w2 * v[2]->position[a];
          if(z <= depth[s]) {
            continue;
          }
          const float u = w0 * v[0]->uv[0] + w1 * v[1]->uv[0] +
                          w2 * v[2]->uv[0];
          const float w = w0 * v[0]->uv[1] + w1 * v[1]->uv[1] +
                          w2 * v[2]->uv[1];
          if(SceneImpostorTexel(*source, u, w)[3] < cutoff) {
            continue; // cut out
          }
          depth[s] = z;
          nearest[s] = int32_t(t);
          weights[2 * s] = w1;
          weights[2 * s + 1] = w2;
        }
      }
    }
    // each texel: the mean of the samples that met the mesh
    for(int j = 0; j < kInside; j++) {
      uchar* color_row = impostor.color.scanLine(y0 + j);
      uchar* normal_row = impostor.normal.scanLine(y0 + j);
      for(int i = 0; i < kInside; i++) {
        float rgb[3] = {0.f, 0.f, 0.f};
        float n[3] = {0.f, 0.f, 0.f};
        int count = 0;
        for(int sj = 0; sj < kSub; sj++) {
          for(int si = 0; si < kSub; si++) {
            const size_t s =
              size_t(j * kSub + sj) * kSamples + size_t(i * kSub + si);
            if(nearest[s] < 0) {
              continue;
            }
            const size_t t = size_t(nearest[s]);
            const Vertex* v[3];
            for(int k = 0; k < 3; k++) {
              v[k] = &mesh.vertices[mesh.indices[3 * t + size_t(k)]];
            }
            const float w1 = weights[2 * s], w2 = weights[2 * s + 1];
            const float w0 = 1.f - w1 - w2;
            const uchar* texel = SceneImpostorTexel(
              *source,
              w0 * v[0]->uv[0] + w1 * v[1]->uv[0] + w2 * v[2]->uv[0],
              w0 * v[0]->uv[1] + w1 * v[1]->uv[1] + w2 * v[2]->uv[1]);
            for(int k = 0; k < 3; k++) {
              rgb[k] += float(texel[k]);
            }
            // two sided: the normal of the side seen, from +a
            float m[3];
            for(int k = 0; k < 3; k++) {
              m[k] = w0 * v[0]->normal[k] + w1 * v[1]->normal[k] +
                     w2 * v[2]->normal[k];
            }
            const float flip = m[a] < 0.f ? -1.f : 1.f;
            const float length =
              std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
            if(length > 0.f) {
              for(int k = 0; k < 3; k++) {
                n[k] += flip * m[k] / length;
              }
            }
            count++;
          }
        }
        if(count == 0) {
          continue;
        }
        uchar* color = color_row + 4 * (x0 + i);
        for(int k = 0; k < 3; k++) {
          color[k] = uchar(std::lround(rgb[k] / float(count)));
        }
        color[3] = 255;
        // in the tangent frame of the card: u along b, v along c, its
        // normal along a
        const float tangent[3] = {n[b], n[c], n[a]};
        float length = std::sqrt(tangent[0] * tangent[0] +
                                 tangent[1] * tangent[1] +
                                 tangent[2] * tangent[2]);
        uchar* normal = normal_row + 4 * (x0 + i);
        for(int k = 0; k < 3; k++) {
          const float value = length > 0.f ? tangent[k] / length
                                           : (k == 2 ? 1.f : 0.f);
          normal[k] = uchar(std::clamp(
            std::lround((value * 0.5f + 0.5f) * 255.f), 0L, 255L));
        }
        filled[size_t(y0 + j) * kSize + size_t(x0 + i)] = 1;
      }
    }
    // the card through the center of the box, across a
    const float middle = 0.5f * (lo[a] + hi[a]);
    const uint32_t first = uint32_t(impostor.lod.vertices.size());
    for(int corner = 0; corner < 4; corner++) {
      const int ub = corner & 1, vc = corner >> 1;
      Vertex vertex{};
      vertex.position[a] = middle;
      vertex.position[b] = ub ? hi[b] : lo[b];
      vertex.position[c] = vc ? hi[c] : lo[c];
      vertex.normal[a] = 1.f;
      vertex.uv[0] = (float(x0) + float(ub) * sides[b] * per_length) /
                     float(kSize);
      vertex.uv[1] = (float(y0) + float(vc) * sides[c] * per_length) /
                     float(kSize);
      impostor.lod.vertices.push_back(vertex);
    }
    for(uint32_t k : {0u, 1u, 3u, 0u, 3u, 2u}) {
      impostor.lod.indices.push_back(first + k);
    }
  }
  if(impostor.lod.indices.empty()) {
    return false;
  }
  // the colors and normals of the empty texels from their neighbors, so
  // that the mipmaps do not darken the edges of the leaves
  constexpr int kFillPasses = 8;
  for(int pass = 0; pass < kFillPasses; pass++) {
    std::vector<uint8_t> next = filled;
    for(int y = 0; y < kSize; y++) {
      for(int x = 0; x < kSize; x++) {
        if(filled[size_t(y) * kSize + size_t(x)]) {
          continue;
        }
        int sum[2][3] = {{0, 0, 0}, {0, 0, 0}};
        int count = 0;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for(int k = 0; k < 4; k++) {
          const int nx = x + dx[k], ny = y + dy[k];
          if(nx < 0 || ny < 0 || nx >= kSize || ny >= kSize ||
             !filled[size_t(ny) * kSize + size_t(nx)]) {
            continue;
          }
          const uchar* color = impostor.color.constScanLine(ny) + 4 * nx;
          const uchar* normal = impostor.normal.constScanLine(ny) + 4 * nx;
          for(int c = 0; c < 3; c++) {
            sum[0][c] += color[c];
            sum[1][c] += normal[c];
          }
          count++;
        }
        if(count == 0) {
          continue;
        }
        uchar* color = impostor.color.scanLine(y) + 4 * x;
        uchar* normal = impostor.normal.scanLine(y) + 4 * x;
        for(int c = 0; c < 3; c++) {
          color[c] = uchar(sum[0][c] / count);
          normal[c] = uchar(sum[1][c] / count);
        }
        next[size_t(y) * kSize + size_t(x)] = 1;
      }
    }
    filled = std::move(next);
  }
  // a card is as far as half the depth of the mesh from what it shows: the
  // middle side of the box, for a tree the width of its crown
  impostor.error = 0.5 * double(sorted[1]) / double(sorted[2]);
  impostor.lod.error = std::max(mesh.lods.back().error, float(impostor.error));
  impostor.lod.max_pixels =
    std::min(mesh.lods.back().max_pixels,
             SceneMeshLod::kPixelError / std::max(impostor.lod.error, 1e-6f));
  return true;
}

void shift::gui::AddSceneMeshImpostor(Scene& scene, int mesh,
                                      SceneMeshImpostor impostor) {
  const SceneMesh& source = scene.meshes[size_t(mesh)];
  // the source's material, with the baked textures, and the cards as its
  // own geometry
  SceneMesh look;
  look.vertices = impostor.lod.vertices;
  look.indices = impostor.lod.indices;
  look.albedo = source.albedo;
  look.roughness = source.roughness;
  look.metallic = source.metallic;
  look.opacity = source.opacity;
  look.alpha_cutoff = source.alpha_cutoff;
  look.glow_light = source.glow_light;
  look.draw_distance = source.draw_distance;
  look.bounds_min = source.bounds_min;
  look.bounds_max = source.bounds_max;
  look.base_color_layer = int(scene.texture_layers.size());
  scene.texture_layers.push_back(std::move(impostor.color));
  look.normal_layer = int(scene.texture_layers.size());
  scene.texture_layers.push_back(std::move(impostor.normal));
  impostor.lod.material = int(scene.meshes.size());
  scene.meshes[size_t(mesh)].lods.push_back(std::move(impostor.lod));
  scene.meshes.push_back(std::move(look));
}
