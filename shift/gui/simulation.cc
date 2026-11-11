#include "shift/gui/simulation.h"

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/body/base.h"
#include "shift/core/geometry/cad.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/integrator/base.h"
#include "shift/core/light/ies.h"
#include "shift/core/light/spot.h"
#include "shift/core/terrain/basic.h"
#include "shift/core/terrain/scm.h"

#include <H5Cpp.h>
#include <QGenericMatrix>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <map>
#include <tuple>
#include <set>


static double WallTime() {
  return std::chrono::duration<double>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

static QMatrix4x4 ToMatrix(const ksl_SE3_t& d) {
  // ksl_SE3_t is column major: as_array[3 * column + row]
  const double* a = d.as_array;
  return QMatrix4x4(float(a[0]), float(a[3]), float(a[6]), float(a[9]),
                    float(a[1]), float(a[4]), float(a[7]), float(a[10]),
                    float(a[2]), float(a[5]), float(a[8]), float(a[11]), 0.f,
                    0.f, 0.f, 1.f);
}

/*! area weighted vertex normals, for meshes imported without normals */
static void ComputeNormals(shift::gui::SceneMesh& mesh) {
  for(auto& vertex : mesh.vertices) {
    std::fill(vertex.normal, vertex.normal + 3, 0.f);
  }
  for(size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    shift::gui::Vertex* v[3] = {&mesh.vertices[mesh.indices[i]],
                                &mesh.vertices[mesh.indices[i + 1]],
                                &mesh.vertices[mesh.indices[i + 2]]};
    const QVector3D p0(v[0]->position[0], v[0]->position[1],
                       v[0]->position[2]);
    const QVector3D p1(v[1]->position[0], v[1]->position[1],
                       v[1]->position[2]);
    const QVector3D p2(v[2]->position[0], v[2]->position[1],
                       v[2]->position[2]);
    const QVector3D n = QVector3D::crossProduct(p1 - p0, p2 - p0);
    for(auto vertex : v) {
      vertex->normal[0] += n.x();
      vertex->normal[1] += n.y();
      vertex->normal[2] += n.z();
    }
  }
  for(auto& vertex : mesh.vertices) {
    QVector3D n(vertex.normal[0], vertex.normal[1], vertex.normal[2]);
    n.normalize();
    vertex.normal[0] = n.x();
    vertex.normal[1] = n.y();
    vertex.normal[2] = n.z();
  }
}

static shift::gui::SceneMesh
MakeSceneMesh(const shift::gui::MeshData& data,
              const std::vector<shift::gui::MaterialData>& materials,
              shift::gui::TextureLoader& textures) {
  shift::gui::SceneMesh mesh;

  const size_t vertex_count = data.vertices.size() / 3;
  const bool has_normals = data.normals.size() == data.vertices.size();
  const bool has_uvs = data.texture_coordinates.size() == data.vertices.size();
  mesh.vertices.resize(vertex_count);
  mesh.bounds_min = QVector3D(1e30f, 1e30f, 1e30f);
  mesh.bounds_max = -mesh.bounds_min;
  for(size_t i = 0; i < vertex_count; i++) {
    for(int k = 0; k < 3; k++) {
      mesh.vertices[i].position[k] = data.vertices[3 * i + k];
      mesh.vertices[i].normal[k] = has_normals ? data.normals[3 * i + k] : 0.f;
      mesh.bounds_min[k] = std::min(mesh.bounds_min[k], data.vertices[3 * i + k]);
      mesh.bounds_max[k] = std::max(mesh.bounds_max[k], data.vertices[3 * i + k]);
    }
    /* assimp gives texture coordinates with v = 0 at the bottom of the image
    (OpenGL); the texture layers are uploaded with the top row of the image at
    v = 0 */
    if(has_uvs) {
      mesh.vertices[i].uv[0] = data.texture_coordinates[3 * i + 0];
      mesh.vertices[i].uv[1] = 1.f - data.texture_coordinates[3 * i + 1];
    } else {
      mesh.vertices[i].uv[0] = mesh.vertices[i].uv[1] = 0.f;
    }
  }
  mesh.indices.assign(data.faces.begin(), data.faces.end());
  if(!has_normals) {
    ComputeNormals(mesh);
  }

  if(data.material_index >= 0 &&
     data.material_index < static_cast<int>(materials.size())) {
    const auto& material = materials[data.material_index];
    mesh.albedo = QVector3D(material.Kd[0], material.Kd[1], material.Kd[2]);
    mesh.opacity = std::clamp(material.opacity, 0.f, 1.f);
    // use the metallic-roughness value if given, otherwise map the
    // Blinn-Phong exponent to an approximately equivalent GGX roughness
    const float roughness = material.roughness >= 0.f
                              ? material.roughness
                              : std::sqrt(2.f / (material.Ns + 2.f));
    mesh.roughness = std::clamp(roughness, 0.05f, 1.f);
    mesh.metallic = std::clamp(material.metalness, 0.f, 1.f);

    mesh.base_color_layer = textures.Load(material.base_color_map);
    mesh.metallic_roughness_layer = textures.LoadMetallicRoughness(
      material.metallic_roughness_map, material.metallic_map,
      material.roughness_map);
    mesh.normal_layer = textures.Load(material.normal_map);
    mesh.normal_scale = material.normal_scale;
    mesh.triplanar = material.triplanar;
    mesh.texture_scale = material.texture_scale;
  }
  return mesh;
}


/* a terrain: its control points for the tessellation shaders, and a
triangle mesh of the surface (terrain::Basic::LocalZ) in world coordinates,
for the ray tracer and devices without tessellation. A deformable terrain
(terrain::Scm) is drawn over its grid only, bilinear between the nodes as
the soil models see it, with a ring of the undisturbed surface one spacing
beyond the grid, where its surface returns to the undisturbed one; its
control points start on the undisturbed surface (Simulation::ScmView) and
its mesh stays there */
static int AddTerrain(const shift::core::terrain::Basic& terrain,
                      shift::gui::TextureLoader& textures,
                      shift::gui::Scene& scene) {
  constexpr int kMaximumMeshSamples = 513; // per side
  constexpr int kMinimumMeshSamples = 257; // per side
  const auto* scm = dynamic_cast<const shift::core::terrain::Scm*>(&terrain);
  double x_extent = terrain.get_x_extent();
  double y_extent = terrain.get_y_extent();
  double x0 = 0.0, y0 = 0.0; // of the control points in the terrain frame
  shift::gui::SceneTerrain t;
  if(scm) {
    const long nx = scm->get_node_count_x(), ny = scm->get_node_count_y();
    x0 = scm->NodeX(-1);
    y0 = scm->NodeY(-1);
    x_extent = scm->get_spacing() * double(nx + 1);
    y_extent = scm->get_spacing() * double(ny + 1);
    t.width = static_cast<int>(nx + 2);
    t.height = static_cast<int>(ny + 2);
    t.z.reserve(size_t(t.width) * size_t(t.height));
    for(long j = -1; j <= ny; j++) {
      for(long i = -1; i <= nx; i++) {
        t.z.push_back(float(scm->UndisturbedZ(i, j)));
      }
    }
    t.x_scale = t.y_scale = float(scm->get_spacing());
    t.bicubic = false;
    t.deformable = true;
  } else {
    t.width = static_cast<int>(terrain.get_x_samples());
    t.height = static_cast<int>(terrain.get_y_samples());
    t.z.assign(terrain.get_control_points().begin(),
               terrain.get_control_points().end());
    t.x_scale = float(terrain.get_x_scale());
    t.y_scale = float(terrain.get_y_scale());
    t.bicubic = terrain.is_bicubic();
  }
  if(!(x_extent > 0.0) || !(y_extent > 0.0)) {
    return -1;
  }
  const auto& d = terrain.get_pose();
  QMatrix4x4 pose;
  for(int r = 0; r < 3; r++) {
    pose(r, 0) = float(d.R.v0.at[r]);
    pose(r, 1) = float(d.R.v1.at[r]);
    pose(r, 2) = float(d.R.v2.at[r]);
    pose(r, 3) = float(d.t.at[r]);
  }
  pose.translate(float(x0), float(y0), 0.f);
  t.pose = pose;

  /* the mesh, in world coordinates */
  shift::gui::SceneMesh mesh;
  mesh.albedo = QVector3D(0.42f, 0.38f, 0.33f); // dry earth
  mesh.roughness = 0.95f;
  mesh.base_color_layer = textures.Load(terrain.get_texture_path());
  if(mesh.base_color_layer >= 0) {
    mesh.albedo = QVector3D(1.f, 1.f, 1.f);
  }
  mesh.normal_layer = textures.Load(terrain.get_normal_map_path());
  mesh.normal_scale = float(terrain.get_normal_scale());
  mesh.triplanar = true; // along the world axes: the mesh is on the fixed body
  mesh.texture_scale = float(terrain.get_texture_scale());
  /* twice the resolution of the control points, and at least
  kMinimumMeshSamples a side so that the curvature of a small map is drawn
  smoothly, at most kMaximumMeshSamples a side */
  int mesh_x = std::clamp(
    std::max(2 * (t.width - 1) + 1, kMinimumMeshSamples), 2,
    kMaximumMeshSamples);
  int mesh_y = std::clamp(
    std::max(2 * (t.height - 1) + 1, kMinimumMeshSamples), 2,
    kMaximumMeshSamples);
  if(scm) {
    /* a vertex at each control point and between them, s per cell along
    each side, which the ray tracer moves with the levels of the nodes
    (Viewport::ShapeRtTerrain): two triangles per square of the mesh, those
    of square (u, v) at 2 (v s (width - 1) + u). At most 2^21 triangles. */
    constexpr double kMaximumTriangles = 2097152.0;
    const double cells = double(t.width - 1) * double(t.height - 1);
    int s = 4;
    while(s > 1 && 2.0 * cells * s * s > kMaximumTriangles) {
      s--;
    }
    t.mesh_subdivisions = s;
    mesh_x = s * (t.width - 1) + 1;
    mesh_y = s * (t.height - 1) + 1;
  }
  const QMatrix3x3 rotation = pose.normalMatrix();
  mesh.bounds_min = QVector3D(1e30f, 1e30f, 1e30f);
  mesh.bounds_max = -mesh.bounds_min;
  for(int j = 0; j < mesh_y; j++) {
    const double y = y_extent * j / (mesh_y - 1);
    for(int i = 0; i < mesh_x; i++) {
      const double x = x_extent * i / (mesh_x - 1);
      double z_x, z_y;
      const double z = terrain.LocalZ(x0 + x, y0 + y, z_x, z_y);
      /* outward, up: (z_x, z_y, -1) normalized */
      const double length = std::sqrt(z_x * z_x + z_y * z_y + 1.0);
      const float n[3] = {float(z_x / length), float(z_y / length),
                          float(-1.0 / length)};
      const QVector3D world = pose.map(QVector3D(float(x), float(y), float(z)));
      shift::gui::Vertex v;
      for(int a = 0; a < 3; a++) {
        v.position[a] = world[a];
        v.normal[a] = rotation(a, 0) * n[0] + rotation(a, 1) * n[1] +
                      rotation(a, 2) * n[2];
        mesh.bounds_min[a] = std::min(mesh.bounds_min[a], world[a]);
        mesh.bounds_max[a] = std::max(mesh.bounds_max[a], world[a]);
      }
      v.uv[0] = float(x0 + x);
      v.uv[1] = float(y0 + y);
      mesh.vertices.push_back(v);
    }
  }
  const uint32_t nx = uint32_t(mesh_x);
  for(uint32_t j = 0; j + 1 < uint32_t(mesh_y); j++) {
    for(uint32_t i = 0; i + 1 < nx; i++) {
      const uint32_t a = j * nx + i, b = a + 1, c = a + nx, e = c + 1;
      mesh.indices.insert(mesh.indices.end(), {a, b, e, a, e, c});
    }
  }
  t.mesh_index = static_cast<int>(scene.meshes.size());
  mesh.terrain = static_cast<int>(scene.terrains.size());
  scene.meshes.push_back(std::move(mesh));
  scene.instances.push_back({t.mesh_index, 0});
  scene.terrains.push_back(std::move(t));
  return static_cast<int>(scene.terrains.size()) - 1;
}

/* a light of the model, if its frame is on one of the bodies drawn */
static void AddLight(const shift::core::light::Base& light,
                     const std::vector<shift::core::body::Base*>& bodies,
                     shift::gui::Scene& scene) {
  using shift::gui::Scene;
  using shift::gui::SceneLight;
  if(!light.get_frame()) {
    return; // Connect warned
  }
  const auto body = std::find(bodies.begin(), bodies.end(),
                              light.get_parent_body());
  if(body == bodies.end()) {
    return;
  }
  SceneLight l;
  const std::string& type = light.get_type();
  if(type == "point") {
    l.type = SceneLight::kPoint;
  } else if(type == "spot") {
    l.type = SceneLight::kSpot;
  } else if(type == "directional") {
    l.type = SceneLight::kDirectional;
  } else if(type == "ies") {
    l.type = SceneLight::kIes;
  } else {
    return;
  }
  l.body_index = static_cast<int>(body - bodies.begin());
  l.frame = ToMatrix(*light.get_frame()->get_d_wrt_origin());
  auto color = [](const ksl_vec3_t& c, double factor) {
    return QVector3D(float(c.x * factor), float(c.y * factor),
                     float(c.z * factor));
  };
  l.ambient = color(light.get_Ka(), 1.0);
  l.diffuse = color(light.get_Kd(), light.get_intensity());
  l.specular = color(light.get_Ks(), light.get_intensity());
  l.attenuation = color(light.get_attenuation(), 1.0);
  if(auto spot = dynamic_cast<const shift::core::light::Spot*>(&light)) {
    l.cone_angle = float(spot->get_cone_angle());
    l.inner_cone_angle = float(spot->get_inner_cone_angle());
  }
  if(auto ies = dynamic_cast<const shift::core::light::IES*>(&light)) {
    std::vector<float> profile(size_t(Scene::kIesVertical) *
                               Scene::kIesHorizontal);
    for(int h = 0; h < Scene::kIesHorizontal; h++) {
      const double horizontal = 2.0 * M_PI * h / (Scene::kIesHorizontal - 1);
      for(int v = 0; v < Scene::kIesVertical; v++) {
        const double vertical = M_PI * v / (Scene::kIesVertical - 1);
        profile[size_t(h) * Scene::kIesVertical + v] =
          float(ies->Distribution(vertical, horizontal));
      }
    }
    l.ies_profile = static_cast<int>(scene.ies_profiles.size());
    scene.ies_profiles.push_back(std::move(profile));
  }
  scene.lights.push_back(l);
}

shift::gui::Simulation::~Simulation() {
  JoinWorker();
  context_.reset();
  RemoveLogDirectory();
}

void shift::gui::Simulation::Load(const std::string& filename) {
  JoinWorker();
  scm_views_.clear();
  context_.reset();
  bodies_.clear();
  scene_ = Scene();
  ClearResults();
  RemoveLogDirectory();
  filename_ = filename;
  std::string extension = std::filesystem::path(filename).extension().string();
  for(char& c : extension) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if(extension == ".h5" || extension == ".hdf5") {
    LoadLog(filename);
  } else {
    LoadModel(filename, {});
  }
}

void shift::gui::Simulation::LoadModel(
  const std::string& path, std::map<std::string, std::string> file_map) {
  auto context = std::make_unique<shift::core::Context>();
  context->set_file_map(std::move(file_map));
  context->LoadJSON(path);
  context_ = std::move(context);

  // the context is the fixed (inertial) body; list it first, followed by
  // the remaining active bodies
  bodies_.push_back(context_->get_fixed_body());
  std::set<shift::core::body::Base*> listed(bodies_.begin(), bodies_.end());
  for(auto list : {&context_->get_active_articulated_bodies(),
                   &context_->get_active_free_bodies()}) {
    for(auto body : *list) {
      if(listed.insert(body).second) {
        bodies_.push_back(body);
      }
    }
  }

  /* meshes that share imported data (the same files, see
  geometry::Mesh::get_asset) become one SceneMesh each, drawn once per body */
  TextureLoader textures(scene_);
  std::map<const shift::core::geometry::MeshAsset*, std::vector<int>>
    asset_meshes;
  std::vector<std::vector<int>> mesh_bodies;
  for(size_t i = 0; i < bodies_.size(); i++) {
    for(auto& geometry : bodies_[i]->get_geometry()) {
      auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(!mesh || !mesh->is_active()) {
        continue;
      }
      auto [it, inserted] = asset_meshes.try_emplace(mesh->get_asset());
      if(inserted) {
        textures.AddEmbedded(mesh->get_asset()->embedded_textures);
        for(const auto& data : mesh->get_mesh_data()) {
          if(data.faces.empty()) {
            continue;
          }
          it->second.push_back(static_cast<int>(scene_.meshes.size()));
          scene_.meshes.push_back(
            MakeSceneMesh(data, mesh->get_material_data(), textures));
          mesh_bodies.emplace_back();
        }
      }
      for(int mesh_index : it->second) {
        mesh_bodies[mesh_index].push_back(static_cast<int>(i));
      }
    }
  }
  for(size_t m = 0; m < mesh_bodies.size(); m++) {
    for(int body : mesh_bodies[m]) {
      scene_.instances.push_back({static_cast<int>(m), body});
    }
  }
  /* CAD geometry: one SceneCad per file read (geometry::Cad::get_asset),
  scale, color and opacity, drawn once per body */
  std::map<std::tuple<const shift::core::geometry::CadAsset*, double,
                      std::vector<double>, double>,
           int>
    asset_cads;
  for(size_t i = 0; i < bodies_.size(); i++) {
    for(auto& geometry : bodies_[i]->get_geometry()) {
      auto cad = dynamic_cast<shift::core::geometry::Cad*>(geometry.get());
      if(!cad || !cad->is_active() || !cad->get_asset()) {
        continue;
      }
      const auto key = std::make_tuple(cad->get_asset(), cad->get_scale(),
                                       cad->get_color(), cad->get_opacity());
      auto [it, inserted] = asset_cads.try_emplace(key, 0);
      if(inserted) {
        it->second = static_cast<int>(scene_.cads.size());
        scene_.cads.push_back(MakeSceneCad(cad->get_asset()->model,
                                           cad->get_asset()->bodies,
                                           cad->get_scale()));
        auto& scene_cad = scene_.cads.back();
        const auto& color = cad->get_color();
        if(color.size() == 3) {
          scene_cad.albedo =
            QVector3D(float(color[0]), float(color[1]), float(color[2]));
        }
        scene_cad.opacity = float(cad->get_opacity());
      }
      scene_.cads[it->second].bodies.push_back(static_cast<int>(i));
    }
  }
  /* terrains, on the fixed body (body 0) */
  scm_views_.clear();
  for(auto terrain : context_->get_active_terrain()) {
    if(auto basic = dynamic_cast<shift::core::terrain::Basic*>(terrain)) {
      const int index = AddTerrain(*basic, textures, scene_);
      auto scm = dynamic_cast<shift::core::terrain::Scm*>(basic);
      if(scm && index >= 0) {
        AddScmView(*scm, index);
      }
    }
  }
  /* lights, on the bodies of their frames */
  for(auto light : context_->get_active_lights()) {
    AddLight(*light, bodies_, scene_);
  }
  scene_.body_count = static_cast<int>(bodies_.size());

  state_size_ = context_->get_state_count();
  StoreInitialState();
  playback_dhva_ = context_->get_dhva();
  interpolated_.assign(state_size_, 0.0);

  context_->set_state_observer(
    [this](const double time, const std::vector<double>& y) {
      Observe(time, y);
    });

  ShowContextState();
}

void shift::gui::Simulation::StoreInitialState() {
  // the integrators copy q, dq and the free body states as one contiguous
  // block starting at q
  const double* q = context_->get_q().data();
  initial_state_.assign(q, q + state_size_);
}

// ------------------------------------------------------------------ run

void shift::gui::Simulation::Run() {
  if(!context_ || is_computing()) {
    return;
  }
  JoinWorker();

  // start from the configuration currently shown
  if(has_results()) {
    std::memcpy(context_->get_q().data(), interpolated_.data(),
                state_size_ * sizeof(double));
    context_->UpdateDHVA();
    RestartTerrains(false);
  }
  ClearResults();

  std::lock_guard<std::mutex> lock(mutex_);
  computing_ = true;
  stop_requested_ = false;
  error_.clear();
  wall_start_ = WallTime();
  real_time_factor_ = 0.0;
  worker_ = std::thread(&Simulation::Work, this);
}

void shift::gui::Simulation::Work() {
  std::string error;
  try {
    context_->Simulate();
  } catch(StopRequested&) {
  } catch(std::exception& e) {
    error = e.what();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  computing_ = false;
  error_ = error;
}

void shift::gui::Simulation::Observe(double time,
                                     const std::vector<double>& y) {
  // runs on the worker thread from Context::WriteStates at each output step
  std::lock_guard<std::mutex> lock(mutex_);
  if(stop_requested_) {
    throw StopRequested();
  }
  if(times_.empty() || time > times_.back()) {
    times_.push_back(time);
    states_.insert(states_.end(), y.begin(), y.begin() + state_size_);
    knot_deltas_.push_back(deltas_.size());
  }
  // the levels changed, with the knot point just recorded (or the last one,
  // at a repeated time)
  for(size_t v = 0; v < scm_views_.size(); v++) {
    ScmView& view = scm_views_[v];
    for(const auto& change : view.scm->get_output_changes()) {
      float& level = view.worker[size_t(change.index)];
      if(level != change.level) {
        deltas_.push_back({int32_t(v), change.index, level, change.level});
        level = change.level;
      }
    }
  }
  const double wall = WallTime() - wall_start_;
  real_time_factor_ = wall > 0.0 ? time / wall : 0.0;
}

void shift::gui::Simulation::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  JoinWorker();
}

void shift::gui::Simulation::JoinWorker() {
  if(worker_.joinable()) {
    worker_.join();
  }
  stop_requested_ = false;
}

void shift::gui::Simulation::Reset() {
  if(!context_) {
    return;
  }
  Stop();
  RestartTerrains(true);
  ClearResults();
  std::memcpy(context_->get_q().data(), initial_state_.data(),
              state_size_ * sizeof(double));
  context_->UpdateDHVA();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_.clear();
  }
  ShowContextState();
}

void shift::gui::Simulation::ClearResults() {
  std::lock_guard<std::mutex> lock(mutex_);
  times_.clear();
  states_.clear();
  deltas_.clear();
  knot_deltas_.clear();
  base_delta_ = partial_begin_ = partial_end_ = 0;
  for(auto& view : scm_views_) {
    view.base = view.worker = view.shown;
  }
  real_time_factor_ = 0.0;
  shown_time_ = 0.0;
}

bool shift::gui::Simulation::is_computing() {
  std::lock_guard<std::mutex> lock(mutex_);
  return computing_;
}

bool shift::gui::Simulation::has_results() {
  std::lock_guard<std::mutex> lock(mutex_);
  return !times_.empty();
}

double shift::gui::Simulation::get_computed_time() {
  std::lock_guard<std::mutex> lock(mutex_);
  return times_.empty() ? 0.0 : times_.back();
}

double shift::gui::Simulation::get_end_time() const {
  if(!context_ || context_->get_active_integrators().empty()) {
    return 0.0;
  }
  return context_->get_active_integrators()[0]->get_time_span();
}

double shift::gui::Simulation::get_time_step() const {
  if(!context_ || context_->get_active_integrators().empty()) {
    return 0.0;
  }
  return context_->get_active_integrators()[0]->get_time_step();
}

void shift::gui::Simulation::set_end_time(double time_span) {
  if(context_ && !is_computing() &&
     !context_->get_active_integrators().empty()) {
    context_->get_active_integrators()[0]->set_time_span(time_span);
  }
}

void shift::gui::Simulation::set_time_step(double time_step) {
  if(context_ && !is_computing() &&
     !context_->get_active_integrators().empty()) {
    context_->get_active_integrators()[0]->set_time_step(time_step);
  }
}

std::string shift::gui::Simulation::get_integrator_type() const {
  if(!context_ || context_->get_active_integrators().empty()) {
    return "";
  }
  return context_->get_active_integrators()[0]->get_type();
}

std::string shift::gui::Simulation::get_error() {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

double shift::gui::Simulation::get_real_time_factor() {
  std::lock_guard<std::mutex> lock(mutex_);
  return real_time_factor_;
}

// ------------------------------------------------------------- playback

void shift::gui::Simulation::ShowTime(double t) {
  if(!context_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(times_.empty()) {
      return;
    }
    // knot points t0 <= t <= t1, y = y0 + (y1 - y0) / (t1 - t0) * (t - t0)
    t = std::clamp(t, times_.front(), times_.back());
    const size_t i1 = std::min<size_t>(
      std::lower_bound(times_.begin(), times_.end(), t) - times_.begin(),
      times_.size() - 1);
    const size_t i0 = i1 > 0 ? i1 - 1 : 0;
    const double span = times_[i1] - times_[i0];
    const double p = span > 0.0 ? (t - times_[i0]) / span : 1.0;
    const double* y0 = states_.data() + i0 * state_size_;
    const double* y1 = states_.data() + i1 * state_size_;
    for(size_t i = 0; i < state_size_; i++) {
      interpolated_[i] = y0[i] + p * (y1[i] - y0[i]);
    }
    // the levels: those of knot i0 and the fraction p of the deltas to i1
    if(!scm_views_.empty()) {
      const auto knot_end = [&](size_t k) {
        return k + 1 < knot_deltas_.size() ? knot_deltas_[k + 1]
                                           : deltas_.size();
      };
      if(i1 > i0) {
        ShowLevels(knot_end(i0), knot_deltas_[i1], knot_end(i1), p);
      } else {
        ShowLevels(knot_end(i1), 0, 0, 0.0);
      }
    }
  }
  shown_time_ = t;
  // the worker owns the context DHVA while computing; evaluate the
  // displacements into a separate buffer
  context_->UpdateD(playback_dhva_, interpolated_.data());
  context_->UpdateFreeDV(playback_dhva_, interpolated_.data() +
                                           context_->get_free_state_offset());
  UpdatePoses(playback_dhva_);
}

void shift::gui::Simulation::ShowContextState() {
  context_->ComputeMobility();
  UpdatePoses(context_->get_dhva());
}

void shift::gui::Simulation::UpdatePoses(
  const shift::core::topology::DHVA& dhva) {
  poses_.time = shown_time_;
  poses_.bodies.resize(bodies_.size());
  for(size_t i = 0; i < bodies_.size(); i++) {
    if(bodies_[i] == context_->get_fixed_body()) {
      poses_.bodies[i].setToIdentity();
    } else {
      poses_.bodies[i] =
        ToMatrix(context_->get_body_origin_pose(bodies_[i], dhva));
    }
  }
  poses_changed_ = true;
}

bool shift::gui::Simulation::TakePoses(Poses& poses) {
  if(!poses_changed_) {
    return false;
  }
  poses = poses_;
  poses_changed_ = false;
  return true;
}

// ------------------------------------------------- deformable terrains

void shift::gui::Simulation::AddScmView(shift::core::terrain::Scm& scm,
                                        int index) {
  ScmView view;
  view.scm = &scm;
  view.terrain = index;
  view.nx = scm.get_node_count_x();
  view.ny = scm.get_node_count_y();
  view.undisturbed.resize(size_t(view.nx * view.ny));
  const auto& z = scene_.terrains[index].z;
  for(long j = 0; j < view.ny; j++) {
    for(long i = 0; i < view.nx; i++) {
      view.undisturbed[size_t(j * view.nx + i)] =
        z[size_t((j + 1) * (view.nx + 2) + i + 1)];
    }
  }
  // the levels the terrain starts from, zero after Load
  std::vector<float> levels;
  scm.GetLevels(levels);
  view.shown.assign(levels.size(), 0.0f);
  for(size_t k = 0; k < levels.size(); k++) {
    ShowLevel(view, int32_t(k), levels[k]);
  }
  view.base = view.worker = view.shown;
  scm_views_.push_back(std::move(view));
}

void shift::gui::Simulation::ShowLevel(ScmView& view, int32_t index,
                                       float level) {
  float& shown = view.shown[size_t(index)];
  if(shown == level) {
    return;
  }
  shown = level;
  const long i = index % view.nx, j = index / view.nx;
  scene_.terrains[view.terrain].z[size_t((j + 1) * (view.nx + 2) + i + 1)] =
    view.undisturbed[size_t(index)] + level;
  // control point (i + 1, j + 1)
  if(view.x_min > view.x_max) {
    view.x_min = view.x_max = i + 1;
    view.y_min = view.y_max = j + 1;
  } else {
    view.x_min = std::min(view.x_min, i + 1);
    view.x_max = std::max(view.x_max, i + 1);
    view.y_min = std::min(view.y_min, j + 1);
    view.y_max = std::max(view.y_max, j + 1);
  }
}

void shift::gui::Simulation::ShowLevels(size_t base, size_t partial_begin,
                                        size_t partial_end, double p) {
  // the deltas shown in part before go back to their base levels
  for(size_t k = partial_begin_; k < partial_end_; k++) {
    const LevelDelta& d = deltas_[k];
    ScmView& view = scm_views_[size_t(d.view)];
    ShowLevel(view, d.index, view.base[size_t(d.index)]);
  }
  // the base levels, forward or back to position base
  for(; base_delta_ < base; base_delta_++) {
    const LevelDelta& d = deltas_[base_delta_];
    ScmView& view = scm_views_[size_t(d.view)];
    view.base[size_t(d.index)] = d.after;
    ShowLevel(view, d.index, d.after);
  }
  for(; base_delta_ > base; base_delta_--) {
    const LevelDelta& d = deltas_[base_delta_ - 1];
    ScmView& view = scm_views_[size_t(d.view)];
    view.base[size_t(d.index)] = d.before;
    ShowLevel(view, d.index, d.before);
  }
  for(size_t k = partial_begin; k < partial_end; k++) {
    const LevelDelta& d = deltas_[k];
    ShowLevel(scm_views_[size_t(d.view)], d.index,
              float(d.before + p * (d.after - d.before)));
  }
  partial_begin_ = partial_begin;
  partial_end_ = partial_end;
}

void shift::gui::Simulation::RestartTerrains(bool reset) {
  // the worker is not running: Run, Reset and AdoptShownState joined it
  for(auto& view : scm_views_) {
    if(reset) {
      for(size_t k = 0; k < view.shown.size(); k++) {
        ShowLevel(view, int32_t(k), 0.0f);
      }
    }
    view.scm->SetLevels(view.shown);
  }
}

void shift::gui::Simulation::TakeTerrainUpdates(
  std::vector<TerrainUpdate>& updates) {
  updates.clear();
  for(auto& view : scm_views_) {
    if(view.x_min > view.x_max) {
      continue;
    }
    const SceneTerrain& terrain = scene_.terrains[view.terrain];
    TerrainUpdate update;
    update.terrain = view.terrain;
    update.x = int(view.x_min);
    update.y = int(view.y_min);
    update.width = int(view.x_max - view.x_min + 1);
    update.height = int(view.y_max - view.y_min + 1);
    update.z.resize(size_t(update.width) * size_t(update.height));
    for(int j = 0; j < update.height; j++) {
      const float* row =
        terrain.z.data() + size_t(update.y + j) * size_t(terrain.width) +
        size_t(update.x);
      std::copy(row, row + update.width,
                update.z.begin() + size_t(j) * size_t(update.width));
    }
    updates.push_back(std::move(update));
    view.x_min = 0;
    view.x_max = -1;
  }
}

// ------------------------------------------------------------ editing q

bool shift::gui::Simulation::AdoptShownState() {
  if(!context_ || is_computing()) {
    return false;
  }
  JoinWorker();
  if(has_results()) {
    // continue from the configuration shown; the recorded results no longer
    // correspond to the context once q is edited
    std::memcpy(context_->get_q().data(), interpolated_.data(),
                state_size_ * sizeof(double));
    RestartTerrains(false);
    ClearResults();
    context_->UpdateDHVA();
  }
  return true;
}

double shift::gui::Simulation::get_shown_q(int q_index) {
  if(!context_ || q_index < 0 || q_index >= static_cast<int>(state_size_)) {
    return 0.0;
  }
  if(has_results()) {
    return interpolated_[q_index];
  }
  return context_->get_q()[q_index];
}

// ------------------------------------------------------------ HDF5 logs

void shift::gui::Simulation::RemoveLogDirectory() {
  if(!log_directory_.empty()) {
    std::error_code error;
    std::filesystem::remove_all(log_directory_, error);
    log_directory_.clear();
  }
}

/* a string attribute of an object of an HDF5 file */
static std::string ReadStringAttribute(const H5::H5Object& object,
                                       const std::string& name) {
  H5::Attribute attribute = object.openAttribute(name);
  H5::StrType type = attribute.getStrType();
  std::string text;
  attribute.read(type, text);
  return text;
}

/* the length of a one-dimensional dataset, or the rows of a 2-D one */
static hsize_t Rows(const H5::DataSet& dataset) {
  H5::DataSpace space = dataset.getSpace();
  if(space.getSimpleExtentNdims() < 1) {
    return 0;
  }
  hsize_t dims[2] = {0, 0};
  space.getSimpleExtentDims(dims);
  return dims[0];
}

void shift::gui::Simulation::LoadLog(const std::string& filename) {
  std::string error_prefix = "Log " + filename + ": ";
  H5::Exception::dontPrint();
  try {
    H5::H5File file(filename, H5F_ACC_RDONLY);
    if(!file.nameExists("/model/json")) {
      throw shift::core::Exception(
        error_prefix + "it holds no model (/model/json); it was written by an "
                       "older shift");
    }
    /* the model's files, under a temporary directory deep enough that the
    paths that climb out of the model's directory (..) stay inside it */
    std::random_device random;
    const std::filesystem::path root =
      std::filesystem::temp_directory_path() /
      ("shift-log-" + std::to_string(random()));
    log_directory_ = root.string();
    struct File {
      std::string path, original, names;
      H5::DataSet dataset;
    };
    std::vector<File> files;
    size_t depth = 0;
    if(file.nameExists("/model/files")) {
      H5::Group group = file.openGroup("/model/files");
      for(hsize_t k = 0; k < group.getNumObjs(); k++) {
        File f;
        f.dataset = group.openDataSet(group.getObjnameByIdx(k));
        f.path = ReadStringAttribute(f.dataset, "path");
        f.original = ReadStringAttribute(f.dataset, "original");
        if(f.dataset.attrExists("names")) {
          f.names = ReadStringAttribute(f.dataset, "names");
        }
        size_t up = 0;
        for(const auto& part : std::filesystem::path(f.path)) {
          if(part != "..") {
            break;
          }
          up++;
        }
        depth = std::max(depth, up);
        files.push_back(std::move(f));
      }
    }
    std::filesystem::path model_directory = root / "model";
    for(size_t k = 0; k < depth; k++) {
      model_directory /= "d";
    }
    std::filesystem::create_directories(model_directory);
    std::map<std::string, std::string> file_map;
    for(auto& f : files) {
      const std::filesystem::path destination =
        (model_directory / f.path).lexically_normal();
      std::filesystem::create_directories(destination.parent_path());
      std::vector<char> bytes(Rows(f.dataset));
      if(!bytes.empty()) {
        f.dataset.read(bytes.data(), H5::PredType::NATIVE_UINT8);
      }
      std::ofstream out(destination, std::ios::binary);
      out.write(bytes.data(), std::streamsize(bytes.size()));
      if(!out) {
        throw shift::core::Exception(error_prefix + "could not write " +
                                     destination.string());
      }
      // the model finds the file here by the names it gave it, whatever
      // its include directories (they may not exist on this computer); the
      // files those name, such as an .mtl, are beside them as they were
      std::istringstream names(f.names);
      for(std::string name; std::getline(names, name);) {
        if(!name.empty()) {
          file_map[name] = destination.string();
        }
      }
      file_map[f.original] = destination.string();
    }
    /* the model, without its loggers */
    H5::DataSet json_dataset = file.openDataSet("/model/json");
    std::string text;
    json_dataset.read(text, json_dataset.getStrType());
    Json::Value model;
    {
      Json::CharReaderBuilder builder;
      std::string errors;
      std::istringstream in(text);
      if(!Json::parseFromStream(builder, in, &model, &errors)) {
        throw shift::core::Exception(error_prefix + "its model: " + errors);
      }
    }
    if(model.isMember("model_parameters")) {
      model["model_parameters"].removeMember("loggers");
    }
    std::filesystem::path model_file =
      model_directory /
      std::filesystem::path(ReadStringAttribute(json_dataset, "filename"))
        .filename();
    model_file.replace_extension(".json");
    {
      std::ofstream out(model_file);
      Json::StreamWriterBuilder builder;
      out << Json::writeString(builder, model);
    }
    LoadModel(model_file.string(), std::move(file_map));

    /* the rows: q, dq and the states of free bodies */
    std::vector<double> times(Rows(file.openDataSet("/time")));
    file.openDataSet("/time").read(times.data(), H5::PredType::NATIVE_DOUBLE);
    const size_t rows = times.size();
    const size_t q_count = context_->get_q_count();
    const size_t dq_count = context_->get_dq_count();
    const size_t free_count = state_size_ - q_count - dq_count;
    std::vector<double> states(rows * state_size_, 0.0);
    const std::pair<const char*, size_t> blocks[] = {
      {"q", q_count}, {"dq", dq_count}, {"free", free_count}};
    size_t offset = 0;
    for(const auto& [name, count] : blocks) {
      if(count == 0) {
        continue;
      }
      const std::string path = std::string("/states/") + name;
      if(!file.nameExists("/states") || !file.nameExists(path)) {
        throw shift::core::Exception(
          error_prefix + "it has no " + path +
          " (log_all_channels was false, or the log is older than the "
          "logging of free bodies)");
      }
      H5::DataSet dataset = file.openDataSet(path);
      hsize_t dims[2] = {0, 0};
      dataset.getSpace().getSimpleExtentDims(dims);
      if(dims[0] < rows || dims[1] != count) {
        throw shift::core::Exception(error_prefix + path + " does not match "
                                                         "the model");
      }
      std::vector<double> block(rows * count);
      H5::DataSpace file_space = dataset.getSpace();
      const hsize_t start[2] = {0, 0}, size[2] = {rows, count};
      file_space.selectHyperslab(H5S_SELECT_SET, size, start);
      H5::DataSpace memory_space(2, size);
      dataset.read(block.data(), H5::PredType::NATIVE_DOUBLE, memory_space,
                   file_space);
      for(size_t r = 0; r < rows; r++) {
        std::copy(block.begin() + r * count, block.begin() + (r + 1) * count,
                  states.begin() + r * state_size_ + offset);
      }
      offset += count;
    }

    /* the changes of the deformable terrains, row by row */
    struct TerrainRows {
      std::vector<int64_t> starts;
      std::vector<int32_t> indices;
      std::vector<float> levels;
    };
    std::vector<TerrainRows> terrain_rows(scm_views_.size());
    for(size_t v = 0; v < scm_views_.size(); v++) {
      const ScmView& view = scm_views_[v];
      const std::string path = "/terrain/" + view.scm->get_name();
      if(!file.nameExists("/terrain") || !file.nameExists(path)) {
        throw shift::core::Exception(error_prefix + "it has no " + path);
      }
      H5::Group group = file.openGroup(path);
      int64_t counts[2] = {0, 0};
      group.openAttribute("node_count").read(H5::PredType::NATIVE_INT64,
                                             counts);
      if(counts[0] != view.nx || counts[1] != view.ny) {
        throw shift::core::Exception(error_prefix + path +
                                     " does not match the grid of the model");
      }
      TerrainRows& t = terrain_rows[v];
      H5::DataSet starts = group.openDataSet("row_start");
      H5::DataSet indices = group.openDataSet("index");
      H5::DataSet levels = group.openDataSet("level");
      t.starts.resize(Rows(starts));
      t.indices.resize(Rows(indices));
      t.levels.resize(Rows(levels));
      if(t.starts.size() < rows || t.levels.size() != t.indices.size()) {
        throw shift::core::Exception(error_prefix + path + " is incomplete");
      }
      if(!t.starts.empty()) {
        starts.read(t.starts.data(), H5::PredType::NATIVE_INT64);
      }
      if(!t.indices.empty()) {
        indices.read(t.indices.data(), H5::PredType::NATIVE_INT32);
        levels.read(t.levels.data(), H5::PredType::NATIVE_FLOAT);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(size_t r = 0; r < rows; r++) {
      if(!times_.empty() && !(times[r] > times_.back())) {
        continue; // a repeated time: Observe keeps the first
      }
      times_.push_back(times[r]);
      states_.insert(states_.end(), states.begin() + r * state_size_,
                     states.begin() + (r + 1) * state_size_);
      knot_deltas_.push_back(deltas_.size());
      for(size_t v = 0; v < scm_views_.size(); v++) {
        ScmView& view = scm_views_[v];
        const TerrainRows& t = terrain_rows[v];
        const int64_t end =
          r + 1 < t.starts.size() ? t.starts[r + 1] : int64_t(t.indices.size());
        for(int64_t k = t.starts[r]; k < end; k++) {
          const int32_t index = t.indices[size_t(k)];
          if(index < 0 || index >= view.nx * view.ny) {
            throw shift::core::Exception(error_prefix + "a node of /terrain/" +
                                         view.scm->get_name() +
                                         " is beyond the grid");
          }
          float& level = view.worker[size_t(index)];
          const float after = t.levels[size_t(k)];
          if(level != after) {
            deltas_.push_back({int32_t(v), index, level, after});
            level = after;
          }
        }
      }
    }
  } catch(H5::Exception& e) {
    throw shift::core::Exception(error_prefix + e.getDetailMsg());
  }
  if(has_results()) {
    std::lock_guard<std::mutex> lock(mutex_);
    shown_time_ = times_.front();
  }
  ShowTime(shown_time_);
}
