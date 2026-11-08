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

#include <QGenericMatrix>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
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
for the ray tracer and devices without tessellation */
static void AddTerrain(const shift::core::terrain::Basic& terrain,
                       shift::gui::TextureLoader& textures,
                       shift::gui::Scene& scene) {
  constexpr int kMaximumMeshSamples = 513; // per side
  const double x_extent = terrain.get_x_extent();
  const double y_extent = terrain.get_y_extent();
  if(!(x_extent > 0.0) || !(y_extent > 0.0)) {
    return;
  }
  shift::gui::SceneTerrain t;
  t.width = static_cast<int>(terrain.get_x_samples());
  t.height = static_cast<int>(terrain.get_y_samples());
  t.z.assign(terrain.get_control_points().begin(),
             terrain.get_control_points().end());
  t.x_scale = float(terrain.get_x_scale());
  t.y_scale = float(terrain.get_y_scale());
  t.bicubic = terrain.is_bicubic();
  const auto& d = terrain.get_pose();
  QMatrix4x4 pose;
  for(int r = 0; r < 3; r++) {
    pose(r, 0) = float(d.R.v0.at[r]);
    pose(r, 1) = float(d.R.v1.at[r]);
    pose(r, 2) = float(d.R.v2.at[r]);
    pose(r, 3) = float(d.t.at[r]);
  }
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
  /* twice the resolution of the control points, at most
  kMaximumMeshSamples a side */
  const int mesh_x = std::clamp(2 * (t.width - 1) + 1, 2, kMaximumMeshSamples);
  const int mesh_y = std::clamp(2 * (t.height - 1) + 1, 2, kMaximumMeshSamples);
  const QMatrix3x3 rotation = pose.normalMatrix();
  mesh.bounds_min = QVector3D(1e30f, 1e30f, 1e30f);
  mesh.bounds_max = -mesh.bounds_min;
  for(int j = 0; j < mesh_y; j++) {
    const double y = y_extent * j / (mesh_y - 1);
    for(int i = 0; i < mesh_x; i++) {
      const double x = x_extent * i / (mesh_x - 1);
      double z_x, z_y;
      const double z = terrain.LocalZ(x, y, z_x, z_y);
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
      v.uv[0] = float(x);
      v.uv[1] = float(y);
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
}

void shift::gui::Simulation::Load(const std::string& filename) {
  JoinWorker();
  context_.reset();
  bodies_.clear();
  scene_ = Scene();
  ClearResults();
  filename_ = filename;

  auto context = std::make_unique<shift::core::Context>();
  context->LoadJSON(filename);
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
  /* CAD geometry: one SceneCad per file read (geometry::Cad::get_asset)
  and scale, drawn once per body */
  std::map<std::pair<const shift::core::geometry::CadAsset*, double>, int>
    asset_cads;
  for(size_t i = 0; i < bodies_.size(); i++) {
    for(auto& geometry : bodies_[i]->get_geometry()) {
      auto cad = dynamic_cast<shift::core::geometry::Cad*>(geometry.get());
      if(!cad || !cad->is_active() || !cad->get_asset()) {
        continue;
      }
      const auto key = std::make_pair(cad->get_asset(), cad->get_scale());
      auto [it, inserted] = asset_cads.try_emplace(key, 0);
      if(inserted) {
        it->second = static_cast<int>(scene_.cads.size());
        scene_.cads.push_back(MakeSceneCad(cad->get_asset()->model,
                                           cad->get_asset()->bodies,
                                           cad->get_scale()));
      }
      scene_.cads[it->second].bodies.push_back(static_cast<int>(i));
    }
  }
  /* terrains, on the fixed body (body 0) */
  for(auto terrain : context_->get_active_terrain()) {
    if(auto basic = dynamic_cast<shift::core::terrain::Basic*>(terrain)) {
      AddTerrain(*basic, textures, scene_);
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
  if(!times_.empty() && time <= times_.back()) {
    return;
  }
  times_.push_back(time);
  states_.insert(states_.end(), y.begin(), y.begin() + state_size_);
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
