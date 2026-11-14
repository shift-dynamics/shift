#include "shift/core/force_module/dem.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/context.h"
#include "shift/core/dem/cpu_solver.h"
#include "shift/core/dem/gpu_solver.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/integrator/base.h"
#include "shift/core/log.h"
#include "shift/core/model_parameters.h"

/* the vertices and triangles of the meshes of a file, read by assimp (the
vertices of all its meshes, welded where they coincide) */
static void ReadTriangles(const std::string& path,
                          std::vector<std::array<float, 3>>& vertices,
                          std::vector<std::array<uint32_t, 3>>& triangles) {
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
    path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
            aiProcess_PreTransformVertices);
  if(!scene) {
    throw shift::core::Exception("DEM: can't read " + path + ": " +
                                 importer.GetErrorString());
  }
  vertices.clear();
  triangles.clear();
  for(unsigned m = 0; m < scene->mNumMeshes; m++) {
    const aiMesh* mesh = scene->mMeshes[m];
    const uint32_t first = static_cast<uint32_t>(vertices.size());
    for(unsigned v = 0; v < mesh->mNumVertices; v++) {
      vertices.push_back(
        {mesh->mVertices[v].x, mesh->mVertices[v].y, mesh->mVertices[v].z});
    }
    for(unsigned f = 0; f < mesh->mNumFaces; f++) {
      if(mesh->mFaces[f].mNumIndices == 3) {
        triangles.push_back({first + mesh->mFaces[f].mIndices[0],
                             first + mesh->mFaces[f].mIndices[1],
                             first + mesh->mFaces[f].mIndices[2]});
      }
    }
  }
  if(triangles.empty()) {
    throw shift::core::Exception("DEM: no triangles in " + path);
  }
}

bool shift::core::force_module::Dem::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "dem", shift::core::force_module::Dem::Create);

std::unique_ptr<shift::core::force_module::Dem>
shift::core::force_module::Dem::Create() {
  return std::make_unique<shift::core::force_module::Dem>();
}

shift::core::force_module::Dem::~Dem() {}

void shift::core::force_module::DemMaterial::DeclareParameters() {
  DeclareParameter("name", &name, "name of the material", true);
  DeclareParameter("youngs_modulus", &youngs_modulus, "Young's modulus (Pa)",
                   true);
  DeclareParameter("poisson_ratio", &poisson_ratio,
                   "Poisson's ratio; default 0.3", false);
  DeclareParameter("restitution", &restitution,
                   "coefficient of restitution, 0 to 1; default 0.5", false);
  DeclareParameter("friction", &friction,
                   "Coulomb's coefficient of friction; default 0.5", false);
  DeclareParameter("rolling_resistance", &rolling_resistance,
                   "coefficient of rolling resistance (a moment of it times "
                   "the normal force and the effective radius); default 0",
                   false);
}

void shift::core::force_module::DemMaterialPair::DeclareParameters() {
  DeclareParameter("materials", &materials,
                   "the names of the two materials of the pair", true);
  DeclareParameter("restitution", &restitution,
                   "coefficient of restitution of the pair", true);
  DeclareParameter("friction", &friction,
                   "Coulomb's coefficient of friction of the pair", true);
  DeclareParameter("rolling_resistance", &rolling_resistance,
                   "coefficient of rolling resistance of the pair; default 0",
                   false);
}

void shift::core::force_module::DemSphere::DeclareParameters() {
  DeclareParameter("center", &center,
                   "center from the center of mass of the clump, along its "
                   "principal axes (m)",
                   true);
  DeclareParameter("radius", &radius, "radius (m)", true);
  DeclareParameter("material", &material, "name of the material", true);
}

void shift::core::force_module::DemGrain::DeclareParameters() {
  DeclareParameter("name", &name, "name of the shape of grains", true);
  DeclareParameter("radius", &radius,
                   "radius of a single sphere (m); not with spheres", false);
  DeclareParameter("material", &material,
                   "name of the material of a single sphere", false);
  DeclareParameter("mass", &mass,
                   "mass of a grain (kg); of a sphere, or density", false);
  DeclareParameter("density", &density,
                   "density of a single sphere (kg/m^3), for its mass if "
                   "mass is not given",
                   false);
  DeclareParameter("inertia", &inertia,
                   "principal moments of inertia of a clump about its center "
                   "of mass (kg m^2); a sphere's are 2/5 m r^2",
                   false);
  DeclareParameter("spheres", &spheres,
                   "the spheres of a clump, rigid together; not with radius",
                   false);
  DeclareParameter("hull", &hull,
                   "mesh file whose vertices' convex hull is the grain, in "
                   "its principal axes from its center of mass (m); with "
                   "material, mass and inertia, not with radius or spheres",
                   false);
  DeclareParameter("mesh", &mesh,
                   "mesh file drawn in the viewer in place of the spheres "
                   "or the hull, in the grain's principal axes from its "
                   "center of mass (m); by default a hull is drawn as it is",
                   false);
}

void shift::core::force_module::DemFill::DeclareParameters() {
  DeclareParameter("file", &file,
                   "grains saved by save (CSV), placed as they were saved, "
                   "with their velocities, in place of a lattice (grain, "
                   "min, max, spacing)",
                   false);
  DeclareParameter("grain", &grain, "name of the shape of the grains", false);
  DeclareParameter("min", &min, "corner of the box, inertial frame (m)",
                   false);
  DeclareParameter("max", &max, "opposite corner of the box (m)", false);
  DeclareParameter("spacing", &spacing,
                   "distance between the grains of the lattice (m), more "
                   "than a grain's width",
                   false);
  DeclareParameter("jitter", &jitter,
                   "largest move of a grain from the lattice, in spacings, "
                   "0 to 0.5; default 0",
                   false);
  DeclareParameter("seed", &seed, "seed of the jitter; default 1", false);
  DeclareParameter("velocity", &velocity,
                   "initial velocity of the grains (m/s); default 0", false);
}

void shift::core::force_module::DemWall::DeclareParameters() {
  DeclareParameter("normal", &normal,
                   "unit normal out of the solid, inertial frame", true);
  DeclareParameter("offset", &offset,
                   "the solid is where normal . p <= offset (m)", true);
  DeclareParameter("material", &material, "name of the material", true);
}

void shift::core::force_module::DemBody::DeclareParameters() {
  DeclareParameter("body", &body, "name of the body", true);
  DeclareParameter("geometry", &geometry,
                   "names of the body's mesh geometry or convex hull "
                   "collision geometry the grains touch",
                   true);
  DeclareParameter("material", &material, "name of the material", true);
}

void shift::core::force_module::Dem::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("materials", &materials_parameter_,
                   "materials of the grains and the bodies", true);
  DeclareParameter("pairs", &pairs_parameter_,
                   "properties of pairs of materials in place of the "
                   "averages of the two",
                   false);
  DeclareParameter("grains", &grains_parameter_, "the shapes of grains",
                   true);
  DeclareParameter("fill", &fill_parameter_,
                   "boxes filled with grains at the start", true);
  DeclareParameter("walls", &walls_parameter_,
                   "fixed planes, at most 8", false);
  DeclareParameter("bodies", &bodies_parameter_,
                   "bodies the grains touch, by the triangles of their "
                   "geometry",
                   false);
  DeclareParameter("device", &device_,
                   "\"gpu\" (default) or \"cpu\" (single threaded, for few "
                   "grains)",
                   false);
  DeclareParameter("margin", &margin_,
                   "distance within which grains are listed as neighbors "
                   "(m); 0, the default, for a fifth of the smallest radius",
                   false);
  DeclareParameter("max_neighbors", &max_neighbors_,
                   "the most spheres within the margin of a sphere; default "
                   "16",
                   false);
  DeclareParameter("max_triangles", &max_triangles_,
                   "the most triangles of bodies within the margin of a "
                   "sphere, at most 32: 6 to 12 on a flat part, up to 20 at "
                   "an edge or corner; default 24",
                   false);
  DeclareParameter("max_speed", &max_speed_,
                   "speed of the fastest collisions, for the substep (m/s); "
                   "default 1",
                   false);
  DeclareParameter("step_fraction", &step_fraction_,
                   "substep over the shorter of the Rayleigh time and the "
                   "collision time, 0.1 to 0.3; default 0.2",
                   false);
  DeclareParameter("step", &fixed_step_,
                   "substep (s); 0, the default, for step_fraction", false);
  DeclareParameter("contact_method", &contact_method_,
                   "\"smc\" (default): smooth contact, Hertz-Mindlin "
                   "forces; or \"nsc\": non-smooth contact, rigid contacts "
                   "resolved as impulses (as Chrono's ChSystemNSC)",
                   false);
  DeclareParameter("nsc_iterations", &nsc_iterations_,
                   "NSC: iterations of the solver each step; default 100",
                   false);
  DeclareParameter("nsc_solver", &nsc_solver_,
                   "NSC: \"apgd\" (default, accelerated projected "
                   "gradient descent), \"gauss_seidel\" (projected "
                   "Gauss-Seidel, on the CPU only) or \"jacobi\" (projected Jacobi)",
                   false);
  DeclareParameter("nsc_relaxation", &nsc_relaxation_,
                   "NSC: relaxation of the iterations; 0, the default, for 1 "
                   "(Gauss-Seidel) or 0.2 (Jacobi)",
                   false);
  DeclareParameter("recovery_speed", &recovery_speed_,
                   "NSC: the fastest overlaps are undone (m/s); default "
                   "0.01, slow so that bodies do not rebound from the "
                   "overlap their step behind the grains leaves",
                   false);
  DeclareParameter("min_bounce_speed", &min_bounce_speed_,
                   "NSC: collisions slower than this do not bounce (m/s); "
                   "default 0.15",
                   false);
  DeclareParameter("save", &save_,
                   "file to write the grains to (CSV), relative to the "
                   "model's directory, for a fill of another model",
                   false);
  DeclareParameter("save_time", &save_time_,
                   "time to save the grains at (s); 0, the default, for the "
                   "end of the simulation",
                   false);
}

uint32_t shift::core::force_module::Dem::MaterialIndex(
  const std::string& name, const std::string& where) {
  const auto& materials = materials_parameter_.get_data();
  for(size_t m = 0; m < materials.size(); m++) {
    if(materials[m]->name == name) {
      return static_cast<uint32_t>(m);
    }
  }
  throw shift::core::Exception("DEM " + get_name() + ": " + where +
                               " names material '" + name +
                               "', which is not among materials");
}

/*!
The grains of the boxes of fill, the shapes, materials, pairs and walls, and
the bodies by name; their triangles wait for the first step, when the
geometry is loaded
*/
void shift::core::force_module::Dem::Load() {
  shift::core::force_module::Base::Load();
  const std::string prefix = "DEM " + get_name() + ": ";
  /* the materials and pairs */
  materials_.clear();
  for(const auto& m : materials_parameter_.get_data()) {
    if(!(m->youngs_modulus > 0.0) || !(m->poisson_ratio > -1.0) ||
       !(m->poisson_ratio < 0.5) || !(m->restitution > 0.0) ||
       !(m->restitution <= 1.0) || !(m->friction >= 0.0) ||
       !(m->rolling_resistance >= 0.0)) {
      throw shift::core::Exception(
        prefix + "material " + m->name +
        ": youngs_modulus must be positive, poisson_ratio from -1 to 0.5, "
        "restitution above 0 to 1, friction and rolling_resistance not "
        "negative");
    }
    shift::core::dem::Material material;
    material.youngs_modulus = static_cast<float>(m->youngs_modulus);
    material.poisson_ratio = static_cast<float>(m->poisson_ratio);
    material.restitution = static_cast<float>(m->restitution);
    material.friction = static_cast<float>(m->friction);
    material.rolling_resistance = static_cast<float>(m->rolling_resistance);
    materials_.push_back(material);
  }
  settings_ = shift::core::dem::Settings();
  for(const auto& p : pairs_parameter_.get_data()) {
    if(p->materials.size() != 2) {
      throw shift::core::Exception(prefix +
                                   "a pair names two materials, not " +
                                   std::to_string(p->materials.size()));
    }
    shift::core::dem::PairProperties pair;
    pair.a = MaterialIndex(p->materials[0], "a pair");
    pair.b = MaterialIndex(p->materials[1], "a pair");
    pair.restitution = static_cast<float>(p->restitution);
    pair.friction = static_cast<float>(p->friction);
    pair.rolling_resistance = static_cast<float>(p->rolling_resistance);
    settings_.pairs.push_back(pair);
  }
  /* the shapes of grains */
  templates_.clear();
  std::vector<std::string> names;
  for(const auto& g : grains_parameter_.get_data()) {
    const std::string where = prefix + "grain " + g->name + ": ";
    const auto& spheres = g->spheres.get_data();
    shift::core::dem::Template t;
    auto file = [&](const std::string& name) {
      const std::string path = context_->FindFile(name);
      if(path.empty()) {
        throw shift::core::Exception(where + "file " + name + " not found");
      }
      return path;
    };
    if(!g->hull.empty()) {
      if(g->radius > 0.0 || !spheres.empty() || g->material.empty() ||
         !(g->mass > 0.0) || !(g->inertia.x > 0.0) || !(g->inertia.y > 0.0) ||
         !(g->inertia.z > 0.0)) {
        throw shift::core::Exception(
          where + "a hull needs a material, its mass and inertia, and no "
                  "radius or spheres");
      }
      std::vector<std::array<float, 3>> vertices;
      std::vector<std::array<uint32_t, 3>> triangles;
      ReadTriangles(file(g->hull), vertices, triangles);
      t.hull = shift::core::dem::Hull::Of(vertices, MaterialIndex(g->material, where),
                                          &t.mesh_triangles);
      t.mesh_vertices = t.hull.vertices;
      t.mass = static_cast<float>(g->mass);
      t.inertia = {static_cast<float>(g->inertia.x),
                   static_cast<float>(g->inertia.y),
                   static_cast<float>(g->inertia.z)};
    } else if(spheres.empty()) {
      if(!(g->radius > 0.0) || g->material.empty()) {
        throw shift::core::Exception(
          where + "a sphere needs a radius and a material, a clump spheres");
      }
      const double volume = 4.0 / 3.0 * M_PI * g->radius * g->radius *
                            g->radius;
      const double mass = g->mass > 0.0 ? g->mass : g->density * volume;
      if(!(mass > 0.0)) {
        throw shift::core::Exception(where +
                                     "a sphere needs its mass or a density");
      }
      t = shift::core::dem::Template::SingleSphere(
        static_cast<float>(g->radius), static_cast<float>(mass / volume),
        MaterialIndex(g->material, where));
      t.mass = static_cast<float>(mass);
      const float moment =
        static_cast<float>(0.4 * mass * g->radius * g->radius);
      t.inertia = {moment, moment, moment};
    } else {
      if(g->radius > 0.0 || !(g->mass > 0.0) || !(g->inertia.x > 0.0) ||
         !(g->inertia.y > 0.0) || !(g->inertia.z > 0.0)) {
        throw shift::core::Exception(
          where + "a clump of spheres needs its mass and inertia, and no "
                  "radius");
      }
      for(const auto& s : spheres) {
        if(!(s->radius > 0.0)) {
          throw shift::core::Exception(where +
                                       "a sphere has no positive radius");
        }
        shift::core::dem::Sphere sphere;
        sphere.center = {static_cast<float>(s->center.x),
                         static_cast<float>(s->center.y),
                         static_cast<float>(s->center.z)};
        sphere.radius = static_cast<float>(s->radius);
        sphere.material = MaterialIndex(s->material, where);
        t.spheres.push_back(sphere);
      }
      t.mass = static_cast<float>(g->mass);
      t.inertia = {static_cast<float>(g->inertia.x),
                   static_cast<float>(g->inertia.y),
                   static_cast<float>(g->inertia.z)};
    }
    if(!g->mesh.empty()) {
      ReadTriangles(file(g->mesh), t.mesh_vertices, t.mesh_triangles);
    }
    templates_.push_back(t);
    names.push_back(g->name);
  }
  /* the grains, on the lattice of each box */
  grain_names_ = names;
  state_ = shift::core::dem::State();
  for(const auto& f : fill_parameter_.get_data()) {
    if(!f->file.empty()) {
      if(!f->grain.empty() || f->spacing > 0.0) {
        throw shift::core::Exception(
          prefix + "a fill is a file or a lattice (grain, min, max, "
                   "spacing), not both");
      }
      const std::string path = context_->FindFile(f->file);
      if(path.empty()) {
        throw shift::core::Exception(prefix + "fill: file " + f->file +
                                     " not found");
      }
      shift::core::dem::ReadGrains(path, names, state_);
      continue;
    }
    const auto found = std::find(names.begin(), names.end(), f->grain);
    if(found == names.end()) {
      throw shift::core::Exception(prefix + "fill names grain '" + f->grain +
                                   "', which is not among grains");
    }
    const uint32_t shape = static_cast<uint32_t>(found - names.begin());
    if(!(f->spacing > 0.0) || !(f->jitter >= 0.0) || !(f->jitter <= 0.5)) {
      throw shift::core::Exception(
        prefix + "fill: spacing must be positive, jitter from 0 to 0.5");
    }
    long n[3];
    for(int k = 0; k < 3; k++) {
      n[k] = static_cast<long>(
        std::floor((f->max.at[k] - f->min.at[k]) / f->spacing + 1e-9));
    }
    uint32_t seed = static_cast<uint32_t>(f->seed);
    const auto random = [&seed]() {
      seed = seed * 1664525u + 1013904223u;
      return (seed >> 8) / 16777216.0 - 0.5;
    };
    for(long z = 0; z < n[2]; z++) {
      for(long y = 0; y < n[1]; y++) {
        for(long x = 0; x < n[0]; x++) {
          const long i[3] = {x, y, z};
          std::array<double, 3> p;
          for(int k = 0; k < 3; k++) {
            p[k] = f->min.at[k] + (i[k] + 0.5) * f->spacing +
                   2.0 * f->jitter * f->spacing * random();
          }
          state_.AddGrain(shape, p);
          state_.velocity.back() = {static_cast<float>(f->velocity.x),
                                    static_cast<float>(f->velocity.y),
                                    static_cast<float>(f->velocity.z)};
        }
      }
    }
  }
  if(state_.size() == 0) {
    throw shift::core::Exception(prefix + "fill places no grains");
  }
  initial_state_ = state_;
  /* the walls */
  for(const auto& w : walls_parameter_.get_data()) {
    const double length = ksl_vec3_l2norm(&w->normal);
    if(!(length > 0.0)) {
      throw shift::core::Exception(prefix + "a wall has no normal");
    }
    shift::core::dem::Wall wall;
    for(int k = 0; k < 3; k++) {
      wall.normal[k] = w->normal.at[k] / length;
    }
    wall.offset = w->offset / length;
    wall.material = MaterialIndex(w->material, "a wall");
    settings_.walls.push_back(wall);
  }
  settings_.margin = static_cast<float>(margin_);
  if(max_neighbors_ < 1 || max_triangles_ < 1 || max_triangles_ > 32) {
    throw shift::core::Exception(
      prefix + "max_neighbors must be at least 1, max_triangles 1 to 32");
  }
  settings_.max_neighbors = static_cast<uint32_t>(max_neighbors_);
  settings_.max_triangles = static_cast<uint32_t>(max_triangles_);
  /* the bodies */
  bodies_.clear();
  bodies_dem_.clear();
  for(const auto& b : bodies_parameter_.get_data()) {
    auto body = context_->FindActiveBody(b->body, get_parent_assembly());
    if(!body) {
      throw shift::core::Exception(prefix + "body " + b->body +
                                   " not found among the active bodies");
    }
    bodies_.push_back(body);
    shift::core::dem::Body dem_body;
    dem_body.material = MaterialIndex(b->material, "body " + b->body);
    bodies_dem_.push_back(dem_body);
  }
  load_indices_.assign(bodies_.size(), -2);
  body_loads_.assign(bodies_.size(), ksl_coscrew_t{});
  body_impulses_.assign(bodies_.size(), ksl_coscrew_t{});
  owed_impulses_.assign(bodies_.size(), ksl_coscrew_t{});
  if(!(step_fraction_ > 0.0) || !(max_speed_ > 0.0) || fixed_step_ < 0.0) {
    throw shift::core::Exception(
      prefix + "step_fraction and max_speed must be positive, step not "
               "negative");
  }
  if(device_ != "gpu" && device_ != "cpu") {
    throw shift::core::Exception(prefix + "device is \"gpu\" or \"cpu\", not \"" +
                                 device_ + "\"");
  }
  if(save_time_ < 0.0) {
    throw shift::core::Exception(prefix + "save_time must not be negative");
  }
  if(contact_method_ != "smc" && contact_method_ != "nsc") {
    throw shift::core::Exception(prefix +
                                 "contact_method is \"smc\" or \"nsc\", "
                                 "not \"" +
                                 contact_method_ + "\"");
  }
  if(nsc_solver_ != "apgd" && nsc_solver_ != "gauss_seidel" &&
     nsc_solver_ != "jacobi") {
    throw shift::core::Exception(prefix +
                                 "nsc_solver is \"apgd\", \"gauss_seidel\" "
                                 "or \"jacobi\", not \"" +
                                 nsc_solver_ + "\"");
  }
  if(nsc_iterations_ < 1 || nsc_relaxation_ < 0.0 || !(recovery_speed_ > 0.0) ||
     min_bounce_speed_ < 0.0) {
    throw shift::core::Exception(
      prefix + "nsc_iterations must be at least 1, recovery_speed positive, "
               "nsc_relaxation and min_bounce_speed not negative");
  }
  if(contact_method_ == "nsc") {
    if(device_ == "gpu" && nsc_solver_ == "gauss_seidel") {
      throw shift::core::Exception(
        prefix + "nsc_solver \"gauss_seidel\" runs on the CPU only: device "
                 "\"cpu\", or nsc_solver \"apgd\" or \"jacobi\" on the "
                 "GPU");
    }
    settings_.method = shift::core::dem::ContactMethod::kNsc;
    settings_.nsc.iterations = static_cast<uint32_t>(nsc_iterations_);
    settings_.nsc.solver =
      nsc_solver_ == "jacobi"
        ? shift::core::dem::NscSolver::kJacobi
        : (nsc_solver_ == "gauss_seidel"
             ? shift::core::dem::NscSolver::kGaussSeidel
             : shift::core::dem::NscSolver::kApgd);
    settings_.nsc.relaxation = static_cast<float>(nsc_relaxation_);
    settings_.nsc.recovery_speed = static_cast<float>(recovery_speed_);
    settings_.nsc.min_bounce_speed = static_cast<float>(min_bounce_speed_);
    /* the step: a tenth of the smallest radius at max_speed; the margin:
    what two grains close at max_speed in a step */
    float smallest = std::numeric_limits<float>::infinity();
    for(const auto& t : templates_) {
      for(const auto& sphere : t.spheres) {
        smallest = std::min(smallest, sphere.radius);
      }
    }
    nsc_step_ = fixed_step_ > 0.0 ? fixed_step_ : 0.1 * smallest / max_speed_;
    if(!(margin_ > 0.0)) {
      settings_.margin = static_cast<float>(
        std::max(0.2 * smallest, 2.0 * max_speed_ * nsc_step_));
    }
  }
  started_ = false;
  saved_ = false;
}

/*!
Mesh geometry gives its triangles as they are (in the coordinates of the
body's origin); a convex hull its hull's, from the frame of the geometry
*/
void shift::core::force_module::Dem::BodyTriangles() {
  const auto& parameters = bodies_parameter_.get_data();
  for(size_t b = 0; b < bodies_.size(); b++) {
    shift::core::dem::Body& out = bodies_dem_[b];
    out.vertices.clear();
    out.triangles.clear();
    const std::string where =
      "DEM " + get_name() + ": body " + parameters[b]->body + ": ";
    for(const auto& name : parameters[b]->geometry) {
      bool found = false;
      for(const auto& g : bodies_[b]->get_geometry()) {
        auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(g.get());
        if(!mesh || g->get_name() != name) {
          continue;
        }
        for(const auto& data : mesh->get_mesh_data()) {
          const uint32_t first = static_cast<uint32_t>(out.vertices.size());
          for(size_t v = 0; v + 2 < data.vertices.size(); v += 3) {
            out.vertices.push_back(
              {data.vertices[v], data.vertices[v + 1], data.vertices[v + 2]});
          }
          for(size_t f = 0; f + 2 < data.faces.size(); f += 3) {
            out.triangles.push_back({first + data.faces[f],
                                     first + data.faces[f + 1],
                                     first + data.faces[f + 2]});
          }
        }
        found = true;
      }
      for(auto g : context_->get_active_collision_geometry()) {
        if(g->get_parent_body() != bodies_[b] || g->get_name() != name) {
          continue;
        }
        auto hull =
          dynamic_cast<const shift::core::collision_geometry::ConvexHull*>(g);
        if(!hull) {
          throw shift::core::Exception(
            where + "collision geometry " + name +
            " is not a convex hull or box; use mesh geometry for others");
        }
        /* the hull is in the frame of the geometry, placed in the body by
        get_d_wrt_body */
        const auto& data = hull->get_hull();
        const ksl_SE3_t& d = hull->get_d_wrt_body();
        const uint32_t first = static_cast<uint32_t>(out.vertices.size());
        for(const auto& v : data.vertices) {
          std::array<float, 3> p;
          for(int k = 0; k < 3; k++) {
            p[k] = static_cast<float>(d.R.v[0].at[k] * v.x +
                                      d.R.v[1].at[k] * v.y +
                                      d.R.v[2].at[k] * v.z + d.t.at[k]);
          }
          out.vertices.push_back(p);
        }
        for(size_t f = 0; f + 2 < data.faces.size(); f += 3) {
          out.triangles.push_back({first + data.faces[f],
                                   first + data.faces[f + 1],
                                   first + data.faces[f + 2]});
        }
        found = true;
      }
      if(!found) {
        throw shift::core::Exception(where + "no mesh geometry or collision "
                                             "geometry named " +
                                     name);
      }
    }
    if(out.triangles.empty()) {
      throw shift::core::Exception(where + "its geometry has no triangles");
    }
  }
}

void shift::core::force_module::Dem::BodyPoses(
  std::vector<shift::core::sph::BodyPose>& poses) {
  poses.resize(bodies_.size());
  for(size_t b = 0; b < bodies_.size(); b++) {
    if(load_indices_[b] == -2) {
      load_indices_[b] = context_->GetBodyLoadIndex(bodies_[b]);
    }
    const ksl_SE3_t pose =
      context_->get_body_origin_pose(bodies_[b], context_->get_dhva());
    const ksl_screw_t velocity = context_->GetBodyVelocity(load_indices_[b]);
    /* the velocity of the body's origin, v + w x t */
    ksl_vec3_t v;
    ksl_cross_vv(&velocity.ang, &pose.t, &v);
    shift::core::sph::BodyPose& p = poses[b];
    for(int row = 0; row < 3; row++) {
      for(int column = 0; column < 3; column++) {
        p.R[3 * column + row] = pose.R.v[column].at[row];
      }
      p.t[row] = pose.t.at[row];
      p.v[row] = v.at[row] + velocity.lin.at[row];
      p.w[row] = velocity.ang.at[row];
    }
  }
}

void shift::core::force_module::Dem::Start() {
  const ksl_vec3_t* g = context_->get_model_parameters().get_gravity();
  settings_.gravity = {static_cast<float>(g->x), static_cast<float>(g->y),
                       static_cast<float>(g->z)};
  BodyTriangles();
  /* the bodies' mass, for the non-smooth method */
  for(size_t b = 0; b < bodies_.size(); b++) {
    const ksl_inertia_t& inertia = bodies_[b]->get_inertia();
    const ksl_vec3_t& center = bodies_[b]->get_center_of_mass_wrt_origin();
    shift::core::dem::Body& body = bodies_dem_[b];
    body.mass = static_cast<float>(inertia.m);
    body.center_of_mass = {static_cast<float>(center.x),
                           static_cast<float>(center.y),
                           static_cast<float>(center.z)};
    body.inertia = {static_cast<float>(inertia.Ixx),
                    static_cast<float>(inertia.Iyy),
                    static_cast<float>(inertia.Izz),
                    static_cast<float>(inertia.Ixy),
                    static_cast<float>(inertia.Iyz),
                    static_cast<float>(inertia.Izx)};
  }
  BodyPoses(last_poses_);
  for(size_t b = 0; b < bodies_dem_.size(); b++) {
    bodies_dem_[b].pose = last_poses_[b];
  }
  save_at_ = save_time_;
  if(!(save_at_ > 0.0) && !context_->get_active_integrators().empty()) {
    save_at_ = context_->get_active_integrators()[0]->get_time_span();
  }
  step_ = settings_.method == shift::core::dem::ContactMethod::kNsc
            ? nsc_step_
          : fixed_step_ > 0.0
            ? fixed_step_
            : shift::core::dem::StableStep(templates_, materials_, max_speed_,
                                           step_fraction_);
  if(device_ == "gpu") {
    device_ptr_ = context_->GetGpuDevice();
    solver_ = std::make_unique<shift::core::dem::GpuSolver>(
      *device_ptr_, settings_, materials_, templates_, state_, bodies_dem_);
  } else {
    solver_ = std::make_unique<shift::core::dem::CpuSolver>(
      settings_, materials_, templates_, state_, bodies_dem_);
  }
  size_t triangles = 0;
  for(const auto& b : bodies_dem_) {
    triangles += b.triangles.size();
  }
  SHIFT_INFO("DEM " + get_name() + ": " + std::to_string(state_.size()) +
             " grains, " + std::to_string(bodies_.size()) + " bodies of " +
             std::to_string(triangles) + " triangles, substep " +
             std::to_string(step_) + " s on the " + device_ + ", " +
             (settings_.method == shift::core::dem::ContactMethod::kNsc
                ? "non-smooth contact (NSC)"
                : "smooth contact (SMC)"));
}

void shift::core::force_module::Dem::AcceptStep() {
  const double t = context_->get_force_time();
  if(!started_) {
    Start();
    started_ = true;
    time_ = last_time_ = path_time_ = t;
    solver_->set_time(t);
    pending_ = 0.0;
    SaveIfDue(t);
    return;
  }
  if(!(t > last_time_)) {
    return;
  }
  pending_ += t - last_time_;
  /* whole substeps; the rest waits for the next step. The bodies follow
  their path from the last accepted step to this one; with NSC, whose steps
  are longer than the integrator's, from where they were when the grains
  last advanced, so that the path spans the steps */
  std::vector<shift::core::sph::BodyPose> poses;
  if(!bodies_.empty()) {
    BodyPoses(poses);
    solver_->SetBodyPaths(last_poses_, poses, path_time_, t);
  }
  last_time_ = t;
  const bool nsc =
    settings_.method == shift::core::dem::ContactMethod::kNsc;
  const long n = static_cast<long>(std::floor(pending_ / step_ + 1e-9));
  if(n > 0) {
    if(!bodies_.empty()) {
      solver_->ClearImpulses();
    }
    /* NSC: to t exactly, in n equal steps, so that the bodies' load starts
    where the grains stop and no impulse is counted twice (see dem::Nsc);
    SMC: whole substeps, the rest carried */
    const double duration = nsc ? pending_ : n * step_;
    solver_->Steps(static_cast<float>(duration / n), n);
    if(nsc) {
      solver_->set_time(t);
    }
    pending_ = nsc ? 0.0 : std::max(0.0, pending_ - n * step_);
    time_ = solver_->get_time();
    if(!bodies_.empty()) {
      UpdateBodyLoads(duration);
    }
  }
  if(!bodies_.empty() && (n > 0 || !nsc)) {
    /* the start of the next path */
    last_poses_.swap(poses);
    path_time_ = t;
  }
  SaveIfDue(t);
}

void shift::core::force_module::Dem::SaveIfDue(double t) {
  if(save_.empty() || saved_ ||
     t < save_at_ - 1e-9 * std::max(1.0, save_at_)) {
    return;
  }
  std::string path = save_;
  if(!std::filesystem::path(path).is_absolute()) {
    path = (std::filesystem::path(context_->get_filename()).parent_path() /
            path)
             .string();
  }
  SaveGrains(path);
  saved_ = true;
  SHIFT_INFO("DEM " + get_name() + ": saved " +
             std::to_string(state_.size()) + " grains at t = " +
             std::to_string(time_) + " s to " + path);
}

void shift::core::force_module::Dem::SaveGrains(const std::string& path) {
  shift::core::dem::State grains;
  GetGrains(grains);
  shift::core::dem::WriteGrains(path, grains, grain_names_);
}

/*!
The mean force and moment of the impulses over duration; the moments are
about the origin of the body at each substep, here about the origin of the
inertial frame, through the origin of the body at this step
*/
void shift::core::force_module::Dem::UpdateBodyLoads(double duration) {
  std::vector<std::array<double, 6>> impulses;
  solver_->GetImpulses(impulses);
  const bool nsc = settings_.method == shift::core::dem::ContactMethod::kNsc;
  for(size_t b = 0; b < bodies_.size(); b++) {
    /* the impulse about the origin of the inertial frame */
    ksl_coscrew_t impulse;
    for(int k = 0; k < 3; k++) {
      impulse.lin.at[k] = impulses[b][k];
    }
    const ksl_vec3_t origin = {
      {last_poses_[b].t[0], last_poses_[b].t[1], last_poses_[b].t[2]}};
    ksl_vec3_t arm;
    ksl_cross_vv(&origin, &impulse.lin, &arm);
    for(int k = 0; k < 3; k++) {
      impulse.ang.at[k] = impulses[b][3 + k] + arm.at[k];
    }
    ksl_coscrew_t& load = body_loads_[b];
    ksl_coscrew_t& owed = owed_impulses_[b];
    for(int k = 0; k < 3; k++) {
      body_impulses_[b].lin.at[k] += impulse.lin.at[k];
      body_impulses_[b].ang.at[k] += impulse.ang.at[k];
      if(nsc) {
        /* NSC: the impulse not yet delivered, less what the last load
        delivered over duration, the time since it was set, and this
        impulse, over the next duration as long as this one */
        owed.lin.at[k] += impulse.lin.at[k] - duration * load.lin.at[k];
        owed.ang.at[k] += impulse.ang.at[k] - duration * load.ang.at[k];
        load.lin.at[k] = owed.lin.at[k] / duration;
        load.ang.at[k] = owed.ang.at[k] / duration;
      } else {
        load.lin.at[k] = impulse.lin.at[k] / duration;
        load.ang.at[k] = impulse.ang.at[k] / duration;
      }
    }
  }
}

void shift::core::force_module::Dem::Service(const double*, double*) {
  for(size_t b = 0; b < bodies_.size(); b++) {
    if(load_indices_[b] == -2) {
      load_indices_[b] = context_->GetBodyLoadIndex(bodies_[b]);
    }
    context_->AddBodyLoad(load_indices_[b], body_loads_[b]);
  }
}

void shift::core::force_module::Dem::GetGrains(
  shift::core::dem::State& state) {
  if(solver_) {
    solver_->GetState(state);
  } else {
    state = state_;
  }
}

void shift::core::force_module::Dem::SetGrains(
  const shift::core::dem::State& state) {
  if(state.size() != initial_state_.size() ||
     state.template_index != initial_state_.template_index ||
     state.orientation.size() != state.size() ||
     state.velocity.size() != state.size() ||
     state.angular_velocity.size() != state.size()) {
    throw shift::core::Exception("DEM " + get_name() +
                                 ": SetGrains needs the grains of the model, "
                                 "in their order");
  }
  state_ = state;
  solver_.reset();
  started_ = false;
  saved_ = false;
  step_ = time_ = last_time_ = pending_ = 0.0;
  body_loads_.assign(bodies_.size(), ksl_coscrew_t{});
  body_impulses_.assign(bodies_.size(), ksl_coscrew_t{});
  owed_impulses_.assign(bodies_.size(), ksl_coscrew_t{});
}

shift::core::dem::Statistics
shift::core::force_module::Dem::GetStatistics() const {
  return solver_ ? solver_->GetStatistics() : shift::core::dem::Statistics();
}
