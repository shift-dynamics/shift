#ifndef SHIFT_FORCEMODULE_DEM_H_
#define SHIFT_FORCEMODULE_DEM_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/vec3.h"
#include "shift/core/dem/dem.h"
#include "shift/core/force_module/base.h"
#include "shift/core/parameter/array.h"

namespace shift {
namespace core {
namespace gpu {
class Device;
}
namespace body {
class Base;
}

namespace force_module {

/*! \brief a material of the grains and of the bodies they touch, see Dem */
class DemMaterial : public parameter::List {
public:
  DemMaterial() {
    DeclareParameters();
  }
  virtual ~DemMaterial() {}

  static const std::string get_array_key() {
    return "materials";
  }

  std::string name;
  double youngs_modulus = 0.0;
  double poisson_ratio = 0.3;
  double restitution = 0.5;
  double friction = 0.5;
  double rolling_resistance = 0.0;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief the restitution, friction and rolling resistance of a pair of
 * materials, in place of the averages of the two, see Dem */
class DemMaterialPair : public parameter::List {
public:
  DemMaterialPair() {
    DeclareParameters();
  }
  virtual ~DemMaterialPair() {}

  static const std::string get_array_key() {
    return "pairs";
  }

  std::vector<std::string> materials;
  double restitution = 0.5;
  double friction = 0.5;
  double rolling_resistance = 0.0;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief a sphere of a clump, see DemGrain */
class DemSphere : public parameter::List {
public:
  DemSphere() {
    DeclareParameters();
  }
  virtual ~DemSphere() {}

  static const std::string get_array_key() {
    return "spheres";
  }

  ksl_vec3_t center = {{0.0, 0.0, 0.0}};
  double radius = 0.0;
  std::string material;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief a shape of grains: a sphere (radius, material, mass), or a clump
 * of spheres (spheres, mass, inertia), see Dem */
class DemGrain : public parameter::List {
public:
  DemGrain() {
    DeclareParameters();
  }
  virtual ~DemGrain() {}

  static const std::string get_array_key() {
    return "grains";
  }

  std::string name;
  double radius = 0.0;
  std::string material;
  double mass = 0.0;
  double density = 0.0;
  ksl_vec3_t inertia = {{0.0, 0.0, 0.0}};
  parameter::Array<DemSphere> spheres;
  std::string hull; //!< a mesh file whose convex hull the grain is
  std::string mesh; //!< a mesh file drawn in place of the spheres or hull

protected:
  virtual void DeclareParameters() override;
};

/*! \brief a box filled with grains on a lattice, or the grains of a file
 * that Dem saved, see Dem */
class DemFill : public parameter::List {
public:
  DemFill() {
    DeclareParameters();
  }
  virtual ~DemFill() {}

  static const std::string get_array_key() {
    return "fill";
  }

  std::string file;
  std::string grain;
  ksl_vec3_t min = {{0.0, 0.0, 0.0}};
  ksl_vec3_t max = {{0.0, 0.0, 0.0}};
  double spacing = 0.0;
  double jitter = 0.0;
  int seed = 1;
  ksl_vec3_t velocity = {{0.0, 0.0, 0.0}};

protected:
  virtual void DeclareParameters() override;
};

/*! \brief a fixed plane the grains rest on, see Dem */
class DemWall : public parameter::List {
public:
  DemWall() {
    DeclareParameters();
  }
  virtual ~DemWall() {}

  static const std::string get_array_key() {
    return "walls";
  }

  ksl_vec3_t normal = {{0.0, 0.0, -1.0}};
  double offset = 0.0;
  std::string material;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief a body the grains touch, by the triangles of its geometry, see
 * Dem */
class DemBody : public parameter::List {
public:
  DemBody() {
    DeclareParameters();
  }
  virtual ~DemBody() {}

  static const std::string get_array_key() {
    return "bodies";
  }

  std::string body;
  std::vector<std::string> geometry;
  std::string material;

protected:
  virtual void DeclareParameters() override;
};

/*!
@brief granular material as rigid grains (the discrete element method, see
shift/core/dem/dem.h and the theory page "Discrete Element Method"), coupled
to the bodies of the model

The grains are made from the shapes of grains: a sphere of a radius and
material, with its mass (or, to compute the mass, a density), or a clump of
spheres, rigid together, with the mass and the principal moments of inertia
of the clump, the spheres' centers from its center of mass along its
principal axes. Each box of fill places grains of a shape on a lattice of
spacing from min to max (inertial frame), moved by up to jitter spacings at
random (seed), at an initial velocity; or a fill names a file of grains
written by save (dem::WriteGrains), such as grains settled in another
simulation, which are placed as they were saved, with their velocities.
Walls are fixed planes, the solid
where normal . p <= offset. Values are in SI units, in the inertial frame,
with the model's gravity.

The grains touch each body of bodies on the triangles of its geometry: the
names of mesh geometry of the body (type "mesh"), whose triangles are used as
they are, or of its collision geometry of a convex hull (types "convex_hull"
and "box"), whose hull is used. The grains move in substeps of step, or, if
step is 0, step_fraction of the shorter of the Rayleigh time and the time of
a collision at max_speed of the stiffest grains (dem::StableStep), on the
GPU (device "gpu", see Context::GetGpuDevice) or the CPU ("cpu", single
threaded, for a few thousand grains).

After each accepted step of the integrator (AcceptStep), the grains advance
by whole substeps to the time of the step; what is left of a substep is
carried to the next one. Over those substeps the bodies follow their poses
and velocities from the step before to the step, by cubic Hermite
interpolation (sph::Interpolate), and the grains push on them. The mean of
that force over the substeps, as a load on each body (get_body_load), acts
on the body over the next step: the grains and the bodies are coupled
explicitly, the grains' force a step behind the bodies, as with the SPH
soil (terrain::Sph).

With contact_method "nsc", contacts are rigid and resolved as impulses each
step (the non-smooth method of Chrono's ChSystemNSC, see dem::Nsc), on the
CPU, or on the GPU with nsc_solver "apgd" or "jacobi": the step is then step, or, if step is 0, the time to cross a tenth
of the smallest radius at max_speed, and the margin, if 0, at least the
distance two grains close at max_speed in a step. After an accepted step
the grains advance to its time exactly, in equal steps no shorter than
step, so that the load on the bodies starts where the grains stop; within
those steps each body takes part with its mass (dem::Body).

With save, the grains are written to that file (dem::WriteGrains, in the
directory of the model if the path is relative) after the accepted step
that reaches save_time, or the end of the simulation if save_time is 0.

\code
{"type": "dem", "name": "gravel", "device": "gpu",
 "materials": [{"name": "rock", "youngs_modulus": 1e8, "poisson_ratio": 0.3,
                "restitution": 0.4, "friction": 0.6,
                "rolling_resistance": 0.05}],
 "grains": [{"name": "pebble", "radius": 0.01, "material": "rock",
             "mass": 0.011}],
 "fill": [{"grain": "pebble", "min": [-0.5, -0.5, -0.5],
           "max": [0.5, 0.5, -0.02], "spacing": 0.025, "jitter": 0.1}],
 "walls": [{"normal": [0, 0, -1], "offset": 0, "material": "rock"}],
 "bodies": [{"body": "bucket", "geometry": ["bucket_mesh"],
             "material": "rock"}]}
\endcode
*/
class Dem : public Base {
public:
  Dem()
    : Base() {
    DeclareParameters();
  }
  Dem(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Dem() override;

  static std::unique_ptr<Dem> Create();

  static bool is_registered() {
    return registered_;
  }

  void Load() override;
  void Connect() override {}
  void Service(const double* y, double* dy) override;
  void AcceptStep() override;

  /*! \brief the grains now, read from the solver, or before the first
   * accepted step those the solver will start from */
  void GetGrains(shift::core::dem::State& state);
  /*! \brief write the grains now to a file (dem::WriteGrains) */
  void SaveGrains(const std::string& path);
  /*! \brief the names of the shapes of the grains, in the order of
   * get_templates */
  const std::vector<std::string>& get_grain_names() const {
    return grain_names_;
  }
  /*! \brief the grains as the fill placed them, at Load */
  const shift::core::dem::State& get_initial_grains() const {
    return initial_state_;
  }
  /*! \brief start the grains over from state at the next simulation
   * (Context::Simulate, which starts from time 0): the solver is made again
   * at the first accepted step, without the history of the contacts, and
   * the loads and impulses on the bodies start from zero. state holds the
   * grains of get_initial_grains, in their order, with their templates;
   * throws otherwise */
  void SetGrains(const shift::core::dem::State& state);
  /*! \brief the shapes of the grains, in the order of grains */
  const std::vector<shift::core::dem::Template>& get_templates() const {
    return templates_;
  }
  /*! \brief the number of grains */
  size_t get_grain_count() const {
    return state_.size();
  }
  /*! \brief the substep (s), from the first accepted step */
  double get_step() const {
    return step_;
  }
  /*! \brief the time the grains have reached */
  double get_time() const {
    return time_;
  }
  /*! \brief the counts of the solver: steps, searches, the most neighbors
   * and triangles of a sphere; NSC: the contacts and residual of the last
   * step */
  shift::core::dem::Statistics GetStatistics() const;
  /*! \brief the load of the grains on body b of bodies over the last
   * accepted step, in the inertial frame about its origin; zero before */
  const ksl_coscrew_t& get_body_load(size_t b) const {
    return body_loads_[b];
  }
  /*! \brief the impulse of the grains on body b since the first step, in
   * the inertial frame about its origin (N s, N m s) */
  const ksl_coscrew_t& get_body_impulse(size_t b) const {
    return body_impulses_[b];
  }
  /*! \brief the number of triangles of body b, from the first step */
  size_t get_body_triangle_count(size_t b) const {
    return b < bodies_dem_.size() ? bodies_dem_[b].triangles.size() : 0;
  }

protected:
  virtual void DeclareParameters() override;

private:
  /*! \brief the index of the material of name, throws if there is none */
  uint32_t MaterialIndex(const std::string& name,
                         const std::string& where);
  /*! \brief the triangles of each body, in the coordinates of its origin */
  void BodyTriangles();
  /*! \brief the poses of the bodies now, in the inertial frame */
  void BodyPoses(std::vector<shift::core::sph::BodyPose>& poses);
  /*! \brief the solver, at the first accepted step */
  void Start();
  /*! \brief write the grains to save once the step reaches save_at_ */
  void SaveIfDue(double t);
  /*! \brief the bodies' loads from the impulses over duration */
  void UpdateBodyLoads(double duration);

  std::string device_ = "gpu";
  double margin_ = 0.0;
  int max_neighbors_ = 16;
  int max_triangles_ = 24;
  double max_speed_ = 1.0;
  double step_fraction_ = 0.2;
  double fixed_step_ = 0.0;
  std::string save_;
  double save_time_ = 0.0;
  std::string contact_method_ = "smc";
  int nsc_iterations_ = 100;
  std::string nsc_solver_ = "apgd";
  double nsc_relaxation_ = 0.0;
  double recovery_speed_ = 0.01;
  double min_bounce_speed_ = 0.15;
  double nsc_step_ = 0.0; //!< the NSC step, from Load
  parameter::Array<DemMaterial> materials_parameter_;
  parameter::Array<DemMaterialPair> pairs_parameter_;
  parameter::Array<DemGrain> grains_parameter_;
  parameter::Array<DemFill> fill_parameter_;
  parameter::Array<DemWall> walls_parameter_;
  parameter::Array<DemBody> bodies_parameter_;

  shift::core::dem::Settings settings_;
  std::vector<shift::core::dem::Material> materials_;
  std::vector<shift::core::dem::Template> templates_;
  std::vector<std::string> grain_names_;
  shift::core::dem::State state_;
  shift::core::dem::State initial_state_; //!< as the fill placed them
  /* the device outlives the solver on it, which is destroyed first */
  std::shared_ptr<shift::core::gpu::Device> device_ptr_;
  std::unique_ptr<shift::core::dem::Solver> solver_;
  double step_ = 0.0;      //!< the substep
  double time_ = 0.0;      //!< reached by the grains
  double last_time_ = 0.0; //!< of the last accepted step
  double path_time_ = 0.0; //!< of last_poses_: when the grains last advanced
  double pending_ = 0.0;   //!< time not yet advanced, less than a substep
  bool started_ = false;
  double save_at_ = 0.0; //!< the time to save the grains at
  bool saved_ = false;

  std::vector<body::Base*> bodies_;
  std::vector<shift::core::dem::Body> bodies_dem_;
  std::vector<int> load_indices_;
  std::vector<ksl_coscrew_t> body_loads_;
  std::vector<ksl_coscrew_t> body_impulses_;
  /* NSC: the impulses of the grains not yet delivered by the loads, which
  act until the next step of the grains, however long it is */
  std::vector<ksl_coscrew_t> owed_impulses_;
  std::vector<shift::core::sph::BodyPose> last_poses_;

  static bool registered_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
