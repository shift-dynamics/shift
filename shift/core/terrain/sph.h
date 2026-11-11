#ifndef SHIFT_TERRAIN_SPH_H_
#define SHIFT_TERRAIN_SPH_H_

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/sph/sph.h"
#include "shift/core/terrain/basic.h"

namespace shift {
namespace core {
namespace gpu {
class Device;
}
namespace body {
class Base;
}
namespace collision_geometry {
class Base;
}

namespace terrain {

/*!
@brief Soil as an elastoplastic continuum carried by particles (SPH, see
shift/core/sph/sph.h), over the extent x, y of the terrain frame

The soil fills the extent from the surface down to depth: a column of
particles spacing apart under each spacing by spacing cell, from the surface
of the elevation map (filename) or, for a region, from the surface of the
terrain that holds it (see terrain::Base). Under the soil, marker_layers
layers of fixed wall markers are its floor, and around it marker_layers
columns of markers are its walls, from the floor up to wall_height above
the surface.

Each particle takes the soil at its x, y (see Basic::LocalSoil): of a region,
the soils of the terrain that holds it, changed by the region's soil (see
force_module::Soil::Override). A soil gives the particles their density (its
unit weight over standard gravity), friction_angle, cohesion and the
properties of a continuum, youngs_modulus and poisson_ratio (required) and
dilatancy_angle (optional, 0 by default). Bekker's sinkage properties are not
needed.

The particles move on the GPU (device "gpu", see Context::GetGpuDevice) or the
CPU ("cpu", single threaded, for small soils), in substeps no longer than the
stable step of the soil (see sph::StableStep). After each accepted step of
the integrator (AcceptStep) the soil advances by whole substeps to the time
of the step; what is left of a substep is carried to the next one, so the
soil is less than a substep behind. Gravity is that of the model, in the
terrain frame.

Bodies on the soil (those of force_module::SoilContact, SetBodies) are
markers too: at the first step, the lattice points of the particles' spacing
inside each body's collision geometry within marker_layers spacings of its
surface, and the points where rays along its axes reach its surface, so that
thin parts get markers too. Over each accepted step the markers follow their
body from its pose and velocity at the step before to those at the step, by
cubic Hermite interpolation (sph::Interpolate), and the soil pushes back on
them. The mean of that force over the step, as a load on each body
(get_body_load), acts on the body over the next step: the soil and the
bodies are coupled explicitly, the soil's force a step behind the bodies.

At each output time (UpdateOutput) the particles are read back, and the
surface (SurfaceZ, Query) is the top of the particles of each column, half a
spacing above the highest, interpolated bilinearly between the columns; a
column with no particles is at the floor. Between output times the surface
is that of the last.

\code
"terrain": [{"type": "sph", "name": "pit", "x": [0, 1], "y": [0, 0.5],
             "depth": 0.2, "spacing": 0.02,
             "soil": {"cohesion": 0, "friction_angle": 0.52,
                      "unit_weight": 16677, "youngs_modulus": 2e6,
                      "poisson_ratio": 0.3}}]
\endcode
*/
class Sph : public Basic {
public:
  Sph()
    : Basic() {
    DeclareParameters();
  }
  Sph(std::string name)
    : Basic(name) {
    DeclareParameters();
  }
  virtual ~Sph() override;

  static std::unique_ptr<Sph> Create();

  static bool is_registered() {
    return registered_;
  }

  void Load() override;
  void AcceptStep() override;
  void UpdateOutput() override;

  double SurfaceZ(double x, double y, double& z_x,
                  double& z_y) const override;

  /*! \brief the particles and markers at the last output time, positions
   * from get_origin() in the terrain frame */
  const shift::core::sph::State& get_state() const {
    return state_;
  }
  /*! \brief the point of the terrain frame the positions are from: x[0],
   * y[0] and 0 */
  const std::array<double, 3>& get_origin() const {
    return origin_;
  }
  /*! \brief the number of particles (not markers) */
  size_t get_particle_count() const {
    return particle_count_;
  }
  /*! \brief the settings of the SPH, from the parameters */
  const shift::core::sph::Settings& get_settings() const {
    return settings_;
  }
  /*! \brief the materials, one per soil of get_soils() */
  const std::vector<shift::core::sph::Material>& get_materials() const {
    return materials_;
  }
  /*! \brief the time the particles have reached */
  double get_time() const {
    return time_;
  }

  /*! \brief the bodies on the soil and the collision geometry of each that
   * presses on it, from force_module::SoilContact; before the first step */
  void SetBodies(const std::vector<body::Base*>& bodies,
                 const std::vector<std::vector<collision_geometry::Base*>>&
                   geometry);
  /*! \brief the load of the soil on body b of SetBodies over the last
   * accepted step, in the inertial frame about its origin; zero before */
  const ksl_coscrew_t& get_body_load(size_t b) const {
    return body_loads_[b];
  }
  /*! \brief the impulse of the soil on body b since the first step, in
   * the inertial frame about its origin (N s, N m s) */
  const ksl_coscrew_t& get_body_impulse(size_t b) const {
    return body_impulses_[b];
  }
  /*! \brief the number of markers of body b, from the first step */
  size_t get_body_marker_count(size_t b) const;

protected:
  virtual void DeclareParameters() override;
  bool requires_elevation_map() const override {
    return false;
  }
  bool requires_bekker() const override {
    return false;
  }

private:
  /*! \brief the solver, with gravity in the terrain frame, at the first
   * accepted step, when the pose of the frame is known */
  void Start();
  /*! \brief the surface of each column from state_ */
  void UpdateSurface();
  /*! \brief the markers of each body, into state */
  void SampleBodies(shift::core::sph::State& state);
  /*! \brief the poses of the bodies now, in the frame of the particles */
  void BodyPoses(std::vector<shift::core::sph::BodyPose>& poses);
  /*! \brief the bodies' loads from the impulses over duration */
  void UpdateBodyLoads(double duration);

  double depth_ = 0.0;
  double spacing_ = 0.02;
  double smoothing_ratio_ = 1.3;
  double artificial_viscosity_ = 0.5;
  double cfl_ = 0.25;
  double sound_speed_ = 0.0;
  int marker_layers_ = 3;
  double wall_height_ = std::numeric_limits<double>::quiet_NaN();
  std::string device_ = "gpu";

  shift::core::sph::Settings settings_;
  std::vector<shift::core::sph::Material> materials_;
  shift::core::sph::State state_;
  /* the device outlives the solver on it, which is destroyed first */
  std::shared_ptr<shift::core::gpu::Device> device_ptr_;
  std::unique_ptr<shift::core::sph::Solver> solver_;
  std::array<double, 3> origin_ = {0.0, 0.0, 0.0};
  size_t particle_count_ = 0;
  double step_ = 0.0;     //!< the substep
  double time_ = 0.0;     //!< reached by the particles
  double last_time_ = 0.0; //!< of the last accepted step
  double pending_ = 0.0;  //!< time not yet advanced, less than a substep
  bool started_ = false;

  std::vector<body::Base*> bodies_;
  std::vector<std::vector<collision_geometry::Base*>> body_geometry_;
  std::vector<int> load_indices_;
  std::vector<ksl_coscrew_t> body_loads_;
  std::vector<ksl_coscrew_t> body_impulses_;
  std::vector<uint32_t> body_offsets_;
  std::vector<shift::core::sph::BodyPose> last_poses_;

  /* the columns: columns_x_ by columns_y_, x fastest, their floor and their
  surface (z down, terrain frame) */
  long columns_x_ = 0;
  long columns_y_ = 0;
  std::vector<double> floor_;
  std::vector<double> surface_;

  static bool registered_;
};

} // namespace terrain
} // namespace core
} // namespace shift

#endif
