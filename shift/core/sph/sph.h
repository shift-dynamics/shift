#ifndef SHIFT_SPH_SPH_H_
#define SHIFT_SPH_SPH_H_

#include <array>
#include <cstdint>
#include <vector>

namespace shift {
namespace core {
namespace sph {

/*!
@brief soil as an elastoplastic continuum carried by particles (smoothed
particle hydrodynamics, SPH), after Bui et al. (2008) and Chrono's CRM
terrain

Each particle carries its position, velocity, density, the six components of
its stress (tension positive) and its material. The density follows the
continuity equation. The stress follows the Jaumann rate of Hooke's law,

\f[
\dot{\boldsymbol\sigma} = \boldsymbol\sigma \boldsymbol\omega^T +
  \boldsymbol\omega \boldsymbol\sigma + 2 G \dot{\mathbf e} +
  K \, \mathrm{tr}(\dot{\boldsymbol\epsilon}) \mathbf I
\f]

with the strain rate \f$\dot{\boldsymbol\epsilon}\f$ (deviatoric part
\f$\dot{\mathbf e}\f$) and spin \f$\boldsymbol\omega\f$ of the velocity
gradient, and then is returned to the Drucker-Prager cone

\f[
f = \alpha_\phi I_1 + \sqrt{J_2} - k_c \le 0, \quad
\alpha_\phi = \frac{2 \sin\phi}{\sqrt{3} (3 - \sin\phi)}, \quad
k_c = \frac{6 c \cos\phi}{\sqrt{3} (3 - \sin\phi)}
\f]

the cone through the corners of Mohr and Coulomb's hexagon in triaxial
compression. (Bui et al. match the cone in plane strain, \f$\alpha_\phi =
\tan\phi / \sqrt{9 + 12 \tan^2\phi}\f$, for their 2D problems; in 3D that
cone is nearly the inner one, and a heap of 30 degree sand slumps to about
22 degrees on it, but to 30 on this one.)

with a plastic potential of the dilatancy angle \f$\psi\f$ in place of
\f$\phi\f$ (non-associated flow): a stress outside the cone takes
\f$\Delta\lambda = f / (G + 9 K \alpha_\phi \alpha_\psi)\f$, so that
\f$\sqrt{J_2}\f$ drops by \f$G \Delta\lambda\f$ and \f$I_1\f$ by
\f$9 K \alpha_\psi \Delta\lambda\f$; where that would pass the apex of the
cone, the stress goes to the apex, \f$I_1 = k_c / \alpha_\phi\f$ and no
deviatoric stress (soil in tension falls apart).

The acceleration is that of the stress, with Monaghan's artificial
viscosity, and gravity. The kernel is Wendland's C2, of support 2 h, with
h = smoothing_ratio spacing. Walls are fixed boundary markers, whose stress
is extrapolated from the particles around them (Adami et al. 2012) and whose
velocity is zero (the soil does not slip on them).

Bodies are markers too, fixed in each body (State::AddBodyMarker), moved with
it along its path (SetBodyPaths) at the start of each substep, with its
velocity. The force on a body's marker is the opposite of the forces it puts
on the particles around it; the impulses of the markers over the substeps,
force and moment about the origin of the positions, sum to each body's
(GetImpulses) in a fixed order, the same on every run.

Each substep: sort the particles and markers by the cell of a hashed grid
(cells 2 h wide), then by id, so that every sum runs in the same order on
every run; the markers' stresses; the density and stress rates and the
return to the cone; the acceleration; then the velocity and position
(symplectic Euler). The substep is cfl h / c, with c the speed of
compression waves of the stiffest material, \f$\sqrt{(K + 4 G / 3) / \rho}\f$,
or sound_speed if given.

Positions are float32, relative to an origin chosen by the caller near the
particles; the GPU (GpuSolver) and the CPU (CpuSolver, a reference for
small problems and tests) do the same arithmetic in the same order.
*/

/*! \brief a material of the particles */
struct Material {
  float density = 1700.0f;       //!< kg/m^3, at rest
  float youngs_modulus = 1.0e6f; //!< Pa
  float poisson_ratio = 0.3f;
  float friction_angle = 0.6f;  //!< radians
  float dilatancy_angle = 0.0f; //!< radians
  float cohesion = 0.0f;        //!< Pa
};

struct Settings {
  float spacing = 0.02f;         //!< of the particles at rest, m
  float smoothing_ratio = 1.3f;  //!< h / spacing
  float artificial_viscosity = 0.5f; //!< Monaghan's alpha
  float cfl = 0.25f;             //!< substep over h / c
  float sound_speed = 0.0f;      //!< c, m/s; 0 for that of the materials
  std::array<float, 3> gravity = {0.0f, 0.0f, 9.81f}; //!< m/s^2
};

/*! \brief the particles and markers, one array per field, in the order the
 * solver keeps them (which changes as it sorts them; id is the order they
 * were given in) */
struct State {
  /*! x, y, z (m, from the origin) and mass (kg) */
  std::vector<std::array<float, 4>> position;
  /*! m/s; the fourth component is unused */
  std::vector<std::array<float, 4>> velocity;
  std::vector<float> density;
  /*! xx, yy, zz, xy, yz, zx (Pa, tension positive), then two unused */
  std::vector<std::array<float, 8>> stress;
  std::vector<uint32_t> id;
  /*! the index of the material; for a marker kMarker, plus 1 + the index
   * of its body for a body's */
  std::vector<uint32_t> kind;
  /*! the positions of the bodies' markers in their bodies (m), in the order
   * of their ids from first_body_marker */
  std::vector<std::array<float, 4>> body_local;
  /*! the id of the first body marker; the bodies' markers come last */
  uint32_t first_body_marker = 0;
  /*! body b's markers are body_local[body_offsets[b]] to
   * body_local[body_offsets[b + 1] - 1]; empty without bodies */
  std::vector<uint32_t> body_offsets;

  static const uint32_t kMarker = 0x80000000u;

  /*! \brief the number of bodies */
  size_t body_count() const {
    return body_offsets.empty() ? 0 : body_offsets.size() - 1;
  }

  size_t size() const {
    return position.size();
  }
  void resize(size_t n);
  /*! \brief add a particle of material at x (at rest, with the density of
   * the material and the mass of a cube of spacing) */
  void AddParticle(const std::array<float, 3>& x, uint32_t material,
                   const Material& properties, float spacing);
  /*! \brief add a fixed wall marker at x */
  void AddMarker(const std::array<float, 3>& x, const Material& properties,
                 float spacing);
  /*! \brief add a marker of body (0, 1, ... in order) at local, in the
   * body; after every particle and wall marker. Its position is the body's
   * from the first substep (see Solver::SetBodyPaths) */
  void AddBodyMarker(const std::array<float, 3>& local, uint32_t body,
                     const Material& properties, float spacing);
  /*! \brief the index of the particle or marker of id, or -1 */
  long Find(uint32_t id) const;
};

/*! \brief the pose and velocity of a body in the frame of the positions:
 * the rotation R of the body (columns, R[3 c + r]), the position t and
 * velocity v of its origin and its angular velocity w */
struct BodyPose {
  std::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> t = {0.0, 0.0, 0.0};
  std::array<double, 3> v = {0.0, 0.0, 0.0};
  std::array<double, 3> w = {0.0, 0.0, 0.0};
};

/*! \brief the pose of a body at s from 0 to 1 of an interval of duration
 * from a to b: cubic Hermite in the position and in the rotation vector of
 * R_b R_a^T, from their values and rates at the ends; s outside 0 to 1
 * extrapolates */
BodyPose Interpolate(const BodyPose& a, const BodyPose& b, double duration,
                     double s);

/*! \brief Wendland's C2 kernel in 3D and its derivative over r, of support
 * 2 h */
float Kernel(float r, float h);
float KernelDerivative(float r, float h);

/*! \brief throw unless the bodies' markers of state are laid out as
 * State::AddBodyMarker lays them out */
void CheckBodyMarkers(const State& state);

/*! \brief the substep of the settings and materials */
float StableStep(const Settings& settings,
                 const std::vector<Material>& materials);

/*! \brief the size of the hash table of the grid: a power of 2, at least
 * twice the number of particles and markers */
uint32_t HashTableSize(size_t count);

/*! \brief the constants of each material as the solvers use them: density,
 * shear modulus G, bulk modulus K, sound speed c, then alpha_phi, k_c,
 * alpha_psi of the Drucker-Prager cone and 0 */
std::vector<std::array<float, 8>> MaterialTable(
  const Settings& settings, const std::vector<Material>& materials);

/*! \brief the cell of the grid a point is in, and its hash, as both solvers
 * compute them */
std::array<int32_t, 3> Cell(const std::array<float, 4>& x, float cell);
uint32_t CellHash(const std::array<int32_t, 3>& c, uint32_t mask);

/*!
@brief the solver's interface: CpuSolver and GpuSolver
*/
class Solver {
public:
  virtual ~Solver() {}
  /*! \brief advance by one substep of dt */
  virtual void Step(float dt) = 0;
  /*! \brief advance by n substeps of dt */
  virtual void Steps(float dt, long n) {
    for(long k = 0; k < n; k++) {
      Step(dt);
    }
  }
  /*! \brief advance by duration in equal substeps no longer than
   * StableStep; returns the number of substeps */
  long Advance(double duration);
  /*! \brief the particles and markers now */
  virtual void GetState(State& state) = 0;

  /*! \brief the bodies' path from time t0, at start, to t1, at end (one
   * pose per body): the substeps from t0 on place the markers on it.
   * Without one, the markers stay where they are */
  void SetBodyPaths(const std::vector<BodyPose>& start,
                    const std::vector<BodyPose>& end, double t0, double t1);
  /*! \brief the time of the particles, from 0 or set_time */
  double get_time() const {
    return time_;
  }
  void set_time(double time) {
    time_ = time;
  }
  /*! \brief set the bodies' impulses to zero */
  virtual void ClearImpulses() = 0;
  /*! \brief each body's impulse since ClearImpulses: force (N s) and
   * moment about the origin of the positions (N m s), in the frame of the
   * positions */
  virtual void GetImpulses(std::vector<std::array<double, 6>>& impulses) = 0;

  const Settings& get_settings() const {
    return settings_;
  }

protected:
  Solver(const Settings& settings, const std::vector<Material>& materials,
         size_t bodies)
    : settings_(settings), materials_(materials), bodies_(bodies) {}

  /*! \brief the poses of the bodies at time, on their paths */
  void PosesAt(double time, std::vector<BodyPose>& poses) const;

  Settings settings_;
  std::vector<Material> materials_;
  double time_ = 0.0;
  size_t bodies_ = 0;
  std::vector<BodyPose> path_start_, path_end_;
  double path_t0_ = 0.0, path_t1_ = 0.0;
};

} // namespace sph
} // namespace core
} // namespace shift

#endif
