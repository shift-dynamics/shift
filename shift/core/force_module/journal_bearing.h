#ifndef SHIFT_CORE_FORCE_MODULE_JOURNAL_BEARING_H_
#define SHIFT_CORE_FORCE_MODULE_JOURNAL_BEARING_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {

/*!
@brief the oil film of a plain journal bearing: the force of the film on the
shaft from the position and motion of the shaft in the bearing, and the
friction torque

The journal and the bearing are two bodies, each with a frame at the center of
its circle, whose z axis is the axis of the bearing (the axes are parallel).
The shaft runs in the clearance c (the radial clearance, bearing radius less
journal radius) with the eccentricity e = (journal center - bearing center)
perpendicular to the axis, the eccentricity ratio epsilon = |e| / c and the
film thickness h = c (1 + epsilon cos(theta)), with theta measured round the
bearing from the thickest part of the film.

The pressure in the film follows from the Reynolds equation of a narrow
(short) bearing, in which the flow along the axis dominates the flow round the
bearing (Ocvirk's short bearing theory; for a ratio of length to diameter up to
about 0.5, and as an approximation to about 1). With the oil of viscosity mu,
the rotation speeds omega_j and omega_b of the journal and the bearing about the
axis, the rate epsilon' of the eccentricity ratio and the velocity of the
journal center across the line of centers e_t = e . u_2, with u_1 along the
line of centers, u_2 = a x u_1 across it (a the axis):

    p = 3 mu (L^2/4 - z^2) (s sin(theta) - q cos(theta)) / (c^2 (1 + epsilon cos(theta))^3)

with s = epsilon (omega_j + omega_b) - 2 e_t / c, which is epsilon times the
effective rotation omega_j + omega_b - 2 phi' of the oil about the line of
centers (phi' the speed of the line of centers about the axis), q = 2 epsilon'
(the squeeze), z along the axis from the middle of the bearing and L its length.
Where the pressure would be negative the oil cavitates and the pressure is
zero (the "pi film" of a half Sommerfeld solution, with the film from
theta_0 = atan2(q, s) for half a turn round the bearing). The force on the
journal is the pressure integrated over it:

    F = (mu R L^3 / (2 c^2)) (I_1 u_1 + I_2 u_2),
    I_1 = integral of (s sin(theta) - q cos(theta)) cos(theta) / (1 + epsilon cos(theta))^3,
    I_2 = integral of (s sin(theta) - q cos(theta)) sin(theta) / (1 + epsilon cos(theta))^3,

over the film, which the module integrates by quadrature (with Sommerfeld's
substitution, which makes the integrands smooth even for large epsilon). In
steady running (epsilon' = 0, e_t = 0) these are the classical results
    F . u_1 = - mu R L^3 omega epsilon^2 / (c^2 (1 - epsilon^2)^2) (the film
pushes the journal back to the center),
    F . u_2 = mu R L^3 omega pi epsilon / (4 c^2 (1 - epsilon^2)^(3/2)),
whose ratio is the attitude angle of the short bearing, tan(phi) = pi
sqrt(1 - epsilon^2) / (4 epsilon), and whose size is the load that the bearing
carries, W = mu R L^3 omega epsilon sqrt(16 epsilon^2 + pi^2 (1 - epsilon^2))
/ (4 c^2 (1 - epsilon^2)^2). The component across the line of centers is the
cross-coupling of the film, which drives the oil whirl of lightly loaded
shafts; the module has it, and the damping of the squeeze, in its force.

The friction torque on the journal is that of the shear of the oil and of its
pressure gradient (Petroff's formula with the eccentricity, and the work of the
pressure force):

    T = -2 pi mu R^3 L (omega_j - omega_b) / (c sqrt(1 - epsilon^2)) + (c epsilon / 2) F . (-u_2),

where the shear of the oil is taken round the whole bearing, the cavitated
half filled with oil for the shear.

The eccentricity ratio is limited to 0.995 in the film (smoothly beyond 0.9),
where the film's force is large but finite; give wall_stiffness for contact of
the journal with the bearing at e > c.

Not in the module: the inertia of the oil, thermal effects (the viscosity can be
changed as the model runs, see the schedule controller), misalignment, the
grooves and the supply pressure of the oil, a bearing that is not narrow (see
the roadmap: Booker's mobility method gives finite bearings), and the
elasticity of the shaft and the bearing.

JSON:
\code
{"type": "journal_bearing", "name": "main_bearing",
 "journal": ["shaft", "origin"], "bearing": ["fixed", "origin"],
 "radius": 0.025, "length": 0.025, "clearance": 5e-5, "viscosity": 0.01}
\endcode

- journal, bearing: [body, frame] of each; the frames are at the centers of
  the journal and of the bearing, the z axes along the axis ("fixed" is the
  fixed body).
- radius: of the journal; length: of the bearing; clearance: the radial
  clearance; viscosity: of the oil (Pa s, for SI).
- wall_stiffness (optional, default 0): stiffness of the contact when the
  journal touches the bearing, e > c; zero for none.

Outputs: eccentricity_ratio, min_film_thickness (c (1 - epsilon), negative
when the journal is in contact), peak_pressure, friction_torque (on the
journal), side_flow (of oil out of the ends of the bearing), radial_force and
tangential_force (F . u_1, F . u_2), and load (|F|).
*/
class JournalBearing : public Base {
public:
  JournalBearing()
    : Base() {
    Initialize();
  }
  JournalBearing(std::string name)
    : Base(name) {
    Initialize();
  }
  virtual ~JournalBearing() override {}

  static std::unique_ptr<JournalBearing> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  /*! \brief the integrals I_1 and I_2 over the film (see above) for the
   * eccentricity ratio epsilon, s and q */
  static void FilmIntegrals(double epsilon, double s, double q, double* i_1,
                            double* i_2);

  double get_eccentricity_ratio() const {
    return eccentricity_ratio_;
  }
  double get_radial_force() const {
    return radial_force_;
  }
  double get_tangential_force() const {
    return tangential_force_;
  }
  double get_friction_torque() const {
    return friction_torque_;
  }
  double get_peak_pressure() const {
    return peak_pressure_;
  }
  double get_side_flow() const {
    return side_flow_;
  }
  double get_min_film_thickness() const {
    return min_film_thickness_;
  }
  /*! \brief the force of the film on the journal in the fixed frame */
  const ksl_vec3_t& get_force() const {
    return force_;
  }

protected:
  void DeclareParameters() override;

private:
  void Initialize() {
    DeclareParameters();
  }

  struct Attachment {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2;
  };

  ksl_SE3_t Pose(const Attachment& attachment) const;
  Attachment Resolve(const std::vector<std::string>& names,
                     const std::string& what) const;
  void Compute(bool apply);

  static bool registered_;

  std::vector<std::string> journal_names_, bearing_names_;
  double radius_ = 0.0;
  double length_ = 0.0;
  double clearance_ = 0.0;
  double viscosity_ = 0.0;
  double wall_stiffness_ = 0.0;

  Attachment journal_, bearing_;

  ksl_vec3_t force_ = {{0.0, 0.0, 0.0}};
  double eccentricity_ratio_ = 0.0, min_film_thickness_ = 0.0;
  double peak_pressure_ = 0.0, friction_torque_ = 0.0, side_flow_ = 0.0;
  double radial_force_ = 0.0, tangential_force_ = 0.0, load_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
