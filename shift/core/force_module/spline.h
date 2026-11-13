#ifndef SHIFT_CORE_FORCE_MODULE_SPLINE_H_
#define SHIFT_CORE_FORCE_MODULE_SPLINE_H_

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
@brief a sliding spline: the torque it carries, with its backlash, and the
sliding friction, which grows with that torque, as in a driveshaft's slip spline

The two halves are two bodies, each with a frame on the axis, whose joints leave
them free to turn and slide on it (a prismatic and a revolute joint in series).
The z axis of the first frame is the axis \f$\underline{a}\f$. The teeth are
lumped: the module is not a contact of each tooth.

*The torque.* With \f$\phi\f$ the angle of the second half relative to the first
about the axis, from the start, a state that counts past a full turn, and the
backlash \f$2 \beta\f$, the torque on the second half is

\f[
\tau = -w \left(k \, d + c \, \dot{\phi}\right), \quad
d = \rho(\phi - \beta) - \rho(-\phi - \beta),
\f]

with the torsional_stiffness \f$k\f$, the torsional_damping \f$c\f$, \f$\rho\f$ a
ramp over the smoothing and \f$w\f$ its slope, so that the damper comes in with the
spring. The first half has the opposite torque.

*The load on the teeth.* The torque \f$\tau\f$, at the pitch_radius \f$r\f$ and
pressure_angle \f$\alpha\f$, presses the flanks together with the normal force

\f[
n = \frac{|\tau|}{r \cos\alpha} + n_0,
\f]

with \f$n_0\f$ the preload, of a tight fit or of the weight of the shaft.

*The friction.* The bristles of the LuGre model, as in force_module::Contact, have
the deflection \f$z\f$, a state. With the rate \f$v\f$ of the second half relative
to the first along the axis, the friction force on the second half is

\f[
\dot{z} = v - \frac{\sigma_0 |v|}{g(v)} z, \quad
g(v) = \mu + (\mu_s - \mu) e^{-(v / v_b)^2}, \quad
f = -n \left(\sigma_0 z + \sigma_1 \dot{z}\right) - c_v v,
\f]

with the friction \f$\mu\f$, the static_friction \f$\mu_s\f$, the
stribeck_velocity \f$v_b\f$, the bristle_stiffness \f$\sigma_0\f$ and
bristle_damping \f$\sigma_1\f$, both per unit of \f$n\f$, and the
viscous_friction \f$c_v\f$. So a spline under torque locks against any steady
axial force up to \f$\mu_s n\f$, and slides against \f$\mu n\f$. The first half has
the opposite force.

The rate \f$\sigma_0 |v| / g(v)\f$ at which \f$z\f$ relaxes is large at speed,
and would set the time step of the whole model while the spline slides. Above
\f$1 / t_b\f$, with the bristle_time \f$t_b\f$, the deflection instead relaxes to
the same sliding value at that rate, \f$\dot{z} = (\mathrm{sgn}(v) \, g(v) /
\sigma_0 - z) / t_b\f$, which meets the equation above where they join and leaves
the sliding friction as it is.

Outputs: angle, torque, normal_force, friction_force and stick, the share of
the static friction in use, \f$\sigma_0 |z| / \mu_s\f$.

JSON (SI units, angles in radians):
\code
{"type": "spline", "name": "slip_spline",
 "frames": [["shaft", "spline"], ["yoke", "spline"]],
 "pitch_radius": 0.02, "pressure_angle": 0.5236, "torsional_stiffness": 5e4,
 "backlash": 0.002, "friction": 0.1, "static_friction": 0.12}
\endcode
*/
class Spline : public Base {
public:
  Spline()
    : Base() {
    DeclareParameters();
  }

  Spline(std::string name)
    : Base(name) {
    DeclareParameters();
  }

  virtual ~Spline() override {}

  static std::unique_ptr<Spline> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the normal force \f$n\f$ on the teeth under the torque \f$\tau\f$ */
  double NormalForce(double torque) const;

  double get_angle() const {
    return angle_;
  }
  double get_torque() const {
    return torque_;
  }
  double get_normal_force() const {
    return normal_force_;
  }
  double get_friction() const {
    return friction_force_;
  }
  double get_stick() const {
    return stick_;
  }

protected:
  void DeclareParameters() override;

private:
  struct Attachment {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2; //!< see Context::GetBodyLoadIndex; -2: not yet
  };

  ksl_SE3_t Pose(const Attachment& a) const;
  void Compute(const double* y, double* dy);
  void Apply();

  static bool registered_;

  std::vector<std::vector<std::string>> frame_names_;
  double pitch_radius_ = 0.0;
  double pressure_angle_ = 0.5236;
  double torsional_stiffness_ = 0.0;
  double torsional_damping_ = 10.0;
  double backlash_ = 0.0;
  double smoothing_ = 1e-4;
  double preload_ = 0.0;
  double friction_ = 0.1;
  double static_friction_ = -1.0; //!< negative: the friction
  double viscous_friction_ = 0.0;
  double bristle_stiffness_ = 2e4;
  double bristle_damping_ = 5.0;
  double bristle_time_ = 1e-4;
  double stribeck_velocity_ = 0.01;

  Attachment attachments_[2];
  ksl_SE3_t poses_[2];
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  double angle_ = 0.0, torque_ = 0.0, normal_force_ = 0.0;
  double friction_force_ = 0.0, stick_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
