#ifndef SHIFT_CORE_FORCE_MODULE_THRUST_COLLAR_H_
#define SHIFT_CORE_FORCE_MODULE_THRUST_COLLAR_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/core/force_module/base.h"
#include "shift/core/force_module/table.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {

/*!
@brief the axial contact of a collar and a shoulder or washer: the oil film
between their faces, the contact of their surfaces, and the friction

The collar and the shoulder are two bodies, each with a frame on its face, on
the axis. The z axis of the first frame is the axis \f$\underline{a}\f$, and the
faces are flat rings of inner radius \f$a\f$ and outer radius \f$b\f$, square to
it. The gap between them is \f$h = (\underline{p}_j - \underline{p}_i) \cdot
\underline{a}\f$, so the faces touch where the frames' origins do. With an
end_play \f$e\f$ the collar runs between two shoulders, and there is a second
gap \f$e - h\f$, with its own film and contact.

Surfaces of roughness \f$\sigma\f$ never close the film altogether, whose
thickness is

\f[
h_f = \sigma + \frac{1}{2} \left(h + \sqrt{h^2 + \sigma^2}\right).
\f]

*Squeeze.* Oil pressed out of a closing gap carries a load, whatever the
rotation. The Reynolds equation for a flat ring gives the force, with the oil of
viscosity \f$\mu\f$ and the closing speed \f$v = -\dot{h}\f$,

\f[
f_s = c \, v, \quad
c = \frac{3 \pi \mu}{2 h_f^3} \left(b^4 - a^4 -
  \frac{(b^2 - a^2)^2}{\ln(b / a)}\right)
\f]

(for a disc, \f$a = 0\f$, Stefan's \f$3 \pi \mu b^4 / (2 h_f^3)\f$). The film
cavitates rather than pull the faces together, so an opening gap has no force,
\f$v\f$ coming in smoothly over slip_speed. As \f$h_f\f$ falls \f$c\f$ becomes far
too great for the steps of an integrator (a time constant \f$m / c\f$ for the mass
\f$m\f$), so that \f$c\f$ levels off at film_damping_limit \f$c_m\f$, as
\f$c c_m / (c + c_m)\f$.

*Lift.* Flat parallel faces carry no steady load: the oil only passes through.
Faces with pads, each rising by the taper \f$t\f$ across it, drag oil into a
narrowing wedge. Each of \f$n\f$ pads of length \f$l = 2 \pi r_m / n\f$ along the
mean radius \f$r_m = (a + b) / 2\f$ and width \f$w = b - a\f$ then carries, by the
Reynolds equation of a long plane pad, with the speed
\f$u = r_m |\omega|\f$ and \f$k = t / h_f\f$,

\f[
f_l = n \frac{6 \mu u l^2 w}{h_f^2 k^2} \left(\ln(1 + k) -
  \frac{2 k}{2 + k}\right).
\f]

The oil lost past the sides of a pad is left out, so this is the upper limit of
the load, and the module does not tell one direction of rotation from the other,
as for pivoted pads.

*Contact.* The surfaces carry what the film does not. With the stiffness \f$k_c\f$
and the damping \f$c_c\f$,

\f[
f_c = \max\left(0, \; k_c \rho(-h) + c_c \rho'(-h) \, v\right),
\f]

\f$\rho\f$ a ramp from zero at \f$h = \sigma\f$ to \f$-h\f$ at \f$h = -\sigma\f$.

*Friction.* With \f$\omega\f$ the speed of the second body relative to the first
about the axis, the torque on the second is

\f[
m = -\frac{\pi \mu (b^4 - a^4)}{2 (h_f + t / 2)} \, \omega -
  \mu_b f_c r_e \frac{\omega}{\sqrt{\omega^2 + \omega_s^2}}, \quad
r_e = \frac{2}{3} \frac{b^3 - a^3}{b^2 - a^2},
\f]

the shear of the oil (Petroff's) and the friction of the surfaces, under an even
pressure, with \f$\omega_s\f$ the slip_rate, so that the friction torque is
continuous. A collar held by its friction therefore creeps slowly.

The module acts along the axis and about it only: the tilt of the faces, and the
heating of the oil, are not modelled.

Outputs: gap, film_force, contact_force and friction_torque.

JSON (SI units):
\code
{"type": "thrust_collar", "name": "collar",
 "frames": [["shaft", "collar_face"], ["housing", "shoulder_face"]],
 "inner_radius": 0.03, "outer_radius": 0.06, "viscosity": 0.05,
 "pads": 6, "taper": 3e-5}
\endcode
*/
class ThrustCollar : public Base {
public:
  ThrustCollar()
    : Base() {
    DeclareParameters();
  }

  ThrustCollar(std::string name)
    : Base(name) {
    DeclareParameters();
  }

  virtual ~ThrustCollar() override {}

  static std::unique_ptr<ThrustCollar> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the thickness \f$h_f\f$ of the film at the gap \f$h\f$ */
  double FilmThickness(double gap) const;

  /*! \brief the load \f$f_l\f$ of the pads at the film thickness \f$h_f\f$ and
   * the speed \f$\omega\f$ */
  double PadLoad(double film, double speed) const;

  /*! \brief the coefficient \f$c\f$ of the squeeze of the film \f$h_f\f$, before
   * the limit */
  double SqueezeDamping(double film) const;

  double get_gap() const {
    return gap_;
  }
  double get_film_force() const {
    return film_force_;
  }
  double get_contact_force() const {
    return contact_force_;
  }
  double get_friction_torque() const {
    return friction_torque_;
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
  /*! \brief the force on the second body along the axis from one gap, closing
   * at the speed v, and the torque about the axis from its friction */
  void Face(double gap, double v, double omega, double& film, double& contact,
            double& torque) const;
  void Compute();
  void Apply();

  static bool registered_;

  std::vector<std::vector<std::string>> frame_names_;
  double inner_radius_ = 0.0;
  double outer_radius_ = 0.0;
  double viscosity_ = 0.0;
  double end_play_ = 0.0;
  double roughness_ = 1e-6;
  int pads_ = 0;
  double taper_ = 0.0;
  double contact_stiffness_ = 1e9;
  double contact_damping_ = 1000.0;
  double friction_ = 0.1;
  double film_damping_limit_ = 1e5;
  double slip_speed_ = 1e-6;
  double slip_rate_ = 0.1;

  Attachment attachments_[2];
  double squeeze_factor_ = 0.0; //!< c h_f^3
  double shear_factor_ = 0.0;   //!< the shear torque per speed, times film
  double friction_radius_ = 0.0;
  ksl_SE3_t poses_[2];
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  double gap_ = 0.0, film_force_ = 0.0, contact_force_ = 0.0;
  double friction_torque_ = 0.0;
  double axial_force_ = 0.0; //!< on the second body, along the axis
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
