#ifndef SHIFT_FORCEMODULE_TIRE_IOWA_H_
#define SHIFT_FORCEMODULE_TIRE_IOWA_H_

#include "shift/core/force_module/tire/model.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace force_module {
namespace tire {

/*!
@brief a simple, fast tire for long runs and driving simulators over rough
ground: the real-time tire of the University of Iowa's National Advanced
Driving Simulator

The tire finds its contact with uneven ground by slicing the wheel, and its
forces from a few parameters. The contact (tire::Base) takes the wheel as a
disc of radius \f$r\f$ in its plane and samples the terrain under the lower half
of it, at segments + 1 points equally spaced along the heading. Each slice between
two points adds the area of the disc below the terrain, and from them follow:

- the point of action, the centroid of the area, the origin of the w frame;
- the terrain normal, the mean of the normals weighted by the area, its z axis;
- the deflection \f$\delta\f$, that of a wheel on flat ground with the same
  area \f$A\f$: the area of the circular segment of half angle \f$\theta\f$,
  \f$r^2 (\theta - \frac{1}{2} \sin 2\theta) = A\f$, gives
  \f$\delta = r (1 - \cos\theta)\f$.

So the force, its point of action and the normal follow bumps, steps and slopes
smaller than the contact patch. With segments 0 the contact is the plane tangent
to the terrain under the wheel, as for the other models.

Axes as in tire::Contact (SAE, z down, y right; \f$f_z < 0\f$ pushes the wheel up).
With \f$(v_x, v_y, v_z)\f$ the velocity of the hub in the w frame and \f$\omega\f$
the spin about the hub's y axis (negative rolling forward):

*Vertical force.* A spring and a damper, not pulling,

\f[
f_n = \max(k \delta + c \, v_z, \; 0).
\f]

*Slip.* The speed of the tread over the ground, with the rolling radius
\f$r - \delta\f$, is \f$v_p = v_x + \omega (r - \delta)\f$. Both slips fade out
below low_speed \f$v_\epsilon\f$, which keeps the forces smooth as the vehicle
comes to rest and the equations from being stiff there:

\f[
\begin{aligned}
\kappa &= -\frac{v_p}{\max(|\omega| (r - \delta), |v_x|, v_\epsilon)} \,
  \frac{|v_p|}{v_\epsilon + |v_p|}, \quad |\kappa| \le 1 \\
\alpha &= \arctan\frac{|v_y|}{\max(|v_x|, v_\epsilon)} \,
  \frac{v_y}{v_\epsilon + |v_y|}
\end{aligned}
\f]

*Longitudinal force.* \f$f_x = \mathrm{sign}(\kappa) \, \mu_x(|\kappa|) \, f_n\f$,
with the friction coefficient \f$\mu\f$ rising to a peak \f$\mu_p = 1.05 \mu\f$:

\f[
\mu_x(s) = \begin{cases}
  5 \mu_p s & s < 0.2 \\
  \mu_p & 0.2 \le s < 0.25 \\
  \mu_p - (\mu_p - \mu) (s - 0.25) / 0.25 & 0.25 \le s < 0.5 \\
  \mu & s \ge 0.5
\end{cases}
\f]

*Lateral force.* With the cornering stiffness \f$C\f$, \f$F = \mu f_n\f$ and
\f$x = C |\alpha| / F\f$, a cubic that leaves the origin with the slope \f$C\f$ and
reaches \f$F\f$ with no slope at \f$x = 2.5\f$:

\f[
f_y = -\mathrm{sign}(\alpha) \, F \, (x - 0.32 x^2 + 0.032 x^3), \quad x \le 2.5
\f]

and \f$-\mathrm{sign}(\alpha) F\f$ beyond.

*Friction ellipse.* If \f$\sqrt{f_x^2 + f_y^2} > F\f$, both are scaled down to
\f$F\f$.

*Rolling resistance.* A force at the hub, along the heading,
\f$-c_r f_n v_x / (v_\epsilon + |v_x|)\f$.

*Aligning moment.* The lateral force acts behind the point of action, by the
trail \f$t\f$ (aligning_coefficient): \f$m_z = -t f_y\f$, with \f$f_y\f$ before the
ellipse. The component of the load across a cambered wheel, \f$f_n \sin\gamma\f$ with
\f$\sin\gamma = \underline{n} \cdot \underline{a}\f$ for the terrain normal and the
axle, acts behind it likewise, about the direction of the wheel's plane nearest
the normal.

Not modelled: the transient response of the slips (relaxation lengths), camber
thrust, and soft soil (see BekkerWong). The wheel's spin is the wheel's joint.
*/
class Iowa
  : public Model
  , public parameter::List {
public:
  Iowa() {
    DeclareParameters();
  }
  virtual ~Iowa() {}

  double unloaded_radius = 0.344;
  double vertical_stiffness = 310000;
  double vertical_damping = 3100;
  double friction_coefficient = 0.9;
  double rolling_resistance = 0.01;
  double cornering_stiffness = 45836.6236;
  double aligning_coefficient = 0.0;
  int segments = 20;
  double low_speed = 0.0635;

  void Prepare() override;

  unsigned int get_state_count() const override {
    return 0;
  }

  double get_unloaded_radius() const override {
    return unloaded_radius;
  }

  int get_contact_segments() const override {
    return segments;
  }

  void Evaluate(const Contact& contact, const double* states,
                double* state_rates, ksl_coscrew_t* force_w) override;

  /*! \brief the longitudinal friction coefficient at the slip \f$s \ge 0\f$ */
  double LongitudinalFriction(double s) const;

  /*! \brief the size of the lateral force at the slip angle \f$\alpha\f$ under
   * the friction limit \f$F\f$ (before the friction ellipse) */
  double LateralForce(double alpha, double limit) const;

  double get_slip() const {
    return slip_;
  }
  double get_slip_angle() const {
    return slip_angle_;
  }

protected:
  virtual void DeclareParameters() override;

private:
  double slip_ = 0.0, slip_angle_ = 0.0;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
