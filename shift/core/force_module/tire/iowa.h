#ifndef SHIFT_FORCEMODULE_TIRE_IOWA_H_
#define SHIFT_FORCEMODULE_TIRE_IOWA_H_

#include <string>

#include "shift/core/force_module/soil.h"
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
  area \f$a_s\f$: the area of the circular segment of half angle \f$\theta\f$,
  \f$r^2 (\theta - \frac{1}{2} \sin 2\theta) = a_s\f$, gives
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

*Soft soil.* With a soil (force_module::Soil), the wheel is rigid and
\f$\delta\f$ is its sinkage \f$z\f$. The load and the motion resistance follow
from Bekker's pressure-sinkage law \f$p = (k_c / b + k_\phi) z^n\f$ under a rigid
wheel of diameter \f$d = 2r\f$ and width \f$b\f$ (Bekker, 1969; Wong, Theory of
Ground Vehicles, eqs. 2.62 and 2.65), with the damper as above:

\f[
\begin{aligned}
f_n &= \max\left(\frac{3 - n}{3} \, b \left(\frac{k_c}{b} + k_\phi\right)
  \sqrt{d} \, z^{n + 1/2} + c \, v_z, \; 0\right) \\
r_c &= b \left(\frac{k_c}{b} + k_\phi\right) \frac{z^{n + 1}}{n + 1}
\end{aligned}
\f]

The compaction resistance \f$r_c\f$ takes the place of the rolling resistance
\f$c_r f_n\f$ below. This is the source's soft soil branch, whose soil force
routine is not in the source: only its call, with these inputs and the load and
motion resistance for outputs. The rut the wheel leaves is the terrain's
(terrain::Scm), not the tire's.

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

*Transient slip.* With a relaxation_length \f$l > 0\f$ the tire is the source's
second variant: the slip ratio is a state \f$\kappa'\f$ that lags the slip of
the tread,

\f[
l \, \dot{\kappa}' = -v_p - (|v_x| + v_\epsilon) \, \kappa',
\f]

and settles to \f$-v_p / (|v_x| + v_\epsilon)\f$; it is held within
\f$|\kappa'| \le 1\f$, and out of contact it relaxes to zero. The source solves
this equation, coupled to the spin of the wheel under the brake and drive
torques, in closed form over each fixed step; here the spin is the wheel's
joint and the integrator solves both. At rest the tread deflects like a
bristle instead of the slip becoming stiff, so a wheel held by its brake holds
the vehicle on a slope, creeping at \f$v_\epsilon \kappa'\f$. The source's
flags for a locked or stopped wheel, kept between steps, are left out: the
brake's friction on the joint locks the wheel, and the bristle stops it. The
forces are then those of the second variant: the longitudinal force takes
the friction first, \f$|f_x| \le f_m\f$, and the lateral force what is left,
\f$|f_y| \le \sqrt{f_m^2 - f_x^2}\f$, and the aligning moment follows the
limited \f$f_y\f$.

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

*Lateral force.* With the cornering stiffness \f$c_\alpha\f$, \f$f_m = \mu f_n\f$ and
\f$x = c_\alpha |\alpha| / f_m\f$, a cubic that leaves the origin with the slope \f$c_\alpha\f$ and
reaches \f$f_m\f$ with no slope at \f$x = 2.5\f$:

\f[
f_y = -\mathrm{sign}(\alpha) \, f_m \, (x - 0.32 x^2 + 0.032 x^3), \quad x \le 2.5
\f]

and \f$-\mathrm{sign}(\alpha) f_m\f$ beyond.

*Friction ellipse.* If \f$\sqrt{f_x^2 + f_y^2} > f_m\f$, both are scaled down to
\f$f_m\f$ (without a relaxation length; with one, see above).

*Rolling resistance.* A force at the hub, along the heading,
\f$-c_r f_n v_x / (v_\epsilon + |v_x|)\f$ (on soft soil \f$-r_c v_x / (v_\epsilon + |v_x|)\f$).

*Aligning moment.* The lateral force acts behind the point of action, by the
trail \f$t\f$ (aligning_coefficient): \f$m_z = -t f_y\f$, with \f$f_y\f$ before the
ellipse. The component of the load across a cambered wheel, \f$f_n \sin\gamma\f$ with
\f$\sin\gamma = \underline{n} \cdot \underline{a}\f$ for the terrain normal and the
axle, acts behind it likewise, about the direction of the wheel's plane nearest
the normal.

Not modelled: a lateral relaxation length (the source has none) and camber
thrust. The wheel's spin is the wheel's joint.
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
  double relaxation_length = 0.0; //!< longitudinal, m; 0: no transient slip
  double width = 0.235;           //!< for the soft soil, m
  SoilProperties soil;            //!< see ResolveSoil; used if is_soft()

  /*! \brief sets soil from the "soil" of the model's JSON, if one is given;
   * the tire is then on soft soil */
  void ResolveSoil(const std::string& owner) {
    soft_ = soil_.is_given();
    if(soft_) {
      soil = soil_.Resolve(owner);
    }
  }

  /*! \brief whether the tire is on soft soil (a soil was given) */
  bool is_soft() const {
    return soft_;
  }

  void Prepare() override;

  /*! \brief one state, the relaxed slip ratio, with a relaxation length */
  unsigned int get_state_count() const override {
    return relaxation_length > 0.0 ? 1 : 0;
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

  /*! \brief the load and the compaction resistance of the rigid wheel sunk
   * by z in the soil, before the damping (Bekker's closed form) */
  void SoilForces(double z, double* load, double* resistance) const;

  /*! \brief the size of the lateral force at the slip angle \f$\alpha\f$ under
   * the friction limit \f$f_m\f$ (before the friction ellipse) */
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
  Soil soil_;
  bool soft_ = false;
  double slip_ = 0.0, slip_angle_ = 0.0;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
