#ifndef SHIFT_FORCEMODULE_TIRE_IOWA_H_
#define SHIFT_FORCEMODULE_TIRE_IOWA_H_

#include <cmath>
#include <string>
#include <vector>

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

*Soft soil.* On soil, the tire's own (soil) or else that of the terrain under
the contact (terrain::Base::SoilAt, e.g. a surface material's), the tire
deflects in series with the sinkage \f$z\f$ of a rigid wheel in the soil:
both carry the load, and \f$\delta = f_n / k + z\f$. Bekker's
pressure-sinkage law \f$p = (k_c / b + k_\phi) z^n\f$ under a rigid wheel of
diameter \f$d = 2r\f$ and width \f$b\f$ (Bekker, 1969; Wong, Theory of
Ground Vehicles, eqs. 2.62 and 2.65) gives the load of the sinkage and the
compaction resistance, with the damper as above:
\f[
\begin{aligned}
f_n &= \max\left(\frac{3 - n}{3} \, b \left(\frac{k_c}{b} + k_\phi\right)
  \sqrt{d} \, z^{n + 1/2} + c \, v_z, \; 0\right) \\
r_c &= b \left(\frac{k_c}{b} + k_\phi\right) \frac{z^{n + 1}}{n + 1}
\end{aligned}
\f]
The compaction resistance \f$r_c\f$ adds to the rolling resistance
\f$c_r f_n\f$ below. On firm soil the tire takes the deflection and the
resistance is its own, as on rigid ground; on soft soil the wheel sinks. The
source's soft soil branch takes the wheel as rigid, which the series spring
gives as \f$k \to \infty\f$; its soil force routine is not in the source,
only its call, with these inputs and the load and motion resistance for
outputs. The rut the wheel leaves is the terrain's (terrain::Scm), not the
tire's.

*Load sensitivity.* With a load_sensitivity \f$n_\mu\f$ the friction
coefficient falls (for \f$n_\mu < 0\f$) with the load,
\f$\mu (f_n / f_0)^{n_\mu}\f$, and is \f$\mu\f$ at the reference_load
\f$f_0\f$ (B. J. Chan, Development of an Off-Road Capable Tire Model for
Vehicle Dynamics Simulations, Virginia Tech, 2008, eq. 5.13). It takes the
place of \f$\mu\f$ in the forces below.

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

*Saturation.* With saturation "exponential", or "soil" on soil, the forces
rise to the friction with no peak, as measured on loose ground, whose side
force fits \f$A (1 - e^{-B \alpha})\f$ (Schwanghart, 1968, and the NIAE,
1978, in A. S. A. El-Razaz, Lateral Tyre Forces on Off-Road Surfaces,
Leeds, 1988, eq. 2.1; R. W. Allen, T. J. Rosenthal and J. P. Chrstos, A
Vehicle Dynamics Tire Model for Both Pavement and Off-Road Conditions, SAE
970559, 1997, switch the saturation by the surface likewise). With the same
slopes at no slip as above:
\f[
\mu_x(s) = \mu \left(1 - e^{-5 \mu_p s / \mu}\right), \quad
f_y = -\mathrm{sign}(\alpha) \, f_m \left(1 - e^{-x}\right)
\f]

*Friction ellipse.* If \f$\sqrt{f_x^2 + f_y^2} > f_m\f$, both are scaled down to
\f$f_m\f$ (without a relaxation length; with one, see above).

*Rolling resistance.* A force at the hub, along the heading,
\f$-(c_r f_n + r_c) \, v_x / (v_\epsilon + |v_x|)\f$, with the compaction
resistance \f$r_c\f$ on soil.

*Aligning moment.* The lateral force acts behind the point of action, by the
trail \f$t\f$ (aligning_coefficient): \f$m_z = -t f_y\f$, with \f$f_y\f$ before the
ellipse. The component of the load across a cambered wheel, \f$f_n \sin\gamma\f$ with
\f$\sin\gamma = \underline{n} \cdot \underline{a}\f$ for the terrain normal and the
axle, acts behind it likewise, about the direction of the wheel's plane nearest
the normal.

*Combined slip.* With combined_slip, the longitudinal and lateral forces
share the friction as the slip of the tread directs, in place of the friction
ellipse or the longitudinal force first. The slips, each over its value at
the peak, \f$s_x = |\kappa| / 0.2\f$ and \f$s_y = |\alpha| / \alpha_p\f$
with \f$\alpha_p = 2.5 \, f_m / c_\alpha\f$, give the combined slip
\f$s = \sqrt{s_x^2 + s_y^2}\f$; the size of the force is that of each pure
slip at it, \f$f_{x0} = \mu_x(0.2 \, s) \, f_n\f$ and
\f$f_{y0} = f_y(\alpha_p s)\f$ above, weighted by the direction of the slip,

\f[
f = \min\left(\frac{s_x^2 f_{x0} + s_y^2 f_{y0}}{s^2}, \; f_m\right).
\f]

While the tread grips, \f$s \le 1\f$, the force is along
\f$\underline{u}_a = (\mathrm{sign}(\kappa) \, s_x, -\mathrm{sign}(\alpha)
\, s_y)\f$, as the stiffnesses share it; as it slides, it turns to act
against the velocity of the sliding tread,
\f$\underline{u}_s = (\kappa \max(|v_x|, v_\epsilon), -v_y)\f$, fully from
\f$s = 2\f$:

\f[
(f_x, f_y) = f \, \frac{(1 - w) \, \hat{\underline{u}}_a
  + w \, \hat{\underline{u}}_s}{\left|(1 - w) \, \hat{\underline{u}}_a
  + w \, \hat{\underline{u}}_s\right|}, \quad
w = \min(\max(s - 1, 0), 1),
\f]

with the hats unit vectors. With one slip alone the forces are those above. A
wheel spun by the engine through a slide, with a large slip angle, so keeps
most of its lateral force, and a wheel locked by the brake, sliding along
the heading, loses it, as a brush tire whose tread slides (H. B. Pacejka,
Tire and Vehicle Dynamics, 3rd ed., 2012, sec. 3.2). The aligning moment
follows \f$f_y\f$. With a relaxation length, \f$\kappa\f$ is the relaxed slip.

*Loose ground's shape.* With loose_shape \f$(B, C, E)\f$, the loose
ground's forces instead follow the Magic Formula of the combined slip
\f$\sigma = \sqrt{\kappa^2 + \tan^2\alpha}\f$, with the friction for
\f$D\f$,
\f[
f = f_m \sin\left(C \arctan\left(B \sigma - E \left(B \sigma -
  \arctan B \sigma\right)\right)\right),
\f]
against the velocity of the sliding tread, \f$\hat{\underline{u}}_s\f$
above (so \f$\kappa\f$ alone or \f$\alpha\f$ alone gives each pure
force), as the tire of D. Tavernini, E. Velenis, R. Lot and M. Massaro
(The optimality of the handbrake cornering technique, J. Dyn. Sys. Meas.
Control 136, 2014), whose loose surface, matched to a rally driver's runs,
has \f$B = 1.5071\f$, \f$C = 1.0901\f$, \f$E = -1.584\f$: a soft tire
whose force rises to the friction without a peak, at about four fifths of it
at 30 degrees of slip. The cornering stiffness is then
\f$B C f_m\f$.

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
  bool combined_slip = false;     //!< the forces share the friction by slip
  double load_sensitivity = 0.0;   //!< exponent of the friction on the load
  double reference_load = 0.0;     //!< N; the load of friction_coefficient
  std::string saturation = "peak"; //!< "peak", "exponential" or "soil"
  /*! B, C, E of the Magic Formula of the loose ground's forces, D the
  friction; B 0: the exponential */
  std::vector<double> loose_shape = {0.0, 0.0, 0.0};
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

  /*! \brief puts the tire on the soil soil, as if its JSON gave it (for a
   * copy of the tire that stands for it on the ground under it, as the
   * rally driver's slides do) */
  void SetSoil(const SoilProperties& ground) {
    soil = ground;
    soft_ = true;
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

  /*! \brief the friction coefficient under the load normal: \f$\mu\f$, or
   * \f$\mu (f_n / f_0)^{n_\mu}\f$ with a load_sensitivity */
  double Friction(double normal) const;

  /*! \brief the longitudinal friction coefficient at the slip \f$s \ge 0\f$,
   * for the friction_coefficient; loose: the exponential saturation */
  double LongitudinalFriction(double s, bool loose = false) const;

  /*! \brief the load and the compaction resistance of the rigid wheel sunk
   * by z in the soil, before the damping (Bekker's closed form) */
  void SoilForces(double z, double* load, double* resistance) const;
  void SoilForces(const SoilProperties& ground, double z, double* load,
                  double* resistance) const;

  /*! \brief the load of the tire pressed by depth into the soil ground, its
   * deflection in series with the sinkage z of the wheel in the soil, and
   * the soil's compaction resistance, before the damping */
  void SoilLoad(const SoilProperties& ground, double depth, double* load,
                double* sinkage, double* resistance) const;

  /*! \brief the size of the lateral force at the slip angle \f$\alpha\f$ under
   * the friction limit \f$f_m\f$ (before the friction ellipse); loose: the
   * exponential saturation */
  double LateralForce(double alpha, double limit, bool loose = false) const;

  /*! \brief the forces of combined slip (combined_slip) for the slip ratio
   * kappa, the slip angle alpha, the hub's velocity v_x, v_y in the w frame
   * and the load normal; loose: the exponential saturation */
  void CombinedForces(double kappa, double alpha, double v_x, double v_y,
                      double normal, double* f_x, double* f_y,
                      bool loose = false) const;

  /*! \brief the Magic Formula's share of the friction at the slip s of
   * the loose ground's curve, loose_shape */
  double LooseShare(double s) const {
    const double b = loose_shape[0], c = loose_shape[1], e = loose_shape[2];
    const double x = b * s;
    return std::sin(c * std::atan(x - e * (x - std::atan(x))));
  }

  /*! \brief whether the loose ground's forces follow loose_shape */
  bool has_loose_shape() const {
    return loose_shape[0] > 0.0;
  }

  /*! \brief whether the forces saturate exponentially (saturation), on soil
   * or not */
  bool is_loose(bool on_soil) const {
    return saturation == "exponential" || (saturation == "soil" && on_soil);
  }

  /*! \brief the forces of the ground on the tread (\f$f_x, f_y\f$ in the w
   * frame) and the aligning moment, within the friction limit, for the slip
   * ratio kappa, the slip angle alpha, the hub's velocity v_x, v_y in the w
   * frame and the load normal, on soil or not (saturation): what Evaluate
   * applies, before the rolling resistance; by combined_slip, or else clipped
   * to the friction limit */
  void TreadForces(double kappa, double alpha, double v_x, double v_y,
                   double normal, bool on_soil, double* f_x, double* f_y,
                   double* aligning) const;

  /*! \brief the deflection under a steady load, and the force that resists
   * the rolling there: \f$f_n / k\f$ and \f$c_r f_n\f$, and on soft soil
   * (is_soft) also the sinkage z whose Bekker load is \f$f_n\f$ and its
   * compaction resistance \f$r_c\f$ (SoilLoad) */
  void RestingDeflection(double load, double* deflection,
                         double* resistance) const;

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
