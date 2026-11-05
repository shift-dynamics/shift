#ifndef SHIFT_FORCEMODULE_TIRE_BEKKER_WONG_H_
#define SHIFT_FORCEMODULE_TIRE_BEKKER_WONG_H_

#include <vector>

#include "shift/core/force_module/soil.h"
#include "shift/core/force_module/tire/model.h"

namespace shift {
namespace core {
namespace force_module {
namespace tire {

/*! \brief parameters of a rigid wheel on soil, see BekkerWong */
struct BekkerWongParameters {
  double radius = 0.344;
  double width = 0.235;
  double vertical_damping = 0.0;
  SoilProperties soil;
  double c1 = 0.4; //!< theta_m = (c1 + c2 slip) theta_1
  double c2 = 0.3;
  double rebound = 0.0;        //!< rear sinkage over the front sinkage
  double grouser_height = 0.0; //!< of lugs, for the shear
  int stations = 16;           //!< per zone, front and rear of theta_m
  int strips = 5;              //!< across the width of a cambered wheel
  bool bulldozing = true;      //!< soil pushed by the sides of the wheel
  double slip_speed = 0.01;    //!< m/s, smooths the slip at standstill
};

/*!
@brief a rigid wheel on soft soil: the Bekker-Wong (Wong-Reece) model, with
camber, slip angle and the bulldozing of the soil by the sides of the wheel

The wheel is a cylinder of radius \f$r\f$ and width \f$b\f$. In the frame of its
rim, \f$(\underline{f}, \underline{a}, \underline{u})\f$: \f$\underline{a}\f$ the
axle (the hub's y axis), \f$\underline{u}\f$ the direction in the wheel's plane
that points most towards the terrain, and
\f$\underline{f} = \underline{a} \times \underline{u}\f$ the heading (it
lies in the terrain plane). A point of the tread at the lateral offset \f$y\f$ and
the angle \f$\theta\f$ from the lowest point, in front of it, is

\f[
\underline{p}_{hub} + y \underline{a} + r (\sin\theta \, \underline{f} +
  \cos\theta \, \underline{u}).
\f]

The wheel's camber is the tilt of its axle out of the terrain plane,
\f$\sin\gamma = \underline{a} \cdot \underline{n}\f$, \f$\underline{n}\f$ the
terrain normal into the terrain (\f$\cos\gamma\f$ is
\f$\underline{u} \cdot \underline{n}\f$). The sinkage at an offset \f$y\f$ is
\f$\eta(y) = \mathrm{depth} + y \sin\gamma\f$, where depth is that of the lowest
point of the rim at \f$y = 0\f$ (tire::Contact), and a point of the tread is at
the depth

\f[
\eta(y) - r \cos\gamma \, (1 - \cos\theta)
\f]

below the terrain. The strip at \f$y\f$ touches the soil from the entry angle
\f$\theta_1(y) = \arccos\left(1 - \frac{\eta}{r \cos\gamma}\right)\f$ in front of
the lowest point to the exit angle
\f$\theta_2(y) = -\arccos\left(1 - \frac{\mathrm{rebound} \, \eta}{r \cos\gamma}\right)\f$
behind it, and only where \f$\eta(y) > 0\f$ (a cambered wheel is pressed in on its
low side).

*Normal stress.* The pressure follows Bekker's pressure-sinkage law at the
depth z of each point of the tread, with the width of the wheel for \f$b\f$ (Wong
and Reece, 1967):

\f[
\sigma = \left(\frac{k_c}{b} + k_\phi\right) z^n, \quad
z = r \cos\gamma \, (\cos\theta - \cos\theta_1)
\f]

in front of the angle of greatest stress \f$\theta_m = (c_1 + c_2 s) \theta_1\f$,
with the slip \f$s\f$, and behind it the same, mirrored, at the angle
\f$\theta_1 - \frac{\theta - \theta_2}{\theta_m - \theta_2} (\theta_1 - \theta_m)\f$.
It acts on the wheel along the radius,
\f$-\sigma (\sin\theta \, \underline{f} + \cos\theta \, \underline{u})\f$. It
has no component along the axle, but the radius of a cambered wheel leans out
of the vertical: that is the camber thrust, the load times \f$\tan\gamma\f$
without shear.

*Shear.* The shear stress follows the Mohr-Coulomb law and the
Janosi-Hanamoto curve, \f$\tau = (c + \sigma \tan\phi) (1 - e^{-j / k_j})\f$, in
the direction opposite to the sliding of the tread over the soil (Jia, Smith
and Peng, 2012). The sliding velocity is that of the point of the tread, along
the tread (\f$\underline{e}_t = \cos\theta \, \underline{f} - \sin\theta \,
\underline{u}\f$) and along the axle,

\f[
\begin{aligned}
s_t &= (v_f - \omega_u y) \cos\theta - (v_u + \omega_f y) \sin\theta +
  \omega_a r \\
s_l &= v_a + \omega_u r \sin\theta - \omega_f r \cos\theta
\end{aligned}
\f]

with \f$(v_f, v_a, v_u)\f$ the velocity of the hub and
\f$(\omega_f, \omega_a, \omega_u)\f$ the angular velocity of the wheel, in the
frame \f$(\underline{f}, \underline{a}, \underline{u})\f$. The shear displacement
\f$j\f$ of a point of the soil is the sliding of the tread over it since it
entered the soil at the angle \f$\theta_1\f$ (with grousers of height \f$h\f$, at
\f$\arccos\frac{r - \eta / \cos\gamma}{r + h}\f$), taking the velocities as
constant over the time \f$r (\theta_1 - \theta) / \bar{v}\f$, with the speed
\f$\bar{v} = \sqrt{\max(|\omega_a| r, |v_f|)^2 + \mathrm{slip\_speed}^2}\f$:

\f[
\begin{aligned}
j_t &= \frac{r}{\bar{v}} \Big( (v_f - \omega_u y) (\sin\theta_1 - \sin\theta) +
  (v_u + \omega_f y) (\cos\theta_1 - \cos\theta) +
  \omega_a r (\theta_1 - \theta) \Big) \\
j_l &= \frac{r}{\bar{v}} \Big( v_a (\theta_1 - \theta) +
  \omega_u r (\cos\theta - \cos\theta_1) -
  \omega_f r (\sin\theta_1 - \sin\theta) \Big)
\end{aligned}
\f]

and \f$j = |(j_t, j_l)|\f$. For a wheel rolling straight, with the rim speed
\f$v_r = -\omega_a r\f$, this is Wong's
\f$r (\theta_1 - \theta) - (1 - s)(\sin\theta_1 - \sin\theta)\f$ driving
(\f$v_r > v_f\f$), slip \f$s = 1 - v_f / v_r\f$, and his skid braking;
at standstill \f$j\f$ goes to zero with the velocities instead of dividing by a zero
speed.

*Bulldozing.* A cambered or slipping wheel pushes the soil with the sides of
its tread. The pressure on a side, at the depth h below the terrain, is
Rankine's passive earth pressure (as Jia, Smith and Peng, after Ishigami,
Miwa, Nagatani and Yoshida, 2007),

\f[
p = 2 c \tan\left(45^\circ + \frac{\phi}{2}\right) +
  \gamma h \tan^2\left(45^\circ + \frac{\phi}{2}\right),
\f]

with the unit_weight \f$\gamma\f$, over the part of the side below the terrain, the circular segment that the
terrain cuts off, and where the side moves into the soil (the velocity of
the side's point along its outward normal is positive). It is fully
mobilized above the speed slip_speed, and smoothly (a cubic) from zero at
rest, so that the force does not jump as the side starts to push. The soil
pushes back along the axle: with the shear, the side force of a wheel
dragged sideways (Ishigami's R_b).

The stresses are integrated by Gauss-Legendre quadrature over the angle in
the two zones of each strip (stations), over the width (strips, over the
part of it in the soil), and over the segment of each side. They give the
force and moment on the wheel about the contact point: the drawbar pull \f$f_x\f$,
the side force \f$f_y\f$, the load \f$f_z\f$, and the driving torque, the sum of r times
the shear force along the tread. The load is clipped at zero after the
damping on the normal velocity of the contact point, as in Fiala. Axes as in
tire::Contact (SAE, z down; \f$f_z < 0\f$ pushes the wheel up).

Not modelled: the longitudinal bulldozing of the soil piled up in front of
the wheel (Hegedus), the shear on the sides, and the rut left by a wheel
for the next (see the deformable terrain on the roadmap).

References: M.G. Bekker, Introduction to Terrain-Vehicle Systems (1969);
J.Y. Wong and A.R. Reece, J. Terramechanics 4(1), 4(2) (1967); Z. Janosi and
B. Hanamoto, 1st Int. Conf. Soil-Vehicle Systems (1961); G. Ishigami, A.
Miwa, K. Nagatani and K. Yoshida, J. Field Robotics 24(3), 233-250 (2007);
Z. Jia, W. Smith and H. Peng, Robotica 30, 491-503 (2012).
*/
class BekkerWong : public Model {
public:
  explicit BekkerWong(const BekkerWongParameters& parameters);

  unsigned int get_state_count() const override {
    return 0;
  }

  double get_unloaded_radius() const override {
    return p_.radius;
  }

  double get_half_width() const override {
    return 0.5 * p_.width;
  }

  void Evaluate(const Contact& contact, const double* states,
                double* state_rates, ksl_coscrew_t* force_w) override;

  /*! \brief the normal stress at the angle theta of the strip at the middle
   * of the width (of the last Evaluate's sinkage and slip) */
  double NormalStress(double theta) const;

  /*! \brief angles of the strip at the middle of the width */
  double get_entry_angle() const {
    return theta_1_;
  }
  double get_exit_angle() const {
    return theta_2_;
  }
  double get_max_stress_angle() const {
    return theta_m_;
  }
  double get_slip() const {
    return slip_;
  }
  /*! \brief the torque of the shear stress about the axle, on the wheel,
   * the sum of r times the shear force along the tread: positive when the
   * soil resists a driving wheel */
  double get_shear_torque() const {
    return shear_torque_;
  }
  /*! \brief tilt of the axle out of the terrain plane, asin(a . n),
   * positive when the +y side of the wheel is deeper in the soil */
  double get_camber() const {
    return camber_;
  }
  /*! \brief the force of the pressure on the sides of the wheel along the
   * axle, positive along it */
  double get_bulldozing_force() const {
    return bulldozing_force_;
  }

private:
  BekkerWongParameters p_;
  std::vector<double> nodes_, weights_;             //!< along the rim
  std::vector<double> strip_nodes_, strip_weights_; //!< across the width
  std::vector<double> side_nodes_, side_weights_;   //!< on a side
  double theta_1_ = 0.0, theta_2_ = 0.0, theta_m_ = 0.0, slip_ = 0.0;
  double shear_torque_ = 0.0, camber_ = 0.0, bulldozing_force_ = 0.0;
  double effective_radius_ = 0.0; //!< r cos(gamma), of the last Evaluate
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
