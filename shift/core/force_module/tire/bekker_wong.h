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

The wheel is a cylinder of radius r and width b. In the frame of its rim,
(f, a, u): a the axle (the hub's y axis), u the direction in the wheel's
plane that points most towards the terrain, and f = a x u the heading (it
lies in the terrain plane). A point of the tread at the lateral offset y and
the angle theta from the lowest point, in front of it, is
  hub + y a + r (sin(theta) f + cos(theta) u).
The wheel's camber is the tilt of its axle out of the terrain plane,
sin(gamma) = a . n, n the terrain normal into the terrain (cos(gamma) is
u . n). The sinkage at an offset y is H(y) = depth + y sin(gamma), where
depth is that of the lowest point of the rim at y = 0 (tire::Contact), and
a point of the tread is at the depth
  H(y) - r cos(gamma) (1 - cos(theta))
below the terrain. The strip at y touches the soil from the entry angle
theta_1(y) = acos(1 - H / (r cos(gamma))) in front of the lowest point to
the exit angle theta_2(y) = -acos(1 - rebound H / (r cos(gamma))) behind it,
and only where H(y) > 0 (a cambered wheel is pressed in on its low side).

*Normal stress.* The pressure follows Bekker's pressure-sinkage law at the
depth z of each point of the tread, with the width of the wheel for b (Wong
and Reece, 1967):
  sigma = (k_c / b + k_phi) z^n,  z = r cos(gamma) (cos(theta) - cos(theta_1))
in front of the angle of greatest stress theta_m = (c1 + c2 s) theta_1, with
the slip s, and behind it the same, mirrored, at the angle theta_1 -
(theta - theta_2) / (theta_m - theta_2) (theta_1 - theta_m). It acts on the
wheel along the radius, -sigma (sin(theta) f + cos(theta) u). It has no
component along the axle, but the radius of a cambered wheel leans out of the
vertical: that is the camber thrust, the load times tan(gamma) without
shear.

*Shear.* The shear stress follows the Mohr-Coulomb law and the
Janosi-Hanamoto curve, tau = (c + sigma tan(phi)) (1 - exp(-j / K)), in the
direction opposite to the sliding of the tread over the soil (Jia, Smith and
Peng, 2012). The sliding velocity is that of the point of the tread, along the
tread (e_t = cos(theta) f - sin(theta) u) and along the axle,
  s_t = (v_f - w_u y) cos(theta) - (v_u + w_f y) sin(theta) + w_a r
  s_l = v_a + w_u r sin(theta) - w_f r cos(theta)
with (v_f, v_a, v_u) the velocity of the hub and (w_f, w_a, w_u) the angular
velocity of the wheel, in the frame (f, a, u). The shear displacement j of a
point of the soil is the sliding of the tread over it since it entered the
soil at the angle theta_1 (with grousers of height h, at acos((r - H /
cos(gamma)) / (r + h))), taking the velocities as constant over the time r
(theta_1 - theta) / V, with the speed V = sqrt(max(|w_a| r, |v_f|)^2 +
slip_speed^2):
  j_t = r / V ((v_f - w_u y) (sin(theta_1) - sin(theta)) +
               (v_u + w_f y) (cos(theta_1) - cos(theta)) +
               w_a r (theta_1 - theta))
  j_l = r / V (v_a (theta_1 - theta) + w_u r (cos(theta) - cos(theta_1)) -
               w_f r (sin(theta_1) - sin(theta)))
and j = |(j_t, j_l)|. For a wheel rolling straight, with the rim speed
V_r = -w_a r, this is Wong's r (theta_1 - theta) - (1 - s)(sin(theta_1) -
sin(theta)) driving (V_r > v_f), slip s = 1 - v_f / V_r, and his skid braking;
at standstill j goes to zero with the velocities instead of dividing by a zero
speed.

*Bulldozing.* A cambered or slipping wheel pushes the soil with the sides of
its tread. The pressure on a side, at the depth h below the terrain, is
Rankine's passive earth pressure (as Jia, Smith and Peng, after Ishigami,
Miwa, Nagatani and Yoshida, 2007),
  p = 2 c tan(45 deg + phi / 2) + unit_weight h tan^2(45 deg + phi / 2),
over the part of the side below the terrain, the circular segment that the
terrain cuts off, and where the side moves into the soil (the velocity of
the side's point along its outward normal is positive). It is fully
mobilized above the speed slip_speed, and smoothly (a cubic) from zero at
rest, so that the force does not jump as the side starts to push. The soil
pushes back along the axle: with the shear, the side force of a wheel
dragged sideways (Ishigami's R_b).

The stresses are integrated by Gauss-Legendre quadrature over the angle in
the two zones of each strip (stations), over the width (strips, over the
part of it in the soil), and over the segment of each side. They give the
force and moment on the wheel about the contact point: the drawbar pull F_x,
the side force F_y, the load F_z, and the driving torque, the sum of r times
the shear force along the tread. The load is clipped at zero after the
damping on the normal velocity of the contact point, as in Fiala. Axes as in
tire::Contact (SAE, z down; F_z < 0 pushes the wheel up).

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
