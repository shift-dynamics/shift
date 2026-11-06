#include "shift/core/force_module/tire/iowa.h"

#include <algorithm>
#include <cmath>

#include "ksl/coscrew.h"
#include "shift/core/exception.h"

void shift::core::force_module::tire::Iowa::DeclareParameters() {
  DeclareParameter("unloaded_radius", &unloaded_radius,
                   "unloaded radius (m); optional, default 0.344", false);
  DeclareParameter("vertical_stiffness", &vertical_stiffness,
                   "vertical stiffness (N/m); optional, default 310000", false);
  DeclareParameter("vertical_damping", &vertical_damping,
                   "vertical damping (N s/m); optional, default 3100", false);
  DeclareParameter("friction_coefficient", &friction_coefficient,
                   "friction coefficient when sliding; optional, default 0.9",
                   false);
  DeclareParameter("rolling_resistance", &rolling_resistance,
                   "rolling resistance force per load; optional, default 0.01",
                   false);
  DeclareParameter("cornering_stiffness", &cornering_stiffness,
                   "cornering stiffness (N/rad); optional, default 45836.6",
                   false);
  DeclareParameter("aligning_coefficient", &aligning_coefficient,
                   "pneumatic trail (m); optional, default 0", false);
  DeclareParameter("segments", &segments,
                   "slices of the wheel for the contact with the terrain, an "
                   "even number so that one point is under the hub; 0 for a "
                   "single point of contact; optional, default 20",
                   false);
  DeclareParameter("low_speed", &low_speed,
                   "speed below which the slips fade out (m/s); optional, "
                   "default 0.0635",
                   false);
}

void shift::core::force_module::tire::Iowa::Prepare() {
  if(!(unloaded_radius > 0.0) || !(vertical_stiffness > 0.0) ||
     vertical_damping < 0.0 || !(friction_coefficient > 0.0) ||
     rolling_resistance < 0.0 || cornering_stiffness < 0.0 ||
     aligning_coefficient < 0.0 || segments < 0 || !(low_speed > 0.0)) {
    throw shift::core::Exception(
      "Iowa tire: unloaded_radius, vertical_stiffness, friction_coefficient "
      "and low_speed must be positive, and the others not negative");
  }
}

double shift::core::force_module::tire::Iowa::LongitudinalFriction(
  double s) const {
  const double mu = friction_coefficient;
  const double peak = 1.05 * mu;
  if(s < 0.2) {
    return 5.0 * peak * s;
  }
  if(s < 0.25) {
    return peak;
  }
  if(s < 0.5) {
    return peak - (peak - mu) * (s - 0.25) / 0.25;
  }
  return mu;
}

double shift::core::force_module::tire::Iowa::LateralForce(double alpha,
                                                           double limit) const {
  if(!(cornering_stiffness > 0.0) || !(limit > 0.0)) {
    return 0.0;
  }
  const double x = cornering_stiffness * std::fabs(alpha) / limit;
  if(x > 2.5) {
    return limit;
  }
  return limit * x * (1.0 + x * (-0.32 + 0.032 * x));
}

void shift::core::force_module::tire::Iowa::Evaluate(const Contact& contact,
                                                     const double*, double*,
                                                     ksl_coscrew_t* force_w) {
  *force_w = ksl_coscrew(0, 0, 0, 0, 0, 0);
  slip_ = 0.0;
  slip_angle_ = 0.0;
  if(!contact.in_contact) {
    return;
  }
  const ksl_vec3_t& v = contact.hub_velocity_w;
  const double normal =
    vertical_stiffness * contact.depth + vertical_damping * v.z;
  if(contact.depth <= 0.0 || normal <= 0.0) {
    return;
  }
  const double low = low_speed;

  /* the slips, which fade out at low speed */
  const double rim = contact.omega * (unloaded_radius - contact.depth);
  const double tread = v.x + rim;
  const double kappa =
    -tread / std::max({std::fabs(rim), std::fabs(v.x), low}) *
    std::fabs(tread) / (low + std::fabs(tread));
  slip_ = std::min(std::max(kappa, -1.0), 1.0);
  slip_angle_ = std::atan(std::fabs(v.y) / std::max(std::fabs(v.x), low)) * v.y /
                (low + std::fabs(v.y));

  /* the forces of the ground on the tread, within the friction limit */
  const double limit = friction_coefficient * normal;
  double f_x = std::copysign(normal * LongitudinalFriction(std::fabs(slip_)),
                             slip_);
  double f_y = -std::copysign(LateralForce(slip_angle_, limit), slip_angle_);
  const double aligning = -aligning_coefficient * f_y;
  const double size = std::sqrt(f_x * f_x + f_y * f_y);
  if(size > limit) {
    f_x *= limit / size;
    f_y *= limit / size;
  }

  /* the rolling resistance acts at the hub: its moment about the w frame's
  origin is arm x f */
  const double rolling =
    -rolling_resistance * normal * v.x / (low + std::fabs(v.x));
  const ksl_vec3_t& arm = contact.hub_w;

  force_w->lin.x = f_x + rolling;
  force_w->lin.y = f_y;
  force_w->lin.z = -normal;
  force_w->ang.y = arm.z * rolling;
  force_w->ang.z = -arm.y * rolling + aligning;

  /* the load across a cambered wheel acts behind it by the trail, about the
  direction in the wheel's plane nearest the normal, x cross the axle */
  const ksl_vec3_t& a = contact.spin_axis_w;
  const double camber = aligning_coefficient * normal * a.z;
  force_w->ang.y -= camber * a.z;
  force_w->ang.z += camber * a.y;
}
