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
  DeclareParameter("relaxation_length", &relaxation_length,
                   "longitudinal relaxation length (m): the slip ratio lags "
                   "the slip of the tread, and the longitudinal force takes "
                   "the friction first; 0 for none; optional, default 0",
                   false);
  DeclareParameter("width", &width,
                   "tire width (m), for the soft soil; optional, default 0.235",
                   false);
  DeclareParameter("soil", &soil_,
                   "the soil (see force_module::Soil): the wheel sinks into "
                   "soft soil, which gives the load and the motion "
                   "resistance; optional, rigid ground if not given",
                   false);
}

void shift::core::force_module::tire::Iowa::Prepare() {
  if(!(unloaded_radius > 0.0) || !(vertical_stiffness > 0.0) ||
     vertical_damping < 0.0 || !(friction_coefficient > 0.0) ||
     rolling_resistance < 0.0 || cornering_stiffness < 0.0 ||
     aligning_coefficient < 0.0 || segments < 0 || !(low_speed > 0.0) ||
     !(relaxation_length >= 0.0) || !(width > 0.0)) {
    throw shift::core::Exception(
      "Iowa tire: unloaded_radius, vertical_stiffness, friction_coefficient, "
      "low_speed and width must be positive, and the others not negative");
  }
  if(soft_ && !(soil.n < 3.0)) {
    throw shift::core::Exception(
      "Iowa tire: the soft soil needs a sinkage exponent n below 3");
  }
}

void shift::core::force_module::tire::Iowa::SoilForces(
  double z, double* load, double* resistance) const {
  const double k = soil.k_c / width + soil.k_phi;
  const double d = 2.0 * unloaded_radius;
  *load = (3.0 - soil.n) / 3.0 * width * k * std::sqrt(d) *
          std::pow(z, soil.n + 0.5);
  *resistance = width * k * std::pow(z, soil.n + 1.0) / (soil.n + 1.0);
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
                                                     const double* states,
                                                     double* state_rates,
                                                     ksl_coscrew_t* force_w) {
  *force_w = ksl_coscrew(0, 0, 0, 0, 0, 0);
  const bool relaxed = relaxation_length > 0.0;
  const double low = low_speed;
  const ksl_vec3_t& v = contact.hub_velocity_w;
  /* out of contact the relaxed slip relaxes to zero; overwritten below */
  if(relaxed) {
    state_rates[0] =
      -(std::fabs(contact.vx) + low) * states[0] / relaxation_length;
  }
  slip_ = relaxed ? std::min(std::max(states[0], -1.0), 1.0) : 0.0;
  slip_angle_ = 0.0;
  if(!contact.in_contact || contact.depth <= 0.0) {
    return;
  }

  /* the load, and the force that resists the rolling, before its sign */
  double normal, resistance;
  if(soft_) {
    SoilForces(contact.depth, &normal, &resistance);
    normal += vertical_damping * v.z;
  } else {
    normal = vertical_stiffness * contact.depth + vertical_damping * v.z;
    resistance = rolling_resistance * normal;
  }
  if(normal <= 0.0) {
    return;
  }

  /* the slips, which fade out at low speed; with a relaxation length the
  slip ratio is the state, which lags the slip of the tread */
  const double rim = contact.omega * (unloaded_radius - contact.depth);
  const double tread = v.x + rim;
  if(relaxed) {
    double rate = (-tread - (std::fabs(v.x) + low) * states[0]) /
                  relaxation_length;
    if((states[0] >= 1.0 && rate > 0.0) || (states[0] <= -1.0 && rate < 0.0)) {
      rate = 0.0;
    }
    state_rates[0] = rate;
  } else {
    const double kappa =
      -tread / std::max({std::fabs(rim), std::fabs(v.x), low}) *
      std::fabs(tread) / (low + std::fabs(tread));
    slip_ = std::min(std::max(kappa, -1.0), 1.0);
  }
  slip_angle_ = std::atan(std::fabs(v.y) / std::max(std::fabs(v.x), low)) * v.y /
                (low + std::fabs(v.y));

  /* the forces of the ground on the tread, within the friction limit */
  const double limit = friction_coefficient * normal;
  double f_x = std::copysign(normal * LongitudinalFriction(std::fabs(slip_)),
                             slip_);
  double f_y = -std::copysign(LateralForce(slip_angle_, limit), slip_angle_);
  double aligning;
  if(relaxed) {
    /* the second variant: the longitudinal force takes the friction first,
    and the aligning moment follows the lateral force that is left */
    if(std::fabs(f_x) >= limit) {
      f_x = std::copysign(limit, f_x);
      f_y = 0.0;
    } else if(std::hypot(f_x, f_y) > limit) {
      f_y = std::copysign(std::sqrt(limit * limit - f_x * f_x), f_y);
    }
    aligning = -aligning_coefficient * f_y;
  } else {
    aligning = -aligning_coefficient * f_y;
    const double size = std::sqrt(f_x * f_x + f_y * f_y);
    if(size > limit) {
      f_x *= limit / size;
      f_y *= limit / size;
    }
  }

  /* the rolling resistance acts at the hub: its moment about the w frame's
  origin is arm x f */
  const double rolling = -resistance * v.x / (low + std::fabs(v.x));
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
