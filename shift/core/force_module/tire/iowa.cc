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
  DeclareParameter("combined_slip", &combined_slip,
                   "the longitudinal and lateral forces share the friction "
                   "as the slip of the tread directs, against its velocity "
                   "once it slides, in place of the friction ellipse or the "
                   "longitudinal force first; needs a cornering stiffness; "
                   "optional, default false",
                   false);
  DeclareParameter("load_sensitivity", &load_sensitivity,
                   "exponent of the friction on the load, \\mu (f_n / "
                   "reference_load)^load_sensitivity, negative as the "
                   "friction falls with the load; optional, default 0",
                   false);
  DeclareParameter("reference_load", &reference_load,
                   "load at which the friction is friction_coefficient (N), "
                   "needed with a load_sensitivity; optional",
                   false);
  DeclareParameter("saturation", &saturation,
                   "the shape of the forces as the slips grow: \"peak\", "
                   "a peak and a fall to the sliding friction, as on paved "
                   "ground; \"exponential\", a rise to the friction with no "
                   "peak, as on loose ground; or \"soil\", exponential on "
                   "soil (the tire's or the terrain's) and peak elsewhere; "
                   "optional, default \"peak\"",
                   false);
  DeclareParameter("loose_shape", &loose_shape,
                   "B, C and E of a Magic Formula of the combined slip for "
                   "the forces on loose ground, D the friction (as the "
                   "loose surface of Tavernini et al., 2014: [1.5071, "
                   "1.0901, -1.584]); optional, default [0, 0, 0], the "
                   "exponential",
                   false)
    .set_row_dim(3);
  DeclareParameter("width", &width,
                   "tire width (m), for the soft soil; optional, default 0.235",
                   false);
  DeclareParameter("soil", &soil_,
                   "the soil (see force_module::Soil): the tire sinks into "
                   "soft soil, which gives the load and the motion "
                   "resistance; optional, if not given the soil of the "
                   "terrain under the tire, if it has one, or else rigid "
                   "ground",
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
  if(combined_slip && !(cornering_stiffness > 0.0)) {
    throw shift::core::Exception(
      "Iowa tire: combined_slip needs a positive cornering_stiffness");
  }
  if(!(load_sensitivity == 0.0 || reference_load > 0.0)) {
    throw shift::core::Exception(
      "Iowa tire: a load_sensitivity needs a positive reference_load");
  }
  if(loose_shape.size() != 3 ||
     (has_loose_shape() && (!(loose_shape[1] > 0.0) || !(loose_shape[2] <= 1.0)))) {
    throw shift::core::Exception(
      "Iowa tire: loose_shape needs B and C positive and E at most 1");
  }
  if(saturation != "peak" && saturation != "exponential" &&
     saturation != "soil") {
    throw shift::core::Exception("Iowa tire: saturation \"" + saturation +
                                 "\" is not \"peak\", \"exponential\" or "
                                 "\"soil\"");
  }
  if(soft_ && !(soil.n < 3.0)) {
    throw shift::core::Exception(
      "Iowa tire: the soft soil needs a sinkage exponent n below 3");
  }
}

void shift::core::force_module::tire::Iowa::SoilForces(
  double z, double* load, double* resistance) const {
  SoilForces(soil, z, load, resistance);
}

void shift::core::force_module::tire::Iowa::SoilForces(
  const SoilProperties& ground, double z, double* load,
  double* resistance) const {
  const double k = ground.k_c / width + ground.k_phi;
  const double d = 2.0 * unloaded_radius;
  *load = (3.0 - ground.n) / 3.0 * width * k * std::sqrt(d) *
          std::pow(z, ground.n + 0.5);
  *resistance = width * k * std::pow(z, ground.n + 1.0) / (ground.n + 1.0);
}

void shift::core::force_module::tire::Iowa::SoilLoad(
  const SoilProperties& ground, double depth, double* load, double* sinkage,
  double* resistance) const {
  *load = *sinkage = *resistance = 0.0;
  if(!(depth > 0.0)) {
    return;
  }
  /* the load f of the tire's deflection f / k and the sinkage z(f) that
  carries it, which add to the depth: by bisection, as both grow with f;
  the tire alone, or the soil alone, would carry more at the depth */
  double unit, ignored;
  SoilForces(ground, 1.0, &unit, &ignored);
  const double exponent = 1.0 / (ground.n + 0.5);
  double lo = 0.0;
  double hi = std::min(vertical_stiffness * depth,
                       unit * std::pow(depth, ground.n + 0.5));
  for(int i = 0; i < 50 && hi - lo > 1e-9 * hi; ++i) {
    const double f = 0.5 * (lo + hi);
    (f / vertical_stiffness + std::pow(f / unit, exponent) < depth ? lo : hi) =
      f;
  }
  *load = 0.5 * (lo + hi);
  *sinkage = std::pow(*load / unit, exponent);
  SoilForces(ground, *sinkage, &ignored, resistance);
}

double shift::core::force_module::tire::Iowa::Friction(double normal) const {
  if(load_sensitivity == 0.0 || !(normal > 0.0)) {
    return friction_coefficient;
  }
  return friction_coefficient *
         std::pow(normal / reference_load, load_sensitivity);
}

double shift::core::force_module::tire::Iowa::LongitudinalFriction(
  double s, bool loose) const {
  const double mu = friction_coefficient;
  const double peak = 1.05 * mu;
  if(loose && has_loose_shape()) {
    return mu * LooseShare(s);
  }
  if(loose) {
    /* the same slope at no slip, rising to the friction with no peak */
    return mu * -std::expm1(-5.0 * peak / mu * s);
  }
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
                                                           double limit,
                                                           bool loose) const {
  if(loose && has_loose_shape()) {
    return limit > 0.0 ? limit * LooseShare(std::tan(std::fabs(alpha))) : 0.0;
  }
  if(!(cornering_stiffness > 0.0) || !(limit > 0.0)) {
    return 0.0;
  }
  const double x = cornering_stiffness * std::fabs(alpha) / limit;
  if(loose) {
    return -limit * std::expm1(-x);
  }
  if(x > 2.5) {
    return limit;
  }
  return limit * x * (1.0 + x * (-0.32 + 0.032 * x));
}

void shift::core::force_module::tire::Iowa::CombinedForces(
  double kappa, double alpha, double v_x, double v_y, double normal,
  double* f_x, double* f_y, bool loose) const {
  *f_x = *f_y = 0.0;
  const double mu = Friction(normal);
  const double limit = mu * normal;
  if(loose && has_loose_shape()) {
    /* the Magic Formula of the combined slip, against the sliding tread */
    if(!(limit > 0.0)) {
      return;
    }
    const double sigma = std::hypot(kappa, std::tan(std::fabs(alpha)));
    double u_x = kappa * std::max(std::fabs(v_x), low_speed), u_y = -v_y;
    const double u = std::hypot(u_x, u_y);
    if(!(sigma > 0.0) || !(u > 0.0)) {
      return;
    }
    const double size = limit * LooseShare(sigma);
    *f_x = size * u_x / u;
    *f_y = size * u_y / u;
    return;
  }
  if(!(limit > 0.0) || !(cornering_stiffness > 0.0)) {
    return;
  }
  /* the slips over their peaks, and the combined slip */
  const double kappa_peak = 0.2;
  const double alpha_peak = 2.5 * limit / cornering_stiffness;
  const double s_x = std::fabs(kappa) / kappa_peak;
  const double s_y = std::fabs(alpha) / alpha_peak;
  const double s = std::hypot(s_x, s_y);
  if(!(s > 0.0)) {
    return;
  }
  /* the size: each pure slip's force at the combined slip, weighted by the
  direction of the slip */
  const double f_x0 = normal * mu / friction_coefficient *
                      LongitudinalFriction(kappa_peak * s, loose);
  const double f_y0 = LateralForce(alpha_peak * s, limit, loose);
  const double size =
    std::min((s_x * s_x * f_x0 + s_y * s_y * f_y0) / (s * s), limit);
  /* the direction: as the stiffnesses share the force while the tread
  grips, against the velocity of the sliding tread once it slides */
  const double a_x = std::copysign(s_x, kappa) / s;
  const double a_y = -std::copysign(s_y, alpha) / s;
  double u_x = kappa * std::max(std::fabs(v_x), low_speed);
  double u_y = -v_y;
  const double u = std::hypot(u_x, u_y);
  if(u > 0.0) {
    u_x /= u;
    u_y /= u;
  } else {
    u_x = a_x;
    u_y = a_y;
  }
  const double w = std::min(std::max(s - 1.0, 0.0), 1.0);
  double d_x = (1.0 - w) * a_x + w * u_x;
  double d_y = (1.0 - w) * a_y + w * u_y;
  double d = std::hypot(d_x, d_y);
  if(!(d > 1e-12)) {
    d_x = u_x;
    d_y = u_y;
    d = 1.0;
  }
  *f_x = size * d_x / d;
  *f_y = size * d_y / d;
}

void shift::core::force_module::tire::Iowa::TreadForces(
  double kappa, double alpha, double v_x, double v_y, double normal,
  bool on_soil, double* f_x, double* f_y, double* aligning) const {
  const bool relaxed = relaxation_length > 0.0;
  const bool loose = is_loose(on_soil);
  const double mu = Friction(normal);
  const double limit = mu * normal;
  *f_x = std::copysign(normal * mu / friction_coefficient *
                         LongitudinalFriction(std::fabs(kappa), loose),
                       kappa);
  *f_y = -std::copysign(LateralForce(alpha, limit, loose), alpha);
  if(combined_slip) {
    CombinedForces(kappa, alpha, v_x, v_y, normal, f_x, f_y, loose);
    *aligning = -aligning_coefficient * *f_y;
  } else if(relaxed) {
    /* the second variant: the longitudinal force takes the friction first,
    and the aligning moment follows the lateral force that is left */
    if(std::fabs(*f_x) >= limit) {
      *f_x = std::copysign(limit, *f_x);
      *f_y = 0.0;
    } else if(std::hypot(*f_x, *f_y) > limit) {
      *f_y = std::copysign(std::sqrt(limit * limit - *f_x * *f_x), *f_y);
    }
    *aligning = -aligning_coefficient * *f_y;
  } else {
    *aligning = -aligning_coefficient * *f_y;
    const double size = std::sqrt(*f_x * *f_x + *f_y * *f_y);
    if(size > limit) {
      *f_x *= limit / size;
      *f_y *= limit / size;
    }
  }
}

void shift::core::force_module::tire::Iowa::RestingDeflection(
  double load, double* deflection, double* resistance) const {
  *deflection = *resistance = 0.0;
  if(!(load > 0.0)) {
    return;
  }
  *deflection = load / vertical_stiffness;
  *resistance = rolling_resistance * load;
  if(!soft_) {
    return;
  }
  /* the tire's deflection, and the sinkage that carries the load: the load
  grows with it */
  double lo = 0.0, hi = unloaded_radius, f, r;
  for(int i = 0; i < 60; ++i) {
    const double z = 0.5 * (lo + hi);
    SoilForces(z, &f, &r);
    (f < load ? lo : hi) = z;
  }
  const double z = 0.5 * (lo + hi);
  SoilForces(z, &f, &r);
  *deflection += z;
  *resistance += r;
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

  /* the load, and the force that resists the rolling, before its sign: on
  soil, the tire's own soil or else the terrain's, the tire and the soil in
  series, and the soil's compaction resistance besides the tire's */
  const SoilProperties* ground = soft_ ? &soil : contact.soil;
  if(ground && !(ground->n < 3.0)) {
    throw shift::core::Exception(
      "Iowa tire: the soil under the tire needs a sinkage exponent n below 3");
  }
  double normal, compaction = 0.0;
  if(ground) {
    double sinkage;
    SoilLoad(*ground, contact.depth, &normal, &sinkage, &compaction);
    normal += vertical_damping * v.z;
  } else {
    normal = vertical_stiffness * contact.depth + vertical_damping * v.z;
  }
  if(normal <= 0.0) {
    return;
  }
  const double resistance = rolling_resistance * normal + compaction;

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
  double f_x, f_y, aligning;
  TreadForces(slip_, slip_angle_, v.x, v.y, normal, ground != nullptr, &f_x,
              &f_y, &aligning);

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
