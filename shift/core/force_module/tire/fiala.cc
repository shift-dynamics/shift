#include "shift/core/force_module/tire/fiala.h"

#include <algorithm>
#include <cmath>

#include "ksl/coscrew.h"

void shift::core::force_module::tire::Fiala::Evaluate(
  const Contact& contact, const double* states, double* state_rates,
  ksl_coscrew_t* force_w) {

  const double kappa = states[0]; // longitudinal slip
  const double alpha = states[1]; // slip angle
  *force_w = ksl_coscrew(0, 0, 0, 0, 0, 0);

  if(!contact.in_contact) {
    /* out of contact, the slip states decay */
    state_rates[0] = -100.0 * kappa;
    state_rates[1] = -100.0 * alpha;
    return;
  }

  const ksl_vec3_t& vs = contact.contact_velocity_w;
  const double abs_vx = std::fabs(contact.vx);

  /* normal force: spring and damper, zero once the wheel moves away from the
  terrain faster than the spring extends (Fz < 0 pushes the wheel up) */
  const double fz = std::min(
    -p_.vertical_stiffness * contact.depth - p_.vertical_damping * vs.z, 0.0);
  const double abs_fz = std::fabs(fz);

  /* transient slip */
  state_rates[0] = -(abs_vx * kappa + vs.x) / p_.x_relaxation_length;
  state_rates[1] = (vs.y - abs_vx * std::tan(alpha)) / p_.y_relaxation_length;

  /* friction falls from u_max to u_min with the combined slip */
  const double combined_slip =
    std::min(1.0, std::sqrt(kappa * kappa + alpha * alpha));
  const double u = p_.u_max - (p_.u_max - p_.u_min) * combined_slip;

  /* longitudinal force */
  double fx;
  const double critical_slip = std::fabs(u * fz / (2.0 * p_.cslip));
  if(std::fabs(kappa) < critical_slip) {
    fx = p_.cslip * kappa;
  } else {
    const double fx1 = u * abs_fz;
    const double fx2 = std::fabs(fx1 * fx1 / (4.0 * kappa * p_.cslip));
    fx = std::copysign(1.0, kappa) * (fx1 - fx2);
  }

  /* lateral force and aligning moment */
  double fy;
  double mz = 0.0;
  const double h_scale = 3.0 * u * abs_fz / p_.calpha;
  const double critical_alpha = std::atan(h_scale);
  if(std::fabs(alpha) <= critical_alpha) {
    const double h = 1.0 - std::fabs(std::tan(alpha)) / h_scale;
    const double h3 = h * h * h;
    fy = -u * abs_fz * (1.0 - h3) * std::copysign(1.0, alpha);
    mz = u * abs_fz * p_.width * (1.0 - h) * h3 * std::copysign(1.0, alpha);
  } else {
    fy = -u * std::copysign(abs_fz, alpha);
  }

  /* rolling resistance */
  const double my = -p_.rolling_resistance * std::copysign(abs_fz, contact.omega);

  force_w->lin.x = fx;
  force_w->lin.y = fy;
  force_w->lin.z = fz;
  force_w->ang.x = 0.0;
  force_w->ang.y = my;
  force_w->ang.z = mz;
}
