#include "shift/core/force_module/tire/fiala.h"

#include <algorithm>
#include <cmath>

#include "ksl/coscrew.h"

void shift::core::force_module::tire::Fiala::DeclareParameters() {
  DeclareParameter("unloaded_radius", &unloaded_radius,
                   "unloaded radius (m); optional, default 0.344", false);
  DeclareParameter("vertical_stiffness", &vertical_stiffness,
                   "vertical stiffness (N/m); optional, default 310000", false);
  DeclareParameter("vertical_damping", &vertical_damping,
                   "vertical damping (N s/m); optional, default 3100", false);
  DeclareParameter("rolling_resistance", &rolling_resistance,
                   "rolling resistance coefficient; optional, default 0.001",
                   false);
  DeclareParameter("width", &width, "tire width (m); optional, default 0.235",
                   false);
  DeclareParameter("cslip", &cslip,
                   "longitudinal slip stiffness (N); optional, default 50000",
                   false);
  DeclareParameter("calpha", &calpha,
                   "cornering stiffness (N/rad); optional, default 45836.6236",
                   false);
  DeclareParameter("u_max", &u_max,
                   "friction coefficient without sliding; optional, default 1",
                   false);
  DeclareParameter("u_min", &u_min,
                   "friction coefficient when sliding; optional, default 0.9",
                   false);
  DeclareParameter("x_relaxation_length", &x_relaxation_length,
                   "longitudinal relaxation length (m); optional, default 0.05",
                   false);
  DeclareParameter("y_relaxation_length", &y_relaxation_length,
                   "lateral relaxation length (m); optional, default 0.15",
                   false);
}

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
    -vertical_stiffness * contact.depth - vertical_damping * vs.z, 0.0);
  const double abs_fz = std::fabs(fz);

  /* transient slip */
  state_rates[0] = -(abs_vx * kappa + vs.x) / x_relaxation_length;
  state_rates[1] = (vs.y - abs_vx * std::tan(alpha)) / y_relaxation_length;

  /* friction falls from u_max to u_min with the combined slip */
  const double combined_slip =
    std::min(1.0, std::sqrt(kappa * kappa + alpha * alpha));
  const double u = u_max - (u_max - u_min) * combined_slip;

  /* longitudinal force */
  double fx;
  const double critical_slip = std::fabs(u * fz / (2.0 * cslip));
  if(std::fabs(kappa) < critical_slip) {
    fx = cslip * kappa;
  } else {
    const double fx1 = u * abs_fz;
    const double fx2 = std::fabs(fx1 * fx1 / (4.0 * kappa * cslip));
    fx = std::copysign(1.0, kappa) * (fx1 - fx2);
  }

  /* lateral force and aligning moment */
  double fy;
  double mz = 0.0;
  const double h_scale = 3.0 * u * abs_fz / calpha;
  const double critical_alpha = std::atan(h_scale);
  if(std::fabs(alpha) <= critical_alpha) {
    const double h = 1.0 - std::fabs(std::tan(alpha)) / h_scale;
    const double h3 = h * h * h;
    fy = -u * abs_fz * (1.0 - h3) * std::copysign(1.0, alpha);
    mz = u * abs_fz * width * (1.0 - h) * h3 * std::copysign(1.0, alpha);
  } else {
    fy = -u * std::copysign(abs_fz, alpha);
  }

  /* rolling resistance */
  const double my = -rolling_resistance * std::copysign(abs_fz, contact.omega);

  force_w->lin.x = fx;
  force_w->lin.y = fy;
  force_w->lin.z = fz;
  force_w->ang.x = 0.0;
  force_w->ang.y = my;
  force_w->ang.z = mz;
}
