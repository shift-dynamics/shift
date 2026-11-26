#include "shift/core/integrator/generalized_alpha.h"

#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::integrator::GeneralizedAlpha::registered_ =
  shift::core::Factory<shift::core::integrator::Base>::get()->RegisterModule(
    "generalized_alpha", shift::core::integrator::GeneralizedAlpha::Create);

std::unique_ptr<shift::core::integrator::GeneralizedAlpha>
shift::core::integrator::GeneralizedAlpha::Create() {
  return std::make_unique<shift::core::integrator::GeneralizedAlpha>();
}

void shift::core::integrator::GeneralizedAlpha::DeclareParameters() {
  DeclareParameter("rho_infinity", &rho_infinity_,
                   "spectral radius at infinite frequency, in [0, 1]: 1 is "
                   "the trapezoidal rule, without numerical damping, smaller "
                   "values damp high frequencies more; defaults to 0.8 if not "
                   "provided",
                   false);
}

void shift::core::integrator::GeneralizedAlpha::SetCoefficients() {
  if(rho_infinity_ < 0.0 || rho_infinity_ > 1.0) {
    throw shift::core::Exception(
      "generalized_alpha: rho_infinity must be in [0, 1], not " +
      std::to_string(rho_infinity_));
  }
  alpha_m_ = (2.0 * rho_infinity_ - 1.0) / (rho_infinity_ + 1.0);
  alpha_f_ = rho_infinity_ / (rho_infinity_ + 1.0);
  gamma_ = 0.5 - alpha_m_ + alpha_f_;
  beta_ = 0.25 * (1.0 - alpha_m_ + alpha_f_) * (1.0 - alpha_m_ + alpha_f_);
}
