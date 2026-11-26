#include "shift/core/integrator/hht.h"

#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::integrator::HHT::registered_ =
  shift::core::Factory<shift::core::integrator::Base>::get()->RegisterModule(
    "hht", shift::core::integrator::HHT::Create);

std::unique_ptr<shift::core::integrator::HHT>
shift::core::integrator::HHT::Create() {
  return std::make_unique<shift::core::integrator::HHT>();
}

void shift::core::integrator::HHT::DeclareParameters() {
  DeclareParameter("alpha", &alpha_,
                   "HHT alpha, in [-1/3, 0]: 0 is the trapezoidal rule, "
                   "without numerical damping, -1/3 damps high frequencies "
                   "most; defaults to -0.1 if not provided",
                   false);
}

void shift::core::integrator::HHT::SetCoefficients() {
  if(alpha_ < -1.0 / 3.0 - 1e-12 || alpha_ > 0.0) {
    throw shift::core::Exception("hht: alpha must be in [-1/3, 0], not " +
                                 std::to_string(alpha_));
  }
  alpha_m_ = 0.0;
  alpha_f_ = -alpha_;
  gamma_ = 0.5 - alpha_;
  beta_ = 0.25 * (1.0 - alpha_) * (1.0 - alpha_);
}
