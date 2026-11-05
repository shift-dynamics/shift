#include "shift/core/integrator/base.h"
#include "shift/core/context.h"
#include "shift/core/log.h"
#include "shift/core/integrator/ode113.h"
#include "shift/core/integrator/rk4.h"

bool shift::core::integrator::Modules::is_initialized() {
  if(!shift::core::integrator::ODE113::is_registered()) {
    return false;
  }
  if(!shift::core::integrator::RK4::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::integrator::Base::ActivateObjects(bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
};

void shift::core::integrator::Base::DeclareParameters() {

  DeclareParameter("omega_n", &omega_n_,
               "characteristic frequency for baumgarte constraint "
               "stabilization, defaults to 200 if not provided.",
               false);
  DeclareParameter("time_step", &time_step_,
               "time step used for numerical integration, defaults to 0.01 "
               "seconds if not provided.",
               false);
  DeclareParameter("time_span", &time_span_,
               "time span for numerical integration, defaults to 10 seconds if "
               "not provided ",
               false);
}

void shift::core::integrator::Base::Load() {}

void shift::core::integrator::Base::Connect() {
  // @TODO in the future, may allow multiple integrators and multiple solvers,
  // each handling a different part of the model, or allow loading multiple
  // integrators / solvers, with only one being active at a time. For now, hook
  // integrator up to first solver
  solver_ =
    (*context_)["model_parameters"]["solvers"][0].get_data<shift::core::solver::Base>();
}

/*!
@brief logs simulation progress each time another 10% of time_span_ is reached
*/
void shift::core::integrator::Base::ReportProgress(const double time) {
  while(next_progress_percent_ <= 100 &&
        time >= time_span_ * next_progress_percent_ / 100.0) {
    SHIFT_INFO("Simulation progress: " + std::to_string(next_progress_percent_) +
               "% (t = " + std::to_string(time) + " of " +
               std::to_string(time_span_) + ")");
    next_progress_percent_ += 10;
  }
}
