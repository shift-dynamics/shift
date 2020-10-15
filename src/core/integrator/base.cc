#include "shift/core/integrator/base.h"
#include "shift/core/context.h"
#include "shift/core/integrator/ode113.h"

bool shift::core::integrator::Modules::initialized() {
  if(!shift::core::integrator::ODE113::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::integrator::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void shift::core::integrator::Base::addParameters() {

  addParameter("omega_n", &omega_n_,
               "characteristic frequency for baumgarte constraint "
               "stabilization, defaults to 200 if not provided.",
               false);
  addParameter("time_step", &time_step_,
               "time step used for numerical integration, defaults to 0.01 "
               "seconds if not provided.",
               false);
  addParameter("time_span", &time_span_,
               "time span for numerical integration, defaults to 10 seconds if "
               "not provided ",
               false);
}

void shift::core::integrator::Base::load() {}

void shift::core::integrator::Base::connect() {
  // @TODO in the future, may allow multiple integrators and multiple solvers,
  // each handling a different part of the model, or allow loading multiple
  // integrators / solvers, with only one being active at a time. For now, hook
  // integrator up to first solver
  solver_ =
    (*context_)["model_parameters"]["solvers"][0].data<shift::core::solver::Base>();
}
