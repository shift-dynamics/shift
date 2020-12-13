#include "shift/core/solver/Base.h"
#include "shift/core/Context.h"
#include "shift/core/integrator/Base.h"
#include "shift/core/solver/RNCA.h"

bool shift::core::solver::Modules::initialized() {
  if(!shift::core::solver::RNCA::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::solver::Base::addParameters() {
  addParameter("type", &type_, "Solver type", true);
}

void shift::core::solver::Base::load() {}

void shift::core::solver::Base::connect() {
  // @TODO in the future, may allow multiple integrators and multiple solvers,
  // each handling a different part of the model, or allow loading multiple
  // integrators / solvers, with only one being active at a time. For now, hook
  // solver up to first integrator
  integrator_ = (*context_)["model_parameters"]["integrators"][0]
                  .data<shift::core::integrator::Base>();
}

void shift::core::solver::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
