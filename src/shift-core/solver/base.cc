#include "shift-core/solver/base.h"
#include "shift-core/context.h"
#include "shift-core/integrator/base.h"
#include "shift-core/solver/rnca.h"

bool Shift::Solver::Modules::initialized() {
  if(!Shift::Solver::RNCA::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Solver::Base::addParameters() {
  addParameter("type", &type_, "Solver type", true);
}

void Shift::Solver::Base::load() {}

void Shift::Solver::Base::connect() {
  // @TODO in the future, may allow multiple integrators and multiple solvers,
  // each handling a different part of the model, or allow loading multiple
  // integrators / solvers, with only one being active at a time. For now, hook
  // solver up to first integrator
  integrator_ = (*context_)["model_parameters"]["integrators"][0]
                  .data<Shift::Integrator::Base>();
}

void Shift::Solver::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
