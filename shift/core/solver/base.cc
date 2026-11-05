#include "shift/core/solver/base.h"
#include "shift/core/context.h"
#include "shift/core/integrator/base.h"
#include "shift/core/solver/rnca.h"

bool shift::core::solver::Modules::is_initialized() {
  if(!shift::core::solver::RNCA::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::solver::Base::DeclareParameters() {
  DeclareParameter("type", &type_, "Solver type", true);
}

void shift::core::solver::Base::Load() {}

void shift::core::solver::Base::Connect() {
  // @TODO in the future, may allow multiple integrators and multiple solvers,
  // each handling a different part of the model, or allow loading multiple
  // integrators / solvers, with only one being active at a time. For now, hook
  // solver up to first integrator
  integrator_ = (*context_)["model_parameters"]["integrators"][0]
                  .get_data<shift::core::integrator::Base>();
}

void shift::core::solver::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
