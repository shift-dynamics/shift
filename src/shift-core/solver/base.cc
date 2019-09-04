#include "shift-core/solver/base.h"
#include "shift-core/context.h"
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

void Shift::Solver::Base::connect() {}

void Shift::Solver::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
