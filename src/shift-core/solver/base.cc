#include "shift-core/solver/base.h"
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
