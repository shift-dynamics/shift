#include "shift-core/solvers/solver.h"
#include "shift-core/solvers/rnca.h"

bool Shift::Solver::Modules::initialized() {
  if(!Shift::Solver::RNCA::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Solver::SolverBase::addKeys() {
  SerialEntry solverType("type", SerialType::STRING, &type_);
  solverType.setRequired(false);
  solverType.setDescription("Solver type");
  addSerialEntry(solverType);
}

void Shift::Solver::SolverBase::load() {}

void Shift::Solver::SolverBase::connect() {}

void Shift::Solver::SolverBase::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
  if(context_) {
    // context_->setActive(this);
  }
}
