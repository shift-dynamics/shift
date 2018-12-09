#include "shift-core/solvers/solver.h"
#include "shift-core/solvers/rnca.h"

bool Shift::Solver::Modules::initialized() {
  if(!Shift::Solver::RNCA::isRegistered()) {
    return false;
  }
  return true;
}

bool Shift::Solver::Modules::initialized_ =
  Shift::Solver::Modules::initialized();

void Shift::Solver::SolverBase::addKeys() {
  SerialEntry solverType("type", SerialType::STRING, &type_);
  solverType.setRequired(false);
  solverType.setDescription("Solver type");
  addSerialEntry(solverType);
}
