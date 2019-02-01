#include "shift-core/solver/base.h"
#include "shift-core/solver/rnca.h"

bool Shift::Solver::Modules::initialized() {
  if(!Shift::Solver::RNCA::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Solver::Base::addKeys() {
  SerialEntry solverType("type", SerialType::STRING, &type_);
  solverType.setRequired(false);
  solverType.setDescription("Solver type");
  addSerialEntry(solverType);
}

void Shift::Solver::Base::load() {}

void Shift::Solver::Base::connect() {}

void Shift::Solver::Base::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
}
