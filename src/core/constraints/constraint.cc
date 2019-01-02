#include "shift-core/constraints/constraint.h"
#include "shift-core/context.h"

bool Shift::Constraint::Modules::initialized() {
  return true;
}

void Shift::Constraint::ConstraintBase::readJSON(Json::Value& value) {
  Serialize::readJSON(value);
}

void Shift::Constraint::ConstraintBase::load() {}

void Shift::Constraint::ConstraintBase::connect() {}
