#include "shift-core/constraints/chord.h"
#include "shift-core/constraints/constraint.h"
#include "shift-core/context.h"

#include "shift-core/factory.h"

bool Shift::Constraint::Chord::registered_ =
  Shift::Factory<Shift::Constraint::ConstraintBase>::get()->registerModule(
    "chord", Shift::Constraint::Chord::create);

std::unique_ptr<Shift::Constraint::Chord> Shift::Constraint::Chord::create() {
  return std::make_unique<Shift::Constraint::Chord>();
}

void Shift::Constraint::Chord::addKeys() {}

void Shift::Constraint::Chord::readJSON(Json::Value& value) {
  Shift::Constraint::ConstraintBase::readJSON(value);
}
