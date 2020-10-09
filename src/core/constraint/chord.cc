#include "shift/core/constraint/chord.h"
#include "shift/core/constraint/base.h"
#include "shift/core/context.h"

#include "shift/core/factory.h"

bool shift::Constraint::Chord::registered_ =
  shift::Factory<shift::Constraint::Base>::get()->registerModule(
    "chord", shift::Constraint::Chord::create);

std::unique_ptr<shift::Constraint::Chord> shift::Constraint::Chord::create() {
  return std::make_unique<shift::Constraint::Chord>();
}

void shift::Constraint::Chord::addParameters() {}
