#include "shift/core/constraint/chord.h"
#include "shift/core/constraint/base.h"
#include "shift/core/context.h"

#include "shift/core/factory.h"

bool shift::core::constraint::Chord::registered_ =
  shift::core::Factory<shift::core::constraint::Base>::get()->registerModule(
    "chord", shift::core::constraint::Chord::create);

std::unique_ptr<shift::core::constraint::Chord> shift::core::constraint::Chord::create() {
  return std::make_unique<shift::core::constraint::Chord>();
}

void shift::core::constraint::Chord::addParameters() {}
