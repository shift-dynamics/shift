#include "shift/core/constraint/Chord.h"
#include "shift/core/constraint/Base.h"
#include "shift/core/Context.h"

#include "shift/core/Factory.h"

bool shift::core::constraint::Chord::registered_ =
  shift::core::Factory<shift::core::constraint::Base>::get()->registerModule(
    "chord", shift::core::constraint::Chord::create);

std::unique_ptr<shift::core::constraint::Chord> shift::core::constraint::Chord::create() {
  return std::make_unique<shift::core::constraint::Chord>();
}

void shift::core::constraint::Chord::addParameters() {}
