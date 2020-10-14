#include "shift/core/light/spot.h"

bool shift::core::light::Spot::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->registerModule(
    "spot", shift::core::light::Spot::create);

std::unique_ptr<shift::core::light::Spot> shift::core::light::Spot::create() {
  return std::make_unique<shift::core::light::Spot>();
}

void shift::core::light::Spot::addParameters() {
  shift::core::light::Base::addParameters();
}
