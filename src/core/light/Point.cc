#include "shift/core/light/Point.h"

bool shift::core::light::Point::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->registerModule(
    "point", shift::core::light::Point::create);

std::unique_ptr<shift::core::light::Point> shift::core::light::Point::create() {
  return std::make_unique<shift::core::light::Point>();
}

void shift::core::light::Point::addParameters() {
  shift::core::light::Base::addParameters();
}
