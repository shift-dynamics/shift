#include "shift/core/light/point.h"

bool shift::Light::Point::registered_ =
  shift::Factory<shift::Light::Base>::get()->registerModule(
    "point", shift::Light::Point::create);

std::unique_ptr<shift::Light::Point> shift::Light::Point::create() {
  return std::make_unique<shift::Light::Point>();
}

void shift::Light::Point::addParameters() {
  shift::Light::Base::addParameters();
}
