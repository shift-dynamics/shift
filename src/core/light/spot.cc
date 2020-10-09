#include "shift/core/light/spot.h"

bool shift::Light::Spot::registered_ =
  shift::Factory<shift::Light::Base>::get()->registerModule(
    "spot", shift::Light::Spot::create);

std::unique_ptr<shift::Light::Spot> shift::Light::Spot::create() {
  return std::make_unique<shift::Light::Spot>();
}

void shift::Light::Spot::addParameters() {
  shift::Light::Base::addParameters();
}
