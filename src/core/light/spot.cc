#include "shift/core/light/spot.h"

bool Shift::Light::Spot::registered_ =
  Shift::Factory<Shift::Light::Base>::get()->registerModule(
    "spot", Shift::Light::Spot::create);

std::unique_ptr<Shift::Light::Spot> Shift::Light::Spot::create() {
  return std::make_unique<Shift::Light::Spot>();
}

void Shift::Light::Spot::addParameters() {
  Shift::Light::Base::addParameters();
}
