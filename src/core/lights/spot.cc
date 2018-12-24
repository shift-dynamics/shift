#include "shift-core/lights/spot.h"

bool Shift::Light::Spot::registered_ =
  Shift::Factory<Shift::Light::LightBase>::get()->registerModule(
    "spot", Shift::Light::Spot::create);

std::unique_ptr<Shift::Light::Spot> Shift::Light::Spot::create() {
  return std::make_unique<Shift::Light::Spot>();
}

void Shift::Light::Spot::addKeys() {
  Shift::Light::LightBase::addKeys();
}

void Shift::Light::Spot::readJSON(Json::Value& value) {
  Shift::Light::LightBase::readJSON(value);
}
