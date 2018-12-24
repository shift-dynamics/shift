#include "shift-core/lights/ies.h"

bool Shift::Light::IES::registered_ =
  Shift::Factory<Shift::Light::LightBase>::get()->registerModule(
    "ies", Shift::Light::IES::create);

std::unique_ptr<Shift::Light::IES> Shift::Light::IES::create() {
  return std::make_unique<Shift::Light::IES>();
}

void Shift::Light::IES::addKeys() {
  Shift::Light::LightBase::addKeys();
}

void Shift::Light::IES::readJSON(Json::Value& value) {
  Shift::Light::LightBase::readJSON(value);
}
