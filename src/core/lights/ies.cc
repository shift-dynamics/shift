#include "shift-core/lights/ies.h"

bool Shift::Light::IES::registered_ =
  Shift::Factory<Shift::Light::LightBase>::get()->registerModule(
    "ies", Shift::Light::IES::create);

Shift::Light::IES* Shift::Light::IES::create() {
  return new Shift::Light::IES;
}

void Shift::Light::IES::addKeys() {
  Shift::Light::LightBase::addKeys();
}

void Shift::Light::IES::readJSON(Json::Value& value) {
  Shift::Light::LightBase::readJSON(value);
}
