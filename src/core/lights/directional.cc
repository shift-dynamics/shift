#include "shift-core/lights/directional.h"
#include "shift-core/lights/light.h"

bool Shift::Light::Directional::registered_ =
  Shift::Factory<Shift::Light::LightBase>::get()->registerModule(
    "directional", Shift::Light::Directional::create);

Shift::Light::Directional* Shift::Light::Directional::create() {
  return new Shift::Light::Directional;
}

void Shift::Light::Directional::addKeys() {
  Shift::Light::LightBase::addKeys();
}

void Shift::Light::Directional::readJSON(Json::Value& value) {
  Shift::Light::LightBase::readJSON(value);
}
