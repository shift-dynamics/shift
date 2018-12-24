#include "shift-core/lights/point.h"

bool Shift::Light::Point::registered_ =
  Shift::Factory<Shift::Light::LightBase>::get()->registerModule(
    "point", Shift::Light::Point::create);

std::unique_ptr<Shift::Light::Point> Shift::Light::Point::create() {
  return std::make_unique<Shift::Light::Point>();
}

void Shift::Light::Point::addKeys() {
  Shift::Light::LightBase::addKeys();
}

void Shift::Light::Point::readJSON(Json::Value& value) {
  Shift::Light::LightBase::readJSON(value);
}
