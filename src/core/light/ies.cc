#include "shift/core/light/ies.h"

bool Shift::Light::IES::registered_ =
  Shift::Factory<Shift::Light::Base>::get()->registerModule(
    "ies", Shift::Light::IES::create);

std::unique_ptr<Shift::Light::IES> Shift::Light::IES::create() {
  return std::make_unique<Shift::Light::IES>();
}

void Shift::Light::IES::addParameters() {
  Shift::Light::Base::addParameters();
}
