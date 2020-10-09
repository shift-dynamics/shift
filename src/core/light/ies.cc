#include "shift/core/light/ies.h"

bool shift::Light::IES::registered_ =
  shift::Factory<shift::Light::Base>::get()->registerModule(
    "ies", shift::Light::IES::create);

std::unique_ptr<shift::Light::IES> shift::Light::IES::create() {
  return std::make_unique<shift::Light::IES>();
}

void shift::Light::IES::addParameters() {
  shift::Light::Base::addParameters();
}
