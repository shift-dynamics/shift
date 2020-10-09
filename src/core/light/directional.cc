#include "shift/core/light/directional.h"
#include "shift/core/light/base.h"

bool shift::Light::Directional::registered_ =
  shift::Factory<shift::Light::Base>::get()->registerModule(
    "directional", shift::Light::Directional::create);

std::unique_ptr<shift::Light::Directional> shift::Light::Directional::create() {
  return std::make_unique<shift::Light::Directional>();
}

void shift::Light::Directional::addParameters() {
  shift::Light::Base::addParameters();
}
