#include "shift/core/light/directional.h"
#include "shift/core/light/base.h"

bool Shift::Light::Directional::registered_ =
  Shift::Factory<Shift::Light::Base>::get()->registerModule(
    "directional", Shift::Light::Directional::create);

std::unique_ptr<Shift::Light::Directional> Shift::Light::Directional::create() {
  return std::make_unique<Shift::Light::Directional>();
}

void Shift::Light::Directional::addParameters() {
  Shift::Light::Base::addParameters();
}
