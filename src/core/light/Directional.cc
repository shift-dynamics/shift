#include "shift/core/light/Directional.h"
#include "shift/core/light/Base.h"

bool shift::core::light::Directional::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->registerModule(
    "directional", shift::core::light::Directional::create);

std::unique_ptr<shift::core::light::Directional> shift::core::light::Directional::create() {
  return std::make_unique<shift::core::light::Directional>();
}

void shift::core::light::Directional::addParameters() {
  shift::core::light::Base::addParameters();
}
