#include "shift/core/light/ies.h"

bool shift::core::light::IES::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->registerModule(
    "ies", shift::core::light::IES::create);

std::unique_ptr<shift::core::light::IES> shift::core::light::IES::create() {
  return std::make_unique<shift::core::light::IES>();
}

void shift::core::light::IES::addParameters() {
  shift::core::light::Base::addParameters();
}
