#include "shift/core/body/virtual.h"
#include "shift/core/factory.h"

bool shift::core::body::Virtual::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->registerModule(
    "virtual", shift::core::body::Virtual::create);

std::unique_ptr<shift::core::body::Virtual> shift::core::body::Virtual::create() {
  return std::make_unique<shift::core::body::Virtual>();
}
