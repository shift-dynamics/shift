#include "shift/core/body/virtual.h"
#include "shift/core/factory.h"

bool shift::Body::Virtual::registered_ =
  shift::Factory<shift::Body::Base>::get()->registerModule(
    "virtual", shift::Body::Virtual::create);

std::unique_ptr<shift::Body::Virtual> shift::Body::Virtual::create() {
  return std::make_unique<shift::Body::Virtual>();
}
