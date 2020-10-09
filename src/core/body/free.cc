#include "shift/core/body/free.h"
#include "shift/core/factory.h"

bool shift::Body::Free::registered_ =
  shift::Factory<shift::Body::Base>::get()->registerModule(
    "free", shift::Body::Free::create);

std::unique_ptr<shift::Body::Free> shift::Body::Free::create() {
  return std::make_unique<shift::Body::Free>();
}

void shift::Body::Free::addParameters() {}
