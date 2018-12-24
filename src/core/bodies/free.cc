#include "shift-core/bodies/free.h"
#include "shift-core/factory.h"

bool Shift::Body::Free::registered_ =
  Shift::Factory<Shift::Body::BodyBase>::get()->registerModule(
    "free", Shift::Body::Free::create);

std::unique_ptr<Shift::Body::Free> Shift::Body::Free::create() {
  return std::make_unique<Shift::Body::Free>();
}

void Shift::Body::Free::addKeys() {}
