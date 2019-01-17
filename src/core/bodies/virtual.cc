#include "shift-core/bodies/virtual.h"
#include "shift-core/factory.h"

bool Shift::Body::Virtual::registered_ =
  Shift::Factory<Shift::Body::BodyBase>::get()->registerModule(
    "virtual", Shift::Body::Virtual::create);

std::unique_ptr<Shift::Body::Virtual> Shift::Body::Virtual::create() {
  return std::make_unique<Shift::Body::Virtual>();
}
