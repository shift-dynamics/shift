#include "shift-core/body/fixed.h"
#include "shift-core/factory.h"

bool Shift::Body::Fixed::registered_ =
  Shift::Factory<Shift::Body::Base>::get()->registerModule(
    "fixed", Shift::Body::Fixed::create);

std::unique_ptr<Shift::Body::Fixed> Shift::Body::Fixed::create() {
  return std::make_unique<Shift::Body::Fixed>();
}

void Shift::Body::Fixed::addParameters() {
  removeParameter("name");
  removeParameter("inertia");
  removeParameter("center_of_mass");
  removeParameter("active");

  addParameter("terrain", &terrain_,
               "an array of one or more Shift terrain objects", false);
}
