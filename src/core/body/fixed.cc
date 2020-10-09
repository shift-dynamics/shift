#include "shift/core/body/fixed.h"
#include "shift/core/factory.h"

bool shift::Body::Fixed::registered_ =
  shift::Factory<shift::Body::Base>::get()->registerModule(
    "fixed", shift::Body::Fixed::create);

std::unique_ptr<shift::Body::Fixed> shift::Body::Fixed::create() {
  return std::make_unique<shift::Body::Fixed>();
}

void shift::Body::Fixed::addParameters() {
  removeParameter("name");
  removeParameter("inertia");
  removeParameter("center_of_mass");
  removeParameter("active");

  addParameter("terrain", &terrain_,
               "an array of one or more Shift terrain objects", false);
}
