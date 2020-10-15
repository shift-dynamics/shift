#include "shift/core/body/fixed.h"
#include "shift/core/factory.h"

bool shift::core::body::Fixed::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->registerModule(
    "fixed", shift::core::body::Fixed::create);

std::unique_ptr<shift::core::body::Fixed> shift::core::body::Fixed::create() {
  return std::make_unique<shift::core::body::Fixed>();
}

void shift::core::body::Fixed::addParameters() {
  removeParameter("name");
  removeParameter("inertia");
  removeParameter("center_of_mass");
  removeParameter("active");

  addParameter("terrain", &terrain_,
               "an array of one or more Shift terrain objects", false);
}
