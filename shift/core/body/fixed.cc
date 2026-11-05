#include "shift/core/body/fixed.h"
#include "shift/core/factory.h"

bool shift::core::body::Fixed::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "fixed", shift::core::body::Fixed::Create);

std::unique_ptr<shift::core::body::Fixed> shift::core::body::Fixed::Create() {
  return std::make_unique<shift::core::body::Fixed>();
}

void shift::core::body::Fixed::DeclareParameters() {
  RemoveParameter("name");
  RemoveParameter("inertia");
  RemoveParameter("center_of_mass");
  RemoveParameter("active");

  DeclareParameter("terrain", &terrain_,
               "an array of one or more Shift terrain objects", false);
}
