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

  addParameter("frames", &frames_,
               "reference coordinate frames, attached to the fixed inertial "
               "reference frame",
               false);

  addParameter("lights", &lights_,
               "an array of one or more lights to illuminate scene, if not "
               "light is specified, a single directional overhead light source "
               "in the opposite direction of the gravity vector will be added",
               false);

  addParameter("terrain", &terrain_,
               "an array of one or more Shift terrain objects", false);
}
