#include "shift/core/geometry/sphere.h"
#include "shift/core/factory.h"

bool shift::Geometry::Sphere::registered_ =
  shift::Factory<shift::Geometry::Base>::get()->registerModule(
    "sphere", shift::Geometry::Sphere::create);

std::unique_ptr<shift::Geometry::Sphere> shift::Geometry::Sphere::create() {
  return std::make_unique<shift::Geometry::Sphere>();
}

void shift::Geometry::Sphere::addParameters() {}
