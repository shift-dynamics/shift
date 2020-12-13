#include "shift/core/geometry/Sphere.h"
#include "shift/core/Factory.h"

bool shift::core::geometry::Sphere::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->registerModule(
    "sphere", shift::core::geometry::Sphere::create);

std::unique_ptr<shift::core::geometry::Sphere> shift::core::geometry::Sphere::create() {
  return std::make_unique<shift::core::geometry::Sphere>();
}

void shift::core::geometry::Sphere::addParameters() {}
