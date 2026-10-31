#include "shift/core/geometry/sphere.h"
#include "shift/core/factory.h"

bool shift::core::geometry::Sphere::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->RegisterModule(
    "sphere", shift::core::geometry::Sphere::create);

std::unique_ptr<shift::core::geometry::Sphere> shift::core::geometry::Sphere::create() {
  return std::make_unique<shift::core::geometry::Sphere>();
}

void shift::core::geometry::Sphere::DeclareParameters() {}
