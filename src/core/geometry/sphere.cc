#include "shift-core/geometry/sphere.h"
#include "shift-core/factory.h"

bool Shift::Geometry::Sphere::registered_ =
  Shift::Factory<Shift::Geometry::GeometryBase>::get()->registerModule(
    "sphere", Shift::Geometry::Sphere::create);

std::unique_ptr<Shift::Geometry::Sphere> Shift::Geometry::Sphere::create() {
  return std::make_unique<Shift::Geometry::Sphere>();
}

void Shift::Geometry::Sphere::addKeys() {}
