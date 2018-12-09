#include "shift-core/geometry/sphere.h"
#include "shift-core/factory.h"

bool Shift::Geometry::Sphere::registered_ =
  Shift::Factory<Shift::Geometry::GeometryBase>::get()->registerModule(
    "sphere", Shift::Geometry::Sphere::create);

Shift::Geometry::Sphere* Shift::Geometry::Sphere::create() {
  return new Shift::Geometry::Sphere;
}

void Shift::Geometry::Sphere::addKeys() {}
