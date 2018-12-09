#include "shift-core/geometry/ellipsoid.h"
#include "shift-core/factory.h"

bool Shift::Geometry::Ellipsoid::registered_ =
  Shift::Factory<Shift::Geometry::GeometryBase>::get()->registerModule(
    "ellipsoid", Shift::Geometry::Ellipsoid::create);

Shift::Geometry::Ellipsoid* Shift::Geometry::Ellipsoid::create() {
  return new Shift::Geometry::Ellipsoid;
}

void Shift::Geometry::Ellipsoid::addKeys() {}
