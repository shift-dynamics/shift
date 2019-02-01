#include "shift-core/geometry/ellipsoid.h"
#include "shift-core/factory.h"

bool Shift::Geometry::Ellipsoid::registered_ =
  Shift::Factory<Shift::Geometry::Base>::get()->registerModule(
    "ellipsoid", Shift::Geometry::Ellipsoid::create);

std::unique_ptr<Shift::Geometry::Ellipsoid>
Shift::Geometry::Ellipsoid::create() {
  return std::make_unique<Shift::Geometry::Ellipsoid>();
}

void Shift::Geometry::Ellipsoid::addKeys() {}
