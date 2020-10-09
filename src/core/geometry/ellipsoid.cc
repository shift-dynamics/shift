#include "shift/core/geometry/ellipsoid.h"
#include "shift/core/factory.h"

bool shift::Geometry::Ellipsoid::registered_ =
  shift::Factory<shift::Geometry::Base>::get()->registerModule(
    "ellipsoid", shift::Geometry::Ellipsoid::create);

std::unique_ptr<shift::Geometry::Ellipsoid>
shift::Geometry::Ellipsoid::create() {
  return std::make_unique<shift::Geometry::Ellipsoid>();
}

void shift::Geometry::Ellipsoid::addParameters() {}
