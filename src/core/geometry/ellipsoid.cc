#include "shift/core/geometry/ellipsoid.h"
#include "shift/core/factory.h"

bool shift::core::geometry::Ellipsoid::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->registerModule(
    "ellipsoid", shift::core::geometry::Ellipsoid::create);

std::unique_ptr<shift::core::geometry::Ellipsoid>
shift::core::geometry::Ellipsoid::create() {
  return std::make_unique<shift::core::geometry::Ellipsoid>();
}

void shift::core::geometry::Ellipsoid::addParameters() {}
