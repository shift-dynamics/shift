#include "shift-core/geometry/pointcloud.h"
#include "shift-core/factory.h"

bool Shift::Geometry::PointCloud::registered_ =
  Shift::Factory<Shift::Geometry::GeometryBase>::get()->registerModule(
    "pointcloud", Shift::Geometry::PointCloud::create);

std::unique_ptr<Shift::Geometry::PointCloud>
Shift::Geometry::PointCloud::create() {
  return std::make_unique<Shift::Geometry::PointCloud>();
}

void Shift::Geometry::PointCloud::addKeys() {}
