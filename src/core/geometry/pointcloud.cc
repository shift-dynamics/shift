#include "shift-core/geometry/pointcloud.h"
#include "shift-core/factory.h"

bool Shift::Geometry::PointCloud::registered_ =
  Shift::Factory<Shift::Geometry::GeometryBase>::get()->registerModule(
    "pointcloud", Shift::Geometry::PointCloud::create);

Shift::Geometry::PointCloud* Shift::Geometry::PointCloud::create() {
  return new Shift::Geometry::PointCloud;
}

void Shift::Geometry::PointCloud::addKeys() {}
