#include "shift/core/geometry/pointcloud.h"
#include "shift/core/factory.h"

bool shift::Geometry::PointCloud::registered_ =
  shift::Factory<shift::Geometry::Base>::get()->registerModule(
    "pointcloud", shift::Geometry::PointCloud::create);

std::unique_ptr<shift::Geometry::PointCloud>
shift::Geometry::PointCloud::create() {
  return std::make_unique<shift::Geometry::PointCloud>();
}

void shift::Geometry::PointCloud::addParameters() {}
