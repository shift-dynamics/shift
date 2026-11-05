#include "shift/core/geometry/point_cloud.h"
#include "shift/core/factory.h"

bool shift::core::geometry::PointCloud::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->RegisterModule(
    "pointcloud", shift::core::geometry::PointCloud::create);

std::unique_ptr<shift::core::geometry::PointCloud>
shift::core::geometry::PointCloud::create() {
  return std::make_unique<shift::core::geometry::PointCloud>();
}

void shift::core::geometry::PointCloud::DeclareParameters() {}
