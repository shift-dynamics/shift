#include "shift-core/geometry/base.h"
#include "shift-core/context.h"
#include "shift-core/geometry/ellipsoid.h"
#include "shift-core/geometry/mesh.h"
#include "shift-core/geometry/pointcloud.h"
#include "shift-core/geometry/sphere.h"

bool Shift::Geometry::Modules::initialized() {
  if(!Shift::Geometry::Ellipsoid::isRegistered()) {
    return false;
  }
  if(!Shift::Geometry::Mesh::isRegistered()) {
    return false;
  }
  if(!Shift::Geometry::PointCloud::isRegistered()) {
    return false;
  }
  if(!Shift::Geometry::Sphere::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Geometry::Base::addParameters() {
  addParameter("name", &name_, "User defined descriptive geometry name.", true);

  addParameter(
    "active", &active_,
    "Indicates whether geometry is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
}

void Shift::Geometry::Base::load() {}

void Shift::Geometry::Base::connect() {}
