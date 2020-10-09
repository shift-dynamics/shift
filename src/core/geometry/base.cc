#include "shift/core/geometry/base.h"
#include "shift/core/context.h"
#include "shift/core/geometry/ellipsoid.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/geometry/pointcloud.h"
#include "shift/core/geometry/sphere.h"

bool shift::Geometry::Modules::initialized() {
  if(!shift::Geometry::Ellipsoid::isRegistered()) {
    return false;
  }
  if(!shift::Geometry::Mesh::isRegistered()) {
    return false;
  }
  if(!shift::Geometry::PointCloud::isRegistered()) {
    return false;
  }
  if(!shift::Geometry::Sphere::isRegistered()) {
    return false;
  }
  return true;
}

void shift::Geometry::Base::addParameters() {
  addParameter("name", &name_, "User defined descriptive geometry name.", true);

  addParameter(
    "active", &active_,
    "Indicates whether geometry is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
}

void shift::Geometry::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, active_ && parent_is_active);
};

void shift::Geometry::Base::load() {}

void shift::Geometry::Base::connect() {}
