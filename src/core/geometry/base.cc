#include "shift/core/geometry/base.h"
#include "shift/core/context.h"
#include "shift/core/geometry/ellipsoid.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/geometry/pointcloud.h"
#include "shift/core/geometry/sphere.h"

bool shift::core::geometry::Modules::initialized() {
  if(!shift::core::geometry::Ellipsoid::isRegistered()) {
    return false;
  }
  if(!shift::core::geometry::Mesh::isRegistered()) {
    return false;
  }
  if(!shift::core::geometry::PointCloud::isRegistered()) {
    return false;
  }
  if(!shift::core::geometry::Sphere::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::geometry::Base::addParameters() {
  addParameter("name", &name_, "User defined descriptive geometry name.", true);

  addParameter(
    "active", &active_,
    "Indicates whether geometry is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
}

void shift::core::geometry::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, active_ && parent_is_active);
};

void shift::core::geometry::Base::load() {}

void shift::core::geometry::Base::connect() {}
