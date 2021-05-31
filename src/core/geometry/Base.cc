#include "shift/core/geometry/Base.h"
#include "shift/core/Context.h"
#include "shift/core/geometry/Ellipsoid.h"
#include "shift/core/geometry/Mesh.h"
#include "shift/core/geometry/PointCloud.h"
#include "shift/core/geometry/Sphere.h"

bool shift::core::geometry::Modules::is_initialized() {
  if(!shift::core::geometry::Ellipsoid::is_registered()) {
    return false;
  }
  if(!shift::core::geometry::Mesh::is_registered()) {
    return false;
  }
  if(!shift::core::geometry::PointCloud::is_registered()) {
    return false;
  }
  if(!shift::core::geometry::Sphere::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::geometry::Base::DeclareParameters() {
  DeclareParameter("name", &name_, "User defined descriptive geometry name.", true);

  DeclareParameter(
    "active", &active_,
    "Indicates whether geometry is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
}

void shift::core::geometry::Base::set_objects_active(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, active_ && parent_is_active);
};

void shift::core::geometry::Base::Load() {}

void shift::core::geometry::Base::Connect() {}
