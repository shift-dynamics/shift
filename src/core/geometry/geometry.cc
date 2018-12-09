#include "shift-core/geometry/geometry.h"

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

void Shift::Geometry::GeometryBase::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(false);
  name.setDescription("User defined descriptive geometry name.");
  addSerialEntry(name);

  SerialEntry active("active", SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether geometry is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.");
  addSerialEntry(active);
}
