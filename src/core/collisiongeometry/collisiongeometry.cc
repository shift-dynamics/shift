#include "shift-core/collisiongeometry/collisiongeometry.h"

void Shift::CollisionGeometry::Factory::registerModule(
  const std::string& name, CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
