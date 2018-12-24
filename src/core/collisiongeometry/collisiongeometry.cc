#include "shift-core/collisiongeometry/collisiongeometry.h"
#include "shift-core/context.h"

bool Shift::CollisionGeometry::Modules::initialized() {
  // if(!Shift::CollisionGeometry::Fixed::isRegistered()) {
  //   return false;
  // }
  // if(!Shift::CollisionGeometry::Free::isRegistered()) {
  //   return false;
  // }
  // if(!Shift::CollisionGeometry::Rigid::isRegistered()) {
  //   return false;
  // }
  return true;
}

void Shift::CollisionGeometry::CollisionGeometryBase::load() {}

void Shift::CollisionGeometry::CollisionGeometryBase::connect() {}

void Shift::CollisionGeometry::CollisionGeometryBase::readJSON(
  Json::Value& value) {
  /* iterate over keys */
  Shift::Serialize::readJSON(value);
}
