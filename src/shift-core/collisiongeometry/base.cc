#include "shift-core/collisiongeometry/base.h"
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

void Shift::CollisionGeometry::Base::load() {}

void Shift::CollisionGeometry::Base::connect() {}
