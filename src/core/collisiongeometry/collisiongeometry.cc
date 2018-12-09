#include "shift-core/collisiongeometry/collisiongeometry.h"

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
