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

void Shift::CollisionGeometry::Base::addParameters() {}

void Shift::CollisionGeometry::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void Shift::CollisionGeometry::Base::load() {}

void Shift::CollisionGeometry::Base::connect() {}
