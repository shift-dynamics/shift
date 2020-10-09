#include "shift/core/collisiongeometry/base.h"
#include "shift/core/context.h"

bool shift::CollisionGeometry::Modules::initialized() {
  // if(!shift::CollisionGeometry::Fixed::isRegistered()) {
  //   return false;
  // }
  // if(!shift::CollisionGeometry::Free::isRegistered()) {
  //   return false;
  // }
  // if(!shift::CollisionGeometry::Rigid::isRegistered()) {
  //   return false;
  // }
  return true;
}

void shift::CollisionGeometry::Base::addParameters() {}

void shift::CollisionGeometry::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void shift::CollisionGeometry::Base::load() {}

void shift::CollisionGeometry::Base::connect() {}
