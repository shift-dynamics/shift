#include "shift/core/collisiongeometry/base.h"
#include "shift/core/context.h"

bool shift::core::collisiongeometry::Modules::initialized() {
  // if(!shift::core::collisiongeometry::Fixed::isRegistered()) {
  //   return false;
  // }
  // if(!shift::core::collisiongeometry::Free::isRegistered()) {
  //   return false;
  // }
  // if(!shift::core::collisiongeometry::Rigid::isRegistered()) {
  //   return false;
  // }
  return true;
}

void shift::core::collisiongeometry::Base::addParameters() {}

void shift::core::collisiongeometry::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void shift::core::collisiongeometry::Base::load() {}

void shift::core::collisiongeometry::Base::connect() {}
