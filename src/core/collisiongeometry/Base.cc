#include "shift/core/collisiongeometry/Base.h"
#include "shift/core/Context.h"

bool shift::core::collisiongeometry::Modules::is_initialized() {
  // if(!shift::core::collisiongeometry::Fixed::is_registered()) {
  //   return false;
  // }
  // if(!shift::core::collisiongeometry::Free::is_registered()) {
  //   return false;
  // }
  // if(!shift::core::collisiongeometry::Rigid::is_registered()) {
  //   return false;
  // }
  return true;
}

void shift::core::collisiongeometry::Base::DeclareParameters() {}

void shift::core::collisiongeometry::Base::ActivateObjects(bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
};

void shift::core::collisiongeometry::Base::Load() {}

void shift::core::collisiongeometry::Base::Connect() {}
