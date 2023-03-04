#include "shift/core/collision_geometry/Base.h"
#include "shift/core/Context.h"

bool shift::core::collision_geometry::Modules::is_initialized() {
  // if(!shift::core::collision_geometry::Fixed::is_registered()) {
  //   return false;
  // }
  // if(!shift::core::collision_geometry::Free::is_registered()) {
  //   return false;
  // }
  // if(!shift::core::collision_geometry::Rigid::is_registered()) {
  //   return false;
  // }
  return true;
}

void shift::core::collision_geometry::Base::DeclareParameters() {}

void shift::core::collision_geometry::Base::ActivateObjects(bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
};

void shift::core::collision_geometry::Base::Load() {}

void shift::core::collision_geometry::Base::Connect() {}
