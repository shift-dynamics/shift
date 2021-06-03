#include "shift/core/behavior/Base.h"
#include "shift/core/behavior/ContactDetection.h"
// #include "shift/core/behavior/IK7DOF.h"
#include "shift/core/Context.h"

bool shift::core::behavior::Modules::is_initialized() {
  if(!shift::core::behavior::ContactDetection::is_registered()) {
    return false;
  }
//  if(!shift::core::behavior::IK7DOF::is_registered()) {
//    return false;
//  }
  return true;
}

void shift::core::behavior::Base::DeclareParameters() {}

void shift::core::behavior::Base::ActivateObjects(bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
};

void shift::core::behavior::Base::Load() {}

void shift::core::behavior::Base::Connect() {}
