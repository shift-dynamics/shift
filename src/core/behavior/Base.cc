#include "shift/core/behavior/Base.h"
#include "shift/core/behavior/ContactDetection.h"
// #include "shift/core/behavior/IK7DOF.h"
#include "shift/core/Context.h"

bool shift::core::behavior::Modules::initialized() {
  if(!shift::core::behavior::ContactDetection::isRegistered()) {
    return false;
  }
//  if(!shift::core::behavior::IK7DOF::isRegistered()) {
//    return false;
//  }
  return true;
}

void shift::core::behavior::Base::addParameters() {}

void shift::core::behavior::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void shift::core::behavior::Base::load() {}

void shift::core::behavior::Base::connect() {}
