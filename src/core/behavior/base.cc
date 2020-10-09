#include "shift/core/behavior/base.h"
#include "shift/core/behavior/contactdetection.h"
#include "shift/core/behavior/ik_7dof.h"
#include "shift/core/context.h"

bool shift::Behavior::Modules::initialized() {
  if(!shift::Behavior::ContactDetection::isRegistered()) {
    return false;
  }
  if(!shift::Behavior::IK7DOF::isRegistered()) {
    return false;
  }
  return true;
}

void shift::Behavior::Base::addParameters() {}

void shift::Behavior::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void shift::Behavior::Base::load() {}

void shift::Behavior::Base::connect() {}
