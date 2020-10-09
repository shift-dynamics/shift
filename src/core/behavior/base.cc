#include "shift/core/behavior/base.h"
#include "shift/core/behavior/contactdetection.h"
#include "shift/core/behavior/ik_7dof.h"
#include "shift/core/context.h"

bool Shift::Behavior::Modules::initialized() {
  if(!Shift::Behavior::ContactDetection::isRegistered()) {
    return false;
  }
  if(!Shift::Behavior::IK7DOF::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Behavior::Base::addParameters() {}

void Shift::Behavior::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void Shift::Behavior::Base::load() {}

void Shift::Behavior::Base::connect() {}
