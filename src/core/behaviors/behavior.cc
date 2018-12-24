#include "shift-core/behaviors/behavior.h"
#include "shift-core/behaviors/contactdetection.h"
#include "shift-core/behaviors/ik_7dof.h"
#include "shift-core/context.h"

bool Shift::Behavior::Modules::initialized() {
  if(!Shift::Behavior::ContactDetection::isRegistered()) {
    return false;
  }
  if(!Shift::Behavior::IK7DOF::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Behavior::BehaviorBase::readJSON(Json::Value& value) {

  /* iterate over keys */
  Shift::Serialize::readJSON(value);
}

void Shift::Behavior::BehaviorBase::addKeys() {}

void Shift::Behavior::BehaviorBase::load() {}

void Shift::Behavior::BehaviorBase::connect() {}
