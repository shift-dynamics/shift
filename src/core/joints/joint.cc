#include "shift-core/joints/joint.h"
#include "shift-core/joints/float.h"
#include "shift-core/joints/prismatic.h"
#include "shift-core/joints/revolute.h"
#include "shift-core/joints/rigid.h"
#include "shift-core/joints/spherical.h"

bool Shift::Joint::Modules::initialized() {
  if(!Shift::Joint::Float::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Prismatic::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Revolute::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Rigid::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Spherical::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Joint::JointBase::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  addSerialEntry(name);
  SerialEntry body_frame_pairs("body_frame_pairs", SerialType::STRING_2DARRAY,
                               &bodyFramePairs_, 2, 2);
  addSerialEntry(body_frame_pairs);
}

void Shift::Joint::JointBase::readJSON(Json::Value& value) {
  Serialize::readJSON(value);
}
