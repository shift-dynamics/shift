#include "shift-core/joints/rigid.h"

Shift::Joint::Rigid* createRevoluteJoint() {
  return new Shift::Joint::Rigid();
}

Shift::Joint::Registration registration("rigid", createRevoluteJoint);

void Shift::Joint::Rigid::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Rigid::updateDH() {}

void Shift::Joint::Rigid::updateDHVA() {}
