#include "shift-core/joints/revolutejoint.h"

Shift::RevoluteJoint* createRevoluteJoint() {
  return new Shift::RevoluteJoint();
}

Shift::JointRegistration registration("revolute", createRevoluteJoint);

void Shift::RevoluteJoint::addKeys() {
  Shift::Joint::addKeys();
}

void Shift::RevoluteJoint::updateDH() {}

void Shift::RevoluteJoint::updateDHVA() {}
