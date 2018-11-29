#include "shift-core/joints/floatjoint.h"

Shift::FloatJoint* createFloatJoint() {
  return new Shift::FloatJoint();
}

Shift::JointRegistration registration("float", createFloatJoint);

void Shift::FloatJoint::addKeys() {
  Shift::Joint::addKeys();
}

void Shift::FloatJoint::updateDH() {}

void Shift::FloatJoint::updateDHVA() {}
