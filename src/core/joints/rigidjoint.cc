#include "shift-core/joints/rigidjoint.h"

Shift::RigidJoint* createRevoluteJoint() {
  return new Shift::RigidJoint();
}

Shift::JointRegistration registration("rigid", createRevoluteJoint);

void Shift::RigidJoint::addKeys() {
  Shift::Joint::addKeys();
}

void Shift::RigidJoint::updateDH() {}

void Shift::RigidJoint::updateDHVA() {}
