#include "shift-core/joints/sphericaljoint.h"

Shift::SphericalJoint* createSphericalJoint() {
  return new Shift::SphericalJoint();
}

Shift::JointRegistration registration("spherical", createSphericalJoint);

void Shift::SphericalJoint::addKeys() {
  Shift::Joint::addKeys();
}

void Shift::SphericalJoint::updateDH() {}

void Shift::SphericalJoint::updateDHVA() {}
