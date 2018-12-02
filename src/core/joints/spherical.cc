#include "shift-core/joints/spherical.h"

Shift::Joint::Spherical* createSphericalJoint() {
  return new Shift::Joint::Spherical();
}

Shift::Joint::Registration registration("spherical", createSphericalJoint);

void Shift::Joint::Spherical::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Spherical::updateDH() {}

void Shift::Joint::Spherical::updateDHVA() {}
