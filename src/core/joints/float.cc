#include "shift-core/joints/float.h"

Shift::Joint::Float* createFloatJoint() {
  return new Shift::Joint::Float();
}

Shift::Joint::Registration registration("float", createFloatJoint);

void Shift::Joint::Float::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Float::updateDH() {}

void Shift::Joint::Float::updateDHVA() {}
