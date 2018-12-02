#include "shift-core/joints/prismatic.h"

Shift::Joint::Prismatic* createPrismaticJoint() {
  return new Shift::Joint::Prismatic();
}

Shift::Joint::Registration registration("prismatic", createPrismaticJoint);

void Shift::Joint::Prismatic::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Prismatic::updateDH() {}

void Shift::Joint::Prismatic::updateDHVA() {}
