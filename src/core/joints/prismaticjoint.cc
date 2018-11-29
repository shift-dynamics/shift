#include "shift-core/joints/prismaticjoint.h"

Shift::PrismaticJoint* createPrismaticJoint() {
  return new Shift::PrismaticJoint();
}

Shift::JointRegistration registration("prismatic", createPrismaticJoint);

void Shift::PrismaticJoint::addKeys() {
  Shift::Joint::addKeys();
}

void Shift::PrismaticJoint::updateDH() {}

void Shift::PrismaticJoint::updateDHVA() {}
