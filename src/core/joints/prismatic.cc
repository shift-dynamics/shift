#include "shift-core/joints/prismatic.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Joint::Prismatic>
  registration("prismatic", Shift::Joint::Prismatic::create);

Shift::Joint::Prismatic* Shift::Joint::Prismatic::create() {
  return new Shift::Joint::Prismatic();
}

void Shift::Joint::Prismatic::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Prismatic::updateDH() {}

void Shift::Joint::Prismatic::updateDHVA() {}
