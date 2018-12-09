#include "shift-core/joints/prismatic.h"
#include "shift-core/factory.h"

bool Shift::Joint::Prismatic::registered_ =
  Shift::Factory<Shift::Joint::JointBase>::get()->registerModule(
    "prismatic", Shift::Joint::Prismatic::create);

Shift::Joint::Prismatic* Shift::Joint::Prismatic::create() {
  return new Shift::Joint::Prismatic;
}

void Shift::Joint::Prismatic::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Prismatic::updateDH() {}

void Shift::Joint::Prismatic::updateDHVA() {}
