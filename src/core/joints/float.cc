#include "shift-core/joints/float.h"
#include "shift-core/factory.h"

bool Shift::Joint::Float::registered_ =
  Shift::Factory<Shift::Joint::JointBase>::get()->registerModule(
    "float", Shift::Joint::Float::create);

Shift::Joint::Float* Shift::Joint::Float::create() {
  return new Shift::Joint::Float;
}

void Shift::Joint::Float::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Float::updateDH() {}

void Shift::Joint::Float::updateDHVA() {}
