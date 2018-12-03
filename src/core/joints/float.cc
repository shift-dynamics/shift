#include "shift-core/joints/float.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Joint::Float>
  registration("pid", Shift::Joint::Float::create);

Shift::Joint::Float* Shift::Joint::Float::create() {
  return new Shift::Joint::Float();
}

void Shift::Joint::Float::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Float::updateDH() {}

void Shift::Joint::Float::updateDHVA() {}
