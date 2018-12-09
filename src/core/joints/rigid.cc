#include "shift-core/joints/rigid.h"
#include "shift-core/factory.h"

bool Shift::Joint::Rigid::registered_ =
  Shift::Factory<Shift::Joint::JointBase>::get()->registerModule(
    "rigid", Shift::Joint::Rigid::create);

Shift::Joint::Rigid* Shift::Joint::Rigid::create() {
  return new Shift::Joint::Rigid;
}

void Shift::Joint::Rigid::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Rigid::updateDH() {}

void Shift::Joint::Rigid::updateDHVA() {}
