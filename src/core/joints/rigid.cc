#include "shift-core/joints/rigid.h"
#include "shift-core/factory.h"

Shift::Registration<Shift::Joint::JointBase>
  registration("revolute", Shift::Joint::Rigid::create);

Shift::Joint::Rigid* Shift::Joint::Rigid::create() {
  return new Shift::Joint::Rigid();
}

void Shift::Joint::Rigid::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Rigid::updateDH() {}

void Shift::Joint::Rigid::updateDHVA() {}
