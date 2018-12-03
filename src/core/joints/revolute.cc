#include "shift-core/joints/revolute.h"
#include "shift-core/factory.h"

Shift::Registration<Shift::Joint::JointBase>
  registration("revolute", Shift::Joint::Revolute::create);

Shift::Joint::Revolute* Shift::Joint::Revolute::create() {
  return new Shift::Joint::Revolute();
}

void Shift::Joint::Revolute::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Revolute::updateDH() {}

void Shift::Joint::Revolute::updateDHVA() {}
