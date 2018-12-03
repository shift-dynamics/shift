#include "shift-core/joints/spherical.h"
#include "shift-core/factory.h"

Shift::Registration<Shift::Joint::JointBase>
  registration("spherical", Shift::Joint::Spherical::create);

Shift::Joint::Spherical* Shift::Joint::Spherical::create() {
  return new Shift::Joint::Spherical();
}

void Shift::Joint::Spherical::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Spherical::updateDH() {}

void Shift::Joint::Spherical::updateDHVA() {}
