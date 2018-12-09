#include "shift-core/joints/spherical.h"
#include "shift-core/factory.h"

bool Shift::Joint::Spherical::registered_ =
  Shift::Factory<Shift::Joint::JointBase>::get()->registerModule(
    "spherical", Shift::Joint::Spherical::create);

Shift::Joint::Spherical* Shift::Joint::Spherical::create() {
  return new Shift::Joint::Spherical;
}

void Shift::Joint::Spherical::addKeys() {
  Shift::Joint::JointBase::addKeys();
}

void Shift::Joint::Spherical::updateDH() {}

void Shift::Joint::Spherical::updateDHVA() {}
