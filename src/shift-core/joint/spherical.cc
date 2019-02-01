#include "shift-core/joint/spherical.h"
#include "shift-core/factory.h"

bool Shift::Joint::Spherical::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "spherical", Shift::Joint::Spherical::create);

std::unique_ptr<Shift::Joint::Spherical> Shift::Joint::Spherical::create() {
  return std::make_unique<Shift::Joint::Spherical>();
}

void Shift::Joint::Spherical::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Spherical::updateD() {}

void Shift::Joint::Spherical::updateDH() {}

void Shift::Joint::Spherical::updateDHVA() {}
