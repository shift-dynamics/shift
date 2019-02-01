#include "shift-core/joint/rigid.h"
#include "shift-core/factory.h"

bool Shift::Joint::Rigid::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "rigid", Shift::Joint::Rigid::create);

std::unique_ptr<Shift::Joint::Rigid> Shift::Joint::Rigid::create() {
  return std::make_unique<Shift::Joint::Rigid>();
}

void Shift::Joint::Rigid::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Rigid::updateD() {}

void Shift::Joint::Rigid::updateDH() {}

void Shift::Joint::Rigid::updateDHVA() {}
