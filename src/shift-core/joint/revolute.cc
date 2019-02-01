#include "shift-core/joint/revolute.h"
#include "shift-core/factory.h"

bool Shift::Joint::Revolute::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "revolute", Shift::Joint::Revolute::create);

std::unique_ptr<Shift::Joint::Revolute> Shift::Joint::Revolute::create() {
  return std::make_unique<Shift::Joint::Revolute>();
}

void Shift::Joint::Revolute::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Revolute::updateD() {}

void Shift::Joint::Revolute::updateDH() {}

void Shift::Joint::Revolute::updateDHVA() {}
