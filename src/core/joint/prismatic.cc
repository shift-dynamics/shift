#include "shift-core/joint/prismatic.h"
#include "shift-core/factory.h"

bool Shift::Joint::Prismatic::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "prismatic", Shift::Joint::Prismatic::create);

std::unique_ptr<Shift::Joint::Prismatic> Shift::Joint::Prismatic::create() {
  return std::make_unique<Shift::Joint::Prismatic>();
}

void Shift::Joint::Prismatic::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Prismatic::updateD() {}

void Shift::Joint::Prismatic::updateDH() {}

void Shift::Joint::Prismatic::updateDHVA() {}
