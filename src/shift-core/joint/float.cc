#include "shift-core/joint/float.h"
#include "shift-core/factory.h"

bool Shift::Joint::Float::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "float", Shift::Joint::Float::create);

std::unique_ptr<Shift::Joint::Float> Shift::Joint::Float::create() {
  return std::make_unique<Shift::Joint::Float>();
}

void Shift::Joint::Float::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Float::updateD() {}

void Shift::Joint::Float::updateDH() {}

void Shift::Joint::Float::updateDHVA() {}
