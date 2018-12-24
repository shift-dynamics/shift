#include "shift-core/behaviors/ik_7dof.h"
#include "shift-core/factory.h"

bool Shift::Behavior::IK7DOF::registered_ =
  Shift::Factory<Shift::Behavior::BehaviorBase>::get()->registerModule(
    "ik_7dof", Shift::Behavior::IK7DOF::create);

std::unique_ptr<Shift::Behavior::IK7DOF> Shift::Behavior::IK7DOF::create() {
  return std::make_unique<Shift::Behavior::IK7DOF>();
}

void Shift::Behavior::IK7DOF::addKeys() {
  Shift::Behavior::BehaviorBase::addKeys();
}
