#include "shift/core/behavior/ik_7dof.h"
#include "shift/core/factory.h"

bool shift::Behavior::IK7DOF::registered_ =
  shift::Factory<shift::Behavior::Base>::get()->registerModule(
    "ik_7dof", shift::Behavior::IK7DOF::create);

std::unique_ptr<shift::Behavior::IK7DOF> shift::Behavior::IK7DOF::create() {
  return std::make_unique<shift::Behavior::IK7DOF>();
}

void shift::Behavior::IK7DOF::addParameters() {
  shift::Behavior::Base::addParameters();
}
