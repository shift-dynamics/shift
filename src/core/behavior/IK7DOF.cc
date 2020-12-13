#include "shift/core/behavior/IK7DOF.h"
#include "shift/core/Factory.h"

bool shift::core::behavior::IK7DOF::registered_ =
  shift::core::Factory<shift::core::behavior::Base>::get()->registerModule(
    "ik_7dof", shift::core::behavior::IK7DOF::create);

std::unique_ptr<shift::core::behavior::IK7DOF> shift::core::behavior::IK7DOF::create() {
  return std::make_unique<shift::core::behavior::IK7DOF>();
}

void shift::core::behavior::IK7DOF::addParameters() {
  shift::core::behavior::Base::addParameters();
}
