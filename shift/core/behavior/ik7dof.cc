#include "shift/core/behavior/ik7dof.h"
#include "shift/core/factory.h"

bool shift::core::behavior::IK7DOF::registered_ =
  shift::core::Factory<shift::core::behavior::Base>::get()->RegisterModule(
    "ik_7dof", shift::core::behavior::IK7DOF::Create);

std::unique_ptr<shift::core::behavior::IK7DOF> shift::core::behavior::IK7DOF::Create() {
  return std::make_unique<shift::core::behavior::IK7DOF>();
}

void shift::core::behavior::IK7DOF::DeclareParameters() {
  shift::core::behavior::Base::DeclareParameters();
}
