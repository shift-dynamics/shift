#include "shift/core/controller/ik7dof.h"
#include "shift/core/factory.h"

bool shift::core::controller::IK7DOF::registered_ =
  shift::core::Factory<shift::core::controller::Base>::get()->RegisterModule(
    "ik_7dof", shift::core::controller::IK7DOF::Create);

std::unique_ptr<shift::core::controller::IK7DOF> shift::core::controller::IK7DOF::Create() {
  return std::make_unique<shift::core::controller::IK7DOF>();
}

void shift::core::controller::IK7DOF::DeclareParameters() {
  shift::core::controller::Base::DeclareParameters();
}
