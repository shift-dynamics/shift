#include "shift/core/controller/base.h"
#include "shift/core/context.h"
#include "shift/core/controller/ik7dof.h"
#include "shift/core/controller/map.h"
#include "shift/core/controller/pid.h"
#include "shift/core/controller/schedule.h"

/* referencing each module's registration keeps its object file, and so its
registration, in the static library */
bool shift::core::controller::Modules::is_initialized() {
  return shift::core::controller::IK7DOF::is_registered() &&
         shift::core::controller::PID::is_registered() &&
         shift::core::controller::Schedule::is_registered() &&
         shift::core::controller::Map::is_registered();
}

void shift::core::controller::Base::DeclareParameters() {
  DeclareParameter("name", &name_, "User defined descriptive controller name",
                   true);
  DeclareParameter("type", &type_, "controller type", true);
  DeclareParameter("active", &active_,
                   "false loads the controller without evaluating it", false);
}

void shift::core::controller::Base::ActivateObjects(bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
}

void shift::core::controller::Base::Load() {}

void shift::core::controller::Base::Connect() {}
