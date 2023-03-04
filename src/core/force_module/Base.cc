#include "shift/core/force_module/Base.h"
#include "shift/core/Context.h"
#include "shift/core/force_module/Contact.h"
#include "shift/core/force_module/PID.h"

bool shift::core::force_module::Modules::is_initialized() {
  if(!shift::core::force_module::Contact::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::PID::is_registered()) {
    return false;
  }
  return true;
}
void shift::core::force_module::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}

void shift::core::force_module::Base::DeclareParameters() {}

void shift::core::force_module::Base::Load() {}

void shift::core::force_module::Base::Connect() {}
