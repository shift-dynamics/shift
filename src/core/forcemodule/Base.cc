#include "shift/core/forcemodule/Base.h"
#include "shift/core/Context.h"
#include "shift/core/forcemodule/Contact.h"
#include "shift/core/forcemodule/PID.h"

bool shift::core::forcemodule::Modules::is_initialized() {
  if(!shift::core::forcemodule::Contact::is_registered()) {
    return false;
  }
  if(!shift::core::forcemodule::PID::is_registered()) {
    return false;
  }
  return true;
}
void shift::core::forcemodule::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}

void shift::core::forcemodule::Base::DeclareParameters() {}

void shift::core::forcemodule::Base::Load() {}

void shift::core::forcemodule::Base::Connect() {}
