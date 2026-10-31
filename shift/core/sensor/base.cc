#include "shift/core/sensor/base.h"
#include "shift/core/context.h"

bool shift::core::sensor::Modules::is_initialized() {
  return true;
}

void shift::core::sensor::Base::Load() {}

void shift::core::sensor::Base::Connect() {}

void shift::core::sensor::Base::DeclareParameters() {}

void shift::core::sensor::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
