#include "shift/core/sensor/base.h"
#include "shift/core/context.h"

bool shift::core::sensor::Modules::initialized() {
  return true;
}

void shift::core::sensor::Base::load() {}

void shift::core::sensor::Base::connect() {}

void shift::core::sensor::Base::addParameters() {}

void shift::core::sensor::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
