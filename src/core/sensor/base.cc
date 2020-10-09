#include "shift/core/sensor/base.h"
#include "shift/core/context.h"

bool shift::Sensor::Modules::initialized() {
  return true;
}

void shift::Sensor::Base::load() {}

void shift::Sensor::Base::connect() {}

void shift::Sensor::Base::addParameters() {}

void shift::Sensor::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
