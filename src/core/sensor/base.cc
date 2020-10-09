#include "shift/core/sensor/base.h"
#include "shift/core/context.h"

bool Shift::Sensor::Modules::initialized() {
  return true;
}

void Shift::Sensor::Base::load() {}

void Shift::Sensor::Base::connect() {}

void Shift::Sensor::Base::addParameters() {}

void Shift::Sensor::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
