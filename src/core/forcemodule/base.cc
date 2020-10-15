#include "shift/core/forcemodule/base.h"
#include "shift/core/context.h"
#include "shift/core/forcemodule/contact.h"
#include "shift/core/forcemodule/pid.h"

bool shift::core::forcemodule::Modules::initialized() {
  if(!shift::core::forcemodule::Contact::isRegistered()) {
    return false;
  }
  if(!shift::core::forcemodule::PID::isRegistered()) {
    return false;
  }
  return true;
}
void shift::core::forcemodule::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}

void shift::core::forcemodule::Base::addParameters() {}

void shift::core::forcemodule::Base::load() {}

void shift::core::forcemodule::Base::connect() {}