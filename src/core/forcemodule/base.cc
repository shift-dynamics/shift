#include "shift/core/forcemodule/base.h"
#include "shift/core/context.h"
#include "shift/core/forcemodule/contact.h"
#include "shift/core/forcemodule/pid.h"

bool shift::ForceModule::Modules::initialized() {
  if(!shift::ForceModule::Contact::isRegistered()) {
    return false;
  }
  if(!shift::ForceModule::PID::isRegistered()) {
    return false;
  }
  return true;
}
void shift::ForceModule::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}

void shift::ForceModule::Base::addParameters() {}

void shift::ForceModule::Base::load() {}

void shift::ForceModule::Base::connect() {}