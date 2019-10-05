#include "shift-core/forcemodule/base.h"
#include "shift-core/context.h"
#include "shift-core/forcemodule/contact.h"
#include "shift-core/forcemodule/pid.h"

bool Shift::ForceModule::Modules::initialized() {
  if(!Shift::ForceModule::Contact::isRegistered()) {
    return false;
  }
  if(!Shift::ForceModule::PID::isRegistered()) {
    return false;
  }
  return true;
}
void Shift::ForceModule::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}

void Shift::ForceModule::Base::addParameters() {}

void Shift::ForceModule::Base::load() {}

void Shift::ForceModule::Base::connect() {}