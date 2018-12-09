#include "shift-core/forcemodules/forcemodule.h"
#include "shift-core/forcemodules/contact.h"
#include "shift-core/forcemodules/pid.h"

bool Shift::ForceModule::Modules::initialized() {
  if(!Shift::ForceModule::Contact::isRegistered()) {
    return false;
  }
  if(!Shift::ForceModule::PID::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::ForceModule::ForceModuleBase::readJSON(Json::Value) {}

void Shift::ForceModule::ForceModuleBase::addKeys() {}
