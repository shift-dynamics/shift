#include "shift-core/forcemodules/pid.h"
#include "shift-core/factory.h"

bool Shift::ForceModule::PID::registered_ =
  Shift::Factory<Shift::ForceModule::ForceModuleBase>::get()->registerModule(
    "pid", Shift::ForceModule::PID::create);

Shift::ForceModule::PID* Shift::ForceModule::PID::create() {
  return new Shift::ForceModule::PID;
}

void Shift::ForceModule::PID::addKeys() {
  Shift::ForceModule::ForceModuleBase::addKeys();
}
