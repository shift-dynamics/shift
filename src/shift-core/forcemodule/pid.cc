#include "shift-core/forcemodule/pid.h"
#include "shift-core/factory.h"

bool Shift::ForceModule::PID::registered_ =
  Shift::Factory<Shift::ForceModule::Base>::get()->registerModule(
    "pid", Shift::ForceModule::PID::create);

std::unique_ptr<Shift::ForceModule::PID> Shift::ForceModule::PID::create() {
  return std::make_unique<Shift::ForceModule::PID>();
}

void Shift::ForceModule::PID::addParameters() {
  Shift::ForceModule::Base::addParameters();
}
