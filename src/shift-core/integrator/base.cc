#include "shift-core/integrator/base.h"
#include "shift-core/context.h"
#include "shift-core/integrator/ode113.h"

bool Shift::Integrator::Modules::initialized() {
  if(!Shift::Integrator::ODE113::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Integrator::Base::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
}

void Shift::Integrator::Base::addKeys() {}

void Shift::Integrator::Base::load() {}

void Shift::Integrator::Base::connect() {}
