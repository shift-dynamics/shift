#include "shift-core/integrators/integrator.h"
#include "shift-core/integrators/ode113.h"

bool Shift::Integrator::Modules::initialized() {
  if(!Shift::Integrator::ODE113::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Integrator::IntegratorBase::readJSON(Json::Value) {}

void Shift::Integrator::IntegratorBase::addKeys() {}
