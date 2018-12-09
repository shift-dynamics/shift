#include "shift-core/integrators/ode113.h"
#include "shift-core/factory.h"

bool Shift::Integrator::ODE113::registered_ =
  Shift::Factory<Shift::Integrator::IntegratorBase>::get()->registerModule(
    "ode113", Shift::Integrator::ODE113::create);

Shift::Integrator::ODE113* Shift::Integrator::ODE113::create() {
  return new Shift::Integrator::ODE113;
}

void Shift::Integrator::ODE113::addKeys() {}
