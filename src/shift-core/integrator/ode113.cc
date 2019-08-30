#include "shift-core/integrator/ode113.h"
#include "shift-core/factory.h"

bool Shift::Integrator::ODE113::registered_ =
  Shift::Factory<Shift::Integrator::Base>::get()->registerModule(
    "ode113", Shift::Integrator::ODE113::create);

std::unique_ptr<Shift::Integrator::ODE113> Shift::Integrator::ODE113::create() {
  return std::make_unique<Shift::Integrator::ODE113>();
}

void Shift::Integrator::ODE113::addParameters() {}
