#include "shift-core/integrators/ode113.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Integrator::IntegratorBase>
  registration("pid", Shift::Integrator::ODE113::create);
