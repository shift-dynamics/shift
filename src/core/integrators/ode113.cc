#include "shift-core/integrators/ode113.h"

class Shift::Integrator::Registration
  registration("ode113", Shift::Integrator::ODE113::create);
