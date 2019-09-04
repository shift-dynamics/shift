#include "shift-core/integrator/base.h"
#include "shift-core/context.h"
#include "shift-core/integrator/ode113.h"

bool Shift::Integrator::Modules::initialized() {
  if(!Shift::Integrator::ODE113::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Integrator::Base::setActiveObjects(bool parent_is_active) {
  if(context_) {
    context_->setActive(this, active_ && parent_is_active);
  }
};

void Shift::Integrator::Base::addParameters() {}

void Shift::Integrator::Base::load() {}

void Shift::Integrator::Base::connect() {}
