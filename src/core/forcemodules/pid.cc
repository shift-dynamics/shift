#include "shift-core/forcemodules/pid.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::ForceModule::ForceModuleBase>
  registration("pid", Shift::ForceModule::PID::create);
