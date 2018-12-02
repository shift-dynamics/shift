#include "shift-core/forcemodules/pid.h"

class Shift::ForceModule::Registration
  registration("pid", Shift::ForceModule::PID::create);
