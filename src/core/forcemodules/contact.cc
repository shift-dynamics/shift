#include "shift-core/forcemodules/contact.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::ForceModule::ForceModuleBase>
  registration("contact", Shift::ForceModule::Contact::create);
