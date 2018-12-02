#include "shift-core/forcemodules/contact.h"

class Shift::ForceModule::Registration
  registration("contact", Shift::ForceModule::Contact::create);
