#include "shift-core/forcemodules/contact.h"
#include "shift-core/factory.h"

bool Shift::ForceModule::Contact::registered_ =
  Shift::Factory<Shift::ForceModule::ForceModuleBase>::get()->registerModule(
    "contact", Shift::ForceModule::Contact::create);

Shift::ForceModule::Contact* Shift::ForceModule::Contact::create() {
  return new Shift::ForceModule::Contact;
}

void Shift::ForceModule::Contact::addKeys() {
  Shift::ForceModule::ForceModuleBase::addKeys();
}
