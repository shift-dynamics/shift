#include "shift-core/forcemodules/contact.h"
#include "shift-core/factory.h"

bool Shift::ForceModule::Contact::registered_ =
  Shift::Factory<Shift::ForceModule::ForceModuleBase>::get()->registerModule(
    "contact", Shift::ForceModule::Contact::create);

std::unique_ptr<Shift::ForceModule::Contact>
Shift::ForceModule::Contact::create() {
  return std::make_unique<Shift::ForceModule::Contact>();
}

void Shift::ForceModule::Contact::addKeys() {
  Shift::ForceModule::ForceModuleBase::addKeys();
}
