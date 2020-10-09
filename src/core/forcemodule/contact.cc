#include "shift/core/forcemodule/contact.h"
#include "shift/core/factory.h"

bool shift::ForceModule::Contact::registered_ =
  shift::Factory<shift::ForceModule::Base>::get()->registerModule(
    "contact", shift::ForceModule::Contact::create);

std::unique_ptr<shift::ForceModule::Contact>
shift::ForceModule::Contact::create() {
  return std::make_unique<shift::ForceModule::Contact>();
}

void shift::ForceModule::Contact::addParameters() {
  shift::ForceModule::Base::addParameters();
}
