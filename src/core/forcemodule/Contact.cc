#include "shift/core/forcemodule/Contact.h"
#include "shift/core/Factory.h"

bool shift::core::forcemodule::Contact::registered_ =
  shift::core::Factory<shift::core::forcemodule::Base>::get()->registerModule(
    "contact", shift::core::forcemodule::Contact::create);

std::unique_ptr<shift::core::forcemodule::Contact>
shift::core::forcemodule::Contact::create() {
  return std::make_unique<shift::core::forcemodule::Contact>();
}

void shift::core::forcemodule::Contact::addParameters() {
  shift::core::forcemodule::Base::addParameters();
}
