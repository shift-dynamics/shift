#include "shift/core/forcemodule/Contact.h"
#include "shift/core/Factory.h"

bool shift::core::forcemodule::Contact::registered_ =
  shift::core::Factory<shift::core::forcemodule::Base>::get()->RegisterModule(
    "contact", shift::core::forcemodule::Contact::Create);

std::unique_ptr<shift::core::forcemodule::Contact>
shift::core::forcemodule::Contact::Create() {
  return std::make_unique<shift::core::forcemodule::Contact>();
}

void shift::core::forcemodule::Contact::DeclareParameters() {
  shift::core::forcemodule::Base::DeclareParameters();
}
