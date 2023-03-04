#include "shift/core/force_module/Contact.h"
#include "shift/core/Factory.h"

bool shift::core::force_module::Contact::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "contact", shift::core::force_module::Contact::Create);

std::unique_ptr<shift::core::force_module::Contact>
shift::core::force_module::Contact::Create() {
  return std::make_unique<shift::core::force_module::Contact>();
}

void shift::core::force_module::Contact::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
}
