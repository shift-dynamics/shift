#include "shift/core/light/ies.h"

bool shift::core::light::IES::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->RegisterModule(
    "ies", shift::core::light::IES::Create);

std::unique_ptr<shift::core::light::IES> shift::core::light::IES::Create() {
  return std::make_unique<shift::core::light::IES>();
}

void shift::core::light::IES::DeclareParameters() {
  shift::core::light::Base::DeclareParameters();
}
