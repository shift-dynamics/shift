#include "shift/core/light/spot.h"

bool shift::core::light::Spot::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->RegisterModule(
    "spot", shift::core::light::Spot::Create);

std::unique_ptr<shift::core::light::Spot> shift::core::light::Spot::Create() {
  return std::make_unique<shift::core::light::Spot>();
}

void shift::core::light::Spot::DeclareParameters() {
  shift::core::light::Base::DeclareParameters();
}
