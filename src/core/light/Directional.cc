#include "shift/core/light/Directional.h"
#include "shift/core/light/Base.h"

bool shift::core::light::Directional::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->RegisterModule(
    "directional", shift::core::light::Directional::Create);

std::unique_ptr<shift::core::light::Directional> shift::core::light::Directional::Create() {
  return std::make_unique<shift::core::light::Directional>();
}

void shift::core::light::Directional::DeclareParameters() {
  shift::core::light::Base::DeclareParameters();
}
