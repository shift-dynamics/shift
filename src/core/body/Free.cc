#include "shift/core/body/Free.h"
#include "shift/core/Factory.h"

bool shift::core::body::Free::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "free", shift::core::body::Free::create);

std::unique_ptr<shift::core::body::Free> shift::core::body::Free::create() {
  return std::make_unique<shift::core::body::Free>();
}

void shift::core::body::Free::DeclareParameters() {}
