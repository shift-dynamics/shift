#include "shift/core/light/point.h"

bool shift::core::light::Point::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->RegisterModule(
    "point", shift::core::light::Point::Create);

std::unique_ptr<shift::core::light::Point> shift::core::light::Point::Create() {
  return std::make_unique<shift::core::light::Point>();
}

void shift::core::light::Point::DeclareParameters() {
  shift::core::light::Base::DeclareParameters();
}
