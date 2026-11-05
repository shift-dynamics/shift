#include "shift/core/base.h"

const std::string& shift::core::Base::get_name() const {
  return name_;
}

void shift::core::Base::set_name(const std::string name) {
  name_ = name;
}
