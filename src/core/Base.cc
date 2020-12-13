#include "shift/core/Base.h"

const std::string& shift::core::Base::name() const {
  return name_;
}

void shift::core::Base::setName(const std::string name) {
  name_ = name;
}
