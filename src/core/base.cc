#include "shift/core/base.h"

const std::string& shift::Base::name() const {
  return name_;
}

void shift::Base::setName(const std::string name) {
  name_ = name;
}
