#include "shift/core/base.h"

const std::string& Shift::Base::name() const {
  return name_;
}

void Shift::Base::setName(const std::string name) {
  name_ = name;
}
