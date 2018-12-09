#include "shift-core/base.h"

const std::string& Shift::Base::getName() {
  return name_;
}

void Shift::Base::setName(std::string name) {
  name_ = name;
}
