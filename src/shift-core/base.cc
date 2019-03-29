#include "shift-core/base.h"

const std::string& Shift::Base::name() const {
  return name_;
}

void Shift::Base::setName(const std::string& name) {
  name_ = name;
}

void Shift::Base::setFullyQualifiedName(
  const std::vector<std::string>& baseName) {
  fullyQualifiedName_ = baseName;
  fullyQualifiedName_.push_back(name_);
}

const std::vector<std::string>& Shift::Base::fullyQualifiedName() const {
  return fullyQualifiedName_;
}

bool Shift::Base::isActive() const {
  return active_;
}

void Shift::Base::setActive(bool active) {
  active_ = active;
}
