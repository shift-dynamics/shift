#include "shift-core/base.h"

const std::string& Shift::Base::name() const {
  return name_;
}

void Shift::Base::setName(const std::string& name) {
  name_ = name;
}

void Shift::Base::setFullyQualifiedName(
  const std::vector<std::string>& baseName) {
  fully_qualified_name_ = baseName;
  fully_qualified_name_.push_back(name_);
}

const std::vector<std::string>& Shift::Base::fullyQualifiedName() const {
  return fully_qualified_name_;
}

bool Shift::Base::isActive() const {
  return active_;
}

void Shift::Base::setActive(bool active) {
  active_ = active;
}
