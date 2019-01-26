#include "shift-core/constraint/base.h"
#include "shift-core/body/base.h"
#include "shift-core/context.h"

bool Shift::Constraint::Modules::initialized() {
  return true;
}

void Shift::Constraint::Base::readJSON(Json::Value& value) {
  Serialize::readJSON(value);
}

void Shift::Constraint::Base::load() {}

void Shift::Constraint::Base::connect() {}

void Shift::Constraint::Base::setParentBodyPointer(Body::Base* parent) {
  parentBodyPointer_ = parent;
  parentBody_ = parent->getFullyQualifiedName();
}

void Shift::Constraint::Base::setChildBodyPointer(Body::Base* child) {
  childBodyPointer_ = child;
  childBody_ = child->getFullyQualifiedName();
}

void Shift::Constraint::Base::setParentBodyPointer(Frame* parent) {
  parentFramePointer_ = parent;
  parentFrame_ = parent->getFullyQualifiedName();
}

void Shift::Constraint::Base::setChildBodyPointer(Frame* child) {
  childFramePointer_ = child;
  childFrame_ = child->getFullyQualifiedName();
}
