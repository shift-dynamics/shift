#include "shift/core/constraint/base.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"

bool Shift::Constraint::Modules::initialized() {
  return true;
}

void Shift::Constraint::Base::addParameters() {}

void Shift::Constraint::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, active_ && parent_is_active);
};

void Shift::Constraint::Base::load() {}

void Shift::Constraint::Base::connect() {}

void Shift::Constraint::Base::setParentBody(Body::Base* parent) {
  parent_body_ptr_ = parent;
  // if(parent) {
  //   parentBody_ = parent->fullyQualifiedName();
  // } else {
  //   parentBody_.clear();
  // }
}

void Shift::Constraint::Base::setChildBody(Body::Base* child) {
  child_body_ptr_ = child;
  // if(child) {
  //   childBody_ = child->fullyQualifiedName();
  // } else {
  //   childBody_.clear();
  // }
}

void Shift::Constraint::Base::setParentFrame(Frame* parent) {
  parent_frame_ptr_ = parent;
  // if(parent) {
  //   parentFrame_ = parent->fullyQualifiedName();
  // } else {
  //   parentFrame_.clear();
  // }
}

void Shift::Constraint::Base::setChildFrame(Frame* child) {
  child_frame_ptr_ = child;
  // if(child) {
  //   childFrame_ = child->fullyQualifiedName();
  // } else {
  //   childFrame_.clear();
  // }
}
