#include "shift/core/constraint/base.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"

bool shift::core::constraint::Modules::initialized() {
  return true;
}

void shift::core::constraint::Base::addParameters() {}

void shift::core::constraint::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, active_ && parent_is_active);
};

void shift::core::constraint::Base::load() {}

void shift::core::constraint::Base::connect() {}

void shift::core::constraint::Base::setParentBody(shift::core::body::Base* parent) {
  parent_body_ptr_ = parent;
  // if(parent) {
  //   parentBody_ = parent->fullyQualifiedName();
  // } else {
  //   parentBody_.clear();
  // }
}

void shift::core::constraint::Base::setChildBody(shift::core::body::Base* child) {
  child_body_ptr_ = child;
  // if(child) {
  //   childBody_ = child->fullyQualifiedName();
  // } else {
  //   childBody_.clear();
  // }
}

void shift::core::constraint::Base::setParentFrame(Frame* parent) {
  parent_frame_ptr_ = parent;
  // if(parent) {
  //   parentFrame_ = parent->fullyQualifiedName();
  // } else {
  //   parentFrame_.clear();
  // }
}

void shift::core::constraint::Base::setChildFrame(Frame* child) {
  child_frame_ptr_ = child;
  // if(child) {
  //   childFrame_ = child->fullyQualifiedName();
  // } else {
  //   childFrame_.clear();
  // }
}
