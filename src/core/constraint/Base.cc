#include "shift/core/constraint/Base.h"
#include "shift/core/body/Base.h"
#include "shift/core/Context.h"

bool shift::core::constraint::Modules::is_initialized() {
  return true;
}

void shift::core::constraint::Base::DeclareParameters() {}

void shift::core::constraint::Base::set_objects_active(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, active_ && parent_is_active);
};

void shift::core::constraint::Base::Load() {}

void shift::core::constraint::Base::Connect() {}

void shift::core::constraint::Base::set_parent_body(shift::core::body::Base* parent) {
  parent_body_ptr_ = parent;
  // if(parent) {
  //   parentBody_ = parent->fullyQualifiedName();
  // } else {
  //   parentBody_.clear();
  // }
}

void shift::core::constraint::Base::set_child_body(shift::core::body::Base* child) {
  child_body_ptr_ = child;
  // if(child) {
  //   childBody_ = child->fullyQualifiedName();
  // } else {
  //   childBody_.clear();
  // }
}

void shift::core::constraint::Base::set_parent_frame(Frame* parent) {
  parent_frame_ptr_ = parent;
  // if(parent) {
  //   parentFrame_ = parent->fullyQualifiedName();
  // } else {
  //   parentFrame_.clear();
  // }
}

void shift::core::constraint::Base::set_child_frame(Frame* child) {
  child_frame_ptr_ = child;
  // if(child) {
  //   childFrame_ = child->fullyQualifiedName();
  // } else {
  //   childFrame_.clear();
  // }
}
