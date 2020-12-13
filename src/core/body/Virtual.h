#ifndef SHIFT_BODY_VIRTUAL_H_
#define SHIFT_BODY_VIRTUAL_H_

#include "shift/core/body/Base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace body {

class Virtual : public Base {
public:
  Virtual()
    : Base() {
    is_virtual_ = true;
  }
  Virtual(std::string name)
    : Base(name) {
    is_virtual_ = true;
    addParameters();
  }
  virtual ~Virtual() override {}
  static std::unique_ptr<Virtual> create();
  static bool isRegistered() {
    return registered_;
  }

  void setIndex(int index) {
    index_ = index;
  }

  int index() {
    return index_;
  }

  void setLevel(int level) {
    level_ = level;
  }

  int getLevel() {
    return level_;
  }

  void referenceFrameIndex(int index) {
    reference_frame_index_ = index;
  }

  int referenceFrameIndex() {
    return reference_frame_index_;
  }

  void setParentJointPointer(shift::core::joint::Base* joint) {
    parent_joint_ptr_ = joint;
  }

  shift::core::joint::Base* parentJointPointer() {
    return parent_joint_ptr_;
  }

  void setParentBodyPointer(Base* body) {
    parent_body_ptr_ = body;
  }

  Base* parentBodyPointer() {
    return parent_body_ptr_;
  }

  void setParentFramePointer(Frame* frame) {
    parent_frame_ptr_ = frame;
  }

  Frame* parentFramePointer() {
    return parent_frame_ptr_;
  }

private:
  int reference_frame_index_ = -1; /*!< pointer to Body's unique reference
                                    frame */

  shift::core::joint::Base* parent_joint_ptr_ =
    nullptr; /*!< Pointer to Body's unique
              *parent* Joint datastructure */

  Base* parent_body_ptr_ = nullptr; /*!< A pointer to the unique *parent*
                                       Body*/

  shift::core::Frame* parent_frame_ptr_ =
    nullptr; /*!< parent frame of body reference frame */

  int index_ = -1; /*!< index in contiguous *run-time* array of Bodies */
  int level_ = -1; /*!< level in the spanning tree */

  static bool registered_;
};

} // namespace body
}
} // namespace shift

#endif
