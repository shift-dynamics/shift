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
    DeclareParameters();
  }
  virtual ~Virtual() override {}
  static std::unique_ptr<Virtual> create();
  static bool is_registered() {
    return registered_;
  }

  Frame* get_parent_frame() {
    return parent_frame_ptr_;
  }

  int get_index() {
    return index_;
  }

  int get_level() {
    return level_;
  }

  int get_reference_frame_index() {
    return reference_frame_index_;
  }

  shift::core::joint::Base* get_parent_joint() {
    return parent_joint_ptr_;
  }

  Base* get_parent_body() {
    return parent_body_ptr_;
  }

  void set_index(int index) {
    index_ = index;
  }

  void set_level(int level) {
    level_ = level;
  }

  void set_reference_frame_index(int index) {
    reference_frame_index_ = index;
  }

  void set_parent_joint(shift::core::joint::Base* joint) {
    parent_joint_ptr_ = joint;
  }

  void set_parent_body(Base* body) {
    parent_body_ptr_ = body;
  }

  void set_parent_frame(Frame* frame) {
    parent_frame_ptr_ = frame;
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
