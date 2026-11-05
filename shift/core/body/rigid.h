#ifndef SHIFT_BODY_RIGID_H_
#define SHIFT_BODY_RIGID_H_

#include "shift/core/body/base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace body {

class Rigid : public Base {
public:
  Rigid()
    : Base() {
    DeclareParameters();
  }
  Rigid(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Rigid() override {}
  static std::unique_ptr<Rigid> Create();
  void DeclareParameters() override;
  static bool is_registered() {
    return registered_;
  }
  
  ksl_inertia_t get_inertia_wrt_reference() override;
  
  int get_reference_frame_index() {
    return reference_frame_index_;
  }

  int get_origin_index() {
    return origin_index_;
  }

  shift::core::joint::Base* get_parent_joint() {
    return parent_joint_ptr_;
  }

  shift::core::body::Base* get_parent_body() {
    return parent_body_ptr_;
  }

  shift::core::Frame* get_parent_frame() {
    return parent_frame_ptr_;
  }

  int get_index() {
    return index_;
  }

  int get_level() {
    return level_;
  }


private:
  int reference_frame_index_ =
    -1; /*!< pointer to Body's unique reference
         frame, which  may not be the Frame's origin. */

  int origin_index_ = -1; /*!< Index of body's unique *origin*
                             coordinate system, which may not be the
                             reference frame */

  shift::core::joint::Base* parent_joint_ptr_ =
    nullptr; /*!< Pointer to Body's unique
              *parent* joint datastructure */

  shift::core::body::Base* parent_body_ptr_ = nullptr; /*!< A pointer to the unique *parent*
                                   Body */

  shift::core::Frame* parent_frame_ptr_ =
    nullptr; /*!< parent frame of body reference frame */

  int index_ = -1; /*!< index in contiguous *run-time* array of bodies */
  int level_ = -1; /*!< level in the spanning tree */

  static bool registered_;
};

} // namespace body
}
} // namespace shift

#endif
