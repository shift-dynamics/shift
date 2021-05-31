#ifndef SHIFT_FRAME_H_
#define SHIFT_FRAME_H_

#include <iostream>
#include <string>

#include "ksl/SE3.h"
#include "ksl/mat4x4.h"
#include "ksl/quaternion.h"

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

class Context;

namespace body {
class Base;
}

/*!
@brief Shift Frame class
*/
class Frame
  : public Base
  , public parameter::List {

public:
  Frame(std::string name)
    : Base(name)
    , parameter::List() {
    DeclareParameters();
  }
  Frame()
    : Base("")
    , parameter::List() {
    DeclareParameters();
  }
  virtual ~Frame() override {}

  static const std::string get_array_key() {
    return "frames";
  }

  static const std::string get_default_type() {
    return "";
  }

  bool is_in_tree() const {
    return in_tree_;
  }

  bool is_reference() const {
    return is_reference_;
  }

  bool is_origin() {
    return is_origin_;
  }

  const ksl_SE3_t* get_d_wrt_reference() const;
  const ksl_SE3_t* get_d_wrt_origin() const;
  const ksl_SE3_t* get_system_d_wrt_reference() const;

  int get_index() const {
    return index_;
  }

  const shift::core::body::Base* get_parent_body() const {
    return parent_body_ptr_;
  }

  const shift::core::Frame* get_reference_frame() const {
    return reference_frame_;
  }

  void set_index(int index) {
    index_ = index;
  }

  void set_in_tree(bool inTree) {
    in_tree_ = inTree;
  }

  void set_is_reference(bool isReference) {
    is_reference_ = isReference;
  }

  void set_is_origin(bool isOrigin) {
    is_origin_ = isOrigin;
  }

  void set_d_wrt_reference(ksl_SE3_t d_wrt_reference) {
    d_wrt_reference_ = d_wrt_reference;
    system_d_wrt_reference_ = d_wrt_reference;
  }

  void set_system_d_wrt_reference(ksl_SE3_t* d_wrt_reference) {
    if(d_wrt_reference) {
      system_d_wrt_reference_ = *d_wrt_reference;
      use_system_reference_frame_ = true;
    } else {
      ksl_SE3_copy(&d_wrt_reference_, &system_d_wrt_reference_);
      use_system_reference_frame_ = false;
    }
  }

  void set_reference_frame(Frame* reference_frame) {
    reference_frame_ = reference_frame;
  }

private:
  virtual void DeclareParameters() override;

  /*! \brief relative transformation from body origin to current frame
   */
  ksl_SE3_t d_wrt_origin_ = {
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};

  /*! \brief relative transform from body reference frame to current frame
   */
  ksl_SE3_t d_wrt_reference_ = {
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};

  /*! \brief relative transform from reference frame, which may be attached to
   * another body, to current frame
   */
  ksl_SE3_t system_d_wrt_reference_ = {
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};
  int index_ = -1;       //!< index in d_wrt_reference_ array
  bool in_tree_ = false; //!< indicates whether frame is in spanning tree
  bool is_reference_ =
    false; //<! indicates whether frame is the reference frame
  // of the body it is attached to
  bool is_origin_ = false; //<! indicates whether frame is origin of rigid body
  // it is attached to
  Frame* reference_frame_ = nullptr;
  bool use_system_reference_frame_ = false;
};

}
} // namespace shift

#endif
