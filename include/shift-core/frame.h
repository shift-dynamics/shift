#ifndef SHIFT_FRAME_H_
#define SHIFT_FRAME_H_

#include <iostream>
#include <string>

#include "ksl/SE3.h"
#include "ksl/mat4x4.h"
#include "ksl/quaternion.h"

#include "shift-core/base.h"
#include "shift-core/parameter.h"

namespace Shift {

class Context;

namespace Body {
class Base;
}

/*!
@brief Shift Frame class
*/
class Frame
  : public Base
  , public Parameter::List {

public:
  Frame(std::string name)
    : Base(name)
    , Parameter::List() {
    addParameters();
  }
  Frame()
    : Base("")
    , Parameter::List() {
    addParameters();
  }
  virtual ~Frame() override {}

  const ksl_SE3_t* dWRTReference() const;
  const ksl_SE3_t* dWRTOrigin() const;
  const ksl_SE3_t* systemDWRTReference() const;

  static const std::string arrayKey() {
    return "frames";
  }
  static const std::string defaultType() {
    return "";
  }

  int index() const {
    return index_;
  }
  void setIndex(int index) {
    index_ = index;
  }
  void setInTree(bool inTree) {
    in_tree_ = inTree;
  }
  bool inTree() const {
    return in_tree_;
  }
  bool isReference() const {
    return is_reference_;
  }
  void setIsReference(bool isReference) {
    is_reference_ = isReference;
  }
  bool isOrigin() {
    return is_origin_;
  }
  void setIsOrigin(bool isOrigin) {
    is_origin_ = isOrigin;
  }
  void setDWRTReference(ksl_SE3_t dReference) {
    d_wrt_reference_ = dReference;
    system_d_wrt_reference_ = dReference;
  }

  void setSystemDWRTReference(ksl_SE3_t* dReference) {
    if(dReference) {
      system_d_wrt_reference_ = *dReference;
      use_system_reference_frame_ = true;
    } else {
      ksl_SE3_copy(&d_wrt_reference_, &system_d_wrt_reference_);
      use_system_reference_frame_ = false;
    }
  }

  void setReferenceFrame(Frame* reference_frame) {
    reference_frame_ = reference_frame;
  }

  const Body::Base* parentBody() const {
    return parent_body_ptr_;
  }

  const Shift::Frame* referenceFrame() const {
    return reference_frame_;
  }

private:
  virtual void addParameters() override;

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

} // namespace Shift

#endif
