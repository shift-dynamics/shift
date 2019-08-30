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
    : Base(name) {
    addParameters();
  }
  Frame()
    : Base("") {
    addParameters();
  }
  ~Frame() {}
  void print();
  // void setNameRequired(bool) {
  //   n
  // }
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
    inTree_ = inTree;
  }
  bool inTree() const {
    return inTree_;
  }
  bool isReference() const {
    return isReference_;
  }
  void setIsReference(bool isReference) {
    isReference_ = isReference;
  }
  bool isOrigin() {
    return isOrigin_;
  }
  void setIsOrigin(bool isOrigin) {
    isOrigin_ = isOrigin;
  }
  void setDWRTReference(ksl_SE3_t dReference) {
    dWRTReference_ = dReference;
    systemDWRTReference_ = dReference;
  }

  void setSystemDWRTReference(ksl_SE3_t* dReference) {
    if(dReference) {
      systemDWRTReference_ = *dReference;
      useSystemReferenceFrame_ = true;
    } else {
      ksl_SE3_copy(&dWRTReference_, &systemDWRTReference_);
      useSystemReferenceFrame_ = false;
    }
  }

  void setReferenceFrame(Frame* referenceFrame) {
    referenceFrame_ = referenceFrame;
  }

  void setParentBody(Body::Base* body) {
    parentBody_ = body;
  }

  const Body::Base* parentBody() const {
    return parentBody_;
  }

  const Shift::Frame* referenceFrame() const {
    return referenceFrame_;
  }

  void setContext(Context* context) {
    context_ = context;
  }

private:
  void addParameters() override;
  std::vector<std::string> fullyQualifiedName_;

  /*! \brief relative transformation from body origin to current frame
   */
  ksl_SE3_t dWRTOrigin_ = {
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};

  /*! \brief relative transform from body reference frame to current frame
   */
  ksl_SE3_t dWRTReference_ = {
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};

  /*! \brief relative transform from reference frame, which may be attached to
   * another body, to current frame
   */
  ksl_SE3_t systemDWRTReference_ = {
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};
  int index_ = -1;           //!< index in dWRTReference_ array
  bool inTree_ = false;      //!< indicates whether frame is in spanning tree
  bool isReference_ = false; //<! indicates whether frame is the reference frame
  // of the body it is attached to
  bool isOrigin_ = false; //<! indicates whether frame is origin of rigid body
  // it is attached to
  Frame* referenceFrame_ = nullptr;
  bool useSystemReferenceFrame_ = false;

  Body::Base* parentBody_ = nullptr;
  Context* context_ = nullptr;
};

} // namespace Shift

#endif
