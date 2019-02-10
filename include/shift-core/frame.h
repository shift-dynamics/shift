#ifndef SHIFT_FRAME_H_
#define SHIFT_FRAME_H_

#include <string>

#include "ksl/SE3.h"
#include "ksl/mat4x4.h"
#include "ksl/quaternion.h"

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

/*!
@brief Shift Frame class
*/
class Frame
  : public Base
  , public Serialize {

public:
  Frame(std::string name)
    : Base(name) {
    addKeys();
  }
  Frame()
    : Base("") {
    addKeys();
  }
  ~Frame() {}
  void print();
  void setNameRequired(bool);
  const ksl_SE3_t* getTransform();
  int getIndex() {
    return index_;
  }
  void setIndex(int index) {
    index_ = index;
  }
  void setInTree(bool inTree) {
    inTree_ = inTree;
  }
  bool getInTree() {
    return inTree_;
  }
  bool getIsReference() {
    return isReference_;
  }
  void setIsReference(bool isReference) {
    isReference_ = isReference;
  }

private:
  void addKeys();
  std::vector<std::string> fullyQualifiedName_;
  ksl_SE3_t transform_ =
    ksl_SE3(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  int index_ = -1;           //!< index in context->dWRTReference
  bool inTree_ = false;      //!< indicates whether frame is in spanning tree
  bool isReference_ = false; //<! indicates whether frame is the reference frame
                             // for the body it is attached to
};

} // namespace Shift

#endif
