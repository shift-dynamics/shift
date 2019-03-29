#ifndef SHIFT_BODY_VIRTUAL_H_
#define SHIFT_BODY_VIRTUAL_H_

#include "shift-core/body/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Body {

class Virtual : public Base {
public:
  Virtual()
    : Base() {}
  Virtual(std::string name)
    : Base(name) {
    addKeys();
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
    referenceFrameIndex_ = index;
  }

  int referenceFrameIndex() {
    return referenceFrameIndex_;
  }

  void setParentJointPointer(Joint::JointBase* joint) {
    parentJointPointer_ = joint;
  }

  Joint::JointBase* parentJointPointer() {
    return parentJointPointer_;
  }

  void setParentBodyPointer(BodyBase* body) {
    parentBodyPointer_ = body;
  }

  BodyBase* parentBodyPointer() {
    return parentBodyPointer_;
  }

  void setParentFramePointer(Frame* frame) {
    parentFramePointer_ = frame;
  }

  Frame* parentFramePointer() {
    return parentFramePointer_;
  }

private:
  int referenceFrameIndex_ = -1; /*!< pointer to Body's unique reference
                                    frame */

  Joint::JointBase* parentJointPointer_ = NULL; /*!< Pointer to Body's unique
                                                 *parent* Joint datastructure */

  BodyBase* parentBodyPointer_ = NULL; /*!< A pointer to the unique *parent*
                                       Body, set by s2_ProcessTopology */

  Frame* parentFramePointer_ =
    NULL; /*!< parent frame of body reference frame */

  int index_ = -1; /*!< index in contiguous *run-time* array of Bodies, set
                  by s2_ProcessTopology */
  int level_ = -1; /*!< level in the spanning tree, set
                  by s2_ProcessTopology */

  static bool registered_;
};

} // namespace Body

} // namespace Shift

#endif
