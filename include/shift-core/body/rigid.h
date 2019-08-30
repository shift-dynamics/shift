#ifndef SHIFT_BODY_RIGID_H_
#define SHIFT_BODY_RIGID_H_

#include "shift-core/body/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Body {

class Rigid : public Base {
public:
  Rigid()
    : Base() {
    addParameters();
  }
  Rigid(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Rigid() override {}
  static std::unique_ptr<Rigid> create();
  void addParameters() override;
  static bool isRegistered() {
    return registered_;
  }
  ksl_inertia_t inertiaWRTReference() override;

private:
  int referenceFrameIndex_ = -1; /*!< pointer to Body's unique reference
                                    frame, which  may not be the Frame's origin.
                                    Assigned automatically by
                                    s2_ProcessTopology */

  int originIndex_ = -1; /*!< Index of body's unique *origin*
                             coordinate system, which may not be the
                             reference frame */

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
