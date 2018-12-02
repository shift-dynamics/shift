#ifndef _SHIFT_RIGIDBODY_
#define _SHIFT_RIGIDBODY_

#include "shift-core/bodies/body.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Body {

class Rigid : public BodyBase {
public:
  Rigid()
    : BodyBase() {
    addKeys();
  }
  Rigid(std::string name)
    : BodyBase(name) {
    addKeys();
  }
  ~Rigid() {}
  static Rigid* create();
  void addKeys();

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

  bool isVirtual_ = false; /*!< indicates whether the Body is *virtual*. All
                       user-defined bodies are not virtual. Massless
                       virtual bodies are only added to simplify processing
                       kinematics if multiple loops are present in the
                       mechanism */
  ksl_inertia_t inertia_;  /*!< centroidal Mass and moment of Inertia
                                            properties expressed in body origin
                               coordinate system */
  ksl_vec3_t centerOfMassWRTOrigin_;    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t centerOfMassWRTReference_; /*!< vector from body reference
                                                frame to center of mass */
};

} // namespace Body

} // namespace Shift

#endif
