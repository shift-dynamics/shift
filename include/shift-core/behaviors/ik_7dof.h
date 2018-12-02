#ifndef _SHIFT_IK7DOFBEHAVIOR_H_
#define _SHIFT_IK7DOFBEHAVIOR_H_

#include "shift-core/behaviors/behavior.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Behavior {

class IK7DOF : public BehaviorBase {
public:
  IK7DOF()
    : BehaviorBase() {
    addKeys();
  }
  IK7DOF(std::string name)
    : BehaviorBase(name) {
    addKeys();
  }
  ~IK7DOF() {}
  static IK7DOF* create();

private:
  void addKeys();
};

} // namespace Behavior

} // namespace Shift

#endif
