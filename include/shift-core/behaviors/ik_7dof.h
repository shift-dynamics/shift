#ifndef _SHIFT_IK7DOFBEHAVIOR_H_
#define _SHIFT_IK7DOFBEHAVIOR_H_

#include <memory>

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
  static std::unique_ptr<IK7DOF> create();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
  void addKeys();
};

} // namespace Behavior

} // namespace Shift

#endif
