#ifndef SHIFT_BEHAVIOR_IK7DOF_H_
#define SHIFT_BEHAVIOR_IK7DOF_H_

#include <memory>

#include "shift/core/behavior/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Behavior {

class IK7DOF : public Base {
public:
  IK7DOF()
    : Base() {
    addParameters();
  }
  IK7DOF(std::string name)
    : Base(name) {
    addParameters();
  }
  ~IK7DOF() {}
  static std::unique_ptr<IK7DOF> create();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
  void addParameters() override;
};

} // namespace Behavior

} // namespace Shift

#endif
