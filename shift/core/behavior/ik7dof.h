#ifndef SHIFT_BEHAVIOR_IK7DOF_H_
#define SHIFT_BEHAVIOR_IK7DOF_H_

#include <memory>

#include "shift/core/behavior/base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace behavior {

class IK7DOF : public Base {
public:
  IK7DOF()
    : Base() {
    DeclareParameters();
  }
  IK7DOF(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  ~IK7DOF() {}

  static std::unique_ptr<IK7DOF> Create();
  static bool is_registered() {
    return registered_;
  }

private:
  static bool registered_;
  void DeclareParameters() override;
};

} // namespace behavior
} 
} // namespace shift

#endif
