#ifndef SHIFT_CONTROLLER_IK7DOF_H_
#define SHIFT_CONTROLLER_IK7DOF_H_

#include <memory>

#include "shift/core/controller/base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace controller {

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

} // namespace controller
} 
} // namespace shift

#endif
