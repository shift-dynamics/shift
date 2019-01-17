#ifndef _SHIFT_RIGIDJOINT_H_
#define _SHIFT_RIGIDJOINT_H_

#include "shift-core/joints/joint.h"

namespace Shift {

namespace Joint {

class Rigid : public JointBase {
public:
  Rigid()
    : JointBase() {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }

  Rigid(std::string name)
    : JointBase(name) {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }

  ~Rigid() {}

  static std::unique_ptr<Rigid> create();

  void addKeys();
  void updateD();
  void updateDH();
  void updateDHVA();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Joint

} // namespace Shift

#endif
