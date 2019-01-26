#ifndef SHIFT_JOINT_RIGID_H_
#define SHIFT_JOINT_RIGID_H_

#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

class Rigid : public Base {
public:
  Rigid()
    : Base() {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }

  Rigid(std::string name)
    : Base(name) {
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
