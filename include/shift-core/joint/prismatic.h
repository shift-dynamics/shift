#ifndef SHIFT_JOINT_PRISMATIC_H_
#define SHIFT_JOINT_PRISMATIC_H_

#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

class Prismatic : public Base {
public:
  Prismatic()
    : Base() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  Prismatic(std::string name)
    : Base(name) {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  ~Prismatic() {
    delete q_0_;
    delete dq_0_;
  }

  static std::unique_ptr<Prismatic> create();
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
