#ifndef _SHIFT_PRISMATICJOINT_H_
#define _SHIFT_PRISMATICJOINT_H_

#include "joint.h"

namespace Shift {

namespace Joint {

class Prismatic : public JointBase {
public:
  Prismatic()
    : JointBase() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  Prismatic(std::string name)
    : JointBase(name) {
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
