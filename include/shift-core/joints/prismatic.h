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

  static Prismatic* create();
  void addKeys();
  void updateDH();
  void updateDHVA();

private:
};

} // namespace Joint

} // namespace Shift

#endif
