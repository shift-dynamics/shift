#ifndef _SHIFT_PRISMATICJOINT_H_
#define _SHIFT_PRISMATICJOINT_H_

#include "joint.h"

namespace Shift {

class PrismaticJoint : public Joint {
public:
  PrismaticJoint()
    : Joint() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  PrismaticJoint(std::string name)
    : Joint(name) {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  ~PrismaticJoint() {
    delete q_0_;
    delete dq_0_;
  }

  void addKeys();
  void updateDH();
  void updateDHVA();

private:
};

} // namespace Shift

#endif
