#ifndef _SHIFT_FLOATJOINT_H_
#define _SHIFT_FLOATJOINT_H_

#include "joint.h"

namespace Shift {

class FloatJoint : public Joint {
public:
  FloatJoint()
    : Joint() {
    dof_ = 6;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  FloatJoint(std::string name)
    : Joint(name) {
    dof_ = 6;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  ~FloatJoint() {
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
