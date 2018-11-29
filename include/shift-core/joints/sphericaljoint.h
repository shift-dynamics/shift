#ifndef _SHIFT_SPHERICALJOINT_H_
#define _SHIFT_SPHERICALJOINT_H_

#include "joint.h"

namespace Shift {

class SphericalJoint : public Joint {
public:
  SphericalJoint()
    : Joint() {
    dof_ = 3;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  SphericalJoint(std::string name)
    : Joint(name) {
    dof_ = 3;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  ~SphericalJoint() {
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
