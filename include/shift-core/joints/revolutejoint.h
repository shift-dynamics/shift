#ifndef _SHIFT_REVOLUTEJOINT_H_
#define _SHIFT_REVOLUTEJOINT_H_

#include "joint.h"

namespace Shift {

class RevoluteJoint : public Joint {
public:
  RevoluteJoint()
    : Joint() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  RevoluteJoint(std::string name)
    : Joint(name) {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  ~RevoluteJoint() {
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
