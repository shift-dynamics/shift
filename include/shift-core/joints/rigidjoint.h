#ifndef _SHIFT_RIGIDJOINT_H_
#define _SHIFT_RIGIDJOINT_H_

#include "joint.h"

namespace Shift {

class RigidJoint : public Joint {
public:
  RigidJoint()
    : Joint() {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }
  RigidJoint(std::string name)
    : Joint(name) {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }
  ~RigidJoint() {}

  void addKeys();
  void updateDH();
  void updateDHVA();

private:
};

} // namespace Shift

#endif
