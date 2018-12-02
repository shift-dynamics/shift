#ifndef _SHIFT_REVOLUTEJOINT_H_
#define _SHIFT_REVOLUTEJOINT_H_

#include "joint.h"

namespace Shift {

namespace Joint {

class Revolute : public JointBase {
public:
  Revolute()
    : JointBase() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  Revolute(std::string name)
    : JointBase(name) {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  ~Revolute() {
    delete q_0_;
    delete dq_0_;
  }
  static Revolute* create();
  void addKeys();
  void updateDH();
  void updateDHVA();

private:
};

} // namespace Joint

} // namespace Shift

#endif
