#ifndef _SHIFT_FLOATJOINT_H_
#define _SHIFT_FLOATJOINT_H_

#include "joint.h"

namespace Shift {

namespace Joint {

class Float : public JointBase {
public:
  Float()
    : JointBase() {
    dof_ = 6;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  Float(std::string name)
    : JointBase(name) {
    dof_ = 6;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  ~Float() {
    delete q_0_;
    delete dq_0_;
  }

  void addKeys();
  void updateDH();
  void updateDHVA();

private:
};

} // namespace Joint

} // namespace Shift

#endif
