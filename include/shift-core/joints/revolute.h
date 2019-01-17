#ifndef _SHIFT_REVOLUTEJOINT_H_
#define _SHIFT_REVOLUTEJOINT_H_

#include "shift-core/joints/joint.h"

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
  static std::unique_ptr<Revolute> create();
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
