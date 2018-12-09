#ifndef _SHIFT_SPHERICALJOINT_H_
#define _SHIFT_SPHERICALJOINT_H_

#include "shift-core/joints/joint.h"

namespace Shift {

namespace Joint {

class Spherical : public JointBase {
public:
  Spherical()
    : JointBase() {
    dof_ = 3;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  Spherical(std::string name)
    : JointBase(name) {
    dof_ = 3;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  ~Spherical() {
    delete q_0_;
    delete dq_0_;
  }

  static Spherical* create();

  void addKeys();
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
