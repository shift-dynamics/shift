#ifndef SHIFT_JOINT_SPHERICAL_H_
#define SHIFT_JOINT_SPHERICAL_H_

#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

class Spherical : public Base {
public:
  Spherical()
    : Base() {
    dof_ = 3;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  Spherical(std::string name)
    : Base(name) {
    dof_ = 3;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  ~Spherical() {
    delete q_0_;
    delete dq_0_;
  }

  static std::unique_ptr<Spherical> create();

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
