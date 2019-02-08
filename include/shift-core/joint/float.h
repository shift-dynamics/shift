#ifndef SHIFT_JOINT_FLOAT_H_
#define SHIFT_JOINT_FLOAT_H_

#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void float_dh(const Shift::Context&, DHVA&, const int*, const double*);

void floatinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void float_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

void floatinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                   const double*);

class Float : public Base {
public:
  Float()
    : Base() {
    dof_ = 6;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  Float(std::string name)
    : Base(name) {
    dof_ = 6;
    q_0_ = new double[dof_ + 1];
    dq_0_ = new double[dof_];
  }
  ~Float() {
    delete q_0_;
    delete dq_0_;
  }

  void addKeys();
  void updateD();
  void updateDH();
  void updateDHVA();
  static std::unique_ptr<Float> create();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Joint

} // namespace Shift

#endif
