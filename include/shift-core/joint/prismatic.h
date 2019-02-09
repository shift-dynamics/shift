#ifndef SHIFT_JOINT_PRISMATIC_H_
#define SHIFT_JOINT_PRISMATIC_H_

#include "shift-core/joint/base.h"

namespace Shift {

class Context;

namespace Joint {

void tx_dh(const Shift::Context&, DHVA&, const int*, const double*);

void txinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void ty_dh(const Shift::Context&, DHVA&, const int*, const double*);

void tyinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void tz_dh(const Shift::Context&, DHVA&, const int*, const double*);

void tzinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void tx_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void txinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

void ty_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void tyinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

void tz_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void tzinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

void ta_dh(const Shift::Context&, DHVA&, const int*, const double*);

void tainv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void ta_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void tainv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

class Prismatic : public Base {
public:
  Prismatic()
    : Base() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  Prismatic(std::string name)
    : Base(name) {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  ~Prismatic() {
    delete q_0_;
    delete dq_0_;
  }

  static std::unique_ptr<Prismatic> create();
  void addKeys();
  void updateD();
  void updateDH();
  void updateDHVA();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  double initialDisplacement_ = 0.0;
  double initialVelocity_ = 0.0;
};

} // namespace Joint

} // namespace Shift

#endif
