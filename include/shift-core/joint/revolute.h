#ifndef SHIFT_JOINT_REVOLUTE_H_
#define SHIFT_JOINT_REVOLUTE_H_

#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void rx_dh(const Shift::Context&, DHVA&, const int*, const double*);

void rxinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void ry_dh(const Shift::Context&, DHVA&, const int*, const double*);

void ryinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void rz_dh(const Shift::Context&, DHVA&, const int*, const double*);

void rzinv_dh(const Shift::Context&, DHVA&, const int*, const double*);

void rx_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void rxinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

void ry_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void ryinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

void rz_dhva(const Shift::Context&, DHVA&, const int*, const double*,
             const double*);

void rzinv_dhva(const Shift::Context&, DHVA&, const int*, const double*,
                const double*);

class Revolute : public Base {
public:
  Revolute()
    : Base() {
    dof_ = 1;
    q_0_ = new double[dof_];
    dq_0_ = new double[dof_];
  }
  Revolute(std::string name)
    : Base(name) {
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
