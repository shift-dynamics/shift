#ifndef SHIFT_JOINT_REVOLUTE_H_
#define SHIFT_JOINT_REVOLUTE_H_

#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void rx_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rxinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void ry_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void ryinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rz_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rzinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rx_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void rxinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void ry_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void ryinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void rz_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void rzinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void ra_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rainv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void ra_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void rainv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

class Revolute : public Base {
public:
  Revolute()
    : Base() {
    dof_ = 1;
    qCount_ = 1;
    dqCount_ = 1;
    dcCount_ = 1;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  Revolute(std::string name)
    : Base(name) {
    dof_ = 1;
    qCount_ = 1;
    dqCount_ = 1;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  ~Revolute() {
    delete q_0_;
    delete dq_0_;
  }
  static std::unique_ptr<Revolute> create();
  static bool isRegistered() {
    return registered_;
  }

  virtual const double* getData();
  virtual unsigned int getDataCount();

  void addKeys();
  dh_fptr getDH();
  dhva_fptr getDHVA();

private:
  enum AxisType { X_AXIS, Y_AXIS, Z_AXIS, ARBITRARY_AXIS };
  static bool registered_;
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  AxisType getAxisType();
};

} // namespace Joint

} // namespace Shift

#endif
