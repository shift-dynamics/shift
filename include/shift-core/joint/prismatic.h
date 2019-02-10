#ifndef SHIFT_JOINT_PRISMATIC_H_
#define SHIFT_JOINT_PRISMATIC_H_

#include "shift-core/joint/base.h"

namespace Shift {

class Context;

namespace Joint {

void tx_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void txinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void ty_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void tyinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void tz_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void tzinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void tx_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void txinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void ty_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void tyinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void tz_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void tzinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void ta_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void tainv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void ta_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
             const double*);

void tainv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

class Prismatic : public Base {
public:
  Prismatic()
    : Base() {
    dof_ = 1;
    qCount_ = 1;
    dqCount_ = 1;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  Prismatic(std::string name)
    : Base(name) {
    dof_ = 1;
    qCount_ = 1;
    dqCount_ = 1;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  ~Prismatic() {
    delete q_0_;
    delete dq_0_;
  }

  static std::unique_ptr<Prismatic> create();
  static bool isRegistered() {
    return registered_;
  }

  virtual const double* getData();
  virtual unsigned int getDataCount();

  void addKeys();
  dh_fptr getDH();
  dhva_fptr getDHVA();

private:
  static bool registered_;
  enum AxisType { X_AXIS, Y_AXIS, Z_AXIS, ARBITRARY_AXIS };
  ksl_vec3_t axis_ = ksl_vec3(0.0, 0.0, 1.0);
  AxisType getAxisType();
};

} // namespace Joint

} // namespace Shift

#endif
