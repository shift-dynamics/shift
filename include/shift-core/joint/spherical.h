#ifndef SHIFT_JOINT_SPHERICAL_H_
#define SHIFT_JOINT_SPHERICAL_H_

#include "shift-core/frame.h"
#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void spherical_d(const Shift::Context&, DHVA&, const Offsets&, const double*);

void sphericalinv_d(const Shift::Context&, DHVA&, const Offsets&,
                    const double*);

void spherical_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void sphericalinv_dh(const Shift::Context&, DHVA&, const Offsets&,
                     const double*);

void spherical_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                    const double*);

void sphericalinv_dhva(const Shift::Context&, DHVA&, const Offsets&,
                       const double*, const double*);

class Spherical : public Base {
public:
  Spherical()
    : Base() {
    dof_ = 3;
    qCount_ = 4;
    dqCount_ = 3;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  Spherical(std::string name)
    : Base(name) {
    dof_ = 3;
    qCount_ = 4;
    dqCount_ = 3;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  virtual ~Spherical() override {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Spherical> create();
  static bool isRegistered() {
    return registered_;
  }

  void addKeys();
  virtual void readJSON(Json::Value&) override;

  dh_fptr getDfptr() override;
  dh_fptr getDHfptr() override;
  dhva_fptr getDHVAfptr() override;

private:
  static bool registered_;
  Shift::Frame frame_;
};

} // namespace Joint

} // namespace Shift

#endif
