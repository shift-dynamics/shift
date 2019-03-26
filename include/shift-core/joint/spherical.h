#ifndef SHIFT_JOINT_SPHERICAL_H_
#define SHIFT_JOINT_SPHERICAL_H_

#include "shift-core/frame.h"
#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void spherical_d(const Shift::Context&, Topology::DHVA&,
                 const Topology::Offsets&, const double*);

void sphericalinv_d(const Shift::Context&, Topology::DHVA&,
                    const Topology::Offsets&, const double*);

void spherical_dh(const Shift::Context&, Topology::DHVA&,
                  const Topology::Offsets&, const double*);

void sphericalinv_dh(const Shift::Context&, Topology::DHVA&,
                     const Topology::Offsets&, const double*);

void spherical_dhva(const Shift::Context&, Topology::DHVA&,
                    const Topology::Offsets&, const double*, const double*);

void sphericalinv_dhva(const Shift::Context&, Topology::DHVA&,
                       const Topology::Offsets&, const double*, const double*);

class Spherical : public Base {
public:
  Spherical()
    : Base() {
    dof_ = 3;
    qCount_ = 4;
    dqCount_ = 3;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
    addKeys();
  }
  Spherical(std::string name)
    : Base(name) {
    dof_ = 3;
    qCount_ = 4;
    dqCount_ = 3;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
    addKeys();
  }
  virtual ~Spherical() override {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Spherical> create();
  static bool isRegistered() {
    return registered_;
  }

  virtual void addKeys() override;
  virtual void readJSON(Json::Value&) override;

  Topology::dh_fptr getDfptr() override;
  Topology::dh_fptr getDHfptr() override;
  Topology::dhva_fptr getDHVAfptr() override;

private:
  static bool registered_;
  Shift::Frame frame_;
};

} // namespace Joint

} // namespace Shift

#endif
