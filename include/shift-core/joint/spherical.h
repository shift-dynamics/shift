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
    q_count_ = 4;
    dq_count_ = 3;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    addParameters();
  }
  Spherical(std::string name)
    : Base(name) {
    dof_ = 3;
    q_count_ = 4;
    dq_count_ = 3;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    addParameters();
  }
  virtual ~Spherical() override {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Spherical> create();
  static bool isRegistered() {
    return registered_;
  }

  Topology::dh_fptr Dfptr() override;
  Topology::dh_fptr DHfptr() override;
  Topology::dhva_fptr DHVAfptr() override;

private:
  virtual void addParameters() override;
  static bool registered_;
  Shift::Frame frame_;
};

} // namespace Joint

} // namespace Shift

#endif
