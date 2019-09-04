#ifndef SHIFT_JOINT_FLOAT_H_
#define SHIFT_JOINT_FLOAT_H_

#include "shift-core/frame.h"
#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

void float_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void floatinv_d(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*);

void float_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void floatinv_dh(const Shift::Context&, Topology::DHVA&,
                 const Topology::Offsets&, const double*);

void float_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void floatinv_dhva(const Shift::Context&, Topology::DHVA&,
                   const Topology::Offsets&, const double*, const double*);

class Float : public Base {
public:
  Float()
    : Base() {
    dof_ = 6;
    q_count_ = 7;
    dq_count_ = 6;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
  }
  Float(std::string name)
    : Base(name) {
    dof_ = 6;
    q_count_ = 7;
    dq_count_ = 6;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
  }
  ~Float() {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Float> create();
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
