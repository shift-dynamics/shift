#ifndef SHIFT_JOINT_PRISMATIC_H_
#define SHIFT_JOINT_PRISMATIC_H_

#include "shift/core/joint/base.h"

namespace shift {

class Context;

namespace Joint {

void tx_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void txinv_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void ty_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void tyinv_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void tz_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void tzinv_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void ta_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void tainv_d(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void tx_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void txinv_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void ty_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void tyinv_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void tz_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void tzinv_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void ta_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void tainv_dh(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void tx_dhva(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void txinv_dhva(const shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void ty_dhva(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void tyinv_dhva(const shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);
void tz_dhva(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void tzinv_dhva(const shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void ta_dhva(const shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void tainv_dhva(const shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);
//
// void service_force(const shift::Context&, const Topology::DHVA&, const
// Topology::Offsets&, double* f);

class Prismatic : public Base {
public:
  Prismatic()
    : Base() {
    dof_ = 1;
    q_count_ = 1;
    dq_count_ = 1;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    addParameters();
  }

  Prismatic(std::string name)
    : Base(name) {
    dof_ = 1;
    q_count_ = 1;
    dq_count_ = 1;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    addParameters();
  }

  virtual ~Prismatic() override {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Prismatic> create();
  static bool isRegistered() {
    return registered_;
  }

  virtual const double* data() override;
  virtual unsigned int dataCount() override;

  Topology::dh_fptr Dfptr() override;
  Topology::dh_fptr DHfptr() override;
  Topology::dhva_fptr DHVAfptr() override;

  virtual void service(const std::vector<double>& y) override;

private:
  virtual void addParameters() override;
  static bool registered_;
  enum AxisType { X_AXIS, Y_AXIS, Z_AXIS, ARBITRARY_AXIS };
  ksl_vec3_t axis_ = ksl_vec3(0.0, 0.0, 1.0);
  AxisType axisType();
};

} // namespace Joint

} // namespace shift

#endif
