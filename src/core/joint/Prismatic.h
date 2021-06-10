#ifndef SHIFT_JOINT_PRISMATIC_H_
#define SHIFT_JOINT_PRISMATIC_H_

#include "shift/core/joint/Base.h"

namespace shift {
namespace core {

class Context;

namespace joint {

void tx_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void txinv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void ty_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void tyinv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void tz_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void tzinv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void ta_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void tainv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void tx_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void txinv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void ty_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void tyinv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void tz_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void tzinv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void ta_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void tainv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void tx_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void txinv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void ty_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void tyinv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);
void tz_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void tzinv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void ta_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void tainv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);
//
// void service_force(const shift::core::Context&, const topology::DHVA&, const
// topology::Offsets&, double* f);

class Prismatic : public Base {
public:
  Prismatic()
    : Base() {
    dof_ = 1;
    q_count_ = 1;
    dq_count_ = 1;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    DeclareParameters();
  }

  Prismatic(std::string name)
    : Base(name) {
    dof_ = 1;
    q_count_ = 1;
    dq_count_ = 1;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    DeclareParameters();
  }

  virtual ~Prismatic() override {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Prismatic> Create();
  static bool is_registered() {
    return registered_;
  }

  virtual const double* get_data() override;
  virtual unsigned int get_data_count() override;

  topology::dh_fptr Dfptr() override;
  topology::dh_fptr DHfptr() override;
  topology::dhva_fptr DHVAfptr() override;

  virtual void Service(const std::vector<double>& y) override;

private:
  virtual void DeclareParameters() override;
  static bool registered_;
  enum AxisType { X_AXIS, Y_AXIS, Z_AXIS, ARBITRARY_AXIS };
  ksl_vec3_t axis_ = ksl_vec3(0.0, 0.0, 1.0);
  AxisType get_axis_type();
};

} // namespace joint
}
} // namespace shift

#endif
