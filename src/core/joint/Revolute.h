#ifndef SHIFT_JOINT_REVOLUTE_H_
#define SHIFT_JOINT_REVOLUTE_H_

#include "shift/core/joint/Base.h"

namespace shift {
namespace core {
namespace joint {

void rx_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void rxinv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void ry_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void ryinv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void rz_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void rzinv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void ra_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
          const double*);

void rainv_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void rx_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void rxinv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void ry_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void ryinv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void rz_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void rzinv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void ra_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
           const double*);

void rainv_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void rx_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void rxinv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void ry_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void ryinv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void rz_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void rzinv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void ra_dhva(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*, const double*);

void rainv_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

class Revolute : public Base {
public:
  Revolute()
    : Base() {
    dof_ = 1;
    q_count_ = 1;
    dq_count_ = 1;
    dc_count_ = 1;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    rotation_indices_.push_back(0);
    is_rotation_ = true;
    DeclareParameters();
  }
  Revolute(std::string name)
    : Base(name) {
    dof_ = 1;
    q_count_ = 1;
    dq_count_ = 1;
    dc_count_ = 1;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    rotation_indices_.push_back(0);
    is_rotation_ = true;
    DeclareParameters();
  }
  ~Revolute() override {
    delete[] q_0_;
    delete[] dq_0_;
  }
  static std::unique_ptr<Revolute> Create();
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
  enum AxisType { X_AXIS, Y_AXIS, Z_AXIS, ARBITRARY_AXIS };
  static bool registered_;
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  AxisType get_axis_type();
};

} // namespace joint
}
} // namespace shift

#endif
