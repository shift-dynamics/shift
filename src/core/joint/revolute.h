#ifndef SHIFT_JOINT_REVOLUTE_H_
#define SHIFT_JOINT_REVOLUTE_H_

#include "shift/core/joint/base.h"

namespace Shift {

namespace Joint {

void rx_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void rxinv_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void ry_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void ryinv_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void rz_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void rzinv_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void ra_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
          const double*);

void rainv_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void rx_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void rxinv_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void ry_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void ryinv_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void rz_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void rzinv_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void ra_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
           const double*);

void rainv_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void rx_dhva(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void rxinv_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void ry_dhva(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void ryinv_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void rz_dhva(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void rzinv_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void ra_dhva(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*, const double*);

void rainv_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

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
    addParameters();
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
    addParameters();
  }
  ~Revolute() override {
    delete[] q_0_;
    delete[] dq_0_;
  }
  static std::unique_ptr<Revolute> create();
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
  enum AxisType { X_AXIS, Y_AXIS, Z_AXIS, ARBITRARY_AXIS };
  static bool registered_;
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  AxisType axisType();
};

} // namespace Joint

} // namespace Shift

#endif
