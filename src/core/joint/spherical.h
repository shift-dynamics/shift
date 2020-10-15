#ifndef SHIFT_JOINT_SPHERICAL_H_
#define SHIFT_JOINT_SPHERICAL_H_

#include "shift/core/frame.h"
#include "shift/core/joint/base.h"

namespace shift {
namespace core {
namespace joint {

struct DHVA;

void spherical_d(const shift::core::Context&, topology::DHVA&,
                 const topology::Offsets&, const double*);

void sphericalinv_d(const shift::core::Context&, topology::DHVA&,
                    const topology::Offsets&, const double*);

void spherical_dh(const shift::core::Context&, topology::DHVA&,
                  const topology::Offsets&, const double*);

void sphericalinv_dh(const shift::core::Context&, topology::DHVA&,
                     const topology::Offsets&, const double*);

void spherical_dhva(const shift::core::Context&, topology::DHVA&,
                    const topology::Offsets&, const double*, const double*);

void sphericalinv_dhva(const shift::core::Context&, topology::DHVA&,
                       const topology::Offsets&, const double*, const double*);

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
    rotation_indices_.push_back(0);
    rotation_indices_.push_back(1);
    rotation_indices_.push_back(2);
  }
  Spherical(std::string name)
    : Base(name) {
    dof_ = 3;
    q_count_ = 4;
    dq_count_ = 3;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    addParameters();
    is_rotation_ = true;
    rotation_indices_.push_back(0);
    rotation_indices_.push_back(1);
    rotation_indices_.push_back(2);
  }
  virtual ~Spherical() override {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Spherical> create();
  static bool isRegistered() {
    return registered_;
  }

  topology::dh_fptr Dfptr() override;
  topology::dh_fptr DHfptr() override;
  topology::dhva_fptr DHVAfptr() override;

private:
  virtual void addParameters() override;
  static bool registered_;
  shift::core::Frame frame_;
};

} // namespace joint
}
} // namespace shift

#endif
