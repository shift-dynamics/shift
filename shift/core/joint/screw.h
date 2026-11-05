#ifndef SHIFT_JOINT_SCREW_H_
#define SHIFT_JOINT_SCREW_H_

#include "shift/core/joint/base.h"

namespace shift {
namespace core {
namespace joint {

void screw_d(const shift::core::Context&, topology::DHVA&,
             const topology::Offsets&, const double*);

void screwinv_d(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*);

void screw_dh(const shift::core::Context&, topology::DHVA&,
              const topology::Offsets&, const double*);

void screwinv_dh(const shift::core::Context&, topology::DHVA&,
                 const topology::Offsets&, const double*);

void screw_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void screwinv_dhva(const shift::core::Context&, topology::DHVA&,
                   const topology::Offsets&, const double*, const double*);

/*!
@brief Screw joint: one coordinate, the angle q turned about an axis, with the
child advancing along the axis by pitch * q.
*/
class Screw : public Base {
public:
  Screw()
    : Base() {
    Initialize();
  }
  Screw(std::string name)
    : Base(name) {
    Initialize();
  }
  ~Screw() override {
    delete[] q_0_;
    delete[] dq_0_;
  }
  static std::unique_ptr<Screw> Create();
  static bool is_registered() {
    return registered_;
  }

  const double* get_data() override;
  unsigned int get_data_count() override;

  topology::dh_fptr Dfptr() override;
  topology::dh_fptr DHfptr() override;
  topology::dhva_fptr DHVAfptr() override;
  void Service(const std::vector<double>& y) override;

  /*! pose of the child frame from that of the parent for the angle q;
  data is {axis, pitch}, sign is -1 when the joint is traversed inverted */
  static void Displace(const ksl_SE3_t* d_parent, ksl_SE3_t* d_child,
                       const double* data, double sign, double q);
  /*! screw of the joint in the reference frame, from the pose of the child
  frame */
  static void JointScrew(const ksl_SE3_t* d_child, const double* data,
                         double sign, ksl_screw_t* h);

private:
  void Initialize();
  void DeclareParameters() override;
  static bool registered_;
  ksl_vec3_t axis_ = {{0.0, 0.0, 1.0}};
  double pitch_ = 0.0;
  double data_[4] = {0.0, 0.0, 1.0, 0.0}; //!< axis (unit) and pitch
};

} // namespace joint
}
} // namespace shift

#endif
