#ifndef SHIFT_JOINT_GEAR_H_
#define SHIFT_JOINT_GEAR_H_

#include "shift/core/joint/base.h"

namespace shift {
namespace core {
namespace joint {

void gear_d(const shift::core::Context&, topology::DHVA&,
            const topology::Offsets&, const double*);

void gearinv_d(const shift::core::Context&, topology::DHVA&,
               const topology::Offsets&, const double*);

void gear_dh(const shift::core::Context&, topology::DHVA&,
             const topology::Offsets&, const double*);

void gearinv_dh(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*);

void gear_dhva(const shift::core::Context&, topology::DHVA&,
               const topology::Offsets&, const double*, const double*);

void gearinv_dhva(const shift::core::Context&, topology::DHVA&,
                  const topology::Offsets&, const double*, const double*);

/*!
@brief Gear joint: a gear rolling on a gear of the parent.

The parent frame is at the center of the parent gear, with its z axis along
the gear axis. The child frame is at the center of the child gear, a
separation distance away, in the x-y plane of the parent frame. The one
coordinate \f$q\f$ is the angle of the line of centers, measured from the x axis
of the parent frame; as \f$q\f$ increases the child gear rolls around the parent
gear without slipping, turning relative to the parent frame by
\f$(1 + n) q\f$ about its axis, the ratio \f$n\f$ being the radius of the parent
gear over that of the child gear.
*/
class Gear : public Base {
public:
  Gear()
    : Base() {
    Initialize();
  }
  Gear(std::string name)
    : Base(name) {
    Initialize();
  }
  ~Gear() override {
    delete[] q_0_;
    delete[] dq_0_;
  }
  static std::unique_ptr<Gear> Create();
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
  data is {separation, ratio}, inverse is true when the joint is traversed
  inverted */
  static void Displace(const ksl_SE3_t* d_parent, ksl_SE3_t* d_child,
                       const double* data, bool inverse, double q);
  /*! the screw h of the joint in the reference frame, and its derivative with
  respect to q at fixed parent pose, from the pose of the parent frame */
  static void JointScrews(const ksl_SE3_t* d_parent, const double* data,
                          bool inverse, double q, ksl_screw_t* h,
                          ksl_screw_t* dh_dq);

private:
  void Initialize();
  void DeclareParameters() override;
  static bool registered_;
  double separation_ = 0.0;
  double ratio_ = 1.0;
  double data_[2] = {0.0, 1.0}; //!< separation and ratio
};

} // namespace joint
}
} // namespace shift

#endif
