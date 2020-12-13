#ifndef SHIFT_FORCEMODULE_TIRE_FIALA_H_
#define SHIFT_FORCEMODULE_TIRE_FIALA_H_

#include <memory>

#include "shift/core/forcemodule/tire/Base.h"

namespace shift {
namespace core {
namespace forcemodule {
namespace tire {

class Fiala : public Base {
public:
  Fiala()
    : Base() {
    addParameters();
  }
  Fiala(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Fiala() override {}
  static std::unique_ptr<Fiala> create();
  void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;

  /*! input parameters */
  double vertical_stiffness_ = 310000;
  double vertical_damping_ = 3100;
  double rolling_resistance_ = 0.001;
  double width_ = 0.235;
  double cslip_ = 1000000;
  double calpha_ = 45836.6236;
  double u_max_ = 1.0;
  double u_min_ = 0.9;
  double x_relaxation_length_ = 0.05;
  double y_relaxation_length_ = 0.15;

  /*! internal parameters */
  ksl_coscrew_t force_w_;        /*!< reaction force in w frame */
  ksl_screw_t velocity_local_w_; /*!< velocity screw expressed in frame w*/

  ksl_vec3_t wheel_normal_;   /*!< wheel normal expressed in global frame */
  ksl_vec3_t terrain_normal_; /*!< terrain normal at wheel location */
  ksl_vec3_t x_direction_;    /*!< longitudinal heading in terrain plane */
  ksl_vec3_t y_direction_;    /*!< lateral direction in terrain plane */

  double omega_;
  double slip_angle_;
  double longitudinal_slip_;
  double camber_angle_;
  double depth_;      /*!< penetration depth */
  ksl_SE3_t w_frame_; /*! frame of contact patch following ISO
                          conventions, forces are computed in w frame */
  ksl_SE3_t contact_frame_;
  bool in_contact_;

  ksl_coscrew_t f_local_;
  ksl_coscrew_t f_global_;
};

} // namespace Tire
}
} // namespace forcemodule

} // namespace shift

#endif
