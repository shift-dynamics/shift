#ifndef SHIFT_FORCEMODULE_TIRE_FIALA_H_
#define SHIFT_FORCEMODULE_TIRE_FIALA_H_

#include <memory>

#include "shift/core/forcemodule/tire/base.h"

namespace shift {
namespace core {
namespace forcemodule {
namespace tire {

class Pac2002 : public Base {
public:
  Pac2002()
    : Base() {
    addParameters();
  }
  Pac2002(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Pac2002() override {}
  static std::unique_ptr<Pac2002> create();
  void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;

  double kappa_;      /*< INPUT desired slip rate */
  double alpha_;      /*< INPUT turn slip angle */
  double alpha_star_; /*< INPUT tan(alpha) */
  double gamma_;      /*< INPUT camber angle */

  double v_cx_;            // tire c-sys  x-vel.
  double v_cy_;            // tire c-sys y-vel
  double v_sx_;            // tire c-sys x-dir slip velocity
  double v_sy_;            // y-dir slip vel
  double psi_dot_;         // turn slip angular velocity
  double cos_prime_alpha_; // ratio,  V.x / V_mag;

  // slip displacements
  double u_; // contact patch slip displacement
  double v_alpha_;
  double v_gamma_;
  double v_phi_;

  // integrated slip velocity terms over time step increment
  double integrated_du_dt_;
  double integrated_dv_alpha_dt_;
  double integrated_dv_gamma_dt_;
  double integrated_dv_phi_dt_;

  //  slips used in all calculations (can be INPUTs or transient slips)
  double alpha_p_;
  double kappa_p_;
  double gamma_p_;
  double phi_t_;
  double phi_p_;
};

} // namespace Tire

} // namespace forcemodule
}
} // namespace shift

#endif
