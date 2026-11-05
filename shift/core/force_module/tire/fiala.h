#ifndef SHIFT_FORCEMODULE_TIRE_FIALA_H_
#define SHIFT_FORCEMODULE_TIRE_FIALA_H_

#include "shift/core/force_module/tire/model.h"

namespace shift {
namespace core {
namespace force_module {
namespace tire {

/*! \brief Fiala tire parameters, see Fiala */
struct FialaParameters {
  double unloaded_radius = 0.344;
  double vertical_stiffness = 310000;
  double vertical_damping = 3100;
  double rolling_resistance = 0.001;
  double width = 0.235;
  /*! longitudinal slip stiffness, N per unit slip: about 16 times the
   * 3 kN load of a car tire. With the relaxation length it sets the
   * longitudinal stiffness of the carcass, \f$c_{slip} / x_{relaxation}\f$
   * (1e6 N/m), and so the wheel's spin mode on it,
   * \f$\sqrt{r^2 c_{slip} / (i_{yy} \, x_{relaxation})}\f$, about 360 rad/s for a
   * 21 kg wheel. The 1e6 of the original implementation (2e7 N/m) put
   * that mode at 1600 rad/s, nearly undamped, and limited the integration
   * steps of the vehicle models four times as much. */
  double cslip = 50000;
  double calpha = 45836.6236; //!< cornering stiffness
  double u_max = 1.0;         //!< friction coefficient without sliding
  double u_min = 0.9;         //!< friction coefficient when fully sliding
  double x_relaxation_length = 0.05;
  double y_relaxation_length = 0.15;
};

/*!
@brief Fiala tire model with transient slip

Ported from the C implementation of Shift (2016). The normal force is a
spring and damper on the penetration of the wheel disc. The longitudinal
slip and the slip angle are internal states with first order relaxation:

\f[
\begin{aligned}
\sigma_x \dot{\kappa} &= -(|v_x| \kappa + v_{sx}) \\
\sigma_y \dot{\alpha} &= v_{sy} - |v_x| \tan\alpha
\end{aligned}
\f]

where \f$v_{sx}, v_{sy}\f$ are the slip velocities of the contact point and
\f$v_x\f$ the hub velocity along the heading. The friction coefficient falls
from u_max to u_min with the combined slip. \f$f_x\f$ is linear in
\f$\kappa\f$ up to the critical slip and saturates beyond it; \f$f_y\f$ and the
aligning moment \f$m_z\f$ follow the Fiala expressions up to the critical slip
angle; \f$m_y\f$ is rolling resistance. Axes as in tire::Contact (SAE, z down;
\f$f_z < 0\f$ pushes the wheel up).
*/
class Fiala : public Model {
public:
  explicit Fiala(const FialaParameters& parameters)
    : p_(parameters) {}

  unsigned int get_state_count() const override {
    return 2;
  }

  double get_unloaded_radius() const override {
    return p_.unloaded_radius;
  }

  void Evaluate(const Contact& contact, const double* states,
                double* state_rates, ksl_coscrew_t* force_w) override;

private:
  FialaParameters p_;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
