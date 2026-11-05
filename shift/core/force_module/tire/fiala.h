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
  double cslip = 1000000;     //!< longitudinal slip stiffness
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

  sigma_x d(kappa)/dt = -(|Vx| kappa + Vsx)
  sigma_y d(alpha)/dt = Vsy - |Vx| tan(alpha)

where Vsx, Vsy are the slip velocities of the contact point and Vx the hub
velocity along the heading. The friction coefficient falls from u_max to
u_min with the combined slip. Fx is linear in kappa up to the critical slip
and saturates beyond it; Fy and the aligning moment Mz follow the Fiala
expressions up to the critical slip angle; My is rolling resistance.
Axes as in tire::Contact (SAE, z down; Fz < 0 pushes the wheel up).
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
