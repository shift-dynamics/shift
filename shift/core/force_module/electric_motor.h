#ifndef SHIFT_FORCEMODULE_ELECTRIC_MOTOR_H_
#define SHIFT_FORCEMODULE_ELECTRIC_MOTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/base.h"

namespace shift {
namespace core {
namespace joint {
class Base;
}
namespace force_module {

/*!
@brief an electric motor on a revolute joint: its stator is the joint's
parent, its rotor the child, and its torque acts on the joint

The motor is a DC motor, whose winding of resistance \f$R\f$ and inductance
\f$L\f$ carries the current \f$i\f$ against the back voltage of the rotor
turning at \f$\omega\f$ (the joint's rate):

\f[
v = R i + L \frac{\mathrm{d}i}{\mathrm{d}t} + k \omega, \qquad
\tau = k i,
\f]

with the torque constant \f$k\f$, which in SI units is also the back voltage
per rotor speed (V s/rad = N m/A). The same equations describe a brushless
DC or permanent magnet synchronous motor under field-oriented control that
keeps the current along the magnet's axis at zero: then \f$i\f$ is the
current across the axis, \f$k = \frac{3}{2} p \psi\f$ for \f$p\f$ pole pairs
and the magnet flux \f$\psi\f$, and \f$R\f$ and \f$L\f$ are those of a phase.

The drive, the electronics between the supply and the winding, is set by
drive:

- "current" (the default): the drive controls the current, taken as fast
  enough to follow the input current at once. The current is limited to
  current_limit, and then to what the supply can drive against the back
  voltage, \f$(\pm v_s - k \omega) / R\f$ for the supply_voltage \f$v_s\f$;
  the voltage limit is applied last, so a rotor turned faster than the supply
  voltage allows pushes current back into the supply whatever the limit. The
  inductance is not used.
- "voltage": the drive applies the input voltage, limited to
  \f$\pm v_s\f$; current_limit is not used. With an inductance, the current
  is an internal state, which
  starts at 0; without one (the default), it is
  \f$(v - k \omega) / R\f$. The inductance makes the equations stiff: its time
  constant \f$L / R\f$ is often a millisecond or less, and the integrator
  takes steps short enough to follow it.

The inputs current (A) and voltage (V) are set by a controller, as a PID
controller with "component" and "input", by a schedule, or in the model
file; only the input of the drive is used, and giving or setting the other is
an error. Without a supply_voltage or a current_limit (0, the defaults) there
is no limit.

Friction and drag of the rotor are left to the joint (its
damping_coefficients) or a joint_friction module, the rotor's inertia to its
body, and a gearbox to a gear_mesh module between the rotor's shaft and the
load's.

Outputs, logged as columns and readable by GetOutput: winding_current,
terminal_voltage, back_voltage, torque, speed, electrical_power (\f$v i\f$,
from the supply) and mechanical_power (\f$\tau \omega\f$, to the rotor); the
difference is the loss in the winding, \f$R i^2\f$, and with an inductance
the rate of the energy stored in it.

JSON:
\code
{"type": "electric_motor", "name": "spindle", "joint": "spindle_bearing",
 "torque_constant": 0.12, "resistance": 0.3, "supply_voltage": 48,
 "current_limit": 20}
\endcode
*/
class ElectricMotor : public Base {
public:
  enum class Drive { kCurrent, kVoltage };

  ElectricMotor()
    : Base() {
    type_ = "electric_motor";
    DeclareParameters();
  }

  ElectricMotor(std::string name)
    : Base(name) {
    type_ = "electric_motor";
    DeclareParameters();
  }

  virtual ~ElectricMotor() override {}

  static std::unique_ptr<ElectricMotor> Create();

  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  Drive get_drive() const {
    return drive_;
  }

  /*! \brief as of the last Service or UpdateOutputs: */
  double get_current() const {
    return winding_current_;
  }
  double get_terminal_voltage() const {
    return terminal_voltage_;
  }
  double get_torque() const {
    return torque_;
  }
  double get_speed() const {
    return speed_;
  }

private:
  void DeclareParameters() override;
  /*! \brief the current, voltages, torque and powers for the state y, and
   * the rate of the current in dy, unless it is null */
  void Evaluate(const double* y, double* dy);
  /*! \brief whether the current is an internal state */
  bool has_current_state() const {
    return drive_ == Drive::kVoltage && inductance_ > 0.0;
  }

  static bool registered_;

  std::string joint_name_;
  double torque_constant_ = 0.0;
  double resistance_ = 0.0;
  double inductance_ = 0.0;
  std::string drive_name_ = "current";
  double supply_voltage_ = 0.0;
  double current_limit_ = 0.0;

  /* inputs */
  double current_ = 0.0;
  double voltage_ = 0.0;

  /* outputs */
  double winding_current_ = 0.0;
  double terminal_voltage_ = 0.0;
  double back_voltage_ = 0.0;
  double torque_ = 0.0;
  double speed_ = 0.0;
  double electrical_power_ = 0.0;
  double mechanical_power_ = 0.0;

  Drive drive_ = Drive::kCurrent;
  joint::Base* joint_ = nullptr;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
