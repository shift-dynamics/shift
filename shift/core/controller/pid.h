#ifndef SHIFT_CONTROLLER_PID_H_
#define SHIFT_CONTROLLER_PID_H_

#include <memory>
#include <string>

#include "shift/core/controller/base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}
namespace sensor {
class Base;
}

namespace controller {

/*!
@brief PID controller acting on a joint, or setting an input of another
component

Reads a scalar signal from a sensor and computes the control effort

\f[
u = k_p \, e + k_i \int e \, \mathrm{d}t + k_d \, \dot{e},
\f]

with the error \f$e = \mathrm{target} - \mathrm{signal}\f$ and the gains
\f$k_p\f$, \f$k_i\f$ and \f$k_d\f$ (kp, ki and kd). The effort is either added
to the working force of a joint (a force for prismatic, a torque for revolute
joints), or set as an input of another component of the assembly
(parameter::List::DeclareInput), such as the valve command of a hydraulic
cylinder or the current of a motor: the controller finds the component by
name when it connects, and sets the input with SetInput at each evaluation.
The integral of the error is an internal state. The derivative term uses the
analytical rate of the sensor signal (\f$\dot{e} = -\dot{\mathrm{signal}}\f$),
so it requires a sensor that provides one, e.g. a position sensor.

The controllers are evaluated before the force modules, so a force module
reads the effort of the same evaluation. An input of another controller is
read in the same evaluation only if that controller comes after this one in
the model file.

JSON:
\code
{"type": "pid", "name": "speed", "kp": -60, "ki": -240, "kd": 0,
 "target": 25, "sensor": "chassis_speed", "joint": "AI"}
{"type": "pid", "name": "boom_position", "kp": 20, "target": 0.1,
 "sensor": "boom_extension", "component": "boom_cylinder",
 "input": "command"}
\endcode
*/
class PID : public controller::Base {
public:
  PID()
    : Base() {
    Initialize();
  }

  PID(std::string name)
    : Base(name) {
    Initialize();
  }

  virtual ~PID() override {}

  static std::unique_ptr<PID> Create();

  static bool is_registered() {
    return registered_;
  }

  void set_target(double target) {
    target_ = target;
  }

  double get_error() const {
    return error_;
  }

  double get_effort() const {
    return effort_;
  }

  void DeclareParameters() override;
  void Connect() override;
  void Service(const double* y, double* dy) override;
  /*! \brief the error and the effort at the state y, with the sensor
   * updated to it, and the input of the component set to the effort */
  void UpdateOutputs(double t, const double* y) override;

private:
  /*! \brief the error and the effort for the state y */
  void Compute(const double* y);
  void Initialize() {
    equation_count_ = 1;
    q_0_ = {0.0};
    DeclareParameters();
  }

  static bool registered_;

  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double target_ = 0.0;
  std::string sensor_name_; //!< name of the input sensor in this assembly
  std::string joint_name_;  //!< name of the output joint in this assembly
  std::string component_name_; //!< component whose input is set, if any
  std::string input_name_;     //!< the input of that component

  sensor::Base* sensor_ = nullptr;
  joint::Base* joint_ = nullptr;
  parameter::List* component_ = nullptr;
  double error_ = 0.0;
  double effort_ = 0.0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
