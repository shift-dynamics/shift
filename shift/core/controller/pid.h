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
@brief PID controller acting on a joint

Reads a scalar signal from a sensor and adds the control effort

\f[
u = k_p \, e + k_i \int e \, \mathrm{d}t + k_d \, \dot{e},
\f]

with the error \f$e = \mathrm{target} - \mathrm{signal}\f$ and the gains
\f$k_p\f$, \f$k_i\f$ and \f$k_d\f$ (kp, ki and kd), to the working force of a
joint (a force for prismatic, a torque for revolute joints). The integral of
the error is an internal state. The derivative term uses the analytical rate
of the sensor signal (\f$\dot{e} = -\dot{\mathrm{signal}}\f$), so it requires a
sensor that provides one, e.g. a position sensor.

JSON:
\code
{"type": "pid", "name": "speed", "kp": -60, "ki": -240, "kd": 0,
 "target": 25, "sensor": "chassis_speed", "joint": "AI"}
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

private:
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

  sensor::Base* sensor_ = nullptr;
  joint::Base* joint_ = nullptr;
  double error_ = 0.0;
  double effort_ = 0.0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
