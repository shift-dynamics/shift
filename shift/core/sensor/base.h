#ifndef SHIFT_SENSOR_BASE_H_
#define SHIFT_SENSOR_BASE_H_

#include <string>
#include <vector>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace sensor {

/*!
@brief Abstract base class for sensors

A sensor reports on the model as it runs, without changing it. It measures a
scalar signal from the state, e.g. a joint coordinate or the velocity of a
frame, for controllers to read by name: sensors are updated each time the
equations of motion are set up, after the kinematics and before the
controllers. A sensor of a signal publishes it as its output value (and
rate, if it has one); others publish theirs, such as the poses of frames
(Marker) or the loads on bodies (BodyLoad). The loggers record the outputs
they are connected to at each output time, as columns named
\<sensor\>__\<output\>.
*/
class Base
  : public shift::core::Base
  , public parameter::List {
public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }

  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }

  virtual ~Base() {}

  static const std::string get_array_key() {
    return "sensors";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  /*! \brief signal from the last Update */
  double get_value() const {
    return value_;
  }

  /*! \brief true if the sensor provides the time derivative of its signal */
  bool has_rate() const {
    return has_rate_;
  }

  /*! \brief time derivative of the signal from the last Update, if has_rate */
  double get_rate() const {
    return rate_;
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();

  /*! \brief measure the signal from the state vector \f$\underline{y} = [\underline{q}, \underline{\dot{q}}, \ldots]\f$ and the
   * kinematics in the context, which are up to date with y */
  virtual void Update(const double* y) = 0;

  /*! \brief compute the sensor's outputs (DeclareOutput) for the state y at
   * time t; Context::WriteStates calls it after Update and before the
   * loggers, which record the outputs they are connected to, while the
   * kinematics are those of y */
  virtual void UpdateOutputs(double t, const double* y) {
    (void)t;
    (void)y;
  }

protected:
  virtual void DeclareParameters() override;
  /*! \brief declare the signal as the output value, and its rate as the
   * output rate if the sensor has one; a sensor of a signal calls it at the
   * end of its Connect, once it knows has_rate */
  void DeclareSignalOutputs();
  std::string type_; //!< sensor type
  double value_ = 0.0;
  double rate_ = 0.0;
  bool has_rate_ = false;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace sensor
} // namespace core
} // namespace shift

#endif
