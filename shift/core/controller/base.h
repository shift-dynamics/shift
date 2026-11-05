#ifndef SHIFT_CONTROLLER_BASE_H_
#define SHIFT_CONTROLLER_BASE_H_

#include <string>
#include <vector>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace controller {

/*!
@brief Abstract base class for controllers

A controller reads signals from sensors and drives actuators, e.g. a PID
controller that applies a joint force to track a measured speed. Controllers
are evaluated each time the equations of motion are set up, after the sensors
are updated and before the force modules.

A controller may have internal states (e.g. the integral of a PID
controller), which are integrated along with the generalized coordinates:
\f$\underline{y} = [\underline{q}, \underline{\dot{q}}, \text{auxiliary states}]\f$, where the auxiliary states of all force
modules and controllers follow the generalized velocities, see
Context::InitializeAuxiliaryStates.
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {
public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }

  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }

  virtual ~Base() override {}

  static const std::string get_array_key() {
    return "controllers";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  unsigned int get_equation_count() const {
    return equation_count_;
  }

  const std::vector<double>& get_q0() const {
    return q_0_;
  }

  /*! \brief whether the controller is evaluated before the other controllers
   * (and the force modules), as a source of values that they read in the same
   * evaluation: a schedule. Those that do are evaluated in the order of the
   * model file, then the others in theirs. */
  virtual bool runs_first() const {
    return false;
  }

  /*! \brief index of the first internal state in the state vector y */
  void set_state_offset(size_t offset) {
    state_offset_ = offset;
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();
  virtual void DeclareParameters() override;

  /*! \brief evaluate the controller: read sensors, apply actuator efforts and
   * set the time derivatives of the internal states in dy
   * \param y [in] state vector [q, dq, auxiliary states]
   * \param dy [out] state time derivatives */
  virtual void Service(const double* y, double* dy) {
    (void)y;
    (void)dy;
  }

protected:
  std::string type_; //!< controller type
  unsigned int equation_count_ = 0; //!< number of internal states
  std::vector<double> q_0_;         //!< initial values of internal states
  size_t state_offset_ = 0;         //!< index of first internal state in y
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
