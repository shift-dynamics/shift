#ifndef SHIFT_FORCEMODULE_BASE_H_
#define SHIFT_FORCEMODULE_BASE_H_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace joint {
class Base;
}

namespace force_module {

/*!
@brief shift::core::Frame abstract base class
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name)
    , parameter::List() {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("")
    , parameter::List() {
    DeclareParameters();
  }
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "force_modules";
  }
  
  static const std::string get_default_type() {
    return "";
  }
  
  unsigned int get_equation_count() {
    return equation_count_;
  }

  const std::vector<double>& get_q0() {
    return q_0_;
  }

  /*! \brief index of the first internal state in the state vector y, see
   * Context::InitializeAuxiliaryStates */
  void set_state_offset(size_t offset) {
    state_offset_ = offset;
  }

  size_t get_state_offset() const {
    return state_offset_;
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  

  virtual void Load();
  virtual void Connect() = 0;

  /*! \brief throw if a name is repeated among the named parts of this module,
   * such as the axles of a powertrain */
  virtual void CheckUniqueNames() {}

  /*! \brief holding: a module that drives a joint (such as a hydraulic
   * cylinder) may ask to start with the internal states that hold the joint
   * still. Before integrating, Context::HoldInitialStates finds the joint
   * forces of all such modules that give their joints zero acceleration,
   * with the modules applying set forces (set_hold_force), and hands each
   * module its force (Hold), which sets its initial states to produce it. */
  virtual bool holds() const {
    return false;
  }
  /*! \brief the joint a holding module drives (one coordinate) */
  virtual const joint::Base* get_held_joint() const {
    return nullptr;
  }
  /*! \brief while active, Service applies the generalized force on the held
   * joint and leaves the states' rates zero, in place of its model */
  virtual void set_hold_force(bool active, double force) {
    (void)active;
    (void)force;
  }
  /*! \brief set the initial states to produce the generalized force on the
   * held joint at the state y, the initial state */
  virtual void Hold(double force, const double* y) {
    (void)force;
    (void)y;
  }

  /*! \brief names of the module's output values (such as pressures or
   * forces), which loggers record as columns named \<module\>_\<output\> */
  virtual std::vector<std::string> get_output_names() const {
    return {};
  }
  /*! \brief compute the output values for the state y at time t, without
   * applying forces or setting rates; Context::WriteStates calls it before
   * the loggers, while the kinematics are those of y */
  virtual void UpdateOutputs(double t, const double* y) {
    (void)t;
    (void)y;
  }
  /*! \brief append the output values of the last UpdateOutputs, in the
   * order of get_output_names */
  virtual void AppendOutputs(std::vector<double>& values) const {
    (void)values;
  }

  /*! \brief add the module's forces to the context and set the time
   * derivatives of its internal states
   * \param y [in] state vector [q, dq, auxiliary states]
   * \param dy [out] state time derivatives */
  virtual void Service(const double* y, double* dy) = 0;

protected:
  std::string type_;
  virtual void DeclareParameters() override;
  unsigned int equation_count_ = 0;
  std::vector<double> q_0_; /*!< vector of state initial conditions */
  size_t state_offset_ = 0;  /*!< index of the first internal state in y */
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace force_module
}
} // namespace shift

#endif
