#ifndef SHIFT_INTEGRATOR_BASE_H_
#define SHIFT_INTEGRATOR_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter/list.h"

namespace Shift {

class Frame;
class Context;

namespace Solver {
class Base;
}

namespace Integrator {

/*!
@brief Abstract Integrator base class
*/
class Base
  : public Shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name) {
    addParameters();
  }

  Base()
    : Shift::Base("") {
    addParameters();
  }

  virtual ~Base() override {}

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "integrators";
  }

  static const std::string defaultType() {
    return "";
  }

  void setActiveObjects(bool parent_is_active) override;

  virtual void load();
  virtual void connect();
  virtual void initialize() = 0;
  virtual void integrate() = 0;

  double baumgarteOmegaN2() const {
    return baumgarte_omega_n_2_;
  }

  double baumgarte2OmegaN() const {
    return baumgarte_2_omega_n_;
  }

  const std::vector<double>& integralConstraintError() {
    return integral_error_;
  }

protected:
  virtual void addParameters() override;
  std::string type_; //!< descriptive name of Integrator type
  unsigned int iterations_since_last_full_factor_ = 0;
  double baumgarte_omega_n_ = 200.0;
  double baumgarte_2_omega_n_ = 2.0 * baumgarte_omega_n_; /*!< 2 * omega_n term,
                                   used in baumgarte stabilization */
  double baumgarte_omega_n_2_ =
    baumgarte_omega_n_ *
    baumgarte_omega_n_; /*! omega_n^2 term, used in baumgarte stabilization */
  std::vector<double> integral_error_; /*!< integral error, equal to the
                                          number of constraints */
  double time_step_ = 0.01; /*!< time step used for reporting intervals */
  double time_span_ = 10.0; /*!< time span for numerical integration, defaults
                               to 10 seconds if not provided */
  Shift::Solver::Base* solver_ = nullptr;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Integrator

} // namespace Shift

#endif
