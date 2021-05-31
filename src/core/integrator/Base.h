#ifndef SHIFT_INTEGRATOR_BASE_H_
#define SHIFT_INTEGRATOR_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

class Frame;
class Context;

namespace solver {
class Base;
}

namespace integrator {

/*!
@brief Abstract Integrator base class
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

  virtual ~Base() override {}

  static const std::string get_array_key() {
    return "integrators";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  double get_baumgarte_omega_n_2() const {
    return baumgarte_omega_n_ * baumgarte_omega_n_;
  }

  double get_baumgarte_2_omega_n() const {
    return 2.0 * baumgarte_omega_n_;
  }

  const std::vector<double>& get_integral_error() {
    return integral_error_;
  }

  virtual void get_current_time(double& time, double& delta_time) = 0;

  void set_objects_active(bool parent_is_active) override;

  virtual void Load();
  virtual void Connect();
  virtual void Initialize() = 0;
  virtual void Integrate() = 0;

protected:
  virtual void DeclareParameters() override;
  std::string type_; //!< descriptive name of Integrator type
  unsigned int iterations_since_last_full_factor_ = 0;

  double omega_n_ = 200.0; //!< initial omega_n used in Baumgarte stabilization
                           //!< loaded from config file
  double baumgarte_omega_n_ =
    200.0; //!< omega_n used during simulation, may change for integrators that
           //!< use adaptive stabilization
  double baumgarte_omega_n_prev_ =
    200.0; //!< omega_n used in last successful integration step

  std::vector<double> integral_error_; /*!< integral error, equal to the
                                          number of constraints */
  double time_step_ = 0.01; /*!< time step used for reporting intervals */
  double time_span_ = 10.0; /*!< time span for numerical integration, defaults
                               to 10 seconds if not provided */
  shift::core::solver::Base* solver_ = nullptr;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace Integrator
}
} // namespace shift

#endif
