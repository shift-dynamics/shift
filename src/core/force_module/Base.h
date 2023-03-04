#ifndef SHIFT_FORCEMODULE_BASE_H_
#define SHIFT_FORCEMODULE_BASE_H_

#include <functional>
#include <map>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

class Context;

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

  virtual void ActivateObjects(bool parent_is_active) override;
  

  virtual void Load();
  virtual void Connect() = 0;
  virtual void Service(const double*, double*) = 0;

protected:
  std::string type_;
  virtual void DeclareParameters() override;
  unsigned int equation_count_ = 0;
  std::vector<double> q_0_; /*!< vector of state initial conditions */
  double* dq_ =
    nullptr; /*!< pointer to state time derivatives for transient modules */
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace force_module
}
} // namespace shift

#endif
