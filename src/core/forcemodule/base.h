#ifndef SHIFT_FORCEMODULE_BASE_H_
#define SHIFT_FORCEMODULE_BASE_H_

#include <functional>
#include <map>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace forcemodule {

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
    addParameters();
  }
  Base()
    : shift::core::Base("")
    , parameter::List() {
    addParameters();
  }
  virtual ~Base() {}

  static const std::string arrayKey() {
    return "force_modules";
  }
  static const std::string defaultType() {
    return "";
  }
  virtual void load();
  virtual void connect() = 0;
  virtual void service(const double*, double*) = 0;
  virtual void setActiveObjects(bool parent_is_active) override;
  unsigned int equationCount() {
    return equation_count_;
  }
  const std::vector<double>& q0() {
    return q_0_;
  }

protected:
  std::string type_;
  virtual void addParameters() override;
  unsigned int equation_count_ = 0;
  std::vector<double> q_0_; /*!< vector of state initial conditions */
  double* dq_ =
    nullptr; /*!< pointer to state time derivatives for transient modules */
};

class Modules {
public:
  static bool initialized();
};

} // namespace forcemodule
}
} // namespace shift

#endif
