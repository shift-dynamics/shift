#ifndef SHIFT_SOLVER_BASE_H_
#define SHIFT_SOLVER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

namespace joint {
class Base;
}

namespace constraint {
class Base;
}

namespace integrator {
class Base;
}

class Frame;
class Context;

namespace solver {

/*!
@brief Abstract Solver base class
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {}
  Base()
    : shift::core::Base("") {}
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "solvers";
  }

  static const std::string defaultType() {
    return "";
  }

  virtual void load();
  virtual void connect();
  virtual void initialize() = 0;
  virtual void setActiveObjects(bool parent_is_active) override;
  virtual bool solve(const double t, const std::vector<double>& y,
                     std::vector<double>& dy) = 0;

protected:
  virtual void addParameters() override;
  std::string type_; //!< descriptive name of Solver type
  shift::core::integrator::Base* integrator_ = nullptr;
};

class Modules {
public:
  static bool initialized();
};

} // namespace solver
}
} // namespace shift

#endif
