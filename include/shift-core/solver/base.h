#ifndef SHIFT_SOLVER_BASE_H_
#define SHIFT_SOLVER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

namespace Shift {

namespace Body {
class Base;
}

namespace Joint {
class Base;
}

namespace Constraint {
class Base;
}

class Frame;
class Context;

namespace Solver {

/*!
@brief Abstract Solver base class
*/
class Base
  : public Shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name) {}
  Base()
    : Shift::Base("") {}
  virtual ~Base() {
    std::cout << "virtual ~Solver::Base() override: " << name_ << std::endl;
  }

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "force_modules";
  }

  static const std::string defaultType() {
    return "";
  }

  virtual void load();
  virtual void connect();
  virtual void initialize() = 0;
  virtual void setActiveObjects(bool parent_is_active) override;

protected:
  virtual void addParameters() override;
  std::string type_; //!< descriptive name of Solver type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Solver

} // namespace Shift

#endif
