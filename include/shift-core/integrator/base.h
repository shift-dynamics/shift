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
    std::cout << "Shift::Integrator::Base()" << std::endl;
  }
  Base()
    : Shift::Base("") {
    addParameters();
    std::cout << "Shift::Integrator::Base()" << std::endl;
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

protected:
  virtual void addParameters() override;
  std::string type_; //!< descriptive name of Integrator type
  unsigned int iterations_since_last_full_factor_ = 0;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Integrator

} // namespace Shift

#endif
