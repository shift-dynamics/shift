#ifndef SHIFT_INTEGRATOR_BASE_H_
#define SHIFT_INTEGRATOR_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

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
  }
  Base()
    : Shift::Base("") {
    addParameters();
  }
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "integrator";
  }
  static const std::string defaultType() {
    return "";
  }
  virtual void load();
  virtual void connect();
  void setContext(Context* context) {
    context_ = context;
  }
  virtual void initialize() = 0;

protected:
  Context* context_ = nullptr;
  void addParameters();

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
