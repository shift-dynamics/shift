#ifndef SHIFT_BEHAVIOR_BASE_H_
#define SHIFT_BEHAVIOR_BASE_H_

#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {

class Context;

namespace Behavior {

/*!
@brief Abstract base class for shift::Behavior C++ objects
*/
class Base
  : public shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : shift::Base(name) {
    addParameters();
  }
  Base()
    : shift::Base("") {
    addParameters();
  }
  virtual ~Base() override {}

  const std::string& getType() {
    return type_;
  }

  virtual void load();
  virtual void connect();
  virtual void addParameters() override;
  virtual void setActiveObjects(bool parent_is_active) override;

  static const std::string arrayKey() {
    return "behaviors";
  }

  static const std::string defaultType() {
    return "";
  }

protected:
  std::string type_; //!< descriptive name of Behavior type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Behavior

} // namespace shift

#endif
