#ifndef SHIFT_BEHAVIOR_BASE_H_
#define SHIFT_BEHAVIOR_BASE_H_

#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace behavior {

/*!
@brief Abstract base class for shift::core::behavior C++ objects
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    addParameters();
  }
  Base()
    : shift::core::Base("") {
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

} // namespace behavior
}
} // namespace shift

#endif
