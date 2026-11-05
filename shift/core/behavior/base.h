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
    DeclareParameters();
  }
  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }
  virtual ~Base() override {}

  static const std::string get_array_key() {
    return "behaviors";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();
  virtual void DeclareParameters() override;

protected:
  std::string type_; //!< descriptive name of Behavior type
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace behavior
}
} // namespace shift

#endif
