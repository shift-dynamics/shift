#ifndef SHIFT_SENSOR_BASE_H_
#define SHIFT_SENSOR_BASE_H_

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace shift {
namespace core {

namespace body {
class Base;
}

class Frame;
class Context;

namespace sensor {

/*!
@brief Abstract shift::Sensor base class
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
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "sensors";
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

protected:
  virtual void DeclareParameters() override;
  std::string type_; //!< descriptive name of Sensor type
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace sensor
}
} // namespace shift

#endif
