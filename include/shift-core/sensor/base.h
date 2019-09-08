#ifndef SHIFT_SENSOR_BASE_H_
#define SHIFT_SENSOR_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

namespace Shift {

namespace Body {
class Body;
}

class Frame;
class Context;

namespace Sensor {

/*!
@brief Abstract Shift::Sensor base class
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
    return "sensors";
  }

  static const std::string defaultType() {
    return "";
  }

  virtual void load();
  virtual void connect();
  virtual void setActiveObjects(bool parent_is_active) override;

protected:
  virtual void addParameters() override;
  std::string type_; //!< descriptive name of Sensor type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Sensor

} // namespace Shift

#endif
