#ifndef _SHIFT_SENSOR_H_
#define _SHIFT_SENSOR_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class Body;
}

class Frame;

namespace Sensor {

/*!
@brief Abstract Shift::Sensor base class
*/
class SensorBase
  : public Base
  , public Serialize {

public:
  SensorBase(std::string name)
    : Base(name) {
    addKeys();
  }
  SensorBase()
    : Base("") {
    addKeys();
  }
  ~SensorBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Sensor type
};

class Modules {
public:
  void list();
  static bool initialized();

private:
  static std::vector<std::string> modules_;
  static bool initialized_;
};

} // namespace Sensor

} // namespace Shift

#endif
