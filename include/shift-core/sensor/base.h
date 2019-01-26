#ifndef SHIFT_SENSOR_BASE_H_
#define SHIFT_SENSOR_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

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
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name) {
    addKeys();
  }
  Base()
    : Shift::Base("") {
    addKeys();
  }
  ~Base() {}

  const std::string& getType() {
    return type_;
  }
  void setContext(Context* context) {
    context_ = context;
  }

  virtual void readJSON(Json::Value&);
  virtual void load();
  virtual void connect();

protected:
  Context* context_;
  void addKeys();

  std::string type_; //!< descriptive name of Sensor type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Sensor

} // namespace Shift

#endif
