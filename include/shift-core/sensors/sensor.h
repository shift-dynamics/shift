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

class Factory {

public:
  Factory();
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<SensorBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& SensorName,
                             CreateFunction pfnCreate);
  static SensorBase* create(const std::string& SensorName);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Sensor

} // namespace Shift

#endif
