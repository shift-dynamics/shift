#ifndef SHIFT_INTEGRATOR_BASE_H_
#define SHIFT_INTEGRATOR_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Frame;
class Context;

namespace Integrator {

/*!
@brief Abstract Integrator base class
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
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  void readJSON(Json::Value&);
  virtual void load();
  virtual void connect();
  void setContext(Context* context) {
    context_ = context;
  }

protected:
  Context* context_ = NULL;
  void addKeys();

  std::string type_; //!< descriptive name of Integrator type
  unsigned int iterationsSinceLastFullFactor_ = 0;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Integrator

} // namespace Shift

#endif
