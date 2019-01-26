#ifndef SHIFT_BEHAVIOR_BASE_H_
#define SHIFT_BEHAVIOR_BASE_H_

#include <functional>
#include <json/json.h>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Context;

namespace Behavior {

/*!
@brief Abstract base class for Shift::Behavior C++ objects
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

  virtual void load();
  virtual void connect();
  virtual void readJSON(Json::Value&);
  void setContext(Context* context) {
    context_ = context;
  }

protected:
  Context* context_ = NULL;
  void addKeys();

  std::string type_; //!< descriptive name of Behavior type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Behavior

} // namespace Shift

#endif
