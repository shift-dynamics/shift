#ifndef SHIFT_BEHAVIOR_BASE_H_
#define SHIFT_BEHAVIOR_BASE_H_

#include <functional>
#include <json/json.h>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

namespace Shift {

class Context;

namespace Behavior {

/*!
@brief Abstract base class for Shift::Behavior C++ objects
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
  ~Base() {}

  const std::string& getType() {
    return type_;
  }

  virtual void load();
  virtual void connect();
  virtual void addParameters() override;

  static const std::string arrayKey() {
    return "behaviors";
  }

  static const std::string defaultType() {
    return "";
  }

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
