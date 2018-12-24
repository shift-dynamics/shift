#ifndef _SHIFT_LOGGER_H_
#define _SHIFT_LOGGER_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Context;

namespace Logger {

class LoggerBase;

/*!
@brief Abstract Shift::Logger base class
*/
class LoggerBase
  : public Base
  , public Serialize {

public:
  LoggerBase(std::string name)
    : Base(name) {
    addKeys();
  }

  LoggerBase()
    : Base("") {
    addKeys();
  }

  ~LoggerBase() {}

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

  std::string type_; //!< descriptive name of Logger type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Logger

} // namespace Shift

#endif
