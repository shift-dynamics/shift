#ifndef _SHIFT_LOGGER_H_
#define _SHIFT_LOGGER_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

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

protected:
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
