#ifndef _SHIFT_CSVLOGGER_H_
#define _SHIFT_CSVLOGGER_H_

#include "shift-core/loggers/logger.h"
#include <string>

namespace Shift {

namespace Logger {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class CSV : public LoggerBase {

public:
  CSV(std::string name)
    : LoggerBase(name) {
    addKeys();
  }
  CSV()
    : LoggerBase("") {
    addKeys();
  }
  ~CSV() {}

  const std::string& getDescription() {
    return description_;
  }

  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

  void readJSON(Json::Value&);

private:
  const std::string description_ = "";
  static bool registered_;
  static std::unique_ptr<CSV> create();
};

} // namespace Logger

} // namespace Shift

#endif
