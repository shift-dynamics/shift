#ifndef _SHIFT_LOGGER_CSV_H_
#define _SHIFT_LOGGER_CSV_H_

#include "shift-core/logger/base.h"
#include <string>

namespace Shift {

namespace Logger {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class CSV : public Base {

public:
  CSV(std::string name)
    : Base(name) {
    addKeys();
  }
  CSV()
    : Base("") {
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
