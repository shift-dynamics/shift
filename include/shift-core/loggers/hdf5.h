#ifndef _SHIFT_HDF5LOGGER_H_
#define _SHIFT_HDF5LOGGER_H_

#include "shift-core/loggers/logger.h"
#include <string>

namespace Shift {

namespace Logger {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class HDF5 : public LoggerBase {

public:
  HDF5(std::string name)
    : LoggerBase(name) {
    addKeys();
  }
  HDF5()
    : LoggerBase("") {
    addKeys();
  }
  ~HDF5() {}

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
  static std::unique_ptr<HDF5> create();
};

} // namespace Logger

} // namespace Shift

#endif
