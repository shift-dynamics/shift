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
    addParameters();
  }
  CSV()
    : Base("") {
    addParameters();
  }
  virtual ~CSV() override {}

  const std::string& getDescription() {
    return description_;
  }

  static bool isRegistered() {
    return registered_;
  }

  virtual void initialize() override {}

private:
  virtual void addParameters() override;

  const std::string description_ = "";
  static bool registered_;
  static std::unique_ptr<CSV> create();
};

} // namespace Logger

} // namespace Shift

#endif
