#ifndef SHIFT_LOGGER_BASE_H_
#define SHIFT_LOGGER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter/list.h"

namespace Shift {

class Context;

namespace Logger {

/*!
@brief Abstract Shift::Logger base class
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

  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "loggers";
  }

  static const std::string defaultType() {
    return "";
  }

  virtual void load();
  virtual void connect();
  virtual void initialize() = 0;
  virtual void setActiveObjects(bool parent_is_active) override;
  virtual void writeHeader() = 0;
  virtual void writeStates(const double t, const std::vector<double>& q) = 0;

protected:
  virtual void addParameters() override;
  bool log_all_channels_ = true;
  std::string type_; //!< descriptive name of Logger type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Logger

} // namespace Shift

#endif
