#ifndef SHIFT_LOGGER_BASE_H_
#define SHIFT_LOGGER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

class Context;

namespace logger {

/*!
@brief Abstract shift::Logger base class
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }

  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }

  virtual ~Base() {}

  const std::string& get_type() {
    return type_;
  }

  static const std::string get_array_key() {
    return "loggers";
  }

  static const std::string get_default_type() {
    return "";
  }

  virtual void set_objects_active(bool parent_is_active) override;
  
  virtual void Load();
  virtual void Connect();
  virtual void Initialize() = 0;
  virtual void WriteHeader() = 0;
  virtual void WriteStates(const double t, const std::vector<double>& q) = 0;

protected:
  virtual void DeclareParameters() override;
  bool log_all_channels_ = true;
  std::string type_; //!< descriptive name of Logger type
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace Logger
}
} // namespace shift

#endif
