#ifndef SHIFT_LOGGER_BASE_H_
#define SHIFT_LOGGER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/log.h"

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

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();
  virtual void Initialize() = 0;
  virtual void WriteHeader() = 0;
  virtual void WriteStates(const double t, const std::vector<double>& q) = 0;

  void RegisterLogValue(shift::core::parameter::Value* value) {
    for(auto& val : log_values_) {
      if(val == value) {
        SHIFT_TRACE("Log value " + value->get_key() + " already added");
        return;
      } 
    }
    log_values_.push_back(value);
  }
  
  void UnregisterLogValue(shift::core::parameter::Value* value) {
    auto it = std::find(log_values_.begin(), log_values_.end(), value);
    auto index = std::distance(log_values_.begin(), it);
    if((size_t)index < log_values_.size()) {
      log_values_.erase(log_values_.begin() + index);
    }
  }

protected:
  virtual void DeclareParameters() override;
  bool log_all_channels_ = true;
  std::string type_; //!< descriptive name of Logger type
  std::vector<shift::core::parameter::Value*> log_values_;
  
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace Logger
}
} // namespace shift

#endif
