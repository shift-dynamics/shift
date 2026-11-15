#ifndef SHIFT_LOGGER_BASE_H_
#define SHIFT_LOGGER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/base.h"
#include "shift/core/log.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace logger {

/*!
@brief a component whose outputs a logger records, an entry of the logger's
force_modules, sensors or controllers: its name, as a joint names a body
(the fully qualified name, \<assembly\>__\<name\>, where the name alone is
not unique), and the outputs to record, all of them if none are given
*/
class LoggedComponent : public parameter::List {
public:
  LoggedComponent() {
    DeclareParameters();
  }
  virtual ~LoggedComponent() {}

  static const std::string get_array_key() {
    return "logged_components";
  }

  std::string name;
  std::vector<std::string> outputs;

protected:
  virtual void DeclareParameters() override;
};

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

  /*! \brief an output of a force module, sensor or controller that the
   * logger records at each output time */
  struct Channel {
    std::string kind;      //!< "force_modules", "sensors" or "controllers"
    std::string component; //!< fully qualified name of the component
    std::string output;    //!< key of the output
    const double* value = nullptr;
    /*! \brief the column of a table of the outputs:
     * \<component\>__\<output\> */
    std::string Column() const {
      return component + "__" + output;
    }
  };

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();
  /*! \brief connect the logger to the outputs it records: those of the
   * components of force_modules, sensors and controllers, in their order, or,
   * without any of the three, every output of the active force modules, then
   * sensors, then controllers. Called once every component has declared its
   * outputs (after Load), before Initialize; throws if an entry names no
   * active component, or an output it does not have, or if its name is
   * ambiguous. */
  void ConnectOutputs();
  virtual void ReadJSON(Json::Value& json_value) override;
  const std::vector<Channel>& get_channels() const {
    return channels_;
  }
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
  /*! \brief add the channels of entry, of the components of kind, to
   * channels_ */
  template <class T>
  void ConnectEntry(const LoggedComponent& entry, const std::string& kind,
                    const std::vector<T*>& components);
  bool log_all_channels_ = true;
  /*! \brief the components whose outputs are recorded, by kind */
  parameter::Array<LoggedComponent> force_modules_, sensors_, controllers_;
  /*! \brief whether the model file gives force_modules, sensors or
   * controllers */
  bool chooses_outputs_ = false;
  std::vector<Channel> channels_;
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
