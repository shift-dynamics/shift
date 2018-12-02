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

class Factory {

public:
  Factory();
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<LoggerBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& LoggerName,
                             CreateFunction pfnCreate);
  static LoggerBase* create(const std::string& LoggerName);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Logger

} // namespace Shift

#endif
