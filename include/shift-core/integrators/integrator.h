#ifndef _SHIFT_INTEGRATOR_H_
#define _SHIFT_INTEGRATOR_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Frame;

namespace Integrator {

/*!
@brief Abstract Integrator base class
*/
class IntegratorBase
  : public Base
  , public Serialize {

public:
  IntegratorBase(std::string name)
    : Base(name) {
    addKeys();
  }
  IntegratorBase()
    : Base("") {
    addKeys();
  }
  ~IntegratorBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Integrator type
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

  typedef std::function<IntegratorBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name, CreateFunction pfnCreate);
  static IntegratorBase* create(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Integrator

} // namespace Shift

#endif
