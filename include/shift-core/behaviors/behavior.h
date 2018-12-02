#ifndef _SHIFT_BEHAVIOR_H_
#define _SHIFT_BEHAVIOR_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Behavior {

/*!
@brief Abstract base class for Shift::Behavior C++ objects
*/
class BehaviorBase
  : public Base
  , public Serialize {

public:
  BehaviorBase(std::string name)
    : Base(name) {
    addKeys();
  }
  BehaviorBase()
    : Base("") {
    addKeys();
  }
  ~BehaviorBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Behavior type
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

  typedef std::function<BehaviorBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name, CreateFunction pfnCreate);
  static BehaviorBase* create(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Behavior

} // namespace Shift

#endif
