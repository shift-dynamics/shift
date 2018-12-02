#ifndef _SHIFT_FORCEMODULE_H_
#define _SHIFT_FORCEMODULE_H_

#include <functional>
#include <map>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace ForceModule {

/*!
@brief Shift::Frame abstract base class
*/
class ForceModuleBase
  : public Base
  , public Serialize {

public:
  ForceModuleBase(std::string name)
    : Base(name)
    , Serialize() {
    addKeys();
  }
  ForceModuleBase()
    : Base("")
    , Serialize() {
    addKeys();
  }
  ~ForceModuleBase() {}

private:
  void addKeys();
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

  typedef std::function<ForceModuleBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name, CreateFunction pfnCreate);
  static ForceModuleBase* create(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace ForceModule

} // namespace Shift

#endif
