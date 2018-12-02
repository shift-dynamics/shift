#ifndef _SHIFT_SOLVER_H_
#define _SHIFT_SOLVER_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class BodyBase;
}

class Frame;

namespace Solver {

/*!
@brief Abstract Solver base class
*/
class SolverBase
  : public Base
  , public Serialize {

public:
  SolverBase(std::string name)
    : Base(name) {
    addKeys();
  }
  SolverBase()
    : Base("") {
    addKeys();
  }
  ~SolverBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Solver type
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

  typedef std::function<SolverBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name, CreateFunction pfnCreate);
  static SolverBase* create(const std::string& name);
  static void setDefaultModule(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
  static std::string defaultModule_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Solver
} // namespace Shift

#endif
