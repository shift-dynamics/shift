#ifndef _SHIFT_INTEGRATOR_H_
#define _SHIFT_INTEGRATOR_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Frame;
class Context;

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

  void readJSON(Json::Value&);
  virtual void load();
  virtual void connect();
  void setContext(Context* context) {
    context_ = context;
  }

protected:
  Context* context_ = NULL;
  void addKeys();

  std::string type_; //!< descriptive name of Integrator type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Integrator

} // namespace Shift

#endif
