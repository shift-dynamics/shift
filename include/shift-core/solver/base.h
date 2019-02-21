#ifndef SHIFT_SOLVER_BASE_H_
#define SHIFT_SOLVER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class Base;
}

namespace Joint {
class Base;
}

namespace Constraint {
class Base;
}

class Frame;
class Context;

namespace Solver {

/*!
@brief Abstract Solver base class
*/
class Base
  : public Shift::Base
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name) {}
  Base()
    : Shift::Base("") {}
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }
  virtual void readJSON(Json::Value&);
  virtual void load();
  virtual void connect();
  void setContext(Context* context) {
    context_ = context;
  }

protected:
  Context* context_;
  void addKeys();
  std::string type_; //!< descriptive name of Solver type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Solver

} // namespace Shift

#endif
