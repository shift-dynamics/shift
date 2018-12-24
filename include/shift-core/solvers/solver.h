#ifndef _SHIFT_SOLVER_H_
#define _SHIFT_SOLVER_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class BodyBase;
}

namespace Joint {
class JointBase;
}

namespace Constraint {
class ConstraintBase;
}

class Frame;
class Context;

namespace Solver {

/*!
@brief Abstract Solver base class
*/
class SolverBase
  : public Base
  , public Serialize {

public:
  SolverBase(std::string name)
    : Base(name) {}
  SolverBase()
    : Base("") {}
  ~SolverBase() {}

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
