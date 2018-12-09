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
    : Base(name) {}
  SolverBase()
    : Base("") {}
  ~SolverBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Solver type
};

class Modules {
public:
  void list();
  static bool initialized();

private:
  static std::vector<std::string> modules_;
  static bool initialized_;
};

} // namespace Solver

} // namespace Shift

#endif
