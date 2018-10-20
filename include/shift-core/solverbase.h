#ifndef _SHIFT_SOLVERBASE_H_
#define _SHIFT_SOLVERBASE_H_

#include <string>

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Abstract base class for Shift C++ objects
*/
class SolverBase : Base {

public:
  SolverBase(std::string name)
    : Base(name) {}
  ~SolverBase() {}

private:
};

} // namespace Shift

#endif
