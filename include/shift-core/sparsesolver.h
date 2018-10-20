#ifndef _SHIFT_SPARSESOLVER_H_
#define _SHIFT_SPARSESOLVER_H_

#include <string>

#include "solverbase.h"

namespace Shift {

/*!
@brief Abstract base class for Shift C++ objects
*/
class SparseSolver : SolverBase {

public:
  SparseSolver(std::string name)
    : SolverBase(name) {}
  ~SparseSolver() {}

private:
};

} // namespace Shift

#endif
