#ifndef _SHIFT_INTEGRATORBASE_H_
#define _SHIFT_INTEGRATORBASE_H_

#include <string>

#include "shift-core/base.h"

namespace Shift {

enum class Integrator { ODE113, ODE113B };

/*!
@brief Abstract base class for Shift Integrator C++ objects
*/
class IntegratorBase : Base {

public:
  IntegratorBase(std::string name)
    : Base(name) {}
  ~IntegratorBase() {}

  Integrator getType();

private:
  Integrator type_;
};

} // namespace Shift

#endif
