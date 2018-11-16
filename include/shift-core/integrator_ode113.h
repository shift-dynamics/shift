#ifndef _SHIFT_INTEGRATORODE113_H_
#define _SHIFT_INTEGRATORODE113_H_

#include <string>

#include "shift-core/integratorbase.h"

namespace Shift {

/*!
@brief Abstract base class for Shift Integrator C++ objects
*/
class IntegratorODE113 : IntegratorBase {

public:
  IntegratorODE113(std::string name)
    : IntegratorBase(name) {}
  ~IntegratorODE113() {}

private:
};

} // namespace Shift

#endif
