#ifndef _SHIFT_ODE113INTEGRATOR_H_
#define _SHIFT_ODE113INTEGRATOR_H_

#include <string>

#include "shift-core/integrators/integrator.h"

namespace Shift {

/*!

*/
class ODE113Integrator : Integrator {

public:
  ODE113Integrator(std::string name)
    : Integrator(name) {}
  ~ODE113Integrator() {}

private:
};

} // namespace Shift

#endif
