#ifndef _SHIFT_ODE113INTEGRATOR_H_
#define _SHIFT_ODE113INTEGRATOR_H_

#include <string>

#include "shift-core/integrators/integrator.h"

namespace Shift {

namespace Integrator {
/*!

*/
class ODE113 : public IntegratorBase {

public:
  ODE113(std::string name)
    : IntegratorBase(name) {
    addKeys();
  }
  ODE113()
    : IntegratorBase("") {
    addKeys();
  }
  ~ODE113() {}
  static ODE113* create();
  void addKeys();

private:
};

} // namespace Integrator

} // namespace Shift

#endif
