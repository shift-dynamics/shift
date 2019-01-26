#ifndef SHIFT_INTEGRATOR_ODE113_H_
#define SHIFT_INTEGRATOR_ODE113_H_

#include <string>

#include "shift-core/integrator/base.h"

namespace Shift {

namespace Integrator {
/*!

*/
class ODE113 : public Base {

public:
  ODE113(std::string name)
    : Base(name) {
    addKeys();
  }
  ODE113()
    : Base("") {
    addKeys();
  }
  ~ODE113() {}
  static std::unique_ptr<ODE113> create();
  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Integrator

} // namespace Shift

#endif
