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
    addParameters();
  }
  ODE113()
    : Base("") {
    addParameters();
  }
  virtual ~ODE113() override {}
  static std::unique_ptr<ODE113> create();
  void addParameters();

  static bool isRegistered() {
    return registered_;
  }
  virtual void initialize() override{};

private:
  static bool registered_;
};

} // namespace Integrator

} // namespace Shift

#endif
