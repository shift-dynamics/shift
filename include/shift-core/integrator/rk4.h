#ifndef SHIFT_INTEGRATOR_RK4_H_
#define SHIFT_INTEGRATOR_RK4_H_

#include <string>

#include "shift-core/integrator/base.h"

namespace Shift {

namespace Integrator {
/*!

*/
class RK4 : public Base {

public:
  RK4(std::string name)
    : Base(name) {
    addParameters();
  }
  RK4()
    : Base("") {
    addParameters();
  }
  virtual ~RK4() override {}
  static std::unique_ptr<RK4> create();

  static bool isRegistered() {
    return registered_;
  }
  virtual void initialize() override{};

private:
  virtual void addParameters() override;
  static bool registered_;
};

} // namespace Integrator

} // namespace Shift

#endif
