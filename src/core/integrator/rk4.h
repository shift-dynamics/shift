#ifndef SHIFT_INTEGRATOR_RK4_H_
#define SHIFT_INTEGRATOR_RK4_H_

#include <string>

#include "shift/core/integrator/base.h"

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
  virtual void connect() override;
  virtual void initialize() override;
  virtual void reset();
  virtual void integrate() override;

private:
  virtual void addParameters() override;
  static bool registered_;

  std::vector<double> y_;
  std::vector<double> dy_;

  unsigned int neqn_ = 0;
  unsigned int step_count_ = 0;

  std::vector<double> f0_;
  std::vector<double> f1_;
  std::vector<double> f2_;
  std::vector<double> f3_;

  std::vector<double> u0_;
  std::vector<double> u1_;
  std::vector<double> u2_;
  std::vector<double> u3_;
};

} // namespace Integrator

} // namespace Shift

#endif
