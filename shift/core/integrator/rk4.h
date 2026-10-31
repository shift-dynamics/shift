#ifndef SHIFT_INTEGRATOR_RK4_H_
#define SHIFT_INTEGRATOR_RK4_H_

#include "shift/core/integrator/base.h"

#include <string>
#include <vector>

namespace shift {
namespace core {
namespace integrator {

/*!
@brief Classical fixed-step fourth-order Runge-Kutta integrator
*/
class RK4 : public Base {

public:
  RK4(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  RK4()
    : Base("") {
    DeclareParameters();
  }
  virtual ~RK4() override {}
  static std::unique_ptr<RK4> Create();

  static bool is_registered() {
    return registered_;
  }

  virtual void get_current_time(double& time, double& delta_time) override;

  virtual void Initialize() override;
  virtual void Connect() override;
  virtual void Integrate() override;
  virtual void Reset();

private:
  virtual void DeclareParameters() override;
  void Step(const double h);
  void StepFunction(const double t, const std::vector<double>& y,
                    std::vector<double>& dy);

  double step_size_ = 0.0; /*!< internal integration step size, defaults to
                              time_step_ if not provided */
  double integral_gain_ = 0.0; /*!< constraint error integral gain, off by
                                  default as in the original ode113b */

  double t_ = 0.0; /*!< time of the current states y_ */
  double h_ = 0.0; /*!< internal integration step size in use */
  unsigned int neqn_ = 0; /*!< number of equations */

  std::vector<double> y_;  /*!< states at time t_ */
  std::vector<double> k1_; /*!< state derivatives at (t_, y_) */
  std::vector<double> k2_; /*!< state derivatives at the first midpoint */
  std::vector<double> k3_; /*!< state derivatives at the second midpoint */
  std::vector<double> k4_; /*!< state derivatives at the end of the step */
  std::vector<double> yt_; /*!< trial states used to evaluate k2_, k3_, k4_ */

  static bool registered_;
};

} // namespace integrator
} // namespace core
} // namespace shift

#endif
