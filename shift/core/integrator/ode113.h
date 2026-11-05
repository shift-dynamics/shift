#ifndef SHIFT_INTEGRATOR_ODE113_H_
#define SHIFT_INTEGRATOR_ODE113_H_

#include "shift/core/array2d.h"
#include "shift/core/integrator/base.h"

#include <string>

namespace shift {
namespace core {
namespace integrator {

/*!

*/
class ODE113 : public Base {

public:
  ODE113(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  ODE113()
    : Base("") {
    DeclareParameters();
  }
  virtual ~ODE113() override {}
  static std::unique_ptr<ODE113> Create();

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
  void Step();
  void StepFunction(const double t, std::vector<double>& y);

  double absolute_error_ = 0.0001;
  double relative_error_ = 0.0;
  std::vector<double> y_;  /*!< y is a de copy of the states communicated to the
                     calling program. Calling program provides y as the
                     the initial conditions. de copies y into yy and
                     sends this to step. If step fails to advance solution,
                     de copies latest yy from step back to y and returns
                     this result to calling program for continuation. */
  std::vector<double> dy_; /*!< This is the array de passes to step to use for
                   predicted and corrected state derivatives. If successful,
                   step computes dy and returns it to de as dy. If step
                   is unsuccessful, de just ignores dy. */
  std::vector<double>
    ydata_; /*!< array to store results from numerical integration */
  std::vector<double> tdata_; /*!< time stamps */
  std::vector<double> yy_;
  std::vector<double> dyout_;
  std::vector<double> wt_;   /*< This array holds computed weights used to
                      estimate correction error limits. */
  std::vector<double> p_;    /*< This is the array de passes to step to use for
  predicted and corrected states. If successful, step
  copies p into its local y vector and returns it as yy
  to de. If unsuccessful, step just returns its latest y
  to de as yy. */
  std::vector<double> yerr_; /*<! Array of estimated integration errors passed
                   to fcn. May be used for debugging purposes. */
  double hmin_ = 0.00001;    /*!< Minimum step size */
  double t_ = 0.0;

  unsigned int step_count_ = 0;
  unsigned int iteration_count_ = 0;

  unsigned int ns_ = 0; /*!< Internal counter of the number of successful
                             solutions. It is used to help set the current
                            integration order. */
  int iflag_ = 1;       /*<! Internal status flag that indicates integrator
                          prediction failure. After four failures, the integrator
                          resets the order k to 1. */
  int ifail_ = 0;
  bool newton_raphson_failure_ = false;
  bool nornd_ = false;
  double x_ = 0.0; /*!< Time variable passed by de to and changed by step */
  // double tout_ = 0.0;
  unsigned int neqn_ = 0; /*!< number of equations */
  std::vector<int> reconfigurations_;
  double hmax_ = 0.01; /*!< maximum step size */

  static const double twou_;  /*!<
 The only machine dependent constants are based on the machine unit
 roundoff error  u  which is the smallest positive number such that
 1.0+u > 1.0: twou_ = 2.0 * u */
  static const double fouru_; /*!<
The only machine dependent constants are based on the machine unit
roundoff error  u  which is the smallest positive number such that
1.0+u > 1.0: fouru_ = 4.0 * u */
  static const double gstr_[14];
  static const double two_[14];
  static const double stability_limits_[17];

  // double hh_ = 0.0;
  double eps_ = 0.0; /*!< local error tolerance per step, must be variable,
                        not a constant */
  // double told_ = 0.0;
  double hold_ = 0.0;
  double beta_[13] = {};
  double psi_[13] = {}; /*!< Internal array used to set up polynomials. */
  double alpha_[13] = {};
  double w_[14] = {};
  double v_[13] = {};
  double g_[14] = {0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double sig_[14] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; /*!< Internal array used to
                                                       set up polynomials. */

  shift::core::Array2d<double> phi_;

  bool crash_ = false;
  bool start_ = false;

  unsigned int k_ = 0;    /*!< the order for the next step */
  unsigned int kold_ = 0; /*! the order of the last successful step */
  int iconv_ = 0;         /*!< Integrator status flag provided to fcn.
                            iconv = 1 when fcn is called after a successful
                            correction. In this case the states are accurate.
                            iconv = 0 when fcn is called after a prediction
                            following a successful correction.
                            iconv = -1 when fcn is called after a prediction
                            following an unsuccessful correction. */
  double h_;              /*!< current step size */
  unsigned int step_ = 0; /*!< current step of numerical integration */

  double smoothing_filter_ =
    0.001; //!< smoothing filter used for adaptive Baumgarte stabilization

  double integral_gain_ = 0.0; /*!< constraint error integral gain, off by
                                  default as in the original ode113b */
  double safety_factor_ = 0.005;
  double omega_n_max_ =
    1000.0; /*!< maximum omega_n used in Baumgarte constraint stabilization */
  double omega_n_min_ =
    10.0; /*!< minimum omega_n used in Baumgarte constraint stabilization */
  bool phase1_ = false; /*<! phase1 is a logical variable. When TRUE all
                   lights are go and the integrator may increase order and
                       integration stepsize. When false, integrator may
                       reduce order and stepsize. Also when false, integrator
                       will move cautiously as its error estimates may be
                   unreliable. */

  static bool registered_;
};

} // namespace integrator
} // namespace core
} // namespace shift

#endif
