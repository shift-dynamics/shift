#ifndef SHIFT_CONTROLLER_MINIMUM_TIME_H_
#define SHIFT_CONTROLLER_MINIMUM_TIME_H_

#include <cmath>
#include <functional>
#include <vector>

#include "shift/core/controller/interior_point.h"
#include "shift/core/controller/slide_equilibrium.h"

namespace shift {
namespace core {
namespace controller {

/*!
@brief the run of least time of a four-wheel drive car along a road, on loose
ground or paved, after the minimum time cornering of D. Tavernini, M.
Massaro, E. Velenis, D. I. Katzourakis and R. Lot (Minimum time cornering:
the effect of road surface and car transmission layout, Vehicle System
Dynamics 51, 2013) and E. Velenis, P. Tsiotras and J. Lu (Optimality
properties and driver input parameterization for trail-braking cornering,
European Journal of Control 14, 2008)

The car is SlideEquilibrium's, in the plane, its motion (SlideEquilibrium::
PathDerivatives) taken along the road's middle by the station \f$s\f$ instead
of the time: with the state \f$\underline{x} = (v_x, v_y, r, e, \Delta\chi,
\delta, \omega_c)\f$, the frame's velocity, the yaw rate, the lateral error
from the middle (positive to the right), the course's error from the road's
direction, the steering of the front wheels and the speed of the wheels'
carriers, the frame advances along the road at

\f[
\dot s = \frac{v \cos\Delta\chi}{1 - \kappa e},
\f]

\f$\kappa\f$ the road's curvature (positive to the right), and
\f$d\underline{x}/ds = \dot{\underline{x}} / \dot s\f$. The inputs are the
rates of the steering and of the carriers' speed along the road,
\f$\underline{u} = (d\delta/ds, d\omega_c/ds)\f$: the wheels' speed is a
state, as the engine and the brakes set it much faster than the car moves,
and the torque of the drive is that which the tires take,
\f$\tau = \sum \tau_i\f$, held within what the engine gives at full throttle
in its best gear (torque_limit) and the brakes (brake_torque). The time is

\f[
T = \int_{s_0}^{s_1} \frac{ds}{\dot s} + \int_{s_0}^{s_1} w_\delta
\left(\frac{d\delta}{ds}\right)^2 + w_\omega
\left(\frac{d\omega_c}{ds}\right)^2 ds,
\f]

the second integral small (rate_weights), to keep the inputs smooth, over
stations step apart, each step by the fourth order Runge-Kutta method and its
time by Simpson's rule. The car's outline (its corners about the frame) stays
on the road, within half_width of the middle, its steering within
steering_limit and its rate within steering_rate (rad/s), its speed above
min_speed, the course within max_course of the road's, and the run ends
within end_lateral of the middle, its heading within end_heading of the
road's and its yaw rate within end_yaw_rate.

The solution is by the interior point method of InteriorPoint (by
multiple shooting, from the first run), or, with interior_point false, by
iterative LQR (W. Li and E. Todorov, Iterative linear
quadratic regulator design for nonlinear biological movement systems, 2004),
the constraints by an augmented Lagrangian (T. A. Howell, B. E. Jackson and
Z. Manchester, ALTRO: A fast solver for constrained trajectory optimization,
IROS 2019): with the multipliers \f$\lambda \ge 0\f$ and the penalty
\f$\mu\f$ of each constraint \f$c(\underline{x}, \underline{u}) \le 0\f$, the
cost \f$T + \sum \lambda c + \frac{1}{2} \mu c^2\f$ over the constraints
violated or held by their multipliers is lowered by iLQR, the multipliers
then made \f$\max(0, \lambda + \mu c)\f$ and the penalty larger, until the
constraints hold. The derivatives of the steps are by central differences,
of the steps of the stations in parallel. The first run follows the road's
middle at guess_speed.
*/
class MinimumTime {
public:
  static constexpr int kStates = 7; //!< v_x, v_y, r, e, course, delta, omega_c
  static constexpr int kInputs = 2; //!< d delta / ds, d omega_c / ds
  static constexpr int kConstraints = 18;   //!< of each step
  static constexpr int kEndConstraints = 14; //!< of the last station

  SlideEquilibrium car;
  /*! the road's curvature at a station (1/m, positive to the right) */
  std::function<double(double)> curvature;
  /*! gravity's acceleration on the road at a station (m/s^2), along it,
   * to its right and normal to it, written to the three; without it, a
   * level road */
  std::function<void(double, double*)> slope;
  /*! the most torque of the drive at the wheels at full throttle at the
   * carriers' speed (rad/s), in the engine's best gear (N m) */
  std::function<double(double)> torque_limit;
  /*! the speed of the first run at a station (m/s) */
  std::function<double(double)> guess_speed;
  double brake_torque = 0.0;   //!< the most braking torque at the wheels (N m)
  double steering_limit = 0.5; //!< of the front wheels (rad)
  double steering_rate = 2.0;  //!< the most rate of the steering (rad/s)
  double half_width = 3.4;     //!< of the road (m)
  /*! the outline of the car about the frame: its rear (negative) and its
   * front (m) along x and its half width */
  double outline[3] = {0.0, 0.0, 0.0};
  double min_speed = 2.0;  //!< of v_x (m/s)
  double max_course = 1.2; //!< of the course from the road's (rad)
  double end_lateral = 0.25;
  double end_heading = 0.03;
  double end_yaw_rate = 0.1;
  /*! w_delta (s per (rad/m)^2 per m) and w_omega (s per (rad/s/m)^2 per m) */
  double rate_weights[2] = {0.05, 1.0e-5};
  double start = 0.0; //!< station of the start (m)
  double end = 0.0;   //!< station of the end (m)
  double step = 0.5;  //!< between stations (m)
  double start_speed = 3.0; //!< v_x at the start (m/s)
  double start_lateral = 0.0;
  bool interior_point = true; //!< InteriorPoint; false: iLQR
  /*! with interior_point, iterations of iLQR first, from whose run the
   * interior point method starts */
  int warm_start = 100;
  bool verbose = false;       //!< InteriorPoint's iterations to stderr
  bool gauss_newton = false;  //!< InteriorPoint's Hessian, see there
  bool project = true;        //!< InteriorPoint's, see there
  int max_iterations = 400; //!< of either, iLQR's over all the multipliers'
                            //!< updates
  double tolerance = 1.0e-3; //!< of the constraints (their units)

  /*! \brief the run of least time; true if the constraints hold within
   * tolerance */
  bool Solve();

  /*! \brief a run solved before, its states (kStates at each station from
   * start, step apart, to end), in place of Solve: the inputs from the
   * steering's and the carriers' changes, the times, the torques and the
   * constraints from the steps; true if the stations match */
  bool Load(const std::vector<double>& states);

  int get_stations() const {
    return static_cast<int>(times_.size());
  }
  /*! \brief the state at each station, kStates each */
  const std::vector<double>& get_states() const {
    return states_;
  }
  /*! \brief the inputs over each step, kInputs each */
  const std::vector<double>& get_inputs() const {
    return inputs_;
  }
  /*! \brief the time at each station from the start (s) */
  const std::vector<double>& get_times() const {
    return times_;
  }
  /*! \brief the drive torque at each station (N m) */
  const std::vector<double>& get_torques() const {
    return torques_;
  }
  /*! \brief the largest violation of the constraints */
  double get_violation() const {
    return violation_;
  }
  int get_iterations() const {
    return iterations_;
  }

  /*! \brief the gains of the time-varying linear quadratic regulator
   * along the solution: about each station's state, the errors
   * \f$\delta\underline{x} = (v_x, v_y, r, e, \Delta\chi, \delta)\f$ give
   * the steering's change along the road and the carriers' speed,
   * \f$(\delta', \omega_c) = (\delta', \omega_c)_k - K_k \delta
   * \underline{x}\f$, by the Riccati recursion from \f$P_N = Q\f$ for the
   * steps' linearizations \f$\delta \underline{x}_{k+1} = A_k \delta
   * \underline{x}_k + B_k \delta \underline{u}_k\f$ (the carriers'
   * speed held over a step) and the cost of each step \f$h (\delta
   * \underline{x}^T Q \delta \underline{x} + \delta \underline{u}^T R
   * \delta \underline{u})\f$, Q and R diagonal, the inverse squares of the
   * scales (kTrackStates, then the steering's change in rad/m and the
   * carriers' speed); the gains kTrackInputs x kTrackStates of each
   * station, row major; false if it fails */
  bool TrackingGains(const double* scales, std::vector<double>* gains);
  static constexpr int kTrackStates = 6;
  static constexpr int kTrackInputs = 2;

  /*! \brief the rates of the state along the road at its sample (each
   * half step from start), the time per meter and the drive torque; inputs
   * as Step */
  void Rates(int sample, const double* x, const double* u, double* dx,
             double* dt, double* torque) const;

  /*! \brief from station k's state x with the inputs u, the next station's,
   * the time of the step and the constraints c (kConstraints, at most 0) */
  void Step(int k, const double* x, const double* u, double* next,
            double* time, double* c) const;

  /*! \brief the constraints of the last station's state (kEndConstraints) */
  void EndConstraints(const double* x, double* c) const;

private:
  /*! \brief the stations, the samples of the curvature and the torque's
   * limit, and the car's tables, for Solve or Load */
  void Prepare();
  /*! \brief the times and the torques at the stations of the run */
  void Finish();
  /*! \brief the problem for InteriorPoint */
  class Shooting : public InteriorPoint::Problem {
  public:
    explicit Shooting(const MinimumTime& plan) : plan_(plan) {}
    int get_states() const override {
      return kStates;
    }
    int get_inputs() const override {
      return kInputs;
    }
    int get_steps() const override {
      return plan_.n_;
    }
    int get_constraints() const override {
      return kConstraints;
    }
    int get_end_constraints() const override {
      return kEndConstraints;
    }
    void Stage(int k, const double* x, const double* u, double* next,
               double* cost, double* c) const override;
    void End(const double* x, double* cost, double* c) const override;
    double StateScale(int i) const override;
    double InputScale(int i) const override;

  private:
    const MinimumTime& plan_;
  };

  /*! \brief iterative LQR with the augmented Lagrangian from the first run,
   * at most iterations; true if the constraints hold */
  bool SolveIlqr(int iterations);
  /*! \brief the cost of the inputs over a step */
  double InputCost(const double* u) const;
  /*! \brief the first run, along the middle at guess_speed */
  void Guess();
  /*! \brief the runs' derivatives at each station */
  void Linearize();
  /*! \brief the backward pass with the regularization rho; false if a
   * Hessian of the inputs is not positive */
  bool Backward(double rho, double* expected_1, double* expected_2);
  /*! \brief a run with the feedforward step alpha and the gains; its
   * augmented cost, the run in xs, us */
  double Forward(double alpha, std::vector<double>* xs, std::vector<double>* us,
                 std::vector<double>* cs, std::vector<double>* ts) const;
  /*! \brief the augmented cost of the run */
  double Cost(const std::vector<double>& xs, const std::vector<double>& us,
              const std::vector<double>& cs,
              const std::vector<double>& ts) const;
  double Violation(const std::vector<double>& cs) const;

  /*! \brief the curvature at the station's offset h (0, a half or a
   * whole step) from station k, from the samples */
  double CurvatureAt(int k, int half_steps) const {
    return curvatures_[2 * k + half_steps];
  }
  /*! \brief torque_limit, from its samples */
  double TorqueLimit(double omega) const;

  int n_ = 0; //!< steps
  std::vector<double> curvatures_; //!< at each half step
  std::vector<double> slopes_;     //!< slope's three at each half step
  std::vector<double> limits_;     //!< torque_limit every kLimitStep rad/s
  static constexpr double kLimitStep = 0.5;
  std::vector<double> states_, inputs_, times_, torques_;
  std::vector<double> constraints_; //!< n_ kConstraints, then the end's
  std::vector<double> step_times_;  //!< of each step
  /* the derivatives of each step: A (kStates^2), B (kStates kInputs), the
  time's gradient and Hessian's diagonal, and the constraints' Jacobian */
  std::vector<double> a_, b_, time_x_, time_u_, time_xx_, time_uu_, c_x_,
    c_u_, end_c_x_;
  std::vector<double> gain_k_, gain_kk_; //!< feedforward, feedback
  std::vector<double> multipliers_;
  double penalty_ = 1.0;
  double violation_ = INFINITY;
  int iterations_ = 0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
