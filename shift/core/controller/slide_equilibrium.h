#ifndef SHIFT_CONTROLLER_SLIDE_EQUILIBRIUM_H_
#define SHIFT_CONTROLLER_SLIDE_EQUILIBRIUM_H_

#include <vector>

#include "shift/core/force_module/tire/iowa.h"

namespace shift {
namespace core {
namespace controller {

/*!
@brief the steady slides of a four-wheel drive car with no centre
differential: the speed, the steering and the spin of the wheels that hold it
on a circle at a sideslip, after the steady-state cornering equilibria of E.
Velenis, E. Frazzoli and P. Tsiotras (On Steady-State Cornering Equilibria
for Wheeled Vehicles with Drift, Proceedings of the 48th IEEE Conference on
Decision and Control, 2009) and K. Hindiyeh and J. C. Gerdes (2014)

The car is a rigid body in the plane on four wheels, its frame on the middle
of the rear axle (x forward, y to the right, z down), the front axle the
wheelbase \f$w\f$ ahead, the wheels at \f$y = \mp t/2\f$ (track), the center of
mass \f$c\f$ ahead of the frame and \f$h\f$ above the ground, mass \f$m\f$. On a
circle at the steady yaw rate \f$r\f$, with the frame at speed \f$v\f$ and
sideslip \f$\beta\f$, its velocity is \f$(v_x, v_y) = v (\cos\beta, \sin\beta)\f$,
a wheel at \f$(x_i, y_i)\f$ moves at \f$(v_x - r y_i, \; v_y + r x_i)\f$, and the
center of mass accelerates at \f$\underline{a} = (-r (v_y + c r), \; r v_x)\f$.
The frame runs on the circle of radius \f$\rho = v / r\f$.

*Loads.* The axles carry the weight and its transfer by the longitudinal
acceleration, and each axle's outer wheel the share \f$\lambda\f$ (front) or
\f$1 - \lambda\f$ (rear) of the lateral transfer (front_load_transfer, from
the roll stiffnesses):

\f[
f_{zf} = m \, \frac{g c - a_x h}{w}, \quad
f_{zr} = m \, \frac{g (w - c) + a_x h}{w}, \quad
f_{z,left/right} = \frac{f_{z,axle}}{2} \pm \lambda_{axle} \,
\frac{m a_y h}{t}.
\f]

*Wheels.* The front wheels are steered by \f$\delta - k_f a_y\f$ and the
rear by \f$-k_r a_y\f$: the steering, and the steer of the suspension as the
body rolls and the bushings give (front_roll_steer \f$k_f\f$ and
rear_roll_steer \f$k_r\f$, rad per m/s\f$^2\f$, positive out of the turn). With no centre
differential the carriers of both axles turn at one speed, their treads at
\f$u = (1 + \sigma) v_x\f$, the mean of each axle's two treads, which differ by
\f$d \, v_x\f$ (left less right). In each wheel's heading the hub moves at
\f$(v_{wx}, v_{wy})\f$, the slip ratio is
\f$\kappa = (u_i - v_{wx}) / \max(u_i, v_{wx})\f$ and the slip angle
\f$\alpha = \arctan(v_{wy} / v_{wx})\f$, and the tire gives its forces and
aligning moment as it does in the simulation (tire::Iowa::TreadForces, with
or without combined_slip). The rolling resistance acts at the hub along the
heading. The torque of a wheel is \f$\tau_i = f_{x,i} \, r_i\f$, at the
rolling radius \f$r_i\f$ of its steady deflection, or its sinkage on soft
soil, which also gives the resistance (tire::Iowa::RestingDeflection).

*Differentials.* Each axle's limited slip differential, as
force_module::Powertrain, passes the torque
\f$(b + \beta_d |\tau_l + \tau_r|) \tanh((\omega_l - \omega_r) / \omega_s)\f$
from its faster wheel to the slower (bias_torque \f$b\f$, bias_ratio
\f$\beta_d\f$, slip_speed \f$\omega_s\f$ of the axle):

\f[
\tau_l - \tau_r = -2 (b + \beta_d |\tau_l + \tau_r|)
\tanh\frac{\omega_l - \omega_r}{\omega_s}.
\f]

*Equilibrium.* For a radius \f$\rho\f$ and a sideslip \f$\beta\f$, the speed,
the steering, the spin \f$\sigma\f$ and the two differences \f$d_f, d_r\f$ solve
the balance of the forces and of the yaw moment about the center of mass,

\f[
m a_x = \sum f_x, \qquad m a_y = \sum f_y, \qquad
0 = \sum (x_i - c) f_{y,i} - y_i f_{x,i} - t_a f'_{y,i},
\f]

with the forces in the frame, the aligning moments of the tires' lateral
forces \f$f'_{y,i}\f$ in their headings (aligning_coefficient \f$t_a\f$), and
the two differentials, by Newton's method
from a guess. The drive torque is the sum of the wheels' torques.

*Motion about a slide.* With the state \f$\underline{s} = (v_x, v_y, r,
\omega_c)\f$, the carrier at \f$\omega_c\f$, and the inputs the steering
\f$\delta\f$ and the drive torque \f$\tau\f$ on the wheels, the car moves as

\f[
\dot v_x = \frac{\sum f_x}{m} + r (v_y + c r), \quad
\dot v_y = \frac{\sum f_y}{m} - r v_x - c \dot r, \quad
\dot r = \frac{m_z}{I_z}, \quad
\dot\omega_c = \frac{\tau - \sum \tau_i}{J},
\f]

with the loads of the accelerations of the center of mass, \f$\sum f_x /
m\f$ and \f$\sum f_y / m\f$, found from those of the steady motion, \f$a_x =
-r (v_y + c r)\f$ and \f$a_y = r v_x\f$, by four passes, each axle's wheels at the difference of speeds that balances
its differential, \f$I_z\f$ yaw_inertia and \f$J\f$ drive_inertia, the
inertia of the drive (engine, transmission, shafts and wheels) at the
wheels. Linearize gives \f$\dot{\underline{s}} \approx A \, \delta
\underline{s} + B \, \delta \underline{u}\f$ about a steady slide, by central
differences, with the path's two states, the lateral error \f$e\f$ and the
course's error \f$\Delta\chi\f$ from the path's direction, \f$\dot e = v
\sin\Delta\chi\f$ and \f$\Delta\dot\chi = r + \dot\beta - v \kappa\f$ (after J. Y.
Goh, T. Goel and J. C. Gerdes, 2020), and Regulator the gain \f$K\f$ of the linear quadratic regulator
\f$\delta \underline{u} = -K \delta \underline{s}\f$ for the weights \f$Q\f$ and
\f$R\f$, of the system held over the sample time \f$\Delta t\f$: the discrete
algebraic Riccati equation

\f[
P = Q \Delta t + A_d^T P A_d - A_d^T P B_d (R \Delta t + B_d^T P B_d)^{-1}
B_d^T P A_d, \qquad K = (R \Delta t + B_d^T P B_d)^{-1} B_d^T P A_d,
\f]

by the structure-preserving doubling algorithm (E. K.-W. Chu, H.-Y. Fan and
W.-W. Lin, A structure-preserving doubling algorithm for continuous-time
algebraic Riccati equations, Linear Algebra and its Applications 396, 2005),
with \f$A_d = e^{A \Delta t}\f$ and \f$B_d = \int_0^{\Delta t}
e^{A t} dt \, B\f$ (B. D. O. Anderson and J. B. Moore, Optimal Control:
Linear Quadratic Methods, Prentice Hall, 1990). With the steering held at
\f$\delta_a\f$ (at its limit), the torque of least cost is that of the
same cost with the steering fixed,

\f[
\delta u_\tau = -K_\tau \delta \underline{s} +
\frac{H_{\tau\delta}}{H_{\tau\tau}} (-K_\delta \delta \underline{s} -
\delta u_{\delta a}), \qquad H = R \Delta t + B_d^T P B_d,
\f]

the solution of the one-step problem with the cost to go \f$P\f$ and the
steering at its bound (F. Borrelli, A. Bemporad and M. Morari, Predictive
Control for Linear and Hybrid Systems, Cambridge University Press, 2017).
*/
class SlideEquilibrium {
public:
  static constexpr int kUnknowns = 5;
  static constexpr int kStates = 4; //!< v_x, v_y, r, omega_c
  static constexpr int kInputs = 2; //!< steering, drive torque
  /*! v_x, v_y, r, omega_c, the lateral error and the course's error */
  static constexpr int kPathStates = 6;

  struct Axle {
    double bias_torque = 0.0; //!< of the limited slip differential (N m)
    double bias_ratio = 0.0;
    double slip_speed = 2.0; //!< (rad/s); 0: an open differential
  };

  struct State {
    double speed = 0.0;    //!< of the frame (m/s)
    double steering = 0.0; //!< of the front wheels (rad)
    double spin = 0.0;     //!< of the treads beyond the rear's rolling, sigma
    double front_difference = 0.0; //!< d_f
    double rear_difference = 0.0;  //!< d_r
    double yaw_rate = 0.0;         //!< (rad/s)
    double drive_torque = 0.0;     //!< sum of the wheels' torques (N m)
    double drive_force = 0.0;      //!< sum of the tires' longitudinal forces
    double front_force = 0.0;  //!< lateral force of the front tires, wheels
    double rear_force = 0.0;   //!< lateral force of the rear tires
    double slip[4] = {0, 0, 0, 0};       //!< kappa: lf, rf, lr, rr
    double slip_angle[4] = {0, 0, 0, 0}; //!< alpha
    double load[4] = {0, 0, 0, 0};
    double force_x[4] = {0, 0, 0, 0}; //!< of the treads, in the wheels' axes
    double force_y[4] = {0, 0, 0, 0};
    double residual = 0.0; //!< of the equations, over m g
  };

  double mass = 0.0;
  double wheelbase = 0.0;
  double track = 0.0;
  double center_of_mass = 0.0;      //!< ahead of the rear axle (m)
  double height = 0.0;              //!< of the center of mass (m)
  double front_load_transfer = 0.5; //!< share of the lateral transfer
  double front_roll_steer = 0.0;    //!< out of the turn, rad per m/s^2
  double rear_roll_steer = 0.0;
  double gravity = 9.81;
  double yaw_inertia = 0.0;   //!< I_z, about the center of mass (kg m^2)
  double drive_inertia = 0.0; //!< J, at the wheels (kg m^2)
  Axle front, rear;
  force_module::tire::Iowa tire; //!< each of the four
  /*! the slip ratio's denominator, the greater of the tread's speed and the
   * hub's, made smooth over this much (m/s), \f$\frac{1}{2}(|u| + |v| +
   * \sqrt{(|u| - |v|)^2 + \epsilon^2})\f$, for the derivatives of a plan
   * (MinimumTime); 0: the tire's */
  double slip_smoothing = 0.0;

  /*! \brief the residuals of the equations, over m g, and the state, for
   * the radius rho (m, positive to the right) and the sideslip beta at the
   * unknowns x: speed, steering, spin and the two differences */
  void Residuals(double rho, double beta, const double* x, double* e,
                 State* state) const;

  /*! \brief solves for the steady slide on the radius rho at the sideslip
   * beta, from the guess in state; true if it converged, with the state */
  bool Solve(double rho, double beta, State* state) const;

  /*! \brief the steady slide on the radius rho at the sideslip beta, from
   * guesses; true if one converged */
  bool Find(double rho, double beta, State* state) const;

  /*! \brief the rates ds of the state s (v_x, v_y, r, omega_c) for the
   * steering and the drive torque; differences, if given, holds the
   * differences of the axles' wheel speeds (rad/s), the guess in and the
   * balance out; slope as PathDerivatives' */
  void Derivatives(const double* s, double delta, double torque, double* ds,
                   double* differences = nullptr,
                   const double* slope = nullptr) const;

  /*! \brief the rates of the state with the path's (kPathStates): the
   * lateral error from the path, positive to the right, and the course's
   * error from the path's direction, on a path of the curvature (1/m,
   * positive to the right); slope, if given, is gravity's acceleration in
   * the car's frame (m/s^2) on a sloping road: along x and y, in the
   * road's plane, and normal to it, which the tires' loads carry in place of
   * gravity */
  void PathDerivatives(const double* s, double delta, double torque,
                       double curvature, double* ds,
                       double* differences = nullptr,
                       const double* slope = nullptr) const;

  /*! \brief the greater share of its friction that a front tire uses in
   * the state, \f$\sqrt{f_x^2 + f_y^2} / (\mu(f_z) f_z)\f$: below one, the
   * front tires have force in hand, and the steering turns the car (the
   * drift equilibria of R. Y. Hindiyeh and J. C. Gerdes, A controller
   * framework for autonomous drifting, J. Dyn. Sys. Meas. Control 136,
   * 2014, have the rear tires saturated and the front ones not) */
  double FrontUse(const State& state) const;

  /*! \brief A (kPathStates x kPathStates, row major) and B (kPathStates x
   * kInputs) of the motion, with the path's, about the steady slide at the
   * sideslip beta, on the slide's circle */
  void Linearize(double beta, const State& state, double* a, double* b) const;

  /*! \brief the gain k (kInputs x kPathStates, row major) of the regulator
   * for a, b, the weights q (kPathStates x kPathStates) and r (kInputs x
   * kInputs) and the sample time dt, and, if coupling is given, the
   * coupling \f$H_{\tau\delta} / H_{\tau\tau}\f$ of the torque to the
   * steering in \f$H = R \Delta t + B_d^T P B_d\f$; false if the iteration
   * fails */
  static bool Regulator(const double* a, const double* b, const double* q,
                        const double* r, double dt, double* k,
                        double* coupling = nullptr);

  /*! \brief the regulator with a cost on the rates of the inputs: the
   * state with the last inputs, \f$\underline{x}' = [\underline{x};
   * \underline{u}_-]\f$, the input their change over a sample,
   * \f$\Delta \underline{u}\f$, weighed by s (kInputs x kInputs) as the
   * rate \f$\Delta \underline{u} / \Delta t\f$ (B. D. O. Anderson and
   * J. B. Moore, Optimal Control: Linear Quadratic Methods, 1990, the
   * regulator of the input's increments), and the inputs by r as the last
   * ones; k (kInputs x (kPathStates + kInputs), row major) gives
   * \f$\Delta \underline{u} = -K \underline{x}'\f$, and coupling that
   * of H of the changes; false if the iteration fails */
  static bool RateRegulator(const double* a, const double* b, const double* q,
                            const double* r, const double* s, double dt,
                            double* k, double* coupling = nullptr);

  /*! \brief A_d, B_d of a, b held over dt (kPathStates, kInputs) */
  static void Discretize(const double* a, const double* b, double dt,
                         double* ad, double* bd);

  /*! \brief the gain k (m x n) of the discrete regulator of ad (n x n), bd
   * (n x m) with the costs of a sample qd (n x n) and rd (m x m), and the
   * coupling H_10 / H_11 of H = R_d + B_d' P B_d; false if it fails */
  static bool DiscreteRegulator(int n, int m, const double* ad,
                                const double* bd, const double* qd,
                                const double* rd, double* k,
                                double* coupling = nullptr);

  /*! \brief samples the tire's resting deflection and rolling resistance
   * (tire::Iowa::RestingDeflection) every step N of load to most, which
   * Wheels then interpolates linearly instead of solving the tire's sinkage
   * on soft soil at each load; most 0 clears the samples */
  void TabulateDeflection(double most, double step);

  /*! \brief solves a x = x in place, a (n x n, row major, overwritten) by
   * Gaussian elimination with partial pivoting; false if singular */
  static bool SolveLinear(int n, double* a, double* x);

private:
  /*! \brief the sums of the forces of the wheels, and each one's torque and
   * speed */
  struct Totals {
    double fx = 0.0, fy = 0.0, moment = 0.0; //!< in the frame, about the CoM
    double drive_force = 0.0, front_force = 0.0, rear_force = 0.0;
    double torque[4] = {0, 0, 0, 0};
    double omega[4] = {0, 0, 0, 0};
  };

  /*! \brief the forces of the wheels at the motion (v_x, v_y, r), the
   * accelerations of the loads, the carrier's speed, the axles' differences
   * of speed and the steering */
  void Wheels(double vx, double vy, double r, double ax, double ay,
              double carrier, const double* difference, double delta,
              Totals* totals, State* state, double normal = -1.0) const;

  /*! \brief the forces of the wheels, as Wheels, with the differences d of
   * the axles' wheel speeds that balance their differentials, from the
   * guess in d */
  void Balance(double vx, double vy, double r, double ax, double ay,
               double carrier, double delta, double* d, Totals* totals,
               double normal = -1.0) const;

  /*! \brief the tire's resting deflection and rolling resistance at the
   * load, from the samples of TabulateDeflection if any */
  void Deflection(double load, double* deflection, double* resistance) const;

  std::vector<double> deflections_, resistances_; //!< TabulateDeflection's
  double deflection_step_ = 0.0;

  /*! \brief the residual of an axle's differential (0 front, 1 rear), over
   * m g times the radius */
  double Differential(const Totals& totals, int axle) const;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
