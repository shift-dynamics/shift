#ifndef SHIFT_CONTROLLER_RALLY_DRIVER_H_
#define SHIFT_CONTROLLER_RALLY_DRIVER_H_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/controller/driver.h"
#include "shift/core/controller/slide_equilibrium.h"
#include "shift/core/force_module/powertrain.h"

namespace shift {
namespace core {
namespace controller {

/*!
@brief drives a vehicle along a path as a rally driver on a loose surface:
the car slid through the bends, its heading turned into the bend from its
direction of travel, about the steady slides of the car (SlideEquilibrium),
after K. Hindiyeh and J. C. Gerdes (A Controller Framework for Autonomous
Drifting: Design, Stability, and Experimental Validation, Journal of Dynamic
Systems, Measurement, and Control 136, 2014), J. Y. Goh, T. Goel and J. C.
Gerdes (Toward Automated Vehicle Control Beyond the Stability Limits:
Drifting Along a General Path, Journal of Dynamic Systems, Measurement, and
Control 142, 2020) and E. Velenis, E. Frazzoli and P. Tsiotras (2009)

The path, the brakes and the gear are those of the driver controller
(Driver). The car (mass, center_of_mass, center_of_mass_height, track,
front_load_transfer, front_roll_steer, rear_roll_steer, the differentials
and its tire, a tire force module of its wheels with an Iowa tire, taken as
the simulation has it) gives the steady slides.

*Sampling.* The driver decides at each output (UpdateOutputs) and holds its
outputs to the next; the steering and the throttle move from their last
outputs at no more than steering_rate and throttle_rate, then through
first-order lags of steering_lag and throttle_lag.

*Sideslip.* With \f$v_x, v_y\f$ the velocity of the vehicle's frame along its
x and y axes (y to the right, z down), on the middle of the rear axle, the
sideslip is \f$\beta = \arctan(v_y / v_x)\f$, the angle of its course \f$\chi\f$
from its heading \f$\psi\f$: \f$\beta = \chi - \psi\f$. In a bend of signed
curvature \f$\kappa\f$ (positive to the right; the mean of the path's over
curvature_time of travel, a little ahead of the frame), the target sideslip
is

\f[
\beta_r = -\mathrm{sign}(\kappa) \, \beta_s \,
\min\left(1, \frac{|\kappa|}{\kappa_s}\right),
\f]

with \f$\beta_s\f$ slide_angle and \f$\kappa_s\f$ slide_curvature: the heading
turned into the bend, past the course. The oversteer is
\f$o = -\mathrm{sign}(\kappa) \, \beta\f$, its target \f$o_r = |\beta_r|\f$.

*The table.* At Connect, the steady slide (SlideEquilibrium) on each
curvature of a table up to 4 \f$\kappa_s\f$ at its target sideslip gives the
speed \f$v_e\f$ and the steering \f$\delta_e\f$ of the slide; where the car
cannot slide so far on so tight a bend, or only with the road wheels steered
past their travel (max_steering_angle times steering_gain) or a front tire
using more than 1 - front_force_margin of its friction
(SlideEquilibrium::FrontUse: the rear tires saturated and the front ones
not, so that the steering holds the slide, as in the drift equilibria of
Hindiyeh and Gerdes, 2014), the sideslip is lowered to one it can, which becomes the target. The car is
slid in the bends of \f$|\kappa| \ge \kappa_s / 2\f$, and kept sliding down to
\f$\kappa_s / 3\f$ (with the regulator, on grip between slides the profile's throttle is taken
as a share of the drive torque \f$\mu m g r_w\f$ the tires can take, given
in the gear that needs least throttle for it, as in a slide; the course
\f$\psi + \beta\f$ rather than the heading is steered to the path, and the
target yaw rate is within \f$\mu g / v\f$), begun only within slide_speed_margin of the speed
profile and ended beyond twice that, the car braked for the bend first, and not with the
body rolled past roll_limit (its
y axis tilted from the horizontal), on a slope across the road, where the car
is straightened, its target sideslip 0, on grip; in the slide bends of the
path, \f$|\kappa| \ge
\kappa_s / 2\f$, the speed profile is that of the slide, \f$v_e \cos\beta_e\f$
along the heading; elsewhere it is the driver controller's.

*Heading.* With \f$\theta\f$ the direction of the path at the nearest point
and \f$e\f$ the lateral error (positive to the right of the path), the target
heading turns the course back to the path, as the Stanley steering law,
within \f$\pm\f$ max_heading_correction \f$\epsilon_m\f$, and past the course
by the target sideslip,

\f[
\psi_d = \theta - \beta_r - \mathrm{clamp}\left(\arctan\frac{k_e \, e}{v},
  \; -\epsilon_m, \; \epsilon_m\right),
\f]

with \f$v\f$ the speed and \f$k_e\f$ lateral_gain; the target yaw rate is
\f$r_d = v \, \kappa - k_\psi \, (\psi - \psi_d)\f$, with \f$k_\psi\f$
heading_gain.

*Steering in a slide.* About the steady slide, the steering turns the car
into the bend for a sideslip short of its target and out of it for one past,
and damps the yaw rate about its target, as the drift controller of Hindiyeh
and Gerdes:

\f[
\delta = \frac{1}{g} \left(\mathrm{sign}(\kappa) \, \delta_e
  + k_\beta \, (\beta - \beta_r) - k_d \, (r - r_d)\right),
\f]

within \f$\pm\f$ max_steering_angle, with \f$k_\beta\f$ sideslip_steering_gain,
\f$k_d\f$ yaw_damping_gain and \f$g\f$ steering_gain, the angle of the road
wheels per unit of the steering output.

*Throttle in a slide.* The drive holds the speed of the slide and turns the
car further for a sideslip short of its target,

\f[
u = \mathrm{clamp}\left(u_e + u_i + k_v \, (v_p - v_x) + k_u \, (o_r - o),
  \; 0, \; u_m\right), \qquad \dot{u}_i = k_i \, (v_p - v_x),
\f]

with \f$v_p\f$ the speed of the profile, \f$k_v\f$ speed_throttle_gain,
\f$k_u\f$ sideslip_throttle_gain and \f$u_m\f$ max_slide_throttle. With the
powertrain named (a force_module::Powertrain of the same assembly), \f$u_e\f$
is the throttle of the slide's drive torque, the wheels turning at
\f$(1 + \sigma_e) v_x / r\f$ with the slide's spin \f$\sigma_e\f$ and the
rolling radius \f$r\f$ of the tire's static load, in the gear of the outputs
(Powertrain::SteadyThrottle), and the integral \f$u_i\f$ starts from 0 as
the slide begins; without it, \f$u_e = 0\f$ and \f$u_i\f$ starts from
slide_throttle. The integral of speed_integral_gain \f$k_i\f$ is held within
\f$\pm u_m\f$ and integrated at the outputs (UpdateOutputs), each step of
the integrator's output.

*Spin.* With spin_throttle_gain \f$k_\sigma\f$ positive and the powertrain
named, the throttle holds the spin of the wheels instead of the speed: the
spin \f$\sigma = \omega_e r / (n_g v_x) - 1\f$ from the speed of the engine
\f$\omega_e\f$ of the last outputs and the overall ratio \f$n_g\f$ of the gear
(Powertrain::OverallRatio), its target \f$\sigma_r = \sigma_e + k_{\sigma\beta}
(o_r - o)\f$ more for a sideslip short of its target (spin_sideslip_gain
\f$k_{\sigma\beta}\f$), within 0 to 1, and

\f[
u = \mathrm{clamp}\left(u_e + k_\sigma \, (\sigma_r - \sigma)
  + k_v \, (v_p - v_x), \; 0, \; u_m\right).
\f]

*Regulator.* With regulator and the powertrain named, the car is held in a
slide instead by the linear quadratic regulator of its motion about the
steady slide (SlideEquilibrium::Linearize and Regulator, for the weights of
regulator_scales over the sample_time, with drive_inertia), scheduled over
the table, about a reference that goes from the motion and the inputs as the
slide begins to the steady slide's, \f$\dot{\underline{x}}_r = (\underline{x}_e
- \underline{x}_r) / T_r\f$ with reference_time \f$T_r\f$, so that the slide
is entered from where the car is: in a right bend's terms (\f$v_y\f$, \f$r\f$, \f$e\f$ and
\f$\Delta\chi\f$ of a left bend's sign changed), the error of the state
\f$\underline{z} = (v_x - v_{xr}, \; v_y - v_{yr}, \; r - r_r, \; \omega_c -
\omega_{cr}, \; e, \; \Delta\chi)\f$, with the carrier's speed \f$\omega_c\f$
from the engine's over the overall ratio of the gear, the lateral error
\f$e\f$ within \f$\pm\f$ max_lateral_error and the error of the course
\f$\chi = \psi + \beta\f$ from the path's direction, \f$\Delta\chi = \chi -
\theta\f$, within \f$\pm\f$ max_heading_correction, and the others within
twice their scales, gives
the steering of the road wheels \f$\delta_r - K_\delta \underline{z}\f$ and the
drive torque \f$\tau_r - K_\tau \underline{z}\f$, and the throttle of that
torque at the carrier's speed (Powertrain::SteadyThrottle), with no brake and
no pull of the handbrake: the regulator starts the slide. The steering is
held within max_steering_angle and steering_rate; with saturation_torque,
the drive torque is then that of least cost with the steering where it is
held (SlideEquilibrium::Regulator), the drive turning the car where the
steering cannot, and with saturation_governor the reference stays where it
is while the steering is held, the reference governor of E. Garone, S. Di
Cairano and I. Kolmanovsky (Reference and command governors for systems with
constraints, Automatica 75, 2017) with \f$\kappa = 0\f$. The gear is that
of
the most torque in hand at a fifth more than the slide's speed of the
wheels.

*Gear in a slide.* With the powertrain named, the gear is the one of the
least throttle \f$u_e\f$ for the slide's drive torque, the most in hand,
kept unless another needs less by gear_hysteresis as a slide begins, and
through it by twice that while it needs less than 0.95: a gear low enough for the torque and
high enough to keep the engine off its fuel cut as the wheels spin.
Elsewhere it is the driver controller's. The brake of the
profile is kept only where the car is faster than it by more than 2 m/s.

*Grip.* Outside the slide bends, the steering turns the heading through the
lateral force of the front tires: the rear tires carry their share of the
force on the arc, \f$f_r = m \, v \, r_d \, a / w\f$, with the center of mass
\f$a\f$ behind the front axle and \f$b\f$ ahead of the rear, and the front force
that brings the yaw rate to its target at the rate \f$k_r\f$ (yaw_rate_gain) is
\f$f_f = (b f_r - I_z k_r (r - r_d)) / a\f$, within \f$\pm 0.98 f_m\f$ of the
static load of the front axle, \f$f_m = \mu m g b / w\f$. Its slip angle, from
the tire's lateral force for the axle (both tires), steers the front wheels
from their velocity, \f$\delta = (\arctan((v_y + w r) / v_x) + \alpha) / g\f$.
The throttle is the speed profile's.

*Handbrake.* To start a slide, the handbrake is pulled, by handbrake_pull
for handbrake_time with the throttle lifted, when the oversteer falls short
of its target by more than handbrake_threshold, at most once each
handbrake_interval, decided at the outputs and held between them.

Slower than slide_speed, the steering is the driver controller's pure
pursuit.

JSON:
\code
{"type": "rally_driver", "name": "driver",
 "body_frame_pair": ["chassis", "rear_axle"], "wheelbase": 2.41,
 "path": [0, 0, 20, 0, 40, 5, 60, 15], "speed": 30,
 "lateral_acceleration": 6, "acceleration": 5, "deceleration": 7,
 "mass": 1413, "yaw_inertia": 2100, "center_of_mass": 1.29,
 "center_of_mass_height": 0.45, "track": 1.5, "front_load_transfer": 0.55,
 "front_roll_steer": 0.0023, "rear_roll_steer": 0.003,
 "steering_gain": 0.93, "front_differential": [80, 0.25, 2],
 "rear_differential": [80, 0.35, 2],
 "tire": "right_front_tire",
 "slide_angle": 0.25, "slide_curvature": 0.025, "slide_speed": 8,
 "shift_speeds": [17, 21, 25.5, 31, 37.5]}
\endcode
*/
class RallyDriver : public Driver {
public:
  RallyDriver()
    : Driver() {
    Initialize();
  }

  RallyDriver(std::string name)
    : Driver(name) {
    Initialize();
  }

  virtual ~RallyDriver() override {}

  static std::unique_ptr<RallyDriver> Create();

  static bool is_registered() {
    return registered_;
  }

  void DeclareParameters() override;
  void Connect() override;
  void UpdateOutputs(double t, const double* y) override;
  /*! \brief holds the outputs between the outputs: the driver decides at
   * each output, UpdateOutputs */
  void Service(const double* y, double* dy) override;

  /*! \brief the outputs for the pose of the vehicle's frame, the velocity of
   * its origin in the fixed frame, and its rate of yaw (rad/s, about its z
   * axis) */
  void DriveMotion(const ksl_SE3_t& pose, const ksl_vec3_t& velocity,
                   double yaw_rate) override;

  /*! \brief the speed of the slide in the slide bends, along the heading;
   * elsewhere that of the driver controller */
  double CornerSpeed(double curvature) const override;

  /*! \brief the slip angle (rad, of the sign of f) at which the front axle
   * carries the lateral force f (N), from the tire's lateral force; at most
   * that of the peak force */
  double FrontSlipAngle(double f) const;

  /*! \brief the soil of the terrain under the start of the path, if any */
  const force_module::SoilProperties* StartSoil() const;

  /*! \brief the steady slide of the table on the curvature (either sign,
   * for a right bend), interpolated: its sideslip (negative), speed and
   * steering */
  void Slide(double curvature, double* sideslip, double* speed,
             double* steering, double* spin = nullptr,
             double* torque = nullptr, double* gain = nullptr,
             double* coupling = nullptr) const;

  /*! \brief the steady slide of slide_speed_table on the curvature at the
   * speed (m/s), interpolated on the speed within each bend's slides up to
   * its fastest, as Slide; Slide's where there is no such table */
  void SlideAtSpeed(double curvature, double speed, double* sideslip,
                    double* steering, double* spin, double* torque,
                    double* gain, double* coupling) const;

  /*! \brief the steady slide of the columns on the curvature at the
   * sideslip (rad, a right bend's, below zero), interpolated on it within
   * each bend's slides up to its fastest, with its speed (m/s); Slide's
   * where there are no columns */
  void SlideAtSideslip(double curvature, double sideslip, double* speed,
                       double* steering, double* spin, double* torque,
                       double* gain, double* coupling) const;

  double get_handbrake() const {
    return handbrake_;
  }
  double get_sideslip() const {
    return sideslip_;
  }
  double get_sideslip_target() const {
    return sideslip_target_;
  }
  double get_yaw_rate() const {
    return yaw_rate_;
  }
  double get_yaw_rate_target() const {
    return yaw_rate_target_;
  }

private:
  void Initialize() {
    type_ = "rally_driver";
    DeclareRallyParameters();
  }
  /*! \brief the parameters and outputs of the rally driver, beyond those of
   * the driver controller */
  void DeclareRallyParameters();
  /*! \brief the greatest lateral force of the front axle (N) */
  double FrontFriction() const;
  /*! \brief the held gear, or the next one up or down if it has a share
   * gear_hysteresis more torque at the wheels at full throttle, at the
   * wheels' planned speed (wheel_speed, rad/s: the car's with the planned
   * spin) capped at the output shaft's; 1 with none held */
  int SlideGear(double wheel_speed) const;
  /*! \brief the steady slides on the curvatures of the table */
  void BuildTable();
  /*! \brief the planar car of the steady slides and of the run of least
   * time (SlideEquilibrium), from the parameters and the tire */
  SlideEquilibrium PlanarCar() const;
  /*! \brief the run of least time from the car's station to
   * minimum_time_end (MinimumTime), written to minimum_time_file, if any,
   * and kept with its tracking gains for minimum_time_track */
  void PlanMinimumTime();
  /*! \brief the height (z, down) of the active terrain's surface at x, y;
   * false if none has a surface there */
  bool GroundHeight(double x, double y, double* z) const;
  /*! \brief the outputs that follow the plan, by its time-varying
   * regulator, at the frame's velocity (vx, vy) and yaw rate and its
   * course's error from the path; false off the plan's stations */
  bool TrackPlan(double vx, double vy, double yaw_rate, double course_error);
  /*! \brief whether the curvature is that of a slide bend */
  bool IsSlideBend(double curvature) const {
    return slide_angle_ > 0.0 &&
           std::fabs(curvature) >= 0.5 * slide_curvature_;
  }

  static bool registered_;

  static constexpr double kGravity = 9.81;

  /* the vehicle */
  double mass_ = 0.0;
  double yaw_inertia_ = 0.0;
  double center_of_mass_ = 0.0; //!< ahead of the frame (m)
  double center_of_mass_height_ = 0.0;
  double track_ = 0.0;
  double front_load_transfer_ = 0.5;
  double front_roll_steer_ = 0.0;
  double rear_roll_steer_ = 0.0;
  double steering_gain_ = 1.0;
  /* bias torque, bias ratio and slip speed; a slip speed of 0 is open */
  std::vector<double> front_differential_ = {0.0, 0.0, 0.0};
  std::vector<double> rear_differential_ = {0.0, 0.0, 0.0};
  std::string tire_name_;         //!< the tire force module
  force_module::tire::Iowa tire_; //!< a copy of its Iowa tire

  /* the slide */
  double slide_angle_ = 0.25;
  double slide_curvature_ = 0.025;
  double slide_speed_ = 8.0;
  double heading_gain_ = 2.0;
  double lateral_gain_ = 1.0;
  double max_heading_correction_ = 0.35;
  double yaw_rate_gain_ = 5.0;
  double sideslip_steering_gain_ = 1.0;
  double yaw_damping_gain_ = 0.1;
  double speed_throttle_gain_ = 0.1;
  double speed_integral_gain_ = 0.2;
  double sideslip_throttle_gain_ = 1.0;
  double spin_throttle_gain_ = 0.0;
  double spin_sideslip_gain_ = 1.0;
  double gear_hysteresis_ = 0.2;
  double shift_interval_ = 0.3;
  double last_shift_ = -INFINITY; //!< time of the last shift (s)
  double steering_rate_ = 1.0;
  double throttle_rate_ = 4.0;
  double steering_lag_ = 0.0;
  double brake_torque_ = 0.0; //!< of the brakes at full pedal (N m)
  std::string minimum_time_file_; //!< the run of least time's CSV, if any
  double minimum_time_end_ = 0.0;  //!< its last station (m); 0: the path's
  double minimum_time_step_ = 0.5; //!< between its stations (m)
  double minimum_time_half_width_ = 3.4; //!< of the road (m)
  /*! the car's outline about the frame: rear, front, half width (m) */
  std::vector<double> minimum_time_outline_ = {0.0, 0.0, 0.0};
  double minimum_time_steering_rate_ = 2.0; //!< of the road wheels (rad/s)
  double minimum_time_start_speed_ = 3.0;   //!< its least speed at start
  int minimum_time_iterations_ = 400;
  bool minimum_time_track_ = false; //!< follow the run of least time
  std::string minimum_time_plan_; //!< a run of least time to load, not solve
  bool minimum_time_slope_ = true; //!< the road's grade and camber in the plan
  /*! the tracking regulator's scales: v_x, v_y (m/s), r (rad/s), e (m), the
   * course, the steering (rad), the steering's change (rad/m) and the
   * carriers' speed (rad/s) */
  std::vector<double> minimum_time_track_scales_ = {1.0, 0.5, 0.3, 0.3,
                                                   0.1, 0.2, 0.05, 1.0};
  double minimum_time_wheel_gain_ = 600.0; //!< N m per rad/s of the wheels
  /* the plan: its first station and step, its states (MinimumTime), drive
  torques and tracking gains at each station */
  double plan_start_ = 0.0, plan_step_ = 0.0;
  std::vector<double> plan_states_, plan_torques_, plan_gains_;
  bool slide_fastest_ = false; //!< each bend's fastest slide
  bool slide_speed_table_ = false; //!< each bend's slides by speed
  double slide_speed_share_ = 0.995; //!< of the fastest slide's speed
  double slide_entry_share_ = 0.0;   //!< of the slide's speed to begin it
  double traction_spin_ = 0.0;     //!< greatest wheelspin on grip
  double slide_entry_rate_ = 0.0;  //!< of the reference's sideslip (rad/s)
  double grip_sideslip_gain_ = 0.0; //!< sideslip's return on grip (1/s)
  double slide_exit_margin_ = 0.1; //!< below slide_entry_share to leave
  double entry_angle_ = 0.0; //!< the reference's sideslip into the bend
  double throttle_lag_ = 0.0;
  double curvature_time_ = 0.5;
  double roll_limit_ = 0.12;
  double slide_speed_margin_ = 3.0;
  double reference_time_ = 0.5;
  bool speed_scheduled_ = true;  //!< the slide at the car's speed, the
                                 //!< speed held by the torque alone
  /*! the regulator's reference: v_x, v_y, r, omega_c, the steering of the
   * road wheels and the drive torque */
  double reference_[6] = {0, 0, 0, 0, 0, 0};
  double reference_at_ = 0.0;  //!< time of the reference (s)
  double last_steering_ = 0.0; //!< the steering of the last outputs
  bool regulator_ = false;
  double front_force_margin_ = 0.02;
  bool saturation_torque_ = true;
  bool saturation_governor_ = true;
  bool steering_saturated_ = false; //!< the regulator's, at the last outputs
  double output_step_ = 0.0;        //!< time since the last outputs (s)
  bool rate_limited_ = false;       //!< whether the steering's rate applies
  double drive_inertia_ = 60.0;
  std::vector<double> regulator_scales_ = {2.0, 0.7, 0.1, 5.0,
                                          1.0, 0.1, 0.1, 500.0};
  double max_lateral_error_ = 2.0;
  /*! the rates' scales; both 0: no cost on the rates */
  std::vector<double> regulator_rate_scales_ = {0.0, 0.0};
  bool RateRegulated() const {
    return regulator_rate_scales_[0] > 0.0 || regulator_rate_scales_[1] > 0.0;
  }
  double regulator_torque_ = 0.0; //!< the regulator's last drive torque
  /*! the gains of the regulator of a slide, kInputs x (kPathStates +
  kInputs): of the path's states, then of the last steering and torque
  (zero without regulator_rate_scales) */
  static constexpr int kGains = 16;
  double sample_time_ = 0.05;
  double slide_throttle_ = 0.4;
  double max_slide_throttle_ = 1.0;
  double handbrake_threshold_ = 0.15;
  double handbrake_pull_ = 1.0;
  double handbrake_time_ = 0.4;
  double handbrake_interval_ = 2.0;

  /* the table of steady slides, on the curvatures of a right bend */
  std::vector<double> table_curvature_, table_sideslip_, table_speed_,
    table_steering_, table_spin_, table_torque_;
  std::vector<double> table_gain_; //!< K of each, kGains
  /*! of each, H_tau,delta / H_tau,tau of the regulator */
  std::vector<double> table_coupling_;
  /*! with slide_speed_table: each bend's first slide in the columns (and
  one past the last bend's last), and the slides, each of a greater speed
  than the last of its bend, to the bend's fastest */
  std::vector<size_t> table_column_;
  std::vector<double> column_speed_, column_sideslip_, column_steering_,
    column_spin_, column_torque_, column_coupling_, column_gain_;
  std::vector<double> column_angle_; //!< - column_sideslip_, rising
  /*! \brief the columns' slide on the curvature at the value of key
   * (column_speed_ or column_angle_), with its speed, as SlideAtSpeed */
  void SlideInColumns(double curvature, const std::vector<double>& key,
                      double value, double* speed, double* sideslip,
                      double* steering, double* spin, double* torque,
                      double* gain, double* coupling) const;
  std::string powertrain_name_; //!< of the powertrain, if any
  const force_module::Powertrain* powertrain_ = nullptr;

  double time_ = 0.0;             //!< time of the last outputs (s)
  bool wants_pull_ = false;       //!< whether a slide is to be started
  double pull_until_ = -INFINITY; //!< end of the pull of the handbrake (s)
  double next_pull_ = -INFINITY;  //!< earliest time of the next pull (s)
  bool sliding_ = false;          //!< in a slide bend at the last outputs
  bool in_slide_ = false;         //!< in a slide bend at the last evaluation
  bool started_ = false;          //!< outputs given since Connect
  double integral_ = 0.0;         //!< the throttle's integral, u_i
  double speed_error_ = 0.0;      //!< v_p - v_x at the last evaluation
  double held_gear_ = 0.0;        //!< the gear of the last outputs

  double handbrake_ = 0.0;
  double sideslip_ = 0.0;
  double sideslip_target_ = 0.0;
  double yaw_rate_ = 0.0;
  double yaw_rate_target_ = 0.0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
