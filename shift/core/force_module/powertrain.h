#ifndef SHIFT_CORE_FORCE_MODULE_POWERTRAIN_H_
#define SHIFT_CORE_FORCE_MODULE_POWERTRAIN_H_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/table.h"
#include "shift/core/parameter/array.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace force_module {

/*! \brief the engine of a powertrain, see Powertrain */
class PowertrainEngine : public parameter::List {
public:
  PowertrainEngine() {
    DeclareParameters();
  }
  virtual ~PowertrainEngine() {}

  double inertia = 1.0;
  double idle_speed = 0.0;
  double high_idle_speed = 0.0;
  double governor_gain = 0.1;
  double cut_range = -1.0;
  double throttle = 0.0;
  std::vector<double> torque_curve;
  std::vector<double> friction_curve;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief the coupling of the engine to the transmission: a torque converter
 * with a lockup clutch, or a clutch; see Powertrain */
class PowertrainCoupling : public parameter::List {
public:
  PowertrainCoupling() {
    DeclareParameters();
  }
  virtual ~PowertrainCoupling() {}

  std::string type = "converter";
  std::vector<double> capacity_curve;
  std::vector<double> torque_ratio_curve;
  double capacity = 0.0;
  double engagement = 1.0;
  double slip_speed = 0.5;
  /* the bristle model of friction (see Powertrain); off if 0 */
  double bristle_stiffness = 0.0;
  double bristle_damping = 0.0;
  double static_ratio = 1.0;
  double stribeck_speed = 1.0;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief the transmission of a powertrain, see Powertrain */
class PowertrainTransmission : public parameter::List {
public:
  PowertrainTransmission() {
    DeclareParameters();
  }
  virtual ~PowertrainTransmission() {}

  std::vector<double> ratios;
  double reverse_ratio = 0.0;
  double gear = 0.0;
  double shift_time = 0.3;
  double clutch_capacity = 0.0;
  double slip_speed = 0.5;
  double input_inertia = 0.5;
  double output_inertia = 0.5;
  double spin_loss = 0.0;
  /* the bristle model of friction (see Powertrain); off if 0 */
  double bristle_stiffness = 0.0;
  double bristle_damping = 0.0;
  double static_ratio = 1.0;
  double stribeck_speed = 1.0;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief an axle of a powertrain: its driveshaft, final drive, differential
 * and two wheels; see Powertrain */
class PowertrainAxle : public parameter::List {
public:
  PowertrainAxle() {
    DeclareParameters();
  }
  virtual ~PowertrainAxle() {}

  static const std::string get_array_key() {
    return "axles";
  }

  std::string name;
  std::string left, right;
  double direction = 1.0;
  double final_drive = 1.0;
  double transfer_ratio = 1.0;
  double shaft_stiffness = 0.0;
  double shaft_damping = 0.0;
  double engaged = 1.0;
  std::string differential = "open";
  double bias_torque = 0.0;
  double bias_ratio = 0.0;
  double lock_damping = 1e3;
  double slip_speed = 0.5;
  /* the bristle model of friction (see Powertrain); off if 0 */
  double bristle_stiffness = 0.0;
  double bristle_damping = 0.0;
  double static_ratio = 1.0;
  double stribeck_speed = 1.0;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief a brake of a powertrain, on a joint; see Powertrain */
class PowertrainBrake : public parameter::List {
public:
  PowertrainBrake() {
    DeclareParameters();
  }
  virtual ~PowertrainBrake() {}

  static const std::string get_array_key() {
    return "brakes";
  }

  std::string name;
  std::string joint;
  double direction = 1.0;
  double capacity = 0.0;
  double pressure = 0.0;
  std::string applied_by = "pressure";
  double slip_speed = 0.1;
  /* the bristle model of friction (see Powertrain); off if 0 */
  double bristle_stiffness = 0.0;
  double bristle_damping = 0.0;
  double static_ratio = 1.0;
  double stribeck_speed = 1.0;

protected:
  virtual void DeclareParameters() override;
};

/*!
@brief the drive of a vehicle from its engine to its wheels: engine, torque
converter or clutch, transmission with gear shifts, driveshafts, final drives
and differentials, and brakes

The powertrain is a chain of rotating inertias joined by torque elements. Its
states are the speed of the engine, the speed of the transmission's input and
that of its output, the twist of the driveshaft of each axle, and the
engagement of each gear's clutch. The wheels are the joints of the multibody
model (revolute joints, whose speeds the module reads and to which it applies
the torques of the differentials), so the module couples to a vehicle's wheels,
tires and chassis as the tire models do.

*The engine.* The torque of the engine at the speed \f$\omega\f$ is the net torque at full
fuel, \f$\tau_{full}(\omega)\f$ (torque_curve, a table of \f$[\omega, \tau]\f$ pairs),
blended with the torque of the closed throttle (the friction and pumping of the
engine, friction_curve, negative) by the fuel fraction \f$f\f$:

\f[
\tau_e = f \, \tau_{full}(\omega) + (1 - f) \, \tau_{friction}(\omega).
\f]

The fuel fraction is the throttle (0 to 1; an input that can be set as the
model runs) or what the governor asks for, whichever is more, and is cut off
above the high idle speed:

\f[
f = \max\left(\mathrm{throttle}, \; k_g (\omega_{idle} - \omega)\right) \chi(\omega),
\f]

where the governor (gain \f$k_g\f$, the governor_gain) opens the throttle as the
engine slows below idle_speed (so the engine holds near idle with some droop)
and \f$\chi\f$ falls smoothly from 1 to 0 over cut_range below high_idle_speed.
\f$j_e \dot{\omega}_e = \tau_e - \tau_{coupling}\f$, with \f$\tau_{coupling}\f$ the torque of
the coupling.

*The coupling.* A torque converter (type "converter") joins the engine, as pump,
to the transmission's input, as turbine, through the oil. With the speed ratio
\f$s_r = \omega_t / \omega_e\f$ of turbine and pump, the capacity curve
\f$k(s_r)\f$ (a table of \f$[s_r, k]\f$ pairs, \f$k\f$ in rad/s per square root
of N m) gives the torque on the pump and the torque ratio curve
\f$\mathrm{TR}(s_r)\f$ the torque on the turbine:

\f[
\tau_p = \frac{\omega_e |\omega_e|}{k(s_r)^2}, \quad \tau_t = \mathrm{TR}(s_r) \, \tau_p,
\f]

\f$\mathrm{TR}\f$ is above 1 when the turbine is slow (torque multiplication at stall)
and falls to 1 at the coupling point. The efficiency \f$\mathrm{TR}(s_r) \, s_r\f$ must
not exceed 1. A lockup clutch in parallel, of capacity "capacity" and engaged
by "engagement" (0 to 1, an input that can be set as the model runs), adds a
torque of capacity \f$\cdot\f$ engagement \f$\cdot
\tanh\left((\omega_e - \omega_t) / \omega_0\right)\f$, with \f$\omega_0\f$ the
slip_speed, between the two, locking them together when engaged. A clutch (type "clutch") has only this second part, and no oil
coupling, as a manual transmission's starting clutch.

*The transmission.* Each forward gear \f$i\f$, of ratio \f$r_i\f$ (input speed over output
speed), has a clutch between the input shaft, of inertia input_inertia, and the
output shaft, of inertia output_inertia, which transmits

\f[
\tau_i = e_i \, c_{max} \tanh\left(\frac{\omega_t - r_i \omega_o}{\omega_0}\right),
\f]

with the capacity \f$c_{max}\f$ (clutch_capacity), the engagement \f$e_i\f$ of the
gear's clutch and the speeds \f$\omega_t\f$ and \f$\omega_o\f$ of input and
output. The input shaft gets \f$-\tau_i\f$ and the output shaft \f$r_i \tau_i\f$. The selected gear (gear: 1 to the number of ratios, -1 for the
reverse ratio, 0 for neutral, as an input that can be set as the model runs) is
engaged, and the others released, by the first order rise or fall of \f$e_i\f$ over
shift_time. During a shift the old gear's clutch releases while the new one
applies, as in a power-shift transmission, so the torque is handed over smoothly
and the clutches slip as they do. The clutch of an engaged gear does not slip
more than its capacity allows: its smoothed Coulomb friction means that it
carries torque \f$\tau\f$ only with a small slip,
\f$\omega_0 \, \mathrm{atanh}(\tau / c_{max})\f$, so with a
generous capacity the gear ratio is kept closely. spin_loss is a viscous loss
on the output shaft, which makes up the losses of the transmission.

*The axles.* Each entry of axles has the propshaft from the transmission's output
to the axle, with a torsional stiffness and damping and the ratio transfer_ratio
of the transfer case (the speed of the propshaft over that of the output), its
final drive of ratio final_drive (the pinion speed over that of the ring gear)
and its differential, and two wheels, the revolute joints left and right,
turning positively forward when direction is 1 (-1 when forward is negative, as in
the vehicle examples). The twist of the propshaft has the rate

\f[
\dot{\theta} = n_t \omega_o - n_f \omega_c, \quad
\omega_c = (\omega_{left} + \omega_{right}) / 2,
\f]

with the transfer_ratio \f$n_t\f$ and the final_drive \f$n_f\f$, and its torque
is \f$\tau_s = \mathrm{engaged} \, (k \theta + c \dot{\theta})\f$. It acts on the
output shaft as \f$-n_t \tau_s\f$ and on the differential's carrier as
\f$n_f \tau_s\f$, which the differential divides:

- open: half to each wheel;
- locked: half to each wheel and a viscous coupling lock_damping
  \f$(\omega_{left} - \omega_{right})\f$ that holds the wheels to the same speed;
- limited_slip: half to each wheel and a torque
  \f$(\tau_{bias} + n_b |\tau_{carrier}|) \tanh\left((\omega_{left} - \omega_{right}) / \omega_0\right)\f$
  (bias_torque \f$\tau_{bias}\f$, bias_ratio \f$n_b\f$) from the faster wheel to the
  slower, from the preload of its clutches and in proportion to the load.

engaged (0 to 1, an input) connects the axle to the transfer case, or
disconnects it (a two wheel drive, or a transfer case that is shifted): the
propshaft transmits nothing at 0. Several axles in one powertrain share the
transmission's output through their propshafts, with a locked transfer case
(whose load sharing follows from the stiffnesses and speeds of the propshafts).

*The brakes.* A brake of capacity "capacity" on a joint (a wheel, or a shaft)
has the torque of smoothed Coulomb friction against the speed of the joint,
\f$\mathrm{capacity} \cdot \mathrm{engagement} \cdot \tanh(\omega / \omega_0)\f$, engagement being the pressure
(applied_by "pressure": the brake is applied by pressure) or 1 minus the pressure
(applied_by "spring": the spring applies it and the pressure releases it, as a
parking brake), the pressure an input between 0 and 1. A smoothed friction does
not hold a vehicle at rest, which creeps at a speed that falls with slip_speed.

*Friction that holds.* The brakes, the lockup clutch or clutch of the coupling, the
clutches of the gears and the clutches of a limited slip differential can use the
bristle model of friction instead (LuGre, as the stick_slip of the contact module),
by giving their bristle_stiffness \f$\sigma_0\f$ (1/rad, greater than 0). The
friction torque of an element of capacity \f$c_e\f$ (the capacity times its
engagement) between parts with the relative speed \f$u\f$ is then

\f[
\tau = c_e (\sigma_0 z + \sigma_1 \dot{z}), \quad
\dot{z} = u - \frac{\sigma_0 |u|}{g(|u|)} z, \quad
g(|u|) = 1 + (r_s - 1) e^{-(|u| / v_s)^2},
\f]

with the bristle deflection \f$z\f$ (an angle, a state of the module for each
such element), bristle_damping \f$\sigma_1\f$ (s/rad), the static friction over
the Coulomb friction \f$r_s\f$ (static_ratio, at least 1) and the speed
\f$v_s\f$ (stribeck_speed) over which it falls from the first to the second. At
rest the deflection settles where \f$\sigma_0 z\f$ carries the load, so the
element holds, to a slip of \f$z\f$, up to \f$r_s c_e\f$, and slides beyond that
with the Coulomb friction \f$c_e\f$: a brake holds a vehicle on a slope, a
locked clutch does not slip at all, and a limited slip differential locks while
the torque it must carry is within its capacity. The bristles are stiff: they
relax within \f$1 / (\sigma_0 |u|)\f$ at the slip speed \f$|u|\f$, which the
integrator resolves.

JSON:
\code
{"type": "powertrain", "name": "drive",
 "engine": {"inertia": 1.5, "idle_speed": 80, "high_idle_speed": 230,
            "torque_curve": [80, 500, 150, 900, 200, 800, 230, 0],
            "friction_curve": [80, -60, 230, -150]},
 "coupling": {"type": "converter",
              "capacity_curve": [0, 10, 0.8, 11, 1.0, 12],
              "torque_ratio_curve": [0, 2.2, 0.85, 1.0, 1.0, 1.0],
              "capacity": 1200},
 "transmission": {"ratios": [3.5, 2.0, 1.0], "reverse_ratio": -3.5,
                  "gear": 1, "clutch_capacity": 4000},
 "axles": [{"name": "rear", "left": "left_wheel", "right": "right_wheel",
            "direction": -1, "final_drive": 4.0, "shaft_stiffness": 2e4,
            "shaft_damping": 100}],
 "brakes": [{"name": "left", "joint": "left_wheel", "direction": -1,
             "capacity": 4000}]}
\endcode

The inputs that can be changed as the model runs (see the schedule controller)
are engine.throttle, coupling.engagement, transmission.gear, an axle's engaged
and a brake's pressure.

Outputs: engine_speed, engine_torque, fuel, turbine_speed, output_speed,
converter_torque_ratio, gear_ratio (the ratio of the engaged gears, weighted by
their engagement), and for each axle <axle>_shaft_torque, <axle>_left_torque and
<axle>_right_torque (the torques on the wheels).

Not in the module: the dynamics of the pressure of the clutches (they rise
and fall as first order lags, not by the hydraulics), the inertia of the fluid
in the converter, the efficiency of the gears and final drives apart from
spin_loss, a center differential that divides the torque in a fixed ratio, and
the stick of the brakes at rest.
*/
class Powertrain : public Base {
public:
  Powertrain()
    : Base() {
    Initialize();
  }
  Powertrain(std::string name)
    : Base(name) {
    Initialize();
  }
  virtual ~Powertrain() override {}

  static std::unique_ptr<Powertrain> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  /*! \brief the torque of the engine at the speed \f$\omega\f$ and throttle, and the
   * fuel fraction after the governor and the fuel cut */
  double EngineTorque(double speed, double throttle, double* fuel = nullptr) const;

  /*! \brief the torque on the pump and on the turbine of the converter (the
   * coupling's type), with its lockup clutch, for pump speed \f$\omega_e\f$ and
   * turbine speed \f$\omega_t\f$, as torque on the engine side and on the turbine side */
  void CouplingTorques(double w_e, double w_t, double* on_pump,
                       double* on_turbine, double* ratio = nullptr,
                       double lockup_friction = NAN) const;

  /*! \brief the number of states of each kind: three speeds, the twists, the
   * engagements */
  size_t get_state_count() const {
    return 3 + axle_count_ + gear_count_ + extra_states_;
  }

  double get_engine_speed() const {
    return outputs_.empty() ? 0.0 : outputs_[0];
  }
  const std::vector<double>& get_output_values() const {
    return outputs_;
  }

protected:
  void DeclareParameters() override;

private:
  void Initialize() {
    DeclareParameters();
  }

  struct Wheel {
    joint::Base* joint = nullptr;
    double direction = 1.0;
  };
  struct Axle {
    Wheel left, right;
  };
  struct Brake {
    joint::Base* joint = nullptr;
  };

  void Compute(const double* y, double* dy, bool apply);

  static bool registered_;

  PowertrainEngine engine_;
  PowertrainCoupling coupling_;
  PowertrainTransmission transmission_;
  parameter::Array<PowertrainAxle> axles_;
  parameter::Array<PowertrainBrake> brakes_;

  Table torque_table_, friction_table_, capacity_table_, ratio_table_;
  size_t gear_count_ = 0; //!< forward gears and the reverse
  size_t axle_count_ = 0;
  /* the states of the bristles of the friction elements, after the others;
  kNone for an element without them */
  static constexpr size_t kNone = static_cast<size_t>(-1);
  size_t extra_states_ = 0;
  size_t coupling_state_ = kNone;
  size_t gear_state_ = kNone; //!< the first of the gears' (one for each)
  std::vector<size_t> axle_state_, brake_state_;
  std::vector<Axle> wheel_;
  std::vector<Brake> brake_;
  std::vector<double> outputs_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
