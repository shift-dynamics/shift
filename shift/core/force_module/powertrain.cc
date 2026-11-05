#include "shift/core/force_module/powertrain.h"

#include <algorithm>
#include <cmath>

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/base.h"

bool shift::core::force_module::Powertrain::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "powertrain", shift::core::force_module::Powertrain::Create);

std::unique_ptr<shift::core::force_module::Powertrain>
shift::core::force_module::Powertrain::Create() {
  return std::make_unique<shift::core::force_module::Powertrain>();
}

void shift::core::force_module::PowertrainEngine::DeclareParameters() {
  DeclareParameter("inertia", &inertia,
                   "moment of inertia of the engine, flywheel and the pump "
                   "of the converter",
                   true);
  DeclareParameter("idle_speed", &idle_speed,
                   "speed below which the governor opens the throttle (rad/s)",
                   true);
  DeclareParameter("high_idle_speed", &high_idle_speed,
                   "speed at which the fuel is cut off (rad/s)", true);
  DeclareParameter("governor_gain", &governor_gain,
                   "fuel fraction the governor adds per rad/s under the idle "
                   "speed; optional, default 0.1",
                   false);
  DeclareParameter("cut_range", &cut_range,
                   "range of speed below the high idle speed over which the "
                   "fuel is cut off; optional, default 5 % of it",
                   false);
  DeclareParameter("throttle", &throttle,
                   "throttle, 0 to 1; optional, default 0", false)
    .set_runtime();
  DeclareParameter("torque_curve", &torque_curve,
                   "net torque at full fuel against speed: [w0, T0, w1, T1, "
                   "...]",
                   true)
    .set_row_dim(-1);
  DeclareParameter("friction_curve", &friction_curve,
                   "torque at closed throttle against speed (friction and "
                   "pumping, negative): [w0, T0, w1, T1, ...]; optional, "
                   "default none",
                   false)
    .set_row_dim(-1);
}

void shift::core::force_module::PowertrainCoupling::DeclareParameters() {
  DeclareParameter("type", &type,
                   "\"converter\" (a torque converter with a lockup clutch) "
                   "or \"clutch\"; optional, default \"converter\"",
                   false);
  DeclareParameter("capacity_curve", &capacity_curve,
                   "converter: capacity factor K (rad/s per square root of "
                   "N m) against speed ratio of turbine over pump: [sr0, K0, "
                   "...]",
                   false)
    .set_row_dim(-1);
  DeclareParameter("torque_ratio_curve", &torque_ratio_curve,
                   "converter: torque ratio against speed ratio: [sr0, TR0, "
                   "...]",
                   false)
    .set_row_dim(-1);
  DeclareParameter("capacity", &capacity,
                   "torque capacity of the lockup clutch (converter) or of "
                   "the clutch; optional, default 0",
                   false);
  DeclareParameter("engagement", &engagement,
                   "engagement of the lockup clutch or the clutch, 0 to 1; "
                   "optional, default 1",
                   false)
    .set_runtime();
  DeclareParameter("slip_speed", &slip_speed,
                   "slip speed that smooths the clutch friction (rad/s); "
                   "optional, default 0.5",
                   false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness,
                   "bristle model of friction: sigma_0, the stiffness of the "
                   "bristles (1/rad), which makes the friction hold at rest; "
                   "optional, default 0 (smoothed Coulomb friction)",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping,
                   "bristle model: sigma_1, the damping of the bristles "
                   "(s/rad); optional, default 0",
                   false);
  DeclareParameter("static_ratio", &static_ratio,
                   "bristle model: the static friction over the Coulomb "
                   "friction, at least 1; optional, default 1",
                   false);
  DeclareParameter("stribeck_speed", &stribeck_speed,
                   "bristle model: the speed over which the friction falls "
                   "from the static to the Coulomb (rad/s); optional, "
                   "default 1",
                   false);
}

void shift::core::force_module::PowertrainTransmission::DeclareParameters() {
  DeclareParameter("ratios", &ratios,
                   "forward gear ratios, input speed over output speed",
                   true)
    .set_row_dim(-1);
  DeclareParameter("reverse_ratio", &reverse_ratio,
                   "reverse gear ratio, negative; optional, default none",
                   false);
  DeclareParameter("gear", &gear,
                   "selected gear: 1 to the number of ratios, -1 reverse, 0 "
                   "neutral; optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("shift_time", &shift_time,
                   "time over which a gear's clutch applies or releases; "
                   "optional, default 0.3",
                   false);
  DeclareParameter("clutch_capacity", &clutch_capacity,
                   "torque capacity of the clutch of a gear, N m", true);
  DeclareParameter("slip_speed", &slip_speed,
                   "slip speed that smooths the clutch friction (rad/s); "
                   "optional, default 0.5",
                   false);
  DeclareParameter("input_inertia", &input_inertia,
                   "moment of inertia of the input shaft with the turbine; "
                   "optional, default 0.5",
                   false);
  DeclareParameter("output_inertia", &output_inertia,
                   "moment of inertia of the output shaft; optional, default "
                   "0.5",
                   false);
  DeclareParameter("spin_loss", &spin_loss,
                   "viscous loss torque on the output shaft per rad/s; "
                   "optional, default 0",
                   false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness,
                   "bristle model of friction: sigma_0, the stiffness of the "
                   "bristles (1/rad), which makes the friction hold at rest; "
                   "optional, default 0 (smoothed Coulomb friction)",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping,
                   "bristle model: sigma_1, the damping of the bristles "
                   "(s/rad); optional, default 0",
                   false);
  DeclareParameter("static_ratio", &static_ratio,
                   "bristle model: the static friction over the Coulomb "
                   "friction, at least 1; optional, default 1",
                   false);
  DeclareParameter("stribeck_speed", &stribeck_speed,
                   "bristle model: the speed over which the friction falls "
                   "from the static to the Coulomb (rad/s); optional, "
                   "default 1",
                   false);
}

void shift::core::force_module::PowertrainAxle::DeclareParameters() {
  DeclareParameter("name", &name, "name of the axle", true);
  DeclareParameter("left", &left, "the joint of the left wheel", true);
  DeclareParameter("right", &right, "the joint of the right wheel", true);
  DeclareParameter("direction", &direction,
                   "1 if the joints of the wheels turn positively going "
                   "forward, -1 if negatively; optional, default 1",
                   false);
  DeclareParameter("final_drive", &final_drive,
                   "ratio of the final drive, pinion speed over wheel speed; "
                   "optional, default 1",
                   false);
  DeclareParameter("transfer_ratio", &transfer_ratio,
                   "ratio of the transfer case, propshaft speed over output "
                   "speed; optional, default 1",
                   false);
  DeclareParameter("shaft_stiffness", &shaft_stiffness,
                   "torsional stiffness of the propshaft, N m/rad", true);
  DeclareParameter("shaft_damping", &shaft_damping,
                   "torsional damping of the propshaft; optional, default 0",
                   false);
  DeclareParameter("engaged", &engaged,
                   "engagement of the axle to the transfer case, 0 to 1; "
                   "optional, default 1",
                   false)
    .set_runtime();
  DeclareParameter("differential", &differential,
                   "\"open\", \"locked\" or \"limited_slip\"; optional, "
                   "default \"open\"",
                   false);
  DeclareParameter("bias_torque", &bias_torque,
                   "limited slip: preload torque of the clutches; optional, "
                   "default 0",
                   false)
    .set_runtime();
  DeclareParameter("bias_ratio", &bias_ratio,
                   "limited slip: torque of the clutches per torque of the "
                   "carrier; optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("lock_damping", &lock_damping,
                   "locked: damping that holds the wheels to one speed "
                   "(N m s/rad); optional, default 1000",
                   false);
  DeclareParameter("slip_speed", &slip_speed,
                   "limited slip: slip speed that smooths the clutches "
                   "(rad/s); optional, default 0.5",
                   false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness,
                   "bristle model of friction: sigma_0, the stiffness of the "
                   "bristles (1/rad), which makes the friction hold at rest; "
                   "optional, default 0 (smoothed Coulomb friction)",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping,
                   "bristle model: sigma_1, the damping of the bristles "
                   "(s/rad); optional, default 0",
                   false);
  DeclareParameter("static_ratio", &static_ratio,
                   "bristle model: the static friction over the Coulomb "
                   "friction, at least 1; optional, default 1",
                   false);
  DeclareParameter("stribeck_speed", &stribeck_speed,
                   "bristle model: the speed over which the friction falls "
                   "from the static to the Coulomb (rad/s); optional, "
                   "default 1",
                   false);
}

void shift::core::force_module::PowertrainBrake::DeclareParameters() {
  DeclareParameter("name", &name, "name of the brake", true);
  DeclareParameter("joint", &joint, "the joint that the brake acts on", true);
  DeclareParameter("direction", &direction,
                   "1 if the joint turns positively going forward, -1 if "
                   "negatively; optional, default 1",
                   false);
  DeclareParameter("capacity", &capacity,
                   "torque capacity of the brake, N m", true);
  DeclareParameter("pressure", &pressure,
                   "pressure on the brake, 0 to 1; optional, default 0", false)
    .set_runtime();
  DeclareParameter("applied_by", &applied_by,
                   "\"pressure\" (applied by pressure) or \"spring\" "
                   "(applied by a spring, released by pressure); optional, "
                   "default \"pressure\"",
                   false);
  DeclareParameter("slip_speed", &slip_speed,
                   "speed below which the friction goes smoothly to zero "
                   "(rad/s); optional, default 0.1",
                   false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness,
                   "bristle model of friction: sigma_0, the stiffness of the "
                   "bristles (1/rad), which makes the friction hold at rest; "
                   "optional, default 0 (smoothed Coulomb friction)",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping,
                   "bristle model: sigma_1, the damping of the bristles "
                   "(s/rad); optional, default 0",
                   false);
  DeclareParameter("static_ratio", &static_ratio,
                   "bristle model: the static friction over the Coulomb "
                   "friction, at least 1; optional, default 1",
                   false);
  DeclareParameter("stribeck_speed", &stribeck_speed,
                   "bristle model: the speed over which the friction falls "
                   "from the static to the Coulomb (rad/s); optional, "
                   "default 1",
                   false);
}

void shift::core::force_module::Powertrain::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("engine", &engine_, "the engine", true);
  DeclareParameter("coupling", &coupling_,
                   "the converter or clutch between the engine and the "
                   "transmission",
                   true);
  DeclareParameter("transmission", &transmission_, "the transmission", true);
  DeclareParameter("axles", &axles_,
                   "the driven axles, each with a propshaft, a final drive, "
                   "a differential and two wheels; optional",
                   false);
  DeclareParameter("brakes", &brakes_, "brakes on joints; optional", false);
}

void shift::core::force_module::Powertrain::Connect() {
  const std::string prefix = "Powertrain " + get_name() + ": ";
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception(prefix + "no parent assembly");
  }
  auto find = [&](const std::string& name) {
    joint::Base* joint = nullptr;
    try {
      joint = (*parent_assembly_ptr_)["joints"][name]
                .get_data<shift::core::joint::Base>();
    } catch(...) {
      joint = nullptr;
    }
    if(joint == nullptr || joint->get_dq_count() != 1) {
      throw shift::core::Exception(prefix + "joint '" + name +
                                   "' not found in its assembly, or not a "
                                   "joint of one coordinate");
    }
    return joint;
  };
  wheel_.clear();
  for(const auto& axle : axles_.get_data()) {
    Axle a;
    a.left.joint = find(axle->left);
    a.right.joint = find(axle->right);
    a.left.direction = a.right.direction = axle->direction;
    wheel_.push_back(a);
  }
  brake_.clear();
  for(const auto& brake : brakes_.get_data()) {
    Brake b;
    b.joint = find(brake->joint);
    brake_.push_back(b);
  }
}

void shift::core::force_module::Powertrain::Load() {
  const std::string prefix = "Powertrain " + get_name() + ": ";
  auto& e = engine_;
  auto& c = coupling_;
  auto& t = transmission_;
  if(!(e.inertia > 0.0) || !(e.idle_speed > 0.0) ||
     !(e.high_idle_speed > e.idle_speed) || e.governor_gain < 0.0) {
    throw shift::core::Exception(
      prefix + "engine needs a positive inertia and idle_speed, a "
               "high_idle_speed above it, and a governor_gain not negative");
  }
  if(e.cut_range < 0.0) {
    e.cut_range = 0.05 * e.high_idle_speed;
  }
  if(!(e.cut_range > 0.0) ||
     !(e.cut_range < e.high_idle_speed - e.idle_speed)) {
    throw shift::core::Exception(prefix + "engine cut_range must be positive "
                                 "and less than the range from idle_speed to "
                                 "high_idle_speed");
  }
  torque_table_ = Table::FromPairs(e.torque_curve, Table::Extrapolation::kHold,
                                   prefix + "engine torque_curve");
  friction_table_ =
    e.friction_curve.empty()
      ? Table()
      : Table::FromPairs(e.friction_curve, Table::Extrapolation::kHold,
                         prefix + "engine friction_curve");

  if(c.type != "converter" && c.type != "clutch") {
    throw shift::core::Exception(prefix + "coupling type must be "
                                 "\"converter\" or \"clutch\"");
  }
  if(c.capacity < 0.0 || !(c.slip_speed > 0.0)) {
    throw shift::core::Exception(prefix + "coupling needs a capacity not "
                                 "negative and a positive slip_speed");
  }
  if(c.type == "converter") {
    capacity_table_ = Table::FromPairs(c.capacity_curve,
                                       Table::Extrapolation::kHold,
                                       prefix + "coupling capacity_curve");
    ratio_table_ = Table::FromPairs(c.torque_ratio_curve,
                                    Table::Extrapolation::kHold,
                                    prefix + "coupling torque_ratio_curve");
    /* the converter cannot give more power than it takes while the turbine is
    slower than the pump: TR(sr) sr <= 1 for sr <= 1 */
    for(double sr = 0.0; sr <= 1.0 + 1e-9; sr += 0.01) {
      if(ratio_table_.Evaluate(sr) * sr > 1.0 + 1e-9 ||
         capacity_table_.Evaluate(sr) <= 0.0 ||
         ratio_table_.Evaluate(sr) < 0.0) {
        throw shift::core::Exception(
          prefix + "the converter's curves give positive capacity and torque "
                   "ratio, with an efficiency TR(sr) sr of at most 1; they "
                   "do not at a speed ratio of " + std::to_string(sr));
      }
    }
  } else {
    capacity_table_ = Table();
    ratio_table_ = Table();
  }

  if(t.ratios.empty() || !(t.clutch_capacity > 0.0) ||
     !(t.slip_speed > 0.0) || !(t.input_inertia > 0.0) ||
     !(t.output_inertia > 0.0) || t.shift_time < 0.0 || t.spin_loss < 0.0) {
    throw shift::core::Exception(
      prefix + "transmission needs ratios, a positive clutch_capacity, "
               "slip_speed and inertias, and a shift_time and spin_loss not "
               "negative");
  }
  for(double r : t.ratios) {
    if(!(r > 0.0)) {
      throw shift::core::Exception(prefix + "forward ratios must be positive");
    }
  }
  if(t.reverse_ratio > 0.0) {
    throw shift::core::Exception(prefix + "reverse_ratio must be negative "
                                 "(or 0 for no reverse)");
  }
  gear_count_ = t.ratios.size() + (t.reverse_ratio < 0.0 ? 1 : 0);
  const double gear = std::round(t.gear);
  if(gear > static_cast<double>(t.ratios.size()) || gear < -1.0 ||
     (gear == -1.0 && t.reverse_ratio == 0.0)) {
    throw shift::core::Exception(prefix + "gear is not a gear of the "
                                 "transmission");
  }

  axle_count_ = axles_.get_data().size();
  for(const auto& axle : axles_.get_data()) {
    if(!(axle->shaft_stiffness > 0.0) || axle->shaft_damping < 0.0 ||
       !(axle->final_drive > 0.0) || !(axle->transfer_ratio > 0.0) ||
       !(axle->direction == 1.0 || axle->direction == -1.0) ||
       axle->engaged < 0.0 || axle->engaged > 1.0 ||
       !(axle->slip_speed > 0.0) || axle->lock_damping < 0.0 ||
       axle->bias_torque < 0.0 || axle->bias_ratio < 0.0) {
      throw shift::core::Exception(
        prefix + "axle '" + axle->name + "': shaft_stiffness, final_drive, "
        "transfer_ratio and slip_speed must be positive, direction 1 or -1, "
        "engaged from 0 to 1, and the damping and bias not negative");
    }
    if(axle->differential != "open" && axle->differential != "locked" &&
       axle->differential != "limited_slip") {
      throw shift::core::Exception(prefix + "axle '" + axle->name +
                                   "': differential must be \"open\", "
                                   "\"locked\" or \"limited_slip\"");
    }
  }
  for(const auto& brake : brakes_.get_data()) {
    if(!(brake->capacity >= 0.0) || !(brake->slip_speed > 0.0) ||
       !(brake->direction == 1.0 || brake->direction == -1.0) ||
       (brake->applied_by != "pressure" && brake->applied_by != "spring")) {
      throw shift::core::Exception(
        prefix + "brake '" + brake->name + "': capacity not negative, "
        "slip_speed positive, direction 1 or -1, and applied_by \"pressure\" "
        "or \"spring\"");
    }
  }

  /* the bristle model of friction, where given: sane values, and a state
  for each element that uses it, after the others */
  auto check_bristles = [&](const std::string& what, double stiffness,
                            double damping, double ratio, double speed) {
    if(stiffness < 0.0 || damping < 0.0 || !(ratio >= 1.0) ||
       !(speed > 0.0)) {
      throw shift::core::Exception(
        prefix + what + ": bristle_stiffness and bristle_damping must not be "
                        "negative, static_ratio at least 1, and "
                        "stribeck_speed positive");
    }
  };
  check_bristles("coupling", c.bristle_stiffness, c.bristle_damping,
                 c.static_ratio, c.stribeck_speed);
  check_bristles("transmission", t.bristle_stiffness, t.bristle_damping,
                 t.static_ratio, t.stribeck_speed);
  size_t next = 3 + axle_count_ + gear_count_;
  const size_t first_extra = next;
  coupling_state_ = c.bristle_stiffness > 0.0 ? next++ : kNone;
  gear_state_ = kNone;
  if(t.bristle_stiffness > 0.0) {
    gear_state_ = next;
    next += gear_count_;
  }
  axle_state_.clear();
  for(const auto& axle : axles_.get_data()) {
    check_bristles("axle '" + axle->name + "'", axle->bristle_stiffness,
                   axle->bristle_damping, axle->static_ratio,
                   axle->stribeck_speed);
    axle_state_.push_back(axle->bristle_stiffness > 0.0 &&
                              axle->differential == "limited_slip"
                            ? next++
                            : kNone);
  }
  brake_state_.clear();
  for(const auto& brake : brakes_.get_data()) {
    check_bristles("brake '" + brake->name + "'", brake->bristle_stiffness,
                   brake->bristle_damping, brake->static_ratio,
                   brake->stribeck_speed);
    brake_state_.push_back(brake->bristle_stiffness > 0.0 ? next++ : kNone);
  }
  extra_states_ = next - first_extra;

  /* the states: the speeds of the engine, of the transmission's input and of
  its output, the twists, the engagements (the selected gear's at 1) and the
  deflections of the bristles */
  equation_count_ = static_cast<unsigned int>(get_state_count());
  q_0_.assign(equation_count_, 0.0);
  q_0_[0] = e.idle_speed;
  if(gear >= 1.0) {
    q_0_[3 + axle_count_ + static_cast<size_t>(gear) - 1] = 1.0;
  } else if(gear == -1.0) {
    q_0_[3 + axle_count_ + t.ratios.size()] = 1.0;
  }
  outputs_.assign(get_output_names().size(), 0.0);
}

/* 0 below 0, 1 above 1, and a cubic between with no slope at either end */
static double SmoothStep(double x) {
  if(!(x > 0.0)) {
    return 0.0;
  }
  if(x >= 1.0) {
    return 1.0;
  }
  return x * x * (3.0 - 2.0 * x);
}

double shift::core::force_module::Powertrain::EngineTorque(
  double speed, double throttle, double* fuel) const {
  const auto& e = engine_;
  const double governor = e.governor_gain * (e.idle_speed - speed);
  double f = std::min(std::max(std::max(throttle, governor), 0.0), 1.0);
  f *= 1.0 - SmoothStep((speed - (e.high_idle_speed - e.cut_range)) /
                        e.cut_range);
  if(fuel) {
    *fuel = f;
  }
  const double motoring =
    friction_table_.empty() ? 0.0 : friction_table_.Evaluate(speed);
  return f * torque_table_.Evaluate(speed) + (1.0 - f) * motoring;
}

void shift::core::force_module::Powertrain::CouplingTorques(
  double w_e, double w_t, double* on_pump, double* on_turbine, double* ratio,
  double lockup_friction) const {
  const auto& c = coupling_;
  double pump = 0.0, turbine = 0.0, torque_ratio = 0.0;
  if(c.type == "converter") {
    /* the speed ratio, smooth for an engine at a standstill */
    const double sr = w_t * w_e / (w_e * w_e + 1.0);
    const double k = capacity_table_.Evaluate(sr);
    torque_ratio = ratio_table_.Evaluate(sr);
    pump = w_e * std::fabs(w_e) / (k * k);
    turbine = torque_ratio * pump;
  }
  /* the friction of the lockup clutch or the clutch: smoothed Coulomb
  friction, or what the bristles give if they are used */
  const double friction = std::isnan(lockup_friction)
                            ? std::tanh((w_e - w_t) / c.slip_speed)
                            : lockup_friction;
  const double lockup =
    c.capacity * std::min(std::max(c.engagement, 0.0), 1.0) * friction;
  *on_pump = pump + lockup;
  *on_turbine = turbine + lockup;
  if(ratio) {
    *ratio = torque_ratio;
  }
}

void shift::core::force_module::Powertrain::Service(const double* y,
                                                    double* dy) {
  Compute(y, dy, true);
}

void shift::core::force_module::Powertrain::UpdateOutputs(double,
                                                          const double* y) {
  std::vector<double> scratch(y, y + state_offset_ + get_state_count());
  std::vector<double> rates(scratch.size(), 0.0);
  Compute(scratch.data(), rates.data(), false);
}

void shift::core::force_module::Powertrain::Compute(const double* y,
                                                    double* dy, bool apply) {
  const auto& t = transmission_;
  const size_t s = state_offset_;
  const size_t axles = axle_count_;
  const double w_e = y[s], w_t = y[s + 1], w_o = y[s + 2];
  const size_t q_count = context_->get_q_count();
  auto speed = [&](const Wheel& w) {
    return w.direction * y[q_count + w.joint->get_dq_index()];
  };
  auto torque_on = [&](const Wheel& w, double torque) {
    context_->get_joint_working_forces()[w.joint->get_dq_index()] +=
      w.direction * torque;
  };

  /* the friction of an element between parts with the relative speed slip, as
  a fraction of its capacity: the smoothed Coulomb friction, or, if it has a
  state for the bristles, the bristle model, which also sets the state's rate */
  auto friction = [&](size_t state, double slip, double smoothing,
                      double stiffness, double damping, double ratio,
                      double stribeck) {
    if(state == kNone) {
      return std::tanh(slip / smoothing);
    }
    const double z = y[s + state];
    const double g =
      1.0 + (ratio - 1.0) * std::exp(-(slip / stribeck) * (slip / stribeck));
    const double rate = slip - stiffness * std::fabs(slip) * z / g;
    dy[s + state] = rate;
    return stiffness * z + damping * rate;
  };

  /* the engine and its coupling to the transmission's input */
  double fuel;
  const double engine_torque =
    EngineTorque(w_e, engine_.throttle, &fuel);
  double on_pump, on_turbine, torque_ratio;
  const auto& coupling = coupling_;
  CouplingTorques(
    w_e, w_t, &on_pump, &on_turbine, &torque_ratio,
    coupling_state_ == kNone
      ? NAN
      : friction(coupling_state_, w_e - w_t, coupling.slip_speed,
                 coupling.bristle_stiffness, coupling.bristle_damping,
                 coupling.static_ratio, coupling.stribeck_speed));

  /* the clutches of the gears */
  double clutch_sum = 0.0, output_sum = 0.0;
  double ratio_sum = 0.0, engaged_sum = 0.0;
  const double target_gear = std::round(t.gear);
  const double tau = std::max(t.shift_time / 3.0, 1e-4);
  for(size_t i = 0; i < gear_count_; i++) {
    const bool reverse = i >= t.ratios.size();
    const double ratio = reverse ? t.reverse_ratio : t.ratios[i];
    const double e = y[s + 3 + axles + i];
    const double slip = w_t - ratio * w_o;
    const double torque =
      e * t.clutch_capacity *
      friction(gear_state_ == kNone ? kNone : gear_state_ + i, slip,
               t.slip_speed, t.bristle_stiffness, t.bristle_damping,
               t.static_ratio, t.stribeck_speed);
    clutch_sum += torque;
    output_sum += ratio * torque;
    ratio_sum += ratio * e;
    engaged_sum += e;
    const double target =
      reverse ? (target_gear == -1.0 ? 1.0 : 0.0)
              : (target_gear == static_cast<double>(i + 1) ? 1.0 : 0.0);
    dy[s + 3 + axles + i] = (target - e) / tau;
  }

  /* the axles: the propshaft, the final drive and the differential */
  double propshaft_load = 0.0; // on the output shaft
  std::vector<double> axle_outputs;
  for(size_t a = 0; a < axles; a++) {
    const auto& axle = *axles_.get_data()[a];
    const Wheel& left = wheel_[a].left;
    const Wheel& right = wheel_[a].right;
    const double w_l = speed(left), w_r = speed(right);
    const double carrier = 0.5 * (w_l + w_r);
    const double twist = y[s + 3 + a];
    const double twist_rate =
      axle.transfer_ratio * w_o - axle.final_drive * carrier;
    dy[s + 3 + a] = twist_rate;
    const double shaft = axle.engaged * (axle.shaft_stiffness * twist +
                                         axle.shaft_damping * twist_rate);
    propshaft_load += axle.transfer_ratio * shaft;
    const double carrier_torque = axle.final_drive * shaft;
    double t_l = 0.5 * carrier_torque, t_r = 0.5 * carrier_torque;
    if(axle.differential == "locked") {
      const double coupling = axle.lock_damping * (w_l - w_r);
      t_l -= coupling;
      t_r += coupling;
    } else if(axle.differential == "limited_slip") {
      const double bias =
        (axle.bias_torque + axle.bias_ratio * std::fabs(carrier_torque)) *
        friction(axle_state_[a], w_l - w_r, axle.slip_speed,
                 axle.bristle_stiffness, axle.bristle_damping,
                 axle.static_ratio, axle.stribeck_speed);
      t_l -= bias;
      t_r += bias;
    }
    if(apply) {
      torque_on(left, t_l);
      torque_on(right, t_r);
    }
    axle_outputs.push_back(shaft);
    axle_outputs.push_back(t_l);
    axle_outputs.push_back(t_r);
  }

  /* the brakes */
  if(apply) {
    for(size_t b = 0; b < brake_.size(); b++) {
      const auto& brake = *brakes_.get_data()[b];
      const double pressure = std::min(std::max(brake.pressure, 0.0), 1.0);
      const double applied =
        brake.applied_by == "pressure" ? pressure : 1.0 - pressure;
      const double w = brake.direction *
                       y[q_count + brake_[b].joint->get_dq_index()];
      const double torque =
        -brake.capacity * applied *
        friction(brake_state_[b], w, brake.slip_speed, brake.bristle_stiffness,
                 brake.bristle_damping, brake.static_ratio,
                 brake.stribeck_speed);
      context_->get_joint_working_forces()[brake_[b].joint->get_dq_index()] +=
        brake.direction * torque;
    }
  }

  dy[s] = (engine_torque - on_pump) / engine_.inertia;
  dy[s + 1] = (on_turbine - clutch_sum) / t.input_inertia;
  dy[s + 2] = (output_sum - propshaft_load - t.spin_loss * w_o) /
              t.output_inertia;

  outputs_.assign({w_e, engine_torque, fuel, w_t, w_o, torque_ratio,
                   engaged_sum > 1e-9 ? ratio_sum / engaged_sum : 0.0});
  outputs_.insert(outputs_.end(), axle_outputs.begin(), axle_outputs.end());
}

std::vector<std::string>
shift::core::force_module::Powertrain::get_output_names() const {
  std::vector<std::string> names = {
    "engine_speed", "engine_torque", "fuel",       "turbine_speed",
    "output_speed", "converter_torque_ratio", "gear_ratio"};
  /* get_data is not const; reading the names does not change the array */
  for(const auto& axle :
      const_cast<parameter::Array<PowertrainAxle>&>(axles_).get_data()) {
    names.push_back(axle->name + "_shaft_torque");
    names.push_back(axle->name + "_left_torque");
    names.push_back(axle->name + "_right_torque");
  }
  return names;
}

void shift::core::force_module::Powertrain::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), outputs_.begin(), outputs_.end());
}
