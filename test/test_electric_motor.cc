/* Electric motor force module (force_module::ElectricMotor): the speed of a
rotor under the voltage drive, with and without inductance, and under the
current drive within its limits, against closed forms; a PID controller
setting its current input; and the errors of its inputs */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/force_module/electric_motor.h"

/* the rotor's inertia about its axis, the torque constant and the
resistance: the mechanical time constant J R / k^2 is 1 s */
static const double kInertia = 0.01;
static const double kTorqueConstant = 0.1;
static const double kResistance = 1.0;

/* a rotor on a revolute joint about z, driven by a motor with the keys of
motor, with a speed sensor and the controllers of the argument; joint adds
keys to the joint */
static std::string Model(const std::string& motor, double time_span,
                         const std::string& controllers = "",
                         const std::string& joint = "") {
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
        << time_span << R"(, "time_step": 0.01,
                     "relative_error": 1e-9, "absolute_error": 1e-11}]},
  "assemblies": [{"name": "a",
    "bodies": [{"name": "rotor", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": )"
        << kInertia << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "bearing", "type": "revolute", "axis": [0, 0, 1],
                "parent": [["fixed"], "origin"],
                "child": [["rotor"], "origin"])"
        << joint << R"(}],
    "force_modules": [{"type": "electric_motor", "name": "motor",
      "joint": "bearing", "torque_constant": )"
        << kTorqueConstant << R"(, "resistance": )" << kResistance;
  if(!motor.empty()) {
    model << ", " << motor;
  }
  model << R"(}],
    "sensors": [{"type": "joint", "name": "speed", "joint": "bearing",
                 "quantity": "velocity"}],
    "controllers": [)"
        << controllers << R"(]}]})";
  return model.str();
}

static void Load(shift::core::Context& context, const std::string& json) {
  const std::string path = testing::TempDir() + "/electric_motor_test.json";
  {
    std::ofstream file(path);
    file << json;
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static shift::core::force_module::ElectricMotor*
FindMotor(shift::core::Context& context) {
  return dynamic_cast<shift::core::force_module::ElectricMotor*>(
    context["assemblies"]["a"]["force_modules"]["motor"]
      .get_data<shift::core::force_module::Base>());
}

/* simulate, checking the rotor's speed at each output time against
expected(t) to within tolerance; returns the final state */
static std::vector<double> Simulate(shift::core::Context& context,
                                    std::function<double(double)> expected,
                                    double tolerance) {
  std::vector<double> y;
  int checked = 0;
  context.set_state_observer(
    [&](double t, const std::vector<double>& state) {
      y = state;
      const double speed = state[context.get_q_count()];
      EXPECT_NEAR(speed, expected(t), tolerance) << "t = " << t;
      checked++;
    });
  context.Simulate();
  EXPECT_GT(checked, 10);
  return y;
}

/* without inductance and load, the rotor's speed rises to v / k with the
time constant J R / k^2: omega = (v / k) (1 - exp(-t / tau)) */
TEST(ElectricMotor, VoltageDriveRisesToTheNoLoadSpeed) {
  const double v = 12.0;
  const double no_load = v / kTorqueConstant;
  const double tau = kInertia * kResistance / (kTorqueConstant *
                                                kTorqueConstant);
  shift::core::Context context;
  Load(context, Model(R"("drive": "voltage", "voltage": 12)", 3.0));
  auto motor = FindMotor(context);
  ASSERT_NE(motor, nullptr);
  EXPECT_EQ(motor->get_equation_count(), 0u);
  Simulate(
    context, [&](double t) { return no_load * (1.0 - std::exp(-t / tau)); },
    1e-5 * no_load);
  /* the power from the supply is the power to the rotor and the loss in the
  winding */
  const double i = motor->GetOutput("winding_current");
  EXPECT_NEAR(motor->GetOutput("electrical_power"),
              motor->GetOutput("mechanical_power") + kResistance * i * i,
              1e-9);
  EXPECT_NEAR(motor->GetOutput("terminal_voltage"), v, 1e-12);
  EXPECT_NEAR(motor->GetOutput("back_voltage"),
              kTorqueConstant * motor->GetOutput("speed"), 1e-12);
  EXPECT_NEAR(motor->GetOutput("torque"), kTorqueConstant * i, 1e-12);
}

/* with inductance, the current is a state and the speed follows the second
order response of J L s^2 + J R s + k^2 to a step of the voltage */
TEST(ElectricMotor, InductanceMakesTheCurrentAState) {
  const double v = 12.0, inductance = 0.05;
  const double no_load = v / kTorqueConstant;
  /* s^2 + (R / L) s + k^2 / (J L), with real roots */
  const double b = kResistance / inductance;
  const double c =
    kTorqueConstant * kTorqueConstant / (kInertia * inductance);
  const double root = std::sqrt(b * b / 4.0 - c);
  const double r1 = -b / 2.0 + root, r2 = -b / 2.0 - root;
  auto omega = [&](double t) {
    return no_load *
           (1.0 + (r2 * std::exp(r1 * t) - r1 * std::exp(r2 * t)) / (r1 - r2));
  };
  shift::core::Context context;
  Load(context, Model(R"("drive": "voltage", "voltage": 12,
                         "inductance": 0.05)",
                      3.0));
  auto motor = FindMotor(context);
  ASSERT_NE(motor, nullptr);
  EXPECT_EQ(motor->get_equation_count(), 1u);
  const std::vector<double> y = Simulate(context, omega, 1e-5 * no_load);
  /* the current, the state after the joint's rate, drives the rotor:
  J d omega / dt = k i */
  const double t = 3.0;
  const double rate =
    no_load * r1 * r2 * (std::exp(r1 * t) - std::exp(r2 * t)) / (r1 - r2);
  EXPECT_NEAR(y[context.get_q_count() + 1], kInertia * rate / kTorqueConstant,
              1e-5);
}

/* the current drive makes the current at once: a constant torque k i, and
a speed that grows as k i t / J */
TEST(ElectricMotor, CurrentDriveGivesAConstantTorque) {
  const double i = 2.0;
  shift::core::Context context;
  Load(context, Model(R"("current": 2)", 3.0));
  Simulate(
    context, [&](double t) { return kTorqueConstant * i * t / kInertia; },
    1e-6);
  EXPECT_NEAR(FindMotor(context)->GetOutput("torque"), kTorqueConstant * i,
              1e-12);
}

/* the current limit clips the input current */
TEST(ElectricMotor, CurrentLimitClipsTheCurrent) {
  shift::core::Context context;
  Load(context, Model(R"("current": 5, "current_limit": 2)", 2.0));
  Simulate(
    context, [&](double t) { return kTorqueConstant * 2.0 * t / kInertia; },
    1e-6);
  EXPECT_NEAR(FindMotor(context)->GetOutput("winding_current"), 2.0, 1e-12);
}

/* with a supply of 12 V the drive holds 2 A only while R i + k omega <=
12 V, up to 100 rad/s at 5 s; past that the current is (12 - k omega) / R,
and the speed approaches 120 rad/s with the time constant of 1 s */
TEST(ElectricMotor, SupplyVoltageLimitsTheCurrentAtSpeed) {
  shift::core::Context context;
  Load(context, Model(R"("current": 2, "supply_voltage": 12)", 8.0));
  auto motor = FindMotor(context);
  ASSERT_NE(motor, nullptr);
  Simulate(
    context,
    [&](double t) {
      return t <= 5.0 ? 20.0 * t : 120.0 - 20.0 * std::exp(-(t - 5.0));
    },
    1e-4);
  EXPECT_NEAR(motor->GetOutput("terminal_voltage"), 12.0, 1e-9);
  EXPECT_LT(motor->GetOutput("winding_current"), 2.0);
}

/* a PI controller on the rotor's speed sets the motor's current input; the
bearing's drag of 0.004 N m s/rad at 50 rad/s takes 0.2 N m, which is 2 A */
TEST(ElectricMotor, PidSetsTheCurrentInput) {
  shift::core::Context context;
  Load(context,
       Model(R"("current_limit": 5, "supply_voltage": 24)", 10.0,
             R"({"type": "pid", "name": "speed_control", "kp": 0.5,
                 "ki": 2.0, "target": 50, "sensor": "speed",
                 "component": "motor", "input": "current"})",
             R"(, "damping_coefficients": [0, 0.004])"));
  auto motor = FindMotor(context);
  ASSERT_NE(motor, nullptr);
  EXPECT_EQ((*motor)["current"].get_driver(), "a__speed_control");
  std::vector<double> y;
  context.set_state_observer(
    [&](double, const std::vector<double>& state) { y = state; });
  context.Simulate();
  ASSERT_FALSE(y.empty());
  EXPECT_NEAR(y[context.get_q_count()], 50.0, 1e-3);
  EXPECT_NEAR(motor->GetOutput("winding_current"), 2.0, 1e-3);
}

/* the message of the error that loading the model raises */
static std::string LoadError(const std::string& motor,
                             const std::string& controllers = "") {
  shift::core::Context context;
  try {
    Load(context, Model(motor, 1.0, controllers));
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

TEST(ElectricMotor, OnlyTheInputOfTheDriveIsUsed) {
  const std::string given = LoadError(R"("voltage": 12)");
  EXPECT_NE(given.find("the drive is \"current\", so the input 'voltage' is "
                       "not used; set 'current' instead"),
            std::string::npos)
    << given;
  const std::string set = LoadError(
    "", R"({"type": "pid", "name": "speed_control", "kp": 1,
            "sensor": "speed", "component": "motor", "input": "voltage"})");
  EXPECT_NE(set.find("'voltage' is not used (a__speed_control sets it)"),
            std::string::npos)
    << set;
  const std::string current =
    LoadError(R"("drive": "voltage", "current": 1)");
  EXPECT_NE(current.find("so the input 'current' is not used"),
            std::string::npos)
    << current;
}

TEST(ElectricMotor, RejectsBadParameters) {
  EXPECT_NE(LoadError(R"("drive": "pwm")").find("drive 'pwm' is not"),
            std::string::npos);
  EXPECT_NE(LoadError(R"("inductance": -1)").find("must not be negative"),
            std::string::npos);
  EXPECT_NE(
    LoadError(R"("speed": 1)").find("'speed' is an output"),
    std::string::npos);
}
