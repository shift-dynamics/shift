/* The powertrain force module (force_module::Powertrain): the torque of the
engine and its governor, the torque converter, a locked-up drive of wheels
through a gear, the ratio after a shift, the three kinds of differential, the
brakes and what the module refuses */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/force_module/powertrain.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::Powertrain;

static const char* kEngine =
  R"("inertia": 0.5, "idle_speed": 80, "high_idle_speed": 230,
     "torque_curve": [80, 500, 150, 900, 200, 800, 230, 0],
     "friction_curve": [80, -60, 230, -150])";
static const char* kConverter =
  R"("type": "converter", "capacity": 1500, "slip_speed": 1.0,
     "capacity_curve": [0, 10, 0.8, 13, 0.95, 30, 1.0, 300],
     "torque_ratio_curve": [0, 2.2, 0.85, 1.0, 1.0, 1.0])";

/* a drive and the two wheels of an axle: flywheels about the z axis that the
drive turns forward, with a drag torque in proportion to the speed (the
road load) */
struct Vehicle {
  std::string engine = kEngine;
  std::string coupling = kConverter;
  std::string transmission =
    R"("ratios": [2.0, 1.0], "reverse_ratio": -2.0, "gear": 1,
       "clutch_capacity": 1500, "slip_speed": 1.0, "input_inertia": 0.2,
       "output_inertia": 0.2, "shift_time": 0.2)";
  std::string axles =
    R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "final_drive": 4.0, "shaft_stiffness": 5e3, "shaft_damping": 50})";
  std::string brakes = "";
  std::string controllers = "";
  double drag_l = 40.0, drag_r = 40.0;
  double wheel_inertia = 1.0;
  double speed_l = 0.0, speed_r = 0.0;
  double time_span = 1.0;
  std::string extra_wheels = "";
  std::string joint_extra_l = ""; //!< e.g. a constant torque on the left wheel
  std::string module_extra = ""; //!< more members of the force module
};

static void Load(shift::core::Context& context, const Vehicle& v) {
  const std::string path = testing::TempDir() + "/powertrain_test.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-7,
                     "absolute_error": 1e-9, "time_span": )"
         << v.time_span << R"(, "time_step": 0.001}]
  },
  "assemblies": [{"name": "vehicle",
    "bodies": [)";
    for(const char* name : {"wheel_l", "wheel_r"}) {
      file << R"({"name": ")" << name
           << R"(", "center_of_mass": [0, 0, 0],
        "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": )"
           << v.wheel_inertia << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}})"
           << (std::string(name) == "wheel_l" ? "," : "");
    }
    file << R"(
    ],
    "joints": [
      {"name": "wheel_l", "type": "revolute", "parent": [["fixed"], "origin"],
       "child": [["wheel_l"], "origin"], "axis": [0, 0, 1],
       "initial_velocity": )"
         << v.speed_l << R"(, "damping_coefficients": [0, )" << v.drag_l
         << R"(])" << v.joint_extra_l << R"(},
      {"name": "wheel_r", "type": "revolute", "parent": [["fixed"], "origin"],
       "child": [["wheel_r"], "origin"], "axis": [0, 0, 1],
       "initial_velocity": )"
         << v.speed_r << R"(, "damping_coefficients": [0, )" << v.drag_r << R"(]}
    ],)"
         << (v.controllers.empty() ? ""
                                   : R"("controllers": [)" + v.controllers +
                                       "],")
         << R"(
    "force_modules": [{"type": "powertrain", "name": "drive",
      "engine": {)" << v.engine << R"(},
      "coupling": {)" << v.coupling << R"(},
      "transmission": {)" << v.transmission << R"(},
      "axles": [)" << v.axles << R"(])"
         << (v.brakes.empty() ? "" : R"(, "brakes": [)" + v.brakes + "]")
         << v.module_extra
         << R"(}]
  }]
})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static Powertrain* Find(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto drive = dynamic_cast<Powertrain*>(module)) {
      return drive;
    }
  }
  return nullptr;
}

/* an output of the module by name, at the last evaluation */
static double Output(Powertrain* drive, const std::string& name) {
  const auto names = drive->get_output_names();
  const auto& values = drive->get_output_values();
  for(size_t i = 0; i < names.size(); i++) {
    if(names[i] == name) {
      return values.at(i);
    }
  }
  ADD_FAILURE() << "no output " << name;
  return 0.0;
}

/* simulate, then evaluate the outputs at the final state */
static std::vector<double> Simulate(shift::core::Context& context) {
  std::vector<double> state;
  context.set_state_observer(
    [&state](double, const std::vector<double>& y) { state = y; });
  context.Simulate();
  Find(context)->UpdateOutputs(0.0, state.data());
  return state;
}

static double WheelSpeed(shift::core::Context& context,
                         const std::vector<double>& state, const char* name) {
  return state[context.get_q_count() +
               context["assemblies"]["vehicle"]["joints"][name]
                 .get_data<shift::core::joint::Base>()
                 ->get_dq_index()];
}

TEST(Powertrain, TheEngineTorqueAndItsGovernor) {
  shift::core::Context context;
  Load(context, Vehicle());
  Powertrain* drive = Find(context);
  ASSERT_NE(drive, nullptr);
  double fuel;
  // full throttle: the torque curve
  EXPECT_NEAR(drive->EngineTorque(150.0, 1.0, &fuel), 900.0, 1e-9);
  EXPECT_NEAR(drive->EngineTorque(200.0, 1.0), 800.0, 1e-9);
  EXPECT_NEAR(fuel, 1.0, 1e-12);
  // closed throttle above idle: the friction and pumping of the engine
  EXPECT_NEAR(drive->EngineTorque(150.0, 0.0, &fuel),
              drive->EngineTorque(150.0, 0.0), 1e-12);
  EXPECT_LT(drive->EngineTorque(150.0, 0.0, &fuel), -60.0);
  EXPECT_EQ(fuel, 0.0);
  // half throttle is half way between
  EXPECT_NEAR(drive->EngineTorque(150.0, 0.5),
              0.5 * (drive->EngineTorque(150.0, 1.0) +
                     drive->EngineTorque(150.0, 0.0)),
              1e-9);
  // below idle the governor adds fuel: 0.1 per rad/s under 80
  drive->EngineTorque(75.0, 0.0, &fuel);
  EXPECT_NEAR(fuel, 0.5, 1e-12);
  drive->EngineTorque(60.0, 0.0, &fuel);
  EXPECT_EQ(fuel, 1.0);
  // above the high idle speed the fuel is cut, smoothly over 5 % of it
  const double cut = 0.05 * 230.0;
  drive->EngineTorque(230.0 - cut, 1.0, &fuel);
  EXPECT_NEAR(fuel, 1.0, 1e-12);
  drive->EngineTorque(230.0, 1.0, &fuel);
  EXPECT_EQ(fuel, 0.0);
  double last = 1.0;
  for(double w = 230.0 - cut; w <= 230.0; w += 0.05) {
    drive->EngineTorque(w, 1.0, &fuel);
    EXPECT_LE(fuel, last + 1e-12);
    last = fuel;
  }
}

TEST(Powertrain, TheTorqueConverterGivesTheTorqueOfItsCurves) {
  shift::core::Context context;
  Vehicle v;
  v.coupling = R"("type": "converter", "capacity": 1500, "engagement": 0,
     "capacity_curve": [0, 10, 0.8, 13, 0.95, 30, 1.0, 300],
     "torque_ratio_curve": [0, 2.2, 0.85, 1.0, 1.0, 1.0])";
  Load(context, v);
  Powertrain* drive = Find(context);
  ASSERT_NE(drive, nullptr);
  double pump, turbine, ratio;
  // stalled turbine: K(0) = 10 rad/s per sqrt(N m): the pump takes
  // (w / K)^2 and the turbine gives 2.2 times it
  drive->CouplingTorques(200.0, 0.0, &pump, &turbine, &ratio);
  EXPECT_NEAR(pump, 400.0, 1e-9);
  EXPECT_NEAR(ratio, 2.2, 1e-12);
  EXPECT_NEAR(turbine, 880.0, 1e-9);
  // the converter cannot give more power than it takes
  for(double sr = 0.0; sr <= 1.0; sr += 0.02) {
    drive->CouplingTorques(180.0, 180.0 * sr, &pump, &turbine);
    EXPECT_GE(pump * 180.0 - turbine * 180.0 * sr, -1e-9) << sr;
    EXPECT_GE(pump, 0.0);
  }
  // at the coupling point and beyond the torques are equal; with the
  // turbine as fast as the pump nothing is transmitted (K is large)
  drive->CouplingTorques(200.0, 0.9 * 200.0, &pump, &turbine, &ratio);
  EXPECT_NEAR(ratio, 1.0, 1e-9);
  EXPECT_NEAR(pump, turbine, 1e-9);
  drive->CouplingTorques(200.0, 200.0, &pump, &turbine);
  EXPECT_LT(std::fabs(pump), 0.6);
  // engaged, the lockup clutch adds capacity tanh(slip / 1 rad/s) between them
  shift::core::Context locked_context;
  Vehicle locked;
  Load(locked_context, locked);
  Powertrain* lock = Find(locked_context);
  double lock_pump, lock_turbine, free_pump, free_turbine;
  lock->CouplingTorques(200.0, 150.0, &lock_pump, &lock_turbine);
  drive->CouplingTorques(200.0, 150.0, &free_pump, &free_turbine);
  EXPECT_NEAR(lock_pump - free_pump, 1500.0, 1e-6);
  EXPECT_NEAR(lock_turbine - free_turbine, 1500.0, 1e-6);
  lock->CouplingTorques(150.0, 200.0, &lock_pump, &lock_turbine);
  EXPECT_LT(lock_pump, 0.0); // the turbine drives the engine
}

TEST(Powertrain, AnIdlingEngineHoldsItsIdleAndRunsUpToTheHighIdleSpeed) {
  // neutral, no load on the converter's turbine: the engine runs at its idle
  // speed (the governor's droop is small), and at full throttle up to the
  // speed where its fuel is cut
  {
    shift::core::Context context;
    Vehicle v;
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 0, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
    v.time_span = 2.0;
    Load(context, v);
    const auto state = Simulate(context);
    Powertrain* drive = Find(context);
    // the governor adds the fuel f that cancels the friction of the engine at
    // idle, f 500 - (1 - f) 60 = 0, at 0.1 per rad/s under the idle speed
    const double fuel = 60.0 / (500.0 + 60.0);
    EXPECT_NEAR(Output(drive, "engine_speed"), 80.0 - fuel / 0.1, 0.02);
    EXPECT_NEAR(Output(drive, "fuel"), fuel, 1e-3);
  }
  {
    shift::core::Context context;
    Vehicle v;
    v.engine = std::string(kEngine);
    v.engine.replace(v.engine.find("\"idle_speed\""), 0,
                     R"("throttle": 1.0, )");
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 0, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
    v.time_span = 2.0;
    Load(context, v);
    Simulate(context);
    Powertrain* drive = Find(context);
    EXPECT_GT(Output(drive, "engine_speed"), 230.0 - 0.05 * 230.0);
    EXPECT_LT(Output(drive, "engine_speed"), 230.0 + 1e-6);
  }
}

/* wheels driven through gear 2 (ratio 1, final drive 4) with the converter
locked up: the engine, the turbine and the wheels turn together in the ratio of
the gear, and the engine's torque is the drag of the wheels in that ratio */
TEST(Powertrain, ALockedUpDriveTurnsTheWheelsInTheRatioOfTheGear) {
  shift::core::Context context;
  Vehicle v;
  v.transmission =
    R"("ratios": [2.0, 1.0], "reverse_ratio": -2.0, "gear": 2,
       "clutch_capacity": 1500, "slip_speed": 1.0, "input_inertia": 0.2,
       "output_inertia": 0.2, "shift_time": 0.2)";
  v.engine = std::string(kEngine);
  v.engine.replace(v.engine.find("\"idle_speed\""), 0, R"("throttle": 1.0, )");
  v.speed_l = v.speed_r = 30.0;
  v.time_span = 4.0;
  Load(context, v);
  const auto state = Simulate(context);
  Powertrain* drive = Find(context);
  const double ratio = 1.0 * 4.0; // gear 2 and the final drive
  const double w_l = WheelSpeed(context, state, "wheel_l");
  const double w_r = WheelSpeed(context, state, "wheel_r");
  const double w_e = Output(drive, "engine_speed");
  EXPECT_NEAR(w_l, w_r, 1e-3 * w_l); // an open differential, equal drag
  EXPECT_NEAR(w_e / w_l, ratio, 0.02 * ratio);
  EXPECT_NEAR(Output(drive, "turbine_speed"), w_e, 0.01 * w_e);
  EXPECT_NEAR(Output(drive, "gear_ratio"), 1.0, 1e-6);
  // the engine's torque times the ratio is the drag of the two wheels (no
  // spin loss and the speeds steady)
  const double drag = 40.0 * (w_l + w_r);
  EXPECT_NEAR(Output(drive, "engine_torque") * ratio, drag, 0.03 * drag);
  // an open differential: half the carrier's torque to each wheel
  EXPECT_NEAR(Output(drive, "rear_left_torque"),
              Output(drive, "rear_right_torque"),
              1e-6 * std::fabs(Output(drive, "rear_left_torque")));
  EXPECT_NEAR(Output(drive, "rear_left_torque") * 2.0, 4.0 *
                Output(drive, "rear_shaft_torque"),
              1e-9 * 4.0 * std::fabs(Output(drive, "rear_shaft_torque")));
}

TEST(Powertrain, AShiftChangesTheRatioOfTheDrive) {
  // gear 1 (ratio 2) to gear 2 (ratio 1) at 3 s, by a schedule controller
  shift::core::Context context;
  Vehicle v;
  v.engine = std::string(kEngine);
  v.engine.replace(v.engine.find("\"idle_speed\""), 0, R"("throttle": 1.0, )");
  v.controllers = R"({"type": "schedule", "name": "shift",
    "interpolation": "step", "points": [0, 1, 3, 2],
    "targets": ["force_modules/drive/transmission/gear"]})";
  v.speed_l = v.speed_r = 27.0;
  v.time_span = 7.0;
  v.drag_l = v.drag_r = 40.0;
  Load(context, v);
  std::vector<double> state;
  double early_ratio = 0.0, early_engine = 0.0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    state = y;
    if(t < 2.99 && t > 2.9) {
      Powertrain* drive = Find(context);
      drive->UpdateOutputs(t, y.data());
      early_ratio = Output(drive, "engine_speed") /
                    WheelSpeed(context, y, "wheel_l");
      early_engine = Output(drive, "engine_speed");
    }
  });
  context.Simulate();
  Powertrain* drive = Find(context);
  drive->UpdateOutputs(0.0, state.data());
  EXPECT_NEAR(early_ratio, 8.0, 0.02 * 8.0);
  EXPECT_GT(early_engine, 150.0);
  const double w = WheelSpeed(context, state, "wheel_l");
  EXPECT_NEAR(Output(drive, "engine_speed") / w, 4.0, 0.02 * 4.0);
  EXPECT_NEAR(Output(drive, "gear_ratio"), 1.0, 1e-3);
  EXPECT_NEAR(Output(drive, "engine_torque") * 4.0, 40.0 * 2.0 * w,
              0.03 * 40.0 * 2.0 * w);
}

TEST(Powertrain, TheDifferentialsDivideTheTorqueOfTheCarrier) {
  // the left wheel has little drag, the right much: an open differential gives
  // both the same torque, so the left wheel spins faster; a locked one holds
  // both to one speed and gives the torque to the wheel that needs it; a
  // limited slip one shifts a bias torque to the wheel that is slower
  auto run = [&](const std::string& differential, const std::string& extra,
                 double* left_speed, double* right_speed) {
    shift::core::Context context;
    Vehicle v;
    v.engine = std::string(kEngine);
    v.engine.replace(v.engine.find("\"idle_speed\""), 0,
                     R"("throttle": 0.6, )");
    v.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "final_drive": 4.0, "shaft_stiffness": 5e3, "shaft_damping": 50,
        "differential": ")" + differential + "\"" + extra + "}";
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 2, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
    v.drag_l = 10.0;
    v.drag_r = 150.0;
    v.time_span = 3.0;
    Load(context, v);
    const auto state = Simulate(context);
    Powertrain* drive = Find(context);
    *left_speed = WheelSpeed(context, state, "wheel_l");
    *right_speed = WheelSpeed(context, state, "wheel_r");
    return std::pair<double, double>{Output(drive, "rear_left_torque"),
                                     Output(drive, "rear_right_torque")};
  };
  double w_l, w_r;
  auto open = run("open", "", &w_l, &w_r);
  EXPECT_NEAR(open.first, open.second, 1e-6 * std::fabs(open.first));
  EXPECT_GT(w_l, 3.0 * w_r); // the free wheel spins up
  auto locked = run("locked", R"(, "lock_damping": 1e4)", &w_l, &w_r);
  EXPECT_NEAR(w_l, w_r, 0.02 * w_l);
  // the torque follows the drag: 10 w and 150 w
  EXPECT_NEAR(locked.second / locked.first, 150.0 / 10.0,
              0.15 * 150.0 / 10.0);
  // a limited slip differential with 60 N m of preload: the slower wheel gets
  // twice the bias more than the faster one
  auto limited = run("limited_slip", R"(, "bias_torque": 60)", &w_l, &w_r);
  EXPECT_GT(w_l, w_r);
  EXPECT_NEAR(limited.second - limited.first, 2.0 * 60.0, 2.0);
  // with torque bias in proportion to the carrier's, the difference is that
  // fraction of it (both wheels slipping apart: tanh saturated)
  auto biased = run("limited_slip", R"(, "bias_ratio": 0.4)", &w_l, &w_r);
  EXPECT_NEAR(biased.second - biased.first,
              2.0 * 0.4 * (biased.first + biased.second), 1.0);
}

TEST(Powertrain, ABrakeStopsAWheelAtItsCapacity) {
  // a pressure brake: with 80 N m on a wheel of inertia 1, the speed falls by
  // 80 rad/s each second (its engagement tanh(w / 0.1) is 1 at speed)
  for(const std::string applied_by : {"pressure", "spring"}) {
    shift::core::Context context;
    Vehicle v;
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 0, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
    v.drag_l = v.drag_r = 0.0;
    v.speed_l = 20.0;
    v.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "final_drive": 4.0, "shaft_stiffness": 5e3, "engaged": 0})";
    // applied by pressure at 1; by spring, released by a pressure of 0 (so it
    // is applied)
    v.brakes = R"({"name": "left", "joint": "wheel_l", "capacity": 80,
                  "pressure": )" +
               std::string(applied_by == "pressure" ? "1" : "0") +
               R"(, "applied_by": ")" + applied_by + R"("})";
    v.time_span = 0.2;
    Load(context, v);
    const auto state = Simulate(context);
    EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"), 20.0 - 80.0 * 0.2,
                0.05)
      << applied_by;
  }
  // released: it keeps its speed (the spring brake with pressure 1, the
  // pressure brake with 0)
  for(const std::string applied_by : {"pressure", "spring"}) {
    shift::core::Context context;
    Vehicle v;
    v.drag_l = v.drag_r = 0.0;
    v.speed_l = 20.0;
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 0, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
    v.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "final_drive": 4.0, "shaft_stiffness": 5e3, "engaged": 0})";
    v.brakes = R"({"name": "left", "joint": "wheel_l", "capacity": 80,
                  "pressure": )" +
               std::string(applied_by == "pressure" ? "0" : "1") +
               R"(, "applied_by": ")" + applied_by + R"("})";
    v.time_span = 0.2;
    Load(context, v);
    const auto state = Simulate(context);
    EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"), 20.0, 1e-6)
      << applied_by;
  }
}

TEST(Powertrain, ReverseTurnsTheWheelsTheOtherWay) {
  shift::core::Context context;
  Vehicle v;
  v.engine = std::string(kEngine);
  v.engine.replace(v.engine.find("\"idle_speed\""), 0, R"("throttle": 1.0, )");
  v.transmission =
    R"("ratios": [2.0, 1.0], "reverse_ratio": -2.0, "gear": -1,
       "clutch_capacity": 1500, "slip_speed": 1.0, "input_inertia": 0.2,
       "output_inertia": 0.2)";
  v.speed_l = v.speed_r = -27.0;
  v.time_span = 4.0;
  Load(context, v);
  const auto state = Simulate(context);
  Powertrain* drive = Find(context);
  const double w = WheelSpeed(context, state, "wheel_l");
  EXPECT_LT(w, -10.0);
  EXPECT_NEAR(Output(drive, "engine_speed") / -w, 8.0, 0.02 * 8.0);
  EXPECT_NEAR(Output(drive, "gear_ratio"), -2.0, 1e-3);
}

TEST(Powertrain, AnAxleThatIsNotEngagedTakesNoTorque) {
  shift::core::Context context;
  Vehicle v;
  v.engine = std::string(kEngine);
  v.engine.replace(v.engine.find("\"idle_speed\""), 0, R"("throttle": 1.0, )");
  v.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
      "final_drive": 4.0, "shaft_stiffness": 5e3, "shaft_damping": 50,
      "engaged": 0})";
  v.time_span = 1.0;
  Load(context, v);
  const auto state = Simulate(context);
  Powertrain* drive = Find(context);
  EXPECT_EQ(Output(drive, "rear_left_torque"), 0.0);
  EXPECT_EQ(Output(drive, "rear_right_torque"), 0.0);
  EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"), 0.0, 1e-9);
}

TEST(Powertrain, WhatItRefuses) {
  auto message = [&](const Vehicle& v) {
    shift::core::Context context;
    try {
      Load(context, v);
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_EQ(message(Vehicle()), "");
  Vehicle gain;
  gain.coupling = R"("type": "converter", "capacity": 100,
     "capacity_curve": [0, 10, 1, 12],
     "torque_ratio_curve": [0, 2.2, 0.5, 2.5, 1.0, 1.0])";
  EXPECT_NE(message(gain).find("efficiency"), std::string::npos);
  Vehicle gear;
  gear.transmission =
    R"("ratios": [2.0, 1.0], "gear": 3, "clutch_capacity": 1500)";
  EXPECT_NE(message(gear).find("gear is not a gear"), std::string::npos);
  Vehicle reverse;
  reverse.transmission =
    R"("ratios": [2.0, 1.0], "gear": -1, "clutch_capacity": 1500)";
  EXPECT_NE(message(reverse).find("gear is not a gear"), std::string::npos);
  Vehicle idle;
  idle.engine = R"("inertia": 0.5, "idle_speed": 80, "high_idle_speed": 60,
     "torque_curve": [80, 500, 150, 900])";
  EXPECT_NE(message(idle).find("high_idle_speed"), std::string::npos);
  Vehicle differential;
  differential.axles =
    R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "shaft_stiffness": 5e3, "differential": "sideways"})";
  EXPECT_NE(message(differential).find("differential"), std::string::npos);
  Vehicle joint;
  joint.axles = R"({"name": "rear", "left": "wheel_l", "right": "nothing",
        "shaft_stiffness": 5e3})";
  EXPECT_NE(message(joint).find("'nothing'"), std::string::npos);
}

/* a wheel pushed by a constant torque of 20 N m (a joint's constant stiffness
term) against a brake of 80 N m, with no drive */
static Vehicle PushedWheel(const std::string& brake_extra) {
  Vehicle v;
  v.transmission =
    R"("ratios": [2.0, 1.0], "gear": 0, "clutch_capacity": 1500,
       "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
  v.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
      "final_drive": 4.0, "shaft_stiffness": 5e3, "engaged": 0})";
  v.drag_l = v.drag_r = 0.0;
  v.joint_extra_l = R"(, "stiffness_coefficients": [-20])";
  v.brakes = R"({"name": "left", "joint": "wheel_l", "capacity": 80,
                "pressure": 1)" +
             brake_extra + "}";
  v.time_span = 0.6;
  return v;
}

TEST(PowertrainBristles, ABrakeWithBristlesHoldsWhereSmoothedFrictionCreeps) {
  // smoothed friction gives the torque of the load, 20 of 80 N m, only at a
  // speed: slip_speed atanh(0.25) = 0.0255 rad/s
  {
    shift::core::Context context;
    Load(context, PushedWheel(""));
    const auto state = Simulate(context);
    EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"),
                0.1 * std::atanh(0.25), 1e-4);
  }
  // bristles hold: the deflection that carries the load is z = 20 / (80
  // sigma_0), and the wheel has turned by more, as the bristles slip a little
  // as they deflect (dz/dt = u (1 - sigma_0 z / g)): by -(1 / sigma_0) ln(1 -
  // 20 / 80), the pre-sliding displacement of the LuGre model
  {
    shift::core::Context context;
    Load(context, PushedWheel(R"(, "bristle_stiffness": 1000,
                               "bristle_damping": 7)"));
    const auto state = Simulate(context);
    EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"), 0.0, 1e-6);
    const double angle =
      state[context["assemblies"]["vehicle"]["joints"]["wheel_l"]
              .get_data<shift::core::joint::Base>()
              ->get_q_index()];
    const double expected = -std::log(1.0 - 20.0 / 80.0) / 1000.0;
    EXPECT_NEAR(angle, expected, 0.01 * expected);
  }
}

TEST(PowertrainBristles, StaticFrictionAboveTheCapacityHoldsALoadBeyondIt) {
  // 90 N m against a brake of 80 N m: it slides; with a static friction of
  // 1.2 times (96 N m) it holds
  Vehicle v = PushedWheel(R"(, "bristle_stiffness": 2000,
                            "bristle_damping": 7, "stribeck_speed": 5)");
  v.joint_extra_l = R"(, "stiffness_coefficients": [-90])";
  {
    shift::core::Context context;
    Load(context, v);
    const auto state = Simulate(context);
    EXPECT_GT(WheelSpeed(context, state, "wheel_l"), 3.0); // a = 10 rad/s^2
  }
  {
    Vehicle holding = v;
    holding.brakes = R"({"name": "left", "joint": "wheel_l", "capacity": 80,
      "pressure": 1, "bristle_stiffness": 2000, "bristle_damping": 7,
      "stribeck_speed": 5, "static_ratio": 1.2})";
    shift::core::Context context;
    Load(context, holding);
    const auto state = Simulate(context);
    EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"), 0.0, 1e-5);
  }
}

TEST(PowertrainBristles, ALockedClutchDoesNotSlip) {
  // gear 2 (ratio 1) with the lockup engaged, driven to a steady speed: the
  // smoothed friction needs a slip for the torque, the bristles do not
  auto slips = [&](bool bristles) {
    shift::core::Context context;
    Vehicle v;
    v.engine = std::string(kEngine);
    v.engine.replace(v.engine.find("\"idle_speed\""), 0,
                     R"("throttle": 1.0, )");
    const std::string extra =
      bristles ? R"(, "bristle_stiffness": 500, "bristle_damping": 0.5)" : "";
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 2, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2,
         "shift_time": 0.2)" + extra;
    v.coupling = std::string(kConverter) + extra;
    v.speed_l = v.speed_r = 40.0;
    v.time_span = 3.0;
    Load(context, v);
    Simulate(context);
    Powertrain* drive = Find(context);
    return std::pair<double, double>{
      Output(drive, "turbine_speed") - Output(drive, "output_speed"),
      Output(drive, "engine_speed") - Output(drive, "turbine_speed")};
  };
  const auto smoothed = slips(false);
  const auto held = slips(true);
  EXPECT_GT(smoothed.first, 0.05);
  EXPECT_GT(smoothed.second, 0.05);
  EXPECT_NEAR(held.first, 0.0, 2e-3);
  EXPECT_NEAR(held.second, 0.0, 2e-3);
}

TEST(PowertrainBristles, ALimitedSlipDifferentialWithBristlesLocks) {
  // wheels with drags of 40 and 50 and a preload of 200 N m: the clutches
  // carry the difference of torque, which smoothed friction does with a slip
  // between the wheels, and bristles without
  auto slip = [&](bool bristles) {
    shift::core::Context context;
    Vehicle v;
    v.engine = std::string(kEngine);
    v.engine.replace(v.engine.find("\"idle_speed\""), 0,
                     R"("throttle": 0.6, )");
    v.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "final_drive": 4.0, "shaft_stiffness": 5e3, "shaft_damping": 50,
        "differential": "limited_slip", "bias_torque": 200)" +
              std::string(bristles ? R"(, "bristle_stiffness": 500,
                                      "bristle_damping": 0.5)"
                                   : "") +
              "}";
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 2, "clutch_capacity": 1500,
         "slip_speed": 1.0, "input_inertia": 0.2, "output_inertia": 0.2)";
    v.drag_l = 40.0;
    v.drag_r = 50.0;
    v.speed_l = v.speed_r = 20.0;
    v.time_span = 3.0;
    Load(context, v);
    const auto state = Simulate(context);
    return WheelSpeed(context, state, "wheel_l") -
           WheelSpeed(context, state, "wheel_r");
  };
  EXPECT_GT(std::fabs(slip(false)), 0.05);
  EXPECT_NEAR(slip(true), 0.0, 2e-3);
}

TEST(PowertrainBristles, WhatItRefuses) {
  auto message = [&](const std::string& brake_extra) {
    shift::core::Context context;
    try {
      Load(context, PushedWheel(brake_extra));
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_NE(message(R"(, "bristle_stiffness": -1)").find("bristle_stiffness"),
            std::string::npos);
  EXPECT_NE(message(R"(, "bristle_stiffness": 100, "static_ratio": 0.9)")
              .find("static_ratio"),
            std::string::npos);
  EXPECT_NE(message(R"(, "bristle_stiffness": 100, "stribeck_speed": 0)")
              .find("stribeck_speed"),
            std::string::npos);
  EXPECT_EQ(message(R"(, "bristle_stiffness": 100)"), "");
}

/* a drive that settles to a steady state: a fixed throttle, the wheels driven
against their drag */
static Vehicle SteadyVehicle() {
  Vehicle v;
  v.engine = std::string(kEngine);
  v.engine.replace(v.engine.find("\"idle_speed\""), 0, R"("throttle": 0.6, )");
  v.speed_l = v.speed_r = 25.0;
  v.time_span = 6.0;
  return v;
}

TEST(PowertrainLosses, TheTeethLoseTheirShareOfThePowerTheyCarry) {
  shift::core::Context context;
  Vehicle v = SteadyVehicle();
  v.transmission =
    R"("ratios": [2.0, 1.0], "gear": 1, "efficiencies": [0.9, 0.95],
       "clutch_capacity": 1500, "slip_speed": 1.0, "input_inertia": 0.2,
       "output_inertia": 0.2)";
  v.axles =
    R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
        "final_drive": 4.0, "efficiency": 0.8, "drag_torque": 5,
        "shaft_stiffness": 5e3, "shaft_damping": 50})";
  Load(context, v);
  const auto state = Simulate(context);
  Powertrain* drive = Find(context);
  const double shaft = Output(drive, "rear_shaft_torque");
  const double loss = Output(drive, "gear_loss");
  ASSERT_GT(shaft, 10.0);
  // of the torque r tau through the gear, the teeth lose 10 % and the rest
  // goes down the propshaft
  EXPECT_NEAR(loss / (loss + shaft), 0.1, 1e-3);
  // the final drive passes on 80 % of its torque, less its drag
  EXPECT_NEAR(Output(drive, "rear_left_torque") +
                Output(drive, "rear_right_torque"),
              0.8 * 4.0 * shaft - 5.0, 1e-2 * shaft);
  // and that is what the wheels' drag takes
  const double w = WheelSpeed(context, state, "wheel_l");
  EXPECT_NEAR(Output(drive, "rear_left_torque"), 40.0 * w, 0.01 * 40.0 * w);
}

TEST(PowertrainLosses, ADragTorqueOpposesTheOutputInEitherDirection) {
  // a coasting output with no drive: in neutral, the wheels turn the output
  // through the propshaft, and the drag slows them
  for(double direction : {1.0, -1.0}) {
    shift::core::Context context;
    Vehicle v;
    v.transmission =
      R"("ratios": [2.0, 1.0], "gear": 0, "drag_torque": 10,
         "clutch_capacity": 1500, "slip_speed": 1.0, "input_inertia": 0.2,
         "output_inertia": 0.2)";
    v.speed_l = v.speed_r = direction * 20.0;
    v.drag_l = v.drag_r = 0.0;
    v.time_span = 0.5;
    Load(context, v);
    const auto state = Simulate(context);
    // 10 N m at the output is 40 N m at the wheels, 20 on each of unit inertia
    const double w = WheelSpeed(context, state, "wheel_l");
    EXPECT_LT(std::fabs(w), 20.0 - 4.0);
    EXPECT_GT(direction * w, 0.0);
    EXPECT_NEAR(Output(Find(context), "gear_loss"),
                direction * 10.0, 0.1);
  }
}

/* each output of a center differential drives one wheel, on both sides of
its axle */
static Vehicle CenterVehicle(const std::string& center) {
  Vehicle v = SteadyVehicle();
  v.axles =
    R"({"name": "front", "left": "wheel_l", "right": "wheel_l",
        "center_output": 1, "final_drive": 4.0, "shaft_stiffness": 5e3,
        "shaft_damping": 50},
       {"name": "rear", "left": "wheel_r", "right": "wheel_r",
        "center_output": 2, "final_drive": 4.0, "shaft_stiffness": 5e3,
        "shaft_damping": 50})";
  v.module_extra = R"(, "center_differential": {)" + center + "}";
  v.drag_l = 40.0;
  v.drag_r = 90.0;
  v.time_span = 8.0;
  return v;
}

TEST(PowertrainCenterDifferential, AnOpenOneDividesTheTorqueInItsRatio) {
  shift::core::Context context;
  Load(context, CenterVehicle(R"("split": 0.4, "inertia": 0.05)"));
  const auto state = Simulate(context);
  Powertrain* drive = Find(context);
  const double front = Output(drive, "front_shaft_torque");
  const double rear = Output(drive, "rear_shaft_torque");
  ASSERT_GT(front, 10.0);
  EXPECT_NEAR(front / (front + rear), 0.4, 1e-3);
  // whatever the speeds of the axles, which differ: each wheel runs where the
  // drag takes its share of the torque
  const double w_f = WheelSpeed(context, state, "wheel_l");
  const double w_r = WheelSpeed(context, state, "wheel_r");
  EXPECT_NEAR(w_f, 4.0 * front / 40.0, 0.01 * w_f);
  EXPECT_NEAR(w_r, 4.0 * rear / 90.0, 0.01 * w_r);
  // the weighted mean of the outputs' speeds is the speed of the carrier
  EXPECT_NEAR(0.4 * 4.0 * w_f + 0.6 * 4.0 * w_r,
              Output(drive, "output_speed"),
              1e-3 * Output(drive, "output_speed"));
  EXPECT_NEAR(4.0 * w_f - 4.0 * w_r, Output(drive, "center_speed"),
              1e-3 * std::fabs(Output(drive, "center_speed")));
}

TEST(PowertrainCenterDifferential, LockedOrLimitedSlipItHoldsTheAxlesTogether) {
  for(const char* center :
      {R"("split": 0.4, "type": "locked", "lock_damping": 3000)",
       R"("split": 0.4, "type": "limited_slip", "bias_torque": 500,
          "slip_speed": 0.2)"}) {
    shift::core::Context context;
    Load(context, CenterVehicle(center));
    const auto state = Simulate(context);
    Powertrain* drive = Find(context);
    EXPECT_NEAR(WheelSpeed(context, state, "wheel_l"),
                WheelSpeed(context, state, "wheel_r"),
                0.02 * WheelSpeed(context, state, "wheel_l"))
      << center;
    // no longer in the ratio: the wheel with more drag takes more
    EXPECT_GT(Output(drive, "rear_shaft_torque"),
              1.8 * Output(drive, "front_shaft_torque"))
      << center;
  }
}

static const char* kHydraulics =
  R"(, "clutch_hydraulics": {"supply_pressure": 2e6, "piston_area": 0.01,
     "piston_stroke": 0.002, "spring_preload": 2000, "orifice_area": 1e-6,
     "drain_orifice_area": 4e-6, "discharge_coefficient": 0.7, "density": 850,
     "transition_pressure": 1e3})";

TEST(PowertrainClutchHydraulics, ThePressureOfAClutchFollowsItsVolume) {
  shift::core::Context context;
  Vehicle v;
  v.module_extra =
    R"(, "clutch_hydraulics": {"supply_pressure": 2e6, "piston_area": 0.01,
       "piston_stroke": 0.002, "spring_preload": 2000, "spring_rate": 1e5,
       "orifice_area": 1e-6, "bulk_modulus": 1e9, "chamber_volume": 1e-4})";
  Load(context, v);
  Powertrain* drive = Find(context);
  const double fill = 0.01 * 0.002;
  const double kiss = (2000.0 + 1e5 * 0.002) / 0.01;
  // on the stop, stroking the piston, and against the plates, and it joins
  EXPECT_NEAR(drive->ClutchPressure(0.0), 2e5, 1e-6);
  EXPECT_NEAR(drive->ClutchPressure(0.5 * fill),
              (2000.0 + 1e5 * 0.001) / 0.01, 1e-6);
  EXPECT_NEAR(drive->ClutchPressure(fill), kiss, 1e-6);
  EXPECT_NEAR(drive->ClutchPressure(fill + 1e-7), kiss + 1e-7 * 1e9 / 1e-4,
              1e-3);
  EXPECT_NEAR(drive->ClutchPressure(-1e-8), 2e5 - 1e-8 * 1e9 / 1e-4, 1e-3);
}

TEST(PowertrainClutchHydraulics, ANewClutchTakesTheTimeToFillThroughItsOrifice) {
  shift::core::Context context;
  Vehicle v;
  v.transmission =
    R"("ratios": [2.0, 1.0], "gear": 0, "shift_time": 0.003,
       "clutch_capacity": 1500, "slip_speed": 1.0, "input_inertia": 0.2,
       "output_inertia": 0.2)";
  v.controllers = R"({"type": "schedule", "name": "shift",
    "interpolation": "step", "points": [0, 0, 0.1, 1],
    "targets": ["force_modules/drive/transmission/gear"]})";
  v.module_extra = kHydraulics;
  v.time_span = 1.0;
  Load(context, v);
  double kiss_time = -1.0, last_pressure = 0.0, last_ratio = 0.0;
  double ratio_at_fill = -1.0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    Powertrain* drive = Find(context);
    drive->UpdateOutputs(t, y.data());
    last_pressure = Output(drive, "clutch_1_pressure");
    last_ratio = Output(drive, "gear_ratio");
    if(kiss_time < 0.0 && last_pressure > 2.1e5) {
      kiss_time = t;
    }
    if(t < 0.5) {
      ratio_at_fill = last_ratio;
    }
  });
  context.Simulate();
  // the oil of the stroke, a l = 2e-5 m^3, enters at
  // Q = Cd A sqrt(2 (p_s - p_k) / rho) against the kiss pressure of 2e5 Pa
  const double flow = 0.7 * 1e-6 * std::sqrt(2.0 * 1.8e6 / 850.0);
  EXPECT_NEAR(kiss_time, 0.1 + 2e-5 / flow, 0.01);
  // the clutch carries nothing while it fills, and all it can after
  EXPECT_NEAR(ratio_at_fill, 0.0, 1e-9);
  EXPECT_NEAR(last_pressure, 2e6, 2e3);
  EXPECT_NEAR(last_ratio, 2.0, 1e-3);
}

TEST(PowertrainClutchHydraulics, AGearSelectedAtTheStartIsFullAndTheOthersDrained) {
  shift::core::Context context;
  Vehicle v = SteadyVehicle();
  v.module_extra = kHydraulics;
  v.time_span = 0.5;
  Load(context, v);
  Simulate(context);
  Powertrain* drive = Find(context);
  EXPECT_NEAR(Output(drive, "clutch_1_pressure"), 2e6, 1e3);
  EXPECT_NEAR(Output(drive, "clutch_2_pressure"), 0.0, 1e3);
  EXPECT_NEAR(Output(drive, "gear_ratio"), 2.0, 1e-6);
}

TEST(PowertrainExtensions, WhatTheyRefuse) {
  auto message = [&](const Vehicle& v) {
    shift::core::Context context;
    try {
      Load(context, v);
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  Vehicle count;
  count.transmission =
    R"("ratios": [2.0, 1.0], "gear": 1, "efficiencies": [0.9],
       "clutch_capacity": 1500)";
  EXPECT_NE(message(count).find("efficiencies"), std::string::npos);
  Vehicle high;
  high.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
    "efficiency": 1.1, "shaft_stiffness": 5e3})";
  EXPECT_NE(message(high).find("efficiency"), std::string::npos);
  Vehicle one_side = CenterVehicle(R"("split": 0.4)");
  one_side.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
    "center_output": 1, "shaft_stiffness": 5e3})";
  EXPECT_NE(message(one_side).find("both"), std::string::npos);
  Vehicle no_center;
  no_center.axles = R"({"name": "rear", "left": "wheel_l", "right": "wheel_r",
    "center_output": 1, "shaft_stiffness": 5e3})";
  EXPECT_NE(message(no_center).find("center_output"), std::string::npos);
  EXPECT_NE(message(CenterVehicle(R"("split": 1.5)")).find("split"),
            std::string::npos);
  Vehicle weak;
  weak.module_extra =
    R"(, "clutch_hydraulics": {"supply_pressure": 1e5, "piston_area": 0.01,
       "piston_stroke": 0.002, "spring_preload": 2000, "orifice_area": 1e-6})";
  EXPECT_NE(message(weak).find("supply_pressure"), std::string::npos);
}
