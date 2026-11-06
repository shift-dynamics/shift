/* The lead screw force module (force_module::LeadScrew): the torque and axial
force of a thread in steady motion against the formulas of power screws, the
backlash, the handedness, and a screw that holds its load or is back driven */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/force_module/lead_screw.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::LeadScrew;

/* a screw on a revolute joint inside a carriage on a prismatic joint, both
about the z axis (down), in a nut that is the fixed body; the load of the
screw is its mass */
static void Load(shift::core::Context& context, const std::string& screw,
                 double mass, double inertia, double time_span) {
  const std::string path = testing::TempDir() + "/lead_screw_test.json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.001,
                     "relative_error": 1e-8, "absolute_error": 1e-10}]
  },
  "assemblies": [{"name": "test",
    "bodies": [
      {"name": "carriage", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1e-3, "Ixx": 1e-6, "Iyy": 1e-6, "Izz": 1e-6,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}},
      {"name": "screw", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": )"
         << mass << R"(, "Ixx": )" << inertia << R"(, "Iyy": )" << inertia
         << R"(, "Izz": )" << inertia << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "slide", "type": "prismatic",
       "parent": [["fixed"], "origin"], "child": [["carriage"], "origin"],
       "axis": [0, 0, 1]},
      {"name": "spin", "type": "revolute",
       "parent": [["carriage"], "origin"], "child": [["screw"], "origin"],
       "axis": [0, 0, 1]}],
    "force_modules": [{"type": "lead_screw", "name": "thread",
      "screw": ["screw", "origin"], "nut": ["fixed", "origin"], )"
         << screw << R"(}]
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

static LeadScrew* Find(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto screw = dynamic_cast<LeadScrew*>(module)) {
      return screw;
    }
  }
  return nullptr;
}

/* put the screw in a state: the advance s along the axis and rotation
theta of the module's state, at the rates ds and omega, and evaluate the
forces */
static void Evaluate(shift::core::Context& context, LeadScrew* screw,
                     double s, double theta, double ds, double omega) {
  auto slide =
    context["assemblies"]["test"]["joints"]["slide"]
      .get_data<shift::core::joint::Base>();
  auto spin = context["assemblies"]["test"]["joints"]["spin"]
                .get_data<shift::core::joint::Base>();
  context.get_q()[slide->get_q_index()] = s;
  context.get_q()[spin->get_q_index()] = theta;
  context.get_dq()[slide->get_dq_index()] = ds;
  context.get_dq()[spin->get_dq_index()] = omega;
  context.UpdateDHVA();
  std::vector<double> y(
    context.get_state_count() + context.get_auxiliary_state_count(), 0.0);
  for(size_t i = 0; i < context.get_q_count(); i++) {
    y[i] = context.get_q()[i];
  }
  y[screw->get_state_offset()] = theta;
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
}

/* the torque to drive a power screw against an axial load F in the
direction of its motion (raising it), and to lower it, with the lead
angle lambda: Budynas and Nisbett, Shigley's Mechanical Engineering Design,
power screws (the square and Acme threads) */
static double TorqueToRaise(double load, double radius, double lambda,
                            double mu, double alpha) {
  const double m = mu / std::cos(alpha);
  return load * radius * (std::tan(lambda) + m) /
         (1.0 - m * std::tan(lambda));
}

static double TorqueToLower(double load, double radius, double lambda,
                            double mu, double alpha) {
  const double m = mu / std::cos(alpha);
  return load * radius * (m - std::tan(lambda)) /
         (1.0 + m * std::tan(lambda));
}

TEST(LeadScrew, SteadyMotionAgainstTheFormulasOfPowerScrews) {
  const double lead = 0.005, radius = 0.012;
  const double lambda = std::atan(lead / (2.0 * M_PI * radius));
  const double p = lead / (2.0 * M_PI);
  for(const auto& [mu, alpha] :
      {std::pair<double, double>{0.15, 0.0},     // a square thread
       std::pair<double, double>{0.15, 0.2531},  // an Acme thread, 14.5 deg
       std::pair<double, double>{0.03, 0.2531},  // not self-locking
       std::pair<double, double>{0.0, 0.0}}) {   // the ideal screw
    SCOPED_TRACE(mu);
    SCOPED_TRACE(alpha);
    shift::core::Context context;
    std::ostringstream thread;
    thread << R"("lead": 0.005, "mean_radius": 0.012, "friction": )" << mu
           << R"(, "thread_angle": )" << alpha
           << R"(, "stiffness": 1e8, "slip_speed": 1e-3)";
    Load(context, thread.str(), 10.0, 0.01, 0.0);
    LeadScrew* screw = Find(context);
    ASSERT_NE(screw, nullptr);
    EXPECT_NEAR(screw->get_lead_angle(), lambda, 1e-15);
    // pushed back by the load: the deflection is behind the ideal advance
    const double theta = 1.0, delta = -2e-6;
    for(double omega : {200.0, -200.0}) {
      SCOPED_TRACE(omega);
      const double ds = p * omega; // along the thread
      Evaluate(context, screw, p * theta + delta, theta, ds, omega);
      // the force on the screw along the axis is the load that it holds
      const double load = std::fabs(screw->get_axial_force());
      ASSERT_GT(load, 1.0);
      EXPECT_GT(screw->get_axial_force(), 0.0);
      // the driver's torque in the direction of the motion
      const double sign = omega > 0.0 ? 1.0 : -1.0;
      const double drive = -sign * screw->get_torque();
      const double expected =
        omega > 0.0 ? TorqueToRaise(load, radius, lambda, mu, alpha)
                    : TorqueToLower(load, radius, lambda, mu, alpha);
      EXPECT_NEAR(drive, expected, 2e-5 * std::fabs(expected) + 1e-9);
    }
  }
}

TEST(LeadScrew, SelfLockingFromTheFriction) {
  // mu sec(alpha) >= tan(lambda): tan(lambda) = 0.0663 here
  shift::core::Context locking, free;
  Load(locking, R"("lead": 0.005, "mean_radius": 0.012, "friction": 0.07,
                   "stiffness": 1e8)",
       10.0, 0.01, 0.0);
  Load(free, R"("lead": 0.005, "mean_radius": 0.012, "friction": 0.06,
                "stiffness": 1e8)",
       10.0, 0.01, 0.0);
  EXPECT_TRUE(Find(locking)->is_self_locking());
  EXPECT_FALSE(Find(free)->is_self_locking());
  // and with the thread angle: mu / cos(alpha)
  shift::core::Context acme;
  Load(acme, R"("lead": 0.005, "mean_radius": 0.012, "friction": 0.065,
                "thread_angle": 0.5, "stiffness": 1e8)",
       10.0, 0.01, 0.0);
  EXPECT_TRUE(Find(acme)->is_self_locking()); // 0.065 / cos(0.5) = 0.074
  // the torque to lower is negative (the load turns the screw) if not
  // self-locking
  const double radius = 0.012, lambda = Find(free)->get_lead_angle();
  EXPECT_LT(TorqueToLower(1000.0, radius, lambda, 0.06, 0.0), 0.0);
  EXPECT_GT(TorqueToLower(1000.0, radius, lambda, 0.07, 0.0), 0.0);
}

TEST(LeadScrew, TheDeflectionIsTheAdvanceBeyondTheIdealScrew) {
  shift::core::Context context;
  Load(context, R"("lead": 0.005, "mean_radius": 0.012, "friction": 0.0,
                   "stiffness": 1e8)",
       10.0, 0.01, 0.0);
  LeadScrew* screw = Find(context);
  ASSERT_NE(screw, nullptr);
  const double p = 0.005 / (2.0 * M_PI);
  // on the ideal screw nothing acts, whatever the turns
  Evaluate(context, screw, p * 7.0, 7.0, 0.0, 0.0);
  EXPECT_NEAR(screw->get_deflection(), 0.0, 1e-15);
  // (to the rounding of the deflection, times the stiffness of 1e8)
  EXPECT_NEAR(screw->get_axial_force(), 0.0, 1e-9);
  EXPECT_NEAR(screw->get_torque(), 0.0, 1e-9);
  // off it, a spring along the flank's normal: the force along the axis is
  // -k delta cos^2(lambda) (the deflection along the normal is cos(lambda)
  // of it, and so is the part of the force along the axis), and the torque
  // is p times its opposite (the screw is pushed back, and is turned back by
  // that push)
  const double cos_lambda = std::cos(screw->get_lead_angle());
  const double spring = 1e-6 * 1e8 * cos_lambda * cos_lambda;
  Evaluate(context, screw, p * 7.0 + 1e-6, 7.0, 0.0, 0.0);
  EXPECT_NEAR(screw->get_deflection(), 1e-6, 1e-15);
  EXPECT_NEAR(screw->get_axial_force(), -spring, 1e-9 * spring);
  EXPECT_NEAR(screw->get_torque(), p * spring, 1e-9 * p * spring);
  EXPECT_NEAR(screw->get_turns(), 7.0 / (2.0 * M_PI), 1e-12);
}

TEST(LeadScrew, BacklashLeavesThePlayFree) {
  shift::core::Context context;
  Load(context, R"("lead": 0.005, "mean_radius": 0.012, "friction": 0.0,
                   "stiffness": 1e8, "backlash": 2e-4)",
       10.0, 0.01, 0.0);
  LeadScrew* screw = Find(context);
  ASSERT_NE(screw, nullptr);
  // free for |delta| < b / 2 = 1e-4, loading beyond it (over a smooth start
  // of 0.1 b), a straight line past that: delta - b / 2 - 0.05 b
  for(double delta : {0.0, 5e-5, -9.9e-5, 1e-4}) {
    Evaluate(context, screw, delta, 0.0, 0.0, 0.0);
    EXPECT_EQ(screw->get_axial_force(), 0.0) << delta;
  }
  for(double delta : {5e-4, -5e-4}) {
    Evaluate(context, screw, delta, 0.0, 0.0, 0.0);
    const double expected = std::fabs(delta) - 1e-4 - 1e-5;
    const double cos_lambda = std::cos(screw->get_lead_angle());
    EXPECT_NEAR(std::fabs(screw->get_axial_force()),
                1e8 * expected * cos_lambda * cos_lambda,
                1e-9 * 1e8 * expected)
      << delta;
    EXPECT_EQ(screw->get_axial_force() < 0.0, delta > 0.0);
  }
  // continuous across the start and the slope too: no jump in the force
  double last = 0.0;
  for(double delta = 0.0; delta < 3e-4; delta += 1e-7) {
    Evaluate(context, screw, delta, 0.0, 0.0, 0.0);
    const double force = -screw->get_axial_force();
    EXPECT_GE(force, last - 1e-9);
    EXPECT_LT(force - last, 1e8 * 1e-7 + 1e-9);
    last = force;
  }
}

TEST(LeadScrew, ALeftHandedThreadAdvancesTheOtherWay) {
  shift::core::Context right, left, negative;
  const std::string common =
    R"("mean_radius": 0.012, "friction": 0.0, "stiffness": 1e8)";
  Load(right, R"("lead": 0.005,)" + common, 10.0, 0.01, 0.0);
  Load(left, R"("lead": 0.005, "left_handed": true,)" + common, 10.0, 0.01,
       0.0);
  Load(negative, R"("lead": -0.005,)" + common, 10.0, 0.01, 0.0);
  const double p = 0.005 / (2.0 * M_PI);
  // the same turn gives the opposite advance: no force on the ideal screw
  Evaluate(right, Find(right), p * 3.0, 3.0, 0.0, 0.0);
  EXPECT_NEAR(Find(right)->get_deflection(), 0.0, 1e-15);
  Evaluate(left, Find(left), -p * 3.0, 3.0, 0.0, 0.0);
  EXPECT_NEAR(Find(left)->get_deflection(), 0.0, 1e-15);
  Evaluate(negative, Find(negative), -p * 3.0, 3.0, 0.0, 0.0);
  EXPECT_NEAR(Find(negative)->get_deflection(), 0.0, 1e-15);
  // and the thread's torque on the screw is the other way
  Evaluate(right, Find(right), 1e-6, 0.0, 0.0, 0.0);
  Evaluate(left, Find(left), 1e-6, 0.0, 0.0, 0.0);
  EXPECT_NEAR(Find(right)->get_torque(), -Find(left)->get_torque(), 1e-9);
  EXPECT_GT(std::fabs(Find(left)->get_torque()), 0.05);
}

TEST(LeadScrew, WhatItRefuses) {
  auto message = [&](const std::string& thread) {
    shift::core::Context context;
    try {
      Load(context, thread, 10.0, 0.01, 0.0);
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  const std::string ok = R"("mean_radius": 0.012, "friction": 0.1,
                            "stiffness": 1e8)";
  EXPECT_NE(message(R"("lead": 0.0,)" + ok).find("lead"), std::string::npos);
  EXPECT_NE(message(R"("lead": 0.005, "mean_radius": 0.0, "friction": 0.1,
                      "stiffness": 1e8)")
              .find("mean_radius"),
            std::string::npos);
  EXPECT_NE(message(R"("lead": 0.005, "mean_radius": 0.012, "friction": -1,
                      "stiffness": 1e8)")
              .find("friction"),
            std::string::npos);
  EXPECT_NE(message(R"("lead": -0.005, "left_handed": true,)" + ok)
              .find("left handed"),
            std::string::npos);
  EXPECT_EQ(message(R"("lead": 0.005,)" + ok), "");
}

/* a mass on a screw under gravity, with the thread's axis down: a screw that
locks holds it, one that does not is back driven by the load, accelerating
as the ideal screw does, the mass and the inertia of the screw reflected
to the axis (m + I / p^2) */
TEST(LeadScrew, ASelfLockingScrewHoldsItsLoadAndAFreeOneIsBackDriven) {
  const double mass = 10.0, inertia = 0.05;
  const double lead = 0.1, radius = 0.02;
  const double p = lead / (2.0 * M_PI);
  (void)radius;
  // locking: mu >= tan(lambda) = 0.796
  {
    shift::core::Context context;
    Load(context, R"("lead": 0.1, "mean_radius": 0.02, "friction": 1.0,
                     "stiffness": 1e7, "damping": 2e3, "slip_speed": 1e-3)",
         mass, inertia, 0.3);
    LeadScrew* screw = Find(context);
    ASSERT_NE(screw, nullptr);
    ASSERT_TRUE(screw->is_self_locking());
    std::vector<double> y;
    context.set_state_observer(
      [&y](double, const std::vector<double>& state) { y = state; });
    context.Simulate();
    ASSERT_FALSE(y.empty());
    // the load rests on the thread: it creeps, slowly, at the speed of the
    // smoothed friction (at most a fraction of slip_speed) and the thread
    // carries its weight
    const double weight = mass * 9.81;
    EXPECT_LT(std::fabs(y[context["assemblies"]["test"]["joints"]["slide"]
                           .get_data<shift::core::joint::Base>()
                           ->get_q_index()]),
              1e-3);
    // the thread carries the load: the force on the screw is the weight
    EXPECT_NEAR(screw->get_axial_force(), -weight, 0.02 * weight);
  }
  // not locking: back driven, a = m g / (m + I / p^2), less the friction
  {
    shift::core::Context context;
    Load(context, R"("lead": 0.1, "mean_radius": 0.02, "friction": 0.02,
                     "stiffness": 1e7, "damping": 2e3, "slip_speed": 1e-3)",
         mass, inertia, 0.5);
    LeadScrew* screw = Find(context);
    ASSERT_NE(screw, nullptr);
    ASSERT_FALSE(screw->is_self_locking());
    std::vector<double> y;
    context.set_state_observer(
      [&y](double, const std::vector<double>& state) { y = state; });
    context.Simulate();
    ASSERT_FALSE(y.empty());
    const double ideal = mass * 9.81 / (mass + inertia / (p * p));
    const double advance =
      y[context["assemblies"]["test"]["joints"]["slide"]
          .get_data<shift::core::joint::Base>()
          ->get_q_index()]; // the advance of the screw
    EXPECT_NEAR(advance, 0.5 * ideal * 0.25, 0.1 * 0.5 * ideal * 0.25);
    EXPECT_GT(advance, 0.0);
  }
}

/* where the load has got to, by the prismatic joint */
static double Travel(shift::core::Context& context,
                     const std::vector<double>& y) {
  return y[context["assemblies"]["test"]["joints"]["slide"]
             .get_data<shift::core::joint::Base>()
             ->get_q_index()];
}

static const char* kBristles = R"("bristle_stiffness": 1e5,
  "bristle_damping": 500, "stribeck_velocity": 0.01)";

TEST(LeadScrewStick, TheBristlesHoldWhereTheSmoothedFrictionCreeps) {
  // the load of a self-locking thread: the smoothed friction lets it creep
  // on, the bristles stop it. The travel between 0.5 s and 1 s
  const std::string thread = R"("lead": 0.1, "mean_radius": 0.02,
    "friction": 1.0, "stiffness": 1e7, "damping": 2e3, "slip_speed": 1e-3)";
  auto creep = [](const std::string& options) {
    shift::core::Context context;
    Load(context, options, 10.0, 0.05, 1.0);
    double half = 0.0, end = 0.0;
    context.set_state_observer([&](double t, const std::vector<double>& y) {
      if(t <= 0.5) {
        half = Travel(context, y);
      }
      end = Travel(context, y);
    });
    context.Simulate();
    return std::fabs(end - half);
  };
  const double smoothed = creep(thread);
  const double held = creep(thread + ", " + kBristles);
  EXPECT_GT(smoothed, 1e-5);
  EXPECT_LT(held, 1e-8);
}

TEST(LeadScrewStick, TheDeflectionOfTheBristlesCarriesTheTorqueOfTheLoad) {
  // the load tries to turn the screw with p w; the friction along the thread
  // holds it, a fraction of the limit, shown by stick
  shift::core::Context context;
  Load(context, std::string(R"("lead": 0.1, "mean_radius": 0.02,
         "friction": 1.0, "stiffness": 1e7, "damping": 2e3, )") + kBristles,
       10.0, 0.05, 1.0);
  LeadScrew* screw = Find(context);
  std::vector<double> y;
  context.set_state_observer(
    [&y](double, const std::vector<double>& state) { y = state; });
  context.Simulate();
  screw->UpdateOutputs(1.0, y.data());
  // at rest the flank's friction balances the part of the load along the
  // thread: f_t = n tan(lambda), so stick = tan(lambda) / mu
  const double lambda = screw->get_lead_angle();
  EXPECT_NEAR(screw->get_stick(), std::tan(lambda), 0.02);
  EXPECT_NEAR(screw->get_axial_force(), -10.0 * 9.81, 0.02 * 98.1);
}

TEST(LeadScrewStick, TheStaticFrictionDecidesWhetherItLocks) {
  // tan(lambda) = 0.796: friction of 0.5 lets it run, unless the static
  // friction is above it. Started at rest, that holds
  const std::string thread = R"("lead": 0.1, "mean_radius": 0.02,
    "friction": 0.5, "stiffness": 1e7, "damping": 2e3, )";
  {
    shift::core::Context context;
    Load(context, thread + kBristles + R"(, "static_friction": 1.0)", 10.0,
         0.05, 0.5);
    EXPECT_TRUE(Find(context)->is_self_locking());
    std::vector<double> y;
    context.set_state_observer(
      [&y](double, const std::vector<double>& state) { y = state; });
    context.Simulate();
    EXPECT_LT(std::fabs(Travel(context, y)), 1e-3);
  }
  {
    shift::core::Context context;
    Load(context, thread + kBristles, 10.0, 0.05, 0.5);
    EXPECT_FALSE(Find(context)->is_self_locking());
    std::vector<double> y;
    context.set_state_observer(
      [&y](double, const std::vector<double>& state) { y = state; });
    context.Simulate();
    EXPECT_GT(std::fabs(Travel(context, y)), 0.01);
  }
}

TEST(LeadScrewStick, InSteadyMotionItIsTheFrictionOfThePowerScrew) {
  // the formulas of power screws, as the smoothed friction gives them
  const double load = 1000.0, r = 0.012, lead = 0.005, mu = 0.15;
  const double lambda = std::atan(lead / (2.0 * M_PI * r));
  std::ostringstream options;
  options << R"("lead": 0.005, "mean_radius": 0.012, "friction": 0.15,
                "stiffness": 1e8, "bristle_stiffness": 1e5)";
  shift::core::Context context;
  Load(context, options.str(), 1.0, 1e-3, 0.1);
  LeadScrew* screw = Find(context);
  // the bristles saturated: z = mu / sigma_0, sliding forward
  const double p = lead / (2.0 * M_PI);
  const double delta = -load / 1e8 / std::pow(std::cos(lambda), 2.0);
  auto joint = [&](const char* name) {
    return context["assemblies"]["test"]["joints"][name]
      .get_data<shift::core::joint::Base>();
  };
  const double omega = 10.0;
  context.get_q()[joint("slide")->get_q_index()] = delta;
  context.get_dq()[joint("slide")->get_dq_index()] = p * omega;
  context.get_dq()[joint("spin")->get_dq_index()] = omega;
  context.UpdateDHVA();
  std::vector<double> y(
    context.get_state_count() + context.get_auxiliary_state_count(), 0.0);
  y[screw->get_state_offset() + 1] = mu / 1e5;
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
  EXPECT_NEAR(dy[screw->get_state_offset() + 1], 0.0, 1e-9);
  // the flank's normal force is the load over cos(lambda)
  EXPECT_NEAR(screw->get_normal_force(), -load / std::cos(lambda), 1e-6 * load);
  EXPECT_NEAR(screw->get_friction_force(), mu * load / std::cos(lambda),
              1e-6 * load);
  EXPECT_NEAR(screw->get_stick(), 1.0, 1e-9);
}

TEST(LeadScrewStick, WhatItRefuses) {
  shift::core::Context context;
  EXPECT_THROW(Load(context, R"("lead": 0.01, "mean_radius": 0.02,
    "friction": 0.5, "stiffness": 1e7, "bristle_stiffness": 1e5,
    "static_friction": 0.3)", 1.0, 0.01, 0.1), std::exception);
}
