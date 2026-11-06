/* Applied load force module (force_module::AppliedLoad): constant loads in the
fixed and body frames, moments of a load off the origin, action and reaction,
curves over time and impulses */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/applied_load.h"

using shift::core::force_module::AppliedLoad;

static const char* kBodies =
  R"({"name": "body", "center_of_mass": [0, 0, 0],
      "frames": [{"name": "tip", "T": [1, 0, 0]}],
      "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}},
     {"name": "other", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 3, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}})";
static const char* kSliders =
  R"({"name": "j", "type": "prismatic", "parent": [["fixed"], "origin"],
      "child": [["body"], "origin"], "axis": [1, 0, 0]},
     {"name": "k", "type": "prismatic", "parent": [["fixed"], "origin"],
      "child": [["other"], "origin"], "axis": [1, 0, 0]})";
static const char* kHinges =
  R"({"name": "j", "type": "revolute", "parent": [["fixed"], "origin"],
      "child": [["body"], "origin"], "axis": [0, 1, 0]},
     {"name": "k", "type": "prismatic", "parent": [["fixed"], "origin"],
      "child": [["other"], "origin"], "axis": [1, 0, 0]})";

static void LoadModel(shift::core::Context& context, const std::string& name,
                      const std::string& joints, const std::string& modules,
                      double time_span, const std::string& controllers = "") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 0], "solvers": [{"type": "rnca"}],
      "include_directories": [")" << testing::TempDir() << R"("],
      "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.001, "hmax": 0.001,
                             "relative_error": 1e-9,
                             "absolute_error": 1e-11}]},
      "assemblies": [{"name": "test", "bodies": [)" << kBodies
         << R"(], "joints": [)" << joints << R"(],
         "controllers": [)" << controllers << R"(],
         "force_modules": [)" << modules << R"(]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

/* the states of the two joints at the end: q_j, q_k, dq_j, dq_k */
static std::vector<double> RunToEnd(shift::core::Context& context) {
  std::vector<double> last;
  context.set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context.Simulate();
  return last;
}

TEST(AppliedLoad, AConstantForce) {
  shift::core::Context context;
  LoadModel(context, "al_constant", kSliders,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "force": [6, 0, 0]})",
            1.0);
  const auto y = RunToEnd(context);
  EXPECT_NEAR(y[2], 3.0, 1e-7); // a = 6 / 2 for 1 s
  EXPECT_NEAR(y[3], 0.0, 1e-12);
}

TEST(AppliedLoad, ActionAndReaction) {
  shift::core::Context context;
  LoadModel(context, "al_reaction", kSliders,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "reaction_body": "other", "force": [6, 0, 0]})",
            1.0);
  const auto y = RunToEnd(context);
  EXPECT_NEAR(y[2], 3.0, 1e-7);
  EXPECT_NEAR(y[3], -2.0, 1e-7);
}

TEST(AppliedLoad, AForceOffTheOriginTurnsTheBody) {
  // 2 N along z at the tip, 1 m along x: a moment about y of -2 N m in the
  // fixed frame (r x f), at Iyy 0.2 for a short time
  shift::core::Context context;
  LoadModel(context, "al_moment", kHinges,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "frame": "tip", "coordinates": "fixed",
                "force": [0, 0, 2]})",
            0.01);
  const auto y = RunToEnd(context);
  EXPECT_NEAR(y[2], -2.0 / 0.2 * 0.01, 1e-4);
}

TEST(AppliedLoad, AFrameLoadTurnsWithTheBody) {
  // the same load in the frame's coordinates stays at right angles to the
  // arm: a steady moment, however far it turns
  shift::core::Context context;
  LoadModel(context, "al_follower", kHinges,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "frame": "tip", "force": [0, 0, 2]})",
            1.0);
  const auto y = RunToEnd(context);
  EXPECT_NEAR(y[2], -10.0, 1e-6);
  EXPECT_NEAR(y[0], -5.0, 1e-6);
}

TEST(AppliedLoad, AMoment) {
  shift::core::Context context;
  LoadModel(context, "al_pure_moment", kHinges,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "moment": [0, 1, 0]})",
            1.0);
  EXPECT_NEAR(RunToEnd(context)[2], 5.0, 1e-7);
}

TEST(AppliedLoad, ACurveOverTime) {
  // the scale rises from 0 to 1 over 1 s, in a monotone cubic of equal end
  // slopes: its integral over the rise is 1/2, then it holds
  shift::core::Context context;
  LoadModel(context, "al_curve", kSliders,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "force": [2, 0, 0], "scale_curve": [0, 0, 1, 1]})",
            2.0);
  EXPECT_NEAR(RunToEnd(context)[2], 1.5, 1e-5);
}

TEST(AppliedLoad, ACurveFromAFile) {
  const std::string path = testing::TempDir() + "/al_curve.txt";
  {
    std::ofstream file(path);
    file << "0, 3\n1, 3\n";
  }
  shift::core::Context context;
  LoadModel(context, "al_file", kSliders,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "force": [2, 0, 0], "scale_file": "al_curve.txt"})",
            1.0);
  std::remove(path.c_str());
  EXPECT_NEAR(RunToEnd(context)[2], 3.0, 1e-7);
}

TEST(AppliedLoad, APulseGivesItsImpulse) {
  // 1000 N for 0.01 s: an impulse of 10 N s, on 2 kg
  shift::core::Context context;
  LoadModel(context, "al_pulse", kSliders,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "force": [1000, 0, 0], "pulse_start": 0.1,
                "pulse_duration": 0.01})",
            0.2);
  EXPECT_NEAR(RunToEnd(context)[2], 5.0, 1e-5);
}

TEST(AppliedLoad, AScheduleChangesTheScale) {
  shift::core::Context context;
  LoadModel(context, "al_schedule", kSliders,
            R"({"type": "applied_load", "name": "l", "body": "body",
                "force": [2, 0, 0]})",
            1.0,
            R"({"type": "schedule", "name": "s", "points": [0, 4, 1, 4],
                "targets": ["force_modules/l/scale"]})");
  EXPECT_NEAR(RunToEnd(context)[2], 4.0, 1e-7);
}

TEST(AppliedLoad, RefusesUnknownCoordinates) {
  shift::core::Context context;
  EXPECT_THROW(LoadModel(context, "al_bad", kSliders,
                         R"({"type": "applied_load", "name": "l",
                             "body": "body", "coordinates": "sideways"})",
                         0.0),
               shift::core::Exception);
}
