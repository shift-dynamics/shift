/* Spline force module (force_module::Spline): a hub on a slide and a pin, joined
to the fixed body by a spline: the torque past the backlash, and the friction of
the teeth, which holds and slides according to that torque */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/spline.h"

static const double kRadius = 0.02, kAngle = 0.5236;
static const double kMass = 10.0;
static const double kStiffness = 5e4;

/* the hub under a steady torque about the axis and force along it. In y: the
slide, the pin, their rates, then the spline's angle and bristles */
static void LoadModel(shift::core::Context& context, const std::string& name,
                      double torque, double force, const std::string& keys,
                      double time_span) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 0], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "relative_error": 1e-7,
                       "absolute_error": 1e-10, "hmax": 0.001,
                       "time_span": )" << time_span << R"(,
                       "time_step": 0.01}]},
      "assemblies": [{"name": "m", "bodies": [
        {"name": "slider", "center_of_mass": [0, 0, 0], "geometry": [],
         "inertia": {"mass": )" << 0.5 * kMass << R"(, "Ixx": 1, "Iyy": 1,
                     "Izz": 1, "Ixy": 0, "Iyz": 0, "Izx": 0}},
        {"name": "hub", "center_of_mass": [0, 0, 0], "geometry": [],
         "inertia": {"mass": )" << 0.5 * kMass << R"(, "Ixx": 0.1, "Iyy": 0.1,
                     "Izz": 0.1, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
      "joints": [
        {"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
         "parent": [["fixed"], "origin"], "child": [["slider"], "origin"]},
        {"name": "pin", "type": "revolute", "axis": [0, 0, 1],
         "parent": [["slider"], "origin"], "child": [["hub"], "origin"]}],
      "force_modules": [
        {"type": "applied_load", "name": "load", "body": "hub",
         "coordinates": "fixed", "force": [0, 0, )" << force << R"(],
         "moment": [0, 0, )" << torque << R"(],
         "scale_curve": [0, 0, 0.05, 1, 100, 1]},
        {"type": "spline", "name": "spline",
         "frames": [["fixed", "origin"], ["hub", "origin"]],
         "pitch_radius": )" << kRadius << R"(, "pressure_angle": )" << kAngle
         << R"(, "torsional_stiffness": )" << kStiffness << R"(,
         "torsional_damping": 100)" << keys << R"(}]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

struct End {
  double slide = 0.0, pin = 0.0, rate = 0.0, angle = 0.0;
};

static End RunToEnd(shift::core::Context& context) {
  End end;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    end.slide = y[0];
    end.pin = y[1];
    end.rate = y[2];
    end.angle = y[4];
  });
  context.Simulate();
  return end;
}

/* the axial force the spline holds under the torque, at the coefficient */
static double Limit(double torque, double coefficient) {
  return coefficient * torque / (kRadius * std::cos(kAngle));
}

TEST(Spline, CarriesTheTorque) {
  shift::core::Context context;
  LoadModel(context, "sp_torque", 100.0, 0.0, "", 0.5);
  const End end = RunToEnd(context);
  EXPECT_NEAR(end.pin, 100.0 / kStiffness, 1e-5);
  EXPECT_NEAR(end.angle, end.pin, 1e-8);
}

TEST(Spline, BacklashTakesUpFirst) {
  // the hub turns through half the backlash before the teeth carry anything
  shift::core::Context context;
  LoadModel(context, "sp_backlash", 100.0, 0.0,
            R"(, "backlash": 0.01, "smoothing": 1e-5)", 1.0);
  const End end = RunToEnd(context);
  EXPECT_NEAR(end.pin, 0.005 + 100.0 / kStiffness, 2e-5);
}

TEST(Spline, TorqueHoldsTheSlideAgainstAForce) {
  // below the limit: the slide gives by the bristles only
  const double torque = 100.0, force = 0.8 * Limit(torque, 0.1);
  shift::core::Context context;
  LoadModel(context, "sp_hold", torque, force, "", 0.5);
  const End end = RunToEnd(context);
  EXPECT_LT(std::fabs(end.slide), 3e-5);
  EXPECT_LT(std::fabs(end.rate), 1e-6);
}

TEST(Spline, ForceAboveTheLimitSlides) {
  // above it: at the acceleration the difference gives. The force and torque
  // come in together from zero, so it slips from the start
  const double torque = 100.0, force = 1.5 * Limit(torque, 0.1);
  const double span = 0.3;
  shift::core::Context context;
  LoadModel(context, "sp_slide", torque, force, "", span);
  double previous_t = 0.0, previous_rate = 0.0, a = 0.0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    if(t > 0.2 && t - previous_t > 1e-3) {
      a = (y[2] - previous_rate) / (t - previous_t);
    }
    if(t - previous_t > 1e-3) {
      previous_t = t;
      previous_rate = y[2];
    }
  });
  context.Simulate();
  EXPECT_NEAR(a, (force - Limit(torque, 0.1)) / kMass, 0.5);
}

TEST(Spline, WithoutTorqueItSlidesFreely) {
  const double force = 100.0, span = 0.5;
  shift::core::Context context;
  LoadModel(context, "sp_free", 0.0, force, "", span);
  const End end = RunToEnd(context);
  // the force comes in over 0.05 s, as a smooth curve
  EXPECT_NEAR(end.rate, force / kMass * (span - 0.025), 0.1);
}

TEST(Spline, BristleTimeLeavesTheSlidingFriction) {
  // the same slide, with the limit and with the plain LuGre model
  const double torque = 100.0, force = 1.5 * Limit(torque, 0.1);
  shift::core::Context limited, plain;
  LoadModel(limited, "sp_limited", torque, force, "", 0.3);
  LoadModel(plain, "sp_plain", torque, force, R"(, "bristle_time": 0)", 0.3);
  const End a = RunToEnd(limited), b = RunToEnd(plain);
  EXPECT_GT(a.rate, 1.0);
  EXPECT_NEAR(a.rate, b.rate, 1e-3 * b.rate);
  EXPECT_NEAR(a.slide, b.slide, 1e-3 * b.slide);
}

TEST(Spline, PreloadHoldsWithoutTorque) {
  const double force = 40.0;
  shift::core::Context context;
  LoadModel(context, "sp_preload", 0.0, force, R"(, "preload": 500)", 0.5);
  const End end = RunToEnd(context);
  // the bristles creep to z = 0.8 of their limit g / sigma_0: with
  // dz/ds = 1 - z / limit that takes the slide limit * ln(1 / (1 - 0.8))
  EXPECT_NEAR(end.slide, 0.1 / 2e4 * std::log(5.0), 1e-6);
  EXPECT_LT(std::fabs(end.rate), 1e-6);
}

TEST(Spline, StaticFrictionHoldsMore) {
  // between the sliding and the static limits: it holds, where without the
  // static friction the same force slides it
  const double torque = 100.0, force = 1.2 * Limit(torque, 0.1);
  shift::core::Context holding;
  LoadModel(holding, "sp_static", torque, force,
            R"(, "static_friction": 0.15)", 0.5);
  EXPECT_LT(std::fabs(RunToEnd(holding).rate), 1e-6);
  shift::core::Context sliding;
  LoadModel(sliding, "sp_kinetic", torque, force, "", 0.5);
  EXPECT_GT(RunToEnd(sliding).rate, 1.0);
}

/* both halves free on a tilted axis away from the origin: a carriage on a
slide, turning a shaft, on which a slider and a hub slide and turn. Constant
loads on the hub along the axis. In y: travel, drive, slide, pin, their rates,
then the spline's angle and bristles */
static const double kFirstMass = 6.0, kSecondMass = 4.0;
static const double kFirstInertia = 2.0, kHubInertia = 0.1;

static void LoadFreeHalves(shift::core::Context& context,
                           const std::string& name, double torque,
                           double force, double time_span) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  auto body = [](const char* name, double mass, double spin) {
    std::ostringstream text;
    text << R"({"name": ")" << name << R"(", "center_of_mass": [0, 0, 0],
      "geometry": [], "inertia": {"mass": )" << mass << R"(, "Ixx": 1,
      "Iyy": 1, "Izz": )" << spin << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}})";
    return text.str();
  };
  {
    std::ofstream file(path);
    file.precision(17);
    // the axis, the z of mount, is (0.6, 0, 0.8) in the fixed frame
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 0], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "relative_error": 1e-8,
                       "absolute_error": 1e-11, "hmax": 0.001,
                       "time_span": )" << time_span << R"(,
                       "time_step": 0.01}]},
      "frames": [{"name": "mount",
                  "R": [[0.8, 0, 0.6], [0, 1, 0], [-0.6, 0, 0.8]],
                  "T": [0.3, 0.2, -0.1]}],
      "assemblies": [{"name": "m", "bodies": [)"
         << body("carriage", kFirstMass - 3.0, 1.0) << ","
         << body("shaft", 3.0, kFirstInertia - 0.5) << ","
         << body("slider", kSecondMass - 1.0, 0.5) << ","
         << body("hub", 1.0, kHubInertia) << R"(],
      "joints": [
        {"name": "travel", "type": "prismatic", "axis": [0, 0, 1],
         "parent": [["fixed"], "mount"], "child": [["carriage"], "origin"]},
        {"name": "drive", "type": "revolute", "axis": [0, 0, 1],
         "parent": [["carriage"], "origin"], "child": [["shaft"], "origin"]},
        {"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
         "parent": [["shaft"], "origin"], "child": [["slider"], "origin"]},
        {"name": "pin", "type": "revolute", "axis": [0, 0, 1],
         "parent": [["slider"], "origin"], "child": [["hub"], "origin"]}],
      "force_modules": [
        {"type": "applied_load", "name": "load", "body": "hub",
         "coordinates": "fixed",
         "force": [)" << 0.6 * force << ", 0, " << 0.8 * force << R"(],
         "moment": [)" << 0.6 * torque << ", 0, " << 0.8 * torque << R"(]},
        {"type": "spline", "name": "spline",
         "frames": [["shaft", "origin"], ["hub", "origin"]],
         "pitch_radius": )" << kRadius << R"(, "pressure_angle": )" << kAngle
         << R"(, "torsional_stiffness": )" << kStiffness << R"(,
         "torsional_damping": 100}]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

TEST(Spline, HalvesShareTheLoadsWhileSliding) {
  // the spline's loads are internal: whatever slips, the momentum along the
  // axis and the angular momentum about it are those the applied loads give
  const double torque = 100.0, force = 1000.0, span = 0.2;
  shift::core::Context context;
  LoadFreeHalves(context, "sp_free_halves", torque, force, span);
  std::vector<double> end;
  context.set_state_observer(
    [&](double, const std::vector<double>& y) { end = y; });
  context.Simulate();
  ASSERT_EQ(end.size(), 10u);
  const double first = end[4], second = end[4] + end[6];
  const double first_spin = end[5], second_spin = end[5] + end[7];
  EXPECT_NEAR(kFirstMass * first + kSecondMass * second, force * span, 1e-3);
  EXPECT_NEAR((kFirstInertia)*first_spin + kHubInertia * second_spin,
              torque * span, 1e-3);
  // it slid, and the friction dragged the first half along
  EXPECT_GT(end[6], 1.0);
  EXPECT_GT(first, 1.0);
  // the angle follows the hub on the shaft
  EXPECT_NEAR(end[8], end[3], 1e-8);
  // the torque reaches the shaft through the teeth
  const double accelerated = torque * kHubInertia / (kFirstInertia + kHubInertia);
  EXPECT_NEAR(end[3], (torque - accelerated) / kStiffness, 1e-5);
}

TEST(Spline, HalvesMoveTogetherWhileHeld) {
  const double torque = 100.0, force = 200.0, span = 0.2;
  shift::core::Context context;
  LoadFreeHalves(context, "sp_held_halves", torque, force, span);
  std::vector<double> end;
  context.set_state_observer(
    [&](double, const std::vector<double>& y) { end = y; });
  context.Simulate();
  ASSERT_EQ(end.size(), 10u);
  EXPECT_NEAR(end[4], force * span / (kFirstMass + kSecondMass), 1e-2);
  EXPECT_LT(std::fabs(end[6]), 1e-2);
}

TEST(Spline, TheExampleLocksAsTheTorqueRises) {
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                   "/force_modules/slip_spline.json");
  double peak = 0.0, slide = 0.0, rate = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    peak = std::max(peak, y[2]);
    slide = y[0];
    rate = y[2];
  });
  context.Simulate();
  EXPECT_GT(peak, 0.5);
  EXPECT_GT(slide, 0.01);
  EXPECT_LT(slide, 0.1);
  EXPECT_LT(std::fabs(rate), 1e-3);
}

TEST(Spline, BadParametersAreRefused) {
  // the same model loads with good keys, so it is the keys that are refused
  {
    shift::core::Context context;
    EXPECT_NO_THROW(LoadModel(context, "sp_good", 1.0, 0.0, "", 0.1));
  }
  for(const char* keys :
      {R"(, "backlash": -1)", R"(, "pressure_angle": 2)", R"(, "friction": 0.2,
          "static_friction": 0.1)", R"(, "bristle_stiffness": 0)"}) {
    SCOPED_TRACE(keys);
    shift::core::Context context;
    try {
      LoadModel(context, "sp_bad", 1.0, 0.0, keys, 0.1);
      ADD_FAILURE() << "not refused";
    } catch(const std::exception& e) {
      EXPECT_NE(std::string(e.what()).find("Spline spline"), std::string::npos)
        << e.what();
    }
  }
}
