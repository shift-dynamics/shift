/* Spring damper force module (force_module::SpringDamper): the law, a body on
a spring against its closed form, the energy of a spring whose line turns,
torsion past a full turn, and Bouc-Wen hysteresis */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/spring_damper.h"

using shift::core::force_module::BoucWen;
using shift::core::force_module::SpringDamper;

static const char* kBody =
  R"({"name": "body", "center_of_mass": [0, 0, 0],
      "frames": [{"name": "tip", "T": [1, 0, 0]}],
      "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}})";
static const char* kSlider =
  R"({"name": "j", "type": "prismatic", "parent": [["fixed"], "origin"],
      "child": [["body"], "origin"], "axis": [0, 0, 1]})";
static const char* kHinge =
  R"({"name": "j", "type": "revolute", "parent": [["fixed"], "origin"],
      "child": [["body"], "origin"], "axis": [0, 1, 0]})";

/* a body on the joint, with the force module; anchor is 1 above the origin */
static void LoadModel(shift::core::Context& context, const std::string& name,
                      const std::string& joint, const std::string& module,
                      double time_span) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.01, "relative_error": 1e-9,
                             "absolute_error": 1e-11}]},
      "frames": [{"name": "anchor", "T": [0, 0, -1]},
                 {"name": "far", "T": [0, 0, -2]},
                 {"name": "floor", "T": [0, 0, 1]}],
      "assemblies": [{"name": "test", "bodies": [)"
         << kBody << R"(], "joints": [)" << joint
         << R"(], "force_modules": [)" << module << R"(]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static SpringDamper* Find(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto m = dynamic_cast<SpringDamper*>(module)) {
      return m;
    }
  }
  return nullptr;
}

static std::string Module(const std::string& keys) {
  return R"({"type": "spring_damper", "name": "s",
             "frames": [["fixed", "anchor"], ["body", "origin"]], )" +
         keys + "}";
}

TEST(SpringDamper, ALinearSpringAndDamper) {
  shift::core::Context context;
  LoadModel(context, "sd_linear", kSlider,
            Module(R"("stiffness": 100, "preload": 5, "damping": 3)"), 0.0);
  EXPECT_DOUBLE_EQ(Find(context)->Force(0.2, 0.5, 0.0),
                   100 * 0.2 + 5 + 3 * 0.5);
}

TEST(SpringDamper, CurvesDifferInCompressionAndRebound) {
  shift::core::Context context;
  LoadModel(context, "sd_curves", kSlider, Module(R"(
    "force_curve": [-0.1, -500, 0, 0, 0.1, 100],
    "damping_curve": [-1, -300, 0, 0, 1, 900])"),
            0.0);
  SpringDamper* s = Find(context);
  EXPECT_DOUBLE_EQ(s->Force(-0.1, 0.0, 0.0), -500.0);
  EXPECT_DOUBLE_EQ(s->Force(0.1, 0.0, 0.0), 100.0);
  EXPECT_DOUBLE_EQ(s->Force(0.0, -1.0, 0.0), -300.0);
  EXPECT_DOUBLE_EQ(s->Force(0.0, 1.0, 0.0), 900.0);
}

TEST(SpringDamper, ACompressionStop) {
  shift::core::Context context;
  LoadModel(context, "sd_stop", kSlider, Module(R"(
    "acts": "compression", "gap": 0.1, "stiffness": 1000, "damping": 10,
    "smoothing": 0.01)"),
            0.0);
  SpringDamper* s = Find(context);
  EXPECT_EQ(s->Force(0.0, 0.0, 0.0), 0.0);
  EXPECT_EQ(s->Force(1.0, 0.0, 0.0), 0.0);     // does not act in tension
  EXPECT_EQ(s->Force(-0.09, -5.0, 0.0), 0.0);  // short of the edge
  EXPECT_DOUBLE_EQ(s->Force(-0.15, 0.0, 0.0), -1000.0 * 0.05);
  EXPECT_DOUBLE_EQ(s->Force(-0.15, -1.0, 0.0), -1000.0 * 0.05 - 10.0);
  // the damper of a fast rebound does not make the stop pull
  EXPECT_EQ(s->Force(-0.15, 100.0, 0.0), 0.0);
  // no jump across the edge
  double last = 0.0, largest = 0.0;
  for(int i = 0; i <= 2000; i++) {
    const double f = s->Force(-0.09 - 2e-5 * i, -0.5, 0.0);
    largest = std::max(largest, std::abs(f - last));
    last = f;
  }
  EXPECT_LT(largest, 0.05);
}

TEST(SpringDamper, ADeadZoneActsOnBothSides) {
  shift::core::Context context;
  LoadModel(context, "sd_dead_zone", kSlider, Module(R"(
    "gap": 0.1, "stiffness": 1000, "smoothing": 0.01)"),
            0.0);
  SpringDamper* s = Find(context);
  EXPECT_EQ(s->Force(0.0, 0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(s->Force(0.2, 0.0, 0.0), 100.0);
  EXPECT_DOUBLE_EQ(s->Force(-0.2, 0.0, 0.0), -100.0);
}

TEST(SpringDamper, ABodyDroppedOnASpring) {
  // released at the free length: z = (m g / k) (1 - cos(w t))
  shift::core::Context context;
  const double k = 800.0, m = 2.0, g = 9.81;
  const double w = std::sqrt(k / m);
  LoadModel(context, "sd_drop", kSlider, Module(R"("stiffness": 800)"),
            M_PI / w);
  double z = 0.0;
  context.set_state_observer(
    [&z](double, const std::vector<double>& y) { z = y[0]; });
  context.Simulate();
  EXPECT_NEAR(z, 2.0 * m * g / k, 1e-6);
}

TEST(SpringDamper, EnergyHoldsAsTheLineTurns) {
  // a spring from a point 2 above the hinge to the tip of the arm, turning
  // the arm about its center of mass: only the spring does work
  shift::core::Context context;
  const double k = 300.0, free = 1.5, inertia = 0.2;
  const std::string module =
    R"({"type": "spring_damper", "name": "s",
        "frames": [["fixed", "far"], ["body", "tip"]], "stiffness": 300,
        "free_length": 1.5})";
  LoadModel(context, "sd_turning", kHinge, module, 1.0);
  auto length = [](double a) {
    // the tip at (cos a, -sin a) in the plane (x, z), about y
    const double x = std::cos(a), z = -std::sin(a) + 2.0;
    return std::sqrt(x * x + z * z);
  };
  const double start = 0.5 * k * std::pow(length(0.0) - free, 2);
  double angle = 0.0, speed = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    angle = y[0];
    speed = y[1];
  });
  context.Simulate();
  EXPECT_GT(std::abs(speed), 0.1); // it has moved
  EXPECT_NEAR(0.5 * inertia * speed * speed +
                0.5 * k * std::pow(length(angle) - free, 2),
              start, 1e-6);
}

TEST(SpringDamper, TorsionCountsFullTurns) {
  // from rest towards a free angle of 4 rad: a = 4 (1 - cos(w t)), past 2 pi
  shift::core::Context context;
  const double k = 5.0, inertia = 0.2;
  const double w = std::sqrt(k / inertia);
  LoadModel(context, "sd_torsion", kHinge,
            Module(R"("axis": [0, 1, 0], "stiffness": 5, "free_angle": 4)"),
            M_PI / w);
  double angle = 0.0;
  context.set_state_observer(
    [&angle](double, const std::vector<double>& y) { angle = y[0]; });
  context.Simulate();
  EXPECT_NEAR(angle, 8.0, 1e-5);
}

TEST(BoucWen, TheLoopFollowsItsClosedForm) {
  // with n = 1, beta = gamma = 1/2: z = 1 - exp(-x / x_y) on the way up, and
  // a straight line of slope 1 / x_y back from there
  BoucWen h;
  h.yield_force = 1.0;
  h.yield_displacement = 0.01;
  const int steps = 200000;
  const double top = 0.03, dx = top / steps;
  double z = 0.0;
  for(int i = 0; i < steps; i++) {
    z += h.Rate(z, 1.0) * dx;
  }
  EXPECT_NEAR(z, 1.0 - std::exp(-top / 0.01), 1e-4);
  const double peak = z;
  for(int i = 0; i < steps / 30; i++) {
    z += h.Rate(z, -1.0) * dx;
  }
  EXPECT_NEAR(z, peak - (top / 30) / 0.01, 1e-3);
}

TEST(BoucWen, TheRateDoesNotChangeTheLoop) {
  BoucWen h;
  h.yield_force = 1.0;
  h.yield_displacement = 0.01;
  h.exponent = 2.0;
  EXPECT_DOUBLE_EQ(h.Rate(0.4, 10.0), 10.0 * h.Rate(0.4, 1.0));
}

TEST(SpringDamper, HysteresisTakesEnergyOut) {
  // a body on a spring with hysteresis swings less each time and settles
  // inside the loop, where the pure spring would keep swinging by 2 m g / k
  shift::core::Context context;
  LoadModel(context, "sd_hysteresis", kSlider, Module(R"(
    "stiffness": 800, "hysteresis": {"yield_force": 5,
                                     "yield_displacement": 0.002})"),
            3.0);
  std::vector<double> z;
  context.set_state_observer(
    [&z](double, const std::vector<double>& y) { z.push_back(y[0]); });
  context.Simulate();
  ASSERT_GT(z.size(), 20u);
  const auto swing = [&](size_t from, size_t to) {
    const auto range = std::minmax_element(z.begin() + from, z.begin() + to);
    return *range.second - *range.first;
  };
  EXPECT_LT(swing(z.size() / 2, z.size()), 0.2 * swing(0, z.size() / 2));
}

TEST(SpringDamper, AStructureCrushesAndStaysCrushed) {
  shift::core::Context context;
  LoadModel(context, "sd_crush_law", kSlider, Module(R"(
    "force_curve": [-1, -1000, 0, 0, 1, 1000], "unloading_stiffness": 50000)"),
            0.0);
  SpringDamper* s = Find(context);
  // loading follows the curve
  EXPECT_NEAR(s->Force(-0.2, 0.0, 0.0, 0.0), -200.0, 1e-9);
  // back from 0.2: down the stiff line to zero at 0.196, then nothing
  EXPECT_NEAR(s->Force(-0.199, 0.0, 0.0, 0.2), -150.0, 1e-9);
  EXPECT_EQ(s->Force(-0.1, 0.0, 0.0, 0.2), 0.0);
  EXPECT_EQ(s->Force(0.0, 0.0, 0.0, 0.2), 0.0);
}

TEST(SpringDamper, AStructureAbsorbsTheEnergyOfTheBlow) {
  // the body comes down on the structure under its weight, with no rebound:
  // the work of the force is the potential energy lost, in the crush
  shift::core::Context context;
  LoadModel(context, "sd_crush_drop", kSlider,
            R"({"type": "spring_damper", "name": "s",
                "frames": [["fixed", "floor"], ["body", "origin"]],
                "acts": "compression", "stiffness": 400,
                "unloading_stiffness": 40000, "smoothing": 1e-4})",
            2.0);
  SpringDamper* s = Find(context);
  double z = 0.0, crush = 0.0, absorbed = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    z = y[0];
    crush = y[s->get_state_offset()];
    absorbed = y[s->get_state_offset() + 1];
  });
  context.Simulate();
  // it stops at twice the sag, m g / k, where the work of the crush is the
  // potential energy lost, and rests there on the stiff line
  EXPECT_NEAR(crush, 2.0 * 19.62 / 400.0, 2e-3);
  EXPECT_NEAR(z, crush, 2e-3);
  EXPECT_NEAR(absorbed, 19.62 * crush, 0.05);
}

TEST(SpringDamper, RefusesConflictingInputs) {
  {
    shift::core::Context context;
    EXPECT_THROW(LoadModel(context, "sd_both", kSlider, Module(R"(
      "stiffness": 1, "force_curve": [0, 0, 1, 1])"),
                           0.0),
                 shift::core::Exception);
  }
  {
    shift::core::Context context;
    EXPECT_THROW(LoadModel(context, "sd_acts", kSlider,
                           Module(R"("acts": "sideways")"), 0.0),
                 shift::core::Exception);
  }
  {
    shift::core::Context context;
    EXPECT_THROW(LoadModel(context, "sd_shape", kSlider, Module(R"(
      "hysteresis": {"yield_force": 1, "yield_displacement": 0})"),
                           0.0),
                 shift::core::Exception);
  }
}
