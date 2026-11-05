/* Rubber mount force module (force_module::RubberMount): the forces and
state rates of its Maxwell elements, and a body settling on the static
stiffness alone */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/rubber_mount.h"

using shift::core::force_module::RubberMount;

/* a body on a vertical slider below the fixed body, held by a mount */
static void LoadMount(shift::core::Context& context, const std::string& name,
                      const std::string& mount, double time_span) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
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
         << time_span << R"(, "time_step": 0.01,
                     "relative_error": 1e-8, "absolute_error": 1e-10}]
  },
  "assemblies": [{"name": "test",
    "bodies": [{"name": "body", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3,
                  "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "C", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["body"], "origin"], "axis": [0, 0, 1]}],
    "force_modules": [{"type": "rubber_mount", "name": "mount",
      "frames": [["fixed", "origin"], ["body", "origin"]], )"
         << mount << R"(}]
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

static RubberMount* FindMount(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto mount = dynamic_cast<RubberMount*>(module)) {
      return mount;
    }
  }
  return nullptr;
}

static const char* kMount = R"(
  "stiffness": [1, 1, 2000, 1, 1, 1], "damping": [0, 0, 20, 0, 0, 0],
  "cubic_stiffness": [0, 0, 1e6, 0, 0, 0],
  "maxwell_stiffness": [0, 0, 5000, 0, 0, 0, 0, 0, 3000, 0, 0, 0],
  "maxwell_damping": [0, 0, 50, 0, 0, 0, 0, 0, 300, 0, 0, 0])";

/* at a state of the slider and of the two springs: the force and the rates
of the springs' stretches */
TEST(RubberMount, ForceAndStateRatesFollowTheElements) {
  shift::core::Context context;
  LoadMount(context, "mount_rates", kMount, 0.01);
  auto mount = FindMount(context);
  ASSERT_NE(mount, nullptr);
  ASSERT_EQ(mount->get_element_count(), 2u);
  ASSERT_EQ(context.get_auxiliary_state_count(), 2u);
  const double x = 0.01, v = 0.3, s1 = 0.004, s2 = -0.002;
  context.get_q()[0] = x;
  context.get_dq()[0] = v;
  context.UpdateDHVA();
  /* y = [q, dq, the springs' stretches] */
  std::vector<double> y(context.get_state_count() + 2, 0.0);
  ASSERT_EQ(context.get_q_count(), 1u);
  context.InitializeAuxiliaryStates(y.data() + context.get_state_count());
  y[0] = x;
  y[1] = v;
  y[2] = s1;
  y[3] = s2;
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
  const double expected =
    -2000 * x - 20 * v - 1e6 * x * x * x - 5000 * s1 - 3000 * s2;
  EXPECT_NEAR(mount->get_forces()[2], expected, 1e-10);
  EXPECT_NEAR(dy[dy.size() - 2], v - 100.0 * s1, 1e-12);
  EXPECT_NEAR(dy[dy.size() - 1], v - 10.0 * s2, 1e-12);
}

/* hanging on the mount, the body settles where the static stiffness alone
carries its weight, k x + k3 x^3 = m g: the springs of the Maxwell
elements relax through their dampers */
TEST(RubberMount, StaticLoadRelaxesTheMaxwellElements) {
  shift::core::Context context;
  LoadMount(context, "mount_static", kMount, 3.0);
  std::vector<double> y;
  context.set_state_observer(
    [&y](double, const std::vector<double>& state) { y = state; });
  context.Simulate();
  ASSERT_EQ(y.size(), context.get_state_count() + 2);
  const double weight = 2.0 * 9.81;
  double x = weight / 2000.0;
  for(int k = 0; k < 50; k++) {
    x -= (2000 * x + 1e6 * x * x * x - weight) / (2000 + 3e6 * x * x);
  }
  EXPECT_NEAR(y[0], x, 1e-7);
  EXPECT_NEAR(y[y.size() - 2], 0.0, 1e-7);
  EXPECT_NEAR(y[y.size() - 1], 0.0, 1e-7);
}

/* a quick motion meets the elements' springs as well: right after a fast
push from rest, the stretch of each spring is the displacement */
TEST(RubberMount, FastMotionLoadsTheSprings) {
  shift::core::Context context;
  LoadMount(context, "mount_fast", R"(
    "stiffness": [1, 1, 10, 1, 1, 1],
    "maxwell_stiffness": [0, 0, 1e3, 0, 0, 0],
    "maxwell_damping": [0, 0, 1e3, 0, 0, 0])", 0.002);
  auto mount = FindMount(context);
  ASSERT_NE(mount, nullptr);
  /* starting with a high speed, the body moves 2 mm in 2 ms, much faster
  than the element's time constant of 1 s */
  context.get_dq()[0] = 1.0;
  std::vector<double> y;
  context.set_state_observer(
    [&y](double, const std::vector<double>& state) { y = state; });
  context.Simulate();
  ASSERT_FALSE(y.empty());
  EXPECT_NEAR(y.back(), y[0], 2e-3 * y[0]);
}

TEST(RubberMount, RejectsAnElementWithoutDamping) {
  shift::core::Context context;
  EXPECT_THROW(LoadMount(context, "mount_bad", R"(
    "maxwell_stiffness": [0, 0, 1e3, 0, 0, 0],
    "maxwell_damping": [0, 0, 0, 0, 0, 0])", 0.01),
               shift::core::Exception);
}
