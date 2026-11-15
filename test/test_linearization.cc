/* Linearization of a model about a state (shift::core::Linearization): the
eigenvalues of models whose modes are known in closed form */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/integrator/base.h"
#include "shift/core/linearization.h"

static std::string Model(const std::string& rest) {
  return R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.05, "time_step": 0.01}]},
 )" + rest + "}";
}

/* the modes of the model after a short run, the fastest first */
static std::vector<shift::core::Linearization::Mode>
Modes(const std::string& json, std::vector<std::string>* names = nullptr) {
  const std::string path = testing::TempDir() + "/linearization_test.json";
  {
    std::ofstream file(path);
    file << json;
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  double time = 0.0;
  std::vector<double> state;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    time = t;
    state = y;
  });
  context.Simulate();
  EXPECT_FALSE(state.empty());
  const double omega_n =
    context.get_active_integrators()[0]->get_baumgarte_omega_n();
  shift::core::Linearization linearization(context);
  linearization.Compute(time, state);
  // the stabilization is the integrator's again
  EXPECT_EQ(context.get_active_integrators()[0]->get_baumgarte_omega_n(),
            omega_n);
  if(names) {
    *names = linearization.get_participant_names();
  }
  return linearization.Modes();
}

TEST(Linearization, DampedSpringMass) {
  // m = 1 kg, k = 400 N/m, c = 4 N s/m: -2 +- i sqrt(396)
  const auto modes = Modes(Model(R"("assemblies": [{"name": "a",
  "bodies": [{"name": "mass", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                "Izx": 0}}],
  "joints": [{"name": "slide", "type": "prismatic", "axis": [1, 0, 0],
    "parent": [["fixed"], "origin"], "child": [["mass"], "origin"],
    "stiffness_coefficients": [0.0, 400.0],
    "damping_coefficients": [0.0, 4.0]}]}])"));
  ASSERT_EQ(modes.size(), 1u);
  EXPECT_NEAR(modes[0].eigenvalue.real(), -2.0, 1e-5);
  EXPECT_NEAR(modes[0].eigenvalue.imag(), std::sqrt(396.0), 1e-5);
  ASSERT_EQ(modes[0].participation.size(), 1u);
  EXPECT_NEAR(modes[0].participation[0], 1.0, 0.02);
}

TEST(Linearization, SphericalPendulumThroughItsQuaternion) {
  // m = 2 kg at d = 1 m below a ball joint, 0.1 kg m^2 about its center:
  // two swings at sqrt(m g d / (I + m d^2)), the spin about the vertical
  // at 0
  const auto modes = Modes(Model(R"("assemblies": [{"name": "a",
  "bodies": [{"name": "bob", "center_of_mass": [0, 0, 1],
    "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.05, "Ixy": 0,
                "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "ball", "type": "spherical",
    "parent": [["fixed"], "origin"], "child": [["bob"], "origin"]}]}])"));
  const double omega = std::sqrt(2.0 * 9.81 / (0.1 + 2.0));
  ASSERT_GE(modes.size(), 3u);
  for(int i = 0; i < 2; i++) {
    EXPECT_NEAR(modes[i].eigenvalue.real(), 0.0, 1e-5);
    EXPECT_NEAR(modes[i].eigenvalue.imag(), omega, 1e-5);
  }
  EXPECT_NEAR(std::abs(modes[2].eigenvalue), 0.0, 1e-5);
}

TEST(Linearization, ClosedLoopKeepsOnlyTheMotionOfTheLoop) {
  // a parallelogram hanging from two pivots 1 m apart: cranks of 1 kg (0.26
  // kg m^2 about the pivot), a coupler of 3 kg that only translates:
  // omega^2 = g (M L + m L) / (M L^2 + 2 I). The perturbations that break
  // the loop drift (eigenvalue 0) without the Baumgarte stabilization
  std::vector<std::string> names;
  const auto modes = Modes(Model(R"("frames": [{"name": "pivot2", "T": [0, 1, 0]}],
 "assemblies": [{"name": "a",
  "bodies": [
   {"name": "crank1", "center_of_mass": [0, 0, 0.5],
    "frames": [{"name": "distal", "T": [0, 0, 1]}],
    "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": 0.001, "Ixy": 0,
                "Iyz": 0, "Izx": 0}},
   {"name": "crank2", "center_of_mass": [0, 0, 0.5],
    "frames": [{"name": "distal", "T": [0, 0, 1]}],
    "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": 0.001, "Ixy": 0,
                "Iyz": 0, "Izx": 0}},
   {"name": "coupler", "center_of_mass": [0, 0.5, 0],
    "frames": [{"name": "right", "T": [0, 1, 0]}],
    "inertia": {"mass": 3, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1, "Ixy": 0,
                "Iyz": 0, "Izx": 0}}],
  "joints": [
   {"name": "a1", "type": "revolute", "axis": [1, 0, 0],
    "parent": [["fixed"], "origin"], "child": [["crank1"], "origin"]},
   {"name": "a2", "type": "revolute", "axis": [1, 0, 0],
    "parent": [["fixed"], "pivot2"], "child": [["crank2"], "origin"]},
   {"name": "b1", "type": "revolute", "axis": [1, 0, 0],
    "parent": [["crank1"], "distal"], "child": [["coupler"], "origin"]},
   {"name": "b2", "type": "revolute", "axis": [1, 0, 0],
    "parent": [["coupler"], "right"], "child": [["crank2"], "distal"]}]}])"),
                           &names);
  const double omega = std::sqrt(9.81 * (3.0 + 1.0) / (3.0 + 2.0 * 0.26));
  ASSERT_GE(modes.size(), 2u);
  EXPECT_NEAR(modes[0].eigenvalue.real(), 0.0, 1e-5);
  EXPECT_NEAR(modes[0].eigenvalue.imag(), omega, 1e-5);
  for(size_t i = 1; i < modes.size(); i++) {
    EXPECT_LT(std::abs(modes[i].eigenvalue), 1e-4) << i;
  }
  // the cranks swing together
  ASSERT_EQ(names.size(), modes[0].participation.size());
  const auto at = [&](const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), "a__" + name +
                                                            "__dq_0");
    EXPECT_NE(it, names.end()) << name;
    return modes[0].participation[it - names.begin()];
  };
  EXPECT_NEAR(at("a1"), 0.5, 0.02);
  EXPECT_NEAR(at("a2"), 0.5, 0.02);
}
