/* Bushing force module (force_module::Bushing): relative coordinates and
their rates, action and reaction, coupled matrices, and a body hanging on a
bushing */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/bushing.h"

static const char* kInertia =
  R"("center_of_mass": [0, 0, 0], "inertia": {"mass": 2, "Ixx": 0.1,
     "Iyy": 0.2, "Izz": 0.3, "Ixy": 0, "Iyz": 0, "Izx": 0})";

/* a model of the given bodies, joints and force modules */
static void LoadModel(shift::core::Context& context, const std::string& name,
                      const std::string& frames, const std::string& bodies,
                      const std::string& joints,
                      const std::string& force_modules,
                      const std::string& integrator =
                        R"({"type": "rk4", "time_span": 0.01,
                            "time_step": 0.01})") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")" << testing::TempDir() << R"("],
    "solvers": [{"type": "rnca"}],
    "integrators": [)" << integrator << R"(]
  },
  "frames": [)" << frames << R"(],
  "assemblies": [{"name": "test",
    "bodies": [)" << bodies << R"(],
    "joints": [)" << joints << R"(],
    "force_modules": [)" << force_modules << R"(]
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

static shift::core::force_module::Bushing* FindBushing(
  shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto bushing = dynamic_cast<shift::core::force_module::Bushing*>(module)) {
      return bushing;
    }
  }
  return nullptr;
}

/* set q and dq, and evaluate the forces */
static void Evaluate(shift::core::Context& context,
                     const std::vector<double>& q,
                     const std::vector<double>& dq) {
  for(size_t i = 0; i < q.size(); i++) {
    context.get_q()[i] = q[i];
    context.get_dq()[i] = dq[i];
  }
  context.UpdateDHVA();
  std::vector<double> y(context.get_q());
  y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
}

/* two arms on revolute joints about different axes, joined by a bushing
between frames off their axes, with full, unsymmetric matrices */
static std::string TwoArms() {
  return std::string(R"({"name": "arm1", )") + kInertia + R"(,
    "frames": [{"name": "tip", "T": [1.0, 0.2, 0.1],
                "R": [[0.8, -0.6, 0], [0.6, 0.8, 0], [0, 0, 1]]}]},
  {"name": "arm2", )" + kInertia + R"(,
    "frames": [{"name": "tip", "T": [0.9, 0.3, -0.2]}]})";
}
static const char* kTwoArmJoints = R"(
  {"name": "A", "type": "revolute", "parent": [["fixed"], "origin"],
   "child": [["arm1"], "origin"], "axis": [0, 0, 1]},
  {"name": "B", "type": "revolute", "parent": [["fixed"], "origin"],
   "child": [["arm2"], "origin"], "axis": [1, 0, 0]})";

static std::string FullMatrix(double scale) {
  std::ostringstream s;
  for(int r = 0; r < 6; r++) {
    for(int c = 0; c < 6; c++) {
      s << (r || c ? ", " : "") << scale * (r == c ? 50.0 : (r + 1) - 0.5 * c);
    }
  }
  return s.str();
}

TEST(Bushing, RatesAreTheDerivativesOfTheDisplacements) {
  shift::core::Context context;
  LoadModel(context, "bushing_rates", "", TwoArms(), kTwoArmJoints,
            R"({"type": "bushing", "name": "b",
                "frames": [["arm1", "tip"], ["arm2", "tip"]],
                "stiffness": [)" + FullMatrix(10.0) + R"(],
                "damping": [)" + FullMatrix(1.0) + R"(]})");
  auto bushing = FindBushing(context);
  ASSERT_NE(bushing, nullptr);
  const std::vector<double> q = {0.02, -0.015};
  const std::vector<double> dq = {1.5, -0.7};
  const double h = 1e-6;
  Evaluate(context, {q[0] + h * dq[0], q[1] + h * dq[1]}, {0, 0});
  const std::vector<double> x_plus = bushing->get_displacements();
  Evaluate(context, {q[0] - h * dq[0], q[1] - h * dq[1]}, {0, 0});
  const std::vector<double> x_minus = bushing->get_displacements();
  Evaluate(context, q, dq);
  const std::vector<double>& v = bushing->get_velocities();
  const std::vector<double>& x = bushing->get_displacements();
  double rate[6];
  for(int k = 0; k < 6; k++) {
    rate[k] = (x_plus[k] - x_minus[k]) / (2.0 * h);
  }
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(v[k], rate[k], 1e-6 * std::fabs(rate[k]) + 1e-8)
      << "coordinate " << k;
  }
  /* the angular velocity w = A(r) dr/dt, where R' = [w]x R and
  A = I + (1 - cos t) / t^2 [r]x + (t - sin t) / t^3 [r]x^2, t = |r| */
  const double r[3] = {x[3], x[4], x[5]};
  const double* dr = rate + 3;
  const double t = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  ASSERT_GT(t, 1e-3);
  const double a = (1.0 - std::cos(t)) / (t * t);
  const double b = (t - std::sin(t)) / (t * t * t);
  const double r_dr[3] = {r[1] * dr[2] - r[2] * dr[1],
                          r[2] * dr[0] - r[0] * dr[2],
                          r[0] * dr[1] - r[1] * dr[0]};
  const double r_r_dr[3] = {r[1] * r_dr[2] - r[2] * r_dr[1],
                            r[2] * r_dr[0] - r[0] * r_dr[2],
                            r[0] * r_dr[1] - r[1] * r_dr[0]};
  for(int c = 0; c < 3; c++) {
    const double w = dr[c] + a * r_dr[c] + b * r_r_dr[c];
    EXPECT_NEAR(v[3 + c], w, 1e-6) << "coordinate " << 3 + c;
  }
}

TEST(Bushing, ForceFollowsTheMatricesAndLoadsBalance) {
  shift::core::Context context;
  LoadModel(context, "bushing_balance", "", TwoArms(), kTwoArmJoints,
            R"({"type": "bushing", "name": "b",
                "frames": [["arm1", "tip"], ["arm2", "tip"]],
                "stiffness": [)" + FullMatrix(10.0) + R"(],
                "damping": [)" + FullMatrix(1.0) + R"(],
                "preload": [1, 2, 3, 0.1, 0.2, 0.3]})");
  auto bushing = FindBushing(context);
  ASSERT_NE(bushing, nullptr);
  Evaluate(context, {0.3, -0.2}, {1.5, -0.7});
  const auto& x = bushing->get_displacements();
  const auto& v = bushing->get_velocities();
  const auto& f = bushing->get_forces();
  const double preload[6] = {1, 2, 3, 0.1, 0.2, 0.3};
  for(int r = 0; r < 6; r++) {
    double expected = preload[r];
    for(int c = 0; c < 6; c++) {
      const double k = r == c ? 50.0 : (r + 1) - 0.5 * c;
      expected -= 10.0 * k * x[c] + k * v[c];
    }
    EXPECT_NEAR(f[r], expected, 1e-9 * (1.0 + std::fabs(expected)));
  }
  /* equal and opposite spatial forces on the two arms */
  const int a1 = context.GetBodyLoadIndex(context.FindActiveBody("arm1"));
  const int a2 = context.GetBodyLoadIndex(context.FindActiveBody("arm2"));
  ASSERT_GE(a1, 0);
  ASSERT_GE(a2, 0);
  const auto& loads = context.get_body_loads();
  double magnitude = 0.0;
  for(int c = 0; c < 6; c++) {
    EXPECT_NEAR(loads[a1].at[c] + loads[a2].at[c], 0.0, 1e-9) << c;
    magnitude += std::fabs(loads[a2].at[c]);
  }
  EXPECT_GT(magnitude, 1.0);
}

/* a torsional bushing about the axis of a revolute joint, with frames that
coincide at q = 0: the moment on the arm is -k q about z */
TEST(Bushing, TorsionOpposesTheRotation) {
  shift::core::Context context;
  LoadModel(context, "bushing_torsion", "",
            std::string(R"({"name": "arm", )") + kInertia + "}",
            R"({"name": "A", "type": "revolute",
                "parent": [["fixed"], "origin"],
                "child": [["arm"], "origin"], "axis": [0, 0, 1]})",
            R"({"type": "bushing", "name": "b",
                "frames": [["fixed", "origin"], ["arm", "origin"]],
                "stiffness": [0, 0, 0, 0, 0, 40], "rest_pose": "frames"})");
  auto bushing = FindBushing(context);
  ASSERT_NE(bushing, nullptr);
  Evaluate(context, {0.2}, {0.0});
  EXPECT_NEAR(bushing->get_displacements()[5], 0.2, 1e-12);
  EXPECT_NEAR(bushing->get_forces()[5], -8.0, 1e-10);
  const int a = context.GetBodyLoadIndex(context.FindActiveBody("arm"));
  const auto& load = context.get_body_loads()[a];
  EXPECT_NEAR(load.ang.z, -8.0, 1e-10);
  EXPECT_NEAR(load.lin.x, 0.0, 1e-12);
}

/* a body on a vertical slider, hanging on a damped bushing from the fixed
body: it settles where the bushing carries its weight, z = m g / k */
TEST(Bushing, HangingBodySettlesUnderItsWeight) {
  shift::core::Context context;
  const double k = 2000.0, m = 2.0, g = 9.81;
  const double c = 2.0 * 0.7 * std::sqrt(k * m);
  std::ostringstream modules;
  modules << R"({"type": "bushing", "name": "b",
                 "frames": [["fixed", "origin"], ["body", "origin"]],
                 "stiffness": [1, 1, )" << k << R"(, 1, 1, 1],
                 "damping": [0, 0, )" << c << R"(, 0, 0, 0]})";
  LoadModel(context, "bushing_hanging", "",
            std::string(R"({"name": "body", )") + kInertia + "}",
            R"({"name": "C", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["body"], "origin"], "axis": [0, 0, 1]})",
            modules.str(),
            R"({"type": "ode113", "time_span": 2.0, "time_step": 0.01,
                "relative_error": 1e-8, "absolute_error": 1e-10})");
  double z = 0.0;
  context.set_state_observer(
    [&z](double, const std::vector<double>& y) { z = y[0]; });
  context.Simulate();
  EXPECT_NEAR(z, m * g / k, 1e-6);
}

/* three frames: three pairs, (0, 1), (0, 2), (1, 2), with an 18 x 18
matrix from a file */
TEST(Bushing, GroupReadsItsMatrixFromAFile) {
  {
    std::ofstream file(testing::TempDir() + "/group_stiffness.txt");
    for(int r = 0; r < 18; r++) {
      for(int c = 0; c < 18; c++) {
        file << (r == c ? 100.0 : 0.0) << (c < 17 ? ", " : "\n");
      }
    }
  }
  const std::string bodies =
    std::string(R"({"name": "a", )") + kInertia + R"(}, {"name": "b", )" +
    kInertia + "}";
  const char* joints = R"(
    {"name": "A", "type": "revolute", "parent": [["fixed"], "origin"],
     "child": [["a"], "origin"], "axis": [0, 0, 1]},
    {"name": "B", "type": "revolute", "parent": [["fixed"], "origin"],
     "child": [["b"], "origin"], "axis": [0, 0, 1]})";
  shift::core::Context context;
  LoadModel(context, "bushing_group", "", bodies, joints,
            R"({"type": "bushing", "name": "g",
                "frames": [["fixed", "origin"], ["a", "origin"],
                           ["b", "origin"]],
                "stiffness_file": "group_stiffness.txt",
                "rest_pose": "frames"})");
  auto bushing = FindBushing(context);
  ASSERT_NE(bushing, nullptr);
  EXPECT_EQ(bushing->get_pair_count(), 3u);
  Evaluate(context, {0.1, 0.3}, {0, 0});
  const auto& x = bushing->get_displacements();
  EXPECT_NEAR(x[5], 0.1, 1e-12);       // a relative to the fixed body
  EXPECT_NEAR(x[11], 0.3, 1e-12);      // b relative to the fixed body
  EXPECT_NEAR(x[17], 0.2, 1e-12);      // b relative to a
  EXPECT_NEAR(bushing->get_forces()[17], -20.0, 1e-10);

  /* a matrix of the wrong size */
  shift::core::Context wrong;
  EXPECT_THROW(LoadModel(wrong, "bushing_wrong", "", bodies, joints,
                         R"({"type": "bushing", "name": "g",
                             "frames": [["fixed", "origin"], ["a", "origin"],
                                        ["b", "origin"]],
                             "stiffness": [1, 2, 3, 4, 5, 6]})"),
               shift::core::Exception);
}
