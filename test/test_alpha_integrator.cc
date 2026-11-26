/* The implicit integrators of the generalized-alpha family
(integrator::GeneralizedAlpha, integrator::HHT) and the position increments
they take (Context::IncrementPositions) */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <json/json.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/integrator/alpha.h"
#include "shift/core/jacobian.h"
#include "shift/core/solver/base.h"
#include "shift/core/tangents.h"

static std::string Model(const std::string& integrator,
                         const std::string& assemblies) {
  return R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [)" +
         integrator + R"(]}, "assemblies": [)" + assemblies + "]}";
}

/* the states reported by simulating the model given: times and states */
struct Trajectory {
  std::vector<double> t;
  std::vector<std::vector<double>> y;
  unsigned int accepted_steps = 0;
  unsigned int rejected_steps = 0;
  unsigned int iteration_matrices = 0;
  unsigned int rescaled_matrices = 0;
  unsigned int newton_iterations = 0;
};

static Trajectory Simulate(shift::core::Context& context) {
  Trajectory run;
  context.set_state_observer([&](double time, const std::vector<double>& s) {
    run.t.push_back(time);
    run.y.push_back(s);
  });
  context.Simulate();
  auto* alpha = dynamic_cast<shift::core::integrator::Alpha*>(
    context.get_active_integrators()[0]);
  if(alpha) {
    run.accepted_steps = alpha->get_accepted_steps();
    run.rejected_steps = alpha->get_rejected_steps();
    run.iteration_matrices = alpha->get_iteration_matrix_count();
    run.rescaled_matrices = alpha->get_rescaled_matrix_count();
    run.newton_iterations = alpha->get_newton_iterations();
  }
  return run;
}

static Trajectory SimulateJson(const std::string& json) {
  const std::string path = testing::TempDir() + "/alpha_integrator_test.json";
  {
    std::ofstream file(path);
    file << json;
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  return Simulate(context);
}

/* a mass on a spring along z: m = 1 kg, k = 400 N/m, falling from rest at
the unstretched length under gravity, x = g / w^2 (1 - cos w t), w = 20 */
static const char* kSpringMass = R"({"name": "a",
  "bodies": [{"name": "mass", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                "Izx": 0}}],
  "joints": [{"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
    "parent": [["fixed"], "origin"], "child": [["mass"], "origin"],
    "stiffness_coefficients": [0.0, 400.0]}]})";

static void CheckSpringMass(const std::string& integrator) {
  SCOPED_TRACE(integrator);
  const Trajectory run = SimulateJson(Model(integrator, kSpringMass));
  ASSERT_GT(run.t.size(), 10u);
  const double g = 9.81, w = 20.0, amplitude = g / (w * w);
  for(size_t i = 0; i < run.t.size(); i++) {
    const double exact = amplitude * (1.0 - std::cos(w * run.t[i]));
    EXPECT_NEAR(run.y[i][0], exact, 1e-3 * amplitude) << "t = " << run.t[i];
  }
}

TEST(AlphaIntegrator, SpringMassGeneralizedAlpha) {
  CheckSpringMass(R"({"type": "generalized_alpha", "time_span": 1.0,
    "time_step": 0.01, "absolute_error": 1e-9, "relative_error": 1e-7})");
}

TEST(AlphaIntegrator, SpringMassHHT) {
  CheckSpringMass(R"({"type": "hht", "alpha": -0.2, "time_span": 1.0,
    "time_step": 0.01, "absolute_error": 1e-9, "relative_error": 1e-7})");
}

/* the spring-mass with steps that alternate between 0.7 and 0.3 of a
period T (hmax 0.7 T, each report at T): second order, the largest error
falling 4 times when T is halved. The pseudo-accelerations of the last step
are scaled to the size of the next (Jay and Negrut); without the scaling the
error fell 2.9 times for HHT and 3.5 times for the generalized-alpha method
from T = 0.005 s to 0.0025 s, and fell further below 4 as T decreased. */
TEST(AlphaIntegrator, VariableStepSize) {
  for(const std::string parameters :
      {R"("type": "hht", "alpha": -0.1)",
       R"("type": "generalized_alpha", "rho_infinity": 0.8)"}) {
    SCOPED_TRACE(parameters);
    double errors[2];
    for(int k = 0; k < 2; k++) {
      const double period = 0.005 / (1 << k);
      const std::string integrator =
        "{" + parameters + R"(, "time_span": 1.0, "time_step": )" +
        std::to_string(period) + R"(, "hmax": )" +
        std::to_string(0.7 * period) +
        R"(, "absolute_error": 0.1, "relative_error": 0.1})";
      const Trajectory run = SimulateJson(Model(integrator, kSpringMass));
      ASSERT_GT(run.t.size(), 100u);
      const double g = 9.81, w = 20.0, amplitude = g / (w * w);
      errors[k] = 0.0;
      for(size_t i = 0; i < run.t.size(); i++) {
        const double exact = amplitude * (1.0 - std::cos(w * run.t[i]));
        errors[k] = std::max(errors[k], std::fabs(run.y[i][0] - exact));
      }
    }
    EXPECT_GT(errors[0] / errors[1], 3.8);
    EXPECT_LT(errors[0] / errors[1], 4.2);
  }
}

/* a mass held by a PID controller, whose integral of the error is an
internal state, with steps of hmax (each report at 0.01 s): with numerical
damping the coordinate, the velocity and the integral are second order, the
error at t = 1 s against ODE113 falling 4 times when hmax is halved. The
internal state advances with pseudo-rates; by the trapezoidal rule with
weight gamma on its rate the errors fell 1.5 to 1.8 times (first order), the
coordinate's too. */
TEST(AlphaIntegrator, InternalStateOrder) {
  const char* assembly = R"({"name": "a",
  "bodies": [{"name": "mass", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                "Izx": 0}}],
  "joints": [{"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
    "parent": [["fixed"], "origin"], "child": [["mass"], "origin"]}],
  "sensors": [{"type": "joint", "name": "x", "joint": "slide",
               "quantity": "position"}],
  "controllers": [{"type": "pid", "name": "hold", "kp": 400.0, "ki": 2000.0,
                   "kd": 10.0, "target": 0.1, "sensor": "x",
                   "joint": "slide"}]})";
  const Trajectory reference = SimulateJson(Model(
    R"({"type": "ode113", "time_span": 1.0, "time_step": 0.01,
        "absolute_error": 1e-12, "relative_error": 1e-12})",
    assembly));
  ASSERT_EQ(reference.y.back().size(), 3u); // q, dq and the integral
  for(const std::string parameters :
      {R"("type": "hht", "alpha": -0.1)",
       R"("type": "generalized_alpha", "rho_infinity": 0.8)"}) {
    SCOPED_TRACE(parameters);
    double errors[2][3];
    for(int k = 0; k < 2; k++) {
      const std::string integrator =
        "{" + parameters + R"(, "time_span": 1.0, "time_step": 0.01, "hmax": )" +
        std::to_string(0.0025 / (1 << k)) +
        R"(, "absolute_error": 10, "relative_error": 10})";
      const Trajectory run = SimulateJson(Model(integrator, assembly));
      ASSERT_EQ(run.y.back().size(), 3u);
      for(int i = 0; i < 3; i++) {
        errors[k][i] = std::fabs(run.y.back()[i] - reference.y.back()[i]);
      }
    }
    for(int i = 0; i < 3; i++) {
      EXPECT_GT(errors[0][i] / errors[1][i], 3.8) << "state " << i;
      EXPECT_LT(errors[0][i] / errors[1][i], 4.2) << "state " << i;
    }
  }
}

/* a stiff, overdamped mass on a spring: k = 1e6 N/m, c = 1e4 N s/m, modes at
about -100 and -9900 1/s. An explicit method takes steps below 2 / 9900 s
throughout; the implicit one follows the transient and then takes steps of
hmax. It settles at g / k. The equations are linear, so that the iteration
matrix scaled for each new step size (without being formed again) is exact,
and Newton converges in at most two iterations a step. A matrix kept for a
step size within kReformFactor of its own is not exact for the stiff mode,
and is formed again a few times in the transient. */
TEST(AlphaIntegrator, StiffSpringMass) {
  const Trajectory run = SimulateJson(Model(
    R"({"type": "generalized_alpha", "time_span": 1.0, "time_step": 0.01,
        "absolute_error": 1e-9, "relative_error": 1e-6})",
    R"({"name": "a",
  "bodies": [{"name": "mass", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                "Izx": 0}}],
  "joints": [{"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
    "parent": [["fixed"], "origin"], "child": [["mass"], "origin"],
    "stiffness_coefficients": [0.0, 1.0e6],
    "damping_coefficients": [0.0, 1.0e4]}]})"));
  ASSERT_FALSE(run.y.empty());
  EXPECT_NEAR(run.y.back()[0], 9.81 / 1e6, 1e-9);
  EXPECT_LT(run.accepted_steps, 400u);
  EXPECT_GT(run.rescaled_matrices, 10u);
  EXPECT_LE(run.iteration_matrices, 6u);
  EXPECT_LE(run.newton_iterations,
            2u * (run.accepted_steps + run.rejected_steps));
}

/* the stiff mass held at a target by a PID controller with a stiff integral
term, an internal state of the Newton system (through the Schur complement):
linear, so that the iteration matrix scaled for a new step size, the blocks
of the internal state too, is exact at that step size, and Newton converges
in about two iterations a step with a matrix formed a few times (17 times
with the blocks of the internal state left at the old step size). It settles
at the target. */
TEST(AlphaIntegrator, RescaledInternalStates) {
  const Trajectory run = SimulateJson(Model(
    R"({"type": "hht", "time_span": 1.0, "time_step": 0.01,
        "absolute_error": 1e-5, "relative_error": 1e-3})",
    R"({"name": "a",
  "bodies": [{"name": "mass", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                "Izx": 0}}],
  "joints": [{"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
    "parent": [["fixed"], "origin"], "child": [["mass"], "origin"],
    "stiffness_coefficients": [0.0, 1.0e4],
    "damping_coefficients": [0.0, 1.0e3]}],
  "sensors": [{"type": "joint", "name": "x", "joint": "slide",
               "quantity": "position"}],
  "controllers": [{"type": "pid", "name": "hold", "kp": 1.0e6, "ki": 1.0e8,
                   "kd": 1.0e4, "target": 0.01, "sensor": "x",
                   "joint": "slide"}]})"));
  ASSERT_FALSE(run.y.empty());
  ASSERT_EQ(run.y.back().size(), 3u); // q, dq and the integral of the error
  EXPECT_NEAR(run.y.back()[0], 0.01, 1e-6);
  EXPECT_GT(run.rescaled_matrices, 10u);
  EXPECT_LE(run.iteration_matrices, 10u);
  EXPECT_LE(run.newton_iterations,
            2u * (run.accepted_steps + run.rejected_steps));
}

/* the position increments: to first order the rates of ConvertdqTody, unit
quaternions, and a rotation about a fixed axis of the child frame in two
increments is that in one */
TEST(AlphaIntegrator, IncrementPositions) {
  const std::string path = testing::TempDir() + "/increment_positions.json";
  {
    std::ofstream file(path);
    file << Model(R"({"type": "ode113", "time_span": 0.01,
                      "time_step": 0.01})",
                  R"({"name": "a",
  "bodies": [
   {"name": "bob", "center_of_mass": [0, 0, 1],
    "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.05, "Ixy": 0,
                "Iyz": 0, "Izx": 0}},
   {"name": "free", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "ball", "type": "spherical",
    "parent": [["fixed"], "origin"], "child": [["bob"], "origin"]}]})");
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  std::vector<double> y;
  context.set_state_observer(
    [&](double, const std::vector<double>& s) { y = s; });
  context.Simulate();
  ASSERT_FALSE(y.empty());

  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();
  ASSERT_EQ(q_count, 4u + 7u);
  ASSERT_EQ(dq_count, 3u + 6u);

  /* a generic orientation of both joints */
  std::mt19937 generator(12345);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  std::vector<double> q(y.begin(), y.begin() + q_count);
  std::vector<double> turn(dq_count);
  for(double& value : turn) {
    value = uniform(generator);
  }
  context.IncrementPositions(q.data(), turn.data(), q.data());

  std::vector<double> delta(dq_count);
  for(double& value : delta) {
    value = uniform(generator);
  }
  const double step = 1e-7;
  std::vector<double> scaled(dq_count);
  for(unsigned int i = 0; i < dq_count; i++) {
    scaled[i] = step * delta[i];
  }
  std::vector<double> q_out(q_count);
  context.IncrementPositions(q.data(), scaled.data(), q_out.data());

  std::vector<double> state(y.size(), 0.0);
  std::copy(q.begin(), q.end(), state.begin());
  std::copy(delta.begin(), delta.end(), state.begin() + q_count);
  std::vector<double> rates(y.size(), 0.0);
  context.ConvertdqTody(state, rates);
  for(unsigned int i = 0; i < q_count; i++) {
    EXPECT_NEAR((q_out[i] - q[i]) / step, rates[i], 1e-5) << "q " << i;
  }

  /* the quaternions stay unit: the ball's at 0, the floating joint's after
  its translation, at 4 + 3 */
  for(unsigned int offset : {0u, 7u}) {
    double norm = 0.0;
    for(int m = 0; m < 4; m++) {
      norm += q_out[offset + m] * q_out[offset + m];
    }
    EXPECT_NEAR(norm, 1.0, 1e-14) << "quaternion at " << offset;
  }

  /* the ball's rotation, child frame: twice delta equals once 2 delta */
  std::vector<double> rotation(dq_count, 0.0);
  std::vector<double> double_rotation(dq_count, 0.0);
  for(int m = 0; m < 3; m++) {
    rotation[m] = 0.4 * delta[m];
    double_rotation[m] = 0.8 * delta[m];
  }
  std::vector<double> twice(q), once(q_count);
  context.IncrementPositions(twice.data(), rotation.data(), twice.data());
  context.IncrementPositions(twice.data(), rotation.data(), twice.data());
  context.IncrementPositions(q.data(), double_rotation.data(), once.data());
  for(int m = 0; m < 4; m++) {
    EXPECT_NEAR(twice[m], once[m], 1e-14) << "quaternion " << m;
  }
}

/* the example model given, simulated with the integrator given */
static Json::Value Example(const std::string& name,
                           const Json::Value& integrator) {
  std::ifstream file(std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  Json::Value model;
  Json::CharReaderBuilder builder;
  std::string errors;
  EXPECT_TRUE(Json::parseFromStream(builder, file, &model, &errors)) << errors;
  Json::Value& parameters = model["model_parameters"];
  parameters["integrators"] = Json::Value(Json::arrayValue);
  parameters["integrators"].append(integrator);
  parameters["loggers"] = Json::Value(Json::arrayValue);
  return model;
}

/* models with kinematic loops: the motion of ODE113 at tight tolerances, and
the position constraints hold at the end, without Baumgarte stabilization */
static void CheckLoop(const std::string& name) {
  SCOPED_TRACE(name);
  Json::Value ode113;
  ode113["type"] = "ode113";
  ode113["time_span"] = 0.5;
  ode113["time_step"] = 0.01;
  ode113["absolute_error"] = 1e-10;
  ode113["relative_error"] = 1e-10;
  Json::Value alpha;
  alpha["type"] = "generalized_alpha";
  alpha["time_span"] = 0.5;
  alpha["time_step"] = 0.01;
  alpha["absolute_error"] = 1e-9;
  alpha["relative_error"] = 1e-7;

  const std::string path = std::string(SHIFT_EXAMPLES_DIR) + "/" + name;
  shift::core::Context reference_context;
  reference_context.LoadJSON(Example(name, ode113), path);
  const Trajectory reference = Simulate(reference_context);
  shift::core::Context context;
  context.LoadJSON(Example(name, alpha), path);
  const Trajectory run = Simulate(context);

  ASSERT_FALSE(run.y.empty());
  ASSERT_EQ(run.y.size(), reference.y.size());
  const unsigned int q_count = context.get_q_count();
  for(unsigned int i = 0; i < q_count; i++) {
    EXPECT_NEAR(run.y.back()[i], reference.y.back()[i], 1e-4) << "q " << i;
  }
  EXPECT_LT(context.get_rms_constraint_error(), 1e-8);
}

TEST(AlphaIntegrator, SliderCrank) {
  CheckLoop("slider_crank/slider_crank.json");
}

TEST(AlphaIntegrator, Bricard) {
  CheckLoop("bricard/bricard.json");
}

/* the partition changes during the run (the refactor criterion of the
refactor_criterion test): the integrator forms the iteration matrix again in
the new partition, and the motion is that of ODE113 */
TEST(AlphaIntegrator, PartitionChanges) {
  const std::string name = "klann_linkage/klann_linkage.json";
  const double time_span = 4.0;
  Json::Value solver;
  solver["type"] = "rnca";
  solver["refactor_criterion"] = "mobility_epsilon";
  solver["epsilon_mobility"] = 0.1;
  Json::Value ode113;
  ode113["type"] = "ode113";
  ode113["time_span"] = time_span;
  ode113["time_step"] = 0.01;
  ode113["absolute_error"] = 1e-10;
  ode113["relative_error"] = 1e-10;
  Json::Value alpha;
  alpha["type"] = "generalized_alpha";
  alpha["time_span"] = time_span;
  alpha["time_step"] = 0.01;
  alpha["absolute_error"] = 1e-9;
  alpha["relative_error"] = 1e-7;

  const std::string path = std::string(SHIFT_EXAMPLES_DIR) + "/" + name;
  Json::Value reference_model = Example(name, ode113);
  reference_model["model_parameters"]["solvers"][0] = solver;
  shift::core::Context reference_context;
  reference_context.LoadJSON(reference_model, path);
  const Trajectory reference = Simulate(reference_context);

  Json::Value model = Example(name, alpha);
  model["model_parameters"]["solvers"][0] = solver;
  shift::core::Context context;
  context.LoadJSON(model, path);
  const Trajectory run = Simulate(context);

  EXPECT_GT(context.get_jacobian().get_infinity_norm_failures().size(), 0u);
  ASSERT_EQ(run.y.size(), reference.y.size());
  const unsigned int q_count = context.get_q_count();
  for(unsigned int i = 0; i < q_count; i++) {
    EXPECT_NEAR(run.y.back()[i], reference.y.back()[i], 1e-3) << "q " << i;
  }
  EXPECT_LT(context.get_rms_constraint_error(), 1e-8);
}

/* the example model given over time_span with ODE113 at tight tolerances
and with HHT at the model's own; the HHT integrator in alpha */
static void CompareHHT(const std::string& name, double time_span,
                       double tolerance,
                       shift::core::Context& context,
                       const shift::core::integrator::Alpha*& alpha) {
  const std::string path = std::string(SHIFT_EXAMPLES_DIR) + "/" + name;
  Json::Value ode113;
  ode113["type"] = "ode113";
  ode113["time_span"] = time_span;
  ode113["time_step"] = time_span / 10.0;
  ode113["absolute_error"] = 1e-9;
  ode113["relative_error"] = 1e-9;
  shift::core::Context reference_context;
  reference_context.LoadJSON(Example(name, ode113), path);
  const Trajectory reference = Simulate(reference_context);

  Json::Value hht;
  hht["type"] = "hht";
  hht["time_span"] = time_span;
  hht["time_step"] = time_span / 10.0;
  hht["absolute_error"] = 1e-5;
  hht["relative_error"] = 1e-5;
  context.LoadJSON(Example(name, hht), path);
  const Trajectory run = Simulate(context);
  alpha = dynamic_cast<const shift::core::integrator::Alpha*>(
    context.get_active_integrators()[0]);
  ASSERT_TRUE(alpha);
  ASSERT_EQ(run.y.size(), reference.y.size());
  for(unsigned int i = 0; i < context.get_q_count(); i++) {
    EXPECT_NEAR(run.y.back()[i], reference.y.back()[i], tolerance)
      << "q " << i;
  }
}

/* the internal states are part of the error estimate: the pressures of the
excavator's hydraulic cylinders at t = 0.6 s with HHT at tolerances of 1e-4
are within 3% of ODE113's at 1e-9 (0.9% here). With the estimate over the
displacements alone the steps did not follow the pressures, which were 9%
off (16% at t = 2 s). */
TEST(AlphaIntegrator, InternalStatesInErrorEstimate) {
  const std::string name = "excavator/excavator.json";
  const std::string path = std::string(SHIFT_EXAMPLES_DIR) + "/" + name;
  const double time_span = 0.6;
  Json::Value ode113;
  ode113["type"] = "ode113";
  ode113["time_span"] = time_span;
  ode113["time_step"] = time_span / 10.0;
  ode113["absolute_error"] = 1e-9;
  ode113["relative_error"] = 1e-9;
  shift::core::Context reference_context;
  reference_context.LoadJSON(Example(name, ode113), path);
  const Trajectory reference = Simulate(reference_context);
  Json::Value hht;
  hht["type"] = "hht";
  hht["time_span"] = time_span;
  hht["time_step"] = time_span / 10.0;
  hht["absolute_error"] = 1e-4;
  hht["relative_error"] = 1e-4;
  shift::core::Context context;
  context.LoadJSON(Example(name, hht), path);
  const Trajectory run = Simulate(context);
  ASSERT_EQ(run.y.size(), reference.y.size());
  const unsigned int first = context.get_state_count();
  ASSERT_GT(run.y.back().size(), first);
  for(size_t i = first; i < run.y.back().size(); i++) {
    const double expected = reference.y.back()[i];
    EXPECT_LT(std::fabs(run.y.back()[i] - expected) /
                std::max(1.0, std::fabs(expected)),
              3e-2)
      << "internal state " << i - first;
  }
}

/* the shoes of a track on the ground plane: a station's plastic sinkage and
each shoe's shear displacement, 353 internal states whose derivatives
track_soil gives (each rate depends on its own state alone), are eliminated
before the coordinates and velocities. HHT at 1e-5 ends within 1.5e-3 of
ODE113 at 1e-9 with either order of elimination */
TEST(AlphaIntegrator, ManyInternalStates) {
  shift::core::Context context;
  const shift::core::integrator::Alpha* alpha = nullptr;
  ASSERT_NO_FATAL_FAILURE(CompareHHT("terramechanics/track_frame_soil.json",
                                     0.05, 5e-3, context, alpha));
  EXPECT_GT(context.get_auxiliary_state_count(), 300u);
  EXPECT_GT(alpha->get_states_first_count(), 0u);
}

/* the pressures of the hydraulic cylinders are stiff internal states, which
are unknowns of Newton (the Schur complement): the steps are not rejected
for want of convergence */
TEST(AlphaIntegrator, StiffInternalStates) {
  shift::core::Context context;
  const shift::core::integrator::Alpha* alpha = nullptr;
  ASSERT_NO_FATAL_FAILURE(
    CompareHHT("excavator/excavator.json", 0.3, 1e-3, context, alpha));
  EXPECT_GT(context.get_auxiliary_state_count(), 0u);
  EXPECT_LT(alpha->get_rejected_steps(), alpha->get_accepted_steps() / 5);
}

/* contact of the feet, a PID on the crank speed: every module gives its
tangents, from which the iteration matrices are formed */
TEST(AlphaIntegrator, ContactTangents) {
  shift::core::Context context;
  const shift::core::integrator::Alpha* alpha = nullptr;
  ASSERT_NO_FATAL_FAILURE(
    CompareHHT("strandbeest/strandbeest.json", 0.3, 1e-3, context, alpha));
  EXPECT_GT(alpha->get_tangent_matrix_count(),
            alpha->get_iteration_matrix_count() / 2);
}

/* the iteration matrix S = [M + A, B^T; B, 0] of the solver (RNCA) with an
A that is not symmetric: the solution of SolveIterationMatrix satisfies
S x = rhs, with M x + B^T la from Residual and B x from the Jacobian, also
after a Residual at another state. A is given as columns by differences:
first a few (the pattern of M and a few columns), then all of them (every
entry, which records the pattern again). */
TEST(AlphaIntegrator, NonSymmetricIterationMatrix) {
  const std::string name = "klann_linkage/klann_walker.json";
  const std::string path = std::string(SHIFT_EXAMPLES_DIR) + "/" + name;
  Json::Value ode113;
  ode113["type"] = "ode113";
  ode113["time_span"] = 0.2;
  ode113["time_step"] = 0.2;
  shift::core::Context context;
  context.LoadJSON(Example(name, ode113), path);
  const Trajectory run = Simulate(context);
  ASSERT_FALSE(run.y.empty());
  const double t = run.t.back();
  const std::vector<double> y = run.y.back();

  auto* solver = context.get_active_solvers()[0];
  ASSERT_TRUE(solver->supports_implicit());
  const unsigned int n = context.get_dq_count();
  const unsigned int nd = solver->get_multiplier_count();
  ASSERT_GT(nd, 0u);

  /* the rows of the velocities of S without A, M x + B^T la */
  std::vector<double> dy(y.size(), 0.0);
  std::vector<double> r0;
  ASSERT_FALSE(solver->Residual(t, y, std::vector<double>(n, 0.0),
                                std::vector<double>(nd, 0.0), r0, dy));
  auto product = [&](const std::vector<double>& x) {
    std::vector<double> ddq(x.begin(), x.begin() + n);
    std::vector<double> la(x.begin() + n, x.end());
    std::vector<double> r;
    EXPECT_FALSE(solver->Residual(t, y, ddq, la, r, dy));
    for(unsigned int l = 0; l < n + nd; l++) {
      r[l] -= r0[l];
    }
    /* the rows of the multipliers, B x: a dependent variable and the
    entries of its row of B (permutation order 3) */
    const auto& jacobian = context.get_jacobian();
    const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
    const auto& pv_13 = jacobian.get_pv13();
    const auto& rb3 = jacobian.get_rb3();
    const std::vector<int> cb3 = jacobian.get_cb3();
    const auto& b3 = jacobian.get_b3();
    unsigned int m = 0;
    for(unsigned int i = 0; i < n; i++) {
      if(is_dependent_po3[i]) {
        double value = x[pv_13[i]];
        for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
          value += b3[j] * x[pv_13[cb3[j]]];
        }
        r[n + m++] = value;
      }
    }
    return r;
  };

  /* the scale of A from the diagonal of M */
  double scale = 0.0;
  for(unsigned int l = 0; l < n; l++) {
    std::vector<double> e(n + nd, 0.0);
    e[l] = 1.0;
    scale = std::max(scale, std::fabs(product(e)[l]));
  }
  ASSERT_GT(scale, 0.0);

  std::mt19937 generator(7);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  for(const unsigned int column_count : {3u, n}) {
    SCOPED_TRACE(column_count);
    std::vector<int> differenced;
    for(unsigned int col = 0; col < column_count; col++) {
      differenced.push_back((col * 7) % n);
    }
    std::sort(differenced.begin(), differenced.end());
    differenced.erase(std::unique(differenced.begin(), differenced.end()),
                      differenced.end());
    const size_t count = differenced.size();
    std::vector<double> a(size_t(n) * count);
    for(double& value : a) {
      value = 0.05 * scale * uniform(generator);
    }

    std::vector<double> r;
    ASSERT_FALSE(solver->Residual(t, y, std::vector<double>(n, 0.0),
                                  std::vector<double>(nd, 0.0), r, dy));
    ASSERT_FALSE(solver->FactorIterationMatrix(shift::core::Tangents(), 0.0,
                                               1.0, differenced, a, {}));

    /* a Residual at another state before the solve */
    std::vector<double> y_other(y);
    for(unsigned int l = 0; l < n; l++) {
      y_other[context.get_q_count() + l] += 0.1;
    }
    ASSERT_FALSE(solver->Residual(t, y_other, std::vector<double>(n, 0.0),
                                  std::vector<double>(nd, 0.0), r, dy));

    std::vector<double> rhs(n + nd);
    for(double& value : rhs) {
      value = uniform(generator);
    }
    std::vector<double> x(rhs);
    ASSERT_FALSE(solver->SolveIterationMatrix(x));

    std::vector<double> s_x = product(x);
    for(unsigned int i = 0; i < n; i++) {
      for(size_t col = 0; col < count; col++) {
        s_x[i] += a[i * count + col] * x[differenced[col]];
      }
    }
    double largest = 0.0;
    for(double value : rhs) {
      largest = std::max(largest, std::fabs(value));
    }
    for(unsigned int l = 0; l < n + nd; l++) {
      EXPECT_NEAR(s_x[l], rhs[l], 1e-8 * largest) << "row " << l;
    }
  }
}

/* two pendulums from the fixed body: a double pendulum, whose tip a2 carries
an applied load (force_module::AppliedLoad, which gives no tangents) turning
with it, and a single one, b1, on a spring */
static const char* kMixedTangents = R"({"name": "a",
  "bodies": [
    {"name": "a1", "center_of_mass": [0.3, 0, 0.5],
     "frames": [{"name": "tip", "T": [0, 0, 1]}],
     "inertia": {"mass": 1, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}},
    {"name": "a2", "center_of_mass": [0.2, 0, 0.5],
     "inertia": {"mass": 1, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}},
    {"name": "b1", "center_of_mass": [0.3, 0, 0.5],
     "inertia": {"mass": 1, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}}],
  "joints": [
    {"name": "ja1", "type": "revolute", "axis": [0, 1, 0],
     "parent": [["fixed"], "origin"], "child": [["a1"], "origin"]},
    {"name": "ja2", "type": "revolute", "axis": [0, 1, 0],
     "parent": [["a1"], "tip"], "child": [["a2"], "origin"]},
    {"name": "jb1", "type": "revolute", "axis": [0, 1, 0],
     "parent": [["fixed"], "origin"], "child": [["b1"], "origin"],
     "stiffness_coefficients": [0.0, 200.0]}],
  "force_modules": [{"type": "applied_load", "name": "push", "body": "a2",
                     "force": [5, 0, 0]}]})";

static int JointDqIndex(shift::core::Context& context,
                        const std::string& name) {
  for(auto* joint : context.get_active_joints()) {
    if(joint->get_name() == name) {
      return joint->get_dq_index();
    }
  }
  ADD_FAILURE() << "no joint " << name;
  return -1;
}

/* the columns of a force module that gives no tangents are found by
differences, those of the joints that move the body it loads (ja1, ja2) and
no others (jb1), and the integrator forms its matrices from the tangents with
those columns; it follows ODE113 */
TEST(AlphaIntegrator, MixedIterationMatrix) {
  const std::string path = testing::TempDir() + "/alpha_mixed.json";
  auto load = [&](shift::core::Context& context,
                  const std::string& integrator) {
    {
      std::ofstream file(path);
      file << Model(integrator, kMixedTangents);
    }
    context.LoadJSON(path);
    std::remove(path.c_str());
  };

  {
    shift::core::Context context;
    load(context, R"({"type": "hht", "time_span": 0.1, "time_step": 0.01})");
    context.get_active_integrators()[0]->Initialize();
    shift::core::solver::Base* solver = context.get_active_solvers()[0];
    solver->Initialize();
    std::vector<double> y(context.get_state_count() +
                            context.get_auxiliary_state_count(),
                          0.0);
    std::copy(context.get_q().begin(), context.get_q().end(), y.begin());
    std::vector<double> dy(y.size(), 0.0);
    context.set_collecting_tangents(true);
    ASSERT_FALSE(solver->Solve(0.0, y, dy));
    context.set_collecting_tangents(false);
    shift::core::Tangents tangents;
    std::vector<int> differenced;
    EXPECT_FALSE(context.CollectTangents(y, tangents, &differenced));
    std::vector<int> expected = {JointDqIndex(context, "ja1"),
                                 JointDqIndex(context, "ja2")};
    std::sort(expected.begin(), expected.end());
    EXPECT_EQ(differenced, expected);
    /* the spring of jb1 is among the tangents */
    EXPECT_FALSE(tangents.get_joint_stiffness().empty());
  }

  const std::string errors =
    R"("time_span": 1.0, "time_step": 0.01, "absolute_error": 1e-10,
       "relative_error": 1e-8})";
  shift::core::Context implicit;
  load(implicit, R"({"type": "hht", )" + errors);
  const Trajectory run_implicit = Simulate(implicit);
  auto* alpha = dynamic_cast<shift::core::integrator::Alpha*>(
    implicit.get_active_integrators()[0]);
  ASSERT_NE(alpha, nullptr);
  EXPECT_GT(alpha->get_mixed_matrix_count(), 0u);
  EXPECT_EQ(alpha->get_mixed_matrix_count(),
            alpha->get_tangent_matrix_count());

  shift::core::Context reference;
  load(reference, R"({"type": "ode113", )" + errors);
  const Trajectory run_reference = Simulate(reference);
  ASSERT_FALSE(run_implicit.y.empty());
  ASSERT_FALSE(run_reference.y.empty());
  const unsigned int q_count = reference.get_q_count();
  double moved = 0.0;
  for(unsigned int i = 0; i < q_count; i++) {
    moved = std::max(moved, std::fabs(run_reference.y.back()[i]));
    EXPECT_NEAR(run_implicit.y.back()[i], run_reference.y.back()[i], 1e-4)
      << "q " << i;
  }
  EXPECT_GT(moved, 0.1);
}
