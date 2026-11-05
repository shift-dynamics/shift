/* free bodies (body::Free): bodies outside the kinematic tree, moved by the
Newton-Euler equations in the fixed frame */
#include "gtest/gtest.h"

#include <json/json.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ksl/mat3x3.h"
#include "ksl/vec3.h"

#include "shift/config.h"
#include "shift/core/body/free.h"
#include "shift/core/context.h"

struct FreeModel {
  double gravity = 9.81;
  std::string bodies;
  std::string joints;
  std::string force_modules;
  std::string collision_geometry;
  double time_span = 1.0;
};

/* load a model, and simulate it if asked; returns the state vector of the
bodies (Context::get_q() layout) at the end */
static std::vector<double> RunFree(shift::core::Context& context,
                                   const std::string& name,
                                   const FreeModel& model,
                                   bool simulate = true) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ostringstream json;
  json.precision(17);
  json << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, )"
       << model.gravity << R"(],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-11,
                     "absolute_error": 1e-12, "time_span": )"
       << model.time_span << R"(, "time_step": 0.01}]
  },
  "collision_geometry": [)"
       << model.collision_geometry << R"(],
  "assemblies": [{"name": "free",
    "bodies": [)"
       << model.bodies << R"(],
    "joints": [)"
       << model.joints << R"(],
    "force_modules": [)"
       << model.force_modules << R"(]
  }]
})";
  {
    std::ofstream file(path);
    file << json.str();
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  std::vector<double> state(context.get_q());
  if(simulate) {
    const size_t count = context.get_state_count();
    context.set_state_observer(
      [&state, count](double, const std::vector<double>& y) {
        state.assign(y.begin(), y.begin() + count);
      });
    context.Simulate();
  }
  return state;
}

static std::string Matrix(const ksl_mat3x3_t& R) {
  /* rows of R; ksl stores columns, R.v[column].at[row] */
  std::ostringstream s;
  s.precision(17);
  s << "[";
  for(int row = 0; row < 3; row++) {
    s << (row ? ", [" : "[");
    for(int column = 0; column < 3; column++) {
      s << (column ? ", " : "") << R.v[column].at[row];
    }
    s << "]";
  }
  s << "]";
  return s.str();
}

static std::string Vector(const double* v, int n) {
  std::ostringstream s;
  s.precision(17);
  s << "[";
  for(int i = 0; i < n; i++) {
    s << (i ? ", " : "") << v[i];
  }
  s << "]";
  return s.str();
}

/* an asymmetric body with its center of mass off its origin */
static const char* kInertia = R"("center_of_mass": [0.1, -0.05, 0.02],
  "inertia": {"mass": 2.0, "Ixx": 0.02, "Iyy": 0.05, "Izz": 0.08,
              "Ixy": 0.005, "Iyz": -0.003, "Izx": 0.002})";

static ksl_mat3x3_t Rotation() {
  ksl_vec3_t angles = ksl_vec3(0.5, -0.3, 0.8); // yaw, pitch, roll
  ksl_mat3x3_t R;
  ksl_mat3x3_setFromEulerAngles(&R, KSL_AXIS_ZYX, &angles);
  return R;
}

/* R^T v */
static ksl_vec3_t RotateBack(const ksl_mat3x3_t& R, const ksl_vec3_t& v) {
  return {{ksl_dot_vv(&R.v[0], &v), ksl_dot_vv(&R.v[1], &v),
           ksl_dot_vv(&R.v[2], &v)}};
}

/* the same body, on a float joint (RNCA) and free (body::Free), follows the
same motion under gravity from the same initial state */
TEST(FreeBody, MatchesFloatJoint) {
  const ksl_mat3x3_t R = Rotation();
  const double t0[3] = {0.3, -0.2, -1.0};
  const ksl_vec3_t v = ksl_vec3(1.0, 0.5, -3.0);   // origin, fixed frame
  const ksl_vec3_t omega = ksl_vec3(2.0, -1.0, 4.0); // fixed frame
  const ksl_vec3_t v_body = RotateBack(R, v);
  const ksl_vec3_t omega_body = RotateBack(R, omega);
  const double velocity[6] = {v.x, v.y, v.z, omega.x, omega.y, omega.z};
  const double velocity_body[6] = {v_body.x,     v_body.y,     v_body.z,
                                   omega_body.x, omega_body.y, omega_body.z};

  FreeModel model;
  model.bodies = std::string(R"({"name": "jointed", )") + kInertia +
                 R"(}, {"name": "free", "type": "free_body", )" + kInertia +
                 R"(, "initial_displacement": {"R": )" + Matrix(R) +
                 R"(, "T": )" + Vector(t0, 3) +
                 R"(}, "initial_velocity": )" + Vector(velocity, 6) + "}";
  model.joints = R"({"name": "float", "type": "float",
    "parent": [["fixed"], "origin"], "child": [["jointed"], "origin"],
    "initial_displacement": {"R": )" +
                 Matrix(R) + R"(, "T": )" + Vector(t0, 3) +
                 R"(}, "initial_velocity": )" + Vector(velocity_body, 6) +
                 "}";
  shift::core::Context context;
  const std::vector<double> state = RunFree(context, "match", model);
  ASSERT_EQ(context.get_free_bodies().size(), 1u);
  std::copy(state.begin(), state.end(), context.get_q().begin());
  context.UpdateDHVA();

  shift::core::body::Base* jointed = nullptr;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "jointed") {
      jointed = body;
    }
  }
  ASSERT_NE(jointed, nullptr);
  const ksl_SE3_t a = context.get_body_origin_pose(jointed);
  const ksl_SE3_t b = context.get_body_origin_pose(context.get_free_bodies()[0]);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(a.t.at[k], b.t.at[k], 1e-7) << "t[" << k << "]";
    for(int i = 0; i < 3; i++) {
      EXPECT_NEAR(a.R.v[i].at[k], b.R.v[i].at[k], 1e-7);
    }
  }
  // the motion is not trivial: the body fell and turned
  EXPECT_GT(std::fabs(b.t.z - t0[2]), 1.0);
  /* the velocities agree too: spatial, referred to the fixed origin */
  const ksl_screw_t va =
    context.GetBodyVelocity(context.GetBodyLoadIndex(jointed));
  const ksl_screw_t vb = context.GetBodyVelocity(
    context.GetBodyLoadIndex(context.get_free_bodies()[0]));
  for(int k = 0; k < 6; k++) {
    EXPECT_NEAR(va.at[k], vb.at[k], 1e-6) << "V[" << k << "]";
  }
}

/* the center of mass, the momentum about it and the kinetic energy, from a
free body state */
struct Motion {
  ksl_vec3_t c, v_c, L_c;
  double energy;
};

/* the displacement and velocity of the first free body for a state vector
of the bodies */
static void FreeDV(shift::core::Context& context,
                   const std::vector<double>& state, ksl_SE3_t& d,
                   ksl_screw_t& V) {
  std::copy(state.begin(), state.end(), context.get_q().begin());
  context.UpdateDHVA();
  d = context.get_dhva().free.d[0];
  V = context.get_dhva().free.v[0];
}

static Motion Measure(shift::core::Context& context,
                      const std::vector<double>& state) {
  const double m = 2.0;
  const ksl_vec3_t com = ksl_vec3(0.1, -0.05, 0.02);
  ksl_SE3_t d;
  ksl_screw_t V;
  FreeDV(context, state, d, V);
  Motion motion;
  ksl_product_dv(&d, &com, &motion.c);
  ksl_vec3_t w_c;
  ksl_cross_vv(&V.ang, &motion.c, &w_c);
  ksl_add_vv(&V.lin, &w_c, &motion.v_c);
  /* I_c omega = R I_b R^T omega */
  const ksl_vec3_t w = RotateBack(d.R, V.ang);
  const ksl_vec3_t Iw = ksl_vec3(0.02 * w.x + 0.005 * w.y + 0.002 * w.z,
                                 0.005 * w.x + 0.05 * w.y - 0.003 * w.z,
                                 0.002 * w.x - 0.003 * w.y + 0.08 * w.z);
  ksl_product_drv(&d.R, &Iw, &motion.L_c);
  motion.energy = 0.5 * m * ksl_dot_vv(&motion.v_c, &motion.v_c) +
                  0.5 * ksl_dot_vv(&w, &Iw);
  return motion;
}

static std::string FreeBody(const double t0[3], const double velocity[6]) {
  return std::string(R"({"name": "free", "type": "free_body", )") + kInertia +
         R"(, "initial_displacement": {"R": )" + Matrix(Rotation()) +
         R"(, "T": )" + Vector(t0, 3) + R"(}, "initial_velocity": )" +
         Vector(velocity, 6) + "}";
}

/* without gravity or loads, an asymmetric body tumbling about no principal
axis keeps its momentum, angular momentum and kinetic energy */
TEST(FreeBody, TorqueFreeConservation) {
  const double t0[3] = {2.0, -1.0, 0.5};
  const double velocity[6] = {0.3, -0.2, 0.1, 3.0, -5.0, 2.0};
  FreeModel model;
  model.gravity = 0.0;
  model.time_span = 2.0;
  model.bodies = FreeBody(t0, velocity);
  shift::core::Context context;
  std::vector<double> initial;
  const std::vector<double> state = RunFree(context, "torque_free", model);
  shift::core::Context context0;
  initial = RunFree(context0, "torque_free0", model, false);
  const Motion before = Measure(context, initial);
  const Motion after = Measure(context, state);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(after.v_c.at[k], before.v_c.at[k], 1e-8);
    EXPECT_NEAR(after.L_c.at[k], before.L_c.at[k], 1e-7);
    EXPECT_NEAR(after.c.at[k], before.c.at[k] + 2.0 * before.v_c.at[k], 1e-7);
  }
  EXPECT_NEAR(after.energy, before.energy, 1e-7 * before.energy);
  /* the angular velocity changed: the body tumbles */
  ksl_SE3_t d;
  ksl_screw_t V0, V1;
  FreeDV(context, initial, d, V0);
  FreeDV(context, state, d, V1);
  ksl_vec3_t change;
  ksl_subtract_vv(&V1.ang, &V0.ang, &change);
  EXPECT_GT(ksl_vec3_l2norm(&change), 0.1);
}

/* under gravity, the center of mass of a spinning body follows a parabola */
TEST(FreeBody, Projectile) {
  const double t0[3] = {0.0, 0.0, -2.0};
  const double velocity[6] = {1.0, 2.0, -4.0, 6.0, 1.0, -3.0};
  FreeModel model;
  model.bodies = FreeBody(t0, velocity);
  shift::core::Context context0, context;
  const std::vector<double> initial =
    RunFree(context0, "projectile0", model, false);
  const std::vector<double> state = RunFree(context, "projectile", model);
  const Motion before = Measure(context, initial);
  const Motion after = Measure(context, state);
  const double g[3] = {0.0, 0.0, 9.81};
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(after.c.at[k], before.c.at[k] + before.v_c.at[k] + 0.5 * g[k],
                1e-7);
    EXPECT_NEAR(after.v_c.at[k], before.v_c.at[k] + g[k], 1e-7);
    EXPECT_NEAR(after.L_c.at[k], before.L_c.at[k], 1e-7);
  }
}

TEST(FreeBody, NoJoints) {
  const double t0[3] = {0.0, 0.0, 0.0};
  const double velocity[6] = {0, 0, 0, 0, 0, 0};
  FreeModel model;
  model.bodies = FreeBody(t0, velocity);
  model.joints = R"({"name": "bad", "type": "revolute",
    "parent": [["fixed"], "origin"], "child": [["free"], "origin"]})";
  shift::core::Context context;
  EXPECT_ANY_THROW(RunFree(context, "no_joints", model, false));
}

/* a free box dropped on the floor comes to rest on its four bottom corners,
pushed by the contact forces (AddBodyLoad of a free body) */
TEST(FreeBody, BoxRestsOnFloor) {
  FreeModel model;
  model.time_span = 2.0;
  model.collision_geometry = R"({"name": "floor", "type": "plane",
                                 "material": "floor"})";
  model.bodies = R"({"name": "box", "type": "free_body",
    "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.01, "Iyy": 0.01, "Izz": 0.01,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
    "initial_displacement": {"T": [0, 0, -0.2]},
    "collision_geometry": [{"name": "box", "type": "convex_hull",
      "half_extents": [0.1, 0.1, 0.1], "material": "box"}]})";
  model.force_modules = R"({"name": "contact", "type": "contact",
    "materials": [
      {"name": "floor", "restitution": 0.3, "friction": 0.5,
       "contact_stiffness": 1e5},
      {"name": "box", "restitution": 0.3, "friction": 0.5,
       "contact_stiffness": 1e5}]})";
  shift::core::Context context;
  const std::vector<double> state = RunFree(context, "box_floor", model);
  ksl_SE3_t d;
  ksl_screw_t V;
  FreeDV(context, state, d, V);
  /* four springs in series, 1/k = 1/1e5 + 1/1e5, carry m g */
  const double depth = 9.81 / (4 * 5e4);
  EXPECT_NEAR(d.t.z, -0.1 + depth, 1e-5);
  EXPECT_LT(std::max(ksl_vec3_l2norm(&V.lin), ksl_vec3_l2norm(&V.ang)), 1e-3);
}

/* the states of an example at each output step, integrated by RK4 with a
fixed step so that two models are evaluated at the same times */
static void Record(shift::core::Context& context, const std::string& example,
                   std::vector<double>& times,
                   std::vector<std::vector<double>>& states) {
  const std::string directory = std::string(SHIFT_EXAMPLES_DIR) + "/collision";
  Json::Value root;
  {
    std::ifstream file(directory + "/" + example + ".json");
    file >> root;
  }
  Json::Value& parameters = root["model_parameters"];
  parameters["include_directories"] = Json::Value(Json::arrayValue);
  parameters["include_directories"].append(directory);
  Json::Value rk4;
  rk4["type"] = "rk4";
  rk4["time_span"] = 2.0;
  rk4["time_step"] = 0.01;
  rk4["step_size"] = 1e-4;
  parameters["integrators"] = Json::Value(Json::arrayValue);
  parameters["integrators"].append(rk4);
  const std::string path = testing::TempDir() + "/" + example + "_rk4.json";
  {
    std::ofstream file(path);
    file << root;
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  const size_t count = context.get_state_count();
  context.set_state_observer(
    [&times, &states, count](double t, const std::vector<double>& y) {
      if(!times.empty() && t <= times.back()) {
        return;
      }
      times.push_back(t);
      states.emplace_back(y.begin(), y.begin() + count);
    });
  context.Simulate();
}

/* the bunnies of the 5-bunny collision example, on float joints solved by
RNCA and as free bodies, fall onto the floor and against each other along
the same paths */
TEST(FreeBody, CollisionExampleMatchesFloatJoints) {
  shift::core::Context jointed, free;
  std::vector<double> times_jointed, times_free;
  std::vector<std::vector<double>> states_jointed, states_free;
  Record(jointed, "collision_test_5", times_jointed, states_jointed);
  Record(free, "collision_test_5_free", times_free, states_free);
  ASSERT_EQ(free.get_free_bodies().size(), 5u);
  ASSERT_EQ(times_jointed.size(), times_free.size());

  const double kCompareUntil = 0.4;
  double max_t = 0.0, max_R = 0.0;
  for(size_t s = 0; s < times_free.size(); s++) {
    EXPECT_NEAR(times_jointed[s], times_free[s], 1e-12);
    std::copy(states_jointed[s].begin(), states_jointed[s].end(),
              jointed.get_q().begin());
    jointed.UpdateDHVA();
    std::copy(states_free[s].begin(), states_free[s].end(),
              free.get_q().begin());
    free.UpdateDHVA();
    double step_t = 0.0, step_R = 0.0;
    for(auto body : free.get_free_bodies()) {
      shift::core::body::Base* twin = nullptr;
      for(auto b : jointed.get_active_articulated_bodies()) {
        if(b->get_name() == body->get_name()) {
          twin = b;
        }
      }
      ASSERT_NE(twin, nullptr) << body->get_name();
      const ksl_SE3_t a = jointed.get_body_origin_pose(twin);
      const ksl_SE3_t b = free.get_body_origin_pose(body);
      for(int k = 0; k < 3; k++) {
        step_t = std::max(step_t, std::fabs(a.t.at[k] - b.t.at[k]));
        for(int i = 0; i < 3; i++) {
          step_R = std::max(step_R, std::fabs(a.R.v[i].at[k] - b.R.v[i].at[k]));
        }
      }
    }
    if(times_free[s] <= kCompareUntil) {
      max_t = std::max(max_t, step_t);
      max_R = std::max(max_R, step_R);
    }
  }
  /* the two formulations integrate different coordinates (velocities of
  the joints, in the body frame, and spatial velocities), so they differ by
  round-off, 1e-15 in free fall; once the bunnies pile up on the floor the
  contacts amplify it by about 1000 every 0.1 s, so the paths are compared
  through the first contacts only, about 3e-8 at 0.4 s */
  EXPECT_LT(max_t, 1e-6);
  EXPECT_LT(max_R, 1e-6);
  /* and the bunnies fell onto the floor */
  for(auto body : free.get_free_bodies()) {
    EXPECT_GT(free.get_body_origin_pose(body).t.z, -0.5) << body->get_name();
  }
}
