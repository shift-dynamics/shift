/* The tangent stiffness and damping that joints, controllers and force
modules give the implicit integrators (Context::CollectTangents) against
central differences of the applied generalized forces they stand for,
Q = f + sum_b J_b^T g_b: the joint working forces f and the body loads g at
a perturbed state, with the body Jacobians J of the unperturbed state (the
tangents are those of the loads, mapped by the Jacobians; the derivatives of
the Jacobians, of inertia and of gravity are not part of them). The state is
the end of a short run with the velocities set to zero: in motion, the twist
of a body at a perturbed configuration also changes with the derivatives of
the Jacobians, which the tangents leave out (see Tangents). */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <json/json.h>
#include <lapacke.h>

#include "shift/config.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/joint/base.h"
#include "shift/core/tangents.h"

/* the example model given, or the model of the text given, simulated for
time_span, without loggers */
static void Run(const std::string& name, const char* text, double time_span,
                shift::core::Context& context, double& t,
                std::vector<double>& y) {
  const std::string path = std::string(SHIFT_EXAMPLES_DIR) + "/" + name;
  Json::Value model;
  Json::CharReaderBuilder builder;
  std::string errors;
  if(text) {
    std::istringstream stream(text);
    ASSERT_TRUE(Json::parseFromStream(builder, stream, &model, &errors))
      << errors;
  } else {
    std::ifstream file(path);
    ASSERT_TRUE(Json::parseFromStream(builder, file, &model, &errors))
      << errors;
  }
  Json::Value& parameters = model["model_parameters"];
  parameters["integrators"][0]["time_span"] = time_span;
  parameters["integrators"][0]["time_step"] = time_span;
  parameters["loggers"] = Json::Value(Json::arrayValue);
  context.LoadJSON(model, path);
  context.set_state_observer([&](double time, const std::vector<double>& s) {
    t = time;
    y = s;
  });
  context.Simulate();
  ASSERT_FALSE(y.empty());
}

/* the Jacobians of the bodies that take loads at y, from the twists at unit
velocities, 6 rows (lin first) by one column per generalized velocity */
static void BodyJacobians(shift::core::Context& context,
                          const std::vector<double>& y, std::set<int>& load_set,
                          std::vector<std::vector<double>>& jacobians) {
  const unsigned int dq_count = context.get_dq_count();
  load_set.clear();
  for(auto* body : context.get_active_articulated_bodies()) {
    const int load_index = context.GetBodyLoadIndex(body);
    if(load_index > -1 &&
       load_index < static_cast<int>(context.get_non_rigid_joint_count())) {
      load_set.insert(load_index);
    }
  }
  jacobians.assign(context.get_non_rigid_joint_count(), {});
  for(int b : load_set) {
    jacobians[b].assign(6 * size_t(dq_count), 0.0);
  }
  for(unsigned int l = 0; l < dq_count; l++) {
    std::vector<double> unit(dq_count, 0.0);
    unit[l] = 1.0;
    context.UpdateDHVA(y.data(), unit.data());
    for(int b : load_set) {
      const ksl_screw_t twist = context.GetBodyVelocity(b);
      for(int m = 0; m < 6; m++) {
        jacobians[b][size_t(m) * dq_count + l] = twist.at[m];
      }
    }
  }
}

/* the applied generalized forces Q at a state, then the rates of the
internal states */
static std::vector<double>
AppliedForces(shift::core::Context& context, double t,
              const std::vector<double>& state, const std::vector<int>& loads,
              const std::vector<std::vector<double>>& jacobians) {
  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();
  std::vector<double> dy(state.size(), 0.0);
  context.UpdateDHVA(state.data(), state.data() + q_count);
  context.SetForces(t, state, dy);
  std::vector<double> q(context.get_joint_working_forces().begin(),
                        context.get_joint_working_forces().begin() + dq_count);
  const auto& body_loads = context.get_body_loads();
  for(int b : loads) {
    for(unsigned int l = 0; l < dq_count; l++) {
      for(int m = 0; m < 6; m++) {
        q[l] += jacobians[b][size_t(m) * dq_count + l] * body_loads[b].at[m];
      }
    }
  }
  q.insert(q.end(), dy.begin() + q_count + dq_count, dy.end());
  return q;
}

/* the stiffness at rest (velocities zero), or the damping at the state of
the run: C involves no derivative of the Jacobians */
static void CheckTangents(const std::string& name, double time_span,
                          double tolerance, bool damping,
                          const char* text = nullptr) {
  SCOPED_TRACE(name);
  shift::core::Context context;
  double t = 0.0;
  std::vector<double> y;
  ASSERT_NO_FATAL_FAILURE(Run(name, text, time_span, context, t, y));
  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();
  if(!damping) {
    std::fill(y.begin() + q_count, y.begin() + q_count + dq_count, 0.0);
  }

  std::set<int> load_set;
  std::vector<std::vector<double>> jacobians;
  BodyJacobians(context, y, load_set, jacobians);
  const std::vector<int> loads(load_set.begin(), load_set.end());
  const size_t state_offset = q_count + dq_count;
  const size_t state_count = y.size() - state_offset;
  auto forces = [&](const std::vector<double>& state) {
    return AppliedForces(context, t, state, loads, jacobians);
  };

  /* the tangents at y, mapped to the generalized velocities */
  context.set_collecting_tangents(true);
  const std::vector<double> q0 = forces(y);
  context.set_collecting_tangents(false);
  shift::core::Tangents tangents;
  ASSERT_TRUE(context.CollectTangents(y, tangents))
    << "a controller or force module gives no tangents";
  std::vector<double> k(size_t(dq_count) * dq_count, 0.0);
  std::vector<double> c(size_t(dq_count) * dq_count, 0.0);
  for(const auto& e : tangents.get_joint_stiffness()) {
    k[size_t(e.dq_a) * dq_count + e.dq_b] += e.value;
  }
  for(const auto& e : tangents.get_joint_damping()) {
    c[size_t(e.dq_a) * dq_count + e.dq_b] += e.value;
  }
  auto map_blocks = [&](const std::vector<shift::core::Tangents::BodyBlock>&
                          blocks,
                        std::vector<double>& matrix) {
    for(const auto& block : blocks) {
      if(!load_set.count(block.load_index_a) ||
         !load_set.count(block.load_index_b)) {
        continue;
      }
      const auto& ja = jacobians[block.load_index_a];
      const auto& jb = jacobians[block.load_index_b];
      for(unsigned int l = 0; l < dq_count; l++) {
        for(unsigned int col = 0; col < dq_count; col++) {
          double value = 0.0;
          for(int r = 0; r < 6; r++) {
            for(int s = 0; s < 6; s++) {
              value += ja[size_t(r) * dq_count + l] * block.block[r * 6 + s] *
                       jb[size_t(s) * dq_count + col];
            }
          }
          matrix[size_t(l) * dq_count + col] += value;
        }
      }
    }
  };
  map_blocks(tangents.get_body_stiffness(), k);
  map_blocks(tangents.get_body_damping(), c);

  /* the derivatives of the rates of the internal states, below K and C */
  k.resize(size_t(dq_count + state_count) * dq_count, 0.0);
  c.resize(size_t(dq_count + state_count) * dq_count, 0.0);
  auto add_state = [&](int state, int l, double value,
                       std::vector<double>& matrix) {
    ASSERT_GE(state, static_cast<int>(state_offset));
    ASSERT_LT(state, static_cast<int>(y.size()));
    matrix[size_t(dq_count + state - state_offset) * dq_count + l] += value;
  };
  for(const auto& e : tangents.get_state_stiffness()) {
    add_state(e.state, e.dq, e.value, k);
  }
  for(const auto& e : tangents.get_state_damping()) {
    add_state(e.state, e.dq, e.value, c);
  }
  auto map_rows = [&](const std::vector<shift::core::Tangents::StateBodyRow>&
                        rows,
                      std::vector<double>& matrix) {
    for(const auto& row : rows) {
      if(!load_set.count(row.load_index)) {
        continue;
      }
      const auto& j = jacobians[row.load_index];
      for(unsigned int l = 0; l < dq_count; l++) {
        double value = 0.0;
        for(int m = 0; m < 6; m++) {
          value += row.row[m] * j[size_t(m) * dq_count + l];
        }
        add_state(row.state, l, value, matrix);
      }
    }
  };
  map_rows(tangents.get_state_body_stiffness(), k);
  map_rows(tangents.get_state_body_damping(), c);

  /* central differences of Q */
  const size_t row_count = dq_count + state_count;
  std::vector<double> k_difference(row_count * dq_count, 0.0);
  std::vector<double> c_difference(row_count * dq_count, 0.0);
  const double step = 1e-7;
  for(unsigned int l = 0; l < dq_count; l++) {
    for(int velocity = 0; velocity < 2; velocity++) {
      std::vector<double> plus(y), minus(y);
      if(velocity) {
        plus[q_count + l] += step;
        minus[q_count + l] -= step;
      } else {
        std::vector<double> delta(dq_count, 0.0);
        delta[l] = step;
        context.IncrementPositions(y.data(), delta.data(), plus.data());
        delta[l] = -step;
        context.IncrementPositions(y.data(), delta.data(), minus.data());
      }
      const std::vector<double> q_plus = forces(plus);
      const std::vector<double> q_minus = forces(minus);
      std::vector<double>& matrix = velocity ? c_difference : k_difference;
      for(size_t i = 0; i < row_count; i++) {
        /* -dQ for the forces, the derivative of the rate for a state */
        matrix[i * dq_count + l] = (i < dq_count ? -1.0 : 1.0) *
                                   (q_plus[i] - q_minus[i]) / (2.0 * step);
      }
    }
  }

  const int kind = damping ? 1 : 0;
  const auto& tangent = kind ? c : k;
  const auto& difference = kind ? c_difference : k_difference;
  /* the joint and the index in it of a generalized velocity, for the
  messages */
  auto velocity_name = [&](unsigned int l) {
    for(auto* joint : context.get_active_joints()) {
      const int first = joint->get_dq_index();
      const int end = first + static_cast<int>(joint->get_dq_count());
      if(static_cast<int>(l) >= first && static_cast<int>(l) < end) {
        return joint->get_name() + "[" +
               std::to_string(l - joint->get_dq_index()) + "]";
      }
    }
    return std::to_string(l);
  };

  /* each row (a generalized force, or the rate of a state) against its
  largest entry, above roundoff of the largest of its kind */
  double scale[2] = {0.0, 0.0};
  for(size_t e = 0; e < difference.size(); e++) {
    double& part = scale[e < size_t(dq_count) * dq_count ? 0 : 1];
    part = std::max(part, std::fabs(difference[e]));
  }
  double largest_any = 0.0;
  for(size_t i = 0; i < row_count; i++) {
    const std::string what =
      std::string(kind ? "damping" : "stiffness") +
      (i < dq_count ? " of the force of " + velocity_name(i)
                    : " of the rate of state " + std::to_string(i - dq_count));
    double largest = 0.0, error = 0.0;
    unsigned int worst = 0;
    for(unsigned int l = 0; l < dq_count; l++) {
      const size_t e = i * dq_count + l;
      largest = std::max(largest, std::fabs(difference[e]));
      if(std::fabs(tangent[e] - difference[e]) > error) {
        error = std::fabs(tangent[e] - difference[e]);
        worst = l;
      }
    }
    largest_any = std::max(largest_any, largest);
    EXPECT_LE(error, tolerance * std::max(largest,
                                          1e-6 * scale[i < dq_count ? 0 : 1]))
      << what << ": largest " << largest << ", at " << velocity_name(worst)
      << " the tangent " << tangent[i * dq_count + worst]
      << " and the difference " << difference[i * dq_count + worst];
  }
  EXPECT_GT(largest_any, 0.0);
}

/* joint springs and dampers along the suspension, the tires on the ground */
TEST(Tangents, VehicleTires) {
  CheckTangents("vehicle/vehicle.json", 0.2, 1e-3, false);
  CheckTangents("vehicle/vehicle.json", 0.2, 1e-3, true);
}

TEST(Tangents, Bushing) {
  CheckTangents("force_modules/bushing.json", 0.1, 1e-3, false);
  CheckTangents("force_modules/bushing.json", 0.1, 1e-3, true);
}

/* contact of the feet with the ground, a PID on the crank speed and
springs of the gears. The stiffness leaves out how the point of contact moves
over the curved feet as they turn, a few percent of the rows of the chassis */
TEST(Tangents, ContactKlannWalker) {
  CheckTangents("klann_linkage/klann_walker.json", 0.5, 5e-2, false);
  CheckTangents("klann_linkage/klann_walker.json", 0.5, 1e-2, true);
}

/* hydraulic cylinders on the boom, arm and bucket: the end stops and seal
friction on the joints, and the pressure rates with the extension and its
rate (the stiffness of the oil) */
TEST(Tangents, HydraulicCylinders) {
  CheckTangents("excavator/excavator.json", 0.2, 1e-3, false);
  CheckTangents("excavator/excavator.json", 0.2, 1e-3, true);
}

/* a powertrain with a torque converter, differentials and brakes, and tires
with relaxation states */
TEST(Tangents, Powertrain) {
  CheckTangents("raptor/raptor.json", 0.2, 1e-3, false);
  CheckTangents("raptor/raptor.json", 0.2, 1e-3, true);
}

/* a powertrain with a clutch, spring-dampers as stops and anti-roll bars
(about an axis, with an angle state), contact of the body with the terrain
and a PID with an integral on the steering rack */
TEST(Tangents, PoloSpringDampers) {
  CheckTangents("polo_r_wrc/polo_r_wrc.json", 0.2, 1e-3, false);
  CheckTangents("polo_r_wrc/polo_r_wrc.json", 0.2, 1e-3, true);
}

/* two bodies on float joints, moving, joined by spring-dampers with every
feature in action: a line with a preload and hysteresis, a stop in tension
past its gap, a line that crushes in compression and a torsion spring about
a skew axis away from its free angle; and a third body, c, resting on the
ground at four points of terrain contact and sliding */
static const char* kSpringDampers = R"({
  "model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.05, "time_step": 0.05,
                     "absolute_error": 1e-8, "relative_error": 1e-8}]},
  "frames": [{"name": "start_a", "T": [0, 0, 0]},
             {"name": "start_b", "T": [0.4, 0.3, -0.6]},
             {"name": "start_c", "T": [1.0, 1.0, 0.0]}],
  "assemblies": [{"name": "pair",
    "bodies": [
      {"name": "a", "center_of_mass": [0, 0, -0.05],
       "inertia": {"mass": 10, "Ixx": 1.0, "Iyy": 1.2, "Izz": 0.8,
                   "Ixy": 0, "Iyz": 0, "Izx": 0},
       "frames": [{"name": "pa", "T": [0.1, 0.05, -0.1]},
                  {"name": "qa", "T": [-0.1, 0.1, -0.05]}]},
      {"name": "b", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 5, "Ixx": 0.4, "Iyy": 0.5, "Izz": 0.3,
                   "Ixy": 0, "Iyz": 0, "Izx": 0},
       "frames": [{"name": "pb", "T": [-0.05, 0.1, 0.05]},
                  {"name": "qb", "T": [0.08, -0.06, 0.02]}]},
      {"name": "c", "center_of_mass": [0, 0, -0.05],
       "inertia": {"mass": 10, "Ixx": 1.0, "Iyy": 1.2, "Izz": 0.8,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "float_a", "type": "float",
       "parent": [["fixed"], "start_a"], "child": [["a"], "origin"],
       "initial_velocity": [0.5, 0.3, 0.0, 0.2, -0.4, 0.6]},
      {"name": "float_b", "type": "float",
       "parent": [["fixed"], "start_b"], "child": [["b"], "origin"],
       "initial_velocity": [-0.3, 0.2, 0.1, 0.5, 0.3, -0.2]},
      {"name": "float_c", "type": "float",
       "parent": [["fixed"], "start_c"], "child": [["c"], "origin"],
       "initial_velocity": [0.5, 0.2, 0.0, 0.0, 0.0, 0.3]}],
    "force_modules": [
      {"type": "spring_damper", "name": "line",
       "frames": [["a", "pa"], ["b", "pb"]], "free_length": 0.3,
       "stiffness": 2000, "damping": 30,
       "hysteresis": {"yield_force": 50, "yield_displacement": 0.01}},
      {"type": "spring_damper", "name": "stop",
       "frames": [["a", "qa"], ["b", "qb"]], "free_length": 0.5,
       "acts": "tension", "gap": 0.05, "stiffness": 10000, "damping": 100,
       "smoothing": 0.01},
      {"type": "spring_damper", "name": "crush",
       "frames": [["a", "origin"], ["b", "origin"]], "free_length": 1.5,
       "stiffness": 500, "unloading_stiffness": 2000, "crush_time": 0.01},
      {"type": "spring_damper", "name": "torsion", "axis": [0.3, 1.0, 0.2],
       "frames": [["a", "origin"], ["b", "origin"]], "free_angle": 0.4,
       "stiffness": 50, "damping": 2},
      {"type": "terrain_contact", "name": "feet", "body": "c",
       "points": [0.2, 0.2, 0.0025, -0.2, 0.2, 0.0025, 0.2, -0.2, 0.0025,
                  -0.2, -0.2, 0.0025],
       "stiffness": 1e4, "damping": 0.5, "friction": 0.6,
       "slip_speed": 0.05}]}]})";

TEST(Tangents, SpringDampersAndTerrainContact) {
  CheckTangents("spring_dampers", 0.05, 1e-3, false, kSpringDampers);
  CheckTangents("spring_dampers", 0.05, 1e-3, true, kSpringDampers);
}

/* the derivatives with respect to the internal states that the modules
give (force_module::Base::AddStateTangents), of the applied generalized
forces, J_b^T dg_b / dz, and of the rates, against central differences, in
the columns of the given states */
static void CheckStateTangents(const std::string& name, double time_span,
                               double tolerance) {
  SCOPED_TRACE(name);
  shift::core::Context context;
  double t = 0.0;
  std::vector<double> y;
  ASSERT_NO_FATAL_FAILURE(Run(name, nullptr, time_span, context, t, y));
  const unsigned int dq_count = context.get_dq_count();
  const size_t state_offset = context.get_q_count() + dq_count;
  const size_t state_count = y.size() - state_offset;
  std::set<int> load_set;
  std::vector<std::vector<double>> jacobians;
  BodyJacobians(context, y, load_set, jacobians);
  const std::vector<int> loads(load_set.begin(), load_set.end());

  context.set_collecting_tangents(true);
  AppliedForces(context, t, y, loads, jacobians);
  context.set_collecting_tangents(false);
  shift::core::Tangents tangents;
  context.CollectTangents(y, tangents);
  std::vector<int> given;
  for(const auto& range : tangents.get_given_states()) {
    for(int state = range.first; state < range.first + range.count; state++) {
      given.push_back(state);
    }
  }
  ASSERT_FALSE(given.empty()) << "no module gives its state derivatives";

  /* the rows of Q and of the rates, one column per state of y */
  const size_t row_count = dq_count + state_count;
  std::vector<double> tangent(row_count * state_count, 0.0);
  auto column = [&](int state) {
    return static_cast<size_t>(state) - state_offset;
  };
  for(const auto& e : tangents.get_state_loads()) {
    if(!load_set.count(e.load_index)) {
      continue;
    }
    const auto& j = jacobians[e.load_index];
    for(unsigned int l = 0; l < dq_count; l++) {
      double value = 0.0;
      for(int m = 0; m < 6; m++) {
        value += j[size_t(m) * dq_count + l] * e.load[m];
      }
      tangent[l * state_count + column(e.state)] += value;
    }
  }
  for(const auto& e : tangents.get_state_forces()) {
    tangent[size_t(e.dq) * state_count + column(e.state)] += e.value;
  }
  for(const auto& e : tangents.get_state_rates()) {
    tangent[(dq_count + column(e.state)) * state_count + column(e.other)] +=
      e.value;
  }

  std::vector<double> difference(row_count * state_count, 0.0);
  for(const int state : given) {
    const double step = 1e-7 * std::max(1e-3, std::fabs(y[state]));
    std::vector<double> plus(y), minus(y);
    plus[state] += step;
    minus[state] -= step;
    const std::vector<double> q_plus =
      AppliedForces(context, t, plus, loads, jacobians);
    const std::vector<double> q_minus =
      AppliedForces(context, t, minus, loads, jacobians);
    for(size_t i = 0; i < row_count; i++) {
      difference[i * state_count + column(state)] =
        (q_plus[i] - q_minus[i]) / (2.0 * step);
    }
  }

  /* each column against its largest entry among the forces and among the
  rates */
  double largest_any = 0.0;
  for(const int state : given) {
    const size_t col = column(state);
    for(int part = 0; part < 2; part++) {
      const size_t begin = part ? dq_count : 0;
      const size_t end = part ? row_count : dq_count;
      double largest = 0.0, error = 0.0;
      size_t worst = begin;
      for(size_t i = begin; i < end; i++) {
        const double d = difference[i * state_count + col];
        largest = std::max(largest, std::fabs(d));
        const double e = std::fabs(tangent[i * state_count + col] - d);
        if(e > error) {
          error = e;
          worst = i;
        }
      }
      largest_any = std::max(largest_any, largest);
      EXPECT_LE(error, tolerance * largest + 1e-9)
        << (part ? "rates" : "forces") << " with state " << col
        << ": largest " << largest << ", at row " << worst << " the tangent "
        << tangent[worst * state_count + col] << " and the difference "
        << difference[worst * state_count + col];
    }
  }
  EXPECT_GT(largest_any, 0.0);
}

/* the plastic sinkage and the shear displacement of the shoes of a track on
the ground plane (pressure under loading and unloading, shear strength and
earth pressure) */
TEST(Tangents, TrackSoilStates) {
  CheckStateTangents("terramechanics/track_frame_soil.json", 0.6, 1e-4);
}

/* the model of an example without its force modules of a type, as text */
static std::string WithoutForceModules(const std::string& name,
                                       const std::string& type) {
  std::ifstream file(std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  Json::Value model;
  Json::CharReaderBuilder builder;
  std::string errors;
  EXPECT_TRUE(Json::parseFromStream(builder, file, &model, &errors)) << errors;
  for(Json::Value& assembly : model["assemblies"]) {
    Json::Value kept(Json::arrayValue);
    for(const Json::Value& module : assembly["force_modules"]) {
      if(module["type"].asString() != type) {
        kept.append(module);
      }
    }
    assembly["force_modules"] = kept;
  }
  return Json::writeString(Json::StreamWriterBuilder(), model);
}

/* the shoes of a track on the ground plane (track_soil): the pressure, the
shear strength and the earth pressure with the sinkage and the turning of
the shoes, and the rates of the plastic sinkage and of the shear
displacement. Without the contact of the rollers and the sprocket with the
links, whose stiffness leaves out how the points of contact move (see
ContactKlannWalker) */
TEST(Tangents, TrackSoil) {
  const std::string name = "terramechanics/track_frame_soil.json";
  const std::string text = WithoutForceModules(name, "contact");
  CheckTangents(name, 0.6, 1e-3, false, text.c_str());
  CheckTangents(name, 0.6, 1e-3, true, text.c_str());
}

/* the tangents of the loads on one body and of the rates of the states
against central differences in which the body's twist is held: the pose is
perturbed through the generalized coordinates, and the generalized velocities
are solved again (least squares) for the twist of the run. The body blocks are
the derivatives at a fixed twist (see Tangents), so the rates of the
generalized velocities' influence coefficients, left out of them, do not
enter. With velocities, unlike CheckTangents, so the terms of the twist are
checked too. adjust changes the time and state of the run before the check. */
static void CheckAtFixedTwist(
  const std::string& name, const char* text, double time_span,
  const std::string& body_name, double tolerance,
  const std::function<void(shift::core::Context&, double&,
                           std::vector<double>&)>& adjust) {
  SCOPED_TRACE(name);
  shift::core::Context context;
  double t = 0.0;
  std::vector<double> y;
  ASSERT_NO_FATAL_FAILURE(Run(name, text, time_span, context, t, y));
  adjust(context, t, y);
  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();
  int body = -1;
  for(auto* b : context.get_active_articulated_bodies()) {
    if(b->get_name() == body_name) {
      body = context.GetBodyLoadIndex(b);
    }
  }
  ASSERT_GT(body, -1);
  /* the body's Jacobian at a state, 6 by dq_count */
  auto jacobian_at = [&](const std::vector<double>& state) {
    std::set<int> load_set;
    std::vector<std::vector<double>> jacobians;
    BodyJacobians(context, state, load_set, jacobians);
    return jacobians[body];
  };
  std::vector<std::vector<double>> jacobians(
    context.get_non_rigid_joint_count());
  jacobians[body] = jacobian_at(y);
  const std::vector<double>& jacobian = jacobians[body];
  std::vector<double> twist(6, 0.0);
  for(int m = 0; m < 6; m++) {
    for(unsigned int l = 0; l < dq_count; l++) {
      twist[m] += jacobian[size_t(m) * dq_count + l] * y[q_count + l];
    }
  }
  const std::vector<int> loads = {body};
  auto forces = [&](const std::vector<double>& state) {
    return AppliedForces(context, t, state, loads, jacobians);
  };

  context.set_collecting_tangents(true);
  forces(y);
  context.set_collecting_tangents(false);
  shift::core::Tangents tangents;
  ASSERT_TRUE(context.CollectTangents(y, tangents));
  const size_t state_offset = q_count + dq_count;
  const size_t row_count = dq_count + (y.size() - state_offset);
  std::vector<double> k(row_count * dq_count, 0.0), c(k);
  for(const auto& e : tangents.get_joint_stiffness()) {
    k[size_t(e.dq_a) * dq_count + e.dq_b] += e.value;
  }
  for(const auto& e : tangents.get_joint_damping()) {
    c[size_t(e.dq_a) * dq_count + e.dq_b] += e.value;
  }
  auto map_blocks = [&](const std::vector<shift::core::Tangents::BodyBlock>&
                          blocks,
                        std::vector<double>& matrix) {
    for(const auto& block : blocks) {
      if(block.load_index_a != body || block.load_index_b != body) {
        continue;
      }
      for(unsigned int l = 0; l < dq_count; l++) {
        for(unsigned int col = 0; col < dq_count; col++) {
          for(int r = 0; r < 6; r++) {
            for(int s = 0; s < 6; s++) {
              matrix[size_t(l) * dq_count + col] +=
                jacobian[size_t(r) * dq_count + l] * block.block[r * 6 + s] *
                jacobian[size_t(s) * dq_count + col];
            }
          }
        }
      }
    }
  };
  map_blocks(tangents.get_body_stiffness(), k);
  map_blocks(tangents.get_body_damping(), c);
  auto add_state = [&](int state, int l, double value,
                       std::vector<double>& matrix) {
    ASSERT_GE(state, static_cast<int>(state_offset));
    ASSERT_LT(state, static_cast<int>(y.size()));
    matrix[size_t(dq_count + state - state_offset) * dq_count + l] += value;
  };
  for(const auto& e : tangents.get_state_stiffness()) {
    add_state(e.state, e.dq, e.value, k);
  }
  for(const auto& e : tangents.get_state_damping()) {
    add_state(e.state, e.dq, e.value, c);
  }
  auto map_rows = [&](const std::vector<shift::core::Tangents::StateBodyRow>&
                        rows,
                      std::vector<double>& matrix) {
    for(const auto& row : rows) {
      if(row.load_index != body) {
        continue;
      }
      for(unsigned int l = 0; l < dq_count; l++) {
        double value = 0.0;
        for(int m = 0; m < 6; m++) {
          value += row.row[m] * jacobian[size_t(m) * dq_count + l];
        }
        add_state(row.state, l, value, matrix);
      }
    }
  };
  map_rows(tangents.get_state_body_stiffness(), k);
  map_rows(tangents.get_state_body_damping(), c);

  /* the generalized velocities at a state for the twist of the run */
  auto hold_twist = [&](std::vector<double>& state) {
    std::vector<double> a = jacobian_at(state);
    std::vector<double> b(twist);
    ASSERT_EQ(LAPACKE_dgels(LAPACK_ROW_MAJOR, 'N', 6, dq_count, 1, a.data(),
                            dq_count, b.data(), 1),
              0);
    std::copy(b.begin(), b.begin() + dq_count, state.begin() + q_count);
  };
  const double step = 1e-7;
  std::vector<double> k_difference(row_count * dq_count, 0.0);
  std::vector<double> c_difference(k_difference);
  for(unsigned int l = 0; l < dq_count; l++) {
    for(int velocity = 0; velocity < 2; velocity++) {
      std::vector<double> plus(y), minus(y);
      if(velocity) {
        plus[q_count + l] += step;
        minus[q_count + l] -= step;
      } else {
        std::vector<double> delta(dq_count, 0.0);
        delta[l] = step;
        context.IncrementPositions(y.data(), delta.data(), plus.data());
        delta[l] = -step;
        context.IncrementPositions(y.data(), delta.data(), minus.data());
        ASSERT_NO_FATAL_FAILURE(hold_twist(plus));
        ASSERT_NO_FATAL_FAILURE(hold_twist(minus));
      }
      const std::vector<double> q_plus = forces(plus);
      const std::vector<double> q_minus = forces(minus);
      std::vector<double>& matrix = velocity ? c_difference : k_difference;
      for(size_t i = 0; i < row_count; i++) {
        matrix[i * dq_count + l] = (i < dq_count ? -1.0 : 1.0) *
                                   (q_plus[i] - q_minus[i]) / (2.0 * step);
      }
    }
  }

  /* each row against its largest entry, above roundoff of the largest of
  its kind */
  for(int kind = 0; kind < 2; kind++) {
    const auto& tangent = kind ? c : k;
    const auto& difference = kind ? c_difference : k_difference;
    double scale[2] = {0.0, 0.0};
    for(size_t e = 0; e < difference.size(); e++) {
      double& part = scale[e < size_t(dq_count) * dq_count ? 0 : 1];
      part = std::max(part, std::fabs(difference[e]));
    }
    /* (the forces of a module may not depend on the velocities, only the
    rates of its states) */
    EXPECT_GT(scale[0] + scale[1], 0.0);
    for(size_t i = 0; i < row_count; i++) {
      double largest = 0.0, error = 0.0;
      unsigned int worst = 0;
      for(unsigned int l = 0; l < dq_count; l++) {
        const size_t e = i * dq_count + l;
        largest = std::max(largest, std::fabs(difference[e]));
        if(std::fabs(tangent[e] - difference[e]) > error) {
          error = std::fabs(tangent[e] - difference[e]);
          worst = l;
        }
      }
      EXPECT_LE(error, tolerance * std::max(largest,
                                            1e-6 * scale[i < dq_count ? 0 : 1]))
        << (kind ? "damping" : "stiffness")
        << (i < dq_count ? " of force " : " of the rate of state ")
        << (i < dq_count ? i : i - dq_count) << ": largest " << largest
        << ", at " << worst << " the tangent " << tangent[i * dq_count + worst]
        << " and the difference " << difference[i * dq_count + worst];
    }
  }
}

/* a sphere of 0.25 m rolling with slip on SCM soil (soil_contact), on
joints along x and z and about y: the rays' points move over its curved
surface, and the moment of the forces about the center changes with them.
Lifted 0.1 mm after the run, so the nodes are on their smooth unloading
lines, not at the corner where loading meets unloading (the accepted plastic
sinkage) */
static const char* kRollingSphere = R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.05, "time_step": 0.01,
                     "relative_error": 1e-8, "absolute_error": 1e-9}]
  },
  "frames": [{"name": "terrain", "T": [0, 0, 0]}],
  "terrain": [{"type": "scm", "name": "site", "parent": [["fixed"], "terrain"],
    "x": [-1, 1], "y": [-0.5, 0.5], "spacing": 0.05, "damping": 5,
    "soil": {"sinkage_exponent": 1.1, "cohesive_modulus": 2e4,
             "frictional_modulus": 1e6, "cohesion": 1000,
             "friction_angle": 0.5, "shear_modulus": 0.01}}],
  "assemblies": [{"name": "a",
    "force_modules": [{"type": "soil_contact", "name": "soil",
                       "bodies": ["sphere"]}],
    "bodies": [
      {"name": "carriage", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}},
      {"name": "lift", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}},
      {"name": "sphere", "center_of_mass": [0, 0, 0],
       "collision_geometry": [{"name": "ball", "type": "sphere",
                               "radius": 0.25}],
       "inertia": {"mass": 100, "Ixx": 2.5, "Iyy": 2.5, "Izz": 2.5,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "x", "type": "prismatic", "parent": [["fixed"], "origin"],
       "child": [["carriage"], "origin"], "axis": [1, 0, 0],
       "initial_velocity": 0.3},
      {"name": "z", "type": "prismatic", "parent": [["carriage"], "origin"],
       "child": [["lift"], "origin"], "axis": [0, 0, 1],
       "initial_displacement": -0.23},
      {"name": "pitch", "type": "revolute", "parent": [["lift"], "origin"],
       "child": [["sphere"], "origin"], "axis": [0, 1, 0],
       "initial_velocity": 0.6}]
  }]
})";

TEST(Tangents, SoilContactRollingSphere) {
  CheckAtFixedTwist(
    "soil_contact_sphere.json", kRollingSphere, 0.05, "sphere", 1e-4,
    [](shift::core::Context& context, double& t, std::vector<double>& y) {
      for(auto* joint : context.get_active_joints()) {
        if(joint->get_name() == "z") {
          y[joint->get_q_index()] -= 1e-4;
        }
      }
      /* 20 ms after the accepted step, for a shear displacement that grows
      with the slip enough to show */
      t += 2e-2;
    });
}

/* a shoe of a track, yawed, pitched and rolled so that no two stations are
as deep, sliding and turning on the ground plane (track_soil): the rates of
the shear displacement change with the velocity of the shoe's center as it
moves with the shoe. After 2 ms, still in the sand (it springs out by 8 ms) */
static const char* kSlidingShoe = R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.002, "time_step": 0.001,
                     "relative_error": 1e-8, "absolute_error": 1e-9}]
  },
  "assemblies": [{"name": "a",
    "bodies": [{"name": "shoe", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 30, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
      "child": [["shoe"], "origin"],
      "initial_displacement": {"T": [0, 0, 0.03],
        "R": [[0.9541, -0.2955, 0.0490], [0.2940, 0.9553, 0.0305],
              [-0.0558, -0.0148, 0.9983]]},
      "initial_velocity": [0.3, 0.1, 0.0, 0.2, -0.3, 0.4]}],
    "force_modules": [{"type": "track_soil", "name": "soil",
      "soil": {"name": "lete_sand", "unit_weight": 16000},
      "shoes": ["shoe"], "shoe_center": [0, 0, 0], "shoe_length": 0.2,
      "shoe_width": 0.5, "grouser_height": 0.02}]}]
})";

TEST(Tangents, TrackSoilSlidingShoe) {
  CheckAtFixedTwist("track_soil_shoe.json", kSlidingShoe, 0.002, "shoe", 1e-4,
                    [](shift::core::Context&, double&, std::vector<double>&) {
                    });
}

