/* A flexible body (body::Flex) in RNCA against an equivalent model of rigid
bodies. The flexible body is two lumped points with springs to its reference
frame, along the axes of its origin, described by six elastic coordinates
through a full, non-diagonal shape matrix: with the full set and no
rotations of the points, the floating frame equations are exact, so the
model must move as the rigid one, in which each point is a body on three
prismatic joints with springs. A body hangs from a node of the flexible
body, the reference frame is offset and turned from the origin, and the
chain moves in three dimensions under gravity. */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/body/flex.h"
#include "shift/core/constraint/chord.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/integrator/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/linearization.h"
#include "shift/core/solver/base.h"

#include "ksl/matrix.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"

class FlexParticles {
public:
  static constexpr int kPoints = 2;
  static constexpr int kCount = 6;

  static ksl_vec3_t Position(int p) {
    return p == 0 ? ksl_vec3(1.0, 0.0, 0.0) : ksl_vec3(0.4, 0.3, -0.2);
  }
  static double Mass(int p) {
    return p == 0 ? 0.7 : 1.3;
  }
  /* rotary inertia about the point, in the origin's axes: {Ixx, Iyy, Izz,
  Ixy, Iyz, Izx} */
  static std::vector<double> Rotary(int p) {
    return p == 0 ? std::vector<double>{0.010, 0.020, 0.030, 0.002, 0.0, 0.0}
                  : std::vector<double>{0.020, 0.010, 0.015, 0.0, -0.001,
                                        0.0005};
  }
  /* spring stiffness of point p along axis a of the origin */
  static double Stiffness(int p, int a) {
    const double k[kCount] = {40.0, 55.0, 70.0, 35.0, 60.0, 45.0};
    return k[3 * p + a];
  }
  /* the shape matrix: the point displacements u (point p, axis a at row
  3 p + a) per elastic coordinate, u = A eta, A = I + 0.1 B */
  static double A(int row, int column) {
    return (row == column ? 1.0 : 0.0) + 0.1 * std::sin(1.0 + 6 * row + column);
  }
  static double U(const double* eta, int row) {
    double u = 0.0;
    for(int j = 0; j < kCount; j++) {
      u += A(row, j) * eta[j];
    }
    return u;
  }

  static constexpr double kEta[kCount] = {0.02, -0.01, 0.015,
                                          0.01, -0.02, 0.005};
  static constexpr double kEtaDot[kCount] = {0.1, -0.2, 0.05, 0.15, -0.1, 0.2};

  /* the frames of body s in both models */
  static std::string Frames(bool flexible) {
    std::stringstream ss;
    ss << std::setprecision(17) << std::showpoint;
    ss << R"([{"name": "root", "T": [0.1, 0.05, 0.0],
               "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]})";
    if(flexible) {
      ss << R"(, {"name": "tip", "T": [1.0, 0.0, 0.0]})";
    } else {
      for(int p = 0; p < kPoints; p++) {
        const ksl_vec3_t t = Position(p);
        ss << R"(, {"name": "a)" << p << R"(", "T": [)" << t.x << ", " << t.y
           << ", " << t.z << "]}";
      }
    }
    ss << "]";
    return ss.str();
  }

  static std::string Inertia(double m, const std::vector<double>& I,
                             const std::string& center = "[0.0, 0.0, 0.0]") {
    std::stringstream ss;
    ss << std::setprecision(17) << std::showpoint << R"("center_of_mass": )" << center
       << R"(, "inertia": {"mass": )" << m << R"(, "Ixx": )" << I[0]
       << R"(, "Iyy": )" << I[1] << R"(, "Izz": )" << I[2] << R"(, "Ixy": )"
       << I[3] << R"(, "Iyz": )" << I[4] << R"(, "Izx": )" << I[5] << "}";
    return ss.str();
  }

  /* \p welded adds a body w welded at the node (at body m0 in the rigid
  model) by a rigid joint at its frame "hook", which is offset and turned */
  static std::string Model(bool flexible, double time_span,
                           bool damped = false, bool welded = false) {
    std::stringstream ss;
    ss << std::setprecision(17) << std::showpoint;
    ss << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
       << time_span << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "assemblies": [{"name": "chain",
  "bodies": [
   {"name": "a", "geometry": [], "frames": [{"name": "tip", "T": [1, 0, 0]}],
    )" << Inertia(1.0, {0.1, 0.12, 0.08, 0.0, 0.0, 0.0}) << R"(},
   {"name": "c", "geometry": [],
    )" << Inertia(0.9, {0.05, 0.04, 0.06, 0.01, 0.0, 0.0}, "[0.2, 0.0, 0.1]")
       << "},\n";
    if(welded) {
      ss << R"(   {"name": "w", "geometry": [],
    "frames": [{"name": "hook", "T": [-0.1, 0.2, 0.05],
                "R": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]}],
    )" << Inertia(0.4, {0.03, 0.02, 0.025, 0.004, -0.003, 0.002},
                  "[0.15, -0.1, 0.2]")
         << "},\n";
    }
    if(flexible) {
      ss << R"(   {"name": "s", "type": )"
         << (damped ? R"("test_flex_particles_damped")"
                    : R"("test_flex_particles")")
         << R"(, "geometry": [], "frames": )" << Frames(true) << "}";
    } else {
      ss << R"(   {"name": "s", "geometry": [], "frames": )" << Frames(false)
         << ", " << Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
      for(int p = 0; p < kPoints; p++) {
        ss << ",\n   {\"name\": \"x" << p << "\", \"geometry\": [], "
           << Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
        ss << ",\n   {\"name\": \"y" << p << "\", \"geometry\": [], "
           << Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
        ss << ",\n   {\"name\": \"m" << p << "\", \"geometry\": [], "
           << Inertia(Mass(p), Rotary(p)) << "}";
      }
    }
    ss << R"(],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 0, 1],
    "initial_velocity": 1.0,
    "parent": [["fixed"], "origin"], "child": [["a"], "origin"]},
   {"name": "J2", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.3, "initial_velocity": -0.5,
    "parent": [["a"], "tip"], "child": [["s"], "root"]},
   {"name": "J3", "type": "revolute", "axis": [0, 1, 0],
    "initial_velocity": 0.8,
    "parent": )"
       << (flexible ? R"([["s"], "tip"])" : R"([["m0"], "origin"])")
       << R"(, "child": [["c"], "origin"]})";
    if(welded) {
      ss << R"(,
   {"name": "W", "type": "rigid", "parent": )"
         << (flexible ? R"([["s"], "tip"])" : R"([["m0"], "origin"])")
         << R"(, "child": [["w"], "hook"]})";
    }
    if(!flexible) {
      const char* axes[3] = {"[1, 0, 0]", "[0, 1, 0]", "[0, 0, 1]"};
      for(int p = 0; p < kPoints; p++) {
        const std::string parents[3] = {"[[\"s\"], \"a" + std::to_string(p) +
                                          "\"]",
                                        "[[\"x" + std::to_string(p) +
                                          "\"], \"origin\"]",
                                        "[[\"y" + std::to_string(p) +
                                          "\"], \"origin\"]"};
        const std::string children[3] = {"x" + std::to_string(p),
                                         "y" + std::to_string(p),
                                         "m" + std::to_string(p)};
        for(int a = 0; a < 3; a++) {
          const int row = 3 * p + a;
          ss << ",\n   {\"name\": \"P" << p << a
             << "\", \"type\": \"prismatic\", \"axis\": " << axes[a]
             << ", \"initial_displacement\": " << U(kEta, row)
             << ", \"initial_velocity\": " << U(kEtaDot, row)
             << ", \"stiffness_coefficients\": [0.0, " << Stiffness(p, a)
             << "]";
          if(damped) {
            ss << ", \"damping_coefficients\": [0.0, " << Damping(p, a) << "]";
          }
          ss << ", \"parent\": " << parents[a] << ", \"child\": [[\""
             << children[a] << "\"], \"origin\"]}";
        }
      }
    }
    ss << "]}]}\n";
    return ss.str();
  }

  static double Damping(int p, int a) {
    return 0.05 * Stiffness(p, a);
  }

  static std::unique_ptr<shift::core::Context> Load(const std::string& model,
                                                    const std::string& name) {
    const std::string path = testing::TempDir() + "/" + name + ".json";
    {
      std::ofstream file(path);
      file << model;
    }
    auto context = std::make_unique<shift::core::Context>();
    context->LoadJSON(path);
    std::remove(path.c_str());
    return context;
  }

  /* the state derivatives at the context's initial state */
  static std::vector<double> Derivatives(shift::core::Context& context) {
    context.get_active_integrators()[0]->Initialize();
    auto solver = context["model_parameters"]["solvers"][0]
                    .get_data<shift::core::solver::Base>();
    solver->Initialize();
    std::vector<double> y(context.get_state_count() +
                            context.get_auxiliary_state_count(),
                          0.0);
    std::copy(context.get_q().begin(), context.get_q().end(), y.begin());
    std::copy(context.get_dq(), context.get_dq() + context.get_dq_count(),
              y.begin() + context.get_q_count());
    std::vector<double> dy(y.size(), 0.0);
    EXPECT_FALSE(solver->Solve(0.0, y, dy));
    return dy;
  }

  static std::vector<double> Run(shift::core::Context& context) {
    std::vector<double> last;
    context.set_state_observer(
      [&last](double, const std::vector<double>& y) { last = y; });
    context.Simulate();
    EXPECT_GE(last.size(), context.get_state_count());
    last.resize(context.get_state_count());
    return last;
  }

  static int DqIndex(shift::core::Context& context, const std::string& name) {
    for(auto joint : context.get_active_joints()) {
      if(joint->get_name() == name) {
        return joint->get_dq_index();
      }
    }
    ADD_FAILURE() << "no joint " << name;
    return 0;
  }

  static int QIndex(shift::core::Context& context, const std::string& name) {
    for(auto joint : context.get_active_joints()) {
      if(joint->get_name() == name) {
        return joint->get_q_index();
      }
    }
    ADD_FAILURE() << "no joint " << name;
    return 0;
  }

  static shift::core::body::Flex* FlexBody(shift::core::Context& context) {
    for(int i : context.get_elastic_bodies()) {
      return dynamic_cast<shift::core::body::Flex*>(
        context.get_active_articulated_bodies()[i]);
    }
    return nullptr;
  }

  /* compare the joints and the point displacements of the two models: \p y
  holds the values (offset 0) or rates (offset q_count) of each */
  static void Compare(shift::core::Context& flexible,
                      const std::vector<double>& y_flexible,
                      shift::core::Context& rigid,
                      const std::vector<double>& y_rigid, bool rates,
                      double tolerance) {
    const int f_offset = rates ? (int)flexible.get_q_count() : 0;
    const int r_offset = rates ? (int)rigid.get_q_count() : 0;
    for(const std::string name : {"J1", "J2", "J3"}) {
      const int f = rates ? DqIndex(flexible, name) : QIndex(flexible, name);
      const int r = rates ? DqIndex(rigid, name) : QIndex(rigid, name);
      EXPECT_NEAR(y_flexible[f_offset + f], y_rigid[r_offset + r], tolerance)
        << name << (rates ? " rate" : "");
    }
    auto body = FlexBody(flexible);
    ASSERT_NE(body, nullptr);
    const int e = rates ? body->get_elastic_dq_index()
                        : body->get_elastic_q_index();
    for(int p = 0; p < kPoints; p++) {
      for(int a = 0; a < 3; a++) {
        const std::string name = "P" + std::to_string(p) + std::to_string(a);
        const int r = rates ? DqIndex(rigid, name) : QIndex(rigid, name);
        EXPECT_NEAR(U(&y_flexible[f_offset + e], 3 * p + a),
                    y_rigid[r_offset + r], tolerance)
          << name << (rates ? " rate" : "");
      }
    }
  }
};

/*! the flexible body of the points, with its model set as it is created */
class TestFlexParticles : public shift::core::body::Flex {
public:
  /* \p at_rest starts it undeformed and at rest */
  /* \p two_nodes adds a second node, "mid" at point 1 */
  explicit TestFlexParticles(bool damped, bool at_rest = false,
                             bool two_nodes = false) {
    const int n = FlexParticles::kCount;
    std::vector<ksl_vec3_t> positions;
    std::vector<double> masses;
    std::vector<ksl_mat3x3_t> rotary;
    std::vector<ksl_screw_t> shapes(FlexParticles::kPoints * n);
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      positions.push_back(FlexParticles::Position(p));
      masses.push_back(FlexParticles::Mass(p));
      const std::vector<double> I = FlexParticles::Rotary(p);
      ksl_mat3x3_t R = {};
      R.m00 = I[0];
      R.m11 = I[1];
      R.m22 = I[2];
      R.m01 = R.m10 = I[3];
      R.m12 = R.m21 = I[4];
      R.m02 = R.m20 = I[5];
      rotary.push_back(R);
      for(int j = 0; j < n; j++) {
        shapes[p * n + j] =
          ksl_screw(FlexParticles::A(3 * p, j), FlexParticles::A(3 * p + 1, j),
                    FlexParticles::A(3 * p + 2, j), 0.0, 0.0, 0.0);
      }
    }
    /* K = A^T diag(k) A, C = A^T diag(c) A */
    std::vector<double> stiffness(n * n, 0.0);
    std::vector<double> damping(n * n, 0.0);
    for(int i = 0; i < n; i++) {
      for(int j = 0; j < n; j++) {
        for(int row = 0; row < n; row++) {
          const double aa = FlexParticles::A(row, i) * FlexParticles::A(row, j);
          stiffness[i * n + j] += aa * FlexParticles::Stiffness(row / 3, row % 3);
          if(damped) {
            damping[i * n + j] += aa * FlexParticles::Damping(row / 3, row % 3);
          }
        }
      }
    }
    SetModel(positions, masses, rotary, n, shapes, stiffness, damping);
    SetNode("tip", 0);
    if(two_nodes) {
      SetNode("mid", 1);
    }
    if(!at_rest) {
      SetInitialElasticState(
        std::vector<double>(FlexParticles::kEta, FlexParticles::kEta + n),
        std::vector<double>(FlexParticles::kEtaDot,
                            FlexParticles::kEtaDot + n));
    }
  }
  static std::unique_ptr<shift::core::body::Base> Create() {
    return std::make_unique<TestFlexParticles>(false);
  }
  static std::unique_ptr<shift::core::body::Base> CreateAtRest() {
    return std::make_unique<TestFlexParticles>(false, true);
  }
  static std::unique_ptr<shift::core::body::Base> CreateDamped() {
    return std::make_unique<TestFlexParticles>(true);
  }
  static std::unique_ptr<shift::core::body::Base> CreateTwoNodes() {
    return std::make_unique<TestFlexParticles>(false, false, true);
  }
  static std::unique_ptr<shift::core::body::Base> CreateTwoNodesAtRest() {
    return std::make_unique<TestFlexParticles>(false, true, true);
  }
  static bool registered_;
  static bool registered_damped_;
  static bool registered_at_rest_;
  static bool registered_two_nodes_;
  static bool registered_two_nodes_at_rest_;
};

bool TestFlexParticles::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_particles", TestFlexParticles::Create);
bool TestFlexParticles::registered_damped_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_particles_damped", TestFlexParticles::CreateDamped);
bool TestFlexParticles::registered_at_rest_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_particles_at_rest", TestFlexParticles::CreateAtRest);
bool TestFlexParticles::registered_two_nodes_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_particles_two_nodes", TestFlexParticles::CreateTwoNodes);
bool TestFlexParticles::registered_two_nodes_at_rest_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_particles_two_nodes_at_rest",
    TestFlexParticles::CreateTwoNodesAtRest);

TEST(FlexBody, AccelerationsMatchRigidModel) {
  auto flexible = FlexParticles::Load(FlexParticles::Model(true, 0.1),
                                      "flex_particles_accel");
  auto rigid = FlexParticles::Load(FlexParticles::Model(false, 0.1),
                                   "flex_particles_accel_rigid");
  ASSERT_EQ(flexible->get_dq_count(), 9u);
  ASSERT_EQ(rigid->get_dq_count(), 9u);

  const std::vector<double> dy_flexible = FlexParticles::Derivatives(*flexible);
  const std::vector<double> dy_rigid = FlexParticles::Derivatives(*rigid);

  /* the same state: velocities, in the first half of dy */
  FlexParticles::Compare(*flexible, dy_flexible, *rigid, dy_rigid, false,
                         1e-12);
  /* and the accelerations */
  FlexParticles::Compare(*flexible, dy_flexible, *rigid, dy_rigid, true,
                         1e-9);
  double largest = 0.0;
  for(size_t k = flexible->get_q_count(); k < dy_flexible.size(); k++) {
    largest = std::fmax(largest, std::fabs(dy_flexible[k]));
  }
  EXPECT_GT(largest, 1.0);
}

TEST(FlexBody, KineticEnergyMatchesRigidModel) {
  auto flexible = FlexParticles::Load(FlexParticles::Model(true, 0.1),
                                      "flex_particles_energy");
  auto rigid = FlexParticles::Load(FlexParticles::Model(false, 0.1),
                                   "flex_particles_energy_rigid");
  FlexParticles::Derivatives(*flexible);
  FlexParticles::Derivatives(*rigid);
  const double ke_flexible = flexible->KineticEnergy();
  const double ke_rigid = rigid->KineticEnergy();
  EXPECT_GT(ke_rigid, 0.1);
  EXPECT_NEAR(ke_flexible, ke_rigid, 1e-12 * ke_rigid);
  EXPECT_NEAR(flexible->GravitationalPotentialEnergy(),
              rigid->GravitationalPotentialEnergy(), 1e-12);
}

TEST(FlexBody, TrajectoryMatchesRigidModel) {
  for(bool damped : {false, true}) {
    auto flexible = FlexParticles::Load(FlexParticles::Model(true, 1.0, damped),
                                        "flex_particles_run");
    auto rigid = FlexParticles::Load(FlexParticles::Model(false, 1.0, damped),
                                     "flex_particles_run_rigid");
    const std::vector<double> y_flexible = FlexParticles::Run(*flexible);
    const std::vector<double> y_rigid = FlexParticles::Run(*rigid);
    SCOPED_TRACE(damped ? "damped" : "undamped");
    FlexParticles::Compare(*flexible, y_flexible, *rigid, y_rigid, false,
                           1e-7);
    FlexParticles::Compare(*flexible, y_flexible, *rigid, y_rigid, true, 1e-6);
  }
}

TEST(FlexBody, EnergyIsConserved) {
  auto context = FlexParticles::Load(FlexParticles::Model(true, 1.0),
                                     "flex_particles_conserve");
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  auto energy = [&]() {
    const double* q = context->get_q().data();
    return context->KineticEnergy() + context->GravitationalPotentialEnergy() +
           body->ElasticEnergy(q + body->get_elastic_q_index());
  };
  FlexParticles::Derivatives(*context);
  const double initial = energy();

  const std::vector<double> y = FlexParticles::Run(*context);
  /* the final state, with the solver's momenta evaluated there */
  std::copy(y.begin(), y.begin() + context->get_q_count(),
            context->get_q().begin());
  std::copy(y.begin() + context->get_q_count(), y.end(), context->get_dq());
  auto solver = (*context)["model_parameters"]["solvers"][0]
                  .get_data<shift::core::solver::Base>();
  std::vector<double> y_full(y);
  y_full.resize(context->get_state_count() +
                context->get_auxiliary_state_count());
  std::vector<double> dy(y_full.size(), 0.0);
  ASSERT_FALSE(solver->Solve(1.0, y_full, dy));
  const double final_energy = energy();
  EXPECT_NEAR(final_energy, initial, 1e-7 * std::fabs(initial) + 1e-9);
  EXPECT_GT(std::fabs(initial), 0.1);
}



/*! a flexible body whose points also turn with the elastic coordinates,
and carry rotary inertia: the shape matrix has rotations, which the rigid
model above cannot represent, so its equations are checked by the
conservation of energy */
class TestFlexRotating : public shift::core::body::Flex {
public:
  static constexpr int kPoints = 3;
  static constexpr int kCount = 4;

  /* \p nodes makes the frames "tip" and "mid" nodes at points 0 and 1;
  without \p rotations the shapes only translate the points, so the nodes'
  motion is exact rather than linear in the elastic coordinates */
  explicit TestFlexRotating(bool nodes = false, bool rotations = true) {
    const ksl_vec3_t positions[kPoints] = {ksl_vec3(1.0, 0.0, 0.0),
                                           ksl_vec3(0.5, 0.2, -0.1),
                                           ksl_vec3(0.2, -0.3, 0.15)};
    const double masses[kPoints] = {0.6, 0.9, 0.5};
    std::vector<ksl_vec3_t> p(positions, positions + kPoints);
    std::vector<double> m(masses, masses + kPoints);
    std::vector<ksl_mat3x3_t> rotary(kPoints);
    std::vector<ksl_screw_t> shapes(kPoints * kCount);
    for(int i = 0; i < kPoints; i++) {
      rotary[i] = {};
      rotary[i].m00 = 0.01 * (i + 1);
      rotary[i].m11 = 0.015 * (i + 1);
      rotary[i].m22 = 0.02;
      rotary[i].m01 = rotary[i].m10 = 0.002;
      for(int j = 0; j < kCount; j++) {
        const double a = 1.0 + 7 * i + j;
        const double r = rotations ? 0.5 : 0.0;
        shapes[i * kCount + j] =
          ksl_screw(0.3 * std::sin(a), 0.3 * std::sin(2 * a),
                    0.3 * std::sin(3 * a), r * std::cos(a),
                    r * std::cos(2 * a), r * std::cos(3 * a));
      }
    }
    /* symmetric, positive definite */
    std::vector<double> stiffness(kCount * kCount, 0.0);
    for(int i = 0; i < kCount; i++) {
      for(int j = 0; j < kCount; j++) {
        stiffness[i * kCount + j] =
          (i == j ? 30.0 + 15.0 * i : 0.0) + 2.0 * std::cos(1.0 + i + j);
      }
    }
    SetModel(p, m, rotary, kCount, shapes, stiffness, {});
    if(nodes) {
      SetNode("tip", 0);
      SetNode("mid", 1);
    }
    SetInitialElasticState({0.03, -0.02, 0.01, 0.025}, {0.4, -0.3, 0.5, -0.2});
  }
  static std::unique_ptr<shift::core::body::Base> Create() {
    return std::make_unique<TestFlexRotating>();
  }

  /* the kinetic energy summed point by point from the model as given, in
  the origin's coordinates, for the velocity v_r of the reference frame in
  its own coordinates */
  double PointKineticEnergy(const double* eta, const double* eta_dot,
                            const ksl_screw_t& v_r) {
    ksl_screw_t v_o;
    ksl_product_Ads(get_reference_frame()->get_d_wrt_origin(), &v_r, &v_o);
    double energy = 0.0;
    for(int p = 0; p < kPoints; p++) {
      ksl_vec3_t rho = positions_[p];
      ksl_vec3_t velocity = v_o.lin;
      ksl_vec3_t omega = v_o.ang;
      for(int j = 0; j < kCount; j++) {
        const ksl_screw_t& phi = shapes_[p * kCount + j];
        ksl_axpy_vv(eta[j], &phi.lin, &rho);
        ksl_axpy_vv(eta_dot[j], &phi.lin, &velocity);
        ksl_axpy_vv(eta_dot[j], &phi.ang, &omega);
      }
      ksl_vec3_t w_x_rho;
      ksl_cross_vv(&v_o.ang, &rho, &w_x_rho);
      ksl_xpy_vv(&w_x_rho, &velocity);
      ksl_vec3_t I_omega;
      ksl_product_drv(&rotary_inertias_[p], &omega, &I_omega);
      energy += 0.5 * masses_[p] * ksl_dot_vv(&velocity, &velocity) +
                0.5 * ksl_dot_vv(&omega, &I_omega);
    }
    return energy;
  }
  static bool registered_;
};

bool TestFlexRotating::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_rotating", TestFlexRotating::Create) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_rotating_nodes",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestFlexRotating>(true);
    }) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_translating_nodes",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestFlexRotating>(true, false);
    });

/* the chain fixed -J1- a -J2- s with the rotating flexible body s, solved
by \p solver */
static std::string RotatingModel(const std::string& solver,
                                 double time_span) {
  std::stringstream ss;
  ss << std::setprecision(17) << std::showpoint;
  ss << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": ")"
     << solver << R"("}],
  "integrators": [{"type": "ode113", "time_span": )"
     << time_span << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "assemblies": [{"name": "chain",
  "bodies": [
   {"name": "a", "geometry": [], "frames": [{"name": "tip", "T": [1, 0, 0]}],
    )" << FlexParticles::Inertia(1.0, {0.1, 0.12, 0.08, 0.0, 0.0, 0.0})
     << R"(},
   {"name": "s", "type": "test_flex_rotating", "geometry": [],
    "frames": [{"name": "root", "T": [0.1, 0.05, 0.0],
               "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}]}],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 0, 1],
    "initial_velocity": 1.5,
    "parent": [["fixed"], "origin"], "child": [["a"], "origin"]},
   {"name": "J2", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.3, "initial_velocity": -2.0,
    "parent": [["a"], "tip"], "child": [["s"], "root"]}]}]}
)";
  return ss.str();
}

TEST(FlexBody, EnergyIsConservedWithRotatingShapes) {
  auto context =
    FlexParticles::Load(RotatingModel("rnca", 1.0), "flex_rotating_energy");
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  auto energy = [&]() {
    const double* q = context->get_q().data();
    return context->KineticEnergy() + context->GravitationalPotentialEnergy() +
           body->ElasticEnergy(q + body->get_elastic_q_index());
  };
  FlexParticles::Derivatives(*context);
  const double initial = energy();
  const double kinetic = context->KineticEnergy();

  const std::vector<double> y = FlexParticles::Run(*context);
  std::copy(y.begin(), y.begin() + context->get_q_count(),
            context->get_q().begin());
  std::copy(y.begin() + context->get_q_count(), y.end(), context->get_dq());
  auto solver = (*context)["model_parameters"]["solvers"][0]
                  .get_data<shift::core::solver::Base>();
  std::vector<double> y_full(y);
  y_full.resize(context->get_state_count() +
                context->get_auxiliary_state_count());
  std::vector<double> dy(y_full.size(), 0.0);
  ASSERT_FALSE(solver->Solve(1.0, y_full, dy));
  EXPECT_GT(kinetic, 0.5);
  EXPECT_NEAR(energy(), initial, 1e-7 * kinetic);
  /* the elastic coordinates have moved */
  const int e = body->get_elastic_q_index();
  EXPECT_GT(std::fabs(y[e] - 0.03), 1e-3);
}

/* ABA and DCA solve a tree with a flexible body, and 'auto' gives it to
ABA, as any tree without loops: all follow RNCA */
TEST(FlexBody, SolversRouteFlexibleTrees) {
  auto rnca = FlexParticles::Load(RotatingModel("rnca", 0.05),
                                  "flex_rotating_rnca");
  const std::vector<double> y_rnca = FlexParticles::Run(*rnca);
  for(const std::string solver : {"aba", "dca", "auto"}) {
    auto context = FlexParticles::Load(RotatingModel(solver, 0.05),
                                       "flex_rotating_" + solver);
    const std::vector<double> y = FlexParticles::Run(*context);
    ASSERT_EQ(y.size(), y_rnca.size());
    for(size_t k = 0; k < y.size(); k++) {
      EXPECT_NEAR(y[k], y_rnca[k], 1e-8 * std::fmax(1.0, std::fabs(y_rnca[k])))
        << solver << " " << k;
    }
  }
}

/* the mass matrix from the invariants against the kinetic energy of the
points, at a deformed, moving state */
TEST(FlexBody, MassMatrixMatchesPoints) {
  auto context =
    FlexParticles::Load(RotatingModel("rnca", 0.1), "flex_rotating_points");
  auto body = dynamic_cast<TestFlexRotating*>(FlexParticles::FlexBody(*context));
  ASSERT_NE(body, nullptr);
  const double eta[4] = {0.05, -0.04, 0.03, 0.02};
  const double eta_dot[4] = {0.7, -0.4, 0.9, -0.6};
  const ksl_screw_t v_r = ksl_screw(0.3, -0.8, 0.5, 1.2, -0.7, 0.4);
  const double expected = body->PointKineticEnergy(eta, eta_dot, v_r);
  EXPECT_GT(expected, 0.5);
  EXPECT_NEAR(body->KineticEnergy(eta, eta_dot, v_r), expected,
              1e-13 * expected);
}

/* a body welded at the node: its inertia is held by the node slot, and must
act as the same body welded to the point mass of the rigid model */
TEST(FlexBody, WeldedBodyMatchesRigidModel) {
  auto flexible = FlexParticles::Load(
    FlexParticles::Model(true, 0.1, false, true), "flex_particles_welded");
  auto rigid = FlexParticles::Load(
    FlexParticles::Model(false, 0.1, false, true), "flex_particles_welded_rigid");
  const std::vector<double> dy_flexible = FlexParticles::Derivatives(*flexible);
  const std::vector<double> dy_rigid = FlexParticles::Derivatives(*rigid);
  FlexParticles::Compare(*flexible, dy_flexible, *rigid, dy_rigid, true, 1e-9);
  EXPECT_NEAR(flexible->KineticEnergy(), rigid->KineticEnergy(),
              1e-12 * rigid->KineticEnergy());
  EXPECT_NEAR(flexible->GravitationalPotentialEnergy(),
              rigid->GravitationalPotentialEnergy(), 1e-12);
}

/*! a uniform cantilever along the x axis of its origin, bending in the x-z
plane (Euler-Bernoulli), by its lowest \p modes clamped-free mode shapes in
closed form: kPoints points of mass mu h at the middles of kPoints segments
of length h, translating along z by phi_n and turning about y by -phi_n'.
With phi_n normalized to int phi_n^2 = L, the modal mass is mu L and the
modal stiffness EI beta_n^4 L, so the clamped frequencies are
beta_n^2 sqrt(EI / mu) up to the error of the midpoint sums */
class TestFlexBeam : public shift::core::body::Flex {
public:
  static constexpr int kPoints = 500;
  static constexpr double kLength = 1.0;
  static constexpr double kMu = 1.0;  //!< mass per length, kg/m
  static constexpr double kEi = 100.0; //!< bending stiffness, N m^2
  /* beta_n L of the clamped-free and pinned-free beams */
  static constexpr double kClamped[4] = {1.87510406871196, 4.69409113297417,
                                         7.85475743823761, 10.9955407348755};
  static constexpr double kPinned[3] = {3.92660231204792, 7.06858274562873,
                                        10.2101761241668};

  explicit TestFlexBeam(int modes) {
    const double h = kLength / kPoints;
    std::vector<ksl_vec3_t> positions(kPoints);
    std::vector<double> masses(kPoints, kMu * h);
    std::vector<ksl_mat3x3_t> rotary(kPoints);
    std::vector<ksl_screw_t> shapes(kPoints * modes);
    for(int p = 0; p < kPoints; p++) {
      const double x = (p + 0.5) * h;
      positions[p] = ksl_vec3(x, 0.0, 0.0);
      rotary[p] = {};
      for(int n = 0; n < modes; n++) {
        const double b = kClamped[n] / kLength;
        const double bl = kClamped[n];
        const double sigma =
          (std::cosh(bl) + std::cos(bl)) / (std::sinh(bl) + std::sin(bl));
        const double bx = b * x;
        const double phi = std::cosh(bx) - std::cos(bx) -
                           sigma * (std::sinh(bx) - std::sin(bx));
        const double slope = b * (std::sinh(bx) + std::sin(bx) -
                                  sigma * (std::cosh(bx) - std::cos(bx)));
        shapes[p * modes + n] = ksl_screw(0.0, 0.0, phi, 0.0, -slope, 0.0);
      }
    }
    std::vector<double> stiffness(modes * modes, 0.0);
    for(int n = 0; n < modes; n++) {
      stiffness[n * modes + n] = kEi * std::pow(kClamped[n], 4) / std::pow(kLength, 3);
    }
    SetModel(positions, masses, rotary, modes, shapes, stiffness, {});
  }
  /* the frequency (rad/s) of beta_n L */
  static double Omega(double beta_l) {
    return beta_l * beta_l / (kLength * kLength) * std::sqrt(kEi / kMu);
  }
  static bool registered_;
};

constexpr double TestFlexBeam::kClamped[4];
constexpr double TestFlexBeam::kPinned[3];

bool TestFlexBeam::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_beam_1",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestFlexBeam>(1);
    }) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_beam_2",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestFlexBeam>(2);
    }) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_beam_3",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestFlexBeam>(3);
    }) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_flex_beam_4",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestFlexBeam>(4);
    });

/* a model without gravity, at rest, of \p assembly */
static std::string BeamModel(const std::string& assembly) {
  return R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 0.0], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.001, "time_step": 0.001}]},
 )" + assembly + "}\n";
}

/* the beam of \p modes modes on a joint of \p type (rigid: clamped,
revolute about y: pinned) at its root */
static std::string FlexBeamModel(int modes, const std::string& type) {
  return BeamModel(R"("assemblies": [{"name": "a",
  "bodies": [{"name": "beam", "type": "test_flex_beam_)" +
                   std::to_string(modes) + R"(", "geometry": []}],
  "joints": [{"name": "root", "type": ")" +
                   type + R"(", "axis": [0, 1, 0],
    "parent": [["fixed"], "origin"], "child": [["beam"], "origin"]}]}])");
}

/* the beam as \p elements beam elements (force_module::Beam) between rigid
bodies at its nodes, each node on a slide along z and a hinge about y: mass
mu h at a node (half at the tip), and the rotary inertia mu h h^2 / 12 */
static std::string ChainBeamModel(int elements) {
  const double h = TestFlexBeam::kLength / elements;
  std::stringstream ss;
  ss << std::setprecision(17);
  std::stringstream frames, bodies, joints, beams;
  for(int k = 1; k <= elements; k++) {
    const double m = TestFlexBeam::kMu * h * (k == elements ? 0.5 : 1.0);
    const double i = m * h * h / 12.0;
    const std::string n = std::to_string(k);
    frames << (k > 1 ? ", " : "") << R"({"name": "home)" << n
           << R"(", "T": [)" << k * h << ", 0, 0]}";
    bodies << (k > 1 ? ",\n" : "") << R"({"name": "z)" << n
           << R"(", "geometry": [], )"
           << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "},\n"
           << R"({"name": "n)" << n << R"(", "geometry": [], )"
           << FlexParticles::Inertia(m, {i, i, i, 0, 0, 0}) << "}";
    joints << (k > 1 ? ",\n" : "") << R"({"name": "Z)" << n
           << R"(", "type": "prismatic", "axis": [0, 0, 1],
    "parent": [["fixed"], "home)"
           << n << R"("], "child": [["z)" << n << R"("], "origin"]},
   {"name": "R)" << n << R"(", "type": "revolute", "axis": [0, 1, 0],
    "parent": [["z)" << n << R"("], "origin"], "child": [["n)" << n
           << R"("], "origin"]})";
    beams << (k > 1 ? ",\n" : "") << R"({"type": "beam", "name": "b)" << n
          << R"(", "frames": [)"
          << (k == 1 ? std::string(R"(["fixed", "origin"])")
                     : R"(["n)" + std::to_string(k - 1) + R"(", "origin"])")
          << R"(, ["n)" << n << R"(", "origin"]],
    "young_modulus": )"
          << TestFlexBeam::kEi << R"(, "poisson_ratio": 0.3,
    "section": "general", "area": 1.0, "Iy": 1.0, "Iz": 1.0, "J": 1.0})";
  }
  ss << R"("frames": [)" << frames.str() << R"(],
 "assemblies": [{"name": "a", "bodies": [)"
     << bodies.str() << R"(],
  "joints": [)" << joints.str() << R"(],
  "force_modules": [)" << beams.str() << "]}]";
  return BeamModel(ss.str());
}

/* the frequencies (rad/s) of the oscillating modes of \p model at rest,
lowest first */
static std::vector<double> Frequencies(const std::string& model,
                                       const std::string& name) {
  auto context = FlexParticles::Load(model, name);
  std::vector<double> state;
  double time = 0.0;
  context->set_state_observer([&](double t, const std::vector<double>& y) {
    time = t;
    state = y;
  });
  context->Simulate();
  EXPECT_FALSE(state.empty());
  shift::core::Linearization linearization(*context);
  linearization.Compute(time, state);
  std::vector<double> omega;
  for(const auto& mode : linearization.Modes()) {
    if(mode.eigenvalue.imag() > 1.0) {
      EXPECT_LT(std::fabs(mode.eigenvalue.real()),
                1e-6 * mode.eigenvalue.imag());
      omega.push_back(mode.eigenvalue.imag());
    }
  }
  std::sort(omega.begin(), omega.end());
  return omega;
}

/* clamped at its root, the modal beam has the frequencies of the
Euler-Bernoulli cantilever */
TEST(FlexBeam, ClampedFrequencies) {
  const std::vector<double> omega =
    Frequencies(FlexBeamModel(4, "rigid"), "flex_beam_clamped");
  ASSERT_EQ(omega.size(), 4u);
  for(int n = 0; n < 4; n++) {
    const double exact = TestFlexBeam::Omega(TestFlexBeam::kClamped[n]);
    EXPECT_NEAR(omega[n], exact, 1e-4 * exact) << n;
  }
}

/* pinned at its root, the joint's rotation couples with the clamped mode
shapes: a Ritz approximation of the pinned-free beam, its frequencies
falling towards the exact ones from above as modes are added */
TEST(FlexBeam, PinnedFrequenciesConverge) {
  std::vector<double> previous;
  for(int modes = 1; modes <= 4; modes++) {
    const std::vector<double> omega =
      Frequencies(FlexBeamModel(modes, "revolute"),
                  "flex_beam_pinned_" + std::to_string(modes));
    ASSERT_EQ(omega.size(), (size_t)modes);
    const double exact = TestFlexBeam::Omega(TestFlexBeam::kPinned[0]);
    EXPECT_GT(omega[0], exact * (1.0 - 1e-4)) << modes;
    if(!previous.empty()) {
      EXPECT_LT(omega[0], previous[0]) << modes;
    }
    previous = omega;
  }
  const double exact = TestFlexBeam::Omega(TestFlexBeam::kPinned[0]);
  EXPECT_NEAR(previous[0], exact, 1e-4 * exact);
}

/* a chain of rigid bodies joined by beam elements is the same cantilever:
its lowest frequencies converge to those of the modal beam at the second
order of the element length (lumped masses), each halving of the elements
taking about three quarters of the difference */
TEST(FlexBeam, MatchesChainOfBeamElements) {
  const std::vector<double> flexible =
    Frequencies(FlexBeamModel(4, "rigid"), "flex_beam_modal");
  ASSERT_EQ(flexible.size(), 4u);
  std::vector<double> previous;
  for(int elements : {10, 20, 40}) {
    const std::vector<double> chain =
      Frequencies(ChainBeamModel(elements),
                  "flex_beam_chain_" + std::to_string(elements));
    ASSERT_GE(chain.size(), 3u);
    std::vector<double> error(3);
    for(int n = 0; n < 3; n++) {
      error[n] = std::fabs(chain[n] - flexible[n]) / flexible[n];
      if(!previous.empty()) {
        EXPECT_GT(previous[n] / error[n], 3.5) << elements << " mode " << n;
        EXPECT_LT(previous[n] / error[n], 4.5) << elements << " mode " << n;
      }
    }
    previous = error;
  }
  EXPECT_LT(previous[0], 1e-3);
  EXPECT_LT(previous[2], 5e-3);
}

/* a four-bar whose coupler s is the flexible body of the points (at rest,
undeformed): pivots 1 m apart along x, cranks of 0.6 m tilted by 0.5 rad,
the second crank's joint D hanging from the node "tip" (point 0) of s, all
axes along y, under gravity. The rigid model replaces s by the points on
prismatic joints with springs, as above; the loop then runs through the
prismatic joints of point 0, and in the flexible model through the elastic
coordinates, which the full-pivoting factorization of the chord Jacobian may
take as dependent like any joint coordinate */
/* \p model with its solver replaced by \p solver */
static std::string WithSolver(std::string model, const std::string& solver) {
  const std::string from = R"("solvers": [{"type": "rnca"}])";
  const size_t at = model.find(from);
  EXPECT_NE(at, std::string::npos);
  if(at != std::string::npos) {
    model.replace(at, from.size(),
                  R"("solvers": [{"type": ")" + solver + R"("}])");
  }
  return model;
}

/* two flexible bodies in a chain: s2, a second particle body, hangs by its
frame "root" from the node "tip" of s, on a revolute joint */
static std::string TwoFlexModel(double time_span) {
  std::string model = FlexParticles::Model(true, time_span);
  const std::string body = R"(   {"name": "c", "geometry": [],)";
  const size_t at = model.find(body);
  EXPECT_NE(at, std::string::npos);
  model.insert(at, R"(   {"name": "s2", "type": "test_flex_particles",
    "geometry": [], "frames": )" + FlexParticles::Frames(true) + "},\n");
  const std::string joint = R"("child": [["c"], "origin"]})";
  const size_t j = model.find(joint);
  EXPECT_NE(j, std::string::npos);
  model.insert(j + joint.size(), R"(,
   {"name": "J4", "type": "revolute", "axis": [1, 0, 0],
    "initial_velocity": 0.6,
    "parent": [["s"], "tip"], "child": [["s2"], "root"]})");
  return model;
}

/* \p dy and \p reference within a relative 1e-9 */
static void ExpectDerivativesNear(const std::vector<double>& dy,
                                  const std::vector<double>& reference,
                                  const std::string& name) {
  ASSERT_EQ(dy.size(), reference.size()) << name;
  double largest = 0.0;
  for(size_t k = 0; k < dy.size(); k++) {
    largest = std::fmax(largest, std::fabs(reference[k]));
    EXPECT_NEAR(dy[k], reference[k],
                1e-9 * std::fmax(1.0, std::fabs(reference[k])))
      << name << " " << k;
  }
  EXPECT_GT(largest, 0.1) << name;
}

/* ABA, eliminating the elastic coordinates inside each body, gives RNCA's
accelerations at the same deformed, moving state: bodies hanging from a node,
a body welded at a node, damping, rotating shapes, a flexible body on a
rigid joint (the clamped beam) and two flexible bodies in a chain */
TEST(FlexAba, AccelerationsMatchRnca) {
  const std::vector<std::pair<std::string, std::string>> models = {
    {"particles", FlexParticles::Model(true, 0.01)},
    {"damped", FlexParticles::Model(true, 0.01, true)},
    {"welded", FlexParticles::Model(true, 0.01, false, true)},
    {"rotating", RotatingModel("rnca", 0.01)},
    {"clamped_beam", FlexBeamModel(4, "rigid")},
    {"pinned_beam", FlexBeamModel(4, "revolute")},
    {"two_flex", TwoFlexModel(0.01)}};
  for(const auto& [name, model] : models) {
    auto rnca = FlexParticles::Load(model, "aba_vs_rnca_" + name);
    auto aba = FlexParticles::Load(WithSolver(model, "aba"),
                                   "aba_vs_rnca_aba_" + name);
    const std::vector<double> dy_rnca = FlexParticles::Derivatives(*rnca);
    const std::vector<double> dy_aba = FlexParticles::Derivatives(*aba);
    if(name.find("beam") != std::string::npos) {
      /* the beams start at rest, undeformed, without gravity: compare the
      accelerations at a deformed, moving state instead */
      std::vector<double> y(rnca->get_state_count() +
                              rnca->get_auxiliary_state_count(),
                            0.0);
      std::copy(rnca->get_q().begin(), rnca->get_q().end(), y.begin());
      for(unsigned int k = 0; k < rnca->get_dq_count(); k++) {
        y[rnca->get_q_count() + k] = 0.3 * std::cos(1.0 + k);
      }
      auto body = FlexParticles::FlexBody(*rnca);
      ASSERT_NE(body, nullptr);
      for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
        y[body->get_elastic_q_index() + m] = 0.01 * std::sin(2.0 + m);
      }
      std::vector<double> d_rnca(y.size()), d_aba(y.size());
      auto solver = [](shift::core::Context& context) {
        return context["model_parameters"]["solvers"][0]
          .get_data<shift::core::solver::Base>();
      };
      ASSERT_FALSE(solver(*rnca)->Solve(0.0, y, d_rnca));
      ASSERT_FALSE(solver(*aba)->Solve(0.0, y, d_aba));
      ExpectDerivativesNear(d_aba, d_rnca, name);
    } else {
      ExpectDerivativesNear(dy_aba, dy_rnca, name);
    }
    EXPECT_NEAR(aba->KineticEnergy(), rnca->KineticEnergy(),
                1e-12 * std::fmax(1.0, rnca->KineticEnergy()))
      << name;
  }
}


/* a tree of two rotating flexible bodies with nodes: fixed -J1- a -J2- s;
c hangs from the node "tip" of s; w is welded at its node "mid", from which
s2 hangs, and d hangs from the node "tip" of s2. The leaf of s has three
handles (its frame and both node slots), the slot "mid" a mass and a child */
static std::string RotatingTreeModel(const std::string& solver,
                                     double time_span) {
  const std::string frames = R"([{"name": "root", "T": [0.1, 0.05, 0.0],
               "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]},
     {"name": "tip", "T": [1.0, 0.0, 0.0]},
     {"name": "mid", "T": [0.5, 0.2, -0.1]}])";
  std::stringstream ss;
  ss << std::setprecision(17) << std::showpoint;
  ss << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": ")"
     << solver << R"("}],
  "integrators": [{"type": "ode113", "time_span": )"
     << time_span << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "assemblies": [{"name": "tree",
  "bodies": [
   {"name": "a", "geometry": [], "frames": [{"name": "tip", "T": [1, 0, 0]}],
    )" << FlexParticles::Inertia(1.0, {0.1, 0.12, 0.08, 0.0, 0.0, 0.0})
     << R"(},
   {"name": "s", "type": "test_flex_rotating_nodes", "geometry": [],
    "frames": )" << frames << R"(},
   {"name": "c", "geometry": [], )"
     << FlexParticles::Inertia(0.4, {0.02, 0.03, 0.025, 0.001, 0.0, 0.0})
     << R"(},
   {"name": "w", "geometry": [], )"
     << FlexParticles::Inertia(0.3, {0.01, 0.012, 0.009, 0.0, 0.001, 0.0})
     << R"(},
   {"name": "s2", "type": "test_flex_rotating_nodes", "geometry": [],
    "frames": )" << frames << R"(},
   {"name": "d", "geometry": [], )"
     << FlexParticles::Inertia(0.5, {0.03, 0.02, 0.025, 0.0, 0.0, 0.002})
     << R"(}],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 0, 1],
    "initial_velocity": 1.5,
    "parent": [["fixed"], "origin"], "child": [["a"], "origin"]},
   {"name": "J2", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.3, "initial_velocity": -2.0,
    "parent": [["a"], "tip"], "child": [["s"], "root"]},
   {"name": "J3", "type": "revolute", "axis": [0, 1, 0],
    "initial_displacement": 0.2, "initial_velocity": 0.8,
    "parent": [["s"], "tip"], "child": [["c"], "origin"]},
   {"name": "W", "type": "rigid",
    "parent": [["s"], "mid"], "child": [["w"], "origin"]},
   {"name": "J4", "type": "revolute", "axis": [0, 0, 1],
    "initial_displacement": -0.4, "initial_velocity": 1.1,
    "parent": [["s"], "mid"], "child": [["s2"], "root"]},
   {"name": "J5", "type": "revolute", "axis": [1, 0, 0],
    "initial_velocity": -0.7,
    "parent": [["s2"], "tip"], "child": [["d"], "origin"]}]}]}
)";
  return ss.str();
}

/* on the tree of two flexible bodies with nodes, DCA and ABA give RNCA's
state derivatives and kinetic energy, and DCA follows RNCA over a run */
TEST(FlexDca, TreeWithNodesMatchesRnca) {
  auto rnca = FlexParticles::Load(RotatingTreeModel("rnca", 0.2),
                                  "dca_tree_rnca");
  const std::vector<double> dy_rnca = FlexParticles::Derivatives(*rnca);
  for(const std::string solver : {"dca", "aba"}) {
    auto context = FlexParticles::Load(RotatingTreeModel(solver, 0.2),
                                       "dca_tree_" + solver);
    ExpectDerivativesNear(FlexParticles::Derivatives(*context), dy_rnca,
                          solver);
    EXPECT_NEAR(context->KineticEnergy(), rnca->KineticEnergy(),
                1e-12 * rnca->KineticEnergy())
      << solver;
  }
  auto dca = FlexParticles::Load(RotatingTreeModel("dca", 0.2),
                                 "dca_tree_dca_run");
  const std::vector<double> y_rnca = FlexParticles::Run(*rnca);
  const std::vector<double> y_dca = FlexParticles::Run(*dca);
  ASSERT_EQ(y_dca.size(), y_rnca.size());
  for(size_t k = 0; k < y_dca.size(); k++) {
    EXPECT_NEAR(y_dca[k], y_rnca[k], 1e-8 * std::fmax(1.0, std::fabs(y_rnca[k])))
      << k;
  }
}

/* a load at a node with nothing attached there: the node slot has no mass,
and the load still reaches the body (and ABA and DCA, with nothing at the
node slots, still factor M_ee once) */
/* RotatingTreeModel with a spherical joint into the first flexible body
and one out of it at a node, and a float joint from a node of the first to
the second */
static std::string BallTreeModel(const std::string& solver, double time_span,
                                 bool rotations = true) {
  std::string model = RotatingTreeModel(solver, time_span);
  auto swap = [&model](const std::string& from, const std::string& to) {
    const size_t at = model.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if(at != std::string::npos) {
      model.replace(at, from.size(), to);
    }
  };
  if(!rotations) {
    for(int k = 0; k < 2; k++) {
      swap("test_flex_rotating_nodes", "test_flex_translating_nodes");
    }
  }
  swap(R"({"name": "J2", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.3, "initial_velocity": -2.0,)",
       R"({"name": "J2", "type": "spherical",
    "initial_displacement": {"R": [[1, 0, 0], [0, 0.8, -0.6], [0, 0.6, 0.8]]},
    "initial_velocity": [-2.0, 0.6, 0.9],)");
  swap(R"({"name": "J3", "type": "revolute", "axis": [0, 1, 0],
    "initial_displacement": 0.2, "initial_velocity": 0.8,)",
       R"({"name": "J3", "type": "spherical",
    "initial_displacement": {"R": [[0.8, 0, 0.6], [0, 1, 0], [-0.6, 0, 0.8]]},
    "initial_velocity": [0.4, 0.8, -0.5],)");
  swap(R"({"name": "J4", "type": "revolute", "axis": [0, 0, 1],
    "initial_displacement": -0.4, "initial_velocity": 1.1,)",
       R"({"name": "J4", "type": "float",
    "initial_displacement": {"T": [0.05, -0.02, 0.1],
                             "R": [[0.8, -0.6, 0], [0.6, 0.8, 0], [0, 0, 1]]},
    "initial_velocity": [0.3, -0.2, 0.1, 0.5, -0.4, 1.1],)");
  return model;
}

/* spherical joints before and after a flexible body and a float joint
between two: ABA and DCA give RNCA's state derivatives and kinetic energy,
and DCA follows RNCA over a run */
TEST(FlexDca, SphericalAndFloatJointsMatchRnca) {
  auto rnca = FlexParticles::Load(BallTreeModel("rnca", 0.3), "ball_rnca");
  const std::vector<double> dy_rnca = FlexParticles::Derivatives(*rnca);
  for(const std::string solver : {"dca", "aba"}) {
    auto context =
      FlexParticles::Load(BallTreeModel(solver, 0.3), "ball_" + solver);
    ExpectDerivativesNear(FlexParticles::Derivatives(*context), dy_rnca,
                          solver);
    EXPECT_NEAR(context->KineticEnergy(), rnca->KineticEnergy(),
                1e-12 * rnca->KineticEnergy())
      << solver;
  }

  auto dca = FlexParticles::Load(BallTreeModel("dca", 0.3), "ball_dca_run");
  const std::vector<double> y_rnca = FlexParticles::Run(*rnca);
  const std::vector<double> y_dca = FlexParticles::Run(*dca);
  ASSERT_EQ(y_dca.size(), y_rnca.size());
  for(size_t k = 0; k < y_dca.size(); k++) {
    EXPECT_NEAR(y_dca[k], y_rnca[k], 1e-8 * std::fmax(1.0, std::fabs(y_rnca[k])))
      << k;
  }
}

/* the same tree with shapes that only translate the points, so that the
nodes' motion is exact: over a run each solver keeps the energy (kinetic,
gravitational and elastic) */
TEST(FlexDca, SphericalAndFloatJointsKeepEnergy) {
  auto energy = [](shift::core::Context& context) {
    double elastic = 0.0;
    const double* q = context.get_q().data();
    for(int i : context.get_elastic_bodies()) {
      auto body = dynamic_cast<shift::core::body::Flex*>(
        context.get_active_articulated_bodies()[i]);
      elastic += body->ElasticEnergy(q + body->get_elastic_q_index());
    }
    return context.KineticEnergy() + context.GravitationalPotentialEnergy() +
           elastic;
  };
  for(const std::string solver : {"rnca", "aba", "dca"}) {
    auto context = FlexParticles::Load(BallTreeModel(solver, 0.3, false),
                                       "ball_energy_" + solver);
    FlexParticles::Derivatives(*context);
    const double initial = energy(*context);
    const double kinetic = context->KineticEnergy();
    const std::vector<double> y = FlexParticles::Run(*context);
    std::copy(y.begin(), y.begin() + context->get_q_count(),
              context->get_q().begin());
    std::copy(y.begin() + context->get_q_count(), y.end(), context->get_dq());
    auto solver_object = (*context)["model_parameters"]["solvers"][0]
                           .get_data<shift::core::solver::Base>();
    std::vector<double> y_full(y);
    y_full.resize(context->get_state_count() +
                  context->get_auxiliary_state_count());
    std::vector<double> dy(y_full.size(), 0.0);
    ASSERT_FALSE(solver_object->Solve(0.3, y_full, dy)) << solver;
    EXPECT_GT(kinetic, 1.0) << solver;
    EXPECT_NEAR(energy(*context), initial, 1e-7 * kinetic) << solver;
  }
}

static std::string LoadedNodeModel(const std::string& solver) {
  std::string model = RotatingModel(solver, 0.2);
  const std::string body = R"("type": "test_flex_rotating", "geometry": [],
    "frames": [{"name": "root", "T": [0.1, 0.05, 0.0],
               "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}]})";
  const size_t at = model.find(body);
  EXPECT_NE(at, std::string::npos);
  model.replace(at, body.size(),
                R"("type": "test_flex_rotating_nodes", "geometry": [],
    "frames": [{"name": "root", "T": [0.1, 0.05, 0.0],
               "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]},
               {"name": "tip", "T": [1.0, 0.0, 0.0]},
               {"name": "mid", "T": [0.5, 0.2, -0.1]}]})");
  const std::string end = "]}]}";
  const size_t last = model.rfind(end);
  EXPECT_NE(last, std::string::npos);
  model.replace(last, end.size(), R"(],
  "force_modules": [{"type": "applied_load", "name": "l", "body": "s",
                     "frame": "mid", "coordinates": "fixed",
                     "force": [3.0, -2.0, 4.0], "moment": [0.5, 0.2, -0.3]}]}]})");
  return model;
}

TEST(FlexDca, LoadAtAnEmptyNodeMatchesRnca) {
  auto rnca = FlexParticles::Load(LoadedNodeModel("rnca"), "dca_load_rnca");
  auto unloaded = FlexParticles::Load(RotatingModel("rnca", 0.2),
                                      "dca_load_none");
  const std::vector<double> dy_rnca = FlexParticles::Derivatives(*rnca);
  const std::vector<double> dy_none = FlexParticles::Derivatives(*unloaded);
  double change = 0.0;
  for(size_t k = 0; k < dy_rnca.size(); k++) {
    change = std::max(change, std::fabs(dy_rnca[k] - dy_none[k]));
  }
  EXPECT_GT(change, 0.1); // the load acts
  for(const std::string solver : {"dca", "aba"}) {
    auto context = FlexParticles::Load(LoadedNodeModel(solver),
                                       "dca_load_" + solver);
    ExpectDerivativesNear(FlexParticles::Derivatives(*context), dy_rnca,
                          solver);
  }
}

/* a flexible body without node slots: ABA and DCA factor its constant M_ee
once, and DCA solves it by the Schur complement on its frame */
TEST(FlexDca, BodyWithoutNodesMatchesRnca) {
  auto rnca = FlexParticles::Load(RotatingModel("rnca", 0.2),
                                  "dca_frame_rnca");
  const std::vector<double> dy_rnca = FlexParticles::Derivatives(*rnca);
  for(const std::string solver : {"dca", "aba"}) {
    auto context = FlexParticles::Load(RotatingModel(solver, 0.2),
                                       "dca_frame_" + solver);
    ExpectDerivativesNear(FlexParticles::Derivatives(*context), dy_rnca,
                          solver);
  }
}

/* the particle body's shapes move its points in every direction, which
repeats the rigid motion of its frame: free of its joint, its mass matrix
is singular, and DCA reports it rather than solving it */
TEST(FlexDca, RefusesShapesThatRepeatTheFrameMotion) {
  auto context = FlexParticles::Load(
    WithSolver(FlexParticles::Model(true, 0.01), "dca"), "dca_particles");
  context->get_active_integrators()[0]->Initialize();
  auto solver = (*context)["model_parameters"]["solvers"][0]
                  .get_data<shift::core::solver::Base>();
  solver->Initialize();
  std::vector<double> y(context->get_state_count() +
                          context->get_auxiliary_state_count(),
                        0.0);
  std::copy(context->get_q().begin(), context->get_q().end(), y.begin());
  std::vector<double> dy(y.size(), 0.0);
  EXPECT_TRUE(solver->Solve(0.0, y, dy));
}

/* under ABA the particle body still moves as its rigid twin */
TEST(FlexAba, TrajectoryMatchesRigidModel) {
  for(bool damped : {false, true}) {
    auto flexible = FlexParticles::Load(
      WithSolver(FlexParticles::Model(true, 1.0, damped), "aba"),
      "aba_particles_run");
    auto rigid = FlexParticles::Load(
      WithSolver(FlexParticles::Model(false, 1.0, damped), "aba"),
      "aba_particles_run_rigid");
    const std::vector<double> y_flexible = FlexParticles::Run(*flexible);
    const std::vector<double> y_rigid = FlexParticles::Run(*rigid);
    SCOPED_TRACE(damped ? "damped" : "undamped");
    FlexParticles::Compare(*flexible, y_flexible, *rigid, y_rigid, false,
                           1e-7);
    FlexParticles::Compare(*flexible, y_flexible, *rigid, y_rigid, true, 1e-6);
  }
}

/* the energy of the rotating shapes is conserved under ABA */
TEST(FlexAba, EnergyIsConservedWithRotatingShapes) {
  auto context =
    FlexParticles::Load(RotatingModel("aba", 1.0), "aba_rotating_energy");
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  auto energy = [&]() {
    const double* q = context->get_q().data();
    return context->KineticEnergy() + context->GravitationalPotentialEnergy() +
           body->ElasticEnergy(q + body->get_elastic_q_index());
  };
  FlexParticles::Derivatives(*context);
  const double initial = energy();
  const double kinetic = context->KineticEnergy();
  const std::vector<double> y = FlexParticles::Run(*context);
  std::copy(y.begin(), y.begin() + context->get_q_count(),
            context->get_q().begin());
  std::copy(y.begin() + context->get_q_count(), y.end(), context->get_dq());
  auto solver = (*context)["model_parameters"]["solvers"][0]
                  .get_data<shift::core::solver::Base>();
  std::vector<double> y_full(y);
  y_full.resize(context->get_state_count() +
                context->get_auxiliary_state_count());
  std::vector<double> dy(y_full.size(), 0.0);
  ASSERT_FALSE(solver->Solve(1.0, y_full, dy));
  EXPECT_GT(kinetic, 0.5);
  EXPECT_NEAR(energy(), initial, 1e-7 * kinetic);
}

/* the clamped cantilever has its frequencies under ABA too */
TEST(FlexAba, ClampedBeamFrequencies) {
  const std::vector<double> omega = Frequencies(
    WithSolver(FlexBeamModel(4, "rigid"), "aba"), "aba_beam_clamped");
  ASSERT_EQ(omega.size(), 4u);
  for(int n = 0; n < 4; n++) {
    const double exact = TestFlexBeam::Omega(TestFlexBeam::kClamped[n]);
    EXPECT_NEAR(omega[n], exact, 1e-4 * exact) << n;
  }
}

/* the spanning tree is grown from the fixed body breadth first: C reaches
c2 before D, so D is cut and its chord is at the node "tip". \p deep_pivot
hangs C from the fixed body through two rigid joints and massless bodies g1
and g2, so that D reaches c2 first and C is cut: the loop then leaves s by
D, a loop member hanging from the node. \p deformed starts the coupler
deformed and moving (test_flex_particles), the loop open */
static std::string FourBarModel(bool flexible, double time_span,
                                bool deep_pivot = false,
                                bool deformed = false,
                                bool spherical = false) {
  /* spherical B and D make it a spatial four-bar, the coupler free to spin
  about the line through them */
  const std::string coupler_joint =
    spherical ? R"("type": "spherical")"
              : R"("type": "revolute", "axis": [0, 1, 0])";
  const double theta = 0.5;
  const double length = 0.6;
  std::stringstream end;
  end << std::setprecision(17) << "[" << length * std::sin(theta) << ", 0, "
      << length * std::cos(theta) << "]";
  std::stringstream ss;
  ss << std::setprecision(17) << std::showpoint;
  ss << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
     << time_span << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "frames": [{"name": "pivot2", "T": [1, 0, 0]}],
 "assemblies": [{"name": "fourbar",
  "bodies": [
   {"name": "c1", "geometry": [], "frames": [{"name": "end", "T": )"
     << end.str() << R"(}],
    )" << FlexParticles::Inertia(1.0, {0.04, 0.05, 0.02, 0.0, 0.0, 0.0},
                                 "[0.15, 0.0, 0.25]")
     << R"(},
   {"name": "c2", "geometry": [], "frames": [{"name": "end", "T": )"
     << end.str() << R"(}],
    )" << FlexParticles::Inertia(0.8, {0.03, 0.04, 0.015, 0.0, 0.0, 0.0},
                                 "[0.1, 0.0, 0.3]")
     << "},\n";
  if(deep_pivot) {
    for(const char* g : {"g1", "g2"}) {
      ss << R"(   {"name": ")" << g << R"(", "geometry": [], )"
         << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "},\n";
    }
  }
  if(flexible) {
    ss << R"(   {"name": "s", "type": )"
       << (deformed ? R"("test_flex_particles")"
                    : R"("test_flex_particles_at_rest")")
       << R"(,
    "geometry": [], "frames": [{"name": "tip", "T": [1.0, 0.0, 0.0]}]})";
  } else {
    ss << R"(   {"name": "s", "geometry": [], "frames": [)";
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      const ksl_vec3_t t = FlexParticles::Position(p);
      ss << (p ? ", " : "") << R"({"name": "a)" << p << R"(", "T": [)" << t.x
         << ", " << t.y << ", " << t.z << "]}";
    }
    ss << "], " << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      ss << ",\n   {\"name\": \"x" << p << "\", \"geometry\": [], "
         << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
      ss << ",\n   {\"name\": \"y" << p << "\", \"geometry\": [], "
         << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
      ss << ",\n   {\"name\": \"m" << p << "\", \"geometry\": [], "
         << FlexParticles::Inertia(FlexParticles::Mass(p),
                                   FlexParticles::Rotary(p))
         << "}";
    }
  }
  ss << R"(],
  "joints": [
   {"name": "A", "type": "revolute", "axis": [0, 1, 0],
    "parent": [["fixed"], "origin"], "child": [["c1"], "origin"]},
   {"name": "B", )" << coupler_joint << R"(,
    "parent": [["c1"], "end"], "child": [["s"], "origin"]},
   {"name": "C", "type": "revolute", "axis": [0, 1, 0],
    "parent": )" << (deep_pivot ? R"([["g2"], "origin"])"
                                : R"([["fixed"], "pivot2"])")
     << R"(, "child": [["c2"], "origin"]},)";
  if(deep_pivot) {
    ss << R"(
   {"name": "E", "type": "rigid",
    "parent": [["fixed"], "pivot2"], "child": [["g1"], "origin"]},
   {"name": "F", "type": "rigid",
    "parent": [["g1"], "origin"], "child": [["g2"], "origin"]},)";
  }
  ss << R"(
   {"name": "D", )" << coupler_joint << R"(,
    "parent": )"
     << (flexible ? R"([["s"], "tip"])" : R"([["m0"], "origin"])")
     << R"(, "child": [["c2"], "end"]})";
  if(!flexible) {
    const char* axes[3] = {"[1, 0, 0]", "[0, 1, 0]", "[0, 0, 1]"};
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      const std::string n = std::to_string(p);
      const std::string parents[3] = {R"([["s"], "a)" + n + R"("])",
                                      R"([["x)" + n + R"("], "origin"])",
                                      R"([["y)" + n + R"("], "origin"])"};
      const std::string children[3] = {"x" + n, "y" + n, "m" + n};
      for(int a = 0; a < 3; a++) {
        ss << ",\n   {\"name\": \"P" << p << a
           << "\", \"type\": \"prismatic\", \"axis\": " << axes[a]
           << ", \"stiffness_coefficients\": [0.0, "
           << FlexParticles::Stiffness(p, a) << "], \"parent\": " << parents[a]
           << ", \"child\": [[\"" << children[a] << "\"], \"origin\"]}";
      }
    }
  }
  ss << "]}]}\n";
  return ss.str();
}

/* the joints \p names (A to D of the four-bars) and the point displacements
of a flexible model and its rigid twin */
static void CompareFourBars(
  shift::core::Context& flexible, const std::vector<double>& y_flexible,
  shift::core::Context& rigid, const std::vector<double>& y_rigid, bool rates,
  double tolerance,
  const std::vector<std::string>& names = {"A", "B", "C", "D"}) {
  const int f_offset = rates ? (int)flexible.get_q_count() : 0;
  const int r_offset = rates ? (int)rigid.get_q_count() : 0;
  for(const std::string& name : names) {
    const int f = rates ? FlexParticles::DqIndex(flexible, name)
                        : FlexParticles::QIndex(flexible, name);
    const int r = rates ? FlexParticles::DqIndex(rigid, name)
                        : FlexParticles::QIndex(rigid, name);
    EXPECT_NEAR(y_flexible[f_offset + f], y_rigid[r_offset + r], tolerance)
      << name << (rates ? " rate" : "");
  }
  auto body = FlexParticles::FlexBody(flexible);
  ASSERT_NE(body, nullptr);
  const int e =
    rates ? body->get_elastic_dq_index() : body->get_elastic_q_index();
  for(int p = 0; p < FlexParticles::kPoints; p++) {
    for(int a = 0; a < 3; a++) {
      const std::string name = "P" + std::to_string(p) + std::to_string(a);
      const int r = rates ? FlexParticles::DqIndex(rigid, name)
                          : FlexParticles::QIndex(rigid, name);
      EXPECT_NEAR(FlexParticles::U(&y_flexible[f_offset + e], 3 * p + a),
                  y_rigid[r_offset + r], tolerance)
        << name << (rates ? " rate" : "");
    }
  }
}

/* the loop through the flexible coupler: the chord Jacobian has columns for
its elastic coordinates, and the factorization takes some as dependent */
TEST(FlexFourBar, ElasticCoordinatesJoinTheLoop) {
  auto context = FlexParticles::Load(FourBarModel(true, 0.01), "fourbar_flex");
  FlexParticles::Derivatives(*context);
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  const auto& jacobian = context->get_jacobian();
  ASSERT_EQ(jacobian.get_substructures().size(), 1u);
  const auto& substructure = jacobian.get_substructures()[0];
  /* 4 joint and 6 elastic coordinates; the loop moves in the plane and the
  node also along y, through the elastic coordinates alone */
  EXPECT_EQ(substructure.get_dq_count(), 10u);
  EXPECT_EQ(substructure.get_rank(), 4);
  const auto& dependent = jacobian.get_is_dependent_po1();
  int elastic_dependent = 0;
  for(int m = 0; m < FlexParticles::kCount; m++) {
    elastic_dependent += dependent[body->get_elastic_dq_index() + m];
  }
  EXPECT_GE(elastic_dependent, 1);
}

TEST(FlexFourBar, AccelerationsMatchRigidModel) {
  auto flexible =
    FlexParticles::Load(FourBarModel(true, 0.01), "fourbar_flex_a");
  auto rigid = FlexParticles::Load(FourBarModel(false, 0.01), "fourbar_rigid_a");
  const std::vector<double> dy_flexible = FlexParticles::Derivatives(*flexible);
  const std::vector<double> dy_rigid = FlexParticles::Derivatives(*rigid);
  /* the accelerations sit where the derivatives of the velocities are */
  std::vector<double> a_flexible(dy_flexible.begin(), dy_flexible.end());
  std::vector<double> a_rigid(dy_rigid.begin(), dy_rigid.end());
  CompareFourBars(*flexible, a_flexible, *rigid, a_rigid, true, 1e-9);
  /* it moves: crank A accelerates */
  EXPECT_GT(std::fabs(a_rigid[rigid->get_q_count() +
                              FlexParticles::DqIndex(*rigid, "A")]),
            0.1);
}

/* C cut: the loop leaves the coupler by D, which hangs from the node */
TEST(FlexFourBar, LoopLeavingByANodeMatchesRigidModel) {
  auto flexible =
    FlexParticles::Load(FourBarModel(true, 0.5, true), "fourbar_flex_deep");
  auto rigid =
    FlexParticles::Load(FourBarModel(false, 0.5, true), "fourbar_rigid_deep");
  CompareFourBars(*flexible, FlexParticles::Derivatives(*flexible), *rigid,
                  FlexParticles::Derivatives(*rigid), true, 1e-9);
  /* C is the cut joint, D a member of the loop */
  const auto& chord = flexible->get_chord_constraints();
  ASSERT_EQ(chord.size(), 1u);
  EXPECT_EQ(chord[0]->get_child_body()->get_name(), "c2");
  const std::vector<double> y_flexible = FlexParticles::Run(*flexible);
  const std::vector<double> y_rigid = FlexParticles::Run(*rigid);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, false, 1e-7);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, true, 1e-6);
}

/* the run-time path (the Jacobian set up in the order of the last full
factorization, factored without pivoting) gives the full factorization's
result, the elastic columns included */
TEST(FlexFourBar, RunTimeJacobianMatchesFullFactorization) {
  for(bool deep_pivot : {false, true}) {
    auto context = FlexParticles::Load(
      FourBarModel(true, 0.01, deep_pivot, true),
      "fourbar_flex_po2_" + std::to_string(deep_pivot));
    FlexParticles::Derivatives(*context);
    auto& jacobian = context->get_jacobian();
    const auto& substructure = jacobian.get_substructures()[0];
    const int columns = (int)substructure.get_dq_count();
    jacobian.SetupPO1();
    const int rank = substructure.get_rank();
    const double* J = substructure.get_jacobian_matrix();
    const std::vector<double> full(J, J + rank * columns);
    jacobian.Analyze();
    ASSERT_EQ(substructure.get_rank(), rank);
    for(int k = 0; k < rank * columns; k++) {
      EXPECT_NEAR(J[k], full[k], 1e-12) << deep_pivot << " " << k;
    }
  }
}

/* started deformed, the loop is open: Newton-Raphson closes it, moving the
dependent coordinates, elastic ones among them */
TEST(FlexFourBar, NewtonRaphsonClosesTheLoopThroughElasticCoordinates) {
  auto context = FlexParticles::Load(FourBarModel(true, 0.01, false, true),
                                     "fourbar_flex_nr");
  FlexParticles::Derivatives(*context);
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  auto& jacobian = context->get_jacobian();
  EXPECT_LT(jacobian.EvaluateConstraints(), 1e-12);
  const auto& dependent = jacobian.get_is_dependent_po1();
  const int e = body->get_elastic_q_index();
  const int e_dq = body->get_elastic_dq_index();
  int moved = 0;
  for(int m = 0; m < FlexParticles::kCount; m++) {
    const double change = std::fabs(context->get_q()[e + m] -
                                    FlexParticles::kEta[m]);
    if(dependent[e_dq + m]) {
      moved += change > 1e-6;
    } else {
      EXPECT_EQ(change, 0.0) << m;
    }
  }
  EXPECT_GE(moved, 1);
}

/* the loop's position error after a step of the coordinates by h times
their rates (q per velocity, ConvertdqTody) from the context's state */
static double ConstraintErrorAfterStep(shift::core::Context& context,
                                       double h) {
  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();
  const std::vector<double> q(context.get_q());
  std::vector<double> y(q_count + dq_count), rates(q_count + dq_count);
  std::copy(q.begin(), q.end(), y.begin());
  std::copy(context.get_dq(), context.get_dq() + dq_count,
            y.begin() + q_count);
  context.ConvertdqTody(y, rates);
  for(unsigned int k = 0; k < q_count; k++) {
    context.get_q()[k] = q[k] + h * rates[k];
  }
  context.UpdateDH();
  const double error = context.get_jacobian().EvaluateConstraints();
  std::copy(q.begin(), q.end(), context.get_q().begin());
  context.UpdateDH();
  return error;
}

/* the deformed coupler starts with elastic rates that open the loop: at load
the rates are made consistent with the loop (MakeVelocitiesConsistent), with
the elastic coordinates among the substructure's columns, so that the loop
stays closed to first order in a step of the coordinates along them */
TEST(FlexFourBar, InitialVelocitiesAreMadeConsistent) {
  for(bool deep_pivot : {false, true}) {
    auto context = FlexParticles::Load(
      FourBarModel(true, 0.01, deep_pivot, true),
      deep_pivot ? "fourbar_flex_v_deep" : "fourbar_flex_v");
    FlexParticles::Derivatives(*context);
    auto body = FlexParticles::FlexBody(*context);
    ASSERT_NE(body, nullptr);
    /* the coupler still moves */
    double elastic_rate = 0.0;
    for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
      elastic_rate =
        std::max(elastic_rate,
                 std::fabs(context->get_dq()[body->get_elastic_dq_index() + m]));
    }
    EXPECT_GT(elastic_rate, 1e-3) << deep_pivot;
    for(const auto& substructure : context->get_jacobian().get_substructures()) {
      EXPECT_LT(substructure.VelocityResidual(context->get_dq()), 1e-10)
        << deep_pivot;
    }
    /* O(h^2): the error over h falls tenfold with h */
    const double coarse = ConstraintErrorAfterStep(*context, 1e-3) / 1e-3;
    const double fine = ConstraintErrorAfterStep(*context, 1e-4) / 1e-4;
    EXPECT_LT(fine, 1e-3) << deep_pivot;
    EXPECT_LT(fine, 0.2 * coarse + 1e-12) << deep_pivot;
  }
}

/* a loop both of whose sides hang from nodes of the flexible body s, which
swings from the fixed body on A: link c1 hangs from the node "tip" (point 0)
by B, c2 from the node "mid" (point 1) by C, and c3 joins them by E and D,
all about y. The loop closes at zero joint angles. Its ancestor is s, whose
joint A is not a member: s's elastic coordinates move both ends of the loop,
by the difference of the two nodes' influence coefficients, and join the
substructure below its members. On s as rigid, B, E and D would be a locked
triangle: the loop moves by the deformation of s (in the rigid twin, the
points on their prismatic joints). \p deformed starts s deformed and moving,
the loop open; \p b_rate is the initial rate of B */
static std::string BridgeModel(bool flexible, double time_span,
                               bool deformed = false, double b_rate = 0.0) {
  std::stringstream ss;
  ss << std::setprecision(17) << std::showpoint;
  ss << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
     << time_span << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "assemblies": [{"name": "bridge",
  "bodies": [
   {"name": "c1", "geometry": [], "frames": [{"name": "end", "T": [0, 0, 0.6]}],
    )" << FlexParticles::Inertia(0.6, {0.02, 0.025, 0.01, 0.0, 0.0, 0.0},
                                 "[0.0, 0.0, 0.3]")
     << R"(},
   {"name": "c2", "geometry": [], "frames": [{"name": "end", "T": [0, 0, 0.8]}],
    )" << FlexParticles::Inertia(0.5, {0.03, 0.035, 0.01, 0.0, 0.0, 0.0},
                                 "[0.0, 0.0, 0.4]")
     << R"(},
   {"name": "c3", "geometry": [],
    "frames": [{"name": "end", "T": [-0.6, 0.3, 0.0]}],
    )" << FlexParticles::Inertia(0.4, {0.01, 0.015, 0.012, 0.0, 0.0, 0.0},
                                 "[-0.3, 0.15, 0.0]")
     << "},\n";
  if(flexible) {
    ss << R"(   {"name": "s", "type": )"
       << (deformed ? R"("test_flex_particles_two_nodes")"
                    : R"("test_flex_particles_two_nodes_at_rest")")
       << R"(, "geometry": [], "frames": [{"name": "tip", "T": [1.0, 0.0, 0.0]},
                {"name": "mid", "T": [0.4, 0.3, -0.2]}]})";
  } else {
    ss << R"(   {"name": "s", "geometry": [], "frames": [)";
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      const ksl_vec3_t t = FlexParticles::Position(p);
      ss << (p ? ", " : "") << R"({"name": "a)" << p << R"(", "T": [)" << t.x
         << ", " << t.y << ", " << t.z << "]}";
    }
    ss << "], " << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      ss << ",\n   {\"name\": \"x" << p << "\", \"geometry\": [], "
         << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
      ss << ",\n   {\"name\": \"y" << p << "\", \"geometry\": [], "
         << FlexParticles::Inertia(0.0, {0, 0, 0, 0, 0, 0}) << "}";
      ss << ",\n   {\"name\": \"m" << p << "\", \"geometry\": [], "
         << FlexParticles::Inertia(FlexParticles::Mass(p),
                                   FlexParticles::Rotary(p))
         << "}";
    }
  }
  ss << R"(],
  "joints": [
   {"name": "A", "type": "revolute", "axis": [0, 1, 0],
    "parent": [["fixed"], "origin"], "child": [["s"], "origin"]},
   {"name": "B", "type": "revolute", "axis": [0, 1, 0],
    "initial_velocity": )" << b_rate << R"(,
    "parent": )" << (flexible ? R"([["s"], "tip"])" : R"([["m0"], "origin"])")
     << R"(, "child": [["c1"], "origin"]},
   {"name": "C", "type": "revolute", "axis": [0, 1, 0],
    "parent": )" << (flexible ? R"([["s"], "mid"])" : R"([["m1"], "origin"])")
     << R"(, "child": [["c2"], "origin"]},
   {"name": "E", "type": "revolute", "axis": [0, 1, 0],
    "parent": [["c1"], "end"], "child": [["c3"], "origin"]},
   {"name": "D", "type": "revolute", "axis": [0, 1, 0],
    "parent": [["c3"], "end"], "child": [["c2"], "end"]})";
  if(!flexible) {
    const char* axes[3] = {"[1, 0, 0]", "[0, 1, 0]", "[0, 0, 1]"};
    for(int p = 0; p < FlexParticles::kPoints; p++) {
      const std::string n = std::to_string(p);
      const std::string parents[3] = {R"([["s"], "a)" + n + R"("])",
                                      R"([["x)" + n + R"("], "origin"])",
                                      R"([["y)" + n + R"("], "origin"])"};
      const std::string children[3] = {"x" + n, "y" + n, "m" + n};
      for(int a = 0; a < 3; a++) {
        ss << ",\n   {\"name\": \"P" << p << a
           << "\", \"type\": \"prismatic\", \"axis\": " << axes[a]
           << ", \"stiffness_coefficients\": [0.0, "
           << FlexParticles::Stiffness(p, a) << "], \"parent\": " << parents[a]
           << ", \"child\": [[\"" << children[a] << "\"], \"origin\"]}";
      }
    }
  }
  ss << "]}]}\n";
  return ss.str();
}

static const std::vector<std::string> kBridgeJoints = {"A", "B", "C", "D",
                                                       "E"};

/* the substructure holds the four loop joints and the six elastic
coordinates of s below them (A is not a member), and the factorization takes
elastic coordinates as dependent */
TEST(FlexBridge, ElasticCoordinatesOfTheAncestorJoinTheLoop) {
  auto context = FlexParticles::Load(BridgeModel(true, 0.01), "bridge_flex");
  FlexParticles::Derivatives(*context);
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  const auto& jacobian = context->get_jacobian();
  ASSERT_EQ(jacobian.get_substructures().size(), 1u);
  const auto& substructure = jacobian.get_substructures()[0];
  EXPECT_EQ(substructure.get_head_elastic_count(), 6u);
  EXPECT_EQ(substructure.get_dq_count(), 10u);
  EXPECT_EQ((int)substructure.get_dq_offset(), body->get_elastic_dq_index());
  const auto& dependent = jacobian.get_is_dependent_po1();
  EXPECT_FALSE(dependent[FlexParticles::DqIndex(*context, "A")]);
  int elastic_dependent = 0;
  for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
    elastic_dependent += dependent[body->get_elastic_dq_index() + m];
  }
  EXPECT_GE(elastic_dependent, 1);
}

TEST(FlexBridge, AccelerationsMatchRigidModel) {
  auto flexible = FlexParticles::Load(BridgeModel(true, 0.01), "bridge_flex_a");
  auto rigid = FlexParticles::Load(BridgeModel(false, 0.01), "bridge_rigid_a");
  const std::vector<double> a_flexible = FlexParticles::Derivatives(*flexible);
  const std::vector<double> a_rigid = FlexParticles::Derivatives(*rigid);
  CompareFourBars(*flexible, a_flexible, *rigid, a_rigid, true, 1e-9,
                  kBridgeJoints);
  /* s has no mass of its own and its springs are at rest: at the start the
  points and the links hanging from them fall freely, s is not loaded and
  the joints do not accelerate yet */
  EXPECT_NEAR(a_rigid[rigid->get_q_count() + FlexParticles::DqIndex(*rigid,
                                                                    "P02")],
              9.81, 1e-9);
}

TEST(FlexBridge, TrajectoryMatchesRigidModel) {
  auto flexible = FlexParticles::Load(BridgeModel(true, 0.5), "bridge_flex_t");
  auto rigid = FlexParticles::Load(BridgeModel(false, 0.5), "bridge_rigid_t");
  const std::vector<double> y_flexible = FlexParticles::Run(*flexible);
  const std::vector<double> y_rigid = FlexParticles::Run(*rigid);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, false, 1e-7,
                  kBridgeJoints);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, true, 1e-6,
                  kBridgeJoints);
  /* once the springs take the load, s swings and the loop bends */
  EXPECT_GT(std::fabs(y_rigid[FlexParticles::QIndex(*rigid, "A")]), 1e-2);
  EXPECT_GT(std::fabs(y_rigid[FlexParticles::QIndex(*rigid, "B")]), 1e-3);
}

/* the run-time Jacobian, with the two nodes' columns of each elastic
coordinate summed in place, gives the full factorization's */
TEST(FlexBridge, RunTimeJacobianMatchesFullFactorization) {
  auto context =
    FlexParticles::Load(BridgeModel(true, 0.01, true), "bridge_flex_po2");
  FlexParticles::Derivatives(*context);
  auto& jacobian = context->get_jacobian();
  const auto& substructure = jacobian.get_substructures()[0];
  const int columns = (int)substructure.get_dq_count();
  jacobian.SetupPO1();
  const int rank = substructure.get_rank();
  const double* J = substructure.get_jacobian_matrix();
  const std::vector<double> full(J, J + rank * columns);
  jacobian.Analyze();
  ASSERT_EQ(substructure.get_rank(), rank);
  for(int k = 0; k < rank * columns; k++) {
    EXPECT_NEAR(J[k], full[k], 1e-12) << k;
  }
}

/* started deformed, the loop is open: Newton-Raphson closes it through
dependent coordinates, the ancestor's elastic coordinates among them, and
the initial rates are made consistent */
TEST(FlexBridge, LoopClosedAtLoad) {
  auto context =
    FlexParticles::Load(BridgeModel(true, 0.01, true), "bridge_flex_nr");
  FlexParticles::Derivatives(*context);
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  auto& jacobian = context->get_jacobian();
  EXPECT_LT(jacobian.EvaluateConstraints(), 1e-12);
  const auto& dependent = jacobian.get_is_dependent_po1();
  const int e = body->get_elastic_q_index();
  const int e_dq = body->get_elastic_dq_index();
  int moved = 0;
  for(int m = 0; m < FlexParticles::kCount; m++) {
    const double change =
      std::fabs(context->get_q()[e + m] - FlexParticles::kEta[m]);
    if(dependent[e_dq + m]) {
      moved += change > 1e-6;
    } else {
      EXPECT_EQ(change, 0.0) << m;
    }
  }
  EXPECT_GE(moved, 1);
  for(const auto& substructure : jacobian.get_substructures()) {
    EXPECT_LT(substructure.VelocityResidual(context->get_dq()), 1e-10);
  }
  const double coarse = ConstraintErrorAfterStep(*context, 1e-3) / 1e-3;
  const double fine = ConstraintErrorAfterStep(*context, 1e-4) / 1e-4;
  EXPECT_LT(fine, 1e-3);
  EXPECT_LT(fine, 0.2 * coarse + 1e-12);
}

/* a rate given to B alone: B can be independent, so the rate is kept and
the dependent rates, the ancestor's elastic ones among them, follow from it
(the factorization with B specified independent) */
TEST(FlexBridge, GivenRateKeptAtLoad) {
  auto context = FlexParticles::Load(BridgeModel(true, 0.01, false, 0.7),
                                     "bridge_flex_rate");
  FlexParticles::Derivatives(*context);
  auto body = FlexParticles::FlexBody(*context);
  ASSERT_NE(body, nullptr);
  EXPECT_DOUBLE_EQ(context->get_dq()[FlexParticles::DqIndex(*context, "B")],
                   0.7);
  double elastic_rate = 0.0;
  for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
    elastic_rate = std::max(
      elastic_rate,
      std::fabs(context->get_dq()[body->get_elastic_dq_index() + m]));
  }
  EXPECT_GT(elastic_rate, 1e-3);
  for(const auto& substructure : context->get_jacobian().get_substructures()) {
    EXPECT_LT(substructure.VelocityResidual(context->get_dq()), 1e-10);
  }
  const double coarse = ConstraintErrorAfterStep(*context, 1e-3) / 1e-3;
  const double fine = ConstraintErrorAfterStep(*context, 1e-4) / 1e-4;
  EXPECT_LT(fine, 1e-3);
  EXPECT_LT(fine, 0.2 * coarse + 1e-12);
}

/* the spatial four-bar (spherical B and D, at the coupler's frame and at
its node) against its rigid twin: accelerations, and a run with every joint
coordinate, quaternions included; the coupler's off-plane point makes it
spin about the line through B and D */
TEST(FlexFourBar, SphericalJointsMatchRigidModel) {
  auto flexible = FlexParticles::Load(FourBarModel(true, 0.01, false, false, true),
                                      "fourbar_flex_ball_a");
  auto rigid = FlexParticles::Load(FourBarModel(false, 0.01, false, false, true),
                                   "fourbar_rigid_ball_a");
  const std::vector<double> dy_flexible = FlexParticles::Derivatives(*flexible);
  const std::vector<double> dy_rigid = FlexParticles::Derivatives(*rigid);
  CompareFourBars(*flexible, dy_flexible, *rigid, dy_rigid, true, 1e-9,
                  {"A", "C"});
  EXPECT_GT(std::fabs(dy_rigid[rigid->get_q_count() +
                               FlexParticles::DqIndex(*rigid, "A")]),
            0.1);

  flexible = FlexParticles::Load(FourBarModel(true, 0.5, false, false, true),
                                 "fourbar_flex_ball_t");
  rigid = FlexParticles::Load(FourBarModel(false, 0.5, false, false, true),
                              "fourbar_rigid_ball_t");
  const std::vector<double> y_flexible = FlexParticles::Run(*flexible);
  const std::vector<double> y_rigid = FlexParticles::Run(*rigid);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, false, 1e-7,
                  {"A", "C"});
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, true, 1e-6,
                  {"A", "C"});
  for(const std::string name : {"B", "D"}) {
    const int f = FlexParticles::QIndex(*flexible, name);
    const int r = FlexParticles::QIndex(*rigid, name);
    for(int k = 0; k < 4; k++) {
      EXPECT_NEAR(y_flexible[f + k], y_rigid[r + k], 1e-7) << name << " " << k;
    }
    const int fv = flexible->get_q_count() + FlexParticles::DqIndex(*flexible, name);
    const int rv = rigid->get_q_count() + FlexParticles::DqIndex(*rigid, name);
    for(int k = 0; k < 3; k++) {
      EXPECT_NEAR(y_flexible[fv + k], y_rigid[rv + k], 1e-6)
        << name << " rate " << k;
    }
  }
  const int b = rigid->get_q_count() + FlexParticles::DqIndex(*rigid, "B");
  EXPECT_GT(std::fabs(y_rigid[b]) + std::fabs(y_rigid[b + 2]), 0.1);
}

TEST(FlexFourBar, TrajectoryMatchesRigidModel) {
  auto flexible =
    FlexParticles::Load(FourBarModel(true, 1.0), "fourbar_flex_t");
  auto rigid = FlexParticles::Load(FourBarModel(false, 1.0), "fourbar_rigid_t");
  const std::vector<double> y_flexible = FlexParticles::Run(*flexible);
  const std::vector<double> y_rigid = FlexParticles::Run(*rigid);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, false, 1e-7);
  CompareFourBars(*flexible, y_flexible, *rigid, y_rigid, true, 1e-6);
  /* the coupler has deformed */
  auto body = FlexParticles::FlexBody(*flexible);
  double largest = 0.0;
  for(int row = 0; row < 3 * FlexParticles::kPoints; row++) {
    largest = std::max(
      largest, std::fabs(FlexParticles::U(&y_flexible[body->get_elastic_q_index()],
                                          row)));
  }
  EXPECT_GT(largest, 1e-3);
}
