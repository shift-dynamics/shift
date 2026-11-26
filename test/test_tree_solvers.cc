/* The recursive tree solvers (aba, dca) against the RNCA solver: the forest
of a model is classified into serial chains, branched trees and closed loops,
and at the same state every solver gives the same joint accelerations. */

#include <omp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/integrator/base.h"
#include "shift/core/solver/auto.h"
#include "shift/core/solver/base.h"
#include "shift/core/solver/dca.h"
#include "shift/core/tangents.h"

class TreeSolvers {
public:
  /* a body with an off-centre mass centre and products of inertia, so that
  the spatial inertia is fully populated */
  static std::string Body(const std::string& name, double scale) {
    const double m = 1.0 + scale;
    return R"({"name": ")" + name + R"(", "geometry": [],
      "center_of_mass": [)" +
           std::to_string(0.1 * scale) + ", " + std::to_string(-0.05) + ", " +
           std::to_string(0.2) + R"(],
      "frames": [{"name": "tip", "T": [0.4, 0.1, 0.3]},
                 {"name": "side", "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                  "T": [0.0, 0.3, -0.1]}],
      "inertia": {"mass": )" +
           std::to_string(m) + R"(, "Ixx": )" + std::to_string(0.2 * m) +
           R"(, "Iyy": )" + std::to_string(0.15 * m) + R"(, "Izz": )" +
           std::to_string(0.1 * m) + R"(, "Ixy": 0.01, "Iyz": -0.02,
                  "Izx": 0.015}})";
  }

  static std::string Joint(const std::string& name, const std::string& type,
                           const std::string& parent,
                           const std::string& parent_frame,
                           const std::string& child,
                           const std::string& extra = "") {
    return R"({"name": ")" + name + R"(", "type": ")" + type + R"(", )" +
           extra + R"("parent": [[")" + parent + R"("], ")" + parent_frame +
           R"("], "child": [[")" + child + R"("], "origin"]})";
  }

  /* a solver object: a type name, or a JSON object as is */
  static std::string Solver(const std::string& solver) {
    return solver[0] == '{' ? solver : R"({"type": ")" + solver + R"("})";
  }

  static std::string Model(const std::string& solver,
                           const std::vector<std::string>& bodies,
                           const std::vector<std::string>& joints,
                           double time_span = 0.3) {
    std::string model = R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [)" +
                        Solver(solver) + R"(],
  "integrators": [{"type": "ode113", "time_span": )" +
                        std::to_string(time_span) +
                        R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "assemblies": [{"name": "a", "bodies": [)";
    for(size_t i = 0; i < bodies.size(); i++) {
      model += (i ? ",\n" : "") + bodies[i];
    }
    model += "],\n \"joints\": [";
    for(size_t i = 0; i < joints.size(); i++) {
      model += (i ? ",\n" : "") + joints[i];
    }
    return model + "]}]}\n";
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

  /* the state derivatives of a model at state y, solved with solver */
  static std::vector<double> Derivatives(const std::string& model,
                                         const std::vector<double>& y_state) {
    auto context = Load(model, "tree_solver_derivatives");
    context->get_active_integrators()[0]->Initialize();
    auto solver = (*context)["model_parameters"]["solvers"][0]
                    .get_data<shift::core::solver::Base>();
    solver->Initialize();
    std::vector<double> y(context->get_state_count() +
                            context->get_auxiliary_state_count(),
                          0.0);
    EXPECT_EQ(y_state.size(), context->get_state_count());
    std::copy(y_state.begin(), y_state.end(), y.begin());
    std::vector<double> dy(y.size(), 0.0);
    EXPECT_FALSE(solver->Solve(0.0, y, dy));
    return dy;
  }

  /* simulate and return the final state (without auxiliary states) */
  static std::vector<double> Run(shift::core::Context& context) {
    std::vector<double> last;
    context.set_state_observer(
      [&last](double, const std::vector<double>& y) { last = y; });
    context.Simulate();
    EXPECT_GE(last.size(), context.get_state_count());
    last.resize(context.get_state_count());
    return last;
  }

  /* simulate with RNCA to a generic state, then compare the derivatives of
  each tree solver with those of RNCA at that state */
  static void Compare(const std::vector<std::string>& bodies,
                      const std::vector<std::string>& joints,
                      const std::vector<std::string>& solvers) {
    auto reference = Load(Model("rnca", bodies, joints), "tree_solver_ref");
    const std::vector<double> y = Run(*reference);
    const std::vector<double> expected =
      Derivatives(Model("rnca", bodies, joints), y);
    double scale = 1.0;
    for(double value : expected) {
      scale = std::fmax(scale, std::fabs(value));
    }
    for(const auto& solver : solvers) {
      const std::vector<double> dy =
        Derivatives(Model(solver, bodies, joints), y);
      ASSERT_EQ(dy.size(), expected.size());
      for(size_t i = 0; i < dy.size(); i++) {
        EXPECT_NEAR(dy[i], expected[i], 1e-9 * scale)
          << Solver(solver) << " state derivative " << i;
      }
    }
  }
};

static std::vector<std::string> ChainBodies() {
  std::vector<std::string> bodies;
  for(int i = 0; i < 7; i++) {
    bodies.push_back(TreeSolvers::Body("b" + std::to_string(i), 0.3 * i));
  }
  return bodies;
}

/* float base, then revolute, prismatic, spherical, screw, a rigid joint (whose
body merges into its parent) and an inverted revolute */
static std::vector<std::string> ChainJoints() {
  return {
    TreeSolvers::Joint("j0", "float", "fixed", "origin", "b0",
                       R"("initial_velocity": [0.3, -0.2, 0.1, 0.5, 0.4, -0.6],
                          )"),
    TreeSolvers::Joint("j1", "revolute", "b0", "tip", "b1",
                       R"("axis": [0, 0.6, 0.8], "initial_velocity": 2.0, )"),
    TreeSolvers::Joint("j2", "prismatic", "b1", "side", "b2",
                       R"("axis": [1, 0, 0], "initial_velocity": -0.5, )"),
    TreeSolvers::Joint("j3", "spherical", "b2", "tip", "b3",
                       R"("initial_velocity": [1.0, -0.5, 0.7], )"),
    TreeSolvers::Joint("j4", "screw", "b3", "tip", "b4",
                       R"("axis": [0.6, 0, 0.8], "pitch": 0.2, )"
                       R"("initial_velocity": 1.5, )"),
    TreeSolvers::Joint("j5", "rigid", "b4", "side", "b5"),
    R"({"name": "j6", "type": "revolute", "axis": [1, 0, 0],
        "initial_velocity": -1.0,
        "parent": [["b6"], "origin"], "child": [["b5"], "tip"]})"};
}

/* a floating base with three branches, one of which branches again */
static std::vector<std::string> BranchedBodies() {
  std::vector<std::string> bodies;
  for(int i = 0; i < 9; i++) {
    bodies.push_back(TreeSolvers::Body("t" + std::to_string(i), 0.2 * i));
  }
  return bodies;
}

static std::vector<std::string> BranchedJoints() {
  return {
    TreeSolvers::Joint("k0", "float", "fixed", "origin", "t0",
                       R"("initial_velocity": [-0.2, 0.3, 0.1, 0.4, -0.3, 0.2],
                          )"),
    TreeSolvers::Joint("k1", "revolute", "t0", "tip", "t1",
                       R"("axis": [0, 1, 0], "initial_velocity": 1.0, )"),
    TreeSolvers::Joint("k2", "spherical", "t1", "tip", "t2",
                       R"("initial_velocity": [0.4, 0.8, -0.3], )"),
    TreeSolvers::Joint("k3", "prismatic", "t0", "side", "t3",
                       R"("axis": [0, 0.8, 0.6], "initial_velocity": 0.3, )"),
    TreeSolvers::Joint("k4", "revolute", "t3", "tip", "t4",
                       R"("axis": [1, 0, 0], "initial_velocity": -2.0, )"),
    TreeSolvers::Joint("k5", "revolute", "t4", "tip", "t5",
                       R"("axis": [0, 0, 1], "initial_velocity": 1.2, )"),
    TreeSolvers::Joint("k6", "revolute", "t4", "side", "t6",
                       R"("axis": [0, 1, 0], "initial_velocity": 0.7, )"),
    TreeSolvers::Joint("k7", "screw", "t0", "origin", "t7",
                       R"("axis": [0, 0, 1], "pitch": -0.1, )"
                       R"("initial_velocity": 0.9, )"),
    TreeSolvers::Joint("k8", "rigid", "t7", "tip", "t8")};
}

/* a tree closed into a loop by a spherical joint at the revolute joint m1:
the loop is satisfied in every configuration of the two revolute joints */
static void AddLoopTree(std::vector<std::string>& bodies,
                        std::vector<std::string>& joints) {
  bodies.push_back(TreeSolvers::Body("l0", 0.5));
  bodies.push_back(TreeSolvers::Body("l1", 0.5));
  joints.push_back(TreeSolvers::Joint("m0", "revolute", "fixed", "origin",
                                      "l0",
                                      R"("axis": [1, 0, 0], )"
                                      R"("initial_velocity": 1.5, )"));
  joints.push_back(TreeSolvers::Joint("m1", "revolute", "l0", "tip", "l1",
                                      R"("axis": [1, 0, 0], )"
                                      R"("initial_velocity": -0.8, )"));
  joints.push_back(R"({"name": "m2", "type": "spherical",
    "parent": [["l0"], "tip"], "child": [["l1"], "origin"]})");
}

TEST(TreeSolvers, ForestIsClassified) {
  auto bodies = ChainBodies();
  auto joints = ChainJoints();
  for(auto& body : BranchedBodies()) {
    bodies.push_back(body);
  }
  for(auto& joint : BranchedJoints()) {
    joints.push_back(joint);
  }
  AddLoopTree(bodies, joints);

  auto context = TreeSolvers::Load(TreeSolvers::Model("rnca", bodies, joints),
                                   "tree_solver_forest");
  const auto& trees = context->get_trees();
  ASSERT_EQ(trees.size(), 3u);
  const auto& active_joints = context->get_active_joints();
  int found = 0;
  for(const auto& tree : trees) {
    const std::string root = active_joints[tree.root_joint]->get_name();
    if(root == "j0") {
      EXPECT_EQ(tree.kind, shift::core::topology::Tree::Kind::kSerialChain);
      EXPECT_EQ(tree.bodies.size(), 6u); /* b5 merges into b4 */
      EXPECT_EQ(tree.dq_count, 6u + 1 + 1 + 3 + 1 + 1);
      found++;
    } else if(root == "k0") {
      EXPECT_EQ(tree.kind, shift::core::topology::Tree::Kind::kBranched);
      EXPECT_EQ(tree.bodies.size(), 8u); /* t8 merges into t7 */
      found++;
    } else if(root == "m0") {
      EXPECT_EQ(tree.kind, shift::core::topology::Tree::Kind::kClosedLoop);
      found++;
    }
  }
  EXPECT_EQ(found, 3);
}

TEST(TreeSolvers, SerialChainMatchesRNCA) {
  TreeSolvers::Compare(ChainBodies(), ChainJoints(), {"aba", "dca"});
}

/* the serial chain with springs and dampers (a constant term included) on
its revolute, prismatic and screw joints: the joint forces enter the joint
accelerations and wrenches */
TEST(TreeSolvers, JointForcesMatchRNCA) {
  std::vector<std::string> joints = ChainJoints();
  const std::string forces =
    R"("stiffness_coefficients": [0.7, 3.0], "damping_coefficients": [0.0, 0.4], )";
  joints[1] = TreeSolvers::Joint(
    "j1", "revolute", "b0", "tip", "b1",
    R"("axis": [0, 0.6, 0.8], "initial_velocity": 2.0, )" + forces);
  joints[2] = TreeSolvers::Joint(
    "j2", "prismatic", "b1", "side", "b2",
    R"("axis": [1, 0, 0], "initial_velocity": -0.5, )" + forces);
  joints[4] = TreeSolvers::Joint(
    "j4", "screw", "b3", "tip", "b4",
    R"("axis": [0.6, 0, 0.8], "pitch": 0.2, "initial_velocity": 1.5, )" +
      forces);
  TreeSolvers::Compare(ChainBodies(), joints, {"aba", "dca"});
}

/* the arm with a gripper of the topology documentation: the hand carries two
finger slides, so the forest is one branched tree with the seven revolute and
two prismatic coordinates; the rigid joints A and I merge the base into the
fixed body and the hand into link_7, so the tree starts at joint B */
TEST(TreeSolvers, GripperIsBranchedTree) {
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                   "/kuka_lbr_iiwa_14_r820/kuka_iiwa_gripper.json");
  const auto& trees = context.get_trees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_EQ(trees[0].kind, shift::core::topology::Tree::Kind::kBranched);
  EXPECT_EQ(trees[0].bodies.size(), 9u);
  EXPECT_EQ(trees[0].dq_count, 9u);
  EXPECT_EQ(context.get_active_joints()[trees[0].root_joint]->get_name(), "B");
}

TEST(TreeSolvers, BranchedTreeMatchesRNCA) {
  TreeSolvers::Compare(BranchedBodies(), BranchedJoints(), {"aba", "dca"});
}

/* two independent trees in one model: a chain and a branched tree */
TEST(TreeSolvers, ForestMatchesRNCA) {
  auto bodies = ChainBodies();
  auto joints = ChainJoints();
  for(auto& body : BranchedBodies()) {
    bodies.push_back(body);
  }
  for(auto& joint : BranchedJoints()) {
    joints.push_back(joint);
  }
  TreeSolvers::Compare(bodies, joints, {"aba", "dca"});
}

/* a long chain of revolute joints: the DCA hierarchy is balanced, about
log2(n) deep, and the trajectories of all solvers agree */
TEST(TreeSolvers, LongChainTrajectories) {
  const int n = 32;
  std::vector<std::string> bodies;
  std::vector<std::string> joints;
  for(int i = 0; i < n; i++) {
    const std::string name = "c" + std::to_string(i);
    bodies.push_back(TreeSolvers::Body(name, 0.01 * i));
    const std::string axis =
      i % 3 == 0 ? "[1, 0, 0]" : (i % 3 == 1 ? "[0, 1, 0]" : "[0, 0, 1]");
    joints.push_back(TreeSolvers::Joint(
      "r" + std::to_string(i), "revolute",
      i == 0 ? "fixed" : "c" + std::to_string(i - 1),
      i == 0 ? "origin" : "tip", name,
      R"("axis": )" + axis + R"(, "initial_velocity": 0.2, )"));
  }

  auto dca = TreeSolvers::Load(TreeSolvers::Model("dca", bodies, joints, 0.2),
                               "tree_solver_long_dca");
  const std::vector<double> y_dca = TreeSolvers::Run(*dca);
  auto solver = dynamic_cast<shift::core::solver::DCA*>(
    (*dca)["model_parameters"]["solvers"][0]
      .get_data<shift::core::solver::Base>());
  ASSERT_NE(solver, nullptr);
  ASSERT_EQ(solver->get_assembly_depths().size(), 1u);
  EXPECT_EQ(solver->get_assembly_depths()[0], 5); /* 2^5 = 32 */

  auto rnca = TreeSolvers::Load(TreeSolvers::Model("rnca", bodies, joints, 0.2),
                                "tree_solver_long_rnca");
  const std::vector<double> y_rnca = TreeSolvers::Run(*rnca);
  auto aba = TreeSolvers::Load(TreeSolvers::Model("aba", bodies, joints, 0.2),
                               "tree_solver_long_aba");
  const std::vector<double> y_aba = TreeSolvers::Run(*aba);
  ASSERT_EQ(y_aba.size(), y_rnca.size());
  ASSERT_EQ(y_dca.size(), y_rnca.size());
  for(size_t i = 0; i < y_rnca.size(); i++) {
    EXPECT_NEAR(y_aba[i], y_rnca[i], 1e-6) << "aba state " << i;
    EXPECT_NEAR(y_dca[i], y_rnca[i], 1e-6) << "dca state " << i;
  }
}

/* a chain long enough that the assemblies of a level share threads: the
same derivatives as with one thread, to the last bit, and as ABA's */
TEST(TreeSolvers, DcaLevelsShareThreads) {
  const int n = 256;
  std::vector<std::string> bodies;
  std::vector<std::string> joints;
  for(int i = 0; i < n; i++) {
    const std::string name = "c" + std::to_string(i);
    bodies.push_back(TreeSolvers::Body(name, 0.01 * (i % 10)));
    const std::string axis =
      i % 3 == 0 ? "[1, 0, 0]" : (i % 3 == 1 ? "[0, 1, 0]" : "[0, 0, 1]");
    joints.push_back(TreeSolvers::Joint(
      "r" + std::to_string(i), "revolute",
      i == 0 ? "fixed" : "c" + std::to_string(i - 1),
      i == 0 ? "origin" : "tip", name,
      R"("axis": )" + axis + R"(, "initial_displacement": )" +
        std::to_string(0.3 * std::sin(i)) + R"(, "initial_velocity": )" +
        std::to_string(0.2 * std::cos(i)) + ", "));
  }
  const std::string model = TreeSolvers::Model("dca", bodies, joints);
  auto probe = TreeSolvers::Load(model, "tree_solver_threads_probe");
  std::vector<double> y(probe->get_state_count(), 0.0);
  std::copy(probe->get_q().begin(), probe->get_q().end(), y.begin());
  std::copy(probe->get_dq(), probe->get_dq() + probe->get_dq_count(),
            y.begin() + probe->get_q_count());

  auto dca = TreeSolvers::Load(model, "tree_solver_threads_dca");
  auto solver = dynamic_cast<shift::core::solver::DCA*>(
    (*dca)["model_parameters"]["solvers"][0]
      .get_data<shift::core::solver::Base>());
  ASSERT_NE(solver, nullptr);
  solver->Initialize();
  int most = 1;
  for(int threads : solver->get_level_threads()) {
    most = std::max(most, threads);
  }
  if(omp_get_max_threads() > 1) {
    EXPECT_GT(most, 1);
  }
  const std::vector<double> threaded = TreeSolvers::Derivatives(model, y);

  const int max_threads = omp_get_max_threads();
  omp_set_num_threads(1);
  const std::vector<double> serial = TreeSolvers::Derivatives(model, y);
  omp_set_num_threads(max_threads);
  ASSERT_EQ(threaded.size(), serial.size());
  for(size_t i = 0; i < serial.size(); i++) {
    EXPECT_EQ(threaded[i], serial[i]) << "state derivative " << i;
  }

  const std::vector<double> aba = TreeSolvers::Derivatives(
    TreeSolvers::Model("aba", bodies, joints), y);
  ASSERT_EQ(aba.size(), serial.size());
  double scale = 1.0;
  for(double value : aba) {
    scale = std::fmax(scale, std::fabs(value));
  }
  /* DCA and ABA differ by rounding, the more the longer the chain: of the
  largest derivative, 2e-12 over 64 bodies, 1.5e-10 over 256 and 8e-10 over
  512 (RNCA and ABA: 2.4e-9 over 256) */
  for(size_t i = 0; i < aba.size(); i++) {
    EXPECT_NEAR(threaded[i], aba[i], 1e-9 * scale) << "state derivative " << i;
  }
}

TEST(TreeSolvers, UnsuitableModelsAreRejected) {
  /* aba and dca on a closed loop */
  std::vector<std::string> loop_bodies;
  std::vector<std::string> loop_joints;
  AddLoopTree(loop_bodies, loop_joints);
  for(const std::string solver_type : {"aba", "dca"}) {
    auto context = TreeSolvers::Load(
      TreeSolvers::Model(solver_type, loop_bodies, loop_joints),
      "tree_solver_loop_" + solver_type);
    auto solver = (*context)["model_parameters"]["solvers"][0]
                    .get_data<shift::core::solver::Base>();
    EXPECT_THROW(solver->Initialize(), shift::core::Exception) << solver_type;
  }

  /* dca on a chain with a massless body in the middle, which aba solves */
  const std::vector<std::string> bodies = {TreeSolvers::Body("c0", 0.5),
                                           R"({"name": "v",
    "frames": [{"name": "tip", "T": [0.4, 0.1, 0.3]}]})",
                                           TreeSolvers::Body("c1", 0.5)};
  const std::vector<std::string> joints = {
    TreeSolvers::Joint("r0", "revolute", "fixed", "origin", "c0",
                       R"("axis": [1, 0, 0], )"),
    TreeSolvers::Joint("r1", "revolute", "c0", "tip", "v",
                       R"("axis": [0, 1, 0], )"),
    TreeSolvers::Joint("r2", "revolute", "v", "tip", "c1",
                       R"("axis": [0, 0, 1], )")};
  {
    auto context = TreeSolvers::Load(TreeSolvers::Model("dca", bodies, joints),
                                     "tree_solver_dca_massless");
    auto solver = (*context)["model_parameters"]["solvers"][0]
                    .get_data<shift::core::solver::Base>();
    EXPECT_THROW(solver->Initialize(), shift::core::Exception);
  }
  TreeSolvers::Compare(bodies, joints, {"aba"});
}

/* the total energy of the conservative chain (gravity only, no joint forces)
after integrating with solver, relative to its initial value; the rigid joint
in the middle of the chain checks that a joint below it is coupled to the
joints above it */
static double EnergyDrift(const std::string& solver_type) {
  auto context = TreeSolvers::Load(
    TreeSolvers::Model(solver_type, ChainBodies(), ChainJoints(), 1.0),
    "tree_solver_energy_" + solver_type);
  auto solver = (*context)["model_parameters"]["solvers"][0]
                  .get_data<shift::core::solver::Base>();
  auto energy = [&](const std::vector<double>& state) {
    std::vector<double> y(context->get_state_count() +
                            context->get_auxiliary_state_count(),
                          0.0);
    std::copy(state.begin(), state.end(), y.begin());
    std::vector<double> dy(y.size(), 0.0);
    EXPECT_FALSE(solver->Solve(0.0, y, dy));
    return solver->KineticEnergy(context->get_dhva()) +
           solver->GravitationalPotentialEnergy(context->get_dhva());
  };
  const std::vector<double> initial_state(
    context->get_q().begin(),
    context->get_q().begin() + context->get_state_count());
  const std::vector<double> final_state = TreeSolvers::Run(*context);
  const double initial = energy(initial_state);
  const double final_energy = energy(final_state);
  return (final_energy - initial) / std::fabs(initial);
}

TEST(TreeSolvers, EnergyIsConserved) {
  for(const std::string solver : {"rnca", "aba", "dca"}) {
    EXPECT_NEAR(EnergyDrift(solver), 0.0, 1e-6) << solver;
  }
}

/* the auto solver: a chain and a branched tree solved by aba or dca, and a
tree with a closed loop by rnca, in one model */
TEST(TreeSolvers, AutoSolvesEachTree) {
  auto bodies = ChainBodies();
  auto joints = ChainJoints();
  for(auto& body : BranchedBodies()) {
    bodies.push_back(body);
  }
  for(auto& joint : BranchedJoints()) {
    joints.push_back(joint);
  }
  AddLoopTree(bodies, joints);

  auto context = TreeSolvers::Load(TreeSolvers::Model("auto", bodies, joints),
                                   "tree_solver_auto");
  auto solver = dynamic_cast<shift::core::solver::Auto*>(
    (*context)["model_parameters"]["solvers"][0]
      .get_data<shift::core::solver::Base>());
  ASSERT_NE(solver, nullptr);
  /* the integrator first, which sizes the integral of the constraint error */
  context->get_active_integrators()[0]->Initialize();
  solver->Initialize();
  const auto& trees = context->get_trees();
  ASSERT_EQ(solver->get_delegated_trees().size(), 2u);
  for(int t : solver->get_delegated_trees()) {
    EXPECT_NE(trees[t].kind, shift::core::topology::Tree::Kind::kClosedLoop);
  }

  TreeSolvers::Compare(bodies, joints,
                       {"auto", R"({"type": "auto", "tree_solver": "dca"})"});
}

/* with closed loops only, auto delegates nothing and solves as rnca */
TEST(TreeSolvers, AutoWithLoopsOnly) {
  std::vector<std::string> bodies;
  std::vector<std::string> joints;
  AddLoopTree(bodies, joints);
  auto context = TreeSolvers::Load(TreeSolvers::Model("auto", bodies, joints),
                                   "tree_solver_auto_loops");
  auto solver = dynamic_cast<shift::core::solver::Auto*>(
    (*context)["model_parameters"]["solvers"][0]
      .get_data<shift::core::solver::Base>());
  ASSERT_NE(solver, nullptr);
  /* the integrator first, which sizes the integral of the constraint error */
  context->get_active_integrators()[0]->Initialize();
  solver->Initialize();
  EXPECT_TRUE(solver->get_delegated_trees().empty());
  TreeSolvers::Compare(bodies, joints, {"auto"});
}

/* Speed of the solvers on trees without closed loops: the time of one
evaluation of the state derivatives (Solver::Solve) at a fixed state, best of
three repeats, for revolute chains and full binary trees of several sizes and a
floating base with limbs. "shared" is the part every solver does the same
(Context::UpdateDHVA and Context::SetForces). Registered as the benchmark
solver_speed, not run by meson test; time it in an optimized build. */
class SolverSpeed {
public:
  /* microseconds per call of f, best of three repeats of about 0.2 s */
  template <typename F>
  static double Time(F&& f) {
    int calls = 1;
    for(;;) {
      const auto start = std::chrono::steady_clock::now();
      for(int i = 0; i < calls; i++) {
        f();
      }
      const double seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                               .count();
      if(seconds > 0.05) {
        calls = static_cast<int>(calls * 0.2 / seconds) + 1;
        break;
      }
      calls *= 4;
    }
    double best = 1e30;
    for(int repeat = 0; repeat < 3; repeat++) {
      const auto start = std::chrono::steady_clock::now();
      for(int i = 0; i < calls; i++) {
        f();
      }
      best = std::fmin(best, std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                                 .count() /
                               calls);
    }
    return best * 1e6;
  }

  static void Run(const std::string& name,
                  const std::vector<std::string>& bodies,
                  const std::vector<std::string>& joints) {
    double shared = 0.0;
    double times[3] = {0.0, 0.0, 0.0};
    const char* solvers[3] = {"rnca", "aba", "dca"};
    size_t dq_count = 0;
    for(int s = 0; s < 3; s++) {
      auto context = TreeSolvers::Load(
        TreeSolvers::Model(solvers[s], bodies, joints), "solver_speed");
      context->get_active_integrators()[0]->Initialize();
      auto solver = (*context)["model_parameters"]["solvers"][0]
                      .get_data<shift::core::solver::Base>();
      solver->Initialize();
      std::vector<double> y(context->get_state_count() +
                              context->get_auxiliary_state_count(),
                            0.0);
      std::copy(context->get_q().begin(),
                context->get_q().begin() + context->get_state_count(),
                y.begin());
      std::vector<double> dy(y.size(), 0.0);
      dq_count = context->get_dq_count();
      times[s] = Time([&]() { solver->Solve(0.0, y, dy); });
      if(s == 0) {
        const double* q = y.data();
        const double* dq = q + context->get_q_count();
        shared = Time([&]() {
          context->UpdateDHVA(q, dq);
          context->SetForces(0.0, y, dy);
        });
      }
    }
    printf("%-22s %5zu dof  shared %8.2f us | solver part: rnca %8.2f  aba "
           "%8.2f  dca %8.2f us | rnca/aba %5.2f  dca/aba %5.2f\n",
           name.c_str(), dq_count, shared, times[0] - shared,
           times[1] - shared, times[2] - shared,
           (times[0] - shared) / (times[1] - shared),
           (times[2] - shared) / (times[1] - shared));
    printf("%-22s            total Solve:   rnca %8.2f  aba %8.2f  dca %8.2f "
           "us | rnca/aba %5.2f\n",
           "", times[0], times[1], times[2], times[0] / times[1]);
  }

  static std::string Axis(int i) {
    return i % 3 == 0 ? "[1, 0, 0]" : (i % 3 == 1 ? "[0, 1, 0]" : "[0, 0, 1]");
  }

  /* n bodies, each joined by a revolute joint to body parent(i) */
  template <typename Parent>
  static void Tree(const std::string& name, int n, Parent parent) {
    std::vector<std::string> bodies;
    std::vector<std::string> joints;
    for(int i = 0; i < n; i++) {
      const std::string body = "s" + std::to_string(i);
      bodies.push_back(TreeSolvers::Body(body, 0.01 * (i % 7)));
      const int p = parent(i);
      joints.push_back(TreeSolvers::Joint(
        "r" + std::to_string(i), "revolute",
        p < 0 ? "fixed" : "s" + std::to_string(p),
        p < 0 ? "origin" : (i % 2 ? "tip" : "side"), body,
        R"("axis": )" + Axis(i) + R"(, "initial_velocity": 0.3, )"));
    }
    Run(name + " " + std::to_string(n), bodies, joints);
  }
};

TEST(SolverSpeed, SerialChains) {
  for(int n : {4, 8, 16, 32, 64, 128, 256}) {
    SolverSpeed::Tree("chain", n, [](int i) { return i - 1; });
  }
}

TEST(SolverSpeed, BinaryTrees) {
  for(int n : {7, 15, 31, 63, 127, 255}) {
    SolverSpeed::Tree("binary tree", n,
                      [](int i) { return i == 0 ? -1 : (i - 1) / 2; });
  }
}

/* a floating base with four limbs of six revolute joints: a legged robot */
TEST(SolverSpeed, FloatingBaseWithLimbs) {
  std::vector<std::string> bodies = {TreeSolvers::Body("base", 2.0)};
  std::vector<std::string> joints = {TreeSolvers::Joint(
    "float", "float", "fixed", "origin", "base",
    R"("initial_velocity": [0.1, 0.2, -0.1, 0.3, -0.2, 0.1], )")};
  for(int limb = 0; limb < 4; limb++) {
    for(int k = 0; k < 6; k++) {
      const std::string body =
        "l" + std::to_string(limb) + "_" + std::to_string(k);
      bodies.push_back(TreeSolvers::Body(body, 0.1 * k));
      joints.push_back(TreeSolvers::Joint(
        "j" + std::to_string(limb) + "_" + std::to_string(k), "revolute",
        k == 0 ? "base" : "l" + std::to_string(limb) + "_" +
                            std::to_string(k - 1),
        k == 0 ? (limb % 2 ? "tip" : "side") : "tip", body,
        R"("axis": )" + SolverSpeed::Axis(k + limb) +
          R"(, "initial_velocity": 0.5, )"));
    }
  }
  SolverSpeed::Run("float base + 4x6 limbs", bodies, joints);
}

/* the iteration matrix of the implicit integrators: for the same state and
the same tangents (entries between the velocities, springs between every
pair of bodies, which couple the trees, and one column found by
differences), the solution of the tree solvers' system (the accelerations'
residual, S = I + M^-1 (c C + k K)) and of Auto's (RNCA's rows of the closed
loop, the accelerations' rows of the delegated trees) is RNCA's, whose system
is that of the forces, M + c C + k K. The right-hand sides and the
differenced column of the accelerations' rows are M^-1 times those of the
forces, from RNCA's system without tangents. */
struct IterationSystem {
  std::unique_ptr<shift::core::Context> context;
  shift::core::solver::Base* solver = nullptr;
};

static IterationSystem LoadIterationSystem(const std::string& model,
                                           const std::vector<double>& y_state,
                                           std::vector<double>& y) {
  IterationSystem system;
  system.context = TreeSolvers::Load(model, "tree_solver_iteration");
  shift::core::Context& context = *system.context;
  context.get_active_integrators()[0]->Initialize();
  system.solver = context["model_parameters"]["solvers"][0]
                    .get_data<shift::core::solver::Base>();
  system.solver->Initialize();
  y.assign(context.get_state_count() + context.get_auxiliary_state_count(),
           0.0);
  std::copy(y_state.begin(), y_state.end(), y.begin());
  std::vector<double> dy(y.size(), 0.0);
  EXPECT_FALSE(system.solver->Solve(0.0, y, dy));
  return system;
}

/* Residual at y, for the iteration matrix to be formed at that state */
static void ResidualAt(IterationSystem& system, const std::vector<double>& y) {
  shift::core::Context& context = *system.context;
  const std::vector<double> ddq(context.get_dq_count(), 0.0);
  const std::vector<double> la(system.solver->get_multiplier_count(), 0.0);
  std::vector<double> r;
  std::vector<double> dy(y.size(), 0.0);
  ASSERT_FALSE(system.solver->Residual(0.0, y, ddq, la, r, dy));
}

static void CheckIterationMatrix(const std::vector<std::string>& bodies,
                                 const std::vector<std::string>& joints,
                                 const std::string& solver_type,
                                 bool expect_delegated) {
  SCOPED_TRACE(solver_type);
  auto reference_run =
    TreeSolvers::Load(TreeSolvers::Model("rnca", bodies, joints, 0.1),
                      "tree_solver_iteration_run");
  const std::vector<double> y_state = TreeSolvers::Run(*reference_run);

  std::vector<double> y;
  IterationSystem reference = LoadIterationSystem(
    TreeSolvers::Model("rnca", bodies, joints), y_state, y);
  IterationSystem tested = LoadIterationSystem(
    TreeSolvers::Model(solver_type, bodies, joints), y_state, y);
  ASSERT_TRUE(tested.solver->supports_implicit());
  shift::core::Context& context = *reference.context;
  const unsigned int n = context.get_dq_count();
  const unsigned int multipliers = reference.solver->get_multiplier_count();
  ASSERT_EQ(tested.solver->get_multiplier_count(), multipliers);
  const unsigned int size = n + multipliers;

  /* the rows in the form of the accelerations */
  std::vector<bool> acceleration_row(n, solver_type != "auto");
  if(solver_type == "auto") {
    auto* automatic = dynamic_cast<shift::core::solver::Auto*>(tested.solver);
    ASSERT_NE(automatic, nullptr);
    EXPECT_EQ(!automatic->get_delegated_trees().empty(), expect_delegated);
    const auto& trees = tested.context->get_trees();
    const auto& dq_lookup = tested.context->get_dq_lookup_compressed();
    const auto& dof = tested.context->get_joint_dof_compressed();
    for(int t : automatic->get_delegated_trees()) {
      for(int j : trees[t].bodies) {
        for(unsigned int m = 0; m < dof[j]; m++) {
          acceleration_row[dq_lookup[j] + m] = true;
        }
      }
    }
  }

  std::mt19937 generator(4321);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  const double c = 0.02;
  const double k = 0.003;
  shift::core::Tangents tangents;
  for(unsigned int i = 0; i < n; i++) {
    tangents.AddJointStiffness(i, i, 2.0 + uniform(generator));
    tangents.AddJointDamping(i, i, 1.0 + 0.5 * uniform(generator));
    if(i + 1 < n) {
      tangents.AddJointStiffness(i, i + 1, 0.3 * uniform(generator));
      tangents.AddJointStiffness(i + 1, i, 0.3 * uniform(generator));
    }
  }
  std::vector<int> loads;
  for(auto* body : context.get_active_articulated_bodies()) {
    const int load_index = context.GetBodyLoadIndex(body);
    if(load_index > -1 &&
       std::find(loads.begin(), loads.end(), load_index) == loads.end()) {
      loads.push_back(load_index);
    }
  }
  ASSERT_GE(loads.size(), 2u);
  double block[36] = {}, minus_block[36] = {};
  for(int m = 0; m < 6; m++) {
    for(int l = 0; l < 6; l++) {
      block[m * 6 + l] = (m == l ? 2.0 : 0.0) + 0.2 * uniform(generator);
      minus_block[m * 6 + l] = -block[m * 6 + l];
    }
  }
  for(size_t a = 0; a < loads.size(); a++) {
    for(size_t b = a + 1; b < loads.size(); b++) {
      tangents.AddBodyStiffness(loads[a], loads[a], block);
      tangents.AddBodyStiffness(loads[b], loads[b], block);
      tangents.AddBodyStiffness(loads[a], loads[b], minus_block);
      tangents.AddBodyStiffness(loads[b], loads[a], minus_block);
    }
  }
  tangents.AddBodyDamping(loads[0], loads[0], block);

  /* one column by differences, of the forces; a right-hand side */
  const std::vector<int> differenced = {static_cast<int>(n / 2)};
  std::vector<double> k_column(n), c_column(n), rhs(size);
  for(unsigned int i = 0; i < n; i++) {
    k_column[i] = 0.5 * uniform(generator) + (i == n / 2 ? 3.0 : 0.0);
    c_column[i] = 0.5 * uniform(generator);
  }
  for(double& value : rhs) {
    value = uniform(generator);
  }

  /* M^-1 of the force vectors, from RNCA's system without tangents */
  ResidualAt(reference, y);
  const shift::core::Tangents none;
  ASSERT_FALSE(
    reference.solver->FactorIterationMatrix(none, 0.0, 0.0, {}, {}, {}));
  auto inverse_mass = [&](const std::vector<double>& force) {
    std::vector<double> x(size, 0.0);
    std::copy(force.begin(), force.begin() + n, x.begin());
    EXPECT_FALSE(reference.solver->SolveIterationMatrix(x));
    return x;
  };
  const std::vector<double> rhs_acceleration = inverse_mass(rhs);
  const std::vector<double> k_acceleration = inverse_mass(k_column);
  const std::vector<double> c_acceleration = inverse_mass(c_column);

  /* RNCA, the forces */
  ResidualAt(reference, y);
  ASSERT_FALSE(reference.solver->FactorIterationMatrix(
    tangents, c, k, differenced, k_column, c_column));
  std::vector<double> expected(rhs);
  ASSERT_FALSE(reference.solver->SolveIterationMatrix(expected));

  /* the tested solver, the accelerations in their rows */
  std::vector<double> tested_rhs(rhs), tested_k(k_column), tested_c(c_column);
  for(unsigned int i = 0; i < n; i++) {
    if(acceleration_row[i]) {
      tested_rhs[i] = rhs_acceleration[i];
      tested_k[i] = k_acceleration[i];
      tested_c[i] = c_acceleration[i];
    }
  }
  ResidualAt(tested, y);
  ASSERT_FALSE(tested.solver->FactorIterationMatrix(
    tangents, c, k, differenced, tested_k, tested_c));
  /* the factors are kept: a Residual at another state in between */
  std::vector<double> y_other(y);
  for(unsigned int i = 0; i < n; i++) {
    y_other[context.get_q_count() + i] += 0.1;
  }
  ResidualAt(tested, y_other);
  ASSERT_FALSE(tested.solver->SolveIterationMatrix(tested_rhs));

  double scale = 1.0;
  for(double value : expected) {
    scale = std::max(scale, std::fabs(value));
  }
  for(unsigned int i = 0; i < size; i++) {
    EXPECT_NEAR(tested_rhs[i], expected[i], 1e-8 * scale) << "entry " << i;
  }
}

TEST(TreeSolvers, IterationMatrixMatchesRNCA) {
  auto bodies = ChainBodies();
  auto joints = ChainJoints();
  for(auto& body : BranchedBodies()) {
    bodies.push_back(body);
  }
  for(auto& joint : BranchedJoints()) {
    joints.push_back(joint);
  }
  CheckIterationMatrix(bodies, joints, "aba", false);
  CheckIterationMatrix(bodies, joints, "dca", false);
  AddLoopTree(bodies, joints);
  CheckIterationMatrix(bodies, joints, "auto", true);
}

/* HHT with each tree solver follows HHT with RNCA: the forest of a chain, a
branched tree and a closed loop, with springs and dampers on the joints of
the chain; the solvers converge Newton to the same tolerance, so their
trajectories agree to it */
TEST(TreeSolvers, ImplicitIntegrationMatchesRNCA) {
  auto bodies = ChainBodies();
  std::vector<std::string> joints = ChainJoints();
  const std::string forces =
    R"("stiffness_coefficients": [0.0, 300.0], "damping_coefficients": [0.0, 2.0], )";
  joints[1] = TreeSolvers::Joint(
    "j1", "revolute", "b0", "tip", "b1",
    R"("axis": [0, 0.6, 0.8], "initial_velocity": 2.0, )" + forces);
  joints[2] = TreeSolvers::Joint(
    "j2", "prismatic", "b1", "side", "b2",
    R"("axis": [1, 0, 0], "initial_velocity": -0.5, )" + forces);
  for(auto& body : BranchedBodies()) {
    bodies.push_back(body);
  }
  for(auto& joint : BranchedJoints()) {
    joints.push_back(joint);
  }
  auto with_hht = [](std::string model) {
    const std::string from = R"("type": "ode113")";
    const size_t at = model.find(from);
    EXPECT_NE(at, std::string::npos);
    model.replace(at, from.size(), R"("type": "hht")");
    return model;
  };
  for(bool loop : {false, true}) {
    if(loop) {
      AddLoopTree(bodies, joints);
    }
    auto reference = TreeSolvers::Load(
      with_hht(TreeSolvers::Model("rnca", bodies, joints, 0.1)),
      "tree_hht_rnca");
    const std::vector<double> expected = TreeSolvers::Run(*reference);
    double scale = 1.0;
    for(double value : expected) {
      scale = std::max(scale, std::fabs(value));
    }
    const std::vector<std::string> solvers =
      loop ? std::vector<std::string>{"auto",
                                      R"({"type": "auto", "tree_solver": "dca"})"}
           : std::vector<std::string>{"aba", "dca", "auto"};
    for(const auto& solver : solvers) {
      SCOPED_TRACE(solver + (loop ? " with the loop" : ""));
      auto context = TreeSolvers::Load(
        with_hht(TreeSolvers::Model(solver, bodies, joints, 0.1)), "tree_hht");
      const std::vector<double> y = TreeSolvers::Run(*context);
      ASSERT_EQ(y.size(), expected.size());
      for(size_t i = 0; i < y.size(); i++) {
        EXPECT_NEAR(y[i], expected[i], 1e-6 * scale) << "state " << i;
      }
    }
  }
}
