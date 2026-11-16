/* body::Nodal: a uniform cantilever of 3-D Euler-Bernoulli frame elements
with lumped masses, reduced to retained nodes by Guyan reduction. The frame
elements are exact for loads at the nodes, so the stiffness reduced to the
tip is that of the continuous cantilever; the frequencies of the clamped
beam approach the clamped-free ones, from above as fewer nodes are kept; and
RNCA, ABA and DCA give the same accelerations with the beam pinned and a
body hanging from its tip. */

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

#include "shift/core/body/nodal.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/integrator/base.h"
#include "shift/core/linearization.h"
#include "shift/core/solver/base.h"

#include "ksl/linalg.h"

class TestNodalBeam : public shift::core::body::Nodal {
public:
  static constexpr double kLength = 1.0;
  static constexpr double kMu = 1.0;    //!< mass per length, kg/m
  static constexpr double kEi = 100.0;  //!< bending stiffness, both planes
  static constexpr double kEa = 1.0e6;  //!< axial stiffness
  static constexpr double kGj = 100.0;  //!< torsional stiffness
  /* beta_n L of the clamped-free beam */
  static constexpr double kClamped[3] = {1.87510406871196, 4.69409113297417,
                                         7.85475743823761};

  /* \p elements elements along x from the origin, clamped at the root
  (point 0); the points k > 0 with k % \p every == 0 and the tip are
  retained; \p deformed starts the tip displaced and moving; \p gj the
  torsional stiffness */
  TestNodalBeam(int elements, int every, bool deformed = false,
                double gj = kGj) {
    const int points = elements + 1;
    const double h = kLength / elements;
    std::vector<ksl_vec3_t> positions(points);
    std::vector<double> masses(points);
    std::vector<ksl_mat3x3_t> rotary(points);
    for(int p = 0; p < points; p++) {
      positions[p] = ksl_vec3(p * h, 0.0, 0.0);
      masses[p] = kMu * h * (p == 0 || p == elements ? 0.5 : 1.0);
      const double i = masses[p] * h * h / 12.0;
      rotary[p] = {};
      rotary[p].m00 = 2.0 * i;
      rotary[p].m11 = i;
      rotary[p].m22 = i;
    }
    const int n = 6 * points;
    std::vector<double> stiffness(n * n, 0.0);
    for(int e = 0; e < elements; e++) {
      double k[12][12] = {};
      Element(h, gj, k);
      for(int a = 0; a < 12; a++) {
        for(int b = 0; b < 12; b++) {
          stiffness[(6 * e + a) * n + 6 * e + b] += k[a][b];
        }
      }
    }
    std::vector<unsigned int> retained;
    for(int p = 1; p < points; p++) {
      if(p % every == 0 || p == elements) {
        retained.push_back(p);
      }
    }
    SetFiniteElementModel(positions, masses, rotary, 6, stiffness, {}, {0},
                          retained);
    SetNode("tip", elements);
    if(deformed) {
      /* the tip's six coordinates last */
      std::vector<double> eta(get_elastic_count(), 0.0);
      std::vector<double> eta_dot(get_elastic_count(), 0.0);
      const double at[6] = {1e-5, 0.004, -0.006, 0.01, 0.009, 0.006};
      const double rate[6] = {-1e-3, 0.3, 0.2, -0.4, -0.3, 0.5};
      for(int d = 0; d < 6; d++) {
        eta[eta.size() - 6 + d] = at[d];
        eta_dot[eta.size() - 6 + d] = rate[d];
      }
      SetInitialElasticState(eta, eta_dot);
    }
  }

  /* the stiffness of a frame element of length h along x, the degrees of
  freedom (u, v, w, theta_x, theta_y, theta_z) of each end; theta_z = v'
  and theta_y = -w' */
  static void Element(double h, double gj, double k[12][12]) {
    auto add = [&](int a, int b, double value) {
      k[a][b] += value;
    };
    const double axial = kEa / h, torsion = gj / h;
    const int u[2] = {0, 6}, rx[2] = {3, 9};
    for(int a = 0; a < 2; a++) {
      for(int b = 0; b < 2; b++) {
        const double sign = a == b ? 1.0 : -1.0;
        add(u[a], u[b], sign * axial);
        add(rx[a], rx[b], sign * torsion);
      }
    }
    const double c = kEi / (h * h * h);
    /* (v1, theta_z1, v2, theta_z2) and (w1, -theta_y1, w2, -theta_y2) */
    const double bend[4][4] = {{12.0, 6.0 * h, -12.0, 6.0 * h},
                               {6.0 * h, 4.0 * h * h, -6.0 * h, 2.0 * h * h},
                               {-12.0, -6.0 * h, 12.0, -6.0 * h},
                               {6.0 * h, 2.0 * h * h, -6.0 * h, 4.0 * h * h}};
    const int v[4] = {1, 5, 7, 11}, w[4] = {2, 4, 8, 10};
    const double w_sign[4] = {1.0, -1.0, 1.0, -1.0};
    for(int a = 0; a < 4; a++) {
      for(int b = 0; b < 4; b++) {
        add(v[a], v[b], c * bend[a][b]);
        add(w[a], w[b], c * w_sign[a] * w_sign[b] * bend[a][b]);
      }
    }
  }

  static double Omega(double beta_l) {
    return beta_l * beta_l / (kLength * kLength) * std::sqrt(kEi / kMu);
  }

  static bool registered_;
};

constexpr double TestNodalBeam::kClamped[3];

bool TestNodalBeam::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_nodal_beam_full",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestNodalBeam>(20, 1);
    }) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_nodal_beam_reduced",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestNodalBeam>(20, 5);
    }) &&
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "test_nodal_beam_tip",
    +[]() -> std::unique_ptr<shift::core::body::Base> {
      return std::make_unique<TestNodalBeam>(20, 20, true);
    });

static std::unique_ptr<shift::core::Context> LoadNodal(const std::string& model,
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

/* the beam of \p type on a joint of \p joint (rigid: clamped, revolute about
y: pinned) at its root, solved by \p solver; \p hanging hangs a body from
its tip on a revolute joint about x; \p gravity along z */
static std::string NodalBeamModel(const std::string& type,
                                  const std::string& joint,
                                  const std::string& solver = "rnca",
                                  bool hanging = false,
                                  double gravity = 0.0) {
  std::stringstream ss;
  ss << std::setprecision(17) << std::showpoint;
  ss << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, )"
     << gravity << R"(], "solvers": [{"type": ")" << solver << R"("}],
  "integrators": [{"type": "ode113", "time_span": 0.001, "time_step": 0.001}]},
 "assemblies": [{"name": "a",
  "bodies": [{"name": "beam", "type": ")"
     << type << R"(", "geometry": [],
    "frames": [{"name": "tip", "T": [1.0, 0.0, 0.0]}]})";
  if(hanging) {
    ss << R"(,
   {"name": "c", "geometry": [], "center_of_mass": [0.0, 0.1, 0.2],
    "inertia": {"mass": 0.3, "Ixx": 0.004, "Iyy": 0.003, "Izz": 0.002,
                "Ixy": 0.0, "Iyz": 0.0005, "Izx": 0.0}})";
  }
  ss << R"(],
  "joints": [{"name": "root", "type": ")"
     << joint << R"(", "axis": [0, 1, 0], "initial_velocity": 0.7,
    "parent": [["fixed"], "origin"], "child": [["beam"], "origin"]})";
  if(hanging) {
    ss << R"(,
   {"name": "hinge", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.4, "initial_velocity": -1.2,
    "parent": [["beam"], "tip"], "child": [["c"], "origin"]})";
  }
  ss << "]}]}\n";
  return ss.str();
}

/* the undamped frequencies (rad/s) of the linearized model at rest */
static std::vector<double> NodalFrequencies(const std::string& model,
                                            const std::string& name) {
  auto context = LoadNodal(model, name);
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
      omega.push_back(mode.eigenvalue.imag());
    }
  }
  std::sort(omega.begin(), omega.end());
  return omega;
}

/* the stiffness reduced to the tip is the cantilever's: a unit force at the
tip along z bends it by L^3 / 3EI and turns it by -L^2 / 2EI about y, along
y by L^3 / 3EI and L^2 / 2EI about z, and along x stretches it by L / EA */
TEST(NodalBody, TipStiffnessIsTheCantilevers) {
  for(int elements : {1, 5, 20}) {
    TestNodalBeam beam(elements, elements);
    ASSERT_EQ(beam.get_elastic_count(), 6u);
    std::vector<double> k = beam.get_stiffness();
    ASSERT_EQ(ksl_linalg_cholesky_rmo(k.data(), 6), 0);
    const double l = TestNodalBeam::kLength, ei = TestNodalBeam::kEi;
    struct Load {
      int force;
      int deflection;
      double expected;
    };
    const Load loads[] = {{2, 2, l * l * l / (3.0 * ei)},
                          {2, 4, -l * l / (2.0 * ei)},
                          {1, 1, l * l * l / (3.0 * ei)},
                          {1, 5, l * l / (2.0 * ei)},
                          {0, 0, l / TestNodalBeam::kEa},
                          {3, 3, l / TestNodalBeam::kGj}};
    for(const Load& load : loads) {
      double f[6] = {}, x[6];
      f[load.force] = 1.0;
      ksl_linalg_cholesky_solve_rmo(k.data(), f, x, 6);
      EXPECT_NEAR(x[load.deflection], load.expected,
                  1e-10 * std::fabs(load.expected))
        << elements << " elements, force " << load.force << ", deflection "
        << load.deflection;
    }
  }
}

/* with every node kept, G is the identity on the degrees of freedom that are
not fixed, so the elastic mass matrix is the lumped one: the mass of each
point on its translations and its rotary inertia on its rotations */
TEST(NodalBody, KeptNodesHaveTheLumpedMassMatrix) {
  auto context = LoadNodal(NodalBeamModel("test_nodal_beam_full", "rigid"),
                           "nodal_lumped");
  ASSERT_EQ(context->get_elastic_bodies().size(), 1u);
  const auto body = context->get_active_articulated_bodies()
                      [context->get_elastic_bodies()[0]];
  const int n = static_cast<int>(body->get_elastic_count());
  ASSERT_EQ(n, 6 * 20);
  const double* mass_ee = body->get_mass_ee();
  const double h = TestNodalBeam::kLength / 20;
  for(int a = 0; a < n; a++) {
    const int point = 1 + a / 6, d = a % 6;
    const double m = TestNodalBeam::kMu * h * (point == 20 ? 0.5 : 1.0);
    const double i = m * h * h / 12.0;
    const double expected = d < 3 ? m : (d == 3 ? 2.0 * i : i);
    for(int b = 0; b < n; b++) {
      EXPECT_NEAR(mass_ee[a * n + b], a == b ? expected : 0.0, 1e-15)
        << a << " " << b;
    }
  }
}

/* clamped, with every node kept, the beam has the clamped-free frequencies
of both planes of bending, to the error of the lumped masses on 20 elements,
of the order of (beta_n h)^2 (0.16, 0.7 and 1.4 % for the first three
modes); kept at every fifth node, it is stiffer, each frequency higher but
near (Guyan reduction is a Rayleigh-Ritz reduction) */
TEST(NodalBody, ClampedBeamFrequencies) {
  const std::vector<double> full = NodalFrequencies(
    NodalBeamModel("test_nodal_beam_full", "rigid"), "nodal_full");
  const std::vector<double> reduced = NodalFrequencies(
    NodalBeamModel("test_nodal_beam_reduced", "rigid"), "nodal_reduced");
  ASSERT_GE(full.size(), 6u);
  ASSERT_GE(reduced.size(), 6u);
  for(int n = 0; n < 3; n++) {
    const double exact = TestNodalBeam::Omega(TestNodalBeam::kClamped[n]);
    for(int plane = 0; plane < 2; plane++) {
      const int k = 2 * n + plane;
      const double lumped[3] = {2e-3, 1e-2, 2e-2};
      EXPECT_NEAR(full[k], exact, lumped[n] * exact) << n;
      EXPECT_GE(reduced[k], full[k] * (1.0 - 1e-12)) << n;
      EXPECT_LT(reduced[k], full[k] * (n == 0 ? 1.001 : 1.05)) << n;
    }
  }
}

/* pinned, swinging, deformed at the tip, with a body hanging from it under
gravity: ABA and DCA give RNCA's state derivatives */
TEST(NodalBody, SolversAgree) {
  std::vector<std::vector<double>> derivatives;
  for(const std::string solver : {"rnca", "aba", "dca"}) {
    auto context = LoadNodal(
      NodalBeamModel("test_nodal_beam_tip", "revolute", solver, true, 9.81),
      "nodal_solvers_" + solver);
    context->get_active_integrators()[0]->Initialize();
    auto s = (*context)["model_parameters"]["solvers"][0]
               .get_data<shift::core::solver::Base>();
    s->Initialize();
    std::vector<double> y(context->get_state_count() +
                            context->get_auxiliary_state_count(),
                          0.0);
    std::copy(context->get_q().begin(), context->get_q().end(), y.begin());
    std::copy(context->get_dq(), context->get_dq() + context->get_dq_count(),
              y.begin() + context->get_q_count());
    std::vector<double> dy(y.size(), 0.0);
    ASSERT_FALSE(s->Solve(0.0, y, dy)) << solver;
    derivatives.push_back(dy);
  }
  double largest = 0.0;
  for(size_t k = 0; k < derivatives[0].size(); k++) {
    const double reference = derivatives[0][k];
    largest = std::fmax(largest, std::fabs(reference));
    for(size_t s = 1; s < derivatives.size(); s++) {
      EXPECT_NEAR(derivatives[s][k], reference,
                  1e-9 * std::fmax(1.0, std::fabs(reference)))
        << s << " " << k;
    }
  }
  EXPECT_GT(largest, 1.0);
}

/* the fixed points must hold the body still, and a point is fixed or
retained once */
TEST(NodalBody, RefusesAnIllPosedModel) {
  EXPECT_THROW(TestNodalBeam(4, 2, false, 0.0), shift::core::Exception);
  shift::core::body::Nodal body("bad");
  const std::vector<ksl_vec3_t> positions = {ksl_vec3(0, 0, 0),
                                             ksl_vec3(1, 0, 0)};
  const std::vector<ksl_mat3x3_t> rotary(2, ksl_mat3x3_t{});
  double k[12][12] = {};
  TestNodalBeam::Element(1.0, TestNodalBeam::kGj, k);
  std::vector<double> stiffness(&k[0][0], &k[0][0] + 144);
  EXPECT_THROW(body.SetFiniteElementModel(positions, {0.5, 0.5}, rotary, 6,
                                          stiffness, {}, {}, {1}),
               shift::core::Exception);
  EXPECT_THROW(body.SetFiniteElementModel(positions, {0.5, 0.5}, rotary, 6,
                                          stiffness, {}, {1}, {1}),
               shift::core::Exception);
  EXPECT_NO_THROW(body.SetFiniteElementModel(positions, {0.5, 0.5}, rotary, 6,
                                             stiffness, {}, {0}, {1}));
}
