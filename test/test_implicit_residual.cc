/* The equations that the implicit integrators solve, for RNCA. The residual
(solver::Base::Residual) is zero in the rows of the generalized velocities at
the accelerations and multipliers of a solve, and in the rows of the
multipliers it is the Newton-Raphson corrections of the dependent coordinates,
whose derivative with respect to the coordinates is [B_di I]. The iteration
matrix (FactorIterationMatrix, SolveIterationMatrix) solves
[M + cC + kK  B^T; B  0] x = rhs. */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <json/json.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/body/base.h"
#include "shift/core/solver/base.h"
#include "shift/core/tangents.h"

/* the example model given, simulated briefly without loggers */
static Json::Value Model(const std::string& name) {
  std::ifstream file(std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  Json::Value model;
  Json::CharReaderBuilder builder;
  std::string errors;
  EXPECT_TRUE(Json::parseFromStream(builder, file, &model, &errors)) << errors;
  Json::Value& parameters = model["model_parameters"];
  parameters["integrators"][0]["time_span"] = 0.05;
  parameters["integrators"][0]["time_step"] = 0.01;
  parameters["loggers"] = Json::Value(Json::arrayValue);
  return model;
}

static double LargestMagnitude(const std::vector<double>& v) {
  double largest = 0.0;
  for(double x : v) {
    largest = std::max(largest, std::fabs(x));
  }
  return largest;
}

/* load the example model given into context and simulate it briefly; the
time and state at the end in t and y */
static void Run(const std::string& name, shift::core::Context& context,
                double& t, std::vector<double>& y) {
  context.LoadJSON(Model(name), std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  context.set_state_observer([&](double time, const std::vector<double>& s) {
    t = time;
    y = s;
  });
  context.Simulate();
  ASSERT_FALSE(y.empty());
}

/* the matrix [B_di I] at the state of the last Residual, in velocity order,
one row per multiplier: a dependent row of B is a 1 at its own variable and
the entries of b3 at the independent ones */
static std::vector<double> ConstraintMatrix(shift::core::Context& context,
                                            unsigned int multiplier_count) {
  const unsigned int dq_count = context.get_dq_count();
  const auto& jacobian = context.get_jacobian();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  const auto& pv_13 = jacobian.get_pv13();
  const auto& rb3 = jacobian.get_rb3();
  const std::vector<int> cb3 = jacobian.get_cb3();
  const auto& b3 = jacobian.get_b3();
  std::vector<double> b(size_t(multiplier_count) * dq_count, 0.0);
  unsigned int m = 0;
  for(unsigned int i = 0; i < dq_count; i++) {
    if(is_dependent_po3[i]) {
      b[size_t(m) * dq_count + pv_13[i]] = 1.0;
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        b[size_t(m) * dq_count + pv_13[cb3[j]]] = b3[j];
      }
      m++;
    }
  }
  EXPECT_EQ(m, multiplier_count);
  return b;
}

static void CheckResidual(const std::string& name) {
  SCOPED_TRACE(name);
  shift::core::Context context;
  double t = 0.0;
  std::vector<double> y;
  ASSERT_NO_FATAL_FAILURE(Run(name, context, t, y));

  shift::core::solver::Base* solver = context.get_active_solvers()[0];
  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();

  std::vector<double> dy(y.size(), 0.0);
  ASSERT_FALSE(solver->Solve(t, y, dy));
  const std::vector<double> ddq(dy.begin() + q_count,
                                dy.begin() + q_count + dq_count);
  std::vector<double> la;
  solver->Multipliers(la);
  const unsigned int multiplier_count = solver->get_multiplier_count();
  ASSERT_EQ(la.size(), multiplier_count);
  ASSERT_GT(multiplier_count, 0u) << "the model has no kinematic loop";

  /* the rows of the velocities vanish at the solution of the solve */
  std::vector<double> r;
  std::vector<double> dy_residual(y.size(), 0.0);
  ASSERT_FALSE(solver->Residual(t, y, ddq, la, r, dy_residual));
  ASSERT_EQ(r.size(), dq_count + multiplier_count);
  const double scale = 1.0 + LargestMagnitude(ddq) + LargestMagnitude(la);
  for(unsigned int i = 0; i < dq_count; i++) {
    EXPECT_NEAR(r[i], 0.0, 1e-9 * scale) << "velocity " << i;
  }
  /* and the rows of the multipliers are the small position errors left by
  the integration */
  for(unsigned int m = 0; m < multiplier_count; m++) {
    EXPECT_LT(std::fabs(r[dq_count + m]), 1e-5) << "multiplier " << m;
  }

  /* central differences of the multiplier rows along each velocity: the
  change of q at a unit velocity of that coordinate (quaternions by their
  rate) */
  const double step = 1e-6;
  std::vector<double> derivative(size_t(multiplier_count) * dq_count);
  std::vector<double> rates(y.size());
  for(unsigned int k = 0; k < dq_count; k++) {
    std::vector<double> unit(y);
    std::fill(unit.begin() + q_count, unit.begin() + q_count + dq_count, 0.0);
    unit[q_count + k] = 1.0;
    context.ConvertdqTody(unit, rates);
    std::vector<double> plus(y), minus(y);
    for(unsigned int i = 0; i < q_count; i++) {
      plus[i] += step * rates[i];
      minus[i] -= step * rates[i];
    }
    std::vector<double> r_plus, r_minus;
    ASSERT_FALSE(solver->Residual(t, plus, ddq, la, r_plus, dy_residual));
    ASSERT_FALSE(solver->Residual(t, minus, ddq, la, r_minus, dy_residual));
    for(unsigned int m = 0; m < multiplier_count; m++) {
      derivative[size_t(m) * dq_count + k] =
        (r_plus[dq_count + m] - r_minus[dq_count + m]) / (2.0 * step);
    }
  }

  /* [B_di I] at y */
  ASSERT_FALSE(solver->Residual(t, y, ddq, la, r, dy_residual));
  const std::vector<double> expected =
    ConstraintMatrix(context, multiplier_count);
  const double b_scale = 1.0 + LargestMagnitude(expected);
  for(unsigned int row = 0; row < multiplier_count; row++) {
    for(unsigned int k = 0; k < dq_count; k++) {
      EXPECT_NEAR(derivative[size_t(row) * dq_count + k],
                  expected[size_t(row) * dq_count + k], 1e-5 * b_scale)
        << "multiplier " << row << ", velocity " << k;
    }
  }
}

TEST(ImplicitResidual, SliderCrank) {
  CheckResidual("slider_crank/slider_crank.json");
}

TEST(ImplicitResidual, Bricard) {
  CheckResidual("bricard/bricard.json");
}

TEST(ImplicitResidual, KlannLinkage) {
  CheckResidual("klann_linkage/klann_linkage.json");
}

/* the iteration matrix with random tangents (fixed seed): entries between
velocities, some of them not symmetric, spring-like blocks between every pair
of bodies, which have entries outside the pattern of the mass matrix, and one
column found by finite differences in place of the tangents */
static void CheckIterationMatrix(const std::string& name) {
  SCOPED_TRACE(name);
  shift::core::Context context;
  double t = 0.0;
  std::vector<double> y;
  ASSERT_NO_FATAL_FAILURE(Run(name, context, t, y));

  shift::core::solver::Base* solver = context.get_active_solvers()[0];
  const unsigned int q_count = context.get_q_count();
  const unsigned int dq_count = context.get_dq_count();
  std::vector<double> dy(y.size(), 0.0);
  ASSERT_FALSE(solver->Solve(t, y, dy));
  const std::vector<double> dy_before(dy);
  const unsigned int multiplier_count = solver->get_multiplier_count();
  ASSERT_GT(multiplier_count, 0u);
  const unsigned int size = dq_count + multiplier_count;

  std::mt19937 generator(12345);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  const double c = 0.2;
  const double k = 0.3;

  /* A = cC + kK, dense, velocity order, as the test expects it */
  std::vector<double> a(size_t(dq_count) * dq_count, 0.0);
  shift::core::Tangents tangents;
  for(unsigned int i = 0; i < dq_count; i++) {
    const double stiffness = 2.0 + uniform(generator);
    const double damping = 1.0 + 0.5 * uniform(generator);
    tangents.AddJointStiffness(i, i, stiffness);
    tangents.AddJointDamping(i, i, damping);
    a[size_t(i) * dq_count + i] += k * stiffness + c * damping;
    if(i + 1 < dq_count) {
      /* (i, i + 1) and (i + 1, i) differ: not symmetric */
      const double upper = 0.3 * uniform(generator);
      const double lower = 0.3 * uniform(generator);
      tangents.AddJointStiffness(i, i + 1, upper);
      tangents.AddJointStiffness(i + 1, i, lower);
      a[size_t(i) * dq_count + i + 1] += k * upper;
      a[size_t(i + 1) * dq_count + i] += k * lower;
    }
  }

  /* the velocity Jacobian of each body, from its twist at unit velocities
  (Context::GetBodyVelocity), 6 rows by dq_count columns */
  std::set<int> load_set;
  for(auto* body : context.get_active_articulated_bodies()) {
    const int load_index = context.GetBodyLoadIndex(body);
    if(load_index > -1) {
      load_set.insert(load_index);
    }
  }
  const std::vector<int> loads(load_set.begin(), load_set.end());
  ASSERT_GE(loads.size(), 2u);
  std::vector<std::vector<double>> jacobians(loads.size());
  for(size_t b = 0; b < loads.size(); b++) {
    jacobians[b].assign(6 * size_t(dq_count), 0.0);
  }
  for(unsigned int l = 0; l < dq_count; l++) {
    std::vector<double> unit(dq_count, 0.0);
    unit[l] = 1.0;
    context.UpdateDHVA(y.data(), unit.data());
    for(size_t b = 0; b < loads.size(); b++) {
      const ksl_screw_t twist = context.GetBodyVelocity(loads[b]);
      for(int m = 0; m < 6; m++) {
        jacobians[b][size_t(m) * dq_count + l] = twist.at[m];
      }
    }
  }
  /* A += scale J_a^T K_ab J_b */
  auto add_block = [&](size_t ia, size_t ib, const double block[36],
                       double scale) {
    for(unsigned int l = 0; l < dq_count; l++) {
      for(unsigned int col = 0; col < dq_count; col++) {
        double value = 0.0;
        for(int row6 = 0; row6 < 6; row6++) {
          for(int col6 = 0; col6 < 6; col6++) {
            value += jacobians[ia][size_t(row6) * dq_count + l] *
                     block[row6 * 6 + col6] *
                     jacobians[ib][size_t(col6) * dq_count + col];
          }
        }
        a[size_t(l) * dq_count + col] += scale * value;
      }
    }
  };
  double identity[36] = {};
  double minus_identity[36] = {};
  for(int m = 0; m < 6; m++) {
    identity[m * 6 + m] = 0.5;
    minus_identity[m * 6 + m] = -0.5;
  }
  for(size_t ia = 0; ia < loads.size(); ia++) {
    for(size_t ib = ia + 1; ib < loads.size(); ib++) {
      tangents.AddBodyStiffness(loads[ia], loads[ia], identity);
      tangents.AddBodyStiffness(loads[ib], loads[ib], identity);
      tangents.AddBodyStiffness(loads[ia], loads[ib], minus_identity);
      tangents.AddBodyStiffness(loads[ib], loads[ia], minus_identity);
      add_block(ia, ia, identity, k);
      add_block(ib, ib, identity, k);
      add_block(ia, ib, minus_identity, k);
      add_block(ib, ia, minus_identity, k);
    }
  }
  tangents.AddBodyDamping(loads[0], loads[0], identity);
  add_block(0, 0, identity, c);

  /* one column by finite differences, which takes the place of the
  tangents in it */
  const std::vector<int> differenced = {0};
  std::vector<double> k_columns(dq_count), c_columns(dq_count);
  for(unsigned int i = 0; i < dq_count; i++) {
    k_columns[i] = (i == 0 ? 1.0 : 0.1 * uniform(generator));
    c_columns[i] = (i == 0 ? 0.5 : 0.1 * uniform(generator));
    a[size_t(i) * dq_count + 0] = k * k_columns[i] + c * c_columns[i];
  }

  /* the equations at y again (UpdateDHVA above changed the velocities) */
  const std::vector<double> zero_ddq(dq_count, 0.0);
  const std::vector<double> zero_la(multiplier_count, 0.0);
  std::vector<double> r0;
  std::vector<double> dy_residual(y.size(), 0.0);
  ASSERT_FALSE(solver->Residual(t, y, zero_ddq, zero_la, r0, dy_residual));
  const std::vector<double> b = ConstraintMatrix(context, multiplier_count);
  ASSERT_FALSE(solver->FactorIterationMatrix(tangents, c, k, differenced,
                                             k_columns, c_columns));

  std::vector<double> rhs(size);
  for(double& value : rhs) {
    value = uniform(generator);
  }
  std::vector<double> x(rhs);
  ASSERT_FALSE(solver->SolveIterationMatrix(x));

  /* the factors are saved: the same solution after a Residual at another
  state */
  std::vector<double> y_other(y);
  for(unsigned int i = q_count; i < q_count + dq_count; i++) {
    y_other[i] += 0.1;
  }
  std::vector<double> r_other;
  ASSERT_FALSE(
    solver->Residual(t, y_other, zero_ddq, zero_la, r_other, dy_residual));
  std::vector<double> x_again(rhs);
  ASSERT_FALSE(solver->SolveIterationMatrix(x_again));
  for(unsigned int i = 0; i < size; i++) {
    EXPECT_DOUBLE_EQ(x_again[i], x[i]) << "entry " << i;
  }

  /* S x = rhs: M x_v + B^T x_la from the Residual, which is linear in the
  accelerations and the multipliers, A (not symmetric), and B x_v */
  const std::vector<double> x_v(x.begin(), x.begin() + dq_count);
  const std::vector<double> x_la(x.begin() + dq_count, x.end());
  std::vector<double> r1;
  ASSERT_FALSE(solver->Residual(t, y, x_v, x_la, r1, dy_residual));
  ASSERT_FALSE(solver->Residual(t, y, zero_ddq, zero_la, r0, dy_residual));
  double scale = 1.0;
  for(unsigned int i = 0; i < dq_count; i++) {
    double row = r1[i] - r0[i];
    double row_scale = std::fabs(r1[i]) + std::fabs(r0[i]);
    for(unsigned int j = 0; j < dq_count; j++) {
      const double a_ij = a[size_t(i) * dq_count + j];
      row += a_ij * x_v[j];
      row_scale += std::fabs(a_ij * x_v[j]);
    }
    scale = std::max(scale, row_scale);
    EXPECT_NEAR(row, rhs[i], 1e-9 * (1.0 + row_scale)) << "velocity " << i;
  }
  for(unsigned int m = 0; m < multiplier_count; m++) {
    double row = 0.0;
    double row_scale = 0.0;
    for(unsigned int j = 0; j < dq_count; j++) {
      row += b[size_t(m) * dq_count + j] * x_v[j];
      row_scale += std::fabs(b[size_t(m) * dq_count + j] * x_v[j]);
    }
    EXPECT_NEAR(row, rhs[dq_count + m], 1e-9 * (1.0 + row_scale))
      << "multiplier " << m;
  }

  /* the explicit solve is unchanged by the iteration matrix, which keeps its
  own pattern and factors */
  std::vector<double> dy_after(y.size(), 0.0);
  ASSERT_FALSE(solver->Solve(t, y, dy_after));
  for(size_t i = 0; i < dy_before.size(); i++) {
    EXPECT_NEAR(dy_after[i], dy_before[i],
                1e-10 * (1.0 + std::fabs(dy_before[i])))
      << "dy " << i;
  }
}

TEST(ImplicitIterationMatrix, SliderCrank) {
  CheckIterationMatrix("slider_crank/slider_crank.json");
}

TEST(ImplicitIterationMatrix, Bricard) {
  CheckIterationMatrix("bricard/bricard.json");
}

TEST(ImplicitIterationMatrix, KlannLinkage) {
  CheckIterationMatrix("klann_linkage/klann_linkage.json");
}
