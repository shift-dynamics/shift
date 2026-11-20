/* The interior point method for optimal control problems in stages
(controller::InteriorPoint): it finds the known solutions of a linear-quadratic
problem, of a constrained double integrator and of a nonlinear problem with two
active constraints */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "shift/core/controller/interior_point.h"

/* x_{k+1} = x_k + u_k from x_0 = 1, the cost sum u_k^2 + w x_N^2: each
input -w / (1 + N w), the same */
class Scalar : public shift::core::controller::InteriorPoint::Problem {
public:
  int get_states() const override {
    return 1;
  }
  int get_inputs() const override {
    return 1;
  }
  int get_steps() const override {
    return 10;
  }
  int get_constraints() const override {
    return 0;
  }
  int get_end_constraints() const override {
    return 0;
  }
  void Stage(int, const double* x, const double* u, double* next,
             double* cost, double*) const override {
    next[0] = x[0] + u[0];
    *cost = u[0] * u[0];
  }
  void End(const double* x, double* cost, double*) const override {
    *cost = 4.0 * x[0] * x[0];
  }
  double StateScale(int) const override {
    return 1.0;
  }
  double InputScale(int) const override {
    return 1.0;
  }
};

TEST(InteriorPoint, ItSolvesALinearQuadraticProblem) {
  const Scalar problem;
  shift::core::controller::InteriorPoint solver;
  std::vector<double> xs(11, 1.0), us(10, 0.0);
  ASSERT_TRUE(solver.Solve(problem, xs, us));
  const double u = -4.0 / (1.0 + 10.0 * 4.0);
  for(const double v : solver.get_inputs()) {
    EXPECT_NEAR(v, u, 1e-5);
  }
  EXPECT_NEAR(solver.get_states().back(), 1.0 + 10.0 * u, 1e-5);
}

/* the double integrator x = (p, v), dt 0.1, |u| <= 1 and v <= 0.5, from
rest, the farthest in 20 steps: full acceleration until the speed limit,
then held there */
class Integrator : public shift::core::controller::InteriorPoint::Problem {
public:
  int get_states() const override {
    return 2;
  }
  int get_inputs() const override {
    return 1;
  }
  int get_steps() const override {
    return 20;
  }
  int get_constraints() const override {
    return 3;
  }
  int get_end_constraints() const override {
    return 1;
  }
  void Stage(int, const double* x, const double* u, double* next,
             double* cost, double* c) const override {
    const double dt = 0.1;
    next[0] = x[0] + dt * x[1] + 0.5 * dt * dt * u[0];
    next[1] = x[1] + dt * u[0];
    *cost = 1.0e-6 * u[0] * u[0];
    c[0] = u[0] - 1.0;
    c[1] = -u[0] - 1.0;
    c[2] = x[1] - 0.5;
  }
  void End(const double* x, double* cost, double* c) const override {
    *cost = -x[0];
    c[0] = x[1] - 0.5;
  }
  double StateScale(int) const override {
    return 1.0;
  }
  double InputScale(int) const override {
    return 1.0;
  }
};

TEST(InteriorPoint, ItHoldsTheBoundsOfADoubleIntegrator) {
  const Integrator problem;
  shift::core::controller::InteriorPoint solver;
  std::vector<double> xs(42, 0.0), us(20, 0.0);
  ASSERT_TRUE(solver.Solve(problem, xs, us));
  /* 5 steps at full acceleration to 0.5 m/s (0.125 m), then 15 at 0.5 */
  const auto& x = solver.get_states();
  EXPECT_NEAR(x[2 * 20], 0.125 + 15 * 0.1 * 0.5, 1e-4);
  EXPECT_LE(x[2 * 20 + 1], 0.5 + 1e-6);
  for(int k = 0; k < 4; ++k) {
    EXPECT_NEAR(solver.get_inputs()[k], 1.0, 1e-3);
  }
}

/* one step, the inputs u: the least (u_1 - 2)^2 + (u_2 - 1)^2 with
u_1^2 <= u_2 and u_1 + u_2 <= 2, at (1, 1) with both active (R. Fletcher,
Practical Methods of Optimization, 1987) */
class Fletcher : public shift::core::controller::InteriorPoint::Problem {
public:
  int get_states() const override {
    return 1;
  }
  int get_inputs() const override {
    return 2;
  }
  int get_steps() const override {
    return 1;
  }
  int get_constraints() const override {
    return 2;
  }
  int get_end_constraints() const override {
    return 0;
  }
  void Stage(int, const double* x, const double* u, double* next,
             double* cost, double* c) const override {
    next[0] = x[0];
    *cost = (u[0] - 2.0) * (u[0] - 2.0) + (u[1] - 1.0) * (u[1] - 1.0);
    c[0] = u[0] * u[0] - u[1];
    c[1] = u[0] + u[1] - 2.0;
  }
  void End(const double*, double* cost, double*) const override {
    *cost = 0.0;
  }
  double StateScale(int) const override {
    return 1.0;
  }
  double InputScale(int) const override {
    return 1.0;
  }
};

TEST(InteriorPoint, ItFindsANonlinearProblemsActiveConstraints) {
  const Fletcher problem;
  shift::core::controller::InteriorPoint solver;
  std::vector<double> xs(2, 0.0), us = {0.0, 0.0};
  ASSERT_TRUE(solver.Solve(problem, xs, us));
  EXPECT_NEAR(solver.get_inputs()[0], 1.0, 1e-5);
  EXPECT_NEAR(solver.get_inputs()[1], 1.0, 1e-5);
  EXPECT_NEAR(solver.get_cost(), 1.0, 1e-5);
}
