/* The criteria by which the RNCA solver refactors the Jacobian
(refactor_criterion): the infinity norm of B, the order of the mobility
numbers, and the mobility numbers of the independent velocities falling below
epsilon_mobility */

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <json/json.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"

/* the example model given, simulated for the time given with the solver
options given and without loggers */
static Json::Value Model(const std::string& name, double time_span,
                         const Json::Value& solver) {
  std::ifstream file(std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  Json::Value model;
  Json::CharReaderBuilder builder;
  std::string errors;
  EXPECT_TRUE(Json::parseFromStream(builder, file, &model, &errors)) << errors;
  Json::Value& parameters = model["model_parameters"];
  parameters["solvers"] = Json::Value(Json::arrayValue);
  parameters["solvers"].append(solver);
  parameters["integrators"][0]["time_span"] = time_span;
  parameters["loggers"] = Json::Value(Json::arrayValue);
  return model;
}

static Json::Value Solver(const std::string& criterion,
                          double epsilon_mobility = 0.01) {
  Json::Value solver;
  solver["type"] = "rnca";
  solver["refactor_criterion"] = criterion;
  solver["epsilon_mobility"] = epsilon_mobility;
  return solver;
}

struct Result {
  std::vector<double> q;
  size_t refactors = 0;
};

static Result Simulate(const std::string& name, double time_span,
                    const Json::Value& solver) {
  shift::core::Context context;
  context.LoadJSON(Model(name, time_span, solver),
                   std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  context.Simulate();
  Result run;
  run.q = context.get_q();
  run.refactors = context.get_jacobian().get_infinity_norm_failures().size();
  return run;
}

/* every criterion simulates the same motion: refactoring changes the
partition, not the solution */
TEST(RefactorCriterion, SameMotion) {
  for(const std::string name :
      {"slider_crank/slider_crank.json", "klann_linkage/klann_linkage.json"}) {
    const Result norm = Simulate(name, 4.0, Solver("infinity_norm"));
    for(const std::string criterion : {"mobility_order", "mobility_epsilon"}) {
      const Result run = Simulate(name, 4.0, Solver(criterion, 0.01));
      ASSERT_EQ(run.q.size(), norm.q.size());
      for(size_t i = 0; i < run.q.size(); i++) {
        EXPECT_NEAR(run.q[i], norm.q[i], 1e-2)
          << name << " " << criterion << " q[" << i << "]";
      }
    }
  }
}

/* the order of the mobility numbers changes as the mechanisms move, so they
are refactored, but not at every check */
TEST(RefactorCriterion, MobilityOrder) {
  for(const std::string name :
      {"slider_crank/slider_crank.json", "klann_linkage/klann_linkage.json",
       "bricard/bricard.json"}) {
    const Result order = Simulate(name, 30.0, Solver("mobility_order"));
    EXPECT_GT(order.refactors, 0u) << name;
    EXPECT_LT(order.refactors, 500u) << name;
  }
}

/* an epsilon of 0 never refactors, and the larger it is the more it does */
TEST(RefactorCriterion, MobilityEpsilon) {
  const std::string name = "bricard/bricard.json";
  const Result never = Simulate(name, 30.0, Solver("mobility_epsilon", 0.0));
  const Result fewer = Simulate(name, 30.0, Solver("mobility_epsilon", 0.01));
  const Result more = Simulate(name, 30.0, Solver("mobility_epsilon", 0.1));
  EXPECT_EQ(never.refactors, 0u);
  EXPECT_GT(fewer.refactors, 0u);
  EXPECT_GE(more.refactors, fewer.refactors);
}

/* the run-time mobility numbers each lie between 0 and 1 and sum to the
mobility of the substructure */
TEST(RefactorCriterion, MobilityNumbers) {
  shift::core::Context context;
  const std::string name = "klann_linkage/klann_linkage.json";
  context.LoadJSON(Model(name, 0.5, Solver("mobility_order")),
                   std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  context.Simulate();
  int checked = 0;
  for(auto& substructure : context.get_jacobian().get_substructures()) {
    const int rank = substructure.get_rank();
    const int count = substructure.get_dq_count();
    if(rank <= 0 || rank >= count) {
      continue;
    }
    const std::vector<double>& mobility = substructure.RunTimeMobility();
    double sum = 0.0;
    for(double m : mobility) {
      EXPECT_GE(m, -1e-12);
      EXPECT_LE(m, 1.0 + 1e-12);
      sum += m;
    }
    EXPECT_NEAR(sum, count - rank, 1e-9);
    checked++;
  }
  EXPECT_GT(checked, 0);
}

TEST(RefactorCriterion, Unknown) {
  shift::core::Context context;
  const std::string name = "slider_crank/slider_crank.json";
  context.LoadJSON(Model(name, 0.1, Solver("pivot")),
                   std::string(SHIFT_EXAMPLES_DIR) + "/" + name);
  EXPECT_THROW(context.Simulate(), shift::core::Exception);
}
