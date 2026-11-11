/* Soil tool force module (force_module::SoilTool): McKyes' factors against
earth-pressure theory, the draft and lift of a blade, and its pile */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/soil_tool.h"

using shift::core::force_module::SoilProperties;
using shift::core::force_module::SoilTool;

TEST(SoilTool, AVerticalSmoothBladeIsRankinesPassivePressure) {
  const double phi = 30.0 * M_PI / 180.0;
  const SoilTool::Factors n = SoilTool::ComputeFactors(0.5 * M_PI, phi, 0.0);
  SoilProperties soil;
  soil.friction_angle = phi;
  const double k = soil.PassiveCoefficient();
  EXPECT_NEAR(2.0 * n.weight, k, 1e-9);
  EXPECT_NEAR(n.cohesion, 2.0 * std::sqrt(k), 1e-6);
  EXPECT_NEAR(n.failure_angle, 0.25 * M_PI - 0.5 * phi, 1e-6);
}

TEST(SoilTool, AVerticalRoughBladeIsCoulombsPassivePressure) {
  // the thrust along its line: the coefficient of the normal part over
  // cos(delta)
  SoilProperties soil;
  soil.friction_angle = 30.0 * M_PI / 180.0;
  soil.wall_friction = 15.0 * M_PI / 180.0;
  const SoilTool::Factors n = SoilTool::ComputeFactors(
    0.5 * M_PI, soil.friction_angle, soil.wall_friction);
  EXPECT_NEAR(2.0 * n.weight,
              soil.PassiveCoefficient() / std::cos(soil.wall_friction), 1e-9);
}

TEST(SoilTool, ALeaningBladeCutsMoreEasily) {
  const double phi = 0.5, delta = 0.25;
  const double upright =
    SoilTool::ComputeFactors(0.5 * M_PI, phi, delta).weight;
  const double leaning =
    SoilTool::ComputeFactors(0.25 * M_PI, phi, delta).weight;
  EXPECT_LT(leaning, upright);
}

TEST(SoilTool, BackwardsRakeStaysFinite) {
  const SoilTool::Factors n = SoilTool::ComputeFactors(3.0, 0.6, 0.3);
  EXPECT_TRUE(std::isfinite(n.weight));
  EXPECT_GT(n.weight, 0.0);
}

/* a body on a slider along x, at the height z of its blade's edge */
static std::string Model(double z, double rake, double speed,
                         double time_span) {
  std::ostringstream s;
  s.precision(17);
  s << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 0], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
    << time_span << R"(, "time_step": 0.01, "relative_error": 1e-8,
                         "absolute_error": 1e-10}]},
    "frames": [{"name": "start", "T": [0, 0, )" << z << R"(]}],
    "assemblies": [{"name": "a",
    "bodies": [{"name": "dozer", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 1e9, "Ixx": 1e9, "Iyy": 1e9, "Izz": 1e9, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "j", "type": "prismatic",
      "parent": [["fixed"], "start"], "child": [["dozer"], "origin"],
      "axis": [1, 0, 0], "initial_velocity": )" << speed << R"(}],
    "force_modules": [{"type": "soil_tool", "name": "blade",
      "soil": {"name": "sandy_loam", "cohesion": 2000, "friction_angle": 0.5,
               "unit_weight": 16000, "wall_friction": 0.25},
      "body": "dozer", "tip": [0, 0, 0], "width": 3, "height": 1,
      "face": [)" << -std::cos(rake) << R"(, 0, )" << -std::sin(rake)
    << R"(], "slip_speed": 1e-4}]}]})";
  return s.str();
}

class SoilToolTest : public testing::Test {
protected:
  void Load(double z, double rake, double speed, double time_span = 0.0) {
    const std::string path = testing::TempDir() + "/soil_tool.json";
    {
      std::ofstream file(path);
      file << Model(z, rake, speed, time_span);
    }
    context_.LoadJSON(path);
    std::remove(path.c_str());
    context_.UpdateDHVA();
    for(auto module : context_.get_active_force_modules()) {
      if(auto m = dynamic_cast<SoilTool*>(module)) {
        tool_ = m;
      }
    }
    ASSERT_NE(tool_, nullptr);
  }

  /* the outputs at the given pile */
  void Evaluate(double pile = 0.0) {
    std::vector<double> y(context_.get_q().begin(), context_.get_q().end());
    y.resize(y.size() + context_.get_auxiliary_state_count(), 0.0);
    y[tool_->get_state_offset()] = pile;
    tool_->UpdateOutputs(0.0, y.data());
  }

  shift::core::Context context_;
  SoilTool* tool_ = nullptr;
};

TEST_F(SoilToolTest, DraftAndLiftFollowTheEquation) {
  const double rake = 1.0, depth = 0.2;
  Load(depth, rake, 2.0);
  Evaluate();
  const auto n = SoilTool::ComputeFactors(rake, 0.5, 0.25);
  const double f =
    3.0 * (16000.0 * depth * depth * n.weight + 2000.0 * depth * n.cohesion);
  EXPECT_NEAR(tool_->get_depth(), depth, 1e-12);
  EXPECT_NEAR(tool_->get_draft(), f * std::sin(rake + 0.25), 1e-3 * f);
  // a leaning blade is pulled into the soil
  EXPECT_NEAR(tool_->get_lift(), -f * std::cos(rake + 0.25), 1e-3 * f);
}

TEST_F(SoilToolTest, APileAddsItsWeight) {
  Load(0.2, 1.0, 2.0);
  Evaluate();
  const double bare = tool_->get_draft();
  Evaluate(0.5);
  const auto n = SoilTool::ComputeFactors(1.0, 0.5, 0.25);
  EXPECT_NEAR(tool_->get_draft() - bare,
              3.0 * 16000.0 * 0.5 * n.surcharge * std::sin(1.25), 1.0);
}

TEST_F(SoilToolTest, NoForceAboveTheGround) {
  Load(-0.1, 1.0, 2.0);
  Evaluate();
  EXPECT_EQ(tool_->get_depth(), 0.0);
  EXPECT_EQ(tool_->get_draft(), 0.0);
}

TEST_F(SoilToolTest, NoForceBackingUp) {
  Load(0.2, 1.0, -2.0);
  Evaluate();
  EXPECT_EQ(tool_->get_draft(), 0.0);
}

TEST_F(SoilToolTest, ThePileGrowsWithTheSoilCutAndFills) {
  // 1 m/s at 0.1 m: 0.1 m^2 a second, up to a heap of 1 / (2 tan(0.5))
  // = 0.915 m^2, and on past it only as fast as it spills
  Load(0.1, 0.5 * M_PI, 1.0, 2.0);
  double pile = 0.0;
  context_.set_state_observer([&](double, const std::vector<double>& y) {
    pile = y[tool_->get_state_offset()];
  });
  context_.Simulate();
  EXPECT_NEAR(pile, 0.2, 1e-3);
}

TEST_F(SoilToolTest, ThePileStopsAtAFullHeap) {
  Load(0.1, 0.5 * M_PI, 1.0, 30.0);
  double pile = 0.0;
  context_.set_state_observer([&](double, const std::vector<double>& y) {
    pile = y[tool_->get_state_offset()];
  });
  context_.Simulate();
  // a balance of the soil cut and spilled, after spill_time
  EXPECT_NEAR(pile, 0.5 / std::tan(0.5) + 0.1, 1e-3);
}
