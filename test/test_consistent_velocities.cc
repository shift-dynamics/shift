/* Initial velocities that violate the velocity constraints of a loop are made
consistent at load (Substructure::MakeVelocitiesConsistent, called by
solver::RNCA::Initialize after the coordinates are assembled): the given
nonzero rates are kept when their coordinates can be independent, and the
other rates follow from J dq = 0; rates that cannot be kept are replaced by
their projection onto the null space of J. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/jacobian.h"
#include "shift/core/joint/base.h"
#include "shift/core/substructure.h"

/* slider-crank in the y-z plane: crank r = 0.6 about x (A), piston along y
(B), rod l = 1.5 pinned to both (C, D); rates of A, B, C, D as given */
static std::string SliderCrank(const double rates[4]) {
  char text[4][32];
  for(int i = 0; i < 4; i++) {
    std::snprintf(text[i], sizeof(text[i]), "%.17g", rates[i]);
  }
  const std::string body = R"(, "type": "rigid", "geometry": [],
    "center_of_mass": [0.0, 0.1, 0.0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0})";
  return std::string(R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "rk4", "time_span": 0.1, "time_step": 0.01}]},
 "assemblies": [{"name": "slider_crank", "bodies": [
  {"name": "crank")") + body + R"(, "frames": [{"name": "tip", "T": [0, 0.6, 0]}]},
  {"name": "rod")" + body + R"(, "frames": [{"name": "end", "T": [0, 1.5, 0]}]},
  {"name": "piston")" + body + R"(, "frames": []}],
  "joints": [
   {"name": "A", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 1.0, "initial_velocity": )" + text[0] + R"(,
    "parent": [["fixed"], "origin"], "child": [["crank"], "origin"]},
   {"name": "B", "type": "prismatic", "axis": [0, 1, 0],
    "initial_displacement": 1.7, "initial_velocity": )" + text[1] + R"(,
    "parent": [["fixed"], "origin"], "child": [["piston"], "origin"]},
   {"name": "C", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": -1.3, "initial_velocity": )" + text[2] + R"(,
    "parent": [["crank"], "tip"], "child": [["rod"], "origin"]},
   {"name": "D", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.3, "initial_velocity": )" + text[3] + R"(,
    "parent": [["rod"], "end"], "child": [["piston"], "origin"]}]}]})";
}

struct Loaded {
  std::vector<double> rates; /* A, B, C, D */
  std::vector<double> q;     /* A, B, C, D */
  double residual = 0.0;
};

static Loaded Load(const double rates[4]) {
  const std::string path =
    testing::TempDir() + "/consistent_velocities.json";
  {
    std::ofstream file(path);
    file << SliderCrank(rates);
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  Loaded loaded;
  loaded.rates.resize(4);
  loaded.q.resize(4);
  for(auto joint : context.get_active_joints()) {
    const int k = joint->get_name()[0] - 'A';
    if(k < 0 || k > 3) continue;
    loaded.rates[k] = context.get_dq()[joint->get_dq_index()];
    loaded.q[k] = context.get_q()[joint->get_q_index()];
  }
  context.UpdateDH();
  for(auto& substructure : context.get_jacobian().get_substructures()) {
    loaded.residual = std::max(loaded.residual,
                               substructure.VelocityResidual(context.get_dq()));
  }
  return loaded;
}

/* the crank rate alone is given: it is kept, and the rest follow it */
TEST(ConsistentVelocities, CrankRateKept) {
  const double given[4] = {1.0, 0.0, 0.0, 0.0};
  const Loaded loaded = Load(given);
  EXPECT_LT(loaded.residual, 1e-12);
  EXPECT_DOUBLE_EQ(loaded.rates[0], 1.0);
  /* closed form: alpha = theta + phi, sin(alpha) = -(r/l) sin(theta),
  dalpha = -r cos(theta) dtheta / (l cos(alpha)), ds = -r sin(theta) dtheta
  - l sin(alpha) dalpha, dpsi = -dalpha */
  const double r = 0.6, l = 1.5, theta = loaded.q[0];
  const double alpha = theta + loaded.q[2];
  const double dalpha = -r * std::cos(theta) / (l * std::cos(alpha));
  EXPECT_NEAR(loaded.rates[1],
              -r * std::sin(theta) - l * std::sin(alpha) * dalpha, 1e-9);
  EXPECT_NEAR(loaded.rates[2], dalpha - 1.0, 1e-9);
  EXPECT_NEAR(loaded.rates[3], -dalpha, 1e-9);
}

/* rates given to every coordinate, inconsistent: replaced by the nearest
consistent rates, whose difference from the given ones is normal to them */
TEST(ConsistentVelocities, ProjectedWhenAllGiven) {
  const double given[4] = {1.0, 0.5, -0.3, 0.2};
  const Loaded loaded = Load(given);
  EXPECT_LT(loaded.residual, 1e-12);
  double dot = 0.0, norm = 0.0;
  for(int k = 0; k < 4; k++) {
    dot += (given[k] - loaded.rates[k]) * loaded.rates[k];
    norm += loaded.rates[k] * loaded.rates[k];
  }
  EXPECT_GT(norm, 1e-6);
  EXPECT_NEAR(dot, 0.0, 1e-12);
}

/* consistent rates are left as they are */
TEST(ConsistentVelocities, ConsistentRatesUnchanged) {
  const double crank[4] = {0.7, 0.0, 0.0, 0.0};
  const Loaded first = Load(crank);
  const double consistent[4] = {first.rates[0], first.rates[1],
                                first.rates[2], first.rates[3]};
  const Loaded second = Load(consistent);
  for(int k = 0; k < 4; k++) {
    EXPECT_DOUBLE_EQ(second.rates[k], consistent[k]);
  }
}
