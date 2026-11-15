/* A loop whose chord is closed by a half-turn: the skew-symmetric part of the
relative rotation is zero and the translation is zero, so an error made of
those alone would accept the model as assembled. The load-time
Newton-Raphson iteration (Substructure::RemoveConstraintViolations) includes
trace(R) - 3 in the angular errors and closes the loop. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/joint/base.h"

/* body b hangs from the fixed body by two revolute joints about the same z
axis through the origin: A at pi and B at 0, so that B, the chord, sees a
relative rotation of pi */
TEST(HalfTurnLoop, ClosedAtLoad) {
  const std::string path = testing::TempDir() + "/half_turn_loop.json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "rk4", "time_span": 0.1, "time_step": 0.01}]},
 "assemblies": [{"name": "a",
  "bodies": [
   {"name": "b", "geometry": [], "center_of_mass": [0.5, 0.2, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0}}],
  "joints": [
   {"name": "A", "type": "revolute", "axis": [0, 0, 1],
    "initial_displacement": 3.141592653589793,
    "parent": [["fixed"], "origin"], "child": [["b"], "origin"]},
   {"name": "B", "type": "revolute", "axis": [0, 0, 1],
    "initial_displacement": 0.0,
    "parent": [["fixed"], "origin"], "child": [["b"], "origin"]}]}]}
)";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  ASSERT_EQ(context.get_chord_constraints().size(), 1u);
  double q_a = 0.0, q_b = 0.0;
  for(auto joint : context.get_active_joints()) {
    if(joint->get_name() == "A") q_a = context.get_q()[joint->get_q_index()];
    if(joint->get_name() == "B") q_b = context.get_q()[joint->get_q_index()];
  }
  /* the two angles agree modulo a full turn */
  EXPECT_NEAR(std::cos(q_a - q_b), 1.0, 1e-9);
  EXPECT_NEAR(std::sin(q_a - q_b), 0.0, 1e-9);
}
