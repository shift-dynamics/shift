/* A part of a model that no path of joints joins to the fixed body gets a
floating joint from the fixed body (Context::AddFloatingJoints), attached to
its body with the most joints. */

#include <array>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/topology.h"

/* bodies 0 and 1 hang from the fixed body (-1); 2, 3, 4 and 5 form a loop
with a branch, joined to nothing else, in which 3 has the most joints; 6 has
no joints at all */
TEST(FloatingJoint, FindFloatingRoots) {
  const std::vector<std::array<int, 2>> joints = {
    {-1, 0}, {0, 1}, {2, 3}, {3, 4}, {4, 2}, {5, 3}};
  const std::vector<int> roots =
    shift::core::topology::FindFloatingRoots(7, joints);
  EXPECT_EQ(roots, (std::vector<int>{3, 6}));

  /* every body reached: no floating parts */
  EXPECT_TRUE(
    shift::core::topology::FindFloatingRoots(2, {{-1, 0}, {1, -1}}).empty());

  /* equal numbers of joints: the lowest index */
  EXPECT_EQ(shift::core::topology::FindFloatingRoots(3, {{-1, 0}, {1, 2}}),
            (std::vector<int>{1}));
}

/* a pendulum on the fixed body and, joined to nothing, two bodies p and q
connected by a revolute joint: Shift adds _float_joint[0] from fixed to p (p
and q have one joint each; p comes first), and p falls freely from the origin
of fixed, 0.5 g t^2 along +z */
TEST(FloatingJoint, AddedToFloatingPart) {
  const double g = 9.81, t = 0.5;
  const std::string path = testing::TempDir() + "/floating_joint.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, )"
         << g << R"(], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
         << t << R"(, "time_step": 0.01, "relative_error": 1e-10,
                   "absolute_error": 1e-10}]},
 "assemblies": [{"name": "a",
  "bodies": [
   {"name": "arm", "geometry": [], "center_of_mass": [0.5, 0, 0],
    "inertia": {"mass": 2.0, "Ixx": 0.2, "Iyy": 0.2, "Izz": 0.2,
                "Ixy": 0, "Iyz": 0, "Izx": 0}},
   {"name": "p", "geometry": [], "center_of_mass": [0, 0, 0],
    "frames": [{"name": "end", "T": [1.0, 0.0, 0.0]}],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0}},
   {"name": "q", "geometry": [], "center_of_mass": [0.5, 0, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0}}],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 1, 0],
    "parent": [["fixed"], "origin"], "child": [["arm"], "origin"]},
   {"name": "J2", "type": "revolute", "axis": [0, 0, 1],
    "parent": [["p"], "end"], "child": [["q"], "origin"]}]}]}
)";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  const auto& joints = context.get_active_joints();
  ASSERT_EQ(joints.size(), 3u);
  int added = 0;
  for(auto joint : joints) {
    if(joint->get_name() == "_float_joint[0]") {
      added++;
      EXPECT_EQ(joint->get_type(), "float");
      EXPECT_EQ(joint->get_parent_body(), context.get_fixed_body());
      EXPECT_EQ(joint->get_child_body()->get_name(), "p");
      EXPECT_EQ(joint->get_level(), 1);
    }
  }
  EXPECT_EQ(added, 1);
  EXPECT_EQ(context.get_active_articulated_bodies().size(), 3u);

  context.Simulate();
  bool found = false;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "p") {
      found = true;
      const ksl_SE3_t pose = context.get_body_origin_pose(body);
      EXPECT_NEAR(pose.t.x, 0.0, 1e-8);
      EXPECT_NEAR(pose.t.y, 0.0, 1e-8);
      EXPECT_NEAR(pose.t.z, 0.5 * g * t * t, 1e-8);
    }
  }
  EXPECT_TRUE(found);
}
