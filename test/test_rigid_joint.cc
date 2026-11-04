/* A rigid joint below a moving body: the frames of its child are placed on
the parent body of the rigid joint, not on the body above it, and the child
moves with that body. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"

/* fixed -revolute z- arm -rigid at the tip- b -revolute z- c: the arm turns
at w, so the origin of c, at the tip, is Rz(w t) (1, 0, 0.2) */
TEST(RigidJoint, ChildFollowsMovingParent) {
  const double w = 2.0, t = 0.5;
  const std::string path = testing::TempDir() + "/rigid_joint.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
         << t << R"(, "time_step": 0.01, "relative_error": 1e-10,
                   "absolute_error": 1e-10}]},
 "assemblies": [{"name": "a",
  "bodies": [
   {"name": "arm", "geometry": [], "center_of_mass": [0.5, 0, 0],
    "frames": [{"name": "tip", "T": [1.0, 0.0, 0.2]}],
    "inertia": {"mass": 2.0, "Ixx": 0.2, "Iyy": 0.2, "Izz": 0.2,
                "Ixy": 0, "Iyz": 0, "Izx": 0}},
   {"name": "b", "geometry": [], "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0}},
   {"name": "c", "geometry": [], "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0}}],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 0, 1],
    "initial_velocity": )"
         << w << R"(,
    "parent": [["fixed"], "origin"], "child": [["arm"], "origin"]},
   {"name": "J2", "type": "rigid",
    "parent": [["arm"], "tip"], "child": [["b"], "origin"]},
   {"name": "J3", "type": "revolute", "axis": [0, 0, 1],
    "parent": [["b"], "origin"], "child": [["c"], "origin"]}]}]}
)";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.Simulate();
  bool found = false;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "c") {
      found = true;
      const ksl_SE3_t pose = context.get_body_origin_pose(body);
      /* the system turns as one about J1 (J3 carries no torque) */
      EXPECT_NEAR(pose.t.x, std::cos(w * t), 1e-8);
      EXPECT_NEAR(pose.t.y, std::sin(w * t), 1e-8);
      EXPECT_NEAR(pose.t.z, 0.2, 1e-8);
    }
  }
  EXPECT_TRUE(found);
}

/* A loop closing onto a body attached by a rigid joint, on a moving base:
a strut suspension corner (the front of shift-models/vaz_2108). The lower
strut tube is welded to the carrier by a rigid joint, and the loop through
the strut closes onto it. The top mount slides along x, but the loop fixes it
at q = 0; if the tube took the velocity of the fixed body instead of the
carrier's, the mount drifts as the base moves. */
TEST(RigidJoint, LoopClosesOntoRigidlyAttachedBody) {
  const double t = 2.0;
  const std::string path = testing::TempDir() + "/rigid_joint_loop.json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 0], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
         << t << R"(, "time_step": 0.01, "relative_error": 1e-8,
                   "absolute_error": 1e-8}]},
 "assemblies": [{"name": "a",
  "bodies": [
   {"name": "base", "geometry": [], "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 500, "Ixx": 50, "Iyy": 50, "Izz": 50,
                "Ixy": 0, "Iyz": 0, "Izx": 0},
    "frames": [{"name": "lever", "T": [1.431, 0.194, -0.343]},
               {"name": "top", "T": [1.523, 0.5, -0.85]}]},
   {"name": "lever", "geometry": [], "center_of_mass": [0, 0.11, 0],
    "inertia": {"mass": 3, "Ixx": 0.3, "Iyy": 0.3, "Izz": 0.3,
                "Ixy": 0, "Iyz": 0, "Izx": 0},
    "frames": [{"name": "carrier", "T": [0, 0.226, -0.009]}]},
   {"name": "carrier", "geometry": [], "center_of_mass": [0, 0.12, 0],
    "inertia": {"mass": 4, "Ixx": 0.4, "Iyy": 0.4, "Izz": 0.4,
                "Ixy": 0, "Iyz": 0, "Izx": 0},
    "frames": [{"name": "strut", "T": [0.092, 0, 0]}]},
   {"name": "upper", "geometry": [], "center_of_mass": [0, 0, 0.15],
    "inertia": {"mass": 0.8, "Ixx": 0.08, "Iyy": 0.08, "Izz": 0.08,
                "Ixy": 0, "Iyz": 0, "Izx": 0}},
   {"name": "v"},
   {"name": "lower", "geometry": [], "center_of_mass": [0, 0.08, 0],
    "inertia": {"mass": 1.5, "Ixx": 0.15, "Iyy": 0.15, "Izz": 0.15,
                "Ixy": 0, "Iyz": 0, "Izx": 0},
    "frames": [{"name": "slide", "T": [0, 0.08, 0]}]}],
  "joints": [
   {"name": "base_x", "type": "prismatic", "axis": [1, 0, 0],
    "initial_velocity": 15.0,
    "parent": [["fixed"], "origin"], "child": [["base"], "origin"]},
   {"name": "lever_pivot", "type": "revolute", "axis": [1, 0, 0],
    "parent": [["base"], "lever"], "child": [["lever"], "origin"]},
   {"name": "pin", "type": "revolute", "axis": [1, 0, 0],
    "parent": [["lever"], "carrier"], "child": [["carrier"], "origin"]},
   {"name": "tube", "type": "rigid",
    "parent": [["carrier"], "strut"], "child": [["lower"], "origin"]},
   {"name": "mount", "type": "prismatic", "axis": [1, 0, 0],
    "parent": [["base"], "top"], "child": [["v"], "origin"]},
   {"name": "top", "type": "spherical",
    "parent": [["v"], "origin"], "child": [["upper"], "origin"]},
   {"name": "strut", "type": "prismatic", "axis": [0, 0, 1],
    "initial_displacement": 0.498,
    "stiffness_coefficients": [-9193.0, 16000],
    "damping_coefficients": [0, 1000],
    "parent": [["upper"], "origin"], "child": [["lower"], "slide"]}]}]}
)";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.Simulate();
  ksl_SE3_t base = ksl_SE3_identity(), upper = ksl_SE3_identity();
  int found = 0;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "base") {
      base = context.get_body_origin_pose(body);
      found++;
    } else if(body->get_name() == "upper") {
      upper = context.get_body_origin_pose(body);
      found++;
    }
  }
  ASSERT_EQ(found, 2);
  /* the base only translates: the top of the strut stays at its frame top */
  EXPECT_NEAR(upper.t.x - base.t.x, 1.523, 1e-6);
  EXPECT_NEAR(upper.t.y - base.t.y, 0.5, 1e-6);
  EXPECT_NEAR(upper.t.z - base.t.z, -0.85, 1e-6);
}
