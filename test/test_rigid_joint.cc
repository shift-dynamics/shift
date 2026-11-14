/* A rigid joint below a moving body: the frames of its child are placed on
the parent body of the rigid joint, not on the body above it, and the child
moves with that body. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
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

/* Bodies with no children, attached by a rigid joint with a displacement:
to the fixed body's origin, to a rotated global frame, and to a moving arm.
Their origins are in no other joint, so their poses come from the rigid
joints alone: Rz(pi/2) about the joint and its translation, after the
parent frame. */
TEST(RigidJoint, LeafBodyTakesTheJointDisplacement) {
  const double w = 2.0, t = 0.25;
  const std::string path = testing::TempDir() + "/rigid_joint_leaf.json";
  const char* displacement =
    R"({"T": [0.1, 0.2, 0.3], "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]})";
  const char* inertia = R"("center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0}})";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 0.0], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
         << t << R"(, "time_step": 0.01, "relative_error": 1e-10,
                   "absolute_error": 1e-10}]},
 "frames": [{"name": "turned", "T": [1.0, 0.0, 0.0],
             "R": [[1, 0, 0], [0, 0, -1], [0, 1, 0]]}],
 "assemblies": [{"name": "a",
  "bodies": [
   {"name": "arm", "geometry": [], "center_of_mass": [0.5, 0, 0],
    "frames": [{"name": "tip", "T": [1.0, 0.0, 0.0]}],
    "inertia": {"mass": 2.0, "Ixx": 0.2, "Iyy": 0.2, "Izz": 0.2,
                "Ixy": 0, "Iyz": 0, "Izx": 0}},
   {"name": "on_fixed", "geometry": [], )"
         << inertia << R"(,
   {"name": "on_frame", "geometry": [], )"
         << inertia << R"(,
   {"name": "on_arm", "geometry": [], )"
         << inertia << R"(],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 0, 1],
    "initial_velocity": )"
         << w << R"(,
    "parent": [["fixed"], "origin"], "child": [["arm"], "origin"]},
   {"name": "J2", "type": "rigid", "initial_displacement": )"
         << displacement << R"(,
    "parent": [["fixed"], "origin"], "child": [["on_fixed"], "origin"]},
   {"name": "J3", "type": "rigid", "initial_displacement": )"
         << displacement << R"(,
    "parent": [["fixed"], "turned"], "child": [["on_frame"], "origin"]},
   {"name": "J4", "type": "rigid", "initial_displacement": )"
         << displacement << R"(,
    "parent": [["arm"], "tip"], "child": [["on_arm"], "origin"]}]}]}
)";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.Simulate();

  /* the expected pose: frame (rotation f, translation p) after the joint */
  auto expect = [&](const std::string& name, const double f[3][3],
                    const double p[3]) {
    SCOPED_TRACE(name);
    const double r[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    const double s[3] = {0.1, 0.2, 0.3};
    bool found = false;
    for(auto body : context.get_active_articulated_bodies()) {
      if(body->get_name() != name) {
        continue;
      }
      found = true;
      const ksl_SE3_t pose = context.get_body_origin_pose(body);
      for(int i = 0; i < 3; i++) {
        double t_i = p[i];
        for(int k = 0; k < 3; k++) {
          t_i += f[i][k] * s[k];
          double r_ik = 0.0;
          for(int m = 0; m < 3; m++) {
            r_ik += f[i][m] * r[m][k];
          }
          // column-major: row i, column k
          EXPECT_NEAR(pose.R.at[k][i], r_ik, 1e-8) << i << " " << k;
        }
        EXPECT_NEAR(pose.t.at[i], t_i, 1e-8) << i;
      }
    }
    EXPECT_TRUE(found);
  };
  const double identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const double zero[3] = {0, 0, 0};
  expect("on_fixed", identity, zero);
  const double turned[3][3] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
  const double turned_p[3] = {1.0, 0.0, 0.0};
  expect("on_frame", turned, turned_p);
  const double c = std::cos(w * t), s = std::sin(w * t);
  const double arm[3][3] = {{c, -s, 0}, {s, c, 0}, {0, 0, 1}};
  const double tip[3] = {c, s, 0.0};
  expect("on_arm", arm, tip);
}

/* the mass properties of a body of the merge test: its mass, its center of
mass and its principal moments, about the center of mass in the body's axes */
struct RigidMergePart {
  const char* name;
  double m;
  double com[3];
  double moments[3];
};

static const RigidMergePart kRigidMergeParts[] = {
  {"arm", 2.0, {0.0, 0.5, 0.1}, {0.3, 0.1, 0.25}},
  {"b", 1.0, {0.1, 0.0, 0.0}, {0.02, 0.03, 0.04}},
  {"c", 0.5, {0.0, 0.2, 0.0}, {0.01, 0.02, 0.03}},
  {"d", 0.7, {0.0, 0.0, 0.1}, {0.05, 0.05, 0.02}}};

/* the body of a part, in the model file */
static std::string RigidMergeBody(const RigidMergePart& part,
                                  const std::string& frames) {
  std::ostringstream body;
  body.precision(17);
  body << R"({"name": ")" << part.name << R"(", "geometry": [],
    "center_of_mass": [)"
       << part.com[0] << ", " << part.com[1] << ", " << part.com[2] << R"(],
    "inertia": {"mass": )"
       << part.m << R"(, "Ixx": )" << part.moments[0] << R"(, "Iyy": )"
       << part.moments[1] << R"(, "Izz": )" << part.moments[2]
       << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0})";
  if(!frames.empty()) {
    body << R"(, "frames": )" << frames;
  }
  body << "}";
  return body.str();
}

/* a pendulum about x, at 0.3 rad, swinging under gravity for t, with the
bodies and the joints after its revolute joint J1 to the arm */
static std::string RigidMergeModel(double t, const std::string& bodies,
                                   const std::string& joints) {
  std::ostringstream model;
  model.precision(17);
  model << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
        << t << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},
 "assemblies": [{"name": "a",
  "bodies": [)"
        << bodies << R"(],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [1, 0, 0],
    "initial_displacement": 0.3,
    "parent": [["fixed"], "origin"], "child": [["arm"], "origin"]})"
        << joints << "]}]}\n";
  return model.str();
}

static void LoadRigidMergeModel(shift::core::Context& context,
                                const std::string& name,
                                const std::string& text) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << text;
  }
  context.LoadJSON(path);
  std::remove(path.c_str());
}

/* Several bodies attached to one body by rigid joints, and a rigid joint
onto one of them, are merged into that body with all of their mass: an arm
on a revolute joint carries b at a frame of the arm, c by a frame of c's own
that is not its origin, and d at a frame of c. The merged inertia of the arm
is the sum of the four, about their common center of mass, and the arm
swings as one body with that inertia does. */
TEST(RigidJoint, MergesTheInertiaOfEveryRigidBody) {
  const double t = 1.0;
  std::string bodies;
  for(const RigidMergePart& part : kRigidMergeParts) {
    std::string frames;
    if(std::string(part.name) == "arm") {
      frames = R"([{"name": "tip", "T": [0.0, 1.0, 0.2]},
                   {"name": "side", "T": [0.1, 0.4, -0.3],
                    "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}])";
    } else if(std::string(part.name) == "c") {
      frames = R"([{"name": "mount", "T": [0.05, 0.0, 0.1],
                    "R": [[1, 0, 0], [0, 0, -1], [0, 1, 0]]},
                   {"name": "end", "T": [0.0, 0.3, 0.0]}])";
    }
    bodies += (bodies.empty() ? "" : ",\n   ") + RigidMergeBody(part, frames);
  }
  const std::string joints = R"(,
   {"name": "Jb", "type": "rigid",
    "parent": [["arm"], "tip"], "child": [["b"], "origin"]},
   {"name": "Jc", "type": "rigid",
    "parent": [["arm"], "side"], "child": [["c"], "mount"]},
   {"name": "Jd", "type": "rigid",
    "parent": [["c"], "end"], "child": [["d"], "origin"]})";
  shift::core::Context context;
  LoadRigidMergeModel(context, "rigid_merge",
                      RigidMergeModel(t, bodies, joints));

  /* the poses of the bodies relative to the arm, from the model's
  kinematics, and the sum of the parts in the arm's axes */
  std::map<std::string, ksl_SE3_t> poses;
  for(auto body : context.get_active_articulated_bodies()) {
    poses[body->get_name()] = context.get_body_origin_pose(body);
  }
  ASSERT_EQ(poses.count("arm"), 1u);
  ksl_SE3_t arm_inverse;
  ksl_SE3_inverted(&poses["arm"], &arm_inverse);
  double m = 0.0, mt[3] = {0.0, 0.0, 0.0};
  double com[4][3];
  for(int k = 0; k < 4; k++) {
    const RigidMergePart& part = kRigidMergeParts[k];
    ASSERT_EQ(poses.count(part.name), 1u) << part.name;
    ksl_SE3_t relative;
    ksl_product_dd(&arm_inverse, &poses[part.name], &relative);
    const ksl_vec3_t c_k = ksl_vec3(part.com[0], part.com[1], part.com[2]);
    ksl_vec3_t c;
    ksl_product_dv(&relative, &c_k, &c);
    m += part.m;
    for(int i = 0; i < 3; i++) {
      com[k][i] = c.at[i];
      mt[i] += part.m * c.at[i];
    }
  }
  const double center[3] = {mt[0] / m, mt[1] / m, mt[2] / m};
  /* the inertia tensor about the common center of mass: each part's
  moments turned into the arm's axes, R diag(I) R^T, and the parallel axis
  term m (|d|^2 E - d d^T) */
  double tensor[3][3] = {};
  for(int k = 0; k < 4; k++) {
    const RigidMergePart& part = kRigidMergeParts[k];
    ksl_SE3_t relative;
    ksl_product_dd(&arm_inverse, &poses[part.name], &relative);
    double d[3], dd = 0.0;
    for(int i = 0; i < 3; i++) {
      d[i] = com[k][i] - center[i];
      dd += d[i] * d[i];
    }
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        double turned = 0.0;
        for(int n = 0; n < 3; n++) {
          // column-major: row i, column n
          turned += relative.R.at[n][i] * part.moments[n] * relative.R.at[n][j];
        }
        tensor[i][j] += turned + part.m * ((i == j ? dd : 0.0) - d[i] * d[j]);
      }
    }
  }

  /* the arm's merged inertia: its mass, first moment, and moments about the
  center of mass (the products as elements of the tensor) */
  const std::vector<ksl_inertia_t>& inertia = context.get_inertia_vector();
  ASSERT_EQ(inertia.size(), 1u); // J1 is the only joint that moves
  const ksl_inertia_t& merged = inertia[0];
  EXPECT_NEAR(merged.m, 4.2, 1e-12);
  for(int i = 0; i < 3; i++) {
    EXPECT_NEAR(merged.mt.at[i], mt[i], 1e-12) << i;
  }
  EXPECT_NEAR(merged.Ixx, tensor[0][0], 1e-12);
  EXPECT_NEAR(merged.Iyy, tensor[1][1], 1e-12);
  EXPECT_NEAR(merged.Izz, tensor[2][2], 1e-12);
  EXPECT_NEAR(merged.Ixy, tensor[0][1], 1e-12);
  EXPECT_NEAR(merged.Iyz, tensor[1][2], 1e-12);
  EXPECT_NEAR(merged.Izx, tensor[2][0], 1e-12);

  /* the same arm as a single body with the merged mass, center of mass
  and moment about x (about the hinge, only these act) swings alike */
  const RigidMergePart one = {"arm", m,
                              {center[0], center[1], center[2]},
                              {tensor[0][0], tensor[0][0], tensor[0][0]}};
  shift::core::Context single;
  LoadRigidMergeModel(single, "rigid_merge_single",
                      RigidMergeModel(t, RigidMergeBody(one, ""), ""));
  context.Simulate();
  single.Simulate();
  ksl_SE3_t swung = ksl_SE3_identity(), reference = ksl_SE3_identity();
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "arm") {
      swung = context.get_body_origin_pose(body);
    }
  }
  for(auto body : single.get_active_articulated_bodies()) {
    if(body->get_name() == "arm") {
      reference = single.get_body_origin_pose(body);
    }
  }
  /* turned about x: the angle from the y axis of the arm */
  const double angle = std::atan2(swung.R.v1.z, swung.R.v1.y);
  const double expected = std::atan2(reference.R.v1.z, reference.R.v1.y);
  EXPECT_GT(std::fabs(expected - 0.3), 0.05); // it swung
  EXPECT_NEAR(angle, expected, 1e-7);
}

/* A body on a revolute joint at a frame of its own that is turned and
offset from its origin (the joint's child frame, which becomes the body's
reference frame): its center of mass and moments are taken into that frame's
coordinates, so it swings as the same body does when its origin is at that
frame. */
TEST(RigidJoint, BodyOnATurnedChildFrameSwingsAsAtItsOrigin) {
  const double t = 1.0;
  const RigidMergePart link = {"arm", 3.0, {0.1, 0.3, -0.2}, {0.4, 0.2, 0.3}};
  const std::string frames = R"([{"name": "pivot", "T": [0.2, -0.1, 0.3],
                                  "R": [[0, 0, 1], [1, 0, 0], [0, 1, 0]]}])";
  std::string turned_model =
    RigidMergeModel(t, RigidMergeBody(link, frames), "");
  const std::string at_origin = R"("child": [["arm"], "origin"]})";
  turned_model.replace(turned_model.find(at_origin), at_origin.size(),
                       R"("child": [["arm"], "pivot"]})");
  shift::core::Context turned;
  LoadRigidMergeModel(turned, "rigid_turned_frame", turned_model);

  /* the pivot frame in the arm's origin coordinates, from the model */
  const ksl_SE3_t* d_op = nullptr;
  for(auto body : turned.get_active_articulated_bodies()) {
    if(body->get_name() != "arm") {
      continue;
    }
    for(auto& frame : body->get_frames()) {
      if(frame->get_name() == "pivot") {
        d_op = frame->get_d_wrt_origin();
      }
    }
  }
  ASSERT_NE(d_op, nullptr);
  /* the center of mass and the moment about x in the pivot's coordinates:
  R^T (c - t) and (R^T diag(I) R)_xx; about the hinge along x only these
  act */
  double center[3], ixx = 0.0;
  for(int i = 0; i < 3; i++) {
    center[i] = 0.0;
    for(int k = 0; k < 3; k++) {
      // column-major: R^T row i, column k is R row k, column i
      center[i] += d_op->R.at[i][k] * (link.com[k] - d_op->t.at[k]);
    }
  }
  for(int n = 0; n < 3; n++) {
    ixx += d_op->R.at[0][n] * link.moments[n] * d_op->R.at[0][n];
  }
  const RigidMergePart moved = {"arm", link.m,
                                {center[0], center[1], center[2]},
                                {ixx, ixx, ixx}};
  shift::core::Context reference;
  LoadRigidMergeModel(reference, "rigid_turned_frame_origin",
                      RigidMergeModel(t, RigidMergeBody(moved, ""), ""));
  turned.Simulate();
  reference.Simulate();
  /* the angle of J1: the turn about x of the joint's child frame */
  auto angle = [](shift::core::Context& context, const std::string& name) {
    for(auto body : context.get_active_articulated_bodies()) {
      if(body->get_name() != "arm") {
        continue;
      }
      for(auto& frame : body->get_frames()) {
        if(frame->get_name() == name) {
          const ksl_SE3_t origin = context.get_body_origin_pose(body);
          ksl_SE3_t pose;
          ksl_product_dd(&origin, frame->get_d_wrt_origin(), &pose);
          return std::atan2(pose.R.v1.z, pose.R.v1.y);
        }
      }
    }
    ADD_FAILURE() << "no frame " << name;
    return 0.0;
  };
  const double expected = angle(reference, "origin");
  EXPECT_GT(std::fabs(expected - 0.3), 0.05); // it swung
  EXPECT_NEAR(angle(turned, "pivot"), expected, 1e-7);
}
