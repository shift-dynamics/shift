/* Revolute and prismatic joints about or along an axis that is not x, y or
z: the kernels of such joints read the axis from Context::get_joint_data().
Each joint is tested both ways round: with the fixed body as its child the
joint is traversed inverted, and the body moves by -q. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"

static const double kGravity = 9.81; // along +z

/* pose of the body "b" after simulating t, with one joint between it and the
fixed body; properties holds the joint's type, axis and initial values */
static ksl_SE3_t BodyPose(const std::string& name, bool inverted,
                          const std::string& properties, double t) {
  const std::string bodies = inverted
                               ? R"("parent": [["b"], "origin"],
                                    "child": [["fixed"], "origin"])"
                               : R"("parent": [["fixed"], "origin"],
                                    "child": [["b"], "origin"])";
  const std::string path = testing::TempDir() + "/joint_axis_" + name +
                           (inverted ? "_inverted" : "_direct") + ".json";
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
  "bodies": [{"name": "b", "geometry": [], "center_of_mass": [0, 0, 0],
              "inertia": {"mass": 2.0, "Ixx": 0.2, "Iyy": 0.2, "Izz": 0.2,
                          "Ixy": 0, "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "J", )"
         << properties << ", " << bodies << "}]}]}";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.Simulate();
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "b") {
      return context.get_body_origin_pose(body);
    }
  }
  ADD_FAILURE() << "no body";
  return ksl_SE3_identity();
}

/* a slider falling along an inclined axis a follows
p(t) = 1/2 g a_z t^2 a */
TEST(JointAxis, PrismaticSlidesAlongArbitraryAxis) {
  const double a[3] = {0.6, 0.0, 0.8};
  const double t = 0.5;
  const double s = 0.5 * kGravity * a[2] * t * t;
  for(bool inverted : {false, true}) {
    const ksl_SE3_t pose =
      BodyPose("prismatic", inverted,
               R"("type": "prismatic", "axis": [0.6, 0.0, 0.8])", t);
    const char* way = inverted ? "inverted" : "direct";
    EXPECT_NEAR(pose.t.x, s * a[0], 1e-8) << way;
    EXPECT_NEAR(pose.t.y, 0.0, 1e-8) << way;
    EXPECT_NEAR(pose.t.z, s * a[2], 1e-8) << way;
  }
}

/* a body with spherical inertia, its center of mass on an inclined axis a,
spinning freely about it at w turns by w t: R = Rot(a, w t), or Rot(a, -w t)
when the joint is inverted */
TEST(JointAxis, RevoluteTurnsAboutArbitraryAxis) {
  const double a[3] = {0.0, 0.6, 0.8};
  const double w = 2.0, t = 0.5;
  for(bool inverted : {false, true}) {
    const ksl_SE3_t pose = BodyPose(
      "revolute", inverted,
      R"("type": "revolute", "axis": [0.0, 0.6, 0.8], "initial_velocity": 2.0)",
      t);
    /* Rodrigues: R = I + sin(theta) K + (1 - cos(theta)) K^2 */
    const double theta = inverted ? -w * t : w * t;
    const double K[3][3] = {
      {0, -a[2], a[1]}, {a[2], 0, -a[0]}, {-a[1], a[0], 0}};
    const char* way = inverted ? "inverted" : "direct";
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        double K2 = 0.0;
        for(int k = 0; k < 3; k++) {
          K2 += K[i][k] * K[k][j];
        }
        const double expected =
          (i == j) + std::sin(theta) * K[i][j] + (1 - std::cos(theta)) * K2;
        /* R.v0, v1, v2 are the columns */
        EXPECT_NEAR(pose.R.at[j][i], expected, 1e-8)
          << way << " R(" << i << ", " << j << ")";
      }
    }
    EXPECT_NEAR(pose.t.x, 0.0, 1e-12) << way;
    EXPECT_NEAR(pose.t.y, 0.0, 1e-12) << way;
    EXPECT_NEAR(pose.t.z, 0.0, 1e-12) << way;
  }
}
