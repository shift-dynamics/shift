/* Screw and gear joints. Each is simulated with an eccentric body under
gravity and compared with the solution of the one equation of motion of the
joint, integrated here with the Runge-Kutta method; each is tested both ways
round, where the fixed body is the child and the joint is traversed inverted.
The reference needs the joint's geometry only: the motion of the center of
mass X(q) and the spin of the body s(q), with M = I s'^2 + m |X'|^2 and
M q'' + 1/2 M' q'^2 = -dV/dq. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"

static const double kPi = 3.14159265358979323846;

/* simulate one body on a joint to the time t and return the pose of the
body's origin; the model is written by the caller */
static ksl_SE3_t Simulate(const std::string& name, const std::string& model,
                          double t) {
  const std::string path = testing::TempDir() + "/screw_gear_" + name + ".json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 9.81], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": )"
         << t << R"(, "time_step": 0.01, "relative_error": 1e-11,
                   "absolute_error": 1e-11}]},)"
         << model;
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

/* integrate q'' = acceleration(q, q') from q0, q0' over t */
static double Integrate(const std::function<double(double, double)>& acceleration,
                        double q0, double dq0, double t) {
  const int steps = 20000;
  const double h = t / steps;
  double q = q0, dq = dq0;
  for(int i = 0; i < steps; i++) {
    const double k1q = dq, k1v = acceleration(q, dq);
    const double k2q = dq + 0.5 * h * k1v,
                 k2v = acceleration(q + 0.5 * h * k1q, k2q);
    const double k3q = dq + 0.5 * h * k2v,
                 k3v = acceleration(q + 0.5 * h * k2q, k3q);
    const double k4q = dq + h * k3v, k4v = acceleration(q + h * k3q, k4q);
    q += h / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q);
    dq += h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v);
  }
  return q;
}

/* a body of mass 2 and spherical inertia 0.3, the center of mass at c from
the origin */
static std::string Body(const std::string& c) {
  return R"("bodies": [{"name": "b", "geometry": [], "center_of_mass": )" + c +
         R"(, "inertia": {"mass": 2.0, "Ixx": 0.3, "Iyy": 0.3, "Izz": 0.3,
                          "Ixy": 0, "Iyz": 0, "Izx": 0}}],)";
}

static const double kMass = 2.0, kInertia = 0.3;

/* A body whose center of mass is c off the axis a, turning on a screw of
pitch p: X(q) = Rot(a, q) c + p q a, spin q, so M = I + m (|c|^2 + p^2) is
constant and M q'' = m G . dX/dq, with dX/dq = a x Rot(a, q) c + p a. */
TEST(ScrewJoint, MatchesTheEquationOfMotionOfAnEccentricBody) {
  const double a[3] = {0.0, 0.6, 0.8};
  const double c[3] = {0.3, 0.0, 0.0}; // perpendicular to the axis
  const double p = 0.25, q0 = 0.4, dq0 = 1.2, t = 1.0;
  const double G[3] = {0.0, 0.0, 9.81};
  const double axc[3] = {a[1] * c[2] - a[2] * c[1], a[2] * c[0] - a[0] * c[2],
                         a[0] * c[1] - a[1] * c[0]}; // a x c
  const double M = kInertia + kMass * (0.3 * 0.3 + p * p);
  auto acceleration = [&](double q, double) {
    // dX/dq = (a x c) cos q - c sin q + p a
    double work = 0.0;
    for(int i = 0; i < 3; i++) {
      work += G[i] * (axc[i] * std::cos(q) - c[i] * std::sin(q) + p * a[i]);
    }
    return kMass * work / M;
  };
  const double expected = Integrate(acceleration, q0, dq0, t);

  for(bool inverted : {false, true}) {
    const std::string joint =
      std::string(R"("joints": [{"name": "J", "type": "screw", )") +
      R"("axis": [0.0, 0.6, 0.8], "pitch": 0.25, )" +
      (inverted ? R"("initial_displacement": -0.4, "initial_velocity": -1.2,
                      "parent": [["b"], "origin"],
                      "child": [["fixed"], "origin"]}]}]})"
                : R"("initial_displacement": 0.4, "initial_velocity": 1.2,
                      "parent": [["fixed"], "origin"],
                      "child": [["b"], "origin"]}]}]})");
    const std::string model = R"("assemblies": [{"name": "a", )" +
                              Body("[0.3, 0.0, 0.0]") + joint;
    const ksl_SE3_t pose =
      Simulate(inverted ? "screw_inverted" : "screw_direct", model, t);
    // the turn of the body about the axis: (R e) = e cos + (a x e) sin
    const double e[3] = {1.0, 0.0, 0.0};
    const double ae[3] = {a[1] * e[2] - a[2] * e[1], a[2] * e[0] - a[0] * e[2],
                          a[0] * e[1] - a[1] * e[0]};
    double re[3];
    for(int i = 0; i < 3; i++) {
      re[i] = e[0] * pose.R.at[0][i] + e[1] * pose.R.at[1][i] +
              e[2] * pose.R.at[2][i];
    }
    const double turn = std::atan2(re[0] * ae[0] + re[1] * ae[1] + re[2] * ae[2],
                                   re[0] * e[0] + re[1] * e[1] + re[2] * e[2]);
    const char* way = inverted ? "inverted" : "direct";
    EXPECT_NEAR(std::remainder(turn - expected, 2 * kPi), 0.0, 1e-7) << way;
    // the origin advances along the axis by p times the turn
    EXPECT_NEAR(pose.t.x, p * expected * a[0], 1e-7) << way;
    EXPECT_NEAR(pose.t.y, p * expected * a[1], 1e-7) << way;
    EXPECT_NEAR(pose.t.z, p * expected * a[2], 1e-7) << way;
  }
}

/* the pose for a given angle, with no time to move */
TEST(ScrewJoint, PoseOfTheChildIsARotationAndAnAdvance) {
  const double a[3] = {0.0, 0.6, 0.8};
  const double q = 0.9, p = -0.1;
  for(bool inverted : {false, true}) {
    const std::string joint =
      std::string(R"("joints": [{"name": "J", "type": "screw", )") +
      R"("axis": [0.0, 0.6, 0.8], "pitch": -0.1, )" +
      (inverted ? R"("initial_displacement": 0.9,
                      "parent": [["b"], "origin"],
                      "child": [["fixed"], "origin"]}]}]})"
                : R"("initial_displacement": 0.9,
                      "parent": [["fixed"], "origin"],
                      "child": [["b"], "origin"]}]}]})");
    const std::string model = R"("assemblies": [{"name": "a", )" +
                              Body("[0.0, 0.0, 0.0]") + joint;
    const ksl_SE3_t pose =
      Simulate(inverted ? "pose_inverted" : "pose_direct", model, 1e-9);
    const double s = inverted ? -1.0 : 1.0;
    // the advance is p q along the axis, the turn is q about it
    EXPECT_NEAR(pose.t.x, s * p * q * a[0], 1e-6);
    EXPECT_NEAR(pose.t.y, s * p * q * a[1], 1e-6);
    EXPECT_NEAR(pose.t.z, s * p * q * a[2], 1e-6);
    // the axis is unchanged by the turn: R a = a
    for(int i = 0; i < 3; i++) {
      EXPECT_NEAR(a[0] * pose.R.at[0][i] + a[1] * pose.R.at[1][i] +
                    a[2] * pose.R.at[2][i],
                  a[i], 1e-6);
    }
  }
}

/* A body on a gear joint: separation d and ratio g, the gear axis along the
frame z; gravity is along the frame y (the frame is turned so that its y axis
is the z axis of the model, which points down). With
  forward:  X = d [cos q, sin q] + Rot((1+g) q) c, spin  (1+g) q
  inverted: X = d [-cos g q, sin g q] + Rot(-(1+g) q) c, spin -(1+g) q
and the potential V = -m G X_y. */
static double GearReference(bool inverse, double d, double g, double cx,
                            double cy, double q0, double dq0, double t) {
  const double k = 1.0 + g, G = 9.81;
  auto acceleration = [=](double q, double dq) {
    const double sigma = inverse ? -1.0 : 1.0;
    const double theta = sigma * k * q;
    double dP[2], ddP[2];
    if(!inverse) {
      dP[0] = -d * std::sin(q);
      dP[1] = d * std::cos(q);
      ddP[0] = -d * std::cos(q);
      ddP[1] = -d * std::sin(q);
    } else {
      dP[0] = d * g * std::sin(g * q);
      dP[1] = d * g * std::cos(g * q);
      ddP[0] = d * g * g * std::cos(g * q);
      ddP[1] = -d * g * g * std::sin(g * q);
    }
    const double dX[2] = {
      dP[0] + sigma * k * (-cx * std::sin(theta) - cy * std::cos(theta)),
      dP[1] + sigma * k * (cx * std::cos(theta) - cy * std::sin(theta))};
    const double ddX[2] = {
      ddP[0] + k * k * (-cx * std::cos(theta) + cy * std::sin(theta)),
      ddP[1] + k * k * (-cx * std::sin(theta) - cy * std::cos(theta))};
    const double M = kInertia * k * k + kMass * (dX[0] * dX[0] + dX[1] * dX[1]);
    const double dM = 2.0 * kMass * (dX[0] * ddX[0] + dX[1] * ddX[1]);
    return (kMass * G * dX[1] - 0.5 * dM * dq * dq) / M;
  };
  return Integrate(acceleration, q0, dq0, t);
}

TEST(GearJoint, MatchesTheEquationOfMotionOfAnEccentricBody) {
  const double d = 0.5, g = 0.4, q0 = 0.3, dq0 = 1.5, t = 1.0;
  const double cx = 0.1, cy = 0.05;
  const std::string frame =
    R"("frames": [{"name": "gear", "T": [0, 0, 0],
                   "R": [[1, 0, 0], [0, 0, -1], [0, 1, 0]]}],)";
  for(bool inverted : {false, true}) {
    const double expected = GearReference(inverted, d, g, cx, cy, q0, dq0, t);
    const std::string joint =
      std::string(R"("joints": [{"name": "J", "type": "gear", )") +
      R"("separation": 0.5, "gear_ratio": 0.4, )" +
      R"("initial_displacement": 0.3, "initial_velocity": 1.5, )" +
      (inverted ? R"("parent": [["b"], "origin"],
                      "child": [["fixed"], "gear"]}]}]})"
                : R"("parent": [["fixed"], "gear"],
                      "child": [["b"], "origin"]}]}]})");
    const std::string model = frame + R"("assemblies": [{"name": "a", )" +
                              Body("[0.1, 0.05, 0.0]") + joint;
    const ksl_SE3_t pose =
      Simulate(inverted ? "gear_inverted" : "gear_direct", model, t);
    // the frame: x along the model x, y along the model z, z along -y
    const double x = pose.t.x, y = pose.t.z;
    const double angle = inverted ? std::atan2(y, -x) / g : std::atan2(y, x);
    const char* way = inverted ? "inverted" : "direct";
    EXPECT_NEAR(std::remainder(angle - expected, 2 * kPi), 0.0, 1e-7) << way;
    // the spin of the body about the gear axis; the model's x axis is the
    // x axis of the frame and the model z axis its y axis
    const double spin = std::atan2(pose.R.v0.z, pose.R.v0.x);
    const double sigma = inverted ? -1.0 : 1.0;
    EXPECT_NEAR(std::remainder(spin - sigma * (1.0 + g) * expected, 2 * kPi),
                0.0, 1e-7)
      << way;
  }
}

/* The gears roll without slipping: when the line of centers is at q, the
point of the child that touches the parent is the one at the angle pi - g q
from the child's axis (the arc g q r_child on the child equals the arc q
r_parent on the parent), and it is at the pitch circle of the parent, at
r_parent [cos q, sin q]. */
TEST(GearJoint, TheGearsRollWithoutSlipping) {
  const double r_parent = 0.3, r_child = 0.2;
  const double g = r_parent / r_child, d = r_parent + r_child;
  const std::string frame = R"("frames": [{"name": "gear", "T": [0, 0, 0]}],)";
  for(double q : {0.0, 0.3, 1.1, -0.8, 2.5}) {
    const std::string joint =
      R"("joints": [{"name": "J", "type": "gear", "separation": )" +
      std::to_string(d) + R"(, "gear_ratio": )" + std::to_string(g) +
      R"(, "initial_displacement": )" + std::to_string(q) +
      R"(, "parent": [["fixed"], "gear"], "child": [["b"], "origin"]}]}]})";
    const std::string model = frame + R"("assemblies": [{"name": "a", )" +
                              Body("[0.0, 0.0, 0.0]") + joint;
    const ksl_SE3_t pose = Simulate("rolling", model, 1e-9);
    // the touching point in the child's frame, and in the parent's
    const double cx = -r_child * std::cos(g * q), cy = r_child * std::sin(g * q);
    const double x = pose.t.x + cx * pose.R.v0.x + cy * pose.R.v1.x;
    const double y = pose.t.y + cx * pose.R.v0.y + cy * pose.R.v1.y;
    EXPECT_NEAR(x, r_parent * std::cos(q), 1e-6) << q;
    EXPECT_NEAR(y, r_parent * std::sin(q), 1e-6) << q;
  }
}

/* A planetary gear: a carrier arm on a revolute joint at the center of the
fixed gear, the planet on a revolute joint at the end of the arm, and the gear
joint from the fixed body to the planet, which closes the loop. The carrier is
light, so the planet moves as on the gear joint alone. */
TEST(GearJoint, ClosesAKinematicLoopWithACarrier) {
  const double d = 0.5, g = 0.4, q0 = 0.3, dq0 = 1.5, t = 1.0;
  const double cx = 0.1, cy = 0.05;
  const double expected = GearReference(false, d, g, cx, cy, q0, dq0, t);
  const std::string frame =
    R"("frames": [{"name": "gear", "T": [0, 0, 0],
                   "R": [[1, 0, 0], [0, 0, -1], [0, 1, 0]]}],)";
  const std::string bodies =
    R"("bodies": [
        {"name": "carrier", "geometry": [],
         "frames": [{"name": "tip", "T": [0.5, 0, 0]}],
         "center_of_mass": [0.25, 0, 0],
         "inertia": {"mass": 1e-3, "Ixx": 1e-6, "Iyy": 1e-6, "Izz": 1e-6,
                     "Ixy": 0, "Iyz": 0, "Izx": 0}},
        {"name": "b", "geometry": [], "center_of_mass": [0.1, 0.05, 0.0],
         "inertia": {"mass": 2.0, "Ixx": 0.3, "Iyy": 0.3, "Izz": 0.3,
                     "Ixy": 0, "Iyz": 0, "Izx": 0}}],)";
  const std::string joints =
    R"("joints": [
        {"name": "arm", "type": "revolute", "axis": [0, 0, 1],
         "initial_displacement": 0.3, "initial_velocity": 1.5,
         "parent": [["fixed"], "gear"], "child": [["carrier"], "origin"]},
        {"name": "pin", "type": "revolute", "axis": [0, 0, 1],
         "initial_displacement": 0.12, "initial_velocity": 0.6,
         "parent": [["carrier"], "tip"], "child": [["b"], "origin"]},
        {"name": "mesh", "type": "gear", "separation": 0.5, "gear_ratio": 0.4,
         "initial_displacement": 0.3, "initial_velocity": 1.5,
         "parent": [["fixed"], "gear"], "child": [["b"], "origin"]}]}]})";
  const std::string model = frame + R"("assemblies": [{"name": "a", )" +
                            bodies + joints;
  const ksl_SE3_t pose = Simulate("planetary", model, t);
  const double angle = std::atan2(pose.t.z, pose.t.x);
  EXPECT_NEAR(std::remainder(angle - expected, 2 * kPi), 0.0, 2e-3);
  const double spin = std::atan2(pose.R.v0.z, pose.R.v0.x);
  EXPECT_NEAR(std::remainder(spin - (1.0 + g) * angle, 2 * kPi), 0.0, 1e-6);
}
