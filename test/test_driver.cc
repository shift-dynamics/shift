/* The driver controller (controller::Driver): pure pursuit steering along a
path, and the throttle, brake and gear for the speed profile of the path */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <ksl/SE3.h>
#include <ksl/mat3x3.h>
#include <ksl/matrix.h>
#include <ksl/vec3.h>

#include "shift/core/context.h"
#include "shift/core/controller/driver.h"

/* a car on a float joint at x, y (m) turned yaw (rad) about z, with the
frame "rear" a meter behind its origin, and a driver of the parameters
driver (JSON members, after the body_frame_pair) */
static std::string Model(const std::string& driver, double x = 0.0,
                         double y = 0.0, double yaw = 0.0) {
  std::ostringstream text;
  const double c = std::cos(yaw), s = std::sin(yaw);
  text << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
 "assemblies": [{"name": "a",
  "bodies": [{"name": "car", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1000, "Ixx": 300, "Iyy": 1000, "Izz": 1000,
                "Ixy": 0, "Iyz": 0, "Izx": 0},
    "frames": [{"name": "rear", "T": [-1, 0, 0]}]}],
  "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
    "child": [["car"], "origin"],
    "initial_displacement": {"T": [)"
       << x << ", " << y << R"(, 0],
      "R": [[)" << c << ", " << -s << ", 0], [" << s << ", " << c
       << R"(, 0], [0, 0, 1]]}}],
  "controllers": [{"type": "driver", "name": "driver",
                   "body_frame_pair": ["car", "rear"], )"
       << driver << "}]}]}";
  return text.str();
}

static void Load(shift::core::Context& context, const std::string& json) {
  const std::string path = testing::TempDir() + "/driver_test.json";
  {
    std::ofstream file(path);
    file << json;
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static shift::core::controller::Driver* FindDriver(
  shift::core::Context& context) {
  return dynamic_cast<shift::core::controller::Driver*>(
    context["assemblies"]["a"]["controllers"]["driver"]
      .get_data<shift::core::controller::Base>());
}

static std::string LoadError(const std::string& driver) {
  shift::core::Context context;
  try {
    Load(context, Model(driver));
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

/* a frame of the car that is not where it is joined: its pose is that of
the car's origin with the frame's offset. Half a meter to the right of a
straight path along x, the driver steers left onto it, on the arc through
the goal point lookahead_distance ahead */
TEST(Driver, PurePursuitFromTheFrame) {
  shift::core::Context context;
  Load(context, Model(R"("wheelbase": 2.5, "path": [-10, 0, 50, 0],
                         "lookahead_distance": 5, "lookahead_time": 0.5)",
                      0.0, 0.5));
  auto* driver = FindDriver(context);
  ASSERT_NE(driver, nullptr);
  context.UpdateDHVA();
  driver->UpdateOutputs(0.0, nullptr);
  EXPECT_NEAR(driver->get_station(), 9.0, 1e-12);
  EXPECT_NEAR(driver->get_lateral_error(), 0.5, 1e-12);
  /* at rest the look-ahead is lookahead_distance: the goal is 5 m ahead and
  0.5 m to the left */
  const double expected = std::atan(2.0 * 2.5 * -0.5 / (25.0 + 0.25));
  EXPECT_NEAR(driver->get_steering(), expected, 1e-12);
  EXPECT_LT(driver->get_steering(), 0.0);
}

/* heading 30 degrees to the right of the path, on it: the driver steers
left, by the angle of the goal point seen from the frame */
TEST(Driver, SteersBackToThePath) {
  shift::core::Context context;
  Load(context, Model(R"("wheelbase": 2.5, "path": [-10, 0, 50, 0],
                         "lookahead_distance": 6, "max_steering_angle": 0.5)"));
  auto* driver = FindDriver(context);
  ASSERT_NE(driver, nullptr);
  const double yaw = M_PI / 6.0;
  ksl_SE3_t pose = ksl_SE3_identity();
  const ksl_vec3_t z = ksl_vec3(0.0, 0.0, 1.0);
  ksl_mat3x3_setFromAxisAngle(&pose.R, &z, yaw);
  driver->Drive(pose, 0.0);
  /* the goal at (6, 0): in the frame, ahead 6 cos 30, right -6 sin 30 */
  const double ahead = 6.0 * std::cos(yaw), right = -6.0 * std::sin(yaw);
  EXPECT_NEAR(driver->get_steering(),
              std::atan(2.0 * 2.5 * right / (ahead * ahead + right * right)),
              1e-12);
  /* across the path, the goal 6 m to the left: the angle is held at
  max_steering_angle */
  ksl_mat3x3_setFromAxisAngle(&pose.R, &z, 0.5 * M_PI);
  driver->Drive(pose, 0.0);
  EXPECT_DOUBLE_EQ(driver->get_steering(), -0.5);
}

/* an open straight path starts and ends at rest; between, the profile rises
at acceleration and falls at deceleration to speed */
TEST(Driver, SpeedProfileOfAnOpenPath) {
  shift::core::Context context;
  Load(context, Model(R"("wheelbase": 2.5, "path": [0, 0, 100, 0],
                         "speed": 12, "acceleration": 2,
                         "deceleration": 4)"));
  auto* driver = FindDriver(context);
  ASSERT_NE(driver, nullptr);
  EXPECT_DOUBLE_EQ(driver->get_length(), 100.0);
  EXPECT_DOUBLE_EQ(driver->ProfileSpeed(0.0), 0.0);
  EXPECT_DOUBLE_EQ(driver->ProfileSpeed(100.0), 0.0);
  /* the profile is linear between the two points: at their speeds */
  shift::core::Context fine;
  std::ostringstream path;
  path << R"("wheelbase": 2.5, "speed": 12, "acceleration": 2,
             "deceleration": 4, "path": [)";
  for(int k = 0; k <= 100; k++) {
    path << (k ? ", " : "") << k << ", 0";
  }
  path << "]";
  Load(fine, Model(path.str()));
  driver = FindDriver(fine);
  ASSERT_NE(driver, nullptr);
  EXPECT_NEAR(driver->ProfileSpeed(9.0), std::sqrt(2.0 * 2.0 * 9.0), 1e-12);
  EXPECT_DOUBLE_EQ(driver->ProfileSpeed(50.0), 12.0);
  EXPECT_NEAR(driver->ProfileSpeed(92.0), std::sqrt(2.0 * 4.0 * 8.0), 1e-12);
}

/* round a circle the speed is that of the lateral acceleration, v^2 / r */
TEST(Driver, SpeedInBends) {
  std::ostringstream path;
  path << R"("wheelbase": 2.5, "speed": 30, "lateral_acceleration": 4,
             "closed": true, "path": [)";
  const double radius = 20.0;
  for(int k = 0; k < 180; k++) {
    const double a = 2.0 * M_PI * k / 180.0;
    path << (k ? ", " : "") << radius * std::cos(a) << ", "
         << radius * std::sin(a);
  }
  path << "]";
  shift::core::Context context;
  Load(context, Model(path.str()));
  auto* driver = FindDriver(context);
  ASSERT_NE(driver, nullptr);
  for(double s : {0.0, 30.0, 61.0, 100.0}) {
    /* within 1 %: the curvature is of points on the chords between the
    points of the path, a little inside the circle */
    EXPECT_NEAR(driver->ProfileSpeed(s), std::sqrt(4.0 * radius), 0.09)
      << "at " << s;
  }
  /* a closed path goes on round */
  EXPECT_NEAR(driver->ProfileSpeed(driver->get_length() + 30.0),
              driver->ProfileSpeed(30.0), 1e-9);
}

/* the throttle below the target speed, the brake above it and at the end of
the path, and the gear by the shift speeds */
TEST(Driver, ThrottleBrakeAndGear) {
  std::ostringstream path;
  path << R"("wheelbase": 2.5, "speed": 10, "acceleration": 3,
             "deceleration": 3, "throttle_gain": 0.25, "brake_gain": 0.2,
             "lookahead_distance": 4, "lookahead_time": 0,
             "shift_speeds": [5, 9], "path": [)";
  for(int k = 0; k <= 200; k += 2) {
    path << (k ? ", " : "") << k << ", 0";
  }
  path << "]";
  shift::core::Context context;
  Load(context, Model(path.str()));
  auto* driver = FindDriver(context);
  ASSERT_NE(driver, nullptr);
  ksl_SE3_t pose = ksl_SE3_identity();
  pose.t.x = 100.0;
  driver->Drive(pose, 7.0);
  EXPECT_DOUBLE_EQ(driver->get_target_speed(), 10.0);
  EXPECT_DOUBLE_EQ(driver->get_throttle(), 0.75);
  EXPECT_DOUBLE_EQ(driver->get_brake(), 0.0);
  EXPECT_DOUBLE_EQ(driver->get_gear(), 2.0);
  driver->Drive(pose, 12.0);
  EXPECT_DOUBLE_EQ(driver->get_throttle(), 0.0);
  EXPECT_NEAR(driver->get_brake(), 0.4, 1e-12);
  EXPECT_DOUBLE_EQ(driver->get_gear(), 3.0);
  driver->Drive(pose, 3.0);
  EXPECT_DOUBLE_EQ(driver->get_throttle(), 1.0);
  EXPECT_DOUBLE_EQ(driver->get_gear(), 1.0);
  /* the goal point past the end: stop */
  pose.t.x = 197.0;
  driver->Drive(pose, 0.5);
  EXPECT_DOUBLE_EQ(driver->get_target_speed(), 0.0);
  EXPECT_DOUBLE_EQ(driver->get_brake(), 1.0);
  EXPECT_DOUBLE_EQ(driver->get_throttle(), 0.0);
}

TEST(Driver, Checks) {
  EXPECT_NE(LoadError(R"("wheelbase": 2.5, "path": [0, 0, 10])")
              .find("pairs"),
            std::string::npos);
  EXPECT_NE(LoadError(R"("wheelbase": 2.5, "path": [0, 0, 0, 0])")
              .find("the same"),
            std::string::npos);
  EXPECT_NE(LoadError(R"("wheelbase": 0, "path": [0, 0, 10, 0])")
              .find("wheelbase"),
            std::string::npos);
  EXPECT_NE(LoadError(R"("wheelbase": 2.5, "path": [0, 0, 10, 0],
                         "shift_speeds": [5, 4])")
              .find("increase"),
            std::string::npos);
  EXPECT_NE(LoadError(R"("wheelbase": 2.5, "path": [0, 0, 10, 0],
                         "body_frame_pair": ["car", "front"])")
              .find("frame 'front'"),
            std::string::npos);
}
