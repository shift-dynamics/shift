/* Joint friction and joint map force modules (force_module::JointFriction,
force_module::JointMap), and a bushing as the spring of a spherical joint */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/joint_force.h"

using shift::core::force_module::JointMap;

static const double kMass = 2.0, kWeight = 2.0 * 9.81;

static const char* kSlider =
  R"({"name": "j", "type": "prismatic", "parent": [["fixed"], "origin"],
      "child": [["body"], "origin"], "axis": [0, 0, 1]})";

static void LoadModel(shift::core::Context& context, const std::string& name,
                      const std::string& joint, const std::string& module,
                      double time_span) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
      "include_directories": [")" << testing::TempDir() << R"("],
      "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.01, "relative_error": 1e-8,
                             "absolute_error": 1e-10}]},
      "assemblies": [{"name": "test", "bodies": [{"name": "body",
        "center_of_mass": [0, 0, 0],
        "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                    "Iyz": 0, "Izx": 0}}],
      "joints": [)" << joint << R"(], "force_modules": [)" << module
         << R"(]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

/* the coordinate of the joint and its rate at the end */
static void RunToEnd(shift::core::Context& context, double& q, double& dq) {
  context.set_state_observer([&](double, const std::vector<double>& y) {
    q = y[0];
    dq = y[1];
  });
  context.Simulate();
}

TEST(JointFriction, SmoothFrictionSlowsTheFall) {
  // sliding fast, past the slip speed: a = g - f / m
  shift::core::Context context;
  LoadModel(context, "jf_slide", kSlider,
            R"({"type": "joint_friction", "name": "f", "joint": "j",
                "friction": 8, "slip_speed": 1e-4})",
            1.0);
  double q = 0.0, dq = 0.0;
  RunToEnd(context, q, dq);
  EXPECT_NEAR(dq, (kWeight - 8.0) / kMass, 1e-3);
}

TEST(JointFriction, ViscousFrictionReachesItsTerminalSpeed) {
  shift::core::Context context;
  LoadModel(context, "jf_viscous", kSlider,
            R"({"type": "joint_friction", "name": "f", "joint": "j",
                "friction": 0, "viscous_friction": 20})",
            5.0);
  double q = 0.0, dq = 0.0;
  RunToEnd(context, q, dq);
  EXPECT_NEAR(dq, kWeight / 20.0, 1e-5);
}

TEST(JointFriction, BristlesHoldBelowTheStaticFriction) {
  // the weight, 19.6 N, above the Coulomb friction, below the static; a
  // stribeck_velocity above the 0.05 m/s of the first swing on the bristles,
  // which would otherwise break them loose
  shift::core::Context context;
  LoadModel(context, "jf_hold", kSlider,
            R"({"type": "joint_friction", "name": "f", "joint": "j",
                "friction": 15, "static_friction": 25,
                "bristle_stiffness": 20000, "bristle_damping": 300,
                "stribeck_velocity": 0.2})",
            3.0);
  double q = 0.0, dq = 0.0;
  RunToEnd(context, q, dq);
  // the joint moves more than the bristles bend, as they slip a little on
  // the way: q = -(g / s) ln(1 - s z / g) rising to s z = m g, 1.9 mm, and
  // more with the overshoot
  EXPECT_GT(q, 0.0019);
  EXPECT_LT(q, 0.004);
  EXPECT_NEAR(dq, 0.0, 1e-4);
}

TEST(JointFriction, BristlesLetGoAboveTheStaticFriction) {
  shift::core::Context context;
  LoadModel(context, "jf_break", kSlider,
            R"({"type": "joint_friction", "name": "f", "joint": "j",
                "friction": 8, "static_friction": 12,
                "bristle_stiffness": 20000, "bristle_damping": 300})",
            1.0);
  double q = 0.0, dq = 0.0;
  RunToEnd(context, q, dq);
  EXPECT_NEAR(dq, (kWeight - 8.0) / kMass, 0.3);
}

TEST(JointFriction, RefusesAStaticFrictionBelowTheFriction) {
  shift::core::Context context;
  EXPECT_THROW(LoadModel(context, "jf_bad", kSlider,
                         R"({"type": "joint_friction", "name": "f",
                             "joint": "j", "friction": 8,
                             "static_friction": 4})",
                         0.0),
               shift::core::Exception);
}

TEST(JointFriction, RefusesAJointOfManyCoordinates) {
  shift::core::Context context;
  EXPECT_THROW(
    LoadModel(context, "jf_float",
              R"({"name": "j", "type": "float",
                  "parent": [["fixed"], "origin"],
                  "child": [["body"], "origin"]})",
              R"({"type": "joint_friction", "name": "f", "joint": "j",
                  "friction": 1})",
              0.0),
    shift::core::Exception);
}

static const char* kGrid =
  R"("x_values": [-1, 0, 1], "y_values": [-1, 0, 2],
     "forces": [-3, -2, -1, 0, 0, 0, 4, 6, 8])";

TEST(JointMap, PassesThroughItsPoints) {
  shift::core::Context context;
  LoadModel(context, "jm_grid", kSlider,
            std::string(R"({"type": "joint_map", "name": "m", "joint": "j", )") +
              kGrid + "}",
            0.0);
  JointMap* map = nullptr;
  for(auto module : context.get_active_force_modules()) {
    map = dynamic_cast<JointMap*>(module);
  }
  ASSERT_NE(map, nullptr);
  EXPECT_DOUBLE_EQ(map->Force(-1.0, -1.0), -3.0);
  EXPECT_DOUBLE_EQ(map->Force(1.0, -1.0), -1.0);
  EXPECT_DOUBLE_EQ(map->Force(0.0, 2.0), 6.0);
  EXPECT_DOUBLE_EQ(map->Force(0.5, 0.0), 0.0);
}

TEST(JointMap, ReadsAFile) {
  const std::string path = testing::TempDir() + "/jm.map";
  {
    std::ofstream file(path);
    file << "2 2\n0, 1\n0 1\n10 20\n30 40\n";
  }
  shift::core::Context context;
  LoadModel(context, "jm_file", kSlider,
            R"({"type": "joint_map", "name": "m", "joint": "j",
                "map_file": "jm.map"})",
            0.0);
  std::remove(path.c_str());
  JointMap* map = nullptr;
  for(auto module : context.get_active_force_modules()) {
    map = dynamic_cast<JointMap*>(module);
  }
  ASSERT_NE(map, nullptr);
  EXPECT_DOUBLE_EQ(map->Force(1.0, 1.0), 40.0);
  EXPECT_DOUBLE_EQ(map->Force(1.0, 0.0), 20.0);
}

TEST(JointMap, ForceActsAlongTheJoint) {
  // f = -100 x - 40 v, bilinear over the corners: the body settles at
  // m g / 100
  shift::core::Context context;
  LoadModel(context, "jm_spring", kSlider,
            R"({"type": "joint_map", "name": "m", "joint": "j",
                "x_values": [-1, 1], "y_values": [-10, 10],
                "forces": [500, 300, -300, -500]})",
            5.0);
  double q = 0.0, dq = 0.0;
  RunToEnd(context, q, dq);
  EXPECT_NEAR(q, kWeight / 100.0, 1e-4);
}

TEST(JointMap, RefusesAShortMap) {
  shift::core::Context context;
  EXPECT_THROW(LoadModel(context, "jm_short", kSlider,
                         R"({"type": "joint_map", "name": "m", "joint": "j",
                             "x_values": [0, 1], "y_values": [0, 1],
                             "forces": [1, 2, 3]})",
                         0.0),
               shift::core::Exception);
}

TEST(SphericalJoint, ABushingIsItsSpring) {
  // torsion stiffness about the three rotations of a spherical joint: the
  // bushing between the two sides, with a period of 2 pi sqrt(I / k)
  shift::core::Context context;
  const double k = 5.0, inertia = 0.2;
  const double w = std::sqrt(k / inertia);
  LoadModel(context, "spherical_spring",
            R"({"name": "j", "type": "spherical",
                "parent": [["fixed"], "origin"],
                "child": [["body"], "origin"],
                "initial_velocity": [0, 1, 0]})",
            R"({"type": "bushing", "name": "b",
                "frames": [["fixed", "origin"], ["body", "origin"]],
                "stiffness": [0, 0, 0, 5, 5, 5]})",
            0.5 * M_PI / w);
  std::vector<double> last;
  context.set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context.Simulate();
  ASSERT_FALSE(last.empty());
  // the turn about y from a speed of 1 stops at 1 / w after a quarter period:
  // the quaternion's y is the sine of half of it
  EXPECT_NEAR(last[1], std::sin(0.5 / w), 1e-5);
}
