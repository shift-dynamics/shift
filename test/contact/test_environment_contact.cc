/* the contact model of the environment ("contact" at the root of the model
file): on its own, and overridden by a contact force module of an assembly */
#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"

static const double kGravity = 9.81;
static const double kRadius = 0.1;
static const double kMass = 1.0;
static const double kPoisson = 0.3;

/* a ball dropped on the ground plane of the environment; root and module
are the contact model at the root and the assembly's contact force module,
each empty for none */
static void RunBall(shift::core::Context& context, const std::string& name,
                    const std::string& root, const std::string& module) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ostringstream model;
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-8,
                     "absolute_error": 1e-10, "time_span": 2.0,
                     "time_step": 0.001}]
  },
  "collision_geometry": [{"name": "ground", "type": "plane",
                          "material": "sand"}],
  )" << (root.empty() ? "" : "\"contact\": " + root + ",") << R"(
  "assemblies": [{"name": "drop",
    "bodies": [
      {"name": "ball", "geometry": [],
       "collision_geometry": [{"name": "ball", "type": "sphere",
                               "radius": 0.1, "material": "ball"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 0.004, "Iyy": 0.004, "Izz": 0.004,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}
    ],
    "joints": [
      {"name": "ball_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["ball"], "origin"],
       "initial_displacement": {"T": [0, 0, -0.102]},
       "initial_velocity": [0, 0, 0, 0, 0, 0]}
    ],
    "force_modules": [)" << module << R"(]
  }]
})";
  {
    std::ofstream file(path);
    file << model.str();
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  context.Simulate();
}

/* the depth of the ball in the ground at rest */
static double RestDepth(shift::core::Context& context) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "ball") {
      return context.get_body_origin_pose(body).t.z + kRadius;
    }
  }
  ADD_FAILURE() << "no ball";
  return 0.0;
}

/* Hertz depth of a sphere on a plane carrying its weight,
d = (m g / (4/3 e* sqrt(r)))^(2/3) */
static double HertzDepth(double e_ball, double e_ground) {
  const double nu = 1.0 - kPoisson * kPoisson;
  const double e = 1.0 / (nu / e_ball + nu / e_ground);
  return std::pow(kMass * kGravity / (4.0 / 3.0 * e * std::sqrt(kRadius)),
                  2.0 / 3.0);
}

static std::string Material(const std::string& name, double youngs) {
  std::ostringstream text;
  text << R"({"name": ")" << name << R"(", "youngs_modulus": )" << youngs
       << R"(, "poisson_ratio": 0.3, "restitution": 0.3, "friction": 0.5})";
  return text.str();
}

/* the root contact model alone acts as a contact force module */
TEST(EnvironmentContact, RootContactAlone) {
  shift::core::Context context;
  RunBall(context, "environment_contact_alone",
          "{\"damping_depth\": 1e-4, \"materials\": [" +
            Material("sand", 1e7) + ", " + Material("ball", 1e7) + "]}",
          "");
  int contacts = 0;
  for(auto module : context.get_active_force_modules()) {
    contacts += dynamic_cast<shift::core::force_module::Contact*>(module) ? 1
                                                                          : 0;
  }
  EXPECT_EQ(contacts, 1);
  EXPECT_NEAR(RestDepth(context), HertzDepth(1e7, 1e7),
              0.01 * HertzDepth(1e7, 1e7));
}

/* a module of the assembly replaces the environment's material of the same
name, and the environment's other materials stay in use */
TEST(EnvironmentContact, AssemblyOverridesMaterial) {
  shift::core::Context context;
  RunBall(context, "environment_contact_override",
          "{\"damping_depth\": 1e-4, \"materials\": [" +
            Material("sand", 1e7) + ", " + Material("ball", 1e9) + "]}",
          R"({"name": "contact", "type": "contact", "materials": [)" +
            Material("ball", 2e6) + "]}");
  int contacts = 0;
  for(auto module : context.get_active_force_modules()) {
    contacts += dynamic_cast<shift::core::force_module::Contact*>(module) ? 1
                                                                          : 0;
  }
  EXPECT_EQ(contacts, 1);
  // the assembly's ball (2e6) on the environment's sand (1e7)
  EXPECT_NEAR(RestDepth(context), HertzDepth(2e6, 1e7),
              0.01 * HertzDepth(2e6, 1e7));
}

/* a setting the module does not give is the environment's: damping_depth
far deeper than the rest depth leaves the ball almost undamped, so it is
still bouncing at the end */
TEST(EnvironmentContact, AssemblyTakesEnvironmentSettings) {
  const std::string materials =
    "[" + Material("sand", 1e7) + ", " + Material("ball", 1e7) + "]";
  shift::core::Context given;
  RunBall(given, "environment_contact_settings_given",
          "{\"damping_depth\": 1e-4, \"materials\": " + materials + "}",
          R"({"name": "contact", "type": "contact", "damping_depth": 1.0,
              "materials": [)" + Material("ball", 1e7) + "]}");
  shift::core::Context inherited;
  RunBall(inherited, "environment_contact_settings_inherited",
          "{\"damping_depth\": 1.0, \"materials\": " + materials + "}",
          R"({"name": "contact", "type": "contact", "materials": [)" +
            Material("ball", 1e7) + "]}");
  // the same law, given in the module or taken from the environment
  EXPECT_NEAR(RestDepth(given), RestDepth(inherited), 1e-9);
  // and not that of the module's default damping_depth
  shift::core::Context own;
  RunBall(own, "environment_contact_settings_own",
          "{\"materials\": " + materials + "}",
          R"({"name": "contact", "type": "contact", "materials": [)" +
            Material("ball", 1e7) + "]}");
  EXPECT_GT(std::fabs(RestDepth(own) - RestDepth(inherited)), 1e-6);
}
