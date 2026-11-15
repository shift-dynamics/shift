/* contact forces (force_module::Contact) on a ball or box dropped on the
ground: Hertz rest depth, restitution, rolling, and the linear spring of a
convex hull */
#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "test/contact/contact_record.h"
#include "shift/config.h"
#include "shift/core/body/base.h"
#include "shift/core/integrator/base.h"
#include "shift/core/context.h"

static const double kGravity = 9.81;

struct Drop {
  std::string shape = R"({"name": "ball", "type": "sphere", "radius": 0.1,
                          "material": "ball"})";
  double height = 0.0; //!< of the lowest point above the ground
  double offset = 0.1; //!< from the body origin to the lowest point
  std::string velocity = "[0, 0, 0, 0, 0, 0]";
  std::string rotation = ""; //!< e.g. ", \"R\": [[...]]" of the float joint
  double inertia = 0.004; //!< 2/5 m r^2 of the ball
  double restitution = 0.5;
  std::string damping = ""; //!< e.g. ", \"damping\": 0" of both materials
  double friction = 0.5;
  /*! the ground's restitution and friction, if not negative; else those of
   * the ball */
  double ground_restitution = -1.0;
  double ground_friction = -1.0;
  std::string module_keys = ""; //!< e.g. "\"friction_combine\": \"minimum\", "
  std::string extra_material = "";
  std::string joint = ""; //!< the ball's joint, a float joint if empty
  /*! the contact module's disabled_pairs, e.g. [["ground", "ball"]] */
  std::string disabled_pairs = "[]";
  std::string ground = R"({"name": "ground", "type": "plane",
                           "material": "ground"})";
  double time_span = 1.0;
  std::string integrator = R"("type": "ode113", "relative_error": 1e-8,
                              "absolute_error": 1e-10)";
};

/* a body on a float joint above a ground plane (z down, up is -z) on a body
rigidly joined to the fixed body; simulate, then return the context */
static void RunDrop(shift::core::Context& context, const std::string& name,
                const Drop& drop, bool simulate = true) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, )"
        << kGravity << R"(],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{)"
        << drop.integrator << R"(, "time_span": )" << drop.time_span
        << R"(, "time_step": 0.001}]
  },
  "assemblies": [{"name": "drop",
    "bodies": [
      {"name": "ground", "geometry": [],
       "collision_geometry": [)"
        << drop.ground << R"(],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}},
      {"name": "ball", "geometry": [], "collision_geometry": [)"
        << drop.shape << R"(],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": )"
        << drop.inertia << R"(, "Iyy": )" << drop.inertia << R"(, "Izz": )"
        << drop.inertia << R"(, "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}
    ],
    "joints": [
      {"name": "ground_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["ground"], "origin"]},
      )";
  if(drop.joint.empty()) {
    model << R"({"name": "ball_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["ball"], "origin"],
       "initial_displacement": {"T": [0, 0, )"
          << -(drop.height + drop.offset) << "]" << drop.rotation << R"(},
       "initial_velocity": )"
          << drop.velocity << "}";
  } else {
    model << drop.joint;
  }
  model << R"(
    ],
    "force_modules": [{"name": "contact", "type": "contact", )"
        << drop.module_keys << R"("disabled_pairs": )"
        << drop.disabled_pairs << R"(,
      "materials": [
        {"name": "ground", "youngs_modulus": 1e7, "poisson_ratio": 0.3,
         "restitution": )"
        << (drop.ground_restitution >= 0.0 ? drop.ground_restitution
                                           : drop.restitution)
        << drop.damping << R"(, "friction": )"
        << (drop.ground_friction >= 0.0 ? drop.ground_friction : drop.friction)
        << R"(, "contact_stiffness": 1e5},
        {"name": "ball", "youngs_modulus": 1e7, "poisson_ratio": 0.3,
         "restitution": )"
        << drop.restitution << drop.damping << R"(, "friction": )" << drop.friction
        << R"(, "contact_stiffness": 1e5})"
        << drop.extra_material << R"(
      ]}]
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
  if(simulate) {
    context.Simulate();
    shift::test::RecordContacts(context);
  }
}

static shift::core::body::Base* Ball(shift::core::Context& context) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "ball") {
      return body;
    }
  }
  return nullptr;
}

/* height of the ball's lowest point above the ground, and its velocity */
static double Height(shift::core::Context& context, const Drop& drop) {
  return -context.get_body_origin_pose(Ball(context)).t.z - drop.offset;
}

static ksl_vec3_t Velocity(shift::core::Context& context) {
  auto ball = Ball(context);
  const ksl_screw_t v =
    context.GetBodyVelocity(context.GetBodyLoadIndex(ball));
  const ksl_vec3_t p = context.get_body_origin_pose(ball).t;
  return {{v.lin.x + v.ang.y * p.z - v.ang.z * p.y,
           v.lin.y + v.ang.z * p.x - v.ang.x * p.z,
           v.lin.z + v.ang.x * p.y - v.ang.y * p.x}};
}

TEST(ContactForce, HertzRestDepth) {
  // at rest, m g = 4/3 E* sqrt(R) d^(3/2), with the plane flat (R = r)
  Drop drop;
  drop.height = 0.001;
  drop.restitution = 0.2;
  drop.time_span = 1.5;
  shift::core::Context context;
  RunDrop(context, "rest", drop);
  const double modulus = 1.0 / (2.0 * (1.0 - 0.09) / 1e7);
  const double depth =
    std::pow(kGravity / (4.0 / 3.0 * modulus * std::sqrt(0.1)), 2.0 / 3.0);
  EXPECT_NEAR(-Height(context, drop), depth, 1e-3 * depth);
}

TEST(ContactForce, Restitution) {
  // dropped from h, the ball rebounds to about r^2 h: the damping of the
  // restitution is that of a linear spring, with the Hertz stiffness
  // linearized at damping_depth, so it is approximate
  Drop drop;
  drop.height = 0.1;
  drop.restitution = 0.9;
  const double v_in = std::sqrt(2.0 * kGravity * drop.height);
  drop.time_span =
    std::sqrt(2.0 * drop.height / kGravity) + 0.9 * v_in / kGravity;
  shift::core::Context context;
  RunDrop(context, "restitution", drop);
  const double rebound = std::sqrt(Height(context, drop) / drop.height);
  EXPECT_NEAR(rebound, 0.9, 0.05);
}

/* the height of the box's bottom face at the top of its first rebound,
dropped flat from 0.1 */
static double BoxRebound(const std::string& name, double restitution,
                         const std::string& damping) {
  Drop drop;
  drop.shape = R"({"name": "box", "type": "convex_hull",
                   "half_extents": [0.1, 0.1, 0.1], "material": "ball"})";
  drop.inertia = 1.0 / 6.0 * 0.04;
  drop.height = 0.1;
  drop.restitution = restitution;
  drop.damping = damping;
  const double v_in = std::sqrt(2.0 * kGravity * drop.height);
  const double r = damping.empty() ? restitution : 1.0;
  drop.time_span =
    std::sqrt(2.0 * drop.height / kGravity) + r * v_in / kGravity;
  shift::core::Context context;
  RunDrop(context, name, drop);
  return std::sqrt(Height(context, drop) / drop.height);
}

TEST(ContactForce, HullRestitution) {
  // a box landing flat rebounds with r: on its four corner springs it is a
  // mass on a linear spring, for which the damping of r is exact but for its
  // ramp over damping_depth, small next to the depth of the impact
  for(double r : {0.7, 0.9}) {
    EXPECT_NEAR(BoxRebound("hull_restitution", r, ""), r, 0.02) << r;
  }
  // strongly damped, the force, which never pulls, ends before the spring
  // is relaxed, and the box rebounds higher
  const double rebound = BoxRebound("hull_restitution", 0.3, "");
  EXPECT_GT(rebound, 0.3);
  EXPECT_LT(rebound, 0.4);
}

TEST(ContactForce, DampingOverRestitution) {
  // a material's damping takes the place of its restitution: without
  // damping the box rebounds to its height
  EXPECT_NEAR(BoxRebound("no_damping", 0.3, R"(, "damping": 0)"), 1.0, 0.01);
}

TEST(ContactForce, SlidingBallRolls) {
  // a solid ball launched sliding without spin rolls at 5/7 of its speed
  // once friction has brought the contact point to rest
  Drop drop;
  drop.height = 0.0;
  drop.velocity = "[1.0, 0, 0, 0, 0, 0]";
  drop.time_span = 1.0;
  drop.restitution = 0.2;
  shift::core::Context context;
  RunDrop(context, "rolling", drop);
  const ksl_vec3_t v = Velocity(context);
  EXPECT_NEAR(v.x, 5.0 / 7.0, 0.01);
  EXPECT_NEAR(Height(context, drop), 0.0, 1e-3);
}

TEST(ContactForce, ArchardWear) {
  // a ball sliding at 1 m/s without friction keeps its speed; each of the
  // two materials wears K / H F |u| per unit time, which over 1 s on
  // average carries its weight, m g: K / H m g |u| t
  Drop drop;
  drop.height = 0.0;
  drop.velocity = "[1.0, 0, 0, 0, 0, 0]";
  drop.time_span = 1.0;
  drop.restitution = 0.2;
  drop.friction = 0.0;
  drop.damping = R"(, "wear_coefficient": 1e-9)";
  shift::core::Context context;
  RunDrop(context, "wear", drop);
  const shift::core::force_module::Base* contact = nullptr;
  for(auto module : context.get_active_force_modules()) {
    if(module->get_name() == "contact") {
      contact = module;
    }
  }
  ASSERT_NE(contact, nullptr);
  const auto names = contact->get_output_keys();
  std::vector<double> worn;
  for(const auto& key : contact->get_output_keys()) {
    worn.push_back(contact->GetOutput(key));
  }
  ASSERT_EQ(names.size(), 2u);
  ASSERT_EQ(worn.size(), 2u);
  for(size_t i = 0; i < names.size(); i++) {
    SCOPED_TRACE(names[i]);
    EXPECT_TRUE(names[i] == "wear_ball" || names[i] == "wear_ground");
    EXPECT_NEAR(worn[i], 1e-9 * kGravity, 1e-2 * 1e-9 * kGravity);
  }
  EXPECT_NEAR(Velocity(context).x, 1.0, 1e-9);
}

TEST(ContactForce, HullLinearSpring) {
  // a box on its face rests on a spring at each of its four bottom corners,
  // the stiffnesses of the materials in series (1/k = 2/1e5): 4 k d = m g
  Drop drop;
  drop.shape = R"({"name": "box", "type": "convex_hull",
                   "half_extents": [0.1, 0.1, 0.1], "material": "ball"})";
  drop.inertia = 1.0 / 6.0 * 0.04;
  drop.height = 0.0;
  drop.restitution = 0.1;
  drop.time_span = 2.0;
  shift::core::Context context;
  RunDrop(context, "hull", drop);
  const double k = 0.5e5;
  EXPECT_NEAR(-Height(context, drop), kGravity / (4.0 * k),
              1e-3 * kGravity / (4.0 * k));
  ASSERT_EQ(context.get_contacts().size(), 1u);
  EXPECT_EQ(context.get_contacts()[0]->manifold.size(), 4u);
  // level: the four corners equally deep
  for(const auto& c : context.get_contacts()[0]->manifold) {
    EXPECT_NEAR(c.depth, kGravity / (4.0 * k), 1e-3 * kGravity / (4.0 * k));
  }
}

TEST(ContactForce, BoxOnBox) {
  // the ground is a box with its top face at z = 0: the box rests on the
  // clipped face, its four corners on springs, and does not drift
  Drop drop;
  drop.ground = R"({"name": "ground", "type": "convex_hull",
                    "half_extents": [1.0, 1.0, 0.1], "material": "ground",
                    "pose": {"T": [0, 0, 0.1]}})";
  drop.shape = R"({"name": "box", "type": "convex_hull",
                   "half_extents": [0.1, 0.1, 0.1], "material": "ball"})";
  drop.inertia = 1.0 / 6.0 * 0.04;
  drop.height = 0.01;
  drop.restitution = 0.1;
  drop.time_span = 2.0;
  shift::core::Context context;
  RunDrop(context, "box_on_box", drop);
  const double k = 0.5e5;
  EXPECT_NEAR(-Height(context, drop), kGravity / (4.0 * k),
              1e-3 * kGravity / (4.0 * k));
  ASSERT_EQ(context.get_contacts().size(), 1u);
  EXPECT_EQ(context.get_contacts()[0]->manifold.size(), 4u);
  const ksl_vec3_t t = context.get_body_origin_pose(Ball(context)).t;
  EXPECT_NEAR(t.x, 0.0, 1e-9);
  EXPECT_NEAR(t.y, 0.0, 1e-9);
}

TEST(ContactForce, TiltedBoxSettles) {
  // dropped tilted about two axes, a box lands on a corner, then an edge,
  // and comes to rest on a face, on the ground plane and on a box
  for(bool on_box : {false, true}) {
    Drop drop;
    if(on_box) {
      drop.ground = R"({"name": "ground", "type": "convex_hull",
                        "half_extents": [1.0, 1.0, 0.1], "material": "ground",
                        "pose": {"T": [0, 0, 0.1]}})";
    }
    drop.shape = R"({"name": "box", "type": "convex_hull",
                     "half_extents": [0.1, 0.1, 0.1], "material": "ball"})";
    drop.inertia = 1.0 / 6.0 * 0.04;
    drop.height = 0.1;
    const double c1 = std::cos(0.4), s1 = std::sin(0.4);
    const double c2 = std::cos(0.25), s2 = std::sin(0.25);
    // R = Rx(0.4) Ry(0.25), row major
    std::ostringstream rotation;
    rotation << R"(, "R": [[)" << c2 << ", 0, " << s2 << "], [" << s1 * s2
             << ", " << c1 << ", " << -s1 * c2 << "], [" << -c1 * s2 << ", "
             << s1 << ", " << c1 * c2 << "]]";
    drop.rotation = rotation.str();
    drop.restitution = 0.3;
    drop.time_span = 3.0;
    shift::core::Context context;
    RunDrop(context, on_box ? "tilted_on_box" : "tilted_on_plane", drop);
    const double k = 0.5e5;
    ASSERT_EQ(context.get_contacts().size(), 1u) << on_box;
    const auto& manifold = context.get_contacts()[0]->manifold;
    ASSERT_EQ(manifold.size(), 4u) << on_box;
    for(const auto& c : manifold) {
      EXPECT_NEAR(c.depth, kGravity / (4.0 * k), 0.01 * kGravity / (4.0 * k))
        << on_box;
    }
    const ksl_vec3_t v = Velocity(context);
    EXPECT_LT(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z), 1e-3) << on_box;
  }
}

TEST(ContactForce, DisabledPairsAreNotTested) {
  // the ball, joined to the ground by a revolute joint about the vertical,
  // overlaps it: the joint leaves the pair tested, unless it is disabled
  for(bool exclude : {true, false}) {
    Drop drop;
    drop.shape = R"({"name": "ball", "type": "sphere", "radius": 0.1,
                     "material": "ball", "pose": {"T": [0, 0, -0.05]}})";
    drop.joint = R"({"name": "ball_joint", "type": "revolute",
       "parent": [["ground"], "origin"], "child": [["ball"], "origin"]})";
    drop.disabled_pairs = exclude ? R"([["ground", "ball"]])" : "[]";
    drop.time_span = 0.001;
    shift::core::Context context;
    RunDrop(context, exclude ? "excluded" : "included", drop);
    EXPECT_EQ(context.get_contacts().empty(), exclude);
  }
}

/* the rules that combine the friction or the restitution of two materials,
the value each gives for 0.2 and 0.8 (friction) or 0.7 and 0.95
(restitution), and the module's key that selects it, empty for the default
(the geometric mean) */
struct CombineRule {
  const char* name;
  double friction;
  double restitution;
};

static const CombineRule kCombineRules[] = {
  {"", 0.4, 0.8154753215150045},
  {"geometric_mean", 0.4, 0.8154753215150045},
  {"arithmetic_mean", 0.5, 0.825},
  {"minimum", 0.2, 0.7},
  {"maximum", 0.8, 0.95},
  {"product", 0.16, 0.665}};

static std::string CombineKey(const char* setting, const char* rule) {
  if(rule[0] == '\0') {
    return "";
  }
  return std::string("\"") + setting + "\": \"" + rule + "\", ";
}

/* the speed of a box launched sliding along the ground at 2 m/s, after 0.2 s */
static double SlideSpeed(const std::string& name, const Drop& base) {
  Drop drop = base;
  drop.shape = R"({"name": "box", "type": "convex_hull",
                   "half_extents": [0.1, 0.1, 0.1], "material": "ball"})";
  drop.inertia = 1.0 / 6.0 * 0.04;
  drop.height = 0.0;
  drop.velocity = "[2.0, 0, 0, 0, 0, 0]";
  drop.restitution = 0.2;
  drop.time_span = 0.2;
  shift::core::Context context;
  RunDrop(context, name, drop);
  return Velocity(context).x;
}

TEST(ContactForce, FrictionCombineRules) {
  /* a box (friction 0.2) sliding on the ground (0.8) slows as it does on
  materials that both have the combined friction, the same law: to
  rounding, and at about mu g */
  for(const auto& rule : kCombineRules) {
    Drop mixed;
    mixed.friction = 0.2;
    mixed.ground_friction = 0.8;
    mixed.module_keys = CombineKey("friction_combine", rule.name);
    const double speed = SlideSpeed("friction_combine", mixed);
    Drop same; // the default rule, the geometric mean, keeps equal values
    same.friction = rule.friction;
    EXPECT_NEAR(speed, SlideSpeed("friction_combined", same), 1e-6)
      << rule.name;
    EXPECT_NEAR(speed, 2.0 - rule.friction * kGravity * 0.2, 0.02)
      << rule.name;
  }
}

TEST(ContactForce, RestitutionCombineRules) {
  /* a box (restitution 0.7) dropped flat on the ground (0.95) rebounds as
  on materials that both have the combined restitution: to rounding, and to
  about that restitution */
  for(const auto& rule : kCombineRules) {
    auto rebound = [&](double ball, double ground, const char* key) {
      Drop drop;
      drop.shape = R"({"name": "box", "type": "convex_hull",
                       "half_extents": [0.1, 0.1, 0.1], "material": "ball"})";
      drop.inertia = 1.0 / 6.0 * 0.04;
      drop.height = 0.1;
      drop.restitution = ball;
      drop.ground_restitution = ground;
      drop.module_keys = CombineKey("restitution_combine", key);
      const double v_in = std::sqrt(2.0 * kGravity * drop.height);
      drop.time_span = std::sqrt(2.0 * drop.height / kGravity) +
                       rule.restitution * v_in / kGravity;
      shift::core::Context context;
      RunDrop(context, "restitution_combine", drop);
      return std::sqrt(Height(context, drop) / drop.height);
    };
    const double mixed = rebound(0.7, 0.95, rule.name);
    /* the default rule, the geometric mean, keeps equal values */
    EXPECT_NEAR(mixed, rebound(rule.restitution, rule.restitution, ""), 1e-6)
      << rule.name;
    EXPECT_NEAR(mixed, rule.restitution, 0.02) << rule.name;
  }
}

TEST(ContactForce, UnknownCombineRule) {
  Drop drop;
  drop.module_keys = R"("friction_combine": "harmonic_mean", )";
  shift::core::Context context;
  EXPECT_ANY_THROW(RunDrop(context, "unknown_rule", drop, false));
  Drop restitution;
  restitution.module_keys = R"("restitution_combine": "median", )";
  shift::core::Context context2;
  EXPECT_ANY_THROW(RunDrop(context2, "unknown_rule", restitution, false));
}

TEST(ContactForce, InvalidMaterials) {
  Drop drop;
  drop.shape = R"({"name": "ball", "type": "sphere", "radius": 0.1,
                   "material": "missing"})";
  shift::core::Context context;
  EXPECT_ANY_THROW(RunDrop(context, "missing_material", drop, false));

  Drop bad_restitution;
  bad_restitution.restitution = 1.5;
  shift::core::Context context2;
  EXPECT_ANY_THROW(RunDrop(context2, "bad_restitution", bad_restitution, false));

  // a hull against a material without a contact stiffness
  Drop hull;
  hull.shape = R"({"name": "box", "type": "convex_hull",
                   "half_extents": [0.1, 0.1, 0.1], "material": "soft"})";
  hull.extra_material = R"(, {"name": "soft", "youngs_modulus": 1e6,
                              "restitution": 0.5, "friction": 0.5})";
  shift::core::Context context3;
  EXPECT_ANY_THROW(RunDrop(context3, "no_stiffness", hull, false));
}

/* the collision examples: bunnies or skulls (hulls, ellipsoids and spheres)
dropped on the floor, a plane of the fixed body, come to rest on it, none
through it. The parameter names the example, collision_test_<name>.json, and
holds the number of bodies, e.g. "5", "skull_5" or "skull_5_free". */
class CollisionExample : public testing::TestWithParam<std::string> {};

static std::string CollisionExampleFile(const std::string& name) {
  return std::string(SHIFT_EXAMPLES_DIR) + "/collision/collision_test_" +
         name + ".json";
}

static size_t CollisionExampleBodyCount(const std::string& name) {
  return std::stoul(name.substr(name.find_first_of("0123456789")));
}

/* no body starts in contact with another, or with the floor */
class CollisionExampleStart : public testing::TestWithParam<std::string> {};

TEST_P(CollisionExampleStart, NothingOverlaps) {
  shift::core::Context context;
  context.LoadJSON(CollisionExampleFile(GetParam()));
  context.DetectContacts();
  shift::test::RecordContacts(context);
  for(auto pair : context.get_contacts()) {
    ADD_FAILURE() << pair->a->get_name() << " and " << pair->b->get_name()
                  << " overlap at t = 0";
  }
}

INSTANTIATE_TEST_SUITE_P(Bunnies, CollisionExampleStart,
                         testing::Values("5", "50", "500", "5000", "5_free",
                                         "50_free", "500_free"));
INSTANTIATE_TEST_SUITE_P(Skulls, CollisionExampleStart,
                         testing::Values("skull_5", "skull_50", "skull_500",
                                         "skull_5000", "skull_5_free",
                                         "skull_50_free", "skull_500_free"));

TEST_P(CollisionExample, BodiesRestOnTheGround) {
  shift::core::Context context;
  context.LoadJSON(CollisionExampleFile(GetParam()));

  double ground = 0.0;
  for(auto geometry : context.get_contact_geometry()) {
    if(geometry->get_name() == "floor") {
      ground = geometry->get_d_absolute().t.z;
      EXPECT_TRUE(geometry->is_contact_enabled());
      EXPECT_EQ(geometry->get_parent_body(), context.get_fixed_body());
    }
  }
  EXPECT_EQ(ground, 0.0);
  context.get_active_integrators()[0]->set_time_span(3.0);
  context.Simulate();
  shift::test::RecordContacts(context);

  size_t on_ground = 0;
  for(auto pair : context.get_contacts()) {
    if(pair->a->get_name() == "floor" || pair->b->get_name() == "floor") {
      on_ground++;
    }
  }
  for(auto body : context.get_active_articulated_bodies()) {
    // the lowest point of its collision geometry (+z is down) rests at most
    // a few millimeters into the ground, and the body is near the ground
    for(auto geometry : context.get_contact_geometry()) {
      if(geometry->get_parent_body() != body) {
        continue;
      }
      ksl_vec3_t local, lowest;
      int hint = 0;
      geometry->Support({{0.0, 0.0, 1.0}}, local, lowest, hint);
      EXPECT_LT(lowest.z, ground + 0.01) << body->get_name();
      EXPECT_GT(lowest.z, ground - 0.5) << body->get_name();
    }
  }
  EXPECT_GE(on_ground, CollisionExampleBodyCount(GetParam()) / 2);
}

// 50 bodies take minutes in a debug build; run them with
// --gtest_also_run_disabled_tests
INSTANTIATE_TEST_SUITE_P(Bunnies, CollisionExample, testing::Values("5"));
INSTANTIATE_TEST_SUITE_P(DISABLED_ManyBunnies, CollisionExample,
                         testing::Values("50"));
INSTANTIATE_TEST_SUITE_P(Skulls, CollisionExample, testing::Values("skull_5"));
INSTANTIATE_TEST_SUITE_P(DISABLED_ManySkulls, CollisionExample,
                         testing::Values("skull_50"));
