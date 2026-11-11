/* Area contact of conformal CAD faces: faces of the same shape, as a brake shoe
whose inner face fits its drum, touch over an area, not at a point or along a
line (see collision_geometry::Cad, force_module::Contact::PointLaw) */

#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "ksl/vec3.h"
#include "shift/config.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"
#include "test/contact/cad_scene.h"
#include "test/contact/contact_record.h"

class CadConformal : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }
};

/* the CAD shape of a body whose geometry is the STEP file step */
static std::string ConformalBody(const std::string& step) {
  return R"({"type": "cad", "filename": ")" + step + R"("})";
}

/* drum.step and brake_shoe.step: the drum's radius, the shoe's half angle
about -z from the drum's axis (y) and its width */
static const double kDrum = 0.05;
static const double kShoeHalfAngle = M_PI / 4.0;
static const double kShoeWidth = 0.02;

/* the area of the shoe's inner face, and the integrals over it of cos and
cos^2 of the angle from -z, over R w */
static const double kShoeArea = kDrum * 2.0 * kShoeHalfAngle * kShoeWidth;
static const double kCos = 2.0 * std::sin(kShoeHalfAngle);
static const double kCos2 = kShoeHalfAngle + 0.5 * std::sin(2.0 * kShoeHalfAngle);

/* the pressure per depth of the area contact with a modulus E* = 1, of a punch
of the shoe's area (Contact::PointLaw) */
static const double kFoundation = 2.0 / std::sqrt(M_PI * kShoeArea);

/* the force of each point of a pair, k d^n along its normal, for E* = 1 */
static double PointForce(const shift::core::collision_geometry::ContactPoint& c) {
  double stiffness, exponent;
  shift::core::force_module::Contact::PointLaw(c, 1.0, 1e-4, stiffness,
                                               exponent);
  return c.weight * stiffness * std::pow(c.depth, exponent);
}

TEST_F(CadConformal, BrakeShoeTouchesItsDrumOverTheArc) {
  // the shoe, whose inner face has the drum's radius, pressed 10 um down onto
  // the drum: an area over all of its inner face, each point as deep as the
  // shoe's motion along its normal, delta cos(theta) at the angle theta from
  // -z. The pressure of an elastic foundation, k_f delta cos(theta), gives a
  // normal force of k_f delta R w (alpha + sin(2 alpha) / 2), and the sum of
  // the magnitudes of the point forces over it, on which the friction acts, is
  // the classical drum brake's 2 sin(alpha) / (alpha + sin(2 alpha) / 2) =
  // 1.100 times the normal force
  const double delta = 10e-6;
  shift::test::CadScene scene("cc_shoe", "drum.step", R"("type": "cad")", "",
                              ConformalBody("brake_shoe.step"));
  const auto& pair = scene.At({{0.0, 0.0, delta}});
  ASSERT_TRUE(pair.collision);
  ASSERT_GE(pair.manifold.size(), 64u);
  double area = 0.0, normal = 0.0, sum = 0.0, sideways = 0.0;
  for(const auto& c : pair.manifold) {
    ASSERT_GT(c.area, 0.0);
    EXPECT_EQ(c.length, 0.0);
    EXPECT_NEAR(c.patch_area, kShoeArea, 1e-9 * kShoeArea);
    const double theta = std::atan2(c.point_a.x, -c.point_a.z);
    EXPECT_LE(std::fabs(theta), kShoeHalfAngle);
    EXPECT_NEAR(std::hypot(c.point_a.x, c.point_a.z), kDrum, 1e-9);
    EXPECT_NEAR(c.depth, delta * std::cos(theta), 1e-3 * delta);
    // from the drum to the shoe: outward from the drum's axis
    EXPECT_NEAR(c.normal.x, std::sin(theta), 1e-6);
    EXPECT_NEAR(c.normal.y, 0.0, 1e-9);
    EXPECT_NEAR(c.normal.z, -std::cos(theta), 1e-6);
    area += c.area;
    const double f = PointForce(c);
    normal -= f * c.normal.z;
    sideways += f * c.normal.x;
    sum += f;
  }
  EXPECT_NEAR(area, kShoeArea, 1e-9 * kShoeArea);
  EXPECT_NEAR(normal, kFoundation * delta * kDrum * kShoeWidth * kCos2,
              2e-3 * normal);
  EXPECT_LT(std::fabs(sideways), 1e-9 * normal);
  EXPECT_NEAR(sum / normal, kCos / kCos2, 2e-3);
}

TEST_F(CadConformal, PressingTheShoeInHasNoJump) {
  // from 2 um above the drum to 10 um into it: the contact grows from nothing,
  // each point entering at no depth, so the force has no jump
  shift::test::CadScene scene("cc_shoe_press", "drum.step", R"("type": "cad")",
                              "", ConformalBody("brake_shoe.step"));
  double previous = 0.0, greatest = 0.0, peak = 0.0;
  for(int i = 0; i <= 240; i++) {
    const double z = -2e-6 + 12e-6 * i / 240.0;
    double force = 0.0;
    const auto& pair = scene.At({{0.0, 0.0, z}});
    for(const auto& c : pair.manifold) {
      force -= PointForce(c) * c.normal.z;
    }
    if(std::fabs(z) > 1e-12) {
      EXPECT_EQ(pair.collision, z > 0.0) << z;
    }
    if(i > 0) {
      greatest = std::max(greatest, std::fabs(force - previous));
    }
    previous = force;
    peak = std::max(peak, force);
  }
  ASSERT_GT(peak, 0.0);
  // a step of the depth by a 200th of the last depth
  EXPECT_LT(greatest / peak, 1.5 / 200.0);
}

TEST_F(CadConformal, TiltedShoeLoadsOneSide) {
  // tilted about x by 1e-4 rad and pressed in by 1 um, the shoe's inner face
  // still fits the drum: its depth falls linearly across its width, to
  // nothing at its edge y = -1 um / 1e-4 = -10 mm, and the area holds it with
  // its centre of force towards +y
  const double tilt = 1e-4, delta = 1e-6;
  shift::test::CadScene scene("cc_shoe_tilt", "drum.step", R"("type": "cad")",
                              "", ConformalBody("brake_shoe.step"));
  const double axis[3] = {1.0, 0.0, 0.0};
  double r[3][3];
  shift::test::Rotation(axis, tilt, r);
  const auto& pair = scene.At({{0.0, 0.0, delta}}, r);
  ASSERT_TRUE(pair.collision);
  ASSERT_GE(pair.manifold.size(), 32u);
  double moment = 0.0, force = 0.0;
  for(const auto& c : pair.manifold) {
    ASSERT_GT(c.area, 0.0);
    const double f = -PointForce(c) * c.normal.z;
    force += f;
    moment += f * c.point_a.y;
  }
  // rotating about x by +tilt moves the shoe's +y side down (+z): deeper there
  EXPECT_GT(moment / force, 1e-3);
}

TEST_F(CadConformal, CoverOnTheBumpsIsAreaContact) {
  // the B-spline face of the twelve bumps and its copy facing down, pressed
  // down 5 um: an area over the whole face, each point as deep as the copy's
  // motion along the normal there, 5 um n_z (a point by the face's border
  // carries the mean depth over its part of the area, which the curve of the
  // bumps takes off that by 0.2% at most), and the sum of the areas along z
  // is that of the rectangle of the face, 130 by 100 mm
  const double delta = 5e-6;
  shift::test::CadScene scene("cc_cover", "eggcrate.step", R"("type": "cad")",
                              "", ConformalBody("eggcrate_cover.step"));
  const auto& pair = scene.At({{0.0, 0.0, delta}});
  ASSERT_TRUE(pair.collision);
  ASSERT_GE(pair.manifold.size(), 256u);
  double projected = 0.0;
  for(const auto& c : pair.manifold) {
    ASSERT_GT(c.area, 0.0);
    EXPECT_NEAR(c.depth, delta * std::fabs(c.normal.z), 2e-3 * delta);
    projected += c.area * std::fabs(c.normal.z);
  }
  EXPECT_NEAR(projected, 0.13 * 0.10, 2e-3 * 0.13 * 0.10);
}

TEST_F(CadConformal, CoverOfAnotherShapeIsNot) {
  // the cover scaled by 1.001, so its bumps are 0.1% larger and 0.1% further
  // apart: not the bumps' shape, though all of its samples are near them. It
  // touches the bumps at points, as other B-spline faces do
  shift::test::CadScene scene(
    "cc_cover_scaled", "eggcrate.step", R"("type": "cad")", "",
    R"({"type": "cad", "filename": "eggcrate_cover.step", "scale": 1.001})");
  const auto& pair = scene.At({{0.0, 0.0, 5e-6}});
  ASSERT_TRUE(pair.collision);
  for(const auto& c : pair.manifold) {
    EXPECT_EQ(c.area, 0.0);
  }
}

/* the drum on a revolute joint about y spinning at omega, and the shoe on a
prismatic joint along z resting on it by its weight, from start; the drum's
angular speed after time_span. lining: the layer_thickness of the material, 0
for none */
static double BrakeSim(const std::string& name, double omega, double mass,
                       double drum_inertia, double friction, double modulus,
                       double start, double time_span, double lining = 0.0) {
  std::ostringstream m;
  m.precision(17);
  m << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "include_directories": [")"
    << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true, "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-6,
                     "absolute_error": 1e-9, "time_span": )"
    << time_span << R"(, "time_step": 0.005}]},
    "assemblies": [{"name": "m", "bodies": [
    {"name": "drum", "geometry": [{"type": "cad", "filename": "drum.step"}],
     "collision_geometry": [{"name": "drum", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 2, "Ixx": )"
    << drum_inertia << R"(, "Iyy": )" << drum_inertia << R"(, "Izz": )"
    << drum_inertia << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}},
    {"name": "shoe", "geometry": [{"type": "cad",
                                   "filename": "brake_shoe.step"}],
     "collision_geometry": [{"name": "shoe", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, -0.054],
     "inertia": {"mass": )"
    << mass << R"(, "Ixx": 1e-4, "Iyy": 1e-4, "Izz": 1e-4, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}}],
    "joints": [
    {"name": "spin", "type": "revolute",
     "parent": [["fixed"], "origin"], "child": [["drum"], "origin"],
     "axis": [0, 1, 0], "initial_velocity": )"
    << omega << R"(},
    {"name": "press", "type": "prismatic",
     "parent": [["fixed"], "origin"], "child": [["shoe"], "origin"],
     "axis": [0, 0, 1], "initial_displacement": )"
    << start << R"(, "initial_velocity": 0.0}],
    "force_modules": [{"name": "contact", "type": "contact",
      "damping_depth": 1e-7,
      "materials": [{"name": "steel", "youngs_modulus": )"
    << modulus << R"(, "poisson_ratio": 0.3, "restitution": 0.3,
                     "layer_thickness": )"
    << lining << R"(, "friction": )" << friction << R"(}]}]}]})";
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << m.str();
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.Simulate();
  shift::test::RecordContacts(context);
  /* the shoe moves by less than a micrometer: the kinetic energy is the
  drum's */
  return std::sqrt(2.0 * context.KineticEnergy() / drum_inertia);
}

TEST_F(CadConformal, BrakeShoeSlowsTheDrum) {
  // the drum spinning at 20 rad/s under a 1 kg shoe resting on it by its
  // weight, from where the area holds it: the friction torque is mu R times
  // the sum of the point forces, 1.100 times the weight (see
  // BrakeShoeTouchesItsDrumOverTheArc), so the drum slows at
  // mu R 1.100 m g / I
  const double omega = 20.0, mass = 1.0, inertia = 0.01, friction = 0.3;
  const double modulus = 2e9, time_span = 0.05;
  const double e_star = modulus / (2.0 * (1.0 - 0.3 * 0.3));
  const double rest = mass * 9.81 /
                      (e_star * kFoundation * kDrum * kShoeWidth * kCos2);
  const double end = BrakeSim("cc_brake", omega, mass, inertia, friction,
                              modulus, rest, time_span);
  const double expected =
    friction * kDrum * (kCos / kCos2) * mass * 9.81 / inertia * time_span;
  std::printf("drum slowed by %.6f rad/s, expected %.6f\n", omega - end,
              expected);
  EXPECT_NEAR(omega - end, expected, 2e-3 * expected);
}

TEST_F(CadConformal, LinedBrakeShoeSlowsTheDrumAlike) {
  // both parts with a layer 5 mm thick: the area presses as an elastic
  // foundation of E (1 - nu) / ((1 + nu) (1 - 2 nu) h) per layer, the two in
  // series, and the shoe rests deeper or shallower than on solid parts, but
  // the friction torque is the same, 1.100 mu R m g
  const double omega = 20.0, mass = 1.0, inertia = 0.01, friction = 0.3;
  const double modulus = 2e9, time_span = 0.05, lining = 0.005;
  const double layer = modulus * 0.7 / (1.3 * 0.4 * lining);
  const double rest =
    mass * 9.81 / (0.5 * layer * kDrum * kShoeWidth * kCos2);
  const double end = BrakeSim("cc_brake_lined", omega, mass, inertia, friction,
                              modulus, rest, time_span, lining);
  const double expected =
    friction * kDrum * (kCos / kCos2) * mass * 9.81 / inertia * time_span;
  EXPECT_NEAR(omega - end, expected, 2e-3 * expected);
}

TEST(ContactForce, AreaPointLaw) {
  // a point carrying an area a: k = k_f a, linear, with the foundation given,
  // or that of a punch of the patch area on a half space of modulus E*
  shift::core::collision_geometry::ContactPoint c;
  c.area = 2e-6;
  c.patch_area = 1e-3;
  double stiffness, exponent;
  shift::core::force_module::Contact::PointLaw(c, 1e9, 1e-6, stiffness,
                                               exponent, 3e11);
  EXPECT_EQ(exponent, 1.0);
  EXPECT_NEAR(stiffness, 3e11 * 2e-6, 1e-9 * 3e11 * 2e-6);
  shift::core::force_module::Contact::PointLaw(c, 1e9, 1e-6, stiffness,
                                               exponent);
  EXPECT_NEAR(stiffness, 2.0 * 1e9 / std::sqrt(M_PI * 1e-3) * 2e-6,
              1e-9 * stiffness);
}
