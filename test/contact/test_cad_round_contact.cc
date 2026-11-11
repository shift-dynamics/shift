/* contact of other collision geometry with the exact surfaces of CAD geometry:
spheres and ellipsoids (CadRoundContact), against the distance to a box, the
curvature of the surfaces at a point and the conditions that define a point of
contact; boxes, hulls, cylinders and capsules (CadExactContact), against the same
shapes from STEP files, closed forms and the forces of a sweep */
#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "test/contact/cad_scene.h"
#include "test/contact/contact_record.h"
#include "shift/config.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"


/* box_mm.step: from the origin to these corners */
static const double kBox[3] = {0.01, 0.02, 0.03};

static std::string SphereKeys(double radius) {
  std::ostringstream s;
  s.precision(17);
  s << R"("type": "sphere", "radius": )" << radius;
  return s.str();
}

static std::string EllipsoidKeys(double a, double b, double c) {
  std::ostringstream s;
  s.precision(17);
  s << R"("type": "ellipsoid", "semi_axes": [)" << a << ", " << b << ", " << c
    << "]";
  return s.str();
}

class CadRound : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }
};

TEST_F(CadRound, SphereOnAFace) {
  const double r = 0.004, depth = 1e-4;
  shift::test::CadScene scene("cr_face", "box_mm.step", SphereKeys(r));
  const ksl_vec3_t c = {{0.005, 0.01, kBox[2] + r - depth}};
  for(bool part_first : {true, false}) {
    const shift::core::collision_geometry::ContactPair& p = scene.At(c, nullptr, part_first);
    ASSERT_TRUE(p.collision);
    ASSERT_EQ(p.manifold.size(), 1u);
    const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
    EXPECT_NEAR(m.depth, depth, 1e-12);
    EXPECT_NEAR(m.normal.z, part_first ? 1.0 : -1.0, 1e-12);
    EXPECT_NEAR(m.curvature, 1.0 / r, 1e-6);
    EXPECT_NEAR(m.curvature_ratio, 1.0, 1e-9);
    const ksl_vec3_t& on_part = part_first ? m.point_a : m.point_b;
    const ksl_vec3_t& on_shape = part_first ? m.point_b : m.point_a;
    EXPECT_NEAR(on_part.z, kBox[2], 1e-12);
    EXPECT_NEAR(on_shape.z, kBox[2] - depth, 1e-12);
    EXPECT_NEAR(on_part.x, 0.005, 1e-12);
  }
}

TEST_F(CadRound, SphereApart) {
  const double r = 0.004;
  shift::test::CadScene scene("cr_apart", "box_mm.step", SphereKeys(r));
  EXPECT_FALSE(scene.At({{0.005, 0.01, kBox[2] + r + 1e-6}}).collision);
  EXPECT_FALSE(scene.At({{0.5, 0.5, 0.5}}).collision);
}

TEST_F(CadRound, SphereOnAnEdgeAndACorner) {
  const double r = 0.003, a = 0.002;
  shift::test::CadScene scene("cr_edge", "box_mm.step", SphereKeys(r));
  {
    const shift::core::collision_geometry::ContactPair& p = scene.At({{kBox[0] + a, 0.01, kBox[2] + a}});
    ASSERT_EQ(p.manifold.size(), 1u);
    EXPECT_NEAR(p.manifold[0].depth, r - a * std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(p.manifold[0].normal.x, std::sqrt(0.5), 1e-7);
    EXPECT_NEAR(p.manifold[0].normal.z, std::sqrt(0.5), 1e-7);
    EXPECT_NEAR(p.manifold[0].curvature,
                1.0 / scene.part().get_edge_radius() + 1.0 / r, 1e-6);
  }
  {
    const double b = 0.0015;
    const shift::core::collision_geometry::ContactPair& p =
      scene.At({{kBox[0] + b, kBox[1] + b, kBox[2] + b}});
    ASSERT_EQ(p.manifold.size(), 1u);
    EXPECT_NEAR(p.manifold[0].depth, r - b * std::sqrt(3.0), 1e-12);
    EXPECT_NEAR(p.manifold[0].normal.y, std::sqrt(1.0 / 3.0), 1e-12);
  }
}

TEST_F(CadRound, SphereMatchesTheDistanceToTheBox) {
  // anywhere around the box: one point, of the depth and normal the distance
  // to the box gives, over faces, edges and corners
  const double r = 0.003;
  shift::test::CadScene scene("cr_distance", "box_mm.step", SphereKeys(r));
  std::mt19937 random(1234);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int touching = 0;
  for(int i = 0; i < 3000; i++) {
    /* a point at a distance just below the radius from the box */
    ksl_vec3_t c;
    for(int k = 0; k < 3; k++) {
      c.at[k] = -0.0035 + (kBox[k] + 0.007) * unit(random);
    }
    double outside[3], squared = 0.0;
    bool inside = true;
    for(int k = 0; k < 3; k++) {
      outside[k] = c.at[k] < 0.0     ? c.at[k]
                   : c.at[k] > kBox[k] ? c.at[k] - kBox[k]
                                       : 0.0;
      squared += outside[k] * outside[k];
      inside = inside && outside[k] == 0.0;
    }
    if(inside) {
      continue;
    }
    const double distance = std::sqrt(squared);
    const double depth = r - distance;
    const shift::core::collision_geometry::ContactPair& p = scene.At(c);
    SCOPED_TRACE("at " + std::to_string(c.x) + " " + std::to_string(c.y) +
                 " " + std::to_string(c.z));
    if(depth <= 1e-9 || depth > scene.part().get_max_depth() - 1e-9) {
      continue;
    }
    touching++;
    ASSERT_TRUE(p.collision);
    ASSERT_EQ(p.manifold.size(), 1u);
    EXPECT_NEAR(p.manifold[0].depth, depth, 1e-9);
    for(int k = 0; k < 3; k++) {
      EXPECT_NEAR(p.manifold[0].normal.at[k], outside[k] / distance, 1e-5);
    }
  }
  EXPECT_GT(touching, 100);
}

TEST_F(CadRound, EllipsoidOnAFace) {
  // on the end of its c axis the curvatures are c / a^2 and c / b^2
  const double a = 0.006, b = 0.004, c = 0.002, depth = 1e-4;
  shift::test::CadScene scene("cr_ell_face", "box_mm.step", EllipsoidKeys(a, b, c));
  const shift::core::collision_geometry::ContactPair& p = scene.At({{0.005, 0.01, kBox[2] + c - depth}});
  ASSERT_EQ(p.manifold.size(), 1u);
  const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
  EXPECT_NEAR(m.depth, depth, 1e-12);
  const double k1 = c / (a * a), k2 = c / (b * b);
  EXPECT_NEAR(m.curvature, 0.5 * (k1 + k2), 1e-6);
  EXPECT_NEAR(m.curvature_ratio, k2 / k1, 1e-9);
}

TEST_F(CadRound, TiltedEllipsoidOnAFace) {
  // its lowest point is sqrt(n^T G n) below the center, off to one side
  const double a = 0.006, b = 0.004, c = 0.002;
  const double axis[3] = {0.0, 1.0, 0.0}, angle = 0.6;
  double rotation[3][3];
  shift::test::Rotation(axis, angle, rotation);
  shift::test::CadScene scene("cr_ell_tilt", "box_mm.step", EllipsoidKeys(a, b, c));
  const double reach = std::sqrt(a * a * std::sin(angle) * std::sin(angle) +
                                 c * c * std::cos(angle) * std::cos(angle));
  const double depth = 1e-4;
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{0.005, 0.01, kBox[2] + reach - depth}}, rotation);
  ASSERT_EQ(p.manifold.size(), 1u);
  EXPECT_NEAR(p.manifold[0].depth, depth, 1e-12);
  EXPECT_NEAR(p.manifold[0].normal.z, 1.0, 1e-12);
  // the point of the ellipsoid: G n / reach from the center, n = -z
  const double lateral =
    (c * c - a * a) * std::sin(angle) * std::cos(angle) / reach;
  EXPECT_NEAR(std::fabs(p.manifold[0].point_b.x - 0.005), std::fabs(lateral),
              1e-10);
}

TEST_F(CadRound, EllipsoidOnAnEdgeIsTheDeepestPointOfTheEdge) {
  const double a = 0.006, b = 0.004, c = 0.002;
  const double axis[3] = {0.6, 0.0, 0.8}, angle = 0.5;
  double rotation[3][3];
  shift::test::Rotation(axis, angle, rotation);
  shift::test::CadScene scene("cr_ell_edge", "box_mm.step", EllipsoidKeys(a, b, c));
  const ksl_vec3_t center = {{kBox[0] + 0.0016, 0.012, kBox[2] + 0.0016}};
  const shift::core::collision_geometry::ContactPair& p = scene.At(center, rotation);
  ASSERT_EQ(p.manifold.size(), 1u);
  const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
  EXPECT_NEAR(m.point_a.x, kBox[0], 1e-9);
  EXPECT_NEAR(m.point_a.z, kBox[2], 1e-9);
  // deepest along the edge: the gap to the ellipsoid is square to it (y)
  ksl_vec3_t gap;
  ksl_subtract_vv(&m.point_a, &m.point_b, &gap);
  EXPECT_NEAR(ksl_vec3_l2norm(&gap), m.depth, 1e-9);
  EXPECT_NEAR(gap.y, 0.0, 1e-6 * m.depth + 1e-9);
}

TEST_F(CadRound, SphereOnACylinder) {
  // cylinder_inch.step: radius 1 in, length 2 in, along z
  const double radius = 0.0254, r = 0.005, depth = 1e-4;
  shift::test::CadScene scene("cr_cyl", "cylinder_inch.step", SphereKeys(r));
  const double angle = 0.7, at = radius + r - depth;
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{at * std::cos(angle), at * std::sin(angle), 0.03}});
  ASSERT_EQ(p.manifold.size(), 1u);
  const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
  EXPECT_NEAR(m.depth, depth, 1e-10);
  EXPECT_NEAR(m.normal.x, std::cos(angle), 1e-8);
  const double k1 = 1.0 / r, k2 = 1.0 / r + 1.0 / radius;
  EXPECT_NEAR(m.curvature, 0.5 * (k1 + k2), 1e-5);
  EXPECT_NEAR(m.curvature_ratio, k2 / k1, 1e-6);
}

/* the conditions of a point of contact of a tilted ellipsoid (semi axes s,
rows of r its rotation, center c) from a part to the shape: the point of the
shape is on it, its normal there is against the contact's, and the gap is
along the normal */
static void ExpectEllipsoidPoint(const shift::core::collision_geometry::ContactPoint& m, const double s[3],
                                 const double r[3][3], const ksl_vec3_t& c) {
  double x[3], g[3], implicit = 0.0, length = 0.0;
  for(int i = 0; i < 3; i++) {
    x[i] = 0.0;
    for(int k = 0; k < 3; k++) {
      x[i] += r[k][i] * (m.point_b.at[k] - c.at[k]);
    }
    implicit += x[i] * x[i] / (s[i] * s[i]);
    g[i] = x[i] / (s[i] * s[i]);
    length += g[i] * g[i];
  }
  EXPECT_NEAR(implicit, 1.0, 1e-9);
  length = std::sqrt(length);
  for(int k = 0; k < 3; k++) {
    double outward = 0.0;
    for(int i = 0; i < 3; i++) {
      outward += r[k][i] * g[i] / length;
    }
    EXPECT_NEAR(outward, -m.normal.at[k], 1e-6);
    EXPECT_NEAR(m.point_a.at[k] - m.point_b.at[k], m.depth * m.normal.at[k],
                1e-6 * m.depth + 1e-10);
  }
}

TEST_F(CadRound, TiltedEllipsoidOnACylinder) {
  const double radius = 0.0254;
  const double s[3] = {0.006, 0.004, 0.002};
  const double axis[3] = {0.0, 0.6, 0.8};
  double rotation[3][3];
  shift::test::Rotation(axis, 0.9, rotation);
  shift::test::CadScene scene("cr_ell_cyl", "cylinder_inch.step",
              EllipsoidKeys(s[0], s[1], s[2]));
  // brought in along x until it touches
  bool found = false;
  for(double x = radius + 0.0065; x > radius; x -= 2e-5) {
    const ksl_vec3_t c = {{x, 0.002, 0.03}};
    const shift::core::collision_geometry::ContactPair& p = scene.At(c, rotation);
    if(p.collision) {
      found = true;
      ASSERT_EQ(p.manifold.size(), 1u);
      const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
      EXPECT_LT(m.depth, 3e-5);
      ExpectEllipsoidPoint(m, s, rotation, c);
      EXPECT_NEAR(std::hypot(m.point_a.x, m.point_a.y), radius, 1e-9);
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(CadRound, SphereOnThePoleOfADome) {
  // tube_dome.step: the pole of a sphere of radius 10 mm, at z = 30 mm; its
  // parameters are singular there
  const double r = 0.004, depth = 1e-4;
  shift::test::CadScene scene("cr_pole", "tube_dome.step", SphereKeys(r));
  const shift::core::collision_geometry::ContactPair& p = scene.At({{0.0, 0.0, 0.03 + r - depth}});
  ASSERT_TRUE(p.collision);
  ASSERT_GE(p.manifold.size(), 1u);
  EXPECT_NEAR(p.depth, depth, 1e-9);
  EXPECT_NEAR(std::fabs(p.normal.z), 1.0, 1e-6);
}

TEST_F(CadRound, ThinWallTouchesFromOneSide) {
  // tube_dome.step has no thickness: a ball against the tube touches it from
  // the side the face looks at, and not from behind
  const double r = 0.002, depth = 1e-4;
  shift::test::CadScene scene("cr_wall", "tube_dome.step", SphereKeys(r));
  const shift::core::collision_geometry::ContactPair& outside =
    scene.At({{0.01 + r - depth, 0.0, 0.01}});
  const bool from_outside = outside.collision;
  const shift::core::collision_geometry::ContactPair& inside = scene.At({{0.01 - r + depth, 0.0, 0.01}});
  EXPECT_NE(from_outside, inside.collision);
}

/* rational_patch.step: a quarter of a cylinder of radius 1 about z, from z = 0
to 1, as a rational B-spline surface. The side that touches is the side its
face looks at: sign is +1 if that is outside the cylinder */
static double PatchSide(shift::test::CadScene& scene, double radius, double angle) {
  const double at_out = 1.0 + radius - 1e-3, at_in = 1.0 - radius + 1e-3;
  const ksl_vec3_t out = {{at_out * std::cos(angle), at_out * std::sin(angle),
                           0.5}};
  const ksl_vec3_t in = {{at_in * std::cos(angle), at_in * std::sin(angle),
                          0.5}};
  const bool outside = scene.At(out).collision;
  const bool inside = scene.At(in).collision;
  EXPECT_NE(outside, inside);
  return outside ? 1.0 : -1.0;
}

TEST_F(CadRound, SphereOnANurbsPatch) {
  const double r = 0.1, depth = 1e-3, angle = 0.7;
  shift::test::CadScene scene("cr_nurbs_sphere", "rational_patch.step", SphereKeys(r));
  const double side = PatchSide(scene, r, angle);
  const double at = 1.0 + side * (r - depth);
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{at * std::cos(angle), at * std::sin(angle), 0.5}});
  ASSERT_EQ(p.manifold.size(), 1u);
  const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
  EXPECT_NEAR(m.depth, depth, 1e-8);
  EXPECT_NEAR(m.normal.x, side * std::cos(angle), 1e-7);
  const double k = 1.0 / r, curved = k + side; // the cylinder's radius is 1
  EXPECT_NEAR(m.curvature, 0.5 * (k + curved), 1e-4);
  EXPECT_NEAR(m.curvature_ratio, std::max(k, curved) / std::min(k, curved),
              1e-4);
}

TEST_F(CadRound, EllipsoidOnANurbsPatch) {
  const double s[3] = {0.1, 0.07, 0.05};
  const double axis[3] = {0.48, 0.6, 0.64};
  double rotation[3][3];
  shift::test::Rotation(axis, 0.8, rotation);
  shift::test::CadScene scene("cr_nurbs", "rational_patch.step",
              EllipsoidKeys(s[0], s[1], s[2]));
  const double angle = 0.7;
  shift::test::CadScene probe("cr_nurbs_probe", "rational_patch.step", SphereKeys(0.1));
  const double side = PatchSide(probe, 0.1, angle);
  bool found = false;
  // brought in radially until it touches, in steps below the deepest depth
  for(double gap = 0.12; gap > 0.0 && !found; gap -= 0.002) {
    const double at = 1.0 + side * gap;
    const ksl_vec3_t c = {{at * std::cos(angle), at * std::sin(angle), 0.5}};
    const shift::core::collision_geometry::ContactPair& p = scene.At(c, rotation);
    if(!p.collision) {
      continue;
    }
    found = true;
    ASSERT_EQ(p.manifold.size(), 1u);
    const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
    ExpectEllipsoidPoint(m, s, rotation, c);
    EXPECT_NEAR(std::hypot(m.point_a.x, m.point_a.y), 1.0, 1e-8);
  }
  EXPECT_TRUE(found);
}

/* the sprocket of make_track_step.py: its axis along z, a seat of 30.5 mm radius
about this point on the x axis, open outward */
static const double kSeatRadius = 0.0305;
static const double kSeatCenter = 0.095 / std::sin(2.0 * M_PI / 21.0);

TEST_F(CadRound, SphereInAHollow) {
  // inside the concave seat the gap is curved 1/r - 1/R around it
  const double r = 0.02, depth = 1e-5;
  shift::test::CadScene scene("cr_seat", "sprocket.step", SphereKeys(r));
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{kSeatCenter - kSeatRadius + r - depth, 0.0, 0.0}});
  ASSERT_EQ(p.manifold.size(), 1u);
  const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
  EXPECT_NEAR(m.depth, depth, 1e-9);
  EXPECT_NEAR(m.normal.x, 1.0, 1e-6);
  const double k1 = 1.0 / r - 1.0 / kSeatRadius, k2 = 1.0 / r;
  EXPECT_NEAR(m.curvature, 0.5 * (k1 + k2), 1e-3);
  EXPECT_NEAR(m.curvature_ratio, k2 / k1, 1e-5);
}

TEST_F(CadRound, SphereTooBigForAHollowRestsOnItsRims) {
  // a ball wider than the seat does not reach its bottom, where the depth
  // would be least: it stands on two points, either side
  const double r = 0.05;
  shift::test::CadScene scene("cr_rims", "sprocket.step", SphereKeys(r));
  bool found = false;
  for(double x = kSeatCenter + r; x > kSeatCenter - kSeatRadius + r && !found;
      x -= 1e-5) {
    const shift::core::collision_geometry::ContactPair& p = scene.At({{x, 0.0, 0.0}});
    if(!p.collision) {
      continue;
    }
    found = true;
    ASSERT_EQ(p.manifold.size(), 2u);
    EXPECT_NEAR(p.manifold[0].depth, p.manifold[1].depth, 1e-8);
    EXPECT_NEAR(p.manifold[0].normal.y, -p.manifold[1].normal.y, 1e-6);
    EXPECT_GT(std::fabs(p.manifold[0].normal.y), 0.1);
  }
  EXPECT_TRUE(found);
}

TEST_F(CadRound, BallSettlesOnTheFaceAtItsHertzDepth) {
  // z is down, so the ball rests on the middle of the face z = 0 of the box, at
  // the depth at which Hertz's law carries its weight
  const double r = 0.003, mass = 0.1, modulus = 2e9, poisson = 0.3;
  const double stiffness = 4.0 / 3.0 * modulus /
                           (2.0 * (1.0 - poisson * poisson)) * std::sqrt(r);
  const double depth = std::pow(mass * 9.81 / stiffness, 2.0 / 3.0);
  std::ostringstream m;
  m.precision(17);
  m << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "include_directories": [")"
    << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true, "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-7,
                     "absolute_error": 1e-10, "time_span": 0.5,
                     "time_step": 0.01}]},
    "frames": [{"name": "above", "T": [0.005, 0.01, 0]}],
    "assemblies": [{"name": "m", "bodies": [
    {"name": "part", "geometry": [{"type": "cad", "filename": "box_mm.step"}],
     "collision_geometry": [{"name": "part", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                 "Izx": 0}},
    {"name": "ball", "geometry": [],
     "collision_geometry": [{"name": "shape", "material": "steel", )"
    << SphereKeys(r) << R"(}], "center_of_mass": [0, 0, 0],
     "inertia": {"mass": )" << mass << R"(, "Ixx": 1e-6, "Iyy": 1e-6,
                 "Izz": 1e-6, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
    {"name": "part_joint", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["part"], "origin"]},
    {"name": "ball_joint", "type": "prismatic", "axis": [0, 0, 1],
     "parent": [["fixed"], "above"], "child": [["ball"], "origin"],
     "initial_displacement": )" << -r - 1e-6 << R"(}],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [{"name": "steel", "youngs_modulus": )" << modulus
    << R"(, "poisson_ratio": )" << poisson
    << R"(, "restitution": 0.1, "friction": 0.3}]}]}]})";
  const std::string path = testing::TempDir() + "/cr_settle.json";
  {
    std::ofstream file(path);
    file << m.str();
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  double z = 0.0, rate = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    z = y[0];
    rate = y[1];
  });
  context.Simulate();
  shift::test::RecordContacts(context);
  EXPECT_LT(std::fabs(rate), 1e-4);
  EXPECT_NEAR(z, -r + depth, 0.02 * depth);
}

TEST_F(CadRound, RollingOverAnEdgeHasNoJumpInDepth) {
  // a ball pressed to the same depth from the top face, round the edge, down
  // the side: the point moves from face to edge to face
  const double r = 0.003, depth = 2e-5;
  shift::test::CadScene scene("cr_roll", "box_mm.step", SphereKeys(r));
  double previous = -1.0;
  for(int i = -100; i <= 300; i++) {
    const double s = i / 200.0;
    ksl_vec3_t c = {{0.0, 0.01, 0.0}};
    const double at = r - depth;
    if(s < 0.0) { // over the top face
      c.x = kBox[0] + 0.004 * s;
      c.z = kBox[2] + at;
    } else if(s <= 1.0) { // round the edge
      c.x = kBox[0] + at * std::sin(0.5 * M_PI * s);
      c.z = kBox[2] + at * std::cos(0.5 * M_PI * s);
    } else { // down the side
      c.x = kBox[0] + at;
      c.z = kBox[2] - 0.004 * (s - 1.0);
    }
    const shift::core::collision_geometry::ContactPair& p = scene.At(c);
    ASSERT_TRUE(p.collision) << s;
    ASSERT_EQ(p.manifold.size(), 1u) << s;
    EXPECT_NEAR(p.manifold[0].depth, depth, 1e-9) << s;
    if(previous >= 0.0) {
      EXPECT_NEAR(p.manifold[0].depth, previous, 1e-9) << s;
    }
    previous = p.manifold[0].depth;
  }
}

/* ------------------------------------------------------------------------ */
/* boxes, hulls, cylinders and capsules */

static std::string BoxKeys(double x, double y, double z) {
  std::ostringstream s;
  s.precision(17);
  s << R"("type": "box", "half_extents": [)" << x << ", " << y << ", " << z
    << "]";
  return s.str();
}

static std::string RodKeys(const char* type, double radius, double half) {
  std::ostringstream s;
  s.precision(17);
  s << R"("type": ")" << type << R"(", "radius": )" << radius
    << R"(, "half_length": )" << half;
  return s.str();
}

/* the rotation of angle about y, so that z leans towards x */
static void Lean(double angle, double r[3][3]) {
  const double axis[3] = {0.0, 1.0, 0.0};
  shift::test::Rotation(axis, angle, r);
}

TEST_F(CadRound, BoxIsTheSameBoxFromAStepFile) {
  // box_mm.step as the primitive it is, and as a CAD body whose origin is at
  // its corner: at any pose against the part, the same points
  const double half[3] = {0.5 * kBox[0], 0.5 * kBox[1], 0.5 * kBox[2]};
  shift::test::CadScene primitive("cr_box", "box_mm.step", BoxKeys(half[0], half[1], half[2]));
  shift::test::CadScene step("cr_box_step", "box_mm.step", R"("type": "cad")", "",
             R"({"type": "cad", "filename": "box_mm.step"})");
  std::mt19937 random(4321);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int touching = 0;
  for(int i = 0; i < 400; i++) {
    double axis[3] = {unit(random) - 0.5, unit(random) - 0.5,
                      unit(random) - 0.5};
    const double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] +
                                  axis[2] * axis[2]);
    for(double& a : axis) {
      a /= norm;
    }
    double r[3][3];
    shift::test::Rotation(axis, 3.0 * unit(random), r);
    /* the center, out along a random direction to just touching */
    double direction[3] = {unit(random) - 0.5, unit(random) - 0.5,
                           unit(random) - 0.5};
    const double length = std::sqrt(direction[0] * direction[0] +
                                    direction[1] * direction[1] +
                                    direction[2] * direction[2]);
    ksl_vec3_t center;
    bool found = false;
    for(double reach = 0.05; reach > 0.0 && !found; reach -= 1e-4) {
      for(int k = 0; k < 3; k++) {
        center.at[k] = half[k] + reach * direction[k] / length;
      }
      found = primitive.At(center, r).collision;
    }
    if(!found) {
      continue;
    }
    /* the points of features, and the load of the area of faces resting on
    faces, the integral of the depth over it: its grid follows the parameters
    of the faces, which the box's two descriptions place differently */
    auto features = [](const shift::core::collision_geometry::ContactPair& p) {
      return std::count_if(p.manifold.begin(), p.manifold.end(),
                           [](const shift::core::collision_geometry::ContactPoint& c) {
                             return c.area == 0.0;
                           });
    };
    auto load = [](const shift::core::collision_geometry::ContactPair& p) {
      double sum = 0.0;
      for(const shift::core::collision_geometry::ContactPoint& c : p.manifold) {
        sum += c.weight * c.area * c.depth;
      }
      return sum;
    };
    const shift::core::collision_geometry::ContactPair& a = primitive.At(center, r);
    const double depth_a = a.depth;
    const auto count_a = features(a);
    const double load_a = load(a);
    const ksl_vec3_t normal_a = a.normal;
    /* the body origin of the CAD body: at -half in the box's frame */
    ksl_vec3_t origin = center;
    for(int k = 0; k < 3; k++) {
      for(int j = 0; j < 3; j++) {
        origin.at[k] -= r[k][j] * half[j];
      }
    }
    const shift::core::collision_geometry::ContactPair& b = step.At(origin, r);
    SCOPED_TRACE(i);
    ASSERT_TRUE(b.collision);
    EXPECT_EQ(count_a, features(b));
    EXPECT_NEAR(load_a, load(b), 1e-9 * load_a);
    EXPECT_NEAR(depth_a, b.depth, 1e-9);
    for(int k = 0; k < 3; k++) {
      EXPECT_NEAR(normal_a.at[k], b.normal.at[k], 1e-7);
    }
    touching++;
  }
  EXPECT_GT(touching, 100);
}

TEST_F(CadRound, BoxOnItsFaceRestsOnTheFace) {
  // its 4 by 6 mm face in the part's face: an area of contact over all of it,
  // at the depth, with no points of its edges or corners
  const double depth = 1e-5;
  shift::test::CadScene scene("cr_box_flat", "box_mm.step", BoxKeys(0.002, 0.003, 0.004));
  const shift::core::collision_geometry::ContactPair& p = scene.At({{0.005, 0.01, kBox[2] + 0.004 - depth}});
  ASSERT_GE(p.manifold.size(), 64u);
  double area = 0.0, x = 0.0, y = 0.0;
  for(const shift::core::collision_geometry::ContactPoint& m : p.manifold) {
    ASSERT_GT(m.area, 0.0);
    EXPECT_EQ(m.weight, 1.0);
    EXPECT_NEAR(m.depth, depth, 1e-12);
    EXPECT_NEAR(m.normal.z, 1.0, 1e-12);
    area += m.area;
    x += m.area * m.point_a.x;
    y += m.area * m.point_a.y;
  }
  EXPECT_NEAR(area, 0.004 * 0.006, 1e-12);
  EXPECT_NEAR(x / area, 0.005, 1e-12);
  EXPECT_NEAR(y / area, 0.01, 1e-12);
}

TEST_F(CadRound, BoxTippingOntoAnEdgeHasNoStep) {
  // its face tipping over 0.1 rad, its edge in the part at the depth: the
  // area of the faces gives way to the edge's line as the face lifts
  const double depth = 1e-5, h[3] = {0.002, 0.003, 0.004};
  shift::test::CadScene scene("cr_tip_box", "box_mm.step", BoxKeys(h[0], h[1], h[2]));
  const int count = 4000;
  double previous = 0.0, greatest = 0.0;
  bool area = false, edge = false;
  for(int i = 0; i <= count; i++) {
    const double tilt = 0.1 * i / count;
    double r[3][3];
    Lean(tilt, r);
    const shift::core::collision_geometry::ContactPair& p = scene.At(
      {{0.005, 0.01, kBox[2] + h[2] * std::cos(tilt) + h[0] * std::sin(tilt) - depth}}, r);
    ASSERT_TRUE(p.collision);
    EXPECT_NEAR(p.depth, depth, 1e-12);
    for(const shift::core::collision_geometry::ContactPoint& m : p.manifold) {
      area = area || m.area > 0.0;
      edge = edge || m.area == 0.0;
    }
    const double force = shift::test::ElasticForce(p);
    if(previous > 0.0) {
      greatest = std::max(greatest, std::fabs(force / previous - 1.0));
    }
    previous = force;
  }
  EXPECT_TRUE(area);
  EXPECT_TRUE(edge);
  EXPECT_LT(greatest, 0.01);
}

TEST_F(CadRound, BoxCornerOnAFace) {
  // leaning over two axes, its lowest corner: the sum of the reach of its
  // half extents along the normal, with the order of the pair
  const double h[3] = {0.002, 0.003, 0.004};
  const double axis[3] = {0.6, 0.8, 0.0}, depth = 2e-5;
  double r[3][3];
  shift::test::Rotation(axis, 0.5, r);
  shift::test::CadScene scene("cr_box_corner", "box_mm.step", BoxKeys(h[0], h[1], h[2]));
  double reach = 0.0;
  for(int j = 0; j < 3; j++) {
    reach += h[j] * std::fabs(r[2][j]);
  }
  const ksl_vec3_t c = {{0.005, 0.01, kBox[2] + reach - depth}};
  for(bool part_first : {true, false}) {
    const shift::core::collision_geometry::ContactPair& p = scene.At(c, r, part_first);
    ASSERT_EQ(p.manifold.size(), 1u);
    EXPECT_NEAR(p.manifold[0].depth, depth, 1e-12);
    EXPECT_NEAR(p.manifold[0].normal.z, part_first ? 1.0 : -1.0, 1e-12);
    EXPECT_NEAR(p.manifold[0].curvature,
                1.0 / (0.002 * scene.shape().get_exact_shape()->size), 1e-6);
  }
}

TEST_F(CadRound, EdgeOfThePartOnAFaceOfABox) {
  // the part's corner into the broad face of a box above it
  const double half = 0.05, depth = 1e-5;
  const double axis[3] = {0.6, 0.8, 0.0};
  double r[3][3];
  shift::test::Rotation(axis, 0.6, r);
  shift::test::CadScene scene("cr_box_big", "box_mm.step", BoxKeys(half, half, half));
  // the box's -z face, normal n = -z of the box; lowest over the part's corners
  double n[3] = {-r[0][2], -r[1][2], -r[2][2]}, reach = -HUGE_VAL;
  for(int i = 0; i < 8; i++) {
    const double x = (i & 1) ? kBox[0] : 0.0, y = (i & 2) ? kBox[1] : 0.0,
                 z = (i & 4) ? kBox[2] : 0.0;
    reach = std::max(reach, -(n[0] * x + n[1] * y + n[2] * z));
  }
  // the face's plane at -n . x = reach - depth, center half beyond it
  const double at = reach - depth + half;
  const ksl_vec3_t c = {{-n[0] * at, -n[1] * at, -n[2] * at}};
  const shift::core::collision_geometry::ContactPair& p = scene.At(c, r);
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, depth, 1e-10);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(p.normal.at[k], -n[k], 1e-9);
  }
}

TEST_F(CadRound, BoxFaceOnANurbsPatchIsALine) {
  // a broad face of a box on the outside of the quarter cylinder of radius 1
  const double half = 0.2, depth = 1e-4, angle = 0.7;
  shift::test::CadScene probe("cr_patch_probe", "rational_patch.step", SphereKeys(0.1));
  const double side = PatchSide(probe, 0.1, angle);
  shift::test::CadScene scene("cr_patch_box", "rational_patch.step", BoxKeys(half, half, half));
  double r[3][3];
  const double axis[3] = {0.0, 0.0, 1.0};
  shift::test::Rotation(axis, angle, r); // x of the box along the radius
  if(side < 0.0) {
    GTEST_SKIP() << "the patch looks inward: a flat face touches at its edges";
  }
  const double at = 1.0 + half - depth;
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{at * std::cos(angle), at * std::sin(angle), 0.5}}, r);
  ASSERT_GE(p.manifold.size(), 3u);
  double length = 0.0;
  for(const shift::core::collision_geometry::ContactPoint& m : p.manifold) {
    EXPECT_GT(m.length, 0.0);
    EXPECT_NEAR(m.depth, depth, 1e-8);
    EXPECT_NEAR(m.normal.x, std::cos(angle), 1e-6);
    length += m.weight * m.length;
  }
  // the line is along z, as long as the face's height
  EXPECT_NEAR(length, 2.0 * half, 1e-6);
}

TEST_F(CadRound, HullOfAMeshOnAFace) {
  // a real hull: its lowest vertex onto the face of the part
  const double depth = 1e-6;
  shift::test::CadScene scene("cr_hull", "box_mm.step", R"("type": "convex_hull")", "",
              R"({"type": "mesh", "filename": ["link.obj"]})");
  auto hull = dynamic_cast<const shift::core::collision_geometry::ConvexHull*>(
    &scene.shape());
  ASSERT_NE(hull, nullptr);
  ASSERT_NE(hull->get_exact_shape(), nullptr);
  ASSERT_FALSE(hull->get_hull().polygons.empty());
  EXPECT_EQ(hull->get_exact_shape()->faces.size(),
            hull->get_hull().polygons.size());
  // a turn that leaves one lowest vertex
  const double axis[3] = {0.48, 0.6, 0.64};
  double r[3][3];
  shift::test::Rotation(axis, 1.1, r);
  ksl_vec3_t lowest = {{0.0, 0.0, 0.0}};
  double least = HUGE_VAL;
  for(const auto& v : hull->get_hull().vertices) {
    double z = 0.0;
    for(int j = 0; j < 3; j++) {
      z += r[2][j] * v.at[j];
    }
    if(z < least) {
      least = z;
      for(int k = 0; k < 3; k++) {
        lowest.at[k] = 0.0;
        for(int j = 0; j < 3; j++) {
          lowest.at[k] += r[k][j] * v.at[j];
        }
      }
    }
  }
  const ksl_vec3_t c = {{0.005 - lowest.x, 0.01 - lowest.y,
                         kBox[2] - depth - lowest.z}};
  const shift::core::collision_geometry::ContactPair& p = scene.At(c, r);
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, depth, 1e-10);
  EXPECT_NEAR(p.normal.z, 1.0, 1e-9);
}

TEST_F(CadRound, CylinderOnAFaceIsALine) {
  // lying on the long top of the part, along y
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cyl_line", "box_mm.step", RodKeys("cylinder", radius, half));
  const double axis[3] = {1.0, 0.0, 0.0};
  double r[3][3];
  shift::test::Rotation(axis, 0.5 * M_PI, r);
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{0.005, 0.01, kBox[2] + radius - depth}}, r);
  ASSERT_GE(p.manifold.size(), 3u);
  double length = 0.0;
  for(const shift::core::collision_geometry::ContactPoint& m : p.manifold) {
    EXPECT_NEAR(m.depth, depth, 1e-10);
    EXPECT_NEAR(m.normal.z, 1.0, 1e-9);
    length += m.weight * m.length;
  }
  EXPECT_NEAR(length, 2.0 * half, 1e-9);
}

TEST_F(CadRound, CylinderOnItsEndRestsOnTheEnd) {
  // its end, a disk, in the face: an area of contact over the disk, about the
  // axis; its trims, a polygon of the four arcs, are a little smaller
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cyl_end", "box_mm.step", RodKeys("cylinder", radius, half));
  const shift::core::collision_geometry::ContactPair& p = scene.At({{0.005, 0.01, kBox[2] + half - depth}});
  ASSERT_GE(p.manifold.size(), 64u);
  double area = 0.0, x = 0.0, y = 0.0;
  for(const shift::core::collision_geometry::ContactPoint& m : p.manifold) {
    ASSERT_GT(m.area, 0.0);
    EXPECT_NEAR(m.depth, depth, 1e-11);
    area += m.area;
    x += m.area * m.point_a.x;
    y += m.area * m.point_a.y;
  }
  EXPECT_LT(area, M_PI * radius * radius);
  EXPECT_GT(area, 0.99 * M_PI * radius * radius);
  EXPECT_NEAR(x / area, 0.005, 1e-9);
  EXPECT_NEAR(y / area, 0.01, 1e-9);
}

TEST_F(CadRound, CylinderIsTheSameCylinderFromAStepFile) {
  // cylinder_inch.step, from z = 0 to 2 in: the deepest point at any pose, though
  // its rim is one edge and the primitive's four
  const double radius = 0.0254, half = 0.0254;
  shift::test::CadScene primitive("cr_cyl_same", "box_mm.step",
                  RodKeys("cylinder", radius, half));
  shift::test::CadScene step("cr_cyl_step", "box_mm.step", R"("type": "cad")", "",
             R"({"type": "cad", "filename": "cylinder_inch.step"})");
  std::mt19937 random(777);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int touching = 0;
  for(int i = 0; i < 300; i++) {
    double axis[3] = {unit(random) - 0.5, unit(random) - 0.5,
                      unit(random) - 0.5};
    const double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] +
                                  axis[2] * axis[2]);
    for(double& a : axis) {
      a /= norm;
    }
    double r[3][3];
    shift::test::Rotation(axis, 3.0 * unit(random), r);
    const double angle = 2.0 * M_PI * unit(random);
    ksl_vec3_t center;
    bool found = false;
    auto place = [&](double reach) {
      center = {{0.005 + reach * std::cos(angle),
                 0.01 + reach * std::sin(angle), 0.015 + 0.3 * reach}};
      return primitive.At(center, r).collision;
    };
    double reach = 0.12;
    for(; reach > 0.0 && !found; reach -= 1e-4) {
      found = place(reach);
    }
    if(!found) {
      continue;
    }
    place(reach + 1e-4);
    const double depth = primitive.At(center, r).depth;
    ksl_vec3_t origin = center;
    for(int k = 0; k < 3; k++) {
      origin.at[k] -= r[k][2] * half;
    }
    const shift::core::collision_geometry::ContactPair& b = step.At(origin, r);
    SCOPED_TRACE(i);
    ASSERT_TRUE(b.collision);
    EXPECT_NEAR(depth, b.depth, 1e-8);
    touching++;
  }
  EXPECT_GT(touching, 100);
}

TEST_F(CadRound, EdgeAlongACylinderOfAStepPart) {
  // a cylinder coming off the top of the box over its edge along y, 20 mm long:
  // the edge is a line on it, the same on cylinder_inch.step as on a primitive
  const double radius = 0.0254, half = 0.0254, depth = 20e-6;
  shift::test::CadScene primitive("cr_edge_line", "box_mm.step",
                  RodKeys("cylinder", radius, half));
  shift::test::CadScene step("cr_edge_line_step", "box_mm.step", R"("type": "cad")", "",
             R"({"type": "cad", "filename": "cylinder_inch.step"})");
  const double r[3][3] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}; // axis along -y
  const double tilt = 0.1;
  const ksl_vec3_t center = {{kBox[0] + (radius - depth) * std::sin(tilt), 0.01,
                              kBox[2] + (radius - depth) * std::cos(tilt)}};
  const ksl_vec3_t origin = {{center.x, center.y + half, center.z}};
  auto length = [](const shift::core::collision_geometry::ContactPair& pair) {
    double sum = 0.0;
    for(const auto& c : pair.manifold) {
      sum += c.length;
    }
    return sum;
  };
  const auto& a = primitive.At(center, r);
  ASSERT_TRUE(a.collision);
  EXPECT_NEAR(length(a), kBox[1], 1e-6);
  const auto& b = step.At(origin, r);
  ASSERT_TRUE(b.collision);
  EXPECT_NEAR(length(b), kBox[1], 1e-6);
  EXPECT_NEAR(b.depth, depth, 1e-8);
}

TEST_F(CadRound, SpinningACylinderChangesNothing) {
  // cylinder_inch.step just past the edge of the box, its line off the top face
  // and on the edge: turning it about its axis takes its seam past the face,
  // flat on it, which must add no point
  const double radius = 0.0254, half = 0.0254, depth = 20e-6, past = 0.5e-3;
  shift::test::CadScene step("cr_seam", "box_mm.step", R"("type": "cad")", "",
             R"({"type": "cad", "filename": "cylinder_inch.step"})");
  const ksl_vec3_t origin = {
    {kBox[0] + past, 0.01 + half,
     kBox[2] + std::sqrt(radius * radius - past * past) - depth}};
  size_t fewest = 1000, most = 0;
  double least = HUGE_VAL, greatest = 0.0;
  for(int i = 0; i < 1800; i++) {
    const double angle = 2.0 * M_PI * i / 1800.0;
    const double c = std::cos(angle), s = std::sin(angle);
    const double r[3][3] = {{c, -s, 0}, {0, 0, -1}, {s, c, 0}}; // axis along -y
    const auto& pair = step.At(origin, r);
    ASSERT_TRUE(pair.collision) << i;
    fewest = std::min(fewest, pair.manifold.size());
    most = std::max(most, pair.manifold.size());
    const double force = shift::test::ElasticForce(pair);
    least = std::min(least, force);
    greatest = std::max(greatest, force);
  }
  EXPECT_EQ(fewest, most);
  EXPECT_NEAR(least, greatest, 1e-6 * greatest);
}

TEST_F(CadRound, EdgeCrossesAnEdge) {
  // an edge of a box pressed across the part's edge along y at x = 10 mm,
  // z = 30 mm: the closest points of the two edges, along their cross product,
  // whichever way the box's edge runs across
  const double h = 0.003, depth = 2e-5;
  shift::test::CadScene scene("cr_edge_edge", "box_mm.step", BoxKeys(h, h, 0.006));
  const double c = std::sqrt(0.5);
  const double n[3] = {c, 0.0, c}, pa[3] = {kBox[0], 0.01, kBox[2]};
  for(double twist : {0.0, 0.3, -0.5}) {
    // the box's edge across w and n, turned by twist from e about n; its
    // faces either side of -n
    const double e[3] = {c, 0.0, -c}; // y, e and n are a right-handed frame
    double w[3];
    for(int k = 0; k < 3; k++) {
      w[k] = std::cos(twist) * (k == 1 ? 1.0 : 0.0) - std::sin(twist) * e[k];
    }
    double r[3][3];
    ksl_vec3_t center;
    for(int k = 0; k < 3; k++) {
      r[k][0] = c * (-n[k] + w[k]);
      r[k][1] = c * (-n[k] - w[k]);
      r[k][2] = (w[(k + 1) % 3] * n[(k + 2) % 3] -
                 w[(k + 2) % 3] * n[(k + 1) % 3]);
      center.at[k] = pa[k] + (std::sqrt(2.0) * h - depth) * n[k];
    }
    for(bool part_first : {true, false}) {
      const shift::core::collision_geometry::ContactPair& p = scene.At(center, r, part_first);
      SCOPED_TRACE(twist);
      ASSERT_EQ(p.manifold.size(), 1u);
      const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
      EXPECT_NEAR(m.depth, depth, 1e-11);
      const double sense = part_first ? 1.0 : -1.0;
      const ksl_vec3_t& on_part = part_first ? m.point_a : m.point_b;
      for(int k = 0; k < 3; k++) {
        EXPECT_NEAR(m.normal.at[k], sense * n[k], 1e-7);
        EXPECT_NEAR(on_part.at[k], pa[k], 1e-9);
      }
    }
  }
}

TEST_F(CadRound, EdgeRadiusOfAShape) {
  const double depth = 1e-5, axis[3] = {0.6, 0.8, 0.0};
  double r[3][3];
  shift::test::Rotation(axis, 0.5, r);
  shift::test::CadScene scene("cr_radius", "box_mm.step",
              BoxKeys(0.002, 0.003, 0.004) + R"(, "edge_radius": 0.0005)");
  double reach = 0.0;
  const double h[3] = {0.002, 0.003, 0.004};
  for(int j = 0; j < 3; j++) {
    reach += h[j] * std::fabs(r[2][j]);
  }
  const shift::core::collision_geometry::ContactPair& p = scene.At({{0.005, 0.01, kBox[2] + reach - depth}}, r);
  ASSERT_EQ(p.manifold.size(), 1u);
  EXPECT_NEAR(p.manifold[0].curvature, 1.0 / 0.0005, 1e-6);
}

TEST_F(CadRound, MaxDepthOfAShape) {
  // the part's own default, 2% of its size, 0.75 mm, is shallower than the
  // corner: the corner is in the part, and found as deep with either
  const double half = 0.004, depth = 0.001;
  double r[3][3];
  Lean(0.2, r);
  const ksl_vec3_t leaning = {{0.005, 0.01, kBox[2] + half * std::cos(0.2) +
                                              0.002 * std::sin(0.2) - depth}};
  shift::test::CadScene small("cr_depth_a", "box_mm.step", BoxKeys(0.002, 0.003, half));
  ASSERT_LT(small.part().get_max_depth(), depth);
  const shift::core::collision_geometry::ContactPair& found = small.At(leaning, r);
  ASSERT_TRUE(found.collision);
  EXPECT_NEAR(found.depth, depth, 1e-9);
  shift::test::CadScene deep("cr_depth_b", "box_mm.step",
             BoxKeys(0.002, 0.003, half) + R"(, "max_depth": 0.002)");
  const shift::core::collision_geometry::ContactPair& p = deep.At(leaning, r);
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, depth, 1e-9);
}

TEST_F(CadRound, FlatHullHasAnExactShape) {
  // a mesh with no thickness: a square plate, then a single line of points
  const std::string path = testing::TempDir() + "/cr_flat.obj";
  {
    std::ofstream file(path);
    file << "v -0.01 -0.01 0\nv 0.01 -0.01 0\nv 0.01 0.01 0\nv -0.01 0.01 0\n"
         << "f 1 2 3\nf 1 3 4\n";
  }
  shift::test::CadScene scene("cr_flat", "box_mm.step", R"("type": "convex_hull")", "",
              R"({"type": "mesh", "filename": [")" + path + R"("]})");
  std::remove(path.c_str());
  ASSERT_NE(scene.shape().get_exact_shape(), nullptr);
  EXPECT_EQ(scene.shape().get_exact_shape()->faces.size(), 6u);
  // a corner of the plate onto the top of the part
  const double axis[3] = {0.6, 0.8, 0.0}, depth = 1e-5;
  double r[3][3];
  shift::test::Rotation(axis, 0.5, r);
  // the lowest corner over the middle of the part
  double corner[3];
  for(int k = 0; k < 3; k++) {
    corner[k] = 0.0;
    for(int j = 0; j < 2; j++) {
      corner[k] -= 0.01 * std::copysign(1.0, r[2][j]) * r[k][j];
    }
  }
  const shift::core::collision_geometry::ContactPair& p = scene.At(
    {{0.005 - corner[0], 0.01 - corner[1], kBox[2] - corner[2] - depth}}, r);
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, depth, 3e-6); // and the plate's thickness, 2.8e-6
  EXPECT_NEAR(p.normal.z, 1.0, 1e-9);
}

/* torus.step standing in the slot of channel.step (scripts/make_channel_step.py)
on its straight start, along x: the torus's axis along y, the bottom of its
tube over the slot's bottom at x = kTorusX */
static const double kTorusX = 0.1;

/* the torus's rotation, rows: its x down (the bottom of the tube at u = 0, in
the middle of an outer face), its z, the axis, along y */
static const double kStanding[3][3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};

/* the height (z, down) of the torus's center at which it first touches the
slot, standing in it at y */
static double FirstTouch(shift::test::CadScene& scene, double y) {
  double apart = -0.08, deep = -0.05;
  for(int i = 0; i < 60; i++) {
    const double z = 0.5 * (apart + deep);
    (scene.At({{kTorusX, y, z}}, kStanding).collision ? deep : apart) = z;
  }
  return apart;
}

TEST_F(CadRound, TorusInASlotRestsOnBothFlanks) {
  // the section of the slot is one B-spline curve, so the slot is one face, and
  // the tube touches its two flanks, either side of the slot's bottom, on one
  // face of the torus: two points of the one pair of faces, mirrored in the
  // plane of the torus
  shift::test::CadScene scene("cr_torus_slot", "channel.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double z = FirstTouch(scene, 0.0), sink = 20e-6;
  const auto& p = scene.At({{kTorusX, 0.0, z + sink}}, kStanding);
  ASSERT_TRUE(p.collision);
  ASSERT_EQ(p.manifold.size(), 2u);
  const auto& left = p.manifold[0].point_a.y > 0.0 ? p.manifold[0] : p.manifold[1];
  const auto& right = &left == &p.manifold[0] ? p.manifold[1] : p.manifold[0];
  EXPECT_GT(left.point_a.y, 0.005);
  EXPECT_NEAR(left.point_a.y, -right.point_a.y, 1e-9);
  EXPECT_NEAR(left.point_a.x, kTorusX, 1e-8);
  EXPECT_NEAR(right.point_a.x, kTorusX, 1e-8);
  EXPECT_NEAR(left.point_a.z, right.point_a.z, 1e-9);
  EXPECT_NEAR(left.depth, right.depth, 1e-9);
  // the normals, from the slot to the torus, up and in towards each other
  EXPECT_NEAR(left.normal.y, -right.normal.y, 1e-9);
  EXPECT_LT(left.normal.y, -0.3);
  EXPECT_LT(left.normal.z, -0.3);
  // sunk by sink, each flank is in by sink along its normal, less the change of
  // the gap as the points move along the curved faces, of the order of sink
  // squared over their radii of curvature
  EXPECT_NEAR(left.depth, -sink * left.normal.z, 1e-7);
}

TEST_F(CadRound, TorusOffTheMiddleOfASlotStillRestsOnBothFlanks) {
  // moved a little across the slot, the torus is deeper in one flank than the
  // other: the shallower point is found too, its depth less by the move along
  // the normals
  shift::test::CadScene scene("cr_torus_aside", "channel.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double z = FirstTouch(scene, 0.0), sink = 20e-6, aside = 8e-6;
  const auto& centered = scene.At({{kTorusX, 0.0, z + sink}}, kStanding);
  ASSERT_EQ(centered.manifold.size(), 2u);
  const double depth = centered.manifold[0].depth;
  const auto& p = scene.At({{kTorusX, aside, z + sink}}, kStanding);
  ASSERT_TRUE(p.collision);
  ASSERT_EQ(p.manifold.size(), 2u);
  for(const auto& c : p.manifold) {
    // the torus moved by aside along y: in deeper where the normal is along -y
    EXPECT_NEAR(c.depth, depth - aside * c.normal.y, 1e-8);
  }
}

TEST_F(CadRound, TurnedTorusKeepsBothPointsInTheSlot) {
  // the torus turned about its axis through a full turn in 400 poses, sunk by
  // 10 um: the points on the flanks cross every part of its outer faces and
  // the seams between them, and at each pose both are found, at one depth
  shift::test::CadScene scene("cr_torus_turned", "channel.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double z = FirstTouch(scene, 0.0), sink = 10e-6;
  const auto& start = scene.At({{kTorusX, 0.0, z + sink}}, kStanding);
  ASSERT_EQ(start.manifold.size(), 2u);
  const double depth = start.manifold[0].depth;
  int both = 0;
  for(int i = 0; i < 400; i++) {
    // turned about its axis, y, by angle: R_y(angle) times the standing
    // rotation, rows
    const double angle = 2.0 * M_PI * i / 400.0, c = std::cos(angle),
                 s = std::sin(angle);
    const double turn[3][3] = {{c, 0, s}, {0, 1, 0}, {-s, 0, c}};
    double r[3][3];
    for(int j = 0; j < 3; j++) {
      for(int k = 0; k < 3; k++) {
        r[j][k] = 0.0;
        for(int l = 0; l < 3; l++) {
          r[j][k] += turn[j][l] * kStanding[l][k];
        }
      }
    }
    const auto& p = scene.At({{kTorusX, 0.0, z + sink}}, r);
    SCOPED_TRACE(i);
    EXPECT_EQ(p.manifold.size(), 2u);
    if(p.manifold.size() == 2u) {
      both++;
      for(const auto& m : p.manifold) {
        EXPECT_NEAR(m.depth, depth, 1e-9);
      }
    }
  }
  EXPECT_EQ(both, 400);
}

/* torus.step, steel, its mass and inertia those of a solid torus of 40 mm
major and 10 mm minor radius: 2 pi^2 R r^2 rho, I about the axis
m (R^2 + 3/4 r^2), about a diameter m (R^2 / 2 + 5/8 r^2) */
static const double kTorusMass = 2.0 * M_PI * M_PI * 0.04 * 0.01 * 0.01 * 7850.0;
static const double kTorusAxial = kTorusMass * (0.04 * 0.04 + 0.75 * 0.01 * 0.01);
static const double kTorusDiametral =
  kTorusMass * (0.5 * 0.04 * 0.04 + 0.625 * 0.01 * 0.01);

/* the torus standing in the slot of the channel at x = kTorusX, its center at
height z, rolling along x at speed: turning about its axis (y) so that the
points at radius off its center, where it rests on the flanks, do not slip */
static std::string RollingTorusModel(double z, double speed, double radius,
                                     double time_span) {
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
    << time_span << R"(, "time_step": 0.01}]},
    "assemblies": [{"name": "m", "bodies": [
    {"name": "channel",
     "geometry": [{"type": "cad", "filename": "channel.step"}],
     "collision_geometry": [{"name": "channel", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0.4, 0.015, 0],
     "inertia": {"mass": 30, "Ixx": 0.01, "Iyy": 1.6, "Izz": 1.6, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}},
    {"name": "torus",
     "geometry": [{"type": "cad", "filename": "torus.step"}],
     "collision_geometry": [{"name": "torus", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": )" << kTorusMass << R"(, "Ixx": )" << kTorusDiametral
    << R"(, "Iyy": )" << kTorusDiametral << R"(, "Izz": )" << kTorusAxial
    << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
    {"name": "channel_joint", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["channel"], "origin"]},
    {"name": "torus_joint", "type": "float",
     "parent": [["fixed"], "origin"], "child": [["torus"], "origin"],
     "initial_displacement": {"T": [)"
    << kTorusX << ", 0, " << z << R"(], "R": [[0, 1, 0], [0, 0, 1], [1, 0, 0]]},
     "initial_velocity": [0, )"
    << speed << R"(, 0, 0, 0, )" << -speed / radius << R"(]}],
    "force_modules": [{"name": "contact", "type": "contact",
      "damping_depth": 1e-6,
      "materials": [{"name": "steel", "youngs_modulus": 2e9,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.3}]}]}]})";
  const std::string path = testing::TempDir() + "/cr_torus_rolls.json";
  std::ofstream file(path);
  file << m.str();
  return path;
}

TEST_F(CadRound, TorusRollsAlongTheChannel) {
  // the torus rolls at 1 m/s along the slot, on its two flanks, through the
  // S-bend of 30 mm between x = 200 and 600 mm and on into the straight end:
  // the walls of the slot steer it, and it stays upright and in the slot,
  // still rolling, with no energy gained
  shift::test::CadScene scene("cr_torus_start", "channel.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double z = FirstTouch(scene, 0.0);
  const auto& start = scene.At({{kTorusX, 0.0, z + 1e-6}}, kStanding);
  ASSERT_EQ(start.manifold.size(), 2u);
  // the rolling radius: from the center down to the points on the flanks
  const double radius = start.manifold[0].point_a.z - z;
  const double speed = 1.0, time_span = 0.6;
  const std::string path = RollingTorusModel(z, speed, radius, time_span);
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  const double energy = 0.5 * kTorusMass * speed * speed +
                        0.5 * kTorusAxial * std::pow(speed / radius, 2.0);
  context.Simulate();
  shift::test::RecordContacts(context);
  ksl_SE3_t d{};
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "torus") {
      d = context.get_body_origin_pose(body);
    }
  }
  // on along the straight end, 30 mm across from the start, within the slot's
  // clearance of 1.5 mm
  EXPECT_GT(d.t.x, 0.6);
  EXPECT_NEAR(d.t.y, 0.03, 0.0015);
  // standing: its axis (the third column of R) level, its center as high as
  // where it rests on the flanks
  EXPECT_LT(std::fabs(d.R.v2.z), std::sin(2.0 * M_PI / 180.0));
  EXPECT_NEAR(d.t.z, z, 1e-4);
  // rolling still, and no energy gained (it is as high as it started)
  EXPECT_GT(context.KineticEnergy(), 0.8 * energy);
  EXPECT_LT(context.KineticEnergy(), energy * (1.0 + 1e-3));
}

/* tube.step (scripts/make_channel_step.py): a tube of kTubeRadius about a path
that descends kTubeSlope along x (z down) on its straight start, x < 0.2 m, and
then bends across in an S of 60 mm, to y = 60 mm from x = 0.6 m on. The torus is
threaded on it at x = kTorusX: its axis, z, along the path's tangent, its x
square to the path and down, so that the top of its hole, u = 180 degrees, is
in the middle of an inner face */
static const double kTubeRadius = 0.02;
static const double kTubeSlope = 0.15;

/* the tube's axis at x on its straight start or its straight end (end: y = 60
mm), and its unit tangent there, along +x */
static void TubeAxis(double x, bool end, ksl_vec3_t& axis, ksl_vec3_t& tangent) {
  const double n = std::sqrt(1.0 + kTubeSlope * kTubeSlope);
  axis = {{x, end ? 0.06 : 0.0, kTubeSlope * x}};
  tangent = {{1.0 / n, 0.0, kTubeSlope / n}};
}

/* the threaded torus's rotation, columns its axes: x down square to the
tangent, y = z x x, across to -y, and z along the tangent; tipped by tip about
its y */
static void Threaded(double tip, double r[3][3]) {
  const double n = std::sqrt(1.0 + kTubeSlope * kTubeSlope);
  const double c = 1.0 / n, s = kTubeSlope / n;
  const double level[3][3] = {{-s, 0.0, c}, {0.0, -1.0, 0.0}, {c, 0.0, s}};
  const double ct = std::cos(tip), st = std::sin(tip);
  const double turn[3][3] = {{ct, 0.0, st}, {0.0, 1.0, 0.0}, {-st, 0.0, ct}};
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      r[j][k] = 0.0;
      for(int l = 0; l < 3; l++) {
        r[j][k] += level[j][l] * turn[l][k];
      }
    }
  }
}

/* the torus's center at drop down from the tube's axis at kTorusX, square to
the path */
static ksl_vec3_t Hanging(double drop) {
  ksl_vec3_t axis, tangent;
  TubeAxis(kTorusX, false, axis, tangent);
  double r[3][3];
  Threaded(0.0, r);
  return {{axis.x + drop * r[0][0], axis.y + drop * r[1][0],
           axis.z + drop * r[2][0]}};
}

/* the drop of the torus's center below the tube's axis at which the top of
its hole first touches the tube */
static double FirstHang(shift::test::CadScene& scene) {
  double r[3][3];
  Threaded(0.0, r);
  double apart = 0.0, deep = 0.015;
  for(int i = 0; i < 60; i++) {
    const double drop = 0.5 * (apart + deep);
    (scene.At(Hanging(drop), r).collision ? deep : apart) = drop;
  }
  return apart;
}

TEST_F(CadRound, TorusHangsOnATube) {
  // the tube through the hole of the torus: hanging from the top of its hole,
  // a saddle, on the top of the tube, the torus touches at one point, 10 mm
  // (30 mm of the hole less 20 mm of the tube) below the axis, with its normal
  // up, square to the path
  shift::test::CadScene scene("cr_torus_tube", "tube.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double drop = FirstHang(scene), sink = 20e-6;
  EXPECT_NEAR(drop, 0.01, 1e-9);
  double r[3][3];
  Threaded(0.0, r);
  const auto& p = scene.At(Hanging(drop + sink), r);
  ASSERT_TRUE(p.collision);
  ASSERT_EQ(p.manifold.size(), 1u);
  const auto& c = p.manifold[0];
  ksl_vec3_t axis, tangent;
  TubeAxis(kTorusX, false, axis, tangent);
  for(int k = 0; k < 3; k++) {
    // the normal, from the tube to the torus, up along -x of the torus
    EXPECT_NEAR(c.normal.at[k], -r[k][0], 1e-6);
    // the point on the tube, at its top
    EXPECT_NEAR(c.point_a.at[k], axis.at[k] - kTubeRadius * r[k][0], 1e-6);
  }
  EXPECT_NEAR(c.depth, sink, 1e-8);
}

TEST_F(CadRound, TorusAroundATubeTouchesEverywhere) {
  // the torus, keeping its orientation, moved off the tube's axis by 20 um more
  // than the clearance, in a direction turned about the axis through a full
  // turn in 400 poses: the point where the tube touches the inside of its hole
  // goes round both, across the four seams of the tube and the two of the
  // torus's inner faces, one point at one depth at each pose
  shift::test::CadScene scene("cr_torus_around", "tube.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double drop = FirstHang(scene), sink = 20e-6;
  double r[3][3];
  Threaded(0.0, r);
  ksl_vec3_t axis, tangent;
  TubeAxis(kTorusX, false, axis, tangent);
  int found = 0;
  for(int i = 0; i < 400; i++) {
    const double angle = 2.0 * M_PI * i / 400.0;
    double away[3];
    ksl_vec3_t center;
    for(int k = 0; k < 3; k++) {
      // the torus's x and y are square to the tangent
      away[k] = std::cos(angle) * r[k][0] + std::sin(angle) * r[k][1];
      center.at[k] = axis.at[k] + (drop + sink) * away[k];
    }
    const auto& p = scene.At(center, r);
    SCOPED_TRACE(i);
    EXPECT_EQ(p.manifold.size(), 1u);
    if(p.manifold.size() == 1u) {
      found++;
      const auto& c = p.manifold[0];
      EXPECT_NEAR(c.depth, sink, 1e-8);
      for(int k = 0; k < 3; k++) {
        // on the side of the tube away from the torus's center
        EXPECT_NEAR(c.normal.at[k], -away[k], 1e-6);
        EXPECT_NEAR(c.point_a.at[k], axis.at[k] - kTubeRadius * away[k], 1e-6);
      }
    }
  }
  EXPECT_EQ(found, 400);
}

TEST_F(CadRound, TippedTorusOnATubeTouchesTwice) {
  // centered on the tube's axis and tipped about a line across it, the torus
  // first touches the tube at two points, at the top of its hole behind the
  // center and at the bottom ahead of it, as a ring jammed on a rod: the two
  // are mirrored through the center, at one depth
  shift::test::CadScene scene("cr_torus_tipped", "tube.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const ksl_vec3_t center = Hanging(0.0);
  double r[3][3];
  double apart = 0.0, deep = 1.2;
  for(int i = 0; i < 60; i++) {
    const double tip = 0.5 * (apart + deep);
    Threaded(tip, r);
    (scene.At(center, r).collision ? deep : apart) = tip;
  }
  // the hole's edge, a circle of 30 mm in the torus's plane, would first meet
  // the tube at a tip of acos(20 / 30); the curved inner face meets it sooner
  EXPECT_GT(apart, 0.1);
  EXPECT_LT(apart, std::acos(2.0 / 3.0));
  Threaded(apart + 1e-4, r);
  const auto& p = scene.At(center, r);
  ASSERT_TRUE(p.collision);
  ASSERT_EQ(p.manifold.size(), 2u);
  const auto& a = p.manifold[0];
  const auto& b = p.manifold[1];
  EXPECT_GT(a.depth, 0.0);
  EXPECT_NEAR(a.depth, b.depth, 1e-9);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(a.point_a.at[k] + b.point_a.at[k], 2.0 * center.at[k], 1e-7);
    EXPECT_NEAR(a.normal.at[k], -b.normal.at[k], 1e-7);
  }
  // on the tube, at its radius from the axis
  ksl_vec3_t axis, tangent;
  TubeAxis(kTorusX, false, axis, tangent);
  for(const auto& c : p.manifold) {
    double off[3], along = 0.0, across = 0.0;
    for(int k = 0; k < 3; k++) {
      off[k] = c.point_a.at[k] - axis.at[k];
      along += off[k] * tangent.at[k];
    }
    for(int k = 0; k < 3; k++) {
      across += std::pow(off[k] - along * tangent.at[k], 2.0);
    }
    EXPECT_NEAR(std::sqrt(across), kTubeRadius, 1e-6);
  }
}

/* the torus threaded on the tube, hanging from it at kTorusX with its center
drop below the axis, sliding along it at speed */
static std::string ThreadedTorusModel(double drop, double speed,
                                      double time_span) {
  const ksl_vec3_t c = Hanging(drop);
  double r[3][3];
  Threaded(0.0, r);
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
    << time_span << R"(, "time_step": 0.01}]},
    "assemblies": [{"name": "m", "bodies": [
    {"name": "tube",
     "geometry": [{"type": "cad", "filename": "tube.step"}],
     "collision_geometry": [{"name": "tube", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0.4, 0.03, 0.06],
     "inertia": {"mass": 8, "Ixx": 0.002, "Iyy": 0.4, "Izz": 0.4, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}},
    {"name": "torus",
     "geometry": [{"type": "cad", "filename": "torus.step"}],
     "collision_geometry": [{"name": "torus", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": )" << kTorusMass << R"(, "Ixx": )" << kTorusDiametral
    << R"(, "Iyy": )" << kTorusDiametral << R"(, "Izz": )" << kTorusAxial
    << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
    {"name": "tube_joint", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["tube"], "origin"]},
    {"name": "torus_joint", "type": "float",
     "parent": [["fixed"], "origin"], "child": [["torus"], "origin"],
     "initial_displacement": {"T": [)"
    << c.x << ", " << c.y << ", " << c.z << R"(], "R": [[)" << r[0][0] << ", "
    << r[0][1] << ", " << r[0][2] << "], [" << r[1][0] << ", " << r[1][1]
    << ", " << r[1][2] << "], [" << r[2][0] << ", " << r[2][1] << ", "
    << r[2][2] << R"(]]},
     "initial_velocity": [0, 0, )"
    << speed << R"(, 0, 0, 0]}],
    "force_modules": [{"name": "contact", "type": "contact",
      "damping_depth": 1e-6,
      "materials": [{"name": "steel", "youngs_modulus": 2e9,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.05}]}]}]})";
  const std::string path = testing::TempDir() + "/cr_torus_threaded.json";
  std::ofstream file(path);
  file << m.str();
  return path;
}

TEST_F(CadRound, ThreadedTorusSlidesAlongTheTube) {
  // threaded on the tube and hanging from it, the torus slides down its slope
  // at 1 m/s and more, through the S-bend, where it swings out across the
  // tube and, turned by the bend, rocks on it, and on into the straight end:
  // the tube stays through its hole, and no energy is gained
  shift::test::CadScene scene("cr_torus_threaded_start", "tube.step", R"("type": "cad")", "",
              R"({"type": "cad", "filename": "torus.step"})");
  const double drop = FirstHang(scene);
  const double speed = 1.0, time_span = 0.5;
  const std::string path = ThreadedTorusModel(drop, speed, time_span);
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  const ksl_vec3_t start = Hanging(drop);
  // kinetic less potential energy, z down
  const double energy =
    0.5 * kTorusMass * speed * speed - kTorusMass * 9.81 * start.z;
  context.Simulate();
  shift::test::RecordContacts(context);
  ksl_SE3_t d{};
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "torus") {
      d = context.get_body_origin_pose(body);
    }
  }
  // on along the straight end
  EXPECT_GT(d.t.x, 0.65);
  ksl_vec3_t axis, tangent;
  TubeAxis(d.t.x, true, axis, tangent);
  double off[3], along = 0.0, across = 0.0, aligned = 0.0;
  for(int k = 0; k < 3; k++) {
    off[k] = d.t.at[k] - axis.at[k];
    along += off[k] * tangent.at[k];
    aligned += d.R.v2.at[k] * tangent.at[k];
  }
  for(int k = 0; k < 3; k++) {
    across += std::pow(off[k] - along * tangent.at[k], 2.0);
  }
  // hanging from the tube: its center no further from the axis than the
  // clearance of 10 mm, nearer when it is tipped
  EXPECT_LT(std::sqrt(across), drop + 1e-5);
  EXPECT_GT(std::sqrt(across), 0.9 * drop);
  // rocking a little, its axis near the tangent, far from the tip of 48
  // degrees, acos(20 / 30), at which the edge of its hole would jam
  EXPECT_GT(std::fabs(aligned), std::cos(15.0 * M_PI / 180.0));
  // faster than it started, and no energy gained
  const double kinetic = context.KineticEnergy();
  EXPECT_GT(kinetic, 0.5 * kTorusMass * speed * speed);
  EXPECT_LT(kinetic - kTorusMass * 9.81 * d.t.z, energy + 1e-3 * kinetic);
}

/* the largest change of the elastic force from one pose of a sweep to the next,
as a share of the greatest force of the sweep: poses from at(0) to at(count) */
template <typename Pose>
static double GreatestStep(shift::test::CadScene& scene, int count, Pose at,
                           double* where = nullptr) {
  std::vector<double> forces;
  double peak = 0.0;
  for(int i = 0; i <= count; i++) {
    ksl_vec3_t c;
    double r[3][3];
    at(i, c, r);
    forces.push_back(shift::test::ElasticForce(scene.At(c, r)));
    peak = std::max(peak, forces.back());
  }
  double greatest = 0.0;
  for(int i = 1; i <= count; i++) {
    const double step = std::fabs(forces[i] - forces[i - 1]) / peak;
    if(step > greatest) {
      greatest = step;
      if(where) {
        *where = static_cast<double>(i) / count;
      }
    }
  }
  return greatest;
}

TEST_F(CadRound, SweepsOverEdges) {
  const double depth = 1e-5;
  double where = 0.0;
  {
    // a sphere along the top of the part and off its edge at x = 10 mm
    const double radius = 0.005;
    shift::test::CadScene scene("cr_sweep_sphere", "box_mm.step", SphereKeys(radius));
    const double g = GreatestStep(scene, 2000, [&](int i, ksl_vec3_t& c, double r[3][3]) {
      Lean(0.0, r);
      c = {{0.0095 + 1e-3 * i / 2000.0, 0.01, kBox[2] + radius - depth}};
    }, &where);
    EXPECT_LT(g, 0.01) << where;
  }
  {
    // a box leaning on one corner, sliding off the part's edge
    const double h[3] = {0.002, 0.003, 0.004}, axis[3] = {0.6, 0.8, 0.0};
    shift::test::CadScene scene("cr_sweep_corner", "box_mm.step", BoxKeys(h[0], h[1], h[2]));
    double r0[3][3];
    shift::test::Rotation(axis, 0.5, r0);
    double reach = 0.0, corner[3] = {0, 0, 0};
    for(int j = 0; j < 3; j++) {
      reach += h[j] * std::fabs(r0[2][j]);
      for(int k = 0; k < 3; k++) {
        corner[k] -= h[j] * std::copysign(1.0, r0[2][j]) * r0[k][j];
      }
    }
    const double g = GreatestStep(scene, 2000, [&](int i, ksl_vec3_t& c, double r[3][3]) {
      for(int k = 0; k < 3; k++) for(int j = 0; j < 3; j++) r[k][j] = r0[k][j];
      c = {{0.0098 + 4e-4 * i / 2000.0 - corner[0], 0.01 - corner[1],
            kBox[2] + reach - depth}};
    }, &where);
    // the corner gives way to the crossing of its edges with the edge of the
    // face, over a few microns
    EXPECT_LT(g, 0.02) << where;
  }
  {
    // a cylinder standing on its end, tipping
    const double radius = 0.002, half = 0.004;
    shift::test::CadScene scene("cr_sweep_rim", "box_mm.step", RodKeys("cylinder", radius, half));
    const double g = GreatestStep(scene, 2000, [&](int i, ksl_vec3_t& c, double r[3][3]) {
      const double tilt = 0.02 * i / 2000.0;
      const double axis[3] = {0.6, 0.8, 0.0};
      shift::test::Rotation(axis, tilt, r);
      c = {{0.005, 0.01, kBox[2] + half * std::cos(tilt) + radius * std::sin(tilt) - depth}};
    }, &where);
    EXPECT_LT(g, 0.01) << where;
  }
  {
    // a box on its face, tipping
    const double h[3] = {0.002, 0.003, 0.004};
    shift::test::CadScene scene("cr_sweep_box", "box_mm.step", BoxKeys(h[0], h[1], h[2]));
    const double g = GreatestStep(scene, 2000, [&](int i, ksl_vec3_t& c, double r[3][3]) {
      const double tilt = 0.02 * i / 2000.0;
      Lean(tilt, r);
      c = {{0.005, 0.01, kBox[2] + h[2] * std::cos(tilt) + h[0] * std::sin(tilt) - depth}};
    }, &where);
    EXPECT_LT(g, 0.01) << where;
  }
  {
    // a cylinder on its side, over the part's edge, along its axis
    const double radius = 0.002, half = 0.004;
    shift::test::CadScene scene("cr_sweep_side", "box_mm.step", RodKeys("cylinder", radius, half));
    const double g = GreatestStep(scene, 2000, [&](int i, ksl_vec3_t& c, double r[3][3]) {
      const double axis[3] = {1.0, 0.0, 0.0};
      shift::test::Rotation(axis, 0.5 * M_PI, r); // along y
      c = {{0.0095 + 1e-3 * i / 2000.0, 0.01, kBox[2] + radius - depth}};
    }, &where);
    EXPECT_LT(g, 0.01) << where;
  }
}

TEST_F(CadRound, CornerOffAnEdgeHasNoStep) {
  // boxes leaning any way, each on one corner, sliding off the part's edge: the
  // corner gives way to the crossing of its edges with the part's edge. A jump
  // stays as the sweep gets finer, and the steps of a smooth force shrink with it
  const double depth = 1e-5, h[3] = {0.002, 0.003, 0.004};
  shift::test::CadScene scene("cr_corner_tilts", "box_mm.step", BoxKeys(h[0], h[1], h[2]));
  std::mt19937 random(5);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for(int trial = 0; trial < 40; trial++) {
    SCOPED_TRACE(trial);
    double axis[3] = {unit(random), unit(random), unit(random)};
    const double length = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] +
                                    axis[2] * axis[2]);
    for(double& a : axis) {
      a /= length;
    }
    double r0[3][3];
    shift::test::Rotation(axis, 0.3 + 0.5 * (unit(random) + 1.0), r0);
    double reach = 0.0, corner[3] = {0, 0, 0};
    for(int j = 0; j < 3; j++) {
      reach += h[j] * std::fabs(r0[2][j]);
      for(int k = 0; k < 3; k++) {
        corner[k] -= h[j] * std::copysign(1.0, r0[2][j]) * r0[k][j];
      }
    }
    double greatest_force = 0.0;
    auto sweep = [&](int count) {
      return GreatestStep(scene, count, [&](int i, ksl_vec3_t& c, double r[3][3]) {
        for(int k = 0; k < 3; k++) {
          for(int j = 0; j < 3; j++) {
            r[k][j] = r0[k][j];
          }
        }
        c = {{0.0098 + 4e-4 * i / count - corner[0], 0.01 - corner[1],
              kBox[2] + reach - depth}};
        greatest_force = std::max(greatest_force, shift::test::ElasticForce(scene.At(c, r)));
      });
    };
    const double coarse = sweep(2000), fine = sweep(16000);
    // an edge lying nearly flat hands over within 50 nm, near the finer sweep's
    // 25 nm between poses
    EXPECT_LT(fine, 0.07);
    EXPECT_LT(fine, 0.6 * coarse + 1e-3);
    EXPECT_LT(greatest_force, 5e-10); // the corner's, measured through the top
  }
}

TEST_F(CadRound, ContactWithCadNeedsTheModulus) {
  // a box on CAD geometry follows Hertz: a contact_stiffness alone gives no force
  const std::string path = testing::TempDir() + "/cr_no_modulus.json";
  std::ostringstream m;
  m << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "include_directories": [")"
    << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true, "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
    "assemblies": [{"name": "m", "bodies": [
    {"name": "part", "geometry": [{"type": "cad", "filename": "box_mm.step"}],
     "collision_geometry": [{"name": "part", "type": "cad", "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                 "Izx": 0}},
    {"name": "body", "geometry": [],
     "collision_geometry": [{"name": "shape", "material": "steel", )"
    << BoxKeys(0.002, 0.003, 0.004) << R"(}], "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                 "Izx": 0}}],
    "joints": [
    {"name": "part_joint", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["part"], "origin"]},
    {"name": "body_joint", "type": "float",
     "parent": [["fixed"], "origin"], "child": [["body"], "origin"]}],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [{"name": "steel", "contact_stiffness": 1e6,
                     "restitution": 0.1, "friction": 0.3}]}]}]})";
  {
    std::ofstream file(path);
    file << m.str();
  }
  shift::core::Context context;
  EXPECT_THROW(context.LoadJSON(path), shift::core::Exception);
  std::remove(path.c_str());
}

TEST_F(CadRound, CapsuleLyingOnAFaceIsALine) {
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cap_line", "box_mm.step", RodKeys("capsule", radius, half));
  const double axis[3] = {1.0, 0.0, 0.0};
  double r[3][3];
  shift::test::Rotation(axis, 0.5 * M_PI, r);
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{0.005, 0.01, kBox[2] + radius - depth}}, r);
  ASSERT_GE(p.manifold.size(), 3u);
  double length = 0.0;
  for(const shift::core::collision_geometry::ContactPoint& m : p.manifold) {
    EXPECT_GT(m.length, 0.0); // the ends add nothing to the line
    EXPECT_NEAR(m.depth, depth, 1e-10);
    length += m.weight * m.length;
  }
  EXPECT_NEAR(length, 2.0 * half, 1e-9);
}

TEST_F(CadRound, CapsuleStandingOnItsTip) {
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cap_tip", "box_mm.step", RodKeys("capsule", radius, half));
  for(bool part_first : {true, false}) {
    const shift::core::collision_geometry::ContactPair& p = scene.At(
      {{0.005, 0.01, kBox[2] + half + radius - depth}}, nullptr, part_first);
    ASSERT_EQ(p.manifold.size(), 1u);
    EXPECT_NEAR(p.manifold[0].depth, depth, 1e-12);
    EXPECT_NEAR(p.manifold[0].curvature, 1.0 / radius, 1e-6);
    EXPECT_NEAR(p.manifold[0].normal.z, part_first ? 1.0 : -1.0, 1e-12);
  }
}

TEST_F(CadRound, TiltedCapsuleTouchesAtItsHemisphere) {
  // the lowest point of the hemisphere, not of the rim: the center of the end
  // is radius above the face, less the depth
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cap_tilt", "box_mm.step", RodKeys("capsule", radius, half));
  for(double lean : {0.3, 0.8, 1.2}) {
    double r[3][3];
    Lean(lean, r);
    const double above = half * std::cos(lean) + radius - depth;
    const shift::core::collision_geometry::ContactPair& p = scene.At({{0.005, 0.01, kBox[2] + above}}, r);
    SCOPED_TRACE(lean);
    ASSERT_EQ(p.manifold.size(), 1u);
    const shift::core::collision_geometry::ContactPoint& m = p.manifold[0];
    EXPECT_NEAR(m.depth, depth, 1e-11);
    EXPECT_NEAR(m.curvature, 1.0 / radius, 1e-6);
    EXPECT_DOUBLE_EQ(m.weight, 1.0);
    // straight below the center of the end
    EXPECT_NEAR(m.point_a.x, 0.005 - half * std::sin(lean), 1e-11);
  }
}

TEST_F(CadRound, CapsuleTippingUpHasNoJump) {
  // from lying flat to resting on one end, its lower end at the same depth:
  // the force of the line goes over to the point of the hemisphere
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cap_sweep", "box_mm.step", RodKeys("capsule", radius, half));
  double previous = -1.0, greatest = 0.0;
  for(int i = 0; i <= 2000; i++) {
    const double lean = 0.5 * M_PI - 0.1 * (i / 2000.0) * (i / 2000.0);
    double r[3][3];
    Lean(lean, r);
    const double above = half * std::cos(lean) + radius - depth;
    const shift::core::collision_geometry::ContactPair& p = scene.At({{0.005, 0.01, kBox[2] + above}}, r);
    ASSERT_TRUE(p.collision) << lean;
    const double force = shift::test::ElasticForce(p);
    if(previous >= 0.0) {
      greatest = std::max(greatest, std::fabs(force - previous) /
                                      std::max(force, previous));
    }
    previous = force;
  }
  EXPECT_LT(greatest, 0.05);
}

TEST_F(CadRound, CapsuleAcrossAnEdge) {
  // along x, over the part's edge at x = 10 mm: the edge presses on its side
  const double radius = 0.002, half = 0.004, depth = 1e-5;
  shift::test::CadScene scene("cr_cap_edge", "box_mm.step", RodKeys("capsule", radius, half));
  double r[3][3];
  Lean(0.5 * M_PI, r);
  const shift::core::collision_geometry::ContactPair& p =
    scene.At({{kBox[0] - 1e-3, 0.01, kBox[2] + radius - depth}}, r);
  ASSERT_GE(p.manifold.size(), 1u);
  EXPECT_NEAR(p.depth, depth, 1e-9);
  EXPECT_NEAR(p.normal.z, 1.0, 1e-6);
}

TEST_F(CadRound, CapsuleEndOnACorner) {
  const double radius = 0.003, half = 0.004, b = 0.0015;
  shift::test::CadScene scene("cr_cap_corner", "box_mm.step", RodKeys("capsule", radius, half));
  const double t = 1.0 / std::sqrt(3.0);
  // its axis out of the corner, along (1, 1, 1)
  const double x[3] = {1.0 / std::sqrt(2.0), -1.0 / std::sqrt(2.0), 0.0};
  const double z[3] = {t, t, t};
  const double y[3] = {z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
                       z[0] * x[1] - z[1] * x[0]};
  double r[3][3];
  for(int k = 0; k < 3; k++) {
    r[k][0] = x[k];
    r[k][1] = y[k];
    r[k][2] = z[k];
  }
  const double reach = b * std::sqrt(3.0) + half; // from the corner, on the axis
  const shift::core::collision_geometry::ContactPair& p = scene.At({{kBox[0] + reach * t, kBox[1] + reach * t,
                                    kBox[2] + reach * t}}, r);
  ASSERT_EQ(p.manifold.size(), 1u);
  EXPECT_NEAR(p.manifold[0].depth, radius - b * std::sqrt(3.0), 1e-11);
}

/* a body of shape settling onto the top of the part (z up here is -z: the face
z = 0 of box_mm.step, as the ball above); where it comes to rest. The damping
of the contact law is fully applied from 1 nm of penetration: a cylinder on its
end rests on a line of contact around its rim, a few nm deep, where the default
damping_depth, 0.1 mm, applies almost none */
static void Settle(const std::string& shape, const std::string& rotation,
                   double height, double& z, double& rate) {
  std::ostringstream m;
  m.precision(17);
  m << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "include_directories": [")"
    << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true, "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-7,
                     "absolute_error": 1e-10, "time_span": 0.5,
                     "time_step": 0.01}]},
    "frames": [{"name": "start", )" << rotation << R"("T": [0, 0, 0]}],
    "assemblies": [{"name": "m", "bodies": [
    {"name": "part", "geometry": [{"type": "cad", "filename": "box_mm.step"}],
     "collision_geometry": [{"name": "part", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0, "Iyz": 0,
                 "Izx": 0}},
    {"name": "body", "geometry": [],
     "collision_geometry": [{"name": "shape", "material": "steel", )"
    << shape << R"(}], "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 0.1, "Ixx": 1e-5, "Iyy": 1e-5, "Izz": 1e-5, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}}],
    "joints": [
    {"name": "part_joint", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["part"], "origin"]},
    {"name": "body_joint", "type": "prismatic", "axis": [0, 0, 1],
     "parent": [["fixed"], "start"], "child": [["body"], "origin"],
     "initial_displacement": )" << -height - 1e-6 << R"(}],
    "force_modules": [{"name": "contact", "type": "contact",
      "damping_depth": 1e-9,
      "materials": [{"name": "steel", "youngs_modulus": 2e9,
                     "poisson_ratio": 0.3, "restitution": 0.1,
                     "friction": 0.3}]}]}]})";
  const std::string path = testing::TempDir() + "/cr_settle_shape.json";
  {
    std::ofstream file(path);
    file << m.str();
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.set_state_observer([&](double, const std::vector<double>& y) {
    z = y[0];
    rate = y[1];
  });
  context.Simulate();
  shift::test::RecordContacts(context);
}

TEST_F(CadRound, ShapesSettleOnAFace) {
  // each comes to rest a little into the face, at a depth the contact carries
  const struct {
    std::string keys;
    double height;
  } shapes[] = {{BoxKeys(0.002, 0.003, 0.004), 0.004},
                {RodKeys("cylinder", 0.002, 0.004), 0.004},
                {RodKeys("capsule", 0.002, 0.004), 0.006}};
  for(const auto& shape : shapes) {
    SCOPED_TRACE(shape.keys);
    double z = 0.0, rate = 1.0;
    Settle(shape.keys, "", shape.height, z, rate);
    EXPECT_LT(std::fabs(rate), 1e-3);
    EXPECT_GT(z, -shape.height);
    EXPECT_LT(z, -shape.height + 1e-4);
  }
}
