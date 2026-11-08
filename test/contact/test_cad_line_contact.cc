/* line contact on curved faces of CAD geometry: an elliptical cam, its side
of rational B-spline or extruded faces, against a roller, a flat follower and
the ground (scripts/make_cam_step.py) */
#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "test/contact/contact_record.h"
#include "shift/config.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"


/* the cam of make_cam_step.py */
static const double kMajor = 0.04;
static const double kMinor = 0.025;
static const double kWidth = 0.02;
static const double kRollerRadius = 0.01;

/* a cam at the origin of the model and a follower, each a free body whose
geometry is placed for each test */
class CamScene {
public:
  CamScene(const std::string& cam, const std::string& follower,
           bool ground = false) {
    const std::string path = testing::TempDir() + "/cam_" + cam + follower +
                             (ground ? "_ground" : "") + ".json";
    {
      std::ofstream file(path);
      file << Model(cam, follower, ground);
    }
    context_.LoadJSON(path);
    std::remove(path.c_str());
    context_.UpdateD();
    for(auto g : context_.get_active_collision_geometry()) {
      if(g->get_name() == "cam") {
        cam_ = g;
      } else {
        follower_ = g;
      }
    }
  }

  /* the contact, with the cam turned by angle about its axis and the
  follower at d */
  shift::core::collision_geometry::ContactPair& Test(const ksl_SE3_t& follower, double angle = 0.0) {
    ksl_SE3_t d = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
    const double c = std::cos(angle), s = std::sin(angle);
    d.R.at[0][0] = c;
    d.R.at[0][1] = s;
    d.R.at[1][0] = -s;
    d.R.at[1][1] = c;
    cam_->UpdatePose(d);
    follower_->UpdatePose(follower);
    pair_ = shift::core::collision_geometry::ContactPair();
    pair_.a = cam_;
    pair_.b = follower_;
    shift::test::TestContact(pair_);
    return pair_;
  }

private:
  static std::string Model(const std::string& cam,
                           const std::string& follower, bool ground) {
    const char* inertia = R"("center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                   "Ixy": 0, "Iyz": 0, "Izx": 0})";
    std::ostringstream m;
    m << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")"
      << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]
  },
  "assemblies": [{"name": "cams",
    "bodies": [
      {"name": "cam",
       "geometry": [{"type": "cad", "filename": ")"
      << cam << R"(.step"}],
       "collision_geometry": [{"name": "cam", "type": "cad",
                               "material": "steel"}],
       )" << inertia
      << R"(},
      {"name": "follower",)";
    if(ground) {
      m << R"(
       "geometry": [],
       "collision_geometry": [{"name": "follower", "type": "plane",
                               "normal": [-1, 0, 0], "material": "steel"}],)";
    } else {
      m << R"(
       "geometry": [{"type": "cad", "filename": ")"
        << follower << R"(.step"}],
       "collision_geometry": [{"name": "follower", "type": "cad",
                               "material": "steel"}],)";
    }
    m << "\n       " << inertia << R"(}
    ],
    "joints": [
      {"name": "cam_joint", "type": "float", "parent": [["fixed"], "origin"],
       "child": [["cam"], "origin"]},
      {"name": "follower_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["follower"], "origin"],
       "initial_displacement": {"T": [1, 0, 0]}}
    ],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [{"name": "steel", "youngs_modulus": 2e11,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.3}]}]
  }]
})";
    return m.str();
  }

  shift::core::Context context_;
  shift::core::collision_geometry::Base* cam_ = nullptr;
  shift::core::collision_geometry::Base* follower_ = nullptr;
  shift::core::collision_geometry::ContactPair pair_;
};

static ksl_SE3_t At(double x, double y, double z = 0.0) {
  ksl_SE3_t d = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  d.t.x = x;
  d.t.y = y;
  d.t.z = z;
  return d;
}

/* the elastic force of a pair's points, sum of w K d^e by the laws of the
contact force module, for a modulus E* = 1 */
static double ElasticForce(const shift::core::collision_geometry::ContactPair& pair) {
  double force = 0.0;
  for(const auto& c : pair.manifold) {
    double stiffness, exponent;
    shift::core::force_module::Contact::PointLaw(c, 1.0, 1e-4, stiffness,
                                                 exponent);
    force += c.weight * stiffness * std::pow(c.depth, exponent);
  }
  return force;
}

/* the point of the ellipse at the parameter t, (a cos t, b sin t), and its
outward unit normal */
static void Ellipse(double t, double p[2], double n[2]) {
  p[0] = kMajor * std::cos(t);
  p[1] = kMinor * std::sin(t);
  n[0] = kMinor * std::cos(t);
  n[1] = kMajor * std::sin(t);
  const double length = std::hypot(n[0], n[1]);
  n[0] /= length;
  n[1] /= length;
}

class CamFaces : public testing::TestWithParam<const char*> {};

INSTANTIATE_TEST_SUITE_P(CadLineContact, CamFaces,
                         testing::Values("cam_nurbs", "cam_extrusion"));

/* a line of contact the width of the cam, at depth, along the cam's axis */
static void ExpectFullLine(const shift::core::collision_geometry::ContactPair& pair, double depth) {
  ASSERT_TRUE(pair.collision);
  ASSERT_EQ(pair.manifold.size(), 3u);
  double length = 0.0;
  for(const auto& c : pair.manifold) {
    EXPECT_GT(c.length, 0.0);
    EXPECT_EQ(c.weight, 1.0);
    EXPECT_NEAR(c.depth, depth, 1e-10);
    length += c.length;
  }
  EXPECT_NEAR(length, kWidth, 1e-9);
}

TEST_P(CamFaces, RollerIsALineAroundTheCam) {
  // at each point of the ellipse, seams and flanks alike, a roller depth
  // into it along the normal touches along the whole width
  CamScene scene(GetParam(), "roller");
  const double depth = 20e-6;
  for(double t = 0.0; t < 2.0 * M_PI; t += M_PI / 24.0) {
    SCOPED_TRACE(t);
    double p[2], n[2];
    Ellipse(t, p, n);
    const double center = kRollerRadius - depth;
    const auto& pair =
      scene.Test(At(p[0] + center * n[0], p[1] + center * n[1]));
    ExpectFullLine(pair, depth);
    for(const auto& c : pair.manifold) {
      EXPECT_NEAR(c.normal.x, n[0], 1e-6);
      EXPECT_NEAR(c.normal.y, n[1], 1e-6);
      EXPECT_NEAR(c.point_a.x, p[0], 1e-6);
      EXPECT_NEAR(c.point_a.y, p[1], 1e-6);
    }
  }
}

TEST_P(CamFaces, TurningCamHasNoJump) {
  // a roller 20 um into the cam at each step of a turn, as the contact
  // passes the seams of its faces: the force is that of the depth along the
  // whole line, each step
  CamScene scene(GetParam(), "roller");
  const double depth = 20e-6;
  double expected = 0.0;
  for(double angle = 0.0; angle < 2.0 * M_PI; angle += 2.0 * M_PI / 360.0) {
    SCOPED_TRACE(angle);
    // the cam's point at the parameter t turned by angle
    const double t = 1.3 * angle;
    double p[2], n[2];
    Ellipse(t, p, n);
    const double c = std::cos(angle), s = std::sin(angle);
    const double center = kRollerRadius - depth;
    const double x = p[0] + center * n[0], y = p[1] + center * n[1];
    const auto& pair = scene.Test(At(c * x - s * y, s * x + c * y), angle);
    ASSERT_TRUE(pair.collision);
    const double force = ElasticForce(pair);
    if(expected > 0.0) {
      EXPECT_NEAR(force / expected, 1.0, 1e-6);
    }
    expected = force;
  }
}

TEST_P(CamFaces, ApartThereIsNoContact) {
  CamScene scene(GetParam(), "roller");
  EXPECT_FALSE(scene.Test(At(kMajor + kRollerRadius + 1e-6, 0.0)).collision);
}

TEST_P(CamFaces, FlatFollowerIsALine) {
  // the flat face of the follower on the cam's flank: a plane tangent at the
  // ellipse's point. The follower's face has its normal -x, turned to oppose
  // the ellipse's normal there
  CamScene scene(GetParam(), "flat_follower");
  const double depth = 20e-6;
  for(double t : {0.0, 0.3, 0.9, M_PI / 2.0, 2.0, M_PI}) {
    SCOPED_TRACE(t);
    double p[2], n[2];
    Ellipse(t, p, n);
    ksl_SE3_t d = At(p[0] - depth * n[0], p[1] - depth * n[1]);
    d.R.at[0][0] = n[0];
    d.R.at[0][1] = n[1];
    d.R.at[1][0] = -n[1];
    d.R.at[1][1] = n[0];
    ExpectFullLine(scene.Test(d), depth);
  }
}

TEST_P(CamFaces, GroundIsALine) {
  // the primitive plane x = kMajor - depth, its solid beyond it
  CamScene scene(GetParam(), "", true);
  ExpectFullLine(scene.Test(At(kMajor - 20e-6, 0.0)), 20e-6);
}

/* the roller at the tip of the cam, tilted by theta in the plane of the
line, the deepest point of its rim depth into the cam */
static ksl_SE3_t Tilted(double theta, double depth) {
  ksl_SE3_t d = At(kMajor + kRollerRadius * std::cos(theta) - depth, 0.0);
  const double c = std::cos(theta), s = std::sin(theta);
  // the axis (s, 0, c), x turned to (c, 0, -s)
  d.R.at[0][0] = c;
  d.R.at[0][2] = -s;
  d.R.at[2][0] = s;
  d.R.at[2][2] = c;
  return d;
}

TEST_P(CamFaces, TiltedRollerLoadsItsEnd) {
  // tilted by 1 mrad, its end 10 um deep: the line is 10 mm long, its depth
  // falling to zero along it
  CamScene scene(GetParam(), "roller");
  const double theta = 1e-3, depth = 10e-6;
  const auto& pair = scene.Test(Tilted(theta, depth));
  ASSERT_TRUE(pair.collision);
  double length = 0.0, load = 0.0;
  for(const auto& c : pair.manifold) {
    ASSERT_GT(c.length, 0.0);
    length += c.length;
    load += c.length * c.depth;
    EXPECT_LT(c.point_a.z, 0.0105);
  }
  const double expected = depth / std::tan(theta);
  EXPECT_NEAR(length, expected, 1e-5);
  EXPECT_NEAR(load, 0.5 * depth * expected, 1e-9);
}

TEST_P(CamFaces, TiltingRollerHasNoJump) {
  // from level, through end loading, to the rim of the roller, and on past
  // 30 and 60 degrees, where its end face and side turn from the cam
  CamScene scene(GetParam(), "roller");
  const double depth = 0.4e-6;
  double previous = 0.0, previous_theta = 0.0;
  bool line = false, rim = false;
  for(double theta = 1e-5; theta < 1.2; theta *= 1.01) {
    SCOPED_TRACE(theta);
    const auto& pair = scene.Test(Tilted(theta, depth));
    const double force = ElasticForce(pair);
    ASSERT_GT(force, 0.0);
    line = line || (pair.manifold.size() > 1 &&
                    pair.manifold.front().length > 0.0);
    rim = rim || (pair.manifold.size() == 1 &&
                  pair.manifold.front().length == 0.0);
    if(previous > 0.0) {
      EXPECT_NEAR(force / previous, 1.0, 0.05)
        << "from theta " << previous_theta;
    }
    previous = force;
    previous_theta = theta;
  }
  EXPECT_TRUE(line);
  EXPECT_TRUE(rim);
}

TEST_P(CamFaces, SteepRollerHasNoLineOnTheEndFace) {
  // tilted past the angle at which a cylinder counts as standing on a plane,
  // and over the cam's end face: the contact is on the cam's side, all of it
  CamScene scene(GetParam(), "roller");
  for(double theta : {0.06, 0.08, 0.1, 0.2, 0.4}) {
    SCOPED_TRACE(theta);
    const auto& pair = scene.Test(Tilted(theta, 10e-6));
    ASSERT_TRUE(pair.collision);
    for(const auto& c : pair.manifold) {
      EXPECT_LT(c.depth, 12e-6);
      EXPECT_GT(c.normal.x, 0.9);
    }
  }
}

TEST_P(CamFaces, CrossedRollerIsAPoint) {
  // a roller across the cam, its axis along y at the tip: the gap is curved
  // in both directions, and the contact a point
  CamScene scene(GetParam(), "roller");
  ksl_SE3_t d = At(kMajor + kRollerRadius - 20e-6, 0.5 * kWidth, 0.5 * kWidth);
  d.R.at[1][1] = 0.0;
  d.R.at[1][2] = 1.0;
  d.R.at[2][1] = -1.0;
  d.R.at[2][2] = 0.0;
  const auto& pair = scene.Test(d);
  ASSERT_TRUE(pair.collision);
  ASSERT_EQ(pair.manifold.size(), 1u);
  EXPECT_EQ(pair.manifold[0].length, 0.0);
  EXPECT_NEAR(pair.manifold[0].depth, 20e-6, 1e-9);
}

/* the pose of a second cam, not turned, touching the cam turned by angle at
the parameter t of its ellipse, the two overlapping by depth along their common
normal, and that normal, out of the first cam */
static ksl_SE3_t Flanks(double t, double angle, double depth, double normal[2]) {
  double p[2], n[2];
  Ellipse(t, p, n);
  const double c = std::cos(angle), s = std::sin(angle);
  const double px = c * p[0] - s * p[1], py = s * p[0] + c * p[1];
  normal[0] = c * n[0] - s * n[1];
  normal[1] = s * n[0] + c * n[1];
  // the point of the second ellipse whose normal is the opposite one
  double q[2], m[2];
  Ellipse(std::atan2(-normal[1] / kMajor, -normal[0] / kMinor), q, m);
  return At(px - depth * normal[0] - q[0], py - depth * normal[1] - q[1]);
}

/* both parts curved, of the kinds of surface given */
class CurvedOnCurved
  : public testing::TestWithParam<std::pair<const char*, const char*>> {};

INSTANTIATE_TEST_SUITE_P(Faces, CurvedOnCurved,
                         testing::Values(std::make_pair("cam_extrusion",
                                                        "cam_extrusion"),
                                         std::make_pair("cam_nurbs",
                                                        "cam_nurbs"),
                                         std::make_pair("cam_nurbs",
                                                        "cam_extrusion")));

TEST_P(CurvedOnCurved, TipsAreALine) {
  // two cams tip to tip, a line across their width
  CamScene scene(GetParam().first, GetParam().second);
  for(double depth : {1e-4, 1e-5}) {
    SCOPED_TRACE(depth);
    const shift::core::collision_geometry::ContactPair& pair = scene.Test(At(2.0 * kMajor - depth, 0.0));
    ASSERT_TRUE(pair.collision);
    ExpectFullLine(pair, depth);
    for(const auto& c : pair.manifold) {
      EXPECT_NEAR(std::fabs(c.normal.x), 1.0, 1e-6);
    }
  }
}

TEST_P(CurvedOnCurved, FlanksAreALineAtTheirDepth) {
  // anywhere around the cam, seams apart: a line at the depth along the normal
  // the two have in common
  CamScene scene(GetParam().first, GetParam().second);
  const double angle = 0.7, depth = 1e-5;
  for(int i = 0; i < 40; i++) {
    const double t = 2.0 * M_PI * (i + 0.37) / 40.0;
    SCOPED_TRACE(t);
    double normal[2];
    const ksl_SE3_t pose = Flanks(t, angle, depth, normal);
    const shift::core::collision_geometry::ContactPair& pair = scene.Test(pose, angle);
      ExpectFullLine(pair, depth);
    for(const auto& c : pair.manifold) {
      EXPECT_NEAR(std::fabs(c.normal.x * normal[0] + c.normal.y * normal[1]),
                  1.0, 1e-6);
    }
  }
}

TEST_P(CurvedOnCurved, FlanksHaveNoJump) {
  // the force changes smoothly as the line runs over the seams of both
  CamScene scene(GetParam().first, GetParam().second);
  const double angle = 0.7, depth = 1e-5;
  double previous = 0.0, greatest = 0.0, peak = 0.0;
  for(int i = 0; i <= 4000; i++) {
    const double t = 2.0 * M_PI * i / 4000.0;
    double normal[2];
    const ksl_SE3_t pose = Flanks(t, angle, depth, normal);
    const double force = ElasticForce(scene.Test(pose, angle));
    if(i > 0) {
      greatest = std::max(greatest, std::fabs(force - previous));
    }
    previous = force;
    peak = std::max(peak, force);
  }
  ASSERT_GT(peak, 0.0);
  EXPECT_LT(greatest / peak, 0.02);
}

/* Flanks, with the second cam then tipped by tilt about the tangent of the
faces in the plane of the ellipses, through the middle of the line */
static ksl_SE3_t TiltedFlanks(double t, double angle, double depth, double tilt) {
  double normal[2];
  ksl_SE3_t d = Flanks(t, angle, depth, normal);
  double p[2], n[2];
  Ellipse(t, p, n);
  const double c = std::cos(angle), s = std::sin(angle);
  const double m[3] = {c * p[0] - s * p[1] - depth * normal[0],
                       s * p[0] + c * p[1] - depth * normal[1], 0.5 * kWidth};
  const double w[3] = {-normal[1], normal[0], 0.0}; // the axis
  const double ct = std::cos(tilt), st = std::sin(tilt);
  const double r[3][3] = {
    {ct + w[0] * w[0] * (1 - ct), w[0] * w[1] * (1 - ct), st * w[1]},
    {w[0] * w[1] * (1 - ct), ct + w[1] * w[1] * (1 - ct), -st * w[0]},
    {-st * w[1], st * w[0], ct}};
  const double from[3] = {m[0] - d.t.x, m[1] - d.t.y, m[2]};
  double to[3];
  for(int i = 0; i < 3; i++) {
    to[i] = m[i];
    for(int k = 0; k < 3; k++) {
      to[i] -= r[i][k] * from[k];
      d.R.at[k][i] = r[i][k];
    }
  }
  d.t.x = to[0];
  d.t.y = to[1];
  d.t.z = to[2];
  return d;
}

TEST_P(CurvedOnCurved, TiltedCamLoadsItsEnd) {
  // the line shortens to one end, and the force has no jump on the way
  CamScene scene(GetParam().first, GetParam().second);
  const double angle = 0.7, depth = 1e-5, t = 0.4;
  double previous = 0.0, greatest = 0.0, peak = 0.0;
  for(int i = 0; i <= 2000; i++) {
    const double tilt = 5e-6 * i;
    const shift::core::collision_geometry::ContactPair& pair = scene.Test(TiltedFlanks(t, angle, depth, tilt), angle);
    ASSERT_TRUE(pair.collision) << tilt;
    double deepest = 0.0;
    for(const auto& p : pair.manifold) {
      deepest = std::max(deepest, p.depth);
    }
    // the end sinks by half the width times the tilt
    EXPECT_NEAR(deepest, depth + 0.5 * kWidth * tilt,
                0.3 * 0.5 * kWidth * tilt + 1e-9)
      << tilt;
    const double force = ElasticForce(pair);
    if(i > 0) {
      greatest = std::max(greatest, std::fabs(force - previous));
    }
    previous = force;
    peak = std::max(peak, force);
  }
  ASSERT_GT(peak, 0.0);
  EXPECT_LT(greatest / peak, 0.02);
}

TEST_P(CurvedOnCurved, TiltedFlanksHaveNoJump) {
  // over the seams of both, the line with both ends touching and loading one
  CamScene scene(GetParam().first, GetParam().second);
  const double angle = 0.7, depth = 1e-5;
  for(double tilt : {5e-4, 5e-3}) {
    SCOPED_TRACE(tilt);
    double previous = 0.0, greatest = 0.0, peak = 0.0;
    for(int i = 0; i <= 4000; i++) {
      const double t = 2.0 * M_PI * i / 4000.0;
      const shift::core::collision_geometry::ContactPair& pair =
        scene.Test(TiltedFlanks(t, angle, depth, tilt), angle);
      const double force = ElasticForce(pair);
      if(i > 0) {
        greatest = std::max(greatest, std::fabs(force - previous));
      }
      previous = force;
      peak = std::max(peak, force);
    }
    ASSERT_GT(peak, 0.0);
    EXPECT_LT(greatest / peak, 0.02);
  }
}

TEST_P(CurvedOnCurved, ApartThereIsNoContact) {
  CamScene scene(GetParam().first, GetParam().second);
  EXPECT_FALSE(scene.Test(At(2.0 * kMajor + 1e-4, 0.0)).collision);
}

/* the pose of a second cam square to the first, its straight lines across the
first's, turned by turn about its own axis, its tip by depth in the first's */
static ksl_SE3_t Crossed(double turn, double depth) {
  const double reach = std::hypot(kMajor * std::cos(turn),
                                  kMinor * std::sin(turn));
  ksl_SE3_t d = At(kMajor + reach - depth, 0.5 * kWidth, 0.5 * kWidth);
  const double c = std::cos(turn), s = std::sin(turn);
  // columns: its x and y in the plane of x and z, and its z along -y
  d.R.at[0][0] = c;
  d.R.at[0][1] = 0.0;
  d.R.at[0][2] = s;
  d.R.at[1][0] = -s;
  d.R.at[1][1] = 0.0;
  d.R.at[1][2] = c;
  d.R.at[2][0] = 0.0;
  d.R.at[2][1] = -1.0;
  d.R.at[2][2] = 0.0;
  return d;
}

TEST_P(CurvedOnCurved, CrossedCamsAreAPoint) {
  // their straight lines cross, so a point, not a line
  CamScene scene(GetParam().first, GetParam().second);
  const shift::core::collision_geometry::ContactPair& pair = scene.Test(Crossed(0.1, 1e-5));
  ASSERT_TRUE(pair.collision);
  ASSERT_EQ(pair.manifold.size(), 1u);
  EXPECT_EQ(pair.manifold.front().length, 0.0);
  EXPECT_NEAR(pair.manifold.front().depth, 1e-5, 1e-7);
}

TEST_P(CurvedOnCurved, CrossedCamsHaveNoJumpOverASeam) {
  // the point runs over the seam at the tip of the second cam
  CamScene scene(GetParam().first, GetParam().second);
  double previous = 0.0, greatest = 0.0, peak = 0.0;
  for(int i = -1000; i <= 1000; i++) {
    const double force = ElasticForce(scene.Test(Crossed(1e-5 * i, 1e-5)));
    if(i > -1000) {
      greatest = std::max(greatest, std::fabs(force - previous));
    }
    previous = force;
    peak = std::max(peak, force);
  }
  ASSERT_GT(peak, 0.0);
  EXPECT_LT(greatest / peak, 0.02);
}

/* a spur gear of 20 teeth of module 5 mm and a pressure angle of 20 degrees:
scripts/make_contact_step.py */
static const double kPitchRadius = 0.05;
static const double kPressureAngle = 20.0 * M_PI / 180.0;
static const double kBaseRadius = kPitchRadius * std::cos(kPressureAngle);
static const double kHalfTooth = M_PI / 40.0;

/* the second tooth, on a gear 100 mm from the first, the first turned by phi
from the pitch point and the second with it, then on by lead into the first */
static ksl_SE3_t Meshing(double phi, double lead) {
  ksl_SE3_t d = At(2.0 * kPitchRadius, 0.0);
  const double turn = M_PI - kHalfTooth - phi + lead;
  const double c = std::cos(turn), s = std::sin(turn);
  d.R.at[0][0] = c;
  d.R.at[0][1] = s;
  d.R.at[1][0] = -s;
  d.R.at[1][1] = c;
  return d;
}

TEST(GearToothContact, FlanksMeshAlongALine) {
  // rolling through the mesh: a line across the width, at the depth of the
  // lead on the base circle, its normal along the line of action, and passing
  // through the pitch point at phi = 0
  CamScene scene("gear_tooth", "gear_tooth");
  const double lead = 2e-4, depth = kBaseRadius * lead;
  for(int i = -10; i <= 10; i++) {
    const double phi = 0.01 * i;
    SCOPED_TRACE(phi);
    const auto& pair = scene.Test(Meshing(phi, lead), phi - kHalfTooth);
    ASSERT_TRUE(pair.collision);
    ASSERT_EQ(pair.manifold.size(), 3u);
    double length = 0.0;
    for(const auto& c : pair.manifold) {
      length += c.length;
      EXPECT_NEAR(c.depth, depth, 2e-6);
      EXPECT_NEAR(std::fabs(c.normal.x * std::sin(kPressureAngle) +
                            c.normal.y * std::cos(kPressureAngle)),
                  1.0, 1e-3);
      // on the line of action, from the pitch point along the base circle
      const double along = (c.point_a.x - kPitchRadius) * std::cos(kPressureAngle) -
                           c.point_a.y * std::sin(kPressureAngle);
      EXPECT_NEAR(along, 0.0, 1e-5);
    }
    EXPECT_NEAR(length, kWidth, 1e-9);
  }
}

TEST(GearToothContact, MeshHasNoJump) {
  CamScene scene("gear_tooth", "gear_tooth");
  const double lead = 2e-4;
  double previous = 0.0, greatest = 0.0, peak = 0.0;
  for(int i = -1000; i <= 1000; i++) {
    const double phi = 1e-4 * i;
    const double force =
      ElasticForce(scene.Test(Meshing(phi, lead), phi - kHalfTooth));
    if(i > -1000) {
      greatest = std::max(greatest, std::fabs(force - previous));
    }
    previous = force;
    peak = std::max(peak, force);
  }
  ASSERT_GT(peak, 0.0);
  EXPECT_LT(greatest / peak, 0.02);
}

TEST(GearToothContact, BacklashHasNoContact) {
  CamScene scene("gear_tooth", "gear_tooth");
  EXPECT_FALSE(scene.Test(Meshing(0.03, -2e-4), 0.03 - kHalfTooth).collision);
}

/* the plane z = at, its solid beyond it */
static ksl_SE3_t GroundAtZ(double at) {
  ksl_SE3_t d = At(0.0, 0.0, at);
  d.R.at[0][0] = 0.0;
  d.R.at[0][2] = 1.0;
  d.R.at[2][0] = -1.0;
  d.R.at[2][2] = 0.0;
  return d;
}

TEST(BentLine, RingOnAPlaneIsAQuarterCircle) {
  // examples/cad/ring_sector.step: a tube of radius 8 mm bent to 30 mm over a
  // quarter of a turn. The line bends with it: its stations are on the circle,
  // at the depth, and their lengths add up to the arc
  const double radius = 0.03, tube = 0.008, depth = 1e-5;
  CamScene scene("ring_sector", "", true);
  const auto& pair = scene.Test(GroundAtZ(tube - depth));
  ASSERT_TRUE(pair.collision);
  ASSERT_EQ(pair.manifold.size(), 3u);
  double length = 0.0;
  for(const auto& c : pair.manifold) {
    EXPECT_GT(c.length, 0.0);
    EXPECT_NEAR(c.depth, depth, 1e-9);
    EXPECT_NEAR(std::fabs(c.normal.z), 1.0, 1e-6);
    EXPECT_NEAR(std::hypot(c.point_a.x, c.point_a.y), radius, 1e-7);
    length += c.length;
  }
  EXPECT_NEAR(length, 0.5 * M_PI * radius, 0.01 * 0.5 * M_PI * radius);
}

TEST(BentLine, TurningRingHasNoJump) {
  const double tube = 0.008, depth = 1e-5;
  CamScene scene("ring_sector", "", true);
  double previous = 0.0, greatest = 0.0, peak = 0.0;
  for(int i = 0; i <= 400; i++) {
    const double force =
      ElasticForce(scene.Test(GroundAtZ(tube - depth), 0.0157 * i));
    if(i > 0) {
      greatest = std::max(greatest, std::fabs(force - previous));
    }
    previous = force;
    peak = std::max(peak, force);
  }
  ASSERT_GT(peak, 0.0);
  EXPECT_LT(greatest / peak, 0.02);
}

/* the distance along x of the center of a roller touching the cam turned by
angle: where the curve parallel to the ellipse at the roller's radius crosses
the x axis, on its +x side */
static double RollerCenter(double angle) {
  auto point = [&](double t, double& x, double& y) {
    double p[2], n[2];
    Ellipse(t, p, n);
    const double px = p[0] + kRollerRadius * n[0];
    const double py = p[1] + kRollerRadius * n[1];
    x = std::cos(angle) * px - std::sin(angle) * py;
    y = std::sin(angle) * px + std::cos(angle) * py;
  };
  // the parameter t of the ellipse near -angle, and y rising through it
  double lo = -angle - 1.5, hi = -angle + 1.5, x, y;
  for(int i = 0; i < 60; i++) {
    const double mid = 0.5 * (lo + hi);
    point(mid, x, y);
    (y > 0.0 ? hi : lo) = mid;
  }
  return x;
}

TEST(CadLineContact, FollowerRidesTheCam) {
  // examples/cad/cam_follower.json: the slider follows the cam's profile
  // through more than a turn, past the seams of its faces
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) + "/cad/cam_follower.json");
  double worst = 0.0, turned = 0.0;
  int samples = 0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    if(t < 0.01) {
      return; // settling on the spring
    }
    turned = y[0]; // cam_shaft, then slide
    const double center = kMajor + kRollerRadius + y[1];
    worst = std::max(worst, std::fabs(center - RollerCenter(turned)));
    samples++;
  });
  context.Simulate();
  shift::test::RecordContacts(context);
  EXPECT_GT(samples, 50);
  EXPECT_GT(std::fabs(turned), 2.0 * M_PI);
  // to within the contact's deflection under the spring, of the 15 mm lift
  EXPECT_LT(worst, 2e-5);
}
