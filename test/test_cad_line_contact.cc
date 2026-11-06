/* line contact on curved faces of CAD geometry: an elliptical cam, its side
of rational B-spline or extruded faces, against a roller, a flat follower and
the ground (scripts/make_cam_step.py) */
#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"

using shift::core::collision_geometry::ContactPair;

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
  ContactPair& Test(const ksl_SE3_t& follower, double angle = 0.0) {
    ksl_SE3_t d = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
    const double c = std::cos(angle), s = std::sin(angle);
    d.R.at[0][0] = c;
    d.R.at[0][1] = s;
    d.R.at[1][0] = -s;
    d.R.at[1][1] = c;
    cam_->UpdatePose(d);
    follower_->UpdatePose(follower);
    pair_ = ContactPair();
    pair_.a = cam_;
    pair_.b = follower_;
    shift::core::collision_geometry::TestContact(pair_);
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
  ContactPair pair_;
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
static double ElasticForce(const ContactPair& pair) {
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
static void ExpectFullLine(const ContactPair& pair, double depth) {
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
  // from level, through end loading, to the rim of the roller
  CamScene scene(GetParam(), "roller");
  const double depth = 0.4e-6;
  double previous = 0.0, previous_theta = 0.0;
  bool line = false, rim = false;
  for(double theta = 1e-5; theta < 0.5; theta *= 1.01) {
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
  EXPECT_GT(samples, 50);
  EXPECT_GT(std::fabs(turned), 2.0 * M_PI);
  // to within the contact's deflection under the spring, of the 15 mm lift
  EXPECT_LT(worst, 2e-5);
}
