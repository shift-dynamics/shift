/* contact on the exact surfaces of CAD geometry (collision_geometry::Cad):
the sprocket and track links of examples/cad (scripts/make_track_step.py),
a bushing in a seat of the sprocket (a line of contact between parallel
cylinders), the grousers of a shoe and a tooth of the sprocket on the ground,
and three links of track settling on the sprocket */
#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/body/base.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/collision_geometry/hertz.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"
#include "shift/core/force_module/track.h"

static const double kGravity = 9.81;
/* the track of make_track_step.py: 190 mm pitch, 21 teeth engaging every
second one, seats of 30.5 mm radius for bushings of 29.5 mm */
static const double kPitch = 0.19;
static const double kTeeth = 21;
static const double kPitchRadius = 0.095 / std::sin(2.0 * M_PI / kTeeth);
static const double kSeatRadius = 0.0305;
static const double kBushingRadius = 0.0295;
static const double kTipRadius = kPitchRadius + 0.03;
static const double kSprocketWidth = 0.06;
/* the angle of the sprocket between the bushings of neighboring links */
static const double kLinkAngle = 4.0 * M_PI / kTeeth;

/* mass properties of a link of steel (7850 kg/m^3), from the volumes of its
solids (bushing, pin, four link plates and the shoe), in the link's frame;
the moments of inertia are those of the shoe and plates, rounded */
static const double kLinkMass = 38.06;
static const char* kLinkInertia = R"("center_of_mass": [0.0974, 0, 0.0359],
       "inertia": {"mass": 38.06, "Ixx": 0.75, "Iyy": 0.15, "Izz": 0.8,
                   "Ixy": 0, "Iyz": 0, "Izx": 0})";

/* the JSON of a displacement: translation p and rotation matrix (rows) */
static std::string Displacement(const double p[3], const double r[3][3]) {
  std::ostringstream s;
  s.precision(17);
  s << "{\"T\": [" << p[0] << ", " << p[1] << ", " << p[2] << "], \"R\": [";
  for(int i = 0; i < 3; i++) {
    s << (i ? ", " : "") << "[" << r[i][0] << ", " << r[i][1] << ", "
      << r[i][2] << "]";
  }
  s << "]}";
  return s.str();
}

/* the rotation whose columns are x, y and z */
static void Columns(const double x[3], const double y[3], const double z[3],
                    double r[3][3]) {
  for(int k = 0; k < 3; k++) {
    r[k][0] = x[k];
    r[k][1] = y[k];
    r[k][2] = z[k];
  }
}

/*
The models are in the frame of the sprocket: its axis along z, a seat on
the x axis, which is up (gravity along -x). The links lie around the
sprocket, their pins along z; the ground is the plane x = 0.
*/
struct Track {
  int links = 0;
  /* the first link's bushing at the seat this far around the sprocket from
  the top (radians, towards +y), moved down the seat by drop */
  double first_angle = 0.0;
  double drop = 0.0;
  /* the sprocket rigidly joined to the fixed body at the origin, or free
  with its axis at this height, a tooth's tip down if tip_down */
  bool sprocket_free = false;
  double sprocket_height = 0.0;
  bool tip_down = false;
  /* a link alone, upside down (its shoe down), the tips of its grousers at
  this height */
  bool link_on_ground = false;
  double grouser_height = 0.0;
  bool ground = false;
  double youngs_modulus = 2e11;
  double time_span = 0.0;
};

static std::string TrackModel(const Track& t) {
  std::ostringstream m;
  m.precision(17);
  m << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [)"
    << -kGravity << R"(, 0.0, 0.0],
    "include_directories": [")"
    << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-6,
                     "absolute_error": 1e-9, "time_span": )"
    << t.time_span << R"(, "time_step": 0.01}]
  },
  "assemblies": [{"name": "track",
    "bodies": [
      {"name": "sprocket",
       "geometry": [{"type": "cad", "filename": "sprocket.step"}],
       "collision_geometry": [{"type": "cad", "material": "steel"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 130.7, "Ixx": 3.8, "Iyy": 3.8, "Izz": 7.5,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}})";
  for(int i = 1; i <= t.links; i++) {
    m << R"(,
      {"name": "link)"
      << i << R"(",
       "frames": [{"name": "pin", "T": [)"
      << kPitch << R"(, 0, 0]}],
       "geometry": [{"type": "cad", "filename": "track_link.step"}],
       "collision_geometry": [{"type": "cad", "material": "steel"}],
       )" << kLinkInertia
      << "}";
  }
  if(t.ground) {
    m << R"(,
      {"name": "ground", "geometry": [],
       "collision_geometry": [{"name": "ground", "type": "plane",
                               "normal": [1, 0, 0], "material": "steel"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}})";
  }
  m << R"(
    ],
    "joints": [)";
  if(t.sprocket_free) {
    /* turned about its axis by phi */
    const double phi = t.tip_down ? M_PI - M_PI / kTeeth : 0.0;
    const double x[3] = {std::cos(phi), std::sin(phi), 0.0};
    const double y[3] = {-std::sin(phi), std::cos(phi), 0.0};
    const double z[3] = {0.0, 0.0, 1.0};
    double r[3][3];
    Columns(x, y, z, r);
    const double p[3] = {t.sprocket_height, 0.0, 0.0};
    m << R"(
      {"name": "sprocket_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["sprocket"], "origin"],
       "initial_displacement": )"
      << Displacement(p, r) << "}";
  } else {
    m << R"(
      {"name": "sprocket_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["sprocket"], "origin"]})";
  }
  if(t.ground) {
    m << R"(,
      {"name": "ground_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["ground"], "origin"]})";
  }
  for(int i = 1; i <= t.links; i++) {
    m << ",";
    if(i > 1) {
      // turned about the previous link's pin to the next seat but one
      m << R"(
      {"name": "link)"
        << i << R"(_joint", "type": "revolute",
       "parent": [["link)"
        << i - 1 << R"("], "pin"], "child": [["link)" << i
        << R"("], "origin"],
       "axis": [0, 1, 0], "initial_displacement": )"
        << kLinkAngle << "}";
      continue;
    }
    double r[3][3], p[3];
    if(t.link_on_ground) {
      // the link's z (out of the track loop) down, its pin along z
      const double x[3] = {0.0, -1.0, 0.0}, y[3] = {0.0, 0.0, 1.0},
                   z[3] = {-1.0, 0.0, 0.0};
      Columns(x, y, z, r);
      p[0] = 0.092 + t.grouser_height; // the grousers' tips at z = 92 mm
      p[1] = 0.0;
      p[2] = 0.0;
    } else {
      /* the bushing at the seat, the link along the chord to the next
      seat but one: x along it, y along the sprocket's axis, z out */
      const double a = t.first_angle, mid = a + 0.5 * kLinkAngle;
      const double radius = kPitchRadius - t.drop;
      const double x[3] = {-std::sin(mid), std::cos(mid), 0.0};
      const double y[3] = {0.0, 0.0, 1.0};
      const double z[3] = {std::cos(mid), std::sin(mid), 0.0};
      Columns(x, y, z, r);
      p[0] = radius * std::cos(a);
      p[1] = radius * std::sin(a);
      p[2] = 0.0;
    }
    m << R"(
      {"name": "link1_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["link1"], "origin"],
       "initial_displacement": )"
      << Displacement(p, r) << "}";
  }
  m << R"(
    ],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [{"name": "steel", "youngs_modulus": )"
    << t.youngs_modulus << R"(, "poisson_ratio": 0.3,
                     "restitution": 0.3, "friction": 0.3}]}]
  }]
})";
  return m.str();
}

class CadContact : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }

  /* loads the model, with the poses of its initial conditions */
  static void Load(shift::core::Context& context, const std::string& name,
                   const Track& track) {
    const std::string path = testing::TempDir() + "/" + name + ".json";
    {
      std::ofstream file(path);
      file << TrackModel(track);
    }
    try {
      context.LoadJSON(path);
      context.UpdateD();
      std::remove(path.c_str());
    } catch(...) {
      std::remove(path.c_str());
      throw;
    }
  }

  /* the contacts of two geometries, by name */
  static std::vector<shift::core::collision_geometry::ContactPair*>
  Between(shift::core::Context& context, const std::string& a,
          const std::string& b) {
    std::vector<shift::core::collision_geometry::ContactPair*> out;
    for(auto pair : context.get_contacts()) {
      const std::string& na = pair->a->get_name();
      const std::string& nb = pair->b->get_name();
      if((na == a && nb == b) || (na == b && nb == a)) {
        out.push_back(pair);
      }
    }
    return out;
  }
};

TEST_F(CadContact, NothingTouchesAtTheSeats) {
  // bushings at the middle of their seats have 1 mm of clearance all around
  Track t;
  t.links = 3;
  t.first_angle = -1.5 * kLinkAngle;
  shift::core::Context context;
  Load(context, "cad_clear", t);
  context.DetectContacts();
  for(auto pair : context.get_contacts()) {
    ADD_FAILURE() << pair->a->get_name() << " and " << pair->b->get_name()
                  << " touch";
  }
}

TEST_F(CadContact, BushingInSeatIsALine) {
  // a bushing moved down its seat by the clearance and depth: a line across
  // the width of the sprocket at the bottom of the seat
  for(double depth : {0.3e-3, 0.05e-3}) {
    SCOPED_TRACE(depth);
    Track t;
    t.links = 1;
    t.drop = kSeatRadius - kBushingRadius + depth;
    shift::core::Context context;
    Load(context, "cad_seat", t);
    context.DetectContacts();
    ASSERT_EQ(context.get_contacts().size(), 1u);
    const auto& pair = *context.get_contacts()[0];
    ASSERT_TRUE(pair.point_laws);
    const bool sprocket_is_a = pair.a->get_name() == "sprocket";
    double length = 0.0;
    for(const auto& c : pair.manifold) {
      EXPECT_GT(c.length, 0.0);
      length += c.length;
      EXPECT_NEAR(c.depth, depth, 1e-9);
      const ksl_vec3_t& on_sprocket = sprocket_is_a ? c.point_a : c.point_b;
      EXPECT_NEAR(on_sprocket.x, kPitchRadius - kSeatRadius, 1e-9);
      EXPECT_NEAR(on_sprocket.y, 0.0, 1e-9);
      EXPECT_LE(std::fabs(on_sprocket.z), 0.5 * kSprocketWidth + 1e-9);
      // the sprocket pushes the bushing up (+x)
      EXPECT_NEAR(c.normal.x, sprocket_is_a ? 1.0 : -1.0, 1e-9);
    }
    EXPECT_NEAR(length, kSprocketWidth, 1e-9);
  }
}

TEST_F(CadContact, GrousersOnTheGround) {
  // a link upside down, its grousers' tips 0.2 mm into the ground: the
  // corners of the tips of the three grousers
  Track t;
  t.links = 1;
  t.sprocket_free = true;
  t.sprocket_height = 5.0; // out of the way
  t.link_on_ground = true;
  t.grouser_height = -0.2e-3;
  t.ground = true;
  shift::core::Context context;
  Load(context, "cad_grousers", t);
  context.DetectContacts();
  EXPECT_EQ(context.get_contacts().size(), 1u);
  auto pairs = Between(context, "link1", "ground");
  ASSERT_EQ(pairs.size(), 1u);
  const auto& manifold = pairs[0]->manifold;
  EXPECT_EQ(manifold.size(), 12u);
  for(const auto& c : manifold) {
    EXPECT_NEAR(c.depth, 0.2e-3, 1e-12);
    EXPECT_EQ(c.length, 0.0);
    EXPECT_NEAR(std::fabs(c.normal.x), 1.0, 1e-12);
  }
}

TEST_F(CadContact, ToothTipOnTheGround) {
  // the sprocket on its side, the tip of a tooth 0.1 mm into the ground: a
  // line across the width of the sprocket (a convex cylinder on a plane),
  // and nothing at the edges and corners of the tip
  Track t;
  t.sprocket_free = true;
  t.tip_down = true;
  t.sprocket_height = kTipRadius - 0.1e-3;
  t.ground = true;
  shift::core::Context context;
  Load(context, "cad_tip", t);
  context.DetectContacts();
  auto pairs = Between(context, "sprocket", "ground");
  ASSERT_EQ(pairs.size(), 1u);
  double length = 0.0;
  for(const auto& c : pairs[0]->manifold) {
    EXPECT_GT(c.length, 0.0);
    EXPECT_NEAR(c.depth, 0.1e-3, 1e-9);
    length += c.length;
  }
  EXPECT_NEAR(length, kSprocketWidth, 1e-9);
}

TEST_F(CadContact, LinksSettleOnTheSprocket) {
  // three links over the top of the sprocket, their bushings 1 mm above the
  // bottoms of their seats: they drop into them and come to rest, carried
  // by the sprocket
  Track t;
  t.links = 3;
  t.first_angle = -1.5 * kLinkAngle;
  t.youngs_modulus = 2e9;
  t.time_span = 0.2;
  shift::core::Context context;
  Load(context, "cad_settle", t);
  context.Simulate();
  double up = 0.0;
  int lines = 0;
  for(auto pair : context.get_contacts()) {
    const bool sprocket_is_a = pair->a->get_name() == "sprocket";
    ASSERT_TRUE(sprocket_is_a || pair->b->get_name() == "sprocket")
      << pair->a->get_name() << " and " << pair->b->get_name();
    up += sprocket_is_a ? pair->force.x : -pair->force.x;
    for(const auto& c : pair->manifold) {
      lines += c.length > 0.0 ? 1 : 0;
    }
  }
  // the bushings in their seats (and the last pin in its) carry the weight
  EXPECT_GE(lines, 6);
  const double weight = 3.0 * kLinkMass * kGravity;
  EXPECT_NEAR(up, weight, 0.02 * weight);
  // each bushing has sunk through the clearance of its seat onto its wall:
  // its center the clearance from the seat's, and still at its seat
  const double seat_angle = 2.0 * M_PI / kTeeth;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name().rfind("link", 0) != 0) {
      continue;
    }
    const ksl_vec3_t p = context.get_body_origin_pose(body).t;
    const double angle = std::atan2(p.y, p.x);
    const double seat = seat_angle * std::round(angle / seat_angle);
    const double dx = p.x - kPitchRadius * std::cos(seat);
    const double dy = p.y - kPitchRadius * std::sin(seat);
    EXPECT_NEAR(std::hypot(dx, dy), kSeatRadius - kBushingRadius, 2e-5)
      << body->get_name();
    EXPECT_LT(dx, 0.0) << body->get_name(); // below the seat's center
  }
}

TEST_F(CadContact, TrackLoopCloses) {
  // the benchmark examples/cad/track_loop.json, x forward and z down, the
  // axles along y: its 44 links close around the sprocket and the idler, and nothing
  // touches before it starts
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) + "/cad/track_loop.json");
  context.UpdateD();
  shift::core::body::Base* first = nullptr;
  shift::core::body::Base* last = nullptr;
  int links = 0;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "link1") {
      first = body;
    }
    if(body->get_name() == "link44") {
      last = body;
    }
    links += body->get_name().rfind("link", 0) == 0;
    // the wheels' axes (z, the third column) along y, the idler in front
    const std::string& name = body->get_name();
    if(name == "sprocket" || name == "idler") {
      const ksl_SE3_t d = context.get_body_origin_pose(body);
      EXPECT_NEAR(d.R.at[2][1], 1.0, 1e-12) << name;
      EXPECT_NEAR(d.t.x, name == "idler" ? 3.1818287751528 : 0.0, 1e-9);
      EXPECT_NEAR(d.t.y, 0.0, 1e-12) << name;
      // above the ground, the plane z = 0
      EXPECT_NEAR(d.t.z, -0.4139001617497, 1e-9) << name;
    }
  }
  EXPECT_EQ(links, 44);
  ASSERT_TRUE(first && last);
  // the last link's pin at the first link's bushing
  const ksl_SE3_t d = context.get_body_origin_pose(last);
  const ksl_vec3_t pin = {{kPitch, 0.0, 0.0}};
  ksl_vec3_t p;
  ksl_product_dv(&d, &pin, &p);
  const ksl_vec3_t q = context.get_body_origin_pose(first).t;
  EXPECT_NEAR(p.x, q.x, 1e-9);
  EXPECT_NEAR(p.y, q.y, 1e-9);
  EXPECT_NEAR(p.z, q.z, 1e-9);
  // the first link's bushing on the sprocket's pitch circle, in the plane
  // y = 0, and the link's pins along y
  EXPECT_NEAR(std::hypot(q.x, q.z + 0.4139001617497), kPitchRadius, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-12);
  EXPECT_NEAR(context.get_body_origin_pose(first).R.at[1][1], 1.0, 1e-12);
  context.DetectContacts();
  for(auto pair : context.get_contacts()) {
    ADD_FAILURE() << pair->a->get_name() << " touches "
                  << pair->b->get_name() << " at "
                  << pair->manifold.size() << " points, deepest "
                  << pair->depth;
  }
}

/* ------------------------------------------------------------------------ */
/* Hertz contact of an ellipse, and lines of contact out of parallel */

TEST(HertzEllipse, EllipticIntegrals) {
  // K(m) and E(m) of parameter m (Abramowitz and Stegun, table 17.1)
  const double table[3][3] = {{0.5, 1.8540746773013719, 1.3506438810476755},
                              {0.9, 2.5780921133481733, 1.1047747327040733},
                              {0.99, 3.6956373629898747, 1.0159935450252240}};
  for(const auto& row : table) {
    double k, e, k_minus_e;
    shift::core::collision_geometry::CompleteEllipticIntegrals(
      row[0], 1.0 - row[0], k, e, k_minus_e);
    EXPECT_NEAR(k, row[1], 1e-13);
    EXPECT_NEAR(e, row[2], 1e-13);
    EXPECT_NEAR(k_minus_e, row[1] - row[2], 1e-13);
  }
  // K - E = pi/4 m (1 + 3/8 m) near m = 0, without cancellation
  double k, e, k_minus_e;
  shift::core::collision_geometry::CompleteEllipticIntegrals(1e-9, 1.0 - 1e-9,
                                                             k, e, k_minus_e);
  EXPECT_NEAR(k_minus_e / (0.25 * M_PI * 1e-9), 1.0, 1e-8);
}

TEST(HertzEllipse, FactorAndElongation) {
  using shift::core::collision_geometry::HertzEllipseFactor;
  double elongation;
  EXPECT_EQ(HertzEllipseFactor(1.0, &elongation), 1.0); // a circle
  EXPECT_EQ(elongation, 1.0);
  EXPECT_NEAR(HertzEllipseFactor(1.0 + 1e-6), 1.0, 1e-5);
  /* against the curve fits of Hamrock and Brewe (1983), within their
  accuracy: the axes' ratio a / b = 1.0339 alpha^0.636, and the
  stiffness from their deflection,
  delta = F [9 / (2 E R) (P / (pi k E'))^2]^(1/3), with
  E = 1.0003 + 0.5968 / alpha and F = 1.5277 + 0.6023 ln alpha, E' = 2 E*
  and 1/R = 2 (A + B): the factor is pi/2 k sqrt(E) F^(-3/2) */
  double previous = 1.0;
  for(double alpha : {1.5, 2.0, 5.0, 10.0, 30.0}) {
    SCOPED_TRACE(alpha);
    const double g = HertzEllipseFactor(alpha, &elongation);
    const double k = 1.0339 * std::pow(alpha, 0.636);
    const double integral_e = 1.0003 + 0.5968 / alpha;
    const double integral_f = 1.5277 + 0.6023 * std::log(alpha);
    EXPECT_NEAR(elongation / k, 1.0, 0.04);
    EXPECT_NEAR(g / (0.5 * M_PI * k * std::sqrt(integral_e) *
                     std::pow(integral_f, -1.5)),
                1.0, 0.1);
    EXPECT_GT(g, previous); // stiffer as the ellipse lengthens
    previous = g;
  }
  /* beyond the fits' range: the ellipse's axes satisfy Hertz's relation
  B / A = (E / (1 - e^2) - K) / (K - E), e^2 = 1 - (b / a)^2 */
  for(double alpha : {100.0, 1e4}) {
    SCOPED_TRACE(alpha);
    EXPECT_GT(HertzEllipseFactor(alpha, &elongation), previous);
    const double complement = 1.0 / (elongation * elongation);
    double k, e, k_minus_e;
    shift::core::collision_geometry::CompleteEllipticIntegrals(
      1.0 - complement, complement, k, e, k_minus_e);
    EXPECT_NEAR((e / complement - k) / k_minus_e / alpha, 1.0, 1e-9);
  }
}

/* idlers (examples/cad/idler.step: tread radius kIdlerRadius, 60 mm wide),
each free at a pose, and the ground, the plane z = 0 (z down) */
static const double kIdlerRadius = kPitchRadius - kBushingRadius - 0.001;
struct Pose {
  double p[3];
  double r[3][3];
};

static std::string WheelsModel(const std::vector<Pose>& idlers, bool ground) {
  std::ostringstream m;
  m.precision(17);
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
  "assemblies": [{"name": "wheels",
    "bodies": [)";
  for(size_t i = 0; i < idlers.size(); i++) {
    m << (i ? "," : "") << R"(
      {"name": "idler)"
      << i << R"(",
       "geometry": [{"type": "cad", "filename": "idler.step"}],
       "collision_geometry": [{"name": "idler)"
      << i << R"(", "type": "cad", "material": "steel"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 111.2, "Ixx": 2.68, "Iyy": 2.68, "Izz": 5.29,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}})";
  }
  if(ground) {
    m << R"(,
      {"name": "ground", "geometry": [],
       "collision_geometry": [{"name": "ground", "type": "plane",
                               "normal": [0, 0, -1], "material": "steel"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}})";
  }
  m << R"(
    ],
    "joints": [)";
  for(size_t i = 0; i < idlers.size(); i++) {
    m << (i ? "," : "") << R"(
      {"name": "idler)"
      << i << R"(_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["idler)" << i
      << R"("], "origin"],
       "initial_displacement": )"
      << Displacement(idlers[i].p, idlers[i].r) << "}";
  }
  if(ground) {
    m << R"(,
      {"name": "ground_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["ground"], "origin"]})";
  }
  m << R"(
    ],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [{"name": "steel", "youngs_modulus": 2e11,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.3}]}]
  }]
})";
  return m.str();
}

/* the contacts of a model of idlers, with the poses of its initial
conditions */
static void LoadWheels(shift::core::Context& context, const std::string& name,
                       const std::vector<Pose>& idlers, bool ground) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << WheelsModel(idlers, ground);
  }
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.UpdateD();
  context.DetectContacts();
}

/* the contact of the geometries of a model of idlers (in the order of
its bodies, the ground last), with idler i moved to poses[i]: the model is
loaded once and its geometry moved, as Context::DetectContacts does */
class Wheels {
public:
  Wheels(const std::string& name, const std::vector<Pose>& idlers,
         bool ground) {
    LoadWheels(context_, name, idlers, ground);
    const size_t count = idlers.size() + (ground ? 1 : 0);
    geometry_.resize(count);
    for(auto g : context_.get_active_collision_geometry()) {
      const std::string& n = g->get_name();
      if(n == "ground") {
        geometry_[count - 1] = g;
      } else if(n.rfind("idler", 0) == 0 && n.size() > 5) {
        geometry_[std::stoul(n.substr(5))] = g;
      }
    }
  }
  shift::core::collision_geometry::ContactPair&
  Test(const std::vector<Pose>& idlers) {
    for(size_t i = 0; i < idlers.size(); i++) {
      ksl_SE3_t d;
      for(int k = 0; k < 3; k++) {
        d.t.at[k] = idlers[i].p[k];
        for(int j = 0; j < 3; j++) {
          d.R.at[j][k] = idlers[i].r[k][j]; // column-major
        }
      }
      geometry_[i]->UpdatePose(d);
    }
    pair_ = shift::core::collision_geometry::ContactPair();
    pair_.a = geometry_[0];
    pair_.b = geometry_[1];
    shift::core::collision_geometry::TestContact(pair_);
    return pair_;
  }

private:
  shift::core::Context context_;
  std::vector<shift::core::collision_geometry::Base*> geometry_;
  shift::core::collision_geometry::ContactPair pair_;
};

/* the elastic force of a pair's points, sum of w K d^e by the laws of the
contact force module, for a modulus E* = 1 */
static double ElasticForce(const shift::core::collision_geometry::ContactPair&
                             pair) {
  double force = 0.0;
  for(const auto& c : pair.manifold) {
    double stiffness, exponent;
    shift::core::force_module::Contact::PointLaw(c, 1.0, 1e-4, stiffness,
                                                 exponent);
    force += c.weight * stiffness * std::pow(c.depth, exponent);
  }
  return force;
}

/* an idler on the ground, its axis tilted by theta from level (about x)
and its lower end depth into the ground: the axis (0, cos, sin), the
lowest line of its tread towards (0, -sin, cos) from the axis */
static Pose TiltedIdler(double theta, double depth) {
  Pose pose;
  const double c = std::cos(theta), s = std::sin(theta);
  const double x[3] = {1.0, 0.0, 0.0}, y[3] = {0.0, s, -c},
               z[3] = {0.0, c, s};
  Columns(x, y, z, pose.r);
  const double half = 0.5 * kSprocketWidth;
  pose.p[0] = 0.0;
  pose.p[1] = 0.0;
  pose.p[2] = depth - half * s - kIdlerRadius * c;
  return pose;
}

TEST_F(CadContact, TiltedIdlerLoadsItsEnd) {
  // tilted by 1 mrad, its end 50 um deep: the line under the ground is
  // 50 mm of its 60 mm, its depth falling to 0 along it
  const double theta = 1e-3, depth = 50e-6;
  shift::core::Context context;
  LoadWheels(context, "tilted_idler", {TiltedIdler(theta, depth)}, true);
  ASSERT_EQ(context.get_contacts().size(), 1u);
  const auto& pair = *context.get_contacts()[0];
  ASSERT_TRUE(pair.point_laws);
  double length = 0.0, load = 0.0;
  const double half = 0.5 * kSprocketWidth;
  for(const auto& c : pair.manifold) {
    ASSERT_GT(c.length, 0.0);
    EXPECT_EQ(c.weight, 1.0);
    length += c.length;
    load += c.length * c.depth;
    // its station along the axis from its depth: on the lowest line
    const ksl_vec3_t& p =
      pair.a->get_name() == "ground" ? c.point_b : c.point_a;
    const double s =
      (p.y + kIdlerRadius * std::sin(theta)) / std::cos(theta);
    EXPECT_NEAR(c.depth, depth - (half - s) * std::sin(theta), 1e-9);
  }
  const double expected = depth / std::sin(theta);
  EXPECT_NEAR(length, expected, 1e-8);
  // the depth integrated along the line: exact for a straight depth
  EXPECT_NEAR(load, 0.5 * depth * expected, 1e-14);
}

TEST_F(CadContact, TiltingIdlerHasNoJump) {
  // the force of an idler on the ground, its lower end 50 um deep, from
  // level (a line) through end loading to its rim, as it tilts up to
  // standing on its end (the rim, an edge): each step of 1% changes it
  // smoothly
  const double depth = 50e-6;
  double previous = 0.0, previous_theta = 0.0;
  bool line = false, rim = false;
  Wheels wheels("tilting_idler", {TiltedIdler(0.0, depth)}, true);
  for(double theta = 1e-5; theta < 0.5 * M_PI - 0.005; theta *= 1.01) {
    SCOPED_TRACE(theta);
    const auto& pair = wheels.Test({TiltedIdler(theta, depth)});
    ASSERT_TRUE(pair.collision);
    const double force = ElasticForce(pair);
    ASSERT_GT(force, 0.0);
    line = line || (pair.manifold.size() > 1 &&
                    pair.manifold.front().length > 0.0);
    rim = rim || (pair.manifold.size() == 1 &&
                  pair.manifold.front().length == 0.0);
    if(previous > 0.0) {
      EXPECT_NEAR(force / previous, 1.0, 0.03)
        << "from theta " << previous_theta;
    }
    previous = force;
    previous_theta = theta;
  }
  EXPECT_TRUE(line);
  EXPECT_TRUE(rim);
}

/* two idlers tread to tread, the lower one level about y, the upper one's
axis turned by theta about z, approach depth */
static std::vector<Pose> CrossedIdlers(double theta, double depth) {
  Pose lower, upper;
  const double x[3] = {1.0, 0.0, 0.0}, y[3] = {0.0, 0.0, -1.0},
               z[3] = {0.0, 1.0, 0.0};
  Columns(x, y, z, lower.r); // its axis (z) along y
  lower.p[0] = lower.p[1] = lower.p[2] = 0.0;
  const double c = std::cos(theta), s = std::sin(theta);
  const double xu[3] = {c, s, 0.0}, zu[3] = {-s, c, 0.0};
  Columns(xu, y, zu, upper.r);
  upper.p[0] = upper.p[1] = 0.0;
  upper.p[2] = -(2.0 * kIdlerRadius - depth);
  return {lower, upper};
}

TEST_F(CadContact, CrossingCylindersHaveNoJump) {
  // two idlers 50 um into each other: a line along their 60 mm when
  // parallel, a point when crossed, where their gap is round (equal
  // radii), changing smoothly between
  const double depth = 50e-6;
  double previous = 0.0;
  Wheels wheels("crossing_idlers", CrossedIdlers(0.0, depth), false);
  for(double theta = 0.0; theta <= 0.5 * M_PI + 1e-12; theta += 0.002) {
    SCOPED_TRACE(theta);
    const auto& pair = wheels.Test(CrossedIdlers(theta, depth));
    ASSERT_TRUE(pair.collision);
    const double force = ElasticForce(pair);
    ASSERT_GT(force, 0.0);
    if(theta == 0.0) {
      double length = 0.0;
      for(const auto& c : pair.manifold) {
        EXPECT_NEAR(c.depth, depth, 1e-9);
        length += c.length;
      }
      EXPECT_NEAR(length, kSprocketWidth, 1e-9);
    }
    if(previous > 0.0) {
      EXPECT_NEAR(force / previous, 1.0, 0.03);
    }
    previous = force;
  }
  shift::core::Context context;
  LoadWheels(context, "crossed_idlers", CrossedIdlers(0.5 * M_PI, depth),
             false);
  ASSERT_EQ(context.get_contacts().size(), 1u);
  const auto& pair = *context.get_contacts()[0];
  ASSERT_EQ(pair.manifold.size(), 1u);
  const auto& c = pair.manifold[0];
  EXPECT_EQ(c.length, 0.0);
  EXPECT_EQ(c.weight, 1.0);
  EXPECT_NEAR(c.depth, depth, 1e-9);
  EXPECT_NEAR(c.curvature, 1.0 / kIdlerRadius, 1e-3 / kIdlerRadius);
  EXPECT_NEAR(c.curvature_ratio, 1.0, 1e-3);
}

/* ------------------------------------------------------------------------ */
/* the track on its frame, and links held by their pins */

TEST_F(CadContact, TrackFrameStartsClear) {
  // the benchmark examples/cad/track_frame.json: the loop of 44 links on a
  // frame with five track rollers and a carrier roller, each 0.5 mm off the
  // rails; nothing touches, the links are in the middle, and the top spans
  // sag by less than the chord's drop at the wheels, r (1 - cos(pi/42))
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) + "/cad/track_frame.json");
  context.UpdateD();
  context.DetectContacts();
  for(auto pair : context.get_contacts()) {
    ADD_FAILURE() << pair->a->get_name() << " touches "
                  << pair->b->get_name() << ", deepest " << pair->depth;
  }
  shift::core::force_module::Track* track = nullptr;
  for(auto module : context.get_active_force_modules()) {
    if(auto t = dynamic_cast<shift::core::force_module::Track*>(module)) {
      track = t;
    }
  }
  ASSERT_NE(track, nullptr);
  const auto names = track->get_output_names();
  const auto values = track->Measure();
  ASSERT_EQ(names.size(), 4u);
  ASSERT_EQ(values.size(), 4u);
  EXPECT_EQ(names[0], "front_sag");
  EXPECT_EQ(names[1], "rear_sag");
  const double chord = kPitchRadius * (1.0 - std::cos(M_PI / 42.0));
  for(int i = 0; i < 2; i++) {
    EXPECT_GT(values[i], 0.0) << names[i];
    EXPECT_LT(values[i], chord) << names[i];
  }
  EXPECT_EQ(names[2], "misguide");
  EXPECT_NEAR(values[2], 0.0, 1e-12);
  EXPECT_EQ(names[3], "side_force");
  EXPECT_EQ(values[3], 0.0);
}

TEST_F(CadContact, LinkHangsOnItsPin) {
  // two links without a joint between them: the second's bushing on the
  // first's pin, sunk through the clearance (20 mm bore, 19 mm pin) and
  // 20 um more, under the pin (+x up): the pin touches the bore of the
  // bushing (120 mm) and of the inner plates (35 mm each), 190 mm in all
  const double depth = 20e-6, clearance = 0.001;
  const std::string path = testing::TempDir() + "/link_on_pin.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [)"
         << -kGravity << R"(, 0.0, 0.0],
    "include_directories": [")"
         << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]
  },
  "assemblies": [{"name": "pair",
    "bodies": [)";
    for(int i = 1; i <= 2; i++) {
      file << (i > 1 ? "," : "") << R"(
      {"name": "link)"
           << i << R"(",
       "geometry": [{"type": "cad", "filename": "track_link.step"}],
       "collision_geometry": [{"name": "link)"
           << i << R"(", "type": "cad", "material": "steel"}],
       )" << kLinkInertia
           << "}";
    }
    /* link 1 along y (its x), its pins along z: the pin at (0, kPitch, 0);
    link 2 the same, its bushing there, moved down (-x) */
    file << R"(
    ],
    "joints": [
      {"name": "link1_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["link1"], "origin"],
       "initial_displacement": {"T": [0, 0, 0],
                                "R": [[0, 0, 1], [1, 0, 0], [0, 1, 0]]}},
      {"name": "link2_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["link2"], "origin"],
       "initial_displacement": {"T": [)"
         << -(clearance + depth) << ", " << kPitch << R"(, 0],
                                "R": [[0, 0, 1], [1, 0, 0], [0, 1, 0]]}}
    ],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [{"name": "steel", "youngs_modulus": 2e11,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.3}]}]
  }]
})";
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.UpdateD();
  context.DetectContacts();
  ASSERT_EQ(context.get_contacts().size(), 1u);
  const auto& pair = *context.get_contacts()[0];
  ASSERT_TRUE(pair.point_laws);
  double length = 0.0;
  for(const auto& c : pair.manifold) {
    ASSERT_GT(c.length, 0.0); // a line, no edges or vertices
    EXPECT_NEAR(c.depth, depth, 1e-9);
    length += c.length;
    // the pin pushes the bushing's bore up (+x), from link1 to link2 or
    // back
    EXPECT_NEAR(std::fabs(c.normal.x), 1.0, 1e-9);
  }
  EXPECT_NEAR(length, 0.19, 1e-9);
}
