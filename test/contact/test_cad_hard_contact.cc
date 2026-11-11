/* contact of CAD parts in exceptional cases, on the parts of
scripts/make_hard_contact_step.py: a circle of contact (a ball in a conical
seat, on the rim of a hole), many points on one pair of faces (a plate on
twelve bumps), conformal surfaces (a ball in a socket of 10 um clearance, a nut
on a bolt), saddles on saddles (links of a chain), rolling in a bowl against
its closed-form period, the poles of a sphere's parameters, the trims of a
B-spline face, a thin shell, and a ball in a helical groove */
#include "gtest/gtest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/config.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"
#include "test/contact/cad_scene.h"
#include "test/contact/contact_record.h"

class CadHard : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }
};

/* ball.step: its radius, and the mass and inertia of a steel ball */
static const double kBall = 0.01;
static const double kBallMass = 4.0 / 3.0 * M_PI * kBall * kBall * kBall * 7850.0;
static const double kBallInertia = 0.4 * kBallMass * kBall * kBall;

static const double kIdentity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

/* the CAD shape of a body whose geometry is the STEP file step */
static std::string CadBody(const std::string& step) {
  return R"({"type": "cad", "filename": ")" + step + R"("})";
}

/* the scene of part and the ball */
static shift::test::CadScene BallOn(const std::string& name,
                                    const std::string& part) {
  return shift::test::CadScene(name, part, R"("type": "cad")", "",
                               CadBody("ball.step"));
}

/* the depth s between apart and deep at which the shape first touches, at
pose(s, c, r), by bisection */
template <typename Pose>
static double FirstTouch(shift::test::CadScene& scene, double apart,
                         double deep, Pose pose) {
  for(int i = 0; i < 60; i++) {
    const double s = 0.5 * (apart + deep);
    ksl_vec3_t c;
    double r[3][3];
    pose(s, c, r);
    (scene.At(c, r).collision ? deep : apart) = s;
  }
  return apart;
}

/* the elastic force of each point along its normal, summed, for E* = 1 */
static ksl_vec3_t ForceVector(const shift::core::collision_geometry::ContactPair& pair) {
  ksl_vec3_t f = {{0.0, 0.0, 0.0}};
  for(const auto& c : pair.manifold) {
    double stiffness, exponent;
    shift::core::force_module::Contact::PointLaw(c, 1.0, 1e-4, stiffness,
                                                 exponent);
    const double size = c.weight * stiffness * std::pow(c.depth, exponent);
    for(int k = 0; k < 3; k++) {
      f.at[k] += size * c.normal.at[k];
    }
  }
  return f;
}

/* the product of two rotations, rows */
static void Product(const double a[3][3], const double b[3][3],
                    double out[3][3]) {
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      out[j][k] = 0.0;
      for(int l = 0; l < 3; l++) {
        out[j][k] += a[j][l] * b[l][k];
      }
    }
  }
}

/* a simulation of a fixed part and a body on a float joint */
struct Sim {
  std::string name;
  std::string part;
  std::string body;
  double part_r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double mass = kBallMass;
  double inertia[3] = {kBallInertia, kBallInertia, kBallInertia};
  ksl_vec3_t at = {{0.0, 0.0, 0.0}};
  double r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  /* the joint's initial velocity [v; w], in the body's frame */
  double velocity[6] = {0, 0, 0, 0, 0, 0};
  double friction = 0.3;
  double restitution = 0.3;
  double modulus = 2e9;
  double damping_depth = 1e-6;
  double time_span = 0.1;
};

static std::string Matrix(const double r[3][3]) {
  std::ostringstream m;
  m.precision(17);
  m << "[";
  for(int j = 0; j < 3; j++) {
    m << (j ? ", [" : "[") << r[j][0] << ", " << r[j][1] << ", " << r[j][2]
      << "]";
  }
  m << "]";
  return m.str();
}

/* the model of a simulation, written to a file; returns its path */
static std::string SimModel(const Sim& s) {
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
    << s.time_span << R"(, "time_step": 0.005}]},
    "assemblies": [{"name": "m", "bodies": [
    {"name": "part", "geometry": [{"type": "cad", "filename": ")"
    << s.part << R"("}],
     "collision_geometry": [{"name": "part", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": 10, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                 "Iyz": 0, "Izx": 0}},
    {"name": "body", "geometry": [{"type": "cad", "filename": ")"
    << s.body << R"("}],
     "collision_geometry": [{"name": "body", "type": "cad",
                             "material": "steel"}],
     "center_of_mass": [0, 0, 0],
     "inertia": {"mass": )"
    << s.mass << R"(, "Ixx": )" << s.inertia[0] << R"(, "Iyy": )"
    << s.inertia[1] << R"(, "Izz": )" << s.inertia[2]
    << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
    {"name": "part_joint", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["part"], "origin"],
     "initial_displacement": {"T": [0, 0, 0], "R": )"
    << Matrix(s.part_r) << R"(}},
    {"name": "body_joint", "type": "float",
     "parent": [["fixed"], "origin"], "child": [["body"], "origin"],
     "initial_displacement": {"T": [)"
    << s.at.x << ", " << s.at.y << ", " << s.at.z << R"(], "R": )"
    << Matrix(s.r) << R"(},
     "initial_velocity": [)";
  for(int k = 0; k < 6; k++) {
    m << (k ? ", " : "") << s.velocity[k];
  }
  m << R"(]}],
    "force_modules": [{"name": "contact", "type": "contact",
      "damping_depth": )"
    << s.damping_depth << R"(,
      "materials": [{"name": "steel", "youngs_modulus": )"
    << s.modulus << R"(, "poisson_ratio": 0.3, "restitution": )"
    << s.restitution << R"(, "friction": )" << s.friction << R"(}]}]}]})";
  const std::string path = testing::TempDir() + "/" + s.name + ".json";
  std::ofstream file(path);
  file << m.str();
  return path;
}

/* runs a simulation: the body's final pose, and its kinetic energy then */
static ksl_SE3_t RunSim(const Sim& s, double& kinetic) {
  const std::string path = SimModel(s);
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  context.Simulate();
  shift::test::RecordContacts(context);
  ksl_SE3_t d{};
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "body") {
      d = context.get_body_origin_pose(body);
    }
  }
  kinetic = context.KineticEnergy();
  return d;
}

/* ------------------------------------------------------------------------ */
/* 1. a ball in a conical seat: a circle of contact */

/* seat.step: the cone r + z = 14 mm from the top to z = 8 mm, over a bore of
6 mm. A ball of 10 mm on the axis first touches with its center at z = 14 -
10 sqrt(2) mm, on the circle r = 5 sqrt(2) mm */
static const double kSeatTouch = 0.014 - kBall * std::sqrt(2.0);
static const double kSeatCircle = 0.005 * std::sqrt(2.0);

TEST_F(CadHard, BallInAConicalSeatRestsOnACircle) {
  // the ball on the axis of the seat touches the cone all round a circle: the
  // depth is the same all round, and the points spread round it so that their
  // forces sum along the axis, with nothing across it
  shift::test::CadScene scene = BallOn("ch_seat", "seat.step");
  const double z = FirstTouch(scene, -0.005, 0.002,
                              [](double s, ksl_vec3_t& c, double r[3][3]) {
                                c = {{0.0, 0.0, s}};
                                std::copy(&kIdentity[0][0],
                                          &kIdentity[0][0] + 9, &r[0][0]);
                              });
  EXPECT_NEAR(z, kSeatTouch, 1e-9);
  const double sink = 10e-6;
  const auto& p = scene.At({{0.0, 0.0, z + sink}}, kIdentity);
  ASSERT_TRUE(p.collision);
  EXPECT_GE(p.manifold.size(), 3u);
  // a ring of contact: points of a line, whose lengths make up the circle
  double length = 0.0;
  for(const auto& c : p.manifold) {
    EXPECT_GT(c.length, 0.0);
    EXPECT_NEAR(c.weight, 1.0, 1e-12);
    length += c.length;
  }
  EXPECT_NEAR(length, 2.0 * M_PI * (kSeatCircle - 0.5 * sink),
              1e-6 * kSeatCircle);
  for(const auto& c : p.manifold) {
    // into the cone along its normal, at 45 degrees: the cone's point of
    // the common normal moves in by half the sink
    EXPECT_NEAR(c.depth, sink / std::sqrt(2.0), 1e-8);
    EXPECT_NEAR(std::hypot(c.point_a.x, c.point_a.y), kSeatCircle - 0.5 * sink,
                1e-8);
  }
  const ksl_vec3_t f = ForceVector(p);
  EXPECT_LT(f.z, 0.0);
  EXPECT_LT(std::hypot(f.x, f.y), 1e-6 * std::fabs(f.z));
}

TEST_F(CadHard, BallOffTheAxisOfAConicalSeatHasASmoothForce) {
  // the ball sunk 10 um into the seat and moved off its axis by up to 40 um
  // in 401 poses: the ring lifts on one side and closes to a point on the
  // other, the force with no jump
  shift::test::CadScene scene = BallOn("ch_seat_off", "seat.step");
  const double z = kSeatTouch + 10e-6;
  const int poses = 401;
  double last = -1.0, step = 0.0, peak = 0.0;
  for(int i = 0; i < poses; i++) {
    const double off = 40e-6 * i / (poses - 1);
    const auto& p = scene.At({{off, 0.0, z}}, kIdentity);
    const ksl_vec3_t f = ForceVector(p);
    const double force = ksl_vec3_l2norm(&f);
    peak = std::max(peak, force);
    if(last >= 0.0) {
      step = std::max(step, std::fabs(force - last));
    }
    last = force;
  }
  EXPECT_GT(peak, 0.0);
  EXPECT_LT(step, 1e-2 * peak);
}

TEST_F(CadHard, BallSettlesInAConicalSeat) {
  // dropped on the axis from 5 um above the seat, the ball lands on the
  // ring all round at once, bounces and comes to rest there, centered
  Sim s;
  s.name = "ch_seat_settle";
  s.part = "seat.step";
  s.body = "ball.step";
  s.at = {{0.0, 0.0, kSeatTouch - 5e-6}};
  s.time_span = 0.01;
  double kinetic;
  const ksl_SE3_t d = RunSim(s, kinetic);
  EXPECT_LT(std::hypot(d.t.x, d.t.y), 1e-7);
  EXPECT_NEAR(d.t.z, kSeatTouch, 2e-6);
  EXPECT_LT(kinetic, 1e-2 * kBallMass * 9.81 * 5e-6);
}

/* ------------------------------------------------------------------------ */
/* 2. a plate on twelve bumps: many points on one pair of faces */

/* eggcrate.step: the bumps' tops, x by y */
static const double kBumpX[4] = {0.0, 0.03, 0.06, 0.09};
static const double kBumpY[3] = {0.0, 0.03, 0.06};

/* the plate (slab.step or sheet.step, its underside at z = 0 of its frame)
over the middle of the bumps */
static void Plate(double z, ksl_vec3_t& c, double r[3][3]) {
  c = {{0.045, 0.03, z}};
  std::copy(&kIdentity[0][0], &kIdentity[0][0] + 9, &r[0][0]);
}

/* the plate pressed onto the bumps: a point at the top of each, at one depth */
static void TwelveBumps(const std::string& name, const std::string& plate) {
  shift::test::CadScene scene(name, "eggcrate.step", R"("type": "cad")", "",
                              CadBody(plate));
  const double z = FirstTouch(scene, -0.004, 0.0, Plate);
  const double sink = 5e-6;
  ksl_vec3_t c;
  double r[3][3];
  Plate(z + sink, c, r);
  const auto& p = scene.At(c, r);
  ASSERT_TRUE(p.collision);
  ASSERT_EQ(p.manifold.size(), 12u);
  int found[4][3] = {};
  for(const auto& m : p.manifold) {
    EXPECT_NEAR(m.depth, sink, 1e-9);
    EXPECT_NEAR(m.normal.z, -1.0, 1e-9);
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 3; j++) {
        if(std::hypot(m.point_a.x - kBumpX[i], m.point_a.y - kBumpY[j]) <
           1e-6) {
          found[i][j]++;
        }
      }
    }
  }
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 3; j++) {
      EXPECT_EQ(found[i][j], 1) << "bump " << i << ", " << j;
    }
  }
}

TEST_F(CadHard, PlateOnTwelveBumpsTouchesEach) {
  // the flat underside of a box, a plane face, on the B-spline face of
  // twelve bumps: a point on each
  TwelveBumps("ch_bumps_slab", "slab.step");
}

TEST_F(CadHard, SplineSheetOnTwelveBumpsTouchesEach) {
  // a flat B-spline face on the twelve bumps: a pair of B-spline faces with
  // twelve points, more than the starting points first kept for a pair
  TwelveBumps("ch_bumps_sheet", "sheet.step");
}

/* ------------------------------------------------------------------------ */
/* 3. a ball in a socket of 10 um clearance: conformal surfaces */

static const double kSocket = 0.01001;

TEST_F(CadHard, BallInATightSocketTouchesWhereItIsPushed) {
  // moved off the center of the socket by 2 um more than the clearance, in
  // directions over the socket's lower half, down to its pole where its two
  // faces meet. The surfaces curve alike, 10 and 10.01 mm, so that the gap
  // grows only as 1e-4 / mm across them: the contact wraps round the ball,
  // an area over the cap where the ball is deeper than the socket, as deep
  // as the move beyond the clearance at its middle. Its normals are those of
  // the ball or the socket, back towards the center, its area that of the cap
  // of the ball (by the depth sqrt(r^2 + e^2 + 2 r e cos t) - R of the ball's point at t
  // from the move e), and its resultant is along the move, until the cap
  // runs onto the socket's rim
  shift::test::CadScene scene = BallOn("ch_socket", "socket.step");
  const double sink = 2e-6;
  const double away = kSocket - kBall + sink;
  /* the half angle and area of the cap on the ball */
  const double cap = std::acos((kSocket * kSocket - kBall * kBall - away * away) /
                               (2.0 * kBall * away));
  const double cap_area = 2.0 * M_PI * kBall * kBall * (1.0 - std::cos(cap));
  int count = 0;
  for(int i = 0; i <= 16; i++) {
    const double polar = 1.4 * i / 16.0;
    for(int j = 0; j < (i ? 24 : 1); j++) {
      const double azimuth = 2.0 * M_PI * (j + 0.5 * (i % 2)) / 24.0;
      const ksl_vec3_t d = {{std::sin(polar) * std::cos(azimuth),
                             std::sin(polar) * std::sin(azimuth),
                             std::cos(polar)}};
      const auto& p = scene.At({{away * d.x, away * d.y, away * d.z}},
                               kIdentity);
      SCOPED_TRACE(std::to_string(i) + ", " + std::to_string(j));
      count++;
      ASSERT_FALSE(p.manifold.empty());
      EXPECT_NEAR(p.depth, sink, 1e-10);
      double area = 0.0;
      ksl_vec3_t load = {{0.0, 0.0, 0.0}};
      for(const auto& c : p.manifold) {
        /* on the socket and on the ball: one of them exactly, the other by
        the point's depth, the mean over its part of the area, or the deepest
        of its face, its area reduced to carry it (see AreaContact); the
        normal one of theirs there */
        const ksl_vec3_t on_ball = {{c.point_b.x - away * d.x, c.point_b.y - away * d.y,
                                     c.point_b.z - away * d.z}};
        const double r_socket = ksl_vec3_l2norm(&c.point_a);
        const double r_ball = ksl_vec3_l2norm(&on_ball);
        EXPECT_LT(std::min(std::fabs(r_socket - kSocket), std::fabs(r_ball - kBall)), 1e-12);
        EXPECT_NEAR(r_socket, kSocket, 0.25 * sink);
        EXPECT_NEAR(r_ball, kBall, 0.25 * sink);
        double off_socket = 0.0, off_ball = 0.0;
        for(int k = 0; k < 3; k++) {
          off_socket = std::max(off_socket,
                                std::fabs(c.normal.at[k] + c.point_a.at[k] / r_socket));
          off_ball = std::max(off_ball, std::fabs(c.normal.at[k] + on_ball.at[k] / r_ball));
        }
        EXPECT_LT(std::min(off_socket, off_ball), 1e-6);
        area += c.weight * c.area;
        for(int k = 0; k < 3; k++) {
          load.at[k] += c.weight * c.area * c.depth * c.normal.at[k];
        }
      }
      if(polar + cap > 1.5) {
        continue; // the cap runs onto the rim
      }
      EXPECT_NEAR(area, cap_area, 1e-2 * cap_area);
      const double length = ksl_vec3_l2norm(&load);
      for(int k = 0; k < 3; k++) {
        EXPECT_NEAR(load.at[k] / length, -d.at[k], 1e-3);
      }
    }
  }
  EXPECT_EQ(count, 1 + 16 * 24);
}

TEST_F(CadHard, BallPushedRoundATightSocketHasASmoothForce) {
  // the ball moved round the socket at a constant 2 um beyond the clearance,
  // across its pole and the seams of its halves: the force changes by no
  // more than 0.1% from one pose to the next while the contact, wrapped round
  // the ball, is clear of the socket's rim, and goes smoothly as it runs onto
  // the rim (its second difference is less than 0.1%)
  shift::test::CadScene scene = BallOn("ch_socket_sweep", "socket.step");
  const double away = kSocket - kBall + 2e-6;
  std::vector<double> force, polars;
  for(int i = 0; i <= 400; i++) {
    const double polar = -1.2 + 2.4 * i / 400.0;
    const auto& p = scene.At({{away * std::sin(polar) * std::cos(0.3),
                               away * std::sin(polar) * std::sin(0.3),
                               away * std::cos(polar)}},
                             kIdentity);
    force.push_back(shift::test::ElasticForce(p));
    polars.push_back(polar);
  }
  const double peak = *std::max_element(force.begin(), force.end());
  EXPECT_GT(peak, 0.0);
  double step = 0.0, bend = 0.0;
  for(size_t i = 1; i < force.size(); i++) {
    if(std::fabs(polars[i]) <= 0.9 && std::fabs(polars[i - 1]) <= 0.9) {
      step = std::max(step, std::fabs(force[i] - force[i - 1]));
    }
    if(i >= 2) {
      bend = std::max(bend, std::fabs(force[i] - 2.0 * force[i - 1] + force[i - 2]));
    }
  }
  EXPECT_LT(step, 1e-3 * peak);
  EXPECT_LT(bend, 1e-3 * peak);
}

/* ------------------------------------------------------------------------ */
/* 4. links of a chain: saddle on saddle */

/* torus.step (make_channel_step.py), 40 mm and 10 mm, as the fixed link about
z; the other link in the xz plane, its axis along y, centered on the x axis,
x down its middle towards the fixed link's: the inner faces touch at x = -30 mm
when its center is at x = -60 mm, each at u = 180 degrees in the middle of an
inner face */
static const double kLink[3][3] = {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}};

TEST_F(CadHard, LinkOfAChainHangsOnTheNext) {
  // the inner faces of two linked tori, each a saddle, curved one way along
  // its tube and the other way round its hole: one point, between the links
  shift::test::CadScene scene("ch_links", "torus.step", R"("type": "cad")", "",
                              CadBody("torus.step"));
  const double x = FirstTouch(scene, -0.055, -0.065,
                              [](double s, ksl_vec3_t& c, double r[3][3]) {
                                c = {{s, 0.0, 0.0}};
                                std::copy(&kLink[0][0], &kLink[0][0] + 9,
                                          &r[0][0]);
                              });
  EXPECT_NEAR(x, -0.06, 1e-9);
  const double sink = 20e-6;
  const auto& p = scene.At({{x - sink, 0.0, 0.0}}, kLink);
  ASSERT_TRUE(p.collision);
  ASSERT_EQ(p.manifold.size(), 1u);
  const auto& c = p.manifold[0];
  EXPECT_NEAR(c.depth, sink, 1e-9);
  EXPECT_NEAR(c.normal.x, 1.0, 1e-9);
  EXPECT_NEAR(c.point_a.x, -0.03, 1e-8);
  EXPECT_NEAR(c.point_a.y, 0.0, 1e-8);
  EXPECT_NEAR(c.point_a.z, 0.0, 1e-8);
}

TEST_F(CadHard, TwistedLinkStillTouchesAtOnePoint) {
  // the hanging link twisted about the line between the links by up to 25
  // degrees either way, in 101 poses: the saddles meet at one point and one
  // depth. Twisted by more than 30 degrees the tube of each link also cuts
  // into the other's on both sides of it, and the links touch at three points
  shift::test::CadScene scene("ch_links_twist", "torus.step",
                              R"("type": "cad")", "", CadBody("torus.step"));
  const double sink = 20e-6;
  const double axis[3] = {1.0, 0.0, 0.0};
  const int poses = 101;
  int one = 0;
  for(int i = 0; i < poses; i++) {
    double twist[3][3], r[3][3];
    const double angle = (-25.0 + 50.0 * i / (poses - 1)) * M_PI / 180.0;
    shift::test::Rotation(axis, angle, twist);
    Product(twist, kLink, r);
    const auto& p = scene.At({{-0.06 - sink, 0.0, 0.0}}, r);
    SCOPED_TRACE(i);
    EXPECT_EQ(p.manifold.size(), 1u);
    if(p.manifold.size() == 1u) {
      one++;
      EXPECT_NEAR(p.manifold[0].depth, sink, 1e-9);
      EXPECT_NEAR(p.manifold[0].normal.x, 1.0, 1e-9);
    }
  }
  EXPECT_EQ(one, poses);
}

TEST_F(CadHard, LinkOfAChainSwingsOnTheNext) {
  // the fixed link upright in the xz plane, the other hanging from the bottom
  // of its hole, square to it, and swinging across at 0.2 m/s: it stays
  // linked, hanging, and gains no energy
  const double mass = 2.0 * M_PI * M_PI * 0.04 * 0.01 * 0.01 * 7850.0;
  const double axial = mass * (0.04 * 0.04 + 0.75 * 0.01 * 0.01);
  const double diametral = mass * (0.5 * 0.04 * 0.04 + 0.625 * 0.01 * 0.01);
  Sim s;
  s.name = "ch_links_swing";
  s.part = "torus.step";
  s.body = "torus.step";
  const double upright[3][3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};
  const double hanging[3][3] = {{0, 0, 1}, {0, -1, 0}, {1, 0, 0}};
  std::copy(&upright[0][0], &upright[0][0] + 9, &s.part_r[0][0]);
  std::copy(&hanging[0][0], &hanging[0][0] + 9, &s.r[0][0]);
  s.mass = mass;
  s.inertia[0] = s.inertia[1] = diametral;
  s.inertia[2] = axial;
  s.at = {{0.0, 0.0, 0.06}};
  // 0.2 m/s along global x, in the hanging link's frame: its z
  const double speed = 0.2;
  s.velocity[2] = speed;
  s.time_span = 0.3;
  double kinetic;
  const ksl_SE3_t d = RunSim(s, kinetic);
  const double energy = 0.5 * mass * speed * speed - mass * 9.81 * 0.06;
  EXPECT_LT(kinetic - mass * 9.81 * d.t.z, energy + 1e-3 * 0.5 * mass * speed * speed);
  // linked: its center no further from the fixed link's than when hanging
  EXPECT_LT(ksl_vec3_l2norm(&d.t), 0.06 + 1e-4);
  EXPECT_GT(d.t.z, 0.03);
}

/* ------------------------------------------------------------------------ */
/* 5. a ball rolling in a bowl: the period of its rocking */

/* bowl.step: a sphere of 100 mm radius about (0, 0, -70) mm */
static const double kBowl = 0.1;
static const double kBowlCenter = -0.07;

TEST_F(CadHard, BallRollsInABowlAtItsPeriod) {
  // released from rest 0.1 rad up the bowl, the ball rolls without slipping
  // and rocks with the period 2 pi sqrt(7 (R - r) / (5 g)): after half of it
  // the ball is up the other side, at rest, as far from the bottom
  const double tilt = 0.1, arm = kBowl - kBall;
  Sim s;
  s.name = "ch_bowl";
  s.part = "bowl.step";
  s.body = "ball.step";
  s.at = {{arm * std::sin(tilt), 0.0, kBowlCenter + arm * std::cos(tilt)}};
  const double period = 2.0 * M_PI * std::sqrt(7.0 * arm / (5.0 * 9.81));
  s.time_span = 0.5 * period;
  double kinetic;
  const ksl_SE3_t d = RunSim(s, kinetic);
  const double x0 = arm * std::sin(tilt);
  EXPECT_NEAR(d.t.x, -x0, 0.03 * x0);
  EXPECT_NEAR(d.t.y, 0.0, 1e-3 * x0);
  // the greatest speed, at the bottom, is tilt arm 2 pi / period
  const double fastest = 0.5 * 1.4 * kBallMass *
                         std::pow(tilt * arm * 2.0 * M_PI / period, 2.0);
  EXPECT_LT(kinetic, 0.01 * fastest);
}

/* ------------------------------------------------------------------------ */
/* 6. the poles of a sphere's parameters */

TEST_F(CadHard, BallOnItsPoleOnASplineFace) {
  // ball.step's spherical surface has its poles on z: the ball resting on its
  // pole on a flat B-spline face, and tipped so that the point of contact
  // crosses the pole: one point, at one depth, below the center
  shift::test::CadScene scene = BallOn("ch_pole", "spline_hole.step");
  const double sink = 10e-6;
  const double axis[3] = {1.0, 0.0, 0.0};
  int one = 0;
  for(int i = 0; i <= 200; i++) {
    double r[3][3];
    shift::test::Rotation(axis, -0.2 + 0.4 * i / 200.0, r);
    const auto& p = scene.At({{0.02, 0.02, -kBall + sink}}, r);
    SCOPED_TRACE(i);
    EXPECT_EQ(p.manifold.size(), 1u);
    if(p.manifold.size() == 1u) {
      one++;
      const auto& c = p.manifold[0];
      EXPECT_NEAR(c.depth, sink, 1e-10);
      EXPECT_NEAR(c.normal.z, -1.0, 1e-9);
      EXPECT_NEAR(c.point_a.x, 0.02, 1e-8);
      EXPECT_NEAR(c.point_a.y, 0.02, 1e-8);
    }
  }
  EXPECT_EQ(one, 201);
}

TEST_F(CadHard, BallOnItsPoleOnABump) {
  // the ball's pole on the top of a bump of eggcrate.step, two curved faces:
  // one point, at the top of the bump
  shift::test::CadScene scene = BallOn("ch_pole_bump", "eggcrate.step");
  const double z = FirstTouch(scene, -0.015, -0.009,
                              [](double s, ksl_vec3_t& c, double r[3][3]) {
                                c = {{0.03, 0.03, s}};
                                std::copy(&kIdentity[0][0],
                                          &kIdentity[0][0] + 9, &r[0][0]);
                              });
  const double sink = 5e-6;
  const auto& p = scene.At({{0.03, 0.03, z + sink}}, kIdentity);
  ASSERT_EQ(p.manifold.size(), 1u);
  const auto& c = p.manifold[0];
  EXPECT_NEAR(c.depth, sink, 1e-9);
  EXPECT_NEAR(c.point_a.x, 0.03, 1e-8);
  EXPECT_NEAR(c.point_a.y, 0.03, 1e-8);
  EXPECT_NEAR(c.normal.z, -1.0, 1e-9);
}

/* ------------------------------------------------------------------------ */
/* 7. the trims of a B-spline face */

TEST_F(CadHard, BallOnTheRimOfAHoleInASplineFace) {
  // spline_hole.step: a flat B-spline face trimmed by a hole of 6 mm. The ball
  // over the hole rests on its rim, a circle, 8 mm below its center: the
  // points spread round the rim, at one depth, and their forces sum along
  // the axis
  shift::test::CadScene scene = BallOn("ch_rim", "spline_hole.step");
  const double z = FirstTouch(scene, -0.012, -0.007,
                              [](double s, ksl_vec3_t& c, double r[3][3]) {
                                c = {{0.0, 0.0, s}};
                                std::copy(&kIdentity[0][0],
                                          &kIdentity[0][0] + 9, &r[0][0]);
                              });
  EXPECT_NEAR(z, -0.008, 1e-9);
  const double sink = 10e-6;
  const auto& p = scene.At({{0.0, 0.0, z + sink}}, kIdentity);
  ASSERT_TRUE(p.collision);
  EXPECT_GE(p.manifold.size(), 3u);
  for(const auto& c : p.manifold) {
    EXPECT_NEAR(c.depth, 0.8 * sink, 1e-8);
    EXPECT_NEAR(std::hypot(c.point_a.x, c.point_a.y), 0.006, 1e-6);
    EXPECT_NEAR(c.point_a.z, 0.0, 1e-8);
  }
  const ksl_vec3_t f = ForceVector(p);
  EXPECT_LT(f.z, 0.0);
  EXPECT_LT(std::hypot(f.x, f.y), 1e-6 * std::fabs(f.z));
}

TEST_F(CadHard, BallOverTheTrimOfASplineFaceHasTheDepthOfTheRim) {
  // the ball, 10 um into the face, moved from over the face to over the
  // hole: its depth stays 10 um while its pole is over the face, and over the
  // hole is that of the rim's nearest point, 10 mm less its distance from the
  // center, plus 10 um; no step where the pole crosses the trim
  shift::test::CadScene scene = BallOn("ch_trim", "spline_hole.step");
  const double sink = 10e-6, zc = -kBall + sink;
  double last = -1.0, step = 0.0;
  for(int i = 0; i <= 300; i++) {
    const double x = -0.008 + 0.003 * i / 300.0;
    const auto& p = scene.At({{x, 0.0, zc}}, kIdentity);
    double deepest = 0.0;
    for(const auto& c : p.manifold) {
      deepest = std::max(deepest, c.depth);
    }
    const double rim = std::hypot(0.006 - std::fabs(x), zc);
    const double expected =
      std::fabs(x) >= 0.006 ? sink : std::max(0.0, kBall - rim);
    SCOPED_TRACE(x);
    EXPECT_NEAR(deepest, expected, 1e-9);
    if(last >= 0.0) {
      step = std::max(step, std::fabs(deepest - last));
    }
    last = deepest;
  }
  EXPECT_LT(step, 0.05 * sink);
}

/* ------------------------------------------------------------------------ */
/* 8. a thin shell */

/* shell.step: the crown of its top at about z = -8 mm and its underside 1 mm
below; the ball's center above or below the crown at s */
static void OverCrown(double s, ksl_vec3_t& c, double r[3][3]) {
  c = {{0.0, 0.0, s}};
  std::copy(&kIdentity[0][0], &kIdentity[0][0] + 9, &r[0][0]);
}

TEST_F(CadHard, ThinShellPushesBackOnTheSideTouched) {
  // a ball on the top of the shell is pushed up, one under it down, each by
  // its own face: the shell is 1 mm thick, less than the 3 mm the shell's
  // contact reaches
  shift::test::CadScene scene = BallOn("ch_shell", "shell.step");
  const double above = FirstTouch(scene, -0.03, -0.015, OverCrown);
  const double below = FirstTouch(scene, 0.01, 0.0, OverCrown);
  EXPECT_NEAR(below - above, 0.021, 1e-5);
  const double sink = 20e-6;
  const auto& top = scene.At({{0.0, 0.0, above + sink}}, kIdentity);
  ASSERT_EQ(top.manifold.size(), 1u);
  EXPECT_NEAR(top.manifold[0].depth, sink, 1e-9);
  EXPECT_NEAR(top.manifold[0].normal.z, -1.0, 1e-9);
  const auto& under = scene.At({{0.0, 0.0, below - sink}}, kIdentity);
  ASSERT_EQ(under.manifold.size(), 1u);
  EXPECT_NEAR(under.manifold[0].depth, sink, 1e-9);
  EXPECT_NEAR(under.manifold[0].normal.z, 1.0, 1e-9);
}

TEST_F(CadHard, DeepIntoAThinShellStillPushesBackOut) {
  // the ball pushed into the top of the shell by more than half its 1 mm,
  // and through it: still pushed back up, out of the top it came through, as
  // deep as it went
  shift::test::CadScene scene = BallOn("ch_shell_deep", "shell.step");
  const double above = FirstTouch(scene, -0.03, -0.015, OverCrown);
  for(double sink : {0.0006, 0.0009, 0.0012}) {
    const auto& p = scene.At({{0.0, 0.0, above + sink}}, kIdentity);
    SCOPED_TRACE(sink);
    ASSERT_TRUE(p.collision);
    double deepest = 0.0;
    for(const auto& c : p.manifold) {
      EXPECT_LT(c.normal.z, -0.99);
      deepest = std::max(deepest, c.depth);
    }
    EXPECT_NEAR(deepest, sink, 1e-6);
  }
}

TEST_F(CadHard, BallBouncesOffAThinShell) {
  // dropped on the crown at 3 m/s from 1 mm above it, the ball bounces off:
  // above the shell 10 ms later, moving, with no energy gained
  shift::test::CadScene scene = BallOn("ch_shell_bounce_start", "shell.step");
  const double above = FirstTouch(scene, -0.03, -0.015, OverCrown);
  Sim s;
  s.name = "ch_shell_bounce";
  s.part = "shell.step";
  s.body = "ball.step";
  s.at = {{0.0, 0.0, above - 0.001}};
  const double speed = 3.0;
  s.velocity[2] = speed;
  s.time_span = 0.01;
  double kinetic;
  const ksl_SE3_t d = RunSim(s, kinetic);
  const double energy =
    0.5 * kBallMass * speed * speed - kBallMass * 9.81 * s.at.z;
  EXPECT_LT(d.t.z, above);
  EXPECT_LT(kinetic - kBallMass * 9.81 * d.t.z, energy);
  EXPECT_GT(kinetic, 0.01 * 0.5 * kBallMass * speed * speed);
}

/* ------------------------------------------------------------------------ */
/* 9. a nut on a bolt: conformal helical faces */

/* bolt.step and nut.step: a square thread of 4 mm lead, rising (-z) as it
turns from x to y; the nut 0.05 mm clear of the bolt on either side when their
frames coincide */
static const double kLead = 0.004;
static const double kThreadClearance = 0.00005;

TEST_F(CadHard, NutRestsOnTheThreadAllRound) {
  // the nut 2 um down onto the bolt: its flank lies on the bolt's over three
  // turns, and the points of contact spread round the axis so that the bolt
  // holds the nut up without tipping it
  shift::test::CadScene scene("ch_nut", "bolt.step", R"("type": "cad")", "",
                              CadBody("nut.step"));
  const double sink = 2e-6;
  const auto& p = scene.At({{0.0, 0.0, kThreadClearance + sink}}, kIdentity);
  ASSERT_TRUE(p.collision);
  ASSERT_GE(p.manifold.size(), 3u);
  std::vector<double> angles;
  for(const auto& c : p.manifold) {
    EXPECT_LT(c.normal.z, -0.99);
    EXPECT_GT(c.depth, 0.0);
    angles.push_back(std::atan2(c.point_a.y, c.point_a.x));
  }
  std::sort(angles.begin(), angles.end());
  double gap = angles.front() + 2.0 * M_PI - angles.back();
  for(size_t i = 1; i < angles.size(); i++) {
    gap = std::max(gap, angles[i] - angles[i - 1]);
  }
  EXPECT_LT(gap, 0.75 * M_PI);
  const ksl_vec3_t f = ForceVector(p);
  EXPECT_LT(std::hypot(f.x, f.y), 0.1 * std::fabs(f.z));
}

/* the nut screwing itself down the bolt under its weight, with no friction,
is the example cad/nut_on_bolt.json: each step evaluates the whole helical
line of contact, too slow for a test */

/* ------------------------------------------------------------------------ */
/* 10. a ball in a helical groove */

/* groove.step: the groove of channel.step along the helix (150 cos t, 150 sin
t, 18 t) mm for t from 0 to 1.5 pi; its frame at t: the path's tangent, the
side square to it and level, and the top, up out of the groove */
static void GrooveFrame(double t, ksl_vec3_t& point, ksl_vec3_t& top) {
  const double radius = 0.15, drop = 0.018;
  point = {{radius * std::cos(t), radius * std::sin(t), drop * t}};
  ksl_vec3_t tangent = {{-radius * std::sin(t), radius * std::cos(t), drop}};
  ksl_vec3_normalize(&tangent);
  const ksl_vec3_t up = {{0.0, 0.0, -1.0}};
  ksl_vec3_t side, cross;
  ksl_cross_vv(&up, &tangent, &side);
  ksl_vec3_normalize(&side);
  ksl_cross_vv(&tangent, &side, &cross);
  top = cross;
}

/* the ball over the groove's bottom at t, its center s above it */
static ksl_vec3_t InGroove(double t, double s) {
  ksl_vec3_t point, top;
  GrooveFrame(t, point, top);
  return {{point.x + s * top.x, point.y + s * top.y, point.z + s * top.z}};
}

TEST_F(CadHard, BallInAHelicalGrooveRestsOnBothFlanks) {
  // the groove swept along a helix: the ball rests on both of its flanks, two
  // points of one pair of faces either side of the groove
  shift::test::CadScene scene = BallOn("ch_groove", "groove.step");
  const double t = 0.5 * M_PI;
  const double h = FirstTouch(scene, 0.03, 0.01,
                              [t](double s, ksl_vec3_t& c, double r[3][3]) {
                                c = InGroove(t, s);
                                std::copy(&kIdentity[0][0],
                                          &kIdentity[0][0] + 9, &r[0][0]);
                              });
  EXPECT_NEAR(h, kBall * std::sqrt(2.0), 5e-4);
  const auto& p = scene.At(InGroove(t, h - 10e-6), kIdentity);
  ASSERT_EQ(p.manifold.size(), 2u);
  // either side of the path, along the radius from the helix's axis
  const double sa = std::hypot(p.manifold[0].point_a.x, p.manifold[0].point_a.y) - 0.15;
  const double sb = std::hypot(p.manifold[1].point_a.x, p.manifold[1].point_a.y) - 0.15;
  EXPECT_LT(sa * sb, 0.0);
  EXPECT_GT(p.manifold[0].depth, 0.0);
  EXPECT_GT(p.manifold[1].depth, 0.0);
}

TEST_F(CadHard, BallRollsDownAHelicalGroove) {
  // released at rest in the groove, the ball rolls down the helix on both
  // flanks: it follows the groove round, stays in it, and gains no energy
  shift::test::CadScene scene = BallOn("ch_groove_start", "groove.step");
  const double t0 = 0.3;
  const double h = FirstTouch(scene, 0.03, 0.01,
                              [t0](double s, ksl_vec3_t& c, double r[3][3]) {
                                c = InGroove(t0, s);
                                std::copy(&kIdentity[0][0],
                                          &kIdentity[0][0] + 9, &r[0][0]);
                              });
  Sim s;
  s.name = "ch_groove_roll";
  s.part = "groove.step";
  s.body = "ball.step";
  s.at = InGroove(t0, h);
  s.time_span = 0.12;
  double kinetic;
  const ksl_SE3_t d = RunSim(s, kinetic);
  // rolling down a grade of 18/150 at 5/7 of g times the grade, it goes
  // 6 mm, 0.04 rad round the helix, in 0.12 s
  const double t = std::atan2(d.t.y, d.t.x);
  EXPECT_GT(t, t0 + 0.03);
  // in the groove: as far from the axis as its path, within the groove's
  // half width, and as high as when resting there
  EXPECT_NEAR(std::hypot(d.t.x, d.t.y), 0.15, 0.005);
  const ksl_vec3_t rest = InGroove(t, h);
  EXPECT_NEAR(d.t.z, rest.z, 0.002);
  EXPECT_GT(kinetic, 0.0);
  EXPECT_LT(kinetic - kBallMass * 9.81 * d.t.z,
            -kBallMass * 9.81 * s.at.z + 1e-6);
}

/* ------------------------------------------------------------------------ */
/* the time contact takes: a benchmark of the cases above, not a test. Run it
with `meson test -C build-release --benchmark cad_contact_speed` */

class CadSpeed : public CadHard {};

/* the collision geometry of the part and of the moving CAD shape, with the
seeds of contact from the samples or from the closest-point fields */
static const char* SeedKeys(bool field) {
  return field ? R"(, "contact_seeds": "field")" : "";
}
static std::string SeedShape(bool field) {
  return std::string(R"("type": "cad")") + SeedKeys(field);
}
static std::string SeedName(const char* name, bool field) {
  return std::string(name) + (field ? ", field" : ", samples");
}

/* the mean time of a contact test, in ms, and the mean number of points,
along a sweep of poses, each a small move from the last as from one step of a
simulation to the next: cold, each a new pair, and warm, the pair kept from
one pose to the next. Returns the forces of the cold tests */
template <typename Pose>
static std::vector<ksl_vec3_t> Time(const std::string& name, shift::test::CadScene& scene, int poses,
                 Pose pose) {
  double ms[2] = {0.0, 0.0}, points[2] = {0.0, 0.0};
  std::vector<ksl_vec3_t> forces[2];
  for(int warm = 0; warm < 2; warm++) {
    const auto start = std::chrono::steady_clock::now();
    for(int i = 0; i < poses; i++) {
      ksl_vec3_t c;
      double r[3][3];
      pose(static_cast<double>(i) / (poses - 1), c, r);
      const auto& p = warm ? scene.Follow(c, r) : scene.At(c, r);
      points[warm] += p.manifold.size();
      forces[warm].push_back(ForceVector(p));
    }
    ms[warm] = 1e3 *
               std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - start)
                 .count() /
               poses;
    points[warm] /= poses;
  }
  /* warm and cold find the same contact: the greatest difference of their
  forces, as a share of the greatest force */
  double largest = 0.0, differ = 0.0;
  for(int i = 0; i < poses; i++) {
    const ksl_vec3_t d = {{forces[1][i].x - forces[0][i].x,
                           forces[1][i].y - forces[0][i].y,
                           forces[1][i].z - forces[0][i].z}};
    largest = std::max(largest, ksl_vec3_l2norm(&forces[0][i]));
    differ = std::max(differ, ksl_vec3_l2norm(&d));
  }
  std::printf("%-35s %4d poses  cold %8.3f ms %5.1f points  warm %8.3f ms "
              "%5.1f points  %5.2fx  force differs by %.1e\n",
              name.c_str(), poses, ms[0], points[0], ms[1], points[1], ms[0] / ms[1],
              largest > 0.0 ? differ / largest : differ);
  return forces[0];
}

/* the greatest difference of the forces of the seeds from the fields and from
the samples, as a share of the greatest force, and the time to load the scene
with each */
static void Compare(const std::vector<ksl_vec3_t>& samples,
                    const std::vector<ksl_vec3_t>& field, const double load[2]) {
  double largest = 0.0, differ = 0.0;
  for(size_t i = 0; i < samples.size() && i < field.size(); i++) {
    const ksl_vec3_t d = {{field[i].x - samples[i].x, field[i].y - samples[i].y,
                           field[i].z - samples[i].z}};
    largest = std::max(largest, ksl_vec3_l2norm(&samples[i]));
    differ = std::max(differ, ksl_vec3_l2norm(&d));
  }
  std::printf("%-35s load %8.1f ms with samples, %8.1f ms with fields; forces "
              "differ by %.1e\n",
              "", load[0], load[1], largest > 0.0 ? differ / largest : differ);
}

/* the time to construct a scene, in ms */
static double Since(const std::chrono::steady_clock::time_point& start) {
  return 1e3 * std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - start)
                 .count();
}

static void Upright(double r[3][3]) {
  std::copy(&kIdentity[0][0], &kIdentity[0][0] + 9, &r[0][0]);
}

TEST_F(CadSpeed, SeatRing) {
  // the ball 10 um into the seat moving off its axis by 20 um
  std::vector<ksl_vec3_t> forces[2];
  double load[2];
  for(bool field : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    shift::test::CadScene scene("cs_seat", "seat.step", SeedShape(field),
                                SeedKeys(field), CadBody("ball.step"));
    load[field] = Since(start);
    forces[field] = Time(SeedName("ball in a conical seat", field), scene, 200,
         [](double f, ksl_vec3_t& c, double r[3][3]) {
           c = {{20e-6 * f, 0.0, kSeatTouch + 10e-6}};
           Upright(r);
         });
  }
  Compare(forces[0], forces[1], load);
}

TEST_F(CadSpeed, TwelveBumps) {
  // the B-spline sheet 5 um onto the bumps, sliding 1 mm
  std::vector<ksl_vec3_t> forces[2];
  double load[2];
  for(bool field : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    shift::test::CadScene scene("cs_bumps", "eggcrate.step", SeedShape(field),
                                SeedKeys(field), CadBody("sheet.step"));
    load[field] = Since(start);
    const double z = FirstTouch(scene, -0.004, 0.0, Plate);
    forces[field] = Time(SeedName("sheet on twelve bumps", field), scene, 200,
         [z](double f, ksl_vec3_t& c, double r[3][3]) {
           Plate(z + 5e-6, c, r);
           c.x += 1e-3 * f;
         });
  }
  Compare(forces[0], forces[1], load);
}

TEST_F(CadSpeed, ChainLinks) {
  // the hanging link 20 um into the next, twisting by 10 degrees
  std::vector<ksl_vec3_t> forces[2];
  double load[2];
  for(bool field : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    shift::test::CadScene scene("cs_links", "torus.step", SeedShape(field),
                                SeedKeys(field), CadBody("torus.step"));
    load[field] = Since(start);
    forces[field] = Time(SeedName("chain links", field), scene, 200,
         [](double f, ksl_vec3_t& c, double r[3][3]) {
           const double axis[3] = {1.0, 0.0, 0.0};
           double twist[3][3];
           shift::test::Rotation(axis, 10.0 * M_PI / 180.0 * f, twist);
           Product(twist, kLink, r);
           c = {{-0.06 - 20e-6, 0.0, 0.0}};
         });
  }
  Compare(forces[0], forces[1], load);
}

TEST_F(CadSpeed, BallInAGroove) {
  // the ball 5 um into the helical groove, moving 15 mm along it
  std::vector<ksl_vec3_t> forces[2];
  double load[2];
  for(bool field : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    shift::test::CadScene scene("cs_groove", "groove.step", SeedShape(field),
                                SeedKeys(field), CadBody("ball.step"));
    load[field] = Since(start);
    const double t0 = 0.3;
    const double h = FirstTouch(scene, 0.03, 0.01,
                                [t0](double s, ksl_vec3_t& c, double r[3][3]) {
                                  c = InGroove(t0, s);
                                  Upright(r);
                                });
    forces[field] = Time(SeedName("ball in a helical groove", field), scene, 200,
         [t0, h](double f, ksl_vec3_t& c, double r[3][3]) {
           c = InGroove(t0 + 0.1 * f, h - 5e-6);
           Upright(r);
         });
  }
  Compare(forces[0], forces[1], load);
}

TEST_F(CadSpeed, NutOnABolt) {
  // the nut 2 um down onto the thread, screwing down by a tenth of a turn
  std::vector<ksl_vec3_t> forces[2];
  double load[2];
  for(bool field : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    shift::test::CadScene scene("cs_nut", "bolt.step", SeedShape(field),
                                SeedKeys(field), CadBody("nut.step"));
    load[field] = Since(start);
    forces[field] = Time(SeedName("nut on a bolt", field), scene, 40,
         [](double f, ksl_vec3_t& c, double r[3][3]) {
           const double turn = 0.2 * M_PI * f;
           const double axis[3] = {0.0, 0.0, 1.0};
           shift::test::Rotation(axis, -turn, r);
           c = {{0.0, 0.0,
                 kThreadClearance + 2e-6 + kLead * turn / (2.0 * M_PI)}};
         });
  }
  Compare(forces[0], forces[1], load);
}

TEST_F(CadHard, FieldSeedsFindTheSameContact) {
  // at the same poses, the seeds from the closest-point fields find every point
  // the seeds from the samples do, one for one, with the same depth, normal,
  // weight and length of line: the twisting links (one point), the ball moving
  // in the seat (a ring), the sheet on the twelve bumps (twelve points), the
  // ball in the groove (two) and the nut on the bolt (lines round the thread)
  auto same = [](const char* name, const char* part, const char* body,
                 int poses, auto pose) -> size_t {
    std::vector<std::vector<shift::core::collision_geometry::ContactPoint>>
      found[2];
    for(bool field : {false, true}) {
      shift::test::CadScene scene(name, part, SeedShape(field), SeedKeys(field),
                                  CadBody(body));
      for(int i = 0; i < poses; i++) {
        ksl_vec3_t c;
        double r[3][3];
        pose(static_cast<double>(i) / std::max(poses - 1, 1), c, r);
        found[field].push_back(scene.At(c, r).manifold);
      }
    }
    size_t total = 0;
    for(int i = 0; i < poses; i++) {
      const auto& samples = found[0][i];
      const auto& field = found[1][i];
      EXPECT_EQ(samples.size(), field.size()) << name << " pose " << i;
      if(samples.size() != field.size()) {
        return 0;
      }
      total += samples.size();
      std::vector<bool> used(field.size(), false);
      for(const auto& c : samples) {
        size_t match = field.size();
        for(size_t j = 0; j < field.size(); j++) {
          const ksl_vec3_t d = {{field[j].point_a.x - c.point_a.x,
                                 field[j].point_a.y - c.point_a.y,
                                 field[j].point_a.z - c.point_a.z}};
          if(!used[j] && ksl_vec3_l2norm(&d) < 1e-9) {
            match = j;
            break;
          }
        }
        EXPECT_LT(match, field.size())
          << name << " pose " << i << ": a point of the samples has no match";
        if(match == field.size()) {
          return 0;
        }
        used[match] = true;
        const auto& m = field[match];
        EXPECT_NEAR(m.depth, c.depth, 1e-12) << name << " pose " << i;
        EXPECT_NEAR(m.weight, c.weight, 1e-9) << name << " pose " << i;
        EXPECT_NEAR(m.length, c.length, 1e-9 * std::max(c.length, 1e-3))
          << name << " pose " << i;
        EXPECT_GT(m.normal.x * c.normal.x + m.normal.y * c.normal.y +
                    m.normal.z * c.normal.z,
                  1.0 - 1e-9)
          << name << " pose " << i;
      }
    }
    EXPECT_GT(total, 0u) << name;
    return total;
  };
  EXPECT_EQ(same("ch_field_links", "torus.step", "torus.step", 5,
                 [](double f, ksl_vec3_t& c, double r[3][3]) {
                   const double axis[3] = {1.0, 0.0, 0.0};
                   double twist[3][3];
                   shift::test::Rotation(axis, 10.0 * M_PI / 180.0 * f, twist);
                   Product(twist, kLink, r);
                   c = {{-0.06 - 20e-6, 0.0, 0.0}};
                 }),
            5u);
  EXPECT_GT(same("ch_field_seat", "seat.step", "ball.step", 5,
                 [](double f, ksl_vec3_t& c, double r[3][3]) {
                   c = {{20e-6 * f, 0.0, kSeatTouch + 10e-6}};
                   Upright(r);
                 }),
            5u * 12u);
  // the heights of first touch, found once, so that both meet the same poses
  double z, h;
  {
    shift::test::CadScene scene("ch_field_touch", "eggcrate.step",
                                R"("type": "cad")", "", CadBody("sheet.step"));
    z = FirstTouch(scene, -0.004, 0.0, Plate);
  }
  {
    shift::test::CadScene scene = BallOn("ch_field_touch_groove", "groove.step");
    h = FirstTouch(scene, 0.03, 0.01,
                   [](double s, ksl_vec3_t& c, double r[3][3]) {
                     c = InGroove(0.3, s);
                     Upright(r);
                   });
  }
  EXPECT_EQ(same("ch_field_bumps", "eggcrate.step", "sheet.step", 3,
                 [z](double f, ksl_vec3_t& c, double r[3][3]) {
                   Plate(z + 5e-6, c, r);
                   c.x += 1e-3 * f;
                 }),
            3u * 12u);
  EXPECT_EQ(same("ch_field_groove", "groove.step", "ball.step", 3,
                 [h](double f, ksl_vec3_t& c, double r[3][3]) {
                   c = InGroove(0.3 + 0.1 * f, h - 5e-6);
                   Upright(r);
                 }),
            3u * 2u);
  EXPECT_GT(same("ch_field_nut", "bolt.step", "nut.step", 2,
                 [](double f, ksl_vec3_t& c, double r[3][3]) {
                   const double turn = 0.2 * M_PI * f;
                   const double axis[3] = {0.0, 0.0, 1.0};
                   shift::test::Rotation(axis, -turn, r);
                   c = {{0.0, 0.0,
                         kThreadClearance + 2e-6 + kLead * turn / (2.0 * M_PI)}};
                 }),
            2u * 100u);
}
