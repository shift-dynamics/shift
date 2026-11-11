/* shift::core::collision_geometry::Cylinder, capsule and box collision geometry: their support functions and
fits, the line contact of a cylinder or a capsule on a plane (against a
numerical integration of Hertz line contact over its stations), and the
rest depth of a capsule, a cylinder and a box on the ground */
#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <vector>

#include "test/contact/contact_record.h"
#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/box.h"
#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/collision_geometry/manifold.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/context.h"


static ksl_vec3_t V(double x, double y, double z) {
  return {{x, y, z}};
}

static double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* rotation about an axis (normalized here); R.v[i] is column i */
static ksl_SE3_t Pose(const ksl_vec3_t& t, const ksl_vec3_t& axis,
                      double angle) {
  ksl_SE3_t d;
  const double c = std::cos(angle), s = std::sin(angle);
  const double norm = std::sqrt(Dot(axis, axis));
  const double k[3] = {axis.x / norm, axis.y / norm, axis.z / norm};
  for(int j = 0; j < 3; j++) {
    for(int i = 0; i < 3; i++) {
      double r = (1.0 - c) * k[i] * k[j] + (i == j ? c : 0.0);
      if(i == 0 && j == 1) r -= s * k[2];
      if(i == 0 && j == 2) r += s * k[1];
      if(i == 1 && j == 0) r += s * k[2];
      if(i == 1 && j == 2) r -= s * k[0];
      if(i == 2 && j == 0) r -= s * k[1];
      if(i == 2 && j == 1) r += s * k[0];
      d.R.v[j].at[i] = r;
    }
  }
  d.t = t;
  return d;
}


static const int kRing = 64;

static ksl_vec3_t Axis(const shift::core::collision_geometry::Base& g) {
  return g.get_d_absolute().R.v[2];
}

/* the rim of a cylinder (or the segment ends of a capsule, radius 0) of radius
r and half length h along z, as points that fit it exactly */
static std::vector<ksl_vec3_t> CylinderPoints(double r, double h) {
  std::vector<ksl_vec3_t> points;
  for(int k = 0; k < kRing; k++) {
    const double a = 2.0 * M_PI * k / kRing;
    for(double z : {-h, h}) {
      points.push_back(V(r * std::cos(a), r * std::sin(a), z));
    }
  }
  return points;
}

template <class T>
static std::unique_ptr<T> Fitted(const std::vector<ksl_vec3_t>& points) {
  auto g = std::make_unique<T>();
  g->FitToPoints(points);
  g->UpdatePose(Pose(V(0, 0, 0), V(1, 0, 0), 0.0));
  shift::test::RecordShape(*g);
  return g;
}

/* the point of a dense sampling of a shape farthest along a direction */
static double SampledSupport(const std::vector<ksl_vec3_t>& samples,
                             const ksl_vec3_t& d) {
  double best = -HUGE_VAL;
  for(const auto& p : samples) {
    best = std::max(best, Dot(p, d));
  }
  return best;
}

TEST(ContactPrimitives, CylinderFitsItsPoints) {
  auto cylinder = Fitted<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.1, 0.5));
  EXPECT_NEAR(cylinder->get_radius(), 0.1, 1e-12);
  EXPECT_NEAR(cylinder->get_half_length(), 0.5, 1e-12);
  const ksl_vec3_t axis = cylinder->get_d_wrt_body().R.v[2];
  EXPECT_NEAR(std::fabs(axis.z), 1.0, 1e-9);
}

TEST(ContactPrimitives, CapsuleEnclosesItsPoints) {
  /* the same points: the capsule has the extent of the cylinder, and reaches
  the corners of its ends with its caps */
  auto capsule = Fitted<shift::core::collision_geometry::Capsule>(CylinderPoints(0.1, 0.5));
  const double h = capsule->get_half_length(), r = capsule->get_radius();
  EXPECT_NEAR(h + r, 0.5 + 0.0, 0.5);  // an end cap reaches at least z = 0.5
  EXPECT_GE(h + r, 0.5 - 1e-12);
  const ksl_SE3_t pose = Pose(V(0, 0, 0), V(1, 0, 0), 0.0);
  capsule->UpdatePose(pose);
  for(const auto& p : CylinderPoints(0.1, 0.5)) {
    /* the distance of the point to the segment is at most the radius */
    ksl_vec3_t local;
    for(int i = 0; i < 3; i++) {
      local.at[i] = Dot(capsule->get_d_wrt_body().R.v[i], p) -
                    Dot(capsule->get_d_wrt_body().R.v[i],
                        capsule->get_d_wrt_body().t);
    }
    const double beyond = std::max(std::fabs(local.z) - h, 0.0);
    EXPECT_LE(std::sqrt(local.x * local.x + local.y * local.y +
                        beyond * beyond),
              r + 1e-12);
  }
}

TEST(ContactPrimitives, BoxFitsAnOrientedBoxOfPoints) {
  std::vector<ksl_vec3_t> points;
  const ksl_SE3_t turned = Pose(V(0.3, -0.2, 0.1), V(0, 0, 1), 0.6);
  for(int i = 0; i < 8; i++) {
    const ksl_vec3_t c = V((i & 1) ? 0.4 : -0.4, (i & 2) ? 0.2 : -0.2,
                           (i & 4) ? 0.1 : -0.1);
    ksl_vec3_t w;
    ksl_product_dv(&turned, &c, &w);
    points.push_back(w);
  }
  auto box = Fitted<shift::core::collision_geometry::Box>(points);
  std::vector<double> e = {box->get_half_extents().x, box->get_half_extents().y,
                           box->get_half_extents().z};
  std::sort(e.begin(), e.end());
  EXPECT_NEAR(e[0], 0.1, 1e-9);
  EXPECT_NEAR(e[1], 0.2, 1e-9);
  EXPECT_NEAR(e[2], 0.4, 1e-9);
  EXPECT_NEAR(box->get_d_wrt_body().t.x, 0.3, 1e-9);
  EXPECT_NEAR(box->get_d_wrt_body().t.y, -0.2, 1e-9);
  EXPECT_NEAR(box->get_d_wrt_body().t.z, 0.1, 1e-9);
}


/* the support function value (the greatest projection onto d) of a cylinder
or a capsule of radius r and half length h with the axis a and center c:
cylinder  c.d + h |a.d| + r sqrt(|d|^2 - (a.d)^2)
capsule   c.d + h |a.d| + r |d| */
static double SupportValue(bool capsule, double r, double h,
                           const ksl_vec3_t& c, const ksl_vec3_t& a,
                           const ksl_vec3_t& d) {
  const double ad = Dot(a, d);
  const double radial = capsule ? std::sqrt(Dot(d, d))
                                : std::sqrt(std::max(Dot(d, d) - ad * ad, 0.0));
  return Dot(c, d) + h * std::fabs(ad) + r * radial;
}

TEST(ContactPrimitives, SupportPointsAreExtreme) {
  auto cylinder = Fitted<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.1, 0.5));
  auto capsule = Fitted<shift::core::collision_geometry::Capsule>(CylinderPoints(0.1, 0.5));
  const ksl_SE3_t pose = Pose(V(0.2, 0.1, -0.3), V(1, 2, 3), 0.7);
  cylinder->UpdatePose(pose);
  capsule->UpdatePose(pose);
  std::mt19937 generator(12345);
  std::normal_distribution<double> normal;
  for(int i = 0; i < 50; i++) {
    const ksl_vec3_t d = V(normal(generator), normal(generator),
                           normal(generator));
    for(bool is_capsule : {false, true}) {
      const shift::core::collision_geometry::Base& g = is_capsule ? static_cast<const shift::core::collision_geometry::Base&>(*capsule)
                                 : static_cast<const shift::core::collision_geometry::Base&>(*cylinder);
      const double r = is_capsule ? capsule->get_radius()
                                  : cylinder->get_radius();
      const double h = is_capsule ? capsule->get_half_length()
                                  : cylinder->get_half_length();
      ksl_vec3_t local, global;
      int hint = 0;
      g.Support(d, local, global, hint);
      const double expected =
        SupportValue(is_capsule, r, h, g.get_d_absolute().t, Axis(g), d);
      EXPECT_NEAR(Dot(global, d), expected, 1e-12);
      /* and the point is on the surface: within the shape's reach */
      ksl_vec3_t from_center = V(global.x - g.get_d_absolute().t.x,
                                 global.y - g.get_d_absolute().t.y,
                                 global.z - g.get_d_absolute().t.z);
      const double along = Dot(from_center, Axis(g));
      EXPECT_LE(std::fabs(along), h + (is_capsule ? r : 0.0) + 1e-12);
    }
  }
}

/* Hertz line contact against a plane by summing stations: the force and the
moment about the shape's center of F = k d ds over the stations of positive
depth, with the surface's lowest point at each station */
struct LineSum {
  double force = 0.0;
  ksl_vec3_t moment = {{0, 0, 0}};
  double length = 0.0;
};

static LineSum SumStations(bool capsule, double r, double h,
                           const ksl_vec3_t& c, const ksl_vec3_t& a,
                           const ksl_vec3_t& n, double level) {
  LineSum sum;
  const int stations = 200000;
  const double g = Dot(n, a);
  const double across = std::sqrt(std::max(1.0 - g * g, 0.0));
  /* the lowest point of the surface at a station */
  ksl_vec3_t down = V(-n.x, -n.y, -n.z);
  if(!capsule) {
    down = V(-(n.x - g * a.x) / across, -(n.y - g * a.y) / across,
             -(n.z - g * a.z) / across);
  }
  const double ds = 2.0 * h / stations;
  for(int i = 0; i < stations; i++) {
    const double s = -h + (i + 0.5) * ds;
    const ksl_vec3_t p = V(c.x + s * a.x + r * down.x, c.y + s * a.y + r * down.y,
                           c.z + s * a.z + r * down.z);
    const double depth = level - Dot(n, p);
    if(depth <= 0.0) {
      continue;
    }
    sum.force += depth * ds;
    sum.length += ds;
    /* moment about c of the force depth * ds along n at p */
    const ksl_vec3_t arm = V(p.x - c.x, p.y - c.y, p.z - c.z);
    sum.moment.x += (arm.y * n.z - arm.z * n.y) * depth * ds;
    sum.moment.y += (arm.z * n.x - arm.x * n.z) * depth * ds;
    sum.moment.z += (arm.x * n.y - arm.y * n.x) * depth * ds;
  }
  return sum;
}

/* the line contact of a shape on a plane against the sum of stations: the
manifold's point of length L and mean depth d carries L d (per unit of the
line law), at a point whose moment about the center matches */
static void CheckLine(bool is_capsule, const ksl_vec3_t& plane_normal,
                      double level, const ksl_SE3_t& pose) {
  auto plane = std::make_unique<shift::core::collision_geometry::Plane>();
  plane->SetPlane(plane_normal, level);
  plane->UpdatePose(Pose(V(0, 0, 0), V(1, 0, 0), 0.0));
  std::unique_ptr<shift::core::collision_geometry::Base> shape;
  double r, h;
  if(is_capsule) {
    auto capsule = Fitted<shift::core::collision_geometry::Capsule>(CylinderPoints(0.1, 0.5));
    r = capsule->get_radius();
    h = capsule->get_half_length();
    shape = std::move(capsule);
  } else {
    auto cylinder = Fitted<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.1, 0.5));
    r = cylinder->get_radius();
    h = cylinder->get_half_length();
    shape = std::move(cylinder);
  }
  shape->UpdatePose(pose);
  shift::core::collision_geometry::ContactPair pair;
  pair.a = plane.get();
  pair.b = shape.get();
  shift::test::TestContact(pair);
  const ksl_vec3_t c = shape->get_d_absolute().t;
  const ksl_vec3_t a = Axis(*shape);
  const ksl_vec3_t n = plane->get_normal();
  const double offset = Dot(n, plane->get_point());
  const LineSum sum = SumStations(is_capsule, r, h, c, a, n, offset);
  ASSERT_TRUE(pair.collision);
  ASSERT_TRUE(pair.point_laws);
  ASSERT_EQ(pair.manifold.size(), 1u);
  const auto& point = pair.manifold[0];
  EXPECT_NEAR(point.length, sum.length, 1e-5);
  EXPECT_NEAR(point.length * point.depth, sum.force, 1e-5 * sum.force);
  /* the normal of the pair is from the plane to the shape */
  EXPECT_NEAR(Dot(point.normal, n), 1.0, 1e-12);
  /* moment about the center of the force L d along n at the shape's point */
  const ksl_vec3_t arm = V(point.point_b.x - c.x, point.point_b.y - c.y,
                           point.point_b.z - c.z);
  const double f = point.length * point.depth;
  const ksl_vec3_t m = V((arm.y * n.z - arm.z * n.y) * f,
                         (arm.z * n.x - arm.x * n.z) * f,
                         (arm.x * n.y - arm.y * n.x) * f);
  EXPECT_NEAR(m.x, sum.moment.x, 1e-5 * (std::fabs(sum.moment.x) + 1e-3));
  EXPECT_NEAR(m.y, sum.moment.y, 1e-5 * (std::fabs(sum.moment.y) + 1e-3));
  EXPECT_NEAR(m.z, sum.moment.z, 1e-5 * (std::fabs(sum.moment.z) + 1e-3));
  /* the plane's point is the shape's point moved by the depth along n */
  EXPECT_NEAR(Dot(point.point_a, n) - Dot(point.point_b, n), point.depth, 1e-12);
}

TEST(ContactPrimitives, LineContactMatchesIntegration) {
  /* the plane z = 0 with its normal out of the solid along -z (up) */
  const ksl_vec3_t up = V(0, 0, -1);
  for(bool is_capsule : {false, true}) {
    /* lying flat, 2 mm deep: the whole length, the mean depth r - height */
    CheckLine(is_capsule, up, 0.0,
              Pose(V(0.1, 0.2, -0.098), V(0, 1, 0), M_PI / 2));
    /* tilted, partly in the plane */
    CheckLine(is_capsule, up, 0.0,
              Pose(V(0.0, 0.0, -0.25), V(0, 1, 0), M_PI / 2 - 0.5));
    /* nearly upright, only the lower end in */
    CheckLine(is_capsule, up, 0.0,
              Pose(V(0.0, 0.0, -0.45), V(1, 0, 0), 0.2));
    /* a tilted plane, the shape in a general pose */
    const ksl_vec3_t tilted = V(0.0, 0.6, -0.8);
    CheckLine(is_capsule, tilted, 0.0,
              Pose(V(0.1, -0.3, -0.05), V(1, 2, 0.5), 1.1));
  }
}

TEST(ContactPrimitives, LineContactIsContinuousAsTheShapeTilts) {
  /* the contact force rolls off smoothly: a capsule lowered through the
  plane at several angles has a force that never jumps by more than a small
  step between nearby poses */
  const ksl_vec3_t up = V(0, 0, -1);
  auto plane = std::make_unique<shift::core::collision_geometry::Plane>();
  plane->SetPlane(up, 0.0);
  plane->UpdatePose(Pose(V(0, 0, 0), V(1, 0, 0), 0.0));
  for(bool is_capsule : {false, true}) {
    std::unique_ptr<shift::core::collision_geometry::Base> shape;
    if(is_capsule) {
      shape = Fitted<shift::core::collision_geometry::Capsule>(CylinderPoints(0.1, 0.5));
    } else {
      shape = Fitted<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.1, 0.5));
    }
    double previous = 0.0;
    for(int i = 0; i <= 2000; i++) {
      const double angle = 0.4 * i / 2000;
      shape->UpdatePose(Pose(V(0, 0, -0.095), V(0, 1, 0), M_PI / 2 - angle));
      shift::core::collision_geometry::ContactPair pair;
      pair.a = plane.get();
      pair.b = shape.get();
      shift::test::TestContact(pair);
      double force = 0.0;
      if(pair.collision && !pair.manifold.empty()) {
        force = pair.manifold[0].length * pair.manifold[0].depth;
      }
      if(i > 0) {
        EXPECT_LT(std::fabs(force - previous), 2e-4) << "angle " << angle;
      }
      previous = force;
    }
  }
}

/* a cylinder or capsule of radius 0.1 and half length 0.5 */
static std::unique_ptr<shift::core::collision_geometry::Base> LineShape(
  bool is_capsule) {
  if(is_capsule) {
    return Fitted<shift::core::collision_geometry::Capsule>(CylinderPoints(0.1, 0.5));
  }
  return Fitted<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.1, 0.5));
}

/* the radius and half length of a LineShape */
static void LineSize(const shift::core::collision_geometry::Base& shape,
                     double& r, double& h) {
  if(auto capsule = dynamic_cast<const shift::core::collision_geometry::Capsule*>(&shape)) {
    r = capsule->get_radius();
    h = capsule->get_half_length();
  } else {
    auto& cylinder = dynamic_cast<const shift::core::collision_geometry::Cylinder&>(shape);
    r = cylinder.get_radius();
    h = cylinder.get_half_length();
  }
}

/* a box 2 x 2 x 1 whose top face is z = 0 (z down, the face's outward normal
-z), its edges at x = +-1 and y = +-1 */
static std::unique_ptr<shift::core::collision_geometry::Box> TableBox() {
  std::vector<ksl_vec3_t> corners;
  for(int i = 0; i < 8; i++) {
    corners.push_back(V((i & 1) ? 1.0 : -1.0, (i & 2) ? 1.0 : -1.0,
                        (i & 4) ? 0.5 : -0.5));
  }
  auto box = Fitted<shift::core::collision_geometry::Box>(corners);
  box->UpdatePose(Pose(V(0, 0, 0.5), V(1, 0, 0), 0.0));
  shift::test::RecordShape(*box);
  return box;
}

static shift::core::collision_geometry::ContactPair Touch(
  shift::core::collision_geometry::Base& a,
  shift::core::collision_geometry::Base& b) {
  shift::core::collision_geometry::ContactPair pair;
  pair.a = &a;
  pair.b = &b;
  shift::test::TestContact(pair);
  return pair;
}

/* the line law's load L d summed over the points of a manifold */
static double LineLoad(const shift::core::collision_geometry::ContactPair& pair) {
  double load = 0.0;
  if(pair.collision && pair.point_laws) {
    for(const auto& c : pair.manifold) {
      load += c.weight * c.length * c.depth;
    }
  }
  return load;
}

TEST(ContactPrimitives, LineOnABoxFaceIsTheLineOnAPlane) {
  /* a cylinder or a capsule on the top face of a box, clear of its edges,
  touches as it would the plane of the face: the same point, length and
  depth, with the box as either geometry of the pair */
  auto box = TableBox();
  auto plane = std::make_unique<shift::core::collision_geometry::Plane>();
  plane->SetPlane(V(0, 0, -1), 0.0);
  plane->UpdatePose(Pose(V(0, 0, 0), V(1, 0, 0), 0.0));
  for(bool is_capsule : {false, true}) {
    auto shape = LineShape(is_capsule);
    for(const ksl_SE3_t& pose :
        {Pose(V(0.1, 0.2, -0.098), V(0, 1, 0), M_PI / 2),
         Pose(V(0.0, 0.0, -0.25), V(0, 1, 0), M_PI / 2 - 0.5),
         Pose(V(-0.2, 0.3, -0.12), V(0.3, 1, 0), M_PI / 2 - 0.1)}) {
      shape->UpdatePose(pose);
      const auto on_plane = Touch(*plane, *shape);
      ASSERT_TRUE(on_plane.collision);
      ASSERT_EQ(on_plane.manifold.size(), 1u);
      const auto& expected = on_plane.manifold[0];
      for(bool box_first : {true, false}) {
        const auto pair = box_first ? Touch(*box, *shape) : Touch(*shape, *box);
        ASSERT_TRUE(pair.collision);
        ASSERT_TRUE(pair.point_laws);
        ASSERT_EQ(pair.manifold.size(), 1u);
        const auto& point = pair.manifold[0];
        EXPECT_NEAR(point.length, expected.length, 1e-9);
        EXPECT_NEAR(point.depth, expected.depth, 1e-9);
        const ksl_vec3_t& on_shape = box_first ? point.point_b : point.point_a;
        const ksl_vec3_t& on_box = box_first ? point.point_a : point.point_b;
        for(int k = 0; k < 3; k++) {
          EXPECT_NEAR(on_shape.at[k], expected.point_b.at[k], 1e-9);
          EXPECT_NEAR(on_box.at[k], expected.point_a.at[k], 1e-9);
          /* the normal is from a to b */
          EXPECT_NEAR(point.normal.at[k],
                      (box_first ? 1.0 : -1.0) * expected.normal.at[k], 1e-9);
        }
      }
    }
  }
}

TEST(ContactPrimitives, LineOverTheEdgeOfABoxIsCut) {
  /* lying flat along x, 2 mm deep, its center on the edge x = 1: the half
  over the face touches, a line of length h centered at x = 1 - h / 2 */
  auto box = TableBox();
  for(bool is_capsule : {false, true}) {
    auto shape = LineShape(is_capsule);
    double r, h;
    LineSize(*shape, r, h);
    shape->UpdatePose(Pose(V(1.0, 0.3, -(r - 0.002)), V(0, 1, 0), M_PI / 2));
    const auto pair = Touch(*box, *shape);
    ASSERT_TRUE(pair.collision);
    ASSERT_TRUE(pair.point_laws);
    ASSERT_EQ(pair.manifold.size(), 1u);
    const auto& point = pair.manifold[0];
    EXPECT_NEAR(point.length, h, 1e-9);
    EXPECT_NEAR(point.depth, 0.002, 1e-9);
    EXPECT_NEAR(point.point_b.x, 1.0 - 0.5 * h, 1e-9);
    EXPECT_NEAR(point.point_b.z, 0.002, 1e-9);
    EXPECT_NEAR(point.point_a.z, 0.0, 1e-9);
  }
}

TEST(ContactPrimitives, LineRollingOverTheEdgeOfABoxHasNoJump) {
  /* a cylinder or capsule parallel to the edge x = 1, 2 mm deep, slides to
  the edge and rolls around it: the load L d never jumps, and around the
  edge it is the full length at the depth of the edge in the shape */
  auto box = TableBox();
  const double depth = 0.002;
  for(bool is_capsule : {false, true}) {
    auto shape = LineShape(is_capsule);
    double r, h;
    LineSize(*shape, r, h);
    double previous = -1.0;
    const int steps = 2000;
    for(int i = 0; i <= steps; i++) {
      const double u = 2.0 * i / steps - 1.0; // -1 to 1
      ksl_vec3_t center;
      if(u < 0.0) {
        center = V(1.0 + 0.2 * u, 0.0, -(r - depth));
      } else {
        const double phi = 1.2 * u; // around the edge, to about 69 degrees
        center = V(1.0 + (r - depth) * std::sin(phi), 0.0,
                   -(r - depth) * std::cos(phi));
      }
      shape->UpdatePose(Pose(center, V(1, 0, 0), M_PI / 2)); // axis along y
      const auto pair = Touch(*box, *shape);
      ASSERT_TRUE(pair.collision) << "u " << u;
      const double load = LineLoad(pair);
      EXPECT_NEAR(load, 2.0 * h * depth, 1e-11) << "u " << u;
      if(previous >= 0.0) {
        EXPECT_LT(std::fabs(load - previous), 1e-11) << "u " << u;
      }
      previous = load;
    }
  }
}

TEST(ContactPrimitives, LineTiltingOffTheEndOfABoxHasNoJump) {
  /* lying along x across the edge x = 1 and tilting about y, the end over
  the face rising out: the load falls to zero without a jump */
  auto box = TableBox();
  for(bool is_capsule : {false, true}) {
    auto shape = LineShape(is_capsule);
    double r, h;
    LineSize(*shape, r, h);
    double previous = -1.0;
    for(int i = 0; i <= 2000; i++) {
      const double tilt = 0.05 * i / 2000;
      shape->UpdatePose(
        Pose(V(1.1, 0.0, -(r - 0.002)), V(0, 1, 0), M_PI / 2 - tilt));
      const auto pair = Touch(*box, *shape);
      const double load = LineLoad(pair);
      if(previous >= 0.0) {
        /* it falls by about 3e-6 a step: a jump would be of the order of
        the whole load, 1e-3 */
        EXPECT_LT(std::fabs(load - previous), 1e-5) << "tilt " << tilt;
      }
      previous = load;
    }
  }
}

/* the ground of RestDepth: a plane, or a box 2 x 2 x 1 whose top face is the
plane z = 0 */
static const char* kPlaneGround = R"({"name": "ground", "type": "plane",
                               "material": "ground"})";
static const char* kBoxGround = R"({"name": "ground", "type": "box",
  "half_extents": [1.0, 1.0, 0.5], "pose": {"T": [0.0, 0.0, 0.5]},
  "material": "ground"})";

/* a body on a float joint above a ground (z down), simulated to rest */
static double RestDepth(const std::string& shape, double offset, double mass,
                        const std::string& rotation = "",
                        const std::string& ground = kPlaneGround) {
  const std::string path = testing::TempDir() + "/primitive_rest.json";
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-8,
                     "absolute_error": 1e-10, "time_span": 1.5,
                     "time_step": 0.001}]
  },
  "assemblies": [{"name": "rest",
    "bodies": [
      {"name": "ground", "geometry": [],
       "collision_geometry": [)" << ground << R"(],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}},
      {"name": "block", "geometry": [], "collision_geometry": [)"
        << shape << R"(],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": )" << mass << R"(, "Ixx": 0.01, "Iyy": 0.01,
                   "Izz": 0.01, "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}
    ],
    "joints": [
      {"name": "ground_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["ground"], "origin"]},
      {"name": "block_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["block"], "origin"],
       "initial_displacement": {"T": [0, 0, )" << -(offset + 0.002) << "]"
        << rotation << R"(}}
    ],
    "force_modules": [{"name": "contact", "type": "contact",
      "materials": [
        {"name": "ground", "youngs_modulus": 1e7, "poisson_ratio": 0.3,
         "restitution": 0.2, "friction": 0.5, "contact_stiffness": 1e5},
        {"name": "block", "youngs_modulus": 1e7, "poisson_ratio": 0.3,
         "restitution": 0.2, "friction": 0.5, "contact_stiffness": 1e5}
      ]}]
  }]
})";
  {
    std::ofstream file(path);
    file << model.str();
  }
  shift::core::Context context;
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  context.Simulate();
  shift::test::RecordContacts(context);
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "block") {
      return -context.get_body_origin_pose(body).t.z - offset;
    }
  }
  return 0.0;
}

static const double kModulus = 1.0 / (2.0 * (1.0 - 0.09) / 1e7);

TEST(ContactPrimitives, CapsuleLyingOnTheGroundRestsOnALine) {
  /* m g = pi/4 E* L d, with L the length of the segment: the cylinder part
  of the capsule is flat on the ground. The axis is turned to lie along x:
  the pose rotates z to x. */
  const double h = 0.3, r = 0.05;
  const double depth = 10.0 * 9.81 / (M_PI / 4.0 * kModulus * 2.0 * h);
  const double height = RestDepth(
    R"({"name": "capsule", "type": "capsule", "radius": 0.05,
        "half_length": 0.3, "material": "block",
        "pose": {"R": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]}})",
    r, 10.0);
  EXPECT_NEAR(-height, depth, 5e-3 * depth);
}

TEST(ContactPrimitives, CylinderLyingOnTheGroundRestsOnALine) {
  const double h = 0.2, r = 0.04;
  const double depth = 2.0 * 9.81 / (M_PI / 4.0 * kModulus * 2.0 * h);
  const double height = RestDepth(
    R"({"name": "cylinder", "type": "cylinder", "radius": 0.04,
        "half_length": 0.2, "material": "block",
        "pose": {"R": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]}})",
    r, 2.0);
  EXPECT_NEAR(-height, depth, 5e-3 * depth);
}

TEST(ContactPrimitives, LinesLyingOnABoxRestAsOnTheGround) {
  /* a capsule and a cylinder lying on the top face of a box settle as they
  do on a plane: m g = pi/4 E* L d over the length of the line */
  const double capsule_depth =
    10.0 * 9.81 / (M_PI / 4.0 * kModulus * 2.0 * 0.3);
  const double capsule = RestDepth(
    R"({"name": "capsule", "type": "capsule", "radius": 0.05,
        "half_length": 0.3, "material": "block",
        "pose": {"R": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]}})",
    0.05, 10.0, "", kBoxGround);
  EXPECT_NEAR(-capsule, capsule_depth, 5e-3 * capsule_depth);
  const double cylinder_depth =
    2.0 * 9.81 / (M_PI / 4.0 * kModulus * 2.0 * 0.2);
  const double cylinder = RestDepth(
    R"({"name": "cylinder", "type": "cylinder", "radius": 0.04,
        "half_length": 0.2, "material": "block",
        "pose": {"R": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]}})",
    0.04, 2.0, "", kBoxGround);
  EXPECT_NEAR(-cylinder, cylinder_depth, 5e-3 * cylinder_depth);
}

TEST(ContactPrimitives, BoxRestsOnItsCorners) {
  /* four corners of a face, each a linear spring: m g = 4 k d */
  const double depth = 1.0 * 9.81 / (4.0 * 0.5e5);
  const double height = RestDepth(
    R"({"name": "box", "type": "box", "half_extents": [0.1, 0.2, 0.05],
        "material": "block"})",
    0.05, 1.0);
  EXPECT_NEAR(-height, depth, 5e-3 * depth);
}
