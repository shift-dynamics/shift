/* Contact data of collision geometry pairs: penetration depth, normal and
contact points from GJK + EPA (collision_geometry/epa.h), on shapes of known
size fitted with Base::FitToPoints */
#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"

using shift::core::collision_geometry::Base;
using shift::core::collision_geometry::ContactPair;
using shift::core::collision_geometry::ConvexHull;
using shift::core::collision_geometry::Ellipsoid;
using shift::core::collision_geometry::Plane;
using shift::core::collision_geometry::Sphere;
using shift::core::collision_geometry::TestContact;

static ksl_vec3_t V(double x, double y, double z) {
  return {{x, y, z}};
}

static double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* rotation about a unit axis (Rodrigues); R.v[i] is column i */
static ksl_SE3_t Pose(const ksl_vec3_t& t, const ksl_vec3_t& axis = V(1, 0, 0),
                      double angle = 0.0) {
  ksl_SE3_t d;
  const double c = std::cos(angle), s = std::sin(angle);
  const double k[3] = {axis.x, axis.y, axis.z};
  for(int j = 0; j < 3; j++) {
    for(int i = 0; i < 3; i++) {
      double r = (1.0 - c) * k[i] * k[j] + (i == j ? c : 0.0);
      /* + s [k]x, [k]x(i, j) = -eps(i, j, m) k[m] */
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

static std::vector<ksl_vec3_t> BoxPoints(double hx, double hy, double hz) {
  std::vector<ksl_vec3_t> points;
  for(int i = 0; i < 8; i++) {
    points.push_back(V((i & 1) ? hx : -hx, (i & 2) ? hy : -hy,
                       (i & 4) ? hz : -hz));
  }
  return points;
}

/* points on an ellipsoid with semi axes a, b, c (a sphere if equal) */
static std::vector<ksl_vec3_t> EllipsoidPoints(double a, double b, double c,
                                               int n = 400) {
  std::vector<ksl_vec3_t> points;
  const double golden = M_PI * (3.0 - std::sqrt(5.0));
  for(int i = 0; i < n; i++) {
    const double z = 1.0 - 2.0 * (i + 0.5) / n;
    const double r = std::sqrt(1.0 - z * z);
    points.push_back(
      V(a * r * std::cos(golden * i), b * r * std::sin(golden * i), c * z));
  }
  return points;
}

template <class T>
static std::unique_ptr<Base> Make(const std::vector<ksl_vec3_t>& points) {
  auto g = std::make_unique<T>();
  g->FitToPoints(points);
  return g;
}

static ContactPair Collide(Base& a, const ksl_SE3_t& pose_a, Base& b,
                        const ksl_SE3_t& pose_b) {
  a.UpdatePose(pose_a);
  b.UpdatePose(pose_b);
  ContactPair pair;
  pair.a = &a;
  pair.b = &b;
  TestContact(pair);
  return pair;
}

static ksl_SE3_t Moved(ksl_SE3_t pose, const ksl_vec3_t& n, double distance) {
  pose.t.x += distance * n.x;
  pose.t.y += distance * n.y;
  pose.t.z += distance * n.z;
  return pose;
}

/* the extreme projection of a geometry along a direction */
static double Extent(Base& g, const ksl_vec3_t& d) {
  ksl_vec3_t local, global;
  int hint = 0;
  g.Support(d, local, global, hint);
  return Dot(global, d);
}

TEST(ContactGeometry, BoxFaceContact) {
  auto a = Make<ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto b = Make<ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  const ContactPair p =
    Collide(*a, Pose(V(0, 0, 0)), *b, Pose(V(0.9, 0.1, 0.05)));
  ASSERT_TRUE(p.collision);
  EXPECT_TRUE(p.depth_converged);
  EXPECT_NEAR(p.depth, 0.1, 1e-9);
  EXPECT_NEAR(p.normal.x, 1.0, 1e-9);
  EXPECT_NEAR(p.point_a.x, 0.5, 1e-9);
  EXPECT_NEAR(p.point_b.x, 0.4, 1e-9);
  // contact points in each geometry's frame (the body origin for a hull)
  EXPECT_NEAR(p.point_local_a.x, 0.5, 1e-9);
  EXPECT_NEAR(p.point_local_b.x, -0.5, 1e-9);
}

TEST(ContactGeometry, SpheresAnalytic) {
  auto a = Make<Sphere>(EllipsoidPoints(1, 1, 1));
  auto b = Make<Sphere>(EllipsoidPoints(1, 1, 1));
  const double r = static_cast<Sphere&>(*a).get_radius();
  const ContactPair p =
    Collide(*a, Pose(V(0, 0, 0)), *b, Pose(V(0, 2 * r - 0.2, 0)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, 0.2, 1e-12);
  EXPECT_NEAR(p.normal.y, 1.0, 1e-12);
  EXPECT_NEAR(p.point_a.y - p.point_b.y, 0.2, 1e-12);
}

TEST(ContactGeometry, SphereOnBoxFace) {
  // a sphere through GJK and EPA (the pair is not two spheres)
  auto box = Make<ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto ball = Make<Sphere>(EllipsoidPoints(0.3, 0.3, 0.3));
  const auto& sphere = static_cast<Sphere&>(*ball);
  const double r = sphere.get_radius();
  // the fitted sphere center is not exactly the origin of its points
  const double cz = sphere.get_d_wrt_body().t.z;
  const ContactPair p =
    Collide(*box, Pose(V(0, 0, 0)), *ball,
            Pose(V(0.1, -0.2, 0.5 + r - cz - 0.05)));
  ASSERT_TRUE(p.collision);
  EXPECT_TRUE(p.depth_converged);
  EXPECT_NEAR(p.depth, 0.05, 1e-6);
  EXPECT_NEAR(p.normal.z, 1.0, 1e-6);
  EXPECT_NEAR(p.point_a.z, 0.5, 1e-6);
}

TEST(ContactGeometry, CoincidentBoxesSeparateThroughAFace) {
  auto a = Make<ConvexHull>(BoxPoints(0.5, 0.3, 0.2));
  auto b = Make<ConvexHull>(BoxPoints(0.5, 0.3, 0.2));
  const ContactPair p = Collide(*a, Pose(V(0, 0, 0)), *b, Pose(V(0, 0, 0)));
  ASSERT_TRUE(p.collision);
  // the shortest way out is along z, through the thinnest dimension
  EXPECT_NEAR(p.depth, 0.4, 1e-9);
  EXPECT_NEAR(std::fabs(p.normal.z), 1.0, 1e-9);
}

TEST(ContactGeometry, SphereCenterInsideBox) {
  // the nearest face of the box is +y, 0.1 from the center
  auto box = Make<ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto ball = Make<Sphere>(EllipsoidPoints(0.2, 0.2, 0.2));
  const auto& sphere = static_cast<Sphere&>(*ball);
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const ContactPair p = Collide(*box, Pose(V(0, 0, 0)), *ball,
                                Pose(V(0.05 - c.x, 0.4 - c.y, -c.z)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, sphere.get_radius() + 0.1, 1e-12);
  EXPECT_NEAR(p.normal.y, 1.0, 1e-12);
  EXPECT_NEAR(p.point_a.y, 0.5, 1e-12);
  EXPECT_NEAR(p.point_a.x, 0.05, 1e-12);
}

TEST(ContactGeometry, SphereOnBoxEdge) {
  // the center beyond an edge of the box: the normal is diagonal
  auto box = Make<ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto ball = Make<Sphere>(EllipsoidPoints(0.2, 0.2, 0.2));
  const auto& sphere = static_cast<Sphere&>(*ball);
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const double k = 0.1 / std::sqrt(2.0);
  const ContactPair p = Collide(*box, Pose(V(0, 0, 0)), *ball,
                                Pose(V(0.5 + k - c.x, 0.5 + k - c.y, -c.z)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, sphere.get_radius() - 0.1, 1e-12);
  EXPECT_NEAR(p.normal.x, 1.0 / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(p.normal.y, 1.0 / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(p.point_a.x, 0.5, 1e-12);
  EXPECT_NEAR(p.point_a.y, 0.5, 1e-12);
}

TEST(ContactGeometry, SphereOnEllipsoidAxes) {
  // outside the ellipsoid along a principal axis, its closest point is the
  // end of that axis
  auto shape = Make<Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  auto ball = Make<Sphere>(EllipsoidPoints(0.1, 0.1, 0.1));
  const auto& ellipsoid = static_cast<Ellipsoid&>(*shape);
  const auto& sphere = static_cast<Sphere&>(*ball);
  const ksl_SE3_t& frame = ellipsoid.get_d_wrt_body();
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const double r = sphere.get_radius();
  for(int axis = 0; axis < 3; axis++) {
    for(double side : {1.0, -1.0}) {
      const ksl_vec3_t& u = frame.R.v[axis];
      const double a = ellipsoid.get_semi_axes().at[axis];
      const double distance = a + r - 0.02; // center to center
      ksl_vec3_t t;
      for(int k = 0; k < 3; k++) {
        t.at[k] = frame.t.at[k] + side * distance * u.at[k] - c.at[k];
      }
      const ContactPair p = Collide(*shape, Pose(V(0, 0, 0)), *ball, Pose(t));
      ASSERT_TRUE(p.collision) << axis << " " << side;
      EXPECT_NEAR(p.depth, 0.02, 1e-12) << axis << " " << side;
      EXPECT_NEAR(Dot(p.normal, u), side, 1e-12) << axis << " " << side;
    }
  }
}

TEST(ContactGeometry, EllipsoidsAlongAxes) {
  // equal, aligned ellipsoids with centers 2 a - 0.02 apart along a principal
  // axis overlap by 0.02 along it, the ends of the axis being the contact
  // points
  auto first = Make<Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  auto second = Make<Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  const auto& ellipsoid = static_cast<Ellipsoid&>(*first);
  const ksl_SE3_t& frame = ellipsoid.get_d_wrt_body();
  for(int axis = 0; axis < 3; axis++) {
    for(double side : {1.0, -1.0}) {
      const ksl_vec3_t& u = frame.R.v[axis];
      const double a = ellipsoid.get_semi_axes().at[axis];
      for(double gap : {-0.02, -1e-7, 1e-7}) {
        const double distance = 2.0 * a + gap;
        const ksl_SE3_t pose_b = Pose(
          V(side * distance * u.x, side * distance * u.y, side * distance * u.z));
        const ContactPair p =
          Collide(*first, Pose(V(0, 0, 0)), *second, pose_b);
        ASSERT_TRUE(p.success);
        ASSERT_EQ(p.collision, gap < 0.0) << axis << " " << side << " " << gap;
        if(!p.collision) {
          continue;
        }
        EXPECT_NEAR(p.depth, -gap, 1e-12) << axis << " " << side;
        EXPECT_NEAR(Dot(p.normal, u), side, 1e-9) << axis << " " << side;
        const ksl_vec3_t c = frame.t;
        EXPECT_NEAR(Dot(p.point_a, u) - Dot(c, u), side * a, 1e-9);
      }
    }
  }
}

/* the penetration depth by brute force: the least translation separating a
and b over a dense set of directions, polished by sampling around the best */
static double BruteForceDepth(Base& a, Base& b, ksl_vec3_t& best) {
  auto distance = [&](const ksl_vec3_t& n) {
    return Extent(a, n) + Extent(b, V(-n.x, -n.y, -n.z));
  };
  double depth = HUGE_VAL;
  const int count = 40000;
  const double golden = M_PI * (3.0 - std::sqrt(5.0));
  for(int i = 0; i < count; i++) {
    const double z = 1.0 - 2.0 * (i + 0.5) / count;
    const double r = std::sqrt(1.0 - z * z);
    const ksl_vec3_t n = V(r * std::cos(golden * i), r * std::sin(golden * i), z);
    const double d = distance(n);
    if(d < depth) {
      depth = d;
      best = n;
    }
  }
  for(double step = 0.02; step > 1e-9; step *= 0.5) {
    bool improved = true;
    while(improved) {
      improved = false;
      for(int k = 0; k < 3; k++) {
        for(double sign : {1.0, -1.0}) {
          ksl_vec3_t n = best;
          n.at[k] += sign * step;
          const double l = std::sqrt(Dot(n, n));
          n = V(n.x / l, n.y / l, n.z / l);
          const double d = distance(n);
          if(d < depth) {
            depth = d;
            best = n;
            improved = true;
          }
        }
      }
    }
  }
  return depth;
}

/* random overlapping poses of ellipsoids of several aspect ratios: the
penetration depth is the global least separating translation */
TEST(ContactGeometry, EllipsoidPairsGlobalDepth) {
  const ksl_vec3_t shapes[] = {
    V(0.4, 0.25, 0.15), V(0.6, 0.05, 0.05), V(0.3, 0.3, 0.02),
    V(0.2, 0.2, 0.2)};
  std::mt19937 random(7);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(const auto& sa : shapes) {
    for(const auto& sb : shapes) {
      auto a = Make<Ellipsoid>(EllipsoidPoints(sa.x, sa.y, sa.z));
      auto b = Make<Ellipsoid>(EllipsoidPoints(sb.x, sb.y, sb.z));
      for(int trial = 0; trial < 12; trial++) {
        ksl_vec3_t axis_a = V(unit(random), unit(random), unit(random));
        ksl_vec3_t axis_b = V(unit(random), unit(random), unit(random));
        for(auto* axis : {&axis_a, &axis_b}) {
          const double k = 1.0 / std::sqrt(Dot(*axis, *axis));
          *axis = V(k * axis->x, k * axis->y, k * axis->z);
        }
        const ksl_SE3_t pose_a = Pose(V(0, 0, 0), axis_a, 3.0 * unit(random));
        const ksl_SE3_t pose_b =
          Pose(V(0.3 * unit(random), 0.3 * unit(random), 0.3 * unit(random)),
               axis_b, 3.0 * unit(random));
        const ContactPair p = Collide(*a, pose_a, *b, pose_b);
        ksl_vec3_t n;
        const double depth = BruteForceDepth(*a, *b, n);
        ASSERT_EQ(p.collision, depth > 0.0) << "trial " << trial;
        if(!p.collision) {
          continue;
        }
        tested++;
        EXPECT_NEAR(p.depth, depth, 1e-7)
          << "trial " << trial << " normal " << Dot(p.normal, n);
        if(depth > 1e-3) {
          EXPECT_GT(Dot(p.normal, n), 1.0 - 1e-4) << "trial " << trial;
        }
      }
    }
  }
  EXPECT_GT(tested, 60);
}

static std::unique_ptr<Plane> MakePlane(const ksl_vec3_t& normal,
                                        double offset) {
  auto plane = std::make_unique<Plane>();
  plane->SetPlane(normal, offset);
  return plane;
}

TEST(ContactGeometry, SphereOnGround) {
  // ground at z = 0, up is -z; a sphere center r - 0.05 above it
  auto ground = MakePlane(V(0, 0, -1), 0.0);
  auto ball = Make<Sphere>(EllipsoidPoints(0.2, 0.2, 0.2));
  const auto& sphere = static_cast<Sphere&>(*ball);
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const double r = sphere.get_radius();
  const ContactPair p = Collide(*ground, Pose(V(0, 0, 0)), *ball,
                                Pose(V(0.3 - c.x, -c.y, -(r - 0.05) - c.z)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, 0.05, 1e-12);
  EXPECT_NEAR(p.normal.z, -1.0, 1e-12);
  EXPECT_NEAR(p.point_a.z, 0.0, 1e-12);
  EXPECT_NEAR(p.point_a.x, 0.3, 1e-12);
  EXPECT_NEAR(p.point_b.z, 0.05, 1e-12);
  // the sphere as a, the plane as b: the normal from a to b reverses
  const ContactPair q =
    Collide(*ball, Pose(V(0.3 - c.x, -c.y, -(r - 0.05) - c.z)), *ground,
            Pose(V(0, 0, 0)));
  ASSERT_TRUE(q.collision);
  EXPECT_NEAR(q.depth, 0.05, 1e-12);
  EXPECT_NEAR(q.normal.z, 1.0, 1e-12);
  // clear of the ground
  EXPECT_FALSE(Collide(*ground, Pose(V(0, 0, 0)), *ball,
                       Pose(V(0, 0, -(r + 0.01) - c.z)))
                 .collision);
}

TEST(ContactGeometry, ShapesOnTiltedPlane) {
  // the plane's body is rotated and moved, and the plane is offset in it;
  // the depth is checked against the lowest point of each shape found by
  // brute force
  std::mt19937 random(5);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  auto plane = MakePlane(V(0.2, -0.1, -1.0), 0.3);
  const ksl_SE3_t plane_pose = Pose(V(0.1, 0.2, -0.3), V(0.6, 0.8, 0), 0.4);
  plane->UpdatePose(plane_pose);
  const ksl_vec3_t n = plane->get_normal();
  const double offset = Dot(n, plane->get_point());

  auto hull = Make<ConvexHull>(BoxPoints(0.4, 0.25, 0.15));
  auto ellipsoid = Make<Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  auto ball = Make<Sphere>(EllipsoidPoints(0.3, 0.3, 0.3));
  for(Base* shape : {hull.get(), ellipsoid.get(), ball.get()}) {
    int tested = 0;
    for(int trial = 0; trial < 50; trial++) {
      ksl_vec3_t axis = V(unit(random), unit(random), unit(random));
      const double k = 1.0 / std::sqrt(Dot(axis, axis));
      axis = V(k * axis.x, k * axis.y, k * axis.z);
      // around the plane: offset along the normal by -0.5 .. 0.5
      const double h = 0.5 * unit(random);
      const ksl_vec3_t t = V(offset * n.x + h * n.x + 0.3 * unit(random),
                             offset * n.y + h * n.y + 0.3 * unit(random),
                             offset * n.z + h * n.z);
      const ksl_SE3_t pose = Pose(t, axis, 3.0 * unit(random));
      shape->UpdatePose(pose);

      // the lowest point along n, by brute force over the shape's points
      std::vector<ksl_vec3_t> points;
      if(shape == hull.get()) {
        points = static_cast<ConvexHull&>(*hull).get_hull().vertices;
      } else if(shape == ellipsoid.get()) {
        const auto& e = static_cast<Ellipsoid&>(*ellipsoid);
        const ksl_SE3_t& frame = e.get_d_wrt_body();
        for(const auto& u : EllipsoidPoints(e.get_semi_axes().x,
                                            e.get_semi_axes().y,
                                            e.get_semi_axes().z, 20000)) {
          ksl_vec3_t x;
          ksl_product_dv(&frame, &u, &x);
          points.push_back(x);
        }
      }
      double lowest = HUGE_VAL;
      if(shape == ball.get()) {
        const auto& sphere = static_cast<Sphere&>(*ball);
        lowest = Dot(n, sphere.get_center()) - sphere.get_radius();
      }
      for(const auto& x : points) {
        ksl_vec3_t world;
        ksl_product_dv(&pose, &x, &world);
        lowest = std::min(lowest, Dot(n, world));
      }
      const double expected = offset - lowest;

      const ContactPair p = Collide(*plane, plane_pose, *shape, pose);
      const double tolerance = shape == ellipsoid.get() ? 2e-4 : 1e-12;
      if(std::fabs(expected) < tolerance) {
        continue; // touching, either answer is right
      }
      ASSERT_EQ(p.collision, expected > 0.0) << trial;
      if(!p.collision) {
        continue;
      }
      tested++;
      EXPECT_NEAR(p.depth, expected, tolerance) << trial;
      EXPECT_NEAR(Dot(p.normal, n), 1.0, 1e-12);
      EXPECT_NEAR(Dot(n, p.point_a), offset, 1e-12); // on the plane
      for(int k = 0; k < 3; k++) {
        EXPECT_NEAR(p.point_a.at[k] - p.point_b.at[k],
                    p.depth * p.normal.at[k], 1e-12);
      }
    }
    EXPECT_GT(tested, 10);
  }
}

TEST(ContactGeometry, PlanePairs) {
  // ground (solid z >= 0) and a ceiling facing it (solid z <= 0.1): they
  // overlap in a slab 0.1 thick
  auto ground = MakePlane(V(0, 0, -1), 0.0);
  auto ceiling = MakePlane(V(0, 0, 1), 0.1);
  const ContactPair p =
    Collide(*ground, Pose(V(0, 0, 0)), *ceiling, Pose(V(0.5, 0, 0)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, 0.1, 1e-12);
  EXPECT_NEAR(p.normal.z, -1.0, 1e-12);
  EXPECT_NEAR(p.point_a.z, 0.0, 1e-12);
  EXPECT_NEAR(p.point_b.z, 0.1, 1e-12);
  // moved apart, the slab vanishes
  EXPECT_FALSE(
    Collide(*ground, Pose(V(0, 0, 0)), *ceiling, Pose(V(0, 0, -0.2)))
      .collision);
  // half spaces that do not face each other overlap without bound, and are
  // not reported
  auto wall = MakePlane(V(1, 0, 0), 0.0);
  EXPECT_FALSE(
    Collide(*ground, Pose(V(0, 0, 0)), *wall, Pose(V(0, 0, 0))).collision);
}

/* every pairing of the three types at random overlapping poses */
class ContactGeometryPairs : public testing::TestWithParam<int> {
protected:
  std::unique_ptr<Base> MakeShape(int type) {
    switch(type) {
      case 0:
        return Make<ConvexHull>(BoxPoints(0.4, 0.25, 0.15));
      case 1:
        return Make<Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
      default:
        return Make<Sphere>(EllipsoidPoints(0.3, 0.3, 0.3));
    }
  }
};

TEST_P(ContactGeometryPairs, MinimumTranslationSeparates) {
  auto a = MakeShape(GetParam() / 3);
  auto b = MakeShape(GetParam() % 3);
  std::mt19937 random(GetParam() + 1);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(int trial = 0; trial < 60; trial++) {
    ksl_vec3_t axis_a = V(unit(random), unit(random), unit(random));
    ksl_vec3_t axis_b = V(unit(random), unit(random), unit(random));
    for(auto* axis : {&axis_a, &axis_b}) {
      const double k = 1.0 / std::sqrt(Dot(*axis, *axis));
      *axis = V(k * axis->x, k * axis->y, k * axis->z);
    }
    const ksl_SE3_t pose_a = Pose(V(0, 0, 0), axis_a, 3.0 * unit(random));
    const ksl_SE3_t pose_b =
      Pose(V(0.4 * unit(random), 0.4 * unit(random), 0.4 * unit(random)),
           axis_b, 3.0 * unit(random));
    const ContactPair p = Collide(*a, pose_a, *b, pose_b);
    if(!p.collision) {
      continue;
    }
    tested++;
    EXPECT_TRUE(p.depth_converged);
    EXPECT_NEAR(Dot(p.normal, p.normal), 1.0, 1e-9);
    const double delta = 1e-3;
    // b moved by a little more than the depth along the normal is clear of a,
    // by a little less it still overlaps
    EXPECT_FALSE(
      Collide(*a, pose_a, *b, Moved(pose_b, p.normal, p.depth + delta))
        .collision)
      << "trial " << trial << " depth " << p.depth;
    if(p.depth > 2 * delta) {
      EXPECT_TRUE(
        Collide(*a, pose_a, *b, Moved(pose_b, p.normal, p.depth - delta))
          .collision)
        << "trial " << trial << " depth " << p.depth;
    }
    // depth = extent of a along the normal + extent of b against it
    a->UpdatePose(pose_a);
    b->UpdatePose(pose_b);
    const ksl_vec3_t minus = V(-p.normal.x, -p.normal.y, -p.normal.z);
    EXPECT_NEAR(p.depth, Extent(*a, p.normal) + Extent(*b, minus), 1e-5);
    // the contact points are those extreme points, depth * normal apart
    EXPECT_NEAR(Dot(p.point_a, p.normal), Extent(*a, p.normal), 1e-5);
    EXPECT_NEAR(Dot(p.point_b, minus), Extent(*b, minus), 1e-5);
    for(int k = 0; k < 3; k++) {
      EXPECT_NEAR(p.point_a.at[k] - p.point_b.at[k], p.depth * p.normal.at[k],
                  1e-5);
    }
    // swapping the geometries reverses the normal
    const ContactPair q = Collide(*b, pose_b, *a, pose_a);
    ASSERT_TRUE(q.collision);
    EXPECT_NEAR(q.depth, p.depth, 1e-5);
    EXPECT_NEAR(Dot(q.normal, p.normal), -1.0, 1e-4);
  }
  EXPECT_GT(tested, 10);
}

/* 0..8: hull, ellipsoid, sphere for a times the same for b */
INSTANTIATE_TEST_SUITE_P(AllTypes, ContactGeometryPairs, testing::Range(0, 9));
