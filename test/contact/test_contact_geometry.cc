/* Contact data of collision geometry pairs: penetration depth, normal and
contact points from GJK + EPA (collision_geometry/epa.h), on shapes of known
size fitted with shift::core::collision_geometry::Base::FitToPoints */
#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "test/contact/contact_record.h"
#include "shift/core/collision_geometry/analytic_contact.h"
#include "shift/core/collision_geometry/box.h"
#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"

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

/* the rims of a cylinder of radius r and half length h along z (the segment
ends of a capsule of radius r), as points that fit it exactly */
static std::vector<ksl_vec3_t> CylinderPoints(double r, double h) {
  std::vector<ksl_vec3_t> points;
  for(int k = 0; k < 64; k++) {
    const double a = 2.0 * M_PI * k / 64;
    for(double z : {-h, h}) {
      points.push_back(V(r * std::cos(a), r * std::sin(a), z));
    }
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
static std::unique_ptr<shift::core::collision_geometry::Base> Make(const std::vector<ksl_vec3_t>& points) {
  auto g = std::make_unique<T>();
  g->FitToPoints(points);
  return g;
}

static shift::core::collision_geometry::ContactPair Collide(shift::core::collision_geometry::Base& a, const ksl_SE3_t& pose_a, shift::core::collision_geometry::Base& b,
                        const ksl_SE3_t& pose_b) {
  a.UpdatePose(pose_a);
  b.UpdatePose(pose_b);
  shift::core::collision_geometry::ContactPair pair;
  pair.a = &a;
  pair.b = &b;
  shift::test::TestContact(pair);
  return pair;
}

static ksl_SE3_t Moved(ksl_SE3_t pose, const ksl_vec3_t& n, double distance) {
  pose.t.x += distance * n.x;
  pose.t.y += distance * n.y;
  pose.t.z += distance * n.z;
  return pose;
}

/* the extreme projection of a geometry along a direction */
static double Extent(shift::core::collision_geometry::Base& g, const ksl_vec3_t& d) {
  ksl_vec3_t local, global;
  int hint = 0;
  g.Support(d, local, global, hint);
  return Dot(global, d);
}

TEST(ContactGeometry, BoxFaceContact) {
  auto a = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto b = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  const shift::core::collision_geometry::ContactPair p =
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
  auto a = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(1, 1, 1));
  auto b = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(1, 1, 1));
  const double r = static_cast<shift::core::collision_geometry::Sphere&>(*a).get_radius();
  const shift::core::collision_geometry::ContactPair p =
    Collide(*a, Pose(V(0, 0, 0)), *b, Pose(V(0, 2 * r - 0.2, 0)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, 0.2, 1e-12);
  EXPECT_NEAR(p.normal.y, 1.0, 1e-12);
  EXPECT_NEAR(p.point_a.y - p.point_b.y, 0.2, 1e-12);
}

TEST(ContactGeometry, SphereOnBoxFace) {
  // a sphere through GJK and EPA (the pair is not two spheres)
  auto box = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.3, 0.3, 0.3));
  const auto& sphere = static_cast<shift::core::collision_geometry::Sphere&>(*ball);
  const double r = sphere.get_radius();
  // the fitted sphere center is not exactly the origin of its points
  const double cz = sphere.get_d_wrt_body().t.z;
  const shift::core::collision_geometry::ContactPair p =
    Collide(*box, Pose(V(0, 0, 0)), *ball,
            Pose(V(0.1, -0.2, 0.5 + r - cz - 0.05)));
  ASSERT_TRUE(p.collision);
  EXPECT_TRUE(p.depth_converged);
  EXPECT_NEAR(p.depth, 0.05, 1e-6);
  EXPECT_NEAR(p.normal.z, 1.0, 1e-6);
  EXPECT_NEAR(p.point_a.z, 0.5, 1e-6);
}

TEST(ContactGeometry, CoincidentBoxesSeparateThroughAFace) {
  auto a = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.3, 0.2));
  auto b = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.3, 0.2));
  const shift::core::collision_geometry::ContactPair p = Collide(*a, Pose(V(0, 0, 0)), *b, Pose(V(0, 0, 0)));
  ASSERT_TRUE(p.collision);
  // the shortest way out is along z, through the thinnest dimension
  EXPECT_NEAR(p.depth, 0.4, 1e-9);
  EXPECT_NEAR(std::fabs(p.normal.z), 1.0, 1e-9);
}

TEST(ContactGeometry, SphereCenterInsideBox) {
  // the nearest face of the box is +y, 0.1 from the center
  auto box = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.2, 0.2, 0.2));
  const auto& sphere = static_cast<shift::core::collision_geometry::Sphere&>(*ball);
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const shift::core::collision_geometry::ContactPair p = Collide(*box, Pose(V(0, 0, 0)), *ball,
                                Pose(V(0.05 - c.x, 0.4 - c.y, -c.z)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, sphere.get_radius() + 0.1, 1e-12);
  EXPECT_NEAR(p.normal.y, 1.0, 1e-12);
  EXPECT_NEAR(p.point_a.y, 0.5, 1e-12);
  EXPECT_NEAR(p.point_a.x, 0.05, 1e-12);
}

TEST(ContactGeometry, SphereOnBoxEdge) {
  // the center beyond an edge of the box: the normal is diagonal
  auto box = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.5, 0.5, 0.5));
  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.2, 0.2, 0.2));
  const auto& sphere = static_cast<shift::core::collision_geometry::Sphere&>(*ball);
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const double k = 0.1 / std::sqrt(2.0);
  const shift::core::collision_geometry::ContactPair p = Collide(*box, Pose(V(0, 0, 0)), *ball,
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
  auto shape = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.1, 0.1, 0.1));
  const auto& ellipsoid = static_cast<shift::core::collision_geometry::Ellipsoid&>(*shape);
  const auto& sphere = static_cast<shift::core::collision_geometry::Sphere&>(*ball);
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
      const shift::core::collision_geometry::ContactPair p = Collide(*shape, Pose(V(0, 0, 0)), *ball, Pose(t));
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
  auto first = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  auto second = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  const auto& ellipsoid = static_cast<shift::core::collision_geometry::Ellipsoid&>(*first);
  const ksl_SE3_t& frame = ellipsoid.get_d_wrt_body();
  for(int axis = 0; axis < 3; axis++) {
    for(double side : {1.0, -1.0}) {
      const ksl_vec3_t& u = frame.R.v[axis];
      const double a = ellipsoid.get_semi_axes().at[axis];
      for(double gap : {-0.02, -1e-7, 1e-7}) {
        const double distance = 2.0 * a + gap;
        const ksl_SE3_t pose_b = Pose(
          V(side * distance * u.x, side * distance * u.y, side * distance * u.z));
        const shift::core::collision_geometry::ContactPair p =
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
static double BruteForceDepth(shift::core::collision_geometry::Base& a, shift::core::collision_geometry::Base& b, ksl_vec3_t& best) {
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
      auto a = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(sa.x, sa.y, sa.z));
      auto b = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(sb.x, sb.y, sb.z));
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
        const shift::core::collision_geometry::ContactPair p = Collide(*a, pose_a, *b, pose_b);
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

/* random points of an irregular solid, for a hull with vertices, edges and
faces of many sizes */
static std::vector<ksl_vec3_t> IrregularPoints(int count, unsigned seed) {
  std::mt19937 random(seed);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::vector<ksl_vec3_t> points;
  for(int i = 0; i < count; i++) {
    points.push_back(V(0.3 * unit(random), 0.2 * unit(random),
                       0.12 * unit(random)));
  }
  return points;
}

/* random overlapping poses of hulls and ellipsoids: the exact contact (see
shift::core::collision_geometry::ConvexHullEllipsoidContact) is the global least separating translation */
TEST(ContactGeometry, HullEllipsoidGlobalDepth) {
  const ksl_vec3_t shapes[] = {V(0.4, 0.25, 0.15), V(0.5, 0.06, 0.06),
                               V(0.3, 0.3, 0.03), V(0.2, 0.2, 0.2)};
  std::vector<std::unique_ptr<shift::core::collision_geometry::Base>> hulls;
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.3, 0.2, 0.1)));
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(IrregularPoints(60, 3)));
  std::mt19937 random(11);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(auto& hull : hulls) {
    for(const auto& sb : shapes) {
      auto ellipsoid = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(sb.x, sb.y, sb.z));
      for(int trial = 0; trial < 12; trial++) {
        ksl_vec3_t axis_a = V(unit(random), unit(random), unit(random));
        ksl_vec3_t axis_b = V(unit(random), unit(random), unit(random));
        for(auto* axis : {&axis_a, &axis_b}) {
          const double k = 1.0 / std::sqrt(Dot(*axis, *axis));
          *axis = V(k * axis->x, k * axis->y, k * axis->z);
        }
        const ksl_SE3_t pose_a = Pose(V(0, 0, 0), axis_a, 3.0 * unit(random));
        const ksl_SE3_t pose_b =
          Pose(V(0.35 * unit(random), 0.35 * unit(random), 0.35 * unit(random)),
               axis_b, 3.0 * unit(random));
        const shift::core::collision_geometry::ContactPair p = Collide(*hull, pose_a, *ellipsoid, pose_b);
        ksl_vec3_t n;
        const double depth = BruteForceDepth(*hull, *ellipsoid, n);
        if(std::fabs(depth) < 1e-6) {
          continue; // touching: either answer
        }
        ASSERT_EQ(p.collision, depth > 0.0) << "trial " << trial;
        if(!p.collision) {
          continue;
        }
        tested++;
        // the depth is d at the normal, d(n) = extent of the hull along n +
        // extent of the ellipsoid against it, and no more than the brute
        // force least d, whose search stalls at the creases of d where the
        // feature of the hull changes
        auto d = [&](const ksl_vec3_t& m) {
          return Extent(*hull, m) + Extent(*ellipsoid, V(-m.x, -m.y, -m.z));
        };
        EXPECT_NEAR(p.depth, d(p.normal), 1e-10) << "trial " << trial;
        EXPECT_LE(p.depth, depth + 1e-10) << "trial " << trial;
        // and a minimum: tilted any way, d grows
        for(int k = 0; k < 3; k++) {
          for(double tilt : {1e-4, -1e-4}) {
            ksl_vec3_t m = p.normal;
            m.at[k] += tilt;
            const double l = std::sqrt(Dot(m, m));
            m = V(m.x / l, m.y / l, m.z / l);
            EXPECT_GE(d(m), p.depth - 1e-12) << "trial " << trial;
          }
        }
        for(int k = 0; k < 3; k++) {
          EXPECT_NEAR(p.point_a.at[k] - p.point_b.at[k],
                      p.depth * p.normal.at[k], 1e-12)
            << "trial " << trial;
        }
      }
    }
  }
  EXPECT_GT(tested, 40);
}

/* the contact of a hull and an ellipsoid is a function of their poses only:
the same from any start of its walk, and continuous in the poses, to the
precision of the arithmetic (EPA's normal on a curved surface is only good
to the square root of its tolerance, about 1e-3) */
TEST(ContactGeometry, HullEllipsoidSmooth) {
  auto hull = Make<shift::core::collision_geometry::ConvexHull>(IrregularPoints(60, 5));
  auto ellipsoid = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(0.25, 0.15, 0.1));
  auto& h = static_cast<shift::core::collision_geometry::ConvexHull&>(*hull);
  auto& e = static_cast<shift::core::collision_geometry::Ellipsoid&>(*ellipsoid);
  std::mt19937 random(13);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(int trial = 0; trial < 40; trial++) {
    ksl_vec3_t axis = V(unit(random), unit(random), unit(random));
    const double k = 1.0 / std::sqrt(Dot(axis, axis));
    axis = V(k * axis.x, k * axis.y, k * axis.z);
    const ksl_SE3_t pose_a = Pose(V(0, 0, 0), axis, 3.0 * unit(random));
    const ksl_SE3_t pose_b =
      Pose(V(0.3 * unit(random), 0.3 * unit(random), 0.3 * unit(random)),
           V(0, 0, 1), 3.0 * unit(random));
    const shift::core::collision_geometry::ContactPair p = Collide(*hull, pose_a, *ellipsoid, pose_b);
    if(!p.collision || p.depth < 1e-3) {
      continue;
    }
    tested++;
    // from starts 0.2 radians off the normal, the same contact
    for(int s = 0; s < 3; s++) {
      ksl_vec3_t start = p.normal;
      start.at[s] += 0.2;
      shift::core::collision_geometry::AnalyticContact c;
      ASSERT_TRUE(shift::core::collision_geometry::ConvexHullEllipsoidContact(h, e, start, false, c)) << trial;
      EXPECT_EQ(c.depth, p.depth) << trial;
      for(int i = 0; i < 3; i++) {
        EXPECT_EQ(c.normal.at[i], p.normal.at[i]) << trial;
      }
    }
    // moved by 1e-8, the normal and the points move by about as little
    const shift::core::collision_geometry::ContactPair q =
      Collide(*hull, pose_a, *ellipsoid, Moved(pose_b, V(0.6, 0.0, 0.8), 1e-8));
    ASSERT_TRUE(q.collision);
    EXPECT_NEAR(q.depth, p.depth, 1e-7) << trial;
    for(int i = 0; i < 3; i++) {
      EXPECT_NEAR(q.normal.at[i], p.normal.at[i], 1e-6) << trial;
      EXPECT_NEAR(q.point_b.at[i], p.point_b.at[i], 1e-6) << trial;
    }
  }
  EXPECT_GT(tested, 10);
}

/* a sphere against a cylinder of radius 0.2 and half length 0.3 along z:
on its side, over an end, beyond a rim and inside, the depth in closed form
(see shift::core::collision_geometry::CylinderSphereContact) */
TEST(ContactGeometry, SphereOnCylinder) {
  auto shape = Make<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.2, 0.3));
  auto& cylinder = static_cast<shift::core::collision_geometry::Cylinder&>(*shape);
  auto sphere = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.05, 0.05, 0.05));
  auto& ball = static_cast<shift::core::collision_geometry::Sphere&>(*sphere);
  const double rs = ball.get_radius();
  // the fitted sphere's center off its body origin
  ball.UpdatePose(Pose(V(0, 0, 0)));
  const ksl_vec3_t offset = ball.get_center();
  const ksl_SE3_t pose = Pose(V(0.1, -0.2, 0.3), V(0.6, 0.0, 0.8), 0.7);
  cylinder.UpdatePose(pose);
  // in the frame of the fitted cylinder, of radius r and half length h
  const ksl_SE3_t frame = cylinder.get_d_absolute();
  const double r = cylinder.get_radius(), h = cylinder.get_half_length();
  struct Case {
    ksl_vec3_t center;
    double depth;
    ksl_vec3_t normal;
  };
  const double k = 1.0 / std::sqrt(2.0);
  const Case cases[] = {
    {V(r + 0.03, 0.0, 0.1), rs - 0.03, V(1, 0, 0)},                 // side
    {V(0.0, -r - 0.02, -0.25), rs - 0.02, V(0, -1, 0)},             // side
    {V(0.05, 0.05, h + 0.03), rs - 0.03, V(0, 0, 1)},               // end
    {V(r + 0.01, 0.0, h + 0.01), rs - 0.01 * std::sqrt(2.0), V(k, 0, k)}, // rim
    {V(0.0, r - 0.03, 0.0), rs + 0.03, V(0, 1, 0)},                 // inside
    {V(0.0, 0.05, -h + 0.02), rs + 0.02, V(0, 0, -1)},              // inside
    {V(r + 0.1, 0.0, 0.0), 0.0, V(1, 0, 0)},                        // clear
  };
  for(const Case& t : cases) {
    ksl_vec3_t center;
    ksl_product_dv(&frame, &t.center, &center);
    center = V(center.x - offset.x, center.y - offset.y, center.z - offset.z);
    const shift::core::collision_geometry::ContactPair p =
      Collide(cylinder, pose, *sphere, Pose(center));
    ASSERT_EQ(p.collision, t.depth > 0.0);
    if(!p.collision) {
      continue;
    }
    EXPECT_TRUE(p.depth_converged);
    EXPECT_NEAR(p.depth, t.depth, 1e-12);
    ksl_vec3_t n;
    for(int i = 0; i < 3; i++) {
      n.at[i] = frame.R.v[0].at[i] * t.normal.x + frame.R.v[1].at[i] * t.normal.y +
                frame.R.v[2].at[i] * t.normal.z;
    }
    EXPECT_NEAR(Dot(p.normal, n), 1.0, 1e-12);
    // the sphere the other way round: the normal reversed
    const shift::core::collision_geometry::ContactPair q =
      Collide(*sphere, Pose(center), cylinder, pose);
    EXPECT_NEAR(q.depth, t.depth, 1e-12);
    EXPECT_NEAR(Dot(q.normal, n), -1.0, 1e-12);
  }
}

/* random overlapping poses of hulls and cylinders: the exact contact (see
shift::core::collision_geometry::CylinderHullContact) is d at its normal, a
minimum of d, and no deeper than the brute force least d */
TEST(ContactGeometry, HullCylinderGlobalDepth) {
  const ksl_vec3_t shapes[] = {V(0.2, 0.3, 0), V(0.35, 0.03, 0),
                               V(0.05, 0.4, 0)}; // radius, half length
  std::vector<std::unique_ptr<shift::core::collision_geometry::Base>> hulls;
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.3, 0.2, 0.1)));
  hulls.push_back(Make<shift::core::collision_geometry::Box>(BoxPoints(0.25, 0.15, 0.05)));
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(IrregularPoints(60, 3)));
  std::mt19937 random(19);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(auto& hull : hulls) {
    for(const auto& sb : shapes) {
      auto cylinder = Make<shift::core::collision_geometry::Cylinder>(CylinderPoints(sb.x, sb.y));
      for(int trial = 0; trial < 15; trial++) {
        ksl_vec3_t axis_a = V(unit(random), unit(random), unit(random));
        ksl_vec3_t axis_b = V(unit(random), unit(random), unit(random));
        for(auto* axis : {&axis_a, &axis_b}) {
          const double k = 1.0 / std::sqrt(Dot(*axis, *axis));
          *axis = V(k * axis->x, k * axis->y, k * axis->z);
        }
        const ksl_SE3_t pose_a = Pose(V(0, 0, 0), axis_a, 3.0 * unit(random));
        const ksl_SE3_t pose_b =
          Pose(V(0.35 * unit(random), 0.35 * unit(random), 0.35 * unit(random)),
               axis_b, 3.0 * unit(random));
        const shift::core::collision_geometry::ContactPair p = Collide(*hull, pose_a, *cylinder, pose_b);
        ksl_vec3_t n;
        const double depth = BruteForceDepth(*hull, *cylinder, n);
        if(std::fabs(depth) < 1e-6) {
          continue; // touching: either answer
        }
        ASSERT_EQ(p.collision, depth > 0.0) << "trial " << trial;
        if(!p.collision) {
          continue;
        }
        tested++;
        auto d = [&](const ksl_vec3_t& m) {
          return Extent(*hull, m) + Extent(*cylinder, V(-m.x, -m.y, -m.z));
        };
        EXPECT_NEAR(p.depth, d(p.normal), 1e-10) << "trial " << trial;
        EXPECT_LE(p.depth, depth + 1e-10) << "trial " << trial;
        for(int k = 0; k < 3; k++) {
          for(double tilt : {1e-4, -1e-4}) {
            ksl_vec3_t m = p.normal;
            m.at[k] += tilt;
            const double l = std::sqrt(Dot(m, m));
            m = V(m.x / l, m.y / l, m.z / l);
            EXPECT_GE(d(m), p.depth - 1e-12) << "trial " << trial;
          }
          EXPECT_NEAR(p.point_a.at[k] - p.point_b.at[k],
                      p.depth * p.normal.at[k], 1e-12)
            << "trial " << trial;
        }
        // the points are on the shapes: their extremes along the normal
        EXPECT_NEAR(Dot(p.point_a, p.normal), Extent(*hull, p.normal), 1e-10);
        EXPECT_NEAR(Dot(p.point_b, p.normal),
                    -Extent(*cylinder, V(-p.normal.x, -p.normal.y, -p.normal.z)),
                    1e-10);
      }
    }
  }
  EXPECT_GT(tested, 50);
}

/* the contact of a hull and a cylinder is a function of their poses only:
the same from any start near it, and continuous in the poses to the
precision of the arithmetic */
TEST(ContactGeometry, HullCylinderSmooth) {
  auto hull = Make<shift::core::collision_geometry::ConvexHull>(IrregularPoints(60, 5));
  auto cylinder = Make<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.15, 0.25));
  auto& hh = static_cast<shift::core::collision_geometry::ConvexHull&>(*hull);
  auto& cc = static_cast<shift::core::collision_geometry::Cylinder&>(*cylinder);
  std::mt19937 random(23);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(int trial = 0; trial < 40; trial++) {
    ksl_vec3_t axis = V(unit(random), unit(random), unit(random));
    const double k = 1.0 / std::sqrt(Dot(axis, axis));
    axis = V(k * axis.x, k * axis.y, k * axis.z);
    const ksl_SE3_t pose_a = Pose(V(0, 0, 0), axis, 3.0 * unit(random));
    const ksl_SE3_t pose_b =
      Pose(V(0.3 * unit(random), 0.3 * unit(random), 0.3 * unit(random)),
           V(0, 0, 1), 3.0 * unit(random));
    const shift::core::collision_geometry::ContactPair p = Collide(*hull, pose_a, *cylinder, pose_b);
    if(!p.collision || p.depth < 1e-3) {
      continue;
    }
    tested++;
    // from starts 0.05 radians off the normal, the same contact
    for(int s = 0; s < 3; s++) {
      ksl_vec3_t start = p.normal;
      start.at[s] += 0.05;
      shift::core::collision_geometry::AnalyticContact c;
      int hint = 0;
      ASSERT_TRUE(shift::core::collision_geometry::CylinderHullContact(hh, cc, start, true, hint, c)) << trial;
      EXPECT_NEAR(c.depth, p.depth, 1e-14) << trial;
      for(int i = 0; i < 3; i++) {
        EXPECT_NEAR(c.normal.at[i], p.normal.at[i], 1e-14) << trial;
      }
    }
    // moved by 1e-8, the depth, the normal and the points move by about as
    // little
    const shift::core::collision_geometry::ContactPair q =
      Collide(*hull, pose_a, *cylinder, Moved(pose_b, V(0.6, 0.0, 0.8), 1e-8));
    ASSERT_TRUE(q.collision);
    EXPECT_NEAR(q.depth, p.depth, 1e-7) << trial;
    for(int i = 0; i < 3; i++) {
      EXPECT_NEAR(q.normal.at[i], p.normal.at[i], 1e-6) << trial;
    }
  }
  EXPECT_GT(tested, 10);
}

/* a cylinder pressed by its end against the face of a box beside its rim,
as a flange against the side of a track link's rail: the normal is the
face's, the depth exact, and as the cylinder tilts by small angles either
way the depth changes in proportion, without a jump */
TEST(ContactGeometry, CylinderEndOnBoxFace) {
  auto shape = Make<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.15, 0.3));
  auto& cylinder = static_cast<shift::core::collision_geometry::Cylinder&>(*shape);
  auto box = Make<shift::core::collision_geometry::Box>(BoxPoints(0.1, 0.02, 0.04));
  cylinder.UpdatePose(Pose(V(0, 0, 0)));
  const ksl_SE3_t frame = cylinder.get_d_absolute(); // the fit's
  const double h = cylinder.get_half_length();
  // the box over the end, from 0.08 to 0.28 off the axis, its lower face
  // 1e-4 into the cylinder
  const ksl_vec3_t local = V(0.18, 0.0, h + 0.04 - 1e-4);
  ksl_vec3_t center;
  ksl_product_dv(&frame, &local, &center);
  ksl_SE3_t pose_box = frame;
  pose_box.t = center;
  auto tilted = [&](double tilt) {
    return Pose(V(0, 0, 0), frame.R.v[1], tilt); // about the fit's y
  };
  const shift::core::collision_geometry::ContactPair flat =
    Collide(*box, pose_box, cylinder, tilted(0.0));
  ASSERT_TRUE(flat.collision);
  EXPECT_NEAR(flat.depth, 1e-4, 1e-12);
  EXPECT_NEAR(Dot(flat.normal, frame.R.v[2]), -1.0, 1e-12);
  for(double tilt : {1e-7, 1e-6, 1e-5, -1e-7, -1e-6, -1e-5}) {
    const shift::core::collision_geometry::ContactPair p =
      Collide(*box, pose_box, cylinder, tilted(tilt));
    ASSERT_TRUE(p.collision) << tilt;
    // the deepest point moves by the tilt times its distance from the axis,
    // at most the radius
    EXPECT_NEAR(p.depth, flat.depth, 0.16 * std::fabs(tilt)) << tilt;
    EXPECT_GT(Dot(p.normal, flat.normal), 1.0 - std::fabs(tilt)) << tilt;
  }
}

/* random overlapping hull pairs: from EPA's normal, the exact contact (see
shift::core::collision_geometry::ConvexHullPairContact) is the minimum EPA found, d at its normal exactly, and
a minimum of d */
TEST(ContactGeometry, HullPairMatchesEpa) {
  std::vector<std::unique_ptr<shift::core::collision_geometry::Base>> hulls;
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.3, 0.2, 0.1)));
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(IrregularPoints(60, 3)));
  hulls.push_back(Make<shift::core::collision_geometry::ConvexHull>(IrregularPoints(40, 8)));
  std::mt19937 random(17);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int tested = 0;
  for(auto& a : hulls) {
    for(auto& b : hulls) {
      if(a == b) {
        continue;
      }
      for(int trial = 0; trial < 15; trial++) {
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
        const shift::core::collision_geometry::ContactPair p = Collide(*a, pose_a, *b, pose_b);
        if(!p.collision || p.depth < 1e-4) {
          continue;
        }
        shift::core::collision_geometry::AnalyticContact c;
        int hint_a = 0, hint_b = 0;
        ASSERT_TRUE(shift::core::collision_geometry::ConvexHullPairContact(static_cast<shift::core::collision_geometry::ConvexHull&>(*a),
                                          static_cast<shift::core::collision_geometry::ConvexHull&>(*b),
                                          p.normal, hint_a, hint_b, c))
          << "trial " << trial;
        ASSERT_TRUE(c.collision);
        tested++;
        auto d = [&](const ksl_vec3_t& m) {
          return Extent(*a, m) + Extent(*b, V(-m.x, -m.y, -m.z));
        };
        EXPECT_NEAR(c.depth, d(c.normal), 1e-12) << "trial " << trial;
        EXPECT_NEAR(c.depth, p.depth, 1e-6) << "trial " << trial;
        EXPECT_GT(Dot(c.normal, p.normal), 1.0 - 1e-6) << "trial " << trial;
        for(int k = 0; k < 3; k++) {
          EXPECT_NEAR(c.point_first.at[k] - c.point_second.at[k],
                      c.depth * c.normal.at[k], 1e-12);
          for(double tilt : {1e-4, -1e-4}) {
            ksl_vec3_t m = c.normal;
            m.at[k] += tilt;
            const double l = std::sqrt(Dot(m, m));
            m = V(m.x / l, m.y / l, m.z / l);
            EXPECT_GE(d(m), c.depth - 1e-12) << "trial " << trial;
          }
        }
      }
    }
  }
  EXPECT_GT(tested, 30);
}

/* two overlapping boxes separate along x by 0.3 or along y by 0.2999: the
deepest point is the y minimum, but a pair whose last accepted step was in
the x minimum keeps it (shift::core::collision_geometry::ContactPair::accepted_normal) */
TEST(ContactGeometry, HullPairKeepsMinimum) {
  auto a = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.2, 0.2, 0.2));
  auto b = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.2, 0.2, 0.2));
  const ksl_SE3_t pose_a = Pose(V(0, 0, 0));
  const ksl_SE3_t pose_b = Pose(V(0.1, 0.1001, 0.0));
  const shift::core::collision_geometry::ContactPair fresh = Collide(*a, pose_a, *b, pose_b);
  ASSERT_TRUE(fresh.collision);
  EXPECT_NEAR(fresh.depth, 0.2999, 1e-6);
  EXPECT_NEAR(fresh.normal.y, 1.0, 1e-6);
  for(const ksl_vec3_t& kept : {V(1, 0, 0), V(1, 0.01, 0.0)}) {
    shift::core::collision_geometry::ContactPair pair;
    pair.a = a.get();
    pair.b = b.get();
    pair.accepted_normal = kept;
    shift::test::TestContact(pair);
    ASSERT_TRUE(pair.collision);
    EXPECT_NEAR(pair.depth, 0.3, 1e-12);
    EXPECT_NEAR(pair.normal.x, 1.0, 1e-12);
  }
}

static std::unique_ptr<shift::core::collision_geometry::Plane> MakePlane(const ksl_vec3_t& normal,
                                        double offset) {
  auto plane = std::make_unique<shift::core::collision_geometry::Plane>();
  plane->SetPlane(normal, offset);
  return plane;
}

TEST(ContactGeometry, SphereOnGround) {
  // ground at z = 0, up is -z; a sphere center r - 0.05 above it
  auto ground = MakePlane(V(0, 0, -1), 0.0);
  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.2, 0.2, 0.2));
  const auto& sphere = static_cast<shift::core::collision_geometry::Sphere&>(*ball);
  const ksl_vec3_t c = sphere.get_d_wrt_body().t;
  const double r = sphere.get_radius();
  const shift::core::collision_geometry::ContactPair p = Collide(*ground, Pose(V(0, 0, 0)), *ball,
                                Pose(V(0.3 - c.x, -c.y, -(r - 0.05) - c.z)));
  ASSERT_TRUE(p.collision);
  EXPECT_NEAR(p.depth, 0.05, 1e-12);
  EXPECT_NEAR(p.normal.z, -1.0, 1e-12);
  EXPECT_NEAR(p.point_a.z, 0.0, 1e-12);
  EXPECT_NEAR(p.point_a.x, 0.3, 1e-12);
  EXPECT_NEAR(p.point_b.z, 0.05, 1e-12);
  // the sphere as a, the plane as b: the normal from a to b reverses
  const shift::core::collision_geometry::ContactPair q =
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

  auto hull = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.4, 0.25, 0.15));
  auto ellipsoid = Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.3, 0.3, 0.3));
  for(shift::core::collision_geometry::Base* shape : {hull.get(), ellipsoid.get(), ball.get()}) {
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
        points = static_cast<shift::core::collision_geometry::ConvexHull&>(*hull).get_hull().vertices;
      } else if(shape == ellipsoid.get()) {
        const auto& e = static_cast<shift::core::collision_geometry::Ellipsoid&>(*ellipsoid);
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
        const auto& sphere = static_cast<shift::core::collision_geometry::Sphere&>(*ball);
        lowest = Dot(n, sphere.get_center()) - sphere.get_radius();
      }
      for(const auto& x : points) {
        ksl_vec3_t world;
        ksl_product_dv(&pose, &x, &world);
        lowest = std::min(lowest, Dot(n, world));
      }
      const double expected = offset - lowest;

      const shift::core::collision_geometry::ContactPair p = Collide(*plane, plane_pose, *shape, pose);
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
  const shift::core::collision_geometry::ContactPair p =
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

static double ManifoldDepth(const shift::core::collision_geometry::ContactPair& p, double& lowest) {
  double deepest = 0.0;
  lowest = HUGE_VAL;
  for(const auto& c : p.manifold) {
    deepest = std::max(deepest, c.depth);
    lowest = std::min(lowest, c.depth);
    for(int k = 0; k < 3; k++) {
      EXPECT_NEAR(c.point_a.at[k] - c.point_b.at[k], c.depth * p.normal.at[k],
                  1e-12);
    }
  }
  return deepest;
}

TEST(ContactGeometry, ManifoldBoxOnPlane) {
  // flat: the four bottom corners; on an edge (tilted 45 degrees about x):
  // the two corners of the edge; curved shapes keep one point
  auto ground = MakePlane(V(0, 0, -1), 0.0);
  auto box = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.1, 0.1, 0.1));
  double lowest;
  shift::core::collision_geometry::ContactPair flat =
    Collide(*ground, Pose(V(0, 0, 0)), *box, Pose(V(0, 0, -0.1 + 0.01)));
  ASSERT_TRUE(flat.collision);
  ASSERT_EQ(flat.manifold.size(), 4u);
  EXPECT_NEAR(ManifoldDepth(flat, lowest), 0.01, 1e-12);
  EXPECT_NEAR(lowest, 0.01, 1e-12);

  const double h = 0.1 * std::sqrt(2.0);
  shift::core::collision_geometry::ContactPair edge = Collide(*ground, Pose(V(0, 0, 0)), *box,
                             Pose(V(0, 0, -h + 0.01), V(1, 0, 0), M_PI / 4));
  ASSERT_TRUE(edge.collision);
  ASSERT_EQ(edge.manifold.size(), 2u);
  EXPECT_NEAR(ManifoldDepth(edge, lowest), 0.01, 1e-12);
  EXPECT_NEAR(lowest, 0.01, 1e-12);

  auto ball = Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.1, 0.1, 0.1));
  const shift::core::collision_geometry::ContactPair round =
    Collide(*ground, Pose(V(0, 0, 0)), *ball, Pose(V(0, 0, -0.09)));
  ASSERT_TRUE(round.collision);
  EXPECT_EQ(round.manifold.size(), 1u);
}

TEST(ContactGeometry, ManifoldBoxOnBox) {
  // a small box on a large one: face on face gives the four corners of the
  // small face, turned 45 degrees about the normal still four; lying on an
  // edge, two
  auto base = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(1.0, 1.0, 0.1));
  auto box = Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.1, 0.1, 0.1));
  double lowest;
  for(double turn : {0.0, M_PI / 4}) {
    const shift::core::collision_geometry::ContactPair p = Collide(*base, Pose(V(0, 0, 0)), *box,
                                  Pose(V(0.3, 0.2, -0.2 + 0.01), V(0, 0, 1), turn));
    ASSERT_TRUE(p.collision);
    ASSERT_EQ(p.manifold.size(), 4u) << turn;
    EXPECT_NEAR(ManifoldDepth(p, lowest), 0.01, 1e-9) << turn;
    EXPECT_NEAR(lowest, 0.01, 1e-9) << turn;
    for(const auto& c : p.manifold) {
      EXPECT_NEAR(Dot(c.point_a, V(0, 0, 1)), -0.1, 1e-9); // on the top face
    }
  }
  const double h = 0.1 * std::sqrt(2.0);
  const shift::core::collision_geometry::ContactPair edge =
    Collide(*base, Pose(V(0, 0, 0)), *box,
            Pose(V(0, 0, -0.1 - h + 0.01), V(1, 0, 0), M_PI / 4));
  ASSERT_TRUE(edge.collision);
  ASSERT_EQ(edge.manifold.size(), 2u);
  EXPECT_NEAR(ManifoldDepth(edge, lowest), 0.01, 1e-9);
  EXPECT_NEAR(lowest, 0.01, 1e-9);

  // overhanging the edge of the base: the corners beyond it are clipped to
  // its side, the manifold stays on the base's top face
  const shift::core::collision_geometry::ContactPair overhang = Collide(*base, Pose(V(0, 0, 0)), *box,
                                       Pose(V(1.05, 0, -0.2 + 0.01)));
  ASSERT_TRUE(overhang.collision);
  ASSERT_EQ(overhang.manifold.size(), 4u);
  for(const auto& c : overhang.manifold) {
    EXPECT_LE(c.point_a.x, 1.0 + 1e-12);
    EXPECT_NEAR(c.depth, 0.01, 1e-9);
  }
}

/* every pairing of the shape types at random overlapping poses: those with
a closed form (sphere, ellipsoid, hull and box among themselves, and the
cylinder with a sphere, a hull or a box) and the others with the cylinder and
the capsule, whose pairs with anything but a plane are GJK and EPA */
static const int kPairTypes = 6;

class ContactGeometryPairs : public testing::TestWithParam<int> {
protected:
  std::unique_ptr<shift::core::collision_geometry::Base> MakeShape(int type) {
    switch(type) {
      case 0:
        return Make<shift::core::collision_geometry::ConvexHull>(BoxPoints(0.4, 0.25, 0.15));
      case 1:
        return Make<shift::core::collision_geometry::Ellipsoid>(EllipsoidPoints(0.4, 0.25, 0.15));
      case 2:
        return Make<shift::core::collision_geometry::Sphere>(EllipsoidPoints(0.3, 0.3, 0.3));
      case 3:
        return Make<shift::core::collision_geometry::Box>(BoxPoints(0.35, 0.2, 0.25));
      case 4:
        return Make<shift::core::collision_geometry::Cylinder>(CylinderPoints(0.2, 0.35));
      default:
        return Make<shift::core::collision_geometry::Capsule>(CylinderPoints(0.15, 0.25));
    }
  }
};

TEST_P(ContactGeometryPairs, MinimumTranslationSeparates) {
  auto a = MakeShape(GetParam() / kPairTypes);
  auto b = MakeShape(GetParam() % kPairTypes);
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
    const shift::core::collision_geometry::ContactPair p = Collide(*a, pose_a, *b, pose_b);
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
    const shift::core::collision_geometry::ContactPair q = Collide(*b, pose_b, *a, pose_a);
    ASSERT_TRUE(q.collision);
    EXPECT_NEAR(q.depth, p.depth, 1e-5);
    EXPECT_NEAR(Dot(q.normal, p.normal), -1.0, 1e-4);
  }
  EXPECT_GT(tested, 10);
}

/* hull, ellipsoid, sphere, box, cylinder, capsule for a times the same for b */
INSTANTIATE_TEST_SUITE_P(AllTypes, ContactGeometryPairs,
                         testing::Range(0, kPairTypes * kPairTypes));
