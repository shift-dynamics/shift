/* ray casts against collision geometry (collision_geometry::Base::RayCast):
spheres, boxes and planes in closed form, ellipsoids, capsules, cylinders and
convex hulls by GJK, and CAD geometry on its exact faces; each hit is checked
against the shape's own inside test along the ray and across its normal */

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <map>
#include <random>
#include <sstream>
#include <string>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/config.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/context.h"

static ksl_vec3_t Along(const ksl_vec3_t& o, double t, const ksl_vec3_t& d) {
  return {{o.x + t * d.x, o.y + t * d.y, o.z + t * d.z}};
}

static double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* a pose turned by angle about a unit axis, at t */
static ksl_SE3_t Pose(const ksl_vec3_t& axis, double angle,
                      const ksl_vec3_t& t) {
  const double c = std::cos(angle), s = std::sin(angle), v = 1.0 - c;
  const double x = axis.x, y = axis.y, z = axis.z;
  const double r[3][3] = {{c + x * x * v, x * y * v - z * s, x * z * v + y * s},
                          {y * x * v + z * s, c + y * y * v, y * z * v - x * s},
                          {z * x * v - y * s, z * y * v + x * s, c + z * z * v}};
  ksl_SE3_t d;
  for(int row = 0; row < 3; row++) {
    for(int column = 0; column < 3; column++) {
      d.R.at[column][row] = r[row][column]; // columns v0, v1, v2
    }
  }
  d.t = t;
  return d;
}

/* p in the frame of d */
static ksl_vec3_t ToFrame(const ksl_SE3_t& d, const ksl_vec3_t& p) {
  const ksl_vec3_t r = {{p.x - d.t.x, p.y - d.t.y, p.z - d.t.z}};
  return {{Dot(d.R.v0, r), Dot(d.R.v1, r), Dot(d.R.v2, r)}};
}

/* bodies, each with one collision geometry, at rest on rigid joints */
class Shapes {
public:
  explicit Shapes(const std::string& name,
                  const std::map<std::string, std::string>& shapes,
                  const std::string& extra_bodies = "",
                  const std::string& extra_joints = "") {
    std::ostringstream m;
    m << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 9.81], "include_directories": [")"
      << SHIFT_EXAMPLES_DIR << R"(/cad"],
      "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "time_span": 0.0,
                       "time_step": 0.01}]},
      "assemblies": [{"name": "m", "bodies": [)";
    std::string joints;
    bool first = true;
    for(const auto& [shape, geometry] : shapes) {
      m << (first ? "" : ",") << R"({"name": ")" << shape
        << R"(", "collision_geometry": [{"name": ")" << shape << R"(", )"
        << geometry << R"(}], "center_of_mass": [0, 0, 0],
         "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                     "Iyz": 0, "Izx": 0}})";
      joints += std::string(first ? "" : ",") + R"({"name": ")" + shape +
                R"(_joint", "type": "float", "parent": [["fixed"], "origin"],
                "child": [[")" + shape + R"("], "origin"]})";
      first = false;
    }
    m << extra_bodies << R"(], "joints": [)" << joints << extra_joints
      << "]}]}";
    const std::string path = testing::TempDir() + "/" + name + ".json";
    {
      std::ofstream file(path);
      file << m.str();
    }
    context_.LoadJSON(path);
    std::remove(path.c_str());
    context_.UpdateD();
  }

  shift::core::collision_geometry::Base* Get(const std::string& name) {
    for(auto g : context_.get_active_collision_geometry()) {
      if(g->get_name() == name) {
        return g;
      }
    }
    ADD_FAILURE() << "no geometry " << name;
    return nullptr;
  }

private:
  shift::core::Context context_;
};

/* the shapes of the convex test, with their inside tests in their own frame */
struct Convex {
  std::string name;
  std::string json;
  std::function<bool(const ksl_vec3_t&)> inside;
};

static std::vector<Convex> ConvexShapes() {
  return {
    {"sphere", R"("type": "sphere", "radius": 0.4)",
     [](const ksl_vec3_t& p) { return Dot(p, p) <= 0.16; }},
    {"box", R"("type": "box", "half_extents": [0.3, 0.5, 0.2])",
     [](const ksl_vec3_t& p) {
       return std::fabs(p.x) <= 0.3 && std::fabs(p.y) <= 0.5 &&
              std::fabs(p.z) <= 0.2;
     }},
    {"ellipsoid", R"("type": "ellipsoid", "semi_axes": [0.5, 0.3, 0.2])",
     [](const ksl_vec3_t& p) {
       return p.x * p.x / 0.25 + p.y * p.y / 0.09 + p.z * p.z / 0.04 <= 1.0;
     }},
    {"capsule", R"("type": "capsule", "radius": 0.2, "half_length": 0.4)",
     [](const ksl_vec3_t& p) {
       const double z = std::clamp(p.z, -0.4, 0.4);
       return p.x * p.x + p.y * p.y + (p.z - z) * (p.z - z) <= 0.04;
     }},
    {"cylinder", R"("type": "cylinder", "radius": 0.3, "half_length": 0.25)",
     [](const ksl_vec3_t& p) {
       return p.x * p.x + p.y * p.y <= 0.09 && std::fabs(p.z) <= 0.25;
     }},
  };
}

/* the first point of the ray inside the shape, by bisection between samples
of the ray: the reference the casts are checked against */
static bool FirstInside(const std::function<bool(const ksl_vec3_t&)>& inside,
                        const ksl_SE3_t& d, const ksl_vec3_t& o,
                        const ksl_vec3_t& r, double max_distance,
                        double& distance) {
  const int samples = 20000;
  double previous = 0.0;
  for(int k = 0; k <= samples; k++) {
    const double t = max_distance * k / samples;
    if(inside(ToFrame(d, Along(o, t, r)))) {
      if(k == 0) {
        distance = 0.0;
        return true;
      }
      double lo = previous, hi = t;
      for(int i = 0; i < 60; i++) {
        const double mid = 0.5 * (lo + hi);
        (inside(ToFrame(d, Along(o, mid, r))) ? hi : lo) = mid;
      }
      distance = hi;
      return true;
    }
    previous = t;
  }
  return false;
}

/* rays from around each shape towards points near it, a fixed set from a
constant seed: the cast agrees with the inside test on where the ray first
enters, and its normal points out of the shape there */
TEST(RayCast, ConvexShapes) {
  std::map<std::string, std::string> json;
  for(const auto& shape : ConvexShapes()) {
    json[shape.name] = shape.json;
  }
  Shapes scene("ray_cast_convex", json);
  const ksl_vec3_t axis = {{1.0 / std::sqrt(14.0), 2.0 / std::sqrt(14.0),
                            3.0 / std::sqrt(14.0)}};
  const ksl_SE3_t d = Pose(axis, 0.7, {{0.3, -0.2, 0.5}});
  std::mt19937 random(20261107u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for(const auto& shape : ConvexShapes()) {
    auto* geometry = scene.Get(shape.name);
    ASSERT_NE(geometry, nullptr);
    /* the body origin pose that puts the geometry frame at d */
    geometry->UpdatePose(d);
    const ksl_SE3_t& frame = geometry->get_d_absolute();
    int hits = 0;
    for(int ray = 0; ray < 200; ray++) {
      ksl_vec3_t from = {{unit(random), unit(random), unit(random)}};
      const double length = std::sqrt(Dot(from, from));
      from = {{2.0 * from.x / length, 2.0 * from.y / length,
               2.0 * from.z / length}};
      const ksl_vec3_t to = {{0.6 * unit(random), 0.6 * unit(random),
                              0.6 * unit(random)}};
      ksl_vec3_t r = {{to.x - from.x, to.y - from.y, to.z - from.z}};
      const double norm = std::sqrt(Dot(r, r));
      r = {{r.x / norm, r.y / norm, r.z / norm}};
      const ksl_vec3_t o = Along(frame.t, 1.0, from);
      const double max_distance = 4.0;
      double expected = 0.0;
      const bool reaches =
        FirstInside(shape.inside, frame, o, r, max_distance, expected);
      double distance = -1.0;
      ksl_vec3_t normal;
      const bool hit = geometry->RayCast(o, r, max_distance, distance, normal);
      ASSERT_EQ(hit, reaches) << shape.name << " ray " << ray;
      if(!hit) {
        continue;
      }
      hits++;
      EXPECT_NEAR(distance, expected, 1e-6) << shape.name << " ray " << ray;
      EXPECT_NEAR(Dot(normal, normal), 1.0, 1e-9) << shape.name;
      EXPECT_LT(Dot(normal, r), 0.0) << shape.name << " ray " << ray;
      /* out of the shape along the normal, into it against the normal (not
      at an edge, where the normal is one of the faces') */
      const ksl_vec3_t p = Along(o, distance, r);
      EXPECT_FALSE(shape.inside(ToFrame(frame, Along(p, 1e-4, normal))))
        << shape.name << " ray " << ray;
      /* the analytic sphere and box agree with GJK on their support */
      if(shape.name == "sphere" || shape.name == "box") {
        double convex_distance;
        ksl_vec3_t convex_normal;
        ASSERT_TRUE(shift::core::collision_geometry::RayCastConvex(
          *geometry, o, r, max_distance, convex_distance, convex_normal));
        EXPECT_NEAR(convex_distance, distance, 1e-7) << shape.name;
      }
    }
    EXPECT_GT(hits, 40) << shape.name; // a share of the rays hit
  }
}

/* a ray from inside reaches the shape at once, against itself */
TEST(RayCast, StartsInside) {
  std::map<std::string, std::string> json;
  for(const auto& shape : ConvexShapes()) {
    json[shape.name] = shape.json;
  }
  Shapes scene("ray_cast_inside", json);
  const ksl_vec3_t r = {{0.0, 0.6, 0.8}};
  for(const auto& shape : ConvexShapes()) {
    auto* geometry = scene.Get(shape.name);
    ASSERT_NE(geometry, nullptr);
    geometry->UpdatePose(Pose({{0.0, 0.0, 1.0}}, 0.3, {{1.0, 2.0, 3.0}}));
    double distance = -1.0;
    ksl_vec3_t normal;
    ASSERT_TRUE(geometry->RayCast({{1.01, 2.0, 3.0}}, r, 1.0, distance, normal))
      << shape.name;
    EXPECT_EQ(distance, 0.0) << shape.name;
    EXPECT_NEAR(normal.y, -0.6, 1e-12) << shape.name;
    EXPECT_NEAR(normal.z, -0.8, 1e-12) << shape.name;
    /* along z to the center, 3 away: max_distance stops it short of the
    shape (at most 0.6 from its center along z) */
    EXPECT_FALSE(geometry->RayCast({{1.0, 2.0, 0.0}}, {{0.0, 0.0, 1.0}}, 2.3,
                                   distance, normal))
      << shape.name;
    ASSERT_TRUE(geometry->RayCast({{1.0, 2.0, 0.0}}, {{0.0, 0.0, 1.0}}, 3.0,
                                  distance, normal))
      << shape.name;
  }
}

/* a half space: from above along the normal, at a slant, parallel, and from
below */
TEST(RayCast, Plane) {
  Shapes scene("ray_cast_plane",
               {{"ground", R"("type": "plane", "normal": [0, 0, -1],
                             "offset": 0)"}});
  auto* plane = scene.Get("ground");
  ASSERT_NE(plane, nullptr);
  const ksl_SE3_t identity = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  plane->UpdatePose(identity);
  double distance;
  ksl_vec3_t normal;
  /* z down: the ground is z >= 0, free space above it at z < 0 */
  ASSERT_TRUE(plane->RayCast({{1.0, 2.0, -3.0}}, {{0.0, 0.0, 1.0}}, 10.0,
                             distance, normal));
  EXPECT_NEAR(distance, 3.0, 1e-12);
  EXPECT_NEAR(normal.z, -1.0, 1e-12);
  ASSERT_TRUE(plane->RayCast({{0.0, 0.0, -3.0}}, {{0.6, 0.0, 0.8}}, 10.0,
                             distance, normal));
  EXPECT_NEAR(distance, 3.75, 1e-12);
  EXPECT_FALSE(plane->RayCast({{0.0, 0.0, -3.0}}, {{0.6, 0.0, 0.8}}, 3.0,
                              distance, normal));
  EXPECT_FALSE(plane->RayCast({{0.0, 0.0, -3.0}}, {{1.0, 0.0, 0.0}}, 10.0,
                              distance, normal));
  EXPECT_FALSE(plane->RayCast({{0.0, 0.0, -3.0}}, {{0.0, 0.0, -1.0}}, 10.0,
                              distance, normal));
  ASSERT_TRUE(plane->RayCast({{0.0, 0.0, 1.0}}, {{1.0, 0.0, 0.0}}, 10.0,
                             distance, normal));
  EXPECT_EQ(distance, 0.0);
}

/* the exact faces of box_mm.step, from the origin to (0.01, 0.02, 0.03): rays
from all around against the slab test of the box, a miss, and a ray from
inside */
TEST(RayCast, CadBox) {
  Shapes scene("ray_cast_cad", {},
               R"({"name": "part", "geometry": [{"type": "cad",
                   "filename": "box_mm.step"}],
                  "collision_geometry": [{"name": "part", "type": "cad"}],
                  "center_of_mass": [0, 0, 0],
                  "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1,
                              "Ixy": 0, "Iyz": 0, "Izx": 0}})",
               R"({"name": "part_joint", "type": "float",
                   "parent": [["fixed"], "origin"],
                   "child": [["part"], "origin"]})");
  auto* part = scene.Get("part");
  ASSERT_NE(part, nullptr);
  const ksl_SE3_t body = Pose({{0.0, 0.6, 0.8}}, 0.5, {{0.1, 0.2, 0.3}});
  part->UpdatePose(body);
  const double size[3] = {0.01, 0.02, 0.03};
  auto inside = [&](const ksl_vec3_t& p) {
    return p.x >= 0.0 && p.x <= size[0] && p.y >= 0.0 && p.y <= size[1] &&
           p.z >= 0.0 && p.z <= size[2];
  };
  std::mt19937 random(7u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int hits = 0;
  for(int ray = 0; ray < 100; ray++) {
    /* in the body frame, from 0.1 m around the center of the box towards a
    point near it */
    ksl_vec3_t from = {{unit(random), unit(random), unit(random)}};
    const double length = std::sqrt(Dot(from, from));
    const ksl_vec3_t center = {{0.005, 0.01, 0.015}};
    ksl_vec3_t o_body = Along(center, 0.1 / length, from);
    ksl_vec3_t aim = {{center.x + 0.012 * unit(random),
                       center.y + 0.012 * unit(random),
                       center.z + 0.012 * unit(random)}};
    ksl_vec3_t r_body = {{aim.x - o_body.x, aim.y - o_body.y,
                          aim.z - o_body.z}};
    const double norm = std::sqrt(Dot(r_body, r_body));
    r_body = {{r_body.x / norm, r_body.y / norm, r_body.z / norm}};
    /* to the inertial frame */
    ksl_vec3_t o, r;
    for(int k = 0; k < 3; k++) {
      o.at[k] = body.t.at[k] + body.R.v0.at[k] * o_body.x +
                body.R.v1.at[k] * o_body.y + body.R.v2.at[k] * o_body.z;
      r.at[k] = body.R.v0.at[k] * r_body.x + body.R.v1.at[k] * r_body.y +
                body.R.v2.at[k] * r_body.z;
    }
    double expected;
    const bool reaches = FirstInside(inside, body, o, r, 0.2, expected);
    double distance = -1.0;
    ksl_vec3_t normal;
    const bool hit = part->RayCast(o, r, 0.2, distance, normal);
    ASSERT_EQ(hit, reaches) << "ray " << ray;
    if(!hit) {
      continue;
    }
    hits++;
    EXPECT_NEAR(distance, expected, 1e-9) << "ray " << ray;
    EXPECT_LT(Dot(normal, r), 0.0) << "ray " << ray;
    EXPECT_FALSE(inside(ToFrame(body, Along(Along(o, distance, r), 1e-5,
                                            normal))))
      << "ray " << ray;
  }
  EXPECT_GT(hits, 50);
  /* from the middle of the box: at once */
  ksl_vec3_t middle;
  for(int k = 0; k < 3; k++) {
    middle.at[k] = body.t.at[k] + body.R.v0.at[k] * 0.005 +
                   body.R.v1.at[k] * 0.01 + body.R.v2.at[k] * 0.015;
  }
  double distance;
  ksl_vec3_t normal;
  ASSERT_TRUE(part->RayCast(middle, body.R.v2, 1.0, distance, normal));
  EXPECT_EQ(distance, 0.0);
}

/* the first point at which a ray (frame of the geometry) reaches CAD
geometry, by brute force: on every face whose box the line crosses, Newton's
method for the ray's point on the surface from every sample of the face and
from a grid of 9 x 9 points over its parameters; a ray whose first point
leaves the solid (or leaves one where it enters another) starts inside, at 0.
HUGE_VAL if it does not reach the geometry within reach. */
static double BruteForceCast(
  const shift::core::collision_geometry::CadContactShape& shape,
  const ksl_vec3_t& o, const ksl_vec3_t& r, double reach) {
  const double tolerance = 1e-10 * shape.size, tie = 10.0 * tolerance;
  double best = HUGE_VAL;
  bool leaving = false;
  for(int face = 0; face < static_cast<int>(shape.faces.size()); face++) {
    const auto& f = shape.faces[face];
    /* the line against the face's box, padded */
    double t0 = -HUGE_VAL, t1 = HUGE_VAL;
    bool misses = false;
    for(int axis = 0; axis < 3; axis++) {
      const double lo = f.box.lo.at[axis] - 1e-6, hi = f.box.hi.at[axis] + 1e-6;
      if(std::fabs(r.at[axis]) < 1e-15) {
        misses = misses || o.at[axis] < lo || o.at[axis] > hi;
        continue;
      }
      double a = (lo - o.at[axis]) / r.at[axis], b = (hi - o.at[axis]) / r.at[axis];
      if(a > b) {
        std::swap(a, b);
      }
      t0 = std::max(t0, a);
      t1 = std::min(t1, b);
    }
    if(misses || t0 > t1 || t1 < 0.0 || t0 > reach) {
      continue;
    }
    std::vector<shift::core::cad::Vec2> starts(f.sample_uv.begin(),
                                               f.sample_uv.end());
    for(int u = 0; u <= 8; u++) {
      for(int v = 0; v <= 8; v++) {
        starts.push_back({f.lo[0] + (f.hi[0] - f.lo[0]) * u / 8.0,
                          f.lo[1] + (f.hi[1] - f.lo[1]) * v / 8.0});
      }
    }
    for(auto uv : starts) {
      const ksl_vec3_t p0 = shape.Point(face, uv);
      double t = Dot({{p0.x - o.x, p0.y - o.y, p0.z - o.z}}, r);
      bool converged = false;
      for(int iteration = 0; iteration < 40; iteration++) {
        ksl_vec3_t du, dv;
        const ksl_vec3_t p = shape.Point(face, uv, &du, &dv);
        const ksl_vec3_t e = {{p.x - o.x - t * r.x, p.y - o.y - t * r.y,
                               p.z - o.z - t * r.z}};
        if(Dot(e, e) <= tolerance * tolerance) {
          converged = true;
          break;
        }
        /* [du dv -r] (du_u, du_v, dt) = -e by Cramer's rule */
        const ksl_vec3_t c = {{-r.x, -r.y, -r.z}};
        auto det3 = [](const ksl_vec3_t& x, const ksl_vec3_t& y,
                       const ksl_vec3_t& z) {
          return x.x * (y.y * z.z - y.z * z.y) - y.x * (x.y * z.z - x.z * z.y) +
                 z.x * (x.y * y.z - x.z * y.y);
        };
        const double det = det3(du, dv, c);
        if(std::fabs(det) < 1e-300) {
          break;
        }
        const ksl_vec3_t rhs = {{-e.x, -e.y, -e.z}};
        uv[0] += det3(rhs, dv, c) / det;
        uv[1] += det3(du, rhs, c) / det;
        t += det3(du, dv, rhs) / det;
        if(!std::isfinite(uv[0] + uv[1] + t)) {
          break;
        }
      }
      if(!converged || t < -tolerance || t > reach ||
         t > std::max(best, 0.0) + tie || !shape.Inside(face, uv)) {
        continue;
      }
      const bool leaves = Dot(shape.Normal(face, uv), r) > 0.0;
      if(t < best - tie) {
        best = std::max(t, 0.0);
        leaving = leaves;
      } else if(leaves) {
        best = std::min(best, std::max(t, 0.0));
        leaving = true;
      }
    }
  }
  return best == HUGE_VAL ? HUGE_VAL : (leaving ? 0.0 : best);
}

/* a grid of parallel rays cast at once (Cad::RayCastGrid, from the faces
projected onto the grid) reaches the exact faces of a link of the excavator
track (its planes and the cylinders of its bushing, pin and plates), turned so
that rays cross the cylinders twice, with the rays starting at random heights,
some inside the link and some ending short of it: as the brute force cast
(BruteForceCast) on 300 random rays of each pose; and on every ray at least as
near as RayCast, or as the brute force where RayCast is nearer. RayCast starts
Newton's method only from the samples of a face nearest the ray, and misses
some crossings of the cylinders. */
TEST(RayCast, CadGridMatchesRays) {
  Shapes scene("ray_cast_grid", {},
               R"({"name": "link", "geometry": [{"type": "cad",
                   "filename": "track_link.step"}],
                  "collision_geometry": [{"name": "link", "type": "cad"}],
                  "center_of_mass": [0, 0, 0],
                  "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1,
                              "Ixy": 0, "Iyz": 0, "Izx": 0}})",
               R"({"name": "link_joint", "type": "float",
                   "parent": [["fixed"], "origin"],
                   "child": [["link"], "origin"]})");
  auto* link = dynamic_cast<shift::core::collision_geometry::Cad*>(
    scene.Get("link"));
  ASSERT_NE(link, nullptr);
  const auto& shape = link->get_shape();
  const ksl_vec3_t axes[3] = {{{0.0, 1.0, 0.0}}, {{0.0, 1.0, 0.0}},
                              {{0.6, 0.8, 0.0}}};
  const double angles[3] = {0.0, 0.3, 1.2};
  std::mt19937 random(20261107u);
  std::uniform_real_distribution<double> height(0.0, 0.35);
  for(int pose = 0; pose < 3; pose++) {
    const ksl_SE3_t d = Pose(axes[pose], angles[pose], {{-0.1, 0.0, -0.05}});
    link->UpdatePose(d);
    shift::core::collision_geometry::Base::RayGrid grid;
    /* off the round numbers of the link's edges, so that no ray runs along
    the edge of a face, where the casts may round to either side */
    grid.corner = {{-0.2513, -0.4017, 0.0}};
    grid.step_i = {{0.004, 0.0, 0.0}};
    grid.step_j = {{0.0, 0.004, 0.0}};
    grid.direction = {{0.0, 0.0, -1.0}};
    grid.count_i = 100;
    grid.count_j = 200;
    grid.reach = 0.2;
    std::vector<double> offsets(grid.count_i * grid.count_j);
    for(double& offset : offsets) {
      offset = height(random);
    }
    offsets[7] = NAN; // not cast
    grid.offsets = offsets.data();
    std::vector<double> distances(offsets.size());
    link->RayCastGrid(grid, distances.data());
    auto origin_of = [&](long k) {
      const long i = k % grid.count_i, j = k / grid.count_i;
      return ksl_vec3_t{{grid.corner.x + i * grid.step_i.x,
                         grid.corner.y + j * grid.step_j.y, offsets[k]}};
    };
    EXPECT_EQ(distances[7], HUGE_VAL);
    int hits = 0, inside = 0, later = 0, nearer = 0, ray_cast_wrong = 0;
    for(long k = 0; k < grid.count_i * grid.count_j; k++) {
      if(std::isnan(offsets[k])) {
        continue;
      }
      double distance;
      ksl_vec3_t normal;
      if(!link->RayCast(origin_of(k), grid.direction, grid.reach, distance,
                        normal)) {
        distance = HUGE_VAL;
      }
      hits += distances[k] != HUGE_VAL;
      inside += distances[k] == 0.0;
      if(distances[k] > distance + 1e-8) {
        /* RayCast found a nearer crossing: the brute force decides */
        const ksl_SE3_t& g = link->get_d_absolute();
        const ksl_vec3_t r = {{Dot(g.R.v0, grid.direction),
                               Dot(g.R.v1, grid.direction),
                               Dot(g.R.v2, grid.direction)}};
        const double expected =
          BruteForceCast(shape, ToFrame(g, origin_of(k)), r, grid.reach);
        const bool grid_right = expected == HUGE_VAL
                                  ? distances[k] == HUGE_VAL
                                  : std::fabs(distances[k] - expected) <= 1e-8;
        if(grid_right) {
          ray_cast_wrong++;
        } else if(later++ < 5) {
          ADD_FAILURE() << "pose " << pose << " ray " << k << ": grid "
                        << distances[k] << ", RayCast " << distance
                        << ", brute force " << expected;
        }
      }
      nearer += distances[k] < distance - 1e-8;
    }
    EXPECT_EQ(later, 0) << "pose " << pose;
    EXPECT_GT(hits, 1000) << "pose " << pose;
    EXPECT_GT(inside, 100) << "pose " << pose;
    std::uniform_int_distribution<long> pick(0, grid.count_i * grid.count_j - 1);
    int checked = 0, wrong = 0;
    for(int n = 0; n < 300; n++) {
      const long k = pick(random);
      if(std::isnan(offsets[k])) {
        continue;
      }
      /* in the frame of the geometry, at the center of the link's bounds */
      const ksl_SE3_t& g = link->get_d_absolute();
      const ksl_vec3_t o = ToFrame(g, origin_of(k));
      const ksl_vec3_t r = {{Dot(g.R.v0, grid.direction),
                             Dot(g.R.v1, grid.direction),
                             Dot(g.R.v2, grid.direction)}};
      const double expected = BruteForceCast(shape, o, r, grid.reach);
      checked++;
      const bool same = expected == HUGE_VAL
                          ? distances[k] == HUGE_VAL
                          : std::fabs(distances[k] - expected) <= 1e-8;
      if(!same && wrong++ < 5) {
        ADD_FAILURE() << "pose " << pose << " ray " << k << ": grid "
                      << distances[k] << ", brute force " << expected;
      }
    }
    EXPECT_EQ(wrong, 0) << "pose " << pose;
    std::printf("pose %d: %d hits, %d inside, %d nearer than RayCast, %d "
                "where RayCast is nearer but wrong, %d checked by brute "
                "force\n",
                pose, hits, inside, nearer, ray_cast_wrong, checked);
  }
}
