#include "shift/core/dem/cpu_solver.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "shift/core/exception.h"

/* the arithmetic of the shaders (dem_common.glsl), on floats */
struct Vec3 {
  float x, y, z;
};

static Vec3 operator+(Vec3 a, Vec3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
static Vec3 operator-(Vec3 a, Vec3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
static Vec3 operator-(Vec3 a) {
  return {-a.x, -a.y, -a.z};
}
static Vec3 operator*(float s, Vec3 a) {
  return {s * a.x, s * a.y, s * a.z};
}
static Vec3 operator*(Vec3 a, float s) {
  return {a.x * s, a.y * s, a.z * s};
}
static Vec3 operator*(Vec3 a, Vec3 b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}
static Vec3 operator/(Vec3 a, float s) {
  return {a.x / s, a.y / s, a.z / s};
}
static Vec3 operator/(Vec3 a, Vec3 b) {
  return {a.x / b.x, a.y / b.y, a.z / b.z};
}
static float Dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static float Length(Vec3 a) {
  return std::sqrt(Dot(a, a));
}
static Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static Vec3 V(const std::array<float, 4>& a) {
  return {a[0], a[1], a[2]};
}
static std::array<float, 4> A(Vec3 v, float w) {
  return {v.x, v.y, v.z, w};
}

struct Quat {
  Vec3 v;
  float w;
};

static Quat Q(const std::array<float, 4>& a) {
  return {{a[0], a[1], a[2]}, a[3]};
}

static Quat QuaternionProduct(Quat a, Quat b) {
  return {a.w * b.v + b.w * a.v + Cross(a.v, b.v), a.w * b.w - Dot(a.v, b.v)};
}

static Vec3 Rotate(Quat q, Vec3 v) {
  const Vec3 t = 2.0f * Cross(q.v, v);
  return v + q.w * t + Cross(q.v, t);
}

static Vec3 RotateBack(Quat q, Vec3 v) {
  return Rotate({-q.v, q.w}, v);
}

static const float kPi = 3.14159265f;

/* the vector from the point anchored at (b, y) to the one at (a, x) */
static Vec3 Relative(const std::array<int32_t, 4>& a, Vec3 x,
                     const std::array<int32_t, 4>& b, Vec3 y, float cell) {
  const Vec3 cells = {static_cast<float>(a[0] - b[0]),
                      static_cast<float>(a[1] - b[1]),
                      static_cast<float>(a[2] - b[2])};
  return cells * cell + (x - y);
}

static std::array<int32_t, 3> Cell(const std::array<int32_t, 4>& a, Vec3 x,
                                   float cell) {
  return {a[0] + static_cast<int32_t>(std::floor(x.x / cell)),
          a[1] + static_cast<int32_t>(std::floor(x.y / cell)),
          a[2] + static_cast<int32_t>(std::floor(x.z / cell))};
}

/* Law() of dem_common.glsl */
/* the damping coefficient c_n of Law at overlap d */
static float NormalDamping(const std::array<float, 8>& p, float r, float d,
                           float m) {
  return -2.0f * std::sqrt(5.0f / 6.0f) * p[2] *
         std::sqrt(2.0f * p[0] * std::sqrt(r * d) * m);
}

/* the moment on the first of a contact over a patch of spread rho^2 (the
mean square distance of its points from the point of contact, weighted as
they are): the normal damping spread over the patch resists the first's
rocking on the second, and with friction the tangential damping spread over
it resists its spinning about the normal, w being the second's angular
velocity less the first's */
static Vec3 Rocking(Vec3 n, float d, float r, float m, Vec3 w,
                    const std::array<float, 8>& p, float spread) {
  const float spin = Dot(w, n);
  Vec3 moment = (NormalDamping(p, r, d, m) * spread) * (w - spin * n);
  if(p[3] > 0.0f) {
    /* c_t of Law */
    const float kt = 8.0f * p[1] * std::sqrt(r * d);
    const float ct = -2.0f * std::sqrt(5.0f / 6.0f) * p[2] * std::sqrt(m * kt);
    moment = moment + (ct * spread * spin) * n;
  }
  return moment;
}

static void Law(Vec3 n, float d, float r, float m, Vec3 u, Vec3 w,
                const std::array<float, 8>& p, float dt,
                std::array<float, 4>& xi, Vec3& force, Vec3& rolling) {
  const float e = p[0], g = p[1], beta = p[2], mu = p[3], mu_r = p[4];
  const float rc = std::sqrt(r * d);
  const float un = Dot(n, u);
  const Vec3 ut = u - un * n;
  Vec3 history = V(xi) + dt * ut;
  history = history - Dot(history, n) * n;
  xi[3] = xi[3] + dt;
  const float kn = (4.0f / 3.0f) * e * rc;
  const float cn = NormalDamping(p, r, d, m);
  const float fn = kn * d - cn * un;
  force = fn * n;
  rolling = {0.0f, 0.0f, 0.0f};
  if(mu_r > 0.0f) {
    const float kns = (4.0f / 3.0f) * e * std::sqrt(r);
    const float gns =
      -2.0f * std::sqrt((5.0f / 3.0f) * m * e) * beta * std::pow(r, 0.25f);
    const float ratio = gns / (2.0f * std::sqrt(kns * m));
    bool apply = true;
    if(ratio < 1.0f) {
      const float collision =
        kPi * std::sqrt(m / (kns * (1.0f - ratio * ratio)));
      apply = xi[3] > collision;
    }
    const Vec3 rolling_w = w - Dot(w, n) * n;
    const float speed = Length(rolling_w);
    if(apply && speed > 1e-12f) {
      rolling = (rolling_w / speed) * (mu_r * std::abs(fn) * r);
    }
  }
  if(mu > 0.0f) {
    const float kt = 8.0f * g * rc;
    const float ct = -2.0f * std::sqrt(5.0f / 6.0f) * beta * std::sqrt(m * kt);
    Vec3 ft = -kt * history - ct * ut;
    const float size = Length(ft);
    const float most = mu * std::abs(fn);
    if(size > most) {
      ft = ft * (most / size);
      history = -(ft + ct * ut) / kt;
    }
    force = force + ft;
  }
  xi = A(history, xi[3]);
}

static const uint32_t kMaxWalls = 8;

/* ClosestPointOnTriangle() of dem_common.glsl */
static Vec3 ClosestPointOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
  const Vec3 ab = b - a, ac = c - a, ap = p - a;
  const float d1 = Dot(ab, ap), d2 = Dot(ac, ap);
  if(d1 <= 0.0f && d2 <= 0.0f) {
    return a;
  }
  const Vec3 bp = p - b;
  const float d3 = Dot(ab, bp), d4 = Dot(ac, bp);
  if(d3 >= 0.0f && d4 <= d3) {
    return b;
  }
  const float vc = d1 * d4 - d3 * d2;
  if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    return a + (d1 / (d1 - d3)) * ab;
  }
  const Vec3 cp = p - c;
  const float d5 = Dot(ab, cp), d6 = Dot(ac, cp);
  if(d6 >= 0.0f && d5 <= d6) {
    return c;
  }
  const float vb = d5 * d2 - d1 * d6;
  if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    return a + (d2 / (d2 - d6)) * ac;
  }
  const float va = d3 * d6 - d5 * d4;
  if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  }
  const float denominator = 1.0f / (va + vb + vc);
  return a + ab * (vb * denominator) + ac * (vc * denominator);
}

/* the convex shapes of contact of dem_hull.glsl: a sphere of radius
(count 0), a hull's vertices and planes at orientation q, centered on the
grain's center of mass, or a triangle of a body (triangle), its corners
from a point of it, with the unit normal of its corners in order */
struct Shape {
  Quat q;
  float radius;
  uint32_t first, count, first_plane, planes;
  const std::array<float, 4>* vertex;
  const std::array<float, 4>* plane;
  const std::array<uint32_t, 4>* face;
  bool triangle;
  Vec3 corner[3];
  Vec3 normal;
};

static const int kMprIterations = 32;
/* the most points of a clipped face: 12 and one more for each of 12 sides */
static const int kClipped = 32;

/* the point of shape farthest along d (from its center) */
static Vec3 Support(const Shape& s, Vec3 d) {
  if(s.count == 0) {
    return d * (s.radius / Length(d));
  }
  if(s.triangle) {
    Vec3 best = s.corner[0];
    for(int k = 1; k < 3; k++) {
      if(Dot(d, s.corner[k]) > Dot(d, best)) {
        best = s.corner[k];
      }
    }
    return best;
  }
  const Vec3 local = RotateBack(s.q, d);
  uint32_t best = s.first;
  float most = Dot(local, V(s.vertex[best]));
  for(uint32_t k = s.first + 1; k < s.first + s.count; k++) {
    const float h = Dot(local, V(s.vertex[k]));
    if(h > most) {
      most = h;
      best = k;
    }
  }
  return Rotate(s.q, V(s.vertex[best]));
}

/* the face of hull s most nearly along the unit d (from its center): its
plane, and how nearly (the cosine). A triangle has two faces, 0 along its
normal and 1 against it */
static uint32_t FaceAlong(const Shape& s, Vec3 d, float& alignment) {
  if(s.triangle) {
    const float a = Dot(d, s.normal);
    alignment = std::abs(a);
    return a >= 0.0f ? 0 : 1;
  }
  const Vec3 local = RotateBack(s.q, d);
  uint32_t best = s.first_plane;
  alignment = Dot(local, V(s.plane[best]));
  for(uint32_t k = s.first_plane + 1; k < s.first_plane + s.planes; k++) {
    const float a = Dot(local, V(s.plane[k]));
    if(a > alignment) {
      alignment = a;
      best = k;
    }
  }
  return best;
}

/* the outward unit normal of face f of s */
static Vec3 FaceNormal(const Shape& s, uint32_t f) {
  if(s.triangle) {
    return f == 0 ? s.normal : -s.normal;
  }
  return Rotate(s.q, V(s.plane[f]));
}

/* the sides of face f of s */
static uint32_t FaceSides(const Shape& s, uint32_t f) {
  return s.triangle ? 3 : s.face[f][0];
}

/* vertex k of face f of s, counterclockwise about its outward normal, from
the shape's center at c */
static Vec3 FaceVertex(const Shape& s, uint32_t f, uint32_t k, Vec3 c) {
  if(s.triangle) {
    return c + s.corner[f == 0 ? k : (3 - k) % 3];
  }
  const uint32_t index = (s.face[f][1 + k / 4] >> (8 * (k % 4))) & 0xffu;
  return c + Rotate(s.q, V(s.vertex[s.first + index]));
}

/* ClippedPoint() of dem_hull.glsl: the point of contact of hulls a at c and
b at 0 touching along n (from b to a): the face of each most nearly facing
the other; the better aligned of the two (b's on a tie) is the reference,
and the other, the incident face, is clipped against the planes through
the reference's sides (Sutherland and Hodgman); the mean of the clipped
points below the reference face, each weighted by its depth and taken
half that depth below the face, and the spread of the patch, the mean of
their squared distances from that point along the face, and the sum of
the depths (the weight of the point). False if none is below it */
static bool ClippedPoint(const Shape& a, const Shape& b, Vec3 c, Vec3 n,
                         Vec3& point, float& spread, float& weight) {
  float align_a, align_b;
  const uint32_t fa = FaceAlong(a, -n, align_a), fb = FaceAlong(b, n, align_b);
  const bool reference_b = align_b >= align_a;
  const Shape& r = reference_b ? b : a;
  const Shape& i = reference_b ? a : b;
  const Vec3 zero = {0.0f, 0.0f, 0.0f};
  const Vec3 cr = reference_b ? zero : c, ci = reference_b ? c : zero;
  const uint32_t fr = reference_b ? fb : fa, fi = reference_b ? fa : fb;
  Vec3 polygon[2][kClipped];
  int count = static_cast<int>(FaceSides(i, fi)), current = 0;
  for(int k = 0; k < count; k++) {
    polygon[0][k] = FaceVertex(i, fi, uint32_t(k), ci);
  }
  const Vec3 normal = FaceNormal(r, fr);
  const uint32_t sides = FaceSides(r, fr);
  for(uint32_t e = 0; e < sides && count > 0; e++) {
    const Vec3 p = FaceVertex(r, fr, e, cr);
    const Vec3 q = FaceVertex(r, fr, (e + 1) % sides, cr);
    const Vec3 side = Cross(q - p, normal);
    int next = 0;
    for(int k = 0; k < count; k++) {
      const Vec3 u = polygon[current][k];
      const Vec3 v = polygon[current][(k + 1) % count];
      const float du = Dot(side, u - p), dv = Dot(side, v - p);
      if(du <= 0.0f && next < kClipped) {
        polygon[1 - current][next++] = u;
      }
      if((du <= 0.0f) != (dv <= 0.0f) && next < kClipped) {
        polygon[1 - current][next++] = u + (du / (du - dv)) * (v - u);
      }
    }
    current = 1 - current;
    count = next;
  }
  const Vec3 base = FaceVertex(r, fr, 0, cr);
  Vec3 sum = {0.0f, 0.0f, 0.0f};
  float total = 0.0f;
  for(int k = 0; k < count; k++) {
    const Vec3 x = polygon[current][k];
    const float depth = Dot(normal, base - x);
    if(depth > 0.0f) {
      sum = sum + depth * (x + (0.5f * depth) * normal);
      total += depth;
    }
  }
  if(!(total > 0.0f)) {
    return false;
  }
  point = sum / total;
  weight = total;
  spread = 0.0f;
  for(int k = 0; k < count; k++) {
    const Vec3 x = polygon[current][k];
    const float depth = Dot(normal, base - x);
    if(depth > 0.0f) {
      const Vec3 arm = x - point;
      const Vec3 tangent = arm - Dot(arm, normal) * normal;
      spread += depth * Dot(tangent, tangent);
    }
  }
  spread = spread / total;
  return true;
}

/* the support of the Minkowski difference of a at c less b at 0 */
static Vec3 Minkowski(const Shape& a, const Shape& b, Vec3 c, Vec3 d) {
  return (c + Support(a, d)) - Support(b, -d);
}

/* v / |v|, false if |v| is not above tiny */
static bool Unit(Vec3 v, float tiny, Vec3& u) {
  const float size = Length(v);
  if(!(size > tiny)) {
    return false;
  }
  u = v / size;
  return true;
}

/* Collide() of dem_hull.glsl: whether shape a, centered at c from b's
center, overlaps b; then the depth, the unit normal from b to a and the
point of contact from b's center, the spread of the patch (ClippedPoint; 0
for a point) and the weight of the point (ClippedPoint; else the depth). Minkowski portal refinement (Snethen,
XenoCollide, Game Programming Gems 7, 2008, as libccd's ccdMPRPenetration
and Chrono's ChNarrowphaseMPR) on a - b, from the interior point c: a
portal (v1, v2, v3) that the ray from c through the origin passes, refined
until the support beyond it is within the tolerance; the depth is the
distance of the origin from the portal's plane. The point is that of the
faces in contact of two hulls or a hull and a triangle (ClippedPoint), or
else the midpoint of the two deepest points */
static bool Collide(const Shape& a, const Shape& b, Vec3 c, float scale,
                    Vec3& n, float& depth, Vec3& point, float& spread,
                    float& weight) {
  spread = 0.0f;
  const float tiny = 1e-12f * scale, tolerance = 1e-6f * scale;
  Vec3 v0 = c;
  if(!(Length(v0) > tiny)) {
    v0 = {tolerance, 0.0f, 0.0f};
  }
  Vec3 dir;
  if(!Unit(-v0, 0.0f, dir)) {
    return false;
  }
  Vec3 v1 = Minkowski(a, b, c, dir);
  if(!(Dot(v1, dir) > 0.0f)) {
    return false;
  }
  if(!Unit(Cross(v0, v1), tiny * scale, dir)) {
    /* the origin on the line of v0 and v1 */
    depth = Length(v1);
    if(!Unit(v0, 0.0f, n)) {
      return false;
    }
    point = 0.5f * ((c + Support(a, -n)) + Support(b, n));
    weight = depth;
    return depth > 0.0f;
  }
  Vec3 v2 = Minkowski(a, b, c, dir);
  if(!(Dot(v2, dir) > 0.0f)) {
    return false;
  }
  if(!Unit(Cross(v1 - v0, v2 - v0), tiny * scale, dir)) {
    return false;
  }
  if(Dot(dir, v0) > 0.0f) {
    std::swap(v1, v2);
    dir = -dir;
  }
  Vec3 v3;
  int k = 0;
  for(; k < kMprIterations; k++) {
    v3 = Minkowski(a, b, c, dir);
    if(!(Dot(v3, dir) > 0.0f)) {
      return false;
    }
    if(Dot(Cross(v1, v3), v0) < 0.0f) {
      v2 = v3;
      if(!Unit(Cross(v1 - v0, v3 - v0), tiny * scale, dir)) {
        return false;
      }
      continue;
    }
    if(Dot(Cross(v3, v2), v0) < 0.0f) {
      v1 = v3;
      if(!Unit(Cross(v3 - v0, v2 - v0), tiny * scale, dir)) {
        return false;
      }
      continue;
    }
    break;
  }
  if(k == kMprIterations) {
    return false;
  }
  /* refine the portal toward the boundary */
  for(k = 0; k < kMprIterations; k++) {
    if(!Unit(Cross(v2 - v1, v3 - v1), tiny * scale, dir)) {
      return false;
    }
    const Vec3 v4 = Minkowski(a, b, c, dir);
    const float d4 = Dot(v4, dir);
    if(!(d4 > 0.0f)) {
      return false;
    }
    const float gap = std::min(d4 - Dot(v1, dir),
                               std::min(d4 - Dot(v2, dir), d4 - Dot(v3, dir)));
    if(gap <= tolerance) {
      break;
    }
    const Vec3 x = Cross(v4, v0);
    if(Dot(v1, x) > 0.0f) {
      if(Dot(v2, x) > 0.0f) {
        v1 = v4;
      } else {
        v3 = v4;
      }
    } else {
      if(Dot(v3, x) > 0.0f) {
        v2 = v4;
      } else {
        v1 = v4;
      }
    }
  }
  depth = Dot(v1, dir);
  if(!(depth > 0.0f)) {
    return false;
  }
  n = -dir;
  /* the point: of the faces in contact, or else the midpoint of the
  deepest points */
  if(a.count == 0 || b.count == 0 ||
     !ClippedPoint(a, b, c, n, point, spread, weight)) {
    spread = 0.0f;
    point = 0.5f * ((c + Support(a, -n)) + Support(b, n));
    weight = depth;
  }
  return true;
}

/* the shape of contact of sphere s of grain g and template sphere k: its
sphere of radius, or the grain's hull */
static Shape ShapeOf(const std::array<uint32_t, 4>& hull, float radius,
                     const std::array<float, 4>& quaternion,
                     const std::vector<std::array<float, 4>>& vertex,
                     const std::vector<std::array<float, 4>>& plane,
                     const std::vector<std::array<uint32_t, 4>>& face) {
  Shape shape;
  shape.q = Q(quaternion);
  shape.radius = hull[1] > 0 ? vertex[hull[0]][3] : radius;
  shape.first = hull[0];
  shape.count = hull[1];
  shape.first_plane = hull[2];
  shape.planes = hull[3];
  shape.vertex = vertex.data();
  shape.plane = plane.data();
  shape.face = face.data();
  shape.triangle = false;
  return shape;
}

static uint32_t Bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, 4);
  return u;
}

static int32_t IntBits(float f) {
  int32_t i;
  std::memcpy(&i, &f, 4);
  return i;
}

shift::core::dem::CpuSolver::CpuSolver(const Settings& settings,
                                       const std::vector<Material>& materials,
                                       const std::vector<Template>& templates,
                                       const State& state,
                                       const std::vector<Body>& bodies)
  : Solver(settings, materials, templates, state, bodies) {
  const size_t grains = state.size(), spheres = layout_.spheres;
  velocity_.resize(grains);
  omega_.resize(grains);
  quaternion_.resize(grains);
  for(size_t g = 0; g < grains; g++) {
    for(int k = 0; k < 3; k++) {
      velocity_[g][k] = state.velocity[g][k];
      omega_[g][k] = state.angular_velocity[g][k];
    }
    velocity_[g][3] = omega_[g][3] = 0.0f;
    quaternion_[g] = state.orientation[g];
  }
  sphere_anchor_.resize(spheres);
  reference_anchor_.assign(spheres, {0, 0, 0, 0});
  sphere_offset_.resize(spheres);
  sphere_arm_.resize(spheres);
  sphere_velocity_.resize(spheres);
  sphere_omega_.resize(spheres);
  reference_offset_.assign(spheres, {0.0f, 0.0f, 0.0f, 0.0f});
  sphere_material_.resize(spheres);
  neighbor_count_.assign(spheres, 0);
  neighbor_.assign(spheres * settings.max_neighbors, 0);
  history_.assign(spheres * settings.max_neighbors, {0.0f, 0.0f, 0.0f, 0.0f});
  wall_history_.assign(spheres * kMaxWalls, {0.0f, 0.0f, 0.0f, 0.0f});
  sphere_force_.resize(spheres);
  sphere_torque_.resize(spheres);
  const size_t triangles = layout_.triangles;
  triangle_anchor_.resize(triangles);
  reference_triangle_anchor_.assign(triangles, {0, 0, 0, 0});
  triangle_vertex_.resize(3 * triangles);
  reference_triangle_vertex_.assign(3 * triangles, {0.0f, 0.0f, 0.0f, 0.0f});
  triangle_count_.assign(spheres, 0);
  triangle_list_.assign(spheres * settings.max_triangles, 0);
  triangle_history_.assign(spheres * settings.max_triangles,
                           {0.0f, 0.0f, 0.0f, 0.0f});
  impulse_.assign(6 * bodies.size() * spheres, 0.0f);
}

bool shift::core::dem::CpuSolver::Triangles() {
  const Layout& l = layout_;
  bool search = false;
  for(uint32_t t = 0; t < l.triangles; t++) {
    const uint32_t b = Bits(l.triangle_local[3 * t][3]);
    const std::array<float, 4>* row = &poses_[7 * b];
    const std::array<int32_t, 4> anchor = {IntBits(row[0][0]),
                                           IntBits(row[0][1]),
                                           IntBits(row[0][2]),
                                           static_cast<int32_t>(b)};
    const Vec3 offset = V(row[1]);
    const Vec3 r0 = V(row[2]), r1 = V(row[3]), r2 = V(row[4]);
    triangle_anchor_[t] = anchor;
    float moved = 0.0f;
    for(uint32_t k = 0; k < 3; k++) {
      const std::array<float, 4>& local = l.triangle_local[3 * t + k];
      const Vec3 x =
        offset + ((r0 * local[0] + r1 * local[1]) + r2 * local[2]);
      triangle_vertex_[3 * t + k] = A(x, local[3]);
      moved = std::max(moved,
                       Length(Relative(anchor, x, reference_triangle_anchor_[t],
                                       V(reference_triangle_vertex_[3 * t + k]),
                                       l.cell)));
    }
    if(2.0f * moved >= l.margin) {
      search = true;
    }
  }
  return search;
}

/* the triangles near each sphere, as dem_triangle_count, the scan, scatter,
cell sort and the triangles' part of dem_neighbors do it */
void shift::core::dem::CpuSolver::SearchTriangles() {
  const Layout& l = layout_;
  const uint32_t M = settings_.max_triangles;
  const size_t table = size_t(l.triangle_mask) + 1;
  std::vector<uint32_t> hashes(8 * size_t(l.triangles), 0xffffffffu),
    start(table + 1, 0);
  bool large = false;
  for(uint32_t t = 0; t < l.triangles; t++) {
    const std::array<int32_t, 4>& a = triangle_anchor_[t];
    std::array<int32_t, 3> lo = Cell(a, V(triangle_vertex_[3 * t]), l.cell),
                           hi = lo;
    for(uint32_t k = 1; k < 3; k++) {
      const std::array<int32_t, 3> c =
        Cell(a, V(triangle_vertex_[3 * t + k]), l.cell);
      for(int i = 0; i < 3; i++) {
        lo[i] = std::min(lo[i], c[i]);
        hi[i] = std::max(hi[i], c[i]);
      }
    }
    if((hi[0] - lo[0] + 1) * (hi[1] - lo[1] + 1) * (hi[2] - lo[2] + 1) > 8) {
      large = true;
    }
    uint32_t slot = 0;
    for(int32_t z = lo[2]; z <= hi[2]; z++) {
      for(int32_t y = lo[1]; y <= hi[1]; y++) {
        for(int32_t x = lo[0]; x <= hi[0]; x++) {
          if(slot < 8) {
            const uint32_t hash = CellHash({x, y, z}, l.triangle_mask);
            hashes[8 * t + slot] = hash;
            start[hash + 1]++;
            slot++;
          }
        }
      }
    }
  }
  if(large) {
    throw shift::core::Exception(
      "DEM: a triangle is in more than 8 cells of the grid");
  }
  for(size_t b = 0; b < table; b++) {
    start[b + 1] += start[b];
  }
  std::vector<uint32_t> order(start[table]);
  std::vector<uint32_t> cursor(start.begin(), start.end() - 1);
  for(uint32_t k = 0; k < hashes.size(); k++) {
    if(hashes[k] != 0xffffffffu) {
      order[cursor[hashes[k]]++] = k;
    }
  }
  uint32_t most = 0;
  bool overflow = false;
  std::vector<uint32_t> found;
  std::vector<std::array<float, 4>> kept;
  for(uint32_t s = 0; s < l.spheres; s++) {
    const std::array<int32_t, 4>& a = sphere_anchor_[s];
    const Vec3 x = V(sphere_offset_[s]);
    const std::array<int32_t, 3> c = Cell(a, x, l.cell);
    uint32_t cells[27];
    int n_cells = 0;
    for(int dz = -1; dz <= 1; dz++) {
      for(int dy = -1; dy <= 1; dy++) {
        for(int dx = -1; dx <= 1; dx++) {
          const uint32_t hash =
            CellHash({c[0] + dx, c[1] + dy, c[2] + dz}, l.triangle_mask);
          if(std::find(cells, cells + n_cells, hash) == cells + n_cells) {
            cells[n_cells++] = hash;
          }
        }
      }
    }
    found.clear();
    uint32_t total = 0;
    for(int ci = 0; ci < n_cells; ci++) {
      for(uint32_t k = start[cells[ci]]; k < start[cells[ci] + 1]; k++) {
        const uint32_t t = order[k] / 8;
        if(std::find(found.begin(), found.end(), t) != found.end()) {
          continue;
        }
        const Vec3 p = Relative(a, x, triangle_anchor_[t], {0.0f, 0.0f, 0.0f},
                                l.cell);
        const Vec3 q = ClosestPointOnTriangle(p, V(triangle_vertex_[3 * t]),
                                              V(triangle_vertex_[3 * t + 1]),
                                              V(triangle_vertex_[3 * t + 2]));
        if(Length(p - q) < sphere_offset_[s][3] + l.margin) {
          if(found.size() < M) {
            found.push_back(t);
          } else {
            overflow = true;
          }
          total++;
        }
      }
    }
    std::sort(found.begin(), found.end());
    const uint32_t base = s * M, old = triangle_count_[s];
    kept.assign(found.size(), {0.0f, 0.0f, 0.0f, 0.0f});
    uint32_t j = 0;
    for(size_t k = 0; k < found.size(); k++) {
      while(j < old && triangle_list_[base + j] < found[k]) {
        j++;
      }
      if(j < old && triangle_list_[base + j] == found[k]) {
        kept[k] = triangle_history_[base + j];
      }
    }
    for(size_t k = 0; k < found.size(); k++) {
      triangle_list_[base + k] = found[k];
      triangle_history_[base + k] = kept[k];
    }
    triangle_count_[s] = static_cast<uint32_t>(found.size());
    most = std::max(most, total);
  }
  for(uint32_t t = 0; t < l.triangles; t++) {
    reference_triangle_anchor_[t] = triangle_anchor_[t];
    for(uint32_t k = 0; k < 3; k++) {
      reference_triangle_vertex_[3 * t + k] = triangle_vertex_[3 * t + k];
    }
  }
  statistics_.most_triangles = std::max(statistics_.most_triangles, most);
  if(overflow) {
    throw shift::core::Exception(
      "DEM: a sphere is near " + std::to_string(most) +
      " triangles, more than max_triangles (" + std::to_string(M) + ")");
  }
}

void shift::core::dem::CpuSolver::TriangleForces(uint32_t s, float dt,
                                                 float force_out[3],
                                                 float torque_out[3]) {
  const Layout& l = layout_;
  if(l.template_hull[l.sphere_info[s][1]][1] > 0) {
    HullTriangleForces(s, dt, force_out, torque_out);
    return;
  }
  const uint32_t M = settings_.max_triangles;
  const uint32_t B = static_cast<uint32_t>(bodies_.size());
  const std::array<int32_t, 4>& a = sphere_anchor_[s];
  const std::array<float, 4>& x = sphere_offset_[s];
  Vec3 force = {force_out[0], force_out[1], force_out[2]};
  Vec3 torque = {torque_out[0], torque_out[1], torque_out[2]};
  Vec3 touched[32];
  uint32_t touched_body[32];
  uint32_t touches = 0;
  const uint32_t base = s * M;
  for(uint32_t k = 0; k < triangle_count_[s]; k++) {
    const uint32_t t = triangle_list_[base + k];
    const std::array<int32_t, 4>& ta = triangle_anchor_[t];
    const uint32_t b = static_cast<uint32_t>(ta[3]);
    const Vec3 p = Relative(a, V(x), ta, {0.0f, 0.0f, 0.0f}, l.cell);
    const std::array<float, 4>& v1 = triangle_vertex_[3 * t + 1];
    const Vec3 q = ClosestPointOnTriangle(p, V(triangle_vertex_[3 * t]), V(v1),
                                          V(triangle_vertex_[3 * t + 2]));
    const Vec3 gap = p - q;
    const float distance = Length(gap);
    const float d = x[3] - distance;
    bool repeated = false;
    for(uint32_t j = 0; j < touches; j++) {
      repeated = repeated || (touched_body[j] == b &&
                              Length(touched[j] - q) < 1e-3f * x[3]);
    }
    if(!(d > 0.0f) || !(distance > 0.0f) || repeated) {
      triangle_history_[base + k] = {0.0f, 0.0f, 0.0f, 0.0f};
      continue;
    }
    touched[touches] = q;
    touched_body[touches] = b;
    touches++;
    std::array<float, 4> xi = triangle_history_[base + k];
    const Vec3 n = gap / distance;
    const std::array<float, 4>* row = &poses_[7 * b];
    const Vec3 lb = (q - 0.5f * d * n) - V(row[1]);
    const Vec3 vb = V(row[5]) + Cross(V(row[6]), lb);
    const std::array<float, 4>& va = sphere_velocity_[s];
    const Vec3 wa = V(sphere_omega_[s]);
    const Vec3 la = V(sphere_arm_[s]) - (x[3] - 0.5f * d) * n;
    const Vec3 u = (V(va) + Cross(wa, la)) - vb;
    const uint32_t pair = sphere_material_[s] * l.materials + Bits(v1[3]);
    Vec3 f, rolling;
    Law(n, d, x[3], va[3], u, V(row[6]) - wa, l.pairs[pair], dt, xi, f,
        rolling);
    triangle_history_[base + k] = xi;
    force = force + f;
    torque = torque + (Cross(la, f) + rolling);
    const Vec3 moment = Cross(lb, -f) - rolling;
    const float values[6] = {-f.x, -f.y, -f.z, moment.x, moment.y, moment.z};
    for(uint32_t c = 0; c < 6; c++) {
      float& slot = impulse_[(size_t(c) * B + b) * l.spheres + s];
      slot = slot + dt * values[c];
    }
  }
  force_out[0] = force.x;
  force_out[1] = force.y;
  force_out[2] = force.z;
  torque_out[0] = torque.x;
  torque_out[1] = torque.y;
  torque_out[2] = torque.z;
}

/* the patches of a hull's contact with the triangles of bodies: those of
one body whose normals are within kSamePatch (the cosine of 15 degrees) are
one contact, so that a hull across several triangles of a face, or of a
gently curved surface, is pushed once */
static const uint32_t kPatches = 8;
static const float kSamePatch = 0.966f;

struct Patch {
  uint32_t body, slot;
  Vec3 n;
  float depth, weight;
  Vec3 sum;     /* of the points (from the hull's center) by weight */
  float second; /* of the spreads and the squared points by weight */
  std::array<float, 4> history;
};

static bool Zero(const std::array<float, 4>& a) {
  return a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f && a[3] == 0.0f;
}

/* the contacts of a hull with the triangles of bodies: the overlap of the
hull and each triangle (Collide, from the triangle's point nearest the
hull's center), joined into patches; each patch at its deepest normal and
depth, at the mean of its points, with the spread of all of them. A patch
keeps its history in the slot of its first triangle, taking that of the
first of its triangles that has one */
void shift::core::dem::CpuSolver::HullTriangleForces(uint32_t s, float dt,
                                                     float force_out[3],
                                                     float torque_out[3]) {
  const Layout& l = layout_;
  const uint32_t M = settings_.max_triangles;
  const uint32_t B = static_cast<uint32_t>(bodies_.size());
  const std::array<int32_t, 4>& a = sphere_anchor_[s];
  const std::array<float, 4>& x = sphere_offset_[s];
  const Shape shape = ShapeOf(l.template_hull[l.sphere_info[s][1]], x[3],
                              quaternion_[l.sphere_info[s][0]], l.hull_vertex,
                              l.hull_plane, l.hull_face);
  Vec3 force = {force_out[0], force_out[1], force_out[2]};
  Vec3 torque = {torque_out[0], torque_out[1], torque_out[2]};
  Patch patches[kPatches];
  uint32_t count = 0;
  const uint32_t base = s * M;
  for(uint32_t k = 0; k < triangle_count_[s]; k++) {
    const uint32_t t = triangle_list_[base + k];
    const std::array<int32_t, 4>& ta = triangle_anchor_[t];
    const uint32_t b = static_cast<uint32_t>(ta[3]);
    const Vec3 p = Relative(a, V(x), ta, {0.0f, 0.0f, 0.0f}, l.cell);
    const Vec3 c0 = V(triangle_vertex_[3 * t]);
    const Vec3 c1 = V(triangle_vertex_[3 * t + 1]);
    const Vec3 c2 = V(triangle_vertex_[3 * t + 2]);
    const Vec3 q = ClosestPointOnTriangle(p, c0, c1, c2);
    const std::array<float, 4> history = triangle_history_[base + k];
    triangle_history_[base + k] = {0.0f, 0.0f, 0.0f, 0.0f};
    Shape triangle = shape;
    triangle.triangle = true;
    triangle.count = 3;
    triangle.corner[0] = c0 - q;
    triangle.corner[1] = c1 - q;
    triangle.corner[2] = c2 - q;
    Vec3 n, point;
    float d, spread, weight;
    if(!(Length(p - q) < x[3]) ||
       !Unit(Cross(c1 - c0, c2 - c0), 0.0f, triangle.normal) ||
       !Collide(shape, triangle, p - q, x[3], n, d, point, spread, weight)) {
      continue;
    }
    /* the point from the hull's center */
    point = point - (p - q);
    uint32_t j = 0;
    float best = -2.0f;
    for(uint32_t i = 0; i < count; i++) {
      const float alignment = Dot(n, patches[i].n);
      if(patches[i].body == b && alignment > best) {
        best = alignment;
        j = i;
      }
    }
    if(!(best > kSamePatch) && count < kPatches) {
      j = count++;
      patches[j] = {b,     base + k, n, d, 0.0f, {0.0f, 0.0f, 0.0f},
                    0.0f,  history};
    } else if(!(best > -2.0f)) {
      continue;
    }
    Patch& patch = patches[j];
    if(d > patch.depth) {
      patch.n = n;
      patch.depth = d;
    }
    patch.weight += weight;
    patch.sum = patch.sum + weight * point;
    patch.second += weight * (spread + Dot(point, point));
    if(Zero(patch.history)) {
      patch.history = history;
    }
  }
  const std::array<float, 4>& va = sphere_velocity_[s];
  const Vec3 wa = V(sphere_omega_[s]);
  for(uint32_t i = 0; i < count; i++) {
    const Patch& patch = patches[i];
    const Vec3 n = patch.n;
    const float d = patch.depth;
    const Vec3 point = patch.sum / patch.weight;
    const float spread =
      std::max(patch.second / patch.weight - Dot(point, point), 0.0f);
    const uint32_t b = patch.body;
    const uint32_t t = triangle_list_[patch.slot];
    const std::array<float, 4>* row = &poses_[7 * b];
    const Vec3 p =
      Relative(a, V(x), triangle_anchor_[t], {0.0f, 0.0f, 0.0f}, l.cell);
    const Vec3 lb = (p + point) - V(row[1]);
    const Vec3 vb = V(row[5]) + Cross(V(row[6]), lb);
    const Vec3 la = V(sphere_arm_[s]) + point;
    const Vec3 u = (V(va) + Cross(wa, la)) - vb;
    const uint32_t pair = sphere_material_[s] * l.materials +
                          Bits(triangle_vertex_[3 * t + 1][3]);
    std::array<float, 4> xi = patch.history;
    Vec3 f, rolling;
    Law(n, d, shape.radius, va[3], u, V(row[6]) - wa, l.pairs[pair], dt, xi,
        f, rolling);
    if(spread > 0.0f) {
      rolling = rolling + Rocking(n, d, shape.radius, va[3], V(row[6]) - wa,
                                  l.pairs[pair], spread);
    }
    triangle_history_[patch.slot] = xi;
    force = force + f;
    torque = torque + (Cross(la, f) + rolling);
    const Vec3 moment = Cross(lb, -f) - rolling;
    const float values[6] = {-f.x, -f.y, -f.z, moment.x, moment.y, moment.z};
    for(uint32_t c = 0; c < 6; c++) {
      float& slot = impulse_[(size_t(c) * B + b) * l.spheres + s];
      slot = slot + dt * values[c];
    }
  }
  force_out[0] = force.x;
  force_out[1] = force.y;
  force_out[2] = force.z;
  torque_out[0] = torque.x;
  torque_out[1] = torque.y;
  torque_out[2] = torque.z;
}

void shift::core::dem::CpuSolver::ClearImpulses() {
  std::fill(impulse_.begin(), impulse_.end(), 0.0f);
}

void shift::core::dem::CpuSolver::GetImpulses(
  std::vector<std::array<double, 6>>& impulses) {
  const size_t B = bodies_.size(), S = layout_.spheres;
  impulses.assign(B, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  for(size_t c = 0; c < 6; c++) {
    for(size_t b = 0; b < B; b++) {
      double sum = 0.0;
      for(size_t s = 0; s < S; s++) {
        sum += impulse_[(c * B + b) * S + s];
      }
      impulses[b][c] = sum;
    }
  }
}

bool shift::core::dem::CpuSolver::Spheres() {
  const Layout& l = layout_;
  bool search = false;
  for(uint32_t s = 0; s < l.spheres; s++) {
    const uint32_t g = l.sphere_info[s][0], k = l.sphere_info[s][1];
    const std::array<int32_t, 4>& a = l.anchor[g];
    const Quat q = Q(quaternion_[g]);
    const std::array<float, 4>& local = l.template_sphere[k];
    const Vec3 arm = Rotate(q, V(local));
    const Vec3 x = V(l.offset[g]) + arm;
    sphere_anchor_[s] = {a[0], a[1], a[2], static_cast<int32_t>(g)};
    sphere_offset_[s] = A(x, local[3]);
    sphere_material_[s] = l.template_material[k];
    float bits;
    std::memcpy(&bits, &sphere_material_[s], 4);
    sphere_arm_[s] = A(arm, bits);
    sphere_velocity_[s] = A(V(velocity_[g]), l.template_mass[a[3]][0]);
    sphere_omega_[s] = A(Rotate(q, V(omega_[g])), 0.0f);
    const Vec3 moved = Relative(sphere_anchor_[s], x, reference_anchor_[s],
                                V(reference_offset_[s]), l.cell);
    if(2.0f * Length(moved) >= l.margin) {
      search = true;
    }
  }
  return search;
}

void shift::core::dem::CpuSolver::Search() {
  const Layout& l = layout_;
  const uint32_t K = settings_.max_neighbors;
  /* sort by cell, then index (the GPU's counting sort gives this order) */
  const size_t table = size_t(l.mask) + 1;
  std::vector<uint32_t> hashes(l.spheres), start(table + 1, 0), order;
  for(uint32_t s = 0; s < l.spheres; s++) {
    hashes[s] = CellHash(
      Cell(sphere_anchor_[s], V(sphere_offset_[s]), l.cell), l.mask);
    start[hashes[s] + 1]++;
  }
  for(size_t b = 0; b < table; b++) {
    start[b + 1] += start[b];
  }
  order.resize(l.spheres);
  std::vector<uint32_t> cursor(start.begin(), start.end() - 1);
  for(uint32_t s = 0; s < l.spheres; s++) {
    order[cursor[hashes[s]]++] = s;
  }
  uint32_t most = 0;
  bool overflow = false;
  std::vector<uint32_t> found;
  std::vector<std::array<float, 4>> kept;
  for(uint32_t s = 0; s < l.spheres; s++) {
    const std::array<int32_t, 4>& a = sphere_anchor_[s];
    const Vec3 x = V(sphere_offset_[s]);
    const float rx = sphere_offset_[s][3];
    const std::array<int32_t, 3> c = Cell(a, x, l.cell);
    uint32_t cells[27];
    int n_cells = 0;
    for(int dz = -1; dz <= 1; dz++) {
      for(int dy = -1; dy <= 1; dy++) {
        for(int dx = -1; dx <= 1; dx++) {
          const uint32_t hash =
            CellHash({c[0] + dx, c[1] + dy, c[2] + dz}, l.mask);
          if(std::find(cells, cells + n_cells, hash) == cells + n_cells) {
            cells[n_cells++] = hash;
          }
        }
      }
    }
    found.clear();
    uint32_t total = 0;
    for(int ci = 0; ci < n_cells; ci++) {
      for(uint32_t k = start[cells[ci]]; k < start[cells[ci] + 1]; k++) {
        const uint32_t t = order[k];
        const std::array<int32_t, 4>& b = sphere_anchor_[t];
        if(t == s || b[3] == a[3]) {
          continue;
        }
        const Vec3 y = V(sphere_offset_[t]);
        const Vec3 rel = s < t ? Relative(a, x, b, y, l.cell)
                               : Relative(b, y, a, x, l.cell);
        if(Length(rel) < rx + sphere_offset_[t][3] + l.margin) {
          if(found.size() < K) {
            found.push_back(t);
          }
          total++;
        }
      }
    }
    std::sort(found.begin(), found.end());
    const uint32_t base = s * K, old = neighbor_count_[s];
    kept.assign(found.size(), {0.0f, 0.0f, 0.0f, 0.0f});
    uint32_t j = 0;
    for(size_t k = 0; k < found.size(); k++) {
      while(j < old && neighbor_[base + j] < found[k]) {
        j++;
      }
      if(j < old && neighbor_[base + j] == found[k]) {
        kept[k] = history_[base + j];
      }
    }
    for(size_t k = 0; k < found.size(); k++) {
      neighbor_[base + k] = found[k];
      history_[base + k] = kept[k];
    }
    neighbor_count_[s] = static_cast<uint32_t>(found.size());
    reference_anchor_[s] = a;
    reference_offset_[s] = sphere_offset_[s];
    most = std::max(most, total);
    overflow = overflow || total > K;
  }
  statistics_.searches++;
  statistics_.most_neighbors = std::max(statistics_.most_neighbors, most);
  if(overflow) {
    throw shift::core::Exception(
      "DEM: a sphere has " + std::to_string(most) +
      " neighbors, more than max_neighbors (" + std::to_string(K) + ")");
  }
}

void shift::core::dem::CpuSolver::Forces(float dt) {
  const Layout& l = layout_;
  const uint32_t K = settings_.max_neighbors;
  for(uint32_t s = 0; s < l.spheres; s++) {
    Vec3 force = {0.0f, 0.0f, 0.0f}, torque = {0.0f, 0.0f, 0.0f};
    const uint32_t base = s * K;
    for(uint32_t k = 0; k < neighbor_count_[s]; k++) {
      const uint32_t t = neighbor_[base + k];
      const bool first = s < t;
      const uint32_t lo = first ? s : t, hi = first ? t : s;
      const std::array<float, 4>& x = sphere_offset_[lo];
      const std::array<float, 4>& y = sphere_offset_[hi];
      const Vec3 rel =
        Relative(sphere_anchor_[lo], V(x), sphere_anchor_[hi], V(y), l.cell);
      const float distance = Length(rel);
      float d = (x[3] + y[3]) - distance;
      std::array<float, 4> xi = history_[base + k];
      if(!(d > 0.0f) || !(distance > 0.0f)) {
        history_[base + k] = {0.0f, 0.0f, 0.0f, 0.0f};
        continue;
      }
      Vec3 n = rel / distance;
      Vec3 la, lb;
      float ra_star = x[3], rb_star = y[3], spread = 0.0f;
      const Shape sa = ShapeOf(l.template_hull[l.sphere_info[lo][1]], x[3],
                               quaternion_[l.sphere_info[lo][0]],
                               l.hull_vertex, l.hull_plane, l.hull_face);
      const Shape sb = ShapeOf(l.template_hull[l.sphere_info[hi][1]], y[3],
                               quaternion_[l.sphere_info[hi][0]],
                               l.hull_vertex, l.hull_plane, l.hull_face);
      if(sa.count > 0 || sb.count > 0) {
        /* a hull: the overlap of the two shapes */
        Vec3 point;
        float weight;
        if(!Collide(sa, sb, rel, x[3] + y[3], n, d, point, spread, weight)) {
          history_[base + k] = {0.0f, 0.0f, 0.0f, 0.0f};
          continue;
        }
        la = V(sphere_arm_[lo]) + (point - rel);
        lb = V(sphere_arm_[hi]) + point;
        ra_star = sa.count > 0 ? sa.radius : x[3];
        rb_star = sb.count > 0 ? sb.radius : y[3];
      } else {
        la = V(sphere_arm_[lo]) - (x[3] - 0.5f * d) * n;
        lb = V(sphere_arm_[hi]) + (y[3] - 0.5f * d) * n;
      }
      const std::array<float, 4>& va = sphere_velocity_[lo];
      const std::array<float, 4>& vb = sphere_velocity_[hi];
      const Vec3 wa = V(sphere_omega_[lo]), wb = V(sphere_omega_[hi]);
      const Vec3 ra = Cross(wa, la), rb = Cross(wb, lb);
      const Vec3 u = (V(va) + ra) - (V(vb) + rb);
      const float r = ra_star * rb_star / (ra_star + rb_star);
      const float m = va[3] * vb[3] / (va[3] + vb[3]);
      const uint32_t pair =
        sphere_material_[lo] * l.materials + sphere_material_[hi];
      Vec3 f, rolling;
      Law(n, d, r, m, u, wb - wa, l.pairs[pair], dt, xi, f, rolling);
      if(spread > 0.0f) {
        rolling = rolling + Rocking(n, d, r, m, wb - wa, l.pairs[pair], spread);
      }
      history_[base + k] = xi;
      if(first) {
        force = force + f;
        torque = torque + (Cross(la, f) + rolling);
      } else {
        force = force - f;
        torque = torque + (Cross(lb, -f) - rolling);
      }
    }
    const std::array<int32_t, 4>& a = sphere_anchor_[s];
    const std::array<float, 4>& x = sphere_offset_[s];
    for(uint32_t w = 0; w < l.wall_normal.size(); w++) {
      const Vec3 n = V(l.wall_normal[w]);
      const float height = Dot(
        n, Relative(a, V(x), l.wall_anchor[w], V(l.wall_offset[w]), l.cell));
      float d = x[3] - height;
      const uint32_t slot = s * kMaxWalls + w;
      Vec3 la = V(sphere_arm_[s]) - (x[3] - 0.5f * d) * n;
      float radius = x[3], spread = 0.0f;
      const Shape shape = ShapeOf(l.template_hull[l.sphere_info[s][1]], x[3],
                                  quaternion_[l.sphere_info[s][0]],
                                  l.hull_vertex, l.hull_plane, l.hull_face);
      if(shape.count > 0 && d > 0.0f) {
        /* a hull: its vertices beyond the wall */
        Vec3 sum = {0.0f, 0.0f, 0.0f};
        float total = 0.0f, deepest = 0.0f;
        for(uint32_t i = shape.first; i < shape.first + shape.count; i++) {
          const Vec3 v = Rotate(shape.q, V(shape.vertex[i]));
          const float depth = -(height + Dot(n, v));
          if(depth > 0.0f) {
            sum = sum + depth * v;
            total += depth;
            deepest = std::max(deepest, depth);
          }
        }
        d = deepest;
        if(total > 0.0f) {
          const Vec3 center = sum / total;
          la = V(sphere_arm_[s]) + center;
          for(uint32_t i = shape.first; i < shape.first + shape.count; i++) {
            const Vec3 v = Rotate(shape.q, V(shape.vertex[i]));
            const float depth = -(height + Dot(n, v));
            if(depth > 0.0f) {
              const Vec3 arm = v - center;
              const Vec3 tangent = arm - Dot(arm, n) * n;
              spread += depth * Dot(tangent, tangent);
            }
          }
          spread = spread / total;
        }
        radius = shape.radius;
      }
      if(!(d > 0.0f)) {
        wall_history_[slot] = {0.0f, 0.0f, 0.0f, 0.0f};
        continue;
      }
      std::array<float, 4> xi = wall_history_[slot];
      const std::array<float, 4>& va = sphere_velocity_[s];
      const Vec3 wa = V(sphere_omega_[s]);
      const Vec3 ra = Cross(wa, la);
      uint32_t material;
      std::memcpy(&material, &l.wall_normal[w][3], 4);
      const uint32_t pair = sphere_material_[s] * l.materials + material;
      Vec3 f, rolling;
      Law(n, d, radius, va[3], V(va) + ra, -wa, l.pairs[pair], dt, xi, f,
          rolling);
      if(spread > 0.0f) {
        rolling = rolling + Rocking(n, d, radius, va[3], -wa, l.pairs[pair],
                                    spread);
      }
      wall_history_[slot] = xi;
      force = force + f;
      torque = torque + (Cross(la, f) + rolling);
    }
    if(l.triangles > 0) {
      float f[3] = {force.x, force.y, force.z};
      float t[3] = {torque.x, torque.y, torque.z};
      TriangleForces(s, dt, f, t);
      force = {f[0], f[1], f[2]};
      torque = {t[0], t[1], t[2]};
    }
    sphere_force_[s] = A(force, 0.0f);
    sphere_torque_[s] = A(torque, 0.0f);
  }
}

void shift::core::dem::CpuSolver::Grains(float dt) {
  Layout& l = layout_;
  const Vec3 gravity = {settings_.gravity[0], settings_.gravity[1],
                        settings_.gravity[2]};
  for(size_t g = 0; g < l.anchor.size(); g++) {
    std::array<int32_t, 4>& a = l.anchor[g];
    const std::array<float, 4>& mass = l.template_mass[a[3]];
    const uint32_t first = l.first_sphere[g],
                   count = l.template_spheres[a[3]][1];
    Vec3 force = {0.0f, 0.0f, 0.0f}, torque = {0.0f, 0.0f, 0.0f};
    for(uint32_t k = 0; k < count; k++) {
      force = force + V(sphere_force_[first + k]);
      torque = torque + V(sphere_torque_[first + k]);
    }
    Quat q = Q(quaternion_[g]);
    const Vec3 v = V(velocity_[g]) + dt * (force / mass[0] + gravity);
    Vec3 w = V(omega_[g]);
    const Vec3 inertia = {mass[1], mass[2], mass[3]};
    const Vec3 moment = RotateBack(q, torque);
    w = w + dt * (moment - Cross(w, inertia * w)) / inertia;
    Vec3 x = V(l.offset[g]) + dt * v;
    const Vec3 cells = {std::floor(x.x / l.cell), std::floor(x.y / l.cell),
                        std::floor(x.z / l.cell)};
    a[0] += static_cast<int32_t>(cells.x);
    a[1] += static_cast<int32_t>(cells.y);
    a[2] += static_cast<int32_t>(cells.z);
    x = x - cells * l.cell;
    const float speed = Length(w);
    const float half_angle = 0.5f * dt * speed;
    const float scale = speed > 0.0f ? std::sin(half_angle) / speed : 0.5f * dt;
    q = QuaternionProduct(q, {scale * w, std::cos(half_angle)});
    const float size = std::sqrt(Dot(q.v, q.v) + q.w * q.w);
    q = {q.v / size, q.w / size};
    l.offset[g] = A(x, 0.0f);
    velocity_[g] = A(v, 0.0f);
    omega_[g] = A(w, 0.0f);
    quaternion_[g] = A(q.v, q.w);
  }
}

void shift::core::dem::CpuSolver::Step(float dt) {
  if(settings_.method == ContactMethod::kNsc) {
    NscStep(dt);
    return;
  }
  AnchoredPosesAt(time_, poses_);
  bool moved = Spheres();
  if(layout_.triangles > 0) {
    moved = Triangles() || moved;
  }
  if(search_ || moved) {
    Search();
    if(layout_.triangles > 0) {
      SearchTriangles();
    }
    search_ = false;
  }
  Forces(dt);
  Grains(dt);
  statistics_.steps++;
  time_ += dt;
}

void shift::core::dem::CpuSolver::GetState(State& state) {
  FromLayout(layout_.anchor, layout_.offset, state);
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      state.velocity[g][k] = velocity_[g][k];
      state.angular_velocity[g][k] = omega_[g][k];
    }
    state.orientation[g] = quaternion_[g];
  }
}

shift::core::dem::Statistics shift::core::dem::CpuSolver::GetStatistics() {
  return statistics_;
}

/* the non-smooth method (NSC, see Nsc in dem.h) */

static Vec3 V3(const float a[3]) {
  return {a[0], a[1], a[2]};
}

static void Put(Vec3 v, float a[3]) {
  a[0] = v.x;
  a[1] = v.y;
  a[2] = v.z;
}

/* two unit tangents of the unit normal n */
static void Tangents(Vec3 n, Vec3& t1, Vec3& t2) {
  const Vec3 axis = std::abs(n.x) < 0.57735f ? Vec3{1.0f, 0.0f, 0.0f}
                                             : Vec3{0.0f, 1.0f, 0.0f};
  t1 = Cross(n, axis);
  t1 = t1 / Length(t1);
  t2 = Cross(n, t1);
}

/* the impulse (gn, gt1, gt2) onto the cone gt <= mu gn, as Chrono's
ChConstraintTwoTuplesFrictionT::Project does it */
static void ProjectOnCone(float mu, float g[3]) {
  const float ft = std::sqrt(g[1] * g[1] + g[2] * g[2]);
  if(mu == 0.0f) {
    g[0] = std::max(g[0], 0.0f);
    g[1] = g[2] = 0.0f;
    return;
  }
  if(ft <= mu * g[0]) {
    return;
  }
  if(mu * ft <= -g[0]) {
    g[0] = g[1] = g[2] = 0.0f;
    return;
  }
  const float fn = (ft * mu + g[0]) / (mu * mu + 1.0f);
  const float scale = mu * fn / ft;
  g[0] = fn;
  g[1] *= scale;
  g[2] *= scale;
}

/* the change of a grain's angular velocity (in its principal axes) of the
moment impulse m (inertial frame) */
static Vec3 AngularChange(Quat q, Vec3 inertia, Vec3 m) {
  return RotateBack(q, m) / inertia;
}

/* the mean, over the three directions of the basis, of the change of the
velocity at arm l of a grain of mass and inertia, for a unit impulse along
each: the grain's part of the diagonal of the Delassus operator */
static float Delassus(Quat q, float mass, Vec3 inertia, Vec3 l, Vec3 n,
                      Vec3 t1, Vec3 t2) {
  float sum = 3.0f / mass;
  const Vec3 e[3] = {n, t1, t2};
  for(int k = 0; k < 3; k++) {
    const Vec3 c = RotateBack(q, Cross(l, e[k]));
    sum += Dot(c, c / inertia);
  }
  return sum;
}

/* that of a body of mass and inverse inertia (inertial frame, rows) */
static float BodyDelassus(float mass, const Vec3 inverse[3], Vec3 l, Vec3 n,
                          Vec3 t1, Vec3 t2) {
  float sum = 3.0f / mass;
  const Vec3 e[3] = {n, t1, t2};
  for(int k = 0; k < 3; k++) {
    const Vec3 c = Cross(l, e[k]);
    sum += Dot(c, Vec3{Dot(inverse[0], c), Dot(inverse[1], c),
                       Dot(inverse[2], c)});
  }
  return sum;
}

static Vec3 Times(const Vec3 m[3], Vec3 v) {
  return {Dot(m[0], v), Dot(m[1], v), Dot(m[2], v)};
}

/* that of a moment along t1 and t2 */
static float RollingDelassus(Quat q, Vec3 inertia, Vec3 t1, Vec3 t2) {
  const Vec3 a = RotateBack(q, t1), b = RotateBack(q, t2);
  return Dot(a, a / inertia) + Dot(b, b / inertia);
}

/* the reductions of the non-smooth method over count contacts, as the GPU
takes them (nsc_reduce.comp then nsc_control.comp, and nsc_body_reduce.comp
then nsc_body_sum.comp): per workgroup of 256 over 1024 contacts, each
invocation over 4 in turn (1024 g + 256 k + i), then a tree over the
workgroup; then 256 invocations over the workgroups' values in turn, and a
tree. TreeReduce sums the 4 values of value(c), or takes the largest of the
first 2 if largest; TreeSum6 sums those of the contacts value(c, out)
returns true for */
static std::array<float, 4> Combine(bool largest, std::array<float, 4> a,
                                    const std::array<float, 4>& b) {
  for(int k = 0; k < 4; k++) {
    a[k] = largest && k < 2 ? std::max(a[k], b[k]) : a[k] + b[k];
  }
  return a;
}

/* the tree over the first n of 256 lanes, the rest 0, which leave the
sums and the largest values (of values not negative) as they are */
template <typename T, typename Add>
static T Tree(std::vector<T>& lanes, size_t n, Add add) {
  for(size_t half = 128; half > 0; half /= 2) {
    for(size_t i = 0; i < half && i + half < n; i++) {
      lanes[i] = add(lanes[i], lanes[i + half]);
    }
    n = std::min(n, half);
  }
  return lanes[0];
}

template <typename T, typename Add, typename Value>
static T TreeOf(size_t count, T zero, Add add, Value value) {
  const size_t groups = (count + 1023) / 1024;
  std::vector<T> partials(groups), lanes(256, zero);
  for(size_t g = 0; g < groups; g++) {
    const size_t n = std::min<size_t>(256, count - 1024 * g);
    for(size_t i = 0; i < n; i++) {
      T sum = zero;
      for(size_t k = 0; k < 4; k++) {
        const size_t c = 1024 * g + 256 * k + i;
        if(c < count) {
          value(c, sum);
        }
      }
      lanes[i] = sum;
    }
    partials[g] = Tree(lanes, n, add);
  }
  const size_t n = std::min<size_t>(256, groups);
  for(size_t i = 0; i < n; i++) {
    T sum = zero;
    for(size_t g = i; g < groups; g += 256) {
      sum = add(sum, partials[g]);
    }
    lanes[i] = sum;
  }
  return n > 0 ? Tree(lanes, n, add) : zero;
}

template <typename Value>
static std::array<float, 4> TreeReduce(size_t count, bool largest,
                                       Value value) {
  auto add = [largest](const std::array<float, 4>& a,
                       const std::array<float, 4>& b) {
    return Combine(largest, a, b);
  };
  return TreeOf(count, std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}, add,
                [&](size_t c, std::array<float, 4>& sum) {
                  sum = Combine(largest, sum, value(c));
                });
}

template <typename Value>
static std::array<float, 6> TreeSum6(size_t count, Value value) {
  auto add = [](const std::array<float, 6>& a, const std::array<float, 6>& b) {
    std::array<float, 6> c;
    for(int k = 0; k < 6; k++) {
      c[k] = a[k] + b[k];
    }
    return c;
  };
  return TreeOf(count, std::array<float, 6>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                add, [&](size_t c, std::array<float, 6>& sum) {
                  std::array<float, 6> v;
                  if(value(c, v)) {
                    for(int k = 0; k < 6; k++) {
                      sum[k] += v[k];
                    }
                  }
                });
}

void shift::core::dem::CpuSolver::NscStep(float h) {
  const Layout& l = layout_;
  const Nsc& nsc = settings_.nsc;
  const uint32_t none = 0xffffffffu;
  const size_t grains = l.anchor.size();
  /* the bodies where they are at the start of the step, at the velocities
  of its end: the path there carries the impulses of the steps before,
  which the multibody step has applied, and not this step's */
  AnchoredPosesAt(time_, poses_);
  std::vector<std::array<float, 4>> end;
  AnchoredPosesAt(time_ + h, end);
  bool moved = Spheres();
  if(l.triangles > 0) {
    moved = Triangles() || moved;
  }
  if(search_ || moved) {
    Search();
    if(l.triangles > 0) {
      SearchTriangles();
    }
    search_ = false;
  }
  /* the velocities without contacts: gravity and the gyroscopic term */
  const Vec3 gravity = {settings_.gravity[0], settings_.gravity[1],
                        settings_.gravity[2]};
  std::vector<Vec3> v(grains), w(grains), v_free(grains), w_free(grains);
  std::vector<Quat> q(grains);
  std::vector<float> mass(grains);
  std::vector<Vec3> inertia(grains);
  for(size_t g = 0; g < grains; g++) {
    const std::array<float, 4>& m = l.template_mass[l.anchor[g][3]];
    mass[g] = m[0];
    inertia[g] = {m[1], m[2], m[3]};
    q[g] = Q(quaternion_[g]);
    const Vec3 omega = V(omega_[g]);
    v_free[g] = V(velocity_[g]) + h * gravity;
    w_free[g] = omega - h * Cross(omega, inertia[g] * omega) / inertia[g];
  }
  /* the bodies of mass: their inverse inertia in the inertial frame, and the
  changes of their velocities in the step */
  const size_t B = bodies_.size();
  std::vector<float> body_mass(B, 0.0f);
  std::vector<std::array<Vec3, 3>> body_inverse(B);
  std::vector<Vec3> body_center(B), dvb(B), dwb(B);
  std::vector<std::array<float, 4>> mass_rows;
  BodyMassRows(poses_, mass_rows);
  for(size_t b = 0; b < B; b++) {
    const std::array<float, 4>* row = &mass_rows[5 * b];
    body_mass[b] = row[0][0];
    body_inverse[b] = {V(row[1]), V(row[2]), V(row[3])};
    body_center[b] = V(row[4]);
  }
  /* the contacts: pairs, by the lower sphere; walls; triangles; each
  sphere's first, and the contact of each of the pairs it is the lower
  sphere of */
  contacts_.clear();
  const uint32_t K = settings_.max_neighbors;
  std::vector<uint32_t> contact_start(l.spheres + 1, 0u),
    slot_contact(size_t(l.spheres) * K, none);
  /* the relative velocity at a contact before the step, for restitution,
  and the bias of its gap */
  auto bias = [&](NscContact& c, float gap, Vec3 u_before, float restitution) {
    const float un = Dot(V3(c.n), u_before);
    c.bias = gap >= 0.0f ? gap / h : std::max(gap / h, -nsc.recovery_speed);
    if(restitution > 0.0f && un < -nsc.min_bounce_speed &&
       gap + un * h < 0.0f) {
      c.bias = std::min(c.bias, restitution * un);
    }
  };
  for(uint32_t s = 0; s < l.spheres; s++) {
    contact_start[s] = static_cast<uint32_t>(contacts_.size());
    const std::array<int32_t, 4>& a = sphere_anchor_[s];
    const std::array<float, 4>& x = sphere_offset_[s];
    const uint32_t ga = static_cast<uint32_t>(a[3]);
    for(uint32_t k = 0; k < neighbor_count_[s]; k++) {
      const uint32_t t = neighbor_[s * K + k];
      if(t < s) {
        continue;
      }
      const std::array<float, 4>& y = sphere_offset_[t];
      const Vec3 rel = Relative(a, V(x), sphere_anchor_[t], V(y), l.cell);
      const float distance = Length(rel);
      const float gap = distance - (x[3] + y[3]);
      if(!(gap < l.margin) || !(distance > 0.0f)) {
        history_[s * K + k] = {0.0f, 0.0f, 0.0f, 0.0f};
        continue;
      }
      NscContact c;
      c.a = ga;
      c.b = static_cast<uint32_t>(sphere_anchor_[t][3]);
      c.sphere = s;
      c.body = none;
      const Vec3 n = rel / distance;
      Vec3 t1, t2;
      Tangents(n, t1, t2);
      Put(n, c.n);
      Put(t1, c.t1);
      Put(t2, c.t2);
      Put(V(sphere_arm_[s]) - (x[3] + 0.5f * gap) * n, c.la);
      Put(V(sphere_arm_[t]) + (y[3] + 0.5f * gap) * n, c.lb);
      Put({0.0f, 0.0f, 0.0f}, c.lc);
      Put({0.0f, 0.0f, 0.0f}, c.vb);
      Put({0.0f, 0.0f, 0.0f}, c.wb);
      const std::array<float, 8>& p =
        l.pairs[sphere_material_[s] * l.materials + sphere_material_[t]];
      c.mu = p[3];
      c.rolling = p[4] * x[3] * y[3] / (x[3] + y[3]);
      const Vec3 u = (V(velocity_[c.a]) +
                      Cross(Rotate(q[c.a], V(omega_[c.a])), V3(c.la))) -
                     (V(velocity_[c.b]) +
                      Cross(Rotate(q[c.b], V(omega_[c.b])), V3(c.lb)));
      bias(c, gap, u, p[5]);
      c.kept = &history_[s * K + k];
      slot_contact[s * K + k] = static_cast<uint32_t>(contacts_.size());
      contacts_.push_back(c);
    }
    for(uint32_t wi = 0; wi < l.wall_normal.size(); wi++) {
      const Vec3 n = V(l.wall_normal[wi]);
      const float gap =
        Dot(n, Relative(a, V(x), l.wall_anchor[wi], V(l.wall_offset[wi]),
                        l.cell)) -
        x[3];
      const uint32_t slot = s * kMaxWalls + wi;
      if(!(gap < l.margin)) {
        wall_history_[slot] = {0.0f, 0.0f, 0.0f, 0.0f};
        continue;
      }
      NscContact c;
      c.a = ga;
      c.b = none;
      c.sphere = s;
      c.body = none;
      Vec3 t1, t2;
      Tangents(n, t1, t2);
      Put(n, c.n);
      Put(t1, c.t1);
      Put(t2, c.t2);
      Put(V(sphere_arm_[s]) - (x[3] + 0.5f * gap) * n, c.la);
      Put({0.0f, 0.0f, 0.0f}, c.lb);
      Put({0.0f, 0.0f, 0.0f}, c.lc);
      Put({0.0f, 0.0f, 0.0f}, c.vb);
      Put({0.0f, 0.0f, 0.0f}, c.wb);
      uint32_t material;
      std::memcpy(&material, &l.wall_normal[wi][3], 4);
      const std::array<float, 8>& p =
        l.pairs[sphere_material_[s] * l.materials + material];
      c.mu = p[3];
      c.rolling = p[4] * x[3];
      const Vec3 u = V(velocity_[ga]) +
                     Cross(Rotate(q[ga], V(omega_[ga])), V3(c.la));
      bias(c, gap, u, p[5]);
      c.kept = &wall_history_[slot];
      contacts_.push_back(c);
    }
    /* the triangles, each point of a body once */
    const uint32_t M = settings_.max_triangles;
    Vec3 touched[32];
    uint32_t touched_body[32];
    uint32_t touches = 0;
    for(uint32_t k = 0; k < (l.triangles > 0 ? triangle_count_[s] : 0); k++) {
      const uint32_t tri = triangle_list_[s * M + k];
      const std::array<int32_t, 4>& ta = triangle_anchor_[tri];
      const uint32_t b = static_cast<uint32_t>(ta[3]);
      const Vec3 p = Relative(a, V(x), ta, {0.0f, 0.0f, 0.0f}, l.cell);
      const std::array<float, 4>& v1 = triangle_vertex_[3 * tri + 1];
      const Vec3 closest = ClosestPointOnTriangle(
        p, V(triangle_vertex_[3 * tri]), V(v1),
        V(triangle_vertex_[3 * tri + 2]));
      const Vec3 offset = p - closest;
      const float distance = Length(offset);
      const float gap = distance - x[3];
      bool repeated = false;
      for(uint32_t j = 0; j < touches; j++) {
        repeated = repeated || (touched_body[j] == b &&
                                Length(touched[j] - closest) < 1e-3f * x[3]);
      }
      if(!(gap < l.margin) || !(distance > 0.0f) || repeated) {
        triangle_history_[s * M + k] = {0.0f, 0.0f, 0.0f, 0.0f};
        continue;
      }
      touched[touches] = closest;
      touched_body[touches] = b;
      touches++;
      NscContact c;
      c.a = ga;
      c.b = none;
      c.sphere = s;
      c.body = b;
      const Vec3 n = offset / distance;
      Vec3 t1, t2;
      Tangents(n, t1, t2);
      Put(n, c.n);
      Put(t1, c.t1);
      Put(t2, c.t2);
      Put(V(sphere_arm_[s]) - (x[3] + 0.5f * gap) * n, c.la);
      const std::array<float, 4>* row = &poses_[7 * b];
      const Vec3 lb = (closest + 0.5f * gap * n) - V(row[1]);
      Put(lb, c.lb);
      Put(lb - body_center[b], c.lc);
      const std::array<float, 4>* moving = &end[7 * b];
      Put(V(moving[5]) + Cross(V(moving[6]), lb), c.vb);
      Put(V(moving[6]), c.wb);
      const std::array<float, 8>& pp =
        l.pairs[sphere_material_[s] * l.materials + Bits(v1[3])];
      c.mu = pp[3];
      c.rolling = pp[4] * x[3];
      const Vec3 u = (V(velocity_[ga]) +
                      Cross(Rotate(q[ga], V(omega_[ga])), V3(c.la))) -
                     V3(c.vb);
      bias(c, gap, u, pp[5]);
      c.kept = &triangle_history_[s * M + k];
      contacts_.push_back(c);
    }
  }
  contact_start[l.spheres] = static_cast<uint32_t>(contacts_.size());
  /* for each neighbor of lower index of each sphere, the contact of their
  pair, which the lower one listed */
  std::vector<uint32_t> other_contact(size_t(l.spheres) * K, none);
  for(uint32_t s = 0; s < l.spheres; s++) {
    for(uint32_t j = 0; j < neighbor_count_[s]; j++) {
      const uint32_t t = neighbor_[s * K + j];
      if(t < s) {
        const uint32_t* list = &neighbor_[t * K];
        const uint32_t* at =
          std::lower_bound(list, list + neighbor_count_[t], s);
        if(at != list + neighbor_count_[t] && *at == s) {
          other_contact[s * K + j] = slot_contact[t * K + uint32_t(at - list)];
        }
      }
    }
  }
  /* with mass splitting, the contacts of each grain and body, its shares
  of the diagonals of its contacts */
  const bool split = SplitsMasses(nsc);
  std::vector<uint32_t> grain_contacts(split ? grains : 0, 0u),
    body_contacts(split ? B : 0, 0u);
  if(split) {
    for(const NscContact& c : contacts_) {
      grain_contacts[c.a]++;
      if(c.b != none) {
        grain_contacts[c.b]++;
      } else if(c.body != none) {
        body_contacts[c.body]++;
      }
    }
  }
  /* the step sizes, and the impulses of the step before */
  for(NscContact& c : contacts_) {
    const Vec3 n = V3(c.n), t1 = V3(c.t1), t2 = V3(c.t2);
    const float share_a = split ? float(grain_contacts[c.a]) : 1.0f;
    float diagonal = share_a * Delassus(q[c.a], mass[c.a], inertia[c.a],
                                        V3(c.la), n, t1, t2);
    float rolling = share_a * RollingDelassus(q[c.a], inertia[c.a], t1, t2);
    if(c.b != none) {
      const float share_b = split ? float(grain_contacts[c.b]) : 1.0f;
      diagonal += share_b * Delassus(q[c.b], mass[c.b], inertia[c.b],
                                     V3(c.lb), n, t1, t2);
      rolling += share_b * RollingDelassus(q[c.b], inertia[c.b], t1, t2);
    } else if(c.body != none && body_mass[c.body] > 0.0f) {
      const float share = split ? float(body_contacts[c.body]) : 1.0f;
      const Vec3* inverse = body_inverse[c.body].data();
      diagonal += share * BodyDelassus(body_mass[c.body], inverse, V3(c.lc),
                                       n, t1, t2);
      rolling += share * (Dot(t1, Times(inverse, t1)) +
                          Dot(t2, Times(inverse, t2)));
    }
    c.eta = 3.0f / diagonal;
    c.eta_r = 2.0f / rolling;
    const Vec3 kept = V(*c.kept);
    c.gamma[0] = Dot(kept, n);
    c.gamma[1] = Dot(kept, t1);
    c.gamma[2] = Dot(kept, t2);
    ProjectOnCone(c.mu, c.gamma);
  }
  /* the velocities of the free motion and the impulses */
  auto apply = [&](const NscContact& c, const float dg[3], Vec3 dr,
                   std::vector<Vec3>& vv, std::vector<Vec3>& ww) {
    const Vec3 impulse = dg[0] * V3(c.n) + dg[1] * V3(c.t1) + dg[2] * V3(c.t2);
    vv[c.a] = vv[c.a] + impulse / mass[c.a];
    ww[c.a] = ww[c.a] + AngularChange(q[c.a], inertia[c.a],
                                      Cross(V3(c.la), impulse) + dr);
    if(c.b != none) {
      vv[c.b] = vv[c.b] - impulse / mass[c.b];
      ww[c.b] = ww[c.b] - AngularChange(q[c.b], inertia[c.b],
                                        Cross(V3(c.lb), impulse) + dr);
    } else if(c.body != none && body_mass[c.body] > 0.0f) {
      dvb[c.body] = dvb[c.body] - impulse / body_mass[c.body];
      dwb[c.body] = dwb[c.body] - Times(body_inverse[c.body].data(),
                                        Cross(V3(c.lc), impulse) + dr);
    }
  };
  /* the velocities of the impulses, as the GPU sums them
  (nsc_velocities.comp, nsc_body_reduce.comp, nsc_body_sum.comp): each
  grain, sphere by sphere, the pairs it is the higher sphere of by index,
  then its own contacts in order; each body's impulses in trees of the
  contacts */
  auto rebuild = [&]() {
    const Layout& lr = layout_;
    for(size_t g = 0; g < grains; g++) {
      Vec3 vg = v_free[g], wg = w_free[g];
      const uint32_t first = lr.first_sphere[g],
                     count = lr.template_spheres[lr.anchor[g][3]][1];
      for(uint32_t sphere = first; sphere < first + count; sphere++) {
        for(uint32_t j = 0; j < neighbor_count_[sphere]; j++) {
          const uint32_t ci = other_contact[sphere * K + j];
          if(ci == none) {
            continue;
          }
          const NscContact& c = contacts_[ci];
          const Vec3 impulse = c.gamma[0] * V3(c.n) + c.gamma[1] * V3(c.t1) +
                               c.gamma[2] * V3(c.t2);
          vg = vg - impulse / mass[g];
          wg = wg - AngularChange(q[g], inertia[g],
                                  Cross(V3(c.lb), impulse) + V3(c.rho));
        }
        for(uint32_t ci = contact_start[sphere];
            ci < contact_start[sphere + 1]; ci++) {
          const NscContact& c = contacts_[ci];
          const Vec3 impulse = c.gamma[0] * V3(c.n) + c.gamma[1] * V3(c.t1) +
                               c.gamma[2] * V3(c.t2);
          vg = vg + impulse / mass[g];
          wg = wg + AngularChange(q[g], inertia[g],
                                  Cross(V3(c.la), impulse) + V3(c.rho));
        }
      }
      v[g] = vg;
      w[g] = wg;
    }
    for(size_t b = 0; b < B; b++) {
      dvb[b] = dwb[b] = Vec3{0.0f, 0.0f, 0.0f};
      if(!(body_mass[b] > 0.0f)) {
        continue;
      }
      std::array<float, 6> sum = TreeSum6(
        contacts_.size(), [&](size_t ci, std::array<float, 6>& value) {
          const NscContact& c = contacts_[ci];
          if(c.body != b) {
            return false;
          }
          const Vec3 impulse = c.gamma[0] * V3(c.n) +
                               c.gamma[1] * V3(c.t1) + c.gamma[2] * V3(c.t2);
          const Vec3 m = Cross(V3(c.lc), impulse) + V3(c.rho);
          value = {impulse.x, impulse.y, impulse.z, m.x, m.y, m.z};
          return true;
        });
      dvb[b] = -Vec3{sum[0], sum[1], sum[2]} / body_mass[b];
      dwb[b] = -Times(body_inverse[b].data(), Vec3{sum[3], sum[4], sum[5]});
    }
  };
  rebuild();
  /* the relative velocity and angular velocity at contact c */
  auto relative = [&](const NscContact& c, Vec3& u, Vec3& relative_w) {
    const Vec3 wa = Rotate(q[c.a], w[c.a]);
    u = v[c.a] + Cross(wa, V3(c.la));
    relative_w = wa;
    if(c.b != none) {
      const Vec3 wb = Rotate(q[c.b], w[c.b]);
      u = u - (v[c.b] + Cross(wb, V3(c.lb)));
      relative_w = relative_w - wb;
    } else {
      u = u - V3(c.vb);
      relative_w = relative_w - V3(c.wb);
      if(c.body != none) {
        u = u - (dvb[c.body] + Cross(dwb[c.body], V3(c.lc)));
        relative_w = relative_w - dwb[c.body];
      }
    }
  };
  /* the new impulse of contact c, from the velocities, in the cone */
  auto sliding = [&](const NscContact& c, float omega, float g[3]) {
    Vec3 u, relative_w;
    relative(c, u, relative_w);
    g[0] = c.gamma[0] - omega * c.eta * (Dot(V3(c.n), u) + c.bias);
    g[1] = c.gamma[1] - omega * c.eta * Dot(V3(c.t1), u);
    g[2] = c.gamma[2] - omega * c.eta * Dot(V3(c.t2), u);
    ProjectOnCone(c.mu, g);
  };
  /* its new rolling impulse, in the disk of the normal impulse gn */
  auto rolling = [&](const NscContact& c, float omega, float gn) {
    if(!(c.rolling > 0.0f)) {
      return Vec3{0.0f, 0.0f, 0.0f};
    }
    Vec3 u, relative_w;
    relative(c, u, relative_w);
    const Vec3 n = V3(c.n);
    const Vec3 rolling_w = relative_w - Dot(relative_w, n) * n;
    Vec3 r = V3(c.rho) - omega * c.eta_r * rolling_w;
    r = r - Dot(r, n) * n;
    const float most = c.rolling * gn, size = Length(r);
    if(size > most) {
      r = size > 0.0f ? r * (most / size) : r;
    }
    return r;
  };
  /* APGD on the scaled impulses z: per contact the normal and tangent
  impulses over the square root of eta, and the rolling impulse over that of
  eta_r; returns the residual of the best iterate, the largest projected
  gradient over the largest impulse */
  auto apgd = [&]() -> float {
    const size_t count = contacts_.size();
    if(count == 0) {
      return 0.0f;
    }
    typedef std::vector<std::array<float, 6>> Z;
    std::vector<float> s(count), sr(count);
    for(size_t i = 0; i < count; i++) {
      const NscContact& c = contacts_[i];
      s[i] = std::sqrt(c.eta);
      sr[i] = std::sqrt(c.eta_r);
    }
    auto set = [&](const Z& z) {
      for(size_t i = 0; i < count; i++) {
        NscContact& c = contacts_[i];
        for(int k = 0; k < 3; k++) {
          c.gamma[k] = s[i] * z[i][k];
          c.rho[k] = sr[i] * z[i][3 + k];
        }
      }
      rebuild();
    };
    /* the gradient N z + r, scaled: the contacts' velocities and biases */
    auto gradient = [&](const Z& z, Z& g) {
      set(z);
      for(size_t i = 0; i < count; i++) {
        const NscContact& c = contacts_[i];
        Vec3 u, rw;
        relative(c, u, rw);
        const Vec3 n = V3(c.n);
        const Vec3 rt = rw - Dot(rw, n) * n;
        g[i] = {s[i] * (Dot(n, u) + c.bias), s[i] * Dot(V3(c.t1), u),
                s[i] * Dot(V3(c.t2), u), sr[i] * rt.x, sr[i] * rt.y,
                sr[i] * rt.z};
      }
    };
    /* the projection of the scaled impulses (nsc_common.glsl,
    ProjectScaled) */
    auto project = [&](size_t i, std::array<float, 6>& z) {
      const NscContact& c = contacts_[i];
      const float cr = c.rolling > 0.0f ? c.rolling * s[i] / sr[i] : 0.0f;
      float g3[3] = {z[0], z[1], z[2]};
      ProjectOnCone(c.mu, g3);
      const Vec3 n = V3(c.n);
      Vec3 r = {z[3], z[4], z[5]};
      r = r - Dot(r, n) * n;
      const float most = cr * g3[0], size = Length(r);
      if(size > most) {
        r = size > 0.0f ? r * (most / size) : Vec3{0.0f, 0.0f, 0.0f};
      }
      z = {g3[0], g3[1], g3[2], r.x, r.y, r.z};
    };
    /* a . b of two scaled impulses, as the shaders take it */
    auto dot6 = [](const std::array<float, 6>& a,
                   const std::array<float, 6>& b) {
      return Dot({a[0], a[1], a[2]}, {b[0], b[1], b[2]}) +
             Dot({a[3], a[4], a[5]}, {b[3], b[4], b[5]});
    };
    auto minus = [](const std::array<float, 6>& a,
                    const std::array<float, 6>& b) {
      std::array<float, 6> c;
      for(int k = 0; k < 6; k++) {
        c[k] = a[k] - b[k];
      }
      return c;
    };
    /* the start: the impulses of the step before */
    Z x(count), y, x_new(count), y_new(count), g_y(count), g_x(count);
    for(size_t i = 0; i < count; i++) {
      const NscContact& c = contacts_[i];
      x[i] = {c.gamma[0] / s[i], c.gamma[1] / s[i], c.gamma[2] / s[i],
              0.0f, 0.0f, 0.0f};
      project(i, x[i]);
    }
    /* the Lipschitz constant, from N applied to the difference of the start
    and a vector of ones; the sums over the contacts are the GPU's
    (nsc_reduce.comp, nsc_control.comp): float, in trees */
    const std::array<float, 6> one = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    Z ones(count, one), g_ones(count);
    gradient(x, g_x);
    gradient(ones, g_ones);
    const std::array<float, 4> lipschitz_sums =
      TreeReduce(count, false, [&](size_t i) {
        const std::array<float, 6> d = minus(g_x[i], g_ones[i]),
                                   e = minus(x[i], one);
        return std::array<float, 4>{dot6(d, d), dot6(e, e), 0.0f, 0.0f};
      });
    const float num = lipschitz_sums[0], den = lipschitz_sums[1];
    float lipschitz = den > 0.0f && num > 0.0f ? std::sqrt(num / den) : 1.0f;
    float theta = 1.0f;
    y = x;
    Z best = x;
    float best_residual = 3.4e38f;
    for(uint32_t it = 0; it < nsc.iterations; it++) {
      gradient(y, g_y);
      float step = 1.0f / lipschitz;
      for(uint32_t backtrack = 0;; backtrack++) {
        for(size_t i = 0; i < count; i++) {
          for(int k = 0; k < 6; k++) {
            x_new[i][k] = y[i][k] - step * g_y[i][k];
          }
          project(i, x_new[i]);
        }
        gradient(x_new, g_x);
        /* the step descends enough, f(x) <= f(y) + g_y . d + L |d|^2 / 2
        with d = x - y, which, f being quadratic with gradient g = N z + r,
        is d . (g_x - g_y) <= L |d|^2: sums of small differences, which
        float keeps, where f(x) and f(y) are large and nearly equal */
        const std::array<float, 4> sums =
          TreeReduce(count, false, [&](size_t i) {
            const std::array<float, 6> d = minus(x_new[i], y[i]);
            return std::array<float, 4>{dot6(d, minus(g_x[i], g_y[i])),
                                        dot6(d, d), 0.0f, 0.0f};
          });
        if(sums[0] <= lipschitz * sums[1] || backtrack + 1 == kApgdTries) {
          break;
        }
        lipschitz = 2.0f * lipschitz;
        step = 1.0f / lipschitz;
      }
      const float theta_new =
        0.5f * (-theta * theta + theta * std::sqrt(theta * theta + 4.0f));
      const float beta = theta * (1.0f - theta) / (theta * theta + theta_new);
      /* the residual of x_new, its projected gradient, over the largest
      impulse; and whether the step went uphill */
      const std::array<float, 4> sums =
        TreeReduce(count, true, [&](size_t i) {
          std::array<float, 6> p = minus(x_new[i], g_x[i]);
          project(i, p);
          float residual_i = 0.0f, largest_i = 0.0f;
          for(int k = 0; k < 6; k++) {
            residual_i = std::max(residual_i, std::abs(x_new[i][k] - p[k]));
            largest_i = std::max(largest_i, std::abs(x_new[i][k]));
          }
          return std::array<float, 4>{
            residual_i, largest_i, dot6(g_y[i], minus(x_new[i], x[i])), 0.0f};
        });
      const float residual_now = sums[1] > 0.0f ? sums[0] / sums[1] : sums[0];
      if(residual_now < best_residual) {
        best_residual = residual_now;
        best = x_new;
      }
      const bool restart = sums[2] > 0.0f;
      for(size_t i = 0; i < count; i++) {
        for(int k = 0; k < 6; k++) {
          y_new[i][k] = restart ? x_new[i][k]
                                : x_new[i][k] + beta * (x_new[i][k] - x[i][k]);
        }
      }
      theta = restart ? 1.0f : theta_new;
      lipschitz = 0.9f * lipschitz;
      x.swap(x_new);
      y.swap(y_new);
    }
    set(best);
    return best_residual;
  };
  const float omega = Relaxation(nsc);
  const bool jacobi = nsc.solver == NscSolver::kJacobi;
  const float no_change[3] = {0.0f, 0.0f, 0.0f};
  float residual = 0.0f;
  std::vector<std::array<float, 6>> next(jacobi ? contacts_.size() : 0);
  if(nsc.solver == NscSolver::kApgd) {
    residual = apgd();
  }
  for(uint32_t it = 0;
      nsc.solver != NscSolver::kApgd && it < nsc.iterations; it++) {
    float change = 0.0f, largest = 0.0f;
    for(size_t i = 0; i < contacts_.size(); i++) {
      NscContact& c = contacts_[i];
      float g[3];
      sliding(c, omega, g);
      const float dg[3] = {g[0] - c.gamma[0], g[1] - c.gamma[1],
                           g[2] - c.gamma[2]};
      change = std::max(change, std::sqrt(dg[0] * dg[0] + dg[1] * dg[1] +
                                          dg[2] * dg[2]));
      largest = std::max(largest, std::abs(g[0]));
      if(jacobi) {
        const Vec3 r = rolling(c, omega, g[0]);
        next[i] = {g[0], g[1], g[2], r.x, r.y, r.z};
      } else {
        /* the impulse first, then the rolling from the spin it leaves */
        apply(c, dg, {0.0f, 0.0f, 0.0f}, v, w);
        for(int k = 0; k < 3; k++) {
          c.gamma[k] = g[k];
        }
        const Vec3 r = rolling(c, omega, g[0]);
        apply(c, no_change, r - V3(c.rho), v, w);
        Put(r, c.rho);
      }
    }
    if(jacobi) {
      for(size_t i = 0; i < contacts_.size(); i++) {
        for(int k = 0; k < 3; k++) {
          contacts_[i].gamma[k] = next[i][k];
          contacts_[i].rho[k] = next[i][3 + k];
        }
      }
      rebuild();
    }
    residual = largest > 0.0f ? change / largest : 0.0f;
  }
  /* keep the impulses for the next step; the bodies' impulses */
  for(const NscContact& c : contacts_) {
    const Vec3 impulse =
      c.gamma[0] * V3(c.n) + c.gamma[1] * V3(c.t1) + c.gamma[2] * V3(c.t2);
    *c.kept = A(impulse, 0.0f);
    if(c.body != none) {
      const Vec3 moment = Cross(V3(c.lb), -impulse) - V3(c.rho);
      const float values[6] = {-impulse.x, -impulse.y, -impulse.z,
                               moment.x,   moment.y,   moment.z};
      for(uint32_t k = 0; k < 6; k++) {
        float& slot =
          impulse_[(size_t(k) * B + c.body) * l.spheres + c.sphere];
        slot = slot + values[k];
      }
    }
  }
  /* each grain moves with its new velocity */
  Layout& lw = layout_;
  for(size_t g = 0; g < grains; g++) {
    std::array<int32_t, 4>& a = lw.anchor[g];
    Vec3 x = V(lw.offset[g]) + h * v[g];
    const Vec3 cells = {std::floor(x.x / lw.cell), std::floor(x.y / lw.cell),
                        std::floor(x.z / lw.cell)};
    a[0] += static_cast<int32_t>(cells.x);
    a[1] += static_cast<int32_t>(cells.y);
    a[2] += static_cast<int32_t>(cells.z);
    x = x - cells * lw.cell;
    const float speed = Length(w[g]);
    const float half_angle = 0.5f * h * speed;
    const float scale = speed > 0.0f ? std::sin(half_angle) / speed : 0.5f * h;
    Quat r = QuaternionProduct(q[g], {scale * w[g], std::cos(half_angle)});
    const float size = std::sqrt(Dot(r.v, r.v) + r.w * r.w);
    r = {r.v / size, r.w / size};
    lw.offset[g] = A(x, 0.0f);
    velocity_[g] = A(v[g], 0.0f);
    omega_[g] = A(w[g], 0.0f);
    quaternion_[g] = A(r.v, r.w);
  }
  statistics_.contacts = contacts_.size();
  statistics_.residual = residual;
  statistics_.steps++;
  time_ += h;
}
