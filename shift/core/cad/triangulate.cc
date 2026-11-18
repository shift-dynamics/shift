#include "shift/core/cad/triangulate.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <unordered_map>

#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"
#include "shift/core/exception.h"

namespace shift {
namespace core {
namespace cad {

static const double kPi = 3.14159265358979323846;

static Vec3 Sub(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

static double Dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

static double Norm(const Vec3& a) {
  return std::sqrt(Dot(a, a));
}

static double Lerp(double a, double b, double f) {
  return a + (b - a) * f;
}

/* the point of an edge at the parameter t of its curve (of its first
pcurve, on that pcurve's surface, if it has no curve) */
static Vec3 EdgePoint(const Model& model, const Edge& e, double t) {
  if(e.curve >= 0) {
    return CurvePoint<3>(model.curves[e.curve], t);
  }
  const Pcurve& pc = e.pcurves[0];
  const Vec2 uv = CurvePoint<2>(pc.curve, t);
  return SurfacePoint(model, model.surfaces[pc.surface], uv[0], uv[1]);
}

/*!
@brief points that are one point when closer than a tolerance, found by a
grid of cells of that size
*/
class Welder {
public:
  Welder(std::vector<Vec3>& points, double tolerance)
    : points_(points), tolerance_(tolerance) {}

  int Add(const Vec3& p) {
    const long cx = Cell(p[0]), cy = Cell(p[1]), cz = Cell(p[2]);
    for(long dx = -1; dx <= 1; dx++) {
      for(long dy = -1; dy <= 1; dy++) {
        for(long dz = -1; dz <= 1; dz++) {
          auto found = cells_.find(Key(cx + dx, cy + dy, cz + dz));
          if(found == cells_.end()) {
            continue;
          }
          for(int i : found->second) {
            if(Norm(Sub(points_[i], p)) <= tolerance_) {
              return i;
            }
          }
        }
      }
    }
    const int i = static_cast<int>(points_.size());
    points_.push_back(p);
    cells_[Key(cx, cy, cz)].push_back(i);
    return i;
  }

private:
  long Cell(double x) const {
    return static_cast<long>(std::floor(x / tolerance_));
  }
  static unsigned long long Key(long x, long y, long z) {
    const unsigned long long m = (1ull << 21) - 1;
    return ((static_cast<unsigned long long>(x) & m) << 42) |
           ((static_cast<unsigned long long>(y) & m) << 21) |
           (static_cast<unsigned long long>(z) & m);
  }

  std::vector<Vec3>& points_;
  double tolerance_;
  std::unordered_map<unsigned long long, std::vector<int>> cells_;
};

/*!
@brief constrained Delaunay triangulation of points in the plane, with
insertion of further points (Bowyer-Watson, the cavity bounded by the
constrained edges) and recovery of constrained edges by flips (Sloan 1993)

Triangles are counterclockwise; edge i of a triangle is opposite its
vertex i, from v[i + 1] to v[i + 2], and n[i] is the triangle across it.
*/
class Cdt {
public:
  struct Triangle {
    int v[3];
    int n[3];
    bool c[3];
    bool alive;
    bool inside;
  };

  std::vector<Vec2> xy;
  std::vector<Triangle> triangles;

  /* a triangle containing all the points to come, inside [lo, hi] */
  void Start(const Vec2& lo, const Vec2& hi) {
    const double dx = hi[0] - lo[0], dy = hi[1] - lo[1];
    const double d = std::max(dx, dy) * 10.0 + 1.0;
    const Vec2 c{0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])};
    xy = {{c[0] - 2.0 * d, c[1] - d}, {c[0] + 2.0 * d, c[1] - d},
          {c[0], c[1] + 2.0 * d}};
    triangles.push_back({{0, 1, 2}, {-1, -1, -1}, {false, false, false},
                         true, false});
    last_ = 0;
  }

  static double Orient(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
  }

  /* > 0 if d is inside the circle through the counterclockwise a, b, c */
  static double InCircle(const Vec2& a, const Vec2& b, const Vec2& c,
                         const Vec2& d) {
    const double ax = a[0] - d[0], ay = a[1] - d[1];
    const double bx = b[0] - d[0], by = b[1] - d[1];
    const double cx = c[0] - d[0], cy = c[1] - d[1];
    return (ax * ax + ay * ay) * (bx * cy - cx * by) -
           (bx * bx + by * by) * (ax * cy - cx * ay) +
           (cx * cx + cy * cy) * (ax * by - bx * ay);
  }

  /* the triangle containing p, by walking from start; -1 if the walk
  fails */
  int Locate(const Vec2& p, int start) const {
    int t = start >= 0 && triangles[start].alive ? start : last_;
    for(size_t step = 0; step < 4 * triangles.size() + 16; step++) {
      const Triangle& tri = triangles[t];
      int next = -1;
      for(int k = 0; k < 3; k++) {
        const int i = (k + static_cast<int>(step)) % 3;
        if(Orient(xy[tri.v[(i + 1) % 3]], xy[tri.v[(i + 2) % 3]], p) < 0.0) {
          next = tri.n[i];
          break;
        }
      }
      if(next < 0) {
        return t;
      }
      t = next;
    }
    return -1;
  }

  /*! the cavity of a new point p in triangle t: the triangles whose
   * circumcircles hold p, reached without crossing a constrained edge */
  std::vector<int> Cavity(const Vec2& p, int t) const {
    std::vector<int> cavity{t};
    std::unordered_map<int, bool> seen{{t, true}};
    for(size_t k = 0; k < cavity.size(); k++) {
      const Triangle& tri = triangles[cavity[k]];
      for(int i = 0; i < 3; i++) {
        const int u = tri.n[i];
        if(u < 0 || tri.c[i] || seen.count(u)) {
          continue;
        }
        seen[u] = true;
        const Triangle& o = triangles[u];
        if(InCircle(xy[o.v[0]], xy[o.v[1]], xy[o.v[2]], p) > 0.0) {
          cavity.push_back(u);
        }
      }
    }
    return cavity;
  }

  /*! insert the point p (index of xy) into the cavity: new triangles from
   * p to each edge of its boundary; they take the inside flag of the
   * cavity's first triangle; returns the new triangles */
  std::vector<int> Insert(int p, const std::vector<int>& cavity) {
    std::unordered_map<int, bool> in;
    for(int t : cavity) {
      in[t] = true;
    }
    const bool inside = triangles[cavity[0]].inside;
    struct Rim {
      int a, b, outside;
      bool constrained;
    };
    std::vector<Rim> rim;
    for(int t : cavity) {
      const Triangle& tri = triangles[t];
      for(int i = 0; i < 3; i++) {
        if(tri.n[i] < 0 || !in.count(tri.n[i])) {
          rim.push_back({tri.v[(i + 1) % 3], tri.v[(i + 2) % 3], tri.n[i],
                         tri.c[i]});
        }
      }
    }
    for(int t : cavity) {
      triangles[t].alive = false;
    }
    /* new triangle (p, a, b) for each rim edge a -> b */
    std::unordered_map<int, int> by_a, by_b;
    std::vector<int> made;
    for(const Rim& r : rim) {
      int slot;
      if(!free_.empty()) {
        slot = free_.back();
        free_.pop_back();
      } else {
        slot = static_cast<int>(triangles.size());
        triangles.push_back({});
      }
      Triangle& tri = triangles[slot];
      tri = {{p, r.a, r.b}, {r.outside, -1, -1}, {r.constrained, false,
                                                   false},
             true, inside};
      by_a[r.a] = slot;
      by_b[r.b] = slot;
      made.push_back(slot);
      if(r.outside >= 0) {
        SetNeighbor(r.outside, r.a, r.b, slot);
      }
    }
    for(int slot : made) {
      Triangle& tri = triangles[slot];
      /* edge opposite a is (b, p): the triangle whose a is our b */
      tri.n[1] = by_a.at(tri.v[2]);
      /* edge opposite b is (p, a): the triangle whose b is our a */
      tri.n[2] = by_b.at(tri.v[1]);
    }
    for(int t : cavity) {
      free_.push_back(t);
    }
    last_ = made[0];
    return made;
  }

  /* make the edge (a, b) of triangle t point to u */
  void SetNeighbor(int t, int a, int b, int u) {
    Triangle& tri = triangles[t];
    for(int i = 0; i < 3; i++) {
      const int x = tri.v[(i + 1) % 3], y = tri.v[(i + 2) % 3];
      if((x == a && y == b) || (x == b && y == a)) {
        tri.n[i] = u;
        return;
      }
    }
  }

  /* the edge index of (a, b) in t, or -1 */
  int EdgeIndex(int t, int a, int b) const {
    const Triangle& tri = triangles[t];
    for(int i = 0; i < 3; i++) {
      const int x = tri.v[(i + 1) % 3], y = tri.v[(i + 2) % 3];
      if((x == a && y == b) || (x == b && y == a)) {
        return i;
      }
    }
    return -1;
  }

  /* flip edge i of t with the triangle across it; false if the
  quadrilateral is not convex */
  bool Flip(int t, int i) {
    const int u = triangles[t].n[i];
    if(u < 0) {
      return false;
    }
    const Triangle tt = triangles[t];
    const int a = tt.v[i], b = tt.v[(i + 1) % 3], c = tt.v[(i + 2) % 3];
    const Triangle uu = triangles[u];
    const int j = EdgeIndex(u, b, c);
    const int d = uu.v[j];
    if(Orient(xy[a], xy[b], xy[d]) <= 0.0 ||
       Orient(xy[a], xy[d], xy[c]) <= 0.0) {
      return false;
    }
    /* neighbors around the quadrilateral */
    const int ic = EdgeIndex(u, b, d), ib = EdgeIndex(u, d, c);
    const int nu_c = uu.n[ic], nu_b = uu.n[ib];
    const bool cu_c = uu.c[ic], cu_b = uu.c[ib];
    const int nt_c = tt.n[(i + 2) % 3], nt_b = tt.n[(i + 1) % 3];
    const bool ct_c = tt.c[(i + 2) % 3], ct_b = tt.c[(i + 1) % 3];
    triangles[t] = {{a, b, d}, {nu_c, u, nt_c}, {cu_c, false, ct_c}, true,
                    tt.inside};
    triangles[u] = {{a, d, c}, {nu_b, nt_b, t}, {cu_b, ct_b, false}, true,
                    tt.inside};
    if(nu_c >= 0) {
      SetNeighbor(nu_c, b, d, t);
    }
    if(nt_c >= 0) {
      SetNeighbor(nt_c, a, b, t);
    }
    if(nu_b >= 0) {
      SetNeighbor(nu_b, d, c, u);
    }
    if(nt_b >= 0) {
      SetNeighbor(nt_b, c, a, u);
    }
    return true;
  }

  static bool Cross(const Vec2& a, const Vec2& b, const Vec2& p,
                    const Vec2& q) {
    const double o1 = Orient(a, b, p), o2 = Orient(a, b, q);
    const double o3 = Orient(p, q, a), o4 = Orient(p, q, b);
    return ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
           ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0));
  }

  /* make (a, b) an edge of the triangulation, marked constrained on both
  sides; false if flips cannot recover it */
  bool Constrain(int a, int b) {
    for(int round = 0; round < 1000; round++) {
      bool found = false;
      std::vector<std::pair<int, int>> crossing;
      for(int t = 0; t < static_cast<int>(triangles.size()); t++) {
        if(!triangles[t].alive) {
          continue;
        }
        const int i = EdgeIndex(t, a, b);
        if(i >= 0) {
          triangles[t].c[i] = true;
          found = true;
          continue;
        }
        for(int k = 0; k < 3; k++) {
          const int x = triangles[t].v[(k + 1) % 3];
          const int y = triangles[t].v[(k + 2) % 3];
          if(x != a && x != b && y != a && y != b &&
             Cross(xy[a], xy[b], xy[x], xy[y])) {
            crossing.push_back({t, k});
          }
        }
      }
      if(found) {
        return true;
      }
      if(crossing.empty()) {
        return false;
      }
      bool flipped = false;
      for(const auto& [t, k] : crossing) {
        if(triangles[t].alive && !triangles[t].c[k] && triangles[t].n[k] >= 0) {
          const int x = triangles[t].v[(k + 1) % 3];
          const int y = triangles[t].v[(k + 2) % 3];
          if(Cross(xy[a], xy[b], xy[x], xy[y]) && Flip(t, k)) {
            flipped = true;
            break;
          }
        }
      }
      if(!flipped) {
        return false;
      }
    }
    return false;
  }

  /* inside: crossing a constrained edge from the triangles at the first
  three (enclosing) points toggles inside and outside */
  void Classify() {
    std::vector<int> state(triangles.size(), -1);
    std::deque<int> queue;
    for(int t = 0; t < static_cast<int>(triangles.size()); t++) {
      const Triangle& tri = triangles[t];
      if(tri.alive && (tri.v[0] < 3 || tri.v[1] < 3 || tri.v[2] < 3)) {
        state[t] = 0;
        queue.push_back(t);
      }
    }
    while(!queue.empty()) {
      const int t = queue.front();
      queue.pop_front();
      const Triangle& tri = triangles[t];
      for(int i = 0; i < 3; i++) {
        const int u = tri.n[i];
        if(u < 0 || state[u] >= 0) {
          continue;
        }
        state[u] = tri.c[i] ? 1 - state[t] : state[t];
        queue.push_back(u);
      }
    }
    for(int t = 0; t < static_cast<int>(triangles.size()); t++) {
      triangles[t].inside = triangles[t].alive && state[t] == 1;
    }
  }

private:
  int last_ = 0;
  std::vector<int> free_;
};

/* a face's surface in its scaled parameters: x = su (u - u0), y = sv (v -
v0), so that lengths in (x, y) are roughly lengths on the surface */
struct FaceMap {
  const Model* model;
  const Surface* surface;
  double u0, v0, su, sv;

  Vec2 ToXy(const Vec2& uv) const {
    return {su * (uv[0] - u0), sv * (uv[1] - v0)};
  }
  Vec2 ToUv(const Vec2& p) const {
    return {u0 + p[0] / su, v0 + p[1] / sv};
  }
  Vec3 Point(const Vec2& uv) const {
    return SurfacePoint(*model, *surface, uv[0], uv[1]);
  }
};

static double TriangleMinAngle(const Vec3& a, const Vec3& b, const Vec3& c) {
  const Vec3 p[3] = {a, b, c};
  double smallest = kPi;
  for(int i = 0; i < 3; i++) {
    const Vec3 e1 = Sub(p[(i + 1) % 3], p[i]);
    const Vec3 e2 = Sub(p[(i + 2) % 3], p[i]);
    const double n1 = Norm(e1), n2 = Norm(e2);
    if(n1 == 0.0 || n2 == 0.0) {
      continue;
    }
    smallest =
      std::min(smallest, std::acos(std::clamp(Dot(e1, e2) / (n1 * n2), -1.0,
                                              1.0)));
  }
  return smallest;
}

Triangulation Triangulate(const Model& model, int body,
                          const TriangulateOptions& options_in) {
  TriangulateOptions options = options_in;
  if(!(options.element_size > 0.0)) {
    throw shift::core::Exception("Triangulate: element_size must be positive");
  }
  const double h = options.element_size;
  if(options.minimum_element_size <= 0.0) {
    options.minimum_element_size = h / 20.0;
  }
  if(options.sag <= 0.0) {
    options.sag = h / 20.0;
  }
  if(body < 0 || body >= static_cast<int>(model.bodies.size())) {
    throw shift::core::Exception("Triangulate: no body " +
                                 std::to_string(body));
  }

  Triangulation result;
  Welder welder(result.points, options.tolerance);

  std::vector<int> faces;
  for(int s : model.bodies[body].shells) {
    for(int f : model.shells[s].faces) {
      faces.push_back(f);
    }
  }

  /* each edge divided once: its parameters and points */
  struct Division {
    std::vector<double> t;
    std::vector<int> point;
  };
  std::unordered_map<int, Division> divisions;
  /* the edges divided so far, by their end points and three points
  between */
  struct Twin {
    int edge, a, b;
    Vec3 between[3];
  };
  std::vector<Twin> twins;
  /* the edge whose points each edge uses: itself, or its twin */
  std::unordered_map<int, int> canonical;
  std::unordered_map<int, int> uses;
  auto divide = [&](int edge_index) -> const Division& {
    auto found = divisions.find(edge_index);
    if(found != divisions.end()) {
      return found->second;
    }
    const Edge& e = model.edges[edge_index];
    if(!e.has_range) {
      throw shift::core::Exception("Triangulate: edge " +
                                   std::to_string(e.step_id) +
                                   " has no parameter range (repair the "
                                   "model first)");
    }
    /* fine samples, then a density per piece from its length, the turn of
    the tangent and the sag; points at equal steps of the summed density */
    int fine = 64;
    if(e.curve >= 0 && model.curves[e.curve].type == Curve3::Type::kNurbs) {
      fine = std::clamp(
        8 * static_cast<int>(model.curves[e.curve].nurbs.knots.size()), 64,
        4096);
    }
    std::vector<Vec3> p(fine + 1);
    for(int i = 0; i <= fine; i++) {
      p[i] = EdgePoint(model, e, Lerp(e.range[0], e.range[1],
                                       static_cast<double>(i) / fine));
    }
    std::vector<double> density(fine, 0.0);
    double total = 0.0;
    for(int i = 0; i < fine; i++) {
      const double length = Norm(Sub(p[i + 1], p[i]));
      double turn = 0.0;
      if(i > 0) {
        const Vec3 a = Sub(p[i], p[i - 1]), b = Sub(p[i + 1], p[i]);
        const double na = Norm(a), nb = Norm(b);
        if(na > 0.0 && nb > 0.0) {
          turn = std::acos(std::clamp(Dot(a, b) / (na * nb), -1.0, 1.0));
        }
      }
      double d = std::max(length / h, turn / options.max_turn);
      d = std::max(d, std::sqrt(length * turn / (8.0 * options.sag)));
      d = std::min(d, length / options.minimum_element_size);
      density[i] = d;
      total += d;
    }
    Division division;

    /* an edge along the same curve as one already divided (a seam written
    as two edges, one for each side): the same points, at the parameters of
    this edge's curve that are nearest them */
    const Vec3 first = e.start >= 0 ? model.vertices[e.start].point : p[0];
    const Vec3 final = e.end >= 0 ? model.vertices[e.end].point : p[fine];
    const int a = welder.Add(first), b = welder.Add(final);
    const Vec3 between[3] = {p[fine / 4], p[fine / 2], p[3 * fine / 4]};
    for(const Twin& twin : twins) {
      const double near =
        10.0 * std::max({options.tolerance, e.tolerance,
                         model.edges[twin.edge].tolerance});
      bool same = twin.a == a && twin.b == b;
      bool reversed = twin.a == b && twin.b == a;
      for(int i = 0; i < 3; i++) {
        same = same && Norm(Sub(between[i], twin.between[i])) <= near;
        reversed =
          reversed && Norm(Sub(between[i], twin.between[2 - i])) <= near;
      }
      if(!same && !reversed) {
        continue;
      }
      const Division& other = divisions.at(twin.edge);
      const int n = static_cast<int>(other.point.size());
      for(int k = 0; k < n; k++) {
        const int m = same ? k : n - 1 - k;
        double t = Lerp(e.range[0], e.range[1], static_cast<double>(k) /
                                                  (n - 1));
        if(k == 0) {
          t = e.range[0];
        } else if(k == n - 1) {
          t = e.range[1];
        } else if(e.curve >= 0) {
          const double guess = t;
          if(!ProjectToCurve(model.curves[e.curve],
                             result.points[other.point[m]], t, &guess)) {
            t = guess;
          }
        }
        division.t.push_back(t);
        division.point.push_back(other.point[m]);
      }
      canonical[edge_index] = twin.edge;
      return divisions.emplace(edge_index, std::move(division)).first->second;
    }
    twins.push_back({edge_index, a, b, {between[0], between[1], between[2]}});
    canonical[edge_index] = edge_index;

    const bool closed = e.start == e.end;
    const int n = std::max(closed ? 3 : 1, static_cast<int>(std::ceil(total)));
    division.t.push_back(e.range[0]);
    double sum = 0.0;
    int piece = 0;
    for(int k = 1; k < n; k++) {
      const double target = total * k / n;
      while(piece < fine - 1 && sum + density[piece] < target) {
        sum += density[piece];
        piece++;
      }
      const double f =
        density[piece] > 0.0 ? (target - sum) / density[piece] : 0.0;
      const double s = (piece + std::clamp(f, 0.0, 1.0)) / fine;
      division.t.push_back(Lerp(e.range[0], e.range[1], s));
    }
    division.t.push_back(e.range[1]);
    for(size_t k = 0; k < division.t.size(); k++) {
      Vec3 q;
      if(k == 0 && e.start >= 0) {
        q = model.vertices[e.start].point;
      } else if(k + 1 == division.t.size() && e.end >= 0) {
        q = model.vertices[e.end].point;
      } else {
        q = EdgePoint(model, e, division.t[k]);
      }
      division.point.push_back(welder.Add(q));
    }
    for(size_t k = 0; k + 1 < division.point.size(); k++) {
      result.edge_of[std::minmax(division.point[k], division.point[k + 1])] =
        edge_index;
    }
    return divisions.emplace(edge_index, std::move(division)).first->second;
  };

  for(int face_index : faces) {
    const Face& face = model.faces[face_index];
    if(face.trims.empty()) {
      throw shift::core::Exception("Triangulate: face " +
                                   std::to_string(face.step_id) +
                                   " has no trim loops (repair the model "
                                   "first)");
    }
    const Surface& surface = model.surfaces[face.surface];

    /* the boundary: (u, v) and point of each vertex, loop by loop */
    std::vector<std::vector<Vec2>> loop_uv;
    std::vector<std::vector<int>> loop_point;
    for(const TrimLoop& loop : face.trims) {
      std::vector<Vec2> uvs;
      std::vector<int> ids;
      /* the point where the previous trim curve ended */
      int previous_end = -1;
      for(const TrimCurve& c : loop.curves) {
        if(c.kind == TrimCurve::Kind::kGap) {
          throw shift::core::Exception(
            "Triangulate: face " + std::to_string(face.step_id) +
            " has a gap between its edges, so its surface is not closed");
        }
        if(c.kind == TrimCurve::Kind::kEdge) {
          const Edge& e = model.edges[c.edge];
          const Pcurve& pc = e.pcurves[c.pcurve];
          const Division& division = divide(c.edge);
          uses[canonical.at(c.edge)]++;
          const bool forward =
            std::fabs(c.s0 - pc.range[0]) <= std::fabs(c.s0 - pc.range[1]);
          const int n = static_cast<int>(division.t.size());
          for(int k = 0; k + 1 < n; k++) {
            const int m = forward ? k : n - 1 - k;
            const double span = e.range[1] - e.range[0];
            const double f =
              span != 0.0 ? (division.t[m] - e.range[0]) / span : 0.0;
            Vec2 guess = CurvePoint<2>(pc.curve, Lerp(pc.range[0],
                                                      pc.range[1], f));
            guess = {guess[0] + c.offset[0], guess[1] + c.offset[1]};
            /* the trim's own parameters at its ends, so that the loop is
            continuous; between them, those of the pcurve, corrected by
            projection where it is not exact, if the projection stays near
            (it may jump where many parameters give nearly one point) */
            Vec2 uv = guess;
            if(k == 0) {
              uv = TrimPoint(model, c, 0.0);
            } else if(!pc.same_parameter) {
              const Vec3& q = result.points[division.point[m]];
              const double step = 0.1 * (std::fabs(c.s1 - c.s0) / (n - 1));
              if(!ProjectToSurface(model, surface, q, uv, &guess) ||
                 std::fabs(uv[0] - guess[0]) + std::fabs(uv[1] - guess[1]) >
                   step) {
                uv = guess;
              }
            }
            uvs.push_back(uv);
            ids.push_back(division.point[m]);
          }
          previous_end = division.point[forward ? n - 1 : 0];
          continue;
        }
        /* a straight segment in (u, v): a join (one point), a seam or a
        singular line (all at one point), divided by its length on the
        surface, or by its length in (u, v) where that is zero */
        int n = 1;
        if(c.kind != TrimCurve::Kind::kJoin) {
          double length = 0.0;
          Vec3 last = SurfacePoint(model, surface, c.a[0], c.a[1]);
          for(int i = 1; i <= 16; i++) {
            const double f = i / 16.0;
            const Vec3 q = SurfacePoint(model, surface, Lerp(c.a[0], c.b[0], f),
                                        Lerp(c.a[1], c.b[1], f));
            length += Norm(Sub(q, last));
            last = q;
          }
          n = std::max(1, static_cast<int>(std::ceil(length / h)));
          if(c.kind == TrimCurve::Kind::kSingular) {
            n = 8;
          }
        }
        /* a singular line is one point, even where the surface only nearly
        collapses there */
        int singular_point = -1;
        for(int k = 0; k < n; k++) {
          const double f = static_cast<double>(k) / n;
          const Vec2 uv{Lerp(c.a[0], c.b[0], f), Lerp(c.a[1], c.b[1], f)};
          uvs.push_back(uv);
          int id = singular_point;
          if(id < 0 && k == 0) {
            id = previous_end;
          }
          if(id < 0) {
            id = welder.Add(SurfacePoint(model, surface, uv[0], uv[1]));
          }
          if(c.kind == TrimCurve::Kind::kSingular) {
            singular_point = id;
          }
          ids.push_back(id);
        }
        previous_end = singular_point >= 0
                         ? singular_point
                         : welder.Add(SurfacePoint(model, surface, c.b[0],
                                                   c.b[1]));
      }
      loop_uv.push_back(uvs);
      loop_point.push_back(ids);
    }

    /* the scale of the parameters: mean lengths of the derivatives over the
    face's box */
    Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
    for(const std::vector<Vec2>& uvs : loop_uv) {
      for(const Vec2& uv : uvs) {
        lo = {std::min(lo[0], uv[0]), std::min(lo[1], uv[1])};
        hi = {std::max(hi[0], uv[0]), std::max(hi[1], uv[1])};
      }
    }
    double su = 0.0, sv = 0.0;
    for(int i = 0; i < 5; i++) {
      for(int j = 0; j < 5; j++) {
        Vec3 du, dv;
        SurfacePoint(model, surface, Lerp(lo[0], hi[0], i / 4.0),
                     Lerp(lo[1], hi[1], j / 4.0), &du, &dv);
        su += Norm(du) / 25.0;
        sv += Norm(dv) / 25.0;
      }
    }
    if(!(su > 0.0)) {
      su = 1.0;
    }
    if(!(sv > 0.0)) {
      sv = 1.0;
    }
    const FaceMap map{&model, &surface, lo[0], lo[1], su, sv};

    /* the constrained triangulation of the boundary */
    Cdt cdt;
    cdt.Start(map.ToXy(lo), map.ToXy(hi));
    std::vector<int> point_of{-1, -1, -1};
    std::vector<Vec2> uv_of{{0, 0}, {0, 0}, {0, 0}};
    std::vector<char> on_boundary{1, 1, 1};
    std::vector<std::vector<int>> loop_local;
    const Vec2 extent = map.ToXy(hi);
    const double coincident = 1e-9 * (std::fabs(extent[0]) +
                                      std::fabs(extent[1]));
    for(size_t l = 0; l < loop_uv.size(); l++) {
      std::vector<int> local;
      for(size_t k = 0; k < loop_uv[l].size(); k++) {
        const Vec2 p = map.ToXy(loop_uv[l][k]);
        /* a point at the same (x, y) as one already placed (the ends of a
        piece of zero length) is that point */
        bool placed = false;
        for(size_t i = 3; i < cdt.xy.size() && !placed; i++) {
          if(std::fabs(cdt.xy[i][0] - p[0]) + std::fabs(cdt.xy[i][1] - p[1]) <=
             coincident) {
            if(local.empty() || local.back() != static_cast<int>(i)) {
              local.push_back(static_cast<int>(i));
            }
            placed = true;
          }
        }
        if(placed) {
          continue;
        }
        const int t = cdt.Locate(p, -1);
        if(t < 0) {
          throw shift::core::Exception("Triangulate: face " +
                                       std::to_string(face.step_id) +
                                       ": a boundary point could not be "
                                       "placed");
        }
        const int index = static_cast<int>(cdt.xy.size());
        cdt.xy.push_back(p);
        point_of.push_back(loop_point[l][k]);
        uv_of.push_back(loop_uv[l][k]);
        on_boundary.push_back(1);
        cdt.Insert(index, cdt.Cavity(p, t));
        local.push_back(index);
      }
      loop_local.push_back(local);
    }
    /* spikes: a loop that runs out and back along one line (a seam or a
    singular line traversed both ways, which Repair writes to join the
    sides of a periodic surface) bounds no area there; repeated points and
    collinear reversals are removed until none are left */
    for(std::vector<int>& local : loop_local) {
      bool changed = true;
      while(changed && local.size() >= 3) {
        changed = false;
        for(size_t k = 0; k < local.size() && local.size() >= 3; k++) {
          const size_t n = local.size();
          const int before = local[(k + n - 1) % n];
          const int here = local[k];
          const int after = local[(k + 1) % n];
          bool remove = here == after;
          if(!remove) {
            const Vec2& a = cdt.xy[before];
            const Vec2& b = cdt.xy[here];
            const Vec2& c = cdt.xy[after];
            const double ux = b[0] - a[0], uy = b[1] - a[1];
            const double wx = c[0] - b[0], wy = c[1] - b[1];
            const double cross = ux * wy - uy * wx;
            const double dot = ux * wx + uy * wy;
            const double scale = std::sqrt((ux * ux + uy * uy) *
                                           (wx * wx + wy * wy));
            remove = before == after ||
                     (dot < 0.0 && std::fabs(cross) <= 1e-9 * scale);
          }
          if(remove) {
            local.erase(local.begin() + static_cast<long>(k));
            changed = true;
          }
        }
      }
    }
    /* the edges already in the triangulation, found once; only the missing
    ones are recovered by flips */
    auto key = [](int a, int b) {
      return (static_cast<long long>(std::min(a, b)) << 32) |
             static_cast<long long>(std::max(a, b));
    };
    std::unordered_map<long long, std::vector<std::pair<int, int>>> sides;
    for(int t = 0; t < static_cast<int>(cdt.triangles.size()); t++) {
      const Cdt::Triangle& o = cdt.triangles[t];
      if(!o.alive) {
        continue;
      }
      for(int i = 0; i < 3; i++) {
        sides[key(o.v[(i + 1) % 3], o.v[(i + 2) % 3])].push_back({t, i});
      }
    }
    for(std::vector<int>& local : loop_local) {
      if(local.size() < 3) {
        continue;
      }
      for(size_t k = 0; k < local.size(); k++) {
        const int a = local[k], b = local[(k + 1) % local.size()];
        auto found = sides.find(key(a, b));
        if(found != sides.end()) {
          bool marked = false;
          for(const auto& [t, i] : found->second) {
            if(cdt.EdgeIndex(t, a, b) == i && cdt.triangles[t].alive) {
              cdt.triangles[t].c[i] = true;
              marked = true;
            }
          }
          if(marked) {
            continue;
          }
        }
        /* the pieces of a singular line, all at one point, still bound the
        face in (u, v) */
        if(!cdt.Constrain(a, b)) {
          throw shift::core::Exception(
            "Triangulate: face " + std::to_string(face.step_id) +
            ": an edge of its boundary could not be recovered (do its trim "
            "loops cross?)");
        }
      }
    }
    cdt.Classify();

    /* refinement: points inside the face until the triangles are short,
    near the surface and not thin */
    auto point3 = [&](int local) -> Vec3 {
      return point_of[local] >= 0 ? result.points[point_of[local]]
                                  : Vec3{0, 0, 0};
    };
    std::deque<int> queue;
    for(int t = 0; t < static_cast<int>(cdt.triangles.size()); t++) {
      if(cdt.triangles[t].inside) {
        queue.push_back(t);
      }
    }
    auto distinct = [&](int a, int b) { return point_of[a] != point_of[b]; };
    while(!queue.empty()) {
      if(static_cast<int>(result.points.size()) >= options.max_points) {
        throw shift::core::Exception("Triangulate: more than " +
                                     std::to_string(options.max_points) +
                                     " points (element_size too small?)");
      }
      const int t = queue.front();
      queue.pop_front();
      const Cdt::Triangle tri = cdt.triangles[t];
      if(!tri.alive || !tri.inside) {
        continue;
      }
      const Vec3 a = point3(tri.v[0]), b = point3(tri.v[1]),
                 c = point3(tri.v[2]);
      const double lengths[3] = {Norm(Sub(b, c)), Norm(Sub(c, a)),
                                 Norm(Sub(a, b))};
      const double longest = std::max({lengths[0], lengths[1], lengths[2]});
      double shortest = longest;
      for(int i = 0; i < 3; i++) {
        if(distinct(tri.v[(i + 1) % 3], tri.v[(i + 2) % 3])) {
          shortest = std::min(shortest, lengths[i]);
        }
      }
      const Vec2 centre_uv{
        (uv_of[tri.v[0]][0] + uv_of[tri.v[1]][0] + uv_of[tri.v[2]][0]) / 3.0,
        (uv_of[tri.v[0]][1] + uv_of[tri.v[1]][1] + uv_of[tri.v[2]][1]) / 3.0};
      const Vec3 centre3{(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0,
                         (a[2] + b[2] + c[2]) / 3.0};
      const double sag = Norm(Sub(map.Point(centre_uv), centre3));
      const bool big = longest > h * 1.0001;
      const bool deep = sag > options.sag &&
                        longest > 2.0 * options.minimum_element_size;
      const bool thin = TriangleMinAngle(a, b, c) < 20.0 * kPi / 180.0 &&
                        shortest > 2.0 * options.minimum_element_size;
      if(!big && !deep && !thin) {
        continue;
      }
      /* the circumcentre in (x, y), unless it falls outside the face or
      encroaches a constrained edge; then the centroid, for a triangle that
      is too big or too far from the surface */
      const Vec2& pa = cdt.xy[tri.v[0]];
      const Vec2& pb = cdt.xy[tri.v[1]];
      const Vec2& pc = cdt.xy[tri.v[2]];
      const double d = 2.0 * Cdt::Orient(pa, pb, pc);
      std::vector<Vec2> candidates;
      if(d > 0.0) {
        const double a2 = pa[0] * pa[0] + pa[1] * pa[1];
        const double b2 = pb[0] * pb[0] + pb[1] * pb[1];
        const double c2 = pc[0] * pc[0] + pc[1] * pc[1];
        candidates.push_back(
          {(a2 * (pb[1] - pc[1]) + b2 * (pc[1] - pa[1]) +
            c2 * (pa[1] - pb[1])) / d,
           (a2 * (pc[0] - pb[0]) + b2 * (pa[0] - pc[0]) +
            c2 * (pb[0] - pa[0])) / d});
      }
      if(big || deep) {
        candidates.push_back({(pa[0] + pb[0] + pc[0]) / 3.0,
                              (pa[1] + pb[1] + pc[1]) / 3.0});
      }
      for(size_t ci = 0; ci < candidates.size(); ci++) {
        const Vec2& p = candidates[ci];
        const bool circumcentre = d > 0.0 && ci == 0;
        const int at = cdt.Locate(p, t);
        if(at < 0 || !cdt.triangles[at].inside) {
          continue;
        }
        const std::vector<int> cavity = cdt.Cavity(p, at);
        const Vec2 uv = map.ToUv(p);
        Vec3 du, dv;
        const Vec3 q = SurfacePoint(model, surface, uv[0], uv[1], &du, &dv);
        /* where the surface collapses (a degenerate piece of its curve),
        points would make triangles of no area */
        bool rejected = Norm(Cross(du, dv)) < 1e-6 * su * sv;
        for(int k : cavity) {
          const Cdt::Triangle& o = cdt.triangles[k];
          rejected = rejected || !o.inside;
          for(int i = 0; i < 3 && !rejected; i++) {
            if(Norm(Sub(point3(o.v[i]), q)) < options.minimum_element_size) {
              rejected = true;
            }
            /* a circumcentre in the diametral circle of a constrained edge
            would make a thin triangle against it */
            if(circumcentre && o.c[i]) {
              const Vec2& x = cdt.xy[o.v[(i + 1) % 3]];
              const Vec2& y = cdt.xy[o.v[(i + 2) % 3]];
              const Vec2 m{0.5 * (x[0] + y[0]), 0.5 * (x[1] + y[1])};
              const double r2 = 0.25 * ((x[0] - y[0]) * (x[0] - y[0]) +
                                        (x[1] - y[1]) * (x[1] - y[1]));
              const double d2 = (p[0] - m[0]) * (p[0] - m[0]) +
                                (p[1] - m[1]) * (p[1] - m[1]);
              rejected = d2 < r2;
            }
          }
        }
        if(rejected) {
          continue;
        }
        const int index = static_cast<int>(cdt.xy.size());
        cdt.xy.push_back(p);
        uv_of.push_back(uv);
        point_of.push_back(static_cast<int>(result.points.size()));
        on_boundary.push_back(0);
        result.points.push_back(q);
        for(int k : cdt.Insert(index, cavity)) {
          queue.push_back(k);
        }
        break;
      }
    }

    /* smoothing: each inner point to the mean of its neighbours in (x, y),
    where no triangle around it turns over */
    std::vector<std::vector<int>> around(cdt.xy.size());
    for(int t = 0; t < static_cast<int>(cdt.triangles.size()); t++) {
      const Cdt::Triangle& o = cdt.triangles[t];
      if(o.alive && o.inside) {
        for(int i = 0; i < 3; i++) {
          around[o.v[i]].push_back(t);
        }
      }
    }
    for(int pass = 0; pass < 3; pass++) {
      for(size_t p = 0; p < cdt.xy.size(); p++) {
        if(on_boundary[p] || around[p].empty()) {
          continue;
        }
        Vec2 mean{0.0, 0.0};
        int count = 0;
        for(int t : around[p]) {
          for(int i = 0; i < 3; i++) {
            const int v = cdt.triangles[t].v[i];
            if(v != static_cast<int>(p)) {
              mean[0] += cdt.xy[v][0];
              mean[1] += cdt.xy[v][1];
              count++;
            }
          }
        }
        mean = {mean[0] / count, mean[1] / count};
        const Vec2 old = cdt.xy[p];
        cdt.xy[p] = mean;
        bool ok = true;
        for(int t : around[p]) {
          const Cdt::Triangle& o = cdt.triangles[t];
          if(Cdt::Orient(cdt.xy[o.v[0]], cdt.xy[o.v[1]], cdt.xy[o.v[2]]) <=
             0.0) {
            ok = false;
          }
        }
        if(!ok) {
          cdt.xy[p] = old;
          continue;
        }
        uv_of[p] = map.ToUv(mean);
        result.points[point_of[p]] = map.Point(uv_of[p]);
      }
    }

    /* the triangles, outward: counterclockwise in (u, v) is along the
    surface's normal, which points out of the solid if same_sense */
    for(const Cdt::Triangle& o : cdt.triangles) {
      if(!o.alive || !o.inside) {
        continue;
      }
      std::array<int, 3> ids{point_of[o.v[0]], point_of[o.v[1]],
                             point_of[o.v[2]]};
      if(ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0]) {
        continue;
      }
      std::array<Vec2, 3> uvs{uv_of[o.v[0]], uv_of[o.v[1]], uv_of[o.v[2]]};
      if(!face.same_sense) {
        std::swap(ids[1], ids[2]);
        std::swap(uvs[1], uvs[2]);
      }
      result.triangles.push_back(ids);
      result.uv.push_back(uvs);
      result.face.push_back(face_index);
    }
  }

  /* closed: every edge (with its twin) bounds two faces, or one face
  twice (a seam) */
  for(const auto& [edge, count] : uses) {
    if(count != 2) {
      throw shift::core::Exception(
        "Triangulate: edge " + std::to_string(model.edges[edge].step_id) +
        " bounds " + std::to_string(count) +
        " face sides, not 2, so the shell of body '" +
        model.bodies[body].name +
        "' is not closed (faces meet along edges they do not share)");
    }
  }

  /* a solid whose faces all point in (written inside out) encloses a
  negative volume: turn its triangles to point out */
  double enclosed = 0.0;
  for(const std::array<int, 3>& t : result.triangles) {
    enclosed += Dot(result.points[t[0]],
                    Cross(result.points[t[1]], result.points[t[2]])) /
                6.0;
  }
  if(enclosed < 0.0) {
    for(size_t k = 0; k < result.triangles.size(); k++) {
      std::swap(result.triangles[k][1], result.triangles[k][2]);
      std::swap(result.uv[k][1], result.uv[k][2]);
    }
  }
  return result;
}

} // namespace cad
} // namespace core
} // namespace shift
