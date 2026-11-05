#include "shift/core/collision_geometry/epa.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/gjk.h"

/* polytope growth stops when the support point along the closest face's
normal is within this fraction of the geometries' size of the face */
static constexpr double kRelativeTolerance = 1.0e-6;
static constexpr int kMaxIterations = 512;

/*! a vertex of the polytope: w = b - a, with the support points of a and b
 * that produced it, all in the inertial frame */
struct EpaVertex {
  ksl_vec3_t w;
  ksl_vec3_t a;
  ksl_vec3_t b;
};

/*! a triangle of the polytope, wound counter-clockwise seen from outside,
 * with its outward unit normal and its distance from the origin */
struct EpaFace {
  int v[3];
  ksl_vec3_t n;
  double distance;
  bool valid;
};

static inline ksl_vec3_t Sub(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x - b.x, a.y - b.y, a.z - b.z}};
}

static inline ksl_vec3_t Add(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x + b.x, a.y + b.y, a.z + b.z}};
}

static inline ksl_vec3_t Scale(double k, const ksl_vec3_t& a) {
  return {{k * a.x, k * a.y, k * a.z}};
}

static inline ksl_vec3_t Cross(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
           a.x * b.y - a.y * b.x}};
}

static inline double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline double Norm(const ksl_vec3_t& a) {
  return std::sqrt(Dot(a, a));
}

/*! support point of b - a along d */
static EpaVertex Support(shift::core::collision_geometry::ContactPair& p,
                         const ksl_vec3_t& d) {
  EpaVertex v;
  ksl_vec3_t local;
  p.a->Support(Scale(-1.0, d), local, v.a, p.hint_a);
  p.b->Support(d, local, v.b, p.hint_b);
  v.w = Sub(v.b, v.a);
  return v;
}

/*! coordinates of an inertial point in a geometry's reference frame,
 * R^T * (p - t) */
static ksl_vec3_t ToLocal(const shift::core::collision_geometry::Base& g,
                          const ksl_vec3_t& p) {
  const ksl_SE3_t& d = g.get_d_absolute();
  const ksl_vec3_t r = Sub(p, d.t);
  ksl_vec3_t local;
  for(int i = 0; i < 3; i++) {
    local.at[i] = Dot(d.R.v[i], r);
  }
  return local;
}

/*! face (i, j, k); false if it has no area */
static bool MakeFace(const std::vector<EpaVertex>& v, int i, int j, int k,
                     EpaFace& f) {
  f.v[0] = i;
  f.v[1] = j;
  f.v[2] = k;
  f.valid = true;
  const ksl_vec3_t n = Cross(Sub(v[j].w, v[i].w), Sub(v[k].w, v[i].w));
  const double length = Norm(n);
  if(length <= 0.0) {
    /* kept in the topology, but never the closest face or visible */
    f.n = {{0.0, 0.0, 0.0}};
    f.distance = HUGE_VAL;
    return false;
  }
  f.n = Scale(1.0 / length, n);
  f.distance = Dot(f.n, v[i].w);
  return true;
}

/*!
Expands the GJK simplex, which contains the origin, to a tetrahedron that
contains it (possibly on its boundary): a segment through the origin gains a
point off its line, a triangle a point off its plane. Returns false if the
Minkowski difference is flat at the origin, i.e. the geometries only touch.
*/
static bool MakeTetrahedron(shift::core::collision_geometry::ContactPair& p,
                            std::vector<EpaVertex>& v, double eps) {
  if(v.size() == 4) {
    const double volume = Dot(Sub(v[1].w, v[0].w),
                              Cross(Sub(v[2].w, v[0].w), Sub(v[3].w, v[0].w)));
    if(std::fabs(volume) > eps * eps * eps) {
      return true;
    }
    v.pop_back();
  }
  if(v.size() == 3) {
    const ksl_vec3_t n = Cross(Sub(v[1].w, v[0].w), Sub(v[2].w, v[0].w));
    if(Norm(n) <= eps * eps) {
      v.pop_back(); // collinear, continue with a segment
    }
  }
  if(v.size() == 2) {
    const ksl_vec3_t e = Sub(v[1].w, v[0].w);
    const double length = Norm(e);
    if(length <= eps) {
      v.pop_back();
    } else {
      /* search directions around the segment for a point off its line */
      const ksl_vec3_t u = Scale(1.0 / length, e);
      const int k = std::fabs(u.x) < std::fabs(u.y)
                      ? (std::fabs(u.x) < std::fabs(u.z) ? 0 : 2)
                      : (std::fabs(u.y) < std::fabs(u.z) ? 1 : 2);
      ksl_vec3_t axis = {{0.0, 0.0, 0.0}};
      axis.at[k] = 1.0;
      const ksl_vec3_t d1 = Cross(u, axis);
      const ksl_vec3_t d2 = Cross(u, d1);
      bool found = false;
      for(int i = 0; i < 6 && !found; i++) {
        const double angle = i * M_PI / 3.0;
        const ksl_vec3_t d =
          Add(Scale(std::cos(angle), d1), Scale(std::sin(angle), d2));
        const EpaVertex c = Support(p, d);
        if(Norm(Cross(u, Sub(c.w, v[0].w))) > eps) {
          v.push_back(c);
          found = true;
        }
      }
      if(!found) {
        return false;
      }
    }
  }
  if(v.size() < 3) {
    return false;
  }
  /* a point off the triangle's plane, on the side where the difference
  extends farther */
  const ksl_vec3_t n = Cross(Sub(v[1].w, v[0].w), Sub(v[2].w, v[0].w));
  const ksl_vec3_t u = Scale(1.0 / Norm(n), n);
  const EpaVertex above = Support(p, u);
  const EpaVertex below = Support(p, Scale(-1.0, u));
  const double h_above = Dot(u, Sub(above.w, v[0].w));
  const double h_below = -Dot(u, Sub(below.w, v[0].w));
  if(std::max(h_above, h_below) <= eps) {
    return false;
  }
  v.push_back(h_above >= h_below ? above : below);
  return true;
}

/*! the vertex coordinates of the face projection of the origin, clamped to
 * the face; returns the smallest coordinate before clamping, negative if the
 * projection lies outside the face */
static double Barycentric(const std::vector<EpaVertex>& v, const EpaFace& f,
                          double lambda[3]) {
  const ksl_vec3_t q = Scale(f.distance, f.n);
  const ksl_vec3_t e0 = Sub(v[f.v[1]].w, v[f.v[0]].w);
  const ksl_vec3_t e1 = Sub(v[f.v[2]].w, v[f.v[0]].w);
  const ksl_vec3_t r = Sub(q, v[f.v[0]].w);
  const double d00 = Dot(e0, e0);
  const double d01 = Dot(e0, e1);
  const double d11 = Dot(e1, e1);
  const double d20 = Dot(r, e0);
  const double d21 = Dot(r, e1);
  const double denominator = d00 * d11 - d01 * d01;
  double l1 = 0.0, l2 = 0.0;
  if(denominator > 0.0) {
    l1 = (d11 * d20 - d01 * d21) / denominator;
    l2 = (d00 * d21 - d01 * d20) / denominator;
  }
  const double smallest = std::min({1.0 - l1 - l2, l1, l2});
  lambda[0] = std::max(1.0 - l1 - l2, 0.0);
  lambda[1] = std::max(l1, 0.0);
  lambda[2] = std::max(l2, 0.0);
  const double sum = lambda[0] + lambda[1] + lambda[2];
  for(int i = 0; i < 3; i++) {
    lambda[i] = sum > 0.0 ? lambda[i] / sum : 1.0 / 3.0;
  }
  return smallest;
}

/*! add a horizon edge, or remove it if its reverse is already there (the
 * edge is shared by two removed faces) */
static void AddEdge(std::vector<std::pair<int, int>>& edges, int a, int b) {
  for(size_t i = 0; i < edges.size(); i++) {
    if(edges[i].first == b && edges[i].second == a) {
      edges[i] = edges.back();
      edges.pop_back();
      return;
    }
  }
  edges.emplace_back(a, b);
}

/*! contact data of geometries that only touch, or whose difference is flat:
 * zero depth, the normal along the line between the reference frames */
static void Touching(shift::core::collision_geometry::ContactPair& p,
                     const std::vector<EpaVertex>& v) {
  ksl_vec3_t a = {{0.0, 0.0, 0.0}};
  ksl_vec3_t b = {{0.0, 0.0, 0.0}};
  for(const auto& vertex : v) {
    a = Add(a, vertex.a);
    b = Add(b, vertex.b);
  }
  const double k = v.empty() ? 0.0 : 1.0 / v.size();
  p.point_a = Scale(k, a);
  p.point_b = Scale(k, b);
  ksl_vec3_t n = Sub(p.b->get_d_absolute().t, p.a->get_d_absolute().t);
  const double length = Norm(n);
  p.normal = length > 0.0 ? Scale(1.0 / length, n)
                          : ksl_vec3_t{{1.0, 0.0, 0.0}};
  p.depth = 0.0;
}

bool shift::core::collision_geometry::ComputePenetration(ContactPair& p) {
  const double scale =
    std::max(p.a->get_bounding_radius() + p.b->get_bounding_radius(), 1e-12);
  const double eps = 1e-9 * scale;
  const double tolerance = kRelativeTolerance * scale;

  /* the GJK simplex, from its points' local coordinates at the current
  poses */
  std::vector<EpaVertex> v(p.simplex_point_count);
  for(int i = 0; i < p.simplex_point_count; i++) {
    ksl_product_dv(&p.a->get_d_absolute(), &p.extreme_vertex_local_a[i],
                   &v[i].a);
    ksl_product_dv(&p.b->get_d_absolute(), &p.extreme_vertex_local_b[i],
                   &v[i].b);
    v[i].w = Sub(v[i].b, v[i].a);
  }

  p.depth_converged = true;
  if(!MakeTetrahedron(p, v, eps)) {
    Touching(p, v);
  } else {
    /* the four faces, oriented away from the centroid */
    ksl_vec3_t centroid = {{0.0, 0.0, 0.0}};
    for(int i = 0; i < 4; i++) {
      centroid = Add(centroid, Scale(0.25, v[i].w));
    }
    std::vector<EpaFace> faces;
    const int tetrahedron[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    for(const auto& t : tetrahedron) {
      EpaFace f;
      MakeFace(v, t[0], t[1], t[2], f);
      if(Dot(f.n, Sub(v[t[0]].w, centroid)) < 0.0) {
        MakeFace(v, t[0], t[2], t[1], f);
      }
      faces.push_back(f);
    }

    auto closest = [&]() {
      int best = -1;
      for(size_t i = 0; i < faces.size(); i++) {
        if(faces[i].valid &&
           (best < 0 || faces[i].distance < faces[best].distance)) {
          best = static_cast<int>(i);
        }
      }
      return best;
    };

    p.depth_converged = false;
    std::vector<std::pair<int, int>> edges;
    for(int iteration = 0; iteration < kMaxIterations; iteration++) {
      const int best = closest();
      const EpaFace f = faces[best];
      const EpaVertex s = Support(p, f.n);
      if(Dot(f.n, s.w) - f.distance <= tolerance) {
        p.depth_converged = true;
        break;
      }
      /* remove the faces that see the new point, and close the hole with
      faces from the new point to the horizon */
      const int index = static_cast<int>(v.size());
      v.push_back(s);
      edges.clear();
      for(auto& g : faces) {
        if(g.valid && g.distance < HUGE_VAL &&
           Dot(g.n, Sub(s.w, v[g.v[0]].w)) > eps) {
          g.valid = false;
          AddEdge(edges, g.v[0], g.v[1]);
          AddEdge(edges, g.v[1], g.v[2]);
          AddEdge(edges, g.v[2], g.v[0]);
        }
      }
      if(edges.empty()) {
        break; // numerically stuck, keep the best face
      }
      faces.erase(std::remove_if(faces.begin(), faces.end(),
                                 [](const EpaFace& g) { return !g.valid; }),
                  faces.end());
      for(const auto& [i, j] : edges) {
        EpaFace g;
        MakeFace(v, i, j, index, g);
        faces.push_back(g);
      }
    }

    /* a flat facet of the difference is split into coplanar triangles at the
    same distance, and only one of them contains the projection of the
    origin: among the faces coplanar with the closest one, take the one the
    projection lies deepest inside */
    const int best = closest();
    const EpaFace& nearest = faces[best];
    int chosen = best;
    double lambda[3];
    double inside = Barycentric(v, nearest, lambda);
    for(size_t i = 0; i < faces.size() && inside < 0.0; i++) {
      const EpaFace& g = faces[i];
      if(!g.valid || static_cast<int>(i) == best ||
         g.distance > nearest.distance + tolerance ||
         Dot(g.n, nearest.n) < 1.0 - 1e-9) {
        continue;
      }
      const double smallest = Barycentric(v, g, lambda);
      if(smallest > inside) {
        chosen = static_cast<int>(i);
        inside = smallest;
      }
    }
    const EpaFace& f = faces[chosen];
    Barycentric(v, f, lambda);
    p.point_a = {{0.0, 0.0, 0.0}};
    p.point_b = {{0.0, 0.0, 0.0}};
    for(int i = 0; i < 3; i++) {
      p.point_a = Add(p.point_a, Scale(lambda[i], v[f.v[i]].a));
      p.point_b = Add(p.point_b, Scale(lambda[i], v[f.v[i]].b));
    }
    /* the face normal points out of b - a; b separates by moving against
    it, so the normal from a to b is its opposite */
    p.depth = std::max(f.distance, 0.0);
    p.normal = Scale(-1.0, f.n);
  }
  p.point_local_a = ToLocal(*p.a, p.point_a);
  p.point_local_b = ToLocal(*p.b, p.point_b);
  return p.depth_converged;
}
