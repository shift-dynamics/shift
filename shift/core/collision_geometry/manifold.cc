#include "shift/core/collision_geometry/manifold.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/plane.h"

/* a point of a hull's local frame in the inertial frame, R v + t */
static ksl_vec3_t ToWorld(const ksl_SE3_t& d, const ksl_vec3_t& v) {
  ksl_vec3_t w;
  ksl_product_dv(&d, &v, &w);
  return w;
}

/* a direction of a hull's local frame in the inertial frame, R v */
static ksl_vec3_t Rotate(const ksl_SE3_t& d, const ksl_vec3_t& v) {
  ksl_vec3_t w = {{0.0, 0.0, 0.0}};
  for(int i = 0; i < 3; i++) {
    for(int k = 0; k < 3; k++) {
      w.at[k] += d.R.v[i].at[k] * v.at[i];
    }
  }
  return w;
}

/*
The vertices of a convex hull below a plane are connected by its edges, so
they are found by a search from the deepest vertex, the support vertex
against the normal, over the neighbors that are also below the plane.
*/
static void PlaneHullManifold(const shift::core::collision_geometry::Plane& plane,
                              const shift::core::collision_geometry::ConvexHull& hull,
                              bool plane_is_a, int& hint,
                              shift::core::collision_geometry::ContactPair& pair) {
  const ksl_vec3_t n = plane.get_normal();
  const ksl_vec3_t& q = plane.get_point();
  const double level = ksl_dot_vv(&n, &q);
  const auto& data = hull.get_hull();
  const ksl_SE3_t& pose = hull.get_d_absolute();
  auto add = [&](const ksl_vec3_t& w, double depth) {
    /* the vertex, and its projection onto the plane */
    shift::core::collision_geometry::ContactPoint point;
    point.depth = depth;
    const ksl_vec3_t on_plane = {{w.x + depth * n.x, w.y + depth * n.y,
                                  w.z + depth * n.z}};
    point.point_a = plane_is_a ? on_plane : w;
    point.point_b = plane_is_a ? w : on_plane;
    pair.manifold.push_back(point);
  };
  if(data.vertices.empty()) {
    return;
  }
  if(data.neighbors.size() != data.vertices.size()) {
    for(const auto& v : data.vertices) { // a degenerate hull, no adjacency
      const ksl_vec3_t w = ToWorld(pose, v);
      const double depth = level - ksl_dot_vv(&n, &w);
      if(depth > 0.0) {
        add(w, depth);
      }
    }
    return;
  }
  ksl_vec3_t local, global;
  const ksl_vec3_t down = {{-n.x, -n.y, -n.z}};
  hull.Support(down, local, global, hint);
  std::vector<unsigned int> stack = {static_cast<unsigned int>(hint)};
  std::vector<unsigned int> visited = stack;
  while(!stack.empty()) {
    const unsigned int i = stack.back();
    stack.pop_back();
    const ksl_vec3_t w = ToWorld(pose, data.vertices[i]);
    const double depth = level - ksl_dot_vv(&n, &w);
    if(depth <= 0.0) {
      continue;
    }
    add(w, depth);
    for(unsigned int j : data.neighbors[i]) {
      if(std::find(visited.begin(), visited.end(), j) == visited.end()) {
        visited.push_back(j);
        stack.push_back(j);
      }
    }
  }
}

/* the outward normal of polygon f of a hull, inertial frame */
static ksl_vec3_t FaceNormal(const shift::core::collision_geometry::ConvexHull& hull,
                             int f) {
  return Rotate(hull.get_d_absolute(), hull.get_hull().polygon_normals[f]);
}

/* the polygons of a hull that may be nearly the most aligned with direction
d (inertial frame): those around its support vertex of d, which include the
most aligned, and those around their vertices; all of them for a hull
without adjacency */
static void CandidateFaces(const shift::core::collision_geometry::ConvexHull& hull,
                           const ksl_vec3_t& d, int& hint,
                           std::vector<int>& faces) {
  const auto& data = hull.get_hull();
  faces.clear();
  ksl_vec3_t local, global;
  hull.Support(d, local, global, hint);
  if(hint < 0 || static_cast<size_t>(hint) >= data.vertex_polygons.size() ||
     data.vertex_polygons[hint].empty()) {
    for(size_t f = 0; f < data.polygons.size(); f++) {
      faces.push_back(static_cast<int>(f));
    }
    return;
  }
  auto add = [&](int f) {
    if(std::find(faces.begin(), faces.end(), f) == faces.end()) {
      faces.push_back(f);
    }
  };
  for(auto f : data.vertex_polygons[hint]) {
    add(static_cast<int>(f));
  }
  const size_t ring = faces.size();
  for(size_t r = 0; r < ring; r++) {
    for(auto v : data.polygons[faces[r]]) {
      for(auto f : data.vertex_polygons[v]) {
        add(static_cast<int>(f));
      }
    }
  }
}

/* the faces of a pair are blended by exp((alignment - best) / kFaceBlend),
the alignment the cosine of the angle between a face's outward normal and the
direction it should face; faces more than kFaceCut kFaceBlend below the best
weigh less than exp(-kFaceCut) and are left out. The blend only bridges
nearly exact ties: the depths below a reference face tilted from the
contact are too deep, and a wide blend (1e-4 and more) gives spurious
stiff forces and torques, 1e-4 tripling the steps of 50 bunnies and 2e-3
stalling them */
static constexpr double kFaceBlend = 1e-6;
static constexpr double kFaceCut = 20.0;

/* the faces of a are favored as reference faces by this much alignment, more
than kFaceCut kFaceBlend, so a face of b takes over only when clearly better
aligned, and two parallel faces in contact are not blended with the same
manifold seen from b */
static constexpr double kReferenceBias = 0.05;

struct WeightedFace {
  bool of_a;    // a face of a, else of b
  int face;
  double weight;
};

/* blend weights, summing to 1, of faces from their alignments */
static void BlendWeights(std::vector<WeightedFace>& faces,
                         const std::vector<double>& alignment) {
  double best = -HUGE_VAL;
  for(double c : alignment) {
    best = std::max(best, c);
  }
  double sum = 0.0;
  std::vector<WeightedFace> kept;
  for(size_t i = 0; i < faces.size(); i++) {
    const double x = (alignment[i] - best) / kFaceBlend;
    if(x > -kFaceCut) {
      kept.push_back(faces[i]);
      kept.back().weight = std::exp(x);
      sum += kept.back().weight;
    }
  }
  for(auto& f : kept) {
    f.weight /= sum;
  }
  faces.swap(kept);
}

/* the inertial vertices, outward normal and offset of polygon f of a hull */
static void Face(const shift::core::collision_geometry::ConvexHull& hull,
                 int f, std::vector<ksl_vec3_t>& polygon, ksl_vec3_t& normal,
                 double& offset) {
  const auto& data = hull.get_hull();
  const ksl_SE3_t& pose = hull.get_d_absolute();
  polygon.clear();
  for(auto i : data.polygons[f]) {
    polygon.push_back(ToWorld(pose, data.vertices[i]));
  }
  normal = FaceNormal(hull, f);
  offset = ksl_dot_vv(&normal, &polygon[0]);
}

/* clip a polygon by the half space s . x <= c (Sutherland-Hodgman) */
static void Clip(const std::vector<ksl_vec3_t>& in, const ksl_vec3_t& s,
                 double c, std::vector<ksl_vec3_t>& out) {
  out.clear();
  const size_t count = in.size();
  for(size_t i = 0; i < count; i++) {
    const ksl_vec3_t& p = in[i];
    const ksl_vec3_t& q = in[(i + 1) % count];
    const double dp = ksl_dot_vv(&s, &p) - c;
    const double dq = ksl_dot_vv(&s, &q) - c;
    if(dp <= 0.0) {
      out.push_back(p);
    }
    if((dp < 0.0 && dq > 0.0) || (dp > 0.0 && dq < 0.0)) {
      const double t = dp / (dp - dq);
      out.push_back({{p.x + t * (q.x - p.x), p.y + t * (q.y - p.y),
                      p.z + t * (q.z - p.z)}});
    }
  }
}

/*
The weights of the points of a clipped polygon: 2 theta / pi of its exterior
angle theta at each point, which sum to 4 (the exterior angles of a convex
polygon sum to 2 pi), so the corners of a rectangle carry 1 each. When a
vertex of the incident face crosses a side plane of the reference face, the
clipping replaces it by two points that start where it was, and their
exterior angles sum to its own, so the weights, and the contact force,
change continuously. Coincident points are merged first; a segment's two
ends carry 2 each, a single point 4.
*/
static void PolygonWeights(std::vector<ksl_vec3_t>& polygon,
                           std::vector<double>& weights) {
  double size = 0.0;
  for(const auto& x : polygon) {
    size = std::max({size, std::fabs(x.x), std::fabs(x.y), std::fabs(x.z)});
  }
  const double tolerance = 1e-12 * std::max(size, 1e-300);
  auto distinct = [&](const ksl_vec3_t& a, const ksl_vec3_t& b) {
    return std::fabs(a.x - b.x) > tolerance ||
           std::fabs(a.y - b.y) > tolerance || std::fabs(a.z - b.z) > tolerance;
  };
  std::vector<ksl_vec3_t> merged;
  for(const auto& x : polygon) {
    if(merged.empty() || distinct(merged.back(), x)) {
      merged.push_back(x);
    }
  }
  while(merged.size() > 1 && !distinct(merged.back(), merged.front())) {
    merged.pop_back();
  }
  polygon.swap(merged);
  const size_t count = polygon.size();
  weights.assign(count, count == 1 ? 4.0 : 2.0);
  if(count < 3) {
    return;
  }
  for(size_t i = 0; i < count; i++) {
    const ksl_vec3_t& prev = polygon[(i + count - 1) % count];
    const ksl_vec3_t& x = polygon[i];
    const ksl_vec3_t& next = polygon[(i + 1) % count];
    const ksl_vec3_t in = {{x.x - prev.x, x.y - prev.y, x.z - prev.z}};
    const ksl_vec3_t out = {{next.x - x.x, next.y - x.y, next.z - x.z}};
    ksl_vec3_t cross;
    ksl_cross_vv(&in, &out, &cross);
    const double angle =
      std::atan2(ksl_vec3_l2norm(&cross), ksl_dot_vv(&in, &out));
    weights[i] = 2.0 * angle / M_PI;
  }
}

/* the manifold points of one reference face and one incident face, their
weights scaled by blend */
static void ClippedManifold(const shift::core::collision_geometry::ConvexHull& reference_hull,
                            int reference_face,
                            const shift::core::collision_geometry::ConvexHull& incident_hull,
                            int incident_face, bool reference_is_a,
                            double blend,
                            shift::core::collision_geometry::ContactPair& pair) {
  std::vector<ksl_vec3_t> reference, incident;
  ksl_vec3_t nr, ni;
  double cr, ci;
  Face(reference_hull, reference_face, reference, nr, cr);
  Face(incident_hull, incident_face, incident, ni, ci);

  /* clip by the side planes of the reference face: for its edges e
  counterclockwise about nr, the outward side normal is e x nr */
  std::vector<ksl_vec3_t> clipped;
  const size_t count = reference.size();
  for(size_t i = 0; i < count && !incident.empty(); i++) {
    const ksl_vec3_t& p = reference[i];
    const ksl_vec3_t& q = reference[(i + 1) % count];
    const ksl_vec3_t e = {{q.x - p.x, q.y - p.y, q.z - p.z}};
    ksl_vec3_t s;
    ksl_cross_vv(&e, &nr, &s);
    const double length = ksl_vec3_l2norm(&s);
    if(length <= 0.0) {
      continue;
    }
    for(int k = 0; k < 3; k++) {
      s.at[k] /= length;
    }
    Clip(incident, s, ksl_dot_vv(&s, &p), clipped);
    incident.swap(clipped);
  }
  if(incident.empty()) {
    return;
  }

  /* the points behind the reference face, and their projections onto it;
  the weights are those of the whole clipped polygon, so a point that comes
  out from behind the face does not change those of the others */
  std::vector<double> weights;
  PolygonWeights(incident, weights);
  for(size_t i = 0; i < incident.size(); i++) {
    const ksl_vec3_t& x = incident[i];
    const double depth = cr - ksl_dot_vv(&nr, &x);
    if(depth <= 0.0) {
      continue;
    }
    const ksl_vec3_t on_reference = {{x.x + depth * nr.x, x.y + depth * nr.y,
                                      x.z + depth * nr.z}};
    shift::core::collision_geometry::ContactPoint point;
    point.depth = depth;
    point.weight = blend * weights[i];
    point.point_a = reference_is_a ? on_reference : x;
    point.point_b = reference_is_a ? x : on_reference;
    pair.manifold.push_back(point);
  }
}

static void HullHullManifold(const shift::core::collision_geometry::ConvexHull& a,
                             const shift::core::collision_geometry::ConvexHull& b,
                             shift::core::collision_geometry::ContactPair& pair) {
  const ksl_vec3_t& n = pair.normal;
  const ksl_vec3_t minus = {{-n.x, -n.y, -n.z}};
  int hint_a = pair.hint_a, hint_b = pair.hint_b;

  /* reference faces: those of a facing along n and those of b facing
  against it */
  std::vector<int> candidates;
  std::vector<WeightedFace> references;
  std::vector<double> alignment;
  CandidateFaces(a, n, hint_a, candidates);
  for(int f : candidates) {
    const ksl_vec3_t normal = FaceNormal(a, f);
    references.push_back({true, f, 0.0});
    alignment.push_back(ksl_dot_vv(&normal, &n) + kReferenceBias);
  }
  CandidateFaces(b, minus, hint_b, candidates);
  for(int f : candidates) {
    const ksl_vec3_t normal = FaceNormal(b, f);
    references.push_back({false, f, 0.0});
    alignment.push_back(ksl_dot_vv(&normal, &minus));
  }
  if(references.empty()) {
    return;
  }
  BlendWeights(references, alignment);

  /* for each, the incident faces of the other hull, facing against it */
  std::vector<WeightedFace> incidents;
  for(const auto& r : references) {
    const auto& reference_hull = r.of_a ? a : b;
    const auto& incident_hull = r.of_a ? b : a;
    const ksl_vec3_t nr = FaceNormal(reference_hull, r.face);
    const ksl_vec3_t against = {{-nr.x, -nr.y, -nr.z}};
    CandidateFaces(incident_hull, against, r.of_a ? hint_b : hint_a,
                   candidates);
    incidents.clear();
    alignment.clear();
    for(int f : candidates) {
      const ksl_vec3_t normal = FaceNormal(incident_hull, f);
      incidents.push_back({!r.of_a, f, 0.0});
      alignment.push_back(ksl_dot_vv(&normal, &against));
    }
    if(incidents.empty()) {
      continue;
    }
    BlendWeights(incidents, alignment);
    for(const auto& i : incidents) {
      ClippedManifold(reference_hull, r.face, incident_hull, i.face, r.of_a,
                      r.weight * i.weight, pair);
    }
  }

  /* blended faces sharing an edge give its points twice: merge coincident
  points, adding their weights, which leaves the force unchanged */
  auto& manifold = pair.manifold;
  auto close = [](const ksl_vec3_t& x, const ksl_vec3_t& y) {
    for(int k = 0; k < 3; k++) {
      const double scale = std::max({1.0, std::fabs(x.at[k]), std::fabs(y.at[k])});
      if(std::fabs(x.at[k] - y.at[k]) > 1e-9 * scale) {
        return false;
      }
    }
    return true;
  };
  for(size_t i = 0; i < manifold.size(); i++) {
    for(size_t j = manifold.size() - 1; j > i; j--) {
      if(close(manifold[i].point_a, manifold[j].point_a) &&
         close(manifold[i].point_b, manifold[j].point_b)) {
        manifold[i].weight += manifold[j].weight;
        manifold.erase(manifold.begin() + static_cast<long>(j));
      }
    }
  }
}


/*
A cylinder or a capsule on a plane touches along a line, the lowest generator
of the cylinder or the segment of the capsule, so a single deepest point
cannot carry its load as it tilts or rolls: the point would jump to an end.
The surface point at station s of the axis, c + s a, with the unit plane
normal n (out of the plane), offset along it o and g = n . a, is deepest at
  depth(s) = o - n . c - s g + r_eff
with r_eff = r for a capsule (the lowest point of the sphere swept along the
segment) and r_eff = r sqrt(1 - g^2) for a cylinder (the lowest point of its
circle), linear in s. Hertz line contact, F = pi/4 E* d per unit of length
(see force_module::Contact), integrates in closed form over the stations in
|s| <= h where the depth is positive: a trapezoid of length L between the
depths d_1 and d_2 at its ends, equal to a point of length L with the mean
depth (d_1 + d_2) / 2 at the centroid of the trapezoid, which moves along the
line as the shape tilts, so the contact force is continuous in the state and
gives the restoring moment.
Station and depth: a point of the line carries a length, as the points of
CAD geometry do (ContactPoint::length).
*/
static void PlaneLineManifold(const shift::core::collision_geometry::Plane& plane,
                              const ksl_vec3_t& center, const ksl_vec3_t& axis,
                              double half_length, double radius, bool capsule,
                              bool plane_is_a,
                              shift::core::collision_geometry::ContactPair& pair) {
  const ksl_vec3_t n = plane.get_normal();
  const double level = ksl_dot_vv(&n, &plane.get_point());
  const double g = ksl_dot_vv(&n, &axis);
  const double across = std::sqrt(std::max(1.0 - g * g, 0.0));
  const double r_eff = capsule ? radius : radius * across;
  const double d0 = level - ksl_dot_vv(&n, &center) + r_eff; // depth at s = 0
  /* the stations s1 to s2 with positive depth d0 - g s */
  double s1 = -half_length, s2 = half_length;
  if(g > 0.0) {
    s2 = std::min(s2, d0 / g);
  } else if(g < 0.0) {
    s1 = std::max(s1, d0 / g);
  } else if(!(d0 > 0.0)) {
    return;
  }
  const double length = s2 - s1;
  if(!(length > 0.0)) {
    return;
  }
  const double d1 = d0 - g * s1;
  const double d2 = d0 - g * s2;
  const double sum = d1 + d2;
  /* the centroid of the trapezoid, from s1 */
  const double s_c = s1 + (sum > 0.0 ? length * (d1 + 2.0 * d2) / (3.0 * sum)
                                      : 0.5 * length);
  /* the lowest point of the surface at the station: below the axis point
  along -n for a capsule, along the part of -n across the axis for a
  cylinder (any direction across the axis where n is along it) */
  ksl_vec3_t down = {{-n.x, -n.y, -n.z}};
  if(!capsule) {
    for(int k = 0; k < 3; k++) {
      down.at[k] = -(n.at[k] - g * axis.at[k]);
    }
    if(across > 1e-12) {
      for(int k = 0; k < 3; k++) {
        down.at[k] /= across;
      }
    } else {
      const ksl_vec3_t e = std::fabs(axis.x) < 0.9
                             ? ksl_vec3_t{{1.0, 0.0, 0.0}}
                             : ksl_vec3_t{{0.0, 1.0, 0.0}};
      ksl_vec3_t perpendicular;
      ksl_cross_vv(&axis, &e, &perpendicular);
      const double norm = ksl_vec3_l2norm(&perpendicular);
      for(int k = 0; k < 3; k++) {
        down.at[k] = perpendicular.at[k] / norm;
      }
    }
  }
  shift::core::collision_geometry::ContactPoint point;
  point.depth = 0.5 * sum;
  point.length = length;
  ksl_vec3_t on_shape, on_plane;
  for(int k = 0; k < 3; k++) {
    on_shape.at[k] = center.at[k] + s_c * axis.at[k] + radius * down.at[k];
    on_plane.at[k] = on_shape.at[k] + point.depth * n.at[k];
  }
  point.point_a = plane_is_a ? on_plane : on_shape;
  point.point_b = plane_is_a ? on_shape : on_plane;
  point.normal = plane_is_a ? n : ksl_vec3_t{{-n.x, -n.y, -n.z}};
  pair.manifold.push_back(point);
  pair.point_laws = true;
}

/* the closest points of the segments p0 p1 and q0 q1 (Ericson, Real-Time
Collision Detection, 5.1.9) */
static void ClosestOfSegments(const ksl_vec3_t& p0, const ksl_vec3_t& p1,
                              const ksl_vec3_t& q0, const ksl_vec3_t& q1,
                              ksl_vec3_t& on_p, ksl_vec3_t& on_q) {
  const ksl_vec3_t d1 = {{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z}};
  const ksl_vec3_t d2 = {{q1.x - q0.x, q1.y - q0.y, q1.z - q0.z}};
  const ksl_vec3_t r = {{p0.x - q0.x, p0.y - q0.y, p0.z - q0.z}};
  const double a = ksl_dot_vv(&d1, &d1), e = ksl_dot_vv(&d2, &d2);
  const double f = ksl_dot_vv(&d2, &r);
  double s = 0.0, t = 0.0;
  if(a <= 0.0 && e <= 0.0) {
    s = t = 0.0;
  } else if(a <= 0.0) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = ksl_dot_vv(&d1, &r);
    if(e <= 0.0) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = ksl_dot_vv(&d1, &d2);
      const double denominator = a * e - b * b;
      s = denominator > 0.0 ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0)
                            : 0.0;
      t = (b * s + f) / e;
      if(t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if(t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  for(int k = 0; k < 3; k++) {
    on_p.at[k] = p0.at[k] + s * d1.at[k];
    on_q.at[k] = q0.at[k] + t * d2.at[k];
  }
}

/*
The unit direction out of a convex hull toward a segment outside it (the
axis of a cylinder or a capsule), from the closest points of the two: the
closest point of the hull is on a face (a segment's end over the face's
inside) or on an edge (the closest points of the segment and the edge). The
side of a capsule, and of a cylinder, is its axis grown by the radius, so
this is the exact normal of their contact on the side, where that of EPA
is only as good as the square root of its tolerance; it is unique and
continuous in the poses. False if the segment touches or enters the hull.
*/
static bool SegmentNormal(const shift::core::collision_geometry::ConvexHull& hull,
                          const ksl_vec3_t& p0, const ksl_vec3_t& p1,
                          ksl_vec3_t& n) {
  const auto& data = hull.get_hull();
  const ksl_SE3_t& pose = hull.get_d_absolute();
  const ksl_vec3_t d = {{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z}};
  double s1 = 0.0, s2 = 1.0; // the part of the segment inside the hull
  double best = HUGE_VAL;
  ksl_vec3_t on_segment = p0, on_hull = p0;
  auto keep = [&](const ksl_vec3_t& x, const ksl_vec3_t& y) {
    const ksl_vec3_t gap = {{x.x - y.x, x.y - y.y, x.z - y.z}};
    const double squared = ksl_dot_vv(&gap, &gap);
    if(squared < best) {
      best = squared;
      on_segment = x;
      on_hull = y;
    }
  };
  std::vector<ksl_vec3_t> polygon;
  for(size_t f = 0; f < data.polygons.size(); f++) {
    const ksl_vec3_t nf = Rotate(pose, data.polygon_normals[f]);
    polygon.clear();
    for(auto i : data.polygons[f]) {
      polygon.push_back(ToWorld(pose, data.vertices[i]));
    }
    const double of = ksl_dot_vv(&nf, &polygon[0]);
    const double h0 = ksl_dot_vv(&nf, &p0) - of;
    const double slope = ksl_dot_vv(&nf, &d);
    if(slope > 0.0) {
      s2 = std::min(s2, -h0 / slope);
    } else if(slope < 0.0) {
      s1 = std::max(s1, -h0 / slope);
    } else if(h0 > 0.0) {
      s2 = -1.0;
    }
    for(const ksl_vec3_t* end : {&p0, &p1}) {
      const double h = ksl_dot_vv(&nf, end) - of;
      if(h <= 0.0) {
        continue;
      }
      const ksl_vec3_t foot = {{end->x - h * nf.x, end->y - h * nf.y,
                                end->z - h * nf.z}};
      bool inside = true;
      for(size_t i = 0; i < polygon.size() && inside; i++) {
        const ksl_vec3_t& a = polygon[i];
        const ksl_vec3_t& b = polygon[(i + 1) % polygon.size()];
        const ksl_vec3_t edge = {{b.x - a.x, b.y - a.y, b.z - a.z}};
        const ksl_vec3_t to = {{foot.x - a.x, foot.y - a.y, foot.z - a.z}};
        ksl_vec3_t cross;
        ksl_cross_vv(&edge, &to, &cross);
        inside = ksl_dot_vv(&cross, &nf) >= 0.0;
      }
      if(inside) {
        keep(*end, foot);
      }
    }
    for(size_t i = 0; i < polygon.size(); i++) {
      ksl_vec3_t x, y;
      ClosestOfSegments(p0, p1, polygon[i], polygon[(i + 1) % polygon.size()],
                        x, y);
      keep(x, y);
    }
  }
  if(s2 >= s1 || !(best > 0.0) || best == HUGE_VAL) {
    return false;
  }
  const double distance = std::sqrt(best);
  for(int k = 0; k < 3; k++) {
    n.at[k] = (on_segment.at[k] - on_hull.at[k]) / distance;
  }
  return true;
}

/*
A cylinder or a capsule on a face of a convex hull touches along a line too,
and the hull's faces bound it. With the unit normal n out of the hull
(toward the shape), the lowest point of the shape at station s of its axis,
q(s) = c + s a + r_eff w (w the unit direction of PlaneLineManifold's
lowest point), is inside the hull where n_f . q(s) <= o_f for every face f
(outward normal n_f, offset o_f): an interval of s, clipped to |s| <= h. Its
depth there is the distance along n to where the line q(s) + t n leaves the
hull,

  depth(s) = min over the faces with n_f . n > 0 of (o_f - n_f . q(s)) / (n_f . n),

the lower envelope of lines in s, so piecewise linear and concave. Each
piece, a trapezoid, is a point of the manifold as for a plane: its length,
its mean depth, at its centroid. On a face (n its normal) there is one piece,
the line clipped by the face's edges; as the shape rolls over an edge, n
turns from one face toward the other and the edge binds instead, without a
jump in the force. On a plane (a hull of one face) this is PlaneLineManifold.
n is SegmentNormal's while the axis is outside the hull, else the pair's.
*/
static void HullLineManifold(const shift::core::collision_geometry::ConvexHull& hull,
                             const ksl_vec3_t& center, const ksl_vec3_t& axis,
                             double half_length, double radius, bool capsule,
                             bool hull_is_a,
                             shift::core::collision_geometry::ContactPair& pair) {
  const auto& data = hull.get_hull();
  if(data.polygons.empty() ||
     data.polygon_normals.size() != data.polygons.size() ||
     data.polygon_offsets.size() != data.polygons.size()) {
    return;
  }
  const ksl_SE3_t& pose = hull.get_d_absolute();
  /* the normal out of the hull: exact from the axis while it is outside the
  hull, else EPA's */
  ksl_vec3_t n = hull_is_a ? pair.normal
                           : ksl_vec3_t{{-pair.normal.x, -pair.normal.y,
                                         -pair.normal.z}};
  {
    ksl_vec3_t p0, p1, exact;
    for(int k = 0; k < 3; k++) {
      p0.at[k] = center.at[k] - half_length * axis.at[k];
      p1.at[k] = center.at[k] + half_length * axis.at[k];
    }
    if(SegmentNormal(hull, p0, p1, exact)) {
      n = exact;
    }
  }
  const ksl_vec3_t normal_ab =
    hull_is_a ? n : ksl_vec3_t{{-n.x, -n.y, -n.z}};
  /* the lowest point of the shape's section at a station, from its axis
  point: r w, w along -n for a capsule, along the part of -n across the
  axis for a cylinder */
  const double g = ksl_dot_vv(&n, &axis);
  const double across = std::sqrt(std::max(1.0 - g * g, 0.0));
  ksl_vec3_t w = {{-n.x, -n.y, -n.z}};
  if(!capsule) {
    if(across > 1e-12) {
      for(int k = 0; k < 3; k++) {
        w.at[k] = -(n.at[k] - g * axis.at[k]) / across;
      }
    } else {
      w = {{0.0, 0.0, 0.0}}; // on its end: the axis point
    }
  }
  const ksl_vec3_t q0 = {{center.x + radius * w.x, center.y + radius * w.y,
                          center.z + radius * w.z}};

  /* the stations inside the hull, and the exit lines t = alpha + beta s */
  double s1 = -half_length, s2 = half_length;
  std::vector<double> alpha, beta;
  for(size_t f = 0; f < data.polygons.size(); f++) {
    const ksl_vec3_t nf = Rotate(pose, data.polygon_normals[f]);
    const double of = data.polygon_offsets[f] + ksl_dot_vv(&nf, &pose.t);
    /* <= 0 inside; a point within rounding of the face's plane is inside,
    so a line in the plane is not cut where the rounding of its offset and
    slope cross */
    const double at_0 = ksl_dot_vv(&nf, &q0) - of -
                        1e-12 * (std::fabs(of) + half_length + radius);
    const double slope = ksl_dot_vv(&nf, &axis);
    if(slope > 0.0) {
      s2 = std::min(s2, -at_0 / slope);
    } else if(slope < 0.0) {
      s1 = std::max(s1, -at_0 / slope);
    } else if(at_0 > 0.0) {
      return;
    }
    const double along = ksl_dot_vv(&nf, &n);
    if(along > 1e-12) {
      alpha.push_back(-at_0 / along);
      beta.push_back(-slope / along);
    }
  }
  if(!(s2 > s1) || alpha.empty()) {
    return;
  }

  /* walk the lower envelope from s1 to s2 */
  auto exit_line = [&](double s, double& depth) {
    size_t best = 0;
    depth = HUGE_VAL;
    for(size_t f = 0; f < alpha.size(); f++) {
      const double t = alpha[f] + beta[f] * s;
      if(t < depth || (t == depth && beta[f] < beta[best])) {
        depth = t;
        best = f;
      }
    }
    return best;
  };
  double s = s1;
  for(size_t piece = 0; piece <= alpha.size() && s < s2; piece++) {
    double d_start;
    const size_t f = exit_line(s, d_start);
    /* where a line falling faster crosses it */
    double s_end = s2;
    for(size_t k = 0; k < alpha.size(); k++) {
      if(beta[k] < beta[f]) {
        const double cross = (alpha[k] - alpha[f]) / (beta[f] - beta[k]);
        if(cross > s && cross < s_end) {
          s_end = cross;
        }
      }
    }
    const double d1 = std::max(d_start, 0.0);
    const double d2 = std::max(alpha[f] + beta[f] * s_end, 0.0);
    const double length = s_end - s;
    const double sum = d1 + d2;
    if(length > 0.0 && sum > 0.0) {
      const double s_c = s + length * (d1 + 2.0 * d2) / (3.0 * sum);
      shift::core::collision_geometry::ContactPoint point;
      point.depth = 0.5 * sum;
      point.length = length;
      ksl_vec3_t on_shape, on_hull;
      for(int k = 0; k < 3; k++) {
        on_shape.at[k] = q0.at[k] + s_c * axis.at[k];
        on_hull.at[k] = on_shape.at[k] + point.depth * n.at[k];
      }
      point.point_a = hull_is_a ? on_hull : on_shape;
      point.point_b = hull_is_a ? on_shape : on_hull;
      point.normal = normal_ab;
      pair.manifold.push_back(point);
    }
    s = s_end;
  }
}

void shift::core::collision_geometry::ComputeManifold(ContactPair& pair) {
  pair.manifold.clear();
  auto plane_a = dynamic_cast<const Plane*>(pair.a);
  auto plane_b = dynamic_cast<const Plane*>(pair.b);
  auto hull_a = dynamic_cast<const ConvexHull*>(pair.a);
  auto hull_b = dynamic_cast<const ConvexHull*>(pair.b);
  /* a cylinder or a capsule on a plane: a line, which has no points where
  the depth is zero to rounding */
  bool line = false;
  for(auto [plane, other, plane_is_a] :
      {std::tuple{plane_a, pair.b, true}, std::tuple{plane_b, pair.a, false}}) {
    if(!plane) {
      continue;
    }
    line = line || dynamic_cast<const Cylinder*>(other) ||
           dynamic_cast<const Capsule*>(other);
    if(auto cylinder = dynamic_cast<const Cylinder*>(other)) {
      PlaneLineManifold(*plane, cylinder->get_d_absolute().t,
                        cylinder->get_axis(), cylinder->get_half_length(),
                        cylinder->get_radius(), false, plane_is_a, pair);
    } else if(auto capsule = dynamic_cast<const Capsule*>(other)) {
      PlaneLineManifold(*plane, capsule->get_d_absolute().t,
                        capsule->get_axis(), capsule->get_half_length(),
                        capsule->get_radius(), true, plane_is_a, pair);
    }
  }
  if(line) {
    pair.point_laws = true;
    return;
  }
  /* a cylinder or a capsule on a convex hull: the line of its faces, or the
  single deepest point (and its law) where the line has no length */
  for(auto [hull, other, hull_is_a] :
      {std::tuple{hull_a, pair.b, true}, std::tuple{hull_b, pair.a, false}}) {
    if(!hull) {
      continue;
    }
    if(auto cylinder = dynamic_cast<const Cylinder*>(other)) {
      HullLineManifold(*hull, cylinder->get_d_absolute().t,
                       cylinder->get_axis(), cylinder->get_half_length(),
                       cylinder->get_radius(), false, hull_is_a, pair);
    } else if(auto capsule = dynamic_cast<const Capsule*>(other)) {
      HullLineManifold(*hull, capsule->get_d_absolute().t,
                       capsule->get_axis(), capsule->get_half_length(),
                       capsule->get_radius(), true, hull_is_a, pair);
    }
    if(!pair.manifold.empty()) {
      pair.point_laws = true;
      return;
    }
  }
  if(plane_a && hull_b) {
    PlaneHullManifold(*plane_a, *hull_b, true, pair.hint_b, pair);
  } else if(plane_b && hull_a) {
    PlaneHullManifold(*plane_b, *hull_a, false, pair.hint_a, pair);
  } else if(hull_a && hull_b) {
    HullHullManifold(*hull_a, *hull_b, pair);
  }
  if(pair.manifold.empty()) {
    ContactPoint point;
    point.depth = pair.depth;
    point.point_a = pair.point_a;
    point.point_b = pair.point_b;
    pair.manifold.push_back(point);
  }
}
