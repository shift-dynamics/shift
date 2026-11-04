#include "shift/core/collision_geometry/manifold.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "shift/core/collision_geometry/convex_hull.h"
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

void shift::core::collision_geometry::ComputeManifold(ContactPair& pair) {
  pair.manifold.clear();
  auto plane_a = dynamic_cast<const Plane*>(pair.a);
  auto plane_b = dynamic_cast<const Plane*>(pair.b);
  auto hull_a = dynamic_cast<const ConvexHull*>(pair.a);
  auto hull_b = dynamic_cast<const ConvexHull*>(pair.b);
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
