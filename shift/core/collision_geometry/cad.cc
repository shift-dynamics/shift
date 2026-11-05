#include "shift/core/collision_geometry/cad.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

#include "ksl/matrix.h"
#include "shift/core/body/base.h"
#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/geometry/cad.h"
#include "shift/core/log.h"

using shift::core::cad::Vec2;
using shift::core::collision_geometry::CadContactFace;
using shift::core::collision_geometry::CadContactShape;

bool shift::core::collision_geometry::Cad::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("cad", shift::core::collision_geometry::Cad::Create);

std::unique_ptr<shift::core::collision_geometry::Cad>
shift::core::collision_geometry::Cad::Create() {
  return std::make_unique<shift::core::collision_geometry::Cad>();
}

void shift::core::collision_geometry::Cad::DeclareParameters() {
  DeclareParameter(
    "edge_radius", &edge_radius_,
    "radius to which the edges and vertices of the CAD geometry are taken to "
    "be rounded, for the Hertz contact of an edge or vertex with a face; "
    "defaults to 0.2% of the size of the geometry (the diagonal of its "
    "bounds)",
    false);
  DeclareParameter(
    "max_depth", &max_depth_,
    "deepest penetration taken for contact: a deeper point of a face, edge "
    "or vertex inside another body is taken to be on its far side; defaults "
    "to 2% of the size of the geometry",
    false);
}

/* ------------------------------------------------------------------------ */
/* placement and boxes */

ksl_vec3_t shift::core::collision_geometry::CadPlacement::Apply(
  const cad::Vec3& p) const {
  const ksl_vec3_t scaled = {{scale * p[0], scale * p[1], scale * p[2]}};
  ksl_vec3_t out;
  ksl_product_dv(&d, &scaled, &out);
  return out;
}

ksl_vec3_t shift::core::collision_geometry::CadPlacement::ApplyVector(
  const cad::Vec3& v) const {
  const ksl_vec3_t scaled = {{scale * v[0], scale * v[1], scale * v[2]}};
  ksl_vec3_t out;
  ksl_product_drv(&d.R, &scaled, &out);
  return out;
}

shift::core::cad::Vec3 shift::core::collision_geometry::CadPlacement::Invert(
  const ksl_vec3_t& p) const {
  ksl_vec3_t q;
  ksl_product_dinvv(&d, &p, &q);
  return {q.x / scale, q.y / scale, q.z / scale};
}

void shift::core::collision_geometry::CadBox::Add(const ksl_vec3_t& p) {
  for(int k = 0; k < 3; k++) {
    lo.at[k] = std::min(lo.at[k], p.at[k]);
    hi.at[k] = std::max(hi.at[k], p.at[k]);
  }
}

void shift::core::collision_geometry::CadBox::Add(const CadBox& b) {
  Add(b.lo);
  Add(b.hi);
}

void shift::core::collision_geometry::CadBox::Pad(double margin) {
  for(int k = 0; k < 3; k++) {
    lo.at[k] -= margin;
    hi.at[k] += margin;
  }
}

bool shift::core::collision_geometry::CadBox::Overlaps(
  const CadBox& b) const {
  for(int k = 0; k < 3; k++) {
    if(lo.at[k] > b.hi.at[k] || b.lo.at[k] > hi.at[k]) {
      return false;
    }
  }
  return true;
}

shift::core::collision_geometry::CadBox
shift::core::collision_geometry::CadBox::Transformed(
  const ksl_SE3_t& d) const {
  CadBox out;
  for(int corner = 0; corner < 8; corner++) {
    const ksl_vec3_t p = {{(corner & 1) ? hi.x : lo.x,
                           (corner & 2) ? hi.y : lo.y,
                           (corner & 4) ? hi.z : lo.z}};
    ksl_vec3_t q;
    ksl_product_dv(&d, &p, &q);
    out.Add(q);
  }
  return out;
}

/* ------------------------------------------------------------------------ */
/* evaluation on the faces of a shape */

const shift::core::cad::Model&
shift::core::collision_geometry::CadContactShape::model() const {
  return asset->model;
}

ksl_vec3_t shift::core::collision_geometry::CadContactShape::Point(
  int face, const Vec2& uv, ksl_vec3_t* du, ksl_vec3_t* dv) const {
  const CadContactFace& f = faces[face];
  const CadPlacement& place = placements[f.placement];
  const auto& m = model();
  const auto& s = m.surfaces[m.faces[f.face].surface];
  cad::Vec3 file_du, file_dv;
  const cad::Vec3 p =
    cad::SurfacePoint(m, s, uv[0], uv[1], du ? &file_du : nullptr,
                      dv ? &file_dv : nullptr);
  if(du) {
    *du = place.ApplyVector(file_du);
  }
  if(dv) {
    *dv = place.ApplyVector(file_dv);
  }
  return place.Apply(p);
}

ksl_vec3_t shift::core::collision_geometry::CadContactShape::Normal(
  int face, const Vec2& uv) const {
  ksl_vec3_t du, dv, n;
  Point(face, uv, &du, &dv);
  ksl_cross_vv(&du, &dv, &n);
  const double length = ksl_vec3_l2norm(&n);
  if(!(length > 0.0)) {
    return {{0.0, 0.0, 0.0}};
  }
  const double sense = model().faces[faces[face].face].same_sense ? 1.0 : -1.0;
  ksl_vec3_scale(&n, sense / length);
  return n;
}

/* the step of finite differences in a parameter spanning lo to hi */
static double Step(double lo, double hi) {
  return 1e-5 * std::max(hi - lo, 1e-9);
}

/* the first and second fundamental forms of a face at (u, v), I = (E, F, G)
and II = (L, M, N) with the outward normal; false where the face is planar
(II = 0) or its parameters degenerate */
static bool FundamentalForms(const CadContactShape& shape, int face,
                             const Vec2& uv, ksl_vec3_t& du, ksl_vec3_t& dv,
                             double first[3], double second[3]) {
  const CadContactFace& f = shape.faces[face];
  const auto& m = shape.model();
  if(m.surfaces[m.faces[f.face].surface].type ==
     shift::core::cad::Surface::Type::kPlane) {
    return false;
  }
  shape.Point(face, uv, &du, &dv);
  const ksl_vec3_t n = shape.Normal(face, uv);
  /* second derivatives by central differences of the first */
  const double hu = Step(f.lo[0], f.hi[0]);
  const double hv = Step(f.lo[1], f.hi[1]);
  ksl_vec3_t du_p, dv_p, du_m, dv_m, du_vp, dv_vp, du_vm, dv_vm;
  shape.Point(face, {uv[0] + hu, uv[1]}, &du_p, &dv_p);
  shape.Point(face, {uv[0] - hu, uv[1]}, &du_m, &dv_m);
  shape.Point(face, {uv[0], uv[1] + hv}, &du_vp, &dv_vp);
  shape.Point(face, {uv[0], uv[1] - hv}, &du_vm, &dv_vm);
  ksl_vec3_t duu, duv, dvv;
  for(int k = 0; k < 3; k++) {
    duu.at[k] = (du_p.at[k] - du_m.at[k]) / (2.0 * hu);
    dvv.at[k] = (dv_vp.at[k] - dv_vm.at[k]) / (2.0 * hv);
    duv.at[k] = 0.5 * ((dv_p.at[k] - dv_m.at[k]) / (2.0 * hu) +
                       (du_vp.at[k] - du_vm.at[k]) / (2.0 * hv));
  }
  first[0] = ksl_dot_vv(&du, &du);
  first[1] = ksl_dot_vv(&du, &dv);
  first[2] = ksl_dot_vv(&dv, &dv);
  second[0] = ksl_dot_vv(&duu, &n);
  second[1] = ksl_dot_vv(&duv, &n);
  second[2] = ksl_dot_vv(&dvv, &n);
  return first[0] * first[2] - first[1] * first[1] > 0.0;
}

double shift::core::collision_geometry::CadContactShape::MeanCurvature(
  int face, const Vec2& uv) const {
  ksl_vec3_t du, dv;
  double I[3], II[3];
  if(!FundamentalForms(*this, face, uv, du, dv, I, II)) {
    return 0.0;
  }
  const double det = I[0] * I[2] - I[1] * I[1];
  /* with the outward normal, a convex face curves away from it: its second
  fundamental form is negative */
  return -(I[0] * II[2] - 2.0 * I[1] * II[1] + I[2] * II[0]) / (2.0 * det);
}

void shift::core::collision_geometry::CadContactShape::Curvature(
  int face, const Vec2& uv, const ksl_vec3_t& t1, const ksl_vec3_t& t2,
  double c[3]) const {
  c[0] = c[1] = c[2] = 0.0;
  ksl_vec3_t du, dv;
  double I[3], II[3];
  if(!FundamentalForms(*this, face, uv, du, dv, I, II)) {
    return;
  }
  /* the (u, v) components w of each tangent t, from du w_u + dv w_v = t in
  the least squares sense: I w = (du . t, dv . t) */
  const double det = I[0] * I[2] - I[1] * I[1];
  double w[2][2];
  const ksl_vec3_t* t[2] = {&t1, &t2};
  for(int i = 0; i < 2; i++) {
    const double a = ksl_dot_vv(&du, t[i]), b = ksl_dot_vv(&dv, t[i]);
    w[i][0] = (I[2] * a - I[1] * b) / det;
    w[i][1] = (I[0] * b - I[1] * a) / det;
  }
  auto form = [&](int i, int j) {
    return -(w[i][0] * (II[0] * w[j][0] + II[1] * w[j][1]) +
             w[i][1] * (II[1] * w[j][0] + II[2] * w[j][1]));
  };
  c[0] = form(0, 0);
  c[1] = form(0, 1);
  c[2] = form(1, 1);
}

bool shift::core::collision_geometry::CadContactShape::Project(
  int face, const ksl_vec3_t& p, Vec2& uv, const Vec2* guess) const {
  const CadContactFace& f = faces[face];
  const auto& m = model();
  const cad::Vec3 q = placements[f.placement].Invert(p);
  return cad::ProjectToSurface(m, m.surfaces[m.faces[f.face].surface], q, uv,
                               guess);
}

/* the crossing of the side of a polygon by a ray from p along +u */
static bool Crosses(const std::array<double, 4>& side, const Vec2& p) {
  const double ua = side[0], va = side[1], ub = side[2], vb = side[3];
  if((va > p[1]) == (vb > p[1])) {
    return false;
  }
  return p[0] < ua + (p[1] - va) * (ub - ua) / (vb - va);
}

/* the band of a face holding v (clamped to the bands) */
static int Band(const shift::core::collision_geometry::CadContactFace& f,
                double v) {
  const int count = static_cast<int>(f.bands.size());
  const double h = f.hi[1] - f.lo[1];
  if(!(h > 0.0)) {
    return 0;
  }
  const int band = static_cast<int>((v - f.lo[1]) / h * count);
  return std::min(std::max(band, 0), count - 1);
}

/* whether (u, v) is inside a face's trims by the crossings of a ray along
+u: an odd count is inside, as for an outer loop and its holes */
static bool InsideTrims(const shift::core::collision_geometry::CadContactFace& f,
                        const Vec2& p) {
  bool inside = false;
  for(int i : f.bands[Band(f, p[1])]) {
    if(Crosses(f.sides[i], p)) {
      inside = !inside;
    }
  }
  return inside;
}

/* the sides of a face's trims and their bands */
static void BuildBands(shift::core::collision_geometry::CadContactFace& f) {
  f.sides.clear();
  for(const auto& poly : f.trims) {
    const size_t n = poly.size();
    for(size_t i = 0, j = n - 1; i < n; j = i++) {
      f.sides.push_back({poly[j][0], poly[j][1], poly[i][0], poly[i][1]});
    }
  }
  const int count =
    std::max(1, std::min(64, static_cast<int>(f.sides.size()) / 4));
  f.bands.assign(count, {});
  for(int i = 0; i < static_cast<int>(f.sides.size()); i++) {
    const auto& side = f.sides[i];
    const int b0 = Band(f, std::min(side[1], side[3]));
    const int b1 = Band(f, std::max(side[1], side[3]));
    for(int b = b0; b <= b1; b++) {
      f.bands[b].push_back(i);
    }
  }
}

/* a periodic parameter shifted by whole periods nearest the middle of lo
and hi */
static double InPeriod(double x, double lo, double hi) {
  const double period = 2.0 * M_PI;
  const double mid = 0.5 * (lo + hi);
  return x - period * std::round((x - mid) / period);
}

bool shift::core::collision_geometry::CadContactShape::Inside(
  int face, const Vec2& uv) const {
  const CadContactFace& f = faces[face];
  Vec2 p = uv;
  if(f.periodic_u) {
    p[0] = InPeriod(p[0], f.lo[0], f.hi[0]);
  }
  if(f.periodic_v) {
    p[1] = InPeriod(p[1], f.lo[1], f.hi[1]);
  }
  if(p[0] < f.lo[0] || p[0] > f.hi[0] || p[1] < f.lo[1] || p[1] > f.hi[1]) {
    return false;
  }
  return InsideTrims(f, p);
}

std::vector<std::array<double, 2>>
shift::core::collision_geometry::CadContactShape::Clip(int face, const Vec2& a,
                                                       const Vec2& b) const {
  const CadContactFace& f = faces[face];
  Vec2 p = a, q = b;
  for(int k = 0; k < 2; k++) {
    if(k == 0 ? f.periodic_u : f.periodic_v) {
      const double mid = 0.5 * (p[k] + q[k]);
      const double shift = InPeriod(mid, f.lo[k], f.hi[k]) - mid;
      p[k] += shift;
      q[k] += shift;
    }
  }
  /* the parameters where the segment crosses the polygons' sides */
  std::vector<double> cuts{0.0, 1.0};
  const double dx = q[0] - p[0], dy = q[1] - p[1];
  const int b0 = Band(f, std::min(p[1], q[1]));
  const int b1 = Band(f, std::max(p[1], q[1]));
  for(int b = b0; b <= b1; b++) {
    for(int i : f.bands[b]) {
      const auto& side = f.sides[i];
      if(b > b0 && Band(f, std::min(side[1], side[3])) < b) {
        continue; // seen in an earlier band
      }
      const double ex = side[2] - side[0], ey = side[3] - side[1];
      const double det = dx * ey - dy * ex;
      if(det == 0.0) {
        continue; // parallel
      }
      const double wx = side[0] - p[0], wy = side[1] - p[1];
      const double t = (wx * ey - wy * ex) / det;
      const double s = (wx * dy - wy * dx) / det;
      if(t > 0.0 && t < 1.0 && s >= 0.0 && s <= 1.0) {
        cuts.push_back(t);
      }
    }
  }
  std::sort(cuts.begin(), cuts.end());
  std::vector<std::array<double, 2>> pieces;
  for(size_t i = 0; i + 1 < cuts.size(); i++) {
    const double t0 = cuts[i], t1 = cuts[i + 1];
    if(!(t1 > t0)) {
      continue;
    }
    const double t = 0.5 * (t0 + t1);
    if(!InsideTrims(f, {p[0] + t * dx, p[1] + t * dy})) {
      continue;
    }
    if(!pieces.empty() && pieces.back()[1] == t0) {
      pieces.back()[1] = t1;
    } else {
      pieces.push_back({t0, t1});
    }
  }
  return pieces;
}

/* ------------------------------------------------------------------------ */
/* the shape */

/* pieces of each curved trim curve in the polygons of the trims, for the
inside test: the chord of an arc of a quarter turn in 32 pieces lies within
0.03% of its radius */
static constexpr int kTrimSegments = 32;
/* samples of a face along u and v, inside its trims */
static constexpr int kFaceSamples = 8;
/* samples of an edge between its ends */
static constexpr int kEdgeSamples = 7;

static void BuildTree(shift::core::collision_geometry::CadContactShape& shape,
                      int node, int first, int count) {
  using shift::core::collision_geometry::CadBox;
  const auto& faces = shape.faces;
  CadBox box;
  for(int i = first; i < first + count; i++) {
    box.Add(faces[shape.order[i]].box);
  }
  shape.nodes[node].box = box;
  if(count <= 2) {
    shape.nodes[node].first = first;
    shape.nodes[node].count = count;
    return;
  }
  int axis = 0;
  for(int k = 1; k < 3; k++) {
    if(box.hi.at[k] - box.lo.at[k] > box.hi.at[axis] - box.lo.at[axis]) {
      axis = k;
    }
  }
  auto center = [&](int i) {
    return faces[i].box.lo.at[axis] + faces[i].box.hi.at[axis];
  };
  const int half = count / 2;
  std::nth_element(shape.order.begin() + first,
                   shape.order.begin() + first + half,
                   shape.order.begin() + first + count,
                   [&](int a, int b) { return center(a) < center(b); });
  const int left = static_cast<int>(shape.nodes.size());
  shape.nodes.emplace_back();
  shape.nodes.emplace_back();
  shape.nodes[node].left = left;
  shape.nodes[node].right = left + 1;
  BuildTree(shape, left, first, half);
  BuildTree(shape, left + 1, first + half, count - half);
}

void shift::core::collision_geometry::CadContactShape::Query(
  const CadBox& query, std::vector<int>& found) const {
  if(nodes.empty()) {
    return;
  }
  std::vector<int> stack{0};
  while(!stack.empty()) {
    const CadBoxNode& node = nodes[stack.back()];
    stack.pop_back();
    if(!node.box.Overlaps(query)) {
      continue;
    }
    if(node.left < 0) {
      for(int i = node.first; i < node.first + node.count; i++) {
        if(faces[order[i]].box.Overlaps(query)) {
          found.push_back(order[i]);
        }
      }
    } else {
      stack.push_back(node.left);
      stack.push_back(node.right);
    }
  }
}

static double Diagonal(const shift::core::collision_geometry::CadBox& box) {
  ksl_vec3_t d;
  ksl_subtract_vv(&box.hi, &box.lo, &d);
  return ksl_vec3_l2norm(&d);
}

std::shared_ptr<const shift::core::collision_geometry::CadContactShape>
shift::core::collision_geometry::MakeCadContactShape(
  std::shared_ptr<const geometry::CadAsset> asset, double scale) {
  auto shape = std::make_shared<CadContactShape>();
  shape->asset = asset;
  const cad::Model& m = asset->model;
  for(const auto& placed : asset->bodies) {
    /* cad::Transform is row-major, ksl_SE3_t column-major */
    CadPlacement place;
    place.scale = scale;
    for(int row = 0; row < 3; row++) {
      for(int col = 0; col < 3; col++) {
        place.d.at[col][row] = placed.transform.rotation[row * 3 + col];
      }
      place.d.t.at[row] = scale * placed.transform.translation[row];
    }
    shape->placements.push_back(place);
  }

  /* the faces, edges and vertices of each placed body, before the frame is
  centered on their bounds */
  for(size_t pi = 0; pi < asset->bodies.size(); pi++) {
    const cad::Body& body = m.bodies[asset->bodies[pi].body];
    std::map<int, int> edge_index, vertex_index;
    for(int shell : body.shells) {
      for(int fi : m.shells[shell].faces) {
        const cad::Face& face = m.faces[fi];
        if(face.trims.empty()) {
          continue; // not trimmed by Repair: not usable for contact
        }
        CadContactFace f;
        f.face = fi;
        f.placement = static_cast<int>(pi);
        const auto domain = cad::SurfaceDomain(m, m.surfaces[face.surface]);
        f.periodic_u = domain[0].periodic;
        f.periodic_v = domain[1].periodic;
        f.lo = {HUGE_VAL, HUGE_VAL};
        f.hi = {-HUGE_VAL, -HUGE_VAL};
        for(const auto& loop : face.trims) {
          f.trims.push_back(cad::TrimPolygon(m, loop, kTrimSegments));
          for(const Vec2& p : f.trims.back()) {
            for(int k = 0; k < 2; k++) {
              f.lo[k] = std::min(f.lo[k], p[k]);
              f.hi[k] = std::max(f.hi[k], p[k]);
            }
          }
        }
        BuildBands(f);
        const int index = static_cast<int>(shape->faces.size());
        for(const auto& loop : face.loops) {
          for(const auto& coedge : loop.coedges) {
            const auto& e = m.edges[coedge.edge];
            auto [it, inserted] = edge_index.try_emplace(
              coedge.edge, static_cast<int>(shape->edges.size()));
            if(inserted) {
              CadContactEdge edge;
              edge.edge = coedge.edge;
              edge.placement = static_cast<int>(pi);
              shape->edges.push_back(edge);
            }
            auto& faces_of_edge = shape->edges[it->second].faces;
            if(std::find(faces_of_edge.begin(), faces_of_edge.end(), index) ==
               faces_of_edge.end()) {
              faces_of_edge.push_back(index);
              f.edges.push_back(it->second);
            }
            for(int v : {e.start, e.end}) {
              if(v < 0) {
                continue;
              }
              auto [vt, vinserted] = vertex_index.try_emplace(
                v, static_cast<int>(shape->vertices.size()));
              if(vinserted) {
                CadContactVertex vertex;
                vertex.point = shape->placements[pi].Apply(m.vertices[v].point);
                shape->vertices.push_back(vertex);
              }
              auto& faces_of_vertex = shape->vertices[vt->second].faces;
              if(std::find(faces_of_vertex.begin(), faces_of_vertex.end(),
                           index) == faces_of_vertex.end()) {
                faces_of_vertex.push_back(index);
                f.vertices.push_back(vt->second);
              }
            }
          }
        }
        shape->faces.push_back(std::move(f));
      }
    }
  }

  /* samples of the edges, and their bounds: through their ends and
  samples, padded by the bulge of a curve between them */
  for(auto& edge : shape->edges) {
    const auto& e = m.edges[edge.edge];
    for(int v : {e.start, e.end}) {
      if(v >= 0) {
        edge.box.Add(shape->placements[edge.placement].Apply(m.vertices[v].point));
      }
    }
    if(e.curve < 0 || !e.has_range) {
      continue;
    }
    for(int i = 1; i <= kEdgeSamples; i++) {
      const double t =
        e.range[0] + (e.range[1] - e.range[0]) * i / (kEdgeSamples + 1);
      edge.sample_t.push_back(t);
      edge.sample_points.push_back(shape->placements[edge.placement].Apply(
        cad::CurvePoint<3>(m.curves[e.curve], t)));
      edge.box.Add(edge.sample_points.back());
    }
    if(m.curves[e.curve].type != cad::Curve3::Type::kLine) {
      edge.box.Pad(0.1 * Diagonal(edge.box));
    }
  }

  /* samples of the faces: a grid inside the trims and the points of the
  trims; their bounds, padded by the bulge of a curved face between them */
  for(int fi = 0; fi < static_cast<int>(shape->faces.size()); fi++) {
    CadContactFace& f = shape->faces[fi];
    for(int i = 0; i < kFaceSamples; i++) {
      for(int j = 0; j < kFaceSamples; j++) {
        const Vec2 uv{f.lo[0] + (f.hi[0] - f.lo[0]) * (i + 0.5) / kFaceSamples,
                      f.lo[1] + (f.hi[1] - f.lo[1]) * (j + 0.5) / kFaceSamples};
        if(InsideTrims(f, uv)) {
          f.sample_uv.push_back(uv);
        }
      }
    }
    for(const auto& poly : f.trims) {
      for(const Vec2& uv : poly) {
        f.boundary_points.push_back(shape->Point(fi, uv));
        f.box.Add(f.boundary_points.back());
      }
    }
    for(const Vec2& uv : f.sample_uv) {
      f.sample_points.push_back(shape->Point(fi, uv));
      f.sample_normals.push_back(shape->Normal(fi, uv));
      f.box.Add(f.sample_points.back());
    }
    if(m.surfaces[m.faces[f.face].surface].type !=
       cad::Surface::Type::kPlane) {
      f.box.Pad(0.1 * Diagonal(f.box));
    }
    shape->box.Add(f.box);
  }

  /* the frame of the geometry: centered on the bounds */
  ksl_vec3_t& center = shape->center;
  ksl_add_vv(&shape->box.lo, &shape->box.hi, &center);
  ksl_vec3_scale(&center, 0.5);
  auto recenter = [&](ksl_vec3_t& p) { ksl_nxpy_vv(&center, &p); };
  for(auto& place : shape->placements) {
    recenter(place.d.t);
  }
  for(auto& f : shape->faces) {
    for(auto& p : f.sample_points) {
      recenter(p);
    }
    for(auto& p : f.boundary_points) {
      recenter(p);
    }
    recenter(f.box.lo);
    recenter(f.box.hi);
  }
  for(auto& e : shape->edges) {
    for(auto& p : e.sample_points) {
      recenter(p);
    }
    recenter(e.box.lo);
    recenter(e.box.hi);
  }
  for(auto& v : shape->vertices) {
    recenter(v.point);
  }
  recenter(shape->box.lo);
  recenter(shape->box.hi);
  shape->size = Diagonal(shape->box);

  /* points for Support */
  for(const auto& f : shape->faces) {
    shape->hull_points.insert(shape->hull_points.end(),
                              f.boundary_points.begin(),
                              f.boundary_points.end());
    shape->hull_points.insert(shape->hull_points.end(),
                              f.sample_points.begin(), f.sample_points.end());
  }
  for(const auto& e : shape->edges) {
    shape->hull_points.insert(shape->hull_points.end(),
                              e.sample_points.begin(), e.sample_points.end());
  }
  for(const auto& v : shape->vertices) {
    shape->hull_points.push_back(v.point);
  }

  /* the box tree */
  if(!shape->faces.empty()) {
    shape->order.resize(shape->faces.size());
    for(size_t i = 0; i < shape->order.size(); i++) {
      shape->order[i] = static_cast<int>(i);
    }
    shape->nodes.emplace_back();
    BuildTree(*shape, 0, 0, static_cast<int>(shape->faces.size()));
  }
  return shape;
}

/* ------------------------------------------------------------------------ */
/* the collision geometry */

void shift::core::collision_geometry::Cad::Load() {
  auto body = get_parent_body();
  if(!body) {
    throw shift::core::Exception("Collision geometry '" + name_ +
                                 "' is not attached to a body");
  }
  if(name_.empty()) {
    name_ = body->get_name();
  }
  const std::string description =
    "CAD collision geometry '" + name_ + "' of body '" + body->get_name() +
    "'";
  if(fit_to_.size() > 1) {
    throw shift::core::Exception(description +
                                 " names more than one geometry in fit_to");
  }
  const ksl_SE3_t& pose = *pose_.get_d_wrt_origin();
  for(int i = 0; i < 4; i++) {
    for(int k = 0; k < 3; k++) {
      if(pose.at[i][k] != (i == k ? 1.0 : 0.0)) {
        throw shift::core::Exception(
          description + " gives a pose; it is placed by its CAD geometry");
      }
    }
  }
  geometry::Cad* cad = nullptr;
  for(auto& geometry : body->get_geometry()) {
    auto g = dynamic_cast<geometry::Cad*>(geometry.get());
    if(g && (fit_to_.empty() ? g->is_active()
                             : g->get_name() == fit_to_.front())) {
      cad = g;
      break;
    }
  }
  if(!cad) {
    throw shift::core::Exception(
      description + (fit_to_.empty()
                       ? " needs an active CAD geometry of the body"
                       : " fits to '" + fit_to_.front() +
                           "', which is not a CAD geometry of the body"));
  }
  if(!cad->get_asset()) {
    throw shift::core::Exception(description + ": the CAD geometry '" +
                                 cad->get_name() + "' is not loaded");
  }

  /* geometry of the same file at the same scale shares its shape */
  std::ostringstream key;
  key << type_ << ":" << cad->get_asset() << ":" << cad->get_scale();
  if(auto prototype = context_->FindCollisionGeometryPrototype(key.str())) {
    CopyFit(*prototype);
  } else {
    shape_ = MakeCadContactShape(cad->get_shared_asset(), cad->get_scale());
    context_->AddCollisionGeometryPrototype(key.str(), this);
  }
  if(shape_->faces.empty()) {
    SHIFT_WARNING(description + " has no trimmed faces; it is ignored in "
                                "contact detection");
    return;
  }

  /* the frame: the body origin frame translated to the center of the
  bounds */
  ksl_SE3_setIdentity(&d_wrt_body_);
  d_wrt_body_.t = shape_->center;
  bounding_radius_ = 0.5 * shape_->size;
  if(!(edge_radius_ > 0.0)) {
    edge_radius_ = 0.002 * shape_->size;
  }
  if(!(max_depth_ > 0.0)) {
    max_depth_ = 0.02 * shape_->size;
  }
  loaded_ = true;
}

void shift::core::collision_geometry::Cad::Fit(
  const std::vector<ksl_vec3_t>&, const std::vector<unsigned int>&) {
  throw shift::core::Exception("CAD collision geometry '" + name_ +
                               "' is not fitted to meshes");
}

void shift::core::collision_geometry::Cad::CopyFit(const Base& other) {
  auto cad = dynamic_cast<const Cad*>(&other);
  if(!cad) {
    throw shift::core::Exception("Collision geometry '" + name_ +
                                 "' copies a geometry of another type");
  }
  shape_ = cad->shape_;
}

void shift::core::collision_geometry::Cad::Support(
  const ksl_vec3_t& direction, ksl_vec3_t& local, ksl_vec3_t& global,
  int& hint) const {
  ksl_vec3_t dir;
  ksl_product_drinvv(&d_absolute_.R, &direction, &dir);
  const auto& points = shape_->hull_points;
  int best = 0;
  double extent = -HUGE_VAL;
  for(size_t i = 0; i < points.size(); i++) {
    const double e = ksl_dot_vv(&points[i], &dir);
    if(e > extent) {
      extent = e;
      best = static_cast<int>(i);
    }
  }
  hint = best;
  local = points[best];
  ksl_product_dv(&d_absolute_, &local, &global);
}
