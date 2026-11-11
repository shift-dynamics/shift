#include "shift/core/collision_geometry/cad.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>
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
    "contact_seeds", &contact_seeds_,
    "how the starting points of the search for contact between two faces are "
    "found: \"samples\" (the default) scores each sample of either face "
    "against each sample of the other; \"field\" looks each sample up in a "
    "field of the closest points of the other face, built at load, and starts "
    "Newton's method there. Two CAD geometries use the field only if both ask "
    "for it",
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

void shift::core::collision_geometry::CadBox::Add(const shift::core::collision_geometry::CadBox& b) {
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
  const shift::core::collision_geometry::CadBox& b) const {
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
  shift::core::collision_geometry::CadBox out;
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
  int face, const shift::core::cad::Vec2& uv, ksl_vec3_t* du, ksl_vec3_t* dv) const {
  const shift::core::collision_geometry::CadContactFace& f = faces[face];
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

/* the parameter, 0 or 1, along which a face's point does not move at (u, v):
at a pole of a sphere, or the apex of a cone, the point is the same for all u.
-1 if neither degenerates */
static int Degenerate(const ksl_vec3_t& du, const ksl_vec3_t& dv) {
  const double lu = ksl_vec3_l2norm(&du), lv = ksl_vec3_l2norm(&dv);
  if(lu <= 1e-9 * lv) {
    return 0;
  }
  if(lv <= 1e-9 * lu) {
    return 1;
  }
  return -1;
}

/* (u, v) moved a share of the face's span along the parameter other than
the degenerate one, into the face */
static shift::core::cad::Vec2
OffPole(const shift::core::collision_geometry::CadContactFace& f,
        const shift::core::cad::Vec2& uv, int degenerate, double share) {
  const int k = 1 - degenerate;
  const double h = share * std::max(f.hi[k] - f.lo[k], 1e-9);
  shift::core::cad::Vec2 moved = uv;
  moved[k] += uv[k] + h <= f.hi[k] ? h : -h;
  return moved;
}

/* the unit normal of the face's surface at (u, v), with the sense of the
surface's parameters; zero where they degenerate */
static ksl_vec3_t RawNormal(const shift::core::collision_geometry::CadContactShape& shape,
                            int face, const shift::core::cad::Vec2& uv,
                            int& degenerate) {
  ksl_vec3_t du, dv, n;
  shape.Point(face, uv, &du, &dv);
  degenerate = Degenerate(du, dv);
  ksl_cross_vv(&du, &dv, &n);
  const double length = ksl_vec3_l2norm(&n);
  if(degenerate >= 0 || !(length > 0.0)) {
    return {{0.0, 0.0, 0.0}};
  }
  ksl_vec3_scale(&n, 1.0 / length);
  return n;
}

ksl_vec3_t shift::core::collision_geometry::CadContactShape::Normal(
  int face, const shift::core::cad::Vec2& uv) const {
  int degenerate;
  ksl_vec3_t n = RawNormal(*this, face, uv, degenerate);
  if(degenerate >= 0) {
    /* at a pole, the limit of the normal toward it, extrapolated from two
    points off it: its error is of the second order in their offset */
    const shift::core::collision_geometry::CadContactFace& f = faces[face];
    int ignored;
    const ksl_vec3_t n1 =
      RawNormal(*this, face, OffPole(f, uv, degenerate, 1e-6), ignored);
    const ksl_vec3_t n2 =
      RawNormal(*this, face, OffPole(f, uv, degenerate, 2e-6), ignored);
    for(int k = 0; k < 3; k++) {
      n.at[k] = 2.0 * n1.at[k] - n2.at[k];
    }
  }
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
static bool FundamentalForms(const shift::core::collision_geometry::CadContactShape& shape, int face,
                             const shift::core::cad::Vec2& uv, ksl_vec3_t& du, ksl_vec3_t& dv,
                             double first[3], double second[3]) {
  const shift::core::collision_geometry::CadContactFace& f = shape.faces[face];
  const auto& m = shape.model();
  if(m.surfaces[m.faces[f.face].surface].type ==
     shift::core::cad::Surface::Type::kPlane) {
    return false;
  }
  shape.Point(face, uv, &du, &dv);
  /* at a pole the forms degenerate: those just off it */
  const int degenerate = Degenerate(du, dv);
  if(degenerate >= 0) {
    const shift::core::cad::Vec2 off = OffPole(f, uv, degenerate, 1e-4);
    shape.Point(face, off, &du, &dv);
    if(Degenerate(du, dv) >= 0) {
      return false;
    }
    return FundamentalForms(shape, face, off, du, dv, first, second);
  }
  const ksl_vec3_t n = shape.Normal(face, uv);
  /* second derivatives by central differences of the first, within the
  face's bounds: a NURBS surface does not go beyond them */
  const double hu = Step(f.lo[0], f.hi[0]);
  const double hv = Step(f.lo[1], f.hi[1]);
  const double u = f.periodic_u ? uv[0]
                                : std::min(std::max(uv[0], f.lo[0] + hu),
                                           f.hi[0] - hu);
  const double v = f.periodic_v ? uv[1]
                                : std::min(std::max(uv[1], f.lo[1] + hv),
                                           f.hi[1] - hv);
  ksl_vec3_t du_p, dv_p, du_m, dv_m, du_vp, dv_vp, du_vm, dv_vm;
  shape.Point(face, {u + hu, v}, &du_p, &dv_p);
  shape.Point(face, {u - hu, v}, &du_m, &dv_m);
  shape.Point(face, {u, v + hv}, &du_vp, &dv_vp);
  shape.Point(face, {u, v - hv}, &du_vm, &dv_vm);
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
  int face, const shift::core::cad::Vec2& uv) const {
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
  int face, const shift::core::cad::Vec2& uv, const ksl_vec3_t& t1, const ksl_vec3_t& t2,
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
  int face, const ksl_vec3_t& p, shift::core::cad::Vec2& uv, const shift::core::cad::Vec2* guess) const {
  const shift::core::collision_geometry::CadContactFace& f = faces[face];
  const auto& m = model();
  const cad::Vec3 q = placements[f.placement].Invert(p);
  const cad::Surface& surface = m.surfaces[m.faces[f.face].surface];
  /* a B-spline, revolution or extrusion is projected on by Newton's method
  from a guess, else from the nearest of a grid over the whole surface: the
  nearest of the face's samples is a guess, and far fewer points */
  const bool analytic = surface.type == cad::Surface::Type::kPlane ||
                        surface.type == cad::Surface::Type::kCylinder ||
                        surface.type == cad::Surface::Type::kCone ||
                        surface.type == cad::Surface::Type::kSphere ||
                        surface.type == cad::Surface::Type::kTorus;
  if(!guess && !analytic && !f.sample_points.empty()) {
    double nearest = HUGE_VAL;
    size_t best = 0;
    for(size_t i = 0; i < f.sample_points.size(); i++) {
      const double dx = f.sample_points[i].x - p.x;
      const double dy = f.sample_points[i].y - p.y;
      const double dz = f.sample_points[i].z - p.z;
      const double squared = dx * dx + dy * dy + dz * dz;
      if(squared < nearest) {
        nearest = squared;
        best = i;
      }
    }
    return cad::ProjectToSurface(m, surface, q, uv, &f.sample_uv[best]);
  }
  if(!cad::ProjectToSurface(m, surface, q, uv, guess)) {
    return false;
  }
  /* an angle with no guess is in [0, 2 pi): that of the turn the face is
  on, as for a face of half a turn about its axis */
  if(!guess && analytic && surface.type != cad::Surface::Type::kPlane) {
    const int angles = surface.type == cad::Surface::Type::kTorus ? 2 : 1;
    for(int k = 0; k < angles; k++) {
      const double middle = 0.5 * (f.lo[k] + f.hi[k]);
      uv[k] += 2.0 * M_PI * std::round((middle - uv[k]) / (2.0 * M_PI));
    }
  }
  return true;
}

/* the crossing of the side of a polygon by a ray from p along +u */
static bool Crosses(const std::array<double, 4>& side, const shift::core::cad::Vec2& p) {
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
                        const shift::core::cad::Vec2& p) {
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
  int face, const shift::core::cad::Vec2& uv) const {
  const shift::core::collision_geometry::CadContactFace& f = faces[face];
  shift::core::cad::Vec2 p = uv;
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
shift::core::collision_geometry::CadContactShape::Clip(int face, const shift::core::cad::Vec2& a,
                                                       const shift::core::cad::Vec2& b,
                                                       double inset) const {
  const shift::core::collision_geometry::CadContactFace& f = faces[face];
  shift::core::cad::Vec2 p = a, q = b;
  for(int k = 0; k < 2; k++) {
    if(k == 0 ? f.periodic_u : f.periodic_v) {
      const double mid = 0.5 * (p[k] + q[k]);
      const double shift = InPeriod(mid, f.lo[k], f.hi[k]) - mid;
      p[k] += shift;
      q[k] += shift;
    }
    if(inset > 0.0) {
      const double margin = inset * (f.hi[k] - f.lo[k]);
      for(double* end : {&p[k], &q[k]}) {
        if(std::fabs(*end - f.lo[k]) < margin) {
          *end = f.lo[k] + margin;
        } else if(std::fabs(*end - f.hi[k]) < margin) {
          *end = f.hi[k] - margin;
        }
      }
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
/* samples of a face along u and v, inside its trims: at least these, more on
a face turning further than half a circle or of more than two spans of a
B-spline, so that the samples stay this close together on a long face */
static constexpr int kFaceSamples = 8;
static constexpr int kSamplesPerSpan = 3;
static constexpr int kMaxFaceSamples = 64;

/* the samples of a face along its parameter k (0 for u, 1 for v), over
[lo, hi]: an angle, or the parameter of a B-spline */
static int FaceSamples(const shift::core::cad::Surface& s, int k, double lo,
                       double hi) {
  const shift::core::cad::Surface::Type t = s.type;
  const bool angle =
    (k == 0 && (t == shift::core::cad::Surface::Type::kCylinder ||
                t == shift::core::cad::Surface::Type::kCone ||
                t == shift::core::cad::Surface::Type::kRevolution)) ||
    t == shift::core::cad::Surface::Type::kSphere ||
    t == shift::core::cad::Surface::Type::kTorus;
  int count = kFaceSamples;
  if(angle) {
    count = static_cast<int>(std::ceil(kFaceSamples * (hi - lo) / M_PI - 1e-9));
  } else if(t == shift::core::cad::Surface::Type::kNurbs) {
    const std::vector<double>& knots = k == 0 ? s.nurbs.u_knots
                                              : s.nurbs.v_knots;
    int spans = 1;
    for(size_t i = 1; i < knots.size(); i++) {
      if(knots[i] > knots[i - 1] && knots[i - 1] > lo && knots[i - 1] < hi) {
        spans++;
      }
    }
    count = kSamplesPerSpan * spans;
  }
  return std::min(std::max(count, kFaceSamples), kMaxFaceSamples);
}
/* samples of an edge between its ends */
static constexpr int kEdgeSamples = 7;

static void BuildTree(shift::core::collision_geometry::CadContactShape& shape,
                      int node, int first, int count) {
  const auto& faces = shape.faces;
  shift::core::collision_geometry::CadBox box;
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
  const shift::core::collision_geometry::CadBox& query, std::vector<int>& found) const {
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

/* marks the convex edges between two faces that meet at an angle, from the
middle of each: the faces' normals there, and the direction into the first */
static void MarkSharpEdges(
  shift::core::collision_geometry::CadContactShape& shape) {
  const auto& m = shape.model();
  for(auto& edge : shape.edges) {
    const auto& e = m.edges[edge.edge];
    if(edge.faces.size() != 2 || e.curve < 0 || !e.has_range) {
      continue;
    }
    const auto& curve = m.curves[e.curve];
    const auto& place = shape.placements[edge.placement];
    auto point_at = [&](double t) {
      return place.Apply(shift::core::cad::CurvePoint<3>(curve, t));
    };
    const double middle = 0.5 * (e.range[0] + e.range[1]);
    const double step = 1e-4 * (e.range[1] - e.range[0]);
    const ksl_vec3_t p = point_at(middle);
    const ksl_vec3_t after = point_at(middle + step);
    const ksl_vec3_t before = point_at(middle - step);
    ksl_vec3_t tangent;
    ksl_subtract_vv(&after, &before, &tangent);
    const double length = ksl_vec3_l2norm(&tangent);
    shift::core::cad::Vec2 uv[2];
    if(!(length > 0.0) || !shape.Project(edge.faces[0], p, uv[0]) ||
       !shape.Project(edge.faces[1], p, uv[1])) {
      continue;
    }
    ksl_vec3_scale(&tangent, 1.0 / length);
    const ksl_vec3_t n0 = shape.Normal(edge.faces[0], uv[0]);
    const ksl_vec3_t n1 = shape.Normal(edge.faces[1], uv[1]);
    ksl_vec3_t cross, inward;
    ksl_cross_vv(&n0, &n1, &cross);
    if(ksl_vec3_l2norm(&cross) < 0.05) {
      edge.smooth = true; // the faces meet smoothly
      continue;
    }
    ksl_cross_vv(&tangent, &n0, &inward);
    for(double sense : {1.0, -1.0}) {
      ksl_vec3_t q = p;
      ksl_axpy_vv(sense * 1e-5 * shape.size, &inward, &q);
      shift::core::cad::Vec2 at;
      if(shape.Project(edge.faces[0], q, at) &&
         shape.Inside(edge.faces[0], at)) {
        edge.sharp = sense * ksl_dot_vv(&n1, &inward) < 0.0;
        break;
      }
    }
  }
}

/* ------------------------------------------------------------------------ */
/* closest-point fields */

const shift::core::collision_geometry::CadFaceField::Node*
shift::core::collision_geometry::CadFaceField::Nearest(const ksl_vec3_t& p) const {
  if(!(cell > 0.0)) {
    return nullptr;
  }
  const std::int64_t limit = (std::int64_t(1) << 20) - 1;
  std::int64_t index[3];
  for(int k = 0; k < 3; k++) {
    const double i = std::floor(p.at[k] / cell + 0.5);
    if(!(std::fabs(i) < static_cast<double>(limit))) {
      return nullptr;
    }
    index[k] = static_cast<std::int64_t>(i);
  }
  const auto found = nodes.find(Key(index[0], index[1], index[2]));
  return found == nodes.end() ? nullptr : &found->second;
}

std::vector<shift::core::collision_geometry::CadFaceField>
shift::core::collision_geometry::MakeCadFaceFields(
  const shift::core::collision_geometry::CadContactShape& shape, double max_depth) {
  std::vector<shift::core::collision_geometry::CadFaceField> fields(shape.faces.size());
  for(size_t face = 0; face < shape.faces.size(); face++) {
    const shift::core::collision_geometry::CadContactFace& f = shape.faces[face];
    shift::core::collision_geometry::CadFaceField& field = fields[face];
    if(f.sample_points.empty()) {
      continue;
    }
    const ksl_vec3_t diagonal = {{f.box.hi.x - f.box.lo.x, f.box.hi.y - f.box.lo.y,
                                  f.box.hi.z - f.box.lo.z}};
    const double spacing =
      ksl_vec3_l2norm(&diagonal) / std::max({f.sample_nu, f.sample_nv, 1});
    if(!(spacing > 0.0)) {
      continue;
    }
    field.band = std::max(3.0 * spacing, max_depth);
    /* a cell of the samples' spacing, or coarser to keep the band at most
    four cells deep: each node reached from a sample is projected on the face
    once, from that sample */
    field.cell = std::max(spacing, 0.25 * field.band);
    /* each node within the band and a cell of a sample, a sample being a
    spacing from the next: the nodes within band of the face */
    const int reach = static_cast<int>(std::ceil((field.band + spacing) / field.cell));
    std::unordered_map<std::int64_t, bool> visited;
    for(size_t s = 0; s < f.sample_points.size(); s++) {
      const ksl_vec3_t& p = f.sample_points[s];
      std::int64_t center[3];
      for(int k = 0; k < 3; k++) {
        center[k] = static_cast<std::int64_t>(std::floor(p.at[k] / field.cell + 0.5));
      }
      for(int i = -reach; i <= reach; i++) {
        for(int j = -reach; j <= reach; j++) {
          for(int k = -reach; k <= reach; k++) {
            const std::int64_t key = shift::core::collision_geometry::CadFaceField::Key(
              center[0] + i, center[1] + j, center[2] + k);
            if(!visited.emplace(key, true).second) {
              continue;
            }
            const ksl_vec3_t node = {{static_cast<double>(center[0] + i) * field.cell,
                                      static_cast<double>(center[1] + j) * field.cell,
                                      static_cast<double>(center[2] + k) * field.cell}};
            shift::core::cad::Vec2 uv;
            if(!shape.Project(static_cast<int>(face), node, uv, &f.sample_uv[s]) ||
               !shape.Inside(static_cast<int>(face), uv)) {
              continue;
            }
            const ksl_vec3_t q = shape.Point(static_cast<int>(face), uv);
            const ksl_vec3_t n = shape.Normal(static_cast<int>(face), uv);
            const ksl_vec3_t d = {{node.x - q.x, node.y - q.y, node.z - q.z}};
            const double length = ksl_vec3_l2norm(&d);
            if(!(length <= field.band)) {
              continue;
            }
            shift::core::collision_geometry::CadFaceField::Node out;
            out.distance = d.x * n.x + d.y * n.y + d.z * n.z;
            out.uv = uv;
            field.nodes.emplace(key, out);
          }
        }
      }
    }
  }
  return fields;
}

std::shared_ptr<const shift::core::collision_geometry::CadContactShape>
shift::core::collision_geometry::MakeCadContactShape(
  std::shared_ptr<const geometry::CadAsset> asset, double scale) {
  auto shape = std::make_shared<shift::core::collision_geometry::CadContactShape>();
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
        shift::core::collision_geometry::CadContactFace f;
        f.face = fi;
        f.placement = static_cast<int>(pi);
        const auto domain = cad::SurfaceDomain(m, m.surfaces[face.surface]);
        f.periodic_u = domain[0].periodic;
        f.periodic_v = domain[1].periodic;
        f.lo = {HUGE_VAL, HUGE_VAL};
        f.hi = {-HUGE_VAL, -HUGE_VAL};
        for(const auto& loop : face.trims) {
          f.trims.push_back(cad::TrimPolygon(m, loop, kTrimSegments));
          for(const shift::core::cad::Vec2& p : f.trims.back()) {
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
            } else {
              /* twice around the same face: its seam */
              shape->edges[it->second].smooth = true;
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
    /* as many of them on each quarter of a turn of a circle: a short stretch
    of a long rim may be all that is inside a face */
    int intervals = kEdgeSamples + 1;
    if(m.curves[e.curve].type == cad::Curve3::Type::kConic) {
      const double quarters = std::fabs(e.range[1] - e.range[0]) / (0.5 * M_PI);
      intervals *= std::min(std::max(static_cast<int>(std::ceil(quarters - 1e-9)),
                                     1), 4);
    }
    for(int i = 1; i < intervals; i++) {
      const double t = e.range[0] + (e.range[1] - e.range[0]) * i / intervals;
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
    shift::core::collision_geometry::CadContactFace& f = shape->faces[fi];
    const auto& surface = m.surfaces[m.faces[f.face].surface];
    f.sample_nu = FaceSamples(surface, 0, f.lo[0], f.hi[0]);
    f.sample_nv = FaceSamples(surface, 1, f.lo[1], f.hi[1]);
    f.sample_cell.assign(f.sample_nu * f.sample_nv, -1);
    for(int i = 0; i < f.sample_nu; i++) {
      for(int j = 0; j < f.sample_nv; j++) {
        const shift::core::cad::Vec2 uv{
          f.lo[0] + (f.hi[0] - f.lo[0]) * (i + 0.5) / f.sample_nu,
          f.lo[1] + (f.hi[1] - f.lo[1]) * (j + 0.5) / f.sample_nv};
        if(InsideTrims(f, uv)) {
          f.sample_cell[i * f.sample_nv + j] =
            static_cast<int>(f.sample_uv.size());
          f.sample_uv.push_back(uv);
        }
      }
    }
    for(const auto& poly : f.trims) {
      for(const shift::core::cad::Vec2& uv : poly) {
        f.boundary_points.push_back(shape->Point(fi, uv));
        f.box.Add(f.boundary_points.back());
      }
    }
    for(const shift::core::cad::Vec2& uv : f.sample_uv) {
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

  MarkSharpEdges(*shape);

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
/* the exact shapes of other collision geometry */

static shift::core::cad::Vec3 ToVec3(const ksl_vec3_t& p) {
  return {p.x, p.y, p.z};
}

/* an asset of the one body of the faces of a model, repaired */
static std::shared_ptr<const shift::core::geometry::CadAsset>
FinishAsset(std::shared_ptr<shift::core::geometry::CadAsset> asset) {
  shift::core::cad::Model& m = asset->model;
  shift::core::cad::Shell shell;
  shell.closed = true;
  for(int f = 0; f < static_cast<int>(m.faces.size()); f++) {
    shell.faces.push_back(f);
  }
  m.shells.push_back(shell);
  shift::core::cad::Body body;
  body.shells.push_back(0);
  body.solid = true;
  m.bodies.push_back(body);
  asset->report = shift::core::cad::Repair(m);
  shift::core::cad::PlacedBody placed;
  placed.body = 0;
  asset->bodies.push_back(placed);
  return asset;
}

/* a line edge of the model between two vertices */
static int AddLineEdge(shift::core::cad::Model& m, int start, int end) {
  shift::core::cad::Curve3 line;
  line.origin = m.vertices[start].point;
  for(int k = 0; k < 3; k++) {
    line.direction[k] = m.vertices[end].point[k] - line.origin[k];
  }
  m.curves.push_back(line);
  shift::core::cad::Edge e;
  e.start = start;
  e.end = end;
  e.curve = static_cast<int>(m.curves.size()) - 1;
  m.edges.push_back(e);
  return static_cast<int>(m.edges.size()) - 1;
}

/* the hull of points that lie in a plane or along a line, as a hull with faces:
thickened by 1e-4 of their extent in the directions they have none; false for a
single point */
static bool Thickened(const shift::core::collision_geometry::ConvexHullData& hull,
                      shift::core::collision_geometry::ConvexHullData& thick) {
  if(hull.vertices.empty()) {
    return false;
  }
  ksl_vec3_t lo = hull.vertices[0], hi = hull.vertices[0];
  for(const auto& v : hull.vertices) {
    for(int k = 0; k < 3; k++) {
      lo.at[k] = std::min(lo.at[k], v.at[k]);
      hi.at[k] = std::max(hi.at[k], v.at[k]);
    }
  }
  ksl_vec3_t diagonal;
  ksl_subtract_vv(&hi, &lo, &diagonal);
  const double extent = ksl_vec3_l2norm(&diagonal);
  if(!(extent > 0.0)) {
    return false;
  }
  ksl_mat3x3_t covariance = shift::core::collision_geometry::Covariance(hull.vertices), axes;
  shift::core::collision_geometry::Jacobi(covariance, axes);
  std::vector<ksl_vec3_t> points = hull.vertices;
  for(int k = 0; k < 3; k++) {
    if(std::sqrt(std::max(covariance.at[k][k], 0.0)) > 1e-6 * extent) {
      continue;
    }
    std::vector<ksl_vec3_t> both;
    for(const auto& p : points) {
      for(double side : {-1e-4, 1e-4}) {
        ksl_vec3_t q = p;
        for(int i = 0; i < 3; i++) {
          q.at[i] += side * extent * axes.v[k].at[i];
        }
        both.push_back(q);
      }
    }
    points = both;
  }
  if(!shift::core::collision_geometry::ComputeConvexHull(points, thick)) {
    return false;
  }
  shift::core::collision_geometry::ComputeFacePlanes(thick);
  shift::core::collision_geometry::ComputeFacePolygons(thick);
  return !thick.polygons.empty();
}

std::shared_ptr<const shift::core::collision_geometry::CadContactShape>
shift::core::collision_geometry::MakeHullShape(const shift::core::collision_geometry::ConvexHullData& hull) {
  if(hull.polygons.empty()) {
    shift::core::collision_geometry::ConvexHullData thick;
    return Thickened(hull, thick) ? MakeHullShape(thick) : nullptr;
  }
  auto asset = std::make_shared<geometry::CadAsset>();
  cad::Model& m = asset->model;
  for(const auto& v : hull.vertices) {
    cad::Vertex vertex;
    vertex.point = ToVec3(v);
    m.vertices.push_back(vertex);
  }
  std::map<std::pair<int, int>, int> edges;
  for(size_t i = 0; i < hull.polygons.size(); i++) {
    const auto& polygon = hull.polygons[i];
    const ksl_vec3_t& normal = hull.polygon_normals[i];
    if(polygon.size() < 3 || ksl_dot_vv(&normal, &normal) == 0.0) {
      continue;
    }
    /* the plane: x along the first side, z out of the hull */
    ksl_vec3_t x, y;
    ksl_subtract_vv(&hull.vertices[polygon[1]], &hull.vertices[polygon[0]], &x);
    ksl_axpy_vv(-ksl_dot_vv(&x, &normal), &normal, &x);
    ksl_vec3_scale(&x, 1.0 / ksl_vec3_l2norm(&x));
    ksl_cross_vv(&normal, &x, &y);
    cad::Surface plane;
    plane.type = cad::Surface::Type::kPlane;
    plane.position.origin = ToVec3(hull.vertices[polygon[0]]);
    plane.position.x = ToVec3(x);
    plane.position.y = ToVec3(y);
    plane.position.z = ToVec3(normal);
    m.surfaces.push_back(plane);

    cad::Loop loop;
    loop.outer = true;
    for(size_t k = 0; k < polygon.size(); k++) {
      const int from = static_cast<int>(polygon[k]);
      const int to = static_cast<int>(polygon[(k + 1) % polygon.size()]);
      const auto key = std::minmax(from, to);
      auto found = edges.find({key.first, key.second});
      if(found == edges.end()) {
        found = edges
                  .insert({{key.first, key.second},
                           AddLineEdge(m, key.first, key.second)})
                  .first;
      }
      cad::Coedge coedge;
      coedge.edge = found->second;
      coedge.orientation = from == key.first;
      loop.coedges.push_back(coedge);
    }
    cad::Face face;
    face.surface = static_cast<int>(m.surfaces.size()) - 1;
    face.loops.push_back(loop);
    m.faces.push_back(face);
  }
  return MakeCadContactShape(FinishAsset(asset), 1.0);
}

std::shared_ptr<const shift::core::collision_geometry::CadContactShape>
shift::core::collision_geometry::MakeCylinderShape(double radius,
                                                   double half_length,
                                                   bool caps) {
  constexpr int kArcs = 4;
  auto asset = std::make_shared<geometry::CadAsset>();
  cad::Model& m = asset->model;

  /* the rims, bottom (0) and top (1): vertices, and arcs between them */
  int vertices[2][kArcs], arcs[2][kArcs];
  for(int end = 0; end < 2; end++) {
    const double z = end ? half_length : -half_length;
    for(int k = 0; k < kArcs; k++) {
      const double angle = 2.0 * M_PI * k / kArcs;
      cad::Vertex v;
      v.point = {radius * std::cos(angle), radius * std::sin(angle), z};
      m.vertices.push_back(v);
      vertices[end][k] = static_cast<int>(m.vertices.size()) - 1;
    }
    cad::Curve3 circle;
    circle.type = cad::Curve3::Type::kConic;
    circle.origin = {0.0, 0.0, z};
    circle.a = {radius, 0.0, 0.0};
    circle.b = {0.0, radius, 0.0};
    m.curves.push_back(circle);
    for(int k = 0; k < kArcs; k++) {
      cad::Edge e;
      e.start = vertices[end][k];
      e.end = vertices[end][(k + 1) % kArcs];
      e.curve = static_cast<int>(m.curves.size()) - 1;
      m.edges.push_back(e);
      arcs[end][k] = static_cast<int>(m.edges.size()) - 1;
    }
  }
  const int seam = AddLineEdge(m, vertices[0][0], vertices[1][0]);

  /* the side: round the bottom, up the seam, back round the top and down */
  cad::Surface side;
  side.type = cad::Surface::Type::kCylinder;
  side.radius = radius;
  side.position.origin = {0.0, 0.0, -half_length};
  m.surfaces.push_back(side);
  cad::Loop around;
  around.outer = true;
  auto add = [](cad::Loop& loop, int edge, bool orientation) {
    cad::Coedge coedge;
    coedge.edge = edge;
    coedge.orientation = orientation;
    loop.coedges.push_back(coedge);
  };
  for(int k = 0; k < kArcs; k++) {
    add(around, arcs[0][k], true);
  }
  add(around, seam, true);
  for(int k = kArcs - 1; k >= 0; k--) {
    add(around, arcs[1][k], false);
  }
  add(around, seam, false);
  cad::Face face;
  face.surface = 0;
  face.loops.push_back(around);
  m.faces.push_back(face);

  if(caps) {
    for(int end = 0; end < 2; end++) {
      /* z out of the cylinder */
      cad::Surface plane;
      plane.type = cad::Surface::Type::kPlane;
      plane.position.origin = {0.0, 0.0, end ? half_length : -half_length};
      plane.position.y = {0.0, end ? 1.0 : -1.0, 0.0};
      plane.position.z = {0.0, 0.0, end ? 1.0 : -1.0};
      m.surfaces.push_back(plane);
      cad::Loop rim;
      rim.outer = true;
      for(int k = 0; k < kArcs; k++) {
        if(end) {
          add(rim, arcs[1][k], true);
        } else {
          add(rim, arcs[0][kArcs - 1 - k], false);
        }
      }
      cad::Face cap;
      cap.surface = static_cast<int>(m.surfaces.size()) - 1;
      cap.loops.push_back(rim);
      m.faces.push_back(cap);
    }
  }
  auto shape = MakeCadContactShape(FinishAsset(asset), 1.0);
  if(caps) {
    return shape;
  }
  auto bare = std::make_shared<shift::core::collision_geometry::CadContactShape>(*shape);
  bare->edges.clear();
  bare->vertices.clear();
  for(auto& f : bare->faces) {
    f.edges.clear();
    f.vertices.clear();
  }
  return bare;
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
  if(contact_seeds_ != "samples" && contact_seeds_ != "field") {
    throw shift::core::Exception(description +
                                 ": contact_seeds is \"samples\" or \"field\", "
                                 "not \"" + contact_seeds_ + "\"");
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
  if(contact_seeds_ == "field" && !fields_) {
    fields_ = std::make_shared<const std::vector<CadFaceField>>(
      MakeCadFaceFields(*shape_, max_depth_));
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
  /* the fields reach the prototype's max_depth: shared only if it is the same */
  if(contact_seeds_ == "field" && cad->fields_ && max_depth_ == cad->max_depth_) {
    fields_ = cad->fields_;
  }
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
