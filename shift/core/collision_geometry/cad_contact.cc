/* contact of CAD geometry on the exact surfaces of its faces, see
collision_geometry::Cad */
#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>

#include "ksl/linalg.h"
#include "ksl/matrix.h"
#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/core/cad/evaluate.h"
#include "shift/core/collision_geometry/analytic_contact.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"


namespace shift {
namespace core {
namespace collision_geometry {

/* a CAD geometry in the frame of the pair's geometry a: d transforms
coordinates in the geometry's frame to those in a's */
struct CadSideOfPair {
  const shift::core::collision_geometry::CadContactShape* shape = nullptr;
  double edge_radius = 0.0; //!< of its edges and vertices, see Cad
  /*! the closest-point fields of its faces, with contact_seeds "field" */
  const std::vector<shift::core::collision_geometry::CadFaceField>* fields = nullptr;
  ksl_SE3_t d = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
};

} // namespace collision_geometry
} // namespace core
} // namespace shift



static ksl_vec3_t Sub(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  ksl_vec3_t c;
  ksl_subtract_vv(&a, &b, &c);
  return c;
}

/* a + s b */
static ksl_vec3_t Axpy(const ksl_vec3_t& a, double s, const ksl_vec3_t& b) {
  ksl_vec3_t c = a;
  ksl_axpy_vv(s, &b, &c);
  return c;
}

static ksl_vec3_t Scaled(const ksl_vec3_t& a, double s) {
  ksl_vec3_t c;
  ksl_vec3_scaled(&a, s, &c);
  return c;
}

static double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return ksl_dot_vv(&a, &b);
}

/* the points found beyond the reach of a test of two shapes, in a's frame:
ShapeOverlap searches again, deeper, when one of them is a point of contact
(not far beyond the reach, and deepest in the other shape where it is). A
thread each, as pairs are tested in parallel */
static thread_local std::vector<shift::core::collision_geometry::ContactPoint> beyond_reach;

/* whether the point of a and the point of b, depth apart, are within the reach
of the test, max_depth; a point beyond it is kept in beyond_reach */
static bool Within(const ksl_vec3_t& point_a, const ksl_vec3_t& point_b,
                   double depth, double max_depth) {
  if(depth <= max_depth) {
    return true;
  }
  if(depth < 4.0 * max_depth && beyond_reach.size() < 64) {
    shift::core::collision_geometry::ContactPoint c;
    c.depth = depth;
    c.point_a = point_a;
    c.point_b = point_b;
    c.normal = Scaled(Sub(point_a, point_b), 1.0 / depth);
    beyond_reach.push_back(c);
  }
  return false;
}

static bool Within(const shift::core::collision_geometry::ContactPoint& c,
                   double max_depth) {
  return Within(c.point_a, c.point_b, c.depth, max_depth);
}

/* ------------------------------------------------------------------------ */
/* the faces of a side, in a's frame */

static ksl_vec3_t SidePoint(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                            const shift::core::cad::Vec2& uv, ksl_vec3_t* du = nullptr,
                            ksl_vec3_t* dv = nullptr) {
  ksl_vec3_t own_du, own_dv, p;
  const ksl_vec3_t own =
    side.shape->Point(face, uv, du ? &own_du : nullptr, dv ? &own_dv : nullptr);
  if(du) {
    ksl_product_drv(&side.d.R, &own_du, du);
  }
  if(dv) {
    ksl_product_drv(&side.d.R, &own_dv, dv);
  }
  ksl_product_dv(&side.d, &own, &p);
  return p;
}

static ksl_vec3_t SideNormal(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                             const shift::core::cad::Vec2& uv) {
  const ksl_vec3_t own = side.shape->Normal(face, uv);
  ksl_vec3_t n;
  ksl_product_drv(&side.d.R, &own, &n);
  return n;
}

static ksl_vec3_t SideVector(const shift::core::collision_geometry::CadSideOfPair& side, const ksl_vec3_t& v) {
  ksl_vec3_t out;
  ksl_product_drv(&side.d.R, &v, &out);
  return out;
}

static ksl_vec3_t SidePosition(const shift::core::collision_geometry::CadSideOfPair& side,
                               const ksl_vec3_t& p) {
  ksl_vec3_t out;
  ksl_product_dv(&side.d, &p, &out);
  return out;
}

static bool SideProject(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                        const ksl_vec3_t& p, shift::core::cad::Vec2& uv,
                        const shift::core::cad::Vec2* guess = nullptr) {
  ksl_vec3_t own;
  ksl_product_dinvv(&side.d, &p, &own);
  return side.shape->Project(face, own, uv, guess);
}

static const shift::core::cad::Surface& SurfaceOf(const shift::core::collision_geometry::CadSideOfPair& side,
                                                  int face) {
  const auto& m = side.shape->model();
  return m.surfaces[m.faces[side.shape->faces[face].face].surface];
}

static bool SameSense(const shift::core::collision_geometry::CadSideOfPair& side, int face) {
  return side.shape->model().faces[side.shape->faces[face].face].same_sense;
}

/* the frame of a face's surface in a's frame: its origin and the unit z
axis (the axis of a cylinder, the normal of a plane), and the scale of its
lengths */
static void SurfaceAxis(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                        ksl_vec3_t& origin, ksl_vec3_t& axis, double& scale) {
  const auto& s = SurfaceOf(side, face);
  const auto& place =
    side.shape->placements[side.shape->faces[face].placement];
  origin = SidePosition(side, place.Apply(s.position.origin));
  axis = SideVector(side, place.ApplyVector(s.position.z));
  ksl_vec3_normalize(&axis);
  scale = place.scale;
}

/* ------------------------------------------------------------------------ */
/* lines of contact */

/* how far a line along the seam of two faces is taken inside them, as a share
of a face's span: exactly on a seam, the tests of a face's trims can go either
way */
static constexpr double kSeam = 1e-7;

/* whether (u, v) is inside a face or on its edge, within kSeam */
static bool InsideOrOnEdge(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                           const shift::core::cad::Vec2& uv) {
  const auto& f = side.shape->faces[face];
  const double du = kSeam * (f.hi[0] - f.lo[0]);
  const double dv = kSeam * (f.hi[1] - f.lo[1]);
  for(const shift::core::cad::Vec2& at : {uv, shift::core::cad::Vec2{uv[0] + du, uv[1]}, shift::core::cad::Vec2{uv[0] - du, uv[1]},
                        shift::core::cad::Vec2{uv[0], uv[1] + dv}, shift::core::cad::Vec2{uv[0], uv[1] - dv}}) {
    if(side.shape->Inside(face, at)) {
      return true;
    }
  }
  return false;
}

/* removes the points from before on that repeat an earlier one: at the seam of
two faces meeting smoothly, the points of both are the same and only the first
face's count; true if there were any */
static bool DropRepeated(std::vector<shift::core::collision_geometry::ContactPoint>& points, size_t before,
                         double tolerance) {
  auto repeated = [&](const shift::core::collision_geometry::ContactPoint& fresh) {
    for(size_t i = 0; i < before; i++) {
      const ksl_vec3_t da = Sub(fresh.point_a, points[i].point_a);
      const ksl_vec3_t db = Sub(fresh.point_b, points[i].point_b);
      if(ksl_vec3_l2norm(&da) <= tolerance &&
         ksl_vec3_l2norm(&db) <= tolerance) {
        return true;
      }
    }
    return false;
  };
  const auto kept =
    std::remove_if(points.begin() + before, points.end(), repeated);
  const bool any = kept != points.end();
  points.erase(kept, points.end());
  return any;
}

/* the pieces [s0, s1] common to two lists of pieces */
static std::vector<std::array<double, 2>>
Intersect(const std::vector<std::array<double, 2>>& a,
          const std::vector<std::array<double, 2>>& b) {
  std::vector<std::array<double, 2>> out;
  for(const auto& x : a) {
    for(const auto& y : b) {
      const double lo = std::max(x[0], y[0]), hi = std::min(x[1], y[1]);
      if(hi > lo) {
        out.push_back({lo, hi});
      }
    }
  }
  return out;
}

/* the pieces of the line p + s direction, s in [s0, s1], that lie inside a
face (projected onto its surface), in s; it may run along the face's seam */
static std::vector<std::array<double, 2>>
ClipLine(const shift::core::collision_geometry::CadSideOfPair& side, int face, const ksl_vec3_t& p,
         const ksl_vec3_t& direction, double s0, double s1) {
  shift::core::cad::Vec2 uv0, uv1;
  if(!SideProject(side, face, Axpy(p, s0, direction), uv0) ||
     !SideProject(side, face, Axpy(p, s1, direction), uv1, &uv0)) {
    return {};
  }
  auto pieces = side.shape->Clip(face, uv0, uv1, kSeam);
  for(auto& piece : pieces) {
    piece = {s0 + piece[0] * (s1 - s0), s0 + piece[1] * (s1 - s0)};
  }
  return pieces;
}

/* 0 below 0, 1 above 1, a cubic with zero slope at both ends between */
static double SmoothStep(double x) {
  x = std::min(std::max(x, 0.0), 1.0);
  return x * x * (3.0 - 2.0 * x);
}

/* the cross section of a line of contact at the station s along the line,
of the faces' surfaces extended beyond their trims: penetration depth (or
-HUGE_VAL if there is no direction of contact), the points of a and b and
the normal from a to b */
typedef double (*Section)(const void* data, double s, ksl_vec3_t& pa,
                          ksl_vec3_t& pb, ksl_vec3_t& n);

static double DepthAt(Section section, const void* data, double s) {
  ksl_vec3_t pa, pb, n;
  return section(data, s, pa, pb, n);
}

/* where the surfaces overlap along a line of contact, s in [s0, s1]: the
deepest station, and the part [lo, hi] of [s0, s1] with positive depth; the
depth is concave along the line (straight for a cylinder on a plane, a
parabola near its top for two cylinders; straight, from the ends, if
straight). reach is the length of the
overlap of the surfaces extended beyond [s0, s1], up to 8 times its length
on each side (HUGE_VAL past that, e.g. for parallel cylinders) */
struct LineOverlap {
  double deepest = 0.0, depth = -HUGE_VAL;
  double lo = 0.0, hi = 0.0;
  double reach = 0.0;
};

static bool Overlap(Section section, const void* data, double s0, double s1,
                    bool straight, LineOverlap& o) {
  if(straight) {
    /* the depth is straight along the line: from its ends */
    const double d0 = DepthAt(section, data, s0);
    const double d1 = DepthAt(section, data, s1);
    o.deepest = d1 > d0 ? s1 : s0;
    o.depth = std::max(d0, d1);
    if(!(o.depth > 0.0)) {
      return false;
    }
    /* where the depth is 0, if one end is above the plane: the depths of
    the ends then differ */
    auto cross = [&]() { return s0 + d0 / (d0 - d1) * (s1 - s0); };
    o.lo = d0 > 0.0 ? s0 : cross();
    o.hi = d1 > 0.0 ? s1 : cross();
    o.reach = HUGE_VAL;
    return o.hi > o.lo;
  }
  /* the deepest station: golden section search */
  const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
  double lo = s0, hi = s1;
  double t1 = hi - golden * (hi - lo), t2 = lo + golden * (hi - lo);
  double d1 = DepthAt(section, data, t1), d2 = DepthAt(section, data, t2);
  for(int i = 0; i < 32; i++) {
    if(d1 > d2) {
      hi = t2;
      t2 = t1;
      d2 = d1;
      t1 = hi - golden * (hi - lo);
      d1 = DepthAt(section, data, t1);
    } else {
      lo = t1;
      t1 = t2;
      d1 = d2;
      t2 = lo + golden * (hi - lo);
      d2 = DepthAt(section, data, t2);
    }
  }
  o.deepest = 0.5 * (lo + hi);
  o.depth = DepthAt(section, data, o.deepest);
  for(double end : {s0, s1}) {
    const double d = DepthAt(section, data, end);
    if(d > o.depth) {
      o.deepest = end;
      o.depth = d;
    }
  }
  if(!(o.depth > 0.0)) {
    return false;
  }
  /* the station between inside (positive depth) and outside, by bisection */
  auto zero = [&](double inside, double outside) {
    for(int i = 0; i < 34; i++) {
      const double mid = 0.5 * (inside + outside);
      (DepthAt(section, data, mid) > 0.0 ? inside : outside) = mid;
    }
    return 0.5 * (inside + outside);
  };
  const double length = s1 - s0;
  double left, right;
  /* each end of the overlap: within [s0, s1], and beyond it for reach */
  auto end = [&](double edge, double direction, double& clipped,
                 double& extended) {
    if(!(DepthAt(section, data, edge) > 0.0)) {
      clipped = extended = zero(o.deepest, edge);
      return;
    }
    clipped = edge;
    for(double far = length; far <= 8.0 * length; far *= 2.0) {
      const double outside = edge + direction * far;
      if(!(DepthAt(section, data, outside) > 0.0)) {
        extended = zero(edge, outside);
        return;
      }
    }
    extended = direction * HUGE_VAL;
  };
  end(s0, -1.0, o.lo, left);
  end(s1, 1.0, o.hi, right);
  o.reach = right - left;
  return o.hi > o.lo;
}

/* Gauss-Legendre stations and weights of a slice of a line of contact,
exact for a depth varying as a cubic along the slice */
static const double kStations[3] = {-0.7745966692414834, 0.0,
                                    0.7745966692414834};
static const double kStationWeights[3] = {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0};

/* the points of a line of contact along direction, over the overlap
[lo, hi]: clipped by the faces' trims along the line through the deepest
station's points (a face of a primitive plane, nullptr, is not clipped),
then a point at each station of each piece, carrying its share of the
piece's length, with weight; limit, if given, are the pieces to start from
instead of [lo, hi] */
static void LinePoints(const shift::core::collision_geometry::CadSideOfPair* a, int fa, const shift::core::collision_geometry::CadSideOfPair* b,
                       int fb, const ksl_vec3_t& direction,
                       const LineOverlap& o, Section section,
                       const void* data, double max_depth, double weight,
                       std::vector<shift::core::collision_geometry::ContactPoint>& points,
                       const std::vector<std::array<double, 2>>* limit =
                         nullptr) {
  ksl_vec3_t pa, pb, n;
  section(data, o.deepest, pa, pb, n);
  std::vector<std::array<double, 2>> pieces = {{o.lo, o.hi}};
  if(limit) {
    pieces = *limit;
  }
  if(a) {
    pieces = Intersect(pieces,
                       ClipLine(*a, fa, Axpy(pa, -o.deepest, direction),
                                direction, o.lo, o.hi));
  }
  if(b) {
    pieces = Intersect(pieces,
                       ClipLine(*b, fb, Axpy(pb, -o.deepest, direction),
                                direction, o.lo, o.hi));
  }
  for(const auto& piece : pieces) {
    const double half = 0.5 * (piece[1] - piece[0]);
    const double center = 0.5 * (piece[0] + piece[1]);
    for(int i = 0; i < 3; i++) {
      shift::core::collision_geometry::ContactPoint c;
      c.depth = section(data, center + kStations[i] * half, c.point_a,
                        c.point_b, c.normal);
      if(c.depth > 0.0 && Within(c, max_depth)) {
        c.length = kStationWeights[i] * half;
        c.weight = weight;
        points.push_back(c);
      }
    }
  }
}

/* unit vectors u and v perpendicular to the unit vector d, and to each
other */
static void Perpendiculars(const ksl_vec3_t& d, ksl_vec3_t& u,
                           ksl_vec3_t& v) {
  const ksl_vec3_t x = {{1.0, 0.0, 0.0}}, y = {{0.0, 1.0, 0.0}};
  const ksl_vec3_t& seed = std::fabs(d.x) < 0.6 ? x : y;
  u = Axpy(seed, -Dot(seed, d), d);
  ksl_vec3_normalize(&u);
  ksl_cross_vv(&d, &u, &v);
}

/* two cylinders, in a's frame: a convex one (inner) whose axis the line of
contact follows, and the other, convex or a hole the inner one lies in */
struct CylinderPair {
  ksl_vec3_t oi, di, ui, vi; //!< the inner one's axis, and perpendiculars
  double ri;
  ksl_vec3_t oo, dO; //!< the other's axis
  double ro;
  bool other_convex;
  bool inner_is_a;
  /* the angle around the inner circle of the last section, where the next
  one, at a station nearby, starts */
  mutable double phi = 0.0;
  mutable bool has_phi = false;
};

/* the cross section of two cylinders at the station s along the inner
one's axis: the point of the inner one's circle there deepest in the other
cylinder (the extremum of its distance from the other's axis, by Newton's
method on the angle around the circle), for axes at any angle */
static double CylinderSection(const void* data, double s, ksl_vec3_t& pa,
                              ksl_vec3_t& pb, ksl_vec3_t& n) {
  const CylinderPair& c = *static_cast<const CylinderPair*>(data);
  const ksl_vec3_t ci = Axpy(c.oi, s, c.di);
  auto perpendicular = [&](const ksl_vec3_t& x) {
    return Axpy(x, -Dot(x, c.dO), c.dO); // to the other's axis
  };
  const ksl_vec3_t w0 = perpendicular(Sub(ci, c.oo));
  const ksl_vec3_t up = perpendicular(c.ui), vp = perpendicular(c.vi);
  /* towards the other's axis, in the plane of the circle */
  const ksl_vec3_t e = Axpy(w0, -Dot(w0, c.di), c.di);
  const double scale = c.ri + c.ro;
  if(!(ksl_vec3_l2norm(&e) > 1e-12 * scale)) {
    return -HUGE_VAL; // coaxial: no direction of contact
  }
  double phi = std::atan2(-Dot(e, c.vi), -Dot(e, c.ui));
  if(!c.other_convex) {
    phi += M_PI; // against the wall of the hole, away from its axis
  }
  if(c.has_phi && std::cos(phi - c.phi) > 0.5) {
    phi = c.phi;
  }
  /* q(phi) = |w0 + ri r(phi)|^2, the squared distance from the other's
  axis of the circle's point at phi, r = cos(phi) up + sin(phi) vp: least
  in a convex cylinder, greatest in a hole */
  const double sense = c.other_convex ? 1.0 : -1.0;
  for(int i = 0; i < 12; i++) {
    const double cs = std::cos(phi), sn = std::sin(phi);
    const ksl_vec3_t r = Axpy(Scaled(up, cs), sn, vp);
    const ksl_vec3_t dr = Axpy(Scaled(up, -sn), cs, vp);
    const ksl_vec3_t w = Axpy(w0, c.ri, r);
    const double slope = 2.0 * c.ri * Dot(w, dr);
    const double curve = 2.0 * c.ri * (c.ri * Dot(dr, dr) - Dot(w, r));
    double step;
    if(sense * curve > 0.0) {
      step = -slope / curve;
    } else {
      step = -sense * (slope > 0.0 ? 0.1 : -0.1); // downhill
    }
    step = std::min(std::max(step, -0.5), 0.5);
    phi += step;
    if(std::fabs(step) < 1e-11) {
      break; // quadratic: the next step would be below 1e-22
    }
  }
  c.phi = phi;
  c.has_phi = true;
  const ksl_vec3_t p =
    Axpy(Axpy(ci, c.ri * std::cos(phi), c.ui), c.ri * std::sin(phi), c.vi);
  const ksl_vec3_t w = Sub(p, c.oo);
  const double axial = Dot(w, c.dO);
  ksl_vec3_t radial = Axpy(w, -axial, c.dO);
  const double distance = ksl_vec3_l2norm(&radial);
  if(!(distance > 1e-12 * scale)) {
    return -HUGE_VAL;
  }
  ksl_vec3_scale(&radial, 1.0 / distance);
  const double depth =
    c.other_convex ? c.ro - distance : distance - c.ro;
  const ksl_vec3_t q = Axpy(Axpy(c.oo, axial, c.dO), c.ro, radial);
  /* the other's outward normal: out of its solid, which surrounds a hole */
  const ksl_vec3_t outward = c.other_convex ? radial : Scaled(radial, -1.0);
  if(c.inner_is_a) {
    pa = p;
    pb = q;
    n = Scaled(outward, -1.0);
  } else {
    pa = q;
    pb = p;
    n = outward;
  }
  return depth;
}

/* the range [lo, hi] of a cylindrical face along an axis through origin */
static void AxialRange(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                       const ksl_vec3_t& axis_origin, const ksl_vec3_t& axis,
                       double& lo, double& hi) {
  ksl_vec3_t o, z;
  double scale;
  SurfaceAxis(side, face, o, z, scale);
  const auto& f = side.shape->faces[face];
  const double s0 = Dot(Sub(Axpy(o, scale * f.lo[1], z), axis_origin), axis);
  const double s1 = Dot(Sub(Axpy(o, scale * f.hi[1], z), axis_origin), axis);
  lo = std::min(s0, s1);
  hi = std::max(s0, s1);
}

static void GapCurvature(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::cad::Vec2& uva,
                         const shift::core::collision_geometry::CadSideOfPair* b, int fb, const shift::core::cad::Vec2& uvb,
                         const ksl_vec3_t& n, shift::core::collision_geometry::ContactPoint& c);

/*
Two cylinders at any angle: a line of contact along the convex one's axis
(the inner one, a's if both are convex), sliced over the length where the
surfaces overlap, so that cylinders tilting out of parallel load their line
towards one end. A line becomes a point as the cylinders cross: where the
overlap of the extended surfaces (its reach) is short beside the faces'
common length, the Hertz contact of an ellipse at the deepest point (of
the deepest station) fits inside the faces, and the contact is that point. Between a quarter and
three quarters of the common length, the line's weight goes from 0 to 1 and
the point's from 1 to 0, so the force has no jump.
*/
static void CylinderCylinder(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                             const shift::core::collision_geometry::CadSideOfPair& b, int fb, double max_depth,
                             std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const bool convex_a = SameSense(a, fa), convex_b = SameSense(b, fb);
  if(!convex_a && !convex_b) {
    return;
  }
  const bool inner_is_a = convex_a;
  const shift::core::collision_geometry::CadSideOfPair& inner = inner_is_a ? a : b;
  const shift::core::collision_geometry::CadSideOfPair& other = inner_is_a ? b : a;
  const int fi = inner_is_a ? fa : fb, fo = inner_is_a ? fb : fa;
  CylinderPair c;
  double scale_i, scale_o;
  SurfaceAxis(inner, fi, c.oi, c.di, scale_i);
  SurfaceAxis(other, fo, c.oo, c.dO, scale_o);
  Perpendiculars(c.di, c.ui, c.vi);
  c.ri = scale_i * SurfaceOf(inner, fi).radius;
  c.ro = scale_o * SurfaceOf(other, fo).radius;
  c.other_convex = SameSense(other, fo);
  c.inner_is_a = inner_is_a;
  /* the faces' common range along the inner axis: the other's ends,
  widened by its radius as it tilts */
  ksl_vec3_t cross;
  ksl_cross_vv(&c.di, &c.dO, &cross);
  const double widen = c.ro * ksl_vec3_l2norm(&cross);
  double i0, i1, o0, o1;
  AxialRange(inner, fi, c.oi, c.di, i0, i1);
  AxialRange(other, fo, c.oi, c.di, o0, o1);
  const double s0 = std::max(i0, o0 - widen), s1 = std::min(i1, o1 + widen);
  if(!(s1 > s0)) {
    return;
  }
  /* apart: the depth is at most ri + ro less the distance between the
  axes (two convex cylinders), or the distance of the inner axis from the
  other's, greatest at an end of the range as it is convex, plus ri less
  ro (in a hole) */
  auto axis_distance = [&](double s) {
    ksl_vec3_t w = Sub(Axpy(c.oi, s, c.di), c.oo);
    w = Axpy(w, -Dot(w, c.dO), c.dO);
    return ksl_vec3_l2norm(&w);
  };
  if(c.other_convex) {
    const double sine = ksl_vec3_l2norm(&cross);
    double apart;
    if(sine > 1e-9) {
      apart = std::fabs(Dot(Sub(c.oi, c.oo), cross)) / sine;
    } else {
      apart = axis_distance(s0); // parallel
    }
    if(c.ri + c.ro - apart <= 0.0) {
      return;
    }
    /* one inside the other over the whole range (a pin in a bushing's
    bore, inside the bushing's outside): the surfaces do not cross, and the
    hole's face carries the contact */
    if(std::max(axis_distance(s0), axis_distance(s1)) <=
       std::fabs(c.ri - c.ro)) {
      return;
    }
  } else if(std::max(axis_distance(s0), axis_distance(s1)) + c.ri - c.ro <=
            0.0) {
    return;
  }
  /* so near parallel that the depth bows by less than 1e-9 of the radii
  over the range, sin^2 L^2 / (8 r): straight, from the ends */
  const double sine = ksl_vec3_l2norm(&cross);
  const double bow = sine * sine * (s1 - s0) * (s1 - s0) /
                     (8.0 * std::min(c.ri, c.ro));
  const bool straight = bow < 1e-9 * (c.ri + c.ro);
  LineOverlap o;
  if(!Overlap(CylinderSection, &c, s0, s1, straight, o)) {
    return;
  }
  double line = SmoothStep((o.reach / (s1 - s0) - 0.25) / 0.5);
  if(line < 1.0) {
    /* the deepest point, of the deepest station */
    shift::core::collision_geometry::ContactPoint p;
    p.depth = CylinderSection(&c, o.deepest, p.point_a, p.point_b, p.normal);
    shift::core::cad::Vec2 uva, uvb;
    if(Within(p, max_depth) && SideProject(a, fa, p.point_a, uva) &&
       SideProject(b, fb, p.point_b, uvb) && a.shape->Inside(fa, uva) &&
       b.shape->Inside(fb, uvb)) {
      GapCurvature(a, fa, uva, &b, fb, uvb, p.normal, p);
      p.weight = 1.0 - line;
      points.push_back(p);
    } else {
      line = 1.0; // not inside both faces: their line
    }
  }
  if(line > 0.0) {
    LinePoints(&a, fa, &b, fb, c.di, o, CylinderSection, &c, max_depth, line,
               points);
  }
}

/* a convex cylinder on a plane, in a's frame */
struct PlaneCylinder {
  ksl_vec3_t op, np; //!< the plane's point and outward normal
  ksl_vec3_t oc, dc; //!< the cylinder's axis
  ksl_vec3_t down;   //!< towards the plane, perpendicular to the axis
  double rc;
  bool plane_is_a;
};

/* the cross section of a cylinder on a plane at the station s along its
axis: the point of its circle there deepest in the plane's solid */
static double PlaneCylinderSection(const void* data, double s, ksl_vec3_t& pa,
                                   ksl_vec3_t& pb, ksl_vec3_t& n) {
  const PlaneCylinder& c = *static_cast<const PlaneCylinder*>(data);
  const ksl_vec3_t pc = Axpy(Axpy(c.oc, s, c.dc), c.rc, c.down);
  const double depth = -Dot(Sub(pc, c.op), c.np);
  const ksl_vec3_t pp = Axpy(pc, depth, c.np);
  if(c.plane_is_a) {
    pa = pp;
    pb = pc;
    n = c.np;
  } else {
    pa = pc;
    pb = pp;
    n = Scaled(c.np, -1.0);
  }
  return depth;
}

/* a cylinder whose axis is within this angle (its sine) of the plane's
normal stands on its end: its rim touches the plane, as an edge */
static constexpr double kUpright = 0.05;

/*
A convex cylinder on a plane at any tilt, in a's frame: a line of contact
along the cylinder, sliced over the length below the plane, which tilts
towards one end. As the line shortens towards the cylinder's rim, the
contact becomes the rim's, an edge rounded to edge_radius: where the line
below the plane is shorter than a few times the rim's Hertz contact width,
sqrt(depth edge_radius), the line's weight goes to 0 and the rim point's to
1, so the force has no jump. plane is the plane's face, or nullptr for a
primitive plane (collision_geometry::Plane), which has no trims.

A steep cylinder whose deep end is off the plane's face, beyond its edge or in
a hole in it, has come round the edge, not through the face: below the plane
near the edge it is in the part, but near a side of it, and far from the
plane. The depth there says nothing, so its line fades out as the cylinder
steepens, and the edge or the side carries the contact.
*/
static void PlaneCylinderLine(const PlaneCylinder& base,
                              const shift::core::collision_geometry::CadSideOfPair* plane, int fp,
                              const shift::core::collision_geometry::CadSideOfPair& cylinder, int fc,
                              double max_depth,
                              std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  PlaneCylinder c = base;
  double scale;
  SurfaceAxis(cylinder, fc, c.oc, c.dc, scale);
  c.rc = scale * SurfaceOf(cylinder, fc).radius;
  c.down = Axpy(c.np, -Dot(c.np, c.dc), c.dc);
  const double tilt = ksl_vec3_l2norm(&c.down);
  if(tilt < kUpright) {
    return; // standing on its end: its rim, an edge, touches
  }
  ksl_vec3_scale(&c.down, -1.0 / tilt);
  double s0, s1;
  AxialRange(cylinder, fc, c.oc, c.dc, s0, s1);
  LineOverlap o;
  if(!Overlap(PlaneCylinderSection, &c, s0, s1, true, o)) {
    return;
  }
  /* the depth along the line, per length */
  const double slope = std::sqrt(std::max(1.0 - tilt * tilt, 0.0)) / tilt;
  double keep = 1.0;
  if(plane && slope > 0.5) {
    ksl_vec3_t pa, pb, n;
    PlaneCylinderSection(&c, o.deepest, pa, pb, n);
    shift::core::cad::Vec2 uv;
    if(!SideProject(*plane, fp, c.plane_is_a ? pa : pb, uv) ||
       !plane->shape->Inside(fp, uv)) {
      keep = 1.0 - SmoothStep((slope - 0.5) / 0.5);
      if(keep == 0.0) {
        return;
      }
    }
  }
  const double edge_radius = cylinder.edge_radius;
  const double width = std::sqrt(o.depth * edge_radius);
  double line = SmoothStep(((o.hi - o.lo) / width - 1.0) / 3.0);
  if(line < 1.0) {
    /* the rim at the deeper end */
    shift::core::collision_geometry::ContactPoint r;
    r.depth = PlaneCylinderSection(&c, o.deepest, r.point_a, r.point_b,
                                   r.normal);
    const ksl_vec3_t& on_plane = c.plane_is_a ? r.point_a : r.point_b;
    /* on this face of the cylinder (it may be one of several around it),
    tested just inside its end, which bounds the face, and inside the
    plane's face */
    const double inset =
      o.deepest + (o.deepest > 0.5 * (s0 + s1) ? -1e-6 : 1e-6) * (s1 - s0);
    ksl_vec3_t inset_a, inset_b, inset_n;
    PlaneCylinderSection(&c, inset, inset_a, inset_b, inset_n);
    const ksl_vec3_t& on_rim = c.plane_is_a ? inset_b : inset_a;
    shift::core::cad::Vec2 uv, uvc;
    const bool inside =
      SideProject(cylinder, fc, on_rim, uvc) &&
      InsideOrOnEdge(cylinder, fc, uvc) &&
      (!plane || (SideProject(*plane, fp, on_plane, uv) &&
                  plane->shape->Inside(fp, uv)));
    if(inside && Within(r, max_depth)) {
      r.curvature = 1.0 / edge_radius;
      r.weight = 1.0 - line;
      points.push_back(r);
    } else {
      line = 1.0;
    }
  }
  if(line > 0.0) {
    const shift::core::collision_geometry::CadSideOfPair* a = c.plane_is_a ? plane : &cylinder;
    const shift::core::collision_geometry::CadSideOfPair* b = c.plane_is_a ? &cylinder : plane;
    const int fa = c.plane_is_a ? fp : fc, fb = c.plane_is_a ? fc : fp;
    LinePoints(a, fa, b, fb, c.dc, o, PlaneCylinderSection, &c, max_depth,
               keep * line, points);
  }
}

/* a cylinder's face on a plane's face, of two CAD geometries */
static void PlaneCylinderFaces(const shift::core::collision_geometry::CadSideOfPair& plane, int fp,
                               const shift::core::collision_geometry::CadSideOfPair& cylinder, int fc,
                               bool plane_is_a, double max_depth,
                               std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  if(!SameSense(cylinder, fc)) {
    return; // a hole: its edges touch a plane, not its surface
  }
  PlaneCylinder c;
  double scale;
  SurfaceAxis(plane, fp, c.op, c.np, scale);
  if(!SameSense(plane, fp)) {
    ksl_vec3_scale(&c.np, -1.0);
  }
  c.plane_is_a = plane_is_a;
  PlaneCylinderLine(c, &plane, fp, cylinder, fc, max_depth, points);
}

/* ------------------------------------------------------------------------ */
/* points of contact by Newton's method */

/* solves the n by n system J x = b (J row-major, overwritten) by LU with
full pivoting; false if J is singular */
static bool Solve(double* J, const double* b, double* x, int n) {
  int pr[4], pc[4];
  const int rank = ksl_linalg_lu_full_rmo(n, n, J, 1e-12, pr, pc);
  if(rank < n) {
    return false;
  }
  /* P_r J P_c = L U: L U y = P_r b, x = P_c y */
  double y[4];
  for(int i = 0; i < n; i++) {
    y[i] = b[pr[i]];
    for(int k = 0; k < i; k++) {
      y[i] -= J[i * n + k] * y[k];
    }
  }
  for(int i = n - 1; i >= 0; i--) {
    for(int k = i + 1; k < n; k++) {
      y[i] -= J[i * n + k] * y[k];
    }
    y[i] /= J[i * n + i];
  }
  for(int i = 0; i < n; i++) {
    x[pc[i]] = y[i];
  }
  return true;
}

/* zero where a point of each face, x = (u_a, v_a, u_b, v_b), lie on a normal
common to both, where they are nearest, or deepest: the line between them is
square to a's tangents, and a's normal to b's tangents. The gradient of their
distance alone would also be zero wherever the faces cross, at no depth */
static void CommonNormal(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                             const shift::core::collision_geometry::CadSideOfPair& b, int fb, const double* x,
                             double* g) {
  ksl_vec3_t pa_u, pa_v, pb_u, pb_v;
  const ksl_vec3_t pa = SidePoint(a, fa, {x[0], x[1]}, &pa_u, &pa_v);
  const ksl_vec3_t pb = SidePoint(b, fb, {x[2], x[3]}, &pb_u, &pb_v);
  const ksl_vec3_t d = Sub(pa, pb);
  const ksl_vec3_t na = SideNormal(a, fa, {x[0], x[1]});
  g[0] = Dot(d, pa_u);
  g[1] = Dot(d, pa_v);
  g[2] = Dot(na, pb_u);
  g[3] = Dot(na, pb_v);
}

/* the curvature tensor of the gap between two faces touching at a point with
the normal n (a's frame): the sum of the faces' tensors in the tangent plane,
in the unit tangents t1 and t2, h = (h11, h12, h22). b is nullptr for a
plane */
static void GapTensor(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::cad::Vec2& uva,
                      const shift::core::collision_geometry::CadSideOfPair* b, int fb, const shift::core::cad::Vec2& uvb,
                      const ksl_vec3_t& n, ksl_vec3_t& t1, ksl_vec3_t& t2,
                      double h[3]) {
  Perpendiculars(n, t1, t2);
  h[0] = h[1] = h[2] = 0.0;
  auto add = [&](const shift::core::collision_geometry::CadSideOfPair& side, int face, const shift::core::cad::Vec2& uv) {
    ksl_vec3_t own1, own2;
    ksl_product_drinvv(&side.d.R, &t1, &own1);
    ksl_product_drinvv(&side.d.R, &t2, &own2);
    double k[3];
    side.shape->Curvature(face, uv, own1, own2, k);
    for(int i = 0; i < 3; i++) {
      h[i] += k[i];
    }
  };
  add(a, fa, uva);
  if(b) {
    add(*b, fb, uvb);
  }
}

/* the curvature of the gap between two faces touching at a point with the
normal n (a's frame): the eigenvalues k1 <= k2 of GapTensor are twice the
gap's half curvatures A and B. Sets c.curvature = A + B, the sum of the mean
curvatures, and c.curvature_ratio = B / A, limited to 1e3 (a gap curved in
one direction only, or a saddle, is taken as that elongated). b is nullptr
for a plane */
static void GapCurvature(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::cad::Vec2& uva,
                         const shift::core::collision_geometry::CadSideOfPair* b, int fb, const shift::core::cad::Vec2& uvb,
                         const ksl_vec3_t& n, shift::core::collision_geometry::ContactPoint& c) {
  ksl_vec3_t t1, t2;
  double h[3];
  GapTensor(a, fa, uva, b, fb, uvb, n, t1, t2, h);
  const double mean = 0.5 * (h[0] + h[2]);
  const double spread =
    std::sqrt(0.25 * (h[0] - h[2]) * (h[0] - h[2]) + h[1] * h[1]);
  const double k2 = mean + spread;
  const double k1 = std::max(mean - spread, 1e-3 * k2);
  c.curvature = mean;
  c.curvature_ratio = k2 > 0.0 ? k2 / k1 : 1.0;
}

static double FaceDepth(const shift::core::collision_geometry::CadSideOfPair& side,
                        int face, const ksl_vec3_t& p,
                        const shift::core::cad::Vec2* guess,
                        shift::core::cad::Vec2& uv, ksl_vec3_t& q,
                        ksl_vec3_t& outward, bool& beyond);

/* a starting point of NewtonPoint: the parameters x = (u_a, v_a, u_b, v_b) of
a sample of one of two faces and of its foot on the other, and its score, the
depth of the sample under the other face */
struct FaceSeed {
  double score = -HUGE_VAL;
  double x[4] = {0.0, 0.0, 0.0, 0.0};
  bool of_b = false; //!< a local maximum of b's samples, not of a's
  /* x is the deepest point found from the seed in the last test, which
  NewtonPoint starts from without the climb */
  bool settled = false;
  /* the climb from the seed ended with the faces apart by this much, with b's
  frame at the pose apart_at in a's: the depth there changes no faster than the
  faces move, so they cannot touch there until they have moved by as much */
  double apart = 0.0;
  ksl_SE3_t apart_at;
};

/* the seeds of each pair of faces of two CAD geometries, from the last search
of the samples (FaceSeeds), with the pose of b's frame in a's then. A region of
contact not among them lay deeper than the threshold of FaceSeeds, by at least
half of the slack it allows, outside the other face: while no point of b's face
has moved by more than margin, a quarter of that slack, the seeds hold every
region in which the faces can touch, and the search is not repeated */
struct shift::core::collision_geometry::CadPairCache {
  struct Entry {
    int fa = -1, fb = -1;
    double max_depth = 0.0;
    double margin = 0.0;
    ksl_SE3_t d;
    std::vector<FaceSeed> seeds;
    /* whether the faces conform (see Conforms), 1 or 0, -1 if not known; and
    the pose of b's frame then, kept while no point of b's face has moved by
    more than half a sample's spacing */
    int conform = -1;
    bool small_is_a = true;
    ksl_SE3_t conform_d;
  };
  std::vector<Entry> entries;

  Entry* Find(int fa, int fb) {
    for(Entry& e : entries) {
      if(e.fa == fa && e.fb == fb) {
        return &e;
      }
    }
    return nullptr;
  }
};

/* the most that a point of the box (in b's frame) moves in a's frame from the
pose d0 of b's frame to d: the motion is affine, so greatest at a corner */
static double BoxMotion(const shift::core::collision_geometry::CadBox& box,
                        const ksl_SE3_t& d0, const ksl_SE3_t& d) {
  double most = 0.0;
  for(int corner = 0; corner < 8; corner++) {
    const ksl_vec3_t p = {{(corner & 1) ? box.hi.x : box.lo.x,
                           (corner & 2) ? box.hi.y : box.lo.y,
                           (corner & 4) ? box.hi.z : box.lo.z}};
    ksl_vec3_t p0, p1;
    ksl_product_dv(&d0, &p, &p0);
    ksl_product_dv(&d, &p, &p1);
    const ksl_vec3_t m = Sub(p1, p0);
    most = std::max(most, ksl_vec3_l2norm(&m));
  }
  return most;
}

/* the most starting points taken for two faces: the deepest, then one for
each other region in which they may touch */
static constexpr int kMaxSeeds = 32;

/* the spacing of the samples of a face: its diagonal over the samples along
its longer side */
static double SampleSpacing(const shift::core::collision_geometry::CadContactFace& f) {
  const ksl_vec3_t d = Sub(f.box.hi, f.box.lo);
  return ksl_vec3_l2norm(&d) / std::max({f.sample_nu, f.sample_nv, 1});
}

/* whether sample i of face f scores at least as high as each of its
neighbours on the grid of samples, the lower index winning a tie */
static bool LocalMaximum(const shift::core::collision_geometry::CadContactFace& f,
                         const std::vector<double>& score, int i, int ci,
                         int cj) {
  for(int di = -1; di <= 1; di++) {
    for(int dj = -1; dj <= 1; dj++) {
      const int ni = ci + di, nj = cj + dj;
      if((di == 0 && dj == 0) || ni < 0 || nj < 0 || ni >= f.sample_nu ||
         nj >= f.sample_nv) {
        continue;
      }
      const int k = f.sample_cell[ni * f.sample_nv + nj];
      if(k >= 0 && (score[k] > score[i] || (score[k] == score[i] && k < i))) {
        return false;
      }
    }
  }
  return true;
}

/* the seeds of two faces from the depth of each sample of either face under
the other, and the parameters of its foot there (-HUGE_VAL where it has none):
each local maximum of the depth on either face's grid of samples that reaches
the threshold, deepest first, or else best */
static void SeedsAtMaxima(const shift::core::collision_geometry::CadContactFace& face_a,
                          const std::vector<double>& depth_a,
                          const std::vector<shift::core::cad::Vec2>& foot_a,
                          const shift::core::collision_geometry::CadContactFace& face_b,
                          const std::vector<double>& depth_b,
                          const std::vector<shift::core::cad::Vec2>& foot_b,
                          double threshold, const FaceSeed& best,
                          std::vector<FaceSeed>& seeds) {
  auto maxima = [&](const shift::core::collision_geometry::CadContactFace& f,
                    const std::vector<double>& depth,
                    const std::vector<shift::core::cad::Vec2>& foot, bool of_a) {
    for(int ci = 0; ci < f.sample_nu; ci++) {
      for(int cj = 0; cj < f.sample_nv; cj++) {
        const int i = f.sample_cell[ci * f.sample_nv + cj];
        if(i < 0 || depth[i] < threshold || !LocalMaximum(f, depth, i, ci, cj)) {
          continue;
        }
        FaceSeed seed;
        seed.score = depth[i];
        const shift::core::cad::Vec2& own = f.sample_uv[i];
        seed.x[0] = of_a ? own[0] : foot[i][0];
        seed.x[1] = of_a ? own[1] : foot[i][1];
        seed.x[2] = of_a ? foot[i][0] : own[0];
        seed.x[3] = of_a ? foot[i][1] : own[1];
        seed.of_b = !of_a;
        seeds.push_back(seed);
      }
    }
  };
  maxima(face_a, depth_a, foot_a, true);
  maxima(face_b, depth_b, foot_b, false);
  if(seeds.empty()) {
    seeds.push_back(best);
  }
  std::stable_sort(seeds.begin(), seeds.end(),
                   [](const FaceSeed& p, const FaceSeed& q) {
                     return p.score > q.score;
                   });
  if(seeds.size() > static_cast<size_t>(kMaxSeeds)) {
    seeds.resize(kMaxSeeds);
  }
}

/* the starting points of the search for the contact of two faces, deepest
first. Each sample of either face is scored with the sample of the other face,
of opposed normal, under which it lies deepest, less their offset across the
normal: the faces are apart if no score reaches the threshold. Each sample
that does is projected on the other face from that partner for its depth under
it, and each local maximum of the depth on the grid of samples of either face
is a seed: two faces touching in separate places have a maximum near each.
Only samples near the other face are scored */
static void FaceSeeds(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                      const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                      double max_depth, std::vector<FaceSeed>& seeds) {
  seeds.clear();
  const auto& face_a = a.shape->faces[fa];
  const auto& face_b = b.shape->faces[fb];
  /* samples are a grid's spacing apart, and none lies on a face's edges: the
  nearest of two faces touching near their ends may be this far off along
  and across the faces, on both */
  const double slack = 2.0 * (SampleSpacing(face_a) + SampleSpacing(face_b));
  const double threshold = -max_depth - slack;
  shift::core::collision_geometry::CadBox near_b = face_b.box.Transformed(b.d);
  shift::core::collision_geometry::CadBox near_a = face_a.box.Transformed(a.d);
  near_b.Pad(4.0 * max_depth + slack);
  near_a.Pad(4.0 * max_depth + slack);
  auto inside = [](const shift::core::collision_geometry::CadBox& box, const ksl_vec3_t& p) {
    for(int k = 0; k < 3; k++) {
      if(p.at[k] < box.lo.at[k] || p.at[k] > box.hi.at[k]) {
        return false;
      }
    }
    return true;
  };
  thread_local std::vector<int> near_ia, near_jb;
  thread_local std::vector<ksl_vec3_t> pa, na, pb, nb;
  near_ia.clear();
  near_jb.clear();
  pa.clear();
  na.clear();
  pb.clear();
  nb.clear();
  for(size_t i = 0; i < face_a.sample_uv.size(); i++) {
    const ksl_vec3_t p = SidePosition(a, face_a.sample_points[i]);
    if(inside(near_b, p)) {
      near_ia.push_back(static_cast<int>(i));
      pa.push_back(p);
      na.push_back(SideVector(a, face_a.sample_normals[i]));
    }
  }
  for(size_t j = 0; j < face_b.sample_uv.size(); j++) {
    const ksl_vec3_t p = SidePosition(b, face_b.sample_points[j]);
    if(inside(near_a, p)) {
      near_jb.push_back(static_cast<int>(j));
      pb.push_back(p);
      nb.push_back(SideVector(b, face_b.sample_normals[j]));
    }
  }
  if(near_ia.empty() || near_jb.empty()) {
    return;
  }
  /* each sample's best score and partner */
  thread_local std::vector<double> score_a, score_b;
  thread_local std::vector<int> partner_a, partner_b;
  score_a.assign(face_a.sample_uv.size(), -HUGE_VAL);
  score_b.assign(face_b.sample_uv.size(), -HUGE_VAL);
  partner_a.assign(face_a.sample_uv.size(), -1);
  partner_b.assign(face_b.sample_uv.size(), -1);
  FaceSeed best;
  for(size_t ii = 0; ii < near_ia.size(); ii++) {
    const int i = near_ia[ii];
    for(size_t jj = 0; jj < near_jb.size(); jj++) {
      const int j = near_jb[jj];
      if(Dot(na[ii], nb[jj]) > -0.5) {
        continue;
      }
      const ksl_vec3_t d = Sub(pa[ii], pb[jj]);
      const double depth = Dot(d, na[ii]);
      const ksl_vec3_t lateral = Axpy(d, -depth, na[ii]);
      const double score = depth - ksl_vec3_l2norm(&lateral);
      if(score > score_a[i]) {
        score_a[i] = score;
        partner_a[i] = j;
      }
      if(score > score_b[j]) {
        score_b[j] = score;
        partner_b[j] = i;
      }
      if(score > best.score) {
        best.score = score;
        best.x[0] = face_a.sample_uv[i][0];
        best.x[1] = face_a.sample_uv[i][1];
        best.x[2] = face_b.sample_uv[j][0];
        best.x[3] = face_b.sample_uv[j][1];
      }
    }
  }
  if(best.score == -HUGE_VAL || best.score < threshold) {
    return;
  }
  /* the depth of each sample, that scores above the threshold, under the
  other face, from its partner: the score of a sample paired with one a
  spacing away is too coarse to show where the faces touch, the depth is
  not. Then each local maximum of the depth on either face's grid */
  thread_local std::vector<double> depth_a, depth_b;
  thread_local std::vector<shift::core::cad::Vec2> foot_a, foot_b;
  auto depths = [&](const shift::core::collision_geometry::CadSideOfPair& side,
                    int face, const shift::core::collision_geometry::CadSideOfPair& other,
                    int other_face, const std::vector<int>& near,
                    const std::vector<double>& score,
                    const std::vector<int>& partner, std::vector<double>& depth,
                    std::vector<shift::core::cad::Vec2>& foot) {
    const auto& f = side.shape->faces[face];
    const auto& g = other.shape->faces[other_face];
    depth.assign(f.sample_uv.size(), -HUGE_VAL);
    foot.resize(f.sample_uv.size());
    for(int i : near) {
      if(score[i] < threshold) {
        continue;
      }
      ksl_vec3_t q, outward;
      bool beyond = false;
      const double d =
        FaceDepth(other, other_face, SidePosition(side, f.sample_points[i]),
                  &g.sample_uv[partner[i]], foot[i], q, outward, beyond);
      if(!beyond && Dot(SideVector(side, f.sample_normals[i]), outward) < -0.5) {
        depth[i] = d;
      }
    }
  };
  depths(a, fa, b, fb, near_ia, score_a, partner_a, depth_a, foot_a);
  depths(b, fb, a, fa, near_jb, score_b, partner_b, depth_b, foot_b);
  SeedsAtMaxima(face_a, depth_a, foot_a, face_b, depth_b, foot_b, threshold,
                best, seeds);
}

/* the seeds of FaceSeeds from the closest-point fields of the faces
(contact_seeds "field"): each sample of either face within the band of the
other face's field is projected on that face from the node nearest it, for
its depth under it, and each local maximum of the depth on either face's grid
of samples is a seed. No pair of samples is scored, and no sample of a face
is searched for the one nearest a point */
static void FieldSeeds(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                       const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                       double max_depth, std::vector<FaceSeed>& seeds) {
  seeds.clear();
  const auto& face_a = a.shape->faces[fa];
  const auto& face_b = b.shape->faces[fb];
  const double slack = 2.0 * (SampleSpacing(face_a) + SampleSpacing(face_b));
  const double threshold = -max_depth - slack;
  FaceSeed best;
  thread_local std::vector<double> depth_a, depth_b;
  thread_local std::vector<shift::core::cad::Vec2> foot_a, foot_b;
  auto depths = [&](const shift::core::collision_geometry::CadSideOfPair& side,
                    int face, const shift::core::collision_geometry::CadSideOfPair& other,
                    int other_face, std::vector<double>& depth,
                    std::vector<shift::core::cad::Vec2>& foot, bool of_a) {
    const auto& f = side.shape->faces[face];
    const shift::core::collision_geometry::CadFaceField& field =
      (*other.fields)[other_face];
    depth.assign(f.sample_uv.size(), -HUGE_VAL);
    foot.resize(f.sample_uv.size());
    for(size_t i = 0; i < f.sample_uv.size(); i++) {
      const ksl_vec3_t p = SidePosition(side, f.sample_points[i]);
      ksl_vec3_t own;
      ksl_product_dinvv(&other.d, &p, &own);
      const shift::core::collision_geometry::CadFaceField::Node* node =
        field.Nearest(own);
      if(!node) {
        continue;
      }
      ksl_vec3_t q, outward = {{0.0, 0.0, 0.0}};
      bool beyond = false;
      const double d =
        FaceDepth(other, other_face, p, &node->uv, foot[i], q, outward, beyond);
      if(d == -HUGE_VAL || beyond ||
         Dot(SideVector(side, f.sample_normals[i]), outward) >= -0.5) {
        continue;
      }
      depth[i] = d;
      if(d > best.score) {
        best.score = d;
        const shift::core::cad::Vec2& at = f.sample_uv[i];
        best.x[0] = of_a ? at[0] : foot[i][0];
        best.x[1] = of_a ? at[1] : foot[i][1];
        best.x[2] = of_a ? foot[i][0] : at[0];
        best.x[3] = of_a ? foot[i][1] : at[1];
        best.of_b = !of_a;
      }
    }
  };
  depths(a, fa, b, fb, depth_a, foot_a, true);
  depths(b, fb, a, fa, depth_b, foot_b, false);
  if(best.score == -HUGE_VAL || best.score < threshold) {
    return;
  }
  SeedsAtMaxima(face_a, depth_a, foot_a, face_b, depth_b, foot_b, threshold,
                best, seeds);
}

/* whether the depth of two faces is greatest at a point of contact c found by
NewtonPoint, not at a saddle of it: the gap between them curves away from the
point in every direction, or not at all in one */
static bool DeepestNear(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                        const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                        const double* x) {
  ksl_vec3_t t1, t2;
  double h[3];
  const shift::core::cad::Vec2 uva{x[0], x[1]}, uvb{x[2], x[3]};
  GapTensor(a, fa, uva, &b, fb, uvb, SideNormal(a, fa, uva), t1, t2, h);
  const double mean = 0.5 * (h[0] + h[2]);
  const double spread =
    std::sqrt(0.25 * (h[0] - h[2]) * (h[0] - h[2]) + h[1] * h[1]);
  return mean - spread >= -1e-3 * std::fabs(mean + spread);
}

/* the deepest point of two faces in contact: Newton's method on CommonNormal,
from the sample points x of a FaceSeed. Faces
touching along a line have no single deepest point, and a singular system:
damping, a share of its diagonal, then keeps each step to the nearest point of
the line. It does not change where the gradient is zero */
static bool NewtonPoint(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                        int fb, double max_depth, double* x, shift::core::collision_geometry::ContactPoint& c,
                        double damping = 0.0) {
  const auto& face_a = a.shape->faces[fa];
  const auto& face_b = b.shape->faces[fb];
  const double span[4] = {face_a.hi[0] - face_a.lo[0],
                          face_a.hi[1] - face_a.lo[1],
                          face_b.hi[0] - face_b.lo[0],
                          face_b.hi[1] - face_b.lo[1]};
  const double lo[4] = {face_a.lo[0], face_a.lo[1], face_b.lo[0], face_b.lo[1]};
  const double hi[4] = {face_a.hi[0], face_a.hi[1], face_b.hi[0], face_b.hi[1]};
  const bool periodic[4] = {face_a.periodic_u, face_a.periodic_v,
                            face_b.periodic_u, face_b.periodic_v};
  /* a NURBS surface does not go beyond its bounds, and a contact on a seam is
  on them */
  auto bounded = [&](int k, double value) {
    return periodic[k] ? value : std::min(std::max(value, lo[k]), hi[k]);
  };
  for(int k = 0; k < 4; k++) {
    x[k] = bounded(k, x[k]);
  }
  bool converged = false;
  for(int iteration = 0; iteration < 30 && !converged; iteration++) {
    double g[4], J[16];
    CommonNormal(a, fa, b, fb, x, g);
    for(int k = 0; k < 4; k++) {
      const double h = 1e-6 * std::max(span[k], 1e-9);
      double xp[4], xm[4], gp[4], gm[4];
      std::copy(x, x + 4, xp);
      std::copy(x, x + 4, xm);
      xp[k] = bounded(k, x[k] + h);
      xm[k] = bounded(k, x[k] - h);
      CommonNormal(a, fa, b, fb, xp, gp);
      CommonNormal(a, fa, b, fb, xm, gm);
      for(int i = 0; i < 4; i++) {
        J[i * 4 + k] = (gp[i] - gm[i]) / (xp[k] - xm[k]);
      }
    }
    const double diagonal = damping * (std::fabs(J[0]) + std::fabs(J[5]) +
                                       std::fabs(J[10]) + std::fabs(J[15]));
    for(int k = 0; k < 4; k++) {
      J[k * 4 + k] += diagonal;
    }
    double minus_g[4] = {-g[0], -g[1], -g[2], -g[3]}, step[4];
    /* at a pole of a face, where its point is the same for all of one
    parameter, the system is singular in that parameter: damped, the step
    leaves it be */
    double undamped[16];
    std::copy(J, J + 16, undamped);
    if(!Solve(J, minus_g, step, 4)) {
      const double pole = 1e-6 * (std::fabs(undamped[0]) + std::fabs(undamped[5]) +
                                  std::fabs(undamped[10]) + std::fabs(undamped[15]));
      for(int k = 0; k < 4; k++) {
        undamped[k * 4 + k] += pole;
      }
      if(diagonal > 0.0 || !Solve(undamped, minus_g, step, 4)) {
        return false;
      }
    }
    /* steps of at most a fifth of the faces' spans */
    double limit = 1.0;
    for(int k = 0; k < 4; k++) {
      if(std::fabs(step[k]) > 0.2 * span[k]) {
        limit = std::min(limit, 0.2 * span[k] / std::fabs(step[k]));
      }
    }
    converged = true;
    for(int k = 0; k < 4; k++) {
      const double next = bounded(k, x[k] + limit * step[k]);
      converged = converged && std::fabs(next - x[k]) <= 1e-10 * span[k];
      x[k] = next;
    }
  }
  if(!converged) {
    return false;
  }
  const shift::core::cad::Vec2 uva{x[0], x[1]}, uvb{x[2], x[3]};
  if(!InsideOrOnEdge(a, fa, uva) || !InsideOrOnEdge(b, fb, uvb)) {
    return false;
  }
  const ksl_vec3_t pa = SidePoint(a, fa, uva);
  const ksl_vec3_t pb = SidePoint(b, fb, uvb);
  const ksl_vec3_t na = SideNormal(a, fa, uva);
  const ksl_vec3_t nb = SideNormal(b, fb, uvb);
  const ksl_vec3_t d = Sub(pa, pb);
  const double depth = Dot(d, na);
  const ksl_vec3_t lateral = Axpy(d, -depth, na);
  /* the normals of a deepest point are opposite: a search stopped at a face's
  bound, with the deepest point beyond it, has not found one */
  if(Dot(na, nb) > -1.0 + 1e-10 || !(depth > 0.0) || !Within(pa, pb, depth, max_depth) ||
     ksl_vec3_l2norm(&lateral) > 0.05 * depth) {
    return false;
  }
  c.depth = depth;
  c.point_a = pa;
  c.point_b = pb;
  c.normal = na;
  GapCurvature(a, fa, uva, &b, fb, uvb, na, c);
  return true;
}

/* ------------------------------------------------------------------------ */
/* lines of contact on curved faces */

/* a curved face (NURBS or extruded) against a cylinder or a plane, in a's
frame. A station s along the line is the plane through origin + s direction
normal to direction, and it cuts the curved face in a curve */
struct CurvedLine {
  const shift::core::collision_geometry::CadSideOfPair* curved = nullptr;
  int face = -1;
  bool curved_is_a = true;
  ksl_vec3_t origin, direction;
  ksl_vec3_t other_point;  //!< the plane's point, or a point of the axis
  ksl_vec3_t other_vector; //!< the plane's outward normal, or the axis
  double radius = 0.0;     //!< of the cylinder, 0 for a plane
  bool convex = true;      //!< a cylinder, not a hole
  /* another curved face, if it is not a plane or a cylinder, with the
  parameters of the last point found on it, where the next search starts */
  const shift::core::collision_geometry::CadSideOfPair* other_side = nullptr;
  int other_face = -1;
  mutable shift::core::cad::Vec2 other_uv{0.0, 0.0};
  mutable bool beyond = false; //!< the last point was past the other face
  /* the parameters of the last section found, where the next one starts */
  mutable shift::core::cad::Vec2 uv{0.0, 0.0};
  /* whether the face touches, with its points the same as another's */
  bool repeated = false;
  /* a ring: the stations are the half planes through the axis of the curved
  face, a face of revolution, s the arc length about it at ring_radius from
  reference, the unit radial of the seed */
  bool ring = false;
  ksl_vec3_t axis_point, axis, reference;
  double ring_radius = 0.0;
  /* a line traced along its bends: the polyline through the deepest points
  of its sections, path[k], with the unit direction of each segment and the
  arc length at its start. A station s is in the plane square to the segment
  holding s, and s is the arc length along the polyline */
  bool traced = false;
  std::vector<ksl_vec3_t> path;
  std::vector<ksl_vec3_t> path_direction;
  std::vector<double> path_length;
};

/* the station of the point p along the line of c, and its gradient: the
distance along direction, or for a ring the arc length about the axis, taken
within half a turn of near */
static double Station(const CurvedLine& c, const ksl_vec3_t& p, double near,
                      ksl_vec3_t* gradient) {
  if(c.traced) {
    /* the segment holding near, the first or last beyond the ends */
    const auto& l = c.path_length;
    const size_t segments = c.path_direction.size();
    size_t k = std::upper_bound(l.begin(), l.begin() + segments, near) -
               l.begin();
    k = k > 0 ? k - 1 : 0;
    if(gradient) {
      *gradient = c.path_direction[k];
    }
    return l[k] + Dot(Sub(p, c.path[k]), c.path_direction[k]);
  }
  if(!c.ring) {
    if(gradient) {
      *gradient = c.direction;
    }
    return Dot(Sub(p, c.origin), c.direction);
  }
  const ksl_vec3_t w = Sub(p, c.axis_point);
  const ksl_vec3_t radial = Axpy(w, -Dot(w, c.axis), c.axis);
  ksl_vec3_t across;
  ksl_cross_vv(&c.axis, &c.reference, &across);
  const double x = Dot(radial, c.reference), y = Dot(radial, across);
  const double turn = 2.0 * M_PI * c.ring_radius;
  double s = c.ring_radius * std::atan2(y, x);
  s += turn * std::round((near - s) / turn);
  if(gradient) {
    /* d(angle) = (axis x radial) . dp / |radial|^2 */
    ksl_vec3_t hoop;
    ksl_cross_vv(&c.axis, &radial, &hoop);
    const double squared = x * x + y * y;
    *gradient = squared > 0.0 ? Scaled(hoop, c.ring_radius / squared)
                              : ksl_vec3_t{{0.0, 0.0, 0.0}};
  }
  return s;
}

/* the depth of the point p under a face, along the face's normal at q, the
point of the face nearest to it, from the parameters guess if given; -HUGE_VAL
if it has none. beyond: the point is past the face, so projects to its bound,
off the normal, and the contact there is the next face's */
static double FaceDepth(const shift::core::collision_geometry::CadSideOfPair& side,
                        int face, const ksl_vec3_t& p,
                        const shift::core::cad::Vec2* guess,
                        shift::core::cad::Vec2& uv, ksl_vec3_t& q,
                        ksl_vec3_t& outward, bool& beyond) {
  shift::core::cad::Vec2 found;
  if(!SideProject(side, face, p, found, guess)) {
    return -HUGE_VAL;
  }
  uv = found;
  q = SidePoint(side, face, uv);
  outward = SideNormal(side, face, uv);
  const double depth = Dot(Sub(q, p), outward);
  const ksl_vec3_t lateral = Axpy(Sub(q, p), -depth, outward);
  beyond = ksl_vec3_l2norm(&lateral) > 1e-9 * side.shape->size;
  return depth;
}

/* the depth of the point p in the cylinder or plane of c, its nearest point
q there and the outward normal; -HUGE_VAL on the axis of the cylinder */
static double OtherDepth(const CurvedLine& c, const ksl_vec3_t& p,
                         ksl_vec3_t& q, ksl_vec3_t& outward) {
  if(c.other_side) {
    /* a point past the face is no start for the next search: it would stay
    at the bound */
    shift::core::cad::Vec2 uv;
    const double depth = FaceDepth(*c.other_side, c.other_face, p, &c.other_uv,
                                   uv, q, outward, c.beyond);
    if(depth != -HUGE_VAL && !c.beyond) {
      c.other_uv = uv;
    }
    return depth;
  }
  const ksl_vec3_t w = Sub(p, c.other_point);
  if(c.radius == 0.0) {
    const double depth = -Dot(w, c.other_vector);
    q = Axpy(p, depth, c.other_vector);
    outward = c.other_vector;
    return depth;
  }
  const double axial = Dot(w, c.other_vector);
  ksl_vec3_t radial = Axpy(w, -axial, c.other_vector);
  const double distance = ksl_vec3_l2norm(&radial);
  if(!(distance > 1e-12 * c.radius)) {
    return -HUGE_VAL;
  }
  ksl_vec3_scale(&radial, 1.0 / distance);
  q = Axpy(Axpy(c.other_point, axial, c.other_vector), c.radius, radial);
  outward = c.convex ? radial : Scaled(radial, -1.0);
  return c.convex ? c.radius - distance : distance - c.radius;
}

/* the distance of the point (u, v) of the curved face from the plane of the
station s, and the slope of the depth along the curve the plane cuts: both are
zero at the deepest point of the section */
static bool SectionResidual(const CurvedLine& c, double s, const shift::core::cad::Vec2& uv,
                            double r[2], ksl_vec3_t* du = nullptr,
                            ksl_vec3_t* dv = nullptr,
                            ksl_vec3_t* gradient = nullptr) {
  ksl_vec3_t pu, pv, q, outward, g;
  const ksl_vec3_t p = SidePoint(*c.curved, c.face, uv, &pu, &pv);
  if(OtherDepth(c, p, q, outward) == -HUGE_VAL) {
    return false;
  }
  const double station = Station(c, p, s, &g);
  ksl_vec3_t along = Axpy(Scaled(pu, Dot(pv, g)), -Dot(pu, g), pv);
  const double length = ksl_vec3_l2norm(&along);
  if(!(length > 0.0)) {
    return false;
  }
  r[0] = station - s;
  r[1] = Dot(outward, along) / length;
  if(du) {
    *du = pu;
    *dv = pv;
  }
  if(gradient) {
    *gradient = g;
  }
  return true;
}

/* how far the normals of two faces must oppose each other, as the cosine of
the angle from opposite, for a curved face to touch along a section */
static constexpr double kOpposed = 0.5;

/* the cross section of a curved face on a cylinder or plane at the station
s: the deepest point of the curve, by Newton's method from the last one, and
inside the bounds of the face, as a NURBS surface does not extend beyond them.
Its depth, or -HUGE_VAL if the section has no deepest point there or the faces
do not oppose each other */
static double CurvedSection(const void* data, double s, ksl_vec3_t& pa,
                            ksl_vec3_t& pb, ksl_vec3_t& n) {
  const CurvedLine& c = *static_cast<const CurvedLine*>(data);
  const auto& f = c.curved->shape->faces[c.face];
  const double span[2] = {f.hi[0] - f.lo[0], f.hi[1] - f.lo[1]};
  const bool periodic[2] = {f.periodic_u, f.periodic_v};
  shift::core::cad::Vec2 uv = c.uv;
  bool converged = false;
  for(int iteration = 0; iteration < 16 && !converged; iteration++) {
    double r[2], ru[2], rv[2];
    ksl_vec3_t du, dv, gradient;
    shift::core::cad::Vec2 at_u = uv, at_v = uv;
    const double hu = 1e-6 * std::max(span[0], 1e-9);
    const double hv = 1e-6 * std::max(span[1], 1e-9);
    at_u[0] += uv[0] + hu > f.hi[0] ? -hu : hu;
    at_v[1] += uv[1] + hv > f.hi[1] ? -hv : hv;
    if(!SectionResidual(c, s, uv, r, &du, &dv, &gradient) ||
       !SectionResidual(c, s, at_u, ru) || !SectionResidual(c, s, at_v, rv)) {
      return -HUGE_VAL;
    }
    /* the first row exactly, the second by differences */
    const double a11 = Dot(du, gradient), a12 = Dot(dv, gradient);
    const double a21 = (ru[1] - r[1]) / (at_u[0] - uv[0]);
    const double a22 = (rv[1] - r[1]) / (at_v[1] - uv[1]);
    const double det = a11 * a22 - a12 * a21;
    if(det == 0.0) {
      return -HUGE_VAL;
    }
    double step[2] = {(-r[0] * a22 + r[1] * a12) / det,
                      (r[0] * a21 - r[1] * a11) / det};
    /* steps of at most a fifth of the face's span, within its bounds */
    double limit = 1.0;
    for(int k = 0; k < 2; k++) {
      if(std::fabs(step[k]) > 0.2 * span[k]) {
        limit = std::min(limit, 0.2 * span[k] / std::fabs(step[k]));
      }
    }
    converged = true;
    for(int k = 0; k < 2; k++) {
      double next = uv[k] + limit * step[k];
      if(!periodic[k]) {
        const double margin = 1e-9 * span[k];
        if((next < f.lo[k] - margin && uv[k] <= f.lo[k]) ||
           (next > f.hi[k] + margin && uv[k] >= f.hi[k])) {
          return -HUGE_VAL; // the deepest point is beyond the face
        }
        next = std::min(std::max(next, f.lo[k]), f.hi[k]);
      }
      converged = converged && std::fabs(next - uv[k]) <= 1e-10 * span[k];
      uv[k] = next;
    }
  }
  if(!converged) {
    return -HUGE_VAL;
  }
  ksl_vec3_t q, outward;
  const ksl_vec3_t p = SidePoint(*c.curved, c.face, uv);
  const double depth = OtherDepth(c, p, q, outward);
  if(depth == -HUGE_VAL || c.beyond ||
     Dot(SideNormal(*c.curved, c.face, uv), outward) > -kOpposed) {
    return -HUGE_VAL;
  }
  c.uv = uv;
  if(c.curved_is_a) {
    pa = p;
    pb = q;
    n = Scaled(outward, -1.0);
  } else {
    pa = q;
    pb = p;
    n = outward;
  }
  return depth;
}

/* the range [lo, hi] of a face along the line of c */
static void FaceRange(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                      const CurvedLine& c, double& lo, double& hi) {
  lo = HUGE_VAL;
  hi = -HUGE_VAL;
  for(const auto& p : side.shape->faces[face].boundary_points) {
    const double s = Station(c, SidePosition(side, p), 0.0, nullptr);
    lo = std::min(lo, s);
    hi = std::max(hi, s);
  }
}

/* where a curved face overlaps a cylinder or plane along the line, s in
[s0, s1]. Unlike Overlap, this cannot look at the surfaces beyond [s0, s1], and
a section may have no deepest point, so it starts from stations along the
range. An end of the overlap at an end of the range is extended for reach by
the parabola through the deepest station and that end, or without limit if the
depth does not fall towards it */
static bool CurvedOverlap(const CurvedLine& c, double s0, double s1,
                          LineOverlap& o) {
  constexpr int kIntervals = 8;
  const double step = (s1 - s0) / kIntervals;
  double depth[kIntervals + 1];
  int best = 0;
  for(int i = 0; i <= kIntervals; i++) {
    depth[i] = DepthAt(CurvedSection, &c, s0 + i * step);
    if(depth[i] > depth[best]) {
      best = i;
    }
  }
  if(!(depth[best] > 0.0)) {
    return false;
  }
  o.deepest = s0 + best * step;
  o.depth = depth[best];
  /* between its neighbors, if the depth falls to them */
  if(best > 0 && best < kIntervals &&
     o.depth - std::max(depth[best - 1], depth[best + 1]) > 1e-9 * o.depth) {
    const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
    double lo = o.deepest - step, hi = o.deepest + step;
    double t1 = hi - golden * (hi - lo), t2 = lo + golden * (hi - lo);
    double d1 = DepthAt(CurvedSection, &c, t1);
    double d2 = DepthAt(CurvedSection, &c, t2);
    for(int i = 0; i < 24; i++) {
      if(d1 > d2) {
        hi = t2;
        t2 = t1;
        d2 = d1;
        t1 = hi - golden * (hi - lo);
        d1 = DepthAt(CurvedSection, &c, t1);
      } else {
        lo = t1;
        t1 = t2;
        d1 = d2;
        t2 = lo + golden * (hi - lo);
        d2 = DepthAt(CurvedSection, &c, t2);
      }
    }
    o.deepest = 0.5 * (lo + hi);
    o.depth = std::max(o.depth, DepthAt(CurvedSection, &c, o.deepest));
  }
  /* each end of the overlap: the first station out from the deepest with
  no depth, and the zero between them by bisection */
  auto zero = [&](double inside, double outside) {
    for(int i = 0; i < 30; i++) {
      const double mid = 0.5 * (inside + outside);
      (DepthAt(CurvedSection, &c, mid) > 0.0 ? inside : outside) = mid;
    }
    return inside;
  };
  auto end = [&](int direction, double& clipped, double& extended) {
    int i = best;
    while(i + direction >= 0 && i + direction <= kIntervals &&
          depth[i + direction] > 0.0) {
      i += direction;
    }
    if(i + direction >= 0 && i + direction <= kIntervals) {
      const double inside = i == best ? o.deepest : s0 + i * step;
      clipped = extended = zero(inside, s0 + (i + direction) * step);
      return;
    }
    clipped = direction < 0 ? s0 : s1;
    const double fall = o.depth - depth[i];
    const double distance = std::fabs(clipped - o.deepest);
    extended = fall > 1e-9 * o.depth && distance > 0.0
                 ? o.deepest + direction * distance * std::sqrt(o.depth / fall)
                 : direction * HUGE_VAL;
  };
  double left, right;
  end(-1, o.lo, left);
  end(1, o.hi, right);
  o.reach = right - left;
  return o.hi > o.lo;
}

/* the most a traced line turns from one segment to the next */
static constexpr double kTraceTurn = 10.0 * M_PI / 180.0;

/* traces the line of c, which bends, from the deepest point of the section
through origin square to direction, both ways along it: each next point is the
deepest of the section a step on, square to the last segment, the step halved
where the line would turn by more than kTraceTurn and the trace ended where
the section has no deepest point, past the face, or the faces are apart by
more than max_depth. Sets the polyline of c; false if there is no section at
the start */
static bool TraceLine(CurvedLine& c, double max_depth) {
  const auto& f = c.curved->shape->faces[c.face];
  const ksl_vec3_t diagonal = Sub(f.box.hi, f.box.lo);
  const double longest = ksl_vec3_l2norm(&diagonal) / 16.0;
  const double shortest = longest / 256.0;
  ksl_vec3_t start, pb, normal;
  c.traced = false;
  if(CurvedSection(&c, 0.0, start, pb, normal) == -HUGE_VAL) {
    return false;
  }
  /* the curved face's point of a section */
  auto own = [&](const ksl_vec3_t& pa, const ksl_vec3_t& pb_) {
    return c.curved_is_a ? pa : pb_;
  };
  start = own(start, pb);
  const shift::core::cad::Vec2 start_uv = c.uv;
  const ksl_vec3_t start_direction = c.direction;
  std::vector<ksl_vec3_t> half[2];
  for(int side = 0; side < 2; side++) {
    std::vector<ksl_vec3_t>& points = half[side];
    ksl_vec3_t last = start;
    ksl_vec3_t direction =
      side == 0 ? start_direction : Scaled(start_direction, -1.0);
    shift::core::cad::Vec2 uv = start_uv;
    double step = longest;
    for(int iteration = 0; iteration < 1000 && step >= shortest;
        iteration++) {
      c.origin = last;
      c.direction = direction;
      c.uv = uv;
      ksl_vec3_t pa, pb2, n;
      const double depth = CurvedSection(&c, step, pa, pb2, n);
      if(depth == -HUGE_VAL) {
        step *= 0.5;
        continue;
      }
      const ksl_vec3_t next = own(pa, pb2);
      ksl_vec3_t chord = Sub(next, last);
      const double length = ksl_vec3_l2norm(&chord);
      if(!(length > 0.0)) {
        break;
      }
      ksl_vec3_scale(&chord, 1.0 / length);
      const double turn =
        std::acos(std::min(1.0, std::max(-1.0, Dot(chord, direction))));
      if(turn > kTraceTurn) {
        step *= 0.5;
        continue;
      }
      points.push_back(next);
      last = next;
      direction = chord;
      uv = c.uv;
      if(turn < 0.3 * kTraceTurn) {
        step = std::min(2.0 * step, longest);
      }
      if(depth < -max_depth) {
        break; // well apart: past the overlap
      }
    }
  }
  /* the polyline from the end of the second half to that of the first */
  c.path.assign(half[1].rbegin(), half[1].rend());
  c.path.push_back(start);
  c.path.insert(c.path.end(), half[0].begin(), half[0].end());
  if(c.path.size() < 2) {
    c.origin = start;
    c.direction = start_direction;
    c.uv = start_uv;
    return false;
  }
  c.path_direction.clear();
  c.path_length.assign(1, 0.0);
  for(size_t k = 0; k + 1 < c.path.size(); k++) {
    ksl_vec3_t d = Sub(c.path[k + 1], c.path[k]);
    const double length = ksl_vec3_l2norm(&d);
    c.path_direction.push_back(Scaled(d, 1.0 / length));
    c.path_length.push_back(c.path_length.back() + length);
  }
  c.traced = true;
  c.uv = start_uv;
  return true;
}

/* how far the line of c turns between the stations s0 and s1, in radians:
a ring's angle, or the turns between the segments of a traced line */
static double Turning(const CurvedLine& c, double s0, double s1) {
  if(c.ring) {
    return (s1 - s0) / c.ring_radius;
  }
  double turning = 0.0;
  for(size_t k = 1; k < c.path_direction.size(); k++) {
    if(c.path_length[k] > s0 && c.path_length[k] < s1) {
      turning += std::acos(std::min(
        1.0, std::max(-1.0, Dot(c.path_direction[k - 1], c.path_direction[k]))));
    }
  }
  return turning;
}

/* the pieces of [t0, t1] along the line of c that lie inside the trims of
the curved face and of the other face (nullptr for a primitive); false if a
section at an end has no deepest point */
static bool ClipSlice(const CurvedLine& c,
                      const shift::core::collision_geometry::CadSideOfPair* other,
                      int fo, double t0, double t1, double inset,
                      std::vector<std::array<double, 2>>& pieces) {
  const shift::core::collision_geometry::CadSideOfPair& curved = *c.curved;
  const int fc = c.face;
  /* the trims of both faces, along the line in their parameters: it may
  run along the seam of two faces, and just inside the bounds of each */
  ksl_vec3_t pa[2], pb[2], normal;
  shift::core::cad::Vec2 ends[2];
  for(int i = 0; i < 2; i++) {
    /* an end of the overlap is at the end of a face, where a search from
    elsewhere on the line can stop past it: a little further in will do */
    double depth = -HUGE_VAL;
    for(double back = 0.0; depth == -HUGE_VAL && back <= 1e3 * inset;
        back = back > 0.0 ? 10.0 * back : inset) {
      depth = CurvedSection(&c, i ? t1 - back : t0 + back, pa[i], pb[i],
                            normal);
    }
    if(depth == -HUGE_VAL) {
      return false;
    }
    ends[i] = c.uv;
  }
  /* the end's parameters nearest the start's, on a periodic face */
  auto unwrap = [&c](const shift::core::collision_geometry::CadContactFace& f,
                   const shift::core::cad::Vec2& from,
                   shift::core::cad::Vec2& to) {
    const bool periodic[2] = {f.periodic_u, f.periodic_v};
    for(int k = 0; k < 2; k++) {
      const double period = f.hi[k] - f.lo[k];
      if((c.ring || c.traced) && periodic[k] && period > 0.0) {
        to[k] += period * std::round((from[k] - to[k]) / period);
      }
    }
  };
  unwrap(curved.shape->faces[fc], ends[0], ends[1]);
  auto in_range = [&](std::vector<std::array<double, 2>> list) {
    for(auto& piece : list) {
      piece = {t0 + piece[0] * (t1 - t0), t0 + piece[1] * (t1 - t0)};
    }
    return list;
  };
  pieces = in_range(curved.shape->Clip(fc, ends[0], ends[1], kSeam));
  if(other) {
    shift::core::cad::Vec2 uv0, uv1;
    if(!SideProject(*other, fo, c.curved_is_a ? pb[0] : pa[0], uv0) ||
       !SideProject(*other, fo, c.curved_is_a ? pb[1] : pa[1], uv1, &uv0)) {
      return false;
    }
    unwrap(other->shape->faces[fo], uv0, uv1);
    pieces = Intersect(pieces,
                       in_range(other->shape->Clip(fo, uv0, uv1, kSeam)));
  }
  return true;
}

/* a gap curved less than this share of its greatest curvature, in the
direction of its least, touches along a line */
static constexpr double kLineCurvature = 0.1;

/*
A curved face on a cylinder or plane (other, nullptr for a primitive plane),
from the point seed of the face, in a's frame: if the gap between them there
is curved mainly in one direction they touch along a line, along the axis of
the cylinder or the direction of the gap's least curvature, sliced over the
length where they overlap, as for two cylinders. False, with nothing done, for
a gap curved in both directions, which touches at a point.

Where the overlap is short beside the faces' common length the contact is the
point of an ellipse, as for two cylinders. Where the depth is greatest at an
end of the range, a line shorter than a few times the Hertz width of the end
becomes that end, an edge rounded to edge_radius, as for a cylinder on a
plane. Either way the weights go smoothly, so the force has no jump.
*/
static bool CurvedFaceLine(CurvedLine& c, const shift::core::cad::Vec2& seed,
                           const shift::core::collision_geometry::CadSideOfPair* other, int fo,
                           double max_depth,
                           std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const shift::core::collision_geometry::CadSideOfPair& curved = *c.curved;
  const int fc = c.face;
  const ksl_vec3_t p = SidePoint(curved, fc, seed);
  const ksl_vec3_t n = SideNormal(curved, fc, seed);
  shift::core::cad::Vec2 other_uv{0.0, 0.0};
  if(other && !SideProject(*other, fo, p, other_uv)) {
    return false;
  }
  ksl_vec3_t t1, t2;
  double h[3];
  GapTensor(curved, fc, seed, other, fo, other_uv, n, t1, t2, h);
  const double mean = 0.5 * (h[0] + h[2]);
  const double spread =
    std::sqrt(0.25 * (h[0] - h[2]) * (h[0] - h[2]) + h[1] * h[1]);
  const double k1 = mean - spread, k2 = mean + spread;
  if(!(k2 * curved.shape->size > 1e-6) ||
     std::fabs(k1) > kLineCurvature * k2) {
    return false;
  }
  c.origin = p;
  c.uv = seed;
  if(c.radius > 0.0) {
    c.direction = c.other_vector;
  } else if(std::fabs(h[1]) > 1e-12 * k2) {
    c.direction = Axpy(Scaled(t1, h[1]), k1 - h[0], t2);
    ksl_vec3_normalize(&c.direction);
  } else {
    c.direction = h[0] < h[2] ? t1 : t2;
  }
  if(c.ring) {
    /* a ring only if the gap is straight round the axis */
    const ksl_vec3_t w = Sub(p, c.axis_point);
    c.reference = Axpy(w, -Dot(w, c.axis), c.axis);
    c.ring_radius = ksl_vec3_l2norm(&c.reference);
    if(!(c.ring_radius > 1e-9 * curved.shape->size)) {
      return false;
    }
    ksl_vec3_scale(&c.reference, 1.0 / c.ring_radius);
    ksl_vec3_t hoop;
    ksl_cross_vv(&c.axis, &c.reference, &hoop);
    if(std::fabs(Dot(hoop, c.direction)) < 0.9) {
      return false;
    }
    c.direction = hoop;
  }
  double s0, s1;
  bool other_ends[2] = {false, false}; //!< whose ends are s0 and s1
  if(c.ring) {
    /* the face's range of its angle about its axis, the first parameter,
    from the seed's; the other face's trims clip the ring in slices */
    const auto& f = curved.shape->faces[fc];
    const double period = 2.0 * M_PI;
    const double middle = 0.5 * (f.lo[0] + f.hi[0]);
    const double u = seed[0] + period * std::round((middle - seed[0]) / period);
    const double h = 1e-6;
    const double sense =
      Station(c, SidePoint(curved, fc, {seed[0] + h, seed[1]}), 0.0, nullptr) >
          0.0
        ? 1.0
        : -1.0;
    s0 = c.ring_radius * sense * (f.lo[0] - u);
    s1 = c.ring_radius * sense * (f.hi[0] - u);
    if(s0 > s1) {
      std::swap(s0, s1);
    }
  } else {
    FaceRange(curved, fc, c, s0, s1);
  }
  if(other && !c.ring) {
    double o0, o1;
    FaceRange(*other, fo, c, o0, o1);
    other_ends[0] = o0 > s0;
    other_ends[1] = o1 < s1;
    s0 = std::max(s0, o0);
    s1 = std::min(s1, o1);
  }
  LineOverlap o;
  /* the faces of a ring in pieces round it: two of them that meet only at a
  seam leave the contact there to the pieces on either side */
  if(c.ring && !(s1 - s0 > 1e-6 * c.ring_radius)) {
    return true;
  }
  if(!(s1 > s0) || !CurvedOverlap(c, s0, s1, o)) {
    return true;
  }
  /* a line that bends: the stations cut it along its chord, from the tangent
  at the seed only if that is near it */
  const double straightness = 1e-9 * curved.shape->size;
  bool bent = false;
  for(int pass = 0; pass < 2 && !c.ring; pass++) {
    ksl_vec3_t ends[2], middle, pb, normal;
    if(CurvedSection(&c, o.lo + 1e-6 * (o.hi - o.lo), ends[0], pb, normal) ==
         -HUGE_VAL ||
       CurvedSection(&c, o.hi - 1e-6 * (o.hi - o.lo), ends[1], pb, normal) ==
         -HUGE_VAL ||
       CurvedSection(&c, 0.5 * (o.lo + o.hi), middle, pb, normal) == -HUGE_VAL) {
      break;
    }
    ksl_vec3_t chord = Sub(ends[1], ends[0]);
    const double span = ksl_vec3_l2norm(&chord);
    if(!(span > 0.0)) {
      break;
    }
    ksl_vec3_scale(&chord, 1.0 / span);
    const ksl_vec3_t off = Sub(middle, ends[0]);
    const ksl_vec3_t bow = Axpy(off, -Dot(off, chord), chord);
    if(ksl_vec3_l2norm(&bow) <= straightness) {
      break;
    }
    bent = true;
    /* a line turning by more than about 30 degrees is traced along its
    bends: a chord would cut it at a slant, or more than once */
    if(ksl_vec3_l2norm(&bow) > 0.06 * span) {
      /* from the middle of the line, along its tangent there */
      auto own = [&](double station, ksl_vec3_t& q) {
        ksl_vec3_t pa, pb_, n;
        if(CurvedSection(&c, station, pa, pb_, n) == -HUGE_VAL) {
          return false;
        }
        q = c.curved_is_a ? pa : pb_;
        return true;
      };
      const double mid = 0.5 * (o.lo + o.hi);
      const double h = 1e-3 * (o.hi - o.lo);
      ksl_vec3_t q0, q1;
      if(!own(mid - h, q0) || !own(mid + h, q1) || !own(mid, middle)) {
        return true;
      }
      ksl_vec3_t tangent = Sub(q1, q0);
      const double tl = ksl_vec3_l2norm(&tangent);
      if(!(tl > 0.0)) {
        return true;
      }
      c.direction = Scaled(tangent, 1.0 / tl);
      c.origin = middle;
      if(!TraceLine(c, max_depth)) {
        return true;
      }
      s0 = c.path_length.front();
      s1 = c.path_length.back();
      other_ends[0] = other_ends[1] = false;
      if(!(s1 > s0) || !CurvedOverlap(c, s0, s1, o)) {
        return true;
      }
      break;
    }
    c.direction = chord;
    c.origin = ends[0];
    FaceRange(curved, fc, c, s0, s1);
    if(other) {
      double o0, o1;
      FaceRange(*other, fo, c, o0, o1);
      other_ends[0] = o0 > s0;
      other_ends[1] = o1 < s1;
      s0 = std::max(s0, o0);
      s1 = std::min(s1, o1);
    }
    if(!(s1 > s0) || !CurvedOverlap(c, s0, s1, o)) {
      return true;
    }
  }
  const size_t before = points.size();
  const double length = s1 - s0;
  const double inset = 1e-6 * length;
  /* the range is of the faces' bounds, and a line at a slant can end short of
  it, where a section stops finding the face */
  const bool at_end =
    o.deepest <= s0 + inset || o.deepest >= s1 - inset ||
    (c.other_side && (o.deepest <= o.lo + 2.0 * inset ||
                      o.deepest >= o.hi - 2.0 * inset));
  /* the end, of the geometry that ends first */
  const bool other_end = other_ends[o.deepest > 0.5 * (s0 + s1) ? 1 : 0];
  const double edge_radius =
    (other_end ? other : &curved)->edge_radius;
  double line;
  if(at_end) {
    line = SmoothStep(((o.hi - o.lo) / std::sqrt(o.depth * edge_radius) - 1.0) /
                      3.0);
  } else {
    line = SmoothStep((o.reach / length - 0.25) / 0.5);
  }
  auto on_other = [&](const ksl_vec3_t& at, shift::core::cad::Vec2& uv) {
    return !other || (SideProject(*other, fo, at, uv) &&
                      InsideOrOnEdge(*other, fo, uv));
  };
  if(line < 1.0) {
    /* the deepest point, tested just inside the end of an end's */
    shift::core::collision_geometry::ContactPoint r;
    const double station =
      at_end ? o.deepest + (o.deepest > 0.5 * (s0 + s1) ? -inset : inset)
             : o.deepest;
    ksl_vec3_t pa, pb, normal;
    shift::core::cad::Vec2 uv{0.0, 0.0};
    const bool found = CurvedSection(&c, station, pa, pb, normal) > 0.0 &&
                       InsideOrOnEdge(curved, fc, c.uv) &&
                       on_other(c.curved_is_a ? pb : pa, uv);
    r.depth = CurvedSection(&c, o.deepest, r.point_a, r.point_b, r.normal);
    if(r.depth == -HUGE_VAL) {
      r.depth = CurvedSection(&c, station, r.point_a, r.point_b, r.normal);
    }
    if(found && r.depth > 0.0 && Within(r, max_depth)) {
      if(at_end) {
        /* the end on the face it presses into */
        r.curvature = 1.0 / edge_radius;
        if(other_end) {
          r.curvature += curved.shape->MeanCurvature(fc, c.uv);
        } else if(other) {
          r.curvature += other->shape->MeanCurvature(fo, uv);
        }
      } else {
        GapCurvature(curved, fc, c.uv, other, fo, uv, r.normal, r);
      }
      r.weight = 1.0 - line;
      points.push_back(r);
    } else {
      line = 1.0; // not inside both faces: their line
    }
  }
  if(line > 0.0) {
    /* the trims of both faces, along the line in their parameters: it may
    run along the seam of two faces, and just inside the bounds of each */
    /* a ring in slices of at most a twelfth of a turn, each clipped on its
    own: a face's parameters about its axis, which may be periodic, are then
    unwrapped from one end of the slice to the other */
    const int slices =
      c.ring || c.traced
        ? std::max(1, static_cast<int>(std::ceil(
                        Turning(c, o.lo, o.hi) / (M_PI / 6.0) - 1e-6)))
        : 1;
    std::vector<std::array<double, 2>> pieces;
    for(int slice = 0; slice < slices; slice++) {
      const double t0 = o.lo + slice * (o.hi - o.lo) / slices;
      const double t1 = o.lo + (slice + 1) * (o.hi - o.lo) / slices;
      std::vector<std::array<double, 2>> part;
      if(!ClipSlice(c, other, fo, t0, t1, inset, part)) {
        return true;
      }
      pieces.insert(pieces.end(), part.begin(), part.end());
    }
    const size_t first = points.size();
    LinePoints(nullptr, -1, nullptr, -1, c.direction, o, CurvedSection, &c,
               max_depth, line, points, &pieces);
    if(bent && !c.traced) {
      /* a station's length is along the line, not its chord */
      const double step = 1e-4 * (o.hi - o.lo);
      for(size_t i = first; i < points.size(); i++) {
        const ksl_vec3_t at = c.curved_is_a ? points[i].point_a : points[i].point_b;
        const double s = Dot(Sub(at, c.origin), c.direction);
        ksl_vec3_t p0, p1, pb, normal;
        if(CurvedSection(&c, s - step, p0, pb, normal) != -HUGE_VAL &&
           CurvedSection(&c, s + step, p1, pb, normal) != -HUGE_VAL) {
          const ksl_vec3_t along = Sub(p1, p0);
          points[i].length *= ksl_vec3_l2norm(&along) / (2.0 * step);
        }
      }
    }
  }
  /* two curved faces find a line on a seam to within their normals */
  c.repeated = DropRepeated(points, before,
                            (c.other_side ? 1e-5 : 1e-9) * curved.shape->size);
  /* a section counts only where the faces oppose each other by kOpposed: as
  they turn away the contact goes smoothly, so that the end of a roller tipping
  over from its side to its end face has no jump at that angle */
  ksl_vec3_t pa, pb, normal;
  const double back = o.deepest > 0.5 * (s0 + s1) ? -inset : inset;
  if(CurvedSection(&c, o.deepest, pa, pb, normal) != -HUGE_VAL ||
     CurvedSection(&c, o.deepest + back, pa, pb, normal) != -HUGE_VAL) {
    const double opposed = (c.curved_is_a ? 1.0 : -1.0) *
                           Dot(SideNormal(curved, fc, c.uv), normal);
    const double share = SmoothStep((opposed - kOpposed) / 0.4);
    for(size_t i = before; i < points.size(); i++) {
      points[i].weight *= share;
    }
  }
  return true;
}

/* whether a surface may touch a cylinder or plane along a line */
static bool IsCurved(shift::core::cad::Surface::Type type) {
  return type == shift::core::cad::Surface::Type::kNurbs || type == shift::core::cad::Surface::Type::kExtrusion;
}

/* a curved face on a cylinder's or plane's face of the other geometry, from
the sample of the curved face deepest in it: see CurvedFaceLine. True, with
nothing done, if the faces are apart */
static bool CurvedFacePair(const shift::core::collision_geometry::CadSideOfPair& curved, int fc,
                           const shift::core::collision_geometry::CadSideOfPair& other, int fo, bool curved_is_a,
                           double max_depth, std::vector<shift::core::collision_geometry::ContactPoint>& points,
                           bool& repeated) {
  CurvedLine c;
  c.curved = &curved;
  c.face = fc;
  c.curved_is_a = curved_is_a;
  double scale;
  SurfaceAxis(other, fo, c.other_point, c.other_vector, scale);
  const auto& surface = SurfaceOf(other, fo);
  if(surface.type == shift::core::cad::Surface::Type::kCylinder) {
    c.radius = scale * surface.radius;
    c.convex = SameSense(other, fo);
  } else if(!SameSense(other, fo)) {
    ksl_vec3_scale(&c.other_vector, -1.0);
  }
  const auto& f = curved.shape->faces[fc];
  int best = -1;
  double deepest = -HUGE_VAL;
  for(size_t i = 0; i < f.sample_uv.size(); i++) {
    ksl_vec3_t q, outward;
    const double depth =
      OtherDepth(c, SidePosition(curved, f.sample_points[i]), q, outward);
    if(depth > deepest &&
       Dot(SideVector(curved, f.sample_normals[i]), outward) < -0.5) {
      deepest = depth;
      best = static_cast<int>(i);
    }
  }
  /* samples are a grid's spacing apart */
  const ksl_vec3_t diagonal = Sub(f.box.hi, f.box.lo);
  if(best < 0 || deepest < -max_depth - ksl_vec3_l2norm(&diagonal) / 8.0) {
    return best >= 0;
  }
  const bool line =
    CurvedFaceLine(c, f.sample_uv[best], &other, fo, max_depth, points);
  repeated = c.repeated;
  return line;
}

/* ------------------------------------------------------------------------ */
/* edges and vertices */

/* a point p of one side (feature) over a face of the other: its depth below
the face, negative above it, the face's point and outward normal; false if p is
not over the face, or deeper than max_depth. PointInFace: below it */
static bool PointOverFace(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                          const ksl_vec3_t& p, double max_depth, double& depth,
                          ksl_vec3_t& q, ksl_vec3_t& n, shift::core::cad::Vec2& uv) {
  /* not near the face: outside its bounds, enlarged by the deepest
  penetration */
  ksl_vec3_t own;
  ksl_product_dinvv(&side.d, &p, &own);
  const auto& box = side.shape->faces[face].box;
  for(int k = 0; k < 3; k++) {
    if(own.at[k] < box.lo.at[k] - max_depth ||
       own.at[k] > box.hi.at[k] + max_depth) {
      return false;
    }
  }
  if(!side.shape->Project(face, own, uv) || !side.shape->Inside(face, uv)) {
    return false;
  }
  q = SidePoint(side, face, uv);
  n = SideNormal(side, face, uv);
  const ksl_vec3_t d = Sub(p, q);
  depth = -Dot(d, n);
  /* a point beyond the bounds of a NURBS face projects to its edge, not
  along its normal: it is not over the face */
  const ksl_vec3_t lateral = Axpy(d, depth, n);
  return depth <= max_depth &&
         ksl_vec3_l2norm(&lateral) <= 0.05 * std::fabs(depth);
}

static bool PointInFace(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                        const ksl_vec3_t& p, double max_depth, double& depth,
                        ksl_vec3_t& q, ksl_vec3_t& n, shift::core::cad::Vec2& uv) {
  return PointOverFace(side, face, p, max_depth, depth, q, n, uv) &&
         depth > 0.0;
}

/* whether a face of a feature (edge or vertex) at its point p faces against
the normal n of the face it is inside: a feature pushing into a face does;
one merely behind a face of a hollow body (outside a bore, say) does not, nor
one whose faces are steep to it, which came round an edge of the face */
static bool Opposes(const shift::core::collision_geometry::CadSideOfPair& side, const std::vector<int>& faces,
                    const ksl_vec3_t& p, const ksl_vec3_t& n) {
  for(int f : faces) {
    shift::core::cad::Vec2 uv;
    if(SideProject(side, f, p, uv) && Dot(SideNormal(side, f, uv), n) < -0.5) {
      return true;
    }
  }
  return false;
}

/* the contact point of a feature point p of side feature against a face of
the other side, with the right sense: from a to b */
/* whether a vertex of a side is the deepest of it along -n, near it: no edge from
the vertex goes further that way */
static bool VertexDeepest(const shift::core::collision_geometry::CadSideOfPair& side, int vertex,
                          const ksl_vec3_t& n) {
  const shift::core::collision_geometry::CadContactShape& shape = *side.shape;
  const ksl_vec3_t& at = shape.vertices[vertex].point;
  ksl_vec3_t own;
  ksl_product_drinvv(&side.d.R, &n, &own);
  auto same = [&](const ksl_vec3_t& x) {
    const ksl_vec3_t d = Sub(x, at);
    return ksl_vec3_l2norm(&d) <= 1e-9 * shape.size;
  };
  for(int f : shape.vertices[vertex].faces) {
    for(int e : shape.faces[f].edges) {
      const auto& edge = shape.edges[e];
      const auto& topology = shape.model().edges[edge.edge];
      const auto& place = shape.placements[edge.placement];
      const ksl_vec3_t start = place.Apply(shape.model().vertices[topology.start].point);
      const ksl_vec3_t end = place.Apply(shape.model().vertices[topology.end].point);
      if(!same(start) && !same(end)) {
        continue;
      }
      /* the direction the edge leaves in, to its nearest sample or its other end */
      ksl_vec3_t next = same(start) ? end : start;
      ksl_vec3_t to_next = Sub(next, at);
      double nearest = same(next) ? HUGE_VAL : ksl_vec3_l2norm(&to_next);
      for(const ksl_vec3_t& sample : edge.sample_points) {
        const ksl_vec3_t d = Sub(sample, at);
        const double length = ksl_vec3_l2norm(&d);
        if(!same(sample) && length < nearest) {
          nearest = length;
          to_next = d;
        }
      }
      if(nearest < HUGE_VAL && Dot(to_next, own) < -1e-6 * nearest) {
        return false;
      }
    }
  }
  return true;
}

/* the share of its force that a point p of a feature keeps in a face, depth
below it at q, beside the faces over the face's sharp convex edges. Inside a
solid near such an edge, the way out is through a nearer face if the feature can
leave by it, as exit(normal) tells: then the depth here is far too deep, and the
point fades out as the other face gets to half its depth */
template <typename Exit>
static double NearerFaceShare(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                              const ksl_vec3_t& p, double depth, const ksl_vec3_t& q,
                              double max_depth, Exit exit) {
  const shift::core::collision_geometry::CadContactShape& shape = *side.shape;
  double share = 1.0;
  for(int e : shape.faces[face].edges) {
    if(!shape.edges[e].sharp) {
      continue;
    }
    for(int g : shape.edges[e].faces) {
      double other_depth;
      ksl_vec3_t other_q, n;
      shift::core::cad::Vec2 uv;
      if(g == face || !PointInFace(side, g, p, max_depth, other_depth, other_q, n, uv) ||
         other_depth >= depth ||
         Dot(Sub(q, other_q), n) >= 0.0 || // concave: both faces push
         !exit(n)) {
        continue;
      }
      share = std::min(share, SmoothStep(2.0 * other_depth / depth - 1.0));
    }
  }
  return share;
}

static shift::core::collision_geometry::ContactPoint FeaturePoint(bool feature_is_a, const ksl_vec3_t& p,
                                 const ksl_vec3_t& q, const ksl_vec3_t& n,
                                 double depth, double curvature) {
  shift::core::collision_geometry::ContactPoint c;
  c.depth = depth;
  c.curvature = curvature;
  if(feature_is_a) {
    c.point_a = p;
    c.point_b = q;
    c.normal = Scaled(n, -1.0);
  } else {
    c.point_a = q;
    c.point_b = p;
    c.normal = n;
  }
  return c;
}

/* the unit direction of an edge at its parameter t, in the frame of the
shape */
static ksl_vec3_t EdgeDirection(const shift::core::collision_geometry::CadContactShape& shape, int edge,
                                double t) {
  const auto& e = shape.edges[edge];
  const auto& topology = shape.model().edges[e.edge];
  const auto& curve = shape.model().curves[topology.curve];
  const auto& place = shape.placements[e.placement];
  const double lo = std::min(topology.range[0], topology.range[1]);
  const double hi = std::max(topology.range[0], topology.range[1]);
  const double h = 1e-6 * (hi - lo);
  const double t0 = std::max(t - h, lo);
  const double t1 = std::min(t + h, hi);
  ksl_vec3_t d = Sub(place.Apply(shift::core::cad::CurvePoint<3>(curve, t1)),
                     place.Apply(shift::core::cad::CurvePoint<3>(curve, t0)));
  ksl_vec3_normalize(&d);
  return d;
}

/* whether a point of a shape, on the faces given, is a corner: the sharp edges
of the faces that end there meet at an angle. Where two edges go on into each
other, as the halves of the rim of a round face, it is not: the rim holds its
contact (see EdgeFaceLine), and so it is at the seam of a closed edge */
static bool IsCorner(const shift::core::collision_geometry::CadContactShape& shape,
                     const std::vector<int>& faces, const ksl_vec3_t& point) {
  bool any = false;
  ksl_vec3_t first = {{0.0, 0.0, 0.0}};
  for(int f : faces) {
    for(int e : shape.faces[f].edges) {
      const auto& edge = shape.edges[e];
      const auto& topology = shape.model().edges[edge.edge];
      if(!edge.sharp || topology.start == topology.end) {
        continue;
      }
      const auto& place = shape.placements[edge.placement];
      for(int k = 0; k < 2; k++) {
        const int end = k == 0 ? topology.start : topology.end;
        const ksl_vec3_t d =
          Sub(point, place.Apply(shape.model().vertices[end].point));
        if(ksl_vec3_l2norm(&d) > 1e-9 * shape.size) {
          continue;
        }
        const ksl_vec3_t direction =
          EdgeDirection(shape, e, topology.range[k]);
        if(!any) {
          any = true;
          first = direction;
        } else if(std::fabs(Dot(first, direction)) < 1.0 - 1e-6) {
          return true;
        }
      }
    }
  }
  return false;
}

static void VertexFace(const shift::core::collision_geometry::CadSideOfPair& feature, int vertex,
                       const shift::core::collision_geometry::CadSideOfPair& other, int face,
                       bool feature_is_a, double max_depth, double weight,
                       std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  if(weight <= 0.0 ||
     !IsCorner(*feature.shape, feature.shape->vertices[vertex].faces,
               feature.shape->vertices[vertex].point)) {
    return;
  }
  const ksl_vec3_t p =
    SidePosition(feature, feature.shape->vertices[vertex].point);
  double depth;
  ksl_vec3_t q, n;
  shift::core::cad::Vec2 uv;
  if(PointInFace(other, face, p, max_depth, depth, q, n, uv) &&
     Opposes(feature, feature.shape->vertices[vertex].faces, p, n)) {
    points.push_back(FeaturePoint(
      feature_is_a, p, q, n, depth,
      1.0 / feature.edge_radius +
        other.shape->MeanCurvature(face, uv)));
    /* a corner leaves by a face if it is the deepest of its part along the
    normal: otherwise the rest of the part is still inside */
    points.back().weight =
      weight * NearerFaceShare(other, face, p, depth, q, max_depth,
                               [&](const ksl_vec3_t& normal) {
                                 return VertexDeepest(feature, vertex, normal);
                               });
  }
}

/*
A curved edge lying in a face, as the rim of a round face resting on a flat
one, or of a hole in it: a line of contact along the arc of the edge below the
face, about its deepest point t, which may be all of a closed edge, or reach the
ends of an open one. The arc is found from t both ways, in steps of 1/64 of the
edge, to the points where it leaves the face, by bisection; its slices, 4,
or one for each 30 degrees the edge turns through along the arc if more, each
carry 3 Gauss stations of Hertz line contact, with their shares of the length
of the arc. A single point,
the deepest, holds the overlap of an edge on one side of it only: an edge lying
flat has its deepest point anywhere along it, and a part on it rocks between
its edges. The line comes in as the arc grows from 1 to 4 times the Hertz width
of the edge at the deepest point, and as the angle the edge turns through along
it grows from 0.5 to 1 radian: a short arc of a round edge, as the rim of a
cylinder on its side, has its point. The deepest point's weight goes out as the
line comes in. False if the line carries all of the contact.
*/
static bool EdgeFaceLine(const shift::core::collision_geometry::CadSideOfPair& feature, int edge,
                         const shift::core::collision_geometry::CadSideOfPair& other, int face,
                         bool feature_is_a, double max_depth, double t,
                         double& weight,
                         std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& e = feature.shape->edges[edge];
  const auto& m = feature.shape->model();
  const auto& curve = m.curves[m.edges[e.edge].curve];
  /* the range of the edge's parameter, which may run either way */
  const double first = std::min(m.edges[e.edge].range[0], m.edges[e.edge].range[1]);
  const double last = std::max(m.edges[e.edge].range[0], m.edges[e.edge].range[1]);
  const double span = last - first;
  const auto& place = feature.shape->placements[e.placement];
  const bool closed = m.edges[e.edge].start == m.edges[e.edge].end;
  auto clamp = [&](double s) {
    return closed ? first + std::fmod(std::fmod(s - first, span) + span, span)
                  : std::min(std::max(s, first), last);
  };
  auto point_at = [&](double s) {
    s = clamp(s);
    return SidePosition(feature,
                        place.Apply(shift::core::cad::CurvePoint<3>(curve, s)));
  };
  struct Station {
    double depth = -HUGE_VAL;
    ksl_vec3_t p, q, n;
    shift::core::cad::Vec2 uv;
  };
  auto station = [&](double s) {
    Station r;
    r.p = point_at(s);
    if(!PointInFace(other, face, r.p, max_depth, r.depth, r.q, r.n, r.uv) ||
       !Opposes(feature, e.faces, r.p, r.n)) {
      r.depth = -HUGE_VAL;
    }
    return r;
  };
  const Station deepest = station(t);
  if(!(deepest.depth > 0.0)) {
    return true;
  }
  /* the ends of the arc below the face, of an open edge at most its ends */
  const double step = span / 64.0;
  auto end = [&](double direction) {
    double inside = t;
    for(int k = 1; k <= (closed ? 32 : 64); k++) {
      double s = t + direction * k * step;
      if(!closed && (s <= first || s >= last)) {
        s = direction < 0.0 ? first : last;
        if(station(s).depth > 0.0) {
          return s;
        }
      }
      if(!(station(s).depth > 0.0)) {
        double outside = s;
        for(int i = 0; i < 34; i++) {
          const double mid = 0.5 * (inside + outside);
          (station(mid).depth > 0.0 ? inside : outside) = mid;
        }
        return 0.5 * (inside + outside);
      }
      inside = s;
    }
    return t + direction * 0.5 * span; // all of the edge
  };
  const double lo = end(-1.0), hi = end(1.0);
  auto speed = [&](double s) {
    const double h = 1e-6 * span;
    double s0 = s - h, s1 = s + h;
    if(!closed) {
      s0 = std::max(s0, first);
      s1 = std::min(s1, last);
    }
    const ksl_vec3_t d = Sub(point_at(s1), point_at(s0));
    return ksl_vec3_l2norm(&d) / (s1 - s0);
  };
  const double arc = speed(t) * (hi - lo);
  const double width = std::sqrt(deepest.depth * feature.edge_radius);
  /* the angle the edge turns through along the arc, from its curvature at t:
  a short arc of a round edge (a rim of a cylinder on its side) is a point */
  const double h = 1e-3 * span;
  const double s0 = closed ? t - h : std::max(t - h, first);
  const double s1 = closed ? t + h : std::min(t + h, last);
  const ksl_vec3_t p0 = point_at(s0), pm = point_at(0.5 * (s0 + s1)),
                   p1 = point_at(s1);
  const ksl_vec3_t bend = Axpy(Sub(p0, pm), 1.0, Sub(p1, pm));
  const double chord = 0.5 * (s1 - s0) * speed(t);
  const double turned = arc * ksl_vec3_l2norm(&bend) / (chord * chord);
  const double line = SmoothStep((arc / width - 1.0) / 3.0) *
                      SmoothStep((turned - 0.5) / 0.5);
  if(line <= 0.0) {
    return true;
  }
  /* in slices of the arc each turning by at most 30 degrees, and at least
  four: a helical edge round a thread turns through whole turns */
  constexpr int kSamples = 64;
  double turning = 0.0;
  ksl_vec3_t last_tangent{{0.0, 0.0, 0.0}};
  for(int k = 0; k < kSamples; k++) {
    const double sa = lo + (hi - lo) * k / kSamples;
    const double sb = lo + (hi - lo) * (k + 1) / kSamples;
    ksl_vec3_t tangent = Sub(point_at(sb), point_at(sa));
    const double length = ksl_vec3_l2norm(&tangent);
    if(!(length > 0.0)) {
      continue;
    }
    ksl_vec3_scale(&tangent, 1.0 / length);
    if(k > 0) {
      turning += std::acos(
        std::min(1.0, std::max(-1.0, Dot(tangent, last_tangent))));
    }
    last_tangent = tangent;
  }
  const int slices =
    std::max(4, static_cast<int>(std::ceil(turning / (M_PI / 6.0) - 1e-6)));
  const double half = 0.5 * (hi - lo) / slices;
  for(int slice = 0; slice < slices; slice++) {
    const double center = lo + (2 * slice + 1) * half;
    for(int i = 0; i < 3; i++) {
      const double s = center + kStations[i] * half;
      const Station r = station(s);
      if(!(r.depth > 0.0)) {
        continue;
      }
      shift::core::collision_geometry::ContactPoint c = FeaturePoint(
        feature_is_a, r.p, r.q, r.n, r.depth,
        1.0 / feature.edge_radius + other.shape->MeanCurvature(face, r.uv));
      c.length = kStationWeights[i] * half * speed(s);
      c.weight = weight * line;
      points.push_back(c);
    }
  }
  weight *= 1.0 - line;
  return weight > 0.0;
}

/* the deepest point of an edge inside a face: the deepest of its samples,
refined by golden section search between its neighbors; ends of the edge
are left to its vertices, and so is an open edge lying flat on the face, whose
deepest point is no deeper than its ends. With bulge, a curved edge whose samples
are all above the face is searched too: it may dip into it between them */
static void EdgeFace(const shift::core::collision_geometry::CadSideOfPair& feature, int edge,
                     const shift::core::collision_geometry::CadSideOfPair& other, int face, bool feature_is_a,
                     double max_depth, bool bulge,
                     std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& e = feature.shape->edges[edge];
  if(e.sample_t.empty()) {
    return;
  }
  const auto& m = feature.shape->model();
  const auto& curve = m.curves[m.edges[e.edge].curve];
  /* a straight smooth edge has no corner, and what touches it touches the
  faces beside it: a seam of a cylinder sweeping past a plate lies flat on it,
  and would add a point at the end of the plate that comes and goes, and a seam
  of a cam across a curved face a point beside the faces' own */
  if(e.smooth && curve.type == shift::core::cad::Curve3::Type::kLine) {
    return;
  }
  const auto& place = feature.shape->placements[e.placement];
  auto point_at = [&](double t) {
    return SidePosition(feature,
                        place.Apply(shift::core::cad::CurvePoint<3>(curve, t)));
  };
  auto depth_at = [&](double t) {
    double depth;
    ksl_vec3_t q, n;
    shift::core::cad::Vec2 uv;
    return PointOverFace(other, face, point_at(t), max_depth, depth, q, n, uv)
             ? depth
             : -HUGE_VAL;
  };
  const size_t count = e.sample_t.size();
  int best = -1;
  double deepest = -HUGE_VAL;
  for(size_t i = 0; i < count; i++) {
    double depth;
    ksl_vec3_t q, n;
    shift::core::cad::Vec2 uv;
    if(PointOverFace(other, face, SidePosition(feature, e.sample_points[i]),
                     max_depth, depth, q, n, uv) &&
       depth > deepest) {
      deepest = depth;
      best = static_cast<int>(i);
    }
  }
  /* a curve can bulge into the face between samples still above it */
  const ksl_vec3_t extent_of = Sub(e.box.hi, e.box.lo);
  if(best < 0 ||
     (deepest <= 0.0 &&
      (!bulge || curve.type == shift::core::cad::Curve3::Type::kLine ||
       deepest < -0.01 * ksl_vec3_l2norm(&extent_of)))) {
    return;
  }
  {
    /* only an edge pushing into the face is refined */
    double depth;
    ksl_vec3_t q, n;
    shift::core::cad::Vec2 uv;
    const ksl_vec3_t p = SidePosition(feature, e.sample_points[best]);
    if(!PointOverFace(other, face, p, max_depth, depth, q, n, uv) ||
       !Opposes(feature, e.faces, p, n)) {
      return;
    }
  }
  const auto& range = m.edges[e.edge].range;
  double lo = best > 0 ? e.sample_t[best - 1] : range[0];
  double hi = best + 1 < static_cast<int>(count) ? e.sample_t[best + 1]
                                                 : range[1];
  const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
  double t1 = hi - golden * (hi - lo), t2 = lo + golden * (hi - lo);
  double d1 = depth_at(t1), d2 = depth_at(t2);
  for(int i = 0; i < 30; i++) {
    if(d1 > d2) {
      hi = t2;
      t2 = t1;
      d2 = d1;
      t1 = hi - golden * (hi - lo);
      d1 = depth_at(t1);
    } else {
      lo = t1;
      t1 = t2;
      d1 = d2;
      t2 = lo + golden * (hi - lo);
      d2 = depth_at(t2);
    }
  }
  double t = 0.5 * (lo + hi);
  const double extent = std::fabs(range[1] - range[0]);
  const auto& topology = m.edges[e.edge];
  const bool curved = curve.type != shift::core::cad::Curve3::Type::kLine;
  /* the ends of an open edge that are corners, which hold their contact */
  bool corner[2] = {false, false};
  for(int k = 0; k < 2 && topology.start != topology.end; k++) {
    corner[k] = IsCorner(*feature.shape, e.faces,
                         place.Apply(m.vertices[k == 0 ? topology.start : topology.end].point));
  }
  for(int k = 0; k < 2; k++) {
    if(std::fabs(t - range[k]) >= 0.01 * extent) {
      continue;
    }
    /* at a vertex, unless the edge goes on into another there, or is closed */
    if(corner[k]) {
      return;
    }
    t = range[k];
  }
  /* a point between ends that also touch comes in as it gets deeper than them,
  so that a rim leaving a face flat has no jump */
  double weight = 1.0;
  if(corner[0] || corner[1]) {
    const double ends = std::max(corner[0] ? depth_at(range[0]) : -HUGE_VAL,
                                 corner[1] ? depth_at(range[1]) : -HUGE_VAL);
    const double middle = depth_at(t);
    if(ends > 0.0) {
      weight = SmoothStep((middle - ends) / (0.1 * middle));
    }
    if(weight <= 0.0) {
      return;
    }
  }
  if(curved &&
     !EdgeFaceLine(feature, edge, other, face, feature_is_a, max_depth, t,
                   weight, points)) {
    return;
  }
  const ksl_vec3_t p = point_at(t);
  double depth;
  ksl_vec3_t q, n;
  shift::core::cad::Vec2 uv;
  if(PointInFace(other, face, p, max_depth, depth, q, n, uv) &&
     Opposes(feature, e.faces, p, n)) {
    points.push_back(FeaturePoint(
      feature_is_a, p, q, n, depth,
      1.0 / feature.edge_radius +
        other.shape->MeanCurvature(face, uv)));
    points.back().weight = weight;
    /* along the edge the depth is still changing: stopped by the end of the face */
    const double step = 1e-4 * extent;
    const ksl_vec3_t along = Sub(point_at(t + step), point_at(t - step));
    if(std::fabs(Dot(along, n)) > 1e-3 * ksl_vec3_l2norm(&along)) {
      points.back().weight *= NearerFaceShare(other, face, p, depth, q, max_depth,
                                              [](const ksl_vec3_t&) { return true; });
    }
  }
}

/* adds c unless the points already hold it, as at a seam between faces */
static bool AddDistinct(std::vector<shift::core::collision_geometry::ContactPoint>& points,
                        const shift::core::collision_geometry::ContactPoint& c, double size) {
  for(const shift::core::collision_geometry::ContactPoint& other : points) {
    const ksl_vec3_t d = Sub(c.point_a, other.point_a);
    if(ksl_vec3_l2norm(&d) <= 1e-7 * size && Dot(c.normal, other.normal) > 0.999) {
      return false;
    }
  }
  points.push_back(c);
  return true;
}

/* the point and unit tangent of an edge at the parameter t */
static ksl_vec3_t EdgePoint(const shift::core::collision_geometry::CadSideOfPair& side, int edge, double t) {
  const auto& e = side.shape->edges[edge];
  const auto& m = side.shape->model();
  return SidePosition(side, side.shape->placements[e.placement].Apply(
                              shift::core::cad::CurvePoint<3>(
                                m.curves[m.edges[e.edge].curve], t)));
}

static ksl_vec3_t EdgeTangent(const shift::core::collision_geometry::CadSideOfPair& side, int edge, double t) {
  const auto& range = side.shape->model().edges[side.shape->edges[edge].edge].range;
  const double step = 1e-5 * (range[1] - range[0]);
  const double lo = std::max(std::min(range[0], range[1]), t - std::fabs(step));
  const double hi = std::min(std::max(range[0], range[1]), t + std::fabs(step));
  ksl_vec3_t d = Sub(EdgePoint(side, edge, hi), EdgePoint(side, edge, lo));
  const double length = ksl_vec3_l2norm(&d);
  return length > 0.0 ? Scaled(d, 1.0 / length) : d;
}

/* whether the unit vector n is among the outward normals of the two faces of a
sharp edge at the point p: between them, in the plane across the edge */
static bool InNormalCone(const shift::core::collision_geometry::CadSideOfPair& side, int edge,
                         const ksl_vec3_t& p, const ksl_vec3_t& n) {
  const auto& faces = side.shape->edges[edge].faces;
  shift::core::cad::Vec2 uv0, uv1;
  if(!SideProject(side, faces[0], p, uv0) ||
     !SideProject(side, faces[1], p, uv1)) {
    return false;
  }
  const ksl_vec3_t n0 = SideNormal(side, faces[0], uv0);
  const ksl_vec3_t n1 = SideNormal(side, faces[1], uv1);
  ksl_vec3_t across, from0, to1;
  ksl_cross_vv(&n0, &n1, &across);
  ksl_cross_vv(&n0, &n, &from0);
  ksl_cross_vv(&n, &n1, &to1);
  return Dot(from0, across) > 0.0 && Dot(to1, across) > 0.0;
}

/* the parameters of the closest points of two edges near (s, t), by Gauss-Newton
steps on the distance between them, halved until it falls and kept to the edges'
ranges (a closed edge goes round); false if it stops at an end, which is left to
the vertices */
static bool ClosestOnEdges(const shift::core::collision_geometry::CadSideOfPair& a, int ea,
                           const shift::core::collision_geometry::CadSideOfPair& b, int eb, double& s,
                           double& t) {
  const auto& topology_a = a.shape->model().edges[a.shape->edges[ea].edge];
  const auto& topology_b = b.shape->model().edges[b.shape->edges[eb].edge];
  const double la = std::min(topology_a.range[0], topology_a.range[1]);
  const double ha = std::max(topology_a.range[0], topology_a.range[1]);
  const double lb = std::min(topology_b.range[0], topology_b.range[1]);
  const double hb = std::max(topology_b.range[0], topology_b.range[1]);
  const bool closed_a = topology_a.start == topology_a.end;
  const bool closed_b = topology_b.start == topology_b.end;
  auto keep = [](double x, double lo, double hi, bool closed) {
    if(closed) {
      return lo + std::fmod(std::fmod(x - lo, hi - lo) + (hi - lo), hi - lo);
    }
    return std::min(std::max(x, lo), hi);
  };
  auto distance = [&](double x, double y) {
    const ksl_vec3_t r = Sub(EdgePoint(a, ea, x), EdgePoint(b, eb, y));
    return Dot(r, r);
  };
  const double ds = 1e-5 * (ha - la), dt = 1e-5 * (hb - lb);
  double current = distance(s, t);
  for(int i = 0; i < 30; i++) {
    /* derivatives of the points by their parameters */
    const double s0 = std::max(s - ds, la), s1 = std::min(s + ds, ha);
    const double t0 = std::max(t - dt, lb), t1 = std::min(t + dt, hb);
    const ksl_vec3_t va = Scaled(Sub(EdgePoint(a, ea, s1), EdgePoint(a, ea, s0)),
                                 1.0 / (s1 - s0));
    const ksl_vec3_t vb = Scaled(Sub(EdgePoint(b, eb, t1), EdgePoint(b, eb, t0)),
                                 1.0 / (t1 - t0));
    const ksl_vec3_t r = Sub(EdgePoint(a, ea, s), EdgePoint(b, eb, t));
    const double aa = Dot(va, va), ab = Dot(va, vb), bb = Dot(vb, vb);
    const double det = aa * bb - ab * ab;
    if(!(det > 1e-12 * aa * bb)) {
      return false; // parallel
    }
    const double ga = Dot(r, va), gb = Dot(r, vb);
    const double step_s = (-bb * ga + ab * gb) / det;
    const double step_t = (aa * gb - ab * ga) / det;
    if(std::fabs(step_s) < 1e-10 * (ha - la) &&
       std::fabs(step_t) < 1e-10 * (hb - lb)) {
      break;
    }
    bool fell = false;
    for(double share = 1.0; share > 1e-3 && !fell; share *= 0.5) {
      const double x = keep(s + share * step_s, la, ha, closed_a);
      const double y = keep(t + share * step_t, lb, hb, closed_b);
      const double next = distance(x, y);
      if(next < current) {
        s = x;
        t = y;
        current = next;
        fell = true;
      }
    }
    if(!fell) {
      break;
    }
  }
  const double end_a = 1e-6 * (ha - la), end_b = 1e-6 * (hb - lb);
  return (closed_a || (s > la + end_a && s < ha - end_a)) &&
         (closed_b || (t > lb + end_b && t < hb - end_b));
}

/* the distance of p, in the shape's coordinates, from an edge of the shape */
static double DistanceToEdge(const shift::core::collision_geometry::CadContactShape& shape,
                             int e, const ksl_vec3_t& p) {
  const auto& m = shape.model();
  const auto& edge = shape.edges[e];
  const auto& topology = m.edges[edge.edge];
  const auto& place = shape.placements[edge.placement];
  double nearest = HUGE_VAL;
  auto to_segment = [&](const ksl_vec3_t& x, const ksl_vec3_t& y) {
    const ksl_vec3_t d = Sub(y, x);
    const double length = Dot(d, d);
    const double t =
      length > 0.0 ? std::min(std::max(Dot(Sub(p, x), d) / length, 0.0), 1.0)
                   : 0.0;
    const ksl_vec3_t gap = Sub(p, Axpy(x, t, d));
    nearest = std::min(nearest, ksl_vec3_l2norm(&gap));
  };
  ksl_vec3_t last = place.Apply(m.vertices[topology.start].point);
  for(const ksl_vec3_t& sample : edge.sample_points) {
    to_segment(last, sample);
    last = sample;
  }
  to_segment(last, place.Apply(m.vertices[topology.end].point));
  return nearest;
}

/* a corner in a face, giving way to the crossing of one of its edges with an
edge of that face */
struct CornerFade {
  bool on_a;
  int vertex;
  int face;    //!< of the other side
  double keep; //!< the share of its force the corner keeps
};

/* the share of its force that a crossing of edge with other_edge takes from a
corner at the near end of edge that is still in a face of other_edge, and adds
that corner to fades. The same contact is seen twice from where the crossing first
appears, at the corner, to where the corner leaves the face over other_edge,
and neither may jump: with the crossing u from the corner along the edge, and
the corner x from other_edge in the face, the crossing's share goes from 0 at
u = 0 to 1 at x = 0. p is the crossing, and it has a share of 1 if no corner
is in a face */
static double CrossingShare(const shift::core::collision_geometry::CadSideOfPair& feature, int edge,
                            const shift::core::collision_geometry::CadSideOfPair& other, int other_edge,
                            bool on_a, const ksl_vec3_t& p, double crossing_depth,
                            double max_depth, std::vector<CornerFade>& fades) {
  const shift::core::collision_geometry::CadContactShape& shape = *feature.shape;
  const auto& e = shape.edges[edge];
  const auto& topology = shape.model().edges[e.edge];
  const auto& place = shape.placements[e.placement];
  double share = 1.0;
  if(topology.start == topology.end) {
    return share;
  }
  /* the end of the edge nearer the crossing: a far corner has no part in it */
  const ksl_vec3_t to_start =
    Sub(SidePosition(feature, place.Apply(shape.model().vertices[topology.start].point)), p);
  const ksl_vec3_t to_end =
    Sub(SidePosition(feature, place.Apply(shape.model().vertices[topology.end].point)), p);
  for(int end : {ksl_vec3_l2norm(&to_start) < ksl_vec3_l2norm(&to_end) ? topology.start
                                                                       : topology.end}) {
    const ksl_vec3_t at = place.Apply(shape.model().vertices[end].point);
    int vertex = -1;
    for(int f : e.faces) {
      for(int v : shape.faces[f].vertices) {
        const ksl_vec3_t d = Sub(shape.vertices[v].point, at);
        if(vertex < 0 && ksl_vec3_l2norm(&d) <= 1e-9 * shape.size) {
          vertex = v;
        }
      }
    }
    if(vertex < 0) {
      continue;
    }
    const ksl_vec3_t corner = SidePosition(feature, shape.vertices[vertex].point);
    /* in both faces of other_edge it leaves them at once: the depth in the
    face it leaves over x = 0 goes to nothing by itself in the other */
    double corner_taken = -1.0;
    /* no deeper than the corner it takes over from, in a face it can leave by, and
    half as deep again at most: past that it is measured through the wrong face */
    double corner_depth = HUGE_VAL;
    for(int f : other.shape->edges[other_edge].faces) {
      double depth;
      ksl_vec3_t q, n;
      shift::core::cad::Vec2 uv;
      if(PointInFace(other, f, corner, max_depth, depth, q, n, uv) &&
         VertexDeepest(feature, vertex, n)) {
        corner_depth = std::min(corner_depth, depth);
      }
    }
    for(int f : other.shape->edges[other_edge].faces) {
      double depth;
      ksl_vec3_t q, n, own;
      shift::core::cad::Vec2 uv;
      if(!PointInFace(other, f, corner, max_depth, depth, q, n, uv)) {
        fades.push_back({on_a, vertex, f, 0.0}); // on the edge, to rounding
        continue;
      }
      if(!Opposes(feature, shape.vertices[vertex].faces, corner, n)) {
        continue; // no contact of the corner to give way
      }
      ksl_product_dinvv(&other.d, &q, &own);
      const ksl_vec3_t along = Sub(corner, p);
      const double u = ksl_vec3_l2norm(&along);
      const double x = DistanceToEdge(*other.shape, other_edge, own);
      const double taken =
        (u + x > 0.0 ? SmoothStep(u / (u + x)) : 1.0) *
        (corner_depth < HUGE_VAL
           ? SmoothStep(2.0 * corner_depth / std::max(crossing_depth, corner_depth) - 1.0)
           : 1.0);
      fades.push_back({on_a, vertex, f, 1.0 - taken});
      corner_taken = std::max(corner_taken, taken);
    }
    if(corner_taken >= 0.0) {
      share = std::min(share, corner_taken);
    }
  }
  return share;
}

/* the points where two sharp edges cross; true if they touch */
static bool EdgeEdge(const shift::core::collision_geometry::CadSideOfPair& a, int ea, const shift::core::collision_geometry::CadSideOfPair& b,
                     int eb, double max_depth, std::vector<CornerFade>& fades,
                     std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& edge_a = a.shape->edges[ea];
  const auto& edge_b = b.shape->edges[eb];
  /* from the local minima of the distance along the samples of one, to the
  closest sample of the other: a curve can pass another twice */
  std::vector<double> distance(edge_a.sample_t.size());
  std::vector<int> nearest(edge_a.sample_t.size(), 0);
  for(size_t i = 0; i < edge_a.sample_t.size(); i++) {
    const ksl_vec3_t p = SidePosition(a, edge_a.sample_points[i]);
    distance[i] = HUGE_VAL;
    for(size_t j = 0; j < edge_b.sample_t.size(); j++) {
      const ksl_vec3_t d = Sub(p, SidePosition(b, edge_b.sample_points[j]));
      const double length = Dot(d, d);
      if(length < distance[i]) {
        distance[i] = length;
        nearest[i] = static_cast<int>(j);
      }
    }
  }
  bool crossed = false;
  for(size_t i = 0; i < distance.size(); i++) {
    if((i > 0 && distance[i - 1] < distance[i]) ||
       (i + 1 < distance.size() && distance[i + 1] <= distance[i])) {
      continue;
    }
    double s = edge_a.sample_t[i], t = edge_b.sample_t[nearest[i]];
    if(!ClosestOnEdges(a, ea, b, eb, s, t)) {
      continue;
    }
    const ksl_vec3_t pa = EdgePoint(a, ea, s), pb = EdgePoint(b, eb, t);
    const ksl_vec3_t ta = EdgeTangent(a, ea, s), tb = EdgeTangent(b, eb, t);
    ksl_vec3_t n;
    ksl_cross_vv(&ta, &tb, &n);
    const double sine = ksl_vec3_l2norm(&n);
    if(sine < 0.02) {
      continue; // nearly parallel: they lie along each other, as on a face
    }
    n = Scaled(n, 1.0 / sine);
    if(!InNormalCone(a, ea, pa, n)) {
      n = Scaled(n, -1.0);
    }
    const ksl_vec3_t gap = Sub(pa, pb);
    const double depth = Dot(gap, n);
    const ksl_vec3_t lateral = Axpy(gap, -depth, n);
    if(depth > 0.0 && Within(pa, pb, depth, max_depth) &&
       ksl_vec3_l2norm(&lateral) <= 1e-3 * depth &&
       InNormalCone(a, ea, pa, n) &&
       InNormalCone(b, eb, pb, Scaled(n, -1.0))) {
      shift::core::collision_geometry::ContactPoint c =
        FeaturePoint(false, pb, pa, n, depth,
                     0.5 * (1.0 / a.edge_radius + 1.0 / b.edge_radius));
      c.weight = std::min(CrossingShare(a, ea, b, eb, true, pa, depth, max_depth, fades),
                          CrossingShare(b, eb, a, ea, false, pb, depth, max_depth, fades));
      crossed = true;
      if(c.weight > 0.0) {
        AddDistinct(points, c, a.shape->size);
      }
    }
  }
  return crossed;
}

/* a straight edge along a cylinder: the line of the edge and the cylinder's axis,
in a's frame */
struct EdgeCylinder {
  ksl_vec3_t origin, direction; //!< of the edge
  ksl_vec3_t oc, dc;            //!< the cylinder's axis
  double rc;
  bool edge_is_a;
};

/* the cross section at the station s along the edge: its point inside the
cylinder, and the point of the cylinder straight out from the axis */
static double EdgeCylinderSection(const void* data, double s, ksl_vec3_t& pa,
                                  ksl_vec3_t& pb, ksl_vec3_t& n) {
  const EdgeCylinder& c = *static_cast<const EdgeCylinder*>(data);
  const ksl_vec3_t pe = Axpy(c.origin, s, c.direction);
  const ksl_vec3_t from_axis = Sub(pe, c.oc);
  const ksl_vec3_t radial = Axpy(from_axis, -Dot(from_axis, c.dc), c.dc);
  const double distance = ksl_vec3_l2norm(&radial);
  if(!(distance > 0.0)) {
    return -HUGE_VAL;
  }
  const ksl_vec3_t outward = Scaled(radial, 1.0 / distance);
  const double depth = c.rc - distance;
  const ksl_vec3_t pc = Axpy(pe, depth, outward);
  if(c.edge_is_a) {
    pa = pe;
    pb = pc;
    n = Scaled(outward, -1.0);
  } else {
    pa = pc;
    pb = pe;
    n = outward;
  }
  return depth;
}

/*
A straight edge lying along a convex cylinder touches it along a line, as the
cylinder touches the face next to the edge, so that a cylinder rolling over the
edge of a face keeps its line of contact: stations of Hertz line contact, sliced
over the length below the surface. The line fades out as the edge turns from the
axis, between sines of 0.05 and 0.1 of the angle, and where it is shorter than a
few times the Hertz width of the edge. True if it carries all of the contact: the
edge's point and its vertices then have nothing more to add.
*/
static bool EdgeCylinderLine(const shift::core::collision_geometry::CadSideOfPair& feature, int edge,
                             const shift::core::collision_geometry::CadSideOfPair& cylinder, int fc,
                             bool edge_is_a, double max_depth,
                             std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& e = feature.shape->edges[edge];
  const auto& m = feature.shape->model();
  const auto& topology = m.edges[e.edge];
  if(m.curves[topology.curve].type != shift::core::cad::Curve3::Type::kLine ||
     !SameSense(cylinder, fc)) {
    return false;
  }
  EdgeCylinder c;
  c.edge_is_a = edge_is_a;
  c.origin = EdgePoint(feature, edge, topology.range[0]);
  const ksl_vec3_t along =
    Sub(EdgePoint(feature, edge, topology.range[1]), c.origin);
  const double length = ksl_vec3_l2norm(&along);
  if(!(length > 0.0)) {
    return false;
  }
  c.direction = Scaled(along, 1.0 / length);
  double scale;
  SurfaceAxis(cylinder, fc, c.oc, c.dc, scale);
  c.rc = scale * SurfaceOf(cylinder, fc).radius;
  ksl_vec3_t cross;
  ksl_cross_vv(&c.direction, &c.dc, &cross);
  const double sine = ksl_vec3_l2norm(&cross);
  if(sine >= 0.1) {
    return false;
  }
  LineOverlap o;
  if(!Overlap(EdgeCylinderSection, &c, 0.0, length, true, o)) {
    return false;
  }
  /* only an edge the cylinder has come onto from a face next to it: that face
  looks at the cylinder. Other edges inside it are parts passing it */
  ksl_vec3_t pa, pb, n;
  EdgeCylinderSection(&c, o.deepest, pa, pb, n);
  const ksl_vec3_t& on_edge = edge_is_a ? pa : pb;
  const ksl_vec3_t outward = edge_is_a ? Scaled(n, -1.0) : n;
  bool facing = false;
  for(int f : e.faces) {
    shift::core::cad::Vec2 uv;
    facing = facing || (SideProject(feature, f, on_edge, uv) &&
                        Dot(SideNormal(feature, f, uv), outward) < -0.9);
  }
  if(!facing) {
    return false;
  }
  const double width = std::sqrt(o.depth * feature.edge_radius);
  const double weight = SmoothStep(((o.hi - o.lo) / width - 1.0) / 3.0) *
                        (1.0 - SmoothStep((sine - 0.05) / 0.05));
  if(weight <= 0.0) {
    return false;
  }
  const size_t before = points.size();
  LinePoints(edge_is_a ? nullptr : &cylinder, edge_is_a ? -1 : fc,
             edge_is_a ? &cylinder : nullptr, edge_is_a ? fc : -1, c.direction,
             o, EdgeCylinderSection, &c, max_depth, weight, points);
  /* each station on the cylinder's face */
  const auto off_face = [&](const shift::core::collision_geometry::ContactPoint& point) {
    shift::core::cad::Vec2 uv;
    return !SideProject(cylinder, fc, edge_is_a ? point.point_b : point.point_a,
                        uv) ||
           !InsideOrOnEdge(cylinder, fc, uv);
  };
  points.erase(std::remove_if(points.begin() + before, points.end(), off_face),
               points.end());
  return weight >= 1.0 && points.size() > before;
}

/* ------------------------------------------------------------------------ */
/* pairs of faces */

/* the sample of a face deepest under another that it opposes, and the
parameters of the point of the other above it; false if none is. Only samples
within a grid's spacing of one of the other's are looked at, from that one */
static bool DeepestSample(const shift::core::collision_geometry::CadSideOfPair& side,
                          int face,
                          const shift::core::collision_geometry::CadSideOfPair& other,
                          int fo, double max_depth, shift::core::cad::Vec2& uv,
                          shift::core::cad::Vec2& other_uv) {
  const auto& f = side.shape->faces[face];
  const auto& g = other.shape->faces[fo];
  auto spacing = [](const shift::core::collision_geometry::CadContactFace& x) {
    const ksl_vec3_t d = Sub(x.box.hi, x.box.lo);
    return ksl_vec3_l2norm(&d) / 8.0;
  };
  const double reach = max_depth + spacing(f) + spacing(g);
  double best = -HUGE_VAL;
  for(size_t i = 0; i < f.sample_uv.size(); i++) {
    const ksl_vec3_t p = SidePosition(side, f.sample_points[i]);
    const ksl_vec3_t normal = SideVector(side, f.sample_normals[i]);
    double nearest = reach * reach;
    int guess = -1;
    for(size_t j = 0; j < g.sample_uv.size(); j++) {
      const ksl_vec3_t d = Sub(p, SidePosition(other, g.sample_points[j]));
      const double squared = Dot(d, d);
      if(squared < nearest &&
         Dot(normal, SideVector(other, g.sample_normals[j])) < -0.5) {
        nearest = squared;
        guess = static_cast<int>(j);
      }
    }
    if(guess < 0) {
      continue;
    }
    shift::core::cad::Vec2 at;
    ksl_vec3_t q, outward;
    bool beyond = false;
    const double depth =
      FaceDepth(other, fo, p, &g.sample_uv[guess], at, q, outward, beyond);
    if(!beyond && depth > best && depth <= max_depth &&
       Dot(normal, outward) < -0.5) {
      best = depth;
      uv = f.sample_uv[i];
      other_uv = at;
    }
  }
  return best != -HUGE_VAL;
}

/* adds the line of two curved faces, from the point x of each, or if there is
none from the sample of either deepest under the other; true if the faces
touch along a line or do not touch, false if they may touch at a point */
static bool CurvedCurvedLine(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                             const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                             const double* x, double max_depth,
                             std::vector<shift::core::collision_geometry::ContactPoint>& points,
                             bool& repeated) {
  CurvedLine line;
  shift::core::cad::Vec2 seed;
  if(x || DeepestSample(a, fa, b, fb, max_depth, seed, line.other_uv)) {
    if(x) {
      seed = {x[0], x[1]};
      line.other_uv = {x[2], x[3]};
    }
    line.curved = &a;
    line.face = fa;
    line.other_side = &b;
    line.other_face = fb;
  } else if(DeepestSample(b, fb, a, fa, max_depth, seed, line.other_uv)) {
    line.curved = &b;
    line.face = fb;
    line.curved_is_a = false;
    line.other_side = &a;
    line.other_face = fa;
  } else {
    return false;
  }
  const bool found = CurvedFaceLine(line, seed, line.other_side, line.other_face,
                                    max_depth, points);
  repeated = line.repeated;
  return found;
}

/* climbs from the seed x of two faces to the deepest point near it of a's face
under b's: steepest ascent along a's face on the depth of its point under b's
(FaceDepth), a step taken only if the depth grows, from a step of a sample's
spacing, doubled after each step taken and halved after each refused. Newton's method on CommonNormal from a seed between two
regions of contact can end at the stationary point between them, where the
faces are apart; from the end of the climb it ends at the deepest point of the
region the seed is in. Sets x and climbed, the depth reached. False if a's
point at the seed has no depth under b's face */
static bool Climb(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                  const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                  double* x, double& climbed, double* reached = nullptr) {
  const auto& face = a.shape->faces[fa];
  const double cell[2] = {
    (face.hi[0] - face.lo[0]) / std::max(face.sample_nu, 1),
    (face.hi[1] - face.lo[1]) / std::max(face.sample_nv, 1)};
  const bool periodic[2] = {face.periodic_u, face.periodic_v};
  shift::core::cad::Vec2 uvb{x[2], x[3]};
  auto depth_at = [&](const shift::core::cad::Vec2& uva,
                      shift::core::cad::Vec2& at) {
    ksl_vec3_t q, outward;
    bool beyond = false;
    const double depth =
      FaceDepth(b, fb, SidePoint(a, fa, uva), &uvb, at, q, outward, beyond);
    return beyond ? -HUGE_VAL : depth;
  };
  auto bounded = [&](int k, double value) {
    return periodic[k] ? value
                       : std::min(std::max(value, face.lo[k]), face.hi[k]);
  };
  shift::core::cad::Vec2 uva{x[0], x[1]}, at;
  double depth = depth_at(uva, at);
  if(depth == -HUGE_VAL) {
    return false;
  }
  uvb = at;
  ksl_vec3_t du, dv;
  SidePoint(a, fa, uva, &du, &dv);
  const double spacing = std::min(ksl_vec3_l2norm(&du) * cell[0],
                                  ksl_vec3_l2norm(&dv) * cell[1]);
  double step = spacing;
  for(int iteration = 0; iteration < 40 && step > 1e-3 * spacing;
      iteration++) {
    double gradient[2];
    for(int k = 0; k < 2; k++) {
      const double h = 1e-3 * cell[k];
      shift::core::cad::Vec2 plus = uva, minus = uva, ignored;
      plus[k] = bounded(k, uva[k] + h);
      minus[k] = bounded(k, uva[k] - h);
      gradient[k] = (depth_at(plus, ignored) - depth_at(minus, ignored)) /
                    (plus[k] - minus[k]);
    }
    if(!std::isfinite(gradient[0]) || !std::isfinite(gradient[1])) {
      break;
    }
    /* the steepest ascent along the face: the gradient through the inverse
    of the face's metric */
    SidePoint(a, fa, uva, &du, &dv);
    const double g11 = Dot(du, du), g12 = Dot(du, dv), g22 = Dot(dv, dv);
    const double det = g11 * g22 - g12 * g12;
    if(!(det > 0.0)) {
      break;
    }
    const double d[2] = {(g22 * gradient[0] - g12 * gradient[1]) / det,
                         (g11 * gradient[1] - g12 * gradient[0]) / det};
    const double length = std::sqrt(g11 * d[0] * d[0] + 2.0 * g12 * d[0] * d[1] +
                                    g22 * d[1] * d[1]);
    if(!(length > 0.0)) {
      break;
    }
    const shift::core::cad::Vec2 next{bounded(0, uva[0] + step * d[0] / length),
                                      bounded(1, uva[1] + step * d[1] / length)};
    const double deeper = depth_at(next, at);
    if(deeper > depth) {
      uva = next;
      uvb = at;
      depth = deeper;
      step = std::min(2.0 * step, 2.0 * spacing);
    } else {
      step *= 0.5;
    }
  }
  x[0] = uva[0];
  x[1] = uva[1];
  x[2] = uvb[0];
  x[3] = uvb[1];
  /* the depth reached, or -HUGE_VAL if the faces are apart there: the
  search from it would end at no point of contact */
  climbed = depth > -1e-2 * spacing ? depth : -HUGE_VAL;
  if(reached) {
    *reached = depth;
  }
  return true;
}

/* whether a surface is one of revolution about an axis: the stations of a ring
of contact are taken about it */
static bool HasAxis(shift::core::cad::Surface::Type type) {
  return type == shift::core::cad::Surface::Type::kCone ||
         type == shift::core::cad::Surface::Type::kCylinder ||
         type == shift::core::cad::Surface::Type::kTorus;
}

/* whether a surface is one of revolution: a sphere is one about any line
through its center */
static bool OfRevolution(shift::core::cad::Surface::Type type) {
  return HasAxis(type) || type == shift::core::cad::Surface::Type::kSphere;
}

/* adds the ring of two faces of revolution touching along a circle about the
axis of one, as a ball does in a conical seat or a ring on a cone, from their
deepest point x: the gap there is straight round the axis. True if they touch
along it or do not touch, false if they may touch at a point */
static bool RingLine(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                     const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                     const double* x, double max_depth,
                     std::vector<shift::core::collision_geometry::ContactPoint>& points,
                     bool& repeated) {
  const bool on_a = HasAxis(SurfaceOf(a, fa).type);
  CurvedLine c;
  c.ring = true;
  c.curved = on_a ? &a : &b;
  c.face = on_a ? fa : fb;
  c.curved_is_a = on_a;
  c.other_side = on_a ? &b : &a;
  c.other_face = on_a ? fb : fa;
  c.other_uv = on_a ? shift::core::cad::Vec2{x[2], x[3]}
                    : shift::core::cad::Vec2{x[0], x[1]};
  double scale;
  SurfaceAxis(*c.curved, c.face, c.axis_point, c.axis, scale);
  const shift::core::cad::Vec2 seed = on_a ? shift::core::cad::Vec2{x[0], x[1]}
                                           : shift::core::cad::Vec2{x[2], x[3]};
  const bool found =
    CurvedFaceLine(c, seed, c.other_side, c.other_face, max_depth, points);
  repeated = c.repeated;
  return found;
}

/* ------------------------------------------------------------------------ */
/* conformal faces: of the same shape, touching over an area */

/* the residual of the fit of Conforms, as a share of the greatest depth */
static constexpr double kConformal = 1e-3;

/* the fewest rows of samples along each parameter of the fit of Conforms */
static constexpr int kConformalRows = 5;

/* the most points of the grid over a face on which AreaContact integrates */
static constexpr int kAreaPoints = 1024;

/* nearly conformal faces (see Conforms): analytic surfaces whose dimensions
differ by this share at most, and faces whose depths are off the fit of a
rigid motion by this share of the greatest depth sought (max_depth) at most */
static constexpr double kNearShape = 0.1;
static constexpr double kNearGap = 0.25;

/* the half angles over which the contact of nearly conformal faces wraps
round them (by the spread of the normals of its area) from which their area
contact begins to take over from the contact of a point or a line, and from
which it is the whole contact */
static constexpr double kWrapLeast = 0.3;
static constexpr double kWrapFull = 0.6;

/* the cosines of the angle between the normals of two flat faces, less than
pi, from which their area contact (see ConformalPair) begins to take over from
that of their edges and vertices, and from which it is the whole contact */
static constexpr double kFacingLeast = 0.9;
static constexpr double kFacingFull = 0.95;

/* the band, a share of the greater depth, over which the area contact of two
flat faces goes from the depth along one's normal to the other's */
static constexpr double kDepthBand = 0.1;

/* the shares of the smaller of two flat faces in contact from which their
area contact begins to take over from that of their edges and vertices, and
from which it is the whole contact: a corner or an edge in a face is the
contact of that feature, a face resting on another, or tilted a little, the
contact of the faces */
static constexpr double kFlushLeast = 0.1;
static constexpr double kFlushFull = 0.5;


/* the diagonal of a box */
static double BoxDiagonal(const shift::core::collision_geometry::CadBox& box) {
  const ksl_vec3_t d = Sub(box.hi, box.lo);
  return ksl_vec3_l2norm(&d);
}

/* whether two faces may be of the same shape, or nearly (see Conforms), by
the type and dimensions of their surfaces: two analytic surfaces of one type
and dimensions within kNearShape of each other, one convex and the other
concave, or a surface of any other kind with a plane or
another of its kind, or two planes (see PlaneAreaContact) */
static bool MayConform(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                       const shift::core::collision_geometry::CadSideOfPair& b, int fb) {
  const shift::core::cad::Surface& sa = SurfaceOf(a, fa);
  const shift::core::cad::Surface& sb = SurfaceOf(b, fb);
  auto analytic = [](shift::core::cad::Surface::Type t) {
    return t == shift::core::cad::Surface::Type::kPlane ||
           t == shift::core::cad::Surface::Type::kCylinder ||
           t == shift::core::cad::Surface::Type::kCone ||
           t == shift::core::cad::Surface::Type::kSphere ||
           t == shift::core::cad::Surface::Type::kTorus;
  };
  if(!analytic(sa.type) || !analytic(sb.type)) {
    return true;
  }
  if(sa.type == shift::core::cad::Surface::Type::kPlane &&
     sb.type == shift::core::cad::Surface::Type::kPlane) {
    return true; // their facing (opposed normals) is checked by Conforms
  }
  if(sa.type != sb.type || SameSense(a, fa) == SameSense(b, fb)) {
    return false;
  }
  const double scale_a = a.shape->placements[a.shape->faces[fa].placement].scale;
  const double scale_b = b.shape->placements[b.shape->faces[fb].placement].scale;
  auto same = [](double x, double y) {
    return std::fabs(x - y) <= kNearShape * std::max(std::fabs(x), std::fabs(y));
  };
  switch(sa.type) {
    case shift::core::cad::Surface::Type::kCylinder:
    case shift::core::cad::Surface::Type::kSphere:
      return same(scale_a * sa.radius, scale_b * sb.radius);
    case shift::core::cad::Surface::Type::kCone:
      return same(sa.semi_angle, sb.semi_angle);
    case shift::core::cad::Surface::Type::kTorus:
      return same(scale_a * sa.radius, scale_b * sb.radius) &&
             same(scale_a * sa.minor_radius, scale_b * sb.minor_radius);
    default:
      return false;
  }
}

/* the solution x of the symmetric positive definite system J x = r of order
6, by Cholesky's method; false if J is not */
static bool SolveSymmetric6(double J[36], const double r[6], double x[6]) {
  double L[36] = {0.0};
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j <= i; j++) {
      double sum = J[i * 6 + j];
      for(int k = 0; k < j; k++) {
        sum -= L[i * 6 + k] * L[j * 6 + k];
      }
      if(i == j) {
        if(!(sum > 0.0)) {
          return false;
        }
        L[i * 6 + i] = std::sqrt(sum);
      } else {
        L[i * 6 + j] = sum / L[j * 6 + j];
      }
    }
  }
  double y[6];
  for(int i = 0; i < 6; i++) {
    y[i] = r[i];
    for(int k = 0; k < i; k++) {
      y[i] -= L[i * 6 + k] * y[k];
    }
    y[i] /= L[i * 6 + i];
  }
  for(int i = 5; i >= 0; i--) {
    x[i] = y[i];
    for(int k = i + 1; k < 6; k++) {
      x[i] -= L[k * 6 + i] * x[k];
    }
    x[i] /= L[i * 6 + i];
  }
  return true;
}

/*
Whether two faces conform: are of the same shape where they face each other,
as a brake shoe on its drum, or a surface on a copy of itself. A small rigid
motion of one of two such faces, a translation v and a rotation w about a
point c, changes the depth of each point p of one under the other, along the
normal n of the other there, by n . v + ((p - c) x n) . w to first order: their
depth is so for some v and w. Each sample of the smaller face (small_is_a: a's)
that opposes the other face, within the reach of max_depth and two of its
sample spacings of it, is projected on it for its depth, and the depths are
fitted by least squares: the faces conform if no depth is off the fit by more
than kConformal of the greatest depth (or 1e-9 of the face's size). Faces whose
gap curves, as a ball in a socket of a larger radius, have depths off any such
fit by about that difference; a few samples near a point where two faces of
different shapes touch have depths off it by a share of the depth at least.
The samples fitted must lie on kConformalRows rows of the grid of samples at
least along each parameter: across a line of contact the motion has three
functions of the position along the section (translations along the normal
and the tangent, and the rotation), so three rows fit any curve of the gap.
Faces that touch along a line, or have fewer samples near, do not conform. A
coarse fit to a sixteenth of the samples refuses most pairs of faces first.
Returns 1 if the faces conform, 2 if they nearly do, no depth off the fit by
more than kNearGap of max_depth (a ball in a socket of a radius a little
larger, a shoe a little larger than its drum: their contact wraps round them
as they press in, see ConformalPair), 0 if not.
*/
static int Conforms(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                     const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                     double max_depth, bool& small_is_a) {
  small_is_a = BoxDiagonal(a.shape->faces[fa].box) <=
               BoxDiagonal(b.shape->faces[fb].box);
  const shift::core::collision_geometry::CadSideOfPair& s = small_is_a ? a : b;
  const shift::core::collision_geometry::CadSideOfPair& o = small_is_a ? b : a;
  const int fs = small_is_a ? fa : fb, fo = small_is_a ? fb : fa;
  const auto& f = s.shape->faces[fs];
  const double size = BoxDiagonal(f.box);
  const double reach = max_depth + 2.0 * SampleSpacing(f);
  ksl_vec3_t center = Scaled(ksl_vec3_t{{f.box.lo.x + f.box.hi.x,
                                         f.box.lo.y + f.box.hi.y,
                                         f.box.lo.z + f.box.hi.z}},
                             0.5);
  center = SidePosition(s, center);
  /* the fit to the samples every stride cells of the grid along each
  parameter: 1 if they conform, 0 if not, -1 if too few are near */
  thread_local std::vector<double> rows, depths;
  thread_local std::vector<char> on_u, on_v;
  auto fit = [&](int stride) {
    rows.clear();
    depths.clear();
    on_u.assign(f.sample_nu, 0);
    on_v.assign(f.sample_nv, 0);
    shift::core::cad::Vec2 guess{0.0, 0.0};
    bool have_guess = false;
    for(int ci = 0; ci < f.sample_nu; ci += stride) {
      for(int cj = 0; cj < f.sample_nv; cj += stride) {
        const int i = f.sample_cell[ci * f.sample_nv + cj];
        if(i < 0) {
          continue;
        }
        const ksl_vec3_t p = SidePosition(s, f.sample_points[i]);
        const shift::core::cad::Vec2* start = have_guess ? &guess : nullptr;
        if(o.fields) {
          ksl_vec3_t own;
          ksl_product_dinvv(&o.d, &p, &own);
          if(const auto* node = (*o.fields)[fo].Nearest(own)) {
            start = &node->uv;
          }
        }
        shift::core::cad::Vec2 uv;
        ksl_vec3_t q, outward = {{0.0, 0.0, 0.0}};
        bool beyond = false;
        const double d = FaceDepth(o, fo, p, start, uv, q, outward, beyond);
        if(d == -HUGE_VAL || beyond || std::fabs(d) > reach ||
           !o.shape->Inside(fo, uv) ||
           Dot(SideVector(s, f.sample_normals[i]), outward) > -0.9) {
          continue;
        }
        guess = uv;
        have_guess = true;
        on_u[ci] = 1;
        on_v[cj] = 1;
        const ksl_vec3_t arm = Scaled(Sub(p, center), 1.0 / size);
        ksl_vec3_t moment;
        ksl_cross_vv(&arm, &outward, &moment);
        for(int k = 0; k < 3; k++) {
          rows.push_back(outward.at[k]);
        }
        for(int k = 0; k < 3; k++) {
          rows.push_back(moment.at[k]);
        }
        depths.push_back(d);
      }
    }
    const size_t count = depths.size();
    if(count < 6 ||
       std::count(on_u.begin(), on_u.end(), 1) < kConformalRows ||
       std::count(on_v.begin(), on_v.end(), 1) < kConformalRows) {
      return -1;
    }
    double J[36] = {0.0}, r[6] = {0.0}, x[6];
    double greatest = 0.0;
    for(size_t n = 0; n < count; n++) {
      const double* row = &rows[6 * n];
      for(int i = 0; i < 6; i++) {
        r[i] += row[i] * depths[n];
        for(int j = 0; j < 6; j++) {
          J[i * 6 + j] += row[i] * row[j];
        }
      }
      greatest = std::max(greatest, std::fabs(depths[n]));
    }
    /* the motions that move no point along its normal (along a cylinder's
    axis, about it, or in a plane) are left out by a little damping */
    double trace = 0.0;
    for(int i = 0; i < 6; i++) {
      trace += J[i * 6 + i];
    }
    for(int i = 0; i < 6; i++) {
      J[i * 6 + i] += 1e-12 * trace;
    }
    if(!SolveSymmetric6(J, r, x)) {
      return 0;
    }
    double off = 0.0;
    for(size_t n = 0; n < count; n++) {
      const double* row = &rows[6 * n];
      double fitted = 0.0;
      for(int i = 0; i < 6; i++) {
        fitted += row[i] * x[i];
      }
      off = std::max(off, std::fabs(fitted - depths[n]));
    }
    if(off <= kConformal * greatest + 1e-9 * size) {
      return 1;
    }
    return off <= kNearGap * max_depth ? 2 : 0;
  };
  const int coarse = std::max(1, std::max(f.sample_nu, f.sample_nv) / 4);
  if(coarse > 1 && fit(coarse) == 0) {
    return 0;
  }
  const int level = fit(1);
  return level < 0 ? 0 : level;
}

/* the part of a square in contact, of a depth linear on each of the two
triangles of its diagonal from (0, 0) to (1, 1) through its values at the
corners d[i][j], at (x0 + i size, y0 + j size): adds its area, the integral of
its depth, and the moments of its area about the axes (for the centroid) */
struct AreaLoad {
  double area = 0.0, load = 0.0, mx = 0.0, my = 0.0;
};

static void ClipSquare(double x0, double y0, double size, const double d[2][2],
                       AreaLoad& sum) {
  const double corners[2][3][3] = {
    {{x0, y0, d[0][0]}, {x0 + size, y0, d[1][0]}, {x0 + size, y0 + size, d[1][1]}},
    {{x0, y0, d[0][0]}, {x0 + size, y0 + size, d[1][1]}, {x0, y0 + size, d[0][1]}}};
  for(const auto& t : corners) {
    /* the triangle clipped to positive depth: four corners at most */
    double poly[4][3];
    int n = 0;
    for(int k = 0; k < 3; k++) {
      const double* u = t[k];
      const double* v = t[(k + 1) % 3];
      if(u[2] > 0.0) {
        for(int m = 0; m < 3; m++) {
          poly[n][m] = u[m];
        }
        n++;
      }
      if((u[2] > 0.0) != (v[2] > 0.0)) {
        const double w = u[2] / (u[2] - v[2]);
        for(int m = 0; m < 2; m++) {
          poly[n][m] = u[m] + w * (v[m] - u[m]);
        }
        poly[n][2] = 0.0;
        n++;
      }
    }
    for(int k = 1; k + 1 < n; k++) {
      const double* a = poly[0];
      const double* b = poly[k];
      const double* c = poly[k + 1];
      const double area =
        0.5 * std::fabs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]));
      sum.area += area;
      sum.load += area * (a[2] + b[2] + c[2]) / 3.0;
      sum.mx += area * (a[0] + b[0] + c[0]) / 3.0;
      sum.my += area * (a[1] + b[1] + c[1]) / 3.0;
    }
  }
}

/* the subsamples along each side of a quarter of a cell of AreaContact that
a trim of either face crosses */
static constexpr int kAreaSubsamples = 6;

/*
The contact of two conformal faces (see Conforms) over the area where they
overlap: a grid over the parameters of the smaller face s, its cells as fine
as its samples or 4 to a side at least, each with the 2 by 2 points of Gauss's
rule, kAreaPoints at most. Each point is projected on the other face o, from
the foot of the point before, for its depth there and its share of the area,
its Gauss weight times the parallelogram of the derivatives of s there (the
rule is exact for a cubic in the parameters, as the area of a B-spline face's
cells is near its clamped ends).

Each point of the rule stands for a quarter of its cell. The depth, the point
and the parameters of its foot on o over a cell are the bilinear functions
through the cell's four points: exact for two planes, and to second order in
the cell's size for other faces of the same shape. A cell wholly inside both
faces' trims (by the corners of its quarters), deeper at all of them than its
depth changes across it, is its four points of the rule. Any other cell, which
a trim of either face, the line of no depth or the end of the face crosses or
may cross, is an edge cell: the depths at its corners are projected, its
depth is the biquadratic function through them (the bilinear one would place
the line of no depth off by the square of the cell's size), and each of its
quarters is subsampled by kAreaSubsamples to a side, each subsample's area by
the bilinear function through the areas of the rule's points. The point of a
quarter is at the centroid of the part of it in contact, on s, with o's normal
at its foot, carrying that part's area and mean depth, so that the force of a
linear law over it is exact. A strip of contact narrower than a
cell along the edge of a tilted face is so found, and its depth at the edge:
if the deepest corner of a quarter in contact is deeper than every point by a
thousandth, the point of that quarter carries the depth, with its area reduced
so that its force is the same. A cell whose points do not all project on o,
opposing it, has its points of the rule only.

The patch area is the sum of the areas of the points, the area in contact. A
point enters the contact at no depth, so the force has no jump as the faces
press in. Adds the points, from a to b; s_is_a: s is a's face.
*/
static void AreaContact(const shift::core::collision_geometry::CadSideOfPair& s, int fs,
                        const shift::core::collision_geometry::CadSideOfPair& o, int fo,
                        bool s_is_a,
                        std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& f = s.shape->faces[fs];
  int cells_u = std::max(4, f.sample_nu), cells_v = std::max(4, f.sample_nv);
  if(4 * cells_u * cells_v > kAreaPoints) {
    const double shrink =
      std::sqrt(static_cast<double>(kAreaPoints) / (4 * cells_u * cells_v));
    cells_u = std::max(1, static_cast<int>(cells_u * shrink));
    cells_v = std::max(1, static_cast<int>(cells_v * shrink));
  }
  const double cell_u = (f.hi[0] - f.lo[0]) / cells_u;
  const double cell_v = (f.hi[1] - f.lo[1]) / cells_v;
  /* the points of the rule along each parameter, at (1 -+ 1 / sqrt(3)) / 2 of
  a cell, each with half of its span */
  const double du = 0.5 * cell_u, dv = 0.5 * cell_v;
  const double gauss[2] = {0.5 - 0.5 / std::sqrt(3.0), 0.5 + 0.5 / std::sqrt(3.0)};
  /* the weights of the two points of the rule in the linear function of x
  (the position across the cell, 0 to 1) through them */
  auto lagrange = [&](double x, double w[2]) {
    w[1] = (x - gauss[0]) / (gauss[1] - gauss[0]);
    w[0] = 1.0 - w[1];
  };
  /* a point of the rule: its foot on o, opposing it (usable), and whether it
  is inside each face's trims */
  struct Rule {
    shift::core::cad::Vec2 uv, at;
    ksl_vec3_t p, q, n;
    double depth = 0.0, area = 0.0;
    bool usable = false, in_s = false, in_o = false;
  };
  /* a position (x, y) in a cell, of the bilinear functions through its rule:
  its parameters on s, its depth, point, and the parameters of its foot on o */
  struct Blend {
    shift::core::cad::Vec2 uv, at;
    ksl_vec3_t p;
    double depth;
  };
  const size_t first = points.size();
  /* the deepest corner of a quarter (or of its subsamples) in contact, by the
  bilinear functions, its parameters on s and on o, and the point of that
  quarter */
  double deepest = 0.0;
  size_t deepest_point = 0;
  shift::core::cad::Vec2 deepest_uv{0.0, 0.0}, deepest_at{0.0, 0.0};
  /* the cells, row by row, each point projected from the foot of the one
  before */
  shift::core::cad::Vec2 guess{0.0, 0.0};
  bool have_guess = false;
  Rule rule[2][2];
  for(int ci = 0; ci < cells_u; ci++) {
    for(int cj = 0; cj < cells_v; cj++) {
      bool all_usable = true;
      for(int a = 0; a < 2; a++) {
        for(int b = 0; b < 2; b++) {
          Rule& r = rule[a][b];
          r = Rule();
          r.uv = {f.lo[0] + (ci + gauss[a]) * cell_u,
                  f.lo[1] + (cj + gauss[b]) * cell_v};
          r.in_s = s.shape->Inside(fs, r.uv);
          ksl_vec3_t pu, pv;
          r.p = SidePoint(s, fs, r.uv, &pu, &pv);
          ksl_vec3_t cross;
          ksl_cross_vv(&pu, &pv, &cross);
          r.area = ksl_vec3_l2norm(&cross) * du * dv;
          const shift::core::cad::Vec2* start = have_guess ? &guess : nullptr;
          if(o.fields) {
            ksl_vec3_t own;
            ksl_product_dinvv(&o.d, &r.p, &own);
            if(const auto* node = (*o.fields)[fo].Nearest(own)) {
              start = &node->uv;
            }
          }
          bool beyond = false;
          r.n = {{0.0, 0.0, 0.0}};
          r.depth = FaceDepth(o, fo, r.p, start, r.at, r.q, r.n, beyond);
          r.usable = r.depth != -HUGE_VAL && !beyond &&
                     Dot(SideNormal(s, fs, r.uv), r.n) <= -0.9;
          if(r.usable) {
            r.in_o = o.shape->Inside(fo, r.at);
            guess = r.at;
            have_guess = true;
          }
          all_usable = all_usable && r.usable;
        }
      }
      /* the point of the rule of a quarter, with its depth and share */
      auto rule_point = [&](const Rule& r) {
        if(!(r.depth > 0.0)) {
          return;
        }
        shift::core::collision_geometry::ContactPoint c =
          FeaturePoint(s_is_a, r.p, r.q, r.n, r.depth, 0.0);
        c.area = r.area;
        points.push_back(c);
      };
      if(!all_usable) {
        for(int a = 0; a < 2; a++) {
          for(int b = 0; b < 2; b++) {
            const Rule& r = rule[a][b];
            if(r.usable && r.in_s && r.in_o) {
              rule_point(r);
            }
          }
        }
        continue;
      }
      auto blend = [&](double x, double y) {
        double wx[2], wy[2];
        lagrange(x, wx);
        lagrange(y, wy);
        Blend m{{f.lo[0] + (ci + x) * cell_u, f.lo[1] + (cj + y) * cell_v},
                {0.0, 0.0}, {{0.0, 0.0, 0.0}}, 0.0};
        for(int a = 0; a < 2; a++) {
          for(int b = 0; b < 2; b++) {
            const double w = wx[a] * wy[b];
            const Rule& r = rule[a][b];
            m.depth += w * r.depth;
            m.at[0] += w * r.at[0];
            m.at[1] += w * r.at[1];
            m.p = Axpy(m.p, w, r.p);
          }
        }
        return m;
      };
      /* the corners of the quarters, a 3 by 3 grid over the cell, a little
      inside its ends: their depths, and whether they are in both trims */
      const double inset = 1e-9;
      double corner_depth[3][3];
      bool corner_in[3][3];
      shift::core::cad::Vec2 corner_uv[3][3], corner_at[3][3];
      for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
          const double x = std::clamp(0.5 * i, inset, 1.0 - inset);
          const double y = std::clamp(0.5 * j, inset, 1.0 - inset);
          const Blend m = blend(x, y);
          corner_depth[i][j] = m.depth;
          corner_uv[i][j] = m.uv;
          corner_at[i][j] = m.at;
          corner_in[i][j] =
            s.shape->Inside(fs, m.uv) && o.shape->Inside(fo, m.at);
        }
      }
      /* a cell that the line of no depth or a trim crosses, or may cross (its
      corners are no deeper than a quarter of the change of depth across it:
      the bilinear functions miss the line by the square of the cell's size),
      has the
      depths of its corners projected, and its depth is the biquadratic
      function through them */
      double least = HUGE_VAL, most = -HUGE_VAL;
      for(int a = 0; a < 2; a++) {
        for(int b = 0; b < 2; b++) {
          least = std::min(least, rule[a][b].depth);
          most = std::max(most, rule[a][b].depth);
        }
      }
      const double margin = 0.25 * (most - least);
      bool edge_cell = !(least > 0.0);
      for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
          edge_cell = edge_cell || !corner_in[i][j] || !(corner_depth[i][j] > margin);
        }
      }
      if(edge_cell) {
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++) {
            ksl_vec3_t q, n = {{0.0, 0.0, 0.0}};
            shift::core::cad::Vec2 at;
            bool beyond = false;
            const double d = FaceDepth(o, fo, SidePoint(s, fs, corner_uv[i][j]),
                                       &corner_at[i][j], at, q, n, beyond);
            if(d != -HUGE_VAL && !beyond) {
              corner_depth[i][j] = d;
            }
          }
        }
      }
      /* the depth at (x, y) in the cell: biquadratic in an edge cell */
      auto quadratic = [](double x, double w[3]) {
        w[0] = 2.0 * (x - 0.5) * (x - 1.0);
        w[1] = -4.0 * x * (x - 1.0);
        w[2] = 2.0 * x * (x - 0.5);
      };
      auto depth_of = [&](double x, double y) {
        if(!edge_cell) {
          return blend(x, y).depth;
        }
        double wx[3], wy[3];
        quadratic(x, wx);
        quadratic(y, wy);
        double d = 0.0;
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++) {
            d += wx[i] * wy[j] * corner_depth[i][j];
          }
        }
        return d;
      };
      for(int a = 0; a < 2; a++) {
        for(int b = 0; b < 2; b++) {
          const Rule& r = rule[a][b];
          double corner_deepest = -HUGE_VAL;
          shift::core::cad::Vec2 corner_deepest_uv{0.0, 0.0}, corner_deepest_at{0.0, 0.0};
          bool corners_in = true;
          for(int i = a; i <= a + 1; i++) {
            for(int j = b; j <= b + 1; j++) {
              corners_in = corners_in && corner_in[i][j];
              if(corner_in[i][j] && corner_depth[i][j] > corner_deepest) {
                corner_deepest = corner_depth[i][j];
                corner_deepest_uv = corner_uv[i][j];
                corner_deepest_at = corner_at[i][j];
              }
            }
          }
          if(!edge_cell) {
            rule_point(r);
            if(corner_deepest > deepest) {
              deepest = corner_deepest;
              deepest_point = points.size() - 1;
              deepest_uv = corner_deepest_uv;
              deepest_at = corner_deepest_at;
            }
            continue;
          }
          /* the part of the quarter of an edge cell in both trims, and in
          contact: each of its subsamples in them, clipped to its depth, with
          its share of the area by the bilinear function through the areas of
          the rule's points (the point of a quarter alone, off its middle, is
          not exact once the cell is not whole) */
          AreaLoad sum;
          double sub_deepest = corner_deepest;
          shift::core::cad::Vec2 sub_deepest_uv = corner_deepest_uv,
                                 sub_deepest_at = corner_deepest_at;
          {
            const double size = 0.5 / kAreaSubsamples;
            double depth_at[kAreaSubsamples + 1][kAreaSubsamples + 1];
            for(int i = 0; i <= kAreaSubsamples; i++) {
              for(int j = 0; j <= kAreaSubsamples; j++) {
                depth_at[i][j] = depth_of(0.5 * a + i * size, 0.5 * b + j * size);
              }
            }
            for(int i = 0; i < kAreaSubsamples; i++) {
              for(int j = 0; j < kAreaSubsamples; j++) {
                const double x = 0.5 * a + i * size, y = 0.5 * b + j * size;
                const Blend m = blend(x + 0.5 * size, y + 0.5 * size);
                if(!corners_in &&
                   (!s.shape->Inside(fs, m.uv) || !o.shape->Inside(fo, m.at))) {
                  continue;
                }
                const double d[2][2] = {{depth_at[i][j], depth_at[i][j + 1]},
                                        {depth_at[i + 1][j], depth_at[i + 1][j + 1]}};
                AreaLoad part;
                ClipSquare(x, y, size, d, part);
                if(!(part.area > 0.0)) {
                  continue;
                }
                /* the area of the cell, by (x, y), here: 4 times a quarter's */
                double wx[2], wy[2];
                lagrange(x + 0.5 * size, wx);
                lagrange(y + 0.5 * size, wy);
                double density = 0.0;
                for(int ra = 0; ra < 2; ra++) {
                  for(int rb = 0; rb < 2; rb++) {
                    density += 4.0 * wx[ra] * wy[rb] * rule[ra][rb].area;
                  }
                }
                sum.area += density * part.area;
                sum.load += density * part.load;
                sum.mx += density * part.mx;
                sum.my += density * part.my;
                for(int k = 0; k < 4; k++) {
                  if(d[k / 2][k % 2] > sub_deepest) {
                    sub_deepest = d[k / 2][k % 2];
                    const Blend c = blend(x + (k / 2) * size, y + (k % 2) * size);
                    sub_deepest_uv = c.uv;
                    sub_deepest_at = c.at;
                  }
                }
              }
            }
          }
          if(!(sum.area > 0.0)) {
            continue;
          }
          const double touching = sum.area;
          /* the centroid on s itself: the blend of the points of the rule
          lies off a curved face by its sag across the cell */
          const double cx = sum.mx / sum.area, cy = sum.my / sum.area;
          const Blend m = blend(cx, cy);
          const double depth = sum.load / sum.area;
          const ksl_vec3_t on_s = SidePoint(s, fs, m.uv);
          /* the normal there: o's at the foot of the point, else that of the
          bilinear function through the rule's */
          ksl_vec3_t n = {{0.0, 0.0, 0.0}};
          {
            double wx[2], wy[2];
            lagrange(cx, wx);
            lagrange(cy, wy);
            for(int ra = 0; ra < 2; ra++) {
              for(int rb = 0; rb < 2; rb++) {
                n = Axpy(n, wx[ra] * wy[rb], rule[ra][rb].n);
              }
            }
            const double length = ksl_vec3_l2norm(&n);
            n = length > 0.0 ? Scaled(n, 1.0 / length) : r.n;
            /* o's own, from the point's foot there */
            ksl_vec3_t foot, normal = {{0.0, 0.0, 0.0}};
            shift::core::cad::Vec2 at;
            bool beyond = false;
            if(FaceDepth(o, fo, on_s, &m.at, at, foot, normal, beyond) != -HUGE_VAL &&
               !beyond && Dot(normal, n) > 0.9) {
              n = normal;
            }
          }
          const ksl_vec3_t q = Axpy(on_s, depth, n);
          shift::core::collision_geometry::ContactPoint c =
            FeaturePoint(s_is_a, on_s, q, n, depth, 0.0);
          c.area = touching;
          points.push_back(c);
          if(sub_deepest > deepest) {
            deepest = sub_deepest;
            deepest_point = points.size() - 1;
            deepest_uv = sub_deepest_uv;
            deepest_at = sub_deepest_at;
          }
        }
      }
    }
  }
  double patch = 0.0, point_deepest = 0.0;
  for(size_t k = first; k < points.size(); k++) {
    patch += points[k].area;
    point_deepest = std::max(point_deepest, points[k].depth);
  }
  for(size_t k = first; k < points.size(); k++) {
    points[k].patch_area = patch;
  }
  /* the deepest corner's depth, projected: the bilinear functions overshoot
  on a curved face, beyond the points of the rule */
  if(points.size() > first && deepest > (1.0 + 1e-3) * point_deepest) {
    const ksl_vec3_t p = SidePoint(s, fs, deepest_uv);
    shift::core::cad::Vec2 at;
    ksl_vec3_t q, n = {{0.0, 0.0, 0.0}};
    bool beyond = false;
    const double d = FaceDepth(o, fo, p, &deepest_at, at, q, n, beyond);
    deepest = d != -HUGE_VAL && !beyond && s.shape->Inside(fs, deepest_uv) &&
                  o.shape->Inside(fo, at)
                ? d
                : 0.0;
  }
  if(points.size() > first && deepest > (1.0 + 1e-3) * point_deepest) {
    shift::core::collision_geometry::ContactPoint& c = points[deepest_point];
    c.area *= c.depth / deepest;
    c.depth = deepest;
    /* the point on the side that is not s, along the normal from a to b */
    if(s_is_a) {
      c.point_b = Axpy(c.point_a, -deepest, c.normal);
    } else {
      c.point_a = Axpy(c.point_b, deepest, c.normal);
    }
  }
}

/* the most cells of the grid over the overlap of two flat faces on which
PlaneAreaContact integrates, and the subsamples along each side of a cell that
the trims of both faces cross */
static constexpr int kPlaneAreaCells = 256;
static constexpr int kPlaneAreaSubsamples = 8;

/* a polygon of a plane's parameters (u, v), clipped by the half-plane
a u + b v + c >= 0 (Sutherland and Hodgman): a polygon that is not convex may
leave sides along the line, of no area, which the integrals of PolygonLoad
ignore */
static void ClipHalfPlane(const std::vector<shift::core::cad::Vec2>& in,
                          double a, double b, double c,
                          std::vector<shift::core::cad::Vec2>& out) {
  out.clear();
  const size_t n = in.size();
  for(size_t i = 0; i < n; i++) {
    const shift::core::cad::Vec2& p = in[i];
    const shift::core::cad::Vec2& q = in[(i + 1) % n];
    const double fp = a * p[0] + b * p[1] + c, fq = a * q[0] + b * q[1] + c;
    if(fp >= 0.0) {
      out.push_back(p);
    }
    if((fp >= 0.0) != (fq >= 0.0)) {
      const double t = fp / (fp - fq);
      out.push_back({p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])});
    }
  }
}

/* the depth of PlaneAreaContact, d0 + du u + dv v */
struct PlaneDepth {
  double d0, du, dv;
  double At(const shift::core::cad::Vec2& p) const {
    return d0 + du * p[0] + dv * p[1];
  }
};

/* adds the signed integrals of a polygon (of a fan of triangles: correct for
any winding): its area, of the depth over it, and of the moments of its area;
and its deepest corner */
static void PolygonLoad(const std::vector<shift::core::cad::Vec2>& poly,
                        const PlaneDepth& depth, AreaLoad& sum,
                        double& deepest, shift::core::cad::Vec2& deepest_at) {
  const size_t n = poly.size();
  if(n < 3) {
    return;
  }
  for(const shift::core::cad::Vec2& p : poly) {
    const double d = depth.At(p);
    if(d > deepest) {
      deepest = d;
      deepest_at = p;
    }
  }
  const shift::core::cad::Vec2& a = poly[0];
  const double da = depth.At(a);
  for(size_t k = 1; k + 1 < n; k++) {
    const shift::core::cad::Vec2& b = poly[k];
    const shift::core::cad::Vec2& c = poly[k + 1];
    const double area =
      0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]));
    sum.area += area;
    sum.load += area * (da + depth.At(b) + depth.At(c)) / 3.0;
    sum.mx += area * (a[0] + b[0] + c[0]) / 3.0;
    sum.my += area * (a[1] + b[1] + c[1]) / 3.0;
  }
}

/* the loops of a face's trims, outer loops counterclockwise and holes
clockwise (by their nesting), so that the signed integrals of PolygonLoad over
them are those of the face */
static void OrientLoops(std::vector<std::vector<shift::core::cad::Vec2>>& loops) {
  auto inside = [](const std::vector<shift::core::cad::Vec2>& loop,
                   const shift::core::cad::Vec2& p) {
    bool in = false;
    for(size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
      const shift::core::cad::Vec2& a = loop[j];
      const shift::core::cad::Vec2& b = loop[i];
      if((a[1] > p[1]) != (b[1] > p[1]) &&
         p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])) {
        in = !in;
      }
    }
    return in;
  };
  std::vector<bool> reverse(loops.size(), false);
  for(size_t i = 0; i < loops.size(); i++) {
    if(loops[i].size() < 3) {
      continue;
    }
    int depth = 0;
    for(size_t j = 0; j < loops.size(); j++) {
      if(j != i && loops[j].size() >= 3 && inside(loops[j], loops[i][0])) {
        depth++;
      }
    }
    double area = 0.0;
    for(size_t k = 0, l = loops[i].size() - 1; k < loops[i].size(); l = k++) {
      area += loops[i][l][0] * loops[i][k][1] - loops[i][k][0] * loops[i][l][1];
    }
    reverse[i] = (area > 0.0) == (depth % 2 == 1);
  }
  for(size_t i = 0; i < loops.size(); i++) {
    if(reverse[i]) {
      std::reverse(loops[i].begin(), loops[i].end());
    }
  }
}

/* the area of a flat face, from its trims (its holes taken away) */
static double PlaneFaceArea(const shift::core::collision_geometry::CadSideOfPair& side, int face) {
  thread_local std::vector<std::vector<shift::core::cad::Vec2>> loops;
  loops.assign(side.shape->faces[face].trims.begin(),
               side.shape->faces[face].trims.end());
  OrientLoops(loops);
  double area = 0.0;
  for(const auto& loop : loops) {
    for(size_t k = 0, l = loop.size() - 1; k < loop.size(); l = k++) {
      area += 0.5 * (loop[l][0] * loop[k][1] - loop[k][0] * loop[l][1]);
    }
  }
  ksl_vec3_t du, dv;
  SidePoint(side, face, shift::core::cad::Vec2{0.0, 0.0}, &du, &dv);
  ksl_vec3_t cross;
  ksl_cross_vv(&du, &dv, &cross);
  return std::max(area, 0.0) * ksl_vec3_l2norm(&cross);
}

/*
The contact of two flat faces s and o that face each other over the area where
they overlap, exactly: the region of s's parameters inside s's trims, inside
o's trims carried to s along o's normal (the feet on o of s's points), of
positive depth under o, a linear function of the parameters, and nearer o than
the planes of the flat faces next to o over its convex edges (a point nearer
one of them, as a corner through the next face, is in o's solid as deep as
that, and its contact is not o's). A grid over the
bounds of that region, of kPlaneAreaCells cells at most, holds a point of
contact in each cell it reaches: at the centroid of the part of the region in
the cell, with its area and mean depth, so that the force and moment of a
linear law over the region are exact. The part is the cell, clipped to positive
depth, clipped in turn by the loops of the trims of whichever face's sides cross
the cell; a cell that the sides of both faces cross, near a corner of the
overlap, is subsampled by kPlaneAreaSubsamples to a side. The deepest corner
of the region, deeper than every point, carries its depth on the point of its
cell, as in AreaContact.
The patch area is that of the region. Adds the points, from a to b; s_is_a: s
is a's face.
*/
static void PlaneAreaContact(const shift::core::collision_geometry::CadSideOfPair& s, int fs,
                             const shift::core::collision_geometry::CadSideOfPair& o, int fo,
                             bool s_is_a,
                             std::vector<shift::core::collision_geometry::ContactPoint>& points,
                             bool& at_bound) {
  at_bound = false;
  const auto& f = s.shape->faces[fs];
  const auto& g = o.shape->faces[fo];
  /* s's plane, P(u, v) = ps + u su + v sv, and o's, with its normal */
  const shift::core::cad::Vec2 zero{0.0, 0.0};
  ksl_vec3_t su, sv, ou, ov;
  const ksl_vec3_t ps = SidePoint(s, fs, zero, &su, &sv);
  const ksl_vec3_t po = SidePoint(o, fo, zero, &ou, &ov);
  const ksl_vec3_t ns = SideNormal(s, fs, zero);
  const ksl_vec3_t no = SideNormal(o, fo, zero);
  const double across = Dot(no, ns);
  if(!(across < 0.0)) {
    return;
  }
  /* the depth of P(u, v) under o, (po - P) . no */
  const PlaneDepth depth{Dot(Sub(po, ps), no), -Dot(su, no), -Dot(sv, no)};
  /* the point is nearer o than the flat faces next to o over its convex
  edges, so that it leaves o's solid through o: half-planes, with the one of
  positive depth */
  thread_local std::vector<std::array<double, 3>> cuts;
  cuts.clear();
  cuts.push_back({depth.du, depth.dv, depth.d0});
  for(int e : g.edges) {
    const auto& edge = o.shape->edges[e];
    if(!edge.sharp) {
      continue;
    }
    for(int next : edge.faces) {
      if(next == fo || SurfaceOf(o, next).type != shift::core::cad::Surface::Type::kPlane) {
        continue;
      }
      const ksl_vec3_t pn = SidePoint(o, next, zero);
      const ksl_vec3_t nn = SideNormal(o, next, zero);
      cuts.push_back({-Dot(su, nn) - depth.du, -Dot(sv, nn) - depth.dv,
                      Dot(Sub(pn, ps), nn) - depth.d0});
    }
  }
  auto clip_cuts = [&](std::vector<shift::core::cad::Vec2>& poly,
                       std::vector<shift::core::cad::Vec2>& scratch) {
    for(const auto& h : cuts) {
      ClipHalfPlane(poly, h[0], h[1], h[2], scratch);
      poly.swap(scratch);
    }
  };
  ksl_vec3_t cross;
  ksl_cross_vv(&su, &sv, &cross);
  const double jacobian = ksl_vec3_l2norm(&cross);
  /* s's parameters of a point of its plane */
  const double guu = Dot(su, su), guv = Dot(su, sv), gvv = Dot(sv, sv);
  const double det = guu * gvv - guv * guv;
  if(!(det > 0.0) || !(jacobian > 0.0)) {
    return;
  }
  auto parameters = [&](const ksl_vec3_t& p) {
    const ksl_vec3_t r = Sub(p, ps);
    const double ru = Dot(r, su), rv = Dot(r, sv);
    return shift::core::cad::Vec2{(gvv * ru - guv * rv) / det,
                                  (guu * rv - guv * ru) / det};
  };
  /* the loops of both faces in s's parameters: o's carried along its normal
  to s's plane */
  thread_local std::vector<std::vector<shift::core::cad::Vec2>> loops_s, loops_o;
  loops_s.assign(f.trims.begin(), f.trims.end());
  loops_o.resize(g.trims.size());
  for(size_t i = 0; i < g.trims.size(); i++) {
    loops_o[i].clear();
    for(const shift::core::cad::Vec2& w : g.trims[i]) {
      const ksl_vec3_t q = Axpy(Axpy(po, w[0], ou), w[1], ov);
      const double t = Dot(Sub(q, ps), ns) / across;
      loops_o[i].push_back(parameters(Axpy(q, -t, no)));
    }
  }
  OrientLoops(loops_s);
  OrientLoops(loops_o);
  /* the bounds of each face's part of positive depth, and of their overlap */
  thread_local std::vector<shift::core::cad::Vec2> clipped, clipped2;
  double lo[2] = {HUGE_VAL, HUGE_VAL}, hi[2] = {-HUGE_VAL, -HUGE_VAL};
  for(const auto* loops : {&loops_s, &loops_o}) {
    double l[2] = {HUGE_VAL, HUGE_VAL}, h[2] = {-HUGE_VAL, -HUGE_VAL};
    for(const auto& loop : *loops) {
      clipped = loop;
      clip_cuts(clipped, clipped2);
      for(const shift::core::cad::Vec2& p : clipped) {
        for(int k = 0; k < 2; k++) {
          l[k] = std::min(l[k], p[k]);
          h[k] = std::max(h[k], p[k]);
        }
      }
    }
    for(int k = 0; k < 2; k++) {
      lo[k] = loops == &loops_s ? l[k] : std::max(lo[k], l[k]);
      hi[k] = loops == &loops_s ? h[k] : std::min(hi[k], h[k]);
    }
  }
  if(!(hi[0] > lo[0]) || !(hi[1] > lo[1])) {
    return;
  }
  /* the grid, of cells about square on the face */
  const double width = (hi[0] - lo[0]) * std::sqrt(guu);
  const double height = (hi[1] - lo[1]) * std::sqrt(gvv);
  int nu = static_cast<int>(std::lround(std::sqrt(kPlaneAreaCells * width / height)));
  nu = std::min(std::max(nu, 1), kPlaneAreaCells);
  const int nv = std::max(1, kPlaneAreaCells / nu);
  const double cu = (hi[0] - lo[0]) / nu, cv = (hi[1] - lo[1]) / nv;
  /* whether a face's sides cross a cell, and if not, whether it holds it */
  auto crossed = [](const std::vector<std::vector<shift::core::cad::Vec2>>& loops,
                    const double box[4]) {
    for(const auto& loop : loops) {
      for(size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
        const shift::core::cad::Vec2& a = loop[j];
        const shift::core::cad::Vec2& b = loop[i];
        if(std::max(a[0], b[0]) < box[0] || std::min(a[0], b[0]) > box[2] ||
           std::max(a[1], b[1]) < box[1] || std::min(a[1], b[1]) > box[3]) {
          continue;
        }
        /* the corners of the box on both sides of the side's line, or an end
        inside the box */
        const double nx = b[1] - a[1], ny = a[0] - b[0];
        bool above = false, below = false;
        for(int k = 0; k < 4; k++) {
          const double x = box[(k & 1) ? 2 : 0], y = box[(k & 2) ? 3 : 1];
          const double side = nx * (x - a[0]) + ny * (y - a[1]);
          above = above || side > 0.0;
          below = below || side < 0.0;
        }
        if(above && below) {
          return true;
        }
      }
    }
    return false;
  };
  auto holds = [](const std::vector<std::vector<shift::core::cad::Vec2>>& loops,
                  const shift::core::cad::Vec2& p) {
    bool in = false;
    for(const auto& loop : loops) {
      for(size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
        const shift::core::cad::Vec2& a = loop[j];
        const shift::core::cad::Vec2& b = loop[i];
        if((a[1] > p[1]) != (b[1] > p[1]) &&
           p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])) {
          in = !in;
        }
      }
    }
    return in;
  };
  /* the integrals of a square clipped to positive depth, then by loops */
  auto clip_square = [&](const double box[4],
                         const std::vector<std::vector<shift::core::cad::Vec2>>* loops,
                         AreaLoad& sum, double& deepest,
                         shift::core::cad::Vec2& deepest_at) {
    const double planes[4][3] = {{1.0, 0.0, -box[0]}, {-1.0, 0.0, box[2]},
                                 {0.0, 1.0, -box[1]}, {0.0, -1.0, box[3]}};
    auto clip_all = [&](std::vector<shift::core::cad::Vec2>& poly) {
      for(const auto& h : planes) {
        ClipHalfPlane(poly, h[0], h[1], h[2], clipped2);
        poly.swap(clipped2);
      }
      clip_cuts(poly, clipped2);
    };
    if(!loops) {
      clipped = {{box[0], box[1]}, {box[2], box[1]}, {box[2], box[3]}, {box[0], box[3]}};
      clip_all(clipped);
      PolygonLoad(clipped, depth, sum, deepest, deepest_at);
      return;
    }
    for(const auto& loop : *loops) {
      clipped = loop;
      clip_all(clipped);
      PolygonLoad(clipped, depth, sum, deepest, deepest_at);
    }
  };
  const size_t first = points.size();
  double deepest = 0.0;
  size_t deepest_point = 0;
  shift::core::cad::Vec2 deepest_at{0.0, 0.0};
  for(int i = 0; i < nu; i++) {
    for(int j = 0; j < nv; j++) {
      const double box[4] = {lo[0] + i * cu, lo[1] + j * cv,
                             lo[0] + (i + 1) * cu, lo[1] + (j + 1) * cv};
      /* no depth anywhere in the cell */
      double most = -HUGE_VAL;
      for(int k = 0; k < 4; k++) {
        most = std::max(most, depth.At({box[(k & 1) ? 2 : 0], box[(k & 2) ? 3 : 1]}));
      }
      if(!(most > 0.0)) {
        continue;
      }
      const shift::core::cad::Vec2 mid{0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])};
      const bool cross_s = crossed(loops_s, box), cross_o = crossed(loops_o, box);
      if((!cross_s && !holds(loops_s, mid)) || (!cross_o && !holds(loops_o, mid))) {
        continue;
      }
      AreaLoad sum;
      double cell_deepest = 0.0;
      shift::core::cad::Vec2 cell_deepest_at{0.0, 0.0};
      if(!cross_s || !cross_o) {
        clip_square(box, cross_s ? &loops_s : cross_o ? &loops_o : nullptr, sum,
                    cell_deepest, cell_deepest_at);
      } else {
        const double du = cu / kPlaneAreaSubsamples, dv = cv / kPlaneAreaSubsamples;
        for(int a = 0; a < kPlaneAreaSubsamples; a++) {
          for(int b = 0; b < kPlaneAreaSubsamples; b++) {
            const double sub[4] = {box[0] + a * du, box[1] + b * dv,
                                   box[0] + (a + 1) * du, box[1] + (b + 1) * dv};
            const shift::core::cad::Vec2 at{0.5 * (sub[0] + sub[2]), 0.5 * (sub[1] + sub[3])};
            if(holds(loops_s, at) && holds(loops_o, at)) {
              clip_square(sub, nullptr, sum, cell_deepest, cell_deepest_at);
            }
          }
        }
      }
      if(!(sum.area > 0.0)) {
        continue;
      }
      const shift::core::cad::Vec2 c{sum.mx / sum.area, sum.my / sum.area};
      const double d = sum.load / sum.area;
      const ksl_vec3_t p = Axpy(Axpy(ps, c[0], su), c[1], sv);
      const ksl_vec3_t q = Axpy(p, d, no);
      shift::core::collision_geometry::ContactPoint point =
        FeaturePoint(s_is_a, p, q, no, d, 0.0);
      point.area = jacobian * sum.area;
      points.push_back(point);
      if(cell_deepest > deepest) {
        deepest = cell_deepest;
        deepest_point = points.size() - 1;
        deepest_at = cell_deepest_at;
      }
    }
  }
  /* whether the deepest point is on s's bound: s's edge or vertex in o */
  if(points.size() > first) {
    const double near = 1e-9 * BoxDiagonal(f.box);
    for(const auto& loop : loops_s) {
      for(size_t i = 0, j = loop.size() - 1; i < loop.size() && !at_bound; j = i++) {
        const shift::core::cad::Vec2& a = loop[j];
        const shift::core::cad::Vec2& b = loop[i];
        const double ex = b[0] - a[0], ey = b[1] - a[1];
        const double length = ex * ex + ey * ey;
        double t = length > 0.0 ? ((deepest_at[0] - a[0]) * ex + (deepest_at[1] - a[1]) * ey) / length : 0.0;
        t = std::min(std::max(t, 0.0), 1.0);
        const double dx = (a[0] + t * ex - deepest_at[0]) * std::sqrt(guu);
        const double dy = (a[1] + t * ey - deepest_at[1]) * std::sqrt(gvv);
        at_bound = dx * dx + dy * dy <= near * near;
      }
    }
  }
  double patch = 0.0, point_deepest = 0.0;
  for(size_t k = first; k < points.size(); k++) {
    patch += points[k].area;
    point_deepest = std::max(point_deepest, points[k].depth);
  }
  for(size_t k = first; k < points.size(); k++) {
    points[k].patch_area = patch;
  }
  if(points.size() > first && deepest > point_deepest) {
    shift::core::collision_geometry::ContactPoint& c = points[deepest_point];
    c.area *= c.depth / deepest;
    c.depth = deepest;
    if(s_is_a) {
      c.point_b = Axpy(c.point_a, -deepest, c.normal);
    } else {
      c.point_a = Axpy(c.point_b, deepest, c.normal);
    }
  }
}

/* a contact of nearly conformal faces (see ConformalPair) that FacePair
leaves to its caller to share between its area and the rest: the points of its
area are [area_begin, area_end), those of a point or a line [area_end, end).
touching: whether the faces touch at a point or along a line */
struct NearPair {
  int fa, fb;
  size_t area_begin, area_end, end;
  bool touching;
};

/* the share of the contact of nearly conformal faces that their area carries
(see ConformalPair), from the points of the areas of near: the half angle over
which the contact wraps round the faces, estimated from the spread of their
normals about the mean, weighted by area (the root mean square angle of a band
of uniform width over [-t, t] is t / sqrt(3)), goes from kWrapLeast to
kWrapFull. The areas of faces next to each other over smooth seams, such as
the halves of a sphere, are one contact: their wrap is that of them all */
static double ContactWrap(const std::vector<shift::core::collision_geometry::ContactPoint>& points,
                          const NearPair* near, size_t count) {
  ksl_vec3_t mean = {{0.0, 0.0, 0.0}};
  double area = 0.0;
  for(size_t i = 0; i < count; i++) {
    for(size_t k = near[i].area_begin; k < near[i].area_end; k++) {
      mean = Axpy(mean, points[k].area, points[k].normal);
      area += points[k].area;
    }
  }
  const double length = ksl_vec3_l2norm(&mean);
  if(!(area > 0.0) || !(length > 0.0)) {
    return 0.0;
  }
  mean = Scaled(mean, 1.0 / length);
  double spread = 0.0;
  for(size_t i = 0; i < count; i++) {
    for(size_t k = near[i].area_begin; k < near[i].area_end; k++) {
      const ksl_vec3_t d = Sub(points[k].normal, mean);
      spread += points[k].area * Dot(d, d);
    }
  }
  const double half_angle = std::sqrt(3.0 * spread / area);
  return SmoothStep((half_angle - kWrapLeast) / (kWrapFull - kWrapLeast));
}

/* the deepest point of the points or lines of the nearly conformal faces
near, where it is deeper than every point of their areas, which miss it
between the points of their grids: the point of the areas nearest to it is
given its depth, with its area reduced so that its force is the same */
static void DeepenArea(std::vector<shift::core::collision_geometry::ContactPoint>& points,
                       const NearPair* near, size_t count) {
  size_t feature = points.size(), area = points.size();
  for(size_t i = 0; i < count; i++) {
    for(size_t k = near[i].area_begin; k < near[i].end; k++) {
      size_t& best = k < near[i].area_end ? area : feature;
      if(best == points.size() || points[k].depth > points[best].depth) {
        best = k;
      }
    }
  }
  if(feature == points.size() || area == points.size() ||
     !(points[feature].depth > points[area].depth)) {
    return;
  }
  const ksl_vec3_t at = points[feature].point_a;
  size_t nearest = area;
  double least = HUGE_VAL;
  for(size_t i = 0; i < count; i++) {
    for(size_t k = near[i].area_begin; k < near[i].area_end; k++) {
      const ksl_vec3_t d = Sub(points[k].point_a, at);
      if(Dot(d, d) < least && points[k].depth > 0.0) {
        least = Dot(d, d);
        nearest = k;
      }
    }
  }
  shift::core::collision_geometry::ContactPoint& c = points[nearest];
  const double deepest = points[feature].depth;
  c.area *= c.depth / deepest;
  c.depth = deepest;
  c.point_b = Axpy(c.point_a, -deepest, c.normal);
}

/* shares the contact of near between its area, wrap, and its point or line,
1 - wrap; the points left with no weight are marked with a weight of -1, for
the caller to remove. true if the faces still touch */
static bool ShareWrap(std::vector<shift::core::collision_geometry::ContactPoint>& points,
                      const NearPair& near, double wrap) {
  for(size_t k = near.area_begin; k < near.end; k++) {
    const double share = k < near.area_end ? wrap : 1.0 - wrap;
    points[k].weight = share > 0.0 ? points[k].weight * share : -1.0;
  }
  return (wrap > 0.0 && near.area_end > near.area_begin) ||
         (wrap < 1.0 && near.touching);
}

/* the contact of two faces if they conform (see Conforms): true, with the
points of their area added; false if they do not. Whether they conform is kept
in the cache while the faces move by less than half a sample's spacing. Nearly
conformal faces, near, have their area's points added at full weight, for
FacePair to share their contact with that of a point or a line (see
ContactWrap): light, it is a point's or a line's, as a ball in a socket a
little larger; as the faces press in and it wraps round them, their area's. Two
flat faces conform if they face each other (see PlaneAreaContact): facing,
the share of the contact law their area carries, goes smoothly from 0 to 1 as
the cosine of the angle of their normals from opposite goes from kFacingLeast
to kFacingFull, and as the share of the smaller face in contact goes from
kFlushLeast to kFlushFull; the edges and vertices of the faces carry the rest
(see ShapePoints) */
static bool ConformalPair(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                          const shift::core::collision_geometry::CadSideOfPair& b, int fb,
                          double max_depth,
                          std::vector<shift::core::collision_geometry::ContactPoint>& points,
                          shift::core::collision_geometry::CadPairCache* cache,
                          double& facing, bool& near) {
  facing = 1.0;
  near = false;
  if(!MayConform(a, fa, b, fb)) {
    return false;
  }
  /* two planes conform where they face each other (see MayConform) */
  if(SurfaceOf(a, fa).type == shift::core::cad::Surface::Type::kPlane &&
     SurfaceOf(b, fb).type == shift::core::cad::Surface::Type::kPlane) {
    const auto& face_a = a.shape->faces[fa];
    const auto& face_b = b.shape->faces[fb];
    const shift::core::cad::Vec2 mid_a{0.5 * (face_a.lo[0] + face_a.hi[0]),
                                       0.5 * (face_a.lo[1] + face_a.hi[1])};
    const shift::core::cad::Vec2 mid_b{0.5 * (face_b.lo[0] + face_b.hi[0]),
                                       0.5 * (face_b.lo[1] + face_b.hi[1])};
    facing = SmoothStep(
      (-Dot(SideNormal(a, fa, mid_a), SideNormal(b, fb, mid_b)) - kFacingLeast) /
      (kFacingFull - kFacingLeast));
    if(!(facing > 0.0)) {
      return false;
    }
    /* the depth along the normal of the face that the other's edge or vertex
    is in, at the deepest point (a face tilted into the other, its edge in it,
    measured along the other's normal): or else of the lesser depth, as the
    least translation that parts them (of faces as deep, along the larger's),
    blended over kDepthBand of the greater depth so that the contact changes
    smoothly from one to the other */
    thread_local std::vector<shift::core::collision_geometry::ContactPoint> in_b, in_a;
    in_b.clear();
    in_a.clear();
    bool bound_a, bound_b;
    PlaneAreaContact(a, fa, b, fb, true, in_b, bound_a);
    PlaneAreaContact(b, fb, a, fa, false, in_a, bound_b);
    auto deepest = [](const std::vector<shift::core::collision_geometry::ContactPoint>& v) {
      double d = 0.0;
      for(const shift::core::collision_geometry::ContactPoint& c : v) {
        d = std::max(d, c.depth);
      }
      return d;
    };
    const double depth_b = deepest(in_b), depth_a = deepest(in_a);
    const double most = std::max(depth_a, depth_b);
    double share_b;
    if(in_a.empty() || in_b.empty()) {
      share_b = in_a.empty() ? 1.0 : 0.0;
    } else if(bound_a != bound_b) {
      share_b = bound_a ? 1.0 : 0.0;
    } else if(std::fabs(depth_b - depth_a) <= 1e-9 * most) {
      share_b = BoxDiagonal(face_a.box) <= BoxDiagonal(face_b.box) ? 1.0 : 0.0;
    } else {
      share_b = SmoothStep(0.5 + (depth_a - depth_b) / (kDepthBand * most));
    }
    /* the share of the smaller face in contact */
    auto contact_area = [](const std::vector<shift::core::collision_geometry::ContactPoint>& v) {
      return v.empty() ? 0.0 : v.front().patch_area;
    };
    const double smaller = std::min(PlaneFaceArea(a, fa), PlaneFaceArea(b, fb));
    const double touching =
      share_b * contact_area(in_b) + (1.0 - share_b) * contact_area(in_a);
    facing *= smaller > 0.0 ? SmoothStep((touching / smaller - kFlushLeast) /
                                         (kFlushFull - kFlushLeast))
                            : 0.0;
    if(!(facing > 0.0)) {
      return false;
    }
    for(const auto* in : {&in_b, &in_a}) {
      const double share = in == &in_b ? share_b : 1.0 - share_b;
      if(!(share > 0.0)) {
        continue;
      }
      for(shift::core::collision_geometry::ContactPoint c : *in) {
        c.weight *= facing * share;
        points.push_back(c);
      }
    }
    return true;
  }
  shift::core::collision_geometry::CadPairCache::Entry* entry =
    cache ? cache->Find(fa, fb) : nullptr;
  const double margin = 0.5 * (SampleSpacing(a.shape->faces[fa]) +
                               SampleSpacing(b.shape->faces[fb]));
  bool small_is_a;
  int level;
  if(entry && entry->conform >= 0 &&
     BoxMotion(b.shape->faces[fb].box, entry->conform_d, b.d) <= margin) {
    level = entry->conform;
    small_is_a = entry->small_is_a;
  } else {
    level = Conforms(a, fa, b, fb, max_depth, small_is_a);
    if(cache) {
      if(!entry) {
        cache->entries.emplace_back();
        entry = &cache->entries.back();
        entry->fa = fa;
        entry->fb = fb;
      }
      entry->conform = level;
      entry->small_is_a = small_is_a;
      entry->conform_d = b.d;
    }
  }
  if(level == 0) {
    return false;
  }
  if(small_is_a) {
    AreaContact(a, fa, b, fb, true, points);
  } else {
    AreaContact(b, fb, a, fa, false, points);
  }
  if(level == 1) {
    return true;
  }
  near = true;
  return true;
}

/* adds the contact of two faces; true if they touch */
static bool FeatureFacePair(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                            int fb, double max_depth,
                            std::vector<shift::core::collision_geometry::ContactPoint>& points,
                            shift::core::collision_geometry::CadPairCache* cache);

/* adds the contact of two faces; true if they touch. Faces of the same shape
touch over an area (see ConformalPair); nearly conformal faces over an area,
at a point or along a line, or both, as their contact wraps round them: with
near given, that is left to the caller (see NearPair), as the contact may go
on over the seams of the faces. facing: the share of the contact of two flat
faces that their area carries */
static bool FacePair(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                     int fb, double max_depth,
                     std::vector<shift::core::collision_geometry::ContactPoint>& points,
                     shift::core::collision_geometry::CadPairCache* cache = nullptr,
                     double* facing = nullptr, NearPair* near = nullptr) {
  const size_t before = points.size();
  double share = 1.0;
  bool nearly = false;
  const bool conform =
    ConformalPair(a, fa, b, fb, max_depth, points, cache, share, nearly);
  if(facing) {
    *facing = share;
  }
  if(conform && !nearly) {
    return points.size() > before;
  }
  const size_t area_end = points.size();
  const bool touching = FeatureFacePair(a, fa, b, fb, max_depth, points, cache);
  if(!conform) {
    return touching;
  }
  const NearPair pair{fa, fb, before, area_end, points.size(), touching};
  if(near) {
    *near = pair;
    return touching || points.size() > before;
  }
  const double wrap = ContactWrap(points, &pair, 1);
  if(wrap >= 1.0) {
    DeepenArea(points, &pair, 1);
  }
  const bool touches = ShareWrap(points, pair, wrap);
  points.erase(std::remove_if(points.begin() + before, points.end(),
                              [](const shift::core::collision_geometry::ContactPoint& c) {
                                return c.weight < 0.0;
                              }),
               points.end());
  return touches;
}

/* adds the contact of two faces at points and along lines; true if they
touch */
static bool FeatureFacePair(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                            int fb, double max_depth,
                            std::vector<shift::core::collision_geometry::ContactPoint>& points,
                            shift::core::collision_geometry::CadPairCache* cache) {
  const size_t before = points.size();
  auto touching = [&]() { return points.size() > before; };
  /* a line along a seam is found on the faces of both sides of it */
  auto line_touching = [&]() {
    return DropRepeated(points, before, 1e-9 * a.shape->size) || touching();
  };
  const shift::core::cad::Surface::Type ta = SurfaceOf(a, fa).type;
  const shift::core::cad::Surface::Type tb = SurfaceOf(b, fb).type;
  if(ta == shift::core::cad::Surface::Type::kCylinder && tb == shift::core::cad::Surface::Type::kCylinder) {
    CylinderCylinder(a, fa, b, fb, max_depth, points);
    return line_touching();
  } else if(ta == shift::core::cad::Surface::Type::kPlane && tb == shift::core::cad::Surface::Type::kCylinder) {
    PlaneCylinderFaces(a, fa, b, fb, true, max_depth, points);
    return line_touching();
  } else if(ta == shift::core::cad::Surface::Type::kCylinder && tb == shift::core::cad::Surface::Type::kPlane) {
    PlaneCylinderFaces(b, fb, a, fa, false, max_depth, points);
    return line_touching();
  } else if(ta == shift::core::cad::Surface::Type::kPlane && tb == shift::core::cad::Surface::Type::kPlane) {
    return false; // their edges and vertices touch
  }
  auto straight = [](shift::core::cad::Surface::Type t) {
    return t == shift::core::cad::Surface::Type::kCylinder || t == shift::core::cad::Surface::Type::kPlane;
  };
  bool repeated = false;
  if((IsCurved(ta) && straight(tb) &&
      CurvedFacePair(a, fa, b, fb, true, max_depth, points, repeated)) ||
     (IsCurved(tb) && straight(ta) &&
      CurvedFacePair(b, fb, a, fa, false, max_depth, points, repeated))) {
    return repeated || touching();
  }
  thread_local std::vector<FaceSeed> seeds;
  /* the seeds from the closest-point fields of both faces, if both have them */
  const bool field = a.fields && b.fields;
  /* the seeds kept from the last test, while the faces have moved too little
  for new regions of contact to appear */
  shift::core::collision_geometry::CadPairCache::Entry* entry =
    cache ? cache->Find(fa, fb) : nullptr;
  if(entry && entry->max_depth == max_depth &&
     BoxMotion(b.shape->faces[fb].box, entry->d, b.d) <= entry->margin) {
    seeds = entry->seeds;
  } else {
    if(field) {
      FieldSeeds(a, fa, b, fb, max_depth, seeds);
    } else {
      FaceSeeds(a, fa, b, fb, max_depth, seeds);
    }
    if(cache) {
      if(!entry) {
        cache->entries.emplace_back();
        entry = &cache->entries.back();
        entry->fa = fa;
        entry->fb = fb;
      }
      entry->max_depth = max_depth;
      entry->margin = 0.5 * (SampleSpacing(a.shape->faces[fa]) +
                             SampleSpacing(b.shape->faces[fb]));
      entry->d = b.d;
      entry->seeds = seeds;
    }
  }
  if(seeds.empty()) {
    return false;
  }
  double x[4];
  std::copy(seeds[0].x, seeds[0].x + 4, x);
  shift::core::collision_geometry::ContactPoint c;
  const bool curved = IsCurved(ta) && IsCurved(tb);
  const double damping = curved ? 1e-6 : 0.0;
  /* from a seed's climb, if the faces are not apart at its end: along the
  face of whose samples the seed is a maximum, since the sample of the other
  face paired with it may lie far from the region */
  /* the top of the last climb, before NewtonPoint */
  double top[4];
  double reached = -HUGE_VAL;
  bool climbed_top = false;
  /* the seeds from which a point was found are kept as that point, the others
  as the top of their climb, where the next climb starts */
  auto settle = [&](size_t k, bool found) {
    if(!entry) {
      return;
    }
    FaceSeed& kept = entry->seeds[k];
    if(found) {
      std::copy(x, x + 4, kept.x);
      kept.settled = true;
      kept.apart = 0.0;
    } else if(climbed_top) {
      std::copy(top, top + 4, kept.x);
      kept.of_b = false;
      kept.apart = 0.0;
    } else if(reached < 0.0 && reached != -HUGE_VAL) {
      kept.apart = -reached;
      kept.apart_at = b.d;
    }
  };
  auto climbed_point = [&](const FaceSeed& seed) {
    climbed_top = false;
    reached = -HUGE_VAL;
    if(seed.settled) {
      std::copy(seed.x, seed.x + 4, x);
      return NewtonPoint(a, fa, b, fb, max_depth, x, c, damping);
    }
    if(seed.apart > 0.0 && entry &&
       BoxMotion(b.shape->faces[fb].box, seed.apart_at, b.d) < seed.apart) {
      return false;
    }
    /* a seed from the fields is a sample projected on the other face at a
    maximum of its depth, within a sample's spacing of the deepest point of
    its region: Newton's method from it, then the climb if it finds none */
    if(field) {
      std::copy(seed.x, seed.x + 4, x);
      if(NewtonPoint(a, fa, b, fb, max_depth, x, c, damping)) {
        return true;
      }
    }
    double climbed = -HUGE_VAL;
    if(seed.of_b) {
      double y[4] = {seed.x[2], seed.x[3], seed.x[0], seed.x[1]};
      if(!Climb(b, fb, a, fa, y, climbed, &reached)) {
        return false;
      }
      x[0] = y[2];
      x[1] = y[3];
      x[2] = y[0];
      x[3] = y[1];
    } else {
      std::copy(seed.x, seed.x + 4, x);
      if(!Climb(a, fa, b, fb, x, climbed, &reached)) {
        return false;
      }
    }
    if(climbed == -HUGE_VAL) {
      return false;
    }
    std::copy(x, x + 4, top);
    climbed_top = true;
    return NewtonPoint(a, fa, b, fb, max_depth, x, c, damping);
  };
  bool point;
  point = climbed_point(seeds[0]);
  settle(0, point);
  /* two faces of revolution, one with an axis: a ring if they touch round
  it, from the end of the climb, as the deepest point of a ring is any of
  its points and Newton's method may find none */
  if(OfRevolution(ta) && OfRevolution(tb) && (HasAxis(ta) || HasAxis(tb))) {
    double y[4];
    double climbed = -HUGE_VAL;
    bool ring_seed;
    if(seeds[0].settled) {
      std::copy(seeds[0].x, seeds[0].x + 4, y);
      ring_seed = true;
      climbed = 0.0;
    } else if(seeds[0].of_b) {
      double z[4] = {seeds[0].x[2], seeds[0].x[3], seeds[0].x[0],
                     seeds[0].x[1]};
      ring_seed = Climb(b, fb, a, fa, z, climbed);
      y[0] = z[2];
      y[1] = z[3];
      y[2] = z[0];
      y[3] = z[1];
    } else {
      std::copy(seeds[0].x, seeds[0].x + 4, y);
      ring_seed = Climb(a, fa, b, fb, y, climbed);
    }
    bool repeated = false;
    if(entry && !point && ring_seed && climbed != -HUGE_VAL) {
      std::copy(y, y + 4, entry->seeds[0].x);
      entry->seeds[0].settled = true;
    }
    if(ring_seed && climbed != -HUGE_VAL &&
       RingLine(a, fa, b, fb, point ? x : y, max_depth, points, repeated)) {
      return repeated || touching();
    }
  }
  /* two curved faces: a line if they touch along one, as tooth flanks do */
  if(curved) {
    bool repeated = false;
    if(CurvedCurvedLine(a, fa, b, fb, point ? x : nullptr, max_depth, points,
                        repeated)) {
      return repeated || touching();
    }
  }
  /* on a seam both faces find a point, or next to it, to within their
  normals: it counts for the first face only */
  const double seam = 1e-5 * a.shape->size;
  bool on_seam = false;
  const size_t found = points.size();
  if(point) {
    points.push_back(c);
    on_seam = DropRepeated(points, before, seam);
  }
  /* each other region in which the faces touch: the search from its seed
  ends at a deepest point apart from those found */
  auto known = [&](const shift::core::collision_geometry::ContactPoint& fresh) {
    for(size_t i = found; i < points.size(); i++) {
      const ksl_vec3_t d = Sub(fresh.point_a, points[i].point_a);
      if(ksl_vec3_l2norm(&d) <= seam) {
        return true;
      }
    }
    return false;
  };
  for(size_t k = 1; k < seeds.size(); k++) {
    /* climbed first: from the seed itself the search may end between the
    regions, or in one already found */
    const bool found = climbed_point(seeds[k]);
    settle(k, found);
    if(!found || known(c) || !DeepestNear(a, fa, b, fb, x)) {
      continue;
    }
    points.push_back(c);
    on_seam = DropRepeated(points, before, seam) || on_seam;
  }
  return on_seam || touching();
}

/* ------------------------------------------------------------------------ */
/* the contact data of a pair */

/* the deepest penetration taken for CAD geometry against other geometry: see
Base::get_max_depth. It does not grow with a large shape by default: the far
side of a small part would then count as inside it */
static double PairMaxDepth(const shift::core::collision_geometry::Cad& cad,
                           const shift::core::collision_geometry::Base& other) {
  return other.get_max_depth() > 0.0 ? other.get_max_depth()
                                     : cad.get_max_depth();
}

/* the contact data of a pair from its points in the frame of a */
static bool Finish(shift::core::collision_geometry::ContactPair& pair, const ksl_SE3_t& frame,
                   std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  pair.success = true;
  pair.collision = !points.empty();
  pair.point_laws = true;
  pair.depth_converged = true;
  if(!pair.collision) {
    return false;
  }
  size_t deepest = 0;
  for(size_t i = 0; i < points.size(); i++) {
    shift::core::collision_geometry::ContactPoint& c = points[i];
    for(ksl_vec3_t* p : {&c.point_a, &c.point_b}) {
      const ksl_vec3_t local = *p;
      ksl_product_dv(&frame, &local, p);
    }
    const ksl_vec3_t n = c.normal;
    ksl_product_drv(&frame.R, &n, &c.normal);
    if(c.depth > points[deepest].depth) {
      deepest = i;
    }
  }
  const shift::core::collision_geometry::ContactPoint& d = points[deepest];
  pair.depth = d.depth;
  pair.normal = d.normal;
  pair.point_a = d.point_a;
  pair.point_b = d.point_b;
  ksl_product_dinvv(&pair.a->get_d_absolute(), &pair.point_a,
                    &pair.point_local_a);
  ksl_product_dinvv(&pair.b->get_d_absolute(), &pair.point_b,
                    &pair.point_local_b);
  pair.manifold = std::move(points);
  return true;
}

/* a set of pairs of indices, kept between tests: its memory follows the pairs
inserted, not the sizes of the shapes, and Clear costs nothing */
class IndexPairSet {
public:
  void Clear() {
    size_ = 0;
    if(++generation_ == 0) {
      std::fill(stamps_.begin(), stamps_.end(), 0u);
      generation_ = 1;
    }
  }

  /* false if the pair was in the set */
  bool Insert(int i, int j) {
    if(2 * (size_ + 1) > keys_.size()) {
      Grow();
    }
    const uint64_t key = Key(i, j);
    const size_t slot = Find(key);
    if(stamps_[slot] == generation_) {
      return false;
    }
    keys_[slot] = key;
    stamps_[slot] = generation_;
    size_++;
    return true;
  }

  bool Contains(int i, int j) const {
    return !keys_.empty() && stamps_[Find(Key(i, j))] == generation_;
  }

private:
  static uint64_t Key(int i, int j) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(i)) << 32) |
           static_cast<uint32_t>(j);
  }

  /* the slot of the key, or the empty slot where it goes */
  size_t Find(uint64_t key) const {
    const size_t mask = keys_.size() - 1;
    size_t slot = (key * 0x9E3779B97F4A7C15ull >> 20) & mask;
    while(stamps_[slot] == generation_ && keys_[slot] != key) {
      slot = (slot + 1) & mask;
    }
    return slot;
  }

  void Grow() {
    std::vector<uint64_t> keys(std::max<size_t>(64, 2 * keys_.size()));
    std::vector<unsigned> stamps(keys.size(), 0u);
    keys.swap(keys_);
    stamps.swap(stamps_);
    for(size_t k = 0; k < keys.size(); k++) {
      if(stamps[k] == generation_) {
        const size_t slot = Find(keys[k]);
        keys_[slot] = keys[k];
        stamps_[slot] = generation_;
      }
    }
  }

  std::vector<uint64_t> keys_;
  std::vector<unsigned> stamps_;
  size_t size_ = 0;
  unsigned generation_ = 1;
};

/* bounds computed at most once in a test, by index */
class BoxCache {
public:
  void Reset(size_t count) {
    boxes_.resize(count);
    have_.assign(count, 0);
  }

  template <typename Compute>
  const shift::core::collision_geometry::CadBox& Get(int i,
                                                     const Compute& compute) {
    if(!have_[i]) {
      boxes_[i] = compute();
      have_[i] = 1;
    }
    return boxes_[i];
  }

private:
  std::vector<shift::core::collision_geometry::CadBox> boxes_;
  std::vector<char> have_;
};

/* whether faces f and g of shape are one face or next to each other over a
smooth seam */
static bool SmoothlyJoined(const shift::core::collision_geometry::CadContactShape& shape,
                           int f, int g) {
  if(f == g) {
    return true;
  }
  for(int e : shape.faces[f].edges) {
    const auto& edge = shape.edges[e];
    if(!edge.sharp &&
       std::find(edge.faces.begin(), edge.faces.end(), g) != edge.faces.end()) {
      return true;
    }
  }
  return false;
}

/* shares the contacts of the nearly conformal faces near between their areas
and their points or lines (see ShareWrap), each by the wrap of all the pairs
it joins over smooth seams on both sides (see ContactWrap), and adds the faces
that still touch to touching. Pairs so joined are one contact: the patch area
of each of their points is that of all their area, and its deepest point is
kept (see DeepenArea) */
static void ShareNearContact(const shift::core::collision_geometry::CadSideOfPair& a,
                             const shift::core::collision_geometry::CadSideOfPair& b,
                             std::vector<NearPair>& near,
                             std::vector<shift::core::collision_geometry::ContactPoint>& points,
                             IndexPairSet& touching, std::vector<int>& touching_a,
                             std::vector<int>& touching_b) {
  /* gather the joined pairs together, one group after another */
  size_t first = 0;
  while(first < near.size()) {
    size_t end = first + 1;
    for(size_t i = first; i < end; i++) {
      size_t j = end;
      while(j < near.size()) {
        if(SmoothlyJoined(*a.shape, near[i].fa, near[j].fa) &&
           SmoothlyJoined(*b.shape, near[i].fb, near[j].fb)) {
          std::swap(near[j], near[end]); // near[j] is now one not yet seen
          end++;
          j = std::max(j, end);
        } else {
          j++;
        }
      }
    }
    /* one contact: its size, for its stiffness, is that of all its area (the
    patch areas of the pairs, as a point's area may be reduced to carry the
    deepest depth: see AreaContact) */
    double patch = 0.0;
    for(size_t i = first; i < end; i++) {
      if(near[i].area_end > near[i].area_begin) {
        patch += points[near[i].area_begin].patch_area;
      }
    }
    for(size_t i = first; i < end; i++) {
      for(size_t k = near[i].area_begin; k < near[i].area_end; k++) {
        points[k].patch_area = patch;
      }
    }
    const double wrap = ContactWrap(points, near.data() + first, end - first);
    if(wrap >= 1.0) {
      DeepenArea(points, near.data() + first, end - first);
    }
    for(size_t i = first; i < end; i++) {
      if(ShareWrap(points, near[i], wrap)) {
        touching.Insert(near[i].fa, near[i].fb);
        touching_a.push_back(near[i].fa);
        touching_b.push_back(near[i].fb);
      }
    }
    first = end;
  }
  size_t lowest = near.front().area_begin;
  for(const NearPair& pair : near) {
    lowest = std::min(lowest, pair.area_begin);
  }
  points.erase(std::remove_if(points.begin() + lowest, points.end(),
                              [](const shift::core::collision_geometry::ContactPoint& c) {
                                return c.weight < 0.0;
                              }),
               points.end());
}

/* what ShapePoints works with, kept so that a test allocates nothing; a thread
each, as pairs are tested in parallel */
struct ShapeScratch {
  std::vector<std::pair<int, int>> candidates;
  std::vector<int> found;
  IndexPairSet touching, edges_a, vertices_a, edges_b, vertices_b, edge_pairs;
  BoxCache near_a, near_b, near_edge, wide_a, wide_b;
  std::vector<shift::core::collision_geometry::ContactPoint> probe_points;
};

/* the points of two shapes, in a's frame, to max_depth. The edges and
vertices of each side within search of a face of the other, and not in it to
max_depth, are probed to search: the points they have there are kept in
beyond_reach (see ShapeOverlap) */
static void ShapePoints(const shift::core::collision_geometry::CadSideOfPair& a, const shift::core::collision_geometry::CadSideOfPair& b,
                        double max_depth, double search,
                        std::vector<shift::core::collision_geometry::ContactPoint>& points,
                        shift::core::collision_geometry::CadPairCache* cache = nullptr) {
  if(!a.shape->box.Overlaps(b.shape->box.Transformed(b.d))) {
    return;
  }

  static thread_local ShapeScratch scratch;
  auto& [candidates, found, touching, edges_a, vertices_a, edges_b, vertices_b,
         edge_pairs, near_a, near_b, near_edge, wide_a, wide_b,
         probe_points] = scratch;

  /* the pairs of faces whose boxes overlap */
  candidates.clear();
  for(int fb = 0; fb < static_cast<int>(b.shape->faces.size()); fb++) {
    found.clear();
    const shift::core::collision_geometry::CadBox box =
      b.shape->faces[fb].box.Transformed(b.d);
    if(SurfaceOf(b, fb).type != shift::core::cad::Surface::Type::kPlane) {
      a.shape->Query(box, found);
      for(int fa : found) {
        candidates.push_back({fa, fb});
      }
      continue;
    }
    /* two flat faces pressed together are apart by their depth, as are their
    flat boxes: a plane's is padded by max_depth for the other planes */
    shift::core::collision_geometry::CadBox padded = box;
    padded.Pad(max_depth);
    a.shape->Query(padded, found);
    for(int fa : found) {
      if(SurfaceOf(a, fa).type == shift::core::cad::Surface::Type::kPlane ||
         a.shape->faces[fa].box.Overlaps(box)) {
        candidates.push_back({fa, fb});
      }
    }
  }
  touching.Clear();
  std::vector<int> touching_a, touching_b; // few: the faces that touch
  /* flat faces touching over an area that carries only a share of their
  contact (see ConformalPair): their edges and vertices carry the rest */
  struct Facing {
    int fa, fb;
    double share;
  };
  std::vector<Facing> facing;
  /* nearly conformal faces, whose contact is shared between area and point
  or line once all of it is found (see NearPair) */
  std::vector<NearPair> near;
  for(const auto& [fa, fb] : candidates) {
    double share = 1.0;
    NearPair pair{-1, -1, 0, 0, 0, false};
    const bool touch = FacePair(a, fa, b, fb, max_depth, points, cache, &share, &pair);
    if(pair.fa >= 0) {
      near.push_back(pair);
      continue;
    }
    if(touch) {
      if(share < 1.0) {
        facing.push_back({fa, fb, share});
        continue;
      }
      touching.Insert(fa, fb);
      touching_a.push_back(fa);
      touching_b.push_back(fb);
    }
  }
  if(!near.empty()) {
    ShareNearContact(a, b, near, points, touching, touching_a, touching_b);
  }

  /* edges and vertices of faces that touch no face of the other side, where
  no other face of theirs does */
  auto touches = [&](const std::vector<int>& faces, int other, bool on_a) {
    for(int f : faces) {
      if(on_a ? touching.Contains(f, other) : touching.Contains(other, f)) {
        return true;
      }
    }
    return false;
  };
  /* whether any of faces touches any face of the other side */
  auto touches_any = [&](const std::vector<int>& faces, bool on_a) {
    for(int f : on_a ? touching_a : touching_b) {
      if(std::find(faces.begin(), faces.end(), f) != faces.end()) {
        return true;
      }
    }
    return false;
  };
  /* whether any of faces, or a face smoothly next to one of them, touches
  other, a face of the other side */
  auto touches_or_beside = [&](const std::vector<int>& faces, int other, bool on_a) {
    if(touches(faces, other, on_a)) {
      return true;
    }
    const shift::core::collision_geometry::CadContactShape& shape = *(on_a ? a : b).shape;
    for(int f : faces) {
      for(int e : shape.faces[f].edges) {
        if(!shape.edges[e].sharp && touches(shape.edges[e].faces, other, on_a)) {
          return true;
        }
      }
    }
    return false;
  };
  /* whether a contact reaches over a smooth seam to an edge of faces in face
  of the other side: faces, or a face smoothly next to one of them, touch face
  or a face smoothly next to it. The edge is then the end of that contact, on
  either side of the seam */
  auto touches_beside = [&](const std::vector<int>& faces, int face, bool on_a) {
    if(touching_a.empty()) {
      return false;
    }
    if(touches_or_beside(faces, face, on_a)) {
      return true;
    }
    const shift::core::collision_geometry::CadContactShape& shape = *(on_a ? b : a).shape;
    for(int e : shape.faces[face].edges) {
      if(shape.edges[e].sharp) {
        continue;
      }
      for(int f : shape.edges[e].faces) {
        if(f != face && touches_or_beside(faces, f, on_a)) {
          return true;
        }
      }
    }
    return false;
  };
  /* the transformation of a's frame to b's, for the bounds of a's faces */
  ksl_SE3_t a_in_b;
  ksl_SE3_inverted(&b.d, &a_in_b);
  for(auto* set : {&edges_a, &vertices_a, &edges_b, &vertices_b, &edge_pairs}) {
    set->Clear();
  }
  near_a.Reset(a.shape->faces.size());
  near_b.Reset(b.shape->faces.size());
  near_edge.Reset(b.shape->edges.size());
  wide_a.Reset(a.shape->faces.size());
  wide_b.Reset(b.shape->faces.size());

  /* sharp edges lying along a cylinder of the other side, where no face of the
  edge touches */
  std::vector<std::array<int, 3>> lines_tried;
  auto edge_along = [&](const shift::core::collision_geometry::CadSideOfPair& feature, int e,
                        const shift::core::collision_geometry::CadSideOfPair& cylinder, int fc, bool on_a,
                        IndexPairSet& edges, IndexPairSet& vertices) {
    const auto& edge = feature.shape->edges[e];
    const std::array<int, 3> key = {e, fc, on_a ? 1 : 0};
    if(!edge.sharp || touches_any(edge.faces, on_a) ||
       std::find(lines_tried.begin(), lines_tried.end(), key) !=
         lines_tried.end()) {
      return;
    }
    lines_tried.push_back(key);
    if(!EdgeCylinderLine(feature, e, cylinder, fc, on_a, max_depth, points)) {
      return;
    }
    edges.Insert(e, fc);
    /* its ends */
    const auto& topology = feature.shape->model().edges[edge.edge];
    const auto& place = feature.shape->placements[edge.placement];
    for(int f : edge.faces) {
      for(int v : feature.shape->faces[f].vertices) {
        for(int end : {topology.start, topology.end}) {
          const auto& model = feature.shape->model().vertices[end].point;
          const ksl_vec3_t d =
            Sub(feature.shape->vertices[v].point, place.Apply(model));
          if(ksl_vec3_l2norm(&d) <= 1e-9 * feature.shape->size) {
            vertices.Insert(v, fc);
          }
        }
      }
    }
  };
  for(const auto& [fa, fb] : candidates) {
    if(touching.Contains(fa, fb)) {
      continue;
    }
    if(SurfaceOf(b, fb).type == shift::core::cad::Surface::Type::kCylinder) {
      const shift::core::collision_geometry::CadBox box = b.shape->faces[fb].box.Transformed(b.d);
      for(int e : a.shape->faces[fa].edges) {
        if(a.shape->edges[e].box.Overlaps(box)) {
          edge_along(a, e, b, fb, true, edges_a, vertices_a);
        }
      }
    }
    if(SurfaceOf(a, fa).type == shift::core::cad::Surface::Type::kCylinder) {
      const shift::core::collision_geometry::CadBox& box = a.shape->faces[fa].box;
      for(int e : b.shape->faces[fb].edges) {
        if(box.Overlaps(b.shape->edges[e].box.Transformed(b.d))) {
          edge_along(b, e, a, fa, false, edges_b, vertices_b);
        }
      }
    }
  }

  /* sharp edges crossing, where no face of either touches a face of the other;
  each then has nothing to do with the faces of the other */
  std::vector<CornerFade> fades; // few: corners with a crossing beside them
  for(const auto& [fa, fb] : candidates) {
    if(touching.Contains(fa, fb)) {
      continue;
    }
    for(int eb : b.shape->faces[fb].edges) {
      const auto& edge_b = b.shape->edges[eb];
      if(!edge_b.sharp) {
        continue;
      }
      const shift::core::collision_geometry::CadBox& bounds = near_edge.Get(eb, [&]() {
        shift::core::collision_geometry::CadBox box = edge_b.box.Transformed(b.d);
        box.Pad(max_depth);
        return box;
      });
      for(int ea : a.shape->faces[fa].edges) {
        const auto& edge_a = a.shape->edges[ea];
        if(!edge_a.sharp || !edge_a.box.Overlaps(bounds) ||
           !edge_pairs.Insert(ea, eb)) {
          continue;
        }
        /* faces of both already touching: it is that contact's end */
        const bool face_touches = touches_any(edge_a.faces, true) &&
                                  touches_any(edge_b.faces, false);
        if(!face_touches && EdgeEdge(a, ea, b, eb, max_depth, fades, points)) {
          for(int f : edge_b.faces) {
            edges_a.Insert(ea, f);
          }
          for(int f : edge_a.faces) {
            edges_b.Insert(eb, f);
          }
        }
      }
    }
  }

  /* the share of its force a corner keeps in a face */
  auto corner_share = [&](bool on_a, int vertex, int face) {
    double keep = 1.0;
    for(const CornerFade& fade : fades) {
      if(fade.on_a == on_a && fade.vertex == vertex && fade.face == face) {
        keep *= fade.keep;
      }
    }
    return keep;
  };
  /* the points of edges in faces and of corners in faces, by their feature */
  struct FeatureInFace {
    bool on_a;
    int feature, face;
    size_t point;
  };
  std::vector<FeatureInFace> edge_points, corner_points;
  for(const auto& [fa, fb] : candidates) {
    if(touching.Contains(fa, fb)) {
      continue;
    }
    /* each face's bounds in the other's frame, enlarged by the deepest
    penetration: the features of the other near it */
    const shift::core::collision_geometry::CadBox& bounds_b = near_b.Get(fb, [&]() {
      shift::core::collision_geometry::CadBox box = b.shape->faces[fb].box.Transformed(b.d);
      box.Pad(max_depth);
      return box;
    });
    const shift::core::collision_geometry::CadBox& bounds_a = near_a.Get(fa, [&]() {
      shift::core::collision_geometry::CadBox box = a.shape->faces[fa].box.Transformed(a_in_b);
      box.Pad(max_depth);
      return box;
    });
    auto inside = [](const shift::core::collision_geometry::CadBox& box, const ksl_vec3_t& p) {
      for(int k = 0; k < 3; k++) {
        if(p.at[k] < box.lo.at[k] || p.at[k] > box.hi.at[k]) {
          return false;
        }
      }
      return true;
    };
    /* the bounds enlarged by search, for the probes */
    const bool probing = search > max_depth;
    const shift::core::collision_geometry::CadBox& wide_bounds_b = wide_b.Get(fb, [&]() {
      shift::core::collision_geometry::CadBox box = b.shape->faces[fb].box.Transformed(b.d);
      box.Pad(search);
      return box;
    });
    const shift::core::collision_geometry::CadBox& wide_bounds_a = wide_a.Get(fa, [&]() {
      shift::core::collision_geometry::CadBox box = a.shape->faces[fa].box.Transformed(a_in_b);
      box.Pad(search);
      return box;
    });
    /* a feature (an edge or vertex k of side a, with on_a, or b) with no
    point in a face to max_depth, probed to search */
    auto probe = [&](bool on_a, bool is_edge, int k, int face) {
      if(!probing) {
        return;
      }
      const shift::core::collision_geometry::CadSideOfPair& feature = on_a ? a : b;
      const shift::core::collision_geometry::CadSideOfPair& other = on_a ? b : a;
      probe_points.clear();
      if(is_edge) {
        EdgeFace(feature, k, other, face, on_a, search, false, probe_points);
      } else {
        VertexFace(feature, k, other, face, on_a, search, 1.0, probe_points);
      }
      for(const shift::core::collision_geometry::ContactPoint& c : probe_points) {
        Within(c, max_depth);
      }
    };
    for(int e : a.shape->faces[fa].edges) {
      const auto& box = a.shape->edges[e].box;
      if(!box.Overlaps(wide_bounds_b) || !edges_a.Insert(e, fb) ||
         touches(a.shape->edges[e].faces, fb, true)) {
        continue;
      }
      const size_t before = points.size();
      if(box.Overlaps(bounds_b) &&
         !touches_beside(a.shape->edges[e].faces, fb, true)) {
        EdgeFace(a, e, b, fb, true, max_depth,
                 !touches_any(a.shape->edges[e].faces, true) &&
                   !touches_any({fb}, false),
                 points);
        for(size_t k = before; k < points.size(); k++) {
          edge_points.push_back({true, e, fb, k});
        }
      }
      if(points.size() == before) {
        probe(true, true, e, fb);
      }
    }
    for(int v : a.shape->faces[fa].vertices) {
      const ksl_vec3_t& point = a.shape->vertices[v].point;
      if(!inside(wide_bounds_b, point) || !vertices_a.Insert(v, fb) ||
         touches(a.shape->vertices[v].faces, fb, true)) {
        continue;
      }
      const size_t before = points.size();
      if(inside(bounds_b, point)) {
        VertexFace(a, v, b, fb, true, max_depth, corner_share(true, v, fb), points);
        if(points.size() > before) {
          corner_points.push_back({true, v, fb, before});
        }
      }
      if(points.size() == before) {
        probe(true, false, v, fb);
      }
    }
    for(int e : b.shape->faces[fb].edges) {
      const auto& box = b.shape->edges[e].box;
      if(!box.Overlaps(wide_bounds_a) || !edges_b.Insert(e, fa) ||
         touches(b.shape->edges[e].faces, fa, false)) {
        continue;
      }
      const size_t before = points.size();
      if(box.Overlaps(bounds_a) &&
         !touches_beside(b.shape->edges[e].faces, fa, false)) {
        EdgeFace(b, e, a, fa, false, max_depth,
                 !touches_any(b.shape->edges[e].faces, false) &&
                   !touches_any({fa}, true),
                 points);
        for(size_t k = before; k < points.size(); k++) {
          edge_points.push_back({false, e, fa, k});
        }
      }
      if(points.size() == before) {
        probe(false, true, e, fa);
      }
    }
    for(int v : b.shape->faces[fb].vertices) {
      const ksl_vec3_t& point = b.shape->vertices[v].point;
      if(!inside(wide_bounds_a, point) || !vertices_b.Insert(v, fa) ||
         touches(b.shape->vertices[v].faces, fa, false)) {
        continue;
      }
      const size_t before = points.size();
      if(inside(bounds_a, point)) {
        VertexFace(b, v, a, fa, false, max_depth, corner_share(false, v, fa), points);
        if(points.size() > before) {
          corner_points.push_back({false, v, fa, before});
        }
      }
      if(points.size() == before) {
        probe(false, false, v, fa);
      }
    }
  }

  /* an edge of a face in a corner's own face, with the corner in that face: the
  edge of the part reaches into the wedge of the corner, and that is the same
  overlap, which the corner's point holds. It is the same overlap only near
  the corner: the overlap of a corner is a few times its depth across, while
  the rim of a face with a hole, say, is in the other's face far from the
  corners of the hole, an overlap of its own. Between 4 and 8 times the
  deeper of the two depths apart, the edge's point comes in smoothly */
  /* the edges and vertices of flat faces in partial area contact carry the
  share of their contact that the area does not */
  auto unshared = [&](const FeatureInFace& feature, bool is_edge) {
    const shift::core::collision_geometry::CadContactShape& own = *(feature.on_a ? a : b).shape;
    const auto& faces = is_edge ? own.edges[feature.feature].faces
                                : own.vertices[feature.feature].faces;
    double share = 0.0;
    for(const Facing& f : facing) {
      const int mine = feature.on_a ? f.fa : f.fb;
      const int theirs = feature.on_a ? f.fb : f.fa;
      if(theirs == feature.face &&
         std::find(faces.begin(), faces.end(), mine) != faces.end()) {
        share = std::max(share, f.share);
      }
    }
    return 1.0 - share;
  };
  if(!facing.empty()) {
    for(const FeatureInFace& edge : edge_points) {
      points[edge.point].weight *= unshared(edge, true);
    }
    for(const FeatureInFace& corner : corner_points) {
      points[corner.point].weight *= unshared(corner, false);
    }
  }
  std::vector<size_t> repeated;
  for(const FeatureInFace& edge : edge_points) {
    const shift::core::collision_geometry::CadContactShape& own = *(edge.on_a ? a : b).shape;
    const shift::core::collision_geometry::CadContactShape& other = *(edge.on_a ? b : a).shape;
    const auto& edge_faces = own.edges[edge.feature].faces;
    double keep = 1.0;
    for(const FeatureInFace& corner : corner_points) {
      if(corner.on_a == edge.on_a) {
        continue;
      }
      const auto& corner_faces = other.vertices[corner.feature].faces;
      if(std::find(edge_faces.begin(), edge_faces.end(), corner.face) != edge_faces.end() &&
         std::find(corner_faces.begin(), corner_faces.end(), edge.face) != corner_faces.end()) {
        const shift::core::collision_geometry::ContactPoint& e = points[edge.point];
        const shift::core::collision_geometry::ContactPoint& c = points[corner.point];
        const ksl_vec3_t apart = Sub(e.point_a, c.point_a);
        const double scale = 4.0 * std::max(e.depth, c.depth);
        keep = std::min(keep, SmoothStep((ksl_vec3_l2norm(&apart) - scale) / scale));
      }
    }
    if(keep <= 0.0) {
      repeated.push_back(edge.point);
    } else {
      points[edge.point].weight *= keep;
    }
  }
  std::sort(repeated.rbegin(), repeated.rend());
  for(size_t point : repeated) {
    points.erase(points.begin() + point);
  }
}

/*
Whether a point is inside geometry, at least depth from its surface in every
direction, as a point of contact that deep is: its depth is the distance to the
nearest point of the surface. A ray along the normal n of the contact, without
end, first meets the surface where it leaves the geometry; rays along n, both
ways, along two tangents, both ways, and along the 8 diagonals between them,
each to 0.9 of the depth, do not meet it (see Base::RayCast). A point on the far
side of a part, or in a thin overlap of two parts (a face a little into another,
paired with a side of it far off), is not. Convex geometry is left where a point
at that distance along a ray is outside it. Inertial frame.
*/
static bool DeepIn(const shift::core::collision_geometry::Base& geometry,
                   bool convex, const ksl_vec3_t& p, const ksl_vec3_t& n,
                   double depth) {
  double distance;
  ksl_vec3_t normal;
  if(!convex && !(geometry.RayCast(p, n, HUGE_VAL, distance, normal) &&
                  distance == 0.0)) {
    return false;
  }
  ksl_vec3_t t1, t2;
  Perpendiculars(n, t1, t2);
  const double length = 0.9 * depth;
  const double third = 1.0 / std::sqrt(3.0);
  for(int k = 0; k < 14; k++) {
    ksl_vec3_t r;
    if(k < 6) {
      const ksl_vec3_t& axis = k < 2 ? n : (k < 4 ? t1 : t2);
      r = Scaled(axis, k % 2 ? -1.0 : 1.0);
    } else {
      const int m = k - 6;
      r = Axpy(Axpy(Scaled(n, m & 1 ? -third : third), m & 2 ? -third : third,
                    t1),
               m & 4 ? -third : third, t2);
    }
    if(convex) {
      /* the end of the ray inside: a ray from it reaches the geometry at
      distance 0 */
      if(!geometry.RayCast(Axpy(p, length, r), r, 0.0, distance, normal)) {
        return false;
      }
    } else if(geometry.RayCast(p, r, length, distance, normal)) {
      return false;
    }
  }
  return true;
}

/*
The points of two shapes, in a's frame, at any depth at which their surfaces
overlap. A point deeper than max_depth may be on the far side of a part, or in
a thin overlap of the two, paired with a face far off: it is taken where
deep(point), with the point in a's frame, tells that its point of a is that
deep in b, or its point of b that deep in a (see DeepIn). Of a point of an
edge or vertex in a face, only the point of the edge or vertex is: the other
is by the edge. A part entirely
inside another, its surfaces apart, so has no contact.

ShapePoints takes points to a reach, at first max_depth, over which it also
searches for the features of each side near the faces of the other. Contact
shallower than half of max_depth is all there is. Without it, or with a deeper
point, the parts may be deeper than max_depth (the penetration of a step of the
integrator, or of a start, beyond it): the shapes are tested again with the
edges and vertices near the faces of the other, within 4 times the reach,
probed for the points they have beyond it. Where a point of contact is found
beyond the reach (within 4 times it), they are tested again, to 1.5 times its
depth: in all at most kPasses times, to half the size of the smaller shape.
*/
template <typename Deep>
static void ShapeOverlap(const shift::core::collision_geometry::CadSideOfPair& a,
                         const shift::core::collision_geometry::CadSideOfPair& b,
                         double max_depth, const Deep& deep,
                         std::vector<shift::core::collision_geometry::ContactPoint>& points,
                         shift::core::collision_geometry::CadPairCache* cache = nullptr) {
  constexpr int kPasses = 5;
  const double largest = 0.5 * std::min(a.shape->size, b.shape->size);
  double reach = max_depth;
  bool probe = false;
  for(int pass = 0;; pass++) {
    beyond_reach.clear();
    points.clear();
    /* the seeds kept from the last test are those of the first pass */
    ShapePoints(a, b, reach, probe ? 4.0 * reach : reach, points,
                pass == 0 ? cache : nullptr);
    double deepest = 0.0;
    if(reach > max_depth) {
      points.erase(std::remove_if(points.begin(), points.end(),
                                  [&](const shift::core::collision_geometry::ContactPoint& c) {
                                    return c.depth > max_depth && !deep(c);
                                  }),
                   points.end());
    }
    for(const shift::core::collision_geometry::ContactPoint& c : points) {
      deepest = std::max(deepest, c.depth);
    }
    if(pass + 1 == kPasses || reach >= largest ||
       (pass == 0 && !points.empty() && deepest <= 0.5 * max_depth)) {
      break;
    }
    /* the shallowest point beyond the reach that is a point of contact */
    std::sort(beyond_reach.begin(), beyond_reach.end(),
              [](const shift::core::collision_geometry::ContactPoint& x,
                 const shift::core::collision_geometry::ContactPoint& y) {
                return x.depth < y.depth;
              });
    double next = 0.0;
    for(const shift::core::collision_geometry::ContactPoint& c : beyond_reach) {
      if(deep(c)) {
        next = c.depth;
        break;
      }
    }
    if(next > 0.0) {
      reach = std::min(1.5 * next, largest);
    } else if(!probe) {
      probe = true;
    } else {
      break;
    }
  }
}

bool shift::core::collision_geometry::CadCadContact(shift::core::collision_geometry::ContactPair& pair) {
  auto cad_a = static_cast<const shift::core::collision_geometry::Cad*>(pair.a);
  auto cad_b = static_cast<const shift::core::collision_geometry::Cad*>(pair.b);
  shift::core::collision_geometry::CadSideOfPair a, b;
  a.shape = &cad_a->get_shape();
  b.shape = &cad_b->get_shape();
  a.edge_radius = cad_a->get_edge_radius();
  b.edge_radius = cad_b->get_edge_radius();
  if(cad_a->get_fields() && cad_b->get_fields()) {
    a.fields = cad_a->get_fields();
    b.fields = cad_b->get_fields();
  }
  ksl_product_dinvd(&pair.a->get_d_absolute(), &pair.b->get_d_absolute(),
                    &b.d);
  const ksl_SE3_t& frame = pair.a->get_d_absolute();
  auto deep = [&](const shift::core::collision_geometry::ContactPoint& c) {
    ksl_vec3_t pa, pb;
    ksl_product_dv(&frame, &c.point_a, &pa);
    ksl_product_dv(&frame, &c.point_b, &pb);
    ksl_vec3_t n;
    ksl_product_drv(&frame.R, &c.normal, &n);
    return DeepIn(*cad_b, false, pa, n, c.depth) ||
           DeepIn(*cad_a, false, pb, Scaled(n, -1.0), c.depth);
  };
  std::vector<shift::core::collision_geometry::ContactPoint> points;
  if(!pair.cad_cache) {
    pair.cad_cache =
      std::make_shared<shift::core::collision_geometry::CadPairCache>();
  }
  ShapeOverlap(a, b, std::min(cad_a->get_max_depth(), cad_b->get_max_depth()),
               deep, points, pair.cad_cache.get());
  return Finish(pair, frame, points);
}

/* ------------------------------------------------------------------------ */
/* CAD geometry on a plane */

/* the least height above a plane of the corners of a box, from the depth
below the plane of a point */
template <typename Depth>
static double Lowest(const shift::core::collision_geometry::CadBox& box,
                     const Depth& depth_of) {
  double lowest = HUGE_VAL;
  for(int corner = 0; corner < 8; corner++) {
    const ksl_vec3_t p = {{(corner & 1) ? box.hi.x : box.lo.x,
                           (corner & 2) ? box.hi.y : box.lo.y,
                           (corner & 4) ? box.hi.z : box.lo.z}};
    lowest = std::min(lowest, -depth_of(p));
  }
  return lowest;
}

bool shift::core::collision_geometry::CadPlaneContact(shift::core::collision_geometry::ContactPair& pair) {
  const bool plane_is_a = dynamic_cast<const Plane*>(pair.a) != nullptr;
  const Plane& plane = *static_cast<const Plane*>(plane_is_a ? pair.a : pair.b);
  auto cad = static_cast<const shift::core::collision_geometry::Cad*>(plane_is_a ? pair.b : pair.a);
  shift::core::collision_geometry::CadSideOfPair side;
  side.shape = &cad->get_shape();
  side.edge_radius = cad->get_edge_radius();
  const shift::core::collision_geometry::CadContactShape& shape = *side.shape;
  const ksl_SE3_t& frame = cad->get_d_absolute();
  const double max_depth = PairMaxDepth(*cad, plane);
  const double edge_curvature = 1.0 / side.edge_radius;

  /* the plane in the frame of the geometry */
  const ksl_vec3_t world_normal = plane.get_normal();
  ksl_vec3_t n, q;
  ksl_product_drinvv(&frame.R, &world_normal, &n);
  ksl_product_dinvv(&frame, &plane.get_point(), &q);

  std::vector<shift::core::collision_geometry::ContactPoint> points;
  auto add = [&](const ksl_vec3_t& p, double depth, double curvature,
                 double length, double ratio = 1.0) {
    if(!(depth > 0.0) || depth > max_depth) {
      return;
    }
    shift::core::collision_geometry::ContactPoint c;
    c.depth = depth;
    c.curvature = curvature;
    c.curvature_ratio = ratio;
    c.length = length;
    const ksl_vec3_t on_plane = Axpy(p, depth, n);
    c.point_a = plane_is_a ? on_plane : p;
    c.point_b = plane_is_a ? p : on_plane;
    c.normal = plane_is_a ? n : Scaled(n, -1.0);
    points.push_back(c);
  };
  auto depth_of = [&](const ksl_vec3_t& p) { return -Dot(Sub(p, q), n); };

  /* nothing below the plane: the bounds of the shape are above it */
  if(Lowest(shape.box, depth_of) > 0.0) {
    return Finish(pair, frame, points);
  }

  /* the lowest points of faces inside their trims */
  shift::core::collision_geometry::CadSideOfPair own; // the geometry in its own frame
  own.shape = side.shape;
  own.edge_radius = side.edge_radius;
  const auto& m = shape.model();
  std::vector<bool> touching(shape.faces.size(), false);
  bool repeated = false;
  auto face_points = [&](int fi) {
    const auto& f = shape.faces[fi];
    const shift::core::cad::Surface::Type type = SurfaceOf(own, fi).type;
    if(!SameSense(own, fi) && !IsCurved(type)) {
      return; // a concave analytic face: lowest at its edges
    }
    if(Lowest(f.box, depth_of) > 0.0) {
      return; // above the plane
    }
    if(type == shift::core::cad::Surface::Type::kPlane) {
      return; // its edges and vertices touch
    }
    if(type == shift::core::cad::Surface::Type::kCylinder) {
      /* in the geometry's own frame (own), the plane's side b or a */
      PlaneCylinder c;
      c.op = q;
      c.np = n;
      c.plane_is_a = plane_is_a;
      const size_t before = points.size();
      PlaneCylinderLine(c, nullptr, -1, own, fi, max_depth, points);
      DropRepeated(points, before, 1e-9 * own.shape->size);
      return;
    }
    /* other surfaces: Newton's method on the derivatives of the height,
    from the lowest sample facing the plane */
    int best = -1;
    double lowest_sample = HUGE_VAL;
    for(size_t i = 0; i < f.sample_uv.size(); i++) {
      const double h = -depth_of(f.sample_points[i]);
      if(Dot(f.sample_normals[i], n) < -0.5 && h < lowest_sample) {
        lowest_sample = h;
        best = static_cast<int>(i);
      }
    }
    if(best < 0) {
      return;
    }
    shift::core::cad::Vec2 uv = f.sample_uv[best];
    if(IsCurved(type)) {
      /* along a line, if the face is curved mainly in one direction */
      CurvedLine line;
      line.curved = &own;
      line.face = fi;
      line.curved_is_a = !plane_is_a;
      line.other_point = q;
      line.other_vector = n;
      if(CurvedFaceLine(line, uv, nullptr, -1, max_depth, points)) {
        repeated = line.repeated;
        return;
      }
    }
    const double span[2] = {f.hi[0] - f.lo[0], f.hi[1] - f.lo[1]};
    auto gradient = [&](const shift::core::cad::Vec2& x, double* g) {
      ksl_vec3_t du, dv;
      shape.Point(fi, x, &du, &dv);
      g[0] = Dot(du, n);
      g[1] = Dot(dv, n);
    };
    bool converged = false;
    for(int iteration = 0; iteration < 30 && !converged; iteration++) {
      double g[2], J[4];
      gradient(uv, g);
      for(int k = 0; k < 2; k++) {
        const double h = 1e-6 * std::max(span[k], 1e-9);
        shift::core::cad::Vec2 xp = uv, xm = uv;
        xp[k] += h;
        xm[k] -= h;
        double gp[2], gm[2];
        gradient(xp, gp);
        gradient(xm, gm);
        for(int i = 0; i < 2; i++) {
          J[i * 2 + k] = (gp[i] - gm[i]) / (2.0 * h);
        }
      }
      double minus_g[2] = {-g[0], -g[1]}, step[2];
      if(!Solve(J, minus_g, step, 2)) {
        break;
      }
      double limit = 1.0;
      for(int k = 0; k < 2; k++) {
        if(std::fabs(step[k]) > 0.2 * span[k]) {
          limit = std::min(limit, 0.2 * span[k] / std::fabs(step[k]));
        }
      }
      converged = true;
      for(int k = 0; k < 2; k++) {
        uv[k] += limit * step[k];
        converged = converged && std::fabs(step[k]) <= 1e-10 * span[k];
      }
    }
    if(converged && shape.Inside(fi, uv) &&
       Dot(shape.Normal(fi, uv), n) < -0.9) {
      const ksl_vec3_t p = shape.Point(fi, uv);
      shift::core::collision_geometry::ContactPoint gap;
      GapCurvature(own, fi, uv, nullptr, -1, uv, n, gap);
      add(p, depth_of(p), gap.curvature, 0.0, gap.curvature_ratio);
    }
  };
  for(int fi = 0; fi < static_cast<int>(shape.faces.size()); fi++) {
    const size_t before = points.size();
    repeated = false;
    face_points(fi);
    touching[fi] = repeated || points.size() > before;
  }
  /* vertices and edges of faces that do not touch the plane themselves */
  auto touches = [&](const std::vector<int>& faces) {
    for(int f : faces) {
      if(touching[f]) {
        return true;
      }
    }
    return false;
  };
  for(const auto& v : shape.vertices) {
    if(touches(v.faces)) {
      continue;
    }
    add(v.point, depth_of(v.point), edge_curvature, 0.0);
  }

  /* the lowest points of curved edges between their ends: closed form on a
  circle or ellipse, the deepest sample refined by golden section search
  otherwise */
  for(const auto& e : shape.edges) {
    if(touches(e.faces)) {
      continue;
    }
    const auto& edge = m.edges[e.edge];
    if(edge.curve < 0 || !edge.has_range) {
      continue;
    }
    const auto& curve = m.curves[edge.curve];
    const auto& place = shape.placements[e.placement];
    const double r0 = std::min(edge.range[0], edge.range[1]);
    const double r1 = std::max(edge.range[0], edge.range[1]);
    const double margin = 0.01 * (r1 - r0);
    if(curve.type == shift::core::cad::Curve3::Type::kLine) {
      continue; // lowest at a vertex
    }
    double t;
    if(curve.type == shift::core::cad::Curve3::Type::kConic) {
      /* h(t) = h0 + (a.n) cos t + (b.n) sin t is least at
      atan2(-(b.n), -(a.n)) */
      const ksl_vec3_t a = place.ApplyVector(curve.a);
      const ksl_vec3_t b = place.ApplyVector(curve.b);
      t = std::atan2(-Dot(b, n), -Dot(a, n));
      t = r0 + std::fmod(std::fmod(t - r0, 2.0 * M_PI) + 2.0 * M_PI,
                         2.0 * M_PI);
    } else {
      auto height = [&](double s) {
        return -depth_of(place.Apply(shift::core::cad::CurvePoint<3>(curve, s)));
      };
      double lo = r0, hi = r1;
      const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
      int best = 0;
      for(size_t i = 1; i < e.sample_t.size(); i++) {
        if(height(e.sample_t[i]) < height(e.sample_t[best])) {
          best = static_cast<int>(i);
        }
      }
      if(!e.sample_t.empty()) {
        lo = best > 0 ? e.sample_t[best - 1] : r0;
        hi = best + 1 < static_cast<int>(e.sample_t.size())
               ? e.sample_t[best + 1]
               : r1;
        lo = std::max(lo, r0);
        hi = std::min(hi, r1);
      }
      double t1 = hi - golden * (hi - lo), t2 = lo + golden * (hi - lo);
      double h1 = height(t1), h2 = height(t2);
      for(int i = 0; i < 40; i++) {
        if(h1 < h2) {
          hi = t2;
          t2 = t1;
          h2 = h1;
          t1 = hi - golden * (hi - lo);
          h1 = height(t1);
        } else {
          lo = t1;
          t1 = t2;
          h1 = h2;
          t2 = lo + golden * (hi - lo);
          h2 = height(t2);
        }
      }
      t = 0.5 * (lo + hi);
    }
    if(t < r0 + margin || t > r1 - margin) {
      continue; // at or beyond a vertex
    }
    const ksl_vec3_t p =
      place.Apply(shift::core::cad::CurvePoint<3>(curve, t));
    add(p, depth_of(p), edge_curvature, 0.0);
  }

  return Finish(pair, frame, points);
}

/* ------------------------------------------------------------------------ */
/* CAD geometry and a sphere or an ellipsoid */

/* a sphere or ellipsoid, in the frame of the CAD geometry: d transforms the
coordinates of its principal axes, about its center, to that frame */
struct Round {
  const shift::core::collision_geometry::Ellipsoid* ellipsoid = nullptr;
  ksl_SE3_t d;
  ksl_SE3_t frame; //!< of the CAD geometry, to the inertial frame
  double axes[3];  //!< semi axes
};

/* the point of the shape whose outward unit normal is m */
static ksl_vec3_t RoundPoint(const Round& r, const ksl_vec3_t& m) {
  ksl_vec3_t local, x;
  ksl_product_drinvv(&r.d.R, &m, &local);
  double s = 0.0;
  for(int k = 0; k < 3; k++) {
    x.at[k] = r.axes[k] * r.axes[k] * local.at[k];
    s += x.at[k] * local.at[k];
  }
  x = Scaled(x, 1.0 / std::sqrt(s));
  ksl_vec3_t p;
  ksl_product_dv(&r.d, &x, &p);
  return p;
}

/* adds the curvature tensor of the shape at its point e, in the unit tangents
t1 and t2, to h = (h11, h12, h22): that of the surface x^T A x = 1 is
t^T A s / |A x| */
static void AddRoundTensor(const Round& r, const ksl_vec3_t& e,
                           const ksl_vec3_t& t1, const ksl_vec3_t& t2,
                           double h[3]) {
  ksl_vec3_t x, a, b;
  ksl_product_dinvv(&r.d, &e, &x);
  ksl_product_drinvv(&r.d.R, &t1, &a);
  ksl_product_drinvv(&r.d.R, &t2, &b);
  double gradient = 0.0, k[3] = {0.0, 0.0, 0.0};
  for(int i = 0; i < 3; i++) {
    const double inverse = 1.0 / (r.axes[i] * r.axes[i]);
    gradient += inverse * x.at[i] * inverse * x.at[i];
    k[0] += inverse * a.at[i] * a.at[i];
    k[1] += inverse * a.at[i] * b.at[i];
    k[2] += inverse * b.at[i] * b.at[i];
  }
  gradient = std::sqrt(gradient);
  for(int i = 0; i < 3; i++) {
    h[i] += k[i] / gradient;
  }
}

/* the point q of the shape closest to p, with its outward unit normal n; the
distance is negative inside */
static double RoundClosest(const Round& r, const ksl_vec3_t& p, ksl_vec3_t& q,
                           ksl_vec3_t& n) {
  if(!r.ellipsoid) {
    const ksl_vec3_t v = Sub(p, r.d.t);
    const double length = ksl_vec3_l2norm(&v);
    n = length > 0.0 ? Scaled(v, 1.0 / length) : ksl_vec3_t{{1.0, 0.0, 0.0}};
    q = Axpy(r.d.t, r.axes[0], n);
    return length - r.axes[0];
  }
  ksl_vec3_t inertial, local_q, local_n;
  ksl_product_dv(&r.frame, &p, &inertial);
  double distance;
  shift::core::collision_geometry::ClosestOnEllipsoid(*r.ellipsoid, inertial,
                                                      local_q, local_n,
                                                      distance);
  ksl_product_dv(&r.d, &local_q, &q);
  ksl_product_drv(&r.d.R, &local_n, &n);
  return distance;
}

/* the equations of a point of contact with a face at (u, v): the line from the
face to the point of the shape facing it is along the face's normal */
static void RoundGradient(const shift::core::collision_geometry::CadSideOfPair& side, int face, const Round& r,
                          const shift::core::cad::Vec2& uv, double* g) {
  ksl_vec3_t du, dv;
  const ksl_vec3_t p = side.shape->Point(face, uv, &du, &dv);
  const ksl_vec3_t d =
    Sub(RoundPoint(r, Scaled(side.shape->Normal(face, uv), -1.0)), p);
  g[0] = Dot(d, du);
  g[1] = Dot(d, dv);
}

/* Newton's method on RoundGradient from uv */
static bool RoundNewton(const shift::core::collision_geometry::CadSideOfPair& side, int face, const Round& r,
                        shift::core::cad::Vec2& uv) {
  const auto& f = side.shape->faces[face];
  const double span[2] = {f.hi[0] - f.lo[0], f.hi[1] - f.lo[1]};
  for(int iteration = 0; iteration < 30; iteration++) {
    double g[2], J[4];
    RoundGradient(side, face, r, uv, g);
    for(int k = 0; k < 2; k++) {
      const double h = 1e-6 * std::max(span[k], 1e-9);
      shift::core::cad::Vec2 xp = uv, xm = uv;
      xp[k] += h;
      xm[k] -= h;
      double gp[2], gm[2];
      RoundGradient(side, face, r, xp, gp);
      RoundGradient(side, face, r, xm, gm);
      for(int i = 0; i < 2; i++) {
        J[i * 2 + k] = (gp[i] - gm[i]) / (2.0 * h);
      }
    }
    double minus_g[2] = {-g[0], -g[1]}, step[2];
    if(!Solve(J, minus_g, step, 2)) {
      return false;
    }
    double limit = 1.0;
    for(int k = 0; k < 2; k++) {
      if(std::fabs(step[k]) > 0.2 * span[k]) {
        limit = std::min(limit, 0.2 * span[k] / std::fabs(step[k]));
      }
    }
    bool converged = true;
    for(int k = 0; k < 2; k++) {
      uv[k] += limit * step[k];
      converged = converged && std::fabs(step[k]) <= 1e-10 * span[k];
    }
    if(converged) {
      return true;
    }
  }
  return false;
}

/* the contact point of a point p of the CAD geometry and a point e of the
shape, the normal n out of the CAD geometry, with the right sense: from a to b */
static shift::core::collision_geometry::ContactPoint RoundPair(bool cad_is_a, const ksl_vec3_t& p,
                              const ksl_vec3_t& e, const ksl_vec3_t& n,
                              double depth) {
  shift::core::collision_geometry::ContactPoint c;
  c.depth = depth;
  c.point_a = cad_is_a ? p : e;
  c.point_b = cad_is_a ? e : p;
  c.normal = cad_is_a ? n : Scaled(n, -1.0);
  return c;
}

/* the distance from the point p of a face, in the shape's frame, to its nearest
sharp edge, along the polyline of the edge's samples; HUGE_VAL if it has none */
static double DistanceToSharpEdge(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                                  const ksl_vec3_t& p) {
  const shift::core::collision_geometry::CadContactShape& shape = *side.shape;
  double nearest = HUGE_VAL;
  for(int e : shape.faces[face].edges) {
    if(shape.edges[e].sharp) {
      nearest = std::min(nearest, DistanceToEdge(shape, e, p));
    }
  }
  return nearest;
}

/* the point of contact of a face at a root (u, v) of RoundGradient; false if it
is not one: outside the trims, apart, a point of least depth rather than of
greatest, or with the center of the shape behind the face */
static bool RoundFacePoint(const shift::core::collision_geometry::CadSideOfPair& side, int face, const Round& r,
                           const shift::core::cad::Vec2& uv, bool cad_is_a, double max_depth,
                           shift::core::collision_geometry::ContactPoint& c) {
  if(!InsideOrOnEdge(side, face, uv)) {
    return false;
  }
  const shift::core::collision_geometry::CadContactShape& shape = *side.shape;
  const ksl_vec3_t p = shape.Point(face, uv);
  const ksl_vec3_t n = shape.Normal(face, uv);
  const ksl_vec3_t e = RoundPoint(r, Scaled(n, -1.0));
  const ksl_vec3_t d = Sub(p, e);
  const double depth = Dot(d, n);
  const ksl_vec3_t lateral = Axpy(d, -depth, n);
  if(!(depth > 0.0) || depth > max_depth ||
     ksl_vec3_l2norm(&lateral) > 0.05 * depth + 1e-9 * shape.size ||
     Dot(Sub(r.d.t, p), n) <= 0.0) {
    return false;
  }
  ksl_vec3_t t1, t2;
  Perpendiculars(n, t1, t2);
  double own[3] = {0.0, 0.0, 0.0}, h[3];
  AddRoundTensor(r, e, t1, t2, own);
  shape.Curvature(face, uv, t1, t2, h);
  for(int i = 0; i < 3; i++) {
    h[i] += own[i];
  }
  const double mean = 0.5 * (h[0] + h[2]);
  const double spread =
    std::sqrt(0.25 * (h[0] - h[2]) * (h[0] - h[2]) + h[1] * h[1]);
  const double scale = 0.5 * (own[0] + own[2]);
  if(mean - spread < -1e-6 * scale) {
    return false;
  }
  /* a shape seated in a face of its own curvature has a gap curved hardly at
  all: the limit keeps its stiffness finite */
  const double k2 = std::max(mean + spread, 1e-3 * scale);
  const double k1 = std::max(mean - spread, 1e-3 * k2);
  c = RoundPair(cad_is_a, p, e, n, depth);
  c.curvature = 0.5 * (k1 + k2);
  c.curvature_ratio = k2 / k1;

  /* within the Hertz radius of a sharp edge of the face, the stiffness goes
  over to that of the edge, rounded to edge_radius, which takes over there */
  const double hertz_radius = std::sqrt(depth / c.curvature);
  const double weight =
    SmoothStep(DistanceToSharpEdge(side, face, p) / hertz_radius);
  if(weight < 1.0) {
    const double edge_curvature = 1.0 / side.edge_radius + scale;
    const double compliance = weight / std::sqrt(c.curvature) +
                              (1.0 - weight) / std::sqrt(edge_curvature);
    c.curvature = 1.0 / (compliance * compliance);
    c.curvature_ratio = 1.0 + weight * (c.curvature_ratio - 1.0);
  }
  return true;
}

/* the points of a face touching the shape, which can be several: a shape in a
groove of one face touches both its sides. The roots start from the samples
nearest to touching that are well apart */
static bool RoundFace(const shift::core::collision_geometry::CadSideOfPair& side, int face, const Round& r,
                      bool cad_is_a, double max_depth,
                      std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& f = side.shape->faces[face];
  const ksl_vec3_t diagonal = Sub(f.box.hi, f.box.lo);
  const double spacing = ksl_vec3_l2norm(&diagonal) / 8.0;
  std::vector<std::pair<double, int>> scores;
  for(size_t i = 0; i < f.sample_uv.size(); i++) {
    const ksl_vec3_t d =
      Sub(f.sample_points[i], RoundPoint(r, Scaled(f.sample_normals[i], -1.0)));
    const double depth = Dot(d, f.sample_normals[i]);
    const ksl_vec3_t lateral = Axpy(d, -depth, f.sample_normals[i]);
    const double score = depth - ksl_vec3_l2norm(&lateral);
    if(score >= -max_depth - spacing) {
      scores.push_back({-score, static_cast<int>(i)});
    }
  }
  std::sort(scores.begin(), scores.end());
  std::vector<int> seeds;
  for(const auto& [score, i] : scores) {
    bool apart = true;
    for(int seed : seeds) {
      const ksl_vec3_t d = Sub(f.sample_points[i], f.sample_points[seed]);
      apart = apart && ksl_vec3_l2norm(&d) > 2.0 * spacing;
    }
    if(apart) {
      seeds.push_back(i);
    }
    if(seeds.size() == 4) {
      break;
    }
  }
  bool touching = false;
  for(int seed : seeds) {
    shift::core::cad::Vec2 uv = f.sample_uv[seed];
    /* the root for a sphere is the point closest to its center, which
    projection finds at the poles of a surface as well */
    const bool found = r.ellipsoid
                         ? RoundNewton(side, face, r, uv)
                         : side.shape->Project(face, r.d.t, uv, &f.sample_uv[seed]);
    shift::core::collision_geometry::ContactPoint c;
    if(found && RoundFacePoint(side, face, r, uv, cad_is_a, max_depth, c)) {
      AddDistinct(points, c, side.shape->size);
      touching = true;
    }
  }
  return touching;
}

/* a feature (edge or vertex) of the faces at its point p inside the shape, with
the outward normal n of the shape: a face of it looks at the shape */
static bool FacesRound(const shift::core::collision_geometry::CadSideOfPair& side, const std::vector<int>& faces,
                       const ksl_vec3_t& p, const ksl_vec3_t& n) {
  for(int f : faces) {
    shift::core::cad::Vec2 uv;
    if(side.shape->Project(f, p, uv) && Dot(side.shape->Normal(f, uv), n) < 0.0) {
      return true;
    }
  }
  return false;
}

/* the contact of a point p of an edge or vertex with the shape, rounded to the
geometry's edge_radius; false if it is outside the shape or too deep */
static bool RoundFeaturePoint(const shift::core::collision_geometry::CadSideOfPair& side, const Round& r,
                              const std::vector<int>& faces,
                              const ksl_vec3_t& p, bool cad_is_a,
                              double max_depth, shift::core::collision_geometry::ContactPoint& c) {
  ksl_vec3_t q, n;
  const double depth = -RoundClosest(r, p, q, n);
  if(!(depth > 0.0) || depth > max_depth || !FacesRound(side, faces, p, n)) {
    return false;
  }
  c = RoundPair(cad_is_a, p, q, Scaled(n, -1.0), depth);
  ksl_vec3_t t1, t2;
  Perpendiculars(n, t1, t2);
  double h[3] = {0.0, 0.0, 0.0};
  AddRoundTensor(r, q, t1, t2, h);
  c.curvature = 1.0 / side.edge_radius + 0.5 * (h[0] + h[2]);
  return true;
}

/* the point of an edge deepest in the shape: the deepest of its samples,
touching or not, refined by golden section search between its neighbors;
its ends are left to its vertices */
static bool RoundEdge(const shift::core::collision_geometry::CadSideOfPair& side, int edge, const Round& r,
                      bool cad_is_a, double max_depth,
                      std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const auto& e = side.shape->edges[edge];
  if(e.sample_t.empty()) {
    return false;
  }
  const auto& m = side.shape->model();
  const auto& curve = m.curves[m.edges[e.edge].curve];
  const auto& place = side.shape->placements[e.placement];
  auto point_at = [&](double t) {
    return place.Apply(shift::core::cad::CurvePoint<3>(curve, t));
  };
  auto depth_at = [&](double t) {
    ksl_vec3_t q, n;
    return -RoundClosest(r, point_at(t), q, n);
  };
  const int count = static_cast<int>(e.sample_t.size());
  int best = 0;
  double deepest = -HUGE_VAL;
  for(int i = 0; i < count; i++) {
    const double depth = depth_at(e.sample_t[i]);
    if(depth > deepest) {
      deepest = depth;
      best = i;
    }
  }
  const auto& range = m.edges[e.edge].range;
  double lo = best > 0 ? e.sample_t[best - 1] : range[0];
  double hi = best + 1 < count ? e.sample_t[best + 1] : range[1];
  const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
  double t1 = hi - golden * (hi - lo), t2 = lo + golden * (hi - lo);
  double d1 = depth_at(t1), d2 = depth_at(t2);
  for(int i = 0; i < 40; i++) {
    if(d1 > d2) {
      hi = t2;
      t2 = t1;
      d2 = d1;
      t1 = hi - golden * (hi - lo);
      d1 = depth_at(t1);
    } else {
      lo = t1;
      t1 = t2;
      d1 = d2;
      t2 = lo + golden * (hi - lo);
      d2 = depth_at(t2);
    }
  }
  const double t = 0.5 * (lo + hi);
  const double extent = std::fabs(range[1] - range[0]);
  if(std::fabs(t - range[0]) < 1e-6 * extent ||
     std::fabs(t - range[1]) < 1e-6 * extent) {
    return false; // at a vertex
  }
  shift::core::collision_geometry::ContactPoint c;
  if(!RoundFeaturePoint(side, r, e.faces, point_at(t), cad_is_a, max_depth, c)) {
    return false;
  }
  AddDistinct(points, c, side.shape->size);
  return true;
}

/* the points of a shape on CAD geometry, in the geometry's frame */
static void RoundPoints(const shift::core::collision_geometry::CadSideOfPair& side, const Round& r,
                        bool cad_is_a, double max_depth,
                        std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  const shift::core::collision_geometry::CadContactShape& shape = *side.shape;
  /* the bounds of the shape: along x_i, the support of G is sqrt(G_ii) */
  shift::core::collision_geometry::CadBox bounds;
  for(int i = 0; i < 3; i++) {
    ksl_vec3_t direction = {{0.0, 0.0, 0.0}};
    direction.at[i] = 1.0;
    const ksl_vec3_t high = RoundPoint(r, direction);
    const double reach = high.at[i] - r.d.t.at[i];
    bounds.lo.at[i] = r.d.t.at[i] - reach;
    bounds.hi.at[i] = r.d.t.at[i] + reach;
  }
  if(!shape.box.Overlaps(bounds)) {
    return;
  }
  std::vector<int> near;
  shape.Query(bounds, near);

  std::vector<bool> touching(shape.faces.size(), false);
  for(int f : near) {
    touching[f] = RoundFace(side, f, r, cad_is_a, max_depth, points);
  }

  /* edges of faces that do not touch, then vertices where neither their
  faces nor edges do */
  auto any = [](const std::vector<bool>& flags, const std::vector<int>& faces) {
    for(int f : faces) {
      if(flags[f]) {
        return true;
      }
    }
    return false;
  };
  std::vector<bool> edge_touching = touching;
  std::set<int> edges, vertices;
  for(int f : near) {
    edges.insert(shape.faces[f].edges.begin(), shape.faces[f].edges.end());
    vertices.insert(shape.faces[f].vertices.begin(),
                    shape.faces[f].vertices.end());
  }
  for(int e : edges) {
    const auto& edge = shape.edges[e];
    if(edge.box.Overlaps(bounds) && !any(touching, edge.faces) &&
       RoundEdge(side, e, r, cad_is_a, max_depth, points)) {
      for(int f : edge.faces) {
        edge_touching[f] = true;
      }
    }
  }
  for(int v : vertices) {
    const auto& vertex = shape.vertices[v];
    shift::core::collision_geometry::ContactPoint c;
    if(!any(edge_touching, vertex.faces) &&
       RoundFeaturePoint(side, r, vertex.faces, vertex.point, cad_is_a,
                         max_depth, c)) {
      AddDistinct(points, c, shape.size);
    }
  }
}

bool shift::core::collision_geometry::CadRoundContact(shift::core::collision_geometry::ContactPair& pair) {
  const bool cad_is_a = dynamic_cast<const shift::core::collision_geometry::Cad*>(pair.a) != nullptr;
  const Base* other = cad_is_a ? pair.b : pair.a;
  auto cad = static_cast<const shift::core::collision_geometry::Cad*>(cad_is_a ? pair.a : pair.b);
  shift::core::collision_geometry::CadSideOfPair side; // in its own frame
  side.shape = &cad->get_shape();
  side.edge_radius = cad->get_edge_radius();
  const ksl_SE3_t& frame = cad->get_d_absolute();

  Round r;
  r.frame = frame;
  r.ellipsoid = dynamic_cast<const Ellipsoid*>(other);
  ksl_product_dinvd(&frame, &other->get_d_absolute(), &r.d);
  if(r.ellipsoid) {
    const ksl_vec3_t& axes = r.ellipsoid->get_semi_axes();
    for(int k = 0; k < 3; k++) {
      r.axes[k] = axes.at[k];
    }
  } else {
    r.axes[0] = r.axes[1] = r.axes[2] =
      static_cast<const Sphere*>(other)->get_radius();
  }
  std::vector<shift::core::collision_geometry::ContactPoint> points;
  RoundPoints(side, r, cad_is_a, PairMaxDepth(*cad, *other), points);
  return Finish(pair, frame, points);
}

/* ------------------------------------------------------------------------ */
/* CAD geometry and a shape with exact faces of its own */

/*
The ends of a capsule of frame d (in the geometry's frame), of radius and half
length, on CAD geometry (side a of the pair), added to the points of its
cylinder. Each end is a sphere, whose points count on its hemisphere only. A
line of contact along the cylinder gives way to the end as it tilts: the
cylinder's rim point, at the share the line has left, says how much of the
contact the hemisphere takes over, and is replaced by it. Under a line that
keeps all its share, the hemisphere takes none.
*/
static void CapsuleEnds(const shift::core::collision_geometry::CadSideOfPair& a, const ksl_SE3_t& d,
                        double radius, double half, double max_depth,
                        std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  std::vector<bool> replaced(points.size(), false);
  std::vector<shift::core::collision_geometry::ContactPoint> added;
  auto along = [&](const ksl_vec3_t& p) {
    ksl_vec3_t local;
    ksl_product_dinvv(&d, &p, &local);
    return local.z;
  };
  for(double end : {-1.0, 1.0}) {
    Round r;
    r.frame = d;
    r.d = d;
    r.axes[0] = r.axes[1] = r.axes[2] = radius;
    const ksl_vec3_t offset = {{0.0, 0.0, end * half}};
    ksl_product_dv(&d, &offset, &r.d.t);
    std::vector<shift::core::collision_geometry::ContactPoint> sphere;
    RoundPoints(a, r, true, max_depth, sphere);
    for(shift::core::collision_geometry::ContactPoint& c : sphere) {
      if(end * along(c.point_b) < half) {
        continue; // on the cylinder
      }
      double share = 1.0;
      bool rim = false;
      for(size_t i = 0; i < replaced.size(); i++) {
        const shift::core::collision_geometry::ContactPoint& p = points[i];
        if(Dot(p.normal, c.normal) < 0.9 ||
           end * along(p.point_b) < 0.0) {
          continue;
        }
        if(p.length == 0.0 && end * along(p.point_b) >= half * (1.0 - 1e-9)) {
          share = p.weight;
          rim = true;
          replaced[i] = true;
        } else if(p.length > 0.0 && !rim) {
          share = 0.0;
        }
      }
      if(share > 0.0) {
        c.weight = share;
        added.push_back(c);
      }
    }
  }
  std::vector<shift::core::collision_geometry::ContactPoint> kept;
  for(size_t i = 0; i < points.size(); i++) {
    if(!replaced[i]) {
      kept.push_back(points[i]);
    }
  }
  kept.insert(kept.end(), added.begin(), added.end());
  points = std::move(kept);
}

bool shift::core::collision_geometry::CadExactContact(shift::core::collision_geometry::ContactPair& pair) {
  const bool cad_is_a = dynamic_cast<const shift::core::collision_geometry::Cad*>(pair.a) != nullptr;
  const Base* other = cad_is_a ? pair.b : pair.a;
  auto cad = static_cast<const shift::core::collision_geometry::Cad*>(cad_is_a ? pair.a : pair.b);
  const shift::core::collision_geometry::CadContactShape& exact = *other->get_exact_shape();
  const ksl_SE3_t& frame = cad->get_d_absolute();
  const double max_depth = PairMaxDepth(*cad, *other);

  /* the CAD geometry as side a, in its own frame */
  shift::core::collision_geometry::CadSideOfPair a, b;
  a.shape = &cad->get_shape();
  a.edge_radius = cad->get_edge_radius();
  b.shape = &exact;
  b.edge_radius = other->get_exact_edge_radius() > 0.0
                    ? other->get_exact_edge_radius()
                    : 0.002 * exact.size;
  ksl_SE3_t d, centered = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  ksl_product_dinvd(&frame, &other->get_d_absolute(), &d);
  centered.t = exact.center;
  ksl_product_dd(&d, &centered, &b.d);

  auto capsule = dynamic_cast<const Capsule*>(other);
  if(capsule) {
    b.edge_radius = capsule->get_radius();
  }
  std::vector<shift::core::collision_geometry::ContactPoint> points;
  /* the other geometry is convex */
  auto deep = [&](const shift::core::collision_geometry::ContactPoint& c) {
    ksl_vec3_t pa, pb, n;
    ksl_product_dv(&frame, &c.point_a, &pa);
    ksl_product_dv(&frame, &c.point_b, &pb);
    ksl_product_drv(&frame.R, &c.normal, &n);
    return DeepIn(*other, true, pa, n, c.depth) ||
           DeepIn(*cad, false, pb, Scaled(n, -1.0), c.depth);
  };
  if(!exact.faces.empty()) {
    ShapeOverlap(a, b, max_depth, deep, points);
  }
  if(capsule) {
    CapsuleEnds(a, d, capsule->get_radius(), capsule->get_half_length(),
                max_depth, points);
  }
  if(!cad_is_a) {
    for(shift::core::collision_geometry::ContactPoint& c : points) {
      std::swap(c.point_a, c.point_b);
      c.normal = Scaled(c.normal, -1.0);
    }
  }
  return Finish(pair, frame, points);
}
