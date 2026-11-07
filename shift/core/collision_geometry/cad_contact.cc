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
      if(c.depth > 0.0 && c.depth <= max_depth) {
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
    if(p.depth <= max_depth && SideProject(a, fa, p.point_a, uva) &&
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
    if(inside && r.depth <= max_depth) {
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

/* the gradient of half the squared distance between a point of each face,
x = (u_a, v_a, u_b, v_b): zero where the points are nearest, or deepest */
static void DistanceGradient(const shift::core::collision_geometry::CadSideOfPair& a, int fa,
                             const shift::core::collision_geometry::CadSideOfPair& b, int fb, const double* x,
                             double* g) {
  ksl_vec3_t pa_u, pa_v, pb_u, pb_v;
  const ksl_vec3_t pa = SidePoint(a, fa, {x[0], x[1]}, &pa_u, &pa_v);
  const ksl_vec3_t pb = SidePoint(b, fb, {x[2], x[3]}, &pb_u, &pb_v);
  const ksl_vec3_t d = Sub(pa, pb);
  g[0] = Dot(d, pa_u);
  g[1] = Dot(d, pa_v);
  g[2] = -Dot(d, pb_u);
  g[3] = -Dot(d, pb_v);
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

/* the pair of sample points of two faces deepest in each other with opposed
normals, x = (u_a, v_a, u_b, v_b); false if the faces are apart */
static bool SeedPoint(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                      int fb, double max_depth, double* x) {
  const auto& face_a = a.shape->faces[fa];
  const auto& face_b = b.shape->faces[fb];
  double best = -HUGE_VAL;
  for(size_t i = 0; i < face_a.sample_uv.size(); i++) {
    const ksl_vec3_t pa = SidePosition(a, face_a.sample_points[i]);
    const ksl_vec3_t na = SideVector(a, face_a.sample_normals[i]);
    for(size_t j = 0; j < face_b.sample_uv.size(); j++) {
      const ksl_vec3_t nb = SideVector(b, face_b.sample_normals[j]);
      if(Dot(na, nb) > -0.5) {
        continue;
      }
      const ksl_vec3_t d = Sub(pa, SidePosition(b, face_b.sample_points[j]));
      const double depth = Dot(d, na);
      const ksl_vec3_t lateral = Axpy(d, -depth, na);
      const double score = depth - ksl_vec3_l2norm(&lateral);
      if(score > best) {
        best = score;
        x[0] = face_a.sample_uv[i][0];
        x[1] = face_a.sample_uv[i][1];
        x[2] = face_b.sample_uv[j][0];
        x[3] = face_b.sample_uv[j][1];
      }
    }
  }
  /* samples are a grid's spacing apart: a pair this far from touching
  may be beside the deepest point */
  auto spacing = [](const shift::core::collision_geometry::CadContactFace& f) {
    const ksl_vec3_t d = Sub(f.box.hi, f.box.lo);
    return ksl_vec3_l2norm(&d) / 8.0;
  };
  return best != -HUGE_VAL &&
         best >= -max_depth - std::max(spacing(face_a), spacing(face_b));
}

/* the deepest point of two faces in contact: Newton's method on the
gradient of their distance, from the sample points x of SeedPoint */
static bool NewtonPoint(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                        int fb, double max_depth, double* x, shift::core::collision_geometry::ContactPoint& c) {
  const auto& face_a = a.shape->faces[fa];
  const auto& face_b = b.shape->faces[fb];
  const double span[4] = {face_a.hi[0] - face_a.lo[0],
                          face_a.hi[1] - face_a.lo[1],
                          face_b.hi[0] - face_b.lo[0],
                          face_b.hi[1] - face_b.lo[1]};
  bool converged = false;
  for(int iteration = 0; iteration < 30 && !converged; iteration++) {
    double g[4], J[16];
    DistanceGradient(a, fa, b, fb, x, g);
    for(int k = 0; k < 4; k++) {
      const double h = 1e-6 * std::max(span[k], 1e-9);
      double xp[4], xm[4], gp[4], gm[4];
      std::copy(x, x + 4, xp);
      std::copy(x, x + 4, xm);
      xp[k] += h;
      xm[k] -= h;
      DistanceGradient(a, fa, b, fb, xp, gp);
      DistanceGradient(a, fa, b, fb, xm, gm);
      for(int i = 0; i < 4; i++) {
        J[i * 4 + k] = (gp[i] - gm[i]) / (2.0 * h);
      }
    }
    double minus_g[4] = {-g[0], -g[1], -g[2], -g[3]}, step[4];
    if(!Solve(J, minus_g, step, 4)) {
      return false;
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
      x[k] += limit * step[k];
      converged = converged && std::fabs(step[k]) <= 1e-10 * span[k];
    }
  }
  if(!converged) {
    return false;
  }
  const shift::core::cad::Vec2 uva{x[0], x[1]}, uvb{x[2], x[3]};
  if(!a.shape->Inside(fa, uva) || !b.shape->Inside(fb, uvb)) {
    return false;
  }
  const ksl_vec3_t pa = SidePoint(a, fa, uva);
  const ksl_vec3_t pb = SidePoint(b, fb, uvb);
  const ksl_vec3_t na = SideNormal(a, fa, uva);
  const ksl_vec3_t nb = SideNormal(b, fb, uvb);
  const ksl_vec3_t d = Sub(pa, pb);
  const double depth = Dot(d, na);
  const ksl_vec3_t lateral = Axpy(d, -depth, na);
  if(Dot(na, nb) > -0.9 || !(depth > 0.0) || depth > max_depth ||
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
  /* the parameters of the last section found, where the next one starts */
  mutable shift::core::cad::Vec2 uv{0.0, 0.0};
  /* whether the face touches, with its points the same as another's */
  bool repeated = false;
};

/* the depth of the point p in the cylinder or plane of c, its nearest point
q there and the outward normal; -HUGE_VAL on the axis of the cylinder */
static double OtherDepth(const CurvedLine& c, const ksl_vec3_t& p,
                         ksl_vec3_t& q, ksl_vec3_t& outward) {
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
                            ksl_vec3_t* dv = nullptr) {
  ksl_vec3_t pu, pv, q, outward;
  const ksl_vec3_t p = SidePoint(*c.curved, c.face, uv, &pu, &pv);
  if(OtherDepth(c, p, q, outward) == -HUGE_VAL) {
    return false;
  }
  ksl_vec3_t along =
    Axpy(Scaled(pu, Dot(pv, c.direction)), -Dot(pu, c.direction), pv);
  const double length = ksl_vec3_l2norm(&along);
  if(!(length > 0.0)) {
    return false;
  }
  r[0] = Dot(Sub(p, c.origin), c.direction) - s;
  r[1] = Dot(outward, along) / length;
  if(du) {
    *du = pu;
    *dv = pv;
  }
  return true;
}

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
    ksl_vec3_t du, dv;
    shift::core::cad::Vec2 at_u = uv, at_v = uv;
    const double hu = 1e-6 * std::max(span[0], 1e-9);
    const double hv = 1e-6 * std::max(span[1], 1e-9);
    at_u[0] += uv[0] + hu > f.hi[0] ? -hu : hu;
    at_v[1] += uv[1] + hv > f.hi[1] ? -hv : hv;
    if(!SectionResidual(c, s, uv, r, &du, &dv) ||
       !SectionResidual(c, s, at_u, ru) || !SectionResidual(c, s, at_v, rv)) {
      return -HUGE_VAL;
    }
    /* the first row exactly, the second by differences */
    const double a11 = Dot(du, c.direction), a12 = Dot(dv, c.direction);
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
  if(depth == -HUGE_VAL ||
     Dot(SideNormal(*c.curved, c.face, uv), outward) > -0.5) {
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

/* the range [lo, hi] of a face along the line origin + s direction */
static void FaceRange(const shift::core::collision_geometry::CadSideOfPair& side, int face,
                      const ksl_vec3_t& origin, const ksl_vec3_t& direction,
                      double& lo, double& hi) {
  lo = HUGE_VAL;
  hi = -HUGE_VAL;
  for(const auto& p : side.shape->faces[face].boundary_points) {
    const double s = Dot(Sub(SidePosition(side, p), origin), direction);
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
  double s0, s1;
  FaceRange(curved, fc, c.origin, c.direction, s0, s1);
  bool other_ends[2] = {false, false}; //!< whose ends are s0 and s1
  if(other) {
    double o0, o1;
    FaceRange(*other, fo, c.origin, c.direction, o0, o1);
    other_ends[0] = o0 > s0;
    other_ends[1] = o1 < s1;
    s0 = std::max(s0, o0);
    s1 = std::min(s1, o1);
  }
  LineOverlap o;
  if(!(s1 > s0) || !CurvedOverlap(c, s0, s1, o)) {
    return true;
  }
  const size_t before = points.size();
  const double length = s1 - s0;
  const double inset = 1e-6 * length;
  const bool at_end = o.deepest <= s0 + inset || o.deepest >= s1 - inset;
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
    if(found && r.depth > 0.0 && r.depth <= max_depth) {
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
    ksl_vec3_t pa[2], pb[2], normal;
    shift::core::cad::Vec2 ends[2];
    for(int i = 0; i < 2; i++) {
      if(CurvedSection(&c, i ? o.hi : o.lo, pa[i], pb[i], normal) ==
         -HUGE_VAL) {
        return true;
      }
      ends[i] = c.uv;
    }
    auto in_range = [&](std::vector<std::array<double, 2>> pieces) {
      for(auto& piece : pieces) {
        piece = {o.lo + piece[0] * (o.hi - o.lo),
                 o.lo + piece[1] * (o.hi - o.lo)};
      }
      return pieces;
    };
    auto pieces = in_range(curved.shape->Clip(fc, ends[0], ends[1], kSeam));
    if(other) {
      shift::core::cad::Vec2 uv0, uv1;
      if(!SideProject(*other, fo, c.curved_is_a ? pb[0] : pa[0], uv0) ||
         !SideProject(*other, fo, c.curved_is_a ? pb[1] : pa[1], uv1, &uv0)) {
        return true;
      }
      pieces = Intersect(pieces,
                         in_range(other->shape->Clip(fo, uv0, uv1, kSeam)));
    }
    LinePoints(nullptr, -1, nullptr, -1, c.direction, o, CurvedSection, &c,
               max_depth, line, points, &pieces);
  }
  c.repeated = DropRepeated(points, before, 1e-9 * curved.shape->size);
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

static void VertexFace(const shift::core::collision_geometry::CadSideOfPair& feature, int vertex,
                       const shift::core::collision_geometry::CadSideOfPair& other, int face,
                       bool feature_is_a, double max_depth,
                       std::vector<shift::core::collision_geometry::ContactPoint>& points) {
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
  }
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
  /* a straight smooth edge has no corner, and no deepest point on a flat face:
  a seam of a cylinder sweeping past a plate lies flat on it, and would add a
  point at the end of the plate that comes and goes */
  if(e.smooth && curve.type == shift::core::cad::Curve3::Type::kLine &&
     SurfaceOf(other, face).type == shift::core::cad::Surface::Type::kPlane) {
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
  const double t = 0.5 * (lo + hi);
  const double extent = std::fabs(range[1] - range[0]);
  if(std::fabs(t - range[0]) < 0.01 * extent ||
     std::fabs(t - range[1]) < 0.01 * extent) {
    return; // at a vertex
  }
  /* a point between ends that also touch comes in as it gets deeper than them,
  so that a rim leaving a face flat has no jump */
  const auto& topology = m.edges[e.edge];
  double weight = 1.0;
  if(topology.start != topology.end) {
    const double ends = std::max(depth_at(range[0]), depth_at(range[1]));
    const double middle = depth_at(t);
    if(ends > 0.0) {
      weight = SmoothStep((middle - ends) / (0.1 * middle));
    }
    if(weight <= 0.0) {
      return;
    }
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

/* the points where two sharp edges cross; true if they touch */
static bool EdgeEdge(const shift::core::collision_geometry::CadSideOfPair& a, int ea, const shift::core::collision_geometry::CadSideOfPair& b,
                     int eb, double max_depth,
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
  const size_t before = points.size();
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
    if(depth > 0.0 && depth <= max_depth &&
       ksl_vec3_l2norm(&lateral) <= 1e-3 * depth &&
       InNormalCone(a, ea, pa, n) &&
       InNormalCone(b, eb, pb, Scaled(n, -1.0))) {
      shift::core::collision_geometry::ContactPoint c =
        FeaturePoint(false, pb, pa, n, depth,
                     0.5 * (1.0 / a.edge_radius + 1.0 / b.edge_radius));
      AddDistinct(points, c, a.shape->size);
    }
  }
  return points.size() > before;
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

/* adds the contact of two faces; true if they touch */
static bool FacePair(const shift::core::collision_geometry::CadSideOfPair& a, int fa, const shift::core::collision_geometry::CadSideOfPair& b,
                     int fb, double max_depth,
                     std::vector<shift::core::collision_geometry::ContactPoint>& points) {
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
  double x[4];
  if(!SeedPoint(a, fa, b, fb, max_depth, x)) {
    return false;
  }
  shift::core::collision_geometry::ContactPoint c;
  if(NewtonPoint(a, fa, b, fb, max_depth, x, c)) {
    points.push_back(c);
  }
  return touching();
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

/* what ShapePoints works with, kept so that a test allocates nothing; a thread
each, as pairs are tested in parallel */
struct ShapeScratch {
  std::vector<std::pair<int, int>> candidates;
  std::vector<int> found;
  IndexPairSet touching, edges_a, vertices_a, edges_b, vertices_b, edge_pairs;
  BoxCache near_a, near_b, near_edge;
};

/* the points of two shapes, in a's frame */
static void ShapePoints(const shift::core::collision_geometry::CadSideOfPair& a, const shift::core::collision_geometry::CadSideOfPair& b,
                        double max_depth, std::vector<shift::core::collision_geometry::ContactPoint>& points) {
  if(!a.shape->box.Overlaps(b.shape->box.Transformed(b.d))) {
    return;
  }

  static thread_local ShapeScratch scratch;
  auto& [candidates, found, touching, edges_a, vertices_a, edges_b, vertices_b,
         edge_pairs, near_a, near_b, near_edge] = scratch;

  /* the pairs of faces whose boxes overlap */
  candidates.clear();
  for(int fb = 0; fb < static_cast<int>(b.shape->faces.size()); fb++) {
    found.clear();
    a.shape->Query(b.shape->faces[fb].box.Transformed(b.d), found);
    for(int fa : found) {
      candidates.push_back({fa, fb});
    }
  }
  touching.Clear();
  std::vector<int> touching_a, touching_b; // few: the faces that touch
  for(const auto& [fa, fb] : candidates) {
    if(FacePair(a, fa, b, fb, max_depth, points)) {
      touching.Insert(fa, fb);
      touching_a.push_back(fa);
      touching_b.push_back(fb);
    }
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
        if(!face_touches && EdgeEdge(a, ea, b, eb, max_depth, points)) {
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
    for(int e : a.shape->faces[fa].edges) {
      if(a.shape->edges[e].box.Overlaps(bounds_b) && edges_a.Insert(e, fb) &&
         !touches(a.shape->edges[e].faces, fb, true) &&
         !touches_beside(a.shape->edges[e].faces, fb, true)) {
        EdgeFace(a, e, b, fb, true, max_depth,
                 !touches_any(a.shape->edges[e].faces, true) &&
                   !touches_any({fb}, false),
                 points);
      }
    }
    for(int v : a.shape->faces[fa].vertices) {
      if(inside(bounds_b, a.shape->vertices[v].point) &&
         vertices_a.Insert(v, fb) &&
         !touches(a.shape->vertices[v].faces, fb, true)) {
        VertexFace(a, v, b, fb, true, max_depth, points);
      }
    }
    for(int e : b.shape->faces[fb].edges) {
      if(b.shape->edges[e].box.Overlaps(bounds_a) && edges_b.Insert(e, fa) &&
         !touches(b.shape->edges[e].faces, fa, false) &&
         !touches_beside(b.shape->edges[e].faces, fa, false)) {
        EdgeFace(b, e, a, fa, false, max_depth,
                 !touches_any(b.shape->edges[e].faces, false) &&
                   !touches_any({fa}, true),
                 points);
      }
    }
    for(int v : b.shape->faces[fb].vertices) {
      if(inside(bounds_a, b.shape->vertices[v].point) &&
         vertices_b.Insert(v, fa) &&
         !touches(b.shape->vertices[v].faces, fa, false)) {
        VertexFace(b, v, a, fa, false, max_depth, points);
      }
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
  ksl_product_dinvd(&pair.a->get_d_absolute(), &pair.b->get_d_absolute(),
                    &b.d);
  std::vector<shift::core::collision_geometry::ContactPoint> points;
  ShapePoints(a, b, std::min(cad_a->get_max_depth(), cad_b->get_max_depth()),
              points);
  return Finish(pair, pair.a->get_d_absolute(), points);
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
  const auto& m = shape.model();
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
  for(int e : shape.faces[face].edges) {
    const auto& edge = shape.edges[e];
    if(!edge.sharp) {
      continue;
    }
    const auto& topology = m.edges[edge.edge];
    const auto& place = shape.placements[edge.placement];
    ksl_vec3_t last = place.Apply(m.vertices[topology.start].point);
    for(const ksl_vec3_t& sample : edge.sample_points) {
      to_segment(last, sample);
      last = sample;
    }
    to_segment(last, place.Apply(m.vertices[topology.end].point));
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
  if(!exact.faces.empty()) {
    ShapePoints(a, b, max_depth, points);
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
