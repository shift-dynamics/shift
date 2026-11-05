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
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/plane.h"

using shift::core::cad::Vec2;
using shift::core::collision_geometry::Cad;
using shift::core::collision_geometry::CadContactShape;
using shift::core::collision_geometry::ContactPair;
using shift::core::collision_geometry::ContactPoint;

namespace shift {
namespace core {
namespace collision_geometry {

/* a CAD geometry in the frame of the pair's geometry a: d transforms
coordinates in the geometry's frame to those in a's */
struct CadSideOfPair {
  const Cad* geometry = nullptr;
  const CadContactShape* shape = nullptr;
  ksl_SE3_t d = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

using shift::core::collision_geometry::CadSideOfPair;


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

static ksl_vec3_t SidePoint(const CadSideOfPair& side, int face,
                            const Vec2& uv, ksl_vec3_t* du = nullptr,
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

static ksl_vec3_t SideNormal(const CadSideOfPair& side, int face,
                             const Vec2& uv) {
  const ksl_vec3_t own = side.shape->Normal(face, uv);
  ksl_vec3_t n;
  ksl_product_drv(&side.d.R, &own, &n);
  return n;
}

static ksl_vec3_t SideVector(const CadSideOfPair& side, const ksl_vec3_t& v) {
  ksl_vec3_t out;
  ksl_product_drv(&side.d.R, &v, &out);
  return out;
}

static ksl_vec3_t SidePosition(const CadSideOfPair& side,
                               const ksl_vec3_t& p) {
  ksl_vec3_t out;
  ksl_product_dv(&side.d, &p, &out);
  return out;
}

static bool SideProject(const CadSideOfPair& side, int face,
                        const ksl_vec3_t& p, Vec2& uv,
                        const Vec2* guess = nullptr) {
  ksl_vec3_t own;
  ksl_product_dinvv(&side.d, &p, &own);
  return side.shape->Project(face, own, uv, guess);
}

static const shift::core::cad::Surface& SurfaceOf(const CadSideOfPair& side,
                                                  int face) {
  const auto& m = side.shape->model();
  return m.surfaces[m.faces[side.shape->faces[face].face].surface];
}

static bool SameSense(const CadSideOfPair& side, int face) {
  return side.shape->model().faces[side.shape->faces[face].face].same_sense;
}

/* the frame of a face's surface in a's frame: its origin and the unit z
axis (the axis of a cylinder, the normal of a plane), and the scale of its
lengths */
static void SurfaceAxis(const CadSideOfPair& side, int face,
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
face (projected onto its surface), in s */
static std::vector<std::array<double, 2>>
ClipLine(const CadSideOfPair& side, int face, const ksl_vec3_t& p,
         const ksl_vec3_t& direction, double s0, double s1) {
  Vec2 uv0, uv1;
  if(!SideProject(side, face, Axpy(p, s0, direction), uv0) ||
     !SideProject(side, face, Axpy(p, s1, direction), uv1, &uv0)) {
    return {};
  }
  auto pieces = side.shape->Clip(face, uv0, uv1);
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
piece's length, with weight */
static void LinePoints(const CadSideOfPair* a, int fa, const CadSideOfPair* b,
                       int fb, const ksl_vec3_t& direction,
                       const LineOverlap& o, Section section,
                       const void* data, double max_depth, double weight,
                       std::vector<ContactPoint>& points) {
  ksl_vec3_t pa, pb, n;
  section(data, o.deepest, pa, pb, n);
  std::vector<std::array<double, 2>> pieces = {{o.lo, o.hi}};
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
      ContactPoint c;
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
static void AxialRange(const CadSideOfPair& side, int face,
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

static void GapCurvature(const CadSideOfPair& a, int fa, const Vec2& uva,
                         const CadSideOfPair* b, int fb, const Vec2& uvb,
                         const ksl_vec3_t& n, ContactPoint& c);

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
static void CylinderCylinder(const CadSideOfPair& a, int fa,
                             const CadSideOfPair& b, int fb, double max_depth,
                             std::vector<ContactPoint>& points) {
  const bool convex_a = SameSense(a, fa), convex_b = SameSense(b, fb);
  if(!convex_a && !convex_b) {
    return;
  }
  const bool inner_is_a = convex_a;
  const CadSideOfPair& inner = inner_is_a ? a : b;
  const CadSideOfPair& other = inner_is_a ? b : a;
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
    ContactPoint p;
    p.depth = CylinderSection(&c, o.deepest, p.point_a, p.point_b, p.normal);
    Vec2 uva, uvb;
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
*/
static void PlaneCylinderLine(const PlaneCylinder& base,
                              const CadSideOfPair* plane, int fp,
                              const CadSideOfPair& cylinder, int fc,
                              double max_depth,
                              std::vector<ContactPoint>& points) {
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
  const double edge_radius = cylinder.geometry->get_edge_radius();
  const double width = std::sqrt(o.depth * edge_radius);
  double line = SmoothStep(((o.hi - o.lo) / width - 1.0) / 3.0);
  if(line < 1.0) {
    /* the rim at the deeper end */
    ContactPoint r;
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
    Vec2 uv, uvc;
    const bool inside =
      SideProject(cylinder, fc, on_rim, uvc) &&
      cylinder.shape->Inside(fc, uvc) &&
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
    const CadSideOfPair* a = c.plane_is_a ? plane : &cylinder;
    const CadSideOfPair* b = c.plane_is_a ? &cylinder : plane;
    const int fa = c.plane_is_a ? fp : fc, fb = c.plane_is_a ? fc : fp;
    LinePoints(a, fa, b, fb, c.dc, o, PlaneCylinderSection, &c, max_depth,
               line, points);
  }
}

/* a cylinder's face on a plane's face, of two CAD geometries */
static void PlaneCylinderFaces(const CadSideOfPair& plane, int fp,
                               const CadSideOfPair& cylinder, int fc,
                               bool plane_is_a, double max_depth,
                               std::vector<ContactPoint>& points) {
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
static void DistanceGradient(const CadSideOfPair& a, int fa,
                             const CadSideOfPair& b, int fb, const double* x,
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

/* the curvature of the gap between two faces touching at a point with the
normal n (a's frame): the sum of the faces' curvature tensors in the
tangent plane, whose eigenvalues k1 <= k2 are twice the gap's half
curvatures A and B. Sets c.curvature = A + B, the sum of the mean
curvatures, and c.curvature_ratio = B / A, limited to 1e3 (a gap curved in
one direction only, or a saddle, is taken as that elongated). b is nullptr
for a plane */
static void GapCurvature(const CadSideOfPair& a, int fa, const Vec2& uva,
                         const CadSideOfPair* b, int fb, const Vec2& uvb,
                         const ksl_vec3_t& n, ContactPoint& c) {
  ksl_vec3_t t1, t2;
  Perpendiculars(n, t1, t2);
  double h[3] = {0.0, 0.0, 0.0};
  auto add = [&](const CadSideOfPair& side, int face, const Vec2& uv) {
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
  const double mean = 0.5 * (h[0] + h[2]);
  const double spread =
    std::sqrt(0.25 * (h[0] - h[2]) * (h[0] - h[2]) + h[1] * h[1]);
  const double k2 = mean + spread;
  const double k1 = std::max(mean - spread, 1e-3 * k2);
  c.curvature = mean;
  c.curvature_ratio = k2 > 0.0 ? k2 / k1 : 1.0;
}

/* the deepest point of two faces in contact: Newton's method on the
gradient of their distance, from the pair of sample points deepest in each
other with opposed normals */
static bool NewtonPoint(const CadSideOfPair& a, int fa, const CadSideOfPair& b,
                        int fb, double max_depth, ContactPoint& c) {
  const auto& face_a = a.shape->faces[fa];
  const auto& face_b = b.shape->faces[fb];
  double best = -HUGE_VAL;
  double x[4];
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
  if(best == -HUGE_VAL ||
     best < -max_depth - std::max(spacing(face_a), spacing(face_b))) {
    return false;
  }
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
  const Vec2 uva{x[0], x[1]}, uvb{x[2], x[3]};
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
/* edges and vertices */

/* a point p of one side (feature) inside a face of the other: its depth
below the face, the face's point and outward normal; false if p is not
inside the face within max_depth */
static bool PointInFace(const CadSideOfPair& side, int face,
                        const ksl_vec3_t& p, double max_depth, double& depth,
                        ksl_vec3_t& q, ksl_vec3_t& n, Vec2& uv) {
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
  depth = -Dot(Sub(p, q), n);
  return depth > 0.0 && depth <= max_depth;
}

/* whether a face of a feature (edge or vertex) at its point p faces against
the normal n of the face it is inside: a feature pushing into a face does;
one merely behind a face of a hollow body (outside a bore, say) does not */
static bool Opposes(const CadSideOfPair& side, const std::vector<int>& faces,
                    const ksl_vec3_t& p, const ksl_vec3_t& n) {
  for(int f : faces) {
    Vec2 uv;
    if(SideProject(side, f, p, uv) && Dot(SideNormal(side, f, uv), n) < -0.05) {
      return true;
    }
  }
  return false;
}

/* the contact point of a feature point p of side feature against a face of
the other side, with the right sense: from a to b */
static ContactPoint FeaturePoint(bool feature_is_a, const ksl_vec3_t& p,
                                 const ksl_vec3_t& q, const ksl_vec3_t& n,
                                 double depth, double curvature) {
  ContactPoint c;
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

static void VertexFace(const CadSideOfPair& feature, int vertex,
                       const CadSideOfPair& other, int face,
                       bool feature_is_a, double max_depth,
                       std::vector<ContactPoint>& points) {
  const ksl_vec3_t p =
    SidePosition(feature, feature.shape->vertices[vertex].point);
  double depth;
  ksl_vec3_t q, n;
  Vec2 uv;
  if(PointInFace(other, face, p, max_depth, depth, q, n, uv) &&
     Opposes(feature, feature.shape->vertices[vertex].faces, p, n)) {
    points.push_back(FeaturePoint(
      feature_is_a, p, q, n, depth,
      1.0 / feature.geometry->get_edge_radius() +
        other.shape->MeanCurvature(face, uv)));
  }
}

/* the deepest point of an edge inside a face: the deepest of its samples,
refined by golden section search between its neighbors; ends of the edge
are left to its vertices */
static void EdgeFace(const CadSideOfPair& feature, int edge,
                     const CadSideOfPair& other, int face, bool feature_is_a,
                     double max_depth, std::vector<ContactPoint>& points) {
  const auto& e = feature.shape->edges[edge];
  if(e.sample_t.empty()) {
    return;
  }
  const auto& m = feature.shape->model();
  const auto& curve = m.curves[m.edges[e.edge].curve];
  const auto& place = feature.shape->placements[e.placement];
  auto point_at = [&](double t) {
    return SidePosition(feature,
                        place.Apply(shift::core::cad::CurvePoint<3>(curve, t)));
  };
  auto depth_at = [&](double t) {
    double depth;
    ksl_vec3_t q, n;
    Vec2 uv;
    return PointInFace(other, face, point_at(t), max_depth, depth, q, n, uv)
             ? depth
             : -HUGE_VAL;
  };
  const size_t count = e.sample_t.size();
  int best = -1;
  double deepest = 0.0;
  for(size_t i = 0; i < count; i++) {
    const double depth = depth_at(e.sample_t[i]);
    if(depth > deepest) {
      deepest = depth;
      best = static_cast<int>(i);
    }
  }
  if(best < 0) {
    return;
  }
  {
    /* only an edge pushing into the face is refined */
    double depth;
    ksl_vec3_t q, n;
    Vec2 uv;
    const ksl_vec3_t p = point_at(e.sample_t[best]);
    if(!PointInFace(other, face, p, max_depth, depth, q, n, uv) ||
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
  const ksl_vec3_t p = point_at(t);
  double depth;
  ksl_vec3_t q, n;
  Vec2 uv;
  if(PointInFace(other, face, p, max_depth, depth, q, n, uv) &&
     Opposes(feature, e.faces, p, n)) {
    points.push_back(FeaturePoint(
      feature_is_a, p, q, n, depth,
      1.0 / feature.geometry->get_edge_radius() +
        other.shape->MeanCurvature(face, uv)));
  }
}

/* ------------------------------------------------------------------------ */
/* pairs of faces */

static void FacePair(const CadSideOfPair& a, int fa, const CadSideOfPair& b,
                     int fb, double max_depth,
                     std::vector<ContactPoint>& points) {
  using Type = shift::core::cad::Surface::Type;
  const Type ta = SurfaceOf(a, fa).type;
  const Type tb = SurfaceOf(b, fb).type;
  if(ta == Type::kCylinder && tb == Type::kCylinder) {
    CylinderCylinder(a, fa, b, fb, max_depth, points);
    return;
  } else if(ta == Type::kPlane && tb == Type::kCylinder) {
    PlaneCylinderFaces(a, fa, b, fb, true, max_depth, points);
    return;
  } else if(ta == Type::kCylinder && tb == Type::kPlane) {
    PlaneCylinderFaces(b, fb, a, fa, false, max_depth, points);
    return;
  } else if(ta == Type::kPlane && tb == Type::kPlane) {
    return; // their edges and vertices touch
  }
  ContactPoint c;
  if(NewtonPoint(a, fa, b, fb, max_depth, c)) {
    points.push_back(c);
  }
}

/* ------------------------------------------------------------------------ */
/* the contact data of a pair */

/* the contact data of a pair from its points in the frame of a */
static bool Finish(ContactPair& pair, const ksl_SE3_t& frame,
                   std::vector<ContactPoint>& points) {
  pair.success = true;
  pair.collision = !points.empty();
  pair.point_laws = true;
  pair.depth_converged = true;
  if(!pair.collision) {
    return false;
  }
  size_t deepest = 0;
  for(size_t i = 0; i < points.size(); i++) {
    ContactPoint& c = points[i];
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
  const ContactPoint& d = points[deepest];
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

bool shift::core::collision_geometry::CadCadContact(ContactPair& pair) {
  CadSideOfPair a, b;
  a.geometry = static_cast<const Cad*>(pair.a);
  b.geometry = static_cast<const Cad*>(pair.b);
  a.shape = &a.geometry->get_shape();
  b.shape = &b.geometry->get_shape();
  ksl_product_dinvd(&pair.a->get_d_absolute(), &pair.b->get_d_absolute(),
                    &b.d);
  const double max_depth =
    std::min(a.geometry->get_max_depth(), b.geometry->get_max_depth());
  std::vector<ContactPoint> points;
  if(!a.shape->box.Overlaps(b.shape->box.Transformed(b.d))) {
    return Finish(pair, pair.a->get_d_absolute(), points);
  }

  /* the pairs of faces whose boxes overlap */
  std::vector<std::pair<int, int>> candidates;
  std::vector<int> found;
  for(int fb = 0; fb < static_cast<int>(b.shape->faces.size()); fb++) {
    found.clear();
    a.shape->Query(b.shape->faces[fb].box.Transformed(b.d), found);
    for(int fa : found) {
      candidates.push_back({fa, fb});
    }
  }
  std::set<std::pair<int, int>> touching;
  for(const auto& [fa, fb] : candidates) {
    const size_t before = points.size();
    FacePair(a, fa, b, fb, max_depth, points);
    if(points.size() > before) {
      touching.insert({fa, fb});
    }
  }

  /* edges and vertices of faces that touch no face of the other side, where
  no other face of theirs does */
  auto touches = [&](const std::vector<int>& faces, int other, bool on_a) {
    for(int f : faces) {
      if(touching.count(on_a ? std::make_pair(f, other)
                             : std::make_pair(other, f))) {
        return true;
      }
    }
    return false;
  };
  /* the transformation of a's frame to b's, for the bounds of a's faces */
  ksl_SE3_t a_in_b;
  ksl_SE3_inverted(&b.d, &a_in_b);
  std::set<std::pair<int, int>> edges_a, vertices_a, edges_b, vertices_b;
  for(const auto& [fa, fb] : candidates) {
    if(touching.count({fa, fb})) {
      continue;
    }
    /* each face's bounds in the other's frame, enlarged by the deepest
    penetration: the features of the other near it */
    CadBox near_b = b.shape->faces[fb].box.Transformed(b.d);
    near_b.Pad(max_depth);
    CadBox near_a = a.shape->faces[fa].box.Transformed(a_in_b);
    near_a.Pad(max_depth);
    auto inside = [](const CadBox& box, const ksl_vec3_t& p) {
      for(int k = 0; k < 3; k++) {
        if(p.at[k] < box.lo.at[k] || p.at[k] > box.hi.at[k]) {
          return false;
        }
      }
      return true;
    };
    for(int e : a.shape->faces[fa].edges) {
      if(a.shape->edges[e].box.Overlaps(near_b) &&
         edges_a.insert({e, fb}).second &&
         !touches(a.shape->edges[e].faces, fb, true)) {
        EdgeFace(a, e, b, fb, true, max_depth, points);
      }
    }
    for(int v : a.shape->faces[fa].vertices) {
      if(inside(near_b, a.shape->vertices[v].point) &&
         vertices_a.insert({v, fb}).second &&
         !touches(a.shape->vertices[v].faces, fb, true)) {
        VertexFace(a, v, b, fb, true, max_depth, points);
      }
    }
    for(int e : b.shape->faces[fb].edges) {
      if(b.shape->edges[e].box.Overlaps(near_a) &&
         edges_b.insert({e, fa}).second &&
         !touches(b.shape->edges[e].faces, fa, false)) {
        EdgeFace(b, e, a, fa, false, max_depth, points);
      }
    }
    for(int v : b.shape->faces[fb].vertices) {
      if(inside(near_a, b.shape->vertices[v].point) &&
         vertices_b.insert({v, fa}).second &&
         !touches(b.shape->vertices[v].faces, fa, false)) {
        VertexFace(b, v, a, fa, false, max_depth, points);
      }
    }
  }
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

bool shift::core::collision_geometry::CadPlaneContact(ContactPair& pair) {
  const bool plane_is_a = dynamic_cast<const Plane*>(pair.a) != nullptr;
  const Plane& plane = *static_cast<const Plane*>(plane_is_a ? pair.a : pair.b);
  CadSideOfPair side;
  side.geometry = static_cast<const Cad*>(plane_is_a ? pair.b : pair.a);
  side.shape = &side.geometry->get_shape();
  const CadContactShape& shape = *side.shape;
  const ksl_SE3_t& frame = side.geometry->get_d_absolute();
  const double max_depth = side.geometry->get_max_depth();
  const double edge_curvature = 1.0 / side.geometry->get_edge_radius();

  /* the plane in the frame of the geometry */
  const ksl_vec3_t world_normal = plane.get_normal();
  ksl_vec3_t n, q;
  ksl_product_drinvv(&frame.R, &world_normal, &n);
  ksl_product_dinvv(&frame, &plane.get_point(), &q);

  std::vector<ContactPoint> points;
  auto add = [&](const ksl_vec3_t& p, double depth, double curvature,
                 double length, double ratio = 1.0) {
    if(!(depth > 0.0) || depth > max_depth) {
      return;
    }
    ContactPoint c;
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
  using Type = shift::core::cad::Surface::Type;
  CadSideOfPair own; // the geometry in its own frame
  own.geometry = side.geometry;
  own.shape = side.shape;
  const auto& m = shape.model();
  std::vector<bool> touching(shape.faces.size(), false);
  auto face_points = [&](int fi) {
    const auto& f = shape.faces[fi];
    const Type type = SurfaceOf(own, fi).type;
    if(!SameSense(own, fi) && type != Type::kNurbs) {
      return; // a concave analytic face: lowest at its edges
    }
    if(Lowest(f.box, depth_of) > 0.0) {
      return; // above the plane
    }
    if(type == Type::kPlane) {
      return; // its edges and vertices touch
    }
    if(type == Type::kCylinder) {
      /* in the geometry's own frame (own), the plane's side b or a */
      PlaneCylinder c;
      c.op = q;
      c.np = n;
      c.plane_is_a = plane_is_a;
      PlaneCylinderLine(c, nullptr, -1, own, fi, max_depth, points);
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
    Vec2 uv = f.sample_uv[best];
    const double span[2] = {f.hi[0] - f.lo[0], f.hi[1] - f.lo[1]};
    auto gradient = [&](const Vec2& x, double* g) {
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
        Vec2 xp = uv, xm = uv;
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
      ContactPoint gap;
      GapCurvature(own, fi, uv, nullptr, -1, uv, n, gap);
      add(p, depth_of(p), gap.curvature, 0.0, gap.curvature_ratio);
    }
  };
  for(int fi = 0; fi < static_cast<int>(shape.faces.size()); fi++) {
    const size_t before = points.size();
    face_points(fi);
    touching[fi] = points.size() > before;
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
    using Kind = shift::core::cad::Curve3::Type;
    if(curve.type == Kind::kLine) {
      continue; // lowest at a vertex
    }
    double t;
    if(curve.type == Kind::kConic) {
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
