/*!
@file
@brief GJK contact detection between two convex collision geometries

Port of the GJK implementation in the original C version of Shift
(src/core/simplex.c). The simplex functions are unchanged; the support mapping
now goes through collision_geometry::Base::Support, and the simplex history is
kept per pair of geometries in a ContactPair.
*/
#include "shift/core/collision_geometry/gjk.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>

#include "ksl/matrix.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/epa.h"
#include "shift/core/collision_geometry/manifold.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/analytic_contact.h"
#include "shift/core/collision_geometry/sphere.h"


static constexpr double kEps = 1.0e-10;
static constexpr int kMaxIterations = 50;

static inline ksl_vec3_t Sub(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x - b.x, a.y - b.y, a.z - b.z}};
}

static inline ksl_vec3_t Neg(const ksl_vec3_t& a) {
  return {{-a.x, -a.y, -a.z}};
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

/**
simplex0: a single simplex support point. simplexVectorsAbsolute[0] contains
the vector OA, which locates the simplex vertex point A relative to the origin
point O. The next search direction is set to -OA. If the point is within EPS of
the origin, the objects are in contact.
*/
static void Simplex0(ksl_vec3_t& d, ksl_vec3_t s[13], bool& collision,
              bool& success) {
  d = Neg(s[0]);
  if(Norm(d) < kEps) {
    collision = true;
    success = true;
  }
}

/**
simplex1() takes in two points from the Minkowski difference hull, an old
point B and a new point A. The two points form a line segment AB. If the
origin lies on this line, it is in the convex hull, and the objects intersect.
Otherwise a triangle is needed, and the search direction is set orthogonal to
the line, in the plane containing the origin, toward the origin:
(OA X AB) X AB.

s[0] = OA, s[1] = OB, s[2] = AB, s[3] = OA X AB
*/
static void Simplex1(ksl_vec3_t& d, ksl_vec3_t s[13], bool& collision,
              bool& success) {
  if(Dot(s[0], d) < -kEps) {
    success = true;
    collision = false;
    return;
  }
  s[2] = Sub(s[1], s[0]); // AB = OB - OA
  s[3] = Cross(s[0], s[2]); // OA X AB
  d = Cross(s[3], s[2]);    // (OA X AB) X AB
  if(Norm(d) < kEps) {
    success = true;
    collision = true;
  }
}

/**
simplex2() takes in three points from the Minkowski difference hull, two old
points B and C and a new point A, forming a triangle TABC with normal
ABCn = AB X AC. ACn = ABCn X AC and ABn = AB X ABCn are outward facing edge
normals in the plane of the triangle.

If the origin is outside edge AB or AC, the simplex is reduced to that edge.
Otherwise the origin lies above, below or on the triangle: on the triangle the
objects intersect, above it the search direction is ABCn, and below it the
search direction is -ABCn and the triangle is flipped so that the same tests
apply on the next call.

The swaps of simplex vectors are mirrored in the local extreme vertices, from
which the simplex vectors are recovered.

s[0] = OA, s[1] = OB, s[2] = OC, s[3] = AB, s[4] = AC, s[5] = ABCn,
s[6] = ACn, s[7] = ABn, s[8] = OA X AB or OA X AC
*/
static void Simplex2(ksl_vec3_t& d, ksl_vec3_t s[13], ksl_vec3_t la[4],
              ksl_vec3_t lb[4], int& count, bool& collision, bool& success) {
  if(Dot(s[0], d) < -kEps) {
    success = true;
    collision = false;
    return;
  }
  s[3] = Sub(s[1], s[0]);   // AB = OB - OA
  s[4] = Sub(s[2], s[0]);   // AC = OC - OA
  s[5] = Cross(s[3], s[4]); // ABCn = AB X AC
  s[6] = Cross(s[5], s[4]); // ACn = ABCn X AC
  s[7] = Cross(s[3], s[5]); // ABn = AB X ABCn

  if(Dot(s[0], s[7]) < -kEps) { // OA•ABn: outside edge AB
    s[8] = Cross(s[0], s[3]);   // OA X AB
    d = Cross(s[8], s[3]);      // (OA X AB) X AB
    count--;                    // simplex1: [OA, OB]
  } else if(Dot(s[0], s[6]) < -kEps) { // OA•ACn: outside edge AC
    s[8] = Cross(s[0], s[4]);          // OA X AC
    d = Cross(s[8], s[4]);             // (OA X AC) X AC
    std::swap(s[1], s[2]);             // swap OB and OC
    std::swap(la[1], la[2]);
    std::swap(lb[1], lb[2]);
    count--; // simplex1: [OA, OC]
  } else {
    const double v = -Dot(s[0], s[5]); // -OA•ABCn
    if(v > kEps) {                     // origin above TABC
      d = s[5];
    } else if(v < -kEps) { // origin below TABC, flip the triangle
      d = Neg(s[5]);
      std::swap(s[0], s[1]);
      std::swap(la[0], la[1]);
      std::swap(lb[0], lb[1]);
    } else {
      success = true;
      collision = true;
    }
  }
}

/**
simplex3() takes in four points from the Minkowski difference hull, three old
points B, C and D and a new point A, forming a tetrahedron TABCD with base
triangle TBCD below A. A previous simplex2() verified that the origin does not
lie below TBCD, so the faces TABC, TACD and TADB, with outward normals
ABCn = AB X AC, ACDn = AC X AD and ADBn = AD X AB, are checked. If the origin
is outside a face, the simplex is reduced to that face or to one of its edges
toward A. If it is inside all faces, the objects intersect.

s[0..3] = OA, OB, OC, OD, s[4..6] = AB, AC, AD, s[7..9] = ABCn, ACDn, ADBn,
s[10], s[11] = edge normals of the face being tested, s[12] = OA X edge
*/
static void Simplex3(ksl_vec3_t& d, ksl_vec3_t s[13], ksl_vec3_t la[4],
              ksl_vec3_t lb[4], int& count, bool& collision, bool& success) {
  if(Dot(s[0], d) < -kEps) {
    success = true;
    collision = false;
    return;
  }
  s[4] = Sub(s[1], s[0]);   // AB = OB - OA
  s[5] = Sub(s[2], s[0]);   // AC = OC - OA
  s[6] = Sub(s[3], s[0]);   // AD = OD - OA
  s[7] = Cross(s[4], s[5]); // ABCn = AB X AC
  s[8] = Cross(s[5], s[6]); // ACDn = AC X AD
  s[9] = Cross(s[6], s[4]); // ADBn = AD X AB

  auto swap = [&](int i, int j) {
    std::swap(s[i], s[j]);
    std::swap(la[i], la[j]);
    std::swap(lb[i], lb[j]);
  };

  if(Dot(s[0], s[7]) < -kEps) { // O outside face TABC
    s[10] = Cross(s[7], s[5]);  // ACn = ABCn X AC
    s[11] = Cross(s[4], s[7]);  // ABn = AB X ABCn
    if(Dot(s[0], s[10]) < -kEps) { // O outside edge AC
      s[12] = Cross(s[0], s[5]);
      d = Cross(s[12], s[5]); // (OA X AC) X AC
      swap(1, 2);             // simplex1: [OA, OC]
      count -= 2;
    } else if(Dot(s[0], s[11]) < -kEps) { // O outside edge AB
      s[12] = Cross(s[0], s[4]);
      d = Cross(s[12], s[4]); // (OA X AB) X AB
      count -= 2;             // simplex1: [OA, OB]
    } else {
      d = s[7]; // simplex2: [OA, OB, OC]
      count--;
    }
  } else if(Dot(s[0], s[8]) < -kEps) { // O outside face TACD
    s[10] = Cross(s[8], s[6]);         // ADn = ACDn X AD
    s[11] = Cross(s[5], s[8]);         // ACn = AC X ACDn
    if(Dot(s[0], s[10]) < -kEps) {     // O outside edge AD
      s[12] = Cross(s[0], s[6]);
      d = Cross(s[12], s[6]); // (OA X AD) X AD
      swap(1, 3);             // simplex1: [OA, OD]
      count -= 2;
    } else if(Dot(s[0], s[11]) < -kEps) { // O outside edge AC
      s[12] = Cross(s[0], s[5]);
      d = Cross(s[12], s[5]); // (OA X AC) X AC
      swap(1, 2);             // simplex1: [OA, OC]
      count -= 2;
    } else {
      d = s[8];
      swap(1, 2);
      swap(2, 3); // simplex2: [OA, OC, OD]
      count--;
    }
  } else if(Dot(s[0], s[9]) < -kEps) { // O outside face TADB
    s[10] = Cross(s[9], s[4]);         // ABn = ADBn X AB
    s[11] = Cross(s[6], s[9]);         // ADn = AD X ADBn
    if(Dot(s[0], s[10]) < -kEps) {     // O outside edge AB
      s[12] = Cross(s[0], s[4]);
      d = Cross(s[12], s[4]); // (OA X AB) X AB
      count -= 2;             // simplex1: [OA, OB]
    } else if(Dot(s[0], s[11]) < -kEps) { // O outside edge AD
      s[12] = Cross(s[0], s[6]);
      d = Cross(s[12], s[6]); // (OA X AD) X AD
      swap(1, 3);             // simplex1: [OA, OD]
      count -= 2;
    } else {
      d = s[9];
      swap(1, 3);
      swap(2, 3); // simplex2: [OA, OD, OB]
      count--;
    }
  } else {
    success = true;
    collision = true;
  }
}

/**
GJK on the Minkowski difference b - a. Each iteration sends the search
direction to the support functions of a (negated) and b to obtain the next
support point, which is inserted at the front of the simplex. The simplex
functions then either decide contact (success = true) or reduce the simplex
and produce a new search direction.

The original implementation restored the whole simplex from the previous test
of the pair. Once the bodies have moved, the ordering that the simplex
functions rely on (e.g. that the origin is not below the base of a
tetrahedron) no longer holds, so only the newest point is restored, recomputed
from its local coordinates at the current poses.
*/
static bool Gjk(shift::core::collision_geometry::ContactPair& p) {
  auto* A = p.a;
  auto* B = p.b;
  ksl_vec3_t* la = p.extreme_vertex_local_a;
  ksl_vec3_t* lb = p.extreme_vertex_local_b;
  ksl_vec3_t global_a;
  ksl_vec3_t global_b;
  ksl_vec3_t s[13];
  ksl_vec3_t& d = p.search_direction;

  p.success = false;
  p.collision = false;

  if(p.simplex_point_count == 0) {
    /* best estimate of the initial search direction */
    d = Sub(A->get_d_absolute().t, B->get_d_absolute().t);
    if(Norm(d) < kEps) {
      d = {{1.0, 0.0, 0.0}};
    }
    A->Support(Neg(d), la[0], global_a, p.hint_a);
    B->Support(d, lb[0], global_b, p.hint_b);
  } else {
    ksl_product_dv(&A->get_d_absolute(), &la[0], &global_a);
    ksl_product_dv(&B->get_d_absolute(), &lb[0], &global_b);
  }
  p.simplex_point_count = 1;
  s[0] = Sub(global_b, global_a);

  for(int iteration = 0; iteration < kMaxIterations; iteration++) {
    switch(p.simplex_point_count) {
      case 1:
        Simplex0(d, s, p.collision, p.success);
        break;
      case 2:
        Simplex1(d, s, p.collision, p.success);
        break;
      case 3:
        Simplex2(d, s, la, lb, p.simplex_point_count, p.collision, p.success);
        break;
      case 4:
        Simplex3(d, s, la, lb, p.simplex_point_count, p.collision, p.success);
        break;
    }
    if(p.success) {
      return p.collision;
    }

    /* shift the simplex down one to make room for the next support point */
    for(int i = p.simplex_point_count; i > 0; i--) {
      la[i] = la[i - 1];
      lb[i] = lb[i - 1];
      s[i] = s[i - 1];
    }
    A->Support(Neg(d), la[0], global_a, p.hint_a);
    B->Support(d, lb[0], global_b, p.hint_b);
    s[0] = Sub(global_b, global_a);
    p.simplex_point_count++;
  }

  /* no decision within the iteration limit, start over on the next test */
  p.simplex_point_count = 0;
  p.collision = false;
  return false;
}


/*! contact data of a pair from a contact computed with the pair's geometry
 * first_is_a ? a : b as the first shape */
static void Assign(shift::core::collision_geometry::ContactPair& pair,
                   const shift::core::collision_geometry::AnalyticContact& c,
                   bool first_is_a) {
  pair.success = true;
  pair.collision = c.collision;
  if(!c.collision) {
    return;
  }
  pair.depth = c.depth;
  pair.depth_converged = true;
  pair.normal = first_is_a ? c.normal : Neg(c.normal);
  pair.point_a = first_is_a ? c.point_first : c.point_second;
  pair.point_b = first_is_a ? c.point_second : c.point_first;
  /* coordinates in each geometry frame, R^T (p - t) */
  for(auto [g, p, local] :
      {std::tuple{pair.a, &pair.point_a, &pair.point_local_a},
       std::tuple{pair.b, &pair.point_b, &pair.point_local_b}}) {
    const ksl_SE3_t& d = g->get_d_absolute();
    const ksl_vec3_t r = Sub(*p, d.t);
    for(int i = 0; i < 3; i++) {
      local->at[i] = Dot(d.R.v[i], r);
    }
  }
}

/* the contact of a pair and its deepest point */
static bool TestDeepestPoint(shift::core::collision_geometry::ContactPair& pair) {
  /* pairs with a closed form or one dimensional contact: a plane with
  anything, a sphere with a sphere, a convex hull, an ellipsoid or a
  cylinder, and two ellipsoids */
  shift::core::collision_geometry::AnalyticContact contact;
  if(auto plane = dynamic_cast<const shift::core::collision_geometry::Plane*>(pair.a)) {
    shift::core::collision_geometry::PlaneContact(*plane, *pair.b, contact);
    Assign(pair, contact, true);
    return pair.collision;
  }
  if(auto plane = dynamic_cast<const shift::core::collision_geometry::Plane*>(pair.b)) {
    shift::core::collision_geometry::PlaneContact(*plane, *pair.a, contact);
    Assign(pair, contact, false);
    return pair.collision;
  }
  auto sphere_a = dynamic_cast<const shift::core::collision_geometry::Sphere*>(pair.a);
  auto sphere_b = dynamic_cast<const shift::core::collision_geometry::Sphere*>(pair.b);
  if(sphere_a && sphere_b) {
    const ksl_vec3_t& ca = sphere_a->get_center();
    const ksl_vec3_t& cb = sphere_b->get_center();
    const ksl_vec3_t t = Sub(cb, ca);
    const double ra = sphere_a->get_radius();
    const double rb = sphere_b->get_radius();
    const double distance = Norm(t);
    contact.collision = distance < ra + rb;
    if(contact.collision) {
      contact.depth = ra + rb - distance;
      contact.normal = distance > 0.0
                         ? ksl_vec3_t{{t.x / distance, t.y / distance,
                                       t.z / distance}}
                         : ksl_vec3_t{{1.0, 0.0, 0.0}};
      const ksl_vec3_t& n = contact.normal;
      contact.point_first = {{ca.x + ra * n.x, ca.y + ra * n.y,
                              ca.z + ra * n.z}};
      contact.point_second = {{cb.x - rb * n.x, cb.y - rb * n.y,
                               cb.z - rb * n.z}};
    }
    Assign(pair, contact, true);
    return pair.collision;
  }
  if(sphere_a || sphere_b) {
    const shift::core::collision_geometry::Sphere& sphere = sphere_a ? *sphere_a : *sphere_b;
    const shift::core::collision_geometry::Base* other = sphere_a ? pair.b : pair.a;
    bool handled = false;
    if(auto hull = dynamic_cast<const shift::core::collision_geometry::ConvexHull*>(other)) {
      handled = shift::core::collision_geometry::ConvexHullSphereContact(*hull, sphere, contact);
    } else if(auto ellipsoid = dynamic_cast<const shift::core::collision_geometry::Ellipsoid*>(other)) {
      handled = shift::core::collision_geometry::EllipsoidSphereContact(*ellipsoid, sphere, contact);
    } else if(auto cylinder = dynamic_cast<const shift::core::collision_geometry::Cylinder*>(other)) {
      handled = shift::core::collision_geometry::CylinderSphereContact(*cylinder, sphere, contact);
    }
    if(handled) {
      Assign(pair, contact, sphere_b != nullptr); // the shape is first
      return pair.collision;
    }
  }
  auto ellipsoid_a = dynamic_cast<const shift::core::collision_geometry::Ellipsoid*>(pair.a);
  auto ellipsoid_b = dynamic_cast<const shift::core::collision_geometry::Ellipsoid*>(pair.b);
  if(ellipsoid_a && ellipsoid_b &&
     shift::core::collision_geometry::EllipsoidEllipsoidContact(*ellipsoid_a, *ellipsoid_b, contact)) {
    Assign(pair, contact, true);
    return pair.collision;
  }
  /* a convex hull and an ellipsoid: GJK, EPA for a first normal, then the
  exact contact of the hull feature near it */
  auto hull_a = dynamic_cast<const shift::core::collision_geometry::ConvexHull*>(pair.a);
  auto hull_b = dynamic_cast<const shift::core::collision_geometry::ConvexHull*>(pair.b);
  if((hull_a && ellipsoid_b) || (hull_b && ellipsoid_a)) {
    const bool hull_first = hull_a != nullptr;
    const shift::core::collision_geometry::ConvexHull& hull = hull_first ? *hull_a : *hull_b;
    const shift::core::collision_geometry::Ellipsoid& ellipsoid = hull_first ? *ellipsoid_b : *ellipsoid_a;
    if(!Gjk(pair)) {
      return pair.collision;
    }
    /* the minimum of the last accepted step, while it lasts, so the normal
    does not jump between minima of nearly equal depth */
    const ksl_vec3_t& kept = pair.accepted_normal;
    if(Dot(kept, kept) > 0.0 &&
       shift::core::collision_geometry::ConvexHullEllipsoidContact(hull, ellipsoid,
                                  hull_first ? kept : Neg(kept), true,
                                  contact)) {
      Assign(pair, contact, hull_first);
      return pair.collision;
    }
    ComputePenetration(pair);
    if(pair.collision &&
       shift::core::collision_geometry::ConvexHullEllipsoidContact(hull, ellipsoid,
                                  hull_first ? pair.normal : Neg(pair.normal),
                                  false, contact)) {
      Assign(pair, contact, hull_first);
    }
    return pair.collision;
  }
  /* a convex hull and a cylinder: GJK, then the minimum of the last
  accepted step while it lasts, and for a new contact the exact minimum
  nearest EPA's normal */
  auto cylinder_a = dynamic_cast<const shift::core::collision_geometry::Cylinder*>(pair.a);
  auto cylinder_b = dynamic_cast<const shift::core::collision_geometry::Cylinder*>(pair.b);
  if((hull_a && cylinder_b) || (hull_b && cylinder_a)) {
    const bool hull_first = hull_a != nullptr;
    const shift::core::collision_geometry::ConvexHull& hull = hull_first ? *hull_a : *hull_b;
    const shift::core::collision_geometry::Cylinder& cylinder = hull_first ? *cylinder_b : *cylinder_a;
    int& hint = hull_first ? pair.hint_a : pair.hint_b;
    if(!Gjk(pair)) {
      return pair.collision;
    }
    const ksl_vec3_t& kept = pair.accepted_normal;
    if(Dot(kept, kept) > 0.0 &&
       shift::core::collision_geometry::CylinderHullContact(hull, cylinder,
                                  hull_first ? kept : Neg(kept), true, hint,
                                  contact)) {
      Assign(pair, contact, hull_first);
      return pair.collision;
    }
    ComputePenetration(pair);
    if(pair.collision &&
       shift::core::collision_geometry::CylinderHullContact(hull, cylinder,
                                  hull_first ? pair.normal : Neg(pair.normal),
                                  false, hint, contact)) {
      Assign(pair, contact, hull_first);
    }
    return pair.collision;
  }
  /* two convex hulls: GJK, then the minimum of the last accepted step
  while it lasts, and EPA for a new contact */
  if(hull_a && hull_b) {
    if(!Gjk(pair)) {
      return pair.collision;
    }
    const ksl_vec3_t& kept = pair.accepted_normal;
    if(Dot(kept, kept) > 0.0 &&
       shift::core::collision_geometry::ConvexHullPairContact(*hull_a, *hull_b, kept, pair.hint_a,
                             pair.hint_b, contact)) {
      Assign(pair, contact, true);
      return pair.collision;
    }
    ComputePenetration(pair);
    return pair.collision;
  }
  /* anything else: GJK, and EPA for the contact data */
  if(Gjk(pair)) {
    ComputePenetration(pair);
  }
  return pair.collision;
}

/* a sphere, or an ellipsoid with a volume, whose contact with CAD geometry is
exact */
static bool IsRound(const shift::core::collision_geometry::Base* g) {
  if(dynamic_cast<const shift::core::collision_geometry::Sphere*>(g)) {
    return true;
  }
  auto e = dynamic_cast<const shift::core::collision_geometry::Ellipsoid*>(g);
  if(!e) {
    return false;
  }
  const ksl_vec3_t& axes = e->get_semi_axes();
  return std::min({axes.x, axes.y, axes.z}) > 0.0;
}

bool shift::core::collision_geometry::TestContact(ContactPair& pair) {
  pair.manifold.clear();
  pair.point_laws = false;
  auto cad_a = dynamic_cast<const Cad*>(pair.a);
  auto cad_b = dynamic_cast<const Cad*>(pair.b);
  if(cad_a && cad_b) {
    return CadCadContact(pair);
  }
  if((cad_a || cad_b) && (dynamic_cast<const Plane*>(pair.a) ||
                          dynamic_cast<const Plane*>(pair.b))) {
    return CadPlaneContact(pair);
  }
  if((cad_a && IsRound(pair.b)) || (cad_b && IsRound(pair.a))) {
    return CadRoundContact(pair);
  }
  if((cad_a && pair.b->get_exact_shape()) ||
     (cad_b && pair.a->get_exact_shape())) {
    return CadExactContact(pair);
  }
  if(!TestDeepestPoint(pair)) {
    return pair.collision;
  }
  if(cad_a || cad_b) {
    /* the convex hull of CAD geometry with a shape that has no exact form: its
    deepest point, Hertz contact of an edge of the CAD geometry */
    const Cad& cad = cad_a ? *cad_a : *cad_b;
    ContactPoint c;
    c.depth = pair.depth;
    c.point_a = pair.point_a;
    c.point_b = pair.point_b;
    c.normal = pair.normal;
    c.curvature = 1.0 / cad.get_edge_radius();
    pair.manifold.push_back(c);
    pair.point_laws = true;
    return pair.collision;
  }
  ComputeManifold(pair);
  return pair.collision;
}
