#ifndef SHIFT_COLLISIONGEOMETRY_ANALYTICCONTACT_H_
#define SHIFT_COLLISIONGEOMETRY_ANALYTICCONTACT_H_

#include "ksl/vec3.h"

namespace shift {
namespace core {
namespace collision_geometry {

class Base;
class ConvexHull;
class Cylinder;
class Ellipsoid;
class Plane;
class Sphere;

/*!
@brief contact of two shapes, first and second, computed directly for the
pairs that need no search over a Minkowski difference (GJK and EPA)

All quantities are in the inertial frame. normal is the unit normal from the
first shape to the second: moving the second by \f$\mathrm{depth} \cdot \underline{n}\f$ separates
them. point_first is the point of the first shape deepest inside the second,
point_second that of the second inside the first, and
\f$\text{point\_first} - \text{point\_second} = \mathrm{depth} \cdot \underline{n}\f$.
*/
struct AnalyticContact {
  bool collision = false;
  double depth = 0.0;
  ksl_vec3_t normal = {{1.0, 0.0, 0.0}};
  ksl_vec3_t point_first = {{0.0, 0.0, 0.0}};
  ksl_vec3_t point_second = {{0.0, 0.0, 0.0}};
};

/*!
@brief a convex hull and a sphere, exact

A sphere is its center grown by its radius, so the contact follows from the
point of the hull closest to the center c: if c is inside the hull, its
projection onto the nearest face plane; if outside, the closest point of the
faces whose planes c is in front of. A face plane farther than the radius in
front of c separates the shapes.

@return false if the hull has no faces (a degenerate hull), so the pair must
be handled otherwise
*/
bool ConvexHullSphereContact(const ConvexHull& hull, const Sphere& sphere,
                             AnalyticContact& contact);

/*!
@brief the point of an ellipsoid closest to a point p, inside or outside

p is in the inertial frame; q and the outward unit normal n there are in the
frame of the ellipsoid. distance is negative if p is inside.
*/
void ClosestOnEllipsoid(const Ellipsoid& ellipsoid, const ksl_vec3_t& p,
                        ksl_vec3_t& q, ksl_vec3_t& n, double& distance);

/*!
@brief an ellipsoid and a sphere

From the point of the ellipsoid closest to the sphere center, inside or
outside. It has no closed form (it is a root of a sixth degree polynomial),
but follows from the root of a monotone function of one variable, found by
bisection to machine precision:

Eberly, D. (2013), "Distance from a Point to an Ellipse, an Ellipsoid, or a
Hyperellipsoid", Geometric Tools.

@return false if a semi axis is zero (a degenerate ellipsoid)
*/
bool EllipsoidSphereContact(const Ellipsoid& ellipsoid, const Sphere& sphere,
                            AnalyticContact& contact);

/*!
@brief a convex hull (first) and an ellipsoid (second), exact

The penetration depth is the least, over unit directions \f$\underline{n}\f$,
of the translation of the ellipsoid along \f$\underline{n}\f$ that separates
the shapes,

\f[
d(\underline{n}) = \max_{\underline{v}} \underline{n} \cdot
  (\underline{v} - \underline{c}) + \sqrt{\underline{n}^T G \underline{n}},
\f]

with \f$\underline{v}\f$ the hull's vertices, \f$\underline{c}\f$ the
ellipsoid's center and \f$G = R \, \mathrm{diag}(a_x^2, a_y^2, a_z^2) R^T\f$ its
shape matrix. \f$d\f$ is the largest of smooth functions, one per vertex, so
its least value is set by a feature of the hull, the vertices whose
functions are largest there:

- a vertex v: n is the inward normal of the ellipsoid at a point nearest to
  v, the least d_v by Newton's method on the unit sphere from the walk's
  current normal (see EllipsoidEllipsoidContact, with b a point)
- an edge: the least d over the normals of the edge, a function of one
  angle, from the root of its derivative by bisection
- a face (a triangle; the polygons of coplanar triangles are only nearly
  planar): its normal

Each candidate feature is solved for its own least d inside its normal cone,
where the support point q of the ellipsoid against n projects onto the
feature; the candidates are the vertices within a margin of the extreme along
start (an estimate of the normal, e.g. EPA's), and the edges and faces made
of them, the margin growing until one passes. Deep overlaps have several
local minima: of the passing ones, the least d is kept, or, if nearest, the
one whose normal is nearest start, so that a pair keeps the minimum of its
last accepted step (see ContactPair::accepted_normal) instead of jumping to
another of nearly equal depth. With start fixed, the result depends on the
poses only, and is smooth where the feature does not change, unlike the
result of EPA, whose normal on a curved surface is only as good as the
square root of its tolerance.
point_first is on the feature and point_second is q.

@return false if no candidate passes or the hull has no faces, so the pair
must be handled otherwise
*/
bool ConvexHullEllipsoidContact(const ConvexHull& hull,
                                const Ellipsoid& ellipsoid,
                                const ksl_vec3_t& start, bool nearest,
                                AnalyticContact& contact);

/*!
@brief two convex hulls, the local minimum of the penetration depth nearest
a normal, exact

The translation of b along a unit direction \f$\underline{n}\f$ that
separates the hulls,
\f$d(\underline{n}) = h_a(\underline{n}) + h_b(-\underline{n})\f$, is least at a face of their Minkowski difference
whose foot point from the origin lies inside it, one of

- a face of a and a vertex of b: n is the face's normal, and the vertex,
  moved by d n, is inside the face
- a vertex of a and a face of b: n is minus the face's normal, likewise
- an edge of a and an edge of b: n is normal to both, in the normal cone of
  a's edge with -n in that of b's, and the closest points of their lines are
  on both edges

For deep overlaps, d has several local minima, of nearly equal depth when
the hulls are nested or stacked, and the deepest point jumps between them as
the poses change. The candidates are made of the vertices of a within a
margin of the extreme along start, and of b against it (the margin growing
until one passes, to 1e-2 of the hulls' size: start is a step old), and the
passing one whose normal is nearest start is
kept: from the normal of a pair's last accepted step (see
ContactPair::accepted_normal), the minimum it was in while it lasts. The
near vertices are found by a search over the hull's edges from the support
vertex (hint_a and hint_b, updated), so only the hulls' vertices near the
contact are transformed. The result is exact (no tolerance), a function of
the poses for a fixed start.

@return false if no candidate passes or a hull has no faces, so the pair
must be handled otherwise (EPA)
*/
bool ConvexHullPairContact(const ConvexHull& a, const ConvexHull& b,
                           const ksl_vec3_t& start, int& hint_a, int& hint_b,
                           AnalyticContact& contact);

/*!
@brief two ellipsoids, by the Perram-Wertheim contact function

With the shape matrices \f$G = R \, \mathrm{diag}(a_x^2, a_y^2, a_z^2) R^T\f$ of the
ellipsoids, \f$\underline{r} = \underline{c}_b - \underline{c}_a\f$ and
\f$M(\ell) = (1 - \ell) G_a + \ell G_b\f$, the function

\f[
f(\ell) = \ell (1 - \ell) \underline{r}^T M(\ell)^{-1} \underline{r},
\quad 0 \le \ell \le 1
\f]

is concave, and its maximum is the square of the factor by which both
ellipsoids must be scaled about their centers to just touch: they overlap if
\f$\max f < 1\f$. The maximum is found from the root of
\f$f'\f$, which is decreasing, by safeguarded regula falsi. At the maximum,
\f$M^{-1} \underline{r}\f$ is the common normal of the scaled ellipsoids at
their contact point.

\f$f\f$ measures the overlap as a scale factor rather than a distance. For the
penetration depth, the separating distance along a unit direction
\f$\underline{n}\f$,

\f[
d(\underline{n}) = h_a(\underline{n}) + h_b(\underline{n}) -
  \underline{n} \cdot \underline{r}, \quad
h(\underline{n}) = \sqrt{\underline{n}^T G \underline{n}},
\f]

(the translation along \f$\underline{n}\f$ that separates the ellipsoids) is
minimized by Newton's method on the unit sphere. \f$d(\underline{n})\f$ is the
distance from \f$\underline{r}\f$ to the
boundary of the Minkowski difference, which for deep overlaps has several
local minima (one on each side of r), so the minimization starts from the
Perram-Wertheim normal, from the principal axes of both ellipsoids and from
their opposites, and keeps the least. At the minimum the support points of a
along n and of b against it are depth * n apart.

Perram, J. W. and Wertheim, M. S. (1985), "Statistical mechanics of hard
ellipsoids. I. Overlap algorithm and the contact function", Journal of
Computational Physics 58(3), 409-416.

@return false if a semi axis is zero or the centers coincide, so the pair
must be handled otherwise
*/
bool EllipsoidEllipsoidContact(const Ellipsoid& a, const Ellipsoid& b,
                               AnalyticContact& contact);

/*!
@brief a cylinder (first) and a sphere, exact

A sphere is its center grown by its radius, so the contact follows from the
point of the cylinder closest to the center: on its side, on an end, or on a
rim, in closed form. Inside, it is the nearer of the side and the end.

@return false if the cylinder has no radius or length
*/
bool CylinderSphereContact(const Cylinder& cylinder, const Sphere& sphere,
                           AnalyticContact& contact);

/*!
@brief a convex hull (first) and a cylinder, the local minimum of the
penetration depth nearest a normal, exact

The translation of the cylinder along a unit direction \f$\underline{n}\f$
that separates the shapes is

\f[
d(\underline{n}) = \max_{\underline{v}} \underline{n} \cdot
  (\underline{v} - \underline{c}) + h \, |g| + r \sqrt{1 - g^2}, \quad
g = \underline{n} \cdot \underline{a},
\f]

with \f$\underline{v}\f$ the hull's vertices, \f$\underline{c}\f$ the
cylinder's center, \f$\underline{a}\f$ its unit axis, \f$h\f$ its half length
and \f$r\f$ its radius: the support function of the cylinder is closed form.
\f$d\f$ is least at a face of the Minkowski difference of the shapes, one of

- a face of the hull (a polygon) and the cylinder's point deepest against
  its normal (on a rim; the nearest point of an end, or of a side line, to
  the polygon where that end or line is parallel to it): n is the face's
  normal, and the point projects into the face
- an end of the cylinder and the hull's vertex, or edge, extreme along its
  normal: n is the end's inward normal, and the vertex or a point of the
  edge projects into the end's disc
- the side of the cylinder and an edge of the hull: n is normal to both the
  axis and the edge, in the edge's normal cone, and the closest points of
  the axis and the edge are on both
- the side and a vertex of the hull: n is from the vertex across the axis,
  in the vertex's normal cone, and the vertex is level with the side
- a rim and an edge of the hull: the least d over the edge's normal cone, a
  function of one angle (the distance of a circle and a line, a root of a
  quartic) from the root of its derivative by bisection to rounding, where
  the rim's point deepest against n projects onto the edge

A rim and a vertex, and a rim and a face, make no face of the Minkowski
difference. The candidates are made of the hull's vertices within a margin
of the extreme along start, the margin growing until one passes, and of the
passing ones the least d is kept, or, if nearest, the one whose normal is
nearest start, as for ConvexHullEllipsoidContact. With start fixed the result
depends on the poses only, and is exact where that of EPA is only as good as
its tolerance. The near vertices are found by a search over the hull's edges
from the support vertex (hint, updated). point_first is on the hull and
point_second on the cylinder.

@return false if no candidate passes, the hull has no polygons or the
cylinder no radius or length, so the pair must be handled otherwise (EPA)
*/
bool CylinderHullContact(const ConvexHull& hull, const Cylinder& cylinder,
                         const ksl_vec3_t& start, bool nearest, int& hint,
                         AnalyticContact& contact);

/*!
@brief a plane (half space) and any shape, exact

The point of a convex shape deepest in a half space is its support point
against the plane normal: c - r n for a sphere, the closed form support point
of an ellipsoid, the extreme vertex of a convex hull. Two planes overlap
without bound unless their normals are opposite; they are then in contact
when the slab between them has positive thickness, which is the depth, and
otherwise never reported in contact.
*/
void PlaneContact(const Plane& plane, const Base& other,
                  AnalyticContact& contact);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
