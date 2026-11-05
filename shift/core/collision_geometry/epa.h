#ifndef SHIFT_COLLISIONGEOMETRY_EPA_H_
#define SHIFT_COLLISIONGEOMETRY_EPA_H_

namespace shift {
namespace core {
namespace collision_geometry {

struct ContactPair;

/*!
@brief penetration depth, contact normal and contact points of a pair that
GJK found in contact, by the expanding polytope algorithm (EPA)

EPA grows a polytope inside the Minkowski difference b - a, starting from the
final GJK simplex (expanded to a tetrahedron if GJK stopped on a point, a
segment or a triangle that contains the origin), until the face of the
polytope closest to the origin lies on the boundary of the difference. That
face gives the minimum translation that separates the geometries: its
distance from the origin is the penetration depth, and its normal is the
contact normal. The barycentric coordinates of the origin's projection onto
the face, applied to the support points of a and b that built the face, give
the contact points.

Fills pair.depth, pair.normal (unit, from a to b: moving b by depth * normal
separates the geometries), pair.point_a (the point of a deepest inside b),
pair.point_b (the point of b deepest inside a), so that
point_a - point_b = depth * normal, and their coordinates in each geometry's
reference frame. On a flat Minkowski difference (touching geometries) the
depth is zero.

Both geometries must have up to date absolute displacements, and pair must
hold the simplex of a GJK test that returned contact.

van den Bergen, G. (2001), "Proximity queries and penetration depth
computation on 3D game objects", Game Developers Conference.

@return pair.depth_converged: false if the iteration limit was reached before
the closest face converged; the best face found is used
*/
bool ComputePenetration(ContactPair& pair);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
