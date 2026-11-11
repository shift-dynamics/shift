#ifndef SHIFT_COLLISIONGEOMETRY_MANIFOLD_H_
#define SHIFT_COLLISIONGEOMETRY_MANIFOLD_H_

#include "shift/core/collision_geometry/gjk.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief fill the contact manifold of a pair in contact, from its deepest
point and normal

A single deepest point cannot hold a flat face at rest: as the face tilts the
deepest point jumps between its corners, and so would a contact force. A
convex hull in contact through a face, or an edge, gets several points:

- a convex hull on a plane: each of its vertices below the plane, with the
  depth of that vertex
- two convex hulls: a reference face, of either hull, facing along the
  contact normal (of a, or opposite to it of b), and an incident face of the
  other hull facing against the reference face. The incident face is clipped
  by the side planes of the reference face (Sutherland-Hodgman), and its
  points behind the reference face are the manifold, with their depths below
  it. Each point is weighted by the exterior angle of the clipped polygon at
  it, 2 theta / pi (the corners of a rectangle 1 each, 4 in all), so the
  contact force does not jump when clipping splits a vertex in two. Faces
  are not chosen but blended: every reference face nearly aligned with the
  normal, and against each every incident face nearly opposed to it, weighs
  \f$\exp((c - c_{best}) / \text{kFaceBlend})\f$, normalized to sum to 1, of
  the cosine \f$c\f$ of its angle and that of the best; the points of each clipped pair of faces
  are scaled by the product of their weights (kFaceBlend is small, so only
  nearly exactly tied faces are blended), and coincident points are
  merged. As two faces become equally aligned the manifold passes from one
  to the other, so the contact force, a function of the state only, does
  not jump between them. The faces of a are favored as reference faces by
  kReferenceBias, so a face of b is one only when clearly better aligned.

- a cylinder or a capsule on a plane or a convex hull: its line of contact,
  as points that each carry a length of it and their own normal
  (ContactPair::point_laws), see PlaneLineManifold and HullLineManifold in
  manifold.cc

Other pairs, and hull pairs whose clipping leaves no point, keep the single
deepest point. Requires the pair's contact data and up to date absolute
displacements of both geometries.

Ericson, C. (2005), Real-Time Collision Detection, Morgan Kaufmann, 8.3.4;
Catto, E. (2007), "Contact manifolds", Game Developers Conference.
*/
void ComputeManifold(ContactPair& pair);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
