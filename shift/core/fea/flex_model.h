#ifndef SHIFT_CORE_FEA_FLEX_MODEL_H_
#define SHIFT_CORE_FEA_FLEX_MODEL_H_

#include <array>
#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift/core/cad/brep.h"
#include "shift/core/fea/element.h"
#include "shift/core/fea/reduction.h"
#include "shift/core/fea/tet_mesh.h"

namespace shift {
namespace core {
namespace fea {

/*!
@brief a solid of a STEP file placed in a body: the body's origin
coordinates of a point p of the solid are \f$s (R \underline{p} +
\underline{t})\f$, with the solid's placement in the file (R, t) and the
geometry's scale s (model length units per meter)
*/
struct PlacedSolid {
  const cad::Model* model = nullptr;
  cad::PlacedBody placed;
  double scale = 1.0;
};

/*!
@brief a mesh of 10-node tetrahedra of a placed solid, in the body's origin
coordinates; \p element_size and \p minimum_element_size are in model units.
The \p interior points (body origin coordinates) inside the solid are nodes
(TetMesh::interior_nodes).
*/
TetMesh MeshPlacedSolid(const PlacedSolid& solid, double element_size,
                        double minimum_element_size,
                        const std::vector<ksl_vec3_t>& interior = {});

/*!
@brief what a frame of a flexible body is tied to, in the body's origin
coordinates

- faces given: the nodes on those B-rep faces;
- kRevolute: the nodes on the cylindrical faces coaxial with the axis
  through position (the bores, whose material lies outside the cylinder,
  with the smallest radius; a pin, if there is no bore); all of them, so a
  frame between two lugs takes both bores;
- kSpherical: the nodes on the spherical faces centred on position (the
  socket before the ball, as for a bore);
- with no B-rep (a mesh made from triangles, ChooseAttachment without a
  solid), the same from the boundary triangles: those whose normals point
  to or from the axis (or centre), within 15 degrees, grouped by their
  distance from it (within 5%), the bores before the pins and the smallest
  first; a group counts if it wraps
  round the axis or centre: the area-weighted mean of the outer products
  of its directions from it has its smallest eigenvalue across the axis
  above 0.2 (0.5 for a whole bore, 0.29 for a third of one, but 0.02 for a
  flat face before the frame or two walls either side of it), or for a
  sphere its smallest above 0.1 (0.21 for a cap of half angle 60
  degrees);
- otherwise, or if no such face is found: a frame inside the material,
  meshed as node (MeshPlacedSolid's interior points), is tied to that node
  and the nodes of the elements around it; any other to the nodes within
  radius of position (element_size if radius is 0), as is a node given a
  radius.
*/
struct AttachmentRequest {
  enum class Joint { kNone, kRevolute, kSpherical };
  std::string name; //!< the frame, for messages
  ksl_vec3_t position = {};
  Joint joint = Joint::kNone;
  ksl_vec3_t axis = {{0.0, 0.0, 1.0}};
  Attachment::Spider spider = Attachment::Spider::kRbe3;
  std::vector<int> faces;
  double radius = 0.0;
  int node = -1; //!< the mesh node at position, if it is one
};

/*!
@brief the attachment of a request on a mesh of a placed solid, or of
triangles if \p solid is null; \p faces_used, if given, receives the B-rep
faces tied (empty for a patch round a point, or with no solid). Throws
shift::core::Exception, naming the frame, if no node is found.
*/
Attachment ChooseAttachment(const TetMesh& mesh, const PlacedSolid* solid,
                            const AttachmentRequest& request,
                            double element_size,
                            std::vector<int>* faces_used = nullptr);

/*! \brief the nodes of the boundary triangles of a revolute joint's bore
 * or pin, or a spherical joint's socket or ball, found from the triangles
 * alone (see AttachmentRequest); empty if there is none */
std::vector<int> BearingNodes(const TetMesh& mesh,
                              const AttachmentRequest& request);

/*!
@brief the wall of a bore that a prismatic joint's child slides along: the
nodes of its boundary triangles, the extent along the axis of those nodes
(low to high, from the frame), and the mean distance of the wall from the
axis
*/
struct Bore {
  std::vector<int> nodes;
  double low = 0.0;
  double high = 0.0;
  double radius = 0.0;
};

/*!
@brief the bore round the axis through request.position along request.axis:
rays cast from the position at right angles to the axis in 16 directions
round it hit the mesh's boundary; the faces first hit (boundary_face) seed
the wall, which is their triangles whose normals are at right angles to the
axis (the cosine of the angle between them below 0.25, as the flat
triangles on a curved wall lean a little). Throws shift::core::Exception, naming the frame, if a ray
hits nothing (the wall does not go all the way round: an open guide, which
is not supported) or a wall not parallel to the axis.
*/
Bore FindBore(const TetMesh& mesh, const AttachmentRequest& request);

/*!
@brief where a part bears on a bore of \p radius round the axis through \p
origin along the unit vector \p axis: the intervals [a, b] of offsets along
the axis from origin covered by the triangles of the part's surface (\p
points, \p triangles) that lie on the bore, their corners within 5% of
radius of it and their normals at right angles to the axis (the cosine of
the angle between them below 0.25, as FindBore), joined where they overlap
or lie within 0.1% of radius of each other; in increasing order. Empty if no
triangle lies on the bore.
*/
std::vector<std::array<double, 2>>
BearingIntervals(const std::vector<ksl_vec3_t>& points,
                 const std::vector<std::array<int, 3>>& triangles,
                 const ksl_vec3_t& origin, const ksl_vec3_t& axis,
                 double radius);

/*! \brief the attachment of a frame that no faces carry: its node and the
 * nodes of the elements around it, or the nodes within a radius (see
 * AttachmentRequest) */
Attachment PatchAttachment(const TetMesh& mesh,
                           const AttachmentRequest& request,
                           double element_size);

/*! @brief the nodes nearest the given positions (body origin coordinates),
 * one each */
std::vector<int> NearestNodes(const TetMesh& mesh,
                              const std::vector<ksl_vec3_t>& positions);

} // namespace fea
} // namespace core
} // namespace shift

#endif
