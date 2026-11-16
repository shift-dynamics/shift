#ifndef SHIFT_CORE_FEA_TET_MESH_H_
#define SHIFT_CORE_FEA_TET_MESH_H_

#include <array>
#include <vector>

#include "ksl/vec3.h"

namespace shift {
namespace core {

/*!
@brief Finite element models of flexible bodies: meshes of solids, element
matrices, and their reduction to the shapes of body::Modal and body::Nodal
*/
namespace fea {

/*!
@brief closed triangulated surface bounding a solid: points, and triangles
of three point indices, counterclockwise seen from outside the solid (normals
pointing out); face gives, for each triangle, the face of the solid it
lies on (a B-rep face index, or -1)
*/
struct ClosedSurface {
  std::vector<ksl_vec3_t> points;
  std::vector<std::array<int, 3>> triangles;
  std::vector<int> face;
};

/*!
@brief mesh of 10-node (quadratic) tetrahedra

A tetrahedron lists its corners 0 to 3 and then the mid-side nodes of its
edges 01, 12, 20, 03, 13, 23, with corner 3 seen from the side of the
counterclockwise triangle 0, 1, 2 (positive volume). boundary holds the
6-node triangles on the surface (corners, then the mid-side nodes of the
edges 01, 12, 20), counterclockwise seen from outside, and boundary_face the
face of the solid each lies on. The first corner_count nodes are corners.
*/
struct TetMesh {
  std::vector<ksl_vec3_t> nodes;
  unsigned int corner_count = 0;
  std::vector<std::array<int, 10>> tets;
  std::vector<std::array<int, 6>> boundary;
  std::vector<int> boundary_face;

  /*! the volume of the straight-sided tetrahedra */
  double Volume() const;
};

/*!
@brief fill a closed surface with tetrahedra, by Netgen

The surface's points and triangles are kept as the boundary of the mesh, so
they set its resolution there: Netgen does not refine them. It places the
interior nodes, with no edge longer than about \p element_size and none
shorter than \p minimum_element_size (0 for no limit), grading from the
sizes of the boundary triangles, and optimizes the elements' shapes. The
result has straight 10-node tetrahedra (mid-side nodes at the middle of each
edge); the caller moves the mid-side nodes of the boundary onto the exact
surface. Throws shift::core::Exception if the surface is not closed (each
edge used once in each direction), or if Netgen fails or its tetrahedra do
not fill the volume the surface encloses (a surface that intersects
itself).
*/
TetMesh MeshVolume(const ClosedSurface& surface, double element_size,
                   double minimum_element_size = 0.0);

} // namespace fea
} // namespace core
} // namespace shift

#endif
