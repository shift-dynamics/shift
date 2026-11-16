#ifndef SHIFT_CORE_FEA_SOLID_MESH_H_
#define SHIFT_CORE_FEA_SOLID_MESH_H_

#include "shift/core/cad/brep.h"
#include "shift/core/fea/tet_mesh.h"

namespace shift {
namespace core {
namespace fea {

/*!
@brief a mesh of 10-node tetrahedra of a solid body of a repaired B-rep, in
the model's coordinates

The boundary is triangulated by cad::Triangulate (edges no longer than \p
element_size, within element_size / 20 of the exact surface), Netgen fills
it (MeshVolume), and each mid-side node of the boundary is moved onto the
exact geometry: onto the edge's curve for a segment along an edge of the
B-rep, onto the face's surface otherwise. A mid-side node whose move would
turn an element inside out stays at the middle of its edge. boundary_face
gives the B-rep face of each boundary triangle.
*/
TetMesh MeshSolid(const cad::Model& model, int body, double element_size,
                  double minimum_element_size = 0.0);

/*! @brief the smallest Jacobian determinant of an element, at its corners
 * and its 4-point rule, relative to that of its straight-sided form */
double Tet10JacobianRatio(const TetMesh& mesh, size_t element);

} // namespace fea
} // namespace core
} // namespace shift

#endif
