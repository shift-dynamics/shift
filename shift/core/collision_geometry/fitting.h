#ifndef SHIFT_COLLISIONGEOMETRY_FITTING_H_
#define SHIFT_COLLISIONGEOMETRY_FITTING_H_

#include <vector>

#include "ksl/mat3x3.h"
#include "ksl/vec3.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief vertices and triangular faces of a convex hull, with the vertex
adjacency used by the hill climbing support function
*/
struct ConvexHullData {
  std::vector<ksl_vec3_t> vertices;
  std::vector<unsigned int> faces; //!< three vertex indices per face
  std::vector<std::vector<unsigned int>> neighbors;
};

/*!
@brief compute the convex hull of a set of points with qhull

@return false if qhull fails, e.g. because the points are coplanar; hull is
then left empty
*/
bool ComputeConvexHull(const std::vector<ksl_vec3_t>& points,
                       ConvexHullData& hull);

/*! \brief fill the vertex adjacency list of a convex hull from its faces */
void ComputeVertexNeighbors(ConvexHullData& hull);

/*! \brief covariance matrix of a set of points */
ksl_mat3x3_t Covariance(const std::vector<ksl_vec3_t>& points);

/*!
@brief eigen decomposition of a symmetric matrix by Jacobi rotations

On return A is diagonal with the eigenvalues, and the columns of V are the
corresponding eigenvectors; V is a proper rotation (det(V) = 1).
*/
void Jacobi(ksl_mat3x3_t& A, ksl_mat3x3_t& V);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
