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
  /*! outward unit normal n and offset o of each face's plane, n . x = o */
    std::vector<ksl_vec3_t> face_normals;
  std::vector<double> face_offsets;
  /*! the faces merged into polygons: the triangles of a plane of the hull,
   * with the vertex indices of each polygon counterclockwise about its
   * outward normal (see ComputeFacePolygons) */
  std::vector<std::vector<unsigned int>> polygons;
    std::vector<ksl_vec3_t> polygon_normals;
  std::vector<double> polygon_offsets;
  /*! the polygons around each vertex, so that the face touching along a
   * direction is found from the support vertex of that direction */
  std::vector<std::vector<unsigned int>> vertex_polygons;
  /*! the faces (triangles) around each vertex */
  std::vector<std::vector<unsigned int>> vertex_faces;
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

/*! \brief fill the face planes of a convex hull, oriented away from the
 * vertex centroid (an interior point); faces without area get a zero normal
 * and an infinite offset, so they never separate */
void ComputeFacePlanes(ConvexHullData& hull);

/*! \brief merge the triangles of each plane of a convex hull into a polygon,
 * for the contact manifold of faces in contact; needs the face planes */
void ComputeFacePolygons(ConvexHullData& hull);

/*! \brief covariance matrix of a set of points */
ksl_mat3x3_t Covariance(const std::vector<ksl_vec3_t>& points);

/*!
@brief eigen decomposition of a symmetric matrix by Jacobi rotations

On return A is diagonal with the eigenvalues, and the columns of V are the
corresponding eigenvectors; V is a proper rotation (det(V) = 1).
*/
void Jacobi(ksl_mat3x3_t& A, ksl_mat3x3_t& V);

/*!
@brief the frame of a shape with an axis (a cylinder or a capsule) around
points: its z axis is the direction of greatest spread of the points, and its
origin is the center of their bounding box along the three axes

On return, low and high are the extents of the points along z about the
origin, and radius is the greatest distance of a point from the axis.
*/
void FitAxis(const std::vector<ksl_vec3_t>& points, ksl_SE3_t& frame,
             double& low, double& high, double& radius);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
