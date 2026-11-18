#ifndef SHIFT_CORE_GEOMETRY_SIMPLIFY_H_
#define SHIFT_CORE_GEOMETRY_SIMPLIFY_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace shift {
namespace core {
namespace geometry {

/*!
@file
@brief Simplification of triangle meshes, for the levels of detail of the
meshes the viewer draws far from the camera

A mesh is given as positions (x, y, z of each vertex), optionally attributes
(a fixed number of floats per vertex, such as the normal and the texture
coordinates), and faces (three vertex indices per triangle). Vertices at the
same position with different attributes (the two sides of a texture seam)
are wedges of one position.

- SimplifyQuadric removes vertices one at a time by half-edge collapses,
  each moving a vertex onto a neighbor, in the order of the least error of
  the quadric error metric (Garland and Heckbert 1997), to which the change
  of the attributes is added. Open borders collapse only along themselves,
  and seams only along themselves, both wedges together, so that the
  outline of a cut-out card and the texture stay in place. A collapse that
  would turn a triangle over is not made. The vertices are those of the
  input (no new ones), so the result is an index buffer into them.
- SimplifyGrid clusters the vertices in a uniform grid, each cluster drawn
  by the vertex nearest to its mean, coarse but able to reach any count:
  for the farthest levels and for meshes of many small parts.
- PruneComponents keeps a share of the connected parts of a mesh (the cards
  of foliage, the blades of grass), chosen at random with a fixed seed, and
  enlarges each kept part about its centroid so that the total area stays
  the same, as foliage is thinned for the distance.
- CompactVertices drops the vertices the faces no longer use.

The errors are in units of the largest side of the mesh's bounding box.
*/

/*! \brief options of SimplifyQuadric */
struct SimplifyOptions {
  /*! stop when the faces are this many or fewer */
  size_t target_face_count = 0;
  /*! stop before a collapse of more error than this (relative to the size
   * of the mesh) */
  double max_error = 1e-2;
  /*! weight of each attribute float in the error, per unit of the size of
   * the mesh: 0 ignores it */
  std::vector<float> attribute_weights;
  /*! keep every vertex of an open border */
  bool lock_border = false;
};

/*!
\brief Simplifies a mesh by quadric error half-edge collapses

\param positions x, y, z of each vertex
\param attributes attribute_count floats per vertex, or empty
\param attribute_count floats of attributes per vertex
\param faces three vertex indices per triangle
\param options the target, the error limit and the attribute weights
\param error if not null, the largest error of the collapses made
\return the faces of the simplified mesh, indices into the same vertices
*/
std::vector<uint32_t> SimplifyQuadric(const std::vector<float>& positions,
                                      const std::vector<float>& attributes,
                                      size_t attribute_count,
                                      const std::vector<uint32_t>& faces,
                                      const SimplifyOptions& options,
                                      double* error = nullptr);

/*!
\brief Simplifies a mesh by clustering its vertices in a grid, the finest
grid that gives at most target_face_count faces

\param positions x, y, z of each vertex
\param faces three vertex indices per triangle
\param target_face_count the most faces of the result
\param error if not null, the diagonal of a cell of the grid
\return the faces of the simplified mesh, indices into the same vertices
*/
std::vector<uint32_t> SimplifyGrid(const std::vector<float>& positions,
                                   const std::vector<uint32_t>& faces,
                                   size_t target_face_count,
                                   double* error = nullptr);

/*!
\brief Keeps a share of the connected parts of a mesh, enlarged so that
their area is that of all the parts

\param positions x, y, z of each vertex
\param faces three vertex indices per triangle
\param keep_fraction the share of the faces to keep, 0 to 1
\param seed of the random choice of the parts
\param scaled_positions the positions, with those of the kept parts
enlarged about the centroids of their parts
\param error if not null, how far the enlarged parts reach beyond where
they were: the enlargement less one times the area-weighted mean radius of
the kept parts (the farthest of their vertices from their centroids)
\return the faces of the kept parts
*/
std::vector<uint32_t> PruneComponents(const std::vector<float>& positions,
                                      const std::vector<uint32_t>& faces,
                                      double keep_fraction, uint32_t seed,
                                      std::vector<float>* scaled_positions,
                                      double* error = nullptr);

/*!
\brief Renumbers the vertices that faces uses, in the order of their first
use, and drops the others

\param faces rewritten in place
\param vertex_count of the vertices faces indexes
\return the old index of each new vertex
*/
std::vector<uint32_t> CompactVertices(std::vector<uint32_t>& faces,
                                      size_t vertex_count);

} // namespace geometry
} // namespace core
} // namespace shift

#endif
