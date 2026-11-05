#ifndef SHIFT_GEOMETRY_MASS_PROPERTIES_H_
#define SHIFT_GEOMETRY_MASS_PROPERTIES_H_

#include <vector>

#include "ksl/types.h"

namespace shift {
namespace core {
namespace geometry {

/*! \brief mass properties of a solid of unit density */
struct MassProperties {
  double volume = 0.0;
  ksl_vec3_t centroid = {{0.0, 0.0, 0.0}};
  /*! moment of inertia about the centroid, the matrix entries (products of
   * inertia negative), as the terms of ksl_inertia_t */
  double Ixx = 0.0, Iyy = 0.0, Izz = 0.0, Ixy = 0.0, Iyz = 0.0, Izx = 0.0;
};

/*!
@brief whether triangles form closed surfaces: with the vertices welded by
position, each edge belongs to exactly two triangles, which traverse it in
opposite directions
*/
bool IsClosedSurface(const std::vector<ksl_vec3_t>& vertices,
                     const std::vector<unsigned int>& faces);

/*!
@brief close the holes of oriented triangle surfaces: with the vertices
welded by position, the edges that no triangle traverses in the opposite
direction are chained into boundary loops, and each loop is capped by a fan
of triangles to the mean of its vertices, wound to match the surface. A
planar hole is closed exactly; a nonplanar one approximately.

@return the number of holes closed, or -1 if the boundary edges do not form
simple loops (e.g. an edge shared by more than two triangles), in which case
vertices and faces are unchanged
*/
int CloseHoles(std::vector<ksl_vec3_t>& vertices,
               std::vector<unsigned int>& faces);

/*!
@brief the volume, centroid and moment of inertia of the solid bounded by
closed triangle surfaces (three vertex indices per face), by the divergence
theorem over each triangle (Eberly, D. (2002), "Polyhedral Mass Properties
(Revisited)", Geometric Tools)

The triangles are counterclockwise seen from outside; if they are all
clockwise the volume would be negative, and the result is that of the
reversed surface. Returns false if the volume is zero.
*/
bool ComputeMassProperties(const std::vector<ksl_vec3_t>& vertices,
                           const std::vector<unsigned int>& faces,
                           MassProperties& properties);

} // namespace geometry
} // namespace core
} // namespace shift

#endif
