#ifndef SHIFT_CORE_CAD_TRIANGULATE_H_
#define SHIFT_CORE_CAD_TRIANGULATE_H_

#include <array>
#include <map>
#include <utility>
#include <vector>

#include "shift/core/cad/brep.h"

namespace shift {
namespace core {
namespace cad {

struct TriangulateOptions {
  /*! longest edge of a triangle (m) */
  double element_size = 0.0;
  /*! shortest edge that refinement may make (m); 0 for element_size / 20 */
  double minimum_element_size = 0.0;
  /*! largest distance of an edge or triangle from the exact curve or
   * surface (m); 0 for element_size / 20 */
  double sag = 0.0;
  /*! largest turn of a curve's tangent along one segment (rad) */
  double max_turn = 0.5;
  /*! distance (m) within which points are one point (vertices, joins, seams)
   */
  double tolerance = 1e-6;
  /*! most points in all, a guard against runaway refinement */
  int max_points = 4000000;
};

/*!
@brief closed triangulation of the faces of a body of a B-rep

points are in the coordinates of the model (as the faces' surfaces);
triangles are counterclockwise seen from outside the solid, each on the face
face[k], with the parameters uv[k] of its corners on that face's surface. A
pair of points (smaller index first) joined by a segment along an edge of the
B-rep maps to that edge in edge_of; every other segment lies inside a face.
*/
struct Triangulation {
  std::vector<Vec3> points;
  std::vector<std::array<int, 3>> triangles;
  std::vector<int> face;
  std::vector<std::array<Vec2, 3>> uv;
  std::map<std::pair<int, int>, int> edge_of;
};

/*!
@brief triangulate the boundary of a solid body of a repaired model (see
Repair) so that the triangles of neighbouring faces share their points along
the edges between them: the surface is closed

Each edge is divided once, by its length, the turn of its tangent and the
sag, and its points are shared by the faces it bounds. Each face is then
triangulated in its parameters (u, v), constrained to its trim loops, and
refined by inserting points inside it until every triangle is shorter than
element_size, within sag of the surface, and not too thin; no point is added
on an edge. Throws shift::core::Exception if a face has no trims (the model
was not repaired) or a trim has a gap.
*/
Triangulation Triangulate(const Model& model, int body,
                          const TriangulateOptions& options);

} // namespace cad
} // namespace core
} // namespace shift

#endif
