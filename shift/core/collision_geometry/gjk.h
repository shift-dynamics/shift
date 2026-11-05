#ifndef SHIFT_COLLISIONGEOMETRY_GJK_H_
#define SHIFT_COLLISIONGEOMETRY_GJK_H_

#include "ksl/vec3.h"

namespace shift {
namespace core {
namespace collision_geometry {

class Base;

/*!
@brief a pair of collision geometries tested for contact, with the simplex
history that is cached between subsequent tests of the pair
*/
struct ContactPair {
  Base* a = nullptr;
  Base* b = nullptr;

  bool collision = false; //!< result of the most recent test
  bool success = false;   //!< whether the most recent test reached a decision
  bool in_scope = false;  //!< bounding spheres overlapped in the last sweep

  /*! @defgroup history Historical data, cached between subsequent tests */
  /*! @{ */
  int simplex_point_count = 0; //!< points in the current simplex, 0 to 4
  ksl_vec3_t search_direction = {{0.0, 0.0, 0.0}};
  ksl_vec3_t extreme_vertex_local_a[4]; //!< simplex points on a, local
  ksl_vec3_t extreme_vertex_local_b[4]; //!< simplex points on b, local
  int hint_a = 0; //!< most recent extreme vertex index on a
  int hint_b = 0; //!< most recent extreme vertex index on b
  /*! @} */
};

/*!
@brief test a pair of collision geometries for contact

Both geometries must have up to date absolute displacements (see
Base::UpdatePose). Sphere pairs are tested directly, all other pairs with the
GJK algorithm on the Minkowski difference b - a.

@return whether the geometries are in contact; pair.success is false if GJK
did not reach a decision within its iteration limit, in which case false is
returned
*/
bool TestContact(ContactPair& pair);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
