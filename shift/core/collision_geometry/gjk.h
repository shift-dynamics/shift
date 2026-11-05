#ifndef SHIFT_COLLISIONGEOMETRY_GJK_H_
#define SHIFT_COLLISIONGEOMETRY_GJK_H_

#include <vector>

#include "ksl/vec3.h"

namespace shift {
namespace core {
namespace collision_geometry {

class Base;

/*!
@brief a pair of collision geometries tested for contact, with the simplex
history that is cached between subsequent tests of the pair
*/
/*! \brief a point of the contact manifold of a pair, see ContactPair */
struct ContactPoint {
  double depth = 0.0; //!< penetration depth at the point, along the normal
  ksl_vec3_t point_a = {{0.0, 0.0, 0.0}}; //!< on a, inertial frame
  ksl_vec3_t point_b = {{0.0, 0.0, 0.0}}; //!< on b, point_a - depth * normal
  /*! share of the contact law carried by the point (see ComputeManifold): 1
   * for a single point or a vertex of a hull on a plane */
  double weight = 1.0;
  /*! @defgroup point_law Contact law of the point, for pairs whose points
   * each have their own (ContactPair::point_laws, CAD geometry) */
  /*! @{ */
  /*! unit normal from a to b at the point, inertial frame */
  ksl_vec3_t normal = {{1.0, 0.0, 0.0}};
  /*! the sum of the mean curvatures of the two surfaces at the point
   * (positive where convex), for Hertz contact at the point */
  double curvature = 0.0;
  /*! ratio of the larger to the smaller principal curvature of the gap
   * between the surfaces at the point, 1 where it is round (see
   * HertzEllipseFactor) */
  double curvature_ratio = 1.0;
  /*! length of a line of contact carried by the point; 0 for a point */
  double length = 0.0;
  /*! @} */
};

struct ContactPair {
  Base* a = nullptr;
  Base* b = nullptr;

  bool collision = false; //!< result of the most recent test
  bool success = false;   //!< whether the most recent test reached a decision
  bool in_scope = false;  //!< a candidate of the last broad phase

  /*! @defgroup contact Contact data, valid while collision is true (see
   * ComputePenetration) */
  /*! @{ */
  double depth = 0.0; //!< penetration depth
  /*! unit contact normal from a to b, inertial frame: moving b by depth *
   * normal separates the geometries */
  ksl_vec3_t normal = {{1.0, 0.0, 0.0}};
  ksl_vec3_t point_a = {{0.0, 0.0, 0.0}}; //!< point of a deepest inside b
  ksl_vec3_t point_b = {{0.0, 0.0, 0.0}}; //!< point of b deepest inside a
  ksl_vec3_t point_local_a = {{0.0, 0.0, 0.0}}; //!< point_a, frame of a
  ksl_vec3_t point_local_b = {{0.0, 0.0, 0.0}}; //!< point_b, frame of b
  /*! false if the penetration depth search hit its iteration limit */
  bool depth_converged = true;
  /*! the points of the manifold each have their own normal and contact law
   * (ContactPoint::normal, curvature and length), as for CAD geometry */
  bool point_laws = false;
  /*! the contact manifold, at least the deepest point (above): for a convex
   * hull on a plane, its vertices below the plane; for two convex hulls, the
   * incident face clipped by the reference face (see manifold.h) */
  std::vector<ContactPoint> manifold;
  /*! @} */

  /*! @defgroup contact_force Contact force data, set by a contact force
   * module (see force_module::Contact) */
  /*! @{ */
  /*! force on b at the contact point, inertial frame; the force on a is
   * opposite */
  ksl_vec3_t force = {{0.0, 0.0, 0.0}};
  /*! @} */

  /*! @defgroup history Historical data, cached between subsequent tests */
  /*! @{ */
  /*! the normal of the last accepted integration step, zero if the pair
   * was not in contact (see Context::AcceptStep): a hull and an ellipsoid
   * keep the minimum of the penetration depth near it (see
   * ConvexHullEllipsoidContact) */
  ksl_vec3_t accepted_normal = {{0.0, 0.0, 0.0}};
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
Base::UpdatePose). Pairs with a plane or a sphere are computed directly (see
analytic_contact.h): a plane with any shape from the shape's support point,
exactly; a sphere with a sphere or a convex hull exactly, with an ellipsoid
from a one dimensional root; two ellipsoids with the Perram-Wertheim contact
function. Convex hulls with hulls or ellipsoids use the GJK algorithm on the
Minkowski difference b - a and, in contact, EPA for the penetration depth
(see epa.h). Pairs in contact get their contact data:
penetration depth, normal and contact points.

@return whether the geometries are in contact; pair.success is false if GJK
did not reach a decision within its iteration limit, in which case false is
returned
*/
bool TestContact(ContactPair& pair);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
