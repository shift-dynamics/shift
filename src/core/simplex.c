#include <stdlib.h>

#include "collisionobject.h"
#include "simplex.h"

/*!
@brief support mapping function for two convexhull collision objects
@param [in] a first s2_collisionobject_t containing an s2_convexhull_t object
@param [in] a second s2_collisionobject_t containing an s2_convexhull_t object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_gjk(s2_collisionobject_t* restrict c1,
                    s2_collisionobject_history_t* restrict c1History,
                    s2_collisionobject_t* restrict c2,
                    s2_collisionobject_history_t* restrict c2History) {
  /* use (void) to suppress unused variable compiler warning. Remove this once
                                        correct support function is loaded */
  (void)c1;
  (void)c1History;
  (void)c2;
  (void)c2History;

  /*! @TODO add call to gjk algorithm here

  access extreme vertex support mapping functions for c1 and c2 by calling:

  s2_vector_t searchDirection;

  s2_vector_t extremeVertex1 =
    s2_collisionobject_support(c1, searchDirection);

  s2_vector_t extremeVertex2 =
    s2_collisionobject_support(c2, searchDirection);
  */

  return true;
}

/*!
@brief support mapping function for two sphere collision objects
@param [in] a first s2_collisionobject_t containing an s2_sphere_t object
@param [in] a second s2_collisionobject_t containing an s2_sphere_t object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_sphereSphere(s2_collisionobject_t* restrict c1,
                             s2_collisionobject_history_t* restrict c1History,
                             s2_collisionobject_t* restrict c2,
                             s2_collisionobject_history_t* restrict c2History) {
  /* use (void) to suppress compiler warning. Remove this once
                                        correct support function is loaded */
  (void)c1;
  (void)c1History;
  (void)c2;
  (void)c2History;
  /*! @TODO add call to sphere-sphere test here

  access extreme vertex support mapping functions for c1 and c2 by calling:

  s2_vector_t searchDirection;

  s2_vector_t extremeVertex1 =
    s2_collisionobject_support(c1, searchDirection);

  s2_vector_t extremeVertex2 =
    s2_collisionobject_support(c2, searchDirection);

  */
  return true;
}

/*!
@brief support mapping function for two sphere collision objects
@param [in] a first s2_collisionobject_t containing an s2_ellipsoid_t object
@param [in] a second s2_collisionobject_t containing an s2_ellipsoid_t object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_ellipsoidEllipsoid(
  s2_collisionobject_t* restrict c1,
  s2_collisionobject_history_t* restrict c1History,
  s2_collisionobject_t* restrict c2,
  s2_collisionobject_history_t* restrict c2History) {

  /* use (void) to suppress unused variable compiler warning. Remove this once
  correct support function is loaded */
  (void)c1;
  (void)c1History;
  (void)c2;
  (void)c2History;
  /*! @TODO add call to ellipsoid-ellipsoid test

  access extreme vertex support mapping functions for c1 and c2 by calling:

  s2_vector_t searchDirection;

  s2_vector_t extremeVertex1 =
    s2_collisionobject_support(c1, searchDirection);

  s2_vector_t extremeVertex2 =
    s2_collisionobject_support(c2, searchDirection);

  */
  return true;
}

/*!
@brief test for collision between two primitive collision objects. This function
uses function pointer stored in s2_simplex_t to test for collision between two
collisionobjects

@param self [in] a pointer to a s2_simplex_t datastructure, which contains
information about a pair of s2_collisionobject_t datastructures and their time
histories
*/
void s2_simplex_support(s2_simplex_t* self) {
  self->collision = self->support(self->objectA, &self->objectAHistory,
                                  self->objectB, &self->objectBHistory);
}

/*!
@brief initializes a new s2_simplex_t datastructure and sets the
appropriate support mapping function for sampling the minkowski difference. This
function will be called from contact.c
*/
s2_simplex_t* s2_simplex_alloc(s2_collisionobject_t* object_a,
                               s2_collisionobject_t* object_b) {
  s2_simplex_t* simplex = calloc(1, sizeof(s2_simplex_t));
  simplex->objectA = object_a;
  simplex->objectB = object_b;
  if(object_a->type == S2_COLLISIONOBJECT_SPHERE &&
     object_a->type == S2_COLLISIONOBJECT_SPHERE) {
    simplex->support = s2_simplex_sphereSphere;
  } else if(object_a->type == S2_COLLISIONOBJECT_ELLIPSOID &&
            object_a->type == S2_COLLISIONOBJECT_ELLIPSOID) {
    simplex->support = s2_simplex_ellipsoidEllipsoid;
  } else {
    simplex->support = s2_simplex_gjk;
  }
  /*! @TODO add other level 2 support mapping tests as appropriate */

  /*! @TODO add quantities related to time history and initialize them here,
  they will have been initialized to zero in the call to calloc above */
  simplex->prev = NULL;
  simplex->next = NULL;

  return simplex;
}

/*!
@brief free memory related to an s2_simplex_t data structure. If other
quantities are malloc'ed to account for time history, they should also be freed
here.
*/
void s2_simplex_free(s2_simplex_t* simplex) {
  free(simplex);
  simplex = NULL;
}
