#include <stdlib.h>

#include "collisionobject.h"
#include "supportmapping.h"

/*!
@brief support mapping function for two convexhull collision objects
@param [in] a first s2_collisionobject_t containing an s2_convexhull_t object
@param [in] a second s2_collisionobject_t containing an s2_convexhull_t object
@return in contact (change this, this is not what the support mapping function
actually returns. This section is a place holder for you to fill out later).
for two convex hulls, this function would call the gjk algorithm to
sample the minkowski difference.
*/
bool s2_supportmapping_convexhullConvexhull(s2_collisionobject_t* restrict c1,
                                            s2_collisionobject_t* restrict c2) {
  /* use (void) to suppress unused variable compiler warning. Remove this once
                                        correct support function is loaded */
  (void)c1;
  (void)c2;
  /*! @TODO add call to gjk algorithm here */
  return true;
}

/*!
@brief support mapping function for two sphere collision objects
@param [in] a first s2_collisionobject_t containing an s2_convexhull_t object
@param [in] a second s2_collisionobject_t containing an s2_convexhull_t object
@return in contact (change this, this is not what the support mapping function
actually returns. This section is a place holder for you to fill out later).
for two convex hulls, this function would call the gjk algorithm to
sample the minkowski difference.
*/
bool s2_supportmapping_sphereSphere(s2_collisionobject_t* restrict c1,
                                    s2_collisionobject_t* restrict c2) {
  /* use (void) to suppress unused variable compiler warning. Remove this once
                                        correct support function is loaded */
  (void)c1;
  (void)c2;
  /*! @TODO add call to sphere-sphere test here */
  return true;
}

/*!
@brief support mapping function for two sphere collision objects
@param [in] a first s2_collisionobject_t containing an s2_convexhull_t object
@param [in] a second s2_collisionobject_t containing an s2_convexhull_t object
@return in contact (change this, this is not what the support mapping function
actually returns. This section is a place holder for you to fill out later).
for two convex hulls, this function would call the gjk algorithm to
sample the minkowski difference.
*/
bool s2_supportmapping_ellipsoidEllipsoid(s2_collisionobject_t* restrict c1,
                                          s2_collisionobject_t* restrict c2) {
  /* use (void) to suppress unused variable compiler warning. Remove this once
  correct support function is loaded */
  (void)c1;
  (void)c2;
  /*! @TODO add call to ellipsoid-ellipsoid test */
  return true;
}

/*!
@brief test for collision between two primitive collision objects. A pointer to
the two collisionobjects and the function pointer stored in s2_supportmapping_t*
*/
void s2_supportmapping_support(s2_supportmapping_t* self) {
  self->collision = self->support(self->object_a, self->object_b);
}

/*!
@brief initializes a new s2_supportmapping_t datastructure and sets the
appropriate support mapping function for sampling the minkowski difference. This
function will be called from contact.c
*/
s2_supportmapping_t* s2_supportmapping_alloc(s2_collisionobject_t* object_a,
                                             s2_collisionobject_t* object_b) {
  s2_supportmapping_t* supportmapping = calloc(1, sizeof(s2_supportmapping_t));
  supportmapping->object_a = object_a;
  supportmapping->object_b = object_b;

  if(object_a->type == S2_COLLISIONOBJECT_SPHERE &&
     object_a->type == S2_COLLISIONOBJECT_SPHERE) {
    supportmapping->support = s2_supportmapping_sphereSphere;
  } else if(object_a->type == S2_COLLISIONOBJECT_ELLIPSOID &&
            object_a->type == S2_COLLISIONOBJECT_ELLIPSOID) {
    supportmapping->support = s2_supportmapping_ellipsoidEllipsoid;
  } else {
    supportmapping->support = s2_supportmapping_convexhullConvexhull;
  }
  /*! @TODO add other level 2 support mapping tests as appropriate */

  /*! @TODO add quantities related to time history and initialize them here */
  supportmapping->prev = NULL;
  supportmapping->next = NULL;

  return supportmapping;
}

/*!
@brief free memory related to an s2_supportmapping_t data structure. If other
quantities are malloc'ed to account for time history, they should also be freed
here.
*/
void s2_supportmapping_free(s2_supportmapping_t* supportmapping) {
  free(supportmapping);
  supportmapping = NULL;
}
