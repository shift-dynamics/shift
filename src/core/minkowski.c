#include "collisionobject.h"

/*!
@brief support mapping function for two convexhull collision objects
@param [in] a first s2_collisionobject_t containing an s2_convexhull_t object
@param [in] a second s2_collisionobject_t containing an s2_convexhull_t object
@return in contact (change this, this is not what the support mapping function
actually returns. This section is a place holder for you to fill out later)
*/
bool s2_minkowski_convexhullConvexhull(s2_collisionobject_t* restrict c1,
                                       s2_collisionobject_t* restrict c2) {
  return true;
}
