#include <stdlib.h>

#include "bitfield.h"
#include "collisionobject.h"
#include "context.h"
#include "simplex.h"

#define EPS 1.0e-10

int s2_simplex_hashIndex(int object_a, int object_b);

/*!
@}
@name restore_ellipsoid_support
@{
*/

/*!
@brief Coordinates of the most recent simplexPointCount extremeVertexLocal
vectors are stored prior to completion of the previous mechanism pose. Before
initializing GJK iteration at the current pose, dAbsolute transforms and
translates the saved extremeVertexLocal coordinates into extremeVertexGlobal
coordinates.

\f$extremeVertexGlobal[i] \leftarrow dAbsolute * extremeVertexLocal[i]\f$

@param [out] extremeVertexGlobal vectors
@param [in] dAbsolute matrix
@param [in] extremeVertexLocal vectors
@param [in] simplexPointCount integer
*/
void s2_simplex_restoreSupport(s2_vector_t extremeVertexGlobal[4],
                               s2_displacement_t* dAbsolute,
                               s2_vector_t extremeVertexLocal[4],
                               int simplexPointCount) {
  printf("\n+++++local extreme vertex support matrix+++++\n");

  for(int i = 0; i < simplexPointCount; i++) {
    s2_vector_print(stdout, &extremeVertexLocal[i]);
  }

  printf("extremeVertexGlobal support matri\n");
  for(int i = 0; i < simplexPointCount; i++) {
    s2_product_dv(dAbsolute, &extremeVertexLocal[i], &extremeVertexGlobal[i]);
  }
}

/*! @brief push a simplex data structure onto end of simplex doubly linked list
 */
void s2_simplex_push(s2_context_t* context, s2_simplex_t* simplex) {

  int index =
    s2_simplex_hashIndex(simplex->objectA->index, simplex->objectB->index);
  s2_bitfield_setTrue(context->simplexExistsBitfield, index);

  if(context->simplexHead == NULL) {
    context->simplexHead = simplex;
    context->simplexTail = simplex;
    simplex->next = NULL;
    simplex->prev = NULL;
  } else {
    context->simplexTail->next = simplex;
    simplex->prev = context->simplexTail;
    context->simplexTail = simplex;
    simplex->next = NULL;
  }
}

/*! @brief remove a simplex data structure from a list of simplices */
void s2_simplex_pop(s2_context_t* context, s2_simplex_t* simplex) {

  int index =
    s2_simplex_hashIndex(simplex->objectA->index, simplex->objectB->index);
  s2_bitfield_setFalse(context->simplexExistsBitfield, index);

  /* test if simplex is at beginning of list */
  if(simplex->prev) {
    simplex->prev->next = simplex->next;
  } else {
    /* beginning of list: */
    context->simplexHead = simplex->next;
  }

  /* test if simplex is at end of list */
  if(simplex->next) {
    simplex->next->prev = simplex->prev;
  } else {
    /* end of list */
    context->simplexTail = simplex->prev;
  }
  simplex->next = NULL;
  simplex->prev = NULL;
}

/*!
@brief support mapping function for two convexhull collision objects
@param [in] a context data structure containing model definition information
@param [in] a simplex data structure containing information relating two
collisionobjects
@return whether the two objects are in contact with one another
*/
bool s2_simplex_gjk(s2_context_t* context, s2_simplex_t* self) {

  s2_collisionobject_t* cA = self->objectA;
  s2_collisionobject_t* cB = self->objectB;

  /**
  This function carries out a full set of GJK contact detection iterations on a
  pair of convex hulls. Before starting the current iteration cycle it must
  first compute the absolute spatial displacement of the hull A/B reference
  frames. For each convex hull pair, on the very first call to this function
  when numberSimplexPoints = 0, it must compute global coordinates of the
  searchDirectionVector. From then on when numberSimplexPoints > 0, the
  searchDirectionVector coordinates will be obtained from the most recent
  simplex. Each GJK iteration cycle starts by sending the most recent
  searchDirectionVector to the convex hull A/B support functions to obtain
  absolute extreme points A/B on the respective surfaces. The simplex support
  function subtracts coordinates of point A from point B to obtain the next
  support point on the convex Minkowski hull, which is also the next point to be
  inserted into the first position of the simplex. To make room for this
  point, existing points are shifted down one position. The simplex is then
  analyzed to determine whether or not the two objects are in contact. If a
  determination of collision or not can be made, a success flag is set to
  true and a collision flag is set to true (contact) or false (no contact). If a
  determination can't be made, success remains false, a new
  searchDirectionVector is generated from the simplex, and zero, one, or two old
  support points are eliminated. Removing support points reduces the simplex
  dimension and numberSimplexPoints, and inserting a support point increases the
  simplex dimension and numberSimplexPoints.
  */

  s2_vector_t extremeVertexAbsoluteA[4];  // numberSimplexPoints absolute
                                          // coordinates of extreme points on
                                          // hull A
  s2_vector_t extremeVertexAbsoluteB[4];  // numberSimplexPoints absolute
                                          // coordinates of extreme points on
                                          // hull B
  s2_vector_t simplexVectorsAbsolute[13]; // Absolute coordinates of
                                          // intermediate simplex vectors

  /**
  The GJK iteration terminates when success = true, or when the maximum
  number of GJK iterations has been reached. SimplexSupportFunction() will
  return with success = true if it has made a contact decision, and it will have
  set collision to true if contact and to false of no contact.
  */
  self->success = false;

  /**
  On each call to SimplexSupportFunction the kinematics or dynamics states may
  have changed. Thus the body spatial displacements may have changed and the
  spatial displacements of convex hulls A and B may have changed. The arrays
  spatialDisplacementHullA and spatialDisplacementHullB contain the spatial
  displacement of the convex hull A and B reference frames.

  @param [in] cA->displacementUpdated boolean
  @param [in] cB->displacementUpdated boolean

  When a displacementUpdated flag is false, the corresponding dabsolute is
  updated and the flag is set to true.
  */
  s2_displacement_t* dAbsoluteA = &cA->dAbsolute;
  if(!cA->displacementUpdated) {
    cA->displacementUpdated = true;
    s2_product_dd(cA->referenceFrame, &cA->dWRTReference, dAbsoluteA);
  }

  s2_displacement_t* dAbsoluteB = &cB->dAbsolute;
  if(!cB->displacementUpdated) {
    cB->displacementUpdated = true;
    s2_product_dd(cB->referenceFrame, &cB->dWRTReference, dAbsoluteB);
  }

  /**
  For each convex hull A and B pair, SimplexSupportFunction is called once
  for each updated kinematics or dynamics state. On the very first call to
  SimplexSupportFunction for a given convex hull A and B pair, no simplex
  history has been established. The program that calls SimplexSupportFunction
  must first set numberSimplexPoints to 0. On subsequent calls to
  SimplexSupportFunction, numberSimplexPoints will never be less than 1 and
  rarely will it be less than 2. An initial test for numberSimplexPoints == 0
  is used to establish the first search-direction and simplex support vectors,
  and to set numberSimplexPoints to one.
  */

  if(self->simplexPointCount == 0) {
    self->simplexPointCount = 1;

    /**
    On the very first call to SimplexSupportFunction for each convex hull A and
    B pair, no searchDirectionVector is known. A best estimate is made by
    subtracting the absolute origin position of objectB from objectA.
    */

    s2_subtract_vv(&dAbsoluteA->T, &dAbsoluteB->T,
                   &self->searchDirectionVector);

    /**
     Use negativeSearchDirectionVector = -self->searchDirectionVector in
     s2_collisionobject_support for objectA. Local and absolute coordinates of
     extreme vertex on convex hull A.
     */

    s2_vector_t negativeSearchDirectionVector;
    s2_vector_invert(&self->searchDirectionVector,
                     &negativeSearchDirectionVector);

    s2_collisionobject_support(cA, context, &negativeSearchDirectionVector,
                               &cA->extremeVertexLocal[0],
                               &extremeVertexAbsoluteA[0]);

    /**
    Use self->searchDirectionVector in s2_collisionobject_support for objectB.
    Local and absolute coordinates of extreme vertex on convex hull B.
    */

    s2_collisionobject_support(cB, context, &self->searchDirectionVector,
                               &cB->extremeVertexLocal[0],
                               &extremeVertexAbsoluteB[0]);

    /**
    Subtract the extremeVertexAbsoluteA on objectA from extremeVertexAbsoluteB
    on objectB to obtain next simplex vector.
    */
    s2_subtract_vv(&extremeVertexAbsoluteB[0], &extremeVertexAbsoluteA[0],
                   &simplexVectorsAbsolute[0]);

  }
  /**
  The current simplex is composed of numberSimplexPoints sets of simplex
  support vertex or point coordinates expressed in the global coordinate
  frame. These coordinates are valid for the current kinematics or dynamics
  states but become invalid for the next states because they may have changed.
  The absolute simplex support vertex or point coordinates are computed from
  the local convex hull A and B support vertex or point coordinates, which
  are stored in the simplex support structure. The functions
  RestoreConvexHullSupportA and RestoreConvexHullSupportB transform the
  respective stored local vertex or point coordinates to global coordinates,
  translate them to global positions, and return them in the respective
  extremeVertexAbsoluteA and extremeVertexAbsoluteB arrays. Coordinates in
  extremeVertexAbsoluteA are subtracted from those in extremeVertexAbsoluteB to
  obtain the simplex support vectors stored in simplexVectorsAbsolute.
  */
  else {

    printf("\n*****Restoring simplex***** with %d vectors.\n",
           self->simplexPointCount);
    self->restoreSupport(extremeVertexAbsoluteA, dAbsoluteA,
                         cA->extremeVertexLocal, self->simplexPointCount);
    self->restoreSupport(extremeVertexAbsoluteB, dAbsoluteB,
                         cB->extremeVertexLocal, self->simplexPointCount);

    for(int i = 0; i < self->simplexPointCount; i++) {
      s2_subtract_vv(&extremeVertexAbsoluteB[i], &extremeVertexAbsoluteA[i],
                     &simplexVectorsAbsolute[i]);
    }
  }

  /**
  Iterate until success or failure. Return success = true when a contact
  decision has been made; otherwise return success = false. Return contact =
  true when contact and false when no contact. success = false signals
  failure of the GJK algorithm, requiring other corrective action.
  numberSimplexPoints can have a value of 1, 2, 3, or 4 in the following GJK
  while() loop. During a normal step that does not terminate the iteration, a
  particular simplex0, simplex1, simplex2, or simplex3 function generates a new
  searchDirectionVector. If a simplex function makes a contact decision, it sets
  success = true, breaks out of the GJK while loop, and returns control to the
  SimplexSupportFunction to prepare for exit. If the simplex support function
  cannot make a contact decision, it computes coordinates of a new
  searchDirectionVector. In addition, simplex2 may reduce numberSimplexPoints by
  1, and simplex3 may reduce numberSimplexPoints by 1 or 2.

  A hard limit of 50 iterations was set on the GJK while loop. For moderately
  coarse convex polytopes, the maximum number of iterations would be around
  10 or so. When the vertex density is high or the surfaces are functionally
  smooth, the maximum number of iterations has been observed as as high as 32
  when the two surfaces were very close to each other, whether contacting or
  not.
  */

  int numiters = 0;
  while(numiters < 50) {
    numiters++;

    /**
    numberSimplexPoints = 1, 2, 3, or 4, depending on the simplex dimension.

    numberSimplexPoints = 1: A single simplex support point will indicate
    surface contact if its magnitude is less than EPS. In this case simplex0
    will set success = true, collision = true, and return. Otherwise it will
    evaluate coordinates of a new searchDirectionVector and return with success
    = false. simplex0 does not change numberSimplexPoints.

    numberSimplexPoints = 2: Two points define a line. The origin may be on the
    line or not. If the origin is not on the line, it will be somewhere around
    the line. Simplex1 can determine if the origin is outside the convex
    Minkowski hull surface (collision = false) or on the surface (collision =
    true). In these cases simplex1 will return with no other changes except
    success = true. Otherwise it will evaluate coordinates of a new
    searchDirectionVector and return with success = false. simplex1 does not
    change numberSimplexPoints.

    numberSimplexPoints = 3: Three points define a triangle. The origin may be
    on the triangle or not. If the origin is not on the triangle, it will be
    somewhere around the triangle. Simplex2 can determine if the origin is on
    the convex Minkowski hull surface (collision = true) or outside it
    (collision = false). In these cases simplex2 will return with no other
    changes except success = true. Otherwise it will evaluate coordinates of a
    new searchDirectionVector and return with success = false. simplex2
    sometimes reduces numberSimplexPoints by 1.

    numberSimplexPoints = 4: Four points define a tetrahedron. The origin may be
    on or in the tetrahedron or not. If the origin is on or in the tetrahedron,
    it will be in the convex Minkowski hull. Otherwise the origin will not be in
    the convex Minkowski hull. Simplex3 can determine if the origin is on or in
    the convex Minkowski hull surface (collision = true) or outside it
    (collision = false). In these cases simplex3 will return with no other
    changes except success = true. Otherwise it will evaluate coordinates of a
    new searchDirectionVector and return with success = false. simplex3
    sometimes reduces numberSimplexPoints by 1 or 2. The final decision that the
    origin is in the convex Minkowski hull will be made in simplex3.
    */

    printf("t_ab: ");
    s2_vector_print(stdout, &simplexVectorsAbsolute[0]);

    switch(self->simplexPointCount) {
      case 1:
        simplex0(&self->searchDirectionVector, simplexVectorsAbsolute,
                 &self->collision, &self->success);
        break;

      case 2:
        simplex1(&self->searchDirectionVector, simplexVectorsAbsolute,
                 &self->collision, &self->success);
        break;

      case 3:
        simplex2(&self->searchDirectionVector, simplexVectorsAbsolute,
                 cA->extremeVertexLocal, cB->extremeVertexLocal,
                 &self->simplexPointCount, &self->collision, &self->success);
        break;

      case 4:
        simplex3(&self->searchDirectionVector, simplexVectorsAbsolute,
                 cA->extremeVertexLocal, cB->extremeVertexLocal,
                 &self->simplexPointCount, &self->collision, &self->success);
        break;

      default:
        printf("ERROR! Incorrect numberSimplexPoints: %d\n",
               self->simplexPointCount);
        exit(0);
        break;
    };

    if(self->success) {
      if(self->collision) {
        printf("Convex hulls are in contact\n");
      } else {
        printf("Convex hulls are not in contact\n");
      }
      break;
    }

    /**
    Arriving at this point means that success = false and one of the simplex
    functions has determined that the current simplex requires another
    support point. To help obtain this new support point, the simplex
    function generated coordinates of a new searchDirectionVector. As noted
    above, the simplex2 function may have reduced numberSimplexPoints by one and
    the simplex3 function will have reduced numberSimplexPoints by at least one
    and possibly two. The searchDirectionVector will be passed to the
    s2_collisionobject_support function and it will evaluate and store extreme
    vertex or point coordinates in cA->extremeVertexLocal[0] and
    cB->extremeVertexLocal[0]. But first the coordinates already stored in
    cA->extremeVertexLocal and cB->extremeVertexLocal must be shifted down
    one to make room for the new ones.
    */

    for(int i = self->simplexPointCount; i > 0; i--) {
      s2_vector_copy(&cA->extremeVertexLocal[i - 1],
                     &cA->extremeVertexLocal[i]);
    }

    for(int i = self->simplexPointCount; i > 0; i--) {
      s2_vector_copy(&cB->extremeVertexLocal[i - 1],
                     &cB->extremeVertexLocal[i]);
    }

    /**
    Use negativeSearchDirectionVector = -self->searchDirectionVector in
    s2_collisionobject_support for objectA.
    */
    s2_vector_t negativeSearchDirectionVector;
    s2_vector_invert(&self->searchDirectionVector,
                     &negativeSearchDirectionVector);

    s2_collisionobject_support(cA, context, &negativeSearchDirectionVector,
                               &cA->extremeVertexLocal[0],
                               &extremeVertexAbsoluteA[0]);

    /**
    Use self->searchDirectionVector in s2_collisionobject_support for
    objectB.
    */
    s2_collisionobject_support(cB, context, &self->searchDirectionVector,
                               &cB->extremeVertexLocal[0],
                               &extremeVertexAbsoluteB[0]);

    /**
    At this point the simplexVectorsAbsolute array contains coordinates of
    numberSimplexPoints simplex support vectors, where the current value of
    numberSimplexPoints is less than 4. The simplex vector coordinates in
    simplexVectorsAbsolute must be shifted down by one to make room for the next
    simplex support vector inserted into the first vector position.
    */

    for(int i = self->simplexPointCount; i > 0; i--) {
      s2_vector_copy(&simplexVectorsAbsolute[i - 1],
                     &simplexVectorsAbsolute[i]);
    }

    /**
    Subtract the support point on hull A from the support point on hull B
    and insert the new point into the first vector position of the simplex.
    */

    s2_subtract_vv(&extremeVertexAbsoluteB[0], &extremeVertexAbsoluteA[0],
                   &simplexVectorsAbsolute[0]);
    self->simplexPointCount++;
  }

  /**
  In trouble if no success. Bail out!
  */
  if(!self->success) {
    return (-1);
  }
  return (self->collision);
}

void simplex0(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13], bool* collision,
              bool* success) {
  /**
  The s2_simplex_gjk support function uses searchDirectionVector to locate the
  next extreme vertex point P relative to the origin point O, which is
  represented by the vector OP. simplexVectorsAbsolute[0] contains the vector
  OA, which locates the simplex vertex point A relative to the origin point O.
  The next searchDirectionVector is set to -OA.

  *success is false on entry to simplex0. When the convex hull does not
  contain the origin, *collision = false and *success = true, and when the
  convex hull contains the origin, *collision = true and *success = true. If no
  decision is made, nothing is changed.
  */
  s2_vector_invert(&simplexVectorsAbsolute[0], searchDirectionVector);
  double norm = s2_vector_l2norm(searchDirectionVector);

  if(norm < EPS) {
    *collision = true;
    *success = true;
  }

  if(*success) {
    printf("collision = %s\n", *collision ? "true" : "false");
  }
  return;
}

/**
simplex1() takes in two points from the Minkowski difference hull, an old
point B and a new point A. The two points form a line segment AB, which is a
convex Minkowski difference hull. If the origin lies on this line, it is in
the convex hull, and objectA and objectB intersect. simplex1() then reports this
and the GJK algorithm terminates. Otherwise a simplex2 triangle is needed.
Before returning, simplex1() generates a search direction vector orthogonal
to the line, in the plane containing the origin, and in the direction of the
origin. s2_collisionobject_support() then uses this direction vector to locate a
third point on the convex hull to form a triangle simplex2 convex hull.
*/
void simplex1(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13], bool* collision,
              bool* success) {

  /**
  The simplex1 line consists of points A and B and vector AB = B - A
  representing a line segment connecting points A and B. Let OA represent
  the vector from the origin to point A. Let searchDirectionVector = (OA X AB) X
  AB represent the direction orthogonal to the line and lying in a plane
  containing the origin. Drawing lines parallel to searchDirectionVector through
  points A and B divides the plane into three regions. Before entering
  simplex1() we have verified that the origin is not in the region off the end
  of the line near point B. Thus we have to check two other regions. First, if
  the magnitude of searchDirectionVector is near EPS the origin lies on the
  line and is in the convex hull. Thus the convex hulls A and B intersect.
  Otherwise we must expand the simplex with another point by searching in the
  direction of searchDirectionVector.

  s2_simplex_gjk will use searchDirectionVector to locate the next extreme
  vertex P relative to origin O, represented by OP. simplexVectorsAbsolute[0]
  contains vector OA, which locates vertex A relative to O.
  simplexVectorsAbsolute[1] contains vector OB, which locates vertex B relative
  to O. simplexVectorsAbsolute[2] contains vector AB = OB - OA, which locates B
  relative to A. simplexVectorsAbsolute[3] contains OA X AB and
  searchDirectionVector = (OA X AB) X AB.

  *success is false on entry to simplex1. When the convex hull does not
  contain the origin, *collision = false and *success = true, and when the
  convex hull contains the origin, *collision = true and *success = true. If no
  decision is made, nothing is changed.
  */

  if(s2_dot_vv(&simplexVectorsAbsolute[0], searchDirectionVector) < -EPS) {
    *success = true;
    *collision = false;
    printf("collision = %s\n", *collision ? "true" : "false");
    return;
  }

  s2_subtract_vv(&simplexVectorsAbsolute[1], &simplexVectorsAbsolute[0],
                 &simplexVectorsAbsolute[2]); // AB = OB - OA
  s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[2],
              &simplexVectorsAbsolute[3]); // OA X AB
  s2_cross_vv(&simplexVectorsAbsolute[3], &simplexVectorsAbsolute[2],
              searchDirectionVector); // (OA X AB) X AB

  double norm = s2_vector_l2norm(searchDirectionVector);
  if(norm < EPS) {
    *success = true;
    *collision = true;
  }
  if(*success) {
    printf("collision = %s\n", *collision ? "true" : "false");
  }
  return;
}

/**
simplex2() takes in three points from the Minkowski difference hull, two old
points B and C and a new point A. The three points form a triangle TABC, which
is a convex Minkowski difference hull. If the origin lies on this triangle, it
is in the convex hull and the two objects intersect. simplex2() then reports
this and the GJK algorithm terminates. Otherwise the GJK algorithm searches for
a new point in the direction of the origin. If it finds one, it throws out the
oldest point and simplex2() will be called again. Or it may determine that a
simplex3 tetrahedron is required. Before returning, simplex2() either generates
a searchDirectionVector orthogonal to a line or to the plane of a triangle and
in the direction of the origin. s2_collisionobject_support() then uses
searchDirectionVector to locate a third or fourth point on the convex hull to
form a new triangle simplex2 or tetrahedron simplex3 convex hull.
*/

void simplex2(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success) {

  /**
  The simplex2 triangle consists of points A, B, and C. Assume that the
  points are arranged counter-clockwise to form a triangle TABC with normal
  vector ABCn = AB X AC pointing upward or outward or frontward as we may
  interpret that direction. If we program simplex2() to always do this and
  we want it to do just the reverse, simply reverse the point order to
  C, B, and A. This is equivalent to flipping the triangle about point B.

  Next let ACn = ABCn X AC and ABn = AB X ABCn represent outward-facing
  vectors, normal to the respective edges or lines AC and AB and lying
  in the TABC plane. Outward facing means they point away from the center
  of the triangle. Let OA represent the vector from the origin to point A.
  Then ABn•OA < 0 indicates that the origin is closer to the edge AB and
  outside the triangle. We set the search direction to (OA X AB) X AB to
  be perpendicular to edge AB and lying in the plane containing the origin.
  Then we use simplex2() to determine where the origin lies relative to
  this edge.

  If this test fails we then test ACn•OA < 0 to see if the origin is closer to
  the edge AC and outside the triangle. In this case we set the search direction
  to (OA X AC) X AC to be perpendicular to edge AC and lying in the plane
  containing the origin. Then we use simplex2() to determine where the origin
  lies relative to this edge.

  If this test fails, the origin then lies above or below or on the triangle.
  Let v = OA•ABCn. If v is close enough to EPS, the origin lies in the plane of
  the triangle; the convex hull contains the origin and the convex hulls objectA
  and objectB intersect. If v < 0 the origin is above the triangle and the next
  search direction is ABCn. And if v > 0 the origin is below the triangle. Now
  we need to search the same regions on this side. To do that we set the search
  direction to -ABCn and flip the triangle over around the point A to use the
  same simplex2() function. Thus we send C, A, B to simplex2() instead of A, B,
  C.

  s2_collisionobject_support() uses searchDirectionVector to locate the next
  extreme vertex P relative to origin O, represented by OP.
  simplexVectorsAbsolute[0] contains vector OA, which locates vertex A relative
  to O. simplexVectorsAbsolute[1] contains vector OB, which locates vertex B
  relative to O. simplexVectorsAbsolute[2] contains vector OC, which locates
  vertex C relative to O. simplexVectorsAbsolute[3] contains vector AB = OB -
  OA, which locates B relative to A. simplexVectorsAbsolute[4] contains vector
  AC = OC - OA, which locates C relative to A. simplexVectorsAbsolute[5]
  contains vector ABCn = AB X AC, normal to TABC. simplexVectorsAbsolute[6]
  contains vector ACn = ABCn X AC, tangent to TABC, normal to AC.
  simplexVectorsAbsolute[7] contains vector ABn = AB X ABCn, tangent to TABC,
  normal to AB. simplexVectorsAbsolute[8] contains vector OA X AB. and
  searchDirectionVector = (OA X AB) X AB

  Some tests require first checking for the origin on one side of a triangle and
  then on the other side. A procedure based on counterclockwise rotations was
  developed to check one side of the triangle. To avoid confusion and to use the
  same procedures to check the opposite side of the triangle, selected vectors
  in the simplex are swapped. This swapping of vectors has physically placed
  them in a different order in the simplex vector array. At the same time the
  corresponding vectors in extremeVertexLocalA and extremeVertexLocalB, from
  which the simplex vectors are recovered, must be swapped to keep them aligned
  with the simplex vectors.

  *success is false on entry to simplex2. When the convex hull does not
  contain the origin, *collision = false and *success = true, and when the
  convex hull contains the origin, *collision = true and *success = true. If no
  decision is made, nothing is changed.
  */

  if(s2_dot_vv(&simplexVectorsAbsolute[0], searchDirectionVector) < -EPS) {
    *success = true;
    *collision = false;
    printf("collision = %s\n", *collision ? "true" : "false");
    return;
  }

  s2_subtract_vv(&simplexVectorsAbsolute[1], &simplexVectorsAbsolute[0],
                 &simplexVectorsAbsolute[3]); // AB = OB - OA
  s2_subtract_vv(&simplexVectorsAbsolute[2], &simplexVectorsAbsolute[0],
                 &simplexVectorsAbsolute[4]); // AC = OC - OA
  s2_cross_vv(&simplexVectorsAbsolute[3], &simplexVectorsAbsolute[4],
              &simplexVectorsAbsolute[5]); // ABCn = AB X AC
  s2_cross_vv(&simplexVectorsAbsolute[5], &simplexVectorsAbsolute[4],
              &simplexVectorsAbsolute[6]); // ACn = ABCn X AC
  s2_cross_vv(&simplexVectorsAbsolute[3], &simplexVectorsAbsolute[5],
              &simplexVectorsAbsolute[7]); // ABn = AB X ABCn

  // if(OA•ABn < -EPS)
  if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[7]) < -EPS) {
    s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[3],
                &simplexVectorsAbsolute[8]); // OA X AB
    s2_cross_vv(&simplexVectorsAbsolute[8], &simplexVectorsAbsolute[3],
                searchDirectionVector); // (OA X AB) X AB
    (*numberSimplexPoints)--;
    return;
    // simplex1: [OA, OB]
  }
  // if(OA•ACn < -EPS)
  else if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[6]) <
          -EPS) {
    s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[4],
                &simplexVectorsAbsolute[8]); // OA X AC
    s2_cross_vv(&simplexVectorsAbsolute[8], &simplexVectorsAbsolute[4],
                searchDirectionVector); // (OA X AC) X AC
    s2_vector_swap(&simplexVectorsAbsolute[1],
                   &simplexVectorsAbsolute[2]); // swap OB and OC
    s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
    s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
    (*numberSimplexPoints)--;
    return;
    // swap results simplex1: [OA, OC]
  } else {
    double v = -s2_dot_vv(&simplexVectorsAbsolute[0],
                          &simplexVectorsAbsolute[5]); // -OA•ABCn
    if(v > EPS) { // Set search direction to ABCn
      s2_vector_copy(&simplexVectorsAbsolute[5], searchDirectionVector);
      return;
    } else if(v < -EPS) { // Set search direction to -ABCn
      s2_vector_invert(&simplexVectorsAbsolute[5], searchDirectionVector);
      s2_vector_swap(&simplexVectorsAbsolute[0],
                     &simplexVectorsAbsolute[1]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[0], &extremeVertexLocalA[1]);
      s2_vector_swap(&extremeVertexLocalB[0], &extremeVertexLocalB[1]);
      return;
    } else {
      *success = true;
      *collision = true;
      printf("collision = true\n");
      return;
    }
  }
}

/**
simplex3() takes in four points from the Minkowski difference hull, three old
points B, C and D and a new point A. The four points form a tetrahedron, TABCD,
which is a convex Minkowski difference hull. If the origin lies on the surface
of TABCD or inside it, the two convex hulls intersect. simplex3()
then reports this and the GJK algorithm terminates. Otherwise the GJK
algorithm searches for a new point in the direction of the origin. If it
finds one, it throws out the oldest point and sometimes the oldest two points
and simplex3() or simplex2() will be called again. If the GJK algorithm finds no
more points it reports no collision and terminates.
*/
void simplex3(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success) {
  /**
  The simplex3 tetrahedron TABCD consists of points A, B, C, and D. Assume that
  points B, C, and D form the tehrahedron's base triangle TBCD and are
  arranged counter-clockwise in that order. Assume that point A lies above
  TBCD and that vector BCDn = BC X BD points upward in the direction of A.
  Vector BCDn is normal to TBCD. The other three faces of the tetrahedron
  are represented by triangles TABC, TACD, and TADB. The outward-facing
  normal vectors to these faces are ABCn = AB X AC, ACDn = AC X AD, and
  ADBn = AD X AB. Let OA represent the vector from origin point O to point A.

  We want to determine if the origin lies inside or outside the tetrahedron.
  If it lies on or in the tetrahedron, the convex hulls objectA and objectB are
  in contact or intersecting. If the origin lies outside the tetrahedron,
  another point will be added to the tegrahedron, if one exists, and the
  oldest point will be thrown away. Then simplex3() will be called again.

  The tetrahedron has four faces represented by the triangles TBCD, TABC,
  TACD, and TADB. A previous simplex2() will have verified that the origin does
  not lie below TBCD, so we need to check that the origin does not lie outside
  the remaining triangle faces TABC, TACD, and TADB.

  Start with triangle TABC with outward-facing normal ABCn. If OA•ABCn < 0
  the origin lies somewhere outside the tetrahedron on its TABC side. It may
  lie closer to the AC edge or the AB edge of the triangular face, but it
  won't lie closer to the BC edge because an earlier simplex1() test verified
  that. The vectors ACn = ABCn X AC and ABn = AB X ABCn point outward from the
  TABC edges AC and AB, and lie in its plane. Thus if OA•ACn < 0 we use a
  simplex1() test on the AC edge with a search direction (OA X AC) X AC.
  Or if OA•ABn < 0 we use a simplex1() test on the AB edge with a search
  direction (OA X AB) X AB. If the origin is not detected on the outside
  face of TABC we flip the triangle and apply a simplex2() test to it. The
  search direction is still ABCn because the triangle's vertex order was
  changed from TABC to TCBA.

  A similar set of tests is applied to triangles TACD and TADB. If all of
  these tests fail to find the origin outside the tetrahedron, then the
  origin is inside the tetrahedron and the convex hulls objectA and objectB
  intersect.

  searchDirectionVector contains the direction vector used by
  s2_collisionobject_support() to locate the next extreme vertex P relative to
  origin O, represented by OP. simplexVectorsAbsolute[0] contains vector OA,
  which locates vertex A relative to O. simplexVectorsAbsolute[1] contains
  vector OB, which locates vertex B relative to O. simplexVectorsAbsolute[2]
  contains vector OC, which locates vertex C relative to O.
  simplexVectorsAbsolute[3] contains vector OD, which locates vertex D relative
  to O. simplexVectorsAbsolute[4] contains vector AB = OB - OA, which locates B
  relative to A. simplexVectorsAbsolute[5] contains vector AC = OC - OA, which
  locates C relative to A. simplexVectorsAbsolute[6] contains vector AD = OD -
  OA, which locates D relative to A. simplexVectorsAbsolute[7] contains vector
  ABCn = AB X AC, normal to TABC. simplexVectorsAbsolute[8] contains vector ACDn
  = AC X AD, normal to TACD. simplexVectorsAbsolute[9] contains vector ADBn = AD
  X AB, normal to TADB.

  For the triangle ABC
  simplexVectorsAbsolute[10] contains vector ACn = ABCn X AC, tangent to TABC,
  normal to AC. simplexVectorsAbsolute[11] contains vector ABn = AB X ABCn,
  tangent to TABC, normal to AB. simplexVectorsAbsolute[12] contains vector OA X
  AC. searchDirectionVector = (OA X AC) X AC simplexVectorsAbsolute[12] contains
  vector OA X AB. searchDirectionVector = (OA X AB) X AB

  For the triangle ACD
  simplexVectorsAbsolute[10] contains vector ADn = ACDn X AD, tangent to TACD,
  normal to AD. simplexVectorsAbsolute[11] contains vector ACn = AC X ACDn,
  tangent to TACD, normal to AC. simplexVectorsAbsolute[12] contains vector OA X
  AD. searchDirectionVector = (OA X AD) X AD simplexVectorsAbsolute[12] contains
  vector OA X AC. searchDirectionVector = (OA X AC) X AC

  For the triangle ABD
  simplexVectorsAbsolute[10] contains vector ABn = ADBn X AB, tangent to TADB,
  normal to AB. simplexVectorsAbsolute[11] contains vector ADn = AD X ADBn,
  tangent to TADB, normal to AD. simplexVectorsAbsolute[12] contains vector OA X
  AB. searchDirectionVector = (OA X AB) X AB simplexVectorsAbsolute[12] contains
  vector OA X AD. searchDirectionVector = (OA X AD) X AD

  Some tests require first checking for the origin on one side of a triangle and
  then on the other side. A procedure based on counterclockwise rotations was
  developed to check one side of the triangle. To avoid confusion and to use the
  same procedures to check the opposite side of the triangle, selected vectors
  in the simplex are swapped. This swapping of vectors has physically placed
  them in a different order in the simplex vector array. At the same time the
  corresponding vectors in extremeVertexLocalA and extremeVertexLocalB, from
  which the simplex vectors are recovered, must be swapped to keep them aligned
  with the simplex vectors.

  *success is false on entry to simplex3. When the convex hull does not
  contain the origin, *collision = false and *success = true, and when the
  convex hull contains the origin, *collision = true and *success = true. If no
  decision is made, nothing is changed.
  */

  if(s2_dot_vv(&simplexVectorsAbsolute[0], searchDirectionVector) < -EPS) {
    *success = true;
    *collision = false;
    printf("collision = %s\n", *collision ? "true" : "false");
    return;
  }

  s2_subtract_vv(&simplexVectorsAbsolute[1], &simplexVectorsAbsolute[0],
                 &simplexVectorsAbsolute[4]); // AB = OB - OA
  s2_subtract_vv(&simplexVectorsAbsolute[2], &simplexVectorsAbsolute[0],
                 &simplexVectorsAbsolute[5]); // AC = OC - OA
  s2_subtract_vv(&simplexVectorsAbsolute[3], &simplexVectorsAbsolute[0],
                 &simplexVectorsAbsolute[6]); // AD = OD - OA
  s2_cross_vv(&simplexVectorsAbsolute[4], &simplexVectorsAbsolute[5],
              &simplexVectorsAbsolute[7]); // ABCn = AB X AC
  s2_cross_vv(&simplexVectorsAbsolute[5], &simplexVectorsAbsolute[6],
              &simplexVectorsAbsolute[8]); // ACDn = AC X AD
  s2_cross_vv(&simplexVectorsAbsolute[6], &simplexVectorsAbsolute[4],
              &simplexVectorsAbsolute[9]); // ADBn = AD X AB

  // if(OA•ABCn < -EPS) O outside face of TABC & tetrahedron
  if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[7]) < -EPS) {
    s2_cross_vv(&simplexVectorsAbsolute[7], &simplexVectorsAbsolute[5],
                &simplexVectorsAbsolute[10]); // ACn = ABCn X AC
    s2_cross_vv(&simplexVectorsAbsolute[4], &simplexVectorsAbsolute[7],
                &simplexVectorsAbsolute[11]); // ABn = AB X ABCn
    if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[10]) <
       -EPS) { // OA•ACn O outside edge AC in plane of TABC
      s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[5],
                  &simplexVectorsAbsolute[12]); // OA X AC
      s2_cross_vv(&simplexVectorsAbsolute[12], &simplexVectorsAbsolute[5],
                  searchDirectionVector); //(OA X AC) X AC
      s2_vector_swap(&simplexVectorsAbsolute[1], &simplexVectorsAbsolute[2]);
      // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
      // simplex1: [OA, OC]
      (*numberSimplexPoints) -= 2;
      return;
    } else if(s2_dot_vv(&simplexVectorsAbsolute[0],
                        &simplexVectorsAbsolute[11]) <
              -EPS) { // OA•ABn O outside edge AB in plane of TABC
      s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[4],
                  &simplexVectorsAbsolute[12]); // OA X AB
      s2_cross_vv(&simplexVectorsAbsolute[12], &simplexVectorsAbsolute[4],
                  searchDirectionVector); //(OA X AB) X AB
      // simplex1: [OA, OB]
      (*numberSimplexPoints) -= 2;
      return;
    } else { // ABCn above TABC
      s2_vector_copy(&simplexVectorsAbsolute[7], searchDirectionVector);
      // simplex2: [OA, OB, OC]
      (*numberSimplexPoints)--;
      return;
    }
  }
  // if(OA•ACDn < -EPS) O outside face of TACD & tetrahedron
  else if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[8]) <
          -EPS) {
    s2_cross_vv(&simplexVectorsAbsolute[8], &simplexVectorsAbsolute[6],
                &simplexVectorsAbsolute[10]); // ADn = ACDn X AD
    s2_cross_vv(&simplexVectorsAbsolute[5], &simplexVectorsAbsolute[8],
                &simplexVectorsAbsolute[11]); // ACn = AC X ACDn
    if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[10]) <
       -EPS) { // OA•ADn O outside edge AD in plane of TACD
      s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[6],
                  &simplexVectorsAbsolute[12]); // OA X AD
      s2_cross_vv(&simplexVectorsAbsolute[12], &simplexVectorsAbsolute[6],
                  searchDirectionVector); //(OA X AD) X AD
      s2_vector_swap(&simplexVectorsAbsolute[1],
                     &simplexVectorsAbsolute[3]); // swap OB and OD
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[3]);
      // simplex1: [OA, OD]
      (*numberSimplexPoints) -= 2;
      return;
    } else if(s2_dot_vv(&simplexVectorsAbsolute[0],
                        &simplexVectorsAbsolute[11]) <
              -EPS) { // OA•ACn O outside edge AC in plane of TACD
      s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[5],
                  &simplexVectorsAbsolute[12]); // OA X AC
      s2_cross_vv(&simplexVectorsAbsolute[12], &simplexVectorsAbsolute[5],
                  searchDirectionVector); //(OA X AC) X AC
      s2_vector_swap(&simplexVectorsAbsolute[1],
                     &simplexVectorsAbsolute[2]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
      // simplex1: [OA, OC]
      (*numberSimplexPoints) -= 2;
      return;
    } else { // ACDn Face of TACD toward O
      s2_vector_copy(&simplexVectorsAbsolute[8], searchDirectionVector);
      s2_vector_swap(&simplexVectorsAbsolute[1],
                     &simplexVectorsAbsolute[2]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
      s2_vector_swap(&simplexVectorsAbsolute[2],
                     &simplexVectorsAbsolute[3]); // swap OC and OD
      s2_vector_swap(&extremeVertexLocalA[2], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[2], &extremeVertexLocalB[3]);
      // simplex2: [OA, OC, OD]
      (*numberSimplexPoints)--;
      return;
    }
  }
  // if(OA•ADBn < -EPS) O outside face of TADB & tetrahedron
  else if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[9]) <
          -EPS) {
    s2_cross_vv(&simplexVectorsAbsolute[9], &simplexVectorsAbsolute[4],
                &simplexVectorsAbsolute[10]); // ABn = ADBn X AB
    s2_cross_vv(&simplexVectorsAbsolute[6], &simplexVectorsAbsolute[9],
                &simplexVectorsAbsolute[11]); // ADn = AD X ADBn
    if(s2_dot_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[10]) <
       -EPS) { // OA•ABn O outside edge AB in plane of TADB
      s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[4],
                  &simplexVectorsAbsolute[12]); // OA X AB
      s2_cross_vv(&simplexVectorsAbsolute[12], &simplexVectorsAbsolute[4],
                  searchDirectionVector); //(OA X AB) X AB
      // simplex1: [OA, OB]
      (*numberSimplexPoints) -= 2;
      return;
    } else if(s2_dot_vv(&simplexVectorsAbsolute[0],
                        &simplexVectorsAbsolute[11]) <
              -EPS) { // OA•ADn O outside edge AD in plane of TADB
      s2_cross_vv(&simplexVectorsAbsolute[0], &simplexVectorsAbsolute[6],
                  &simplexVectorsAbsolute[12]); // OA X AD
      s2_cross_vv(&simplexVectorsAbsolute[12], &simplexVectorsAbsolute[6],
                  searchDirectionVector); //(OA X AD) X AD
      s2_vector_swap(&simplexVectorsAbsolute[1],
                     &simplexVectorsAbsolute[3]); // swap OB and OD
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[3]);
      // simplex1: [OA, OD]
      (*numberSimplexPoints) -= 2;
      return;
    } else { // ADBn Face of TADB toward O
      s2_vector_copy(&simplexVectorsAbsolute[9], searchDirectionVector);
      s2_vector_swap(&simplexVectorsAbsolute[1],
                     &simplexVectorsAbsolute[3]); // swap OB and OD
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[3]);
      s2_vector_swap(&simplexVectorsAbsolute[2],
                     &simplexVectorsAbsolute[3]); // swap OC and OD
      s2_vector_swap(&extremeVertexLocalA[2], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[2], &extremeVertexLocalB[3]);
      // simplex2: [OA, OD, OB]
      (*numberSimplexPoints)--;
      return;
    }
  } else {
    *success = true;
    *collision = true;
    if(*success) {
      printf("collision = true\n");
    }
    return;
  }
}

/*!
@brief support mapping function for two sphere collision objects
@param [in] a first s2_collisionobject_t containing an s2_sphere_t object
@param [in] a second s2_collisionobject_t containing an s2_sphere_t object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_sphereSphere(s2_context_t* context, s2_simplex_t* self) {

  s2_collisionobject_t* c1 = self->objectA;
  s2_collisionobject_t* c2 = self->objectB;

  s2_sphere_t* s1 = &context->spheres[c1->objectIndex];
  s2_sphere_t* s2 = &context->spheres[c2->objectIndex];

  if(!c1->displacementUpdated) {
    s2_product_rv(&c1->referenceFrame->R, &c1->dWRTReference.T,
                  &c1->dAbsolute.T);
    s2_addequal_vv(&c1->referenceFrame->T, &c1->dAbsolute.T);
    c1->displacementUpdated = true;
  }

  if(!c2->displacementUpdated) {
    s2_product_rv(&c2->referenceFrame->R, &c2->dWRTReference.T,
                  &c2->dAbsolute.T);
    s2_addequal_vv(&c2->referenceFrame->T, &c2->dAbsolute.T);
    c2->displacementUpdated = true;
  }

  double d = s1->radius + s2->radius;
  s2_vector_t t = {0.0, 0.0, 0.0};
  s2_subtract_vv(&c1->dAbsolute.T, &c2->dAbsolute.T, &t);

  if((t.x * t.x + t.y * t.y + t.z * t.z) < (d * d)) {
    return true;
  } else {
    return false;
  }
}

/*!
@brief support mapping function for two ellipsoid collision objects
@param [in] a first s2_collisionobject_t containing an s2_ellipsoid_t
object
@param [in] a second s2_collisionobject_t containing an s2_ellipsoid_t
object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_ellipsoidEllipsoid(s2_context_t* context, s2_simplex_t* self) {

  /* use (void) to suppress unused variable compiler warning. Remove this
  once correct support function is loaded */
  (void)context;
  s2_collisionobject_t* c1 = self->objectA;
  s2_collisionobject_t* c2 = self->objectB;
  (void)c1;
  (void)c2;

  /*! @TODO add call to ellipsoid-ellipsoid test

  access extreme vertex support mapping functions for c1 and c2 by
  calling:

  s2_vector_t searchDirection;

  s2_vector_t extremeVertex1 =
    s2_collisionobject_support(c1, searchDirection);

  s2_vector_t extremeVertex2 =
    s2_collisionobject_support(c2, searchDirection);

  */
  return true;
}

/*!
@brief test for collision between two primitive collision objects. This
function uses function pointer stored in s2_simplex_t to test for
collision between two collisionobjects

@param self [in] a pointer to a s2_simplex_t datastructure, which contains
information about a pair of s2_collisionobject_t datastructures and their
time histories
*/
void s2_simplex_support(s2_context_t* context, s2_simplex_t* self) {
  self->collision = self->support(context, self);
}

int s2_simplex_hashIndex(int object_a, int object_b) {
  int i = fmin(object_a, object_b);
  int j = fmax(object_a, object_b);
  return (j - 1) * j / 2 + i;
}

/*!
@brief detect if simplex has already been appended to array of linked lists
*/
bool s2_simplex_exists(s2_context_t* context, s2_collisionobject_t* object_a,
                       s2_collisionobject_t* object_b) {

  if(object_a->index == object_b->index) {
    return false;
  } else {
    int index = s2_simplex_hashIndex(object_a->index, object_b->index);
    return s2_bitfield_get(context->simplexExistsBitfield, index);
  }
}

/*!
@brief initializes a new s2_simplex_t datastructure and sets the
appropriate support mapping function for sampling the minkowski
difference. This function will be called from contact.c
*/
s2_simplex_t* s2_simplex_alloc(s2_collisionobject_t* object_a,
                               s2_collisionobject_t* object_b) {
  s2_simplex_t* simplex = calloc(1, sizeof(s2_simplex_t));
  simplex->objectA = object_a;
  simplex->objectB = object_b;
  simplex->restoreSupport = s2_simplex_restoreSupport;
  if(object_a->type == S2_COLLISIONOBJECT_SPHERE &&
     object_b->type == S2_COLLISIONOBJECT_SPHERE) {
    simplex->support = s2_simplex_sphereSphere;
    // } else if(object_a->type == S2_COLLISIONOBJECT_ELLIPSOID &&
    //           object_b->type == S2_COLLISIONOBJECT_ELLIPSOID) {
    //   simplex->support = s2_simplex_ellipsoidEllipsoid;
  } else {
    simplex->support = s2_simplex_gjk;
  }
  /*! @TODO add other level 2 support mapping tests as appropriate */

  /*! @TODO add quantities related to time history and initialize them
  here, they will have been initialized to zero in the call to calloc
  above */
  simplex->prev = NULL;
  simplex->next = NULL;

  return simplex;
}

/*!
@brief free memory related to an s2_simplex_t data structure. If other
quantities are malloc'ed to account for time history, they should also be
freed here.
*/
void s2_simplex_free(s2_simplex_t* simplex) {
  free(simplex);
  simplex = NULL;
}
