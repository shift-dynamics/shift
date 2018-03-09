#include <stdlib.h>

#include "collisionobject.h"
#include "simplex.h"

void s2_restore_ellipsoid_support(s2_vector_t extremeVertexGlobal[4],
                                  s2_displacement_t* dAbsolute,
                                  s2_vector_t extremeVertexLocal[4],
                                  int simplexPointCount) {
  printf("\n+++++local extreme vertex support matrix+++++\n");
  for(int i = 0; i < simplexPointCount; i++) {
    s2_vector_print(stdout, &extremeVertexLocal[i]);
    s2_product_dv(dAbsolute, &extremeVertexLocal[i], &extremeVertexGlobal[i]);
  }
}

/*!
@brief support mapping function for two convexhull collision objects
@param [in] a context data structure containing model definition information
@param [in] a simplex data structure containing information about two
collisionobjects
@return whether the two objects are in contact with one another
*/
bool s2_simplex_gjk(s2_context_t* context, s2_simplex_t* self) {
  /* use (void) to suppress unused variable compiler warning. Remove this once
                                        correct support function is loaded */
  (void)context;
  s2_collisionobject_t* cA = self->objectA;
  s2_collisionobject_t* cB = self->objectB;

  /*
  This function carries out a full GJK contact detection iteration on each
  pair of convex hulls. Before starting the current iteration cycle it must
  first compute the absolute spatial displacement of the hull A/B reference
  frames. For each convex hull pair, on the very first call to this function
  when numberSimplexPoints = 0, it must compute global coordinates of the
  searchDirectionVector. From then on when numberSimplexPoints > 0, the
  searchDirectionVector coordinates will be obtained from the most recent
  simplex. Each GJK iteration cycle starts by sending the most recent
  searchDirectionVector to the convex hull A/B support functions to obtain
  absolute extreme points A/B on the respective surfaces. The simplex support
  function subtracts coordinates of point A from B to obtain the next support
  point on the convex Minkowski hull, which is also the next point to be
  inserted into the first position of the simplex. To make room for this
  point, existing points are shifted down one position. The simplex is then
  analyzed to determine whether or not the two objects are in contact. If a
  determination can be made, a flag is set to 1 (contact) or 0 (no contact.)
  If a determination can't be made a new searchDirectionVector is generated
  from the simplex, and zero, one, or two old support points are eliminated.
  Removing support points reduces the simplex dimension and
  numberSimplexPoints, and inserting a support point increases the simplex
  dimension and numberSimplexPoints.

  Each convex hull support function has access to a struct containing the
  hull's definition data, but no historic or state information is stored in
  this struct. Many of these structs may contain a lot of data, so only one
  instance of each struct is used multiple times when possible. However each
  call to a simplex support function for a convex hull A/B pair is supplied
  with a unique instantiated struct that stores mostly historic and state
  information. All inquires and computations related to the individual A and B
  hulls are performed in their respective support functions, but the related
  historic and state information is stored in the simplex support struct,
  which is passed to the hull support function through its call list. The
  simplex support function stores additional historic and state information in
  its support struct.
  */

  s2_vector_t EVA[4]; // Absolute coordinates of extreme points on hull A
  s2_vector_t EVB[4]; // Absolute coordinates of extreme points on hull B
  s2_vector_t VW[13]; // Absolute coordinates of intermediate simplex vectors

  /*
  Absolute direction cosine matrix and position vector of convex hull A and B
  reference frames.
  */
  // double spatialDisplacementHullA[4][3];
  // double spatialDisplacementHullB[4][3];

  /*
  origin = -1 until a surface contact decision has been made
  origin =  0 when the surfaces are not in contact
  origin =  1 when the surfaces are touching or penetrating
  The GJK iteration terminates when origin = 0 or 1, or when the maximum
  number of GJK iterations has been reached. SimplexSupportFunction() will
  return with origin set to 0 or 1 on successful contact decision and with
  origin set to –1 on unsuccessful contact decision.
  */
  self->success = false;

  printf("\n~~~~~In s2_simplex_gjk support function~~~~~\n");

  /*
  On each SimplexSupportFunction call the kinematics or dynamics states may
  have changed. Thus the body spatial displacements may have changed and the
  spatial displacements of convex hulls A and B may have changed. The arrays
  spatialDisplacementHullA and spatialDisplacementHullB contain the spatial
  displacement of the convex hull A and B reference frames.
  */

  s2_displacement_t* dAbsoluteA;
  dAbsoluteA = &cA->dAbsolute;
  if(!cA->displacementUpdated) {
    cA->displacementUpdated = true;
    s2_product_dd(cA->referenceFrame, &cA->dWRTReference, dAbsoluteA);
    printf("Absolute spatial displacement of hull A reference frame:\n");
    printf("Direction cosine matrix and origin point\n");
    s2_displacement_print(stdout, dAbsoluteA);
  }

  s2_displacement_t* dAbsoluteB;
  dAbsoluteB = &cB->dAbsolute;
  if(!cB->displacementUpdated) {
    cB->displacementUpdated = true;
    s2_product_dd(cB->referenceFrame, &cB->dWRTReference, dAbsoluteB);
    printf("Absolute spatial displacement of hull B reference frame:\n");
    printf("Direction cosine matrix and origin point\n");
    s2_displacement_print(stdout, dAbsoluteB);
  }

  /*
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
    s2_subtract_vv(&dAbsoluteA->T, &dAbsoluteB->T,
                   &self->searchDirectionVector);
    printf("simplexPointCount = %d\n", self->simplexPointCount);
    printf("Initialize searchDirectionVector\n");
    s2_vector_print(stdout, &self->searchDirectionVector);

    /* Use sDV = -self->searchDirectionVector in ConvexHullSupportFunctionA()
     */
    s2_vector_t sDV;
    s2_vector_invert(&self->searchDirectionVector, &sDV);
    /* Local and absolute coordinates of extreme vertex on convex hull A. */
    printf("\n***** Entering ConvexHullSupportFunctionA *****\n");
    cA->support(self, cA, &sDV, &cA->extremeVertex[0], &EVA[0]);
    printf("***** Leaving ConvexHullSupportFunctionA *****\n");

    /* Use self->searchDirectionVector in ConvexHullSupportFunctionB()
       Local and absolute coordinates of extreme vertex on convex hull B. */
    printf("\n***** Entering ConvexHullSupportFunctionB *****\n");
    cB->support(self, cB, &sDV, &cB->extremeVertex[0], &EVB[0]);
    printf("***** Leaving ConvexHullSupportFunctionB *****\n");
    s2_subtract_vv(&EVB[0], &EVA[0], &VW[0]);
  }
  /*
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
  EVA and EVB arrays. Coordinates in EVA are subtracted from those in EVB to
  obtain the simplex support vectors stored in VW.

  */
  else {
    printf("\n*****Restoring simplex***** with %d vectors.\n",
           self->simplexPointCount);
    s2_restore_ellipsoid_support(EVA, dAbsoluteA, &cA->extremeVertex,
                                 self->simplexPointCount);
    s2_restore_ellipsoid_support(EVB, dAbsoluteB, &cB->extremeVertex,
                                 self->simplexPointCount);
    for(int i = 0; i < self->simplexPointCount; i++) {
      s2_subtract_vv(&EVB[i], &EVA[i], &VW[i]);
    }
  }

  /*
  Iterate until success or failure.
  return 1 => origin on or in convex hull and contact.
  return 0 => origin not in convex hull and no contact.
  return -1 => failure and no determination.
  numberSimplexPoints can have a value of 1, 2, 3, or 4 in the following GJK
  while() loop. During a normal step that does not terminate the iteration,
  a particular simplex0, simplex1, simplex2, or simplex3 function generates
  a search-direction vector and stores it in searchDirectionVector.
  If a simplex function makes a contact decision, it sets origin to 0 or 1.
  A test for origin > –1 will break out of the GJK while loop and return
  control to the SimplexSupportFunction to prepare for exit. If a particular
  simplex support function cannot make a contact decision, it computes
  coordinates of a new search-direction vector and returns them in
  searchDirectionVector. In addition simplex2 may reduce numberSimplexPoints
  by 1 and simplex3 may reduce searchDirectionVector by 1 or 2.

  A hard limit of 50 iterations was set on the GJK while loop. For moderately
  coarse convex polytopes, the maximum number of iterations would be around
  10 or so. When the vertex density is high or the surfaces are functionally
  smooth, the maximum number of iterations was observed as as high as 32
  when the two surfaces were very close to each other, contacting or not.
  */

  int numiters = 0;
  while(numiters < 50) {
    numiters++;
    printf("\n††††† numiters = %d †††††\n\n", numiters);
    /*
    numberSimplexPoints = 1, 2, 3, or 4, depending on the simplex dimension.
    numberSimplexPoints = 1: A single simplex support point will indicate
    surface contact if its magnitude is less than ZERO. In this case simplex0
    will set origin = 1 and return. Otherwise it will evaluate coordinates
    of a new searchDirectionVector and return. simplex0 does not change
    numberSimplexPoints.
    numberSimplexPoints = 2: Two points define a line. The origin may be on
    the line or not. If the origin is not on the line, it will be somewhere
    around the line. Simplex1 can determine if the origin is outside the
    convex Minkowski hull surface (origin = 0) or on the surface
    (origin = 1). In these cases simplex1 will return with no other changes.
    Otherwise it will evaluate coordinates of a new searchDirectionVector
    and return. simplex1 does not change numberSimplexPoints.
    numberSimplexPoints = 3: Three points define a triangle. The origin may
    be on the triangle or not. If not on the triangle, the origin will be
    somewhere around the triangle. Simplex2 can determine if the origin is
    on the convex Minkowski hull surface (origin = 1) or outside
    it (origin = 0). In these cases simplex2 will return with no other
    changes. Otherwise it will evaluate coordinates of a new
    searchDirectionVector and return. simplex2 sometimes reduces
    numberSimplexPoints by 1.
    numberSimplexPoints = 4: Four points define a tetrahedron. The origin
    may be on or in the tetrahedron or not. If the origin is on or in the
    tetrahedron, it will be in the convex Minkowski hull. Otherwise the
    origin will not be in the convex Minkowski hull. Simplex3 can determine
    if the origin is on or in the convex Minkowski hull surface (origin = 1)
    or outside it (origin = 0). In these cases simplex3 will return with no
    other changes. Otherwise it will evaluate coordinates of a new
    searchDirectionVector and return. simplex3 sometimes reduces
    numberSimplexPoints by 1 or 2. The final decision that the origin is
    in the convex Minkowski hull will be made in simplex3.
    */

    switch(self->simplexPointCount) {
      case 1:
        printf("***** entering simplex0 *****\n");
        simplex0(self->searchDirectionVector, VW, cA->extremeVertex,
                 cB->extremeVertex, &self->simplexPointCount, &self->collision,
                 &self->success);
        printf("***** leaving simplex0 *****\n");
        break;
      case 2:
        printf("***** entering simplex1 *****\n");
        simplex1(self->searchDirectionVector, VW, cA->extremeVertex,
                 cB->extremeVertex, &self->simplexPointCount, &self->collision,
                 &self->success);
        printf("***** leaving simplex1 *****\n");
        break;
      case 3:
        printf("***** entering simplex2 *****\n");
        simplex2(self->searchDirectionVector, VW, cA->extremeVertex,
                 cB->extremeVertex, &self->simplexPointCount, &self->collision,
                 &self->success);
        printf("***** leaving simplex2 *****\n");
        break;
      case 4:
        printf("***** entering simplex3 *****\n");
        simplex3(self->searchDirectionVector, VW, cA->extremeVertex,
                 cB->extremeVertex, &self->simplexPointCount, &self->collision,
                 &self->success);
        printf("***** leaving simplex3 *****\n");
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

    /*
    Arriving at this point means that origin == –1 and one of the simplex
    functions has determined that the current simplex requires another
    support point. To help obtain this new support point, the simplex
    function generated coordinates of a new search-direction vector and
    stored them in searchDirectionVector. As noted above, the simplex2
    function may have reduced numberSimplexPoints by one and the simplex3
    function will have reduced numberSimplexPoints by at least one and
    possibly two. The searchDirectionVector will be passed to the
    ConvexHullSupportFunctionA() and ConvexHullSupportFunctionB() functions
    and they will evaluate and store extreme vertex or point coordinates in
    extremeVertexLocalA[0] and extremeVertexLocalB[0]. But first the
    coordinates already stored in extremeVertexLocalA and
    extremeVertexLocalB must be shifted down one to make room for the new
    ones.
    */

    printf("\n^^^^^numberSimplexPointsBefore^^^^^ %d\n",
           self->simplexPointCount);

    printf("\n+++++simplex support matrix+++\n");
    for(int i = 0; i < self->simplexPointCount; i++) {
      s2_vector_print(stdout, &VW[i]);
    }

    for(int i = self->simplexPointCount; i > 0; i--) {
      s2_vector_copy(&cA->extremeVertex[i - 1], &cA->extremeVertex[i]);
      s2_vector_copy(&cB->extremeVertex[i - 1], &cB->extremeVertex[i]);
    }

    /* Use sDV = -self->searchDirectionVector in
     * ConvexHullSupportFunctionA() */
    s2_vector_t sDV;
    s2_vector_invert(&self->searchDirectionVector, &sDV);
    /* Local and absolute coordinates of extreme vertex on convex hull A. */
    printf("\n***** Entering ConvexHullSupportFunctionA *****\n");
    cA->support(self, cA, &sDV, &cA->extremeVertex[0], &EVA[0]);
    printf("***** Leaving ConvexHullSupportFunctionA *****\n");

    /* Use self->searchDirectionVector in ConvexHullSupportFunctionB()
       Local and absolute coordinates of extreme vertex on convex hull B. */
    printf("\n***** Entering ConvexHullSupportFunctionB *****\n");
    cB->support(self, cB, &sDV, &cB->extremeVertex[0], &EVB[0]);
    printf("***** Leaving ConvexHullSupportFunctionB *****\n");
    s2_subtract_vv(&EVB[0], &EVA[0], &VW[0]);

    /*
    The array VW[4][3] contains coordinates of up to four simplex support
    vectors where the current value of numberSimplexPoints is less than 4.
    The simplex vector coordinates in VW must be shifted down by one to
    make room for the next simplex support vector.
    */

    for(int i = self->simplexPointCount; i > 0; i--) {
      s2_vector_copy(&VW[i - 1], &VW[i]);
      s2_vector_copy(&VW[i - 1], &VW[i]);
    }

    /*
    Subtract the support point on hull A from the support point on hull B
    and insert the new point in the first position of the simplex.
    */
    s2_subtract_vv(&EVB[0], &EVA[0], &VW[0]);
    /*
    Record the latest insertion into the simplex.
    */
    self->simplexPointCount++;

    printf("\n^^^^^numberSimplexPointsAfter^^^^^ %d\n",
           self->simplexPointCount);

    printf("\n+++++simplex support matrix+++\n");
    for(int i = 0; i < self->simplexPointCount; i++) {
      s2_vector_print(stdout, &VW[i]);
    }
  }

  if(!self->success) {
    return (-1);
  }
  return (self->collision);
}

/*
The GJK algorithm uses a convex Minkowski hull (CMH) support function to
locate an extreme point when given a search-direction vector (sDV). The CMH
support function calls two convex hull (CH) support functions to locate
an extreme point on each of the two (potentially) contacting convex hulls.

The GJK algorithm uses a simplex composed of one to four points taken from the
CMH surface. However, computing all points on the CMH surface would be too
expensive, so only the simplex points on the CMH surface are computed. Because
the two CH surfaces are generally translating and rotating relative to each
other, absolute coordinates of the involved CH points are computed and used
to evaluate absolute coordinates of the involved CMH points.

While the GJK algorithm iterates to update the current simplex, the two objects'
spatial displacements remain constant. When one or both objects are moving
in an MBD simulation, the absolute and relative coordinates of the CH and CMH
points may have changed from the previous results and will have to be updated
before the next GJK iteration can take place. It is insufficient just to store
the computed relative coordinates of the previous CMH simplex for use in the
current GJK iteration because they may have changed. The best approach may be
to store local coordinates of the simplex points on the two CH objects from
the previous GJK solution and use the updated spatial displacements to
compute new absolute coordinates of the CMH simplex points.

The convex hulls are assumed to be rigid, and the local point coordinates are
stored. New spatial displacements are required to update the points for the
Minkowski hull. Failure to update the simplex points at the current step could
result in an incorrect contact decision. The local point coordinates evaluated
in the support functions are stored and used to update the simplex because they
don't change with the system states. The number of points
CMH->numberSimplexPoints and the corresponding points stored in CMH->PA[4] and
CMH->PB[4] from the previous GJK iteration are used to recover the last simplex
and continue the GJK iteration on the next function evaluation.
*/

/*
The GJK algorithm requires a support function to locate the extreme distance
to vertices in two convex hulls, say HA and HB, in the convex hull of a
Minkowski difference HB - HA. The extreme distance is expressed as
max(v•HB - v•HA) where v is a coordinate matrix representing some search
direction. This measure may be expressed alternately as max(v•HB) - max(-v•HA).
Given v, the support function must find the unique vertex VA in HA satisfying
max(-v•HA) and the unique vertex VB in HB satisfying max(v•HB). The desired
VA is obtained by sending -v into HA's hash table function evaluator, and VB is
obtained by sending v into HB's hash table function evaluator. The support
function will then return the desired point A = VB - VA in the Minkowski
difference, relative to its origin point O.

The GJK algorithm requires simplexes of dimensions, 0, 1, 2, and 3 in its
search for contacts between convex hulls HA and HB. A simplex of dimension 0
consists of a single point A on the convex Minkowski difference hull formed
from HB - HA. A simplex of dimension 1 consists of two points A and B on the
Minkowski difference hull. These two points define a line segment AB. A
simplex of dimension 2 consists of three points A, B, and C on the Minkowski
difference hull. These three points define three line segments AB, BC, and CA,
and a triangle ABC. A simplex of dimension 3 consists of four points A, B, C,
and D on the Minkowski difference hull. These four points define six line
segments, AB, BC, CD, DA, AC, and BD; four triangles ABC, BCD, CDA, and DAB;
and one tetrahedron ABCD. To process these simplexes as needed, four functions
simplex0(), simplex1(), simplex2(), and simplex3() will be defined. simplex0()
is called only once to obtain an initial vertex in the Minkowski difference
hull for the GJK process or not at all if a seed vertex is stored and recalled
from a previous GJK process. A line segment is the unique feature of simplex1(),
a triangle is the unique feature of simplex2(), and a tetrahedron is the
unique feature of simplex3(). Points are common to all three. simplex1() will
be used in simplex2() to simplify its implementation, and simplex1() and
simplex2() will be used in simplex3() to simplify its implementation.

The GJK algorithm basically loops, adding a Minkowski point to increase the
simplex dimension or removing a point to decrease the simplex dimension. To
add a point and increase simplex dimension, the GJK algorithm calls support()
with the current search direction. To remove a point and decrease simplex
dimension, the GJK algorithm simply drops the oldest, unused point. The number
of active points determines which simplex function the GJK algorithm must call.
Thus it keeps track of the running number of active points in the simplex.
*/

void simplex0(s2_vector_t sDV, s2_vector_t V[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success) {
  /*
  sDV will contain the next search-direction vector used by support() to
  locate the next extreme vertex point P relative to the origin point O,
  represented by the vector OP.
  V[0] contains the vector OA, which locates the simplex vertex point A
  relative to the origin point O.
  origin == 0: The convex hull does not contain the origin.
  origin == 1: The convex hull contains the origin.
  */
  s2_vector_invert(&V[0], &sDV);
  printf("numberSimplexPoints = %d\n", *numberSimplexPoints);
  double norm = s2_vector_l2norm(&sDV);
  if(norm < 1e-10) {
    *collision = true;
    *success = true;
  }
  printf("searchDirectionVector:\n");
  s2_vector_print(stdout, &sDV);
  printf("success = %s\n", *success ? "true" : "false");
  if(*success) {
    printf("collision = %s\n", *collision ? "true" : "false");
  }
  return;
}

/*
simplex1() takes in two points from the Minkowski difference hull, an old
point B and a new point A. The two points form a line segment, which is a
convex Minkowski difference hull. If the origin lies on this line, it is in
the convex hull and HA and HB intersect. simplex1() then reports this
and the GJK algorithm terminates. Otherwise we need a simplex2 triangle.
Before returning, simplex1() generates a search direction vector orthogonal
to the line, in the plane containing the origin, and in the direction of the
origin. support() then uses this direction vector to locate a third point
on the convex hull to form a triangle simplex2 convex hull.
*/
void simplex1(s2_vector_t sDV, s2_vector_t V[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success) {

  /*
  The simplex1 line consists of points A and B and vector AB = B - A
  representing a line segment connecting points A and B. Let OA represent
  the vector from the origin to point A. Let dir = (OA X AB) X AB represent
  the direction orthogonal to the line and lying in a plane containing
  the origin. Drawing lines parallel to dir through points A and B divides
  the plane into three regions. Before entering simplex1() we have verified
  that the origin is not in the region off the end of the line near point B.
  Thus we have to check two other regions. First, if the magnitude of dir is
  near zero the origin lies on the line and is in the convex hull. Thus
  the convex hulls HA and HB intersect. Otherwise we must expand the simplex
  with another point by searching in the direction of dir.

  sDV will contain the next search-direction vector used by support() to
  locate the next extreme vertex P relative to origin O, represented by OP.
  V[0] contains vector OA, which locates vertex A relative to O.
  V[1] contains vector OB, which locates vertex B relative to O.
  V[2] contains vector AB = OB - OA, which locates B relative to A.
  V[3] contains OA X AB
  sDV = (OA X AB) X AB
  origin == 0: The convex hull does not contain the origin.
  origin == 1: The convex hull contains the origin.
  */
  if(s2_dot_vv(&V[0], &sDV) < -1e-10) {
    printf("~~~~~ simplex 1: OA•sDV < -1e-10 ~~~~~\n");
    printf("Minkowski hull does not contain Origin.\n");
    *success = true;
    *collision = false;
    printf("searchDirectionVector:\n");
    s2_vector_print(stdout, &sDV);
    printf("success = %s\n", *success ? "true" : "false");
    printf("collision = %s\n", *collision ? "true" : "false");
    return;
  }

  s2_subtract_vv(&V[1], &V[0], &V[2]); // AB = OB - OA
  printf("OA = ");
  s2_vector_print(stdout, &V[0]);
  printf("OB = ");
  s2_vector_print(stdout, &V[1]);
  printf("AB = OB - OA = ");
  s2_vector_print(stdout, &V[2]);
  s2_cross_vv(&V[0], &V[2], &V[3]); // OA X AB
  s2_cross_vv(&V[3], &V[2], &sDV);  // (OA X AB) X AB
  printf("OA X AB = ");
  s2_vector_print(stdout, &V[3]);
  printf("(OA X AB) X AB = ");
  s2_vector_print(stdout, &sDV);
  printf("numberSimplexPoints = %d\n", *numberSimplexPoints);

  double norm = s2_vector_l2norm(&sDV);
  if(norm < 1e-10) {
    *collision = true;
    *success = true;
  }
  printf("searchDirectionVector:\n");
  s2_vector_print(stdout, &sDV);
  printf("success = %s\n", *success ? "true" : "false");
  if(*success) {
    printf("collision = %s\n", *collision ? "true" : "false");
  }
  return;
}

/*
simplex2() takes in three points from the Minkowski difference hull, two old
points B and C and a new point A. The three points form a triangle, which is a
convex Minkowski difference hull. If the origin lies on this triangle, it is in
the convex hull and the two objects intersect. simplex2() then reports this
and the GJK algorithm terminates. Otherwise we need a simplex3 tetrahedron.
Before returning, simplex2() generates a search direction vector orthogonal
to the plane of the triangle and in the direction of the origin. support()
then uses this direction vector to locate a fourth point on the convex hull
to form a tetrahedron simplex3 convex hull.
*/
void simplex2(s2_vector_t sDV, s2_vector_t V[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success) {

  /*
  The simplex2 triangle consists of points A, B, and C. Assume that the
  points are arranged counter-clockwise to form a triangle TABC with normal
  vector ABCn = AB X AC pointing upward or outward or frontward as we may
  interpret that direction. If we program simplex2() to always do this and
  we want it to do just the reverse, simply reverse the point order to
  C, B, and A. It is easy to see that this is equivalent to flipping the
  triangle about point B.

  Next let ACn = ABCn X AC and ABn = AB X ABCn represent outward-facing
  vectors, normal to the respective edges or lines AC and AB and lying
  in the TABC plane. Outward facing means they point away from the center
  of the triangle. Let OA represent the vector from the origin to point A.
  Then ABn•OA < 0 indicates that the origin is closer to the edge AB and
  outside the triangle. We set the search direction to (OA X AB) X AB to
  be perpendicular to edge AB and lying in the plane containing the origin.
  Then we use simplex2() to determine where the origin lies relative to
  this edge. If this test fails we then test ACn•OA < 0 to see if the origin
  is closer to the edge AC and outside the triangle. In this case we set the
  search direction to (OA X AC) X AC to be perpendicular to edge AC and lying
  in the plane containing the origin. Then we use simplex2() to determine
  where the origin lies relative to this edge. If this test fails, the origin
  then lies above or below or on the triangle. Let v = OA•ABCn. If v is
  close enough to zero the origin lies in the plane of the triangle; the
  convex hull contains the origin and the convex hulls HA and HB intersect.
  If v < 0 the origin is above the triangle and the next search direction is
  ABCn. And if v > 0 the origin is below the triangle. Now we need to search
  the same regions on this side. To do that we set the search direction to
  -ABCn and flip the triangle over around the point A to use the same
  simplex2() function. Thus we send C, A, B to simplex2() instead of A, B, C.

  sDV contains the search-direction vector used by support() to locate the
  next extreme vertex P relative to origin O, represented by OP.
  V[0] contains vector OA, which locates vertex A relative to O.
  V[1] contains vector OB, which locates vertex B relative to O.
  V[2] contains vector OC, which locates vertex C relative to O.
  V[3] contains vector AB = OB - OA, which locates B relative to A.
  V[4] contains vector AC = OC - OA, which locates C relative to A.
  V[5] contains vector ABCn = AB X AC, normal to TABC.
  V[6] contains vector ACn = ABCn X AC, tangent to TABC, normal to AC.
  V[7] contains vector ABn = AB X ABCn, tangent to TABC, normal to AB.
  V[8] contains vector OA X AB.
  sDV = (OA X AB) X AB
  origin == 0: The convex hull does not contain the origin.
  origin == 1: The convex hull contains the origin.
  */

  if(s2_dot_vv(&V[0], &sDV) < -1e-10) {
    printf("~~~~~ simplex 2: OA•sDV < -1e-10 ~~~~~\n");
    printf("Minkowski hull does not contain Origin.\n");
    *success = true;
    *collision = false;
    printf("searchDirectionVector:\n");
    s2_vector_print(stdout, &sDV);
    printf("success = %s\n", *success ? "true" : "false");
    printf("collision = %s\n", *collision ? "true" : "false");
    return;
  }

  s2_subtract_vv(&V[1], &V[0], &V[3]); // AB = OB - OA
  s2_subtract_vv(&V[2], &V[0], &V[4]); // AC = OC - OA
  s2_cross_vv(&V[3], &V[4], &V[5]);    // ABCn = AB X AC
  s2_cross_vv(&V[5], &V[4], &V[6]);    // ACn = ABCn X AC
  s2_cross_vv(&V[3], &V[5], &V[7]);    // ABn = AB X ABCn
  printf("OA = ");
  s2_vector_print(stdout, &V[0]);
  printf("OB = ");
  s2_vector_print(stdout, &V[1]);
  printf("OC = ");
  s2_vector_print(stdout, &V[2]);
  printf("AB = OB - OA = ");
  s2_vector_print(stdout, &V[3]);
  printf("AC = OC - OA = ");
  s2_vector_print(stdout, &V[4]);
  printf("ABCn = AB X AC = ");
  s2_vector_print(stdout, &V[5]);
  printf("ACn = ABCn X AC = ");
  s2_vector_print(stdout, &V[6]);
  printf("ABn = AB X ABCn = ");
  s2_vector_print(stdout, &V[7]);

  // if(OA•ABn < -1e-10)
  if(s2_dot_vv(&V[0], &V[7]) < -1e-10) {
    printf("~~~~~ simplex 2: OA•ABn < -1e-10 ~~~~~\n");
    s2_cross_vv(&V[0], &V[3], &V[8]); // OA X AB
    s2_cross_vv(&V[8], &V[3], &sDV);  // (OA X AB) X AB
    printf("OA X AB = ");
    s2_vector_print(stdout, &V[8]);
    printf("searchDirectionVector = (OA X AB) X AB = ");
    s2_vector_print(stdout, &sDV);
    (*numberSimplexPoints)--;
    printf("~~~1~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
    return;
    // simplex1: [OA, OB]
  }
  // if(OA•ACn < -1e-10)
  else if(s2_dot_vv(&V[0], &V[6]) < -1e-10) {
    printf("~~~~~ simplex 2: OA•ACn < -1e-10 ~~~~~\n");
    s2_cross_vv(&V[0], &V[4], &V[8]); // OA X AC
    s2_cross_vv(&V[8], &V[4], &sDV);  // (OA X AC) X AC
    printf("OA X AC = ");
    s2_vector_print(stdout, &V[8]);
    printf("searchDirectionVector = (OA X AC) X AC = ");
    s2_vector_print(stdout, &sDV);
    printf("Swap OB and OC\n");
    s2_vector_swap(&V[1], &V[2]); // swap OB and OC
    s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
    s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
    (*numberSimplexPoints)--;
    printf("~~~2~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
    return;
    // swap results simplex1: [OA, OC]
  } else {
    double v = -s2_dot_vv(&V[0], &V[5]); // -OA•ABCn
    if(v > 1e-10) {                      // Set search direction to ABCn
      printf("~~~~~ simplex 2: -OA•ABCn > 1e-10 ~~~~~\n");
      s2_vector_copy(&V[5], &sDV);
      printf("searchDirectionVector = ABCn ");
      s2_vector_print(stdout, &sDV);
      printf("~~~3~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else if(v < -1e-10) { // Set search direction to -ABCn
      printf("~~~~~ simplex 2: -OA•ABCn < -ZERO ~~~~~\n");
      s2_vector_invert(&V[5], &sDV);
      printf("searchDirectionVector = -ABCn ");
      s2_vector_print(stdout, &sDV);
      s2_vector_swap(&V[0], &V[1]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[0], &extremeVertexLocalA[1]);
      s2_vector_swap(&extremeVertexLocalB[0], &extremeVertexLocalB[1]);
      printf("Swap OA and OB\n");
      // swap results simplex2: [OB, OA, OC]
      printf("~~~4~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else {
      printf("~~~5~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      *collision = true;
      *success = true;
      printf("searchDirectionVector:\n");
      s2_vector_print(stdout, &sDV);
      printf("success = %s\n", *success ? "true" : "false");
      if(*success) {
        printf("collision = %s\n", *collision ? "true" : "false");
      }
      return;
    }
  }
}

/*
simplex3() takes in four points from the Minkowski difference hull, three old
points B, C and D and a new point A. The four points form a tetrahedron, which
is a convex Minkowski difference hull. If the origin lies on the surface of
this tetrahedron or inside it, the two convex hulls intersect. simplex3()
then reports this and the GJK algorithm terminates. Otherwise the GJK
algorithm searches for a new point in the direction of the origin. If it
finds one, it throws out the oldest point and calls simplex3() again.
If the GJK algorithm finds no more points it reports no intersection and
terminates.
*/
void simplex3(s2_vector_t sDV, s2_vector_t V[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success) {
  /*
  The simplex3 tetrahedron consists of points A, B, C, and D. Assume that
  points B, C, and D form the tehrahedron's base triangle TBCD and are
  arranged counter-clockwise in that order. Assume that point A lies above
  TBCD and that vector BCDn = BC X BD points upward in the direction of A.
  Vector BCDn is normal to TBCD. The other three faces of the tetrahedron
  are represented by triangles TABC, TACD, and TADB. The outward-facing
  normal vectors to these faces are ABCn = AB X AC, ACDn = AC X AD, and
  ADBn = AD X AB. Let OA represent the vector from origin point O to point A.

  We want to determine if the origin lies inside or outside the tetrahedron.
  If it lies on or in the tetrahedron, the convex hulls HA and HB are in
  contact or intersecting. If the origin lies outside the tetrahedron,
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
  origin is inside the tetrahedron and the convex hulls HA and HB intersect.

  sDV contains the direction vector used by support() to locate the
  next extreme vertex P relative to origin O, represented by OP.
  V[0] contains vector OA, which locates vertex A relative to O.
  V[1] contains vector OB, which locates vertex B relative to O.
  V[2] contains vector OC, which locates vertex C relative to O.
  V[3] contains vector OD, which locates vertex D relative to O.
  V[4] contains vector AB = OB - OA, which locates B relative to A.
  V[5] contains vector AC = OC - OA, which locates C relative to A.
  V[6] contains vector AD = OD - OA, which locates D relative to A.
  V[7] contains vector ABCn = AB X AC, normal to TABC.
  V[8] contains vector ACDn = AC X AD, normal to TACD.
  V[9] contains vector ADBn = AD X AB, normal to TADB.

  For the triangle ABC
  V[10] contains vector ACn = ABCn X AC, tangent to TABC, normal to AC.
  V[11] contains vector ABn = AB X ABCn, tangent to TABC, normal to AB.
  V[12] contains vector OA X AC.
  sDV = (OA X AC) X AC
  V[12] contains vector OA X AB.
  sDV = (OA X AB) X AB

  For the triangle ACD
  V[10] contains vector ADn = ACDn X AD, tangent to TACD, normal to AD.
  V[11] contains vector ACn = AC X ACDn, tangent to TACD, normal to AC.
  V[12] contains vector OA X AD.
  sDV = (OA X AD) X AD
  V[12] contains vector OA X AC.
  sDV = (OA X AC) X AC

  For the triangle ABD
  V[10] contains vector ABn = ADBn X AB, tangent to TADB, normal to AB.
  V[11] contains vector ADn = AD X ADBn, tangent to TADB, normal to AD.
  V[12] contains vector OA X AB.
  sDV = (OA X AB) X AB
  V[12] contains vector OA X AD.
  sDV = (OA X AD) X AD

  origin == 0: The convex hull does not contain the origin.
  origin == 1: The convex hull contains the origin.
  */

  if(s2_dot_vv(&V[0], &sDV) < -1e-10) {
    printf("~~~~~ simplex 3: OA•sDV < -1e-10 ~~~~~\n");
    printf("Minkowski hull does not contain Origin.\n");
    *success = true;
    *collision = false;
    printf("searchDirectionVector:\n");
    s2_vector_print(stdout, &sDV);
    printf("success = %s\n", *success ? "true" : "false");
    printf("collision = %s\n", *collision ? "true" : "false");
    return;
  }

  s2_subtract_vv(&V[1], &V[0], &V[4]); // AB = OB - OA
  s2_subtract_vv(&V[2], &V[0], &V[5]); // AC = OC - OA
  s2_subtract_vv(&V[3], &V[0], &V[6]); // AD = OD - OA
  s2_cross_vv(&V[4], &V[5], &V[7]);    // ABCn = AB X AC
  s2_cross_vv(&V[5], &V[6], &V[8]);    // ACDn = AC X AD
  s2_cross_vv(&V[6], &V[4], &V[9]);    // ADBn = AD X AB

  printf("OA = ");
  s2_vector_print(stdout, &V[0]);
  printf("OB = ");
  s2_vector_print(stdout, &V[1]);
  printf("OC = ");
  s2_vector_print(stdout, &V[2]);
  printf("OD = ");
  s2_vector_print(stdout, &V[3]);
  printf("AB = OB - OA = ");
  s2_vector_print(stdout, &V[4]);
  printf("AC = OC - OA = ");
  s2_vector_print(stdout, &V[5]);
  printf("AD = OD - OA = ");
  s2_vector_print(stdout, &V[6]);
  printf("ABCn = AB X AC = ");
  s2_vector_print(stdout, &V[7]);
  printf("ACDn = AC X AD = ");
  s2_vector_print(stdout, &V[8]);
  printf("ADBn = AD X AB = ");
  s2_vector_print(stdout, &V[9]);

  // if(OA•ABCn < -1e-10) O outside face of TABC & tetrahedron
  if(s2_dot_vv(&V[0], &V[7]) < -1e-10) {
    printf("~~~~~ simplex 3: OA•ABCn < -1e-10 ~~~~~\n");
    printf("if(OA•ABCn < -1e-10) O outside face of TABC & tetrahedron\n");
    s2_cross_vv(&V[7], &V[5], &V[10]); // ACn = ABCn X AC
    s2_cross_vv(&V[4], &V[7], &V[11]); // ABn = AB X ABCn
    printf("ACn = ABCn X AC = ");
    s2_vector_print(stdout, &V[10]);
    printf("ABn = AB X ABCn = ");
    s2_vector_print(stdout, &V[11]);
    if(s2_dot_vv(&V[0], &V[10]) <
       -1e-10) { // OA•ACn O outside edge AC in plane of TABC
      printf("~~~~~ simplex 3: OA•ACn < -ZERO ~~~~~\n");
      printf("OA•ACn O outside edge AC in plane of TABC\n");
      s2_cross_vv(&V[0], &V[5], &V[12]); // OA X AC
      s2_cross_vv(&V[12], &V[5], &sDV);  //(OA X AC) X AC
      printf("OA X AC = ");
      s2_vector_print(stdout, &V[12]);
      printf("searchDirectionVector = (OA X AC) X AC = ");
      s2_vector_print(stdout, &sDV);
      printf("Swap OB and OC\n");
      s2_vector_swap(&V[1], &V[2]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
      // simplex1: [OA, OC]
      (*numberSimplexPoints) -= 2;
      printf("~~~1~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else if(s2_dot_vv(&V[0], &V[11]) <
              -1e-10) { // OA•ABn O outside edge AB in plane of TABC
      printf("~~~~~ simplex 3: OA•ABn < -1e-10 ~~~~~\n");
      printf("OA•ABn O outside edge AB in plane of TABC\n");
      s2_cross_vv(&V[0], &V[4], &V[12]); // OA X AB
      s2_cross_vv(&V[12], &V[4], &sDV);  //(OA X AB) X AB
      printf("OA X AB = ");
      s2_vector_print(stdout, &V[12]);
      printf("searchDirectionVector = (OA X AB) X AB = ");
      s2_vector_print(stdout, &sDV);
      // simplex1: [OA, OB]
      (*numberSimplexPoints) -= 2;
      printf("~~~2~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else { // ABCn above TABC
      printf("~~~~~ simplex 3: ABCn above TABC ~~~~~\n");
      s2_vector_copy(&V[7], &sDV);
      printf("searchDirectionVector = ABCn ");
      s2_vector_print(stdout, &sDV);
      // simplex2: [OA, OB, OC]
      (*numberSimplexPoints)--;
      printf("~~~3~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    }
  }
  // if(OA•ACDn < -1e-10) O outside face of TACD & tetrahedron
  else if(s2_dot_vv(&V[0], &V[8]) < -1e-10) {
    printf("~~~~~ simplex 3: OA•ACDn < -1e-10 ~~~~~\n");
    printf("if(OA•ACDn < -1e-10) O outside face of TACD & tetrahedron\n");
    s2_cross_vv(&V[8], &V[6], &V[10]); // ADn = ACDn X AD
    s2_cross_vv(&V[5], &V[8], &V[11]); // ACn = AC X ACDn
    printf("ADn = ACDn X AD = ");
    s2_vector_print(stdout, &V[10]);
    printf("ACn = AC X ACDn = ");
    s2_vector_print(stdout, &V[11]);
    if(s2_dot_vv(&V[0], &V[10]) <
       -1e-10) { // OA•ADn O outside edge AD in plane of TACD
      printf("~~~~~ simplex 3: OA•ADn < -1e-10~~~~~ \n");
      printf("OA•ADn O outside edge AD in plane of TACD\n");
      s2_cross_vv(&V[0], &V[6], &V[12]); // OA X AD
      s2_cross_vv(&V[12], &V[6], &sDV);  //(OA X AD) X AD
      printf("OA X AD = ");
      s2_vector_print(stdout, &V[12]);
      printf("searchDirectionVector = (OA X AD) X AD = ");
      s2_vector_print(stdout, &sDV);
      printf("Swap OB and OD\n");
      s2_vector_swap(&V[1], &V[3]); // swap OB and OD
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[3]);
      // simplex1: [OA, OD]
      (*numberSimplexPoints) -= 2;
      printf("~~~4~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else if(s2_dot_vv(&V[0], &V[11]) <
              -1e-10) { // OA•ACn O outside edge AC in plane of TACD
      printf("~~~~~ simplex 3: OA•ACn < -1e-10~~~~~ \n");
      printf("OA•ACn O outside edge AC in plane of TACD\n");
      s2_cross_vv(&V[0], &V[5], &V[12]); // OA X AC
      s2_cross_vv(&V[12], &V[5], &sDV);  //(OA X AC) X AC
      printf("OA X AC = ");
      s2_vector_print(stdout, &V[12]);
      printf("searchDirectionVector = (OA X AC) X AC = ");
      s2_vector_print(stdout, &sDV);
      printf("Swap OB and OC\n");
      s2_vector_swap(&V[1], &V[2]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
      // simplex1: [OA, OC]
      (*numberSimplexPoints) -= 2;
      printf("~~~5~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else { // ACDn Face of TACD toward O
      printf("~~~~~ simplex 3: ACDn Face of TACD toward O ~~~~~\n");
      s2_vector_copy(&V[8], &sDV);
      printf("searchDirectionVector = ACDn ");
      s2_vector_print(stdout, &sDV);
      printf("Swap OB and OC followed by swap OC and OD\n");
      s2_vector_swap(&V[1], &V[2]); // swap OB and OC
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[2]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[2]);
      s2_vector_swap(&V[2], &V[3]); // swap OC and OD
      s2_vector_swap(&extremeVertexLocalA[2], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[2], &extremeVertexLocalB[3]);
      // simplex2: [OA, OC, OD]
      (*numberSimplexPoints)--;
      printf("~~~6~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    }
  }
  // if(OA•ADBn < -1e-10) O outside face of TADB & tetrahedron
  else if(s2_dot_vv(&V[0], &V[9]) < -1e-10) {
    printf("~~~~~ simplex 3: OA•ADBn < -1e-10 ~~~~~\n");
    printf("if(OA•ADBn < -1e-10) O outside face of TADB & tetrahedron\n");
    s2_cross_vv(&V[9], &V[4], &V[10]); // ABn = ADBn X AB
    s2_cross_vv(&V[6], &V[9], &V[11]); // ADn = AD X ADBn
    printf("ABn = ADBn X AB = ");
    s2_vector_print(stdout, &V[10]);
    printf("ADn = AD X ADBn = ");
    s2_vector_print(stdout, &V[11]);
    if(s2_dot_vv(&V[0], &V[10]) <
       -1e-10) { // OA•ABn O outside edge AB in plane of TADB
      printf("~~~~~ simplex 3: OA•ABn < -1e-10 ~~~~~\n");
      printf("OA•ABn O outside edge AB in plane of TADB\n");
      s2_cross_vv(&V[0], &V[4], &V[12]); // OA X AB
      s2_cross_vv(&V[12], &V[4], &sDV);  //(OA X AB) X AB
      printf("OA X AB = ");
      s2_vector_print(stdout, &V[12]);
      printf("searchDirectionVector = (OA X AB) X AB = ");
      s2_vector_print(stdout, &sDV);
      // simplex1: [OA, OB]
      (*numberSimplexPoints) -= 2;
      printf("~~~7~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else if(s2_dot_vv(&V[0], &V[11]) <
              -1e-10) { // OA•ADn O outside edge AD in plane of TADB
      printf("~~~~~ simplex 3: OA•ADn < -1e-10 ~~~~~\n");
      printf("OA•ADn O outside edge AD in plane of TADB\n");
      s2_cross_vv(&V[0], &V[6], &V[12]); // OA X AD
      s2_cross_vv(&V[12], &V[6], &sDV);  //(OA X AD) X AD
      s2_vector_swap(&V[1], &V[3]);      // swap OB and OD
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[3]);
      printf("OA X AD = ");
      s2_vector_print(stdout, &V[12]);
      printf("searchDirectionVector = (OA X AD) X AD = ");
      s2_vector_print(stdout, &sDV);
      printf("Swap OB and OD\n");
      // simplex1: [OA, OD]
      (*numberSimplexPoints) -= 2;
      printf("~~~8~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    } else { // ADBn Face of TADB toward O
      printf("~~~~~ simplex 3: ADBn Face of TADB toward O ~~~~~\n");
      s2_vector_copy(&V[9], &sDV);
      printf("searchDirectionVector = ADBn ");
      s2_vector_print(stdout, &sDV);
      printf("Swap OB and OD followed by swap OC and OD\n");
      s2_vector_swap(&V[1], &V[3]); // swap OB and OD
      s2_vector_swap(&extremeVertexLocalA[1], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[1], &extremeVertexLocalB[3]);
      s2_vector_swap(&V[2], &V[3]); // swap OC and OD
      s2_vector_swap(&extremeVertexLocalA[2], &extremeVertexLocalA[3]);
      s2_vector_swap(&extremeVertexLocalB[2], &extremeVertexLocalB[3]);
      // simplex2: [OA, OD, OB]
      (*numberSimplexPoints)--;
      printf("~~~9~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
      return;
    }
  } else {
    printf("~~~10~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
    *collision = true;
    *success = true;
    printf("searchDirectionVector:\n");
    s2_vector_print(stdout, &sDV);
    printf("success = %s\n", *success ? "true" : "false");
    if(*success) {
      printf("collision = %s\n", *collision ? "true" : "false");
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
  /* use (void) to suppress compiler warning. Remove this once
                                        correct support function is loaded
   */
  (void)context;
  s2_collisionobject_t* c1 = self->objectA;
  s2_collisionobject_t* c2 = self->objectB;
  (void)c1;
  (void)c2;
  /*! @TODO add call to sphere-sphere test here

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
@brief support mapping function for two sphere collision objects
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
