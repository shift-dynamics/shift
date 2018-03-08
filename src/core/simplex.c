#include <stdlib.h>

#include "collisionobject.h"
#include "simplex.h"

/*!
@brief support mapping function for two convexhull collision objects
@param [in] a first s2_collisionobject_t containing an s2_convexhull_t object
@param [in] a second s2_collisionobject_t containing an s2_convexhull_t object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_gjk(s2_context_t* context, s2_simplex_t* self) {
  /* use (void) to suppress unused variable compiler warning. Remove this once
                                        correct support function is loaded */
  // (void)context;
  // (void)c1;
  // (void)c1History;
  // (void)c2;
  // (void)c2History;
  s2_collisionobject_t* objectA;
  objectA = self->objectA;
  s2_collisionobject_t* objectB;
  objectB = self->objectB;
  s2_collisionobject_history_t* objectAHistory;
  objectAHistory = &self->objectAHistory;
  s2_collisionobject_history_t* objectBHistory;
  objectBHistory = &self->objectBHistory;

  /*! @TODO add call to gjk algorithm here

  access extreme vertex support mapping functions for c1 and c2 by calling:

  s2_vector_t searchDirection;

  s2_vector_t extremeVertex1 =
    s2_collisionobject_support(c1, context, searchDirection);

  s2_vector_t extremeVertex2 =
    s2_collisionobject_support(c2, context, searchDirection);
  */

  /*
  This function carries out a full GJK contact detection iteration on each pair
  of convex hulls. Before starting the current iteration cycle it must first
  compute the absolute spatial displacement of the hull A/B reference frames.
  For each convex hull pair, on the very first call to this function when
  numberSimplexPoints = 0, it must compute global coordinates of the
  searchDirectionVector. From then on when numberSimplexPoints > 0, the
  searchDirectionVector coordinates will be obtained from the most recent
  simplex. Each GJK iteration cycle starts by sending the most recent
  searchDirectionVector to the convex hull A/B support functions to obtain
  absolute extreme points A/B on the respective surfaces. The simplex support
  function subtracts coordinates of point A from B to obtain the next support
  point on the convex Minkowski hull, which is also the next point to be
  inserted into the first position of the simplex. To make room for this point,
  existing points are shifted down one position. The simplex is then analyzed to
  determine whether or not the two objects are in contact. If a determination
  can be made, a flag is set to 1 (contact) or 0 (no contact.) If a
  determination can't be made a new searchDirectionVector is generated from the
  simplex, and zero, one, or two old support points are eliminated. Removing
  support points reduces the simplex dimension and numberSimplexPoints, and
  inserting a support point increases the simplex dimension and
  numberSimplexPoints.

  Each convex hull support function has access to a struct containing the hull's
  definition data, but no historic or state information is stored in this
  struct. Many of these structs may contain a lot of data, so only one instance
  of each struct is used multiple times when possible. However each call to a
  simplex support function for a convex hull A/B pair is supplied with a unique
  instantiated struct that stores mostly historic and state information. All
  inquires and computations related to the individual A and B hulls are
  performed in their respective support functions, but the related historic and
  state information is stored in the simplex support struct, which is passed to
  the hull support function through its call list. The simplex support function
  stores additional historic and state information in its support struct.
  //     double extremeVertexLocalA[4][3];
  //     double extremeVertexLocalB[4][3];

  */
  s2_vector_t EVA[4]; // Absolute coordinates of extreme points on hull A
  s2_vector_t EVB[4]; // Absolute coordinates of extreme points on hull B
  s2_vector_t VW[13]; // Absolute coordinates of intermediate simplex vectors
  s2_vector_t sDV;    // Absolute coordinates of search-direction vector.

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

  // s2_product_dd(D1i, D2i, Do);
  // typedef struct {
  //   s2_mat3x3_t R;
  //   s2_vector_t T;
  // } s2_displacement_t;
  // typedef struct {
  //   double m00, m10, m20, m01, m11, m21, m02, m12, m22;
  // } s2_mat3x3_t;
  // typedef struct {
  //   double x, y, z;
  // } s2_vector_t;

  s2_displacement_t* dAbsoluteA;
  dAbsoluteA = &objectA->dAbsolute;
  if(!objectA->displacementUpdated) {
    objectA->displacementUpdated = true;
    s2_displacement_t* dWRTReferenceA;
    dWRTReferenceA = &objectA->dWRTReference;
    s2_displacement_t* referenceFrameA;
    referenceFrameA = objectA->referenceFrame;
    s2_product_dd(dWRTReferenceA, referenceFrameA, dAbsoluteA);
    printf("Absolute spatial displacement of hull A reference frame:\n");
    printf("Direction cosine matrix and origin point\n");
    printf("%g %g %g | %g\n", dAbsoluteA->R.m00, dAbsoluteA->R.m01,
           dAbsoluteA->R.m02, dAbsoluteA->T.x);
    printf("%g %g %g | %g\n", dAbsoluteA->R.m10, dAbsoluteA->R.m11,
           dAbsoluteA->R.m12, dAbsoluteA->T.y);
    printf("%g %g %g | %g\n", dAbsoluteA->R.m20, dAbsoluteA->R.m21,
           dAbsoluteA->R.m22, dAbsoluteA->T.z);
  }

  s2_displacement_t* dAbsoluteB;
  dAbsoluteB = &objectB->dAbsolute;
  if(!objectB->displacementUpdated) {
    objectB->displacementUpdated = true;
    s2_displacement_t* dWRTReferenceB;
    dWRTReferenceB = &objectB->dWRTReference;
    s2_displacement_t* referenceFrameB;
    referenceFrameB = objectB->referenceFrame;
    s2_product_dd(dWRTReferenceB, referenceFrameB, dAbsoluteB);
    printf("Absolute spatial displacement of hull B reference frame:\n");
    printf("Direction cosine matrix and origin point\n");
    printf("%g %g %g | %g\n", dAbsoluteB->R.m00, dAbsoluteB->R.m01,
           dAbsoluteB->R.m02, dAbsoluteB->T.x);
    printf("%g %g %g | %g\n", dAbsoluteB->R.m10, dAbsoluteB->R.m11,
           dAbsoluteB->R.m12, dAbsoluteB->T.y);
    printf("%g %g %g | %g\n", dAbsoluteB->R.m20, dAbsoluteB->R.m21,
           dAbsoluteB->R.m22, dAbsoluteB->T.z);
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

  // s2_subtract_vv(v1i, v2i, vo) { vo = v1i - v2i; }

  if(self->simplexPointCount == 0) {
    self->simplexPointCount = 1;
    s2_subtract_vv(&dAbsoluteA->T, &dAbsoluteB->T,
                   &self->searchDirectionVector);
    printf("simplexPointCount = %d\n", self->simplexPointCount);
    printf("Initialize searchDirectionVector: [");
    printf("%g %g %g]\n", self->searchDirectionVector.x,
           self->searchDirectionVector.y, self->searchDirectionVector.z);
  }

  // Use -searchDirectionVector in ConvexHullSupportFunctionA()
  // s2_vector_invert(vi, vo);
  s2_vector_invert(&self->searchDirectionVector, &sDV);

  void EllipsoidSupportFunction(void* SimplexSupportStructV,
                                EllipsoidSupportStruct* EllipsoidSupportStructI,
                                double sDV[3], double VL[3], double SDH[4][3],
                                double VA[3]) {

    printf("\n***** Entering ConvexHullSupportFunctionA *****\n");
    SimplexSupportStructI->ConvexHullSupportFunctionA(
      SimplexSupportStructI, SimplexSupportStructI->ConvexHullSupportStructA,
      sDV, SimplexSupportStructI->extremeVertexLocalA[0],
      spatialDisplacementHullA, EVA[0]);
    printf("***** Leaving ConvexHullSupportFunctionA *****\n");

    // Use  searchDirectionVector in ConvexHullSupportFunctionB()
    for(int i = 0; i < 3; i++) {
      sDV[i] = -sDV[i];
    }

    printf("\n***** Entering ConvexHullSupportFunctionB *****\n");
    SimplexSupportStructI->ConvexHullSupportFunctionB(
      SimplexSupportStructI, SimplexSupportStructI->ConvexHullSupportStructB,
      sDV, SimplexSupportStructI->extremeVertexLocalB[0],
      spatialDisplacementHullB, EVB[0]);
    printf("***** Leaving ConvexHullSupportFunctionB *****\n\n");

    for(int i = 0; i < 3; i++) {
      VW[0][i] = EVB[0][i] - EVA[0][i];
    }
  }
  /*
  The current simplex is composed of numberSimplexPoints sets of simplex
  support vertex or point coordinates expressed in the global coordinate frame.
  These coordinates are valid for the current kinematics or dynamics states
  but become invalid for the next states because they may have changed.
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
           SimplexSupportStructI->numberSimplexPoints);
    SimplexSupportStructI->RestoreConvexHullSupportA(
      EVA, spatialDisplacementHullA, SimplexSupportStructI->extremeVertexLocalA,
      &(SimplexSupportStructI->numberSimplexPoints));
    SimplexSupportStructI->RestoreConvexHullSupportB(
      EVB, spatialDisplacementHullB, SimplexSupportStructI->extremeVertexLocalB,
      &(SimplexSupportStructI->numberSimplexPoints));
    for(int i = 0; i < SimplexSupportStructI->numberSimplexPoints; i++) {
      for(int j = 0; j < 3; j++) {
        VW[i][j] = EVB[i][j] - EVA[i][j];
      }
    }
    printf("\n+++++simplex support matrix+++\n");
    for(int j = 0; j < 3; j++) {
      for(int i = 0; i < SimplexSupportStructI->numberSimplexPoints; i++) {
        printf("%g ", VW[i][j]);
      }
      printf("\n");
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

    switch(SimplexSupportStructI->numberSimplexPoints) {
      case 1:
        printf("***** entering simplex0 *****\n");
        simplex0(SimplexSupportStructI->searchDirectionVector, VW,
                 SimplexSupportStructI->extremeVertexLocalA,
                 SimplexSupportStructI->extremeVertexLocalB,
                 &(SimplexSupportStructI->numberSimplexPoints),
                 &(SimplexSupportStructI->origin));
        printf("***** leaving simplex0 *****\n");
        break;
      case 2:
        printf("***** entering simplex1 *****\n");
        simplex1(SimplexSupportStructI->searchDirectionVector, VW,
                 SimplexSupportStructI->extremeVertexLocalA,
                 SimplexSupportStructI->extremeVertexLocalB,
                 &(SimplexSupportStructI->numberSimplexPoints),
                 &(SimplexSupportStructI->origin));
        printf("***** leaving simplex1 *****\n");
        break;
      case 3:
        printf("***** entering simplex2 *****\n");
        simplex2(SimplexSupportStructI->searchDirectionVector, VW,
                 SimplexSupportStructI->extremeVertexLocalA,
                 SimplexSupportStructI->extremeVertexLocalB,
                 &(SimplexSupportStructI->numberSimplexPoints),
                 &(SimplexSupportStructI->origin));
        printf("***** leaving simplex2 *****\n");
        break;
      case 4:
        printf("***** entering simplex3 *****\n");
        simplex3(SimplexSupportStructI->searchDirectionVector, VW,
                 SimplexSupportStructI->extremeVertexLocalA,
                 SimplexSupportStructI->extremeVertexLocalB,
                 &(SimplexSupportStructI->numberSimplexPoints),
                 &(SimplexSupportStructI->origin));
        printf("***** leaving simplex3 *****\n");
        break;
      default:
        printf("ERROR! Incorrect numberSimplexPoints: %d\n",
               SimplexSupportStructI->numberSimplexPoints);
        exit(0);
        break;
    };

    if(SimplexSupportStructI->origin == 1) {
      printf("Minkowski hull Contains Origin. Setting origin = 1\n");
      break;
    }
    if(SimplexSupportStructI->origin == 0) {
      printf("Minkowski hull does not contain Origin. Setting origin = 0\n");
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
    coordinates already stored in extremeVertexLocalA and extremeVertexLocalB
    must be shifted down one to make room for the new ones.
    */

    printf("\n^^^^^numberSimplexPointsBefore^^^^^ %d\n",
           SimplexSupportStructI->numberSimplexPoints);

    printf("\n+++++simplex support matrix+++\n");
    for(int j = 0; j < 3; j++) {
      for(int i = 0; i < SimplexSupportStructI->numberSimplexPoints; i++) {
        printf("%g ", VW[i][j]);
      }
      printf("\n");
    }

    for(int i = SimplexSupportStructI->numberSimplexPoints; i > 0; i--) {
      for(int j = 0; j < 3; j++) {
        SimplexSupportStructI->extremeVertexLocalA[i][j] =
          SimplexSupportStructI->extremeVertexLocalA[i - 1][j];
        SimplexSupportStructI->extremeVertexLocalB[i][j] =
          SimplexSupportStructI->extremeVertexLocalB[i - 1][j];
      }
    }

    // Use -searchDirectionVector in ConvexHullSupportFunctionA()
    for(int i = 0; i < 3; i++) {
      sDV[i] = -SimplexSupportStructI->searchDirectionVector[i];
    }
    printf("\n***** Entering ConvexHullSupportFunctionA *****\n");
    SimplexSupportStructI->ConvexHullSupportFunctionA(
      SimplexSupportStructI, SimplexSupportStructI->ConvexHullSupportStructA,
      sDV, SimplexSupportStructI->extremeVertexLocalA[0],
      spatialDisplacementHullA, EVA[0]);
    printf("***** Leaving ConvexHullSupportFunctionA *****\n");

    // Use  searchDirectionVector in ConvexHullSupportFunctionB()
    for(int i = 0; i < 3; i++) {
      sDV[i] = -sDV[i];
    }

    printf("\n***** Entering ConvexHullSupportFunctionB *****\n");
    SimplexSupportStructI->ConvexHullSupportFunctionB(
      SimplexSupportStructI, SimplexSupportStructI->ConvexHullSupportStructB,
      sDV, SimplexSupportStructI->extremeVertexLocalB[0],
      spatialDisplacementHullB, EVB[0]);
    printf("***** Leaving ConvexHullSupportFunctionB *****\n\n");

    /*
    The array VW[4][3] contains coordinates of up to four simplex support
    vectors where the current value of numberSimplexPoints is less than 4.
    The simplex vector coordinates in VW must be shifted down by one to
    make room for the next simplex support vector.
    */

    for(int i = SimplexSupportStructI->numberSimplexPoints; i > 0; i--) {
      for(int j = 0; j < 3; j++) {
        VW[i][j] = VW[i - 1][j];
      }
    }
    /*
    Subtract the support point on hull A from the support point on hull B
    and insert the new point in the first position of the simplex.
    */
    for(int i = 0; i < 3; i++) {
      VW[0][i] = EVB[0][i] - EVA[0][i];
    }
    /*
    Record the latest insertion into the simplex.
    */
    SimplexSupportStructI->numberSimplexPoints++;

    printf("\n^^^^^numberSimplexPointsAfter^^^^^ %d\n",
           SimplexSupportStructI->numberSimplexPoints);

    printf("\n+++++simplex support matrix+++\n");
    for(int j = 0; j < 3; j++) {
      for(int i = 0; i < SimplexSupportStructI->numberSimplexPoints; i++) {
        printf("%g ", VW[i][j]);
      }
      printf("\n");
    }
  }

  if(SimplexSupportStructI->numberSimplexPoints < 0) {
    return (-1);
  }
  return (SimplexSupportStructI->origin);

  return true;
}

/*!
@brief support mapping function for two sphere collision objects
@param [in] a first s2_collisionobject_t containing an s2_sphere_t object
@param [in] a second s2_collisionobject_t containing an s2_sphere_t object
@return whether the two objects are in contact with one another
*/
bool s2_simplex_sphereSphere(s2_context_t* context,
                             s2_collisionobject_t* restrict c1,
                             s2_collisionobject_history_t* restrict c1History,
                             s2_collisionobject_t* restrict c2,
                             s2_collisionobject_history_t* restrict c2History) {
  /* use (void) to suppress compiler warning. Remove this once
                                        correct support function is loaded */
  (void)context;
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
  s2_context_t* context, s2_collisionobject_t* restrict c1,
  s2_collisionobject_history_t* restrict c1History,
  s2_collisionobject_t* restrict c2,
  s2_collisionobject_history_t* restrict c2History) {

  /* use (void) to suppress unused variable compiler warning. Remove this once
  correct support function is loaded */
  (void)context;
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
void s2_simplex_support(s2_context_t* context, s2_simplex_t* self) {
  self->collision = self->support(context, self->objectA, &self->objectAHistory,
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
