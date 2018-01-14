//<assert.h> <ctype.h> <errno.h> <float.h> <limits.h> <locale.h> <math.h> <setjmp.h>
//<signal.h> <stdarg.h> <stddef.h> <stdio.h> <stdlib.h> <string.h> <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>
#include <stdbool.h>

#include "contact.h"

#define PI acos(-1.0)
#define dPI 2.0*PI
#define hPI PI/2.0
#define ZERO 1.0e-10

/*
This main function is a wrapper for testing the contact detection algorithm
between two arbitrary convex hulls A/B. The convex hulls A/B may be simple,
such as an analytical representation of a sphere, to complex, such as a polytope
surface represented by thousands of vertices and potentially using hash-table
lookup to rapidly locate an extreme vertex when provided coordinates of a
search-direction vector.
*/

int main() {

    /*
    The contact detection algorithm makes no assumptions about the internal 
    representation of the convex hulls A/B, so there is no default structure
    or support function to fit all. Typically the user determines what data a
    convex hull support function requires to efficiently locate an extreme
    vertex or point on its surface when provided absolute coordinates of a
    search-direction vector.
    */

    EllipsoidSupportStruct *EllipsoidSupportStructA;
    EllipsoidSupportStructA = malloc(sizeof(EllipsoidSupportStruct));
    EllipsoidSupportStructA->a = 1.0;
    EllipsoidSupportStructA->b = 1.0;
    EllipsoidSupportStructA->c = 1.0;
    EllipsoidSupportStruct *EllipsoidSupportStructB;
    EllipsoidSupportStructB = malloc(sizeof(EllipsoidSupportStruct));
    EllipsoidSupportStructB->a = 1.0;
    EllipsoidSupportStructB->b = 1.0;
    EllipsoidSupportStructB->c = 1.0;

    SimplexSupportStruct *SimplexSupportStructAB;
    SimplexSupportStructAB = malloc(sizeof(SimplexSupportStruct));
    SimplexSupportStructAB->ConvexHullSupportFunctionA =
                                EllipsoidSupportFunction;
    SimplexSupportStructAB->ConvexHullSupportStructA =
                                (void*) EllipsoidSupportStructA;
    SimplexSupportStructAB->ConvexHullSupportFunctionB =
                                EllipsoidSupportFunction;
    SimplexSupportStructAB->ConvexHullSupportStructB =
                                (void*) EllipsoidSupportStructB;
    SimplexSupportStructAB->numberSimplexPoints = 0;

    double spatialDisplacementBodyA[4][3];
    double spatialDisplacementBodyB[4][3];
    SimplexSupportStructAB->spatialDisplacementBodyA =
                                &spatialDisplacementBodyA[0];
    SimplexSupportStructAB->spatialDisplacementBodyB =
                                &spatialDisplacementBodyB[0];

    for (int i=0; i<4; i++) {
        for (int j=0; j<3; j++) {
            spatialDisplacementBodyA[i][j] = 0.0;
            spatialDisplacementBodyB[i][j] = 0.0;
            SimplexSupportStructAB->shapeMatrixA[i][j] = 0.0;
            SimplexSupportStructAB->shapeMatrixB[i][j] = 0.0;
        }
    }
    for (int i=0; i<3; i++) {
        spatialDisplacementBodyA[i][i] = 1.0;
        spatialDisplacementBodyB[i][i] = 1.0;
        SimplexSupportStructAB->shapeMatrixA[i][i] = 1.0;
        SimplexSupportStructAB->shapeMatrixB[i][i] = 1.0;
    }
    spatialDisplacementBodyA[3][0] =  0.6;
    spatialDisplacementBodyA[3][1] =  0.6;
    spatialDisplacementBodyA[3][2] =  0.6;
    spatialDisplacementBodyB[3][0] = -0.6;
    spatialDisplacementBodyB[3][1] = -0.6;
    spatialDisplacementBodyB[3][2] = -0.6;
    SimplexSupportStructAB->RestoreConvexHullSupportA = RestoreEllipsoidSupport;
    SimplexSupportStructAB->RestoreConvexHullSupportB = RestoreEllipsoidSupport;

    SimplexSupportStructAB->shapeMatrixA[3][0] =  0.0;
    SimplexSupportStructAB->shapeMatrixA[3][1] =  0.0;
    SimplexSupportStructAB->shapeMatrixA[3][2] =  0.0;
    SimplexSupportStructAB->shapeMatrixB[3][0] =  0.0;
    SimplexSupportStructAB->shapeMatrixB[3][1] =  0.0;
    SimplexSupportStructAB->shapeMatrixB[3][2] =  0.0;
    printf("\nshapeMatrixA\n");
    for (int j=0; j<3; j++) {
        for (int i=0; i<4; i++) {
            printf("%g ", SimplexSupportStructAB->shapeMatrixA[i][j]);
        }
        printf("\n");
    }
    printf("\nshapeMatrixB\n");
    for (int j=0; j<3; j++) {
        for (int i=0; i<4; i++) {
            printf("%g ", SimplexSupportStructAB->shapeMatrixB[i][j]);
        }
        printf("\n");
    }

    for (int i=0; i<20; i++){
        printf("\n\n*****************************************\n");
        printf("              step number: %d\n", i+1);
        printf("*****************************************\n");
        SimplexSupportFunction(SimplexSupportStructAB);
        spatialDisplacementBodyA[3][0] -= 0.005;
        spatialDisplacementBodyA[3][1] -= 0.005;
        spatialDisplacementBodyA[3][2] -= 0.005;
        spatialDisplacementBodyB[3][0] += 0.005;
        spatialDisplacementBodyB[3][1] += 0.005;
        spatialDisplacementBodyB[3][2] += 0.005;
    }
    // for (int i=150; i<300; i++){
    //     printf("\n\n*****************************************\n");
    //     printf("              step number: %d\n", i+1);
    //     printf("*****************************************\n");
    //     SimplexSupportFunction(SimplexSupportStructAB);
    //     spatialDisplacementBodyA[3][0] += 0.0001;
    //     spatialDisplacementBodyA[3][1] += 0.0001;
    //     spatialDisplacementBodyA[3][2] += 0.0001;
    //     spatialDisplacementBodyB[3][0] -= 0.0001;
    //     spatialDisplacementBodyB[3][1] -= 0.0001;
    //     spatialDisplacementBodyB[3][2] -= 0.0001;
    // }
    return(0);
}

/*
This function carries out a full GJK contact detection iteration on each pair
of convex hulls. Before starting the current iteration cycle it must first
compute the absolute spatial displacement of the hull A/B reference frames.
For each convex hull pair, on the very first call to this function when
numberSimplexPoints = 0, it must compute global coordinates of the
searchDirectionVector. From then on when numberSimplexPoints > 0, the
searchDirectionVector coordinates will be obtained from the most recent simplex.
Each GJK iteration cycle starts by sending the most recent searchDirectionVector
to the convex hull A/B support functions to obtain absolute extreme points A/B
on the respective surfaces. The simplex support function subtracts coordinates
of point A from B to obtain the next support point on the convex Minkowski hull,
which is also the next point to be inserted into the first position of the
simplex. To make room for this point, existing points are shifted down one
position. The simplex is then analyzed to determine whether or not the two
objects are in contact. If a determination can be made, a flag is set to
1 (contact) or 0 (no contact.) If a determination can't be made a new
searchDirectionVector is generated from the simplex, and zero, one, or two old
support points are eliminated. Removing support points reduces the simplex
dimension and numberSimplexPoints, and inserting a support point increases
the simplex dimension and numberSimplexPoints.

Each convex hull support function has access to a struct containing the hull's
definition data, but no historic or state information is stored in this struct.
Many of these structs may contain a lot of data, so only one instance of each
struct is used multiple times when possible. However each call to a simplex
support function for a convex hull A/B pair is supplied with a unique
instantiated struct that stores mostly historic and state information. All
inquires and computations related to the individual A and B hulls are performed
in their respective support functions, but the related historic and state
information is stored in the simplex support struct, which is passed to the
hull support function through its call list. The simplex support function stores
additional historic and state information in its support struct.
//     double extremeVertexLocalA[4][3];
//     double extremeVertexLocalB[4][3];

*/
int SimplexSupportFunction(SimplexSupportStruct* SimplexSupportStructI) {
    double EVA[4][3]; //Absolute coordinates of extreme points on hull A
    double EVB[4][3]; //Absolute coordinates of extreme points on hull B
    double VW[13][3]; //Absolute coordinates of intermediate simplex vectors
    double sDV[3];    //Absolute coordinates of search-direction vector.

    /*
    Absolute direction cosine matrix and position vector of convex hull A and B
    reference frames.
    */
    double spatialDisplacementHullA[4][3];
    double spatialDisplacementHullB[4][3];

    /*
    origin = -1 until a surface contact decision has been made
    origin =  0 when the surfaces are not in contact
    origin =  1 when the surfaces are touching or penetrating
    The GJK iteration terminates when origin = 0 or 1, or when the maximum
    number of GJK iterations has been reached. SimplexSupportFunction() will
    return with origin set to 0 or 1 on successful contact decision and with
    origin set to –1 on unsuccessful contact decision.
    */
    SimplexSupportStructI->origin = -1;

    printf("\n~~~~~In SimplexSupportFunction~~~~~\n");

    /*
    On each SimplexSupportFunction call the kinematics or dynamics states may
    have changed. Thus the body spatial displacements may have changed and the
    spatial displacements of convex hulls A and B may have changed. The arrays
    spatialDisplacementHullA and spatialDisplacementHullB contain the spatial
    displacement of the convex hull A and B reference frames.
    */
    SDMPY(spatialDisplacementHullA,
          SimplexSupportStructI->shapeMatrixA,
          SimplexSupportStructI->spatialDisplacementBodyA);
    printf("Absolute spatial displacement of hull A reference frame:\n");
    printf("Direction cosine matrix and origin point\n");
    for (int i=0; i<3; i++) {
        printf("%g %g %g | %g\n",
                spatialDisplacementHullA[i][0],
                spatialDisplacementHullA[i][1],
                spatialDisplacementHullA[i][2],
                spatialDisplacementHullA[3][i]);
    }
    SDMPY(spatialDisplacementHullB,
          SimplexSupportStructI->shapeMatrixB,
          SimplexSupportStructI->spatialDisplacementBodyB);
    printf("Absolute spatial displacement of hull B reference frame:\n");
    printf("Direction cosine matrix and origin point\n");
    for (int i=0; i<3; i++) {
        printf("%g %g %g | %g\n",
                spatialDisplacementHullB[i][0],
                spatialDisplacementHullB[i][1],
                spatialDisplacementHullB[i][2],
                spatialDisplacementHullB[3][i]);
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
    if (SimplexSupportStructI->numberSimplexPoints == 0) {
        SimplexSupportStructI->numberSimplexPoints = 1;
        printf("numberSimplexPoints = %d\n",
                    SimplexSupportStructI->numberSimplexPoints);
        printf("Initialize searchDirectionVector: [");
        for (int i=0; i<3; i++){
            SimplexSupportStructI->searchDirectionVector[i] =
                spatialDisplacementHullA[3][i] - spatialDisplacementHullB[3][i];
            printf(" %g", SimplexSupportStructI->searchDirectionVector[i]);
        }
        printf ("]\n");

        //Use -searchDirectionVector in ConvexHullSupportFunctionA()
        for (int i=0; i<3; i++) {
            sDV[i] = -SimplexSupportStructI->searchDirectionVector[i];
        }
        printf("\n***** Entering ConvexHullSupportFunctionA *****\n");
        SimplexSupportStructI->ConvexHullSupportFunctionA(SimplexSupportStructI,
            SimplexSupportStructI->ConvexHullSupportStructA,
            sDV, SimplexSupportStructI->extremeVertexLocalA[0],
            spatialDisplacementHullA, EVA[0]);
        printf("***** Leaving ConvexHullSupportFunctionA *****\n");

        //Use  searchDirectionVector in ConvexHullSupportFunctionB()
        for (int i=0; i<3; i++) {
            sDV[i] = -sDV[i];
        }

        printf("\n***** Entering ConvexHullSupportFunctionB *****\n");
        SimplexSupportStructI->ConvexHullSupportFunctionB(SimplexSupportStructI,
            SimplexSupportStructI->ConvexHullSupportStructB,
            sDV, SimplexSupportStructI->extremeVertexLocalB[0],
            spatialDisplacementHullB, EVB[0]);
        printf("***** Leaving ConvexHullSupportFunctionB *****\n\n");

        for (int i=0; i<3; i++)
        {
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
        SimplexSupportStructI->RestoreConvexHullSupportA(EVA,
            spatialDisplacementHullA,
            SimplexSupportStructI->extremeVertexLocalA,
            &(SimplexSupportStructI->numberSimplexPoints));
        SimplexSupportStructI->RestoreConvexHullSupportB(EVB,
            spatialDisplacementHullB,
            SimplexSupportStructI->extremeVertexLocalB,
            &(SimplexSupportStructI->numberSimplexPoints));
        for (int i=0; i<SimplexSupportStructI->numberSimplexPoints; i++) {
            for (int j=0; j<3; j++)
            {
                VW[i][j] = EVB[i][j] - EVA[i][j];
            }
        }
        printf("\n+++++simplex support matrix+++\n");
        for (int j=0; j<3; j++) {
            for (int i=0; i<SimplexSupportStructI->numberSimplexPoints; i++) {
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
    while (numiters < 50) {
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

        switch (SimplexSupportStructI->numberSimplexPoints) {
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

        if (SimplexSupportStructI->origin == 1) {
            printf("Minkowski hull Contains Origin. Setting origin = 1\n");
            break;
        }
        if (SimplexSupportStructI->origin == 0) {
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
        for (int j=0; j<3; j++) {
            for (int i=0; i<SimplexSupportStructI->numberSimplexPoints; i++) {
                printf("%g ", VW[i][j]);
            }
            printf("\n");
        }

        for (int i=SimplexSupportStructI->numberSimplexPoints; i>0; i--) {
            for ( int j=0; j<3; j++ ) {
                SimplexSupportStructI->extremeVertexLocalA[i][j] =
                    SimplexSupportStructI->extremeVertexLocalA[i-1][j];
                SimplexSupportStructI->extremeVertexLocalB[i][j] =
                    SimplexSupportStructI->extremeVertexLocalB[i-1][j];
            }
        }

        //Use -searchDirectionVector in ConvexHullSupportFunctionA()
        for (int i=0; i<3; i++) {
            sDV[i] = -SimplexSupportStructI->searchDirectionVector[i];
        }
        printf("\n***** Entering ConvexHullSupportFunctionA *****\n");
        SimplexSupportStructI->ConvexHullSupportFunctionA(SimplexSupportStructI,
            SimplexSupportStructI->ConvexHullSupportStructA,
            sDV, SimplexSupportStructI->extremeVertexLocalA[0],
            spatialDisplacementHullA, EVA[0]);
        printf("***** Leaving ConvexHullSupportFunctionA *****\n");

        //Use  searchDirectionVector in ConvexHullSupportFunctionB()
        for (int i=0; i<3; i++) {
            sDV[i] = -sDV[i];
        }

        printf("\n***** Entering ConvexHullSupportFunctionB *****\n");
        SimplexSupportStructI->ConvexHullSupportFunctionB(SimplexSupportStructI,
            SimplexSupportStructI->ConvexHullSupportStructB,
            sDV, SimplexSupportStructI->extremeVertexLocalB[0],
            spatialDisplacementHullB, EVB[0]);
        printf("***** Leaving ConvexHullSupportFunctionB *****\n\n");

        /*
        The array VW[4][3] contains coordinates of up to four simplex support
        vectors where the current value of numberSimplexPoints is less than 4.
        The simplex vector coordinates in VW must be shifted down by one to
        make room for the next simplex support vector.
        */

        for ( int i=SimplexSupportStructI->numberSimplexPoints; i>0; i-- ) {
            for ( int j=0; j<3; j++ ) {
                VW[i][j] = VW[i-1][j];
            }
        }
        /*
        Subtract the support point on hull A from the support point on hull B
        and insert the new point in the first position of the simplex.
        */
        for (int i=0; i<3; i++) {
            VW[0][i] = EVB[0][i] - EVA[0][i];
        }
        /*
        Record the latest insertion into the simplex.
        */
        SimplexSupportStructI->numberSimplexPoints++;

        printf("\n^^^^^numberSimplexPointsAfter^^^^^ %d\n",
                    SimplexSupportStructI->numberSimplexPoints);

        printf("\n+++++simplex support matrix+++\n");
        for (int j=0; j<3; j++) {
            for (int i=0; i<SimplexSupportStructI->numberSimplexPoints; i++) {
                printf("%g ", VW[i][j]);
            }
            printf("\n");
        }
    }

    if(SimplexSupportStructI->numberSimplexPoints < 0) {
        return(-1);
    }
    return(SimplexSupportStructI->origin);
}

void EllipsoidSupportFunction(
    void* SimplexSupportStructV,
    EllipsoidSupportStruct* EllipsoidSupportStructI,
    double sDV[3], double VL[3],
    double SDH[4][3], double VA[3]) {

    SimplexSupportStruct* SSSV;
    SSSV = (SimplexSupportStruct*) SimplexSupportStructV;

    double sDVL[3];
    //Transform sDV coordinates to local coordinate frame and store in sDVL.
    for (int i=0; i<3; i++) {
        sDVL[i] = 0.0;
        for (int j=0; j<3; j++) {
            sDVL[i] += SDH[j][i] * sDV[j];
        }
    }
    printf("sDVL = [%g, %g, %g]\n", sDVL[0], sDVL[1], sDVL[2]);

    double a = EllipsoidSupportStructI->a;
    double b = EllipsoidSupportStructI->b;
    double c = EllipsoidSupportStructI->c;
    printf("a = %g, b = %g, c = %g\n", a, b, c);

    double au = a * sDVL[0];
    double bu = b * sDVL[1];
    double cu = c * sDVL[2];
    printf("au = %g, bu = %g, cu = %g\n", au, bu, cu);

    double sf;
    sf = 2000.0;

    double theta = atan2( bu, au );
    if(theta < 0.0) {
        theta = dPI + theta;
    }

    theta = round(sf*theta)/sf;

    double ct = cos(theta);
    double st = sin(theta);
    printf("theta = %g, cos(theta) = %g, sin(theta) = %g\n", theta, ct, st);

    double phi;
    if (fabs(ct) > fabs(st) ) {
        phi = atan2( au, cu*ct );
    }
    else {
        phi = atan2( bu, cu*st );
    }
    if (phi < 0.0) {
        phi = PI + phi;
    }

    phi = round(sf*phi)/sf;

    double cp = cos(phi);
    double sp = sin(phi);
    printf("phi = %g, cos(phi) = %g, sin(phi) = %g\n", phi, cp, sp);

    VL[0] = a*ct*sp;
    VL[1] = b*st*sp;
    VL[2] = c*cp;
    printf("Local extreme point on convex hull: [%g %g %g]\n",
                VL[0], VL[1], VL[2]);
    // printf("(VL[0]/a)^2+(VL[1]/b)^2+(VL[2]/c)^2 = %g\n",
    //         VL[0]*VL[0]/(a*a)+VL[1]*VL[1]/(b*b)+VL[2]*VL[2]/(c*c));

    // double tvt[3];
    // tvt[0] = -a*st*sp;
    // tvt[1] =  b*ct*sp;
    // tvt[2] =  0.0;
    // printf("tvt = [%g, %g, %g]\n", tvt[0], tvt[1], tvt[2]);
    // printf("sDV^T*tvt = %g\n", sDVL[0]*tvt[0]+sDVL[1]*tvt[1]+sDVL[2]*tvt[2]);
    //
    // double tvp[3];
    // tvp[0] =  a*ct*cp;
    // tvp[1] =  b*st*cp;
    // tvp[2] = -c*sp;
    // printf("tvp = [%g, %g, %g]\n", tvp[0], tvp[1], tvp[2]);
    // printf("sDV^T*tvp = %g\n", sDVL[0]*tvp[0]+sDVL[1]*tvp[1]+sDVL[2]*tvp[2]);

    /*
    Transform coordinates of VL to global coordinate frame when necessary
    and store in VA.
    */

    for (int i=0; i<3; i++) {
        VA[i] = SDH[3][i];
        for(int j=0; j<3; j++) {
            VA[i] += SDH[i][j]*VL[j];
        }
    }

    printf("Absolute extreme point on convex hull: [%g %g %g]\n",
                VA[0], VA[1], VA[2]);

    return;
}

void RestoreEllipsoidSupport(
                double EVA[4][3], double SDH[4][3],
                double WVL[4][3], int *numberSimplexPoints) {
    printf("\n+++++local extreme vertex support matrix+++++\n");
    for (int j=0; j<3; j++) {
        for (int i=0; i<*numberSimplexPoints; i++) {
            printf("%g ", WVL[i][j]);
        }
        printf("\n");
    }

    for (int i=0; i<*numberSimplexPoints; i++){
        for (int j=0; j<3; j++) {
            EVA[i][j] = SDH[3][j];
            for(int k=0; k<3; k++) {
                EVA[i][j] += SDH[j][k]*WVL[i][k];
            }
        }
    }
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
don't change with the system states. The number of points CMH->numberSimplexPoints and
the corresponding points stored in CMH->PA[4] and CMH->PB[4] from the previous
GJK iteration are used to recover the last simplex and continue the GJK
iteration on the next function evaluation.
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

void simplex0(double sDV[3], double V[13][3],
              double eVLA[4][3], double eVLB[4][3],
              int *numberSimplexPoints, int *origin) {
    /*
    sDV will contain the next search-direction vector used by support() to
    locate the next extreme vertex point P relative to the origin point O,
    represented by the vector OP.
    V[0] contains the vector OA, which locates the simplex vertex point A
    relative to the origin point O.
    origin == 0: The convex hull does not contain the origin.
    origin == 1: The convex hull contains the origin.
    */
    sDV[0] = -V[0][0];
    sDV[1] = -V[0][1];
    sDV[2] = -V[0][2];
    printf("numberSimplexPoints = %d\n", *numberSimplexPoints);
    double norm;
    norm = norm_vec(sDV);
    if( norm < ZERO ) {
        *origin = 1;
    }
    printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
        sDV[0], sDV[1], sDV[2], *origin);
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
void simplex1(double sDV[3], double V[13][3],
              double eVLA[4][3], double eVLB[4][3],
              int *numberSimplexPoints, int *origin) {

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
    if (dot_vec(V[0], sDV) < -ZERO) {
        printf("~~~~~ simplex 1: OA•sDV < -ZERO ~~~~~\n");
        printf("Minkowski hull does not contain Origin. Setting origin = 0\n");
        *origin = 0;
        printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
            sDV[0], sDV[1], sDV[2], *origin);
        return;
    }

    dif_vec(V[1], V[0], V[2]); // AB = OB - OA
    printf("OA = [%g, %g, %g]\n", V[0][0], V[0][1], V[0][2]);
    printf("OB = [%g, %g, %g]\n", V[1][0], V[1][1], V[1][2]);
    printf("AB = OB - OA = [%g, %g, %g]\n", V[2][0], V[2][1], V[2][2]);
    cross_vec(V[0], V[2], V[3]); // OA X AB
    cross_vec(V[3], V[2], sDV); // (OA X AB) X AB
    printf("OA X AB = [%g, %g, %g]\n", V[3][0], V[3][1], V[3][2]);
    printf("(OA X AB) X AB = [%g, %g, %g]\n", sDV[0], sDV[1], sDV[2]);
    printf("numberSimplexPoints = %d\n", *numberSimplexPoints);
    double norm;
    norm = norm_vec(sDV);
    if( norm < ZERO ) {
        *origin = 1;
    }
    printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
        sDV[0], sDV[1], sDV[2], *origin);
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
void simplex2(double sDV[3], double V[13][3],
              double eVLA[4][3], double eVLB[4][3],
              int *numberSimplexPoints, int *origin) {

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

    if (dot_vec(V[0], sDV) < -ZERO) {
        printf("~~~~~ simplex 2: OA•sDV < -ZERO ~~~~~\n");
        printf("Minkowski hull does not contain Origin. Setting origin = 0\n");
        *origin = 0;
        printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
            sDV[0], sDV[1], sDV[2], *origin);
        return;
    }

    dif_vec(V[1], V[0], V[3]);   //AB = OB - OA
    dif_vec(V[2], V[0], V[4]);   //AC = OC - OA
    cross_vec(V[3], V[4], V[5]); //ABCn = AB X AC
    cross_vec(V[5], V[4], V[6]); //ACn = ABCn X AC
    cross_vec(V[3], V[5], V[7]); //ABn = AB X ABCn
    printf("OA = [%g, %g, %g]\n", V[0][0], V[0][1], V[0][2]);
    printf("OB = [%g, %g, %g]\n", V[1][0], V[1][1], V[1][2]);
    printf("OC = [%g, %g, %g]\n", V[2][0], V[2][1], V[2][2]);
    printf("AB = OB - OA = [%g, %g, %g]\n", V[3][0], V[3][1], V[3][2]);
    printf("AC = OC - OA = [%g, %g, %g]\n", V[4][0], V[4][1], V[4][2]);
    printf("ABCn = AB X AC = [%g, %g, %g]\n", V[5][0], V[5][1], V[5][2]);
    printf("ACn = ABCn X AC = [%g, %g, %g]\n", V[6][0], V[6][1], V[6][2]);
    printf("ABn = AB X ABCn = [%g, %g, %g]\n", V[7][0], V[7][1], V[7][2]);

    //if(OA•ABn < -ZERO)
    if(dot_vec(V[0], V[7]) < -ZERO) {
        printf("~~~~~ simplex 2: OA•ABn < -ZERO ~~~~~\n");
        cross_vec(V[0], V[3], V[8]); // OA X AB
        cross_vec(V[8], V[3], sDV); // (OA X AB) X AB
        printf("OA X AB = [%g, %g, %g]\n", V[8][0], V[8][1], V[8][2]);
        printf("searchDirectionVector = (OA X AB) X AB = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
        (*numberSimplexPoints)--;
        printf("~~~1~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
        return;
        // simplex1: [OA, OB]
    }
    //if(OA•ACn < -ZERO)
    else if(dot_vec(V[0], V[6]) < -ZERO) {
        printf("~~~~~ simplex 2: OA•ACn < -ZERO ~~~~~\n");
        cross_vec(V[0], V[4], V[8]); // OA X AC
        cross_vec(V[8], V[4], sDV); // (OA X AC) X AC
        printf("OA X AC = [%g, %g, %g]\n", V[8][0], V[8][1], V[8][2]);
        printf("searchDirectionVector = (OA X AC) X AC = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
        printf("Swap OB and OC\n");
        swap_vec(V[1], V[2]); // swap OB and OC
        swap_vec(eVLA[1], eVLA[2]);
        swap_vec(eVLB[1], eVLB[2]);
        (*numberSimplexPoints)--;
        printf("~~~2~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
        return;
        // swap results simplex1: [OA, OC]
    }
    else {
        double v = -dot_vec(V[0], V[5]); // -OA•ABCn
        if (v > ZERO) { // Set search direction to ABCn
            printf("~~~~~ simplex 2: -OA•ABCn > ZERO ~~~~~\n");
            sDV[0] = V[5][0];
            sDV[1] = V[5][1];
            sDV[2] = V[5][2];
            printf("searchDirectionVector = ABCn [%g, %g, %g]\n",
                    sDV[0], sDV[1], sDV[2]);
            printf("~~~3~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else if (v < -ZERO) { // Set search direction to -ABCn
            printf("~~~~~ simplex 2: -OA•ABCn < -ZERO ~~~~~\n");
            sDV[0] = -V[5][0];
            sDV[1] = -V[5][1];
            sDV[2] = -V[5][2];
            printf("searchDirectionVector = -ABCn [%g, %g, %g]\n",
                    sDV[0], sDV[1], sDV[2]);
            swap_vec(V[0], V[1]); // swap OA and OB
            swap_vec(eVLA[0], eVLA[1]);
            swap_vec(eVLB[0], eVLB[1]);
            printf("Swap OA and OB\n");
            //swap results simplex2: [OB, OA, OC]
            printf("~~~4~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else {
            printf("~~~5~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            *origin = 1;
            printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
                sDV[0], sDV[1], sDV[2], *origin);
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
void simplex3(double sDV[3], double V[13][3],
              double eVLA[4][3], double eVLB[4][3],
              int *numberSimplexPoints, int *origin) {
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

    if (dot_vec(V[0], sDV) < ZERO) {
        printf("~~~~~ simplex 3: OA•sDV < ZERO ~~~~~\n");
        printf("Minkowski hull does not contain Origin. Setting origin = 0\n");
        *origin = 0;
        printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
            sDV[0], sDV[1], sDV[2], *origin);
        return;
    }

    dif_vec(V[1], V[0], V[4]);   //AB = OB - OA
    dif_vec(V[2], V[0], V[5]);   //AC = OC - OA
    dif_vec(V[3], V[0], V[6]);   //AD = OD - OA
    cross_vec(V[4], V[5], V[7]); //ABCn = AB X AC
    cross_vec(V[5], V[6], V[8]); //ACDn = AC X AD
    cross_vec(V[6], V[4], V[9]); //ADBn = AD X AB

    printf("OA = [%g, %g, %g]\n",
        V[0][0], V[0][1], V[0][2]);
    printf("OB = [%g, %g, %g]\n",
        V[1][0], V[1][1], V[1][2]);
    printf("OC = [%g, %g, %g]\n",
        V[2][0], V[2][1], V[2][2]);
    printf("OD = [%g, %g, %g]\n",
        V[3][0], V[3][1], V[3][2]);
    printf("AB = OB - OA = [%g, %g, %g]\n",
        V[4][0], V[4][1], V[4][2]);
    printf("AC = OC - OA = [%g, %g, %g]\n",
        V[5][0], V[5][1], V[5][2]);
    printf("AD = OD - OA = [%g, %g, %g]\n",
        V[6][0], V[6][1], V[6][2]);
    printf("ABCn = AB X AC = [%g, %g, %g]\n",
        V[7][0], V[7][1], V[7][2]);
    printf("ACDn = AC X AD = [%g, %g, %g]\n",
        V[8][0], V[8][1], V[8][2]);
    printf("ADBn = AD X AB = [%g, %g, %g]\n",
        V[9][0], V[9][1], V[9][2]);

    //if(OA•ABCn < -ZERO) O outside face of TABC & tetrahedron
    if(dot_vec(V[0], V[7]) < -ZERO) {
        printf("~~~~~ simplex 3: OA•ABCn < -ZERO ~~~~~\n");
        printf("if(OA•ABCn < -ZERO) O outside face of TABC & tetrahedron\n");
        cross_vec(V[7], V[5], V[10]); //ACn = ABCn X AC
        cross_vec(V[4], V[7], V[11]); //ABn = AB X ABCn
        printf("ACn = ABCn X AC = [%g, %g, %g]\n",
            V[10][0], V[10][1], V[10][2]);
        printf("ABn = AB X ABCn = [%g, %g, %g]\n",
            V[11][0], V[11][1], V[11][2]);
        if(dot_vec(V[0], V[10]) < -ZERO) { //OA•ACn O outside edge AC in plane of TABC
            printf("~~~~~ simplex 3: OA•ACn < -ZERO ~~~~~\n");
            printf("OA•ACn O outside edge AC in plane of TABC\n");
            cross_vec(V[0], V[5], V[12]); //OA X AC
            cross_vec(V[12], V[5], sDV); //(OA X AC) X AC
            printf("OA X AC = [%g, %g, %g]\n",
                V[12][0], V[12][1], V[12][2]);
            printf("searchDirectionVector = (OA X AC) X AC = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            printf("Swap OB and OC\n");
            swap_vec(V[1], V[2]); //swap OB and OC
            swap_vec(eVLA[1], eVLA[2]);
            swap_vec(eVLB[1], eVLB[2]);
            //simplex1: [OA, OC]
            (*numberSimplexPoints)-=2;
            printf("~~~1~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else if(dot_vec(V[0], V[11]) < -ZERO) { //OA•ABn O outside edge AB in plane of TABC
            printf("~~~~~ simplex 3: OA•ABn < -ZERO ~~~~~\n");
            printf("OA•ABn O outside edge AB in plane of TABC\n");
            cross_vec(V[0], V[4], V[12]); //OA X AB
            cross_vec(V[12], V[4], sDV); //(OA X AB) X AB
            printf("OA X AB = [%g, %g, %g]\n",
                V[12][0], V[12][1], V[12][2]);
            printf("searchDirectionVector = (OA X AB) X AB = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            //simplex1: [OA, OB]
            (*numberSimplexPoints)-=2;
            printf("~~~2~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else { //ABCn above TABC
            printf("~~~~~ simplex 3: ABCn above TABC ~~~~~\n");
            sDV[0] = V[7][0];
            sDV[1] = V[7][1];
            sDV[2] = V[7][2];
            printf("searchDirectionVector = ABCn [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            //simplex2: [OA, OB, OC]
            (*numberSimplexPoints)--;
            printf("~~~3~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
    }
    //if(OA•ACDn < -ZERO) O outside face of TACD & tetrahedron
    else if(dot_vec(V[0], V[8]) < -ZERO) {
        printf("~~~~~ simplex 3: OA•ACDn < -ZERO ~~~~~\n");
        printf("if(OA•ACDn < -ZERO) O outside face of TACD & tetrahedron\n");
        cross_vec(V[8], V[6], V[10]); //ADn = ACDn X AD
        cross_vec(V[5], V[8], V[11]); //ACn = AC X ACDn
        printf("ADn = ACDn X AD = [%g, %g, %g]\n",
            V[10][0], V[10][1], V[10][2]);
        printf("ACn = AC X ACDn = [%g, %g, %g]\n",
            V[11][0], V[11][1], V[11][2]);
        if(dot_vec(V[0], V[10]) < -ZERO) { //OA•ADn O outside edge AD in plane of TACD
            printf("~~~~~ simplex 3: OA•ADn < -ZERO~~~~~ \n");
            printf("OA•ADn O outside edge AD in plane of TACD\n");
            cross_vec(V[0], V[6], V[12]); //OA X AD
            cross_vec(V[12], V[6], sDV); //(OA X AD) X AD
            printf("OA X AD = [%g, %g, %g]\n",
                V[12][0], V[12][1], V[12][2]);
            printf("searchDirectionVector = (OA X AD) X AD = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            printf("Swap OB and OD\n");
            swap_vec(V[1], V[3]); //swap OB and OD
            swap_vec(eVLA[1], eVLA[3]);
            swap_vec(eVLB[1], eVLB[3]);
            // simplex1: [OA, OD]
            (*numberSimplexPoints)-=2;
            printf("~~~4~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else if(dot_vec(V[0], V[11]) < -ZERO) { //OA•ACn O outside edge AC in plane of TACD
            printf("~~~~~ simplex 3: OA•ACn < -ZERO~~~~~ \n");
            printf("OA•ACn O outside edge AC in plane of TACD\n");
            cross_vec(V[0], V[5], V[12]); //OA X AC
            cross_vec(V[12], V[5], sDV); //(OA X AC) X AC
            printf("OA X AC = [%g, %g, %g]\n",
                V[12][0], V[12][1], V[12][2]);
            printf("searchDirectionVector = (OA X AC) X AC = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            printf("Swap OB and OC\n");
            swap_vec(V[1], V[2]); //swap OB and OC
            swap_vec(eVLA[1], eVLA[2]);
            swap_vec(eVLB[1], eVLB[2]);
            // simplex1: [OA, OC]
            (*numberSimplexPoints)-=2;
            printf("~~~5~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else { //ACDn Face of TACD toward O
            printf("~~~~~ simplex 3: ACDn Face of TACD toward O ~~~~~\n");
            sDV[0] = V[8][0];
            sDV[1] = V[8][1];
            sDV[2] = V[8][2];
            printf("searchDirectionVector = ACDn [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            printf("Swap OB and OC followed by swap OC and OD\n");
            swap_vec(V[1], V[2]); //swap OB and OC
            swap_vec(eVLA[1], eVLA[2]);
            swap_vec(eVLB[1], eVLB[2]);
            swap_vec(V[2], V[3]); //swap OC and OD
            swap_vec(eVLA[2], eVLA[3]);
            swap_vec(eVLB[2], eVLB[3]);
            // simplex2: [OA, OC, OD]
            (*numberSimplexPoints)--;
            printf("~~~6~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
    }
    //if(OA•ADBn < -ZERO) O outside face of TADB & tetrahedron
    else if(dot_vec(V[0], V[9]) < -ZERO) {
        printf("~~~~~ simplex 3: OA•ADBn < -ZERO ~~~~~\n");
        printf("if(OA•ADBn < -ZERO) O outside face of TADB & tetrahedron\n");
        cross_vec(V[9], V[4], V[10]); //ABn = ADBn X AB
        cross_vec(V[6], V[9], V[11]); //ADn = AD X ADBn
        printf("ABn = ADBn X AB = [%g, %g, %g]\n",
            V[10][0], V[10][1], V[10][2]);
        printf("ADn = AD X ADBn = [%g, %g, %g]\n",
            V[11][0], V[11][1], V[11][2]);
        if(dot_vec(V[0], V[10]) < -ZERO) { //OA•ABn O outside edge AB in plane of TADB
            printf("~~~~~ simplex 3: OA•ABn < -ZERO ~~~~~\n");
            printf("OA•ABn O outside edge AB in plane of TADB\n");
            cross_vec(V[0], V[4], V[12]); //OA X AB
            cross_vec(V[12], V[4], sDV); //(OA X AB) X AB
            printf("OA X AB = [%g, %g, %g]\n",
                V[12][0], V[12][1], V[12][2]);
            printf("searchDirectionVector = (OA X AB) X AB = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            // simplex1: [OA, OB]
            (*numberSimplexPoints)-=2;
            printf("~~~7~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else if(dot_vec(V[0], V[11]) < -ZERO) { //OA•ADn O outside edge AD in plane of TADB
            printf("~~~~~ simplex 3: OA•ADn < -ZERO ~~~~~\n");
            printf("OA•ADn O outside edge AD in plane of TADB\n");
            cross_vec(V[0], V[6], V[12]); //OA X AD
            cross_vec(V[12], V[6], sDV); //(OA X AD) X AD
            swap_vec(V[1], V[3]); //swap OB and OD
            swap_vec(eVLA[1], eVLA[3]);
            swap_vec(eVLB[1], eVLB[3]);
            printf("OA X AD = [%g, %g, %g]\n",
                V[12][0], V[12][1], V[12][2]);
            printf("searchDirectionVector = (OA X AD) X AD = [%g, %g, %g]\n",
                sDV[0], sDV[1], sDV[2]);
            printf("Swap OB and OD\n");
            // simplex1: [OA, OD]
            (*numberSimplexPoints)-=2;
            printf("~~~8~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
        else { //ADBn Face of TADB toward O
            printf("~~~~~ simplex 3: ADBn Face of TADB toward O ~~~~~\n");
            sDV[0] = V[9][0];
            sDV[1] = V[9][1];
            sDV[2] = V[9][2];
            printf("searchDirectionVector = ADBn [%g, %g, %g]\n", sDV[0], sDV[1], sDV[2]);
            printf("Swap OB and OD followed by swap OC and OD\n");
            swap_vec(V[1], V[3]); //swap OB and OD
            swap_vec(eVLA[1], eVLA[3]);
            swap_vec(eVLB[1], eVLB[3]);
            swap_vec(V[2], V[3]); //swap OC and OD
            swap_vec(eVLA[2], eVLA[3]);
            swap_vec(eVLB[2], eVLB[3]);
            // simplex2: [OA, OD, OB]
            (*numberSimplexPoints)--;
            printf("~~~9~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
            return;
        }
    }
    else {
        printf("~~~10~~~ numberSimplexPoints = %d\n", *numberSimplexPoints);
        printf("~~~~~ simplex 3: Inside the tetrahedron ~~~~~\n");
        *origin = 1;
        printf("searchDirectionVector = [%g, %g, %g], origin = %d\n",
            sDV[0], sDV[1], sDV[2], *origin);
        return;
    }
}

void cross_vec(double x[3], double y[3], double z[3]) {
    z[0] = x[1] * y[2] - x[2] * y[1];
    z[1] = x[2] * y[0] - x[0] * y[2];
    z[2] = x[0] * y[1] - x[1] * y[0];
}

void dif_vec(double x[3], double y[3], double z[3]) {
    z[0] = x[0] - y[0];
    z[1] = x[1] - y[1];
    z[2] = x[2] - y[2];
}

double dot_vec(double x[3], double y[3]) {
    return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

double norm_vec(double x[3]) {
    return (sqrt(dot_vec(x, x)));
}

void u_vec(double x[3], double y[3]) {
    double nx = norm_vec(x);
    y[0] = x[0]/nx;
    y[1] = x[1]/nx;
    y[2] = x[2]/nx;
}

void swap_vec(double x[3], double y[3]) {
    double z;
    z = x[0];
    x[0] = y[0];
    y[0] = z;
    z = x[1];
    x[1] = y[1];
    y[1] = z;
    z = x[2];
    x[2] = y[2];
    y[2] = z;
}

void SDMPY(double DO[4][3], double DI1[4][3], double DI2[4][3]) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            DO[i][j] = 0.0;
            for (int k=0; k<3; k++) {
                DO[i][j] += DI1[i][k] * DI2[k][j];
            }
        }
    }
    for (int i=0; i<3; i++) {
        DO[3][i] = DI1[3][i];
        for (int j=0; j<3; j++) {
            DO[3][i] += DI1[i][j] * DI2[3][j];
        }
    }
}
