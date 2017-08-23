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

/*
  This is the Convex Hull structure. It will contain everything needed to
  represent the local physical interaction between the convex hull it
  represents and other convex hulls. This includes everything needed to
  generate keywords from a direction vector and return a valid vertex
  from a hash table evaluator function. This structure is not complete
  and will be expanded as more model interactionm details evolve.
  Preprocessors will assign various arrays and functions to the pointer
  stubs when the convex hull is loaded into the model.
*/
struct CH {
  void (*e_Vertex)(struct CH *self, double uV[3], double P[3]);
  void (*hash)(const unsigned char *keyword);
  char keyword[10];
  char **keyword_list;
  int v_Index;
  double **vertex_list;
  double radius;
};

/*
  This is the Minkowski Convex Hull structure. It will contain some of
  what is needed to obtain the local physical interaction between the
  two convex hulls it points to. This structure is not complete and will
  be expanded as more model interaction details evolve. Preprocessors will
  assign various arrays and functions to the pointer stubs when the convex
  hulls are loaded into the model.

  The Minkowski Convex Hull structures are used in conjunction with
  determining the interaction between objects. Their contact detection
  algorithms will not function properly without pointers to the absolute
  displacement, veloxity, and force information for the bodies to which
  the convex hulls are attached. Next we describe OBJ object structures
  that contain this necessary data and OBJI object interaction structures.
*/
struct CMH {
  struct CH *HA;
  struct CH *HB;
  double V[20][3];
  int num_points;
  int origin;
};

/*
  This is the object structure for rigid bodies. It represents a rigid body
  whose reference frame is the fixed frame or the frame of a moving body.
  It contains various pointers to data arrays and funtions. When the convex
  hulls are attached to a body, they must have access to the body's absolute
  spatial displacement submatrix D and its absolute spatial velocity
  submatrix v. And they must have access to the fixed frame or body's spatial
  force submatrix g, allowing the contact forces stored in the spatial
  force submatrix f in the OBJ structure to be added to g. Adding f to g is
  reqired to incorporate all surface contact spatial forces into the MBD
  equations of motion. As we will see below, the object requires no knowledge
  of the convex hulls attached to the body.
*/
struct OBJ {
  /*
    For initial GJK debugging only a dummy spatial displacement submatrix D
    is provided. These will be changed to pointers later, and other pointers
    will be added.
  */
  //Absolute position of Object frame O relative to fixed frame f.
  double t_fO[3];
  //Absolute orientation of Object frame O relative to fixed frame f.
  double R_fO[3][3];
};

/*
  This is the rigid object interaction structure. It contains pointers to
  two OBJ structures OBJX and OBJY related to bodies X and Y. It's purpose
  is to support the processes of rapidly determining the proximity of
  bodies X and Y through some attached convex hulls, computing the spatial
  interference penetrations and velocities between other convex hulls, and
  computing the normal and tangential spatial forces acting on the
  contacting convex hulls. Let a specific OBJI structure OBJIXY contain
  pointers to the OBJ structures OBJX and OBJY. And let the respective X
  and Y bodies have n_x and n_y convex hulls attached to them. Now there
  will be n_x*n_y potential contacts between the n_x convex hulls comprising
  the OBJX object and the n_y convex hulls comprising the OBJY object. To
  represent these n_x*n_y potential contacts the OBJIXY structure needs an
  array of up to n_x*n_y pointers to CMH structures CMHAB_ij, and it needs
  pointers to arrays, functions, and structures required to perform proximity
  and contact detections and compute and store spatial forces between the
  various convex hull pairs on bodies X and Y. The individual CMH structures
  will get the necessary body information from the OBJ structures OBJX and
  OBJY and from the OBJI structure OBJIXY.
*/
struct OBJI {
  // Pointer to object structure for body X
  struct OBJ *OBJX;
  // Pointer to object structure for body Y
  struct OBJ *OBJY;
  /*
    For now each body has only one attached convex hull, so there is only
    one CMH structure pointer. We'll just call it CMH. Later this will
    become a pointer to an array of CMH structures.

    CMH is a convex Minkowski hull used to determine interference between
    convex hulls on bodies X and Y. When CMH becomes a pointer to an array
    of convex Minkowski hulls, n_xy <= n_x*n_y will indicate the maximum
    number of potential interference checks to be performed between the
    various convex hulls on objects OBJX and OBJY representing bodies X
    and Y. n_xy will be the number of CMH structures in the CMH array.
  */
  struct CMH *CMH;
  int n_xy;
};

/*
  e_Vertex() is the workhorse function of the GJK algorithm. In order to
  rapidly detect the interaction between physical objects in 3D space, each
  object may be represented by one or more convex hulls of different
  resolutions and complexity. The GJK algorithm requires rapid determination
  of an extreme point on each convex hull that it points to. For example, an
  object may have one or more simple convex hulls for verifying potential
  interaction with other objects. And an object may have zero or more
  general convex hulls and zero or more complex convex hulls for computing
  physical interaction between it and other objects. The simple convex hulls
  generally require simple e_Vertex() functions and the general convex hulls
  generally require more complex e_Vertex() functions. Each unique
  e_Vertex() function will have its own name and will be associated with
  a particular convex hull through its structure. Thus a preprocessor
  will set a pointer in the convex hull structure to the e_Vertex()
  function designed to handle it. For debugging purposes we use an
  e_Vertex function for a simple sphere.
*/
void e_Vertex(struct CH *CH, double uV[3], double P[3]) {
  P[0] = CH->radius * uV[0];
  P[1] = CH->radius * uV[1];
  P[2] = CH->radius * uV[2];
}

/*
  e_Vertex returns a new point in matrix P and add_point() inserts it into
  the simplex at CMH->V[1]. Before this point can be inserted the current
  points starting at CMH->V[1] must be shifted down by one to make room in
  CMH->V[1]. CMH->V[0] contains the direction vector and is not modified by
  add_point(). Adding a new point requires increasing CMH->num_points by 1.
*/
void add_point(struct CMH *CMH, double P[3]) {
  printf("In add_point: CMH->num_points = %d\n", CMH->num_points);
  for (int i = 1; i <= CMH->num_points; i++)
    for (int j = 0; j < 3; j++)
      CMH->V[i+1][j] = CMH->V[i][j];
  CMH->V[1][0] = P[0];
  CMH->V[1][1] = P[1];
  CMH->V[1][2] = P[2];
  CMH->num_points++;
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
  return sqrt(dot_vec(x, x));
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

/*
Each convex hull structure requires a hash table for rapid lookup of vertex
indices. The hash table contains a character-string key code for each
outward-facing unit vector that is normal to the hull at the respective
vertex. Each key code is obtained from a scaled, integer representation of
a vector's three coordinates, expressed in the hull's local coordinate frame,
with origin at the hull's centroid. If x, y, and z represent the three vector
coordinates and sf the scale factor, the corresponding integers are
obtained as ix = int(round(sf*x)), iy = int(round(sf*y)), and
iz = int(round(sf*z)). Then ix, iy, and iz are each converted to a character
string with a leading '+' or '-', and the three strings are concatenated
into a single key string. Each key entry in a hash table must be unique,
so extra vertex normal vectors that would yield the same key must be weeded
out. Only the vertex whose normal vector is most representative of the
surrounding region is retained.

The purpose of the hash table is to rapidly obtain an index to the vertex
whose outward-facing unit normal vector most closely matches a unit search
vector. A key code representation of the unit search vector is obtained
using the method described above. The key is entered into a hash table
function evaluator, which then returns an index to the desired vertex.

Each convex hull structure must also contain an inverse key table for
the hash table. When the input key associated with a search direction does
not match any key in the hash table, the hash-table function evaluator will
return a random index matching some key, which is not what we want. To verify
that a valid index has been returned, it is inserted into the inverse
key table, and the returned key is compared with the original key. If the
two keys agree then we know that the hash table function evaluator has
returned the correct vertex index. Otherwise the search vector's orientation
must be tweaked, a new key code genrated, and the new key sent back to
the hash table function evaluator.

The hash table generator creates a unique hash table function evaluator
for each convex hull. Each hash table function evaluator is given a unique
name, compiled, and linked with the program. Each convex hull structure
has a pointer to its hash table function evaluator.

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
*/

/*
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

/*
simplex0() returns an arbitrary point on the convex Minkowski difference
hull when a direction vector is not given, or it returns a particular
point on the convex hull when a direction vector is given. When a direction
vector is not given, simplex0() obtains point A in the Minkowski hull as
A = VB0 - VA0, where VA0 and VB0 are the first point in the respective convex
hulls HA and HB. The direction vector in this case will be taken as -A. When a
direction vector is given, simplex0() simply calls support(), which returns
point A. The GJK algorithm calls simplex0() once at the beginning. If A is
the origin of the convex Minkowski difference hull, the hulls HA and HB
intersect and the GJK algorithm terminates.
*/
void simplex0(struct CMH *CMH) {
  /*
    CMH->V[0] will contain the next direction vector used by support() to
    locate the next extreme vertex point P relative to the origin point O,
    represented by the vector OP.
    CMH->V[1] contains the vector OA, which locates vertex point A relative
    to the origin point O.
    CMH->origin == 0: The convex hull does not contain the origin.
    CMH->origin == 1: The convex hull contains the origin.
  */
  printf("In simplex0 norm CMH->V[1] = %lf\n", norm_vec(CMH->V[1]));
  if( norm_vec(CMH->V[1]) > 1e-6 ) {
    CMH->V[0][0] = -CMH->V[1][0];
    CMH->V[0][1] = -CMH->V[1][1];
    CMH->V[0][2] = -CMH->V[1][2];
    printf("In simplex0: CMH->origin = %d\n", CMH->origin);
    return;
  }
  CMH->origin = 1;
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
void simplex1(struct CMH *CMH) {

  /*
  The simplex1 line consists of points A and B and vector AB = B - A
  representing a line segment connecting points A and B. Let AO represent
  the vector from point A to the origin. Let dir = (0A X AB) X AB represent
  the direction orthogonal to the line and lying in a plane containing
  the origin. Drawing lines parallel to dir through points A and B divides
  the plane into three regions. Before entering simplex1() we have verified
  that the origin is not in the region off the end of the line near point B.
  Thus we have to check two other regions. First, if the magnitude of dir is
  near zero the origin lies on the line and is in the convex hull. Thus
  the convex hulls HA and HB intersect. Otherwise we must expand the simplex
  with another point by searching in the direction of dir.

  CMH->V[0] will contain the next direction vector used by support() to
  locate the next extreme vertex P relative to origin O, represented by OP.
  CMH->V[1] contains vector OA, which locates vertex A relative to O.
  CMH->V[2] contains vector OB, which locates vertex B relative to O.
  CMH->V[3] contains vector AB = OB - OA, which locates B relative to A.
  CMH->V[4] contains OA X AB
  CMH->V[0] = (OA X AB) X AB
  CMH->origin == 0: The convex hull does not contain the origin.
  CMH->origin == 1: The convex hull contains the origin.
  */
  dif_vec(CMH->V[2], CMH->V[1], CMH->V[3]);
  printf("Vector AB in Simplex1: [%lf, %lf, %lf]\n",
    CMH->V[3][0], CMH->V[3][1], CMH->V[3][2]);
  cross_vec(CMH->V[1], CMH->V[3], CMH->V[4]);
  printf("Vector 0A X AB in Simplex1: [%lf, %lf, %lf]\n",
    CMH->V[4][0], CMH->V[4][1], CMH->V[4][2]);
  cross_vec(CMH->V[4], CMH->V[3], CMH->V[0]);
  printf("Vector (0A X AB) X AB in Simplex1: [%lf, %lf, %lf]\n",
    CMH->V[0][0], CMH->V[0][1], CMH->V[0][2]);
  printf("In simplex1 norm CMH->V[0] = %lf\n", norm_vec(CMH->V[0]));
  if(norm_vec(CMH->V[0]) < 1.e-6) CMH->origin = 1;
  printf("In simplex1 CMH->origin = %d\n", CMH->origin);
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
void simplex2(struct CMH *CMH) {

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

  CMH->V[0] contains the direction vector used by support() to locate the
  next extreme vertex P relative to origin O, represented by OP.
  CMH->V[1] contains vector OA, which locates vertex A relative to O.
  CMH->V[2] contains vector OB, which locates vertex B relative to O.
  CMH->V[3] contains vector OC, which locates vertex C relative to O.
  CMH->V[4] contains vector AB = OB - OA, which locates B relative to A.
  CMH->V[5] contains vector AC = OC - OA, which locates C relative to A.
  CMH->V[6] contains vector ABCn = AB X AC, normal to TABC.
  CMH->V[7] contains vector ACn = ABCn X AC, tangent to TABC, normal to AC.
  CMH->V[8] contains vector ABn = AB X ABCn, tangent to TABC, normal to AB.
  CMH->V[9] contains vector OA X AB.
  CMH->V[0] = (OA X AB) X AB
  CMH->origin == 0: The convex hull does not contain the origin.
  CMH->origin == 1: The convex hull contains the origin.
  */

  printf("In simplex 2\n");
  dif_vec(CMH->V[2], CMH->V[1], CMH->V[4]);   //AB = OB - OA
  dif_vec(CMH->V[3], CMH->V[1], CMH->V[5]);   //AC = OC - OA
  cross_vec(CMH->V[4], CMH->V[5], CMH->V[6]); //ABCn = AB X AC
  cross_vec(CMH->V[6], CMH->V[5], CMH->V[7]); //ACn = ABCn X AC
  cross_vec(CMH->V[4], CMH->V[6], CMH->V[8]); //ABn = AB X ABCn

  //if(OA•ABn < -1e-6)
  if(dot_vec(CMH->V[1], CMH->V[8]) < -1e-6) {
    cross_vec(CMH->V[1], CMH->V[4], CMH->V[9]); // OA X AB
    cross_vec(CMH->V[9], CMH->V[4], CMH->V[0]); // (OA X AB) X AB
    swap_vec(CMH->V[1], CMH->V[2]); // swap OA and OB
    CMH->num_points--;
    return;
    // swap results simplex1: [OB, OA]
  }
  //if(OA•ACn < -1e-6)
  else if(dot_vec(CMH->V[1], CMH->V[7]) < -1e-6) {
    cross_vec(CMH->V[1], CMH->V[5], CMH->V[9]); // OA X AC
    cross_vec(CMH->V[9], CMH->V[5], CMH->V[0]); // (OA X AC) X AC
    swap_vec(CMH->V[1], CMH->V[2]); // swap OA and OB
    swap_vec(CMH->V[1], CMH->V[3]); // swap OB and OC
    CMH->num_points--;
    return;
    // swap results simplex1: [OC, OA]
  }
  else {
    double v = -dot_vec(CMH->V[1], CMH->V[6]); // -OA•ABCn
    if (v > 1e-6) { // Set search direction to ABCn
      CMH->V[0][0] = CMH->V[6][0];
      CMH->V[0][1] = CMH->V[6][1];
      CMH->V[0][2] = CMH->V[6][2];
      return;
    }
    else if (v < -1e-6) { // Set search direction to -ABCn
      CMH->V[0][0] = -CMH->V[6][0];
      CMH->V[0][1] = -CMH->V[6][1];
      CMH->V[0][2] = -CMH->V[6][2];
      swap_vec(CMH->V[1], CMH->V[3]); // swap OA and OC
      swap_vec(CMH->V[2], CMH->V[3]); // swap OB and OA
      // swap results simplex2: [OC, OA, OB]
      return;
    }
    else CMH->origin = 1;
    return;
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
void simplex3(struct CMH *CMH) {
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
  TACD, and TADB. A previous simplex2() has verified that the origin does
  not lie below TBCD, so we need to check that the origin does not lie outside
  the remaining triangles TABC, TACD, and TADB.

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

  CMH->V[0] contains the direction vector used by support() to locate the
  next extreme vertex P relative to origin O, represented by OP.
  CMH->V[1] contains vector OA, which locates vertex A relative to O.
  CMH->V[2] contains vector OB, which locates vertex B relative to O.
  CMH->V[3] contains vector OC, which locates vertex C relative to O.
  CMH->V[4] contains vector OD, which locates vertex D relative to O.
  CMH->V[5] contains vector AB = OB - OA, which locates B relative to A.
  CMH->V[6] contains vector AC = OC - OA, which locates C relative to A.
  CMH->V[7] contains vector AD = OD - OA, which locates D relative to A.
  CMH->V[8] contains vector ABCn = AB X AC, normal to TABC.
  CMH->V[9] contains vector ACDn = AC X AD, normal to TACD.
  CMH->V[10] contains vector ADBn = AD X AB, normal to TADB.

  For the triangle ABC
  CMH->V[11] contains vector ACn = ABCn X AC, tangent to TABC, normal to AC.
  CMH->V[12] contains vector ABn = AB X ABCn, tangent to TABC, normal to AB.
  CMH->V[13] contains vector OA X AC.
  CMH->V[0] = (OA X AC) X AC
  CMH->V[13] contains vector OA X AB.
  CMH->V[0] = (OA X AB) X AB

  For the triangle ACD
  CMH->V[11] contains vector ADn = ACDn X AD, tangent to TACD, normal to AD.
  CMH->V[12] contains vector ACn = AC X ACDn, tangent to TACD, normal to AC.
  CMH->V[13] contains vector OA X AD.
  CMH->V[0] = (OA X AD) X AD
  CMH->V[13] contains vector OA X AC.
  CMH->V[0] = (OA X AC) X AC

  For the triangle ABD
  CMH->V[11] contains vector ABn = ADBn X AB, tangent to TADB, normal to AB.
  CMH->V[12] contains vector ADn = AD X ADBn, tangent to TADB, normal to AD.
  CMH->V[13] contains vector OA X AC.
  CMH->V[0] = (OA X AB) X AB
  CMH->V[13] contains vector OA X AD.
  CMH->V[0] = (OA X AD) X AD

  CMH->origin == 0: The convex hull does not contain the origin.
  CMH->origin == 1: The convex hull contains the origin.
  */

  dif_vec(CMH->V[2], CMH->V[1], CMH->V[5]);   //AB = OB - OA
  dif_vec(CMH->V[3], CMH->V[1], CMH->V[6]);   //AC = OC - OA
  dif_vec(CMH->V[4], CMH->V[1], CMH->V[7]);   //AD = OD - OA
  cross_vec(CMH->V[5], CMH->V[6], CMH->V[8]); //ABCn = AB X AC
  cross_vec(CMH->V[6], CMH->V[7], CMH->V[9]); //ACDn = AC X AD
  cross_vec(CMH->V[7], CMH->V[5], CMH->V[10]); //ADBn = AD X AB

  //if(OA•ABCn < -1e-6)
  if(dot_vec(CMH->V[1], CMH->V[8]) < -1e-6) {
    cross_vec(CMH->V[8], CMH->V[6], CMH->V[11]); //ACn = ABCn X AC
    cross_vec(CMH->V[5], CMH->V[8], CMH->V[12]); //ABn = AB X ABCn
    if(dot_vec(CMH->V[1], CMH->V[11]) < -1e-6) { //OA•ACn
      cross_vec(CMH->V[1], CMH->V[6], CMH->V[13]); //OA X AC
      cross_vec(CMH->V[13], CMH->V[6], CMH->V[0]); //(OA X AC) X AC
      swap_vec(CMH->V[1], CMH->V[2]); //swap OA and OB
      swap_vec(CMH->V[1], CMH->V[3]); //swap OB and OC
      // swap results simplex1: [OC, OA]
      CMH->num_points-=2;
      return;
    }
    else if(dot_vec(CMH->V[1], CMH->V[12]) < -1e-6) { //OA•ABn
      cross_vec(CMH->V[1], CMH->V[5], CMH->V[13]); //OA X AB
      cross_vec(CMH->V[13], CMH->V[5], CMH->V[0]); //(OA X AB) X AB
      swap_vec(CMH->V[1], CMH->V[2]); //swap OA and OB
      // swap results simplex1: [OB, OA]
      CMH->num_points-=2;
      return;
    }
    else { //ABCn
      CMH->V[0][0] = CMH->V[8][0];
      CMH->V[0][1] = CMH->V[8][1];
      CMH->V[0][2] = CMH->V[8][2];
      swap_vec(CMH->V[1], CMH->V[3]); //swap OA and OC
      // swap results simplex2: [OC, OB, OA]
      CMH->num_points--;
      return;
    }
  }
  //if(OA•ACDn < -1e-6)
  else if(dot_vec(CMH->V[1], CMH->V[9]) < -1e-6) {
    cross_vec(CMH->V[9], CMH->V[7], CMH->V[11]); //ADn = ACDn X AD
    cross_vec(CMH->V[6], CMH->V[9], CMH->V[12]); //ACn = AC X ACDn
    if(dot_vec(CMH->V[1], CMH->V[11]) < -1e-6) { //OA•ADn
      cross_vec(CMH->V[1], CMH->V[7], CMH->V[13]); //OA X AD
      cross_vec(CMH->V[13], CMH->V[7], CMH->V[0]); //(OA X AD) X AD
      swap_vec(CMH->V[1], CMH->V[2]); //swap OA and OB
      swap_vec(CMH->V[1], CMH->V[4]); //swap OB and OD
      // swap results simplex1: [OD, OA]
      CMH->num_points-=2;
      return;
    }
    else if(dot_vec(CMH->V[1], CMH->V[12]) < -1e-6) { //OA•ACn
      cross_vec(CMH->V[1], CMH->V[6], CMH->V[13]); //OA X AC
      cross_vec(CMH->V[13], CMH->V[6], CMH->V[0]); //(OA X AC) X AC
      swap_vec(CMH->V[1], CMH->V[2]); //swap OA and OB
      swap_vec(CMH->V[1], CMH->V[3]); //swap OB and OC
      // swap results simplex1: [OC, OA]
      CMH->num_points-=2;
      return;
    }
    else { //ACDn
      CMH->V[0][0] = CMH->V[9][0];
      CMH->V[0][1] = CMH->V[9][1];
      CMH->V[0][2] = CMH->V[9][2];
      swap_vec(CMH->V[1], CMH->V[4]); //swap OA and OD
      swap_vec(CMH->V[2], CMH->V[3]); //swap OB and OC
      swap_vec(CMH->V[3], CMH->V[4]); //swap OB and OD
      // swap results simplex2: [OD, OC, OA]
      CMH->num_points--;
      return;
    }
  }
  //if(OA•ADBn < -1e-6)
  else if(dot_vec(CMH->V[1], CMH->V[10]) < -1e-6) {
    cross_vec(CMH->V[10], CMH->V[5], CMH->V[11]); //ABn = ADBn X AB
    cross_vec(CMH->V[7], CMH->V[10], CMH->V[12]); //ADn = AD X ADBn
    if(dot_vec(CMH->V[1], CMH->V[11]) < -1e-6) { //OA•ABn
      cross_vec(CMH->V[1], CMH->V[5], CMH->V[13]); //OA X AB
      cross_vec(CMH->V[13], CMH->V[5], CMH->V[0]); //(OA X AB) X AB
      swap_vec(CMH->V[1], CMH->V[2]); //swap OA and OB
      // swap results simplex1: [OB, OA]
      CMH->num_points-=2;
      return;
    }
    else if(dot_vec(CMH->V[1], CMH->V[12]) < -1e-6) { //OA•ADn
      cross_vec(CMH->V[1], CMH->V[7], CMH->V[13]); //OA X AD
      cross_vec(CMH->V[13], CMH->V[7], CMH->V[0]); //(OA X AD) X AD
      swap_vec(CMH->V[1], CMH->V[4]); //swap OA and OD
      swap_vec(CMH->V[2], CMH->V[4]); //swap OB and OA
      // swap results simplex1: [OD, OA]
      CMH->num_points-=2;
      return;
    }
    else { //ABDn
      CMH->V[0][0] = CMH->V[10][0];
      CMH->V[0][1] = CMH->V[10][1];
      CMH->V[0][2] = CMH->V[10][2];
      swap_vec(CMH->V[1], CMH->V[2]); //swap OA and OB
      swap_vec(CMH->V[2], CMH->V[3]); //swap OA and OC
      swap_vec(CMH->V[2], CMH->V[4]); //swap OC and OD
      // swap results simplex2: [OB, OD, OA]
      CMH->num_points--;
      return;
    }
  }
  else CMH->origin = 1;
  return;
}

/*
  Put each unique named function e_Vertex in the named file containing
  the hash table function. Each named e_Vertex funtion is associated
  with the corresponding convex hull through a pointer CH->e_Vertex().
  support() returns the local extreme point on convex hull CH.
  CH is the pointer to the current convex hull structure.
  V[0] is the current direction vector.
  dir = 1 directs support() to use V[0] as the search direction.
  dir = -1 directs support() to use -V[0] as the search direction.
  P[0] contains the local extreme point on convex hull A in the
  respective search direction.
*/
void support(struct CH *CH, double V[3], int dir, double P[3]){
  //e_Vertex() requires a unit direction vector so set it in uV.
  double uV[3];
  u_vec(V, uV);
  if(dir < 0) {
    uV[0] = -uV[0];
    uV[1] = -uV[1];
    uV[2] = -uV[2];
  }
  printf("In support()\n");
  if(dir < 0)
    printf("Search direction in HA: [%lf, %lf, %lf]\n", uV[0], uV[1], uV[2]);
  else
    printf("Search direction in HB: [%lf, %lf, %lf]\n", uV[0], uV[1], uV[2]);
  //P contains the local extreme point in the convex hull HA or HB
  CH->e_Vertex(CH, uV, P);
  if(dir < 0)
    printf("Extreme point in HA: [%lf, %lf, %lf]\n", P[0], P[1], P[2]);
  else
    printf("Extreme point in HB: [%lf, %lf, %lf]\n", P[0], P[1], P[2]);
  return;
}

int contact(struct OBJI *OBJIXY) {
  /*
    This function encloses the GJK algorithm.
    Return 1 (contact) when origin is in convex Minkowski hull.
    Return 0 (no contact) when origin is not in convex Minkowski hull.
    Return -1 when the GJK algorithm fails.
    Iteration count: limit iterations to 20 and bail out with failure -1
  */
  int numiters = 0;
  /*
    P[0] contains the current direction vector.
    P[1] contains the local extreme vector on convex hull A.
    P[2] contains the local extreme vector on convex hull B.
  */
  double P[3][3];
  /*
    For initial debugging, seed the initial point 0A as a unit
    vector pointing in the local x direction. This will be changed to a
    saved point 0A from the previous GJK step. In fact, the GJK
    algorithm returns with the previous point 0A already in CMH->V[1].
    In this case the following assignment step would not be necessary.
  */
  OBJIXY->CMH->V[0][0] = OBJIXY->CMH->V[1][0] = 0.0;
  OBJIXY->CMH->V[0][1] = OBJIXY->CMH->V[1][1] = 0.0;
  OBJIXY->CMH->V[0][2] = OBJIXY->CMH->V[1][2] = 1.0;
  /*
    Iterate until success or failure.
      return 1 => origin in convex hull and contact.
      return 0 => origin not in convex hull and no contact.
      return -1 => failure and no determination.
  */
  while (numiters < 20) {
    numiters++;
    printf("numiters = %d\n", numiters);
    /*
      support() returns the local extreme point on convex hull A.
      CMH->HA is the convex hull structure for convex hull A.
      CMH->V[0] is the current direction vector.
      -1 directs support() to use -CMH->V[0] as the search direction.
      P[0] contains the local extreme point on convex hull A in the
      -CMH->V[0] search direction.
    */
    support(OBJIXY->CMH->HA, OBJIXY->CMH->V[0], -1, P[0]);
    /*
      The local extreme point on convex hull A must be converted to global.
      The point must be rotated by CMH->HA.orient and translated by
      CMH->HA.center. Use P[2] as a temporary work array.
    */
    P[2][0] = OBJIXY->OBJX->R_fO[0][0]*P[0][0]
            + OBJIXY->OBJX->R_fO[0][1]*P[0][1]
            + OBJIXY->OBJX->R_fO[0][2]*P[0][2];
    P[2][1] = OBJIXY->OBJX->R_fO[1][0]*P[0][0]
            + OBJIXY->OBJX->R_fO[1][1]*P[0][1]
            + OBJIXY->OBJX->R_fO[1][2]*P[0][2];
    P[2][2] = OBJIXY->OBJX->R_fO[2][0]*P[0][0]
            + OBJIXY->OBJX->R_fO[2][1]*P[0][1]
            + OBJIXY->OBJX->R_fO[2][2]*P[0][2];
    P[0][0] = P[2][0] + OBJIXY->OBJX->t_fO[0];
    P[0][1] = P[2][1] + OBJIXY->OBJX->t_fO[1];
    P[0][2] = P[2][2] + OBJIXY->OBJX->t_fO[2];
    printf("Extreme point on CHA: [%lf, %lf, %lf]\n",
      P[0][0], P[0][1], P[0][2]);
    /*
      support() returns the local extreme point on convex hull B.
      CMH->HB is the convex hull structure for convex hull B.
      CMH->V[0] is the current direction vector.
      1 directs support() to use CMH->V[0] as the search direction.
      P[1] contains the local extreme point on convex hull B in the
      CMH->V[0] search direction.
    */
    support(OBJIXY->CMH->HB, OBJIXY->CMH->V[0], 1, P[1]);
    /*
      The local extreme point on convex hull B must be converted to global.
      The point must be rotated by CMH->Rb and translated by CMH->xb.
      Use P[2] as a temporary work array.
    */
    P[2][0] = OBJIXY->OBJY->R_fO[0][0]*P[1][0]
            + OBJIXY->OBJY->R_fO[0][1]*P[1][1]
            + OBJIXY->OBJY->R_fO[0][2]*P[1][2];
    P[2][1] = OBJIXY->OBJY->R_fO[1][0]*P[1][0]
            + OBJIXY->OBJY->R_fO[1][1]*P[1][1]
            + OBJIXY->OBJY->R_fO[1][2]*P[1][2];
    P[2][2] = OBJIXY->OBJY->R_fO[2][0]*P[1][0]
            + OBJIXY->OBJY->R_fO[2][1]*P[1][1]
            + OBJIXY->OBJY->R_fO[2][2]*P[1][2];
    P[1][0] = P[2][0] + OBJIXY->OBJY->t_fO[0];
    P[1][1] = P[2][1] + OBJIXY->OBJX->t_fO[1];
    P[1][2] = P[2][2] + OBJIXY->OBJY->t_fO[2];
    printf("Extreme point on CHB: [%lf, %lf, %lf]\n",
      P[1][0], P[1][1], P[1][2]);
    /*
      P[2] contains the extreme point on the Minkowski convex hull AB.
    */
    P[2][0] = P[1][0] - P[0][0];
    P[2][1] = P[1][1] - P[0][1];
    P[2][2] = P[1][2] - P[0][2];
    printf("Extreme point on CMHAB: [%lf, %lf, %lf]\n",
      P[2][0], P[2][1], P[2][2]);
    /*
      Function add_point() adds this new extreme point to the simplex.
      First it shifts the previous points down one level in CMH->V
      and inserts P[2] into CMH->V[0]. Then it increments CMH->num_points
      by 1.
    */
    add_point(OBJIXY->CMH, P[2]);
    printf("Verify new Extreme point in CMH: [%lf, %lf, %lf]\n",
      OBJIXY->CMH->V[0][0], OBJIXY->CMH->V[0][1], OBJIXY->CMH->V[0][2]);
    /*
      This first test is common to the simplex1, simplex2, and simplex3
      functions, so put it here once instead of in each simplex function.
      CMH->V[0] contains the vector 0A to the last point on the simplex.
      P[2] contains the vector 0P to the extreme point on the hull.
      If 0P•0A < -1e-6 the last point A on the simplex did not pass the
      origin so the convex Minkowski hull does not contain the origin.
    */
    if (OBJIXY->CMH->num_points > 1 && dot_vec(P[2], OBJIXY->CMH->V[0]) < -1e-6) {
      printf("Minkowski hull does not Contain Origin\n");
      printf("Number of iterations: %d\n", numiters);
      OBJIXY->CMH->origin = 0;
      return 0;
    }
    /*
      CMH->num_points can be 1, 2, 3, or 4 depending on the simplex
      dimension.
      CMH->num_points = 1: A single point is not useful unless it is the
      origin.
      CMH->num_points = 2: Two points define a line. The origin may be on
      the line or not. If not on the line, the origin will be somewhere
      around the line. Unless the origin is on the line, simplex1 cannot
      determine if the origin is or is not in the convex Minkowski hull.
      CMH->num_points = 3: Three points define a triangle. The origin may
      be on the triangle or not. If not on the triangle, the origin will
      be somewhere around the triangle. Unless the origin is on the triangle,
      simplex2 cannot determine if the origin is or is not in the convex
      Minkowski hull.
      CMH->num_points = 4: Four points define a tetrahedron. The origin may
      be on or in the tetrahedron or not. If on or in the tetrahedron, the
      origin will be in the convex Minkowski hull. Otherwise the origin will
      not be in the convex Minkowski hull. The final decision that the origin
      is not in the convex Minkowski hull can only be made in simplex3.
    */
    switch (OBJIXY->CMH->num_points) {
      case 1:
      printf("simplex0\n");
      simplex0(OBJIXY->CMH);
      break;
      case 2:
      printf("simplex1\n");
      simplex1(OBJIXY->CMH);
      break;
      case 3:
      printf("simplex2\n");
      simplex2(OBJIXY->CMH);
      break;
      case 4:
      printf("simplex3\n");
      simplex3(OBJIXY->CMH);
      break;
      default:
      printf("ERROR! Incorrect number of points: %d\n", OBJIXY->CMH->num_points);
    };
    if (OBJIXY->CMH->origin > 0) {
      printf("Contains Origin and Number of iterations: %d\n", numiters);
      return 1;
    }
  }
  printf("Number of iterations: %d\n", numiters);
  return -1;
}

/*
  For test purposes main() will repeatedly call contact(struct CMH *CMH) with
  a pointer to a Convex Hull Contact (CMH) structure. Before main() can call
  contact() it must set up two Convex Hull (CH) structures. For illustration
  purposes call these CH structures CHA and CHB. Then main() must set up one
  Convex Hull Contact (CMH) structure, which we'll call CMHAB.


*/
int main() {
  //The two convex hull structures.
  struct CH CHA;
  struct CH CHB;
  //The convex hull contact structure.
  struct CMH CMHAB;
  //The two object structures.
  struct OBJ OBJX;
  struct OBJ OBJY;
  //The object interaction structure.
  struct OBJI OBJIXY;
  double t_fO[3];
  //Absolute orientation of Object frame O relative to fixed frame f.
  double R_fO[3][3];

  //Spatial displacement of OBJX
  OBJX.t_fO[0] = 2.0;
  OBJX.t_fO[1] = 2.0;
  OBJX.t_fO[2] = 2.0;
  //Spatial rotation of OBJX
  OBJX.R_fO[0][0] = 1.0;
  OBJX.R_fO[0][1] = 0.0;
  OBJX.R_fO[0][2] = 0.0;
  OBJX.R_fO[1][0] = 0.0;
  OBJX.R_fO[1][1] = 1.0;
  OBJX.R_fO[1][2] = 0.0;
  OBJX.R_fO[2][0] = 0.0;
  OBJX.R_fO[2][1] = 0.0;
  OBJX.R_fO[2][2] = 1.0;
  //Spatial displacement of OBJY
  OBJY.t_fO[0] = -2.0;
  OBJY.t_fO[1] = -2.0;
  OBJY.t_fO[2] = -2.0;
  //Spatial rotation of OBJY
  OBJY.R_fO[0][0] = 1.0;
  OBJY.R_fO[0][1] = 0.0;
  OBJY.R_fO[0][2] = 0.0;
  OBJY.R_fO[1][0] = 0.0;
  OBJY.R_fO[1][1] = 1.0;
  OBJY.R_fO[1][2] = 0.0;
  OBJY.R_fO[2][0] = 0.0;
  OBJY.R_fO[2][1] = 0.0;
  OBJY.R_fO[2][2] = 1.0;
  //Number of points in the simplex.
  CMHAB.num_points = 0;
  //CMHAB.origin = 0: Assume that origin is not in the convex hull.
  //CMHAB.origin = 1: Know that origin is in the covex hull.
  CMHAB.origin = 0;
  //Assign convex hull A structure to the contact structure.
  CMHAB.HA = &CHA;
  //Assign convex hull B structure to the contact structure.
  CMHAB.HB = &CHB;
  //Assign object OBJX structure to the object interference OBJXY structure.
  OBJIXY.OBJX = &OBJX;
  //Assign object OBJY structure to the object interference OBJXY structure.
  OBJIXY.OBJY = &OBJY;
  //Assign Minkowski convex hull structure CMHAB to the OBJXY structure.
  OBJIXY.CMH = &CMHAB;
  /*
    For this initial debugging the e_Vertex function is the same for
    both convex hulls. Generally there may be a different e_Vertex
    function for each convex hull with different unique features and
    requirements. For example we may have e_Vertex_A and e_Vertex_B.
    Then a preprocessor would make assignments like
    CHA.e_Vertex = &e_Vertex_A; and CHB.e_Vertex = &e_Vertex_B; For
    now the assignments are made to the same e_Vertex() function here.
  */
  //Set pointer to e_Vertex for convex hull A.
  CHA.e_Vertex = &e_Vertex;
  //Set pointer to e_Vertex for convex hull B.
  CHB.e_Vertex = &e_Vertex;
  //Set radius in convex hull A structure.
  CHA.radius = 3.0;
  //Set radius in convex hull B structure.
  CHB.radius = 2.0;
  //Launch the GJK algorithm. i=0 => no contact; i = 1 => contact.
  int i = contact(&OBJIXY);
  printf("contact = %d\n", i);
}
