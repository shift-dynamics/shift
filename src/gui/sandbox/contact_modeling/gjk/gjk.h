/**
@file
@author Roger Wehage
@brief Utilities to perform contact detection between convex hulls.
@date 2017
@copyright Roger Wehage 2017

This function library uses a gjk algorithm to detect potential or actual
contact between objects represented by convex hulls.

@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#ifndef _GJK_H_
#define _GJK_H_

typedef void (*ConvexHullSupport)()

/*
This is an example Convex Hull struct. A similar struct would be defined for
each unique convex hull representation. The struct serves three purposes. It
provides access to all fixed data required to represent the convex hull, it
stores one or more pointers to specialized functions required to process the
convex hull, and it stores changing historical data set and used by the
convex hull support function.

Each convex hull requires at least a support function. Three arguments are
passed to the support function, an input pointer to its Convex Hull struct,
an input double array of search-direction vector coordinates expressed in the
convex hull's local coordinate frame, and an output double array of extreme
surface vertex or point coordinates, also expressed in the convex hull's local
coordinate system.

To save operations, the search-direction vector is not normalized, so the
support function must normalize it if necessary. The contents of an instantiated
Convex Hull struct are saved between calls to its support function, so if any
information must be saved between calls to the support function, it will be
included in the struct definition.

When a the same convex hull surface is used to represent two or more objects,
the same instantiation of the Convex Hull struct will be used in each case.
When historical information must be saved in the struct between calls to the
support function, an indexed array large enough to hold the information for all
unique applications must be malloced.
*/

/*
Struct for a Convex Ellipsoid Hull.
The defining equation for a convex ellipsoid hull is
(x/a)^2+(y/b)^2+(z/c)^2 = 1
where a>0, b>0. c>0 are constants and x, y, and z are coordinates of the
extremePoint[3] on the ellipsoid's surface. The ellipsoid's local coordinate
frame axes align with its principal axes with origin at its geometrical center.
The elippsoid's parametric equations are
x = a*cos(theta)*sin(phi)
y = b*sin(theta)*sin(phi)
z = c*cos(phi)
where -pi <= theta <= pi and 0 <= phi <= pi
Coordinates of a non unit-normal vector at any the extremePoint[3] on the
ellipsoid are
n = [x/a^2, y/b^2, z/c^2]^T or
n = [cos(theta)*sin(phi)/a, sin(theta)*sin(phi)/b, cos(phi)/c]^T
Let s_x, s_y, s_z represent coordinates of the searchDirectionVector[3]
Aligning the searchDirectionVector with the normal vector yields
alpha*[s_x, s_y, s_z] = [cos(theta)*sin(phi)/a, sin(theta)*sin(phi)/b, cos(phi)/c]
where alpha > 0. These equations yield
theta = atan2(b*s_y, a*s_x)
and
if(abs(cos(theta)) > abs(sin(theta)))
    phi = atan2(a*s_x, c*s_z)
else
    phi = atan2(b*s_y, c*s_z)
where alpha has canceled out in the above atan2(alpha*y,alpha*x) expressions.
The atan2(y,x) function may return a negative value for phi, so
if(phi < 0.0) phi += pi
Support is a pointer to a specialized support function that returns an
extremePoint[3] on the convex hull surface when given a searchDirectionVector[3]
a, b, and c are the constant terms defining the ellipsoid's principal axes.
*/
struct EllipsoidHull {
    void (*SupportFunction)(struct ConvexEllipsoidHull *self,
      double searchDirectionVector[3],
      double extremePoint[3],
      int index);
    double a;
    double b;
    double c;
};

/*
This is the ConvexMinkowskiHull structure. It contains information needed to
detect local physical contact between two convex hulls whose instanced structs
are accessed by void pointers CHA and CHB. A preprocessor will assign
appropriate instantiated structs to these void pointers.
*/
typedef struct CMH {
    void* CHA;
    void* CHB;
    v
    double V[14][3];
    double VLA[4][3];
    double VLB[4][3];
}CMH;

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
two OBJ structures OBJX and OBJY related to bodies X and Y. Its purpose
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

void add_point(struct CMH *CMH, double P[3]);

void cross_vec(double x[3], double y[3], double z[3]);

void dif_vec(double x[3], double y[3], double z[3]);

double dot_vec(double x[3], double y[3]);

double norm_vec(double x[3]);

void u_vec(double x[3], double y[3]);

void swap_vec(double x[3], double y[3]);

void e_Vertex(struct CH *CH, double uV[3], double P[3]);

void ellipse(double uV[3], double P[3]);

void simplex0(struct CMH *CMH);

void simplex1(struct CMH *CMH);

void simplex2(struct CMH *CMH);

void simplex3(struct CMH *CMH);

void support(struct OBJ *OBJ, struct CH *CH, double V[3], int dir, double P[3]);

int contact(struct OBJI *OBJIXY);

#endif
