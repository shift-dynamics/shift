/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Body datastructure.
@date 2017
@copyright Kristopher Wehage 2016

This function library is used to initialize, free memory for, and operate on
data structures defined in sphere.h

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

#include <assimp/cimport.h> // Plain-C interface to assimp loader
#include <assimp/material.h>
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h>       // Output data structure
#include <float.h>
#include <jansson.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "assembly.h"
#include "body.h"
#include "collisionobject.h"
#include "context.h"
#include "convexhull.h"
#include "ellipsoid.h"
#include "factor.h"
#include "frame.h"
#include "inertia.h"
#include "material.h"
#include "pca.h"
#include "stack.h"
#include "utility.h"

int s2_ellipsoid(s2_ellipsoid_t*, s2_context_t*, s2_body_t*,
                 s2_collisionobject_t*);
void subtract_outerProduct(double*, const s2_vector_t*);
double norm_diff(const double* a, const double* b, const int n);
double max_a(const double* a, int* index, const int count);
void product_PDPt(const double*, const double*, double*, const int);
void product_ak(const double*, double, double*, const int);

/*!
@brief constructor for ellipsoid geometry
*/
s2_ellipsoid_t* s2_ellipsoid_alloc(int count) {
  s2_ellipsoid_t* self = calloc(count, sizeof(s2_ellipsoid_t));
  return self;
}

/*!
@brief destructor for sphere geometry
*/
void s2_ellipsoid_free(s2_sphere_t* self) {
  if(self) {
    free(self);
    self = NULL;
  }
}

/*!
@brief compute c = a - b
*/
inline void subtract_aa(const double* restrict a, const double* restrict b,
                        double* restrict c, const int n) {
  for(int i = 0; i < n; i++) {
    c[i] = a[i] - b[i];
  }
}

/*!
@brief compute the l2 norm of (a - b)

Computes norm(a - b)
*/
inline double norm_diff(const double* restrict a, const double* restrict b,
                        const int n) {
  double sum_sq = 0;
  for(int i = 0; i < n; i++) {
    double diff = a[i] - b[i];
    sum_sq += diff * diff;
  }
  return sqrt(sum_sq);
}

inline double max_a(const double* restrict a, int* max_index, const int n) {
  double max_value = -DBL_MAX;
  for(int i = 0; i < n; i++) {
    if(a[i] > max_value) {
      max_value = a[i];
      *max_index = i;
    }
  }
  return max_value;
}

/*!
@brief compute the product a * k -> b
where a and b are n x 1 arrays and k is a scalar
*/
inline void product_ak(const double* restrict a, double k, double* restrict b,
                       const int n) {
  for(int i = 0; i < n; i++) {
    b[i] = a[i] * k;
  }
}

/*! @brief compute product of Q * D * Q.transpose, where D is a diagonal nxn
matrix and Q is a row major ordered 4 x n matrix with ones in the last row.
Since the last row of Q consists entirely of ones, this optimized routine
does not carry out any matrix products with entries in that row

only the lower triangular portion of X is computed as it is a symmetric matrix
*/
inline void product_QDQt(const double* restrict Q, const double* restrict D,
                         double* restrict X, const int n) {
  for(int i = 0; i < 3; i++) {       /* row index in Q */
    for(int j = 0; j < i + 1; j++) { /* column index in Q^T */
      X[i * 4 + j] = 0;
      for(int k = 0; k < n; k++) {
        X[i * 4 + j] += Q[i * n + k] * D[k] * Q[j * n + k];
      }
    }
  }
  const int i = 3;
  for(int j = 0; j < i; j++) { /* column index in Q^T */
    X[i * 4 + j] = 0;
    for(int k = 0; k < n; k++) {
      X[i * 4 + j] += D[k] * Q[j * n + k];
    }
  }
  const int j = i;
  X[i * 4 + j] = 0;
  for(int k = 0; k < n; k++) {
    X[i * 4 + j] += D[k];
  }
}

/*! @brief compute product of P * D * P' -> X, where D is a diagonal
nxn matrix and P is a row major ordered 3 x n matrix

only the lower triangular portion of X is computed for this symmetric matrix
*/
inline void product_PDPt(const double* restrict P, const double* restrict D,
                         double* restrict X, const int n) {
  for(int i = 0; i < 3; i++) {       /* row index in Q */
    for(int j = 0; j < i + 1; j++) { /* column index in Q^T */
      X[i * 3 + j] = 0;
      for(int k = 0; k < n; k++) {
        X[i * 3 + j] += P[i * n + k] * D[k] * P[j * n + k];
      }
    }
  }
}

void print_A(const double* restrict A, const int m, const int n) {
  for(int i = 0; i < m; i++) {
    printf("   ");
    for(int j = 0; j < n; j++) {
      printf("%g ", A[i * m + j]);
    }
    printf("\n");
  }
}

/*!
computes diag(Q^T * inv(X) * Q)
*/
int product_QtXinvQ(const double* restrict Q, double* restrict X,
                    double* restrict m, const int n) {

  /* X is a symmetric 4x4 matrix in row major order, factor using ldlt
  factorization, the L and D matrices will be overwritten in X */
  int status = s2_factor_ldlt(X, 4);
  if(status) {
    return -1;
  }

  double q[4] = {0.0, 0.0, 0.0, 1.0};
  double y[4] = {0.0, 0.0, 0.0, 0.0};

  for(int i = 0; i < n; i++) { /* rows of Q^T */

    /* copy a row of Q^T to the stack in contiguous memory */
    for(int j = 0; j < 3; j++) {
      q[j] = Q[j * n + i];
    }

    /* compute quantity q^T * L^-T = x^T, equivalent to solving
    L^-1 * q = y */
    s2_factor_ldlt_forwardSubstitution(X, q, y, 4);

    /* compute term along diagonal of m */
    m[i] = y[0] * y[0] / X[0 * 4 + 0] + y[1] * y[1] / X[1 * 4 + 1] +
           y[2] * y[2] / X[2 * 4 + 2] + y[3] * y[3] / X[3 * 4 + 3];
  }

  return 0;
}

/*!
@computes the matrix operation A <- A - c * c^T
*/
inline void subtract_outerProduct(double* restrict A,
                                  const s2_vector_t* restrict c) {

  /* c * c^T =

    [cx * cx, cx * cy, cx * cz]
    [cy * cx, cy * cy, cy * cz]
    [cz * cx, cz * cy, cz * cz]

    A is symmetric, so only need to compute terms on and below diagonal
  */

  A[0 * 3 + 0] -= c->x * c->x;
  A[1 * 3 + 0] -= c->y * c->x;
  A[1 * 3 + 1] -= c->y * c->y;
  A[2 * 3 + 0] -= c->z * c->x;
  A[2 * 3 + 1] -= c->z * c->y;
  A[2 * 3 + 2] -= c->z * c->z;
}

/*!
@brief compute minimum bounding ellipsoid using the the Khachiyan Algorithm as
described in:

Moshtagh, Nima. "Minimum volume enclosing ellipsoid." Convex Optimization 111
(2005): 112.

This implementation has been optimized to avoid computing unecessary
intermediate matrix products.

Finds the ellipse equation in 'center form':
(x-c)^T * A * (x-c) = 1
Compute a 3x3 matrix 'A' and a 3 dimensional vector 'c' as the center
of the ellipse.

Solves the optimization problem:
minimize log(det(A))
subject to (P_i - c)^T * A * (P_i - c) <= 1

*/
int s2_ellipsoid_minimumBoundingEllipsoid(s2_ellipsoid_t* ellipsoid,
                                          float* restrict vertices,
                                          unsigned int vertexCount,
                                          s2_collisionobject_t* collisionobject,
                                          double tolerance) {

  unsigned int count = 0;
  double error = 1.0;

  /* Q is a 4xn matrix in row major order containing one homogeneous
  coordinate in the last column */
  double* Q = calloc(3 * vertexCount, sizeof(double));

  for(unsigned int i = 0; i < vertexCount; i++) {
    Q[0 * vertexCount + i] = (double)(vertices[i * 3 + 0]);
    Q[1 * vertexCount + i] = (double)(vertices[i * 3 + 1]);
    Q[2 * vertexCount + i] = (double)(vertices[i * 3 + 2]);
  }
  // FILE* f = fopen("vertices.dat", "w");
  // for(unsigned int i = 0; i < vertexCount; i++) {
  //   fprintf(f, "%g %g %g;\n", vertices[i * 3 + 0], vertices[i * 3 + 1],
  //           vertices[i * 3 + 2]);
  // }
  // fclose(f);

  // u is an nxn diagonal matrix initialized with 1 / vertexCount on the
  // diagonal
  double* u = calloc(vertexCount, sizeof(double));
  const double n_inverse = 1.0 / vertexCount;
  for(unsigned int i = 0; i < vertexCount; i++) {
    u[i] = n_inverse;
  }

  double* u_new = calloc(vertexCount, sizeof(double));

  double X[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double* m = calloc(vertexCount, sizeof(double));

  while(error > tolerance && count < 10000) {

    /* compute the product Q * diag(u) * Q^T -> X where
    Q is a 4xn matrix stored in row major order */
    product_QDQt(Q, u, X, vertexCount);

    /* this step computes the quantity:

    M = diag(Q^T * inv(X) * Q);

    X will be overwritten with its LDLT factors in this step
    m is the nx1 diagonal vector of an nxn matrix*/
    product_QtXinvQ(Q, X, m, vertexCount);

    int j = 0;
    double maximum = max_a(m, &j, vertexCount);
    double step_size = (maximum - 4) / (4 * (maximum - 1));

    /* u_new = (1 - step_size) * u */
    product_ak(u, 1.0 - step_size, u_new, vertexCount);
    u_new[j] += step_size;

    error = norm_diff(u_new, u, vertexCount);

    /* u = u_new */
    memcpy(u, u_new, vertexCount * sizeof(double));

    count++;
    // printf(" [%d], error: %g, maximum: %g, index: %d, step_size: %g\n",
    // count,
    //        error, maximum, j, step_size);
  }

  // printf("u: \n");
  // for(unsigned int i = 0; i < vertexCount; i++) {
  //   printf("  %g\n", u[i]);
  // }

  /* obtain center of ellipsoid
     c = P_(3 x n)* u_(n x 1);
  */
  s2_vector_t* c = &collisionobject->dWRTReference.T;
  memset(c, 0, sizeof(s2_vector_t));
  for(unsigned int i = 0; i < vertexCount; i++) {
    c->x += Q[0 * vertexCount + i] * u[i];
  }
  for(unsigned int i = 0; i < vertexCount; i++) {
    c->y += Q[1 * vertexCount + i] * u[i];
  }
  for(unsigned int i = 0; i < vertexCount; i++) {
    c->z += Q[2 * vertexCount + i] * u[i];
  }

  // printf("c: ");
  // s2_vector_print(stdout, c);

  /* compute the matrix A which defines an ellipsoid in quadric form

  where (x - c)^T * A * (x - c) = 1

  by solving:
  A = (1.0 / 3.0) *
       inv(P * diag(u) * P^T - (P * u) * (P * u)^T);

  A = (1.0 / 3.0) * inv(P * diag(u) * P^T - c * c')
  */

  double A[9];
  memset(A, 0, 9 * sizeof(double));

  /* A <- P * diag(u) * P^T */
  product_PDPt(Q, u, A, vertexCount);
  // printf("P * diag(u) * P^T :\n");
  // print_A(A, 3, 3);

  /* A <- A - c * c^T */
  subtract_outerProduct(A, c);
  // printf("P * diag(u) * P^T - c * c^T:\n");
  // print_A(A, 3, 3);

  /* A <- A^{-1} */
  s2_factor_symmetricMatrixInverse(A, 3);
  // printf("inv(P * diag(u) * P^T - c * c^T):\n");
  // print_A(A, 3, 3);

  /* A <- 1 / 3 * A */
  const double k = 1.0 / 3.0;
  for(int i = 0; i < 9; i++) {
    A[i] *= k;
  }
  // printf("(1 / 3) * inv(P * diag(u) * P^T - c * c^T):\n");
  // print_A(A, 3, 3);

  /* the matrix A can be decomposed into its scalar and rotation components
    as:

    (x - c)' * R' * S' * S * R (x - c) = 1
  */
  s2_pca_jacobi((s2_mat3x3_t*)A, &collisionobject->dWRTReference.R);
  // printf("D:\n");
  // print_A(A, 3, 3);
  ellipsoid->a = 1.0 / sqrt(A[0 * 3 + 0]);
  ellipsoid->b = 1.0 / sqrt(A[1 * 3 + 1]);
  ellipsoid->c = 1.0 / sqrt(A[2 * 3 + 2]);

  // printf("a: %g, b: %g, c: %g\n", ellipsoid->a, ellipsoid->b, ellipsoid->c);
  free(Q);
  free(u);
  free(u_new);
  free(m);
  return 0;
}

/*!
This is the s2_ellipsoid_t constructor.

*/
int s2_ellipsoid(s2_ellipsoid_t* self, s2_context_t* context, s2_body_t* body,
                 s2_collisionobject_t* collisionObject) {

  if(body->materialIndex != -1) {

    /* initialize memory and set support function */

    /* generate temporary convex hull */
    s2_convexhull_t* convexhull = s2_convexhull_alloc(1);
    s2_convexhull(convexhull, context, body);

    /* degenerate case with one vertex */
    if(convexhull->vertexCount == 1) {
      self->a = 0;
      self->b = 0;
      self->c = 0;

      collisionObject->dWRTReference.T =
        s2_pca_vectorFromFloatArrayAtIndex(convexhull->vertices, 0);
      return 0;
    }

    const double tolerance = 0.001;

    clock_t begin = clock();

    /* here, do your time-consuming job */

    s2_ellipsoid_minimumBoundingEllipsoid(self, convexhull->vertices,
                                          convexhull->vertexCount,
                                          collisionObject, tolerance);

    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("ellipsoid computed in %g seconds\n", time_spent);
    s2_convexhull_free(convexhull);
    return 0;
  }
  return -1;
}

/*!
@brief simple sphere support function
*/
void s2_ellipsoid_support(void* object, s2_collisionobject_t* collisionObject,
                          s2_vector_t* searchDirectionVectorGlobal,
                          s2_vector_t* extremeVertexLocal,
                          s2_vector_t* extremeVertexGlobal) {

  /* access the ellipsoid data structure */
  s2_ellipsoid_t* ellipsoid = (s2_ellipsoid_t*)object;
  s2_displacement_t* dAbsolute = &collisionObject->dAbsolute;

  /*search direction vector local coordinates*/
  s2_vector_t searchDirectionVectorLocal;
  s2_product_rvinverse(&dAbsolute->R, searchDirectionVectorGlobal,
                       &searchDirectionVectorLocal);
  printf("searchDirectionVectorLocal\n");
  s2_vector_print(stdout, &searchDirectionVectorLocal);

  double a = ellipsoid->a;
  double b = ellipsoid->b;
  double c = ellipsoid->c;
  /* temp changes until fixed */
  // a = 1.0;
  // b = 2.0;
  // c = 3.0;
  printf("a = %g, b = %g, c = %g\n", a, b, c);

  double au = a * searchDirectionVectorLocal.x;
  double bu = b * searchDirectionVectorLocal.y;
  double cu = c * searchDirectionVectorLocal.z;
  printf("au = %g, bu = %g, cu = %g\n", au, bu, cu);

  // double sf;
  // sf = 2000.0;

  double theta = atan2(bu, au);
  if(theta < 0.0) {
    theta = 6.283185307 + theta;
  }

  // theta = round(sf*theta)/sf;

  double ct = cos(theta);
  double st = sin(theta);
  printf("theta = %g, cos(theta) = %g, sin(theta) = %g\n", theta, ct, st);

  double phi;
  if(fabs(ct) > fabs(st)) {
    phi = atan2(au, cu * ct);
  } else {
    phi = atan2(bu, cu * st);
  }
  if(phi < 0.0) {
    phi = 3.141592654 + phi;
  }

  // phi = round(sf*phi)/sf;

  double cp = cos(phi);
  double sp = sin(phi);
  printf("phi = %g, cos(phi) = %g, sin(phi) = %g\n", phi, cp, sp);

  extremeVertexLocal->x = a * ct * sp;
  extremeVertexLocal->y = b * st * sp;
  extremeVertexLocal->z = c * cp;
  printf("Relative local coordinates of extreme point on convex hull\n");
  s2_vector_print(stdout, extremeVertexLocal);
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
  s2_product_dv(dAbsolute, extremeVertexLocal, extremeVertexGlobal);
  printf("Absolute global coordinates of extreme point on convex hull\n");
  s2_vector_print(stdout, extremeVertexGlobal);

  // double r[2];
  // double sn[2];
  // double cs[2];
  // r[0] = sqrt(au * au + bu * bu);
  // cs[0] = au / r[0];
  // sn[0] = bu / r[0];
  // if(fabs(cs[0]) > fabs(sn[0])) {
  //   r[1] = sqrt(au * au + cu * cu * cs[0] * cs[0]);
  //   cs[1] = cu * cs[0] / r[1];
  //   sn[1] = au / r[1];
  //   // phi = atan2( au, cu*ct );
  // } else {
  //   r[1] = sqrt(bu * bu + cu * cu * sn[0] * sn[0]);
  //   cs[1] = cu * sn[0] / r[1];
  //   sn[1] = bu / r[1];
  //   // phi = atan2( bu, cu*st );
  // }
  // printf("cos(theta) = %g, sin(theta) = %g\n", cs[0], sn[0]);
  // printf("cos(phi) = %g, sin(phi) = %g\n", cs[1], sn[1]);
  // extremeVertexLocal->x = a * ct * sp;
  // extremeVertexLocal->y = b * st * sp;
  // extremeVertexLocal->z = c * cp;
  // printf("VL: [%g, %g, %g]\n", copysign(a * cs[0] * sn[1], sDV[0]),
  //        copysign(b * sn[0] * sn[1], sDV[1]), copysign(c * cs[1], sDV[2]));

  return;
}
