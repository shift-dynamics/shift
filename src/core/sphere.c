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

#include <jansson.h>
#include <assimp/cimport.h>        // Plain-C interface to assimp loader
#include <assimp/scene.h>          // Output data structure
#include <assimp/postprocess.h>    // Post processing flags
#include <assimp/material.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <float.h>

#include "sphere.h"
#include "stack.h"
#include "context.h"
#include "body.h"
#include "material.h"
#include "convexhull.h"
#include "frame.h"
#include "inertia.h"
#include "assembly.h"
#include "utility.h"

void s2_sphere_approximateMinBoundSphere(s2_sphere_t*, s2_body_t*);


/*!
@brief constructor for sphere geometry
*/
s2_sphere_t* s2_sphere(s2_body_t* parentBody) {
  s2_sphere_t* sphere = calloc(1, sizeof(s2_sphere_t));
  sphere->parentBody = parentBody;
  return sphere;
}


/*!
@brief destructor for sphere geometry
*/
void s2_sphere_free(s2_sphere_t* sphere) {
  free(sphere);
  sphere = NULL;
}


/*!
@brief get a double precision vector from array of single precision vectors
stored in a float array
*/
inline s2_vector_t s2_vectorFromFloatArrayAtIndex(
  const GLfloat* vertices,
  const size_t index) {

  s2_vector_t vector;
  s2_vector_set(&vector, 0, (double)vertices[3 * index + 0]);
  s2_vector_set(&vector, 1, (double)vertices[3 * index + 1]);
  s2_vector_set(&vector, 2, (double)vertices[3 * index + 2]);
  return vector;
}


/*!
@brief compute squared distance at specified index of a single precision vector array
*/
inline float s2_squaredDistanceAtIndex(
  const GLfloat* vertices,
  const size_t index0,
  const size_t index1) {

  float sq_distance = 0;
  sq_distance += vertices[3 * index0 + 0] * vertices[3 * index1 + 0];
  sq_distance += vertices[3 * index0 + 1] * vertices[3 * index1 + 1];
  sq_distance += vertices[3 * index0 + 2] * vertices[3 * index1 + 2];
  return sq_distance;
}


/*!
@brief compute covariance matrix of point set
*/
void s2_covarianceMatrix(
  s2_mat3x3f_t* cov,
  const GLfloat* vertices,
  const size_t vertexCount) {

  float oon = 1.0f / (float)vertexCount;
  s2_vectorf_t c = {0.0f, 0.0f, 0.0f};


  /* compute center of mass */
  for(int i = 0; i < vertexCount; i++) {
    s2_vectorf_t* pt = (s2_vectorf_t*)(&vertices[i * 3]);
    s2_addequal_vvf(pt, &c);
  }
  s2_vectorf_scale(oon, &c);

  float e00 = 0.f;
  float e11 = 0.f;
  float e22 = 0.f;
  float e01 = 0.f;
  float e02 = 0.f;
  float e12 = 0.f;

  /* compute covariance elements */
  for(int i = 0; i < vertexCount; i++) {
    /* translate points so center of mass is at origin */
    s2_vectorf_t p;
    s2_vectorf_t* pt = (s2_vectorf_t*)(&vertices[i * 3]);
    s2_subtract_vvf(pt, &c, &p);

    /* compute covariance of translated points */
    e00 += p.x * p.x;
    e11 += p.y * p.y;
    e22 += p.z * p.z;
    e01 += p.x * p.y;
    e02 += p.x * p.z;
    e12 += p.y * p.z;
  }

  /* fill in covariance elements */
  s2_mat3x3f_set(cov, 0, 0, e00 * oon);
  s2_mat3x3f_set(cov, 1, 1, e11 * oon);
  s2_mat3x3f_set(cov, 2, 2, e22 * oon);

  float temp = e01 * oon;
  s2_mat3x3f_set(cov, 0, 1, temp);
  s2_mat3x3f_set(cov, 1, 0, temp);

  temp = e02 * oon;
  s2_mat3x3f_set(cov, 0, 2, temp);
  s2_mat3x3f_set(cov, 2, 0, temp);

  temp = e12 * oon;
  s2_mat3x3f_set(cov, 1, 2, temp);
  s2_mat3x3f_set(cov, 2, 1, temp);

}


/*!
@brief 2x2 symmetric schur decomposition
Given an nxn symmetric matrix, A, and indices p,q such that 1 <= p < q <= n,
compute a sine-cosine pair (s,c) that servers to form a Jacobi rotation matrix
*/
inline void s2_symSchur2(s2_mat3x3_t* A, int p, int q, double* cs) {
  if(fabs(s2_mat3x3_get(A, p, q)) > 1e-9) {
    double r = (s2_mat3x3_get(A, q, q) - s2_mat3x3_get(A, p, p)) /
               (2.0 * s2_mat3x3_get(A, p, q));
    double t;
    if(r >= 0.0) {
      t = 1.0f / (r + sqrt(1.0f + r * r));
    } else {
      t = -1.0f / (-r + sqrt(1.0f + r * r));
    }
    cs[0] = 1.0 / sqrt(1.0 + t * t);
    cs[1] = t * cs[0];
  } else {
    cs[0] = 1.0;
    cs[1] = 0.0;
  }
}


/*!
@brief compute eigenvectors and eigenvalues of a symmetric matrix A using
Jacobi method of iteratively updating A as A = J^T * A * J where J is a Jacobi
rotation matrix
*/
void s2_jacobi(s2_mat3x3_t* A, s2_mat3x3_t* v) {
  s2_mat3x3_setIdentity(v);
  s2_mat3x3_t J;
  s2_mat3x3_t temp;
  const int MAX_ITERATIONS = 50;
  int p = 0;
  int q = 1;
  double cs[2];
  double prev_norm = DBL_MAX;

  for(int n = 0; n < MAX_ITERATIONS; n++) {
    /* find largest off diagonal element */
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(i == j) {
          continue;
        }
        if(fabs(s2_mat3x3_get(A, i, j)) > fabs(s2_mat3x3_get(A, p, q))) {
          p = i;
          q = j;
        }
      }
    }

    /* compute jacobi rotation matrix */
    s2_symSchur2(A, p, q, cs);
    s2_mat3x3_setIdentity(&J);
    s2_mat3x3_set(&J, p, p, cs[0]);
    s2_mat3x3_set(&J, p, q, cs[1]);
    s2_mat3x3_set(&J, q, p, -cs[1]);
    s2_mat3x3_set(&J, q, q, cs[0]);

    s2_product_rr(v, &J, &temp);
    s2_mat3x3_copy(&temp, v);

    s2_product_rinverser(&J, A, &temp);
    s2_product_rr(&temp, &J, A);

    double norm = 0.0;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(i == j) {
          continue;
        } else {
          norm += s2_mat3x3_get(A, i, j) * s2_mat3x3_get(A, i, j);
        }
      }
    }

    if(n > 2 && norm >= prev_norm) {
      return;
    }

    prev_norm = norm;
  }
}


void s2_extremePointsAlongDirection(
  s2_vectorf_t* restrict e,
  const float* restrict vertices,
  const int vertexCount,
  int* restrict imin,
  int* restrict imax) {

  float max_projection2 = -FLT_MAX;
  float min_projection2 = FLT_MAX;
  for(int i = 0; i < vertexCount; i++) {
    float projection2 = 0;
    projection2 += e->x * vertices[i * 3 + 0];
    projection2 += e->y * vertices[i * 3 + 1];
    projection2 += e->z * vertices[i * 3 + 2];
    if(projection2 > max_projection2) {
      *imax = i;
      max_projection2 = projection2;
    }
    if(projection2 < max_projection2) {
      *imin = i;
      min_projection2 = projection2;
    }
  }
}


void s2_sphere_eigenSphere(
  s2_sphere_t* restrict s,
  const float* restrict vertices,
  const int vertexCount) {

  /* compute single prevision covariance matrix */
  s2_mat3x3f_t Mf;
  s2_covarianceMatrix(&Mf, vertices, vertexCount);

  s2_mat3x3_t M, v;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      s2_mat3x3_set(&M, i, j, s2_mat3x3f_get(&Mf, i, j));
    }
  }
  s2_jacobi(&M, &v);

  /* find component with largest eigenvalue */
  int maxc = 0;
  double maxf;
  double maxe = fabs(s2_mat3x3_get(&M, 0, 0));
  if((maxf = fabs(s2_mat3x3_get(&M, 1, 1))) > maxe) {
    maxc = 1;
    maxe = maxf;
  }
  if((maxf = fabs(s2_mat3x3_get(&M, 2, 2))) > maxe) {
    maxc = 2;
    maxe = maxf;
  }

  s2_vectorf_t e;
  for(int i = 0; i < 3; i++) {
    s2_vectorf_set(&e, i, (float)s2_mat3x3_get(&v, i, maxc));
  }

  int imin;
  int imax;
  s2_extremePointsAlongDirection(&e, vertices, vertexCount, &imin, &imax);

  s2_vector_t min_vertex = s2_vectorFromFloatArrayAtIndex(vertices, imin);
  s2_vector_t max_vertex = s2_vectorFromFloatArrayAtIndex(vertices, imax);

  s2_vector_t temp;
  s2_subtract_vv(&max_vertex, &min_vertex, &temp);
  double dist = 0;
  dist += temp.x * temp.x;
  dist += temp.y * temp.y;
  dist += temp.z * temp.z;
  dist = sqrt(dist);
  s->radius = dist * 0.5;
  s->centerLocal.x = (min_vertex.x + max_vertex.x) * 0.5;
  s->centerLocal.y = (min_vertex.y + max_vertex.y) * 0.5;
  s->centerLocal.z = (min_vertex.z + max_vertex.z) * 0.5;
}


/*!
@brief test for global collision between two spheres, requires that global centroid has been updated
*/
bool s2_sphere_collisionGlobal(s2_sphere_t* a, s2_sphere_t* b) {
  s2_vector_t d;
  s2_subtract_vv(&a->centerGlobal, &b->centerGlobal, &d);
  double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
  double radiusSum = a->radius + b->radius;
  return dist2 < radiusSum * radiusSum;
}


/*!
@brief test for local collision between two spheres. The "local" centroid is used for this test. Spheres must therefore be in the same "local" coordinate system
*/
bool s2_sphere_collisionLocal(s2_sphere_t* a, s2_sphere_t* b) {
  s2_vector_t d;
  s2_subtract_vv(&a->centerLocal, &b->centerLocal, &d);
  double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
  double radiusSum = a->radius + b->radius;
  return dist2 < radiusSum * radiusSum;
}


/*!
@brief update sphere's global centroid
*/
void s2_sphere_updateCentroid(s2_sphere_t* s) {
  if(!s->parentBody) {
    return;
  }

  /* transform centroid coordinates to global reference frame */
  s2_product_dv(
    s->parentBody->origin->dAbsolute, &s->centerLocal, &s->centerGlobal);

}


/*!
@brief create a new sphere from existing sphere and point p. The sphere s is updated (if needed) to just encompass the new point p
*/
void s2_sphere_fromSphereAndPoint(s2_sphere_t* s, s2_vector_t* p) {
  s2_vector_t d;
  s2_subtract_vv(p, &s->centerLocal, &d);
  double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
  if(dist2 > s->radius * s->radius) {
    double dist = sqrt(dist2);
    double newRadius = (s->radius + dist) * 0.5;
    double k = (newRadius - s->radius) / dist;
    s->radius = newRadius;
    s2_axpy_vv(k, &d, &s->centerLocal);
  }
}


/*!
Find a bounding sphere for a set of M points in
3-dimensional space using Ritter's algorithm. In 3D, the resulting sphere
is approximately 5% bigger than the optimal minimum radius sphere.

@param radius and centroid of bounding sphere returned in
@param convexhull, computing using s2_convexhull() defined in convexhull.h

REFERENCES:
[1] Ritter, J. (1990), 'An efficient bounding sphere', in Graphics Gems,
    A. Glassner, Ed. Academic Press, pp.301303
*/
void s2_sphere_approximateMinBoundSphere(
  s2_sphere_t* sphere,
  s2_body_t* body) {

  sphere->parentBody = body;
  s2_convexhull_t* convexhull = &body->convexHulls[0];
  if(convexhull) {
    if(convexhull->vertexCount == 1) {
      sphere->radius = 0;
      sphere->centerLocal = s2_vectorFromFloatArrayAtIndex(
                              convexhull->vertices,
                              0);
      return;
    }

    s2_sphere_eigenSphere(
      sphere,
      convexhull->vertices,
      convexhull->vertexCount);

    /* test if all points in bounding eigen sphere
    If all points inside sphere, we are done.
    Otherwise, enlarge bounding sphere to cover point outside of initial sphere */
    for(int i = 1; i < convexhull->vertexCount; i++) {
      s2_vector_t p = s2_vectorFromFloatArrayAtIndex(convexhull->vertices, i);
      s2_sphere_fromSphereAndPoint(sphere, &p);
    }
  }
}
