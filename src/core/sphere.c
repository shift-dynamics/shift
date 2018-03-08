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
#include "frame.h"
#include "inertia.h"
#include "material.h"
#include "pca.h"
#include "sphere.h"
#include "stack.h"
#include "utility.h"

void s2_sphere_approximateMinBoundSphere(s2_body_t*);

/*!
@brief constructor for sphere geometry
*/
s2_sphere_t* s2_sphere_alloc(int count) {
  s2_sphere_t* self = calloc(count, sizeof(s2_sphere_t));
  return self;
}

/*!
@brief destructor for sphere geometry
*/
void s2_sphere_free(s2_sphere_t* self) {
  free(self);
}

void s2_sphere_eigenSphere(s2_sphere_t* restrict s,
                           const float* restrict vertices,
                           const int vertexCount,
                           s2_collisionobject_t* collisionObject) {

  /* compute single prevision covariance matrix */
  s2_mat3x3f_t Mf = s2_pca_covariancef(vertices, vertexCount);

  s2_mat3x3_t M, v;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      s2_mat3x3_set(&M, i, j, s2_mat3x3f_get(&Mf, i, j));
    }
  }
  s2_pca_jacobi(&M, &v);

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
  s2_pca_extremePointsAlongDirection(&e, vertices, vertexCount, &imin, &imax);

  s2_vector_t min_vertex = s2_pca_vectorFromFloatArrayAtIndex(vertices, imin);
  s2_vector_t max_vertex = s2_pca_vectorFromFloatArrayAtIndex(vertices, imax);

  s2_vector_t temp;
  s2_subtract_vv(&max_vertex, &min_vertex, &temp);
  double dist = 0;
  dist += temp.x * temp.x;
  dist += temp.y * temp.y;
  dist += temp.z * temp.z;
  dist = sqrt(dist);
  s->radius = dist * 0.5;

  collisionObject->dWRTReference.T.x = (min_vertex.x + max_vertex.x) * 0.5;
  collisionObject->dWRTReference.T.y = (min_vertex.y + max_vertex.y) * 0.5;
  collisionObject->dWRTReference.T.z = (min_vertex.z + max_vertex.z) * 0.5;
}

/*!
@brief create a new sphere from existing sphere and point p. The sphere s is
updated (if needed) to just encompass the new point p
*/
void s2_sphere_fromSphereAndPoint(s2_sphere_t* s, s2_vector_t* p,
                                  s2_collisionobject_t* collisionObject) {
  s2_vector_t d;

  s2_subtract_vv(p, &collisionObject->dWRTReference.T, &d);
  double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
  if(dist2 > s->radius * s->radius) {
    double dist = sqrt(dist2);
    double newRadius = (s->radius + dist) * 0.5;
    double k = (newRadius - s->radius) / dist;
    s->radius = newRadius;
    s2_axpy_vv(k, &d, &collisionObject->dWRTReference.T);
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
int s2_sphere(s2_sphere_t* self, s2_context_t* context, s2_body_t* body,
              s2_collisionobject_t* collisionObject) {

  if(body->materialIndex != -1) {

    /* initialize memory and set support function */

    /* generate temporary convex hull */
    s2_convexhull_t* convexhull = s2_convexhull_alloc(1);
    s2_convexhull(convexhull, context, body);

    /* degenerate case with one vertex */
    if(convexhull->vertexCount == 1) {
      self->radius = 0;
      collisionObject->dWRTReference.T =
        s2_pca_vectorFromFloatArrayAtIndex(convexhull->vertices, 0);
      return 0;
    }

    s2_sphere_eigenSphere(self, convexhull->vertices, convexhull->vertexCount,
                          collisionObject);

    /* test if all points in bounding eigen sphere
    If all points inside sphere, we are done.
    Otherwise, enlarge bounding sphere to cover point outside of initial sphere
    */
    for(uint i = 1; i < convexhull->vertexCount; i++) {
      s2_vector_t p =
        s2_pca_vectorFromFloatArrayAtIndex(convexhull->vertices, i);
      s2_sphere_fromSphereAndPoint(self, &p, collisionObject);
    }

    s2_convexhull_free(convexhull);
    return 0;
  }
  return -1;
}

/*!
@brief simple sphere support function
@TODO this function is currently a stub and does nothing, it needs to
be populated to return the correct extreme vertex
*/
s2_vector_t s2_sphere_support(void* object,
                              s2_collisionobject_t* collisionObject,
                              s2_vector_t* direction,
                              s2_vector_t* extremeVertexLocal,
                              s2_vector_t* extremeVertexGlobal) {

  /* access quantities in the s2_sphere_t data structure */
  s2_sphere_t* sphere = (s2_sphere_t*)object;

  /*! @TODO currently, this function does nothing. Adding (void) to suppress
  unused compiler warning. */
  (void)sphere;
  (void)collisionObject;
  (void)direction;
  (void)extremeVertexLocal;
  (void)extremeVertexGlobal;

  // s2_sphere_t* sphere = (s2_sphere_t*)collisionobject;
  // s2_vector_normalize(direction);
  // s2_vector_t temp;
  // s2_product_vk(direction, &sphere->radius, &temp);
  s2_vector_t max_vector = {0.0, 0.0, 0.0};

  // s2_add_vv(&temp, &sphere->centerGlobal, &max_vector);
  return max_vector;
}
