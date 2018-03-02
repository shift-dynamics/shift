/**
@file
@author Kristopher Wehage
@brief Utilities to initialize, free and operate on an s2_collisionobject_t
datastructure.
@copyright Kristopher Wehage 2016-2017

This function library is used to initialize, free and operate on
data structures defined in collisionobject.h

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

#ifndef _SHIFT_COLLISIONOBJECT_H_
#define _SHIFT_COLLISIONOBJECT_H_

#include "convexhull.h"
#include "ellipsoid.h"
#include "kinematics.h"
#include "sphere.h"

/*!
@brief supported collision object types
*/
typedef enum {
  S2_COLLISIONOBJECT_CONVEXHULL,
  S2_COLLISIONOBJECT_ELLIPSOID,
  S2_COLLISIONOBJECT_SPHERE,
  S2_COLLISIONOBJECT_OBB,  /*!< object-aligned bounding box */
  S2_COLLISIONOBJECT_AABB, /*!< axis-aligned bounding box */
  S2_COLLISIONOBJECT_NONE
} s2_collisionobject_enum_t;

/*!
@brief cached information for support mapping queries
*/
typedef struct s2_collisionobject_history_t {

  int extremeVertexIndex;    /*!< index of extreme vertex */
  s2_vector_t extremeVertex; /*!< extreme vertex */

} s2_collisionobject_history_t;

/*!
@brief generic interface to convex collision geometry
*/
typedef struct s2_collisionobject_t {
  bool displacementUpdated; /*!< flag to indicate whether displacement
                               submatrices need to be updated */
  s2_displacement_t
    dAbsolute; /*!< absolute displacement of collision geometry */
  s2_displacement_t dWRTReference;   /*!< relative displacement of collision
                                         geometry with respect to reference
                                         frame */
  s2_displacement_t* referenceFrame; /*! pointer to absolute displacement of
    collision object's reference frame, typically the body reference frame or
    the body origin */
  void* object; /*!< void pointer to collision geometry, which contains
                   underlying convex geometry used by s2_collisionobject_support
                   functions */
  s2_vector_t (*support)(
    void*, s2_vector_t*,
    s2_collisionobject_history_t*); /*!< function pointer to primitive support
                                       function*/
  int index; /*! unique index of collision geometry, currently not used */
  s2_collisionobject_enum_t type; /*!< collision object type, used for initial
                                     set up of a s2_collisionobject_t
                                     datstructure and for debugging
                                     purposes, generally it would not be
                                     necessary to check this enum at run-time */
  float color[3]; /*!< color of collision geometry, for display in GUI only */
} s2_collisionobject_t;

s2_vector_t s2_collisionobject_support(s2_collisionobject_t*, s2_vector_t*,
                                       s2_collisionobject_history_t*);

s2_collisionobject_t* s2_collisionobject_alloc(int count);

void s2_collisionobject_init(s2_collisionobject_t*);

void s2_collisionobject_free(s2_collisionobject_t*, int);

void s2_collisionobject_sphere(s2_context_t*, s2_body_t*);

void s2_collisionobject_convexhull(s2_context_t*, s2_body_t*);

void s2_collisionobject_ellipsoid(s2_context_t*, s2_body_t*);

s2_collisionobject_enum_t s2_collisionobject_enumFromChar(const char*);

#endif
