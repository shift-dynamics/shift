/**
@file
@author Kristopher Wehage
@brief Utilities to initialize, free and operate on an s2_collisionobject_t datastructure.
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

#include "kinematics.h"
#include "convexhull.h"
#include "ellipsoid.h"
#include "sphere.h"

/*!
@brief supported collision object types
*/
typedef enum {
  S2_COLLISIONOBJECT_CONVEXHULL,
  S2_COLLISIONOBJECT_ELLIPSOID,
  S2_COLLISIONOBJECT_SPHERE,
  S2_COLLISIONOBJECT_OBB, /*!< object-aligned bounding box */
  S2_COLLISIONOBJECT_AABB, /*!< axis-aligned bounding box */
  S2_COLLISIONOBJECT_NONE
} s2_collisionobject_enum_t;


/*!
@brief generic interface to convex collision geometry
*/
typedef struct s2_collisionobject_t {
  s2_displacement_t dAbsolute; /*!< absolute displacement of collision geometry */
  s2_collisionobject_enum_t type; /*!< collision object type */
  union {
    s2_convexhull_t convexHull;
    s2_ellipsoid_t ellipsoid;
    s2_sphere_t sphere;
  } object;
  s2_vector_t (*support)(void*, s2_vector_t*); /*!< support function pointer */
  float color[3];
} s2_collisionobject_t;

s2_vector_t s2_collisionobject_support(s2_collisionobject_t*, s2_vector_t*);

s2_collisionobject_t* s2_collisionobject(int count);

void s2_collisionobject_free(s2_collisionobject_t*, int);

void s2_collisionobject_sphere(s2_body_t*);

void s2_collisionobject_convexhull(s2_body_t*);

void s2_collisionobject_ellipsoid(s2_body_t*);
