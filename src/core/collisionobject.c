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

#include "collisionobject.h"
#include "convexhull.h"

/*!
@brief generic collision object constructor
*/
s2_collisionobject_t* s2_collisionobject() {
  s2_collisionobject_t* collisionobject = malloc(sizeof(s2_collisionobject_t));
  collisionobject->type = S2_COLLISIONOBJECT_NONE;
  collisionobject->support = NULL;
  collisionobject->object = NULL;
  return collisionobject;
}


/*!
@brief generic collision object destructor

*/
void s2_collisionobject_free(s2_collisionobject_t* self) {
  switch(self->type) {
    case S2_COLLISIONOBJECT_CONVEXHULL: {
      s2_convexhull_free((s2_convexhull_t*)(self->object));
    } break;
    case S2_COLLISIONOBJECT_ELLIPSOID:
    case S2_COLLISIONOBJECT_SPHERE:
    case S2_COLLISIONOBJECT_OBB:
    case S2_COLLISIONOBJECT_AABB:
    case S2_COLLISIONOBJECT_POLYTOPE:
    case S2_COLLISIONOBJECT_NONE:
      break;
  }
  self->support = NULL;
  self->object = NULL;
  free(self);
}


/*!
@brief generates collision object from convex hull

*/
s2_collisionobject_t* s2_collisionobject_fromConvexHull(
  s2_convexhull_t* convexhull) {

  s2_collisionobject_t* collisionobject = s2_collisionobject();
  collisionobject->type = S2_COLLISIONOBJECT_CONVEXHULL;
  collisionobject->support = s2_convexhull_basicSupport;
  return collisionobject;
}


/*!
@brief collision object support function, wraps support function
for primitive collision object
*/
s2_vector_t s2_collisionobject_support(
  s2_collisionobject_t* self,
  s2_vector_t* direction) {
  return self->support(self->object, direction);
}
