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
#include "sphere.h"
#include "body.h"

/*!
@brief generic collision object constructor
*/
s2_collisionobject_t* s2_collisionobject(int count) {
  s2_collisionobject_t* collisionobjects =
    calloc(count, sizeof(s2_collisionobject_t));
  for(int i = 0; i < count; i++) {
    collisionobjects[i].type = S2_COLLISIONOBJECT_NONE;
    collisionobjects[i].support = NULL;
    collisionobjects[i].color[0] = 194.f / 255.f;
    collisionobjects[i].color[1] = 9.f / 255.f;
    collisionobjects[i].color[2] = 226.f / 255.f;
  }
  return collisionobjects;
}


/*!
@brief generic collision object destructor

*/
void s2_collisionobject_free(s2_collisionobject_t* collisionObjects, int count) {
  for(int i = 0; i < count; i++) {
    switch(collisionObjects[i].type) {
      case S2_COLLISIONOBJECT_CONVEXHULL: {
        s2_convexhull_free((s2_convexhull_t*)&(collisionObjects[i].object));
      } break;
      case S2_COLLISIONOBJECT_ELLIPSOID:
      case S2_COLLISIONOBJECT_SPHERE:
      case S2_COLLISIONOBJECT_OBB:
      case S2_COLLISIONOBJECT_AABB:
      case S2_COLLISIONOBJECT_NONE:
        break;
    }
    collisionObjects[i].support = NULL;
  }
  free(collisionObjects);
}


/*!
@brief creates a single convex hull to encompass body using qhull algorithm
*/
void s2_collisionobject_convexhull(s2_body_t* body) {
  body->collisionObjectCount = 1;
  body->collisionObjects = s2_collisionobject(1);
  s2_collisionobject_t* collisionObject = &body->collisionObjects[0];
  collisionObject->type = S2_COLLISIONOBJECT_CONVEXHULL;
  collisionObject->support = s2_convexhull_basicSupport;
  s2_convexhull_t* convexhull = (s2_convexhull_t*)&collisionObject->object;
  s2_convexhull(convexhull, body);
}


/*!
@brief create a single shere collision object
*/
void s2_collisionobject_sphere(s2_body_t* body) {
  body->collisionObjectCount = 1;
  body->collisionObjects = s2_collisionobject(1);
  s2_collisionobject_t* collisionObject = &body->collisionObjects[0];
  collisionObject->type = S2_COLLISIONOBJECT_SPHERE;
  collisionObject->support = s2_sphere_support;
  s2_sphere_t* sphere = (s2_sphere_t*)&collisionObject->object;
  s2_sphere(sphere, body);
}


/*!
@brief create a single ellipsoid collision object
*/
void s2_collisionobject_ellipsoid(s2_body_t* body) {
  body->collisionObjectCount = 1;
  body->collisionObjects = s2_collisionobject(1);
  s2_collisionobject_t* collisionObject = &body->collisionObjects[0];
  collisionObject->type = S2_COLLISIONOBJECT_ELLIPSOID;
  collisionObject->support = s2_ellipsoid_support;
  s2_ellipsoid_t* ellipsoid = (s2_ellipsoid_t*)&collisionObject->object;
  s2_ellipsoid(ellipsoid, body);
}


/*!
@brief collision object support function, wraps support function
for primitive collision object
*/
s2_vector_t s2_collisionobject_support(
  s2_collisionobject_t* self,
  s2_vector_t* direction) {
  return self->support((void*)&self->object, direction);
}
