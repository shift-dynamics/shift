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

#include "collisionobject.h"
#include "body.h"
#include "context.h"
#include "convexhull.h"
#include "frame.h"
#include "sphere.h"

/*!
@brief collision object constructor
*/
s2_collisionobject_t* s2_collisionobject_alloc(int count) {
  s2_collisionobject_t* collisionobjects =
    calloc(count, sizeof(s2_collisionobject_t));
  for(int i = 0; i < count; i++) {
    s2_collisionobject_init(&collisionobjects[i]);
  }
  return collisionobjects;
}

/*!
@brief initialize s2_collisionobject_t with default settings
*/
void s2_collisionobject_init(s2_collisionobject_t* self) {
  self->type = S2_COLLISIONOBJECT_NONE;
  self->displacementUpdated = false;
  self->support = NULL;
  self->color[0] = 194.f / 255.f;
  self->color[1] = 9.f / 255.f;
  self->color[2] = 226.f / 255.f;
}

/*!
@brief collision object destructor
*/
void s2_collisionobject_free(s2_collisionobject_t* collisionObjects,
                             int count) {
  for(int i = 0; i < count; i++) {
    switch(collisionObjects[i].type) {
      case S2_COLLISIONOBJECT_CONVEXHULL: {
        s2_convexhull_free((s2_convexhull_t*)(collisionObjects[i].object));
      } break;
      case S2_COLLISIONOBJECT_ELLIPSOID:
      case S2_COLLISIONOBJECT_SPHERE: {
        break;
      }
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
@brief get an s2_collisionobject_enum_t from a character string
*/
s2_collisionobject_enum_t
s2_collisionobject_enumFromChar(const char* collisionObjectType) {

  if(!strcmp(collisionObjectType, "convex_hull")) {
    return S2_COLLISIONOBJECT_CONVEXHULL;
  } else if(!strcmp(collisionObjectType, "ellipsoid")) {
    return S2_COLLISIONOBJECT_ELLIPSOID;
  } else if(!strcmp(collisionObjectType, "sphere")) {
    return S2_COLLISIONOBJECT_SPHERE;
  } else if(!strcmp(collisionObjectType, "obb")) {
    return S2_COLLISIONOBJECT_OBB;
  } else if(!strcmp(collisionObjectType, "aabb")) {
    return S2_COLLISIONOBJECT_AABB;
  } else {
    return S2_COLLISIONOBJECT_NONE;
  }
}

/*!
@brief append a single convex hull to encompass body, convex hull generated
using qhull library
*/
void s2_collisionobject_convexhull(s2_context_t* context, s2_body_t* body) {

  body->collisionObjectCount = 1;
  body->collisionObjects = s2_collisionobject_alloc(1);
  s2_collisionobject_t* collisionObject = &body->collisionObjects[0];
  collisionObject->type = S2_COLLISIONOBJECT_CONVEXHULL;
  collisionObject->support = s2_convexhull_basicSupport;
  collisionObject->referenceFrame = body->origin->dAbsolute;
  s2_displacement_setIdentity(&collisionObject->dAbsolute);
  s2_displacement_setIdentity(&collisionObject->dWRTReference);
  collisionObject->index = context->collisionObjectCount++;

  printf("loading collision object at index %d\n", collisionObject->index);
  s2_convexhull_t* convexhull = NULL;
  if(!body->collisionGeometryInstanceOf) {
    /* if collision geometry is not an instance, allocate memory for a new
    convex hull and append it to array of convexhull objects */
    context->convexhullCount++;
    if(!context->convexhulls) {
      context->convexhulls = malloc(sizeof(s2_convexhull_t));
    } else {
      context->convexhulls =
        realloc(context->convexhulls,
                context->convexhullCount * sizeof(s2_convexhull_t));
    }
    convexhull = context->convexhulls + context->convexhullCount - 1;
    convexhull->index = context->convexhullCount - 1;
    s2_convexhull(convexhull, context, body);
  } else {
    /* otherwise, set reference to original instance of collision object */
    if(!body->collisionGeometryInstanceOf->collisionObjects) {
      printf("NULL COLLISION OBJECT\n");
    }

    s2_collisionobject_t* instance =
      body->collisionGeometryInstanceOf->collisionObjects;
    convexhull = (s2_convexhull_t*)instance->object;
  }
  printf("   loaded convex hull at index %d\n", convexhull->index);
  collisionObject->object = convexhull;
}

/*!
@brief append a sphere collision object
*/
void s2_collisionobject_sphere(s2_context_t* context, s2_body_t* body) {
  body->collisionObjectCount = 1;
  body->collisionObjects = s2_collisionobject_alloc(1);
  s2_collisionobject_t* collisionObject = &body->collisionObjects[0];
  collisionObject->type = S2_COLLISIONOBJECT_SPHERE;
  collisionObject->support = s2_sphere_support;
  s2_displacement_setIdentity(&collisionObject->dAbsolute);

  context->sphereCount++;
  if(!context->spheres) {
    context->spheres = malloc(sizeof(s2_sphere_t));
  } else {
    context->spheres =
      realloc(context->spheres, context->sphereCount * sizeof(s2_sphere_t));
  }
  if(!body->collisionGeometryInstanceOf) {
    s2_sphere(&context->spheres[context->sphereCount - 1], context, body,
              collisionObject);
  } else {
    s2_collisionobject_t* instanceOf =
      &body->collisionGeometryInstanceOf->collisionObjects[0];
    collisionObject->index = instanceOf->index;
    collisionObject->object = &context->spheres[collisionObject->index];
  }
}

/*!
@brief append an ellipsoid collision object
*/
void s2_collisionobject_ellipsoid(s2_context_t* context, s2_body_t* body) {
  body->collisionObjectCount = 1;
  body->collisionObjects = s2_collisionobject_alloc(1);
  s2_collisionobject_t* collisionObject = &body->collisionObjects[0];
  collisionObject->type = S2_COLLISIONOBJECT_ELLIPSOID;
  collisionObject->support = s2_ellipsoid_support;
  s2_displacement_setIdentity(&collisionObject->dAbsolute);

  context->ellipsoidCount++;
  if(!context->ellipsoids) {
    context->ellipsoids = malloc(sizeof(s2_ellipsoid_t));
  } else {
    context->ellipsoids = realloc(
      context->ellipsoids, context->ellipsoidCount * sizeof(s2_ellipsoid_t));
  }
  s2_ellipsoid(&context->ellipsoids[context->ellipsoidCount - 1], context, body,
               collisionObject);
}

/*!
@brief collision object extreme vertex support function, wraps extreme vertex
support function defined in each primitive collision object
*/
s2_vector_t s2_collisionobject_support(s2_collisionobject_t* self,
                                       s2_vector_t* direction,
                                       s2_collisionobject_history_t* history) {
  return self->support((void*)self->object, direction, history);
}
