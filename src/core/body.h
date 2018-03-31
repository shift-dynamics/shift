/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Context datastructure.
@date 2016
@copyright Kristopher Wehage 2016

This function library is used to initialize and free memory for
data structures defined in body.h

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

#ifndef _SHIFT_BODY_H_
#define _SHIFT_BODY_H_

#include "kinematics.h"

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif
#include <jansson.h>

typedef struct s2_body_t s2_body_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_material_t s2_material_t;
typedef struct s2_inertia_t s2_inertia_t;
typedef struct s2_convexhull_t s2_convexhull_t;
typedef struct s2_context_t s2_context_t;
typedef struct s2_collisionobject_t s2_collisionobject_t;

#include "collisionobject.h"

/*!
@brief Describes properties related to a rigid body

A Body datastructure contains:
  - A descriptive name
  - A Frame linked list
  - Pointer to Frame defined as origin

The following attributes are assigned by the s2_ProcessTopology() function:
  - An index, which is used to identify position in minimal *run-time* arrays
  - A level, which indicates level in the spanning tree
  - The unique reference frame, which may not be the origin
*/
typedef struct s2_body_t {
  s2_stackstr_t* name; /*!< Descriptive name of body */

  s2_stackstr_t* definition; /*!< Path to files that define body definitions,
                                    i.e. obj and mtl files */

  s2_assembly_t* assembly; /*!< Pointer to assembly that body is a
                                  member of. */

  s2_frame_t* frameHead; /*!< head of Frame linked list */
  s2_frame_t* frameTail; /*!< tail of Frame linked list */

  s2_frame_t* referenceFrame; /*!< pointer to Body's unique reference frame,
                                    which  may not be the Frame's origin.
                                    Assigned automatically by
                                    s2_ProcessTopology */
  s2_frame_t* origin; /*!< A pointer to a body's unique *origin* coordinate
                             system, which may not be the reference frame */

  s2_joint_t* parentJointPointer; /*!< Pointer to Body's unique *parent*
                                         Joint datastructure */

  s2_body_t* parentBodyPointer; /*!< A pointer to the unique *parent* Body,
                                       set by s2_ProcessTopology */

  s2_frame_t* parentFramePointer; /*!< parent frame of body reference frame */
  s2_body_t* next; /*!< Pointer to next Body in linked list datastructure */

  // s2_material_t* materials;      /*!< s2_material_t datastructure */
  s2_body_t* geometryInstanceOf; /*!< if this body has identical geometry to
                              another body, this pointer is set to the original
                              body. Otherwise, if this geometry is unique to
                              this body or is the first instance of this
                              geometry, the pointer is set to NULL */
  s2_body_t* collisionGeometryInstanceOf; /*!< if this body has identical
                              collision geometry to another body, this pointer
                              is set to the original body. Otherwise, if the
                              collision geometry is unique to this body or is
                              the first instance of this collision geometry, the
                              pointer is set to NULL */

  int frameCount;    /*!< count of total number of frames */
  int index;         /*!< index in contiguous *run-time* array of Bodies, set
                          by s2_ProcessTopology */
  int level;         /*!< level in the spanning tree, set
                          by s2_ProcessTopology */
  int materialIndex; /*!< offset into context->materials */
  int materialCount; /*!< number of materials assigned to a body */
  bool isVirtual;    /*!< indicates whether the Body is *virtual*. All
                          user-defined bodies are not virtual. Massless
                          virtual bodies are added to simplify processing
                          kinematics if multiple loops are present in the
                          mechanism */
  s2_inertia_t* inertia;              /*!< centroidal Mass and moment of Inertia
                                             properties in origin coordinate system */
  s2_vector_t* centerOfMassWRTOrigin; /*!< vector from body origin to
                                             center of mass */
  s2_vector_t* centerOfMassWRTReference; /*!< vector from body reference
                                                frame to center of mass */
  int collisionObjectCount;
  int collisionObjectOffset; /*!< index to beginning of array of one or more
                       s2_collisionobject_t* datastructures stored in
                       s2_context_t datastructure, s2_collisionobject_t is the
                       level 3 interface to all primitive convex geometries */
  bool drawEnabled;
  bool isVisible;
  bool collisionObjectDrawEnabled;
} s2_body_t;

void s2_body_appendFrame(s2_body_t*, s2_frame_t*);

s2_body_t* s2_body(s2_stackstr_t*);

void s2_body_printFrames(FILE* f, s2_body_t* body);

void s2_body_append(s2_assembly_t* assembly, s2_body_t* body);

void s2_body_free(s2_body_t* body);

void s2_virtualbody_free(s2_body_t* body);

int s2_body_load(s2_context_t*, s2_assembly_t*, s2_stackstr_t*, json_t*);

int s2_body_loadGeometry(s2_context_t* context, s2_body_t* body);

#endif
