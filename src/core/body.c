/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Body datastructure.
@date 2017
@copyright Kristopher Wehage 2016

This function library is used to initialize, free memory for, and operate on
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

#include "stack.h"
#include "body.h"
#include "inertia.h"
#include "assembly.h"
#include "frame.h"
#include "material.h"
#include "context.h"
#include "geometry.h"

#include <jansson.h>


/*!
@brief Append a frame datastructure to an s2_body_t datastructure

@param body a Body datastructure to append to
@param frame a Frame datastructure to append
*/
void s2_body_appendFrame(
  s2_body_t* body,
  s2_frame_t* frame) {

  if(!body->frameTail) {
    body->frameHead = frame;
    body->frameTail = frame;
  } else {
    body->frameTail->next = frame;
    body->frameTail = frame;
  }
}

/*!
@brief Initialize an s2_body_t datastructure. Appends the origin frame to the body frame

@param None

@returns a s2_body_t datastructure
*/
s2_body_t* s2_body() {
  s2_body_t* body = malloc(sizeof(s2_body_t));
  if (!body) {
    fprintf(stderr, "Error: failed to initialize body.\n");
    return NULL;
  }
  body->name = NULL;
  body->index = -1;
  body->level = -1;
  body->assembly = NULL;
  body->definition = NULL;
  body->isVirtual = false;
  body->frameHead = NULL;
  body->frameTail = NULL;
  body->referenceFrame = NULL;
  body->origin = NULL;
  body->frameCount = 0;
  body->inertia = s2_inertia();
  body->parentJointPointer = NULL;
  body->parentBodyPointer = NULL;
  body->next = NULL;
  body->materials = NULL;
  body->materialCount = 0;
  body->centerOfMassWRTOrigin = s2_vector();
  body->centerOfMassWRTReference = s2_vector();
  body->glReferenceFrame = NULL;
  body->glReferenceFramef = NULL;
  body->glModelView = NULL;
  body->glModelViewProjection = NULL;
  body->convexHullCount = 0;
  body->convexHulls = NULL;

  /* append origin to body */
  s2_frame_t* origin = s2_frame();
  origin->isOrigin = true;
  origin->name = s2_stackstr_copy(body->name);
  s2_stackstr_append(origin->name, "origin");
  body->origin = origin;
  body->referenceFrame = origin;
  s2_body_appendFrame(body, origin);

  return body;
}


/*!
@brief Append a body to an s2_assembly_t datastructure

@param assembly an Assembly datastructure to append to
@param body a Body datastructure to append
*/
void s2_body_append(
  s2_assembly_t* assembly,
  s2_body_t* body) {

  if (!assembly->bodyTail) {
    assembly->bodyHead = body;
    assembly->bodyTail = body;
  } else {
    assembly->bodyTail->next = body;
    assembly->bodyTail = body;
  }
}


/*!
@brief print frames in a s2_body_t datastructure
@param body a Body datastructure
*/
void s2_body_printFrames(
  FILE* f,
  s2_body_t* body) {

  s2_frame_t *frameHead = body->frameHead;
  while(frameHead) {
    fprintf(f, "\tFrame: ");
    s2_stackstr_print(f, frameHead->name);
    // printf("\t\tframePointer: %p\n", frameHead);
    fprintf(f, "\t\tdWRTOrigin:\n");
    s2_displacement_print(f, frameHead->dWRTOrigin);
    fprintf(f, "\t\tdWRTOriginInverse:\n");
    s2_displacement_print(f, frameHead->dWRTOriginInverse);
    fprintf(f, "\t\tdWRTReference:\n");
    s2_displacement_print(f, frameHead->dWRTReference);
    fprintf(f, "\t\trotationWRTReferenceInverse:\n");
    s2_displacement_print(f, frameHead->dWRTReferenceInverse);
    fprintf(f, "\t\trotationAbsolute:\n");
    s2_displacement_print(f, frameHead->dAbsolute);
    if(frameHead->velocityAbsolute) {
      fprintf(f, "\t\tvelocityAbsolute:\n");
      s2_screw_print(f, frameHead->velocityAbsolute);
    }
    if(frameHead->accelerationQuadraticAbsolute) {
      fprintf(f, "\t\taccelerationQuadraticAbsolute:\n");
      s2_screw_print(f, frameHead->accelerationQuadraticAbsolute);
    }
    if(frameHead->referenceFramePointer) {
      fprintf(f, "\t\tReference Frame: ");
      s2_stackstr_print(f, frameHead->referenceFramePointer->name);
    }
    fprintf(f, "\t\tisParent: %s\n", frameHead->isReference ? "True" : "False");
    frameHead = frameHead->next;
  }
}


/*!
@brief Free a s2_body_t datastructure
@param body an s2_body_t datastructure
*/
void s2_body_free(
  s2_body_t* body) {

  if(body) {
    s2_stackstr_free(body->name);
    if (body->definition) {
      s2_stackstr_free(body->definition);
    }
    s2_inertia_free(body->inertia);
    s2_materials_free(body->materials, body->materialCount);

    free(body->centerOfMassWRTOrigin);
    free(body->centerOfMassWRTReference);

    s2_frame_t* frameHead = body->frameHead;
    while(frameHead) {
      s2_frame_t* nextFrame = frameHead->next;
      s2_frame_free(frameHead);
      frameHead = nextFrame;
    }
    free(body);
    body = NULL;
  }
}


/*!
@brief Free a s2_body_t datastructure
@param body a virtual Body datastructure to free
*/
void s2_virtualbody_free(
  s2_body_t* body) {

  if(body) {
    s2_stackstr_free(body->name);
    if(body->definition) {
      s2_stackstr_free(body->definition);
    }
    s2_inertia_free(body->inertia);
    free(body->centerOfMassWRTOrigin);
    free(body->centerOfMassWRTReference);
    s2_frame_t* frameHead = body->frameHead;
    while(frameHead) {
      s2_frame_t* nextFrame = frameHead->next;
      s2_stackstr_free(frameHead->name);
      free(frameHead->dWRTOrigin);
      frameHead->dWRTOrigin = NULL;
      free(frameHead->dWRTOriginInverse);
      frameHead->dWRTOriginInverse = NULL;
      // if(frameHead->dAbsolute) free(frameHead->dAbsolute);
      // frameHead->dAbsolute = NULL;
      free(frameHead);
      frameHead = NULL;
      frameHead = nextFrame;
    }
    free(body);
    body = NULL;
  }
}


/*!
@brief load body to an assembly data structure
@return status: 0 indicates that body geometry was successfully loaded, -1 indicates failure in parsing geometry file, -2 indicates no body geometry was found, -3 indicates body is inactive and nothing was loaded
*/
int s2_body_load(
  s2_context_t* context,
  s2_assembly_t* assembly,
  s2_stackstr_t* prefix,
  json_t* bodyObject) {

  bool active = true;
  json_t* activeObject =
    json_object_get(bodyObject, "active");
  if(activeObject != NULL) {
    active = json_boolean_value(activeObject);
  }

  int return_val = -3;

  if(active) {

    s2_body_t* body = s2_body();
    body->assembly = assembly;

    /* get body descriptive name */
    const char* bodyRootName =
      json_string_value(json_object_get(bodyObject, "name"));

    body->name = s2_stackstr_copy(prefix);
    s2_stackstr_append(body->name, bodyRootName);

    /* get body definition files */
    json_t* definitionsObject =
      json_object_get(bodyObject, "definition");

    body->definition = s2_stackstr();

    if (json_is_null(definitionsObject)) {
      // printf("no definition files loaded\n");
    } else if(json_is_string(definitionsObject)) {
      s2_stackstr_append(
        body->definition,
        json_string_value(definitionsObject));
    } else if(json_is_array(definitionsObject)) {
      json_t* definitionObject;
      size_t i;
      json_array_foreach(definitionsObject, i, definitionObject) {
        s2_stackstr_append(
          body->definition,
          json_string_value(definitionObject));
      }
    }

    /* load body geometry */
    return_val = s2_body_loadGeometry(context, body);

    json_t* framesObject = json_object_get(bodyObject, "frames");
    if (!json_is_null(framesObject)) {
      json_t* frameObject;
      size_t frameIndex;

      json_array_foreach(framesObject, frameIndex, frameObject) {

        s2_frame_t* frame = s2_frame();

        frame->name = s2_stackstr_copy(body->name);
        s2_stackstr_append(
          frame->name,
          json_string_value(json_object_get(frameObject, "name")));

        json_t* rotation = json_object_get(frameObject, "R");
        if (json_is_array(rotation)) {
          for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
              ((double*)frame->dWRTOrigin)[3 * k + j] = json_number_value(
                  json_array_get(json_array_get(rotation, j), k));
            }
          }
          s2_gramSchmidt(
            3, 3, (double*)frame->dWRTOrigin);
        }
        json_t* translation = json_object_get(frameObject, "T");
        if (json_is_array(translation)) {
          for (int j = 0; j < 3; j++) {
            ((double*)frame->dWRTOrigin)[9 + j] =
                json_number_value(json_array_get(translation, j));
          }
        }
        s2_displacement_invert(
          frame->dWRTOrigin,
          frame->dWRTOriginInverse);

            // verify that inverse translations are computed correctly:
            // printf("rWRT: ");
            // kinematics_PrintVector(*(frame->rotationWRTOrigin) + 9);
            // printf("inverse: ");
            // kinematics_PrintVector(*(frame->rotationWRTOriginInverse) + 9);
            s2_frame_append(body, frame);
      }
    }

    /* get mass */
    body->inertia->m =
      json_number_value(json_object_get(bodyObject, "mass"));

    /*
      get body center of mass
    */
    for (int i = 0; i < 3; i++) {
      ((double*)body->centerOfMassWRTOrigin)[i] = json_number_value(
          json_array_get(json_object_get(bodyObject, "center_of_mass"), i));
    }
    s2_product_vk(
      body->centerOfMassWRTOrigin,
      &(body->inertia->m),
      &(body->inertia->mt));
    // printf("\ninertia loaded: ");
    // s2_vector_print(&(body->inertia->mt));
    // printf("\n");
    body->inertia->Ixx = json_number_value(
      json_object_get(
        json_object_get(bodyObject, "moment_of_inertia"), "Ixx"));
    body->inertia->Ixy = json_number_value(
      json_object_get(
        json_object_get(bodyObject, "moment_of_inertia"), "Ixy"));
    body->inertia->Izx = json_number_value(
      json_object_get(
        json_object_get(bodyObject, "moment_of_inertia"), "Izx"));
    body->inertia->Iyy = json_number_value(
      json_object_get(
        json_object_get(bodyObject, "moment_of_inertia"), "Iyy"));
    body->inertia->Iyz = json_number_value(
      json_object_get(
        json_object_get(bodyObject, "moment_of_inertia"), "Iyz"));
    body->inertia->Izz = json_number_value(
      json_object_get(
        json_object_get(bodyObject, "moment_of_inertia"), "Izz"));
    (context->bodyCount)++;
    s2_body_append(assembly, body);
  }
  return return_val;
}
