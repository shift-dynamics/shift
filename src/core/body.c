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

/*!
@brief Initialize a Body datastructure

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
