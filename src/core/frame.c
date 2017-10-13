#include "frame.h"
#include "body.h"
#include "stack.h"



/*!
@brief Initialize a Frame datastructure and set to identity

@param None
@returns a Frame datastructure
*/
s2_frame_t* s2_frame() {
  s2_frame_t* frame = malloc(sizeof(s2_frame_t));
  if(!frame) {
    fprintf(stderr, "Error: failed to initialize frame.\n");
    return NULL;
  }
  frame->name = NULL;

  frame->dWRTOrigin = s2_displacement();
  frame->dWRTOriginInverse = s2_displacement();
  frame->dAbsolute = s2_displacement();

  frame->dWRTReference = frame->dWRTOrigin;
  frame->dWRTReferenceInverse = frame->dWRTOriginInverse;

  frame->referenceFramePointer = NULL;
  frame->next = NULL;
  frame->isReference = false;
  frame->isOrigin = false;
  frame->inTree = false;

  frame->velocityAbsolute = NULL;
  frame->accelerationQuadraticAbsolute = NULL;
  return frame;
}


/*!
@brief Append a frame datastructure to an s2_body_t datastructure

@param body a Body datastructure to append to
@param frame a Frame datastructure to append
*/
void s2_frame_append(
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
@brief Free a s2_frame_t datastructure
@param frame an s2_frame_t datastructure
*/
void s2_frame_free(
  s2_frame_t* frame) {

  if(frame) {
    s2_stackstr_free(frame->name);

    if(frame->dWRTReference == frame->dWRTOrigin) {
      if(frame->dWRTReference) {
        free(frame->dWRTReference);
        frame->dWRTReference = NULL;
        frame->dWRTOrigin = NULL;
      }
    } else{
      if(frame->dWRTReference) {
        free(frame->dWRTReference);
        frame->dWRTReference = NULL;
      }
      if(frame->dWRTOrigin) {
        free(frame->dWRTOrigin);
        frame->dWRTOrigin = NULL;
      }
    }

    if(frame->dWRTReferenceInverse == frame->dWRTOriginInverse) {
      if(frame->dWRTReferenceInverse) {
        free(frame->dWRTReferenceInverse);
        frame->dWRTReferenceInverse = NULL;
        frame->dWRTOriginInverse = NULL;
      }
    } else{
      if(frame->dWRTReferenceInverse) {
        free(frame->dWRTReferenceInverse);
        frame->dWRTReferenceInverse = NULL;
      }
      if(frame->dWRTOriginInverse) {
        free(frame->dWRTOriginInverse);
        frame->dWRTOriginInverse = NULL;
      }
    }

    if(frame->dAbsolute) {
      free(frame->dAbsolute);
      frame->dAbsolute = NULL;
    }
    if(frame->velocityAbsolute) {
      free(frame->velocityAbsolute);
      frame->velocityAbsolute = NULL;
    }
    if(frame->accelerationQuadraticAbsolute) {
      free(frame->accelerationQuadraticAbsolute);
      frame->accelerationQuadraticAbsolute = NULL;
    }
    free(frame);
    frame = NULL;
  }
}
