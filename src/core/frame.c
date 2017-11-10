#include "frame.h"
#include "body.h"
#include "stack.h"



/*!
@brief Initialize an s2_frame_t datastructure and set to identity

@param None
@returns an initialized s2_frame_t datastructure
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


/*!
@brief load frame from json file
*/
void s2_frame_load(
  s2_body_t* body,
  json_t* frameObject) {

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
  s2_body_appendFrame(body, frame);
}
