/**
@file
@brief Defines a frame datastructure which represents a coordinate system.
@author Kristopher Wehage
@date 2016
@version 1.1
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
  frame->isVisible = false;
  return frame;
}

/*!
@brief Free a s2_frame_t datastructure
@param frame an s2_frame_t datastructure
*/
void s2_frame_free(s2_frame_t* frame) {

  if(frame) {
    s2_stackstr_free(frame->name);

    if(frame->dWRTReference == frame->dWRTOrigin) {
      if(frame->dWRTReference) {
        free(frame->dWRTReference);
        frame->dWRTReference = NULL;
        frame->dWRTOrigin = NULL;
      }
    } else {
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
    } else {
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
@brief load frame from json file.

For convenience, frames, i.e. rigid body displacements or shape matrices, may be
specified in a number of different ways in the input file.

  1. a 7 x 1 array with x, y, z, translation specified in the first three
  indices and orientation specified as a quaternion following the q.x, q.y,
  q.z, q.w convention (vector followed by scalar)
    "frames": [
      {
        "name": "A",
        "displacement": [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
      }
    ]

  2. a 4 x 4 Special Euclidian matrix specified in row-major order. Only the
  first three rows of the matrix will be read.

    "frames": [
      {
        "name": "B",
        "D":
          [[R.xx, R.xy, R.xz, T.x],
           [R.yx, R.yy, R.yz, T.y],
           [R.zx, R.zy, R.zz, T.z],
           [0,    0,    0,    1  ]]
      }
    ]

    or

    "frames": [
      {
        "name": "C",
        "displacement":
          [[R.xx, R.xy, R.xz, T.x],
           [R.yx, R.yy, R.yz, T.y],
           [R.zx, R.zy, R.zz, T.z],
           [0,    0,    0,    1  ]]
      }
    ]



  3. By specifiying rotation and translation separately in a 3x3 rotation
  matrix and 3x1 translation vector

    "frames": [
      {
        "name": "D",
        "T": [T.x, T.y, T.z],
        "R": [[R.xx, R.xy, R.xz],
              [R.yx, R.yy, R.yz],
              [R.zx, R.zy, R.zz]]
      }
    ]

    or

    "frames": [
      {
        "name": "E",
        "t": [T.x, T.y, T.z],
        "R": [[R.xx, R.xy, R.xz],
              [R.yx, R.yy, R.yz],
              [R.zx, R.zy, R.zz]]
      }
    ]

  4. By specifying translation and rotation separately in a 3x1 translation
  vector and a 4x1 quaternion

    "frames": [
      {
        "name": "F",
        "T": [T.x, T.y, T.z],
        "Q": [q.x, q.y, q.z, q.w]
      }
    ]

    or

    "frames": [
      {
        "name": "G",
        "t": [T.x, T.y, T.z],
        "q": [q.x, q.y, q.z, q.w]
      }
    ]

  5. By specifying translation as a 3x1 translation vector and rotation as a
  sequence of euler angles (specified in radians). If no sequence is specified,
  the XYZ sequence is used by default.

  "frames": [
    {
      "name": "H",
      "T": [T.x, T.y, T.z],
      "euler_angles": {
        "angles": [Rx, Ry, Rz]
      }
    }
  }

  or

  "frames": [
    {
      "name": "I",
      "t": [T.x, T.y, T.z],
      "euler_angles": {
        "sequence": "XYX",
        "angles": [Rx, Ry, Rx]
      }
    }
  }


*/
void s2_frame_load(s2_body_t* body, json_t* frameObject) {

  s2_frame_t* frame = s2_frame();

  frame->name = s2_stackstr_copy(body->name);
  s2_stackstr_append(frame->name,
                     json_string_value(json_object_get(frameObject, "name")));

  bool translationLoaded = false;
  bool rotationLoaded = false;

  /* try to load displacement field, (case 1 and case 2 b above) */
  if(!translationLoaded && !rotationLoaded) {
    json_t* displacementObject = json_object_get(frameObject, "displacement");
    if(displacementObject) {
      if(json_is_array(json_array_get(displacementObject, 0))) {
        /* case 2b, the displacement field contains a 4x4 matrix */
        for(int j = 0; j < 3; j++) {
          for(int k = 0; k < 4; k++) {
            s2_displacement_set(frame->dWRTOrigin, j, k,
                                json_number_value(json_array_get(
                                  json_array_get(displacementObject, j), k)));
          }
        }
      } else {
        /* case 1, the displacement field contains a 7 x 1 array */
        /* obtain 3x1 translation vector in first three positions of array */
        for(int i = 0; i < 3; i++) {
          s2_vector_set(
            &frame->dWRTOrigin->T, i,
            json_number_value(json_array_get(displacementObject, i)));
        }

        /* obtain 4x1 quaternion specified following [q.x, q.y, q.z, q.w]
         * convention in last four positions of array */
        s2_quaternion_t q;
        for(int i = 0; i < 4; i++) {
          s2_quaternion_set(
            &q, i,
            json_number_value(json_array_get(displacementObject, i + 3)));
        }
        s2_quaternion_normalize(&q);
        s2_quaternion_toMat3x3(&q, &frame->dWRTOrigin->R);
      }
      translationLoaded = true;
      rotationLoaded = true;
    }
  }

  /* try to load "D" field, case 2 a above */
  if(!rotationLoaded && !translationLoaded) {
    json_t* displacementObject = json_object_get(frameObject, "D");
    if(displacementObject) {
      /* case 2a, the "D"" field contains a 4x4 matrix */
      for(int j = 0; j < 3; j++) {
        for(int k = 0; k < 4; k++) {
          s2_displacement_set(frame->dWRTOrigin, j, k,
                              json_number_value(json_array_get(
                                json_array_get(displacementObject, j), k)));
        }
      }
      translationLoaded = true;
      rotationLoaded = true;
    }
  }

  /* attempt to load 3x3 rotation matrix specified in "R" field, case 3 above */
  if(!rotationLoaded) {
    json_t* rotation = json_object_get(frameObject, "R");
    if(rotation) {
      for(int j = 0; j < 3; j++) {
        for(int k = 0; k < 3; k++) {
          s2_displacement_set(
            frame->dWRTOrigin, j, k,
            json_number_value(json_array_get(json_array_get(rotation, j), k)));
        }
      }
      s2_gramSchmidt(3, 3, (double*)frame->dWRTOrigin);
      rotationLoaded = true;
    }
  }

  /* attempt to load 4x1 quaternion specified in "Q" or "q" field, case 4 above
   */
  if(!rotationLoaded) {
    json_t* quaternion = json_object_get(frameObject, "Q");
    if(!quaternion) {
      quaternion = json_object_get(frameObject, "q");
    }
    if(quaternion) {
      s2_quaternion_t q;
      for(int i = 0; i < 4; i++) {
        s2_quaternion_set(&q, i,
                          json_number_value(json_array_get(quaternion, i)));
      }
      s2_quaternion_normalize(&q);
      s2_quaternion_toMat3x3(&q, &frame->dWRTOrigin->R);
      rotationLoaded = true;
    }
  }

  /* attempt to load 3x1 translation vector specified in "T" or "t" field, cases
  3, 4 and 5 above */
  if(!translationLoaded) {
    json_t* translation = json_object_get(frameObject, "T");
    if(!translation) {
      translation = json_object_get(frameObject, "t");
    }
    if(translation) {
      for(int i = 0; i < 3; i++) {
        s2_vector_set(&frame->dWRTOrigin->T, i,
                      json_number_value(json_array_get(translation, i)));
      }
      translationLoaded = true;
    }
  }

  /* attempt to load rotation matrix as a sequence of euler angles, case 5
   * above*/
  if(!rotationLoaded) {
    json_t* euler_angles = json_object_get(frameObject, "euler_angles");
    if(euler_angles) {
      s2_axis_enum_t sequence = S2_AXIS_XYZ;
      json_t* sequenceObject = json_object_get(euler_angles, "sequence");
      if(sequenceObject) {
        sequence = s2_axis_enumFromChar(json_string_value(sequenceObject));
      }
      s2_vector_t angles = {0.0, 0.0, 0.0};
      json_t* angleObject = json_object_get(euler_angles, "angles");
      if(angleObject) {
        for(int i = 0; i < 3; i++) {
          s2_vector_set(&angles, i,
                        json_number_value(json_array_get(angleObject, i)));
        }
      } else {
        fprintf(stderr,
                "warning, no angles object specified for euler_angles field of "
                "frame: ");
        s2_stackstr_print(stderr, frame->name);
        fprintf(stderr, "  using {0.0, 0.0, 0.0} as angle sequence.\n");
      }
      s2_axis_getMat3x3(&angles, sequence, &frame->dWRTOrigin->R);
    }
  }

  /* validate rotation matrix, compute determinant and throw error if not close
   * to 1 */
  double det = s2_mat3x3_determinant(&frame->dWRTOrigin->R);
  if(fabs(det - 1) > 1e-2) {
    fprintf(stderr, "error loading frame: ");
    s2_stackstr_print(stderr, frame->name);
    fprintf(stderr, "rotation matrix has determinant: %g\n", det);
    exit(1);
  }

  s2_displacement_invert(frame->dWRTOrigin, frame->dWRTOriginInverse);

  // verify that inverse translations are computed correctly:
  // printf("rWRT: ");
  // kinematics_PrintVector(*(frame->rotationWRTOrigin) + 9);
  // printf("inverse: ");
  // kinematics_PrintVector(*(frame->rotationWRTOriginInverse) + 9);
  s2_body_appendFrame(body, frame);
}
