/**
@file
@author Kristopher Wehage
@brief Utilities for frame datastructure.
@date 2017
@copyright Kristopher Wehage 2016-2017

This function library is used to initialize and free memory for
data structures defined in context.h

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

#ifndef _STAR2_FRAME_H_
#define _STAR2_FRAME_H_

#include "kinematics.h"

#include <jansson.h>

typedef struct s2_frame_t s2_frame_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_stackstr_t s2_stackstr_t;


/*!
@brief Frame or coordinate system
*/
typedef struct s2_frame_t {
  s2_stackstr_t* name;

  s2_frame_t* referenceFramePointer; /*!< reference frame of the body
                                      set to null if bool isParent = true,
                                      i.e. this frame is the reference
                                      frame */

  s2_frame_t* next; /*!< next frame in linked list */
  s2_frame_t* childFrame;

  s2_displacement_t* dWRTOrigin; /*!< rotation with respect to origin */

  s2_displacement_t* dWRTOriginInverse; /*!< inverse of rotation with respect */

  s2_displacement_t* dWRTReference; /*!< displacement with respect to reference
                                           frame */

  s2_displacement_t* dWRTReferenceInverse; /*!< rotation with respect to
                                                  reference frame inverse */

  s2_displacement_t* dAbsolute; /*!< absolute rotation in inertial reference
                                       frame */

  s2_screw_t* velocityAbsolute; /*!< for a rigid body, the spatial velocity is
                                   constant for all frames on the body, for
                                   flexible bodies, the spatial velocity is
                                   not constant */
  s2_screw_t* accelerationQuadraticAbsolute; /*! for a rigid body, the spatial
                                   acceleration is constant for all frames on
                                   the body, for flexible bodies, the spatial
                                   acceleration is
                                   not constant */

  bool isReference; /*!< designates if the current frame is the reference
                        frame - or is the child of a Joint */

  bool isOrigin; /*!< designates whether the current frame is the origin */
  bool isParent; /*!< designates if the current frame is the parent of a joint */
  bool inTree; /*!< designates if the current frame is either the parent or */
              /*!< child of a joint */
  int index;
  int dIndex; /*!< index into d and dWRTReference arrays*/
  int vaIndex; /*!< index into velocity and acceleration arrays */
} s2_frame_t;


s2_frame_t* s2_frame();


void s2_frame_append(
  s2_body_t* body,
  s2_frame_t* frame);


void s2_frame_free(
  s2_frame_t* frame);

void s2_frame_load(
  s2_body_t*,
  json_t*);

#endif
