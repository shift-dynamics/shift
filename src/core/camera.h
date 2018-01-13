/**
@file
@author Kristopher Wehage
@date 2016
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
#ifndef _SHIFT_CAMERA_H_
#define _SHIFT_CAMERA_H_

#include "kinematics.h"

typedef struct s2_camera_t s2_camera_t;

typedef enum {
  S2_PROJECTION_PERSPECTIVE,
  S2_PROJECTION_ORTHOGRAPHIC
} s2_projection_enum_t;

typedef struct s2_camera_t {

  s2_projection_enum_t projectionType;

  float fieldOfView;
  float aspectRatio;
  float nearClippingPlane;
  float farClippingPlane;

  s2_mat4x4f_t view;
  s2_mat4x4f_t projection;

  bool isVisible;

  s2_camera_t* next;
} s2_camera_t;

void s2_camera_perspective(float fieldOfView, float aspectRatio,
                           float nearClippingPlane, float farClippingPlane,
                           s2_mat4x4f_t* perspective);

void s2_camera_orthographic(float left, float right, float bottom, float top,
                            float near, float far, s2_mat4x4f_t* orthographic);

s2_camera_t* s2_camera();
//
// void s2_glInitialize(
//   s2_context_t* context);
//
// GLuint s2_glShaderProgram(
//   const GLchar* vertexSource,
//   const GLchar* geometrySource,
//   const GLchar* fragmentSource);
//
// void s2_glCheckError(
//   const char* prefix);
//
// void s2_glRender(
//   s2_context_t* context);

#endif
