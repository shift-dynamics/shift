/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage 2017
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

#ifndef _STAR2_LIGHT_H_
#define _STAR2_LIGHT_H_

#include <jansson.h>
#include <stdbool.h>

typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_context_t s2_context_t;

#ifdef __APPLE__
  #include <OpenGL/OpenGL.h>
#else
  #include <GL/gl.h>
#endif


/*!
@brief Light types
*/
typedef enum {
  S2_LIGHT_POINT,
  S2_LIGHT_DIRECTIONAL,
  S2_LIGHT_SPOT
} s2_light_enum_t;


/*!
@brief Defines a Light
*/
typedef struct s2_light_t {
  s2_stackstr_t* name; /*!< descriptive name of Light */
  s2_stackstr_t* parentFrame; /*!< name of frame that Light is attached to */
  s2_frame_t* parentFramePointer; /*!< pointer to frame that Light is
                                         attached to */
  s2_stackstr_t* parentBody; /*!< descriptive name of body that Light is
                                    attached to note, set to "fixed" for
                                    inertial reference frame */
  s2_body_t* parentBodyPointer; /*!< pointer to parent body that Light is
                                       light is attached to */
  int index;
  GLuint lightUniform;
  s2_light_enum_t lightType;
  struct glLight {
    GLfloat position[3];
    GLfloat Ka[3];
    GLfloat Kd[3];
    GLfloat Ks[3];
    int directional;
    GLfloat fov; /*!< field of view in degrees */
  } glLight;
  bool isVisible;
  struct s2_light_t* next;
} s2_light_t;


s2_light_t* s2_light();


void s2_light_free(s2_light_t* light);


void s2_light_load(
  s2_context_t*,
  s2_assembly_t*,
  s2_stackstr_t*,
  json_t*);

#endif
