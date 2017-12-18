/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016-2017
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

#ifndef _SHIFT_MATERIAL_H_
#define _SHIFT_MATERIAL_H_

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

typedef struct s2_material_t s2_material_t;

typedef enum {
  S2_COLOR_AMBIENT,
  S2_COLOR_DIFFUSE,
  S2_COLOR_SPECULAR,
  S2_COLOR_EMISSIVE
} s2_color_enum_t;


typedef enum  {
  S2_SHADING_WIREFRAME,
  S2_SHADING_FLAT,
  S2_SHADING_GOURAUD,
  S2_SHADING_PHONG,
  S2_SHADING_LINE
} s2_shading_enum_t;


/*!
@brief data structure defining a material

*/
typedef struct s2_material_t {
  char* name;
  int index;
  char* filePath;
  s2_shading_enum_t shadingType;

  struct GLuniforms {
    GLfloat Ka[4]; /*!< ambient color properties */
    GLfloat Kd[4]; /*!< diffuse color properties */
    GLfloat Ks[4]; /*!< specular color properties */
    GLfloat Ke[4]; /*!< emissive color properties */
    GLint Ns; /*!< specular highlight */
  } GLuniforms;

  int vertexCount;
  int faceCount;

  GLfloat* vertices;
  GLfloat* vertexNormals;
  GLuint* faces;

  GLuint vertexBuffer; /*!< vertex buffer objects */
  GLuint vertexNormalBuffer;
  GLuint vao; /*!< vertex array objects */
  GLuint ibo; /*!< index buffer objects */

} s2_material_t;


s2_material_t* s2_materials(
  int materialCount);


void s2_materials_free(
  s2_material_t* materials,
  int materialCount);


#endif
