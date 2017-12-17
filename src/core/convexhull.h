/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a convex hull datastructure.
@copyright Kristopher Wehage 2016-2017

This function library is used to initialize and free memory for
data structures defined in s2_convexhull_t
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

#ifndef _STAR2_CONVEXHULL_H_
#define _STAR2_CONVEXHULL_H_

#include <stdarg.h>
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

#include "kinematics.h"

typedef struct s2_body_t s2_body_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_material_t s2_material_t;
typedef struct s2_inertia_t s2_inertia_t;


/*!
@brief convex hull data structure contains vertices and faces in contiguous blocks of memory
*/
typedef struct s2_convexhull_t {
  GLfloat* vertices; /*!< vertices in contiguous memory */
  int vertexCount; /*!< number of vertices */
  GLuint* faces; /*!< faces in contiguous memory */
  int faceCount; /*!< number of faces */
  bool drawEnabled; /*!< indicates whether the convexhull object should be drawin in gui */
} s2_convexhull_t;


int s2_convexhull_qhull(s2_body_t*);

#endif
