/**
@file
@author Kristopher Wehage
@date 2017
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

#ifndef _STAR2_POINTCLOUD_H_
#define _STAR2_POINTCLOUD_H_

#include <stdio.h>
#include "kinematics.h"

#define ThrowWandException(wand) \
{ \
  char \
    *description; \
 \
  ExceptionType \
    severity; \
 \
  description = MagickGetException(wand,&severity); \
  (void) fprintf(stderr, "%s %s %lu %s\n", GetMagickModule(), description); \
  description = (char *)MagickRelinquishMemory(description); \
  exit(-1); \
}

typedef struct s2_context_t s2_context_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_frame_t s2_frame_t;

typedef struct s2_pointcloud_t {
  double* vertices;
  float* verticesf;
  float* colors;
  size_t vertexCount;
  s2_stackstr_t* definition; /*!< Path to files that define body definitions,
                                    i.e. ply files */
  s2_assembly_t* assembly; /*!< Pointer to assembly that pointcloud is a
                                  member of. */
  s2_body_t* body; /*!< pointer to body that point cloud belongs to */
  s2_frame_t* frame;
  s2_displacement_t* transform;
  s2_stackstr_t* name;
  s2_pointcloud_t* next;
  bool isVisible;
} s2_pointcloud_t;

s2_pointcloud_t* s2_pointcloud(s2_stackstr_t*);

void s2_pointcloud_free(s2_pointcloud_t*);




#endif
