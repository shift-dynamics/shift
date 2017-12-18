/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Context datastructure.
@date 2016
@copyright Kristopher Wehage 2016

This function library is used to initialize and free memory for
data structures defined in spheretree.h

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

#ifndef _SHIFT_SPHERE_H_
#define _SHIFT_SPHERE_H_

#include "kinematics.h"

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif
#include <jansson.h>

typedef struct s2_body_t s2_body_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_material_t s2_material_t;
typedef struct s2_inertia_t s2_inertia_t;
typedef struct s2_convexhull_t s2_convexhull_t;
typedef struct s2_context_t s2_context_t;

typedef struct s2_sphere_t {
  s2_vector_t centerGlobal;
  s2_vector_t centerLocal;
  double radius;
  s2_body_t* parentBody;
} s2_sphere_t;

void s2_sphere_approximateMinBoundSphere(
  s2_sphere_t*,
  s2_body_t*);

void s2_sphere_updateCentroid(
  s2_sphere_t*);

#endif
