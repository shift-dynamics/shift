/**
@file
@brief Kinematics subroutine library: a vectorized math library for kinematics
and dynamics
@author Kristopher Wehage, Roger Wehage
@date 2016
@copyright Kristopher Wehage, Roger Wehage

This library contains a collection of functions for performing basic operations
on 3x1 column vectors and 6x1 screws and coscrews.

The following abbreviations are used to designate vectors, screws and coscrews
in the function prototypes

v = 3x1 vector, bksl_vec3_t
s = 6x1 screw, bksl_screw_t
c = 6x1 coscrew, bksl_coscrew_t

vectors consisting of only an x, y, or z component are denoted as:

vx

vy

vz

Vectors quantities are transformed using 3x3 rotation
matrices (i.e. SO3 matrices) and column vectors. The 3x3 rotation datastructure
is denoted as:

R = 3x3 rotation matrix in column major order, bksl_mat3x3_t

Rx = rotation about x axis

Ry = rotation about y axis

Rz = rotation about z axis


The letter "D" is used to
designate a general rigid body transformation consisting of a rotation and
translation component.

D = 3x3 rotation matrix and 3x1 translation vector, bksl_displacement_t

In the function prototypes, displacements consisting of only a translation
component are denoted as:

Dt = General Translation

Dtx = Translation along x axis

Dty = Translation along y axis

Dtz = Translation along z axis

Likewise, displacements consisting of only rotations are denoted as

Dr = General rotation

Drx = rotation about x axis

Dry = rotation about y axis

Drz = rotation about z axis

The inverse of any of the above operations is noted with "inverse". For
example, the inverse of a general rigid body transformation, D, is noted as
"Dinverse".

Screw and coscrew quantities are transformed using the 6x6 Adjoint and Coadjoint
transformation matrices respectively. The full 6x6 matrix is formed from
a rotation matrix and a vector quantity stored in a bksl_displacement_t data
structure. Therefore the full 6x6 matrix is never formed in the code.

The following abbreviations are used for Adjoint and Coadjoint transformations

Ad = A general adjoint transformation consisting of a rotation and translation

Adt = An adjoint transformation consisting of only a translation component

Adtx = An adjoint transformation consisting of only a translation component
  about the x-axis

Adty = An adjoint transformation consisting of only a translation component
  about the y-axis

Adtx = An adjoint transformation consisting of only a translation component
  about the z-axis

AdR = An adjoint transformation consisting of only a rotation component

AdRx = An adjoint transformation consisting of only a rotation component
  about the x-axis

AdRy = An adjoint transformation consisting of only a rotation component
  about the y-axis

AdRx = An adjoint transformation consisting of only a rotation component
  about the z-axis

CoAd = A general coadjoint transformation consisting of a rotation and translation

CoAdt = A coadjoint transformation consisting of only a translation component

CoAdtx = An adjoint transformation consisting of only a translation component
  about the x-axis

CoAdty = A coadjoint transformation consisting of only a translation component
  about the y-axis

CoAdtx = A coadjoint transformation consisting of only a translation component
  about the z-axis

CoAdR = A coadjoint transformation consisting of only a rotation component

CoAdRx = A coadjoint transformation consisting of only a rotation component
  about the x-axis

CoAdRy = A coadjoint transformation consisting of only a rotation component
  about the y-axis

CoAdRx = A coadjoint transformation consisting of only a rotation component
  about the z-axis

Readers are referred to *J.M. Selig, Gqometric Fundamentals of robotics (2005)*
or *R. Featherstone, rigid Body Dynamics Algorithms (2008)* for more
information.

@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS const uint32_tERRUPTION)
HOWEVER CAUSED AND ON ANY THqoRY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#ifndef _BKSL_H_
#define _BKSL_H_


#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "inttypes.h"

typedef struct {
  double x, y, z;
} bksl_vec3_t;
typedef struct {
  float x, y, z;
} bksl_vec3f_t;
typedef struct {
  int x, y, z;
} bksl_vec3i_t;

typedef struct {
  double x, y, z, w;
} bksl_vec4_t;
typedef struct {
  float x, y, z, w;
} bksl_vec4f_t;
typedef struct {
  int x, y, z, w;
} bksl_vec4i_t;

typedef struct {
  double x, y, z, w;
} bksl_quaternion_t;
typedef struct {
  float x, y, z, w;
} bksl_quaternionf_t;

typedef struct {
  double m00, m10, m20,
         m01, m11, m21,
         m02, m12, m22;
} bksl_mat3x3_t;

typedef struct {
  float m00, m10, m20,
        m01, m11, m21,
        m02, m12, m22;
} bksl_mat3x3f_t;

typedef struct {
  double m00, m10, m20, m30,
         m01, m11, m21, m31,
         m02, m12, m22, m32,
         m03, m13, m23, m33;
} bksl_mat4x4_t;

typedef struct {
  float m00, m10, m20, m30,
        m01, m11, m21, m31,
        m02, m12, m22, m32,
        m03, m13, m23, m33;
} bksl_mat4x4f_t;

typedef struct {
  bksl_mat3x3_t R;
  bksl_vec3_t t;
} bksl_displacement_t;

typedef struct {
  bksl_mat3x3f_t R;
  bksl_vec3f_t t;
} bksl_displacementf_t;

typedef struct {
  bksl_vec3_t v, w;
} bksl_screw_t;
typedef struct {
  bksl_vec3f_t v, w;
} bksl_screwf_t;

typedef struct {
  bksl_vec3_t f, m;
} bksl_coscrew_t;
typedef struct {
  bksl_vec3f_t f, m;
} bksl_coscrewf_t;

typedef enum {
  bksl_AXIS_XYZ,
  bksl_AXIS_YZX,
  bksl_AXIS_ZXY,
  bksl_AXIS_XZY,
  bksl_AXIS_YXZ,
  bksl_AXIS_ZYX,
  bksl_AXIS_XYX,
  bksl_AXIS_XZX,
  bksl_AXIS_YXY,
  bksl_AXIS_YZY,
  bksl_AXIS_ZXZ,
  bksl_AXIS_ZYZ
} bksl_axis_enum_t;

typedef enum {
  bksl_ROWMAJOR,
  bksl_COLMAJOR
} bksl_matrix_enum_t;


bksl_vec3_t* bksl_vec3();

bksl_vec3f_t* bksl_vec3f();

bksl_vec3i_t* bksl_vec3i();

bksl_displacement_t* bksl_displacement();

bksl_displacementf_t* bksl_displacementf();

bksl_screw_t* bksl_screw();

bksl_screwf_t* bksl_screwf();

bksl_mat3x3_t* bksl_mat3x3();

bksl_mat3x3f_t* bksl_mat3x3f();

bksl_mat4x4_t* bksl_mat4x4();

bksl_mat4x4f_t* bksl_mat4x4f();

void bksl_displacement_toMat4x4(
  bksl_displacement_t* __restrict__ D,
  bksl_mat4x4_t* m);

void bksl_displacementf_toMat4x4f(
  bksl_displacementf_t* __restrict__ D,
  bksl_mat4x4f_t* __restrict__ m);

void bksl_displacement_toMat4x4f(
  bksl_displacement_t* __restrict__ D,
  bksl_mat4x4f_t* __restrict__ M);

void bksl_vec3_print(
  FILE* f,
  bksl_vec3_t* __restrict__ v);

void bksl_vec3f_print(
  FILE* f,
  bksl_vec3f_t* __restrict__ v);

void bksl_screw_print(
  FILE* f,
  bksl_screw_t* __restrict__ s);

void bksl_coscrew_print(
  FILE* f,
  bksl_coscrew_t* __restrict__ c);

void bksl_mat3x3_print(
  FILE* f,
  bksl_mat3x3_t* __restrict__ R);

void bksl_mat3x3f_print(
  FILE* f,
  bksl_mat3x3f_t* __restrict__ R);

void bksl_displacement_print(
  FILE* f,
  bksl_displacement_t* __restrict__ D);

void bksl_mat4x4_print(
  FILE* f,
  bksl_mat4x4_t* __restrict__ M);

void bksl_mat4x4f_print(
  FILE* f,
  bksl_mat4x4f_t* __restrict__ M);

void bksl_array_print(
  FILE* f,
  const uint32_t n,
  double* __restrict__ a);

void bksl_array2D_print(
  FILE* f,
  const uint32_t rowDim,
  const uint32_t colDim,
  bksl_matrix_enum_t matrixType,
  double* __restrict__ a);

void bksl_arrayi_print(
  FILE* f,
  const uint32_t n,
  int* __restrict__ a);

void bksl_triang_print(
  FILE* f,
  const uint32_t n,
  double* __restrict__ a);

void bksl_triangi_print(
  FILE* f,
  const uint32_t n,
  int* __restrict__ a);

void bksl_triangb_print(
  FILE* f,
  const uint32_t n,
  bool* __restrict__ a);

void bksl_dc(
  double* __restrict__ thetai,
  double dc[2]);

void bksl_displacement_setIdentity(
  bksl_displacement_t* __restrict__ d);

void bksl_mat3x3_setIdentity(
  bksl_mat3x3_t* __restrict__ r);

void bksl_vec3_set(
  bksl_vec3_t* __restrict__ v,
  const uint32_t index,
  const double value);

void bksl_screw_set(
  bksl_screw_t* __restrict__ s,
  const uint32_t index,
  const double value);

void bksl_coscrew_set(
  bksl_coscrew_t* __restrict__ s,
  const uint32_t index,
  const double value);

double bksl_vec3_l2norm(
  bksl_vec3_t* __restrict__ v);

float bksl_vec3f_l2norm(
  bksl_vec3f_t* __restrict__ v);

double bksl_screw_l2norm(
  bksl_screw_t* __restrict__ s);

float bksl_screwf_l2norm(
  bksl_screwf_t* __restrict__ s);

double bksl_coscrew_l2norm(
  bksl_coscrew_t* __restrict__ c);

float bksl_coscrewf_l2norm(
  bksl_coscrewf_t* __restrict__ c);

void bksl_mat3x3_set(
  bksl_mat3x3_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  const double value);

void bksl_mat3x3_setFromVectors(
  bksl_mat3x3_t* __restrict__ R,
  bksl_vec3_t* __restrict__ x,
  bksl_vec3_t* __restrict__ y,
  bksl_vec3_t* __restrict__ z);

void bksl_mat4x4_set(
  bksl_mat4x4_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const double value);

void bksl_displacement_set(
  bksl_displacement_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const double value);

void bksl_vec3f_set(
  bksl_vec3f_t* __restrict__ v,
  const uint32_t index,
  const float value);

void bksl_screwf_set(
  bksl_screwf_t* __restrict__ s,
  const uint32_t index,
  const float value);

void bksl_coscrewf_set(
  bksl_coscrewf_t* __restrict__ c,
  const uint32_t index,
  const float value);

void bksl_mat3x3f_set(
  bksl_mat3x3f_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  const float value);

void bksl_mat4x4f_set(
  bksl_mat4x4f_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const float value);

void bksl_mat4x4f_setIdentity(
  bksl_mat4x4f_t* D);

void bksl_displacementf_set(
  bksl_displacementf_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const float value);

double bksl_vec3_get(
  bksl_vec3_t* __restrict__ v,
  const uint32_t index);

double bksl_screw_get(
  bksl_screw_t* __restrict__ s,
  const uint32_t index);

double bksl_coscrew_get(
  bksl_coscrew_t* __restrict__ c,
  const uint32_t index);

double bksl_mat3x3_get(
  bksl_mat3x3_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column);

double bksl_displacement_get(
  bksl_displacement_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column);

float bksl_vec3f_get(
  bksl_vec3f_t* __restrict__ v,
  const uint32_t index);

float bksl_screwf_get(
  bksl_screwf_t* __restrict__ s,
  const uint32_t index);

float bksl_coscrewf_get(
  bksl_coscrewf_t* __restrict__ c,
  const uint32_t index);

void bksl_displacement_getTranslation(
  bksl_displacement_t* __restrict__ D,
  bksl_vec3_t* t);

void bksl_vec3_normalize(
  bksl_vec3_t* __restrict__ v);

void bksl_vec3f_normalize(
  bksl_vec3f_t* __restrict__ v);

void bksl_vec3_scale(
  double k,
  bksl_vec3_t* __restrict__ v);

void bksl_vec3f_scale(
  double k,
  bksl_vec3f_t* __restrict__ v);

void bksl_copy_vv(
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_copy_vvf(
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo);

void bksl_invert_v(
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_invert_vf(
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo);

double bksl_dot_vv(
  bksl_vec3_t* __restrict__ vi0,
  bksl_vec3_t* __restrict__ vi1);

double bksl_dot_vvf(
  bksl_vec3f_t* __restrict__ vi0,
  bksl_vec3f_t* __restrict__ vi1);

double bksl_dot_cs(
  bksl_coscrew_t* __restrict__ ci,
  bksl_screw_t* __restrict__ si);

double bksl_dot_csf(
  bksl_coscrewf_t* __restrict__ ci,
  bksl_screwf_t* __restrict__ si);

void bksl_product_vk(
  bksl_vec3_t* __restrict__ vi,
  const double ki,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_vinversek(
  bksl_vec3_t* __restrict__ vi,
  const double ki,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_vkinverse(
  bksl_vec3_t* __restrict__ vi,
  const double ki,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_vkf(
  bksl_vec3f_t* __restrict__ vi,
  const float ki,
  bksl_vec3f_t* __restrict__ vo);

void bksl_add_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo);

void bksl_add_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo);

void bksl_addequal_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i);

void bksl_addequal_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i);

void bksl_subtract_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo);

void bksl_subtract_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo);

void bksl_subtractequal_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i);

void bksl_subtractequal_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i);

void bksl_axpy_vv(
  double scalar,
  bksl_vec3_t* __restrict__ v1x,
  bksl_vec3_t* __restrict__ v2y);

void bksl_axpy_vvf(
  float scalar,
  bksl_vec3f_t* __restrict__ v1x,
  bksl_vec3f_t* __restrict__ v2y);

void bksl_axpy_ss(
  double scalar,
  bksl_screw_t* __restrict__ v1x,
  bksl_screw_t* __restrict__ v2y);

void bksl_axpy_ssf(
  float scalar,
  bksl_screwf_t* __restrict__ v1x,
  bksl_screwf_t* __restrict__ v2y);

void bksl_cross_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo);

void bksl_cross_vvinverse(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo);

void bksl_cross_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo);

void bksl_cross_vvinversef(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo);

void bksl_copy_ss(
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_copy_ssf(
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_copy_cc(
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co);

void bksl_copy_ccf(
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co);

void bksl_invert_ss(
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_invert_sf(
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_sk(
  bksl_screw_t* __restrict__ si,
  const double ki,
  bksl_screw_t* __restrict__ so);

void bksl_product_skf(
  bksl_screwf_t* __restrict__ si,
  const float ki,
  bksl_screwf_t* __restrict__ so);

void bksl_add_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so);

void bksl_add_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so);

void bksl_addequal_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i);

void bksl_addequal_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i);

void bksl_subtract_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so);

void bksl_subtract_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so);

void bksl_subtractequal_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i);

void bksl_subtractequal_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i);

void bksl_add_sst(
  bksl_screw_t* __restrict__ si1,
  bksl_screw_t* __restrict__ si2,
  bksl_screw_t* __restrict__ so);

void bksl_add_sstf(
  bksl_screwf_t* __restrict__ si1,
  bksl_screwf_t* __restrict__ si2,
  bksl_screwf_t* __restrict__ so);

void bksl_copy_R(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_copy_Rf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_invert_R(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_invert_Rf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_Rv(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_Rvf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo);

void bksl_product_Rvinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_Rvinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo);

void bksl_product_Rinversev(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_Rinversevf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo);

void bksl_product_Rinversevinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_Rinversevinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo);

void bksl_product_Rs(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Rsinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Rsf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Rsinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Rinverses(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Rinversesf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_RRx(
  bksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRxf(
  bksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RRxinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRxinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RRy(
  bksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRyf(
  bksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RRyinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRyinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RRz(
  bksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRzf(
  bksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RRzinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRzinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RR(
  bksl_mat3x3_t* __restrict__ R1i,
  bksl_mat3x3_t* __restrict__ R2i,
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRf(
  bksl_mat3x3f_t* __restrict__ R1i,
  bksl_mat3x3f_t* __restrict__ R2i,
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RRinverse(
  bksl_mat3x3_t* __restrict__ R1i,
  bksl_mat3x3_t* __restrict__ R2i,
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RRinversef(
  bksl_mat3x3f_t* __restrict__ R1i,
  bksl_mat3x3f_t* __restrict__ R2i,
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_RinverseR(
  bksl_mat3x3_t* __restrict__ R1i,
  bksl_mat3x3_t* __restrict__ R2i,
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_product_RinverseRf(
  bksl_mat3x3f_t* __restrict__ R1i,
  bksl_mat3x3f_t* __restrict__ R2i,
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_product_Rvx(
  bksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to);

void bksl_product_Rvxf(
  bksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to);

void bksl_product_Rvxinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to);

void bksl_product_Rvxinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to);

void bksl_product_Rvy(
  bksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ti,
  bksl_vec3_t* __restrict__ to);

void bksl_product_Rvyf(
  bksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ti,
  bksl_vec3f_t* __restrict__ to);

void bksl_product_Rvyinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ti,
  bksl_vec3_t* __restrict__ to);

void bksl_product_Rvyinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ti,
  bksl_vec3f_t* __restrict__ to);

void bksl_product_Rvz(
  bksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to);

void bksl_product_Rvzf(
  bksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to);

void bksl_product_Rvzinverse(
  bksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to);

void bksl_product_Rtzinversef(
  bksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_vvx(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to);

void bksl_add_vvxf(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_vvxinverse(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to);

void bksl_add_vvxinversef(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_vvy(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to);

void bksl_add_vvyf(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_vvyinverse(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to);

void bksl_add_vvyinversef(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_vvz(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to);

void bksl_add_vvzf(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_vvzinverse(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to);

void bksl_add_vvzinversef(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to);

void bksl_invert_D(
  bksl_displacement_t* __restrict__ Di,
  bksl_displacement_t* __restrict__ Do);

void bksl_invert_Df(
  bksl_displacementf_t* __restrict__ Di,
  bksl_displacementf_t* __restrict__ Do);

void bksl_copy_D(
  bksl_displacement_t* __restrict__ Di,
  bksl_displacement_t* __restrict__ Do);

void bksl_copy_Df(
  bksl_displacementf_t* __restrict__ Di,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDrx(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDrxf(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDrxinverse(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDrxinversef(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDry(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDryf(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDryinverse(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDryinversef(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDrz(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDrzf(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDrzinverse(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDrzinversef(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDt(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDtinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_Dv(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_Dinversev(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo);

void bksl_product_DDtf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_vec3f_t* __restrict__ vi,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_Dinverset(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DinverseDtf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_vec3f_t* __restrict__ vi,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDtx(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDtxinverse(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_Dtxinversef(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDty(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDtyf(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDtyinverse(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ti,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDtyinversef(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ti,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDtz(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDtzf(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDtzinverse(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDr(
  bksl_displacement_t* __restrict__ Di,
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDrf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDrinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDrinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DD(
  bksl_displacement_t* __restrict__ D1i,
  bksl_displacement_t* __restrict__ D2i,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDf(
  bksl_displacementf_t* __restrict__ D1i,
  bksl_displacementf_t* __restrict__ D2i,
  bksl_displacementf_t* __restrict__ Do);

void bksl_product_DDinverse(
  bksl_displacement_t* __restrict__ D1i,
  bksl_displacement_t* __restrict__ D2i,
  bksl_displacement_t* __restrict__ Do);

void bksl_product_DDinversef(
  bksl_displacementf_t* __restrict__ D1i,
  bksl_displacementf_t* __restrict__ D2i,
  bksl_displacementf_t* __restrict__ Do);

void bksl_htx(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_htxf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_hty(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_htyf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_htz(
  bksl_displacement_t* __restrict__ Ri,
  bksl_screw_t* __restrict__ ho);

void bksl_htzf(
  bksl_displacementf_t* __restrict__ Ri,
  bksl_screwf_t* __restrict__ ho);

void bksl_hrx(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_hrxf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_hry(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_hrz(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_hrzf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_htxinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_htxinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_htyinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_htyinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_htzinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_htzinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_hrxinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_hrxinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_hryinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_hryinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_hrzinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho);

void bksl_hrzinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho);

void bksl_cross_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so);

void bksl_cross_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so);

void bksl_cross_sst(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so);

void bksl_cross_sstf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Adts(
  bksl_vec3_t* __restrict__ ti,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Adtsf(
  bksl_vec3f_t* __restrict__ ti,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Adtinverses(
  bksl_vec3_t* __restrict__ ti,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Adtinversesf(
  bksl_vec3f_t* __restrict__ ti,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Adtsinverse(
  bksl_vec3_t* __restrict__ ti,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Adtsinversef(
  bksl_vec3f_t* __restrict__ ti,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Ads(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Adsinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Adsf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_product_Adinverses(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so);

void bksl_product_Adinversesf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so);

void bksl_cross_cc(
  bksl_coscrew_t* __restrict__ c1i,
  bksl_coscrew_t* __restrict__ c2i,
  bksl_coscrew_t* __restrict__ co);

void bksl_cross_ccf(
  bksl_coscrewf_t* __restrict__ c1i,
  bksl_coscrewf_t* __restrict__ c2i,
  bksl_coscrewf_t* __restrict__ co);

void bksl_product_CoAdtc(
  bksl_vec3_t* __restrict__ ti,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co);

void bksl_product_CoAdtcf(
  bksl_vec3f_t* __restrict__ ti,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co);

void bksl_product_CoAdtcinverse(
  bksl_vec3_t* __restrict__ ti,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co);

void bksl_product_CoAdtcinversef(
  bksl_vec3f_t* __restrict__ ti,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co);

void bksl_product_CoAdc(
  bksl_displacement_t* __restrict__ Di,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co);

void bksl_product_CoAdcf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co);

void bksl_product_CoAdinversec(
  bksl_displacement_t* __restrict__ Di,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co);

void bksl_product_CoAdinversecf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co);

void bksl_normalize_qe(
  bksl_quaternion_t* __restrict__ qi);

void bksl_normalize_qf(
  bksl_quaternionf_t* __restrict__ qi);

void bksl_R_to_q(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_quaternion_t* __restrict__ qo);

void bksl_Rf_to_qf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_quaternionf_t* __restrict__ qo);

void bksl_q_to_R(
  bksl_quaternion_t* __restrict__ qi,
  bksl_mat3x3_t* __restrict__ Ro);

void bksl_qf_to_Rf(
  bksl_quaternionf_t* __restrict__ qi,
  bksl_mat3x3f_t* __restrict__ Ro);

void bksl_slerp(
  bksl_quaternion_t* __restrict__ q1i,
  bksl_quaternion_t* __restrict__ q2i,
  double* __restrict__ t,
  bksl_quaternion_t* __restrict__ qo);

void bksl_slerpf(
  bksl_quaternionf_t* __restrict__ q1i,
  bksl_quaternionf_t* __restrict__ q2i,
  float* __restrict__ t,
  bksl_quaternionf_t* __restrict__ qo);

void bksl_nlerp(
  bksl_quaternion_t* __restrict__ q1i,
  bksl_quaternion_t* __restrict__ q2i,
  double* __restrict__ t,
  bksl_quaternion_t* __restrict__ qo);

void bksl_nlerpf(
  bksl_quaternionf_t* __restrict__ q1i,
  bksl_quaternionf_t* __restrict__ q2i,
  float* __restrict__ t,
  bksl_quaternionf_t* __restrict__ qo);

double bksl_normalizeDegrees(
  double angle);

float bksl_normalizeDegreesf(
  float angle);

double bksl_normalizeRadians(
  double angle);

float bksl_normalizeRadiansf(
  float angle);

double catan2pi(
  double y,
  double x,
  double theta);

double catan1pi(
  double y,
  double x,
  double theta);

void bksl_mat3x3_getEulerAngles(
  bksl_mat3x3_t* __restrict__ rin,
  bksl_vec3_t* __restrict__ angles,
  bksl_axis_enum_t axisType);

double bksl_mat3x3_getScalar(
  bksl_mat3x3_t* __restrict__ ri);

float bksl_mat3x3f_getScalar(
  bksl_mat3x3f_t* __restrict__ ri);

void bksl_mat4x4_getTranslation(
  bksl_mat4x4_t* __restrict__ Mi,
  bksl_vec3_t* __restrict__ to);

void bksl_mat4x4f_getTranslation(
  bksl_mat4x4f_t* __restrict__ Mi,
  bksl_vec3f_t* __restrict__ to);

void bksl_add_dpiToq(
  bksl_vec3_t* __restrict__ dpi,
  bksl_quaternion_t* __restrict__ qi);

void bksl_subtract_dpiFromq(
  bksl_vec3_t* __restrict__ dpi,
  bksl_quaternion_t* __restrict__ qi);

void bksl_omega_to_dquaternion(
  bksl_vec3_t* __restrict__ w,
  bksl_quaternion_t* __restrict__ qi,
  bksl_quaternion_t* __restrict__ dqo);

void bksl_array_lerp(
  const uint32_t qCount,
  const double p,
  double* __restrict__ y0,
  double* __restrict__ y1,
  double* __restrict__ yOut);

void bksl_gramSchmidt(
  const uint32_t m,
  const uint32_t n,
  double* __restrict__ A);

bool bksl_all_close(
  const uint32_t count,
  double* __restrict__ a1,
  double* __restrict__ a2);

#endif
