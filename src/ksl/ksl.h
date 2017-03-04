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

v = 3x1 vector, ksl_vector_t
s = 6x1 screw, ksl_screw_t
c = 6x1 coscrew, ksl_coscrew_t

vectors consisting of only an x, y, or z component are denoted as:

vx

vy

vz

Vectors quantities are transformed using 3x3 rotation
matrices (i.e. SO3 matrices) and column vectors. The 3x3 rotation datastructure
is denoted as:

R = 3x3 rotation matrix in column major order, ksl_mat3x3_t

Rx = rotation about x axis

Ry = rotation about y axis

Rz = rotation about z axis


The letter "D" is used to
designate a general rigid body transformation consisting of a rotation and
translation component.

D = 3x3 rotation matrix and 3x1 translation vector, ksl_displacement_t

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
a rotation matrix and a vector quantity stored in a ksl_displacement_t data
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

#ifndef _KSL_H_
#define _KSL_H_


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
#include <inttypes.h>

typedef struct {double x, y, z;} ksl_vector_t;
typedef struct {float x, y, z;} ksl_vectorf_t;
typedef struct {int x, y, z;} ksl_vectori_t;

typedef struct {double x, y, z, w;} ksl_quaternion_t;
typedef struct {float x, y, z, w;} ksl_quaternionf_t;

typedef struct {double m00, m10, m20, m01, m11, m21, m02, m12, m22;} ksl_mat3x3_t;
typedef struct {float m00, m10, m20, m01, m11, m21, m02, m12, m22;} ksl_mat3x3f_t;

typedef struct {
double m00, m10, m20, m30,
       m01, m11, m21, m31,
       m02, m12, m22, m32,
       m03, m13, m23, m33;} ksl_mat4x4_t;

typedef struct {
float m00, m10, m20, m30,
      m01, m11, m21, m31,
      m02, m12, m22, m32,
      m03, m13, m23, m33;} ksl_mat4x4f_t;

typedef struct {
  ksl_mat3x3_t R;
  ksl_vector_t t;
} ksl_displacement_t;

typedef struct {
  ksl_mat3x3f_t R;
  ksl_vectorf_t t;
} ksl_displacementf_t;

typedef struct {ksl_vector_t v, w;} ksl_screw_t;
typedef struct {ksl_vectorf_t v, w;} ksl_screwf_t;

typedef struct {ksl_vector_t f, m;} ksl_coscrew_t;
typedef struct {ksl_vectorf_t f, m;} ksl_coscrewf_t;

typedef enum {
  KSL_AXIS_XYZ,
  KSL_AXIS_YZX,
  KSL_AXIS_ZXY,
  KSL_AXIS_XZY,
  KSL_AXIS_YXZ,
  KSL_AXIS_ZYX,
  KSL_AXIS_XYX,
  KSL_AXIS_XZX,
  KSL_AXIS_YXY,
  KSL_AXIS_YZY,
  KSL_AXIS_ZXZ,
  KSL_AXIS_ZYZ
} ksl_axis_enum_t;

typedef enum {
  KSL_ROWMAJOR,
  KSL_COLMAJOR
} ksl_matrix_enum_t;


ksl_vector_t* ksl_vector();

ksl_vectorf_t* ksl_vectorf();

ksl_vectori_t* ksl_vectori();

ksl_displacement_t* ksl_displacement();

ksl_displacementf_t* ksl_displacementf();

ksl_screw_t* ksl_screw();

ksl_screwf_t* ksl_screwf();

ksl_mat3x3_t* ksl_mat3x3();

ksl_mat3x3f_t* ksl_mat3x3f();

ksl_mat4x4_t* ksl_mat4x4();

ksl_mat4x4f_t* ksl_mat4x4f();

void ksl_displacement_toMat4x4(
  ksl_displacement_t* __restrict__ D,
  ksl_mat4x4_t* m);

void ksl_displacementf_toMat4x4f(
  ksl_displacementf_t* __restrict__ D,
  ksl_mat4x4f_t* __restrict__ m);

void ksl_displacement_toMat4x4f(
  ksl_displacement_t* __restrict__ D,
  ksl_mat4x4f_t* __restrict__ M);

void ksl_vector_print(
  FILE* f,
  ksl_vector_t* __restrict__ v);

void ksl_screw_print(
  FILE* f,
  ksl_screw_t* __restrict__ s);

void ksl_coscrew_print(
  FILE* f,
  ksl_coscrew_t* __restrict__ c);

void ksl_mat3x3_print(
  FILE* f,
  ksl_mat3x3_t* __restrict__ R);

void ksl_mat3x3f_print(
  FILE* f,
  ksl_mat3x3f_t* __restrict__ R);

void ksl_displacement_print(
  FILE* f,
  ksl_displacement_t* __restrict__ D);

void ksl_mat4x4_print(
  FILE* f,
  ksl_mat4x4_t* __restrict__ M);

void ksl_mat4x4f_print(
  FILE* f,
  ksl_mat4x4f_t* __restrict__ M);

void ksl_array_print(
  FILE* f,
  const uint32_t n,
  double* __restrict__ a);

void ksl_array2D_print(
  FILE* f,
  const uint32_t rowDim,
  const uint32_t colDim,
  ksl_matrix_enum_t matrixType,
  double* __restrict__ a);

void ksl_arrayi_print(
  FILE* f,
  const uint32_t n,
  int* __restrict__ a);

void ksl_triang_print(
  FILE* f,
  const uint32_t n,
  double* __restrict__ a);

void ksl_triangi_print(
  FILE* f,
  const uint32_t n,
  int* __restrict__ a);

void ksl_triangb_print(
  FILE* f,
  const uint32_t n,
  bool* __restrict__ a);

void ksl_dc(
  double* __restrict__ thetai,
  double dc[2]);

void ksl_displacement_setIdentity(
  ksl_displacement_t* __restrict__ d);

void ksl_mat3x3_setIdentity(
  ksl_mat3x3_t* __restrict__ r);

void ksl_vector_set(
  ksl_vector_t* __restrict__ v,
  const uint32_t index,
  const double value);

void ksl_screw_set(
  ksl_screw_t* __restrict__ s,
  const uint32_t index,
  const double value);

void ksl_coscrew_set(
  ksl_coscrew_t* __restrict__ s,
  const uint32_t index,
  const double value);

double ksl_vector_l2norm(
  ksl_vector_t* __restrict__ v);

float ksl_vectorf_l2norm(
  ksl_vectorf_t* __restrict__ v);

double ksl_screw_l2norm(
  ksl_screw_t* __restrict__ s);

float ksl_screwf_l2norm(
  ksl_screwf_t* __restrict__ s);

double ksl_coscrew_l2norm(
  ksl_coscrew_t* __restrict__ c);

float ksl_coscrewf_l2norm(
  ksl_coscrewf_t* __restrict__ c);

void ksl_mat3x3_set(
  ksl_mat3x3_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  const double value);

void ksl_mat3x3_setFromVectors(
  ksl_mat3x3_t* __restrict__ R,
  ksl_vector_t* __restrict__ x,
  ksl_vector_t* __restrict__ y,
  ksl_vector_t* __restrict__ z);

void ksl_mat4x4_set(
  ksl_mat4x4_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const double value);

void ksl_displacement_set(
  ksl_displacement_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const double value);

void ksl_vectorf_set(
  ksl_vectorf_t* __restrict__ v,
  const uint32_t index,
  const float value);

void ksl_screwf_set(
  ksl_screwf_t* __restrict__ s,
  const uint32_t index,
  const float value);

void ksl_coscrewf_set(
  ksl_coscrewf_t* __restrict__ c,
  const uint32_t index,
  const float value);

void ksl_mat3x3f_set(
  ksl_mat3x3f_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  const float value);

void ksl_mat4x4f_set(
  ksl_mat4x4f_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const float value);

void ksl_mat4x4f_setIdentity(
  ksl_mat4x4f_t* D);

void ksl_displacementf_set(
  ksl_displacementf_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const float value);

double ksl_vector_get(
  ksl_vector_t* __restrict__ v,
  const uint32_t index);

double ksl_screw_get(
  ksl_screw_t* __restrict__ s,
  const uint32_t index);

double ksl_coscrew_get(
  ksl_coscrew_t* __restrict__ c,
  const uint32_t index);

double ksl_mat3x3_get(
  ksl_mat3x3_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column);

double ksl_displacement_get(
  ksl_displacement_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column);

float ksl_vectorf_get(
  ksl_vectorf_t* __restrict__ v,
  const uint32_t index);

float ksl_screwf_get(
  ksl_screwf_t* __restrict__ s,
  const uint32_t index);

float ksl_coscrewf_get(
  ksl_coscrewf_t* __restrict__ c,
  const uint32_t index);

void ksl_displacement_getTranslation(
  ksl_displacement_t* __restrict__ D,
  ksl_vector_t* t);

void ksl_vector_normalize(
  ksl_vector_t* __restrict__ v);

void ksl_vectorf_normalize(
  ksl_vectorf_t* __restrict__ v);

void ksl_vector_scale(
  double k,
  ksl_vector_t* __restrict__ v);

void ksl_vectorf_scale(
  double k,
  ksl_vectorf_t* __restrict__ v);

void ksl_copy_vv(
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_copy_vvf(
  ksl_vectorf_t* __restrict__ vi,
  ksl_vectorf_t* __restrict__ vo);

void ksl_invert_v(
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_invert_vf(
  ksl_vectorf_t* __restrict__ vi,
  ksl_vectorf_t* __restrict__ vo);

double ksl_dot_vv(
  ksl_vector_t* __restrict__ vi0,
  ksl_vector_t* __restrict__ vi1);

double ksl_dot_vvf(
  ksl_vectorf_t* __restrict__ vi0,
  ksl_vectorf_t* __restrict__ vi1);

double ksl_dot_cs(
  ksl_coscrew_t* __restrict__ ci,
  ksl_screw_t* __restrict__ si);

double ksl_dot_csf(
  ksl_coscrewf_t* __restrict__ ci,
  ksl_screwf_t* __restrict__ si);

void ksl_product_vk(
  ksl_vector_t* __restrict__ vi,
  const double ki,
  ksl_vector_t* __restrict__ vo);

void ksl_product_vinversek(
  ksl_vector_t* __restrict__ vi,
  const double ki,
  ksl_vector_t* __restrict__ vo);

void ksl_product_vkinverse(
  ksl_vector_t* __restrict__ vi,
  const double ki,
  ksl_vector_t* __restrict__ vo);

void ksl_product_vkf(
  ksl_vectorf_t* __restrict__ vi,
  const float ki,
  ksl_vectorf_t* __restrict__ vo);

void ksl_add_vv(
  ksl_vector_t* __restrict__ v1i,
  ksl_vector_t* __restrict__ v2i,
  ksl_vector_t* __restrict__ vo);

void ksl_add_vvf(
  ksl_vectorf_t* __restrict__ v1i,
  ksl_vectorf_t* __restrict__ v2i,
  ksl_vectorf_t* __restrict__ vo);

void ksl_addequal_vv(
  ksl_vector_t* __restrict__ v1i,
  ksl_vector_t* __restrict__ v2i);

void ksl_addequal_vvf(
  ksl_vectorf_t* __restrict__ v1i,
  ksl_vectorf_t* __restrict__ v2i);

void ksl_subtract_vv(
  ksl_vector_t* __restrict__ v1i,
  ksl_vector_t* __restrict__ v2i,
  ksl_vector_t* __restrict__ vo);

void ksl_subtract_vvf(
  ksl_vectorf_t* __restrict__ v1i,
  ksl_vectorf_t* __restrict__ v2i,
  ksl_vectorf_t* __restrict__ vo);

void ksl_subtractequal_vv(
  ksl_vector_t* __restrict__ v1i,
  ksl_vector_t* __restrict__ v2i);

void ksl_subtractequal_vvf(
  ksl_vectorf_t* __restrict__ v1i,
  ksl_vectorf_t* __restrict__ v2i);

void ksl_axpy_vv(
  double scalar,
  ksl_vector_t* __restrict__ v1x,
  ksl_vector_t* __restrict__ v2y);

void ksl_axpy_vvf(
  float scalar,
  ksl_vectorf_t* __restrict__ v1x,
  ksl_vectorf_t* __restrict__ v2y);

void ksl_axpy_ss(
  double scalar,
  ksl_screw_t* __restrict__ v1x,
  ksl_screw_t* __restrict__ v2y);

void ksl_axpy_ssf(
  float scalar,
  ksl_screwf_t* __restrict__ v1x,
  ksl_screwf_t* __restrict__ v2y);

void ksl_cross_vv(
  ksl_vector_t* __restrict__ v1i,
  ksl_vector_t* __restrict__ v2i,
  ksl_vector_t* __restrict__ vo);

void ksl_cross_vvinverse(
  ksl_vector_t* __restrict__ v1i,
  ksl_vector_t* __restrict__ v2i,
  ksl_vector_t* __restrict__ vo);

void ksl_cross_vvf(
  ksl_vectorf_t* __restrict__ v1i,
  ksl_vectorf_t* __restrict__ v2i,
  ksl_vectorf_t* __restrict__ vo);

void ksl_cross_vvinversef(
  ksl_vectorf_t* __restrict__ v1i,
  ksl_vectorf_t* __restrict__ v2i,
  ksl_vectorf_t* __restrict__ vo);

void ksl_copy_ss(
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_copy_ssf(
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_copy_cc(
  ksl_coscrew_t* __restrict__ ci,
  ksl_coscrew_t* __restrict__ co);

void ksl_copy_ccf(
  ksl_coscrewf_t* __restrict__ ci,
  ksl_coscrewf_t* __restrict__ co);

void ksl_invert_ss(
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_invert_sf(
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_sk(
  ksl_screw_t* __restrict__ si,
  const double ki,
  ksl_screw_t* __restrict__ so);

void ksl_product_skf(
  ksl_screwf_t* __restrict__ si,
  const float ki,
  ksl_screwf_t* __restrict__ so);

void ksl_add_ss(
  ksl_screw_t* __restrict__ s1i,
  ksl_screw_t* __restrict__ s2i,
  ksl_screw_t* __restrict__ so);

void ksl_add_ssf(
  ksl_screwf_t* __restrict__ s1i,
  ksl_screwf_t* __restrict__ s2i,
  ksl_screwf_t* __restrict__ so);

void ksl_addequal_ss(
  ksl_screw_t* __restrict__ s1i,
  ksl_screw_t* __restrict__ s2i);

void ksl_addequal_ssf(
  ksl_screwf_t* __restrict__ s1i,
  ksl_screwf_t* __restrict__ s2i);

void ksl_subtract_ss(
  ksl_screw_t* __restrict__ s1i,
  ksl_screw_t* __restrict__ s2i,
  ksl_screw_t* __restrict__ so);

void ksl_subtract_ssf(
  ksl_screwf_t* __restrict__ s1i,
  ksl_screwf_t* __restrict__ s2i,
  ksl_screwf_t* __restrict__ so);

void ksl_subtractequal_ss(
  ksl_screw_t* __restrict__ s1i,
  ksl_screw_t* __restrict__ s2i);

void ksl_subtractequal_ssf(
  ksl_screwf_t* __restrict__ s1i,
  ksl_screwf_t* __restrict__ s2i);

void ksl_add_sst(
  ksl_screw_t* __restrict__ si1,
  ksl_screw_t* __restrict__ si2,
  ksl_screw_t* __restrict__ so);

void ksl_add_sstf(
  ksl_screwf_t* __restrict__ si1,
  ksl_screwf_t* __restrict__ si2,
  ksl_screwf_t* __restrict__ so);

void ksl_copy_R(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_copy_Rf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_invert_R(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_invert_Rf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_Rv(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_product_Rvf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_vectorf_t* __restrict__ vi,
  ksl_vectorf_t* __restrict__ vo);

void ksl_product_Rvinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_product_Rvinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_vectorf_t* __restrict__ vi,
  ksl_vectorf_t* __restrict__ vo);

void ksl_product_Rinversev(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_product_Rinversevf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_vectorf_t* __restrict__ vi,
  ksl_vectorf_t* __restrict__ vo);

void ksl_product_Rinversevinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_product_Rinversevinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_vectorf_t* __restrict__ vi,
  ksl_vectorf_t* __restrict__ vo);

void ksl_product_Rs(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Rsinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Rsf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Rsinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Rinverses(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Rinversesf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_RRx(
  ksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRxf(
  ksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RRxinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRxinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RRy(
  ksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRyf(
  ksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RRyinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRyinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RRz(
  ksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRzf(
  ksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RRzinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  double dc[2],
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRzinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  float dc[2],
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RR(
  ksl_mat3x3_t* __restrict__ R1i,
  ksl_mat3x3_t* __restrict__ R2i,
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRf(
  ksl_mat3x3f_t* __restrict__ R1i,
  ksl_mat3x3f_t* __restrict__ R2i,
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RRinverse(
  ksl_mat3x3_t* __restrict__ R1i,
  ksl_mat3x3_t* __restrict__ R2i,
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RRinversef(
  ksl_mat3x3f_t* __restrict__ R1i,
  ksl_mat3x3f_t* __restrict__ R2i,
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_RinverseR(
  ksl_mat3x3_t* __restrict__ R1i,
  ksl_mat3x3_t* __restrict__ R2i,
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_product_RinverseRf(
  ksl_mat3x3f_t* __restrict__ R1i,
  ksl_mat3x3f_t* __restrict__ R2i,
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_product_Rvx(
  ksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  ksl_vector_t* __restrict__ to);

void ksl_product_Rvxf(
  ksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  ksl_vectorf_t* __restrict__ to);

void ksl_product_Rvxinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  ksl_vector_t* __restrict__ to);

void ksl_product_Rvxinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  ksl_vectorf_t* __restrict__ to);

void ksl_product_Rvy(
  ksl_mat3x3_t* __restrict__ Ri,
  double * __restrict__ti,
  ksl_vector_t* __restrict__ to);

void ksl_product_Rvyf(
  ksl_mat3x3f_t* __restrict__ Ri,
  float * __restrict__ti,
  ksl_vectorf_t* __restrict__ to);

void ksl_product_Rvyinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  double * __restrict__ti,
  ksl_vector_t* __restrict__ to);

void ksl_product_Rvyinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  float * __restrict__ti,
  ksl_vectorf_t* __restrict__ to);

void ksl_product_Rvz(
  ksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  ksl_vector_t* __restrict__ to);

void ksl_product_Rvzf(
  ksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  ksl_vectorf_t* __restrict__ to);

void ksl_product_Rvzinverse(
  ksl_mat3x3_t* __restrict__ Ri,
  double* __restrict__ ti,
  ksl_vector_t* __restrict__ to);

void ksl_product_Rtzinversef(
  ksl_mat3x3f_t* __restrict__ Ri,
  float* __restrict__ ti,
  ksl_vectorf_t* __restrict__ to);

void ksl_add_vvx(
  ksl_vector_t* __restrict__ t1i,
  double* __restrict__ t2i,
  ksl_vector_t* __restrict__ to);

void ksl_add_vvxf(
  ksl_vectorf_t* __restrict__ t1i,
  float* __restrict__ t2i,
  ksl_vectorf_t* __restrict__ to);

void ksl_add_vvxinverse(
  ksl_vector_t* __restrict__ t1i,
  double* __restrict__ t2i,
  ksl_vector_t* __restrict__ to);

void ksl_add_vvxinversef(
  ksl_vectorf_t* __restrict__ t1i,
  float* __restrict__ t2i,
  ksl_vectorf_t* __restrict__ to);

void ksl_add_vvy(
  ksl_vector_t* __restrict__ t1i,
  double* __restrict__ t2i,
  ksl_vector_t* __restrict__ to);

void ksl_add_vvyf(
  ksl_vectorf_t* __restrict__ t1i,
  float* __restrict__ t2i,
  ksl_vectorf_t* __restrict__ to);

void ksl_add_vvyinverse(
  ksl_vector_t* __restrict__ t1i,
  double* __restrict__ t2i,
  ksl_vector_t* __restrict__ to);

void ksl_add_vvyinversef(
  ksl_vectorf_t* __restrict__ t1i,
  float* __restrict__ t2i,
  ksl_vectorf_t* __restrict__ to);

void ksl_add_vvz(
  ksl_vector_t* __restrict__ t1i,
  double* __restrict__ t2i,
  ksl_vector_t* __restrict__ to);

void ksl_add_vvzf(
  ksl_vectorf_t* __restrict__ t1i,
  float* __restrict__ t2i,
  ksl_vectorf_t* __restrict__ to);

void ksl_add_vvzinverse(
  ksl_vector_t* __restrict__ t1i,
  double* __restrict__ t2i,
  ksl_vector_t* __restrict__ to);

void ksl_add_vvzinversef(
  ksl_vectorf_t* __restrict__ t1i,
  float* __restrict__ t2i,
  ksl_vectorf_t* __restrict__ to);

void ksl_invert_D(
  ksl_displacement_t* __restrict__ Di,
  ksl_displacement_t* __restrict__ Do);

void ksl_invert_Df(
  ksl_displacementf_t* __restrict__ Di,
  ksl_displacementf_t* __restrict__ Do);

void ksl_copy_D(
  ksl_displacement_t* __restrict__ Di,
  ksl_displacement_t* __restrict__ Do);

void ksl_copy_Df(
  ksl_displacementf_t* __restrict__ Di,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDrx(
  ksl_displacement_t* __restrict__ Di,
  double dc[2],
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDrxf(
  ksl_displacementf_t* __restrict__ Di,
  float dc[2],
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDrxinverse(
  ksl_displacement_t* __restrict__ Di,
  double dc[2],
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDrxinversef(
  ksl_displacementf_t* __restrict__ Di,
  float dc[2],
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDry(
  ksl_displacement_t* __restrict__ Di,
  double dc[2],
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDryf(
  ksl_displacementf_t* __restrict__ Di,
  float dc[2],
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDryinverse(
  ksl_displacement_t* __restrict__ Di,
  double dc[2],
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDryinversef(
  ksl_displacementf_t* __restrict__ Di,
  float dc[2],
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDrz(
  ksl_displacement_t* __restrict__ Di,
  double dc[2],
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDrzf(
  ksl_displacementf_t* __restrict__ Di,
  float dc[2],
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDrzinverse(
  ksl_displacement_t* __restrict__ Di,
  double dc[2],
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDrzinversef(
  ksl_displacementf_t* __restrict__ Di,
  float dc[2],
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDt(
  ksl_displacement_t* __restrict__ Di,
  ksl_vector_t* __restrict__ vi,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDtinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_vector_t* __restrict__ vi,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_Dv(
  ksl_displacement_t* __restrict__ Di,
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_product_Dinversev(
  ksl_displacement_t* __restrict__ Di,
  ksl_vector_t* __restrict__ vi,
  ksl_vector_t* __restrict__ vo);

void ksl_product_DDtf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_vectorf_t* __restrict__ vi,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_Dinverset(
  ksl_displacement_t* __restrict__ Di,
  ksl_vector_t* __restrict__ vi,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DinverseDtf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_vectorf_t* __restrict__ vi,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDtx(
  ksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDtxinverse(
  ksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_Dtxinversef(
  ksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDty(
  ksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDtyf(
  ksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDtyinverse(
  ksl_displacement_t* __restrict__ Di,
  double * __restrict__ti,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDtyinversef(
  ksl_displacementf_t* __restrict__ Di,
  float * __restrict__ti,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDtz(
  ksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDtzf(
  ksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDtzinverse(
  ksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDr(
    ksl_displacement_t* __restrict__ Di,
    ksl_mat3x3_t* __restrict__ Ri,
    ksl_displacement_t* __restrict__ Do);

void ksl_product_DDrf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDrinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDrinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DD(
  ksl_displacement_t* __restrict__ D1i,
  ksl_displacement_t* __restrict__ D2i,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDf(
  ksl_displacementf_t* __restrict__ D1i,
  ksl_displacementf_t* __restrict__ D2i,
  ksl_displacementf_t* __restrict__ Do);

void ksl_product_DDinverse(
  ksl_displacement_t* __restrict__ D1i,
  ksl_displacement_t* __restrict__ D2i,
  ksl_displacement_t* __restrict__ Do);

void ksl_product_DDinversef(
  ksl_displacementf_t* __restrict__ D1i,
  ksl_displacementf_t* __restrict__ D2i,
  ksl_displacementf_t* __restrict__ Do);

void ksl_htx(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_htxf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_hty(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_htyf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_htz(
  ksl_displacement_t* __restrict__ Ri,
  ksl_screw_t* __restrict__ ho);

void ksl_htzf(
  ksl_displacementf_t* __restrict__ Ri,
  ksl_screwf_t* __restrict__ ho);

void ksl_hrx(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_hrxf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_hry(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_hrz(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_hrzf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_htxinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_htxinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_htyinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_htyinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_htzinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_htzinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_hrxinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_hrxinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_hryinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_hryinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_hrzinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ ho);

void ksl_hrzinversef(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ ho);

void ksl_cross_ss(
  ksl_screw_t* __restrict__ s1i,
  ksl_screw_t* __restrict__ s2i,
  ksl_screw_t* __restrict__ so);

void ksl_cross_ssf(
  ksl_screwf_t* __restrict__ s1i,
  ksl_screwf_t* __restrict__ s2i,
  ksl_screwf_t* __restrict__ so);

void ksl_cross_sst(
  ksl_screw_t* __restrict__ s1i,
  ksl_screw_t* __restrict__ s2i,
  ksl_screw_t* __restrict__ so);

void ksl_cross_sstf(
  ksl_screwf_t* __restrict__ s1i,
  ksl_screwf_t* __restrict__ s2i,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Adts(
  ksl_vector_t* __restrict__ ti,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Adtsf(
  ksl_vectorf_t* __restrict__ ti,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Adtinverses(
  ksl_vector_t* __restrict__ ti,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Adtinversesf(
  ksl_vectorf_t* __restrict__ ti,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Adtsinverse(
  ksl_vector_t* __restrict__ ti,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Adtsinversef(
  ksl_vectorf_t* __restrict__ ti,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Ads(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Adsinverse(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Adsf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_product_Adinverses(
  ksl_displacement_t* __restrict__ Di,
  ksl_screw_t* __restrict__ si,
  ksl_screw_t* __restrict__ so);

void ksl_product_Adinversesf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_screwf_t* __restrict__ si,
  ksl_screwf_t* __restrict__ so);

void ksl_cross_cc(
  ksl_coscrew_t* __restrict__ c1i,
  ksl_coscrew_t* __restrict__ c2i,
  ksl_coscrew_t* __restrict__ co);

void ksl_cross_ccf(
  ksl_coscrewf_t* __restrict__ c1i,
  ksl_coscrewf_t* __restrict__ c2i,
  ksl_coscrewf_t* __restrict__ co);

void ksl_product_CoAdtc(
  ksl_vector_t* __restrict__ ti,
  ksl_coscrew_t* __restrict__ ci,
  ksl_coscrew_t* __restrict__ co);

void ksl_product_CoAdtcf(
  ksl_vectorf_t* __restrict__ ti,
  ksl_coscrewf_t* __restrict__ ci,
  ksl_coscrewf_t* __restrict__ co);

void ksl_product_CoAdtcinverse(
  ksl_vector_t* __restrict__ ti,
  ksl_coscrew_t* __restrict__ ci,
  ksl_coscrew_t* __restrict__ co);

void ksl_product_CoAdtcinversef(
  ksl_vectorf_t* __restrict__ ti,
  ksl_coscrewf_t* __restrict__ ci,
  ksl_coscrewf_t* __restrict__ co);

void ksl_product_CoAdc(
  ksl_displacement_t* __restrict__ Di,
  ksl_coscrew_t* __restrict__ ci,
  ksl_coscrew_t* __restrict__ co);

void ksl_product_CoAdcf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_coscrewf_t* __restrict__ ci,
  ksl_coscrewf_t* __restrict__ co);

void ksl_product_CoAdinversec(
  ksl_displacement_t* __restrict__ Di,
  ksl_coscrew_t* __restrict__ ci,
  ksl_coscrew_t* __restrict__ co);

void ksl_product_CoAdinversecf(
  ksl_displacementf_t* __restrict__ Di,
  ksl_coscrewf_t* __restrict__ ci,
  ksl_coscrewf_t* __restrict__ co);

void ksl_normalize_qe(
  ksl_quaternion_t* __restrict__ qi);

void ksl_normalize_qf(
  ksl_quaternionf_t* __restrict__ qi);

void ksl_R_to_q(
  ksl_mat3x3_t* __restrict__ Ri,
  ksl_quaternion_t* __restrict__ qo);

void ksl_Rf_to_qf(
  ksl_mat3x3f_t* __restrict__ Ri,
  ksl_quaternionf_t* __restrict__ qo);

void ksl_q_to_R(
  ksl_quaternion_t* __restrict__ qi,
  ksl_mat3x3_t* __restrict__ Ro);

void ksl_qf_to_Rf(
  ksl_quaternionf_t* __restrict__ qi,
  ksl_mat3x3f_t* __restrict__ Ro);

void ksl_slerp(
  ksl_quaternion_t* __restrict__ q1i,
  ksl_quaternion_t* __restrict__ q2i,
  double* __restrict__ t,
  ksl_quaternion_t* __restrict__ qo);

void ksl_slerpf(
  ksl_quaternionf_t* __restrict__ q1i,
  ksl_quaternionf_t* __restrict__ q2i,
  float * __restrict__ t,
  ksl_quaternionf_t* __restrict__ qo);

void ksl_nlerp(
  ksl_quaternion_t* __restrict__ q1i,
  ksl_quaternion_t* __restrict__ q2i,
  double* __restrict__ t,
  ksl_quaternion_t* __restrict__ qo);

void ksl_nlerpf(
  ksl_quaternionf_t* __restrict__ q1i,
  ksl_quaternionf_t* __restrict__ q2i,
  float* __restrict__ t,
  ksl_quaternionf_t* __restrict__ qo);

double ksl_normalizeDegrees(
  double angle);

float ksl_normalizeDegreesf(
  float angle);

double ksl_normalizeRadians(
  double angle);

float ksl_normalizeRadiansf(
  float angle);

double catan2pi(
  double y,
  double x,
  double theta);

double catan1pi(
  double y,
  double x,
  double theta);

void ksl_mat3x3_getEulerAngles(
  ksl_mat3x3_t* __restrict__ rin,
  ksl_vector_t* __restrict__ angles,
  ksl_axis_enum_t axisType);

double ksl_mat3x3_getScalar(
  ksl_mat3x3_t* __restrict__ ri);

float ksl_mat3x3f_getScalar(
  ksl_mat3x3f_t* __restrict__ ri);

void ksl_mat4x4_getTranslation(
  ksl_mat4x4_t* __restrict__ Mi,
  ksl_vector_t* __restrict__ to);

void ksl_mat4x4f_getTranslation(
  ksl_mat4x4f_t* __restrict__ Mi,
  ksl_vectorf_t* __restrict__ to );

void ksl_add_dpiToq(
  ksl_vector_t* __restrict__ dpi,
  ksl_quaternion_t* __restrict__ qi);

void ksl_subtract_dpiFromq(
  ksl_vector_t* __restrict__ dpi,
  ksl_quaternion_t* __restrict__ qi);

void ksl_omega_to_dquaternion(
  ksl_vector_t* __restrict__ w,
  ksl_quaternion_t* __restrict__ qi,
  ksl_quaternion_t* __restrict__ dqo);

void ksl_array_lerp(
  const uint32_t qCount,
  const double p,
  double* __restrict__ y0,
  double* __restrict__ y1,
  double* __restrict__ yOut);

void ksl_gramSchmidt(
  const uint32_t m,
  const uint32_t n,
  double* __restrict__ A);

bool ksl_all_close(
  const uint32_t count,
  double* __restrict__ a1,
  double* __restrict__ a2);

#endif
