/**
@file
@brief Kinematics Subroutine Library
@author Kristopher Wehage
@copyright Kristopher Wehage
@date 2016

This library contains a collection of functions for translating,
rotating and displacing (translation and rotation) of spatial quantities including vectors and screws. The following quantities can be
translated, rotated or generally displaced:

s2_screw_t = A 6x1 Double Precision Screw

s2_screwf_t = A 6x1 Single Precision Screw

s2_vector_t = A 3x1 Double Precision Vector

s2_vectorf_t = A 3x1 Single Precision Vector

The following namnig conventions are used:

Translations are denoted as:

T = General Translation

Tx = Translation along x axis

Ty = Translation along y axis

Tz = Translation along z axis


Likewise, rotations are denoted as

R = General rotation

Rx = rotation about x axis

Ry = rotation about y axis

Rz = rotation about z axis


Displacements, consisting of both rotation and translation are
abbreviated using a "D"

D = General Displacement

Functions for computing products using the Inverse of D, R, and T are
are also provided. When the Inverse operation is specified, the term
Inverse appears in the function declaration AFTER D, R or T. For example:

The inverse of D is noted as "DInverse".

When transforming screw quantities the D, R and T operators take on
different forms depending on whether the spatial quantity lies in
Configuration Space or Function Space. Therefore, different functions
are provided for the two spaces.  Configuration space consists of the
spatial quantities that can be measured using a ruler or clock (i.e. position,
velocity, and acceleration). Function space (also called dual space)
consists of linear operators on configuration space (i.e. forces and torques).

In some literature, 6x1 quantities in configuration space are referred to as
screws and 6x1 quantities in function space are referred to as coscrews.

Readers are referred to *J.M. Selig, Gqometric Fundamentals of robotics (2005)*
or *R. Featherstone, Rigid Body Dynamics Algorithms (2008)* for more information.

@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THqoRY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

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

#ifndef _SHIFT_KINEMATICS_H_
#define _SHIFT_KINEMATICS_H_


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
#include <assert.h>

typedef struct {
  double x, y, z;
} s2_vector_t;
typedef struct {
  float x, y, z;
} s2_vectorf_t;
typedef struct {
  int x, y, z;
} s2_vectori_t;

typedef struct {
  double x, y, z, w;
} s2_quaternion_t;
typedef struct {
  float x, y, z, w;
} s2_quaternionf_t;

typedef struct {
  double m00, m10, m20, m01, m11, m21, m02, m12, m22;
} s2_mat3x3_t;
typedef struct {
  float m00, m10, m20, m01, m11, m21, m02, m12, m22;
} s2_mat3x3f_t;

typedef struct {
  double m00, m10, m20, m30,
         m01, m11, m21, m31,
         m02, m12, m22, m32,
         m03, m13, m23, m33;
} s2_mat4x4_t;

typedef struct {
  float m00, m10, m20, m30,
        m01, m11, m21, m31,
        m02, m12, m22, m32,
        m03, m13, m23, m33;
} s2_mat4x4f_t;

typedef struct {
  s2_mat3x3_t R;
  s2_vector_t T;
} s2_displacement_t;

typedef struct {
  s2_mat3x3f_t R;
  s2_vectorf_t T;
} s2_displacementf_t;

typedef struct {
  double m0, m1, m2, m3, m4, m5;
} s2_screw_t;
typedef struct {
  double m0, m1, m2, m3, m4, m5;
} s2_screwf_t;

typedef enum {
  XYZ,
  YZX,
  ZXY,
  XZY,
  YXZ,
  ZYX,
  XYX,
  XZX,
  YXY,
  YZY,
  ZXZ,
  ZYZ
} s2_axis_enum_t;

s2_vector_t* s2_vector();

s2_vectorf_t* s2_vectorf();

s2_vectori_t* s2_vectori();

s2_displacement_t* s2_displacement();

s2_displacementf_t* s2_displacementf();

s2_screw_t* s2_screw();

s2_screwf_t* s2_screwf();

s2_mat3x3_t* s2_mat3x3();

s2_mat3x3f_t* s2_mat3x3f();

s2_mat4x4_t* s2_mat4x4();

s2_mat4x4f_t* s2_mat4x4f();

void s2_displacement_toMat4x4(
  s2_displacement_t* d,
  s2_mat4x4_t* m);

void s2_displacementf_toMat4x4f(
  s2_displacementf_t* d,
  s2_mat4x4f_t* m);

void s2_displacement_toMat4x4f(
  s2_displacement_t* d,
  s2_mat4x4f_t* m);

void s2_vector_print(
  FILE* f,
  s2_vector_t* v);

void s2_quaternion_print(
  FILE* f,
  s2_quaternion_t* q);

void s2_screw_print(
  FILE* f,
  s2_screw_t* s);

void s2_mat3x3_print(
  FILE* f,
  s2_mat3x3_t* r);

void s2_mat3x3f_print(
  FILE* f,
  s2_mat3x3f_t* r);

void s2_displacement_print(
  FILE* f,
  s2_displacement_t* d);

void s2_mat4x4_print(
  FILE* f,
  s2_mat4x4_t* d);

void s2_mat4x4f_print(
  FILE* f,
  s2_mat4x4f_t* d);

void s2_array_print(
  FILE* f,
  int n,
  double* a);

void s2_array_print2D(
  FILE* f,
  int rowDim,
  int colDim,
  bool isRowMajor,
  double* a);

void s2_arrayi_print(
  FILE* f,
  int n,
  int* a);

void s2_triang_print(
  FILE* f,
  int n,
  double* a);

void s2_triangi_print(
  FILE* f,
  int n,
  int* a);

void s2_triangb_print(
  FILE* f,
  int n,
  bool* a);

void s2_dc(
  double* thetai,
  double dc[2]);

void s2_displacement_setIdentity(
  s2_displacement_t* d);

void s2_mat3x3_setIdentity(
  s2_mat3x3_t* r);

void s2_vector_set(
  s2_vector_t* v,
  int index,
  double value);

void s2_screw_set(
  s2_screw_t* s,
  int index,
  double value);

double s2_vector_l2norm(
  s2_vector_t* v);

float s2_vectorf_l2norm(
  s2_vectorf_t* v);

double s2_screw_l2norm(
  s2_screw_t* v);

float s2_screwf_l2norm(
  s2_screwf_t* v);

void s2_mat3x3_set(
  s2_mat3x3_t* R,
  int row,
  int column,
  double value);

void s2_mat3x3_setFromVectors(
  s2_mat3x3_t* R,
  s2_vector_t* x,
  s2_vector_t* y,
  s2_vector_t* z);

void s2_mat4x4_set(
  s2_mat4x4_t* R,
  int row,
  int column,
  double value);

void s2_displacement_set(
  s2_displacement_t* D,
  int row,
  int column,
  double value);

void s2_vectorf_set(
  s2_vectorf_t* v,
  int index,
  float value);

void s2_screwf_set(
  s2_screwf_t* s,
  int index,
  float value);

void s2_mat3x3f_set(
  s2_mat3x3f_t* R,
  int row,
  int column,
  float value);

void s2_mat4x4f_set(
  s2_mat4x4f_t* d,
  int row,
  int column,
  float value);

void s2_mat4x4f_setIdentity(
  s2_mat4x4f_t* R);

void s2_displacementf_set(
  s2_displacementf_t* D,
  int row,
  int column,
  float value);

double s2_vector_get(
  s2_vector_t* v,
  int index);

double s2_screw_get(
  s2_screw_t* s,
  int index);

double s2_mat3x3_get(
  s2_mat3x3_t* R,
  int row,
  int column);

double s2_displacement_get(
  s2_displacement_t* D,
  int row,
  int column);

float s2_vectorf_get(
  s2_vectorf_t* v,
  int index);

float s2_screwf_get(
  s2_screwf_t* s,
  int index);

float s2_mat3x3f_get(
  s2_mat3x3f_t* R,
  int row,
  int column);

float s2_displacementf_get(
  s2_displacementf_t* D,
  int row,
  int column);

void s2_displacement_getTranslation(
  s2_displacement_t* d,
  s2_vector_t* t);

void s2_vector_normalize(
  s2_vector_t* v);

void s2_vectorf_normalize(
  s2_vectorf_t* v);

void s2_vector_scale(
  double k,
  s2_vector_t* v);

void s2_vectorf_scale(
  double k,
  s2_vectorf_t* v);

void s2_vector_copy(
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_vectorf_copy(
  s2_vectorf_t* vi,
  s2_vectorf_t* vo);

void s2_vector_invert(
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_vectorf_invert(
  s2_vectorf_t* vi,
  s2_vectorf_t* vo);

double s2_dot_vv(
  s2_vector_t* vi0,
  s2_vector_t* vi1);

double s2_dot_vvf(
  s2_vectorf_t* vi0,
  s2_vectorf_t* vi1);

double s2_dot_ss(
  s2_screw_t* vi0,
  s2_screw_t* vi1);

double s2_dot_ssf(
  s2_screwf_t* vi0,
  s2_screwf_t* vi1);

void s2_product_vk(
  s2_vector_t* vi,
  double* ki,
  s2_vector_t* vo);

void s2_product_vinversek(
  s2_vector_t* vi,
  double* ki,
  s2_vector_t* vo);

void s2_product_vkinverse(
  s2_vector_t* vi,
  double* ki,
  s2_vector_t* vo);

void s2_product_vkf(
  s2_vectorf_t* vi,
  float* ki,
  s2_vectorf_t* vo);

void s2_add_vv(
  s2_vector_t* v1i,
  s2_vector_t* v2i,
  s2_vector_t* vo);

void s2_add_vvf(
  s2_vectorf_t* v1i,
  s2_vectorf_t* v2i,
  s2_vectorf_t* vo);

void s2_addequal_vv(
  s2_vector_t* v1i,
  s2_vector_t* v2i);

void s2_addequal_vvf(
  s2_vectorf_t* v1i,
  s2_vectorf_t* v2i);

void s2_subtract_vv(
  s2_vector_t* v1i,
  s2_vector_t* v2i,
  s2_vector_t* vo);

void s2_subtract_vvf(
  s2_vectorf_t* v1i,
  s2_vectorf_t* v2i,
  s2_vectorf_t* vo);

void s2_subtractequal_vv(
  s2_vector_t* v1i,
  s2_vector_t* v2i);

void s2_subtractequal_vvf(
  s2_vectorf_t* v1i,
  s2_vectorf_t* v2i);

void s2_axpy_vv(
  double scalar,
  s2_vector_t* v1x,
  s2_vector_t* v2y);

void s2_axpy_vvf(
  float scalar,
  s2_vectorf_t* v1x,
  s2_vectorf_t* v2y);

void s2_axpy_ss(
  double scalar,
  s2_screw_t* v1x,
  s2_screw_t* v2y);

void s2_axpy_ssf(
  float scalar,
  s2_screwf_t* v1x,
  s2_screwf_t* v2y);

void s2_cross_vv(
  s2_vector_t* v1i,
  s2_vector_t* v2i,
  s2_vector_t* vo);

void s2_cross_vvinverse(
  s2_vector_t* v1i,
  s2_vector_t* v2i,
  s2_vector_t* vo);

void s2_cross_vvf(
  s2_vectorf_t* v1i,
  s2_vectorf_t* v2i,
  s2_vectorf_t* vo);

void s2_cross_vvinversef(
  s2_vectorf_t* v1i,
  s2_vectorf_t* v2i,
  s2_vectorf_t* vo);

void s2_screw_copy(
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwf_copy(
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_screw_invert(
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwf_invert(
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_product_sk(
  s2_screw_t* si,
  double* ki,
  s2_screw_t* so);

void s2_product_skf(
  s2_screwf_t* si,
  float* ki,
  s2_screwf_t* so);

void s2_add_ss(
  s2_screw_t* s1i,
  s2_screw_t* s2i,
  s2_screw_t* so);

void s2_add_ssf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i,
  s2_screwf_t* so);

void s2_addequal_ss(
  s2_screw_t* s1i,
  s2_screw_t* s2i);

void s2_addequal_ssf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i);

void s2_subtract_ss(
  s2_screw_t* s1i,
  s2_screw_t* s2i,
  s2_screw_t* so);

void s2_subtract_ssf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i,
  s2_screwf_t* so);

void s2_subtractequal_ss(
  s2_screw_t* s1i,
  s2_screw_t* s2i);

void s2_subtractequal_ssf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i);

void s2_add_sst(
  s2_screw_t* si1,
  s2_screw_t* si2,
  s2_screw_t* so);

void s2_add_sstf(
  s2_screwf_t* si1,
  s2_screwf_t* si2,
  s2_screwf_t* so);

void s2_mat3x3_copy(
  s2_mat3x3_t* ri,
  s2_mat3x3_t* ro);

void s2_mat3x3f_copy(
  s2_mat3x3f_t* ri,
  s2_mat3x3f_t* ro);

void s2_mat3x3_invert(
  s2_mat3x3_t* ri,
  s2_mat3x3_t* ro);

void s2_mat3x3f_invert(
  s2_mat3x3f_t* ri,
  s2_mat3x3f_t* ro);

void s2_product_rv(
  s2_mat3x3_t* ri,
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_product_rvf(
  s2_mat3x3f_t* ri,
  s2_vectorf_t* vi,
  s2_vectorf_t* vo);

void s2_product_rvinverse(
  s2_mat3x3_t* ri,
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_product_rvinversef(
  s2_mat3x3f_t* ri,
  s2_vectorf_t* vi,
  s2_vectorf_t* vo);

void s2_product_rinversev(
  s2_mat3x3_t* ri,
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_product_rinversevf(
  s2_mat3x3f_t* ri,
  s2_vectorf_t* vi,
  s2_vectorf_t* vo);

void s2_product_rinversevinverse(
  s2_mat3x3_t* ri,
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_product_rinversevinversef(
  s2_mat3x3f_t* ri,
  s2_vectorf_t* vi,
  s2_vectorf_t* vo);

void s2_product_rs(
  s2_mat3x3_t* ri,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_product_rsinverse(
  s2_mat3x3_t* ri,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_product_rsf(
  s2_mat3x3f_t* ri,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_product_rsinversef(
  s2_mat3x3f_t* ri,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_product_rinverses(
  s2_mat3x3_t* ri,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_product_rinversesf(
  s2_mat3x3f_t* ri,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_product_rrx(
  s2_mat3x3_t* ri,
  double dc[2],
  s2_mat3x3_t* ro);

void s2_product_rrxf(
  s2_mat3x3f_t* ri,
  float dc[2],
  s2_mat3x3f_t* ro);

void s2_product_rrxinverse(
  s2_mat3x3_t* ri,
  double dc[2],
  s2_mat3x3_t* ro);

void s2_product_rrxinversef(
  s2_mat3x3f_t* ri,
  float dc[2],
  s2_mat3x3f_t* ro);

void s2_product_rry(
  s2_mat3x3_t* ri,
  double dc[2],
  s2_mat3x3_t* ro);

void s2_product_rryf(
  s2_mat3x3f_t* ri,
  float dc[2],
  s2_mat3x3f_t* ro);

void s2_product_rryinverse(
  s2_mat3x3_t* ri,
  double dc[2],
  s2_mat3x3_t* ro);

void s2_product_rryinversef(
  s2_mat3x3f_t* ri,
  float dc[2],
  s2_mat3x3f_t* ro);

void s2_product_rrz(
  s2_mat3x3_t* ri,
  double dc[2],
  s2_mat3x3_t* ro);

void s2_product_rrzf(
  s2_mat3x3f_t* ri,
  float dc[2],
  s2_mat3x3f_t* ro);

void s2_product_rrzinverse(
  s2_mat3x3_t* ri,
  double dc[2],
  s2_mat3x3_t* ro);

void s2_product_rrzinversef(
  s2_mat3x3f_t* ri,
  float dc[2],
  s2_mat3x3f_t* ro);

void s2_product_rr(
  s2_mat3x3_t* r1i,
  s2_mat3x3_t* r2i,
  s2_mat3x3_t* ro);

void s2_product_rrf(
  s2_mat3x3f_t* r1i,
  s2_mat3x3f_t* r2i,
  s2_mat3x3f_t* ro);

void s2_product_rrinverse(
  s2_mat3x3_t* r1i,
  s2_mat3x3_t* r2i,
  s2_mat3x3_t* ro);

void s2_product_rrinversef(
  s2_mat3x3f_t* r1i,
  s2_mat3x3f_t* r2i,
  s2_mat3x3f_t* ro);

void s2_product_rinverser(
  s2_mat3x3_t* r1i,
  s2_mat3x3_t* r2i,
  s2_mat3x3_t* ro);

void s2_product_rinverserf(
  s2_mat3x3f_t* r1i,
  s2_mat3x3f_t* r2i,
  s2_mat3x3f_t* ro);

void s2_product_rtx(
  s2_mat3x3_t* ri,
  double* ti,
  s2_vector_t* to);

void s2_product_rtxf(
  s2_mat3x3f_t* ri,
  float* ti,
  s2_vectorf_t* to);

void s2_product_rtxinverse(
  s2_mat3x3_t* ri,
  double* ti,
  s2_vector_t* to);

void s2_product_rtxinversef(
  s2_mat3x3f_t* ri,
  float* ti,
  s2_vectorf_t* to);

void s2_product_rty(
  s2_mat3x3_t* ri,
  double* ti,
  s2_vector_t* to);

void s2_product_rtyf(
  s2_mat3x3f_t* ri,
  float* ti,
  s2_vectorf_t* to);

void s2_product_rtyinverse(
  s2_mat3x3_t* ri,
  double* ti,
  s2_vector_t* to);

void s2_product_rtyinversef(
  s2_mat3x3f_t* ri,
  float* ti,
  s2_vectorf_t* to);

void s2_product_rtz(
  s2_mat3x3_t* ri,
  double* ti,
  s2_vector_t* to);

void s2_product_rtzf(
  s2_mat3x3f_t* ri,
  float* ti,
  s2_vectorf_t* to);

void s2_product_rtzinverse(
  s2_mat3x3_t* ri,
  double* ti,
  s2_vector_t* to);

void s2_product_rtzinversef(
  s2_mat3x3f_t* ri,
  float* ti,
  s2_vectorf_t* to);

void s2_add_ttx(
  s2_vector_t* t1i,
  double* t2i,
  s2_vector_t* to);

void s2_add_ttxf(
  s2_vectorf_t* t1i,
  float* t2i,
  s2_vectorf_t* to);

void s2_add_ttxinverse(
  s2_vector_t* t1i,
  double* t2i,
  s2_vector_t* to);

void s2_add_ttxinversef(
  s2_vectorf_t* t1i,
  float* t2i,
  s2_vectorf_t* to);

void s2_add_tty(
  s2_vector_t* t1i,
  double* t2i,
  s2_vector_t* to);

void s2_add_ttyf(
  s2_vectorf_t* t1i,
  float* t2i,
  s2_vectorf_t* to);

void s2_add_ttyinverse(
  s2_vector_t* t1i,
  double* t2i,
  s2_vector_t* to);

void s2_add_ttyinversef(
  s2_vectorf_t* t1i,
  float* t2i,
  s2_vectorf_t* to);

void s2_add_ttz(
  s2_vector_t* t1i,
  double* t2i,
  s2_vector_t* to);

void s2_add_ttzf(
  s2_vectorf_t* t1i,
  float* t2i,
  s2_vectorf_t* to);

void s2_add_ttzinverse(
  s2_vector_t* t1i,
  double* t2i,
  s2_vector_t* to);

void s2_add_ttzinversef(
  s2_vectorf_t* t1i,
  float* t2i,
  s2_vectorf_t* to);

void s2_displacement_invert(
  s2_displacement_t* Di,
  s2_displacement_t* Do);

void s2_displacementf_invert(
  s2_displacementf_t* Di,
  s2_displacementf_t* Do);

void s2_displacement_copy(
  s2_displacement_t* Di,
  s2_displacement_t* Do);

void s2_displacementf_copy(
  s2_displacementf_t* Di,
  s2_displacementf_t* Do);

void s2_product_drx(
  s2_displacement_t* Di,
  double dc[2],
  s2_displacement_t* Do);

void s2_product_drxf(
  s2_displacementf_t* Di,
  float dc[2],
  s2_displacementf_t* Do);

void s2_product_drxinverse(
  s2_displacement_t* Di,
  double dc[2],
  s2_displacement_t* Do);

void s2_product_drxinversef(
  s2_displacementf_t* Di,
  float dc[2],
  s2_displacementf_t* Do);

void s2_product_dry(
  s2_displacement_t* Di,
  double dc[2],
  s2_displacement_t* Do);

void s2_product_dryf(
  s2_displacementf_t* Di,
  float dc[2],
  s2_displacementf_t* Do);

void s2_product_dryinverse(
  s2_displacement_t* Di,
  double dc[2],
  s2_displacement_t* Do);

void s2_product_dryinversef(
  s2_displacementf_t* Di,
  float dc[2],
  s2_displacementf_t* Do);

void s2_product_drz(
  s2_displacement_t* Di,
  double dc[2],
  s2_displacement_t* Do);

void s2_product_drzf(
  s2_displacementf_t* Di,
  float dc[2],
  s2_displacementf_t* Do);

void s2_product_drzinverse(
  s2_displacement_t* Di,
  double dc[2],
  s2_displacement_t* Do);

void s2_product_drzinversef(
  s2_displacementf_t* Di,
  float dc[2],
  s2_displacementf_t* Do);

void s2_product_dt(
  s2_displacement_t* Di,
  s2_vector_t* vi,
  s2_displacement_t* Do);

void s2_product_dtinverse(
  s2_displacement_t* Di,
  s2_vector_t* vi,
  s2_displacement_t* Do);

void s2_product_dv(
  s2_displacement_t* Di,
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_product_dinversev(
  s2_displacement_t* Di,
  s2_vector_t* vi,
  s2_vector_t* vo);

void s2_product_dtf(
  s2_displacementf_t* Di,
  s2_vectorf_t* vi,
  s2_displacementf_t* Do);

void s2_product_dinverset(
  s2_displacement_t* Di,
  s2_vector_t* vi,
  s2_displacement_t* Do);

void s2_product_dinversetf(
  s2_displacementf_t* Di,
  s2_vectorf_t* vi,
  s2_displacementf_t* Do);

void s2_product_dtx(
  s2_displacement_t* Di,
  double* ti,
  s2_displacement_t* Do);

void s2_product_dtxinverse(
  s2_displacement_t* Di,
  double* ti,
  s2_displacement_t* Do);

void s2_product_dtxinversef(
  s2_displacementf_t* Di,
  float* ti,
  s2_displacementf_t* Do);

void s2_product_dty(
  s2_displacement_t* Di,
  double* ti,
  s2_displacement_t* Do);

void s2_product_dtyf(
  s2_displacementf_t* Di,
  float* ti,
  s2_displacementf_t* Do);

void s2_product_dtyinverse(
  s2_displacement_t* Di,
  double* ti,
  s2_displacement_t* Do);

void s2_product_dtyinversef(
  s2_displacementf_t* Di,
  float* ti,
  s2_displacementf_t* Do);

void s2_product_dtz(
  s2_displacement_t* Di,
  double* ti,
  s2_displacement_t* Do);

void s2_product_dtzf(
  s2_displacementf_t* Di,
  float* ti,
  s2_displacementf_t* Do);

void s2_product_dtzinverse(
  s2_displacement_t* Di,
  double* ti,
  s2_displacement_t* Do);

void s2_product_dr(
  s2_displacement_t* Di,
  s2_mat3x3_t* Ri,
  s2_displacement_t* Do);

void s2_product_drf(
  s2_displacementf_t* Di,
  s2_mat3x3f_t* Ri,
  s2_displacementf_t* Do);

void s2_product_drinverse(
  s2_displacement_t* Di,
  s2_mat3x3_t* Ri,
  s2_displacement_t* Do);

void s2_product_drinversef(
  s2_displacementf_t* Di,
  s2_mat3x3f_t* Ri,
  s2_displacementf_t* Do);

void s2_product_dd(
  s2_displacement_t* D1i,
  s2_displacement_t* D2i,
  s2_displacement_t* Do);

void s2_product_ddf(
  s2_displacementf_t* D1i,
  s2_displacementf_t* D2i,
  s2_displacementf_t* Do);

void s2_product_ddinverse(
  s2_displacement_t* D1i,
  s2_displacement_t* D2i,
  s2_displacement_t* Do);

void s2_product_ddinversef(
  s2_displacementf_t* D1i,
  s2_displacementf_t* D2i,
  s2_displacementf_t* Do);

void s2_htx(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_htxf(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_hty(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_htyf(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_htz(
  s2_displacement_t* ri,
  s2_screw_t* ho);

void s2_htzf(
  s2_displacementf_t* ri,
  s2_screwf_t* ho);

void s2_hrx(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_hrxf(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_hry(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_hrz(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_hrzf(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_htxinverse(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_htxinversef(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_htyinverse(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_htyinversef(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_htzinverse(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_htzinversef(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_hrxinverse(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_hrxinversef(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_hryinverse(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_hryinversef(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_hrzinverse(
  s2_displacement_t* Di,
  s2_screw_t* ho);

void s2_hrzinversef(
  s2_displacementf_t* Di,
  s2_screwf_t* ho);

void s2_screwcross_ss(
  s2_screw_t* s1i,
  s2_screw_t* s2i,
  s2_screw_t* so);

void s2_screwcross_ssf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i,
  s2_screwf_t* so);

void s2_screwcross_sst(
  s2_screw_t* s1i,
  s2_screw_t* s2i,
  s2_screw_t* so);

void s2_screwcross_sstf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i,
  s2_screwf_t* so);

void s2_screwproduct_dts(
  s2_vector_t* ti,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwproduct_dtsf(
  s2_vectorf_t* ti,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_screwproduct_dtinverses(
  s2_vector_t* ti,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwproduct_dtinversesf(
  s2_vectorf_t* ti,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_screwproduct_dtsinverse(
  s2_vector_t* ti,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwproduct_dtsinversef(
  s2_vectorf_t* ti,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_screwproduct_ds(
  s2_displacement_t* Di,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwproduct_dsinverse(
  s2_displacement_t* Di,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwproduct_dsf(
  s2_displacementf_t* Di,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_screwproduct_dinverses(
  s2_displacement_t* Di,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_screwproduct_dinversesf(
  s2_displacementf_t* Di,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_coscrewcross_ss(
  s2_screw_t* s1i,
  s2_screw_t* s2i,
  s2_screw_t* so);

void s2_coscrewcross_ssf(
  s2_screwf_t* s1i,
  s2_screwf_t* s2i,
  s2_screwf_t* so);

void s2_coscrewproduct_dts(
  s2_vector_t* ti,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_coscrewproduct_dtsf(
  s2_vectorf_t* ti,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_coscrewproduct_dtsinverse(
  s2_vector_t* ti,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_coscrewproduct_dtsinversef(
  s2_vectorf_t* ti,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_coscrewproduct_ds(
  s2_displacement_t* Di,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_coscrewproduct_dsf(
  s2_displacementf_t* Di,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_coscrewproduct_dinverses(
  s2_displacement_t* Di,
  s2_screw_t* si,
  s2_screw_t* so);

void s2_coscrewproduct_dinversesf(
  s2_displacementf_t* Di,
  s2_screwf_t* si,
  s2_screwf_t* so);

void s2_quaternion_normalize(
  s2_quaternion_t* qi);

void s2_quaternionf_normalize(
  s2_quaternionf_t* qi);

void s2_mat3x3_toQuaternion(
  s2_mat3x3_t* ri,
  s2_quaternion_t* qo);

void s2_mat3x3f_toQuaternion(
  s2_mat3x3f_t* ri,
  s2_quaternionf_t* qo);

void s2_quaternion_toMat3x3(
  s2_quaternion_t* qi,
  s2_mat3x3_t* ro);

void s2_quaternionf_toMat3x3f(
  s2_quaternionf_t* qi,
  s2_mat3x3f_t* ro);

void s2_quaternion_slerp(
  s2_quaternion_t* q1i,
  s2_quaternion_t* q2i,
  double* t,
  s2_quaternion_t* qo);

void s2_quaternionf_slerp(
  s2_quaternionf_t* q1i,
  s2_quaternionf_t* q2i,
  float* t,
  s2_quaternionf_t* qo);

void s2_quaternion_nlerp(
  s2_quaternion_t* q1i,
  s2_quaternion_t* q2i,
  double* t,
  s2_quaternion_t* qo);

void s2_quaternionf_nlerp(
  s2_quaternionf_t* q1i,
  s2_quaternionf_t* q2i,
  float* t,
  s2_quaternionf_t* qo);

double s2_normalizeDegrees(
  double angle);

float s2_normalizeDegreesf(
  float angle);

double s2_normalizeRadians(
  double angle);

float s2_normalizeRadiansf(
  float angle);

double catan2pi(
  double y,
  double x,
  double theta);

double catan1pi(
  double y,
  double x,
  double theta);

void s2_mat3x3_getEulerAngles(
  s2_mat3x3_t* rin,
  s2_vector_t* angles,
  s2_axis_enum_t axisType);

double s2_mat3x3_getScalar(
  s2_mat3x3_t* ri);

float s2_mat3x3f_getScalar(
  s2_mat3x3f_t* ri);

void s2_mat4x4_getTranslation(
  s2_mat4x4_t* Mi,
  s2_vector_t* to);

void s2_mat4x4f_getTranslation(
  s2_mat4x4f_t* Mi,
  s2_vectorf_t* to);

void s2_mat4x4f_get(
  s2_mat4x4f_t* Mi,
  int row,
  int column);

void s2_add_dpiToq(
  s2_vector_t* dpi,
  s2_quaternion_t* qi);

void s2_subtract_dpiFromq(
  s2_vector_t* dpi,
  s2_quaternion_t* qi);

void s2_omega_to_dquaternion(
  s2_vector_t* w,
  s2_quaternion_t* qi,
  s2_quaternion_t* dqo);

void s2_array_lerp(
  int qCount,
  double p,
  double* y0,
  double* y1,
  double* yOut);

void s2_gramSchmidt(
  int m,
  int n,
  double* A);

bool s2_all_close(int count, double* a1, double* a2);

#endif
