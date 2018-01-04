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

Vectors quantities are transformed using 3x3 rotation
matrices (i.e. SO3 matrices) and column vectors. The 3x3 rotation datastructure
is denoted as:

R = 3x3 rotation matrix in column major order, bksl_mat3x3_t

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
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS uint32_tERRUPTION)
HOWEVER CAUSED AND ON ANY THqoRY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include "bksl.h"

/*!
@name Utilities for initializing basic datastructures

Utilities to  vectors, screws, SO3 (mat3) rotation matrices and SE3 (mat4)
transformations
@{
*/

/*! @brief s a vector_t datastructure (an array of doubles of length 3)

*/
bksl_vec3_t* bksl_vec3() {
  return calloc(1, sizeof(bksl_vec3_t));
}

/*! @brief s a vectorf_t datastructure (an array of floats of length 3)

*/
bksl_vec3f_t* bksl_vec3f() {
  return calloc(1, sizeof(bksl_vec3f_t));
}

/*! @brief s a vectori_t datastructure (an array of uint32_ts of length 3)

*/
bksl_vec3i_t* bksl_vec3i() {
  return calloc(1, sizeof(bksl_vec3i_t));
}


/*!
@brief Normalize a double precision vector

@param v [in/out] vector to normalize
*/
inline void bksl_vec3_normalize(
  bksl_vec3_t* __restrict__ v) {

  double norm = bksl_vec3_l2norm(v);
  for(uint32_t i = 0; i < 3; i++) {
    ((double*)v)[i] /= norm;
  }

  // double norm = sqrt(cblas_ddot(3, (double*)v, 1, (double*)v, 1));
  // cblas_dscal(3, 1.0 / norm, (double*)v, 1);
}


/*!
@brief scale a double precision vector
@param [in/out] vector to scale
*/
inline void bksl_vec3_scale(
  double k,
  bksl_vec3_t* __restrict__ v) {

  cblas_dscal(3, k, (double*)v, 1);
}


/*!
@brief Normalize a double precision vector

@param v [in/out] vector to normalize
*/
inline void bksl_vec3f_normalize(
  bksl_vec3f_t* __restrict__ v) {

  float norm = sqrt(cblas_sdot(3, (float*)v, 1, (float*)v, 1));
  cblas_sscal(3, 1.0 / norm, (float*)v, 1);
}


/*!
@brief scale a double precision vector
@param [in/out] vector to scale
*/
inline void bksl_vec3f_scale(
  double k,
  bksl_vec3f_t* __restrict__ v) {

  cblas_sscal(3, k, (float*)v, 1);
}


/*! @brief initializes a general bksl_displacement_t datastructure

A general displacement is an array of doubles of length 12, consisting of a
rotation matrix in column major order followed by a translation vector.

@returns A displacement with the rotation component set to identity and
translation set to zero.
*/
bksl_displacement_t* bksl_displacement() {

  bksl_displacement_t* D =
    calloc(1, sizeof(bksl_displacement_t));
  D->R.m00 = 1.0;
  D->R.m11 = 1.0;
  D->R.m22 = 1.0;
  return D;
}

/*! @brief s a general bksl_displacementf_t datastructure

A general displacement is an array of float of length 12, consisting of a
rotation matrix in column major order followed by a translation vector
*/
bksl_displacementf_t* bksl_displacementf() {

  bksl_displacementf_t* D =
    calloc(1, sizeof(bksl_displacementf_t));
  D->R.m00 = 1.f;
  D->R.m11 = 1.f;
  D->R.m22 = 1.f;
  return D;
}

/*! @brief s a bksl_screw_t datastructure (an array of doubles of length 6)

*/
bksl_screw_t* bksl_screw() {
  return calloc(1, sizeof(bksl_screw_t));
}


/*! @brief s a bksl_screw_t datastructure (an array of floats of length 6)

*/
bksl_screwf_t* bksl_screwf() {
  return calloc(1, sizeof(bksl_screwf_t));
}


/*! @brief s a double precision bksl_mat3x3_t datastructure for a general
rotation (i.e. a SO3 matrix). The matrix is specified in column major order.

@returns A bksl_mat3x3_t rotation matrix set to identity.
*/
bksl_mat3x3_t* bksl_mat3x3() {

  bksl_mat3x3_t* mat3 = calloc(1, sizeof(bksl_mat3x3_t));
  mat3->m00 = 1.0;
  mat3->m11 = 1.0;
  mat3->m22 = 1.0;
  return mat3;
}


/*! @brief s a single precision bksl_mat3x3f_t datastructure for a general
rotation (i.e. a SO3 matrix). The matrix is specified in column major order.

@returns A bksl_mat3x3f_t rotation matrix set to identity.
*/
bksl_mat3x3f_t* bksl_mat3x3f() {

  bksl_mat3x3f_t* mat3 =
    calloc(1, sizeof(bksl_mat3x3f_t));
  mat3->m00 = 1.f;
  mat3->m11 = 1.f;
  mat3->m22 = 1.f;
  return mat3;
}


/*! @brief s a double precision bksl_Mat4 datastructure for a general
transformation (i.e. a SE3 matrix). The matrix is specified in column major order.

@returns A bksl_Mat4 SE3 matrix set to identity.
*/
bksl_mat4x4_t* bksl_mat4x4() {

  bksl_mat4x4_t* mat4 = calloc(1, sizeof(bksl_mat4x4_t));
  mat4->m00 = 1.0;
  mat4->m11 = 1.0;
  mat4->m22 = 1.0;
  mat4->m33 = 1.0;
  return mat4;
}


/*! @brief s a single precision bksl_Mat4f datastructure for a general
transformation (i.e. a SE3 matrix). The matrix is specified in column major order.

@returns A bksl_Mat4f SE3 matrix set to identity.
*/
bksl_mat4x4f_t* bksl_mat4x4f() {

  bksl_mat4x4f_t* mat4 =
    calloc(1, sizeof(bksl_mat4x4f_t));
  mat4->m00 = 1.f;
  mat4->m11 = 1.f;
  mat4->m22 = 1.f;
  mat4->m33 = 1.f;
  return mat4;
}


/*! @brief converts a double precision bksl_displacement_t datastructure to bksl_Mat4
format.

*/
inline void bksl_displacement_toMat4x4(
  bksl_displacement_t* __restrict__ Di,
  bksl_mat4x4_t* __restrict__ Mo) {

  memset(Mo, 0, sizeof(bksl_mat4x4_t));
  memcpy(&Mo->m00, &Di->R.m00, 3 * sizeof(double));
  memcpy(&Mo->m01, &Di->R.m01, 3 * sizeof(double));
  memcpy(&Mo->m02, &Di->R.m02, 3 * sizeof(double));
  memcpy(&Mo->m03, &Di->t, 3 * sizeof(double));
  Mo->m33 = 1.0;

}


/*! @brief converts a single precision bksl_displacementf_t datastructure to bksl_Mat4f
format.

*/
inline void bksl_displacementf_toMat4x4f(
  bksl_displacementf_t* __restrict__ Di,
  bksl_mat4x4f_t* __restrict__ Mo) {

  memset(Mo, 0, sizeof(bksl_mat4x4f_t));
  memcpy(&Mo->m00, &Di->R.m00, 3 * sizeof(float));
  memcpy(&Mo->m01, &Di->R.m01, 3 * sizeof(float));
  memcpy(&Mo->m02, &Di->R.m02, 3 * sizeof(float));
  memcpy(&Mo->m03, &Di->t, 3 * sizeof(float));
  Mo->m33 = 1.f;
}


/*! @brief converts a double precision bksl_displacement_t datastructure to a
single precision bksl_Mat4f format.

*/
inline void bksl_displacement_toMat4x4f(
  bksl_displacement_t* __restrict__ Di,
  bksl_mat4x4f_t* __restrict__ Mo) {

  memset(Mo, 0, sizeof(bksl_mat4x4_t));
  for(uint32_t i = 0; i < 3; i++) {
    for(uint32_t j = 0; j < 3; j++) {
      bksl_mat4x4f_set(Mo, i, j, (float)bksl_mat3x3_get(&Di->R, i, j));
    }
  }
  for(uint32_t i = 0; i < 3; i++) {
    bksl_mat4x4f_set(Mo, i, 3, (float)bksl_vec3_get(&Di->t, i));
  }
  Mo->m33 = 1.f;
}


/*!
@}
@name Printing utilities
Utilities to print vectors, screws, rotation matrices and SE3 transformations
@{
*/

/*!
@brief Utility to print a 3x1 column vector

prints a 3x1 column vector as a row vector

@param ti 3x1 column vector
*/
void bksl_vec3_print(
  FILE* f,
  bksl_vec3_t* __restrict__ ti) {

  fprintf(f, "%f %f %f\n", ti->x, ti->y, ti->z);

}


/*!
@brief Utility to print a 3x1 column vector

prints a 3x1 column vector as a row vector

@param ti 3x1 column vector
*/
void bksl_vec3f_print(
  FILE* f,
  bksl_vec3f_t* __restrict__ ti) {

  fprintf(f, "%f %f %f\n", ti->x, ti->y, ti->z);

}


/*!
@brief Utility to print a 6x1 screw vector

prints a 6x1 column vector as a row vector

@param si 6x1 column vector
*/
void bksl_screw_print(
  FILE* f,
  bksl_screw_t* __restrict__ si) {

  fprintf(
    f,
    "%g %g %g %g %g %g\n",
    si->v.x, si->v.y, si->v.z, si->w.x, si->w.y, si->w.z);

}

/*!
@brief Utility to print a 3x3 rotation matrix

prints a rotation matrix

@param ri 3x3 rotation matrix
*/
void bksl_mat3x3_print(
  FILE* f,
  bksl_mat3x3_t* __restrict__ ri) {

  fprintf(f, "%f %f %f\n", ri->m00, ri->m01, ri->m02);
  fprintf(f, "%f %f %f\n", ri->m10, ri->m11, ri->m12);
  fprintf(f, "%f %f %f\n", ri->m20, ri->m21, ri->m22);
}

/*!
@brief Utility to print a 3x3 rotation matrix

prints a rotation matrix

@param ri 3x3 rotation matrix
*/
void bksl_mat3x3f_print(
  FILE* f,
  bksl_mat3x3f_t* __restrict__ ri) {

  fprintf(f, "%f %f %f\n", ri->m00, ri->m01, ri->m02);
  fprintf(f, "%f %f %f\n", ri->m10, ri->m11, ri->m12);
  fprintf(f, "%f %f %f\n", ri->m20, ri->m21, ri->m22);
}

/*!
@brief Utility to print a general displacement, \f$D_i\f$

prints a general displacement as a 4x4 matrix for transforming
homogeneous coordinates. Note that the displacement is stored in
memory as a 12 x 1 contiguous array with terms of the rotation matrix
in the first 9 entries of the array (in row major order), followed by
translation terms in the last 3 entries of the array.

@param Di A general displacement, \f$D_i\f$
*/
void bksl_displacement_print(
  FILE* f,
  bksl_displacement_t* __restrict__ Di) {

  fprintf(f, "  % 9.9f % 9.9f % 9.9f % 9.9f\n",
          Di->R.m00, Di->R.m01, Di->R.m02, Di->t.x);
  fprintf(f, "  % 9.9f % 9.9f % 9.9f % 9.9f\n",
          Di->R.m10, Di->R.m11, Di->R.m12, Di->t.y);
  fprintf(f, "  % 9.9f % 9.9f % 9.9f % 9.9f\n",
          Di->R.m20, Di->R.m21, Di->R.m22, Di->t.z);
  fprintf(f, "  ");
  for(int i = 0; i < 3; i++) {
    fprintf(f, " 0.000000000 ");
  }
  fprintf(f, " 1.000000000\n");
}

/*!
@brief Utility to print a SE3 mat4 matrix

prints a bksl_Mat4f as a 4x4 matrix for transforming
homogeneous coordinates. Note that the displacement is stored in
memory as a 12 x 1 contiguous array with terms of the rotation matrix
in the first 9 entries of the array (in row major order), followed by
translation terms in the last 3 entries of the array.

@param Di A general displacement, \f$D_i\f$
*/
void bksl_mat4x4_print(
  FILE* f,
  bksl_mat4x4_t* __restrict__ Di) {

  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m00, Di->m01, Di->m02, Di->m03);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m10, Di->m11, Di->m12, Di->m13);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m20, Di->m21, Di->m22, Di->m23);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m30, Di->m31, Di->m32, Di->m33);
}

/*!
@brief Utility to print a SE3 mat4 matrix

prints a bksl_Mat4f as a 4x4 matrix for transforming
homogeneous coordinates. Note that the displacement is stored in
memory as a 12 x 1 contiguous array with terms of the rotation matrix
in the first 9 entries of the array (in row major order), followed by
translation terms in the last 3 entries of the array.

@param Di A general displacement, \f$D_i\f$
*/
void bksl_mat4x4f_print(
  FILE* f,
  bksl_mat4x4f_t* __restrict__ Di) {

  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m00, Di->m01, Di->m02, Di->m03);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m10, Di->m11, Di->m12, Di->m13);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m20, Di->m21, Di->m22, Di->m23);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n",
          Di->m30, Di->m31, Di->m32, Di->m33);
}


/*!
@brief utility to print a general array of double of length n on one line
*/
void bksl_array_print(
  FILE* f,
  const uint32_t n,
  double* __restrict__ a) {

  for(uint32_t i = 0; i < n; i++) {
    fprintf(f, "%0.6g ", a[i]);
  }
  fprintf(f, "\n");
}


/*!
@brief utility to print a 2D array stored in a single pointer
with dimension rowDim * colDim
*/
void bksl_array2D_print(
  FILE* f,
  const uint32_t rowDim,
  const uint32_t colDim,
  bksl_matrix_enum_t matrixType,
  double* __restrict__ A) {

  switch(matrixType) {

    case bksl_ROWMAJOR: {
      for(uint32_t i = 0; i < rowDim; i++) {
        fprintf(f, "  [");
        for(int j = 0; j < colDim; j++) {
          fprintf(f, "% 0.10f", A[i * colDim + j]);
          if(j != colDim - 1) {
            fprintf(f, ", ");
          }
        }
        fprintf(f, "]\n");
      }
    }
    break;

    case bksl_COLMAJOR: {
      for(int i = 0; i < rowDim; i++) {
        fprintf(f, "  ");
        for(int j = 0; j < colDim; j++) {
          fprintf(f, "% 0.10f ", A[i + j * rowDim]);
        }
        fprintf(f, "\n");
      }
    }
    break;

  }
}


/*!
@brief utility to print a general array of uint32_ts of length n on one line
*/
void bksl_arrayi_print(
  FILE* f,
  const uint32_t n,
  int* __restrict__ a) {

  for(uint32_t i = 0; i < n; i++) {
    fprintf(f, "%d ", a[i]);
  }
  fprintf(f, "\n");
}


/*!
@brief utility to print a lower triangular matrix of doubles stored in a
linear array
*/
void bksl_triang_print(
  FILE* f,
  const uint32_t n,
  double* __restrict__ A) {

  for(uint32_t i = 0; i < n; i++) {
    uint32_t i_s = (i * (i + 1) / 2);
    for(uint32_t j = 0; j < i + 1; j++) {
      fprintf(f, "% 9.4f ", A[i_s + j]);
    }
    fprintf(f, "\n");
  }
}


/*!
@brief utility to print a lower triangular matrix of uint32_tegers stored in a
linear array
*/
void bksl_triangi_print(
  FILE* f,
  const uint32_t n,
  int* __restrict__ A) {

  for(uint32_t i = 0; i < n; i++) {
    uint32_t i_s = (i * (i + 1) / 2);
    for(uint32_t j = 0; j < i + 1; j++) {
      fprintf(f, "% d ", A[i_s + j]);
    }
    fprintf(f, "\n");
  }
}


/*!
@brief utility to print a lower triangular matrix of uint32_tegers stored in a
linear array
*/
void bksl_triangb_print(
  FILE* f,
  const uint32_t n,
  bool* __restrict__ A) {

  for(int i = 0; i < n; i++) {
    uint32_t i_s = (i * (i + 1) / 2);
    for(int j = 0; j < i + 1; j++) {
      fprintf(f, "%s ", A[i_s + j] ? "1" : "0");
    }
    fprintf(f, "\n");
  }
}


/*!
@}
@name Direction cosine utilities
Utilities for computing direction cosines
@{
*/

/*!
@brief Computes direction cosine and sine terms

Computes direction cosine and sine terms, terms are saved to array dc

@param anglqi angle, in radians
@param dco Array of cos, sin terms, \f$cos(theta_i) \rightarrow dc_o[0]\f$,
\f$sin(thetai) \rightarrow dc_o[1]\f$
*/
inline void bksl_dc(
  double* __restrict__ thetai,
  double dc[2]) {

  dc[0] = sin(*thetai);
  dc[1] = cos(*thetai);
}


/*!
@}
@name Initialization utilities
Utilities for initializing general displacements, vectors and screws
@{
*/

/*!
@brief Set an SE3 displacement to identity

Overwrites an SE3 matrix with the identity matrix.

@param D an SE3 matrix to set to identity, the resulting matrix will be overwritten
*/

inline void bksl_displacement_setIdentity(
  bksl_displacement_t* __restrict__ D) {

  memset(D, 0, sizeof(bksl_displacement_t));
  D->R.m00 = 1.0;
  D->R.m11 = 1.0;
  D->R.m22 = 1.0;
}



/*!
@brief Set a rotation matrix to identity

Overwrites a rotation matrix with the identity matrix.

@param R an SE3 matrix to set to identity, the resulting matrix will be overwritten
*/

inline void bksl_mat3x3_setIdentity(
  bksl_mat3x3_t* __restrict__ R) {

  memset(R, 0, sizeof(bksl_mat3x3_t));
  R->m00 = 1.0;
  R->m11 = 1.0;
  R->m22 = 1.0;
}


/*!
@brief Set a rotation matrix to identity

Overwrites a rotation matrix with the identity matrix.

@param R an SE3 matrix to set to identity, the resulting matrix will be overwritten
*/

inline void bksl_mat4x4f_setIdentity(
  bksl_mat4x4f_t* __restrict__ M) {

  memset(M, 0, sizeof(bksl_mat4x4f_t));
  ((float*)M)[0 * 4 + 0] = 1.f;
  ((float*)M)[1 * 4 + 1] = 1.f;
  ((float*)M)[2 * 4 + 2] = 1.f;
  ((float*)M)[3 * 4 + 3] = 1.f;
}


/*!
@}
@name utilities to set and get quantities by index

@{
*/

/*!
@brief set a value of vector
*/
inline void bksl_vec3_set(
  bksl_vec3_t* __restrict__ v,
  const uint32_t index,
  double value) {

  // if(index > 2 || index < 0) {
  //   fprintf(stderr, "ksl_vec3_set: out of bounds\n");
  //   return;
  // }
  ((double*)v)[index] = value;
}


/*!
@brief l2 norm (length) of double precision vector
*/
inline double bksl_vec3_l2norm(
  bksl_vec3_t* __restrict__ v) {

  double sum = 0;
  for(uint32_t i = 0; i < 3; i++) {
    sum += bksl_vec3_get(v, i) * bksl_vec3_get(v, i);
  }
  return sqrt(sum);

  /* previously was */
  // sqrt(
  //   cblas_ddot(
  //     3,
  //     (double*)v,
  //     1,
  //     (double*)v,
  //     1));
}


/*!
@brief l2 norm (length) of single precision vector
*/
inline float bksl_vec3f_l2norm(
  bksl_vec3f_t* __restrict__ v) {

  double sum = 0;
  for(uint32_t i = 0; i < 3; i++) {
    sum += bksl_vec3f_get(v, i) * bksl_vec3f_get(v, i);
  }
  return sqrt(sum);

  /* previously was */
  // return
  //   sqrt(
  //     cblas_sdot(
  //       3,
  //       (float*)v,
  //       1,
  //       (float*)v,
  //       1));
}


/*!
@brief l2 norm of double precision screw
*/
inline double bksl_screw_l2norm(
  bksl_screw_t* __restrict__ s) {

  double sum = 0;
  for(uint32_t i = 0; i < 6; i++) {
    sum += bksl_screw_get(s, i) * bksl_screw_get(s, i);
  }
  return sqrt(sum);
}


/*!
@brief l2 norm of single precision screw
*/
inline float bksl_screwf_l2norm(
  bksl_screwf_t* __restrict__ s) {

  float sum = 0;
  for(uint32_t i = 0; i < 6; i++) {
    sum += bksl_screwf_get(s, i) * bksl_screwf_get(s, i);
  }
  return sqrt(sum);

  /* compare to cblas function */
  // return
  //   sqrt(
  //     cblas_sdot(
  //       6,
  //       (float*)v,
  //       1,
  //       (float*)v,
  //       1));
}


/*!
@brief set a value of a screw
*/
inline void bksl_screw_set(
  bksl_screw_t* __restrict__ s,
  const uint32_t index,
  const double value) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return;
  // }
  ((double*)s)[index] = value;
}


/*!
@brief set a value of a screw
*/
inline void bksl_coscrew_set(
  bksl_coscrew_t* __restrict__ c,
  const uint32_t index,
  const double value) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return;
  // }
  ((double*)c)[index] = value;
}


/*!
@brief set a value of a screw
*/
inline void bksl_coscrewf_set(
  bksl_coscrewf_t* __restrict__ c,
  const uint32_t index,
  const float value) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return;
  // }
  ((float*)c)[index] = value;
}


/*!
@brief set a value of a mat3x3

note that mat3x3 is stored in column major order
*/
inline void bksl_mat3x3_set(
  bksl_mat3x3_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  const double value) {

  // if(row > 2 || row < 0 || column < 0 || column > 2) {
  //   fprintf(stderr, "ksl_mat3x3_set: out of bounds\n");
  //   return;
  // }
  ((double*)R)[3 * column + row] = value;
}


/*!
@brief set a mat3x3 rotation matrix from three vectors

note that mat3x3 is stored in column major order
*/
inline void bksl_mat3x3_setFromVectors(
  bksl_mat3x3_t* __restrict__ R,
  bksl_vec3_t* __restrict__ x,
  bksl_vec3_t* __restrict__ y,
  bksl_vec3_t* __restrict__ z) {

  memcpy(&(R->m00), x, sizeof(bksl_vec3_t));
  memcpy(&(R->m01), y, sizeof(bksl_vec3_t));
  memcpy(&(R->m02), z, sizeof(bksl_vec3_t));

}



/*!
@brief set a value of a mat3x3

note that mat3x3 is stored in column major order
*/
inline void bksl_mat4x4_set(
  bksl_mat4x4_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  const double value) {

  // if(row > 3 || row < 0 || column < 0 || column > 3) {
  //   fprintf(stderr, "ksl_mat4x4_set: out of bounds\n");
  //   return;
  // }
  ((double*)R)[4 * column + row] = value;
}


/*!
@brief set a value of a displacement
*/
inline void bksl_displacement_set(
  bksl_displacement_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  const double value) {

  // if(row > 3 || row < 0 || column < 0 || column > 3) {
  //   fprintf(stderr, "ksl_displacement_set: out of bounds\n");
  //   return;
  // }
  if(column == 3) {
    bksl_vec3_set(&D->t, row, value);
  } else {
    bksl_mat3x3_set(&D->R, row, column, value);
  }
}


/*!
@brief set a value of vectorf
*/
inline void bksl_vec3f_set(
  bksl_vec3f_t* __restrict__ v,
  const uint32_t index,
  float value) {

  // if(index > 2 || index < 0) {
  //   fprintf(stderr, "ksl_vec3_set: out of bounds\n");
  //   return;
  // }
  ((float*)v)[index] = value;
}


/*!
@brief set a value of a screwf
*/
inline void bksl_screwf_set(
  bksl_screwf_t* __restrict__ s,
  const uint32_t index,
  float value) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return;
  // }
  ((float*)s)[index] = value;
}


/*!
@brief set a value of a mat3x3f

note that mat3x3 is stored in column major order
*/
inline void bksl_mat3x3f_set(
  bksl_mat3x3f_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  float value) {

  // if(row > 3 || row < 0 || column < 0 || column > 2) {
  //   fprintf(stderr, "ksl_mat3x3_set: out of bounds\n");
  //   return;
  // }
  ((float*)R)[row + column * 3] = value;
}


/*!
@brief set a value of a mat4x4f

note that mat3x3 is stored in column major order
*/
inline void bksl_mat4x4f_set(
  bksl_mat4x4f_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column,
  float value) {

  // if(row > 3 || row < 0 || column < 0 || column > 3) {
  //   fprintf(stderr, "ksl_mat4x4f_set: out of bounds\n");
  //   return;
  // }
  ((float*)R)[4 * column + row] = value;
}


/*!
@brief set a value of a displacementf
*/
inline void bksl_displacementf_set(
  bksl_displacementf_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column,
  float value) {

  if(column == 3) {
    bksl_vec3f_set(&D->t, row, value);
  } else {
    bksl_mat3x3f_set(&D->R, row, column, value);
  }
}


/*!
@brief get a value at specified index of a 3x1 column vector
*/
inline double bksl_vec3_get(
  bksl_vec3_t* __restrict__ v,
  const uint32_t index) {

  // if(index > 2 || index < 0) {
  //   fprintf(stderr, "ksl_vec3_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((double*)v)[index];
}


/*!
@brief set a value of a screw
*/
inline double bksl_screw_get(
  bksl_screw_t* __restrict__ s,
  const uint32_t index) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((double*)s)[index];
}


/*!
@brief set a value of a coscrew
*/
inline double bksl_coscrew_get(
  bksl_coscrew_t* __restrict__ c,
  const uint32_t index) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((double*)c)[index];
}


/*!
@brief set a value of a coscrew
*/
inline float bksl_coscrewf_get(
  bksl_coscrewf_t* __restrict__ c,
  const uint32_t index) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((float*)c)[index];
}



/*!
@brief set a value of a mat3x3

note that mat3x3 is stored in column major order
*/
inline double bksl_mat3x3_get(
  bksl_mat3x3_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column) {

  // assert(row < 3 && row > -1 && column < 3 && column > -1);
  // if(row > 3 || row < 0 || column < 0 || column > 2) {
  //   fprintf(stderr, "ksl_mat3x3_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((double*)R)[3 * column + row];
}


/*!
@brief get a value of a displacement
*/
inline double bksl_displacement_get(
  bksl_displacement_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column) {

  if(column == 3) {
    return bksl_vec3_get(&D->t, row);
  } else {
    return bksl_mat3x3_get(&D->R, row, column);
  }
}

/*!
@brief set a value of vectorf
*/
inline float bksl_vec3f_get(
  bksl_vec3f_t* __restrict__ v,
  const uint32_t index) {

  // if(index > 2 || index < 0) {
  //   fprintf(stderr, "ksl_vec3_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((float*)v)[index];
}

/*!
@brief set a value of a screwf
*/
inline float bksl_screwf_get(
  bksl_screwf_t* __restrict__ s,
  const uint32_t index) {

  // if(index > 5 || index < 0) {
  //   fprintf(stderr, "ksl_screw_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((float*)s)[index];
}

/*!
@brief set a value of a mat3x3f

note that mat3x3 is stored in column major order
*/
inline float bksl_mat3x3f_get(
  bksl_mat3x3f_t* __restrict__ R,
  const uint32_t row,
  const uint32_t column) {

  // if(row > 3 || row < 0 || column < 0 || column > 2) {
  //   fprintf(stderr, "ksl_mat3x3_set: out of bounds\n");
  //   return -INFINITY;
  // }
  return ((float*)R)[3 * column + row];
}

/*!
@brief set a value of a displacementf
*/
inline float bksl_displacementf_get(
  bksl_displacementf_t* __restrict__ D,
  const uint32_t row,
  const uint32_t column) {

  // if(row > 3 || row < 0 || column < 0 || column > 3) {
  //   fprintf(stderr, "ksl_displacement_set: out of bounds\n");
  //   return -INFINITY;
  // }
  if(column == 3) {
    return bksl_vec3f_get(&D->t, row);
  } else {
    return bksl_mat3x3f_get(&D->R, row, column);
  }
}

/*!
@brief get translation component of a general displacement

*/
inline void bksl_displacement_getTranslation(
  bksl_displacement_t* __restrict__ D,
  bksl_vec3_t* __restrict__ t) {

  bksl_copy_vv(&D->t, t);
}

/*!
@}
@name 3x1 vector algebra utilities
Utilities for carrying out Vector/Vector and Vector/Scalar operations
@{
*/


/*!
@brief Copy a double precision Vector vi uint32_to Vector vo.


\f$V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void bksl_copy_vv(
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  memcpy(vo, vi, sizeof(bksl_vec3_t));
  // cblas_dcopy(3, (double*)vi, 1, (double*)vo, 1);
}

/*!
@brief Copy a single precision Vectorf vi uint32_to Vectorf vo.

\f$V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void bksl_copy_vvf(
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo) {

  memcpy(vo, vi, sizeof(bksl_vec3f_t));

  // cblas_scopy(3, (float*)vi, 1, (float*)vo, 1);
}


/*!
@brief Copy a double precision Vector vi uint32_to vo.

\f$-V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void bksl_vec3_invert(
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  memset(vo, 0, 3 * sizeof(double));
  cblas_daxpy(3, -1.0, (double*)vi, 1, (double*)vo, 1);
}


/*!
@brief Copy a single prevision Vectorf vi uint32_to vo.

\f$-V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void bksl_vec3f_invert(
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo) {

  memset(vo, 0, 3 * sizeof(float));
  cblas_saxpy(3, -1.0, (float*)vi, 1, (float*)vo, 1);
}


/*!
@brief double precision Dot product of two vectors

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline double bksl_dot_vv(
  bksl_vec3_t* __restrict__ vi0,
  bksl_vec3_t* __restrict__ vi1) {

  return cblas_ddot(
           3,
           (double*)vi0,
           1,
           (double*)vi1,
           1);
}


/*!
@brief single precision Dot product of two vectors

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline double bksl_dot_vvf(
  bksl_vec3f_t* __restrict__ vi0,
  bksl_vec3f_t* __restrict__ vi1) {

  return cblas_sdot(
           3,
           (float*)vi0,
           1,
           (float*)vi1,
           1);
}


/*!
@brief double precision Dot product of two screws

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline double bksl_dot_cs(
  bksl_coscrew_t* __restrict__ ci0,
  bksl_screw_t* __restrict__ si1) {

  return cblas_ddot(
           6,
           (double*)ci0,
           1,
           (double*)si1,
           1);
}


/*!
@brief single precision Dot product of two screws

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline double bksl_dot_csf(
  bksl_coscrewf_t* __restrict__ ci0,
  bksl_screwf_t* __restrict__ si1) {

  return cblas_sdot(
           6,
           (float*)ci0,
           1,
           (float*)si1,
           1);
}


/*!
@brief Computes the product of a vector and a scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void bksl_product_vk(
  bksl_vec3_t* __restrict__ vi,
  const double ki,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dcopy(3, (double*)vi, 1, (double*)vo, 1);
  cblas_dscal(3, ki, (double*)vo, 1);
}


/*!
@brief Computes the product of a vector and a 1/scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void bksl_product_vkinverse(
  bksl_vec3_t* __restrict__ vi,
  const double ki,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dcopy(3, (double*)vi, 1, (double*)vo, 1);
  cblas_dscal(3, 1. / ki, (double*)vo, 1);
}


/*!
@brief Computes the product of a vector and a 1/scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void bksl_product_vinversek(
  bksl_vec3_t* __restrict__ vi,
  const double ki,
  bksl_vec3_t* __restrict__ vo) {

  memcpy(vo, vi, sizeof(bksl_vec3_t));
  cblas_dscal(3, -ki, (double*)vo, 1);
}


/*!
@brief Computes the product of a vector and a scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void bksl_product_vkf(
  bksl_vec3f_t* __restrict__ vi,
  const float ki,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_scopy(3, (float*)vi, 1, (float*)vo, 1);
  cblas_sscal(3, ki, (float*)vo, 1);
}


/*!
@brief Adds two double precision vectors

Add 3 by 1 column matrices v1i and v2i and store results in vo.

\f$V_{1i} + V_{2i} \rightarrow V_o\f$

@param v1i a first column vector input
@param v2i second column vector
@param vo the results of v1i + v2i are stored in vo
*/
inline void bksl_add_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dcopy(3, (double*)v1i, 1, (double*)vo, 1);
  cblas_daxpy(3, 1.0, (double*)v2i, 1, (double*)vo, 1);
  // vo[0] = v1i[0] + v2i[0];
  // vo[1] = v1i[1] + v2i[1];
  // vo[2] = v1i[2] + v2i[2];
}

/*!
@brief Adds two single precision vectors

Add 3 by 1 column matrices v1i and v2i and store results in vo.

\f$V_{1i} + V_{2i} \rightarrow V_o\f$

@param v1i a first column vector input
@param v2i second column vector
@param vo the results of v1i + v2i are stored in vo
*/
inline void bksl_add_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_scopy(3, (float*)v1i, 1, (float*)vo, 1);
  cblas_saxpy(3, 1.0, (float*)v2i, 1, (float*)vo, 1);
  // vo[0] = v1i[0] + v2i[0];
  // vo[1] = v1i[1] + v2i[1];
  // vo[2] = v1i[2] + v2i[2];
}


/*!
@brief Adds two double precision vectors

Add 3 by 1 column matrix v1i to v2i

\f$V_{2i} += V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void bksl_addequal_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i) {

  cblas_daxpy(3, 1.0, (double*)v1i, 1, (double*)v2i, 1);
}


/*!
@brief Adds two single precision vectors

Add 3 by 1 column matrix v1i to v2i

\f$V_{2i} += V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void bksl_addequal_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i) {

  cblas_saxpy(3, 1.0, (float*)v1i, 1, (float*)v2i, 1);
  // vo[0] = v1i[0] + v2i[0];
  // vo[1] = v1i[1] + v2i[1];
  // vo[2] = v1i[2] + v2i[2];
}


/*!
@brief Adds two double precision vectors

subtract 3 by 1 column matrix v1i from v2i

\f$V_{2i} -= V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void bksl_subtractequal_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i) {

  cblas_daxpy(3, -1.0, (double*)v1i, 1, (double*)v2i, 1);
}


/*!
@brief Adds two single precision vectors

Subtract 3 by 1 column matrix v1i from v2i

\f$V_{2i} -= V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void bksl_subtractequal_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i) {

  cblas_saxpy(3, -1.0, (float*)v1i, 1, (float*)v2i, 1);
  // vo[0] = v1i[0] + v2i[0];
  // vo[1] = v1i[1] + v2i[1];
  // vo[2] = v1i[2] + v2i[2];
}


/*!
@brief axpy (ax plus y stored in y) for double precision vectors

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void bksl_axpy_vv(
  const double k,
  bksl_vec3_t* __restrict__ v1x,
  bksl_vec3_t* __restrict__ v2y) {

  cblas_daxpy(3, k, (double*)v1x, 1, (double*)v2y, 1);
}


/*!
@brief axpy (ax plus y stored in y) for single precision vectors

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void bksl_axpy_vvf(
  const float k,
  bksl_vec3f_t* __restrict__ v1x,
  bksl_vec3f_t* __restrict__ v2y) {

  cblas_saxpy(3, k, (float*)v1x, 1, (float*)v2y, 1);
}


/*!
@brief axpy (ax plus y stored in y) for double precision screws

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void bksl_axpy_ss(
  const double k,
  bksl_screw_t* __restrict__ v1x,
  bksl_screw_t* __restrict__ v2y) {

  cblas_daxpy(6, k, (double*)v1x, 1, (double*)v2y, 1);
}


/*!
@brief axpy (ax plus y stored in y) for single precision screws

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void bksl_axpy_ssf(
  const float k,
  bksl_screwf_t* __restrict__ v1x,
  bksl_screwf_t* __restrict__ v2y) {

  cblas_saxpy(6, k, (float*)v1x, 1, (float*)v2y, 1);
}


/*!
@brief Subtract two double precision vectors

Subtract 3 by 1 column matrix v2i from v1i and store results in vo.

\f$V_{1i} - V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param V21 a second 3x1 vector
@param vo the results of v1i - v2i are returned in vo
*/
inline void bksl_subtract_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dcopy(3, (double*)v1i, 1, (double*)vo, 1);
  cblas_daxpy(3, -1, (double*)v2i, 1, (double*)vo, 1);
}


/*!
@brief Subtract two single precision vectors

Subtract 3 by 1 column matrix v2i from v1i and store results in vo.

\f$V_{1i} - V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param V21 a second 3x1 vector
@param vo the results of v1i - v2i are returned in vo
*/
inline void bksl_subtract_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_scopy(3, (float*)v1i, 1, (float*)vo, 1);
  cblas_saxpy(3, -1, (float*)v2i, 1, (float*)vo, 1);
}


/*!
@brief Compute double precision cross product of v1i with v2i

\f$V_{1i} \times V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param v2i a second 3x1 vector
@param vo the results are stored in vo
*/
inline void bksl_cross_vv(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo) {

  vo->x = v1i->y * v2i->z - v1i->z * v2i->y;
  vo->y = v1i->z * v2i->x - v1i->x * v2i->z;
  vo->z = v1i->x * v2i->y - v1i->y * v2i->x;
}


/*!
@brief Compute double precision cross product of v1i with v2i^-1

\f$V_{1i} \times V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param v2i a second 3x1 vector
@param vo the results are stored in vo
*/
inline void bksl_cross_vvinverse(
  bksl_vec3_t* __restrict__ v1i,
  bksl_vec3_t* __restrict__ v2i,
  bksl_vec3_t* __restrict__ vo) {

  vo->x = v1i->z * v2i->y - v1i->y * v2i->z;
  vo->y = v1i->x * v2i->z - v1i->z * v2i->x;
  vo->z = v1i->y * v2i->x - v1i->x * v2i->y;
}


/*!
@brief Compute single precision cross product of v1i with v2i

\f$V_{1i} \times V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param v2i a second 3x1 vector
@param vo the results are stored in vo
*/
inline void bksl_cross_vvf(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo) {

  vo->x = v1i->y * v2i->z - v1i->z * v2i->y;
  vo->y = v1i->z * v2i->x - v1i->x * v2i->z;
  vo->z = v1i->x * v2i->y - v1i->y * v2i->x;
}


/*!
@brief Compute single precision cross product of v1i with v2i inverse

\f$V_{1i} \times V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param v2i a second 3x1 vector
@param vo the results are stored in vo
*/
inline void bksl_cross_vvinversef(
  bksl_vec3f_t* __restrict__ v1i,
  bksl_vec3f_t* __restrict__ v2i,
  bksl_vec3f_t* __restrict__ vo) {

  vo->x = v1i->z * v2i->y - v1i->y * v2i->z;
  vo->y = v1i->x * v2i->z - v1i->z * v2i->x;
  vo->z = v1i->y * v2i->x - v1i->x * v2i->y;
}


/*!
@}
@name 6x1 screw algebra utilities
Utilities for carrying out 6x1 Vector/Vector and Vector/Scalar operations
for 6x1 screws
@{
*/

/*!
@brief Copy double precision bksl_screw_t si to so.

\f$S_i \rightarrow S_o\f$

@param si 6x1 screw input
@param so si is copied to so
*/
inline void bksl_copy_ss(
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  memcpy(so, si, sizeof(bksl_screw_t));

  /* previously was */
  // cblas_dcopy(6, (double*)si, 1, (double*)so, 1);
}

/*!
@brief Copy single precision bksl_screw_t si to so.

\f$S_i \rightarrow S_o\f$

@param si 6x1 screw input
@param so si is copied to so
*/
inline void bksl_copy_ssf(
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  memcpy(so, si, sizeof(bksl_screwf_t));

  // cblas_scopy(6, (float*)si, 1, (float*)so, 1);
}

/*!
@brief Copy double precision bksl_screw_t si to so.

\f$S_i \rightarrow S_o\f$

@param si 6x1 screw input
@param so si is copied to so
*/
inline void bksl_copy_cc(
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  memcpy(co, ci, sizeof(bksl_coscrew_t));

  /* previously was */
  // cblas_dcopy(6, (double*)si, 1, (double*)so, 1);
}

/*!
@brief Copy single precision bksl_screw_t si to so.

\f$S_i \rightarrow S_o\f$

@param si 6x1 screw input
@param so si is copied to so
*/
inline void bksl_copy_ccf(
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  memcpy(co, ci, sizeof(bksl_coscrewf_t));

  // cblas_scopy(6, (float*)si, 1, (float*)so, 1);
}


/*!
@brief Invert a double precision bksl_screw_t si, store the results in so.

\f$-S_i \rightarrow S_o\f$

@param si input vector \f$S_i\f$
@param so output vector \f$S_o\f$
*/
void bksl_invert_ss(
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  memset(so, 0, sizeof(bksl_screw_t));
  cblas_daxpy(6, -1.0, (double*)si, 1, (double*)so, 1);
}

/*!
@brief Invert a single precision bksl_screwf_t si, store the results in so.

\f$-S_i \rightarrow S_o\f$

@param si input vector \f$S_i\f$
@param so output vector \f$S_o\f$
*/
void bksl_invert_ssf(
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  memset(so, 0, sizeof(bksl_screwf_t));
  cblas_saxpy(6, -1.0, (float*)si, 1, (float*)so, 1);
}


/*!
@brief Invert a double precision bksl_screw_t si, store the results in so.

\f$-S_i \rightarrow S_o\f$

@param si input vector \f$S_i\f$
@param so output vector \f$S_o\f$
*/
void bksl_invert_cc(
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  for(uint32_t i = 0; i < 6; i++) {
    bksl_coscrew_set(co, i, -bksl_coscrew_get(ci, i));
  }

  /* previously was */
  // memset(so, 0, sizeof(bksl_screw_t));
  // cblas_daxpy(6, -1.0, (double*)si, 1, (double*)so, 1);
}


/*!
@brief Invert a single precision bksl_screwf_t si, store the results in so.

\f$-S_i \rightarrow S_o\f$

@param si input vector \f$S_i\f$
@param so output vector \f$S_o\f$
*/
void bksl_invert_ccf(
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  for(uint32_t i = 0; i < 6; i++) {
    bksl_coscrewf_set(co, i, -bksl_coscrewf_get(ci, i));
  }

  /* previously was */
  // memset(so, 0, sizeof(bksl_screw_t));
  // cblas_saxpy(6, -1.0, (float*)si, 1, (float*)so, 1);
}


/*!
@brief Computes the product of a a double precision bksl_screw_t and a scalar

\f$S_i * k_i \rightarrow S_o\f$

@param si[6] A 3x1 input vector
@param *ki a pointer to a scalar
@param so[6] the results are stored in so[6]
*/
void bksl_product_sk(
  bksl_screw_t* __restrict__ si,
  const double ki,
  bksl_screw_t* __restrict__ so) {

  cblas_dcopy(6, (double*)si, 1, (double*)so, 1);
  cblas_dscal(6, ki, (double*)so, 1);
}

/*!
@brief Computes the product of a single precision bksl_screwf_t and a scalar

\f$S_i * k_i \rightarrow S_o\f$

@param si[6] A 3x1 input vector
@param *ki a pointer to a scalar
@param so[6] the results are stored in so[6]
*/
void bksl_product_skf(
  bksl_screwf_t* __restrict__ si,
  const float ki,
  bksl_screwf_t* __restrict__ so) {

  cblas_scopy(6, (float*)si, 1, (float*)so, 1);
  cblas_sscal(6, ki, (float*)so, 1);
}

/*!
@brief Add double precision bksl_screw_ts s1i and s2i and store results in so.

\f$S_{1i} + S_{2i} \rightarrow S_o\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
void bksl_add_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so) {

  cblas_dcopy(6, (double*)s1i, 1, (double*)so, 1);
  cblas_daxpy(6, 1, (double*)s2i, 1, (double*)so, 1);
}


/*!
@brief Add double precision bksl_screw_ts s1i to s2i.

\f$S_{2i} += S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
void bksl_addequal_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i) {

  cblas_daxpy(6, 1, (double*)s1i, 1, (double*)s2i, 1);
}


/*!
@brief Add double precision bksl_screw_ts s1i to s2i.

\f$S_{2i} += S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
void bksl_addequal_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i) {

  cblas_saxpy(6, 1, (float*)s1i, 1, (float*)s2i, 1);
}



/*!
@brief Add single precision bksl_screw_ts s1i and s2i and store results in so.

\f$S_{1i} + S_{2i} \rightarrow S_o\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
void bksl_add_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so) {

  cblas_scopy(6, (float*)s1i, 1, (float*)so, 1);
  cblas_saxpy(6, 1, (float*)s2i, 1, (float*)so, 1);
}

/*!
@brief Subtract double precision bksl_screw_t s2i from s1i and store results in so.

\f$S_{1i} - S_{2i} \rightarrow S_o\f$

@param s1i a first bksl_screw_t
@param s2i a second bksl_screw_t
@param so The results of s1i - s2i are stored in so
*/
void bksl_subtract_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so) {

  cblas_dcopy(6, (double*)s1i, 1, (double*)so, 1);
  cblas_daxpy(6, -1.0, (double*)s2i, 1, (double*)so, 1);
}

/*!
@brief Subtract single precision bksl_screw_t s2i from s1i and store results in so.

\f$S_{1i} - S_{2i} \rightarrow S_o\f$

@param s1i a first bksl_screw_t
@param s2i a second bksl_screw_t
@param so The results of s1i - s2i are stored in so
*/
void bksl_subtract_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so) {

  cblas_scopy(6, (float*)s1i, 1, (float*)so, 1);
  cblas_saxpy(6, -1.0, (float*)s2i, 1, (float*)so, 1);
}

/*!
@brief Add double precision bksl_screw_ts s1i to s2i.

\f$S_{2i} -= S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
void bksl_subtractequal_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i) {

  cblas_daxpy(6, -1.0, (double*)s1i, 1, (double*)s2i, 1);
}


/*!
@brief Add double precision bksl_screw_ts s1i to s2i.

\f$S_{2i} += S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
void bksl_subtractequal_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i) {

  cblas_saxpy(6, -1.0, (float*)s1i, 1, (float*)s2i, 1);
}


/*!
@}
@name Mixed 3x1 vector and 6x1 screw utilities

Utilities for performing operations on a combination of 3x1 vectors and
6x1 screws.
@{
*/

/*!
@brief Adds a double prevision bksl_screw_t and Vector

\f$S_i[0:3] + V_i \rightarrow S_o[0:3]\f$

\f$S_i[3:6] \rightarrow S_o[3:6]\f$

@param si a 6x1 screw
@param vi a 3x1 vector
@param so the results of the operation are returned in so
*/
void bksl_add_sst(
  bksl_screw_t* __restrict__ si1,
  bksl_screw_t* __restrict__ si2,
  bksl_screw_t* __restrict__ so) {

  bksl_copy_ss(si1, so);
  cblas_daxpy(3, 1.0, (double*)si2, 1, (double*)so, 1);
}


/*!
@brief Adds a single prevision bksl_screwf_t and Vectorf

\f$S_i[0:3] + V_i \rightarrow S_o[0:3]\f$

\f$S_i[3:6] \rightarrow S_o[3:6]\f$

@param si a 6x1 screw
@param vi a 3x1 vector
@param so the results of the operation are returned in so
*/
void bksl_add_sstf(
  bksl_screwf_t* __restrict__ si1,
  bksl_screwf_t* __restrict__ si2,
  bksl_screwf_t* __restrict__ so) {

  bksl_copy_ssf(si1, so);
  cblas_saxpy(3, 1.0, (float*)si2, 1, (float*)so, 1);
}


/*!
@}
@name rotation matrix utilities
Utilities for performing matrix algebra with 3x3 rotation matrices. Note that
all general rotation matrices are stored as 9x1 arrays in row major order.
Primitive rotations (Rx, Ry, Rz) are characterized by a 2-dimensional array of
direction cosines.
@{
*/

/*!
@brief Copy a double precision bksl_mat3x3_t direction cosine matrix ri uint32_to ro.

\f$R_i \rightarrow R_o\f$

@param ri rotation matrix input
@param ro rotation matrix output
*/
void bksl_mat3x3_copy(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_mat3x3_t* __restrict__ ro) {

  cblas_dcopy(9, (double*)ri, 1, (double*)ro, 1);
}


/*!
@brief Copy a single precision bksl_mat3x3f_t direction cosine matrix ri uint32_to ro.

\f$R_i \rightarrow R_o\f$

@param ri rotation matrix input
@param ro rotation matrix output
*/
void bksl_mat3x3f_copy(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_mat3x3f_t* __restrict__ ro) {

  cblas_scopy(9, (float*)ri, 1, (float*)ro, 1);
}


/*!
@brief Inverts (transposes) a double precision bksl_mat3x3_t rotation matrix

\f$R_i^T \rightarrow R_o\f$

@param ri rotation matrix to be inverted
@param ro transposed rotation matrix returned in ro
*/
void bksl_mat3x3_invert(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_mat3x3_t* __restrict__ ro) {

  cblas_dcopy(3, (double*)ri, 1, (double*)ro, 3);
  cblas_dcopy(3, ((double*)ri) + 3, 1, ((double*)ro) + 1, 3);
  cblas_dcopy(3, ((double*)ri) + 6, 1, ((double*)ro) + 2, 3);
  // ro[0 * 3 + 0] = ri[0][0]; ro[0][1] = ri[1][0]; ro[0][2] = ri[2][0];
  // ro[1][0] = ri[0][1]; ro[1][1] = ri[1][1]; ro[1][2] = ri[2][1];
  // ro[2][0] = ri[0][2]; ro[2][1] = ri[1][2]; ro[2][2] = ri[2][2];
}


/*!
@brief Inverts (transposes) a single precision bksl_mat3x3f_t rotation matrix

\f$R_i^T \rightarrow R_o\f$

@param ri rotation matrix to be inverted
@param ro transposed rotation matrix returned in ro
*/
void bksl_invert_Rf(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_mat3x3f_t* __restrict__ ro) {

  cblas_scopy(3, (float*)ri, 1, (float*)ro, 3);
  cblas_scopy(3, ((float*)ri) + 3, 1, (float*)ro + 1, 3);
  cblas_scopy(3, ((float*)ri) + 6, 1, (float*)ro + 2, 3);
  // ro[0 * 3 + 0] = ri[0][0]; ro[0][1] = ri[1][0]; ro[0][2] = ri[2][0];
  // ro[1][0] = ri[0][1]; ro[1][1] = ri[1][1]; ro[1][2] = ri[2][1];
  // ro[2][0] = ri[0][2]; ro[2][1] = ri[1][2]; ro[2][2] = ri[2][2];
}


/*!
@brief Store product of a double precision direction cosine matrix ri with
vector vi in vo.

\f$R_i * V_i \rightarrow V_o\f$
*/
void bksl_product_Rv(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  /* suggest to leave blas for matrix-vector operations at least for now,
  I found a 10-20% speed up when using blas vs using unwrapped loops in earlier
  testing, although was not using restrict at the time. new version
  of compiler may be able to do better */

  cblas_dgemv(
    CblasColMajor,
    CblasNoTrans,
    3,
    3,
    1.0,
    (double*)ri,
    3,
    (double*)vi,
    1,
    0.0,
    (double*)vo,
    1);
  // vo[0] =  ri[0][0] * vi[0] + ri[0][1] * vi[1] + ri[0][2] * vi[2];
  // vo[1] =  ri[1][0] * vi[0] + ri[1][1] * vi[1] + ri[1][2] * vi[2];
  // vo[2] =  ri[2][0] * vi[0] + ri[2][1] * vi[1] + ri[2][2] * vi[2];
}

/*!
@brief Store product of direction cosine matrix ri with vector vi in vo.

\f$R_i * V_i \rightarrow V_o\f$
*/
void bksl_product_Rvf(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_sgemv(
    CblasColMajor,
    CblasNoTrans,
    3,
    3,
    1.0,
    (float*)ri,
    3,
    (float*)vi,
    1,
    0.0,
    (float*)vo,
    1);
  // vo[0] =  ri[0][0] * vi[0] + ri[0][1] * vi[1] + ri[0][2] * vi[2];
  // vo[1] =  ri[1][0] * vi[0] + ri[1][1] * vi[1] + ri[1][2] * vi[2];
  // vo[2] =  ri[2][0] * vi[0] + ri[2][1] * vi[1] + ri[2][2] * vi[2];
}

/*!
@brief Store product of a double precision direction cosine matrix ri with
vector -vi in vo.

\f$R_i * -V_i \rightarrow V_o\f$
*/
void bksl_product_Rvinverse(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dgemv(
    CblasColMajor,
    CblasNoTrans,
    3,
    3,
    -1.0,
    (double*)ri,
    3,
    (double*)vi,
    1,
    0.0,
    (double*)vo,
    1);
  // vo[0] = -ri[0][0] * vi[0] - ri[0][1] * vi[1] - ri[0][2] * vi[2];
  // vo[1] = -ri[1][0] * vi[0] - ri[1][1] * vi[1] - ri[1][2] * vi[2];
  // vo[2] = -ri[2][0] * vi[0] - ri[2][1] * vi[1] - ri[2][2] * vi[2];
}

/*!
@brief Store product of a single precision direction cosine matrix ri with
vector -vi in vo.

\f$R_i * -V_i \rightarrow V_o\f$
*/
void bksl_product_Rvinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_sgemv(
    CblasColMajor,
    CblasNoTrans,
    3,
    3,
    -1.0,
    (float*)ri,
    3,
    (float*)vi,
    1,
    0.0,
    (float*)vo,
    1);
  // vo[0] = -ri[0][0] * vi[0] - ri[0][1] * vi[1] - ri[0][2] * vi[2];
  // vo[1] = -ri[1][0] * vi[0] - ri[1][1] * vi[1] - ri[1][2] * vi[2];
  // vo[2] = -ri[2][0] * vi[0] - ri[2][1] * vi[1] - ri[2][2] * vi[2];
}

/*!
@brief Store product of a double precision direction cosine matrix ri^-1 with vector vi in vo.

\f$R_i^T * V_i \rightarrow V_o\f$
*/
void bksl_product_Rinversev(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dgemv(
    CblasColMajor,
    CblasTrans,
    3,
    3,
    1.0,
    (double*)ri,
    3,
    (double*)vi,
    1,
    0.0,
    (double*)vo,
    1);
  // vo[0] =  ri[0][0] * vi[0] + ri[1][0] * vi[1] + ri[2][0] * vi[2];
  // vo[1] =  ri[0][1] * vi[0] + ri[1][1] * vi[1] + ri[2][1] * vi[2];
  // vo[2] =  ri[0][2] * vi[0] + ri[1][2] * vi[1] + ri[2][2] * vi[2];
}


/*!
@brief Store product of a single precision direction cosine matrix ri^-1 with vector vi in vo.

\f$R_i^T * V_i \rightarrow V_o\f$
*/
void bksl_product_Rinversevf(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_sgemv(
    CblasColMajor,
    CblasTrans,
    3,
    3,
    1.0,
    (float*)ri,
    3,
    (float*)vi,
    1,
    0.0,
    (float*)vo,
    1);
  // vo[0] =  ri[0][0] * vi[0] + ri[1][0] * vi[1] + ri[2][0] * vi[2];
  // vo[1] =  ri[0][1] * vi[0] + ri[1][1] * vi[1] + ri[2][1] * vi[2];
  // vo[2] =  ri[0][2] * vi[0] + ri[1][2] * vi[1] + ri[2][2] * vi[2];
}


/*!
@brief Store product of a double precision direction cosine matrix ri^-1 with
vector -vi in vo.

\f$R_i^T * -V_i \rightarrow V_o\f$
*/
void bksl_product_rinversevinverse(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  cblas_dgemv(
    CblasColMajor,
    CblasTrans,
    3,
    3,
    -1.0,
    (double*)ri,
    3,
    (double*)vi,
    1,
    0.0,
    (double*)vo,
    1);
}


/*!
@brief Store product of a double precision direction cosine matrix ri^-1 with
vector -vi in vo.

\f$R_i^T * -V_i \rightarrow V_o\f$
*/
void bksl_product_Rinversevinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_vec3f_t* __restrict__ vi,
  bksl_vec3f_t* __restrict__ vo) {

  cblas_sgemv(
    CblasColMajor,
    CblasTrans,
    3,
    3,
    -1.0,
    (float*)ri,
    3,
    (float*)vi,
    1,
    0.0,
    (float*)vo,
    1);
}


/*!
@brief rotates a double precision spatial screw

Store product of direction cosine matrix ri with spatial vector si in so.

\f$ R_i * S_i[0:2] \rightarrow S_o[0:2]\f$

\f$ R_i * S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri a 1x9 rotation matrix
@param si a 1x6 screw
@param so the output of ri * si is returned in so
*/
void bksl_product_Adrs(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_product_Rv(Ri, &si->v, &so->v);
  bksl_product_Rv(Ri, &si->w, &so->w);
}

/*!
@brief rotates a double precision spatial coscrew

Store product of direction cosine matrix ri with spatial vector si in so.

\f$ R_i * S_i[0:2] \rightarrow S_o[0:2]\f$

\f$ R_i * S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri a 1x9 rotation matrix
@param si a 1x6 screw
@param so the output of ri * si is returned in so
*/
void bksl_product_CoAdrc(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  bksl_product_Rv(Ri, &ci->f, &co->f);
  bksl_product_Rv(Ri, &ci->m, &co->m);
}


/*!
@brief rotates a single precision spatial coscrew

Store product of direction cosine matrix ri with spatial vector si in so.

\f$ R_i * S_i[0:2] \rightarrow S_o[0:2]\f$

\f$ R_i * S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri a 1x9 rotation matrix
@param si a 1x6 screw
@param so the output of ri * si is returned in so
*/
void bksl_product_CoAdrcf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  bksl_product_Rvf(Ri, &ci->f, &co->f);
  bksl_product_Rvf(Ri, &ci->m, &co->m);
}


/*!
@brief rotates a single precision spatial screw

Store product of direction cosine matrix ri with spatial vector si in so.

\f$ R_i * S_i[0:2] \rightarrow S_o[0:2]\f$

\f$ R_i * S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri a 1x9 rotation matrix
@param si a 1x6 screw
@param so the output of ri * si is returned in so
*/
inline void bksl_product_Adrsf(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_product_Rvf(ri, (bksl_vec3f_t*)si, (bksl_vec3f_t*)so);
  bksl_product_Rvf(
    ri, (bksl_vec3f_t*)(((float*)si) + 3), (bksl_vec3f_t*)(((float*)so) + 3));
}

/*!
@brief rotates a double precision spatial screw inverse

Store product of direction cosine matrix ri with spatial vector si in so.

\f$ R_i * S_i[0:2] \rightarrow S_o[0:2]\f$

\f$ R_i * S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri a 1x9 rotation matrix
@param si a 1x6 screw
@param so the output of ri * si is returned in so
*/
inline void bksl_product_Adrsinverse(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_product_Rvinverse(ri, (bksl_vec3_t*)si, (bksl_vec3_t*)so);
  bksl_product_Rvinverse(
    ri, (bksl_vec3_t*)(((double*)si) + 3), (bksl_vec3_t*)(((double*)so) + 3));
}

/*!
@brief rotates a double precision spatial screw inverse

Store product of direction cosine matrix ri with spatial vector si in so.

\f$ R_i * S_i[0:2] \rightarrow S_o[0:2]\f$

\f$ R_i * S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri a 1x9 rotation matrix
@param si a 1x6 screw
@param so the output of ri * si is returned in so
*/
void bksl_product_Adrsinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_product_Rvinversef(ri, (bksl_vec3f_t*)si, (bksl_vec3f_t*)so);
  bksl_product_Rvinversef(
    ri,
    (bksl_vec3f_t*)(((float*)si) + 3), (bksl_vec3f_t*)(((float*)so) + 3));
}

/*!
@brief rotates a single precision spatial screw's inverse

Store product of direction cosine matrix ri with spatial vector si in so.

\f$R_i * -S_i[0:2] \rightarrow S_o[0:2]\f$
\f$R_i * -S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri rotation matrix
@param si screw
@param so the output of ri * -si is returned in so
*/
void bksl_product_Adrinverses(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_product_Rinversev(ri, (bksl_vec3_t*)si, (bksl_vec3_t*)so);
  bksl_product_Rinversev(
    ri, (bksl_vec3_t*)(((double*)si) + 3), (bksl_vec3_t*)(((double*)so) + 3));
}

/*!
@brief rotates a single precision spatial screw's inverse

Store product of direction cosine matrix ri with spatial vector si in so.

\f$R_i * -S_i[0:2] \rightarrow S_o[0:2]\f$
\f$R_i * -S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri rotation matrix
@param si screw
@param so the output of ri * -si is returned in so
*/
void bksl_product_Adrinversesf(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_product_Rinversevf(ri, (bksl_vec3f_t*)si, (bksl_vec3f_t*)so);
  bksl_product_Rinversevf(
    ri, (bksl_vec3f_t*)(((float*)si) + 3), (bksl_vec3f_t*)(((float*)so) + 3));
}


/*!
@brief inverse rotates a double precision coscrew

Store product of direction cosine matrix ri with spatial vector si in so.

\f$R_i * -S_i[0:2] \rightarrow S_o[0:2]\f$
\f$R_i * -S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri rotation matrix
@param si screw
@param so the output of ri * -si is returned in so
*/
void bksl_product_CoAdrinversec(
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  bksl_product_Rinversev(Ri, &ci->f, &co->f);
  bksl_product_Rinversev(Ri, &ci->m, &co->m);
}


/*!
@brief inverse rotates a single precision spatial co screw

Store product of direction cosine matrix ri with spatial vector si in so.

\f$R_i * -S_i[0:2] \rightarrow S_o[0:2]\f$
\f$R_i * -S_i[3:5] \rightarrow S_o[3:5]\f$

@param ri rotation matrix
@param si screw
@param so the output of ri * -si is returned in so
*/
void bksl_product_CoAdrinversecf(
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  bksl_product_Rinversevf(Ri, &ci->f, &co->f);
  bksl_product_Rinversevf(Ri, &ci->m, &co->m);
}


/*!
@brief Double precision product of R and Rx

Computes the product of a general rotation matrix, R, with a primitive rotation
about the x axis, Rx, characterized by its direction cosines.

\f$R_i * R_x \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRx(
  bksl_mat3x3_t* __restrict__ ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ ro) {

  memset(ro, 0, 9 * sizeof(double));

  cblas_dcopy(3, (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(
    3, dc[1],
    ((double*)ri) + 3, 1,
    ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 6, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 3, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0  0.0    0.0|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0  dc[1] -dc[0]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0  dc[0] dc[1]|

  // ro[0 * 3 + 1] =  ri[0 * 3 + 1] * dc[1] + ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3 + 1] =  ri[1 * 3 + 1] * dc[1] + ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3 + 1] =  ri[2 * 3 + 1] * dc[1] + ri[2 * 3 + 2] * dc[0];

  // ro[0 * 3 + 2] = -ri[0 * 3 + 1] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] = -ri[1 * 3 + 1] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] = -ri[2 * 3 + 1] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // cblas_dcopy(3, ri, 3, ro, 3);
}

/*!
@brief Product of R and Rx

Computes the product of a general rotation matrix, R, with a primitive rotation
about the x axis, Rx, characterized by its direction cosines.

\f$R_i * R_x \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRxf(
  bksl_mat3x3f_t* __restrict__ ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3f_t));
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 6, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 3, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);
  cblas_scopy(3, (float*)ri, 1, (float*)ro, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0  0.0    0.0|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0  dc[1] -dc[0]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0  dc[0] dc[1]|

  // ro[0 * 3 + 1] =  ri[0 * 3 + 1] * dc[1] + ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3 + 1] =  ri[1 * 3 + 1] * dc[1] + ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3 + 1] =  ri[2 * 3 + 1] * dc[1] + ri[2 * 3 + 2] * dc[0];

  // ro[0 * 3 + 2] = -ri[0 * 3 + 1] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] = -ri[1 * 3 + 1] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] = -ri[2 * 3 + 1] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // cblas_dcopy(3, ri, 3, ro, 3);
}

/*!
@brief Double precision product of R and Rxinverse

Computes the product of a general rotation matrix, R, with the inverse of a
primitive rotation about the x axis, Rx^T, characterized by its direction
cosines.

\f$R_i * R_x^T \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRxinverse(
  bksl_mat3x3_t* __restrict__ ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3_t));
  cblas_dcopy(3, (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 6, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 3, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0   0.0    0.0
  // |
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0  dc[1] dc[0]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0 -dc[0] dc[1]|

  // ro[0 * 3 + 2] =  ri[0 * 3 + 1] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] =  ri[1 * 3 + 1] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] =  ri[2 * 3 + 1] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // ro[0 * 3 + 1] =  ri[0 * 3 + 1] * dc[1] - ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3 + 1] =  ri[1 * 3 + 1] * dc[1] - ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3 + 1] =  ri[2 * 3 + 1] * dc[1] - ri[2 * 3 + 2] * dc[0];

  // cblas_dcopy(3, ri, 3, ro, 3);
}

/*!
@brief single precision product of R and Rxinverse

Computes the product of a general rotation matrix, R, with the inverse of a
primitive rotation about the x axis, Rx^T, characterized by its direction
cosines.

\f$R_i * R_x^T \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRxinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3f_t));
  cblas_scopy(3, (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 6, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 3, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0   0.0    0.0
  // |
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0  dc[1] dc[0]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0 -dc[0] dc[1]|

  // ro[0 * 3 + 2] =  ri[0 * 3 + 1] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] =  ri[1 * 3 + 1] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] =  ri[2 * 3 + 1] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // ro[0 * 3 + 1] =  ri[0 * 3 + 1] * dc[1] - ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3 + 1] =  ri[1 * 3 + 1] * dc[1] - ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3 + 1] =  ri[2 * 3 + 1] * dc[1] - ri[2 * 3 + 2] * dc[0];

  // cblas_dcopy(3, ri, 3, ro, 3);
}

/*!
@brief Product of R and Ry

Computes the product of a general rotation matrix, R, with a primitive rotation
about the x axis, Rx, characterized by its direction cosines.

\f$R_i * R_y \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRy(
  bksl_mat3x3_t* __restrict__ ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 6, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, dc[0], (double*)ri, 1, ((double*)ro) + 6, 1);
  cblas_dcopy(3, ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   | dc[1] 0.0 dc[0]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |  0.0  1.0   0.0
  // |
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |-dc[0] 0.0 dc[1]|

  // ro[0 * 3 + 2] =  ri[0 * 3] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] =  ri[1 * 3] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] =  ri[2 * 3] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // ro[0 * 3] =  ri[0 * 3] * dc[1] - ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] - ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] - ri[2 * 3 + 2] * dc[0];

  // cblas_dcopy(3, ri + 1, 3, ro + 1, 3);
}

/*!
@brief Product of R and Ry

Computes the product of a general rotation matrix, R, with a primitive rotation
about the x axis, Rx, characterized by its direction cosines.

\f$R_i * R_y \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRyf(
  bksl_mat3x3f_t* __restrict__ ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 6, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, dc[0], (float*)ri, 1, ((float*)ro) + 6, 1);
  cblas_scopy(3, ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   | dc[1] 0.0 dc[0]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |  0.0  1.0   0.0
  // |
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |-dc[0] 0.0 dc[1]|

  // ro[0 * 3 + 2] =  ri[0 * 3] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] =  ri[1 * 3] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] =  ri[2 * 3] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // ro[0 * 3] =  ri[0 * 3] * dc[1] - ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] - ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] - ri[2 * 3 + 2] * dc[0];

  // cblas_dcopy(3, ri + 1, 3, ro + 1, 3);
}

/*!
@brief Double precision product of R and Ryinverse

Computes the product of a general rotation matrix, R, with the inverse of a
primitive rotation about the y axis, Ry^T, characterized by its direction
cosines.

\f$R_i * R_y^T \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRyinverse(
  bksl_mat3x3_t* __restrict__ ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 6, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, -dc[0], (double*)ri, 1, ((double*)ro) + 6, 1);
  cblas_dcopy(3, ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] 0.0 -dc[0]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * | 0.0  1.0   0.0 |
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |dc[0] 0.0  dc[1]|

  // ro[0 * 3] =  ri[0 * 3] * dc[1] + ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] + ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] + ri[2 * 3 + 2] * dc[0];

  // ro[0 * 3 + 2] = -ri[0 * 3] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] = -ri[1 * 3] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] = -ri[2 * 3] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // cblas_dcopy(3, ri + 1, 3, ro + 1, 3);
}


/*!
@brief Double precision product of R and Ryinverse

Computes the product of a general rotation matrix, R, with the inverse of a
primitive rotation about the y axis, Ry^T, characterized by its direction
cosines.

\f$R_i * R_y^T \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRyinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 6, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, -dc[0], (float*)ri, 1, ((float*)ro) + 6, 1);
  cblas_scopy(3, ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] 0.0 -dc[0]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * | 0.0  1.0   0.0 |
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |dc[0] 0.0  dc[1]|

  // ro[0 * 3] =  ri[0 * 3] * dc[1] + ri[0 * 3 + 2] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] + ri[1 * 3 + 2] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] + ri[2 * 3 + 2] * dc[0];

  // ro[0 * 3 + 2] = -ri[0 * 3] * dc[0] + ri[0 * 3 + 2] * dc[1];
  // ro[1 * 3 + 2] = -ri[1 * 3] * dc[0] + ri[1 * 3 + 2] * dc[1];
  // ro[2 * 3 + 2] = -ri[2 * 3] * dc[0] + ri[2 * 3 + 2] * dc[1];

  // cblas_dcopy(3, ri + 1, 3, ro + 1, 3);
}


/*!
@brief Double precision product of R and Rz

Computes the product of a general rotation matrix, R, with a primitive rotation
about the z axis, Rz, characterized by its direction cosines.

\f$R_i * R_z \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRz(
  bksl_mat3x3_t* __restrict__ ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 3, 1, (double*)ro, 1);
  cblas_daxpy(3, -dc[0], (double*)ri, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_dcopy(3, ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] -dc[0] 0.0|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |dc[0]  dc[1] 0.0|
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   | 0.0    0.0  1.0|

  // ro[0 * 3] =  ri[0 * 3] * dc[1] + ri[0 * 3 + 1] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] + ri[1 * 3 + 1] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] + ri[2 * 3 + 1] * dc[0];

  // ro[0 * 3 + 1] = -ri[0 * 3] * dc[0] + ri[0 * 3 + 1] * dc[1];
  // ro[1 * 3 + 1] = -ri[1 * 3] * dc[0] + ri[1 * 3 + 1] * dc[1];
  // ro[2 * 3 + 1] = -ri[2 * 3] * dc[0] + ri[2 * 3 + 1] * dc[1];

  // cblas_dcopy(3, ri + 2, 3, ro + 2, 3);
}


/*!
@brief single precision product of R and Rz

Computes the product of a general rotation matrix, R, with a primitive rotation
about the z axis, Rz, characterized by its direction cosines.

\f$R_i * R_z \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
void bksl_product_RRzf(
  bksl_mat3x3f_t* __restrict__ ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 3, 1, (float*)ro, 1);
  cblas_saxpy(3, -dc[0], (float*)ri, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_scopy(3, ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] -dc[0] 0.0|
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |dc[0]  dc[1] 0.0|
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   | 0.0    0.0  1.0|

  // ro[0 * 3] =  ri[0 * 3] * dc[1] + ri[0 * 3 + 1] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] + ri[1 * 3 + 1] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] + ri[2 * 3 + 1] * dc[0];

  // ro[0 * 3 + 1] = -ri[0 * 3] * dc[0] + ri[0 * 3 + 1] * dc[1];
  // ro[1 * 3 + 1] = -ri[1 * 3] * dc[0] + ri[1 * 3 + 1] * dc[1];
  // ro[2 * 3 + 1] = -ri[2 * 3] * dc[0] + ri[2 * 3 + 1] * dc[1];

  // cblas_dcopy(3, ri + 2, 3, ro + 2, 3);
}


/*!
@brief Product of R and Rzinverse

Computes the product of a general rotation matrix, R, with the inverse of a
primitive rotation about the z axis, Rz^T, characterized by its direction
cosines.

\f$R_i * R_z^T \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRzinverse(
  bksl_mat3x3_t* __restrict__ ri,
  double dc[2],
  bksl_mat3x3_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 3, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[0], (double*)ri, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_dcopy(3, ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0 * 3 + 0] ro[0 * 3 + 1] ro[0 * 3 + 2]|   |ri[0 * 3 + 0] ri[0 * 3 + 1] ri[0
  // * 3 + 2]|   | dc[1] dc[0] 0.0|
  // |ro[1 * 3 + 0] ro[1 * 3 + 1] ro[1 * 3 + 2]| = |ri[1 * 3 + 0] ri[1 * 3 + 1] ri[1
  // * 3 + 2]| * |-dc[0] dc[1] 0.0|
  // |ro[2 * 3 + 0] ro[2 * 3 + 1] ro[2 * 3 + 2]|   |ri[2 * 3 + 0] ri[2 * 3 + 1] ri[2
  // * 3 + 2]|   |  0.0   0.0  1.0|

  // ro[0 * 3] =  ri[0 * 3] * dc[1] - ri[0 * 3 + 1] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] - ri[1 * 3 + 1] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] - ri[2 * 3 + 1] * dc[0];

  // ro[0 * 3 + 1] =  ri[0 * 3] * dc[0] + ri[0 * 3 + 1] * dc[1];
  // ro[1 * 3 + 1] =  ri[1 * 3] * dc[0] + ri[1 * 3 + 1] * dc[1];
  // ro[2 * 3 + 1] =  ri[2 * 3] * dc[0] + ri[2 * 3 + 1] * dc[1];

  // cblas_dcopy(3, ri + 2, 3, ro + 2, 3);
}


/*!
@brief Product of R and Rzinverse

Computes the product of a general rotation matrix, R, with the inverse of a
primitive rotation about the z axis, Rz^T, characterized by its direction
cosines.

\f$R_i * R_z^T \rightarrow R_o\f$

@param ri a pointer to a general rotation matrix
@param dc a pointer to an array of direction cosines
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRzinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  float dc[2],
  bksl_mat3x3f_t* __restrict__ ro) {

  memset(ro, 0, sizeof(bksl_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 3, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[0], (float*)ri, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_scopy(3, ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);

  // |ro[0 * 3 + 0] ro[0 * 3 + 1] ro[0 * 3 + 2]|   |ri[0 * 3 + 0] ri[0 * 3 + 1] ri[0
  // * 3 + 2]|   | dc[1] dc[0] 0.0|
  // |ro[1 * 3 + 0] ro[1 * 3 + 1] ro[1 * 3 + 2]| = |ri[1 * 3 + 0] ri[1 * 3 + 1] ri[1
  // * 3 + 2]| * |-dc[0] dc[1] 0.0|
  // |ro[2 * 3 + 0] ro[2 * 3 + 1] ro[2 * 3 + 2]|   |ri[2 * 3 + 0] ri[2 * 3 + 1] ri[2
  // * 3 + 2]|   |  0.0   0.0  1.0|

  // ro[0 * 3] =  ri[0 * 3] * dc[1] - ri[0 * 3 + 1] * dc[0];
  // ro[1 * 3] =  ri[1 * 3] * dc[1] - ri[1 * 3 + 1] * dc[0];
  // ro[2 * 3] =  ri[2 * 3] * dc[1] - ri[2 * 3 + 1] * dc[0];

  // ro[0 * 3 + 1] =  ri[0 * 3] * dc[0] + ri[0 * 3 + 1] * dc[1];
  // ro[1 * 3 + 1] =  ri[1 * 3] * dc[0] + ri[1 * 3 + 1] * dc[1];
  // ro[2 * 3 + 1] =  ri[2 * 3] * dc[0] + ri[2 * 3 + 1] * dc[1];

  // cblas_dcopy(3, ri + 2, 3, ro + 2, 3);
}


/*!
@brief Double precision product of two general rotation matrices

Computes the product of two general rotation matrices, R1i and R2i.

\f$R_{1i} * R_{2i} \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RR(
  bksl_mat3x3_t* __restrict__ R1i,
  bksl_mat3x3_t* __restrict__ R2i,
  bksl_mat3x3_t* __restrict__ ro) {

  cblas_dgemm(
    CblasColMajor,
    CblasNoTrans,
    CblasNoTrans,
    3,
    3,
    3,
    1.0,
    (double*)R1i,
    3,
    (double*)R2i,
    3,
    0.0,
    (double*)ro,
    3);
  // |ro[0][0] ro[0][1] ro[0][2]|   |R1i[0][0] R1i[0][1] R1i[0][2]|   |R2i[0][0]
  // R2i[0][1] R2i[0][2]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |R1i[1][0] R1i[1][1] R1i[1][2]| * |R2i[1][0]
  // R2i[1][1] R2i[1][2]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |R1i[2][0] R1i[2][1] R1i[2][2]|   |R2i[2][0]
  // R2i[2][1] R2i[2][2]|

  // ro[0][0] = R1i[0][0] * R2i[0][0] + R1i[0][1] * R2i[1][0] + R1i[0][2] *
  // R2i[2][0];
  // ro[0][1] = R1i[0][0] * R2i[0][1] + R1i[0][1] * R2i[1][1] + R1i[0][2] *
  // R2i[2][1];
  // ro[0][2] = R1i[0][0] * R2i[0][2] + R1i[0][1] * R2i[1][2] + R1i[0][2] *
  // R2i[2][2];
  // ro[1][0] = R1i[1][0] * R2i[0][0] + R1i[1][1] * R2i[1][0] + R1i[1][2] *
  // R2i[2][0];
  // ro[1][1] = R1i[1][0] * R2i[0][1] + R1i[1][1] * R2i[1][1] + R1i[1][2] *
  // R2i[2][1];
  // ro[1][2] = R1i[1][0] * R2i[0][2] + R1i[1][1] * R2i[1][2] + R1i[1][2] *
  // R2i[2][2];
  // ro[2][0] = R1i[2][0] * R2i[0][0] + R1i[2][1] * R2i[1][0] + R1i[2][2] *
  // R2i[2][0];
  // ro[2][1] = R1i[2][0] * R2i[0][1] + R1i[2][1] * R2i[1][1] + R1i[2][2] *
  // R2i[2][1];
  // ro[2][2] = R1i[2][0] * R2i[0][2] + R1i[2][1] * R2i[1][2] + R1i[2][2] *
  // R2i[2][2];
}


/*!
@brief single precision product of two general rotation matrices

Computes the product of two general rotation matrices, R1i and R2i.

\f$R_{1i} * R_{2i} \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRf(
  bksl_mat3x3f_t* __restrict__ R1i,
  bksl_mat3x3f_t* __restrict__ R2i,
  bksl_mat3x3f_t* __restrict__ ro) {

  cblas_sgemm(
    CblasColMajor,
    CblasNoTrans,
    CblasNoTrans,
    3,
    3,
    3,
    1.0,
    (float*)R1i,
    3,
    (float*)R2i,
    3,
    0.0,
    (float*)ro,
    3);
  // |ro[0][0] ro[0][1] ro[0][2]|   |R1i[0][0] R1i[0][1] R1i[0][2]|   |R2i[0][0]
  // R2i[0][1] R2i[0][2]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |R1i[1][0] R1i[1][1] R1i[1][2]| * |R2i[1][0]
  // R2i[1][1] R2i[1][2]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |R1i[2][0] R1i[2][1] R1i[2][2]|   |R2i[2][0]
  // R2i[2][1] R2i[2][2]|

  // ro[0][0] = R1i[0][0] * R2i[0][0] + R1i[0][1] * R2i[1][0] + R1i[0][2] *
  // R2i[2][0];
  // ro[0][1] = R1i[0][0] * R2i[0][1] + R1i[0][1] * R2i[1][1] + R1i[0][2] *
  // R2i[2][1];
  // ro[0][2] = R1i[0][0] * R2i[0][2] + R1i[0][1] * R2i[1][2] + R1i[0][2] *
  // R2i[2][2];
  // ro[1][0] = R1i[1][0] * R2i[0][0] + R1i[1][1] * R2i[1][0] + R1i[1][2] *
  // R2i[2][0];
  // ro[1][1] = R1i[1][0] * R2i[0][1] + R1i[1][1] * R2i[1][1] + R1i[1][2] *
  // R2i[2][1];
  // ro[1][2] = R1i[1][0] * R2i[0][2] + R1i[1][1] * R2i[1][2] + R1i[1][2] *
  // R2i[2][2];
  // ro[2][0] = R1i[2][0] * R2i[0][0] + R1i[2][1] * R2i[1][0] + R1i[2][2] *
  // R2i[2][0];
  // ro[2][1] = R1i[2][0] * R2i[0][1] + R1i[2][1] * R2i[1][1] + R1i[2][2] *
  // R2i[2][1];
  // ro[2][2] = R1i[2][0] * R2i[0][2] + R1i[2][1] * R2i[1][2] + R1i[2][2] *
  // R2i[2][2];
}

/*!
@brief Product of a general rotation matrix and the inverse of a general
rotation matrix

Computes the product of a general rotation matrix, R1i, and the inverse
of a second rotation matrix, R2i.

\f$R_{1i} * R_{2i}^T \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRinverse(
  bksl_mat3x3_t* __restrict__ R1i,
  bksl_mat3x3_t* __restrict__ R2i,
  bksl_mat3x3_t* __restrict__ ro) {

  cblas_dgemm(
    CblasColMajor,
    CblasNoTrans,
    CblasTrans,
    3,
    3,
    3,
    1.0,
    (double*)R1i,
    3,
    (double*)R2i,
    3,
    0.0,
    (double*)ro,
    3);

  // |ro[0][0] ro[0][1] ro[0][2]|   |R1i[0][0] R1i[0][1] R1i[0][2]|   |R2i[0][0]
  // R2i[1][0] R2i[2][0]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |R1i[1][0] R1i[1][1] R1i[1][2]| * |R2i[0][1]
  // R2i[1][1] R2i[2][1]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |R1i[2][0] R1i[2][1] R1i[2][2]|   |R2i[0][2]
  // R2i[1][2] R2i[2][2]|

  // ro[0][0] = R1i[0][0] * R2i[0][0] + R1i[0][1] * R2i[0][1] + R1i[0][2] *
  // R2i[0][2];
  // ro[0][1] = R1i[0][0] * R2i[1][0] + R1i[0][1] * R2i[1][1] + R1i[0][2] *
  // R2i[1][2];
  // ro[0][2] = R1i[0][0] * R2i[2][0] + R1i[0][1] * R2i[2][1] + R1i[0][2] *
  // R2i[2][2];
  // ro[1][0] = R1i[1][0] * R2i[0][0] + R1i[1][1] * R2i[0][1] + R1i[1][2] *
  // R2i[0][2];
  // ro[1][1] = R1i[1][0] * R2i[1][0] + R1i[1][1] * R2i[1][1] + R1i[1][2] *
  // R2i[1][2];
  // ro[1][2] = R1i[1][0] * R2i[2][0] + R1i[1][1] * R2i[2][1] + R1i[1][2] *
  // R2i[2][2];
  // ro[2][0] = R1i[2][0] * R2i[0][0] + R1i[2][1] * R2i[0][1] + R1i[2][2] *
  // R2i[0][2];
  // ro[2][1] = R1i[2][0] * R2i[1][0] + R1i[2][1] * R2i[1][1] + R1i[2][2] *
  // R2i[1][2];
  // ro[2][2] = R1i[2][0] * R2i[2][0] + R1i[2][1] * R2i[2][1] + R1i[2][2] *
  // R2i[2][2];
}


/*!
@brief Product of a general rotation matrix and the inverse of a general
rotation matrix

Computes the product of a general rotation matrix, R1i, and the inverse
of a second rotation matrix, R2i.

\f$R_{1i} * R_{2i}^T \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RRinversef(
  bksl_mat3x3f_t* __restrict__ R1i,
  bksl_mat3x3f_t* __restrict__ R2i,
  bksl_mat3x3f_t* __restrict__ ro) {

  cblas_sgemm(
    CblasColMajor,
    CblasNoTrans,
    CblasTrans,
    3,
    3,
    3,
    1.0,
    (float*)R1i,
    3,
    (float*)R2i,
    3,
    0.0,
    (float*)ro,
    3);

  // |ro[0][0] ro[0][1] ro[0][2]|   |R1i[0][0] R1i[0][1] R1i[0][2]|   |R2i[0][0]
  // R2i[1][0] R2i[2][0]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |R1i[1][0] R1i[1][1] R1i[1][2]| * |R2i[0][1]
  // R2i[1][1] R2i[2][1]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |R1i[2][0] R1i[2][1] R1i[2][2]|   |R2i[0][2]
  // R2i[1][2] R2i[2][2]|

  // ro[0][0] = R1i[0][0] * R2i[0][0] + R1i[0][1] * R2i[0][1] + R1i[0][2] *
  // R2i[0][2];
  // ro[0][1] = R1i[0][0] * R2i[1][0] + R1i[0][1] * R2i[1][1] + R1i[0][2] *
  // R2i[1][2];
  // ro[0][2] = R1i[0][0] * R2i[2][0] + R1i[0][1] * R2i[2][1] + R1i[0][2] *
  // R2i[2][2];
  // ro[1][0] = R1i[1][0] * R2i[0][0] + R1i[1][1] * R2i[0][1] + R1i[1][2] *
  // R2i[0][2];
  // ro[1][1] = R1i[1][0] * R2i[1][0] + R1i[1][1] * R2i[1][1] + R1i[1][2] *
  // R2i[1][2];
  // ro[1][2] = R1i[1][0] * R2i[2][0] + R1i[1][1] * R2i[2][1] + R1i[1][2] *
  // R2i[2][2];
  // ro[2][0] = R1i[2][0] * R2i[0][0] + R1i[2][1] * R2i[0][1] + R1i[2][2] *
  // R2i[0][2];
  // ro[2][1] = R1i[2][0] * R2i[1][0] + R1i[2][1] * R2i[1][1] + R1i[2][2] *
  // R2i[1][2];
  // ro[2][2] = R1i[2][0] * R2i[2][0] + R1i[2][1] * R2i[2][1] + R1i[2][2] *
  // R2i[2][2];
}


/*!
@brief Double precision product of the inverse of a general rotation matrix and a general
rotation matrix

Computes the product of the inverse of a general rotation matrix, R1i, and
a second rotation matrix, R2i.

\f$R_{1i}^T * R_{2i} \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RinverseR(
  bksl_mat3x3_t* __restrict__ R1i,
  bksl_mat3x3_t* __restrict__ R2i,
  bksl_mat3x3_t* __restrict__ ro) {

  cblas_dgemm(
    CblasColMajor,
    CblasTrans,
    CblasNoTrans,
    3,
    3,
    3,
    1.0,
    (double*)R1i,
    3,
    (double*)R2i,
    3,
    0.0,
    (double*)ro,
    3);

  // |ro[0][0] ro[0][1] ro[0][2]|   |R1i[0][0] R1i[1][0] R1i[2][0]|   |R2i[0][0]
  // R2i[0][1] R2i[0][2]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |R1i[0][1] R1i[1][1] R1i[2][1]| * |R2i[1][0]
  // R2i[1][1] R2i[1][2]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |R1i[0][2] R1i[1][2] R1i[2][2]|   |R2i[2][0]
  // R2i[2][1] R2i[2][2]|

  // ro[0][0] = R1i[0][0] * R2i[0][0] + R1i[1][0] * R2i[1][0] + R1i[2][0] *
  // R2i[2][0];
  // ro[0][1] = R1i[0][0] * R2i[0][1] + R1i[1][0] * R2i[1][1] + R1i[2][0] *
  // R2i[2][1];
  // ro[0][2] = R1i[0][0] * R2i[0][2] + R1i[1][0] * R2i[1][2] + R1i[2][0] *
  // R2i[2][2];

  // ro[1][0] = R1i[0][1] * R2i[0][0] + R1i[1][1] * R2i[1][0] + R1i[2][1] *
  // R2i[2][0];
  // ro[1][1] = R1i[0][1] * R2i[0][1] + R1i[1][1] * R2i[1][1] + R1i[2][1] *
  // R2i[2][1];
  // ro[1][2] = R1i[0][1] * R2i[0][2] + R1i[1][1] * R2i[1][2] + R1i[2][1] *
  // R2i[2][2];

  // ro[2][0] = R1i[0][2] * R2i[0][0] + R1i[1][2] * R2i[1][0] + R1i[2][2] *
  // R2i[2][0];
  // ro[2][1] = R1i[0][2] * R2i[0][1] + R1i[1][2] * R2i[1][1] + R1i[2][2] *
  // R2i[2][1];
  // ro[2][2] = R1i[0][2] * R2i[0][2] + R1i[1][2] * R2i[1][2] + R1i[2][2] *
  // R2i[2][2];
}



/*!
@brief single precision product of the inverse of a general rotation matrix and a general
rotation matrix

Computes the product of the inverse of a general rotation matrix, R1i, and
a second rotation matrix, R2i.

\f$R_{1i}^T * R_{2i} \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_RinverseRf(
  bksl_mat3x3f_t* __restrict__ R1i,
  bksl_mat3x3f_t* __restrict__ R2i,
  bksl_mat3x3f_t* __restrict__ ro) {

  cblas_sgemm(
    CblasColMajor,
    CblasTrans,
    CblasNoTrans,
    3,
    3,
    3,
    1.0,
    (float*)R1i,
    3,
    (float*)R2i,
    3,
    0.0,
    (float*)ro,
    3);

  // |ro[0][0] ro[0][1] ro[0][2]|   |R1i[0][0] R1i[1][0] R1i[2][0]|   |R2i[0][0]
  // R2i[0][1] R2i[0][2]|
  // |ro[1][0] ro[1][1] ro[1][2]| = |R1i[0][1] R1i[1][1] R1i[2][1]| * |R2i[1][0]
  // R2i[1][1] R2i[1][2]|
  // |ro[2][0] ro[2][1] ro[2][2]|   |R1i[0][2] R1i[1][2] R1i[2][2]|   |R2i[2][0]
  // R2i[2][1] R2i[2][2]|

  // ro[0][0] = R1i[0][0] * R2i[0][0] + R1i[1][0] * R2i[1][0] + R1i[2][0] *
  // R2i[2][0];
  // ro[0][1] = R1i[0][0] * R2i[0][1] + R1i[1][0] * R2i[1][1] + R1i[2][0] *
  // R2i[2][1];
  // ro[0][2] = R1i[0][0] * R2i[0][2] + R1i[1][0] * R2i[1][2] + R1i[2][0] *
  // R2i[2][2];

  // ro[1][0] = R1i[0][1] * R2i[0][0] + R1i[1][1] * R2i[1][0] + R1i[2][1] *
  // R2i[2][0];
  // ro[1][1] = R1i[0][1] * R2i[0][1] + R1i[1][1] * R2i[1][1] + R1i[2][1] *
  // R2i[2][1];
  // ro[1][2] = R1i[0][1] * R2i[0][2] + R1i[1][1] * R2i[1][2] + R1i[2][1] *
  // R2i[2][2];

  // ro[2][0] = R1i[0][2] * R2i[0][0] + R1i[1][2] * R2i[1][0] + R1i[2][2] *
  // R2i[2][0];
  // ro[2][1] = R1i[0][2] * R2i[0][1] + R1i[1][2] * R2i[1][1] + R1i[2][2] *
  // R2i[2][1];
  // ro[2][2] = R1i[0][2] * R2i[0][2] + R1i[1][2] * R2i[1][2] + R1i[2][2] *
  // R2i[2][2];
}

/*!
@brief Double precision product of the a general rotation matrix and a
vector consisting of a translation along the x axis

Computes the product of a general rotation matrix and a primitive translation
along the x axis using sparse algebra.

\f$R_{i} * t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param to a pointer to a vector where result is stored.
*/
inline void bksl_product_Rvx(
  bksl_mat3x3_t* __restrict__ ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(double));
  cblas_daxpy(3, *ti, (double*)ri, 1, (double*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   | ti|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] = ri[0 * 3] * ti;
  // to[0+1] = ri[1 * 3] * ti;
  // to[0+2] = ri[2 * 3] * ti;
}


/*!
@brief single precision product of the a general rotation matrix and a primitive translation
along the x axis

Computes the product of a general rotation matrix and a primitive translation
along the x axis using sparse algebra.

\f$R_{i} * t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvxf(
  bksl_mat3x3f_t* __restrict__ ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(float));
  cblas_saxpy(3, *ti, (float*)ri, 1, (float*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   | ti|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] = ri[0 * 3] * ti;
  // to[0+1] = ri[1 * 3] * ti;
  // to[0+2] = ri[2 * 3] * ti;
}


/*!
@brief Double precision product of the a general rotation matrix and the inverse of a
primitive translation along the x axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * -t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvxinverse(
  bksl_mat3x3_t* __restrict__ ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(double));
  cblas_daxpy(3, -(*ti), (double*)ri, 1, (double*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |-ti|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] = -ri[0 * 3] * ti;
  // to[0+1] = -ri[1 * 3] * ti;
  // to[0+2] = -ri[2 * 3] * ti;
}


/*!
@brief single precision product of the a general rotation matrix and the inverse of a
primitive translation along the x axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * -t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvxinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(float));
  cblas_saxpy(3, -(*ti), (float*)ri, 1, (float*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |-ti|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] = -ri[0 * 3] * ti;
  // to[0+1] = -ri[1 * 3] * ti;
  // to[0+2] = -ri[2 * 3] * ti;
}

/*!
@brief Double precision product of the a general rotation matrix and a primitive
translation along the y axis

Computes the product of a general rotation matrix and a primitive translation
along the y axis using sparse algebra.

\f$R_{i} * t_{y} \rightarrow t_o\f$

is equivalent to:

\f$R_i[1,4,7] * t_y \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvy(
  bksl_mat3x3_t* __restrict__ ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(double));
  cblas_daxpy(3, *ti, ((double*)ri) + 3, 1, (double*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * | ti|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] =  ri[0 * 3 + 1] * ti;
  // to[0+1] =  ri[1 * 3 + 1] * ti;
  // to[0+2] =  ri[2 * 3 + 1] * ti;
}

/*!
@brief single precision product of the a general rotation matrix and a
primitive translation along the y axis

Computes the product of a general rotation matrix and a primitive translation
along the y axis using sparse algebra.

\f$R_{i} * t_{y} \rightarrow t_o\f$

is equivalent to:

\f$R_i[1,4,7] * t_y \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvyf(
  bksl_mat3x3f_t* __restrict__ ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(float));
  cblas_saxpy(3, *ti, ((float*)ri) + 3, 1, (float*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * | ti|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] =  ri[0 * 3 + 1] * ti;
  // to[0+1] =  ri[1 * 3 + 1] * ti;
  // to[0+2] =  ri[2 * 3 + 1] * ti;
}

/*!
@brief Double precision product of a general rotation matrix and the
inverse of a primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{y} \rightarrow t_o\f$

is equivalent to:

\f$R_i[1,4,7] * -t_y \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvyinverse(
  bksl_mat3x3_t* __restrict__ ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(double));
  cblas_daxpy(3, -(*ti), ((double*)ri) + 3, 1, (double*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |-ti|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] = -ri[0 * 3 + 1] * ti;
  // to[0+1] = -ri[1 * 3 + 1] * ti;
  // to[0+2] = -ri[2 * 3 + 1] * ti;
}

/*!
@brief single precision product of a general rotation matrix and the inverse of a
primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{y} \rightarrow t_o\f$

is equivalent to:

\f$R_i[1,4,7] * -t_y \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvyinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(float));
  cblas_saxpy(3, -(*ti), ((float*)ri) + 3, 1, (float*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |-ti|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0|

  // to[0] = -ri[0 * 3 + 1] * ti;
  // to[0+1] = -ri[1 * 3 + 1] * ti;
  // to[0+2] = -ri[2 * 3 + 1] * ti;
}


/*!
@brief Double precision product of a general rotation matrix and a
primitive translation along the z axis

Computes the product of a general rotation matrix and a primitive translation
along the z axis using sparse algebra.

\f$R_{i} * t_{z} \rightarrow t_o\f$

is equivalent to:

\f$R_i[2,5,8] * t_z \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvz(
  bksl_mat3x3_t* __restrict__ ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(double));
  cblas_daxpy(3, *ti, ((double*)ri) + 6, 1, (double*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   | ti|

  // to[0] =  ri[0 * 3 + 2] * ti;
  // to[0+1] =  ri[1 * 3 + 2] * ti;
  // to[0+2] =  ri[2 * 3 + 2] * ti;
}

/*!
@brief Double precision product of a general rotation matrix and a
primitive translation along the z axis

Computes the product of a general rotation matrix and a primitive translation
along the z axis using sparse algebra.

\f$R_{i} * t_{z} \rightarrow t_o\f$

is equivalent to:

\f$R_i[2,5,8] * t_z \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvzf(
  bksl_mat3x3f_t* __restrict__ ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(float));
  cblas_saxpy(3, *ti, ((float*)ri) + 6, 1, (float*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   | ti|

  // to[0] =  ri[0 * 3 + 2] * ti;
  // to[0+1] =  ri[1 * 3 + 2] * ti;
  // to[0+2] =  ri[2 * 3 + 2] * ti;
}

/*!
@brief Double precision product of the a general rotation matrix and
the inverse of a primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{z} \rightarrow t_o\f$

is equivalent to:

\f$R_i[2,5,8] * -t_z \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvzinverse(
  bksl_mat3x3_t* __restrict__ ri,
  double* __restrict__ ti,
  bksl_vec3_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(double));
  cblas_daxpy(3, -(*ti), ((double*)ri) + 6, 1, (double*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |-ti|

  // to[0] = -ri[0 * 3 + 2] * ti;
  // to[0+1] = -ri[1 * 3 + 2] * ti;
  // to[0+2] = -ri[2 * 3 + 2] * ti;
}


/*!
@brief single precision product of the a general rotation matrix and
the inverse of a primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{z} \rightarrow t_o\f$

is equivalent to:

\f$R_i[2,5,8] * -t_z \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void bksl_product_Rvzinversef(
  bksl_mat3x3f_t* __restrict__ ri,
  float* __restrict__ ti,
  bksl_vec3f_t* __restrict__ to) {

  memset(to, 0, 3 * sizeof(float));
  cblas_saxpy(3, -(*ti), ((float*)ri) + 6, 1, (float*)to, 1);

  // |to[0]|   |ri[0][0] ri[0][1] ri[0][2]|   |0.0|
  // |to[1]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0|
  // |to[2]|   |ri[2][0] ri[2][1] ri[2][2]|   |-ti|

  // to[0] = -ri[0 * 3 + 2] * ti;
  // to[0+1] = -ri[1 * 3 + 2] * ti;
  // to[0+2] = -ri[2 * 3 + 2] * ti;
}

/*!
@}
@name Utilities for Translations
Utilities for performing matrix algebra on 3x1 translation vectors.
@{
*/

/*!
@brief Add a double precision Vector with a transformation along x-axis

Computes the result of a general 3-d translation, followed by a translation
along the x axis using sparse algebra.

\f$t_{i} + t_{x} \rightarrow t_o\f$

is equivalent to:

\f$t_{i} \rightarrow t_o\f$

\f$t_o[0] + t_x \rightarrow t_o[0]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvx(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to) {

  bksl_copy_vv(t1i, to);
  to->x += *t2i;
}

/*!
@brief Add a single precision Vector with a transformation along x-axis

Computes the result of a general 3-d translation, followed by a translation
along the x axis using sparse algebra.

\f$t_{i} + t_{x} \rightarrow t_o\f$

is equivalent to:

\f$t_{i} \rightarrow t_o\f$

\f$t_o[0] + t_x \rightarrow t_o[0]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvxf(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to) {

  bksl_copy_vvf(t1i, to);
  to->x += *t2i;
}

/*!
@brief Add a double precision Vector with the inverse of a translation
along the x-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the x axis using sparse algebra.

\f$t_{i} + t_{x} \rightarrow t_o\f$

is equivalent to:

\f$t_{i} \rightarrow t_o\f$

\f$t_o[0] + t_x \rightarrow t_o[0]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvxinverse(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to) {

  bksl_copy_vv(t1i, to);
  to->x -= *t2i;
  // |to[0]|   |t1i[0]|   |t2i|
  // |to[1]| = |t1i[1]| - |0.0|
  // |to[2]|   |t1i[2]|   |0.0|
}

/*!
@brief Add a single precision Vectorf with the inverse of a translation
along the x-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the x axis using sparse algebra.

\f$t_{i} + t_{x} \rightarrow t_o\f$

is equivalent to:

\f$t_{i} \rightarrow t_o\f$

\f$t_o[0] + t_x \rightarrow t_o[0]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvxinversef(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to) {

  bksl_copy_vvf(t1i, to);
  to->x -= *t2i;
  // |to[0]|   |t1i[0]|   |t2i|
  // |to[1]| = |t1i[1]| - |0.0|
  // |to[2]|   |t1i[2]|   |0.0|
}

/*!
@brief Add a double precision Vector with a translation
along the y-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the y axis using sparse algebra.

\f$t_{i} + t_{y} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[1] + t_y \rightarrow t_o[1]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvy(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to) {

  bksl_copy_vv(t1i, to);
  to->y += *t2i;

  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| + |t2i|
  // |to[2]|   |t1i[2]|   |0.0|
}

/*!
@brief Add a single precision Vectorf with a translation
along the y-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the y axis using sparse algebra.

\f$t_{i} + t_{y} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[1] + t_y \rightarrow t_o[1]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvyf(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to) {

  bksl_copy_vvf(t1i, to);
  to->y += *t2i;

  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| + |t2i|
  // |to[2]|   |t1i[2]|   |0.0|
}

/*!
@brief Add a double precision Vector with the inverse of a translation
along the y-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the y axis using sparse algebra.

\f$t_{i} + t_{y} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[1] - t_y \rightarrow t_o[1]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvyinverse(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to) {

  bksl_copy_vv(t1i, to);
  to->y -= *t2i;

  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| - |t2i|
  // |to[2]|   |t1i[2]|   |0.0|
}

/*!
@brief Add a single precision Vector with the inverse of a translation
along the y-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the y axis using sparse algebra.

\f$t_{i} + t_{y} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[1] - t_y \rightarrow t_o[1]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvyinversef(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to) {

  bksl_copy_vvf(t1i, to);
  to->y -= *t2i;

  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| - |t2i|
  // |to[2]|   |t1i[2]|   |0.0|
}

/*!
@brief Add a double precision Vector with a translation
along the z-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the z axis using sparse algebra.

\f$t_{i} + t_{z} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[2] + t_z \rightarrow t_o[2]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvz(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to) {

  bksl_copy_vv(t1i, to);
  to->z += *t2i;

  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| + |0.0|
  // |to[2]|   |t1i[2]|   |t2i|
}

/*!
@brief Add a single precision Vectorf with a translation
along the z-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the z axis using sparse algebra.

\f$t_{i} + t_{z} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[2] + t_z \rightarrow t_o[2]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvzf(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to) {

  bksl_copy_vvf(t1i, to);
  to->z += *t2i;

  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| + |0.0|
  // |to[2]|   |t1i[2]|   |t2i|
}

/*!
@brief Add a double precision Vector with the inverse of a translation
along the z-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the z axis using sparse algebra.

\f$t_{i} + t_{z} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[2] - t_z \rightarrow t_o[2]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvzinverse(
  bksl_vec3_t* __restrict__ t1i,
  double* __restrict__ t2i,
  bksl_vec3_t* __restrict__ to) {

  bksl_copy_vv(t1i, to);
  to->z -= *t2i;
  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| - |0.0|
  // |to[2]|   |t1i[2]|   |t2i|
}

/*!
@brief Add a single precision Vector with the inverse of a translation
along the z-axis

Computes the result of a general 3-d transformation, followed by a tranformation
along the z axis using sparse algebra.

\f$t_{i} + t_{z} \rightarrow t_o\f$

is equivalent to:

\f$t_{i}[0:2] \rightarrow t_o[0:2]\f$

\f$t_o[2] - t_z \rightarrow t_o[2]\f$

@param t1i a pointer to a general 3-d translation
@param t2i a pointer to a double
@param to a pointer to a general 3-d translation where the result is stored.
*/
inline void bksl_add_vvzinversef(
  bksl_vec3f_t* __restrict__ t1i,
  float* __restrict__ t2i,
  bksl_vec3f_t* __restrict__ to) {

  bksl_copy_vvf(t1i, to);
  to->z -= *t2i;
  // |to[0]|   |t1i[0]|   |0.0|
  // |to[1]| = |t1i[1]| - |0.0|
  // |to[2]|   |t1i[2]|   |t2i|
}

/*!
@}
@name Utilities for general displacements
Utilities to perform matrix algebra on general displacements, consisting
of both a rotation and translation component.
@{
*/

/*!
@brief Compute double precision inverse of a bksl_displacement_t

\f$D_i^{-1} \rightarrow D_o\f$

@param Di a general displacement to take the inverse of
@param Do inverse of Di is returned in Do
*/
inline void bksl_displacement_invert(
  bksl_displacement_t* __restrict__ Di,
  bksl_displacement_t* __restrict__ Do) {

  bksl_invert_R(&(Di->R), &(Do->R));
  bksl_product_Rinversevinverse(&(Di->R), &(Di->t), &(Do->t));
}

/*!
@brief Compute single precision inverse of a bksl_displacement_t

\f$D_i^{-1} \rightarrow D_o\f$

@param Di a general displacement to take the inverse of
@param Do inverse of Di is returned in Do
*/
inline void bksl_displacementf_invert(
  bksl_displacementf_t* __restrict__ Di,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_invert_Rf(&(Di->R), &(Do->R));
  bksl_product_Rinversevinversef(&(Di->R), &(Di->t), &(Do->t));
}

/*!
@brief Copy a double precision bksl_displacement_t consisting of a rotation and
translation

\f$D_i \rightarrow D_o\f$

@param Di a general displacement input
@param Do Di is copied to Do
*/
inline void bksl_displacement_copy(
  bksl_displacement_t* __restrict__ Di,
  bksl_displacement_t* __restrict__ Do) {

  memcpy(Do, Di, sizeof(bksl_displacement_t));
  // cblas_dcopy(12, (double*)Di, 1, (double*)Do, 1);
}

/*!
@brief Copy a single precision bksl_displacementf_t consisting of a rotation and
translation

\f$D_i \rightarrow D_o\f$

@param Di a general displacement input
@param Do Di is copied to Do
*/
inline void bksl_displacementf_copy(
  bksl_displacementf_t* __restrict__ Di,
  bksl_displacementf_t* __restrict__ Do) {

  memcpy(Do, Di, sizeof(bksl_displacementf_t));
  // cblas_scopy(12, (float*)Di, 1, (float*)Do, 1);
}

/*!
@brief Double precision product of a bksl_displacement_t with a primitive rotation
about the x-axis

\f$R_i * R_x \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a pointer to a general displacement
@param dc direction cosines characterizing a primitive rotation
@param the result of the computation is stored in Do
*/
inline void bksl_product_DDrx(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRx(&(Di->R), dc, &(Do->R));
  bksl_copy_vv(&(Di->t), &(Do->t));
}

/*!
@brief single precision product of a bksl_displacement_t with a primitive rotation
about the x-axis

\f$R_i * R_x \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a pointer to a general displacement
@param dc direction cosines characterizing a primitive rotation
@param the result of the computation is stored in Do
*/
inline void bksl_product_DDrxf(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRxf(&(Di->R), dc, &(Do->R));
  bksl_copy_vvf(&(Di->t), &(Do->t));
}

/*!
@brief Double precision product of a general displacement with the inverse
of a primitive rotation about the x-axis

\f$D_i * R_x^T \rightarrow D_o\f$

is equivalent to:

\f$R_i * R_x^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a pointer to a general displacement
@param dc direction cosines characterizing a primitive rotation
@param the result of the computation is stored in Do
*/
inline void bksl_product_DDrxinverse(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRxinverse(&(Di->R), dc, &(Do->R));
  bksl_copy_vv(&(Di->t), &(Do->t));
}

/*!
@brief single precision product of a general displacement with the inverse
of a primitive rotation about the x-axis

\f$D_i * R_x^T \rightarrow D_o\f$

is equivalent to:

\f$R_i * R_x^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a pointer to a general displacement
@param dc direction cosines characterizing a primitive rotation
@param the result of the computation is stored in Do
*/
inline void bksl_product_DDrxinversef(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRxinversef(&(Di->R), dc, &(Do->R));
  bksl_copy_vvf(&(Di->t), &(Do->t));
}

/*!
@brief Double precision product of a bksl_displacement_t, \f$D\f$, and a rotation
about the y axis, \f$R_y\f$

The following operation:
\f$D_i * R_y \rightarrow D_o\f$

is equivalent to:

\f$R_i * R_y \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Y axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDry(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRy(&(Di->R), dc, &(Do->R));
  bksl_copy_vv(&(Di->t), &(Do->t));
}

/*!
@brief single precision product of a bksl_displacementf_t, \f$D\f$, and a rotation
about the y axis, \f$R_y\f$

The following operation:
\f$D_i * R_y \rightarrow D_o\f$

is equivalent to:

\f$R_i * R_y \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Y axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDryf(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRyf(&(Di->R), dc, &(Do->R));
  bksl_copy_vvf(&(Di->t), &(Do->t));
}

/*!
@brief Double precision product of a bksl_displacement_t, \f$D\f$, and the inverse of a
rotation about the y axis, \f$R_y^T\f$

The following operation:
\f$D_i * R_y^T \rightarrow D_o\f$

Can be split in to:

\f$R_i * R_y^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Y axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDryinverse(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRyinverse(&(Di->R), dc, &(Do->R));
  bksl_copy_vv(&(Di->t), &(Do->t));
}

/*!
@brief single precision product of a bksl_displacementf_t, \f$D\f$, and the inverse of a
rotation about the y axis, \f$R_y^T\f$

The following operation:
\f$D_i * R_y^T \rightarrow D_o\f$

Can be split in to:

\f$R_i * R_y^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Y axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDryinversef(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRyinversef(&(Di->R), dc, &(Do->R));
  bksl_copy_vvf(&(Di->t), &(Do->t));
}

/*!
@brief Double precision product of a general displacement, \f$D\f$, and a
rotation about the z axis, \f$R_z\f$

The following operation:
\f$D_i * R_z \rightarrow D_o\f$

Can be split in to:

\f$R_i*R_z \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Z axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrz(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRz(&(Di->R), dc, &(Do->R));
  bksl_copy_vv(&(Di->t), &(Do->t));
}

/*!
@brief single precision product of a bksl_displacementf_t, \f$D\f$, and a
rotation about the z axis, \f$R_z\f$

The following operation:
\f$D_i * R_z \rightarrow D_o\f$

Can be split in to:

\f$R_i*R_z \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Z axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrzf(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRzf(&(Di->R), dc, &(Do->R));
  bksl_copy_vvf(&(Di->t), &(Do->t));
}

/*!
@brief Double precision product of a bksl_displacement_t, \f$D\f$, and an inverse
rotation about the z axis, \f$R_z^T\f$

The following operation:
\f$D_i * R_z^T \rightarrow D_o\f$

Can be split in to:

\f$R_i * R_z^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Z axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrzinverse(
  bksl_displacement_t* __restrict__ Di,
  double dc[2],
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRzinverse(&(Di->R), dc, &(Do->R));
  bksl_copy_vv(&(Di->t), &(Do->t));
}

/*!
@brief single precision product of a bksl_displacement_t, \f$D\f$, and an inverse
rotation about the z axis, \f$R_z^T\f$

The following operation:
\f$D_i * R_z^T \rightarrow D_o\f$

Can be split in to:

\f$R_i * R_z^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Z axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrzinversef(
  bksl_displacementf_t* __restrict__ Di,
  float dc[2],
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRzinversef(&(Di->R), dc, &(Do->R));
  bksl_copy_vvf(&(Di->t), &(Do->t));
}


/*!
@brief Double precision product of a general bksl_displacement_t with a vector

@param Di a general displacement, \f$D_i\f$
@param vi a general 3-d Vector
@param vo the results are stored in vector, \f$v_o\f$
*/
inline void bksl_product_Dv(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {

  bksl_product_Rv(&(Di->R), vi, vo);
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    (double*)vo,
    1);
}


/*!
@brief Double precision product of a general bksl_displacement_t inverse
with a vector

@param Di a general displacement, \f$D\f$
@param vi a general 3-d Vector
@param vo the results are stored in vector, \f$v_o\f$
*/
inline void bksl_product_Dinversev(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_vec3_t* __restrict__ vo) {


  bksl_vec3_t temp;

  /* R^-1 * t -> temp*/
  bksl_product_Rinversev(
    &(Di->R),
    &(Di->t),
    &temp);

  /* R^-1 * vi -> vo */
  bksl_product_Rinversev(&(Di->R), vi, vo);

  /* vo -= temp */
  cblas_daxpy(
    3,
    -1.0,
    (double*)&temp,
    1,
    (double*)vo,
    1);
}


/*!
@brief Double precision product of a general bksl_displacement_t, followed by
a 3D translation

Computes the result of a general displacement, followed by a general
3-d translation.

\f$R_i \rightarrow R_o\f$

\f$v1i + ri*v2i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param vi[3] a general 3-d Vector
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDt(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_displacement_t* __restrict__ Do) {

  cblas_dcopy(9, (double*)Di, 1, (double*)Do, 1);
  bksl_product_Rv(&(Di->R), vi, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}

/*!
@brief Double precision product of a general bksl_displacement_t, followed by
a 3D translation

Computes the result of a general displacement, followed by the inverse of
a general 3-d translation.

\f$R_i \rightarrow R_o\f$

\f$v1i + ri*v2i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param vi[3] a general 3-d Vector
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_displacement_t* __restrict__ Do) {

  cblas_dcopy(9, (double*)Di, 1, (double*)Do, 1);
  bksl_product_Rvinverse(&(Di->R), vi, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}

/*!
@brief Product of a general bksl_displacement_t and a displacement consisting
of only a translation component

Computes the result of a general displacement, followed by a general
3-d translation.

\f$R_i \rightarrow R_o\f$

\f$v1i + ri*v2i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param vi[3] a general 3-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_vec3f_t* __restrict__ vi,
  bksl_displacementf_t* __restrict__ Do) {

  cblas_scopy(9, (float*)Di, 1, (float*)Do, 1);
  bksl_product_Rvf(&(Di->R), vi, &(Do->t));
  cblas_saxpy(
    3,
    1.0,
    ((float*)Di) + 9,
    1,
    ((float*)Do) + 9,
    1);
}


/*!
@brief Double precision product of the inverse of a general
displacement, \f$D_i^{-1}\f$, and a general translation \f$v_{i}\f$

The following operation:
\f$D_i^{-1} * V \rightarrow D_o\f$

Can be split in to:

\f$R_i^{T} \rightarrow R_o\f$

\f$R_i^T * (V_i - T_i) \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Z axis.
@param Do the results are returned in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DinverseDt(
  bksl_displacement_t* __restrict__ Di,
  bksl_vec3_t* __restrict__ vi,
  bksl_displacement_t* __restrict__ Do) {

  bksl_subtract_vv(vi, &(Di->t), &(Do->t));
  bksl_mat3x3_invert(&(Di->R), &(Do->R));
  bksl_product_Rinversev(&(Di->R), &(Do->t), &(Do->t));
}


/*!
@brief single precision product of the inverse of a general
displacement, \f$D_i^{-1}\f$, and a general translation \f$v_{i}\f$

The following operation:
\f$D_i^{-1} * V \rightarrow D_o\f$

Can be split in to:

\f$R_i^{T} \rightarrow R_o\f$

\f$R_i^T * (V_i - T_i) \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Z axis.
@param Do the results are returned in a general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_vec3f_t* __restrict__ vi,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_subtract_vvf(vi, &(Di->t), &(Do->t));
  bksl_invert_Rf(&(Di->R), &(Do->R));
  bksl_product_Rinversevf(&(Di->R), &(Do->t), &(Do->t));
}


/*!
@brief Product of a general displacement and a primitive translation
along the x-axis

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_x \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtx(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do) {

  bksl_mat3x3_copy(&(Di->R), &(Do->R));
  bksl_product_Rvx(&(Di->R), ti, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief Double precision product of a bksl_displacement_t and a primitive translation

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_x \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtxinverse(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do) {

  bksl_mat3x3_copy(&(Di->R), &(Do->R));
  bksl_product_Rvxinverse(&(Di->R), ti, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief single precision product of a bksl_displacement_t and a primitive translation

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_x \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtxinversef(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_mat3x3f_copy(&(Di->R), &(Do->R));
  bksl_product_Rvxinversef(&(Di->R), ti, &(Do->t));
  cblas_saxpy(
    3,
    1.0,
    ((float*)Di) + 9,
    1,
    ((float*)Do) + 9,
    1);
}

/*!
@brief Double precision general bksl_displacement_t multiplied with a primitive
translation along the y-axis

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDty(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do) {

  bksl_mat3x3_copy(&(Di->R), &(Do->R));
  bksl_product_Rvy(&(Di->R), ti, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief single precision bksl_displacementf_t multiplied with a primitive
translation along the y-axis

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtyf(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_mat3x3f_copy(&(Di->R), &(Do->R));
  bksl_product_Rvyf(&(Di->R), ti, &(Do->t));
  cblas_saxpy(
    3,
    1.0,
    ((float*)Di) + 9,
    1,
    ((float*)Do) + 9,
    1);
}


/*!
@brief Double precision bksl_displacement_t multiplied with the inverse of a
primitive translation along y axis

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtyinverse(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do) {

  bksl_mat3x3_copy(&(Di->R), &(Do->R));
  bksl_product_Rvyinverse(&(Di->R), ti, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief single precision bksl_displacementf_t multiplied with the inverse of a
primitive translation along y axis

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtyinversef(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_mat3x3f_copy(&(Di->R), &(Do->R));
  bksl_product_Rvyinversef(&(Di->R), ti, &(Do->t));
  cblas_saxpy(
    3,
    1.0,
    ((float*)Di) + 9,
    1,
    ((float*)Do) + 9,
    1);
}


/*!
@brief Double precision bksl_displacement_t multiplied with a primitive translation
along z axis

Computes the result of a general displacement, followed by a primitive
1-d translation along z axis.

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_z \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtz(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do) {

  bksl_mat3x3_copy(&(Di->R), &(Do->R));
  bksl_product_Rvz(&(Di->R), ti, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief single precision bksl_displacementf_t multiplied with a primitive translation
along z axis

Computes the result of a general displacement, followed by a primitive
1-d translation along z axis.

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_z \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtzf(
  bksl_displacementf_t* __restrict__ Di,
  float* __restrict__ ti,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_mat3x3f_copy(&(Di->R), &(Do->R));
  bksl_product_Rvzf(&(Di->R), ti, &(Do->t));
  cblas_saxpy(
    3,
    1.0,
    ((float*)Di) + 9,
    1,
    ((float*)Do) + 9,
    1);
}


/*!
@brief Double precision bksl_displacement_t, multiplied with the inverse of a
primitive translation along z axis

Computes the result of a general displacement, followed by the inverse of a
primitive 1-d translation along the z axis.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_z \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDtzinverse(
  bksl_displacement_t* __restrict__ Di,
  double* __restrict__ ti,
  bksl_displacement_t* __restrict__ Do) {

  bksl_mat3x3_copy(&(Di->R), &(Do->R));
  bksl_product_Rvzinverse(&(Di->R), ti, &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)Di) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief Double precision product of two general displacement matrices

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i} \rightarrow R_{o}\f$

\f$t_{1i} + R_{1i} * t_{2i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DD(
  bksl_displacement_t* __restrict__ D1i,
  bksl_displacement_t* __restrict__ D2i,
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RR(&(D1i->R), &(D2i->R), &(Do)->R);
  bksl_product_Rv(&(D1i->R), &(D2i->t), &(Do ->t));
  cblas_daxpy(3, 1.0, ((double*)D1i) + 9, 1, ((double*)Do) + 9, 1);
}


/*!
@brief single precision product of two general displacement matrices

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i} \rightarrow R_{o}\f$

\f$t_{1i} + R_{1i} * t_{2i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDf(
  bksl_displacementf_t* __restrict__ D1i,
  bksl_displacementf_t* __restrict__ D2i,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRf(&(D1i->R), &(D2i->R), &(Do)->R);
  bksl_product_Rvf(&(D1i->R), &(D2i->t), &(Do ->t));
  cblas_saxpy(3, 1.0, ((float*)D1i) + 9, 1, ((float*)Do) + 9, 1);
}


/*!
@brief Double precision product of q displacement matrix with a mat3x3 rotation
matrix

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i} \rightarrow R_{o}\f$

\f$t_{1i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDr(
  bksl_displacement_t* __restrict__ Di,
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RR(&Di->R, Ri, &Do->R);
  bksl_copy_vv(&Di->t, &Do->t);

  // cblas_dcopy(3, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}


/*!
@brief Double precision product of q displacement matrix with a mat3x3 rotation
matrix

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i} \rightarrow R_{o}\f$

\f$t_{1i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRf(&(Di->R), Ri, &(Do)->R);
  bksl_copy_vvf(&Di->t, &Do->t);

}


/*!
@brief Double precision product of q displacement matrix with a mat3x3 rotation
matrix

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i} \rightarrow R_{o}\f$

\f$t_{1i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_mat3x3_t* __restrict__ Ri,
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRinverse(&(Di->R), Ri, &(Do)->R);
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}


/*!
@brief Double precision product of q displacement matrix with a mat3x3 rotation
matrix

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i} \rightarrow R_{o}\f$

\f$t_{1i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDrinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_mat3x3f_t* __restrict__ Ri,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRinversef(&(Di->R), Ri, &(Do)->R);
  cblas_saxpy(3, 1.0, ((float*)Di) + 9, 1, ((float*)Do) + 9, 1);
}


/*!
@brief Double precision product of a first general displacement matrix and
the inverse of a second displacement matrix

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i}^{-1} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i}^T \rightarrow R_{o}\f$

\f$t_{1i} - R_{1i} * t_{2i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDinverse(
  bksl_displacement_t* __restrict__ D1i,
  bksl_displacement_t* __restrict__ D2i,
  bksl_displacement_t* __restrict__ Do) {

  bksl_product_RRinverse(&(D1i->R), &(D2i->R), &(Do->R));
  bksl_product_Rvinverse(&(Do->R), &(D2i->t), &(Do->t));
  cblas_daxpy(
    3,
    1.0,
    ((double*)D1i) + 9,
    1,
    ((double*)Do) + 9,
    1);
}


/*!
@brief single precision product of a first general displacement matrix and
the inverse of a second displacement matrix

Computes the product of two general displacement matrices consisting
of both a rotation and translation component

\f$D_{1i} * D_{2i}^{-1} \rightarrow D_{0}\f$

can be simplified as:

\f$R_{1i}*R_{2i}^T \rightarrow R_{o}\f$

\f$t_{1i} - R_{1i} * t_{2i} \rightarrow t_o\f$

@param D1i a first general displacement, \f$D_{1i}\f$
@param D2i a second general displacement, \f$D_{2i}\f$
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void bksl_product_DDinversef(
  bksl_displacementf_t* __restrict__ D1i,
  bksl_displacementf_t* __restrict__ D2i,
  bksl_displacementf_t* __restrict__ Do) {

  bksl_product_RRinversef(&(D1i->R), &(D2i->R), &(Do->R));
  bksl_product_Rvinversef(&(D1i->R), &(D2i->t), &(Do->t));
  cblas_saxpy(
    3,
    1.0,
    ((float*)D1i) + 9,
    1,
    ((float*)Do) + 9,
    1);
}


/*!
@}
@name Influence coeficient matrices utilities

Utilities for computing influence coefficient matrices

@{
*/

/*!
@brief Double precision influence coefficient matrix for a primitive
translation along x-axis

Computes the influence coefficient matrix for a translation along the x-axis.

\f$D[0,3,6] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is initially set to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htx(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  cblas_dcopy(3, (double*)Di, 1, (double*)ho, 1);
  //memset(ho + 3, 0, 3 * sizeof(double));
}


/*!
@brief single precision influence coefficient matrix for a primitive
translation along x-axis

Computes the influence coefficient matrix for a translation along the x-axis.

\f$D[0,3,6] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htxf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  cblas_scopy(3, (float*)Di, 1, (float*)ho, 1);
  //memset(ho + 3, 0, 3 * sizeof(double));
}


/*!
@brief Double precision influence coefficient matrix for a primitive
translation along y-axis

Computes the influence coefficient matrix for a translation along the y-axis.

\f$D[1,4,7] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_hty(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  cblas_dcopy(3, ((double*)Di) + 3, 1, (double*)ho, 1);
  //memset(ho + 3, 0, 3 * sizeof(double));
}


/*!
@brief single precision influence coefficient matrix for a primitive
translation along y-axis

Computes the influence coefficient matrix for a translation along the y-axis.

\f$D[1,4,7] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htyf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  cblas_scopy(3, ((float*)Di) + 3, 1, (float*)ho, 1);
  //memset(ho + 3, 0, 3 * sizeof(double));
}


/*!
@brief Double precision influence coefficient matrix for a primitive
translation along z-axis

Computes the influence coefficient matrix for a translation along the z-axis.

\f$D[2,5,8] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htz(
  bksl_displacement_t* __restrict__ ri,
  bksl_screw_t* __restrict__ ho) {

  cblas_dcopy(3, ((double*)ri) + 6, 1, (double*)ho, 1);
  //memset(ho + 3, 0, 3 * sizeof(double));
}


/*!
@brief single precision influence coefficient matrix for a primitive
translation along z-axis

Computes the influence coefficient matrix for a translation along the z-axis.

\f$D[2,5,8] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htzf(
  bksl_displacementf_t* __restrict__ ri,
  bksl_screwf_t* __restrict__ ho) {

  cblas_scopy(3, ((float*)ri) + 6, 1, (float*)ho, 1);
  //memset(ho + 3, 0, 3 * sizeof(double));
}


/*!
@brief Double precision influence coefficient matrix for a
primitive rotation about x-axis

Computes the influence coefficient matrix for a rotation about x-axis.

\f$D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hrx(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  bksl_htx(Di, (bksl_screw_t*)(((double*)ho) + 3));
  bksl_cross_vv(&(Di->t), (bksl_vec3_t*)(((double*)ho) + 3), (bksl_vec3_t*)ho);
}


/*!
@brief single precision influence coefficient matrix for a
primitive rotation about x-axis

Computes the influence coefficient matrix for a rotation about x-axis.

\f$D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hrxf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  bksl_htxf(Di, (bksl_screwf_t*)(((float*)ho) + 3));
  bksl_cross_vvf(&(Di->t), (bksl_vec3f_t*)(((float*)ho) + 3), (bksl_vec3f_t*)ho);
}


/*!
@brief Double precision influence coefficient matrix for a primitive
rotation about y-axis

Computes the influence coefficient matrix for a rotation about y-axis.

\f$D[1,4,7] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hry(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  bksl_hty(Di, (bksl_screw_t*)(((double*)ho) + 3));
  bksl_cross_vv(&(Di->t), (bksl_vec3_t*)(((double*)ho) + 3), (bksl_vec3_t*)ho);
}


/*!
@brief Double precision influence coefficient matrix for a primitive rotation
about z-axis

Computes the influence coefficient matrix for a rotation about z-axis.

\f$D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hrz(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  bksl_htz(Di, (bksl_screw_t*)(((double*)ho) + 3));
  bksl_cross_vv(&(Di->t), (bksl_vec3_t*)(((double*)ho) + 3), (bksl_vec3_t*)ho);
}


/*!
@brief single precision influence coefficient matrix for a primitive rotation
about z-axis

Computes the influence coefficient matrix for a rotation about z-axis.

\f$D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hrzf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  bksl_htzf(Di, (bksl_screwf_t*)(((float*)ho) + 3));
  bksl_cross_vvf(&(Di->t), (bksl_vec3f_t*)(((float*)ho) + 3), (bksl_vec3f_t*)ho);
}


/*!
@brief Double precision influence coefficient matrix for inverse primitive translation along x-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the x-axis.

\f$-D[0,3,6] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htxinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  memset(ho, 0, 3 * sizeof(double));
  cblas_daxpy(3, -1, (double*)Di, 1, (double*)ho, 1);
}


/*!
@brief single precision influence coefficient matrix for inverse primitive
translation along x-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the x-axis.

\f$-D[0,3,6] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void bksl_htxinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  memset(ho, 0, 3 * sizeof(float));
  cblas_saxpy(3, -1, (float*)Di, 1, (float*)ho, 1);
}


/*!
@brief Double precision influence coefficient matrix for inverse primitive
translation along y-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the y-axis.

\f$-D[1,4,7] \rightarrow h_{ty}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{ty}[3:5]\f$
*/
inline void bksl_htyinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  memset(ho, 0, 3 * sizeof(double));
  cblas_daxpy(3, -1, ((double*)Di) + 3, 1, (double*)ho, 1);
}


/*!
@brief single precision influence coefficient matrix for inverse primitive
translation along y-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the y-axis.

\f$-D[1,4,7] \rightarrow h_{ty}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{ty}[3:5]\f$
*/
inline void bksl_htyinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  memset(ho, 0, 3 * sizeof(float));
  cblas_saxpy(3, -1, ((float*)Di) + 3, 1, (float*)ho, 1);
}


/*!
@brief Double precision influence coefficient matrix for inverse primitive
translation along z-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the z-axis.

\f$-D[1,4,7] \rightarrow h_{tz}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tz}[3:5]\f$
*/
inline void bksl_htzinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {


  // ho[0] = -ri[0 * 3 + 2];
  // ho[1] = -ri[1 * 3 + 2];
  // ho[2] = -ri[2 * 3 + 2];

  for(uint32_t i = 0; i < 3; i++) {
    bksl_screw_set(ho, i, -bksl_mat3x3_get(&Di->R, i, 2));
  }

  /* previously was */
  // memset(ho, 0, 3 * sizeof(double));
  // cblas_daxpy(3, -1, ((double*)Di) + 6, 1, (double*)ho, 1);
}


/*!
@brief single precision influence coefficient matrix for inverse primitive
translation along z-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the z-axis.

\f$-D[1,4,7] \rightarrow h_{tz}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tz}[3:5]\f$
*/
inline void bksl_htzinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  memset(ho, 0, 3 * sizeof(float));
  cblas_saxpy(3, -1, ((float*)Di) + 6, 1, (float*)ho, 1);
  // ho[0] = -ri[0 * 3 + 2];
  // ho[1] = -ri[1 * 3 + 2];
  // ho[2] = -ri[2 * 3 + 2];
}


/*!
@brief Double precision influence coefficient matrix for inverse of a primitive
rotation about x-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about x-axis.

\f$-D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hrxinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  bksl_htxinverse(Di, (bksl_screw_t*)(((double*)ho) + 3));
  bksl_cross_vv(&(Di->t), (bksl_vec3_t*)(((double*)ho) + 3), (bksl_vec3_t*)ho);
}


/*!
@brief single precision influence coefficient matrix for inverse of a primitive
rotation about x-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about x-axis.

\f$-D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void bksl_hrxinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  bksl_htxinversef(Di, (bksl_screwf_t*)(((float*)ho) + 3));
  bksl_cross_vvf(&(Di->t), (bksl_vec3f_t*)(((float*)ho) + 3), (bksl_vec3f_t*)ho);
}


/*!
@brief Double precision influence coefficient matrix for inverse of a primitive
rotation about y-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about y-axis.

\f$-D[1,4,7] \rightarrow h_{ty}[3:5]\f$

\f$T_i \times h_{ty}[3:5] \rightarrow h_{ry}[0:2]\f$
*/
inline void bksl_hryinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  bksl_htyinverse(Di, (bksl_screw_t*)((double*)ho + 3));
  bksl_cross_vv(
    &(Di->t),
    (bksl_vec3_t*)(((double*)ho) + 3),
    (bksl_vec3_t*)ho);
}


/*!
@brief single precision influence coefficient matrix for inverse of a primitive
rotation about y-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about y-axis.

\f$-D[1,4,7] \rightarrow h_{ty}[3:5]\f$

\f$T_i \times h_{ty}[3:5] \rightarrow h_{ry}[0:2]\f$
*/
inline void bksl_hryinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  bksl_htyinversef(Di, (bksl_screwf_t*)(((float*)ho) + 3));
  bksl_cross_vvf(&(Di->t), (bksl_vec3f_t*)(((float*)ho) + 3), (bksl_vec3f_t*)ho);
}


/*!
@brief Double precision influence coefficient matrix for inverse of a
primitive rotation about z-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about z-axis.

\f$-D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tz}[3:5] \rightarrow h_{rz}[0:2]\f$
*/
inline void bksl_hrzinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ ho) {

  bksl_htzinverse(Di, (bksl_screw_t*)(((double*)ho) + 3));
  bksl_cross_vv(&(Di->t), (bksl_vec3_t*)(((double*)ho) + 3), (bksl_vec3_t*)ho);
}


/*!
@brief single precision influence coefficient matrix for inverse of a
primitive rotation about z-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about z-axis.

\f$-D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tz}[3:5] \rightarrow h_{rz}[0:2]\f$
*/
inline void bksl_hrzinversef(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ ho) {

  bksl_htzinversef(Di, (bksl_screwf_t*)(((float*)ho) + 3));
  bksl_cross_vvf(&(Di->t), (bksl_vec3f_t*)(((float*)ho) + 3), (bksl_vec3f_t*)ho);
}


/*!
@}
@name Utilities for transforming quantities in configuration space
Quantities in configuration space include spatial quantities that can be
be measured with a ruler or clock. bksl_screw_t quantities in configuration space
are often referred to as screws/twists.
@{
*/


/*!
@brief Compute double precision bksl_screw_t Cross product in configuration space

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$S_{1i}[0:2] \times S_{2i}[0:2] + S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void bksl_cross_ss(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so) {

  /* vo <- w1i x v2i */
  bksl_cross_vv(&s1i->w, &s2i->v, &so->v);

  /* wo <- w1i x w2i */
  bksl_cross_vv(&s1i->w, &s2i->w, &so->w);

  /* vo += v1i x w2i */
  so->v.x += s1i->v.y * s2i->w.z - s1i->v.z * s2i->w.y;
  so->v.y += s1i->v.z * s2i->w.x - s1i->v.x * s2i->w.z;
  so->v.z += s1i->v.x * s2i->w.y - s1i->v.y * s2i->w.x;
}


/*!
@brief Compute single precision bksl_screwf_t Cross product in configuration space

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$S_{1i}[0:2] \times S_{2i}[0:2] + S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void bksl_cross_ssf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so) {

  /* @TODO change to follow convention above to avoid multiple casts,
  much cleaner! */

  /* vo <- w1i x v2i */
  bksl_cross_vvf(
    (bksl_vec3f_t*)(((float*)s1i) + 3),
    (bksl_vec3f_t*)s2i,
    (bksl_vec3f_t*)(((float*)so)));

  /* wo <- w1i x w2i */
  bksl_cross_vvf(
    (bksl_vec3f_t*)(((float*)s1i) + 3),
    (bksl_vec3f_t*)(((float*)s2i) + 3),
    (bksl_vec3f_t*)(((float*)so) + 3));

  /* vo += v1i x w2i */
  so->v.x += s1i->v.y * s2i->w.z - s1i->v.z * s2i->w.y;
  so->v.y += s1i->v.z * s2i->w.x - s1i->v.x * s2i->w.z;
  so->v.z += s1i->v.x * s2i->w.y - s1i->v.y * s2i->w.x;
}


/*!
@brief Compute a double precision bksl_screw_t Cross product in configuration space
Compute bksl_screw_t Cross product in configuration space, where
the second screw consists of only a translation component.

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$0 \rightarrow S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void bksl_cross_sst(
  bksl_screw_t* __restrict__ s1i,
  bksl_screw_t* __restrict__ s2i,
  bksl_screw_t* __restrict__ so) {

  /* @TODO change to follow convention above to avoid multiple casts,
  much cleaner! */


  bksl_cross_vv(
    (bksl_vec3_t*)(((double*)s1i) + 3),
    (bksl_vec3_t*)s2i,
    (bksl_vec3_t*)so);
  memset(((double*)so) + 3, 0, 3 * sizeof(double));
}

/*!
@brief Compute a single precision bksl_screwf_t Cross product in configuration space
Compute bksl_screw_t Cross product in configuration space, where
the second screw consists of only a translation component.

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$0 \rightarrow S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void bksl_cross_sstf(
  bksl_screwf_t* __restrict__ s1i,
  bksl_screwf_t* __restrict__ s2i,
  bksl_screwf_t* __restrict__ so) {


  /* @TODO change to follow convention above to avoid multiple casts,
  much cleaner! */

  bksl_cross_vvf(
    (bksl_vec3f_t*)(((float*)s1i) + 3),
    (bksl_vec3f_t*)s2i,
    (bksl_vec3f_t*)so);
  memset(((float*)so) + 3, 0, 3 * sizeof(float));
}


/*!
@brief Double precision translate a screw, si, in configuration space by ti
and store in so.

\f$S_i[0:2] + T_i \times S_i[3:5] \rightarrow S_o[0:2]\f$

\f$si[3:5] \rightarrow S_o[3:5]\f$

@param ti a general 3-d translation
@param si a screw to translate
@param so the results are stored in so
*/
inline void bksl_product_Adts(
  bksl_vec3_t* __restrict__ ti,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_copy_ss(si, so);

  /* vo += ti x w2i */
  so->v.x += ti->y * si->w.z - ti->z * si->w.y;
  so->v.y += ti->z * si->w.x - ti->x * si->w.z;
  so->v.z += ti->x * si->w.y - ti->y * si->w.x;
}


/*!
@brief single precision translate a screw, si, in configuration space by ti
and store in so.

\f$S_i[0:2] + T_i \times S_i[3:5] \rightarrow S_o[0:2]\f$

\f$si[3:5] \rightarrow S_o[3:5]\f$

@param ti a general 3-d translation
@param si a screw to translate
@param so the results are stored in so
*/
inline void bksl_product_Adtsf(
  bksl_vec3f_t* __restrict__ ti,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_copy_ssf(si, so);

  /* vo += ti x w2i */
  so->v.x += ti->y * si->w.z - ti->z * si->w.y;
  so->v.y += ti->z * si->w.x - ti->x * si->w.z;
  so->v.z += ti->x * si->w.y - ti->y * si->w.x;
}


/*!
@brief In configuration space, double precision translate screw si by ti^-1
and store in so.

\f$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]\f$

\f$si[3:5] \rightarrow S_o[3:5]\f$

@param ti a general 3-d translation
@param si a screw to translate
@param so the results are stored in so
*/
inline void bksl_product_Adtinverses(
  bksl_vec3_t* __restrict__ ti,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_copy_ss(si, so);

  /* vo -= ti x w2i */
  so->v.x -= ti->y * si->w.z - ti->z * si->w.y;
  so->v.y -= ti->z * si->w.x - ti->x * si->w.z;
  so->v.z -= ti->x * si->w.y - ti->y * si->w.x;
}


/*!
@brief In configuration space, single precision translate screw si by ti^1
and store in so.

\f$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]\f$

\f$si[3:5] \rightarrow S_o[3:5]\f$

@param ti a general 3-d translation
@param si a screw to translate
@param so the results are stored in so
*/
inline void bksl_product_Adtinversesf(
  bksl_vec3f_t* __restrict__ ti,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_copy_ssf(si, so);

  /* vo -= ti x w2i */
  so->v.x -= ti->y * si->w.z - ti->z * si->w.y;
  so->v.y -= ti->z * si->w.x - ti->x * si->w.z;
  so->v.z -= ti->x * si->w.y - ti->y * si->w.x;
}


/*!
@brief In configuration space, double precision translate screw si by ti^-1
and store in so.

\f$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]\f$

\f$si[3:5] \rightarrow S_o[3:5]\f$

@param ti a general 3-d translation
@param si a screw to translate
@param so the results are stored in so
*/
inline void bksl_product_Adtsinverse(
  bksl_vec3_t* __restrict__ ti,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_invert_ss(si, so);

  /* vo -= ti x w2i */
  so->v.x -= ti->y * si->w.z - ti->z * si->w.y;
  so->v.y -= ti->z * si->w.x - ti->x * si->w.z;
  so->v.z -= ti->x * si->w.y - ti->y * si->w.x;
}

/*!
@brief In configuration space, single precision translate screw si by ti^1
and store in so.

\f$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]\f$

\f$si[3:5] \rightarrow S_o[3:5]\f$

@param ti a general 3-d translation
@param si a screw to translate
@param so the results are stored in so
*/
inline void bksl_product_Adtsinversef(
  bksl_vec3f_t* __restrict__ ti,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_invert_ssf(si, so);

  /* vo -= ti x w2i */
  so->v.x -= ti->y * si->w.z - ti->z * si->w.y;
  so->v.y -= ti->z * si->w.x - ti->x * si->w.z;
  so->v.z -= ti->x * si->w.y - ti->y * si->w.x;
}

/*!
@brief Double precision displace screw si by general displacement
and store in so.

\f$Ad(R, t) * \underline{s}_i \rightarrow s_o\f$

@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void bksl_product_Ads(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_product_Adrs(&(Di->R), si, so);

  so->v.x += Di->t.y * so->w.z - Di->t.z * so->w.y;
  so->v.y += Di->t.z * so->w.x - Di->t.x * so->w.z;
  so->v.z += Di->t.x * so->w.y - Di->t.y * so->w.x;

}

/*!
@brief Double precision displace negative screw si by general displacement
and store in so.

\f$D_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void bksl_product_Adsinverse(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_product_Adrsinverse(&(Di->R), si, so);
  so->v.x += Di->t.y * so->w.z - Di->t.z * so->w.y;
  so->v.y += Di->t.z * so->w.x - Di->t.x * so->w.z;
  so->v.z += Di->t.x * so->w.y - Di->t.y * so->w.x;

}


/*!
@brief single precision displace screw si by general displacement
and store in Do.

\f$D_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void bksl_product_Adsf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_product_Adrsf(&(Di->R), si, so);

  so->v.x += Di->t.y * so->w.z - Di->t.z * so->w.y;
  so->v.y += Di->t.z * so->w.x - Di->t.x * so->w.z;
  so->v.z += Di->t.x * so->w.y - Di->t.y * so->w.x;
}

/*!
@brief In configuration space, double precision displace screw si by ti^-1
& ri^-1 and store in so.

\f$D_i^{-1} * S_i \rightarrow S_o\f$
@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void bksl_product_Adinverses(
  bksl_displacement_t* __restrict__ Di,
  bksl_screw_t* __restrict__ si,
  bksl_screw_t* __restrict__ so) {

  bksl_screw_t temp;
  bksl_copy_ss(si, &temp);

  temp.v.x -= Di->t.y * temp.w.z - Di->t.z * temp.w.y;
  temp.v.y -= Di->t.z * temp.w.x - Di->t.x * temp.w.z;
  temp.v.z -= Di->t.x * temp.w.y - Di->t.y * temp.w.x;

  bksl_product_Adrinverses(&Di->R, &temp, so);

}

/*!
@brief In configuration space, single precision displace screw si by ti^-1
& ri^-1 and store in so.

\f$D_i^{-1} * S_i \rightarrow S_o\f$
@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void bksl_product_Adinversesf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_screwf_t temp;
  bksl_copy_ssf(si, &temp);

  temp.v.x -= Di->t.y * temp.w.z - Di->t.z * temp.w.y;
  temp.v.y -= Di->t.z * temp.w.x - Di->t.x * temp.w.z;
  temp.v.z -= Di->t.x * temp.w.y - Di->t.y * temp.w.x;

  bksl_product_Adrinversesf(&Di->R, &temp, so);
}

/*!
@}
@name Utilites for transforming quantities in function space
Quantities in function space include spatial quantities that are linear
operators on quantities in configuration space. These
quantities are often referred to as coscrews or wrenches.
@{
*/

/*!
@brief In function space, store spatial cross product of s1i with s2i in so.

\f$ S_{1i}[3:5]\times S_{2i}[0:2] + S_{1i}[0:2] \times S_{2i}[3:5] \rightarrow S_o[0:2]\f$

\f$ S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]\f$

@param s1i a first coscrew
@param s2i a second coscrew
@param so the results are stored in so
*/
inline void bksl_cross_cc(
  bksl_coscrew_t* __restrict__ s1i,
  bksl_coscrew_t* __restrict__ s2i,
  bksl_coscrew_t* __restrict__ so) {

  /* @TODO restrict for what types??  */
  // /* vo <- w1 x v2 */
  // bksl_cross_vv(
  //   (bksl_vec3_t*)(((double*)s1i) + 3),
  //   (bksl_vec3_t*)s2i,
  //   (bksl_vec3_t*)so);
  //
  // /* wo <- v1 x v2 */
  // bksl_cross_vv(
  //   (bksl_vec3_t*)s1i,
  //   (bksl_vec3_t*)((double*)s2i),
  //   (bksl_vec3_t*)(((double*)so) + 3));
  //
  // /* wo += w1 x w2 */
  // so->w.x += s1i->w.y * s2i->w.z - s1i->w.z * s2i->w.y;
  // so->w.y += s1i->w.z * s2i->w.x - s1i->w.x * s2i->w.z;
  // so->w.z += s1i->w.x * s2i->w.y - s1i->w.y * s2i->w.x;
}

/*!
@brief In function space, store single precision spatial cross product of s1i
with s2i in so.

\f$ S_{1i}[3:5]\times S_{2i}[0:2] + S_{1i}[0:2] \times S_{2i}[3:5] \rightarrow S_o[0:2]\f$

\f$ S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]\f$

@param s1i a first coscrew
@param s2i a second coscrew
@param so the results are stored in so
*/
inline void bksl_cross_ccf(
  bksl_coscrewf_t* __restrict__ s1i,
  bksl_coscrewf_t* __restrict__ s2i,
  bksl_coscrewf_t* __restrict__ so) {


  /* @TODO restrict for what types */

  // /* vo <- w1 x v2 */
  // bksl_cross_vvf(
  //   (bksl_vec3f_t*)(((float*)s1i) + 3),
  //   (bksl_vec3f_t*)s2i,
  //   (bksl_vec3f_t*)so);
  //
  // /* wo <- v1 x v2 */
  // bksl_cross_vvf(
  //   (bksl_vec3f_t*)s1i,
  //   (bksl_vec3f_t*)s2i,
  //   (bksl_vec3f_t*)(((float*)so) + 3));
  //
  // /* wo += w1 x w2 */
  // so->w.x += s1i->w.y * s2i->w.z - s1i->w.z * s2i->w.y;
  // so->w.y += s1i->w.z * s2i->w.x - s1i->w.x * s2i->w.z;
  // so->w.z += s1i->w.x * s2i->w.y - s1i->w.y * s2i->w.x;
}


/*!
@brief In function space displace spatial vector si by ti^T and store in so.

\f$S_i[3:5] - T_i \times S_i[0:2] \rightarrow S_o[3:5] \f$

\f$S_i[0:2] \rightarrow S_o[0:2]\f$

@param ti a 3-d translation
@param si a coscrew
@param so the results are stored in so
*/
inline void bksl_product_CoAdtc(
  bksl_vec3_t* __restrict__ ti,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  bksl_copy_cc(ci, co);

  /* wo += w1 x w2 */
  co->m.x += ti->y * ci->f.z - ti->z * ci->f.y;
  co->m.y += ti->z * ci->f.x - ti->x * ci->f.z;
  co->m.z += ti->x * ci->f.y - ti->y * ci->f.x;

}

/*!
@brief In function space displace single precision spatial vector si by
ti^T and store in so.

\f$S_i[3:5] - T_i \times S_i[0:2] \rightarrow S_o[3:5] \f$

\f$S_i[0:2] \rightarrow S_o[0:2]\f$

@param ti a 3-d translation
@param si a coscrew
@param so the results are stored in so
*/
inline void bksl_product_CoAdtcf(
  bksl_vec3f_t* __restrict__ ti,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  bksl_copy_ccf(ci, co);

  /* wo += w1 x w2 */
  co->m.x += ti->y * ci->f.z - ti->z * ci->f.y;
  co->m.y += ti->z * ci->f.x - ti->x * ci->f.z;
  co->m.z += ti->x * ci->f.y - ti->y * ci->f.x;
}

/*!
@brief In function space displace spatial vector si by ti^-T and store in so.

\f$ S_i[3:5] + T_i \times S_i[0:2] \rightarrow S_o[3:5]\f$

\f$ S_i[0:2] \rightarrow S_o[0:2] \f$
*/
inline void bksl_product_CoAdtcinverse(
  bksl_vec3_t* __restrict__ ti,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  bksl_coscrew_t temp;
  bksl_invert_cc(ci, &temp);
  bksl_product_CoAdtc(ti, &temp, co);

  /* wo -= w1 x w2 */
  // so->w.x -= ti->y * si->v.z - ti->z * si->v.y;
  // so->w.y -= ti->z * si->v.x - ti->x * si->v.z;
  // so->w.z -= ti->x * si->v.y - ti->y * si->v.x;
}

/*!
@brief In function space displace single precision spatial vector si by ti^-T
and store in so.

\f$ S_i[3:5] + T_i \times S_i[0:2] \rightarrow S_o[3:5]\f$

\f$ S_i[0:2] \rightarrow S_o[0:2] \f$
*/
inline void bksl_product_CoAdtcinversef(
  bksl_vec3f_t* __restrict__ ti,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  bksl_coscrewf_t temp;
  bksl_invert_ccf(ci, &temp);
  bksl_product_CoAdtcf(ti, &temp, co);

  /* previously was */
  // bksl_copy_ssf(si, so);
  //
  // /* wo -= w1 x w2 */
  // so->w.x -= ti->y * si->v.z - ti->z * si->v.y;
  // so->w.y -= ti->z * si->v.x - ti->x * si->v.z;
  // so->w.z -= ti->x * si->v.y - ti->y * si->v.x;
}

/*!
@brief In function space displace spatial vector si by ti^T and ri^-1 and store in so.

\f$ R_i^{-1} * T_i^T * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void bksl_product_CoAdc(
  bksl_displacement_t* __restrict__ Di,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  bksl_product_CoAdrc(&(Di->R), ci, co);

  co->m.x += Di->t.y * co->f.z - Di->t.z * co->f.y;
  co->m.y += Di->t.z * co->f.x - Di->t.x * co->f.z;
  co->m.z += Di->t.x * co->f.y - Di->t.y * co->f.x;

}

/*!
@brief In function space displace single precision spatial vector si by ti^T
and ri^-1 and store in so.

\f$ R_i^{-1} * T_i^T * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void bksl_product_CoAdcf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_coscrewf_t* __restrict__ ci,
  bksl_coscrewf_t* __restrict__ co) {

  bksl_product_CoAdrcf(&(Di->R), ci, co);

  co->m.x += Di->t.y * co->f.z - Di->t.z * co->f.y;
  co->m.y += Di->t.z * co->f.x - Di->t.x * co->f.z;
  co->m.z += Di->t.x * co->f.y - Di->t.y * co->f.x;
}

/*!
@brief In function space, displace spatial vector si by ri and ti^-T and store
in so.

\f$ T_i^{-T} * R_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void bksl_product_CoAdinversec(
  bksl_displacement_t* __restrict__ Di,
  bksl_coscrew_t* __restrict__ ci,
  bksl_coscrew_t* __restrict__ co) {

  bksl_coscrew_t temp;
  bksl_copy_cc(ci, &temp);

  temp.m.x -= Di->t.y * temp.f.z - Di->t.z * temp.f.y;
  temp.m.y -= Di->t.z * temp.f.x - Di->t.x * temp.f.z;
  temp.m.z -= Di->t.x * temp.f.y - Di->t.y * temp.f.x;

  bksl_product_CoAdrinversec(&(Di->R), &temp, co);

}

/*!
@brief In function space, displace single precision spatial vector si by ri
and ti^-T and store in so.

\f$ T_i^{-T} * R_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void bksl_coscrewproduct_dinversesf(
  bksl_displacementf_t* __restrict__ Di,
  bksl_screwf_t* __restrict__ si,
  bksl_screwf_t* __restrict__ so) {

  bksl_screwf_t temp;
  bksl_copy_ssf(si, &temp);

  temp.w.x -= Di->t.y * temp.v.z - Di->t.z * temp.v.y;
  temp.w.y -= Di->t.z * temp.v.x - Di->t.x * temp.v.z;
  temp.w.z -= Di->t.x * temp.v.y - Di->t.y * temp.v.x;

  bksl_product_Adrinversesf(&(Di->R), &temp, so);
}

/*!
@}
@name Quaternion utilities
Utilities to convert to and from matrix/quaternion representation
@{
*/

/*!
@brief Normalize a double precision quaternion

Normalize a quaternion and overwrite.

@param qi euler parameters to normalize
*/
void bksl_quaternion_normalize(
  bksl_quaternion_t* __restrict__ qi) {

  double norm = sqrt(cblas_ddot(4, (double*)qi, 1, (double*)qi, 1));
  cblas_dscal(4, 1.0 / norm, (double*)qi, 1);
}

/*!
@brief Normalize single precision quaternion

Normalize a quaternion and overwrite.

@param qi euler parameters to normalize
*/
void bksl_quaternionf_normalize(
  bksl_quaternionf_t* __restrict__ qi) {

  float norm = sqrt(cblas_sdot(4, (float*)qi, 1, (float*)qi, 1));
  cblas_sscal(4, 1.0 / norm, (float*)qi, 1);
}

/*!
@brief Double precision function to convert bksl_mat3x3_t rotation matrix to quaternion
representation

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\
\end{bmatrix}\f$

@todo Supply *ER*, a reference Euler Parameter. If necessary, flip the sign of *(E) so that each *(E+i) is the closest match to ER[i].
For this algorithm to work, *(E) must be a close follow-on to ER.
This function would then prove more useful when continuous Euler
Parameters are desired for uint32_terpolation purposes. Matrix R
is quadratic in *(E) so *(E+i) and -*(E+i) give the same R.

@param ri a rotation matrix in a 9x1 column format
@param qo the resulting euler parameters are stored in the 4x1 array, qo
*/
void bksl_mat3x3_toQuaternion(
  bksl_mat3x3_t* __restrict__ ri,
  bksl_quaternion_t* __restrict__ qo) {

  if(fabs(ri->m00) > 1e-9 || fabs(ri->m11) > 1e-9 || fabs(ri->m22) > 1e-9) {
    double p;
    // p = 4e0^2-1 > 0 or e0^2 > 1/4 or |e0| > 1/2
    if((p = ri->m00 + ri->m11 + ri->m22) > 0) {

      // Now p = 2|e0| > 1
      p = sqrt(p + 1);

      // If (2|e0| - 2er0) > 1, then p = 2|e0| has the wrong sign.
      // if ( p - ER[0] - ER[0] > 1 ) p = -p; // 2e0

      qo->w = p / 2; // e0
      p = p + p;     // 4e0

      qo->x =
        (((double*)ri)[2 + 1 * 3] - ((double*)ri)[1 + 2 * 3]) / p; // e1 = 4e0e1/4e0
      qo->y =
        (((double*)ri)[0 + 2 * 3] - ((double*)ri)[2 + 0 * 3]) / p; // e2 = 4e0e2/4e0
      qo->z =
        (((double*)ri)[1 + 0 * 3] - ((double*)ri)[0 + 1 * 3]) / p; // e3 = 4e0e3/4e0
    }

    // 4e1^2-1 > 0 or e1^2 > 1/4 or |e1| > 1/2
    else if((p = ri->m00 - ri->m11 - ri->m22) > 0) {
      // Now p = 2|e1| > 1
      p = sqrt(p + 1);

      // If (2|e1| - 2er1) > 1, then p = 2|e1| has the wrong sign.
      // if ( p - ER[1] - ER[1] > 1 ) p = -p; // 2e1

      qo->x = p / 2; // e1
      p = p + p;     // 4e1

      qo->w = (((double*)ri)[2 + 1 * 3] - ((double*)ri)[1 + 2 * 3]) / p; // e0 = 4e0e1/4e1
      qo->y = (((double*)ri)[1 + 0 * 3] + ((double*)ri)[0 + 1 * 3]) / p; // e2 = 4e1e2/4e1
      qo->z = (((double*)ri)[2 + 0 * 3] + ((double*)ri)[0 + 2 * 3]) / p; // e3 = 4e3e1/4e1
    }

    // 4e2^2-1 > 0 or e2^2 > 1/4 or |e2| > 1/2
    else if((p = ri->m11 - ri->m00 - ri->m22) > 0) {
      // Now p = 2|e2| > 1
      p = sqrt(p + 1);

      // If (2|e2| - 2er2) > 1, then p = 2|e2| has the wrong sign.
      // if ( p - ER[2] - ER[2] > 1 ) p = -p; // 2e2

      qo->y = p / 2; // e2
      p = p + p;     // 4e2

      qo->w = (((double*)ri)[0 + 2 * 3] - ((double*)ri)[2 + 0 * 3]) / p; // e0 = 4e0e2/4e2
      qo->x = (((double*)ri)[0 + 1 * 3] + ((double*)ri)[1 + 0 * 3]) / p; // e1 = 4e1e2/4e2
      qo->z = (((double*)ri)[1 + 2 * 3] + ((double*)ri)[2 + 1 * 3]) / p; // e3 = 4e2e3/4e2
    }

    // 4e3^2-1 > 0 or e3^2 > 1/4 or |e3| > 1/2
    else {
      p = ri->m22 - ri->m00 - ri->m11;

      // Now p = 2|e3| > 1
      p = sqrt(p + 1);

      // If (2|e3| - 2er3) > 1, then p = 2|e3| has the wrong sign.
      // if ( p - ER[3] - ER[3] > 1 ) p = -p; // 2e3

      qo->z = p / 2; // e3
      p = p + p;     // 4e3

      qo->w = (((double*)ri)[1 + 0 * 3] - ((double*)ri)[0 + 1 * 3]) / p; // e0 = 4e0e3/4e3
      qo->x = (((double*)ri)[0 + 2 * 3] + ((double*)ri)[2 + 0 * 3]) / p; // e1 = 4e3e1/4e3
      qo->y = (((double*)ri)[2 + 1 * 3] + ((double*)ri)[1 + 2 * 3]) / p; // e2 = 4e2e3/4e3
    }
  }

  else {
    qo->w = 0.5;
    qo->x = copysign(0.5, ((double*)ri)[2 + 1 * 3] - ((double*)ri)[1 + 2 * 3]);
    qo->y = copysign(0.5, ((double*)ri)[0 + 2 * 3] - ((double*)ri)[2 + 0 * 3]);
    qo->z = copysign(0.5, ((double*)ri)[1 + 0 * 3] - ((double*)ri)[0 + 1 * 3]);
  }
  return;
}


/*!
@brief single precision function to convert bksl_mat3x3_t rotation matrix to quaternion
representation

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\
\end{bmatrix}\f$

@todo Supply *ER*, a reference Euler Parameter. If necessary, flip the sign of *(E) so that each *(E+i) is the closest match to ER[i].
For this algorithm to work, *(E) must be a close follow-on to ER.
This function would then prove more useful when continuous Euler
Parameters are desired for uint32_terpolation purposes. Matrix R
is quadratic in *(E) so *(E+i) and -*(E+i) give the same R.

@param ri a rotation matrix in a 9x1 column format
@param qo the resulting euler parameters are stored in the 4x1 array, qo
*/
void bksl_mat3x3f_toQuaternion(
  bksl_mat3x3f_t* __restrict__ ri,
  bksl_quaternionf_t* __restrict__ qo) {

  if(fabs(ri->m00) > 1e-9 || fabs(ri->m11) > 1e-9 || fabs(ri->m22) > 1e-9) {
    double p;
    // p = 4e0^2-1 > 0 or e0^2 > 1/4 or |e0| > 1/2
    if((p = ri->m00 + ri->m11 + ri->m22) > 0) {

      // Now p = 2|e0| > 1
      p = sqrt(p + 1);

      // If (2|e0| - 2er0) > 1, then p = 2|e0| has the wrong sign.
      // if ( p - ER[0] - ER[0] > 1 ) p = -p; // 2e0

      qo->w = p / 2; // e0
      p = p + p;     // 4e0

      qo->x =
        (((float*)ri)[2 + 1 * 3] - ((float*)ri)[1 + 2 * 3]) / p; // e1 = 4e0e1/4e0
      qo->y =
        (((float*)ri)[0 + 2 * 3] - ((float*)ri)[2 + 0 * 3]) / p; // e2 = 4e0e2/4e0
      qo->z =
        (((float*)ri)[1 + 0 * 3] - ((float*)ri)[0 + 1 * 3]) / p; // e3 = 4e0e3/4e0
    }

    // 4e1^2-1 > 0 or e1^2 > 1/4 or |e1| > 1/2
    else if((p = ri->m00 - ri->m11 - ri->m22) > 0) {
      // Now p = 2|e1| > 1
      p = sqrt(p + 1);

      // If (2|e1| - 2er1) > 1, then p = 2|e1| has the wrong sign.
      // if ( p - ER[1] - ER[1] > 1 ) p = -p; // 2e1

      qo->x = p / 2; // e1
      p = p + p;     // 4e1

      qo->w = (((float*)ri)[2 + 1 * 3] - ((float*)ri)[1 + 2 * 3]) / p; // e0 = 4e0e1/4e1
      qo->y = (((float*)ri)[1 + 0 * 3] + ((float*)ri)[0 + 1 * 3]) / p; // e2 = 4e1e2/4e1
      qo->z = (((float*)ri)[2 + 0 * 3] + ((float*)ri)[0 + 2 * 3]) / p; // e3 = 4e3e1/4e1
    }

    // 4e2^2-1 > 0 or e2^2 > 1/4 or |e2| > 1/2
    else if((p = ri->m11 - ri->m00 - ri->m22) > 0) {
      // Now p = 2|e2| > 1
      p = sqrt(p + 1);

      // If (2|e2| - 2er2) > 1, then p = 2|e2| has the wrong sign.
      // if ( p - ER[2] - ER[2] > 1 ) p = -p; // 2e2

      qo->y = p / 2; // e2
      p = p + p;     // 4e2

      qo->w = (((float*)ri)[0 + 2 * 3] - ((float*)ri)[2 + 0 * 3]) / p; // e0 = 4e0e2/4e2
      qo->x = (((float*)ri)[0 + 1 * 3] + ((float*)ri)[1 + 0 * 3]) / p; // e1 = 4e1e2/4e2
      qo->z = (((float*)ri)[1 + 2 * 3] + ((float*)ri)[2 + 1 * 3]) / p; // e3 = 4e2e3/4e2
    }

    // 4e3^2-1 > 0 or e3^2 > 1/4 or |e3| > 1/2
    else {
      p = ri->m22 - ri->m00 - ri->m11;

      // Now p = 2|e3| > 1
      p = sqrt(p + 1);

      // If (2|e3| - 2er3) > 1, then p = 2|e3| has the wrong sign.
      // if ( p - ER[3] - ER[3] > 1 ) p = -p; // 2e3

      qo->z = p / 2; // e3
      p = p + p;     // 4e3

      qo->w = (((float*)ri)[1 + 0 * 3] - ((float*)ri)[0 + 1 * 3]) / p; // e0 = 4e0e3/4e3
      qo->x = (((float*)ri)[0 + 2 * 3] + ((float*)ri)[2 + 0 * 3]) / p; // e1 = 4e3e1/4e3
      qo->y = (((float*)ri)[2 + 1 * 3] + ((float*)ri)[1 + 2 * 3]) / p; // e2 = 4e2e3/4e3
    }
  }

  else {
    qo->w = 0.5;
    qo->x = copysign(0.5, ((float*)ri)[2 + 1 * 3] - ((float*)ri)[1 + 2 * 3]);
    qo->y = copysign(0.5, ((float*)ri)[0 + 2 * 3] - ((float*)ri)[2 + 0 * 3]);
    qo->z = copysign(0.5, ((float*)ri)[1 + 0 * 3] - ((float*)ri)[0 + 1 * 3]);
  }
  return;
}


/*!
@brief Double precision function to convert quaternion to rotation
matrix representation

The following formula is used to perform conversion:

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\
\end{bmatrix}\f$

The Quaternion will be normalized during conversion.

@param qi [in] euler parameters to convert in a 4x1 array, in the form [w,x,y,z]
@param ro [out] the resulting rotation matrix is stored in ro
*/
void bksl_quaternion_toMat3x3(
  bksl_quaternion_t* __restrict__ qi,
  bksl_mat3x3_t* __restrict__ ro) {

  bksl_quaternion_normalize(qi);

  double w2 = qi->w * qi->w;
  double x2 = qi->x * qi->x;
  double y2 = qi->y * qi->y;
  double z2 = qi->z * qi->z;

  ro->m00 = (x2 - y2 - z2 + w2);
  ro->m11 = (-x2 + y2 - z2 + w2);
  ro->m22 = (-x2 - y2 + z2 + w2);

  double tmp1 = qi->x * qi->y;
  double tmp2 = qi->z * qi->w;
  ro->m10 = 2.0 * (tmp1 + tmp2);
  ro->m01 = 2.0 * (tmp1 - tmp2);

  tmp1 = qi->x * qi->z;
  tmp2 = qi->y * qi->w;
  ro->m20 = 2.0 * (tmp1 - tmp2);
  ro->m02 = 2.0 * (tmp1 + tmp2);

  tmp1 = qi->y * qi->z;
  tmp2 = qi->x * qi->w;
  ro->m21 = 2.0 * (tmp1 + tmp2);
  ro->m12 = 2.0 * (tmp1 - tmp2);
}

/*!
@brief single precision function to convert Quaternionf to rotation
matrix (bksl_mat3x3f_t) representation

The following formula is used to perform conversion:

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\
\end{bmatrix}\f$

The Quaternion will be normalized during conversion.

@param qi euler parameters to convert in a 4x1 array, in the form [w,x,y,z]
@param ro the resulting rotation matrix is stored in ro
*/
void bksl_quaternionf_toMat3x3f(
  bksl_quaternionf_t* __restrict__ qi,
  bksl_mat3x3f_t* __restrict__ ro) {

  bksl_quaternionf_normalize(qi);

  float w2 = qi->w * qi->w;
  float x2 = qi->x * qi->x;
  float y2 = qi->y * qi->y;
  float z2 = qi->z * qi->z;

  ro->m00 = (x2 - y2 - z2 + w2);
  ro->m11 = (-x2 + y2 - z2 + w2);
  ro->m22 = (-x2 - y2 + z2 + w2);

  float tmp1 = qi->x * qi->y;
  float tmp2 = qi->z * qi->w;
  ro->m10 = 2.0 * (tmp1 + tmp2);
  ro->m01 = 2.0 * (tmp1 - tmp2);

  tmp1 = qi->x * qi->z;
  tmp2 = qi->y * qi->w;
  ro->m20 = 2.0 * (tmp1 - tmp2);
  ro->m02 = 2.0 * (tmp1 + tmp2);

  tmp1 = qi->y * qi->z;
  tmp2 = qi->x * qi->w;
  ro->m21 = 2.0 * (tmp1 + tmp2);
  ro->m12 = 2.0 * (tmp1 - tmp2);
}

/*!
@brief Double precision, vectorized version of Shoemake's Spherical
Linear uint32_terpolation algorithm

Refer to [Shoemake, Animating rotation with Quaternion Curves, SIGGRAPH, vol 19, No. 3, 1985](http://run.usc.edu/cs520-s12/assign2/p245-shoemake.pdf) for
specific information on the algorithm.

@param q1i a first quaternion in the form [w,x,y,z] to uint32_terpolate from
@param q2i a second quaternion in the form [w,x,y,z] to uint32_terpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
void bksl_quaternion_slerp(
  bksl_quaternion_t* __restrict__ q1i,
  bksl_quaternion_t* __restrict__ q2i,
  double* __restrict__ t,
  bksl_quaternion_t* __restrict__ qo) {

  double cosHalfTheta = cblas_ddot(4, (double*)q1i, 1, (double*)q2i, 1);

  /*
    if q1i=q2i or q1i=-q2i then theta = 0 and we can return q1i
  */
  if(fabs(cosHalfTheta) >= 1.0) {
    cblas_dcopy(4, (double*)q1i, 1, (double*)qo, 1);
    return;
  }

  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
  /*
    if theta = 180 degrees then result is not defined
    we could rotate around any axis normal to q1i or q2i
  */
  memset(qo, 0, 4 * sizeof(double));
  if(fabs(sinHalfTheta) < 0.001) { // fabs is floating point absolute
    cblas_daxpy(4, 0.5, (double*)q1i, 1, (double*)qo, 1);
    cblas_daxpy(4, 0.5, (double*)q2i, 1, (double*)qo, 1);
    return;
  }
  double ratioA = sin((1 - *t) * halfTheta) / sinHalfTheta;
  double ratioB = sin(*t * halfTheta) / sinHalfTheta;

  /*
    calculate quaternion.
  */
  cblas_daxpy(4, ratioA, (double*)q1i, 1, (double*)qo, 1);
  cblas_daxpy(4, ratioB, (double*)q2i, 1, (double*)qo, 1);
}

/*!
@brief single precision, vectorized version of Shoemake's Spherical
Linear uint32_terpolation algorithm

Refer to [Shoemake, Animating rotation with Quaternion Curves, SIGGRAPH, vol 19, No. 3, 1985](http://run.usc.edu/cs520-s12/assign2/p245-shoemake.pdf) for
specific information on the algorithm.

@param q1i a first quaternion in the form [w,x,y,z] to uint32_terpolate from
@param q2i a second quaternion in the form [w,x,y,z] to uint32_terpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
void bksl_quaternionf_slerp(
  bksl_quaternionf_t* __restrict__ q1i,
  bksl_quaternionf_t* __restrict__ q2i,
  float* __restrict__ t,
  bksl_quaternionf_t* __restrict__ qo) {

  float cosHalfTheta = cblas_sdot(4, (float*)q1i, 1, (float*)q2i, 1);

  /*
    if q1i=q2i or q1i=-q2i then theta = 0 and we can return q1i
  */
  if(fabs(cosHalfTheta) >= 1.0) {
    cblas_scopy(4, (float*)q1i, 1, (float*)qo, 1);
    return;
  }
  float halfTheta = acos(cosHalfTheta);
  float sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);

  /*
    if theta = 180 degrees then result is not defined
    we could rotate around any axis normal to q1i or q2i
  */
  memset(qo, 0, 4 * sizeof(double));
  if(fabs(sinHalfTheta) < 0.001) { // fabs is floating point absolute
    cblas_saxpy(4, 0.5, (float*)q1i, 1, (float*)qo, 1);
    cblas_saxpy(4, 0.5, (float*)q2i, 1, (float*)qo, 1);
    return;
  }
  float ratioA = sin((1 - *t) * halfTheta) / sinHalfTheta;
  float ratioB = sin(*t * halfTheta) / sinHalfTheta;

  /*
    calculate quaternion.
  */
  cblas_saxpy(4, ratioA, (float*)q1i, 1, (float*)qo, 1);
  cblas_saxpy(4, ratioB, (float*)q2i, 1, (float*)qo, 1);
}


/*!
@brief Vectorized version of quaternion Linear uint32_terpolation (Lerp)

Less expensive numerically than Shoemake's Slerp algorithm (ksl_slerp)
but does not uint32_terpolate with constant velocity

@param q1i a first quaternion in the form [w,x,y,z] to uint32_terpolate from
@param q2i a second quaternion in the form [w,x,y,z] to uint32_terpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
void bksl_quaternion_nlerp(
  bksl_quaternion_t* __restrict__ q1i,
  bksl_quaternion_t* __restrict__ q2i,
  double* __restrict__ t,
  bksl_quaternion_t* __restrict__ qo) {

  if((*t) < 1e-9) {
    cblas_dcopy(4, (double*)q1i, 1, (double*)qo, 1);
    return;
  }
  if((*t) >= 1.0) {
    cblas_dcopy(4, (double*)q2i, 1, (double*)qo, 1);
    return;
  }
  double alpha = 1.0f - *t;
  double omega = *t;

  if(cblas_ddot(4, (double*)q1i, 1, (double*)q2i, 1) < 0) {
    omega = -omega;
  }
  /*
    simple uint32_terpolation doesn't yield quaternions of unit length,
    so must normalize before returning to user
  */
  memset(qo, 0, 4 * sizeof(double));
  cblas_daxpy(4, alpha, (double*)q1i, 1, (double*)qo, 1);
  cblas_daxpy(4, omega, (double*)q2i, 1, (double*)qo, 1);
  bksl_quaternion_normalize(qo);
}

/*!
@brief Vectorized version of quaternion Linear uint32_terpolation (Lerp)

Less expensive numerically than Shoemake's Slerp algorithm (ksl_Slerp)
but does not uint32_terpolate with constant velocity

@param q1i a first quaternion in the form [w,x,y,z] to uint32_terpolate from
@param q2i a second quaternion in the form [w,x,y,z] to uint32_terpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
void bksl_quaternionf_nlerp(
  bksl_quaternionf_t* __restrict__ q1i,
  bksl_quaternionf_t* __restrict__ q2i,
  float* __restrict__ t,
  bksl_quaternionf_t* __restrict__ qo) {

  if((*t) < 1e-9) {
    cblas_scopy(4, (float*)q1i, 1, (float*)qo, 1);
    return;
  }
  if((*t) >= 1.0) {
    cblas_scopy(4, (float*)q2i, 1, (float*)qo, 1);
    return;
  }
  float alpha = 1.0f - *t;
  float omega = *t;

  if(cblas_sdot(4, (float*)q1i, 1, (float*)q2i, 1) < 0) {
    omega = -omega;
  }
  memset(qo, 0, 4 * sizeof(float));
  cblas_saxpy(4, alpha, (float*)q1i, 1, (float*)qo, 1);
  cblas_saxpy(4, omega, (float*)q2i, 1, (float*)qo, 1);
  bksl_quaternionf_normalize(qo);
}

/*!
@}
@name Miscellaneous utilities
Utilities to for normalizing radians and degrees
@{
*/

/*!
@brief Normalize double precision angle in degrees

@param normalizes degrees to be between 0 and 360 degrees
*/
double bksl_normalizeDegrees(
  double angle) {

  while(angle < 0) {
    angle += 360;
  }
  while(angle > 360) {
    angle -= 360;
  }
  return angle;
}

/*!
@brief Normalize single precision angle in degrees

@param normalizes degrees to be between 0 and 360 degrees
*/
float bksl_normalizeDegreesf(
  float angle) {

  while(angle < 0) {
    angle += 360;
  }
  while(angle > 360) {
    angle -= 360;
  }
  return angle;
}

/*!
@brief Normalize angle in radians

@param normalizes double precision radians to be between 0 and 2 * PI radians
*/
double bksl_normalizeRadians(
  double angle) {

  while(angle < 0) {
    angle += (2 * M_PI);
  }
  while(angle > 2 * M_PI) {
    angle -= (2 * M_PI);
  }
  return angle;
}

/*!
@brief Normalize angle in radians

@param normalizes single precision radians to be between 0 and 2 * PI radians
*/
float bksl_normalizeRadiansf(
  float angle) {

  while(angle < 0) {
    angle += (2 * M_PI);
  }
  while(angle > 2 * M_PI) {
    angle -= (2 * M_PI);
  }
  return angle;
}

/*!
@brief returns a continuous angle obtained from atan2(y, x)

Caution: atan2(y,x) will return an angle with +-2m*pi accuracy only if the
sign on x and y are both correct. If x and y are both off by (-1) then
atan2 will return an angle with accuracy of only +-m*pi. If the sign of
x and y are uncertain, use the alternate function, catan1pi.

This function returns a continuous angle obtained from the standard math
atan2(y,x) function. On input, the argument theta contains the
previous value associated with the angle that is being computed.
catan2() adjusts the value returned from atan2(), so it will be as close to
theta as possible. The assumption is that the current value to be returned
by catan2() is not more than (+-)pi/4 from the previous value. This should
be a valid assumption, because angles generally don't change very fast over
the course of a simulation. Both input arguments x and y must have the
correct sign for catan2pi to work correctly.

*/
double catan2pi(
  double y,
  double x,
  double theta) {

  double temp;
  double a;

  /*
    temp holds the exact current angle, but it may be off by multiples of 2*pi. so it is brought closer to theta by adding multiples of 2*pi obtained from theta.
  */
  temp = atan2(y, x) + 2 * M_PI * floor(theta / 2 / M_PI);

  /*
    temp might still be a 2*pi increment above the desired angle.
    a is the upper limit for the current angle. Add QUARTER_PI to bracket
    the current angle.
  */
  a = theta + M_PI_4;

  /*
    Decrement the angle until it is below this upper limit if necessary.
  */
  while(temp > a) {
    temp -= (2 * M_PI);
  }

  /*
    Or temp may be a 2*pi increment below the desired angle.
    Now a is the lower limit for the current angle. Subtract QUARTER_PI
    to bracket the current angle.
  */
  a = theta - M_PI_4;

  // Increment the angle until it is above this lower limit if necessary.

  while(temp < a) {
    temp += (2 * M_PI);
  }

  /*
    Now the true angle is bracketed by theta - pi/4 and theta + pi/4.
  */
  return temp;
}


/*!
@brief returns a continuous angle from the atan2(y, x) function

Caution: atan2(y,x) will return an angle with +-2m*pi accuracy only if the
sign on x and y are both correct. If x and y are both off by (-1) then
atan2 will return an angle with accuracy of only +-m*pi. If the sign of
x and y are certain, this or the alternate function, catan2pi will work.
If the sign of x and y are uncertain, do not use catan2pi.
This function returns a continuous angle from the standard
math atan2(y,x) function. On input, the argument theta contains
the previous value associated with the angle that is being computed.
catan2() adjusts the value returned from atan2(), so it will be as close to
theta as possible. The assumption is that the current value to be returned
by catan2() is not more than (+-)pi/4 from the previous value. This should
be a valid assumption, because angles generally don't change very fast over
the course of a simulation. Both input arguments x and y must either
have the correct sign or be off by the same sign for catan1pia() to work
correctly.
*/
double catan1pi(
  double y,
  double x,
  double theta) {

  double temp;
  double a;

  /*
    temp holds the exact current angle, but it may be off by multiples of pi.
    so it is brought closer to theta by adding multiples of pi obtained from theta.
  */
  temp = atan2(y, x) + M_PI * floor(theta / M_PI);

  /*
    temp might still be one or more pi increments above the desired angle.
    a is the upper limit for the current angle. Add QUARTER_PI to bracket
    the current angle.
  */
  a = theta + M_PI_4;

  /*
    Decrement the angle until it is below this upper limit if necessary.
  */
  while(temp > a) {
    temp -= M_PI;
  }

  /*
    Or temp may be one or more pi increments below the desired angle.
    Now a is the lower limit for the current angle. Subtract QUARTER_PI
    to bracket the current angle.
  */
  a = theta - M_PI_4;

  /*
    Increment the angle until it is above this lower limit if necessary.
  */
  while(temp < a) {
    temp += M_PI;
  }

  /*
    Now the true angle is bracketed by theta - pi/4 and theta + pi/4.
  */
  return temp;
}

/*!
@brief extract Euler angles from a bksl_mat3x3_t rotation matrix

This function decomposes a general direction cosine matrix uint32_to three primitive
direction cosine matrices, whose axes are determined by the uint32_tegers passed
in through axis (see below). The direction cosine matrix is passed in through ri, and the three computed angles are updated and returned in angle. The previous values assigned to angle are passed in to RtoA in angle. RtoA calls catan2, which uses the values stored in angle to maintain continuity if at all possible.

This function does not check for bad axis indices. These checks could easily
be added. For actual reporting purposes, especially when a large
number of user output requests are to be processed, it would be more efficient
to program four or twelve different functions and have a preprocessor
generate pointers to the appropriate functions as needed. That way, no logical
tests would have to be performed at runtime, except in the function catan2().

If four functions were coded, each would handle one of the four Cases below.
In this case, the if statements would be eliminated, and only the axis indices
would be needed. If twelve functions were coded, each would handle one of the
twelve Cases. Again, the if statements would be eliminated, and the axis indices would be hard coded, eliminating the need to pass them in or to perform any other tests.

Used to determine if the next axis is forward '+' or reverse '-' sequence.
It also identifies index locations in the direction cosine matrix when
axes are repeated, such as in the sequence 1 --> 2 --> 1.

The function can be modified to decompose a matrix in any sequence of angles by
by changing "axis", but presently the sequence [rx=0, ry=1, rz=2] is hard coded
in to the function.

*/
void bbksl_mat3x3_getEulerAngles(
  bksl_mat3x3_t* __restrict__ rin,
  bksl_vec3_t* __restrict__ angles,
  bksl_axis_enum_t axisType) {

  uint32_t axis[3];
  switch(axisType) {
    case bksl_AXIS_XYZ: {
      axis[0] = 0;
      axis[1] = 1;
      axis[2] = 2;
    }
    break;
    case bksl_AXIS_YZX: {
      axis[0] = 1;
      axis[1] = 2;
      axis[2] = 0;
    }
    break;
    case bksl_AXIS_ZXY: {
      axis[0] = 2;
      axis[1] = 1;
      axis[2] = 2;
    }
    break;
    case bksl_AXIS_XZY: {
      axis[0] = 0;
      axis[1] = 2;
      axis[2] = 1;
    }
    break;
    case bksl_AXIS_YXZ: {
      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 2;
    }
    break;
    case bksl_AXIS_ZYX: {
      axis[0] = 2;
      axis[1] = 1;
      axis[2] = 0;
    }
    break;
    case bksl_AXIS_XYX: {
      axis[0] = 0;
      axis[1] = 1;
      axis[2] = 0;
    }
    break;
    case bksl_AXIS_XZX: {
      axis[0] = 0;
      axis[1] = 2;
      axis[2] = 0;
    }
    break;
    case bksl_AXIS_YXY: {
      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 1;
    }
    break;
    case bksl_AXIS_YZY: {
      axis[0] = 1;
      axis[1] = 2;
      axis[2] = 1;
    }
    break;
    case bksl_AXIS_ZXZ: {
      axis[0] = 2;
      axis[1] = 0;
      axis[2] = 2;
    }
    break;
    case bksl_AXIS_ZYZ: {
      axis[0] = 2;
      axis[1] = 1;
      axis[2] = 2;
    }
    break;
    default: {
      axis[0] = 0;
      axis[1] = 1;
      axis[2] = 2;
    }
  }

  const uint32_t f[3] = {1, 2, 0};

  /*
    Short notation for the axis identifiers.
  */
  uint32_t i = axis[0];
  uint32_t j = axis[1];
  uint32_t k = axis[2];

  /*
    The value of k will be changed in Cases 3 and 4.
    Temp variables to hold true sin and cos values of third angle.
  */
  double c;
  double s;

  /*
    convert column major bksl_mat3x3_t matrix to row major
  */
  double ri[3][3] = {{rin->m00, rin->m01, rin->m02},
    {rin->m10, rin->m11, rin->m12},
    {rin->m20, rin->m21, rin->m22}
  };


  if(j == f[i]) {   // Identifies Cases 1 and 3: forward cycle i --> j
    if(k != i) { //forward cycle j --> k
      // Case 1: 0 --> 1 --> 2; 1 --> 2 --> 0; 2 --> 0 --> 1
      angles->y = catan1pi(-ri[i][j], ri[i][i], angles->y);
      c = cos(angles->y);
      s = sin(angles->y);
      angles->x = catan2pi(ri[k][j] * c + ri[k][i] * s, ri[j][j] * c + ri[j][i] * s, angles->x);
      angles->y = catan2pi(ri[i][k], ri[i][i] * c - ri[i][j] * s, angles->y);
    } else { // k == i; reverse cycle j --> i
      // Case 3: 0 --> 1 --> 0; 1 --> 2 --> 1; 2 --> 0 --> 2
      // Reset k for correct index uint32_to direction cosine matrix.
      k = f[j];
      angles->z = catan1pi(ri[i][j], ri[i][k], angles->z);
      c = cos(angles->z);
      s = sin(angles->z);
      angles->x = catan2pi(ri[k][j] * c - ri[k][k] * s, ri[j][j] * c - ri[j][k] * s, angles->x);
      angles->y = catan2pi(ri[i][k] * c + ri[i][j] * s, ri[i][i], angles->y);
    }
  } else { // Cases 2 and 4: reverse cycle i --> j
    if(k != i) {  //reverse cycle j --> k
      // Case 2: 0 --> 2 --> 1; 1 --> 0 --> 2; 2 --> 1 --> 0
      angles->z = catan1pi(ri[i][j], ri[i][i], angles->z);
      c = cos(angles->z);
      s = sin(angles->z);
      angles->x = catan2pi(-(ri[k][j] * c - ri[k][i] * s), ri[j][j] * c - ri[j][i] * s, angles->x);
      angles->y = catan2pi(-ri[i][k], ri[i][i] * c + ri[i][j] * s, angles->y);
    } else {// k == i; forward cycle j --> i
      // Case 4: 0 --> 2 --> 0; 1 --> 0 --> 1; 2 --> 1 --> 2
      // Reset k for correct index uint32_to direction cosine matrix.
      k = f[i];
      angles->z = catan1pi(-ri[i][j], ri[i][k], angles->z);
      c = cos(angles->z);
      s = sin(angles->z);
      angles->x = catan2pi(-(ri[k][j] * c + ri[k][k] * s), ri[j][j] * c + ri[j][k] * s, angles->x);
      angles->y = catan2pi(-(ri[i][k] * c - ri[i][j] * s), ri[i][i], angles->y);
    }
  }
}

/*! @brief Extract scalar from a uniformly scaled bksl_mat3x3_t matrix

The norm of each column or row of an orthonormal rotation matrix is one.
Therefore, to find the scale factor for a rotation matrix that has been
<uniformly> scaled, simply find the norm of any row or column of the rotation
matrix. Warning, this approach will not work for nonuniform scaling.
For nonuniform scaling, one needs to find the eigenvectors using SVD or other means.

*/
double bbksl_mat3x3_getScalar(
  bksl_mat3x3_t* __restrict__ ri) {

  return sqrt(ri->m00 * ri->m00 + ri->m11 * ri->m11 + ri->m22 * ri->m22);
}

// /*! @brief Extract scalar from a uniformly scaled single precision bksl_mat3x3f_t matrix
//
// The norm of each column or row of an orthonormal rotation matrix is one.
//
// Therefore, to find the scale factor for a rotation matrix that has been
// <uniformly> scaled, simply find the norm of any row or column of the rotation
// matrix. Warning, this approach will not work for nonuniform scaling.
// For nonuniform scaling, one needs to find the eigenvectors using SVD or other means.
//
// */
// float bksl_mat3x3f_getScalar(
//   bksl_mat3x3f_t* ri) {
//
//   return sqrt(ri->m00 * ri->m00 + ri->m11 * ri->m11 + ri->m22 * ri->m22);
// }

/*!
@brief extract translation components of a bksl_Mat4 matrix
*/
void bksl_mat4x4_getTranslation(
  bksl_mat4x4_t* __restrict__ Mi,
  bksl_vec3_t* __restrict__ to) {

  cblas_dcopy(3, (double*)Mi + 12, 1, (double*)to, 1);
}

/*!
@brief extract translation component of a bksl_mat4x4f matrix
*/
void bksl_mat4x4f_getTranslation(
  bksl_mat4x4f_t* __restrict__ Mi,
  bksl_vec3f_t* __restrict__ to) {

  cblas_scopy(3, (float*)Mi + 12, 1, (float*)to, 1);
}

/*!
@brief utility to convert dpi, an array of length 3, to deltaq (an incremental
change in quaternion), add incremental change to an input quaternion
*/
void bksl_add_dpiToq(
  bksl_vec3_t* __restrict__ dpi,
  bksl_quaternion_t* __restrict__ qi) {

  bksl_quaternion_t dq;
  dq.x = -qi->w * dpi->x + qi->z * dpi->y - qi->y * dpi->z;
  dq.y = -qi->z * dpi->x - qi->w * dpi->y + qi->x * dpi->z;
  dq.z = qi->y * dpi->x - qi->x * dpi->y - qi->w * dpi->z;
  dq.w = -qi->x * dpi->x - qi->y * dpi->y - qi->z * dpi->z;
  cblas_daxpy(4, 0.5, (double*)&dq, 1, (double*)qi, 1);
}


/*!
@brief utility to convert dpi, an array of length 3, to deltaq (an incremental
change in quaternion), subtract incremental change from an input quaternion
*/
void bksl_subtract_dpiFromq(
  bksl_vec3_t* __restrict__ dpi,
  bksl_quaternion_t* __restrict__ qi) {

  bksl_quaternion_t dq;
  dq.x = -qi->w * dpi->x + qi->z * dpi->y - qi->y * dpi->z;
  dq.y = -qi->z * dpi->x - qi->w * dpi->y + qi->x * dpi->z;
  dq.z = qi->y * dpi->x - qi->x * dpi->y - qi->w * dpi->z;
  dq.w = -qi->x * dpi->x - qi->y * dpi->y - qi->z * dpi->z;
  cblas_daxpy(4, -0.5, (double*)&dq, 1, (double*)qi, 1);
}


/*!
@brief utility to convert angular velocity to time derivative of quaternion

\f$ \underline{\dot{e}}^{ij} \leftarrow 1/2 \mathscr{R}(\underline{e}) \underline{\omega}_{ij}^i\f$

The result is then normalized by adding a feedback term

\f$ \underline{\dot{e}}^{ij} - \frac{0.5}{\tau}(1 - \underline{e}^{ijT} \underline{e}^{ij})\underline{e}^{ij} \f$

where \f$\tau=0.5\f$ is hard coded

@param w [in] omega, angular velocity
@param qi [in] reference quaternion
@param dqo [out] output quaternion time derivative
*/
void bksl_omega_to_dquaternion(
  bksl_vec3_t* __restrict__ w,
  bksl_quaternion_t* __restrict__ qi,
  bksl_quaternion_t* __restrict__ dqo) {

  /*
    1/2 * [ qi->w  qi->z -qi->y] * {w->x}
          [-qi->z  qi->w  qi->x]   {w->y}
          [ qi->y -qi->x  qi->w]   {w->z}
          [-qi->x -qi->y -qi->z]
  */
  dqo->x =  qi->w * w->x + qi->z * w->y - qi->y * w->z;
  dqo->y = -qi->z * w->x + qi->w * w->y + qi->x * w->z;
  dqo->z =  qi->y * w->x - qi->x * w->y + qi->w * w->z;
  dqo->w = -qi->x * w->x - qi->y * w->y - qi->z * w->z;

  cblas_dscal(4, 0.5, (double*)dqo, 1);

  /*
    add feedback term to normalize quaternion
  */

  /*
    if tau is 0.5
  */
  double alpha =
    1.0 - cblas_ddot(4, (double*)qi, 1, (double*)qi, 1);

  /*
    if tau is not 0.5

    let coeff = 0.5 / tau
  */
  // double alpha =
  //   coeff * (1 - cblas_ddot(4, (double*)qi, 1, (double*)qi, 1));

  cblas_daxpy(4, alpha, (double*)qi, 1, (double*)dqo, 1);

}


/*!
@}
*/



/*!
@brief linear interpolation

@param count [in] number of entries to interpolate
@param p [in] uint32_terpolant value, ranges from 0 to 1
@param y0 [in] value of function at y=0
@param y1 [in] value of function at y=1
@param yOut [out] uint32_terpolated data returned in yOut

*/
void bksl_array_lerp(
  const uint32_t count,
  const double p,
  double* __restrict__ y0,
  double* __restrict__ y1,
  double* __restrict__ yOut) {

  /* suggest to keep using blas here, as the array we are normalizing may be
  large, and benefit from using the blas library */

  if(p < 1e-9) {
    cblas_dcopy(count, y0, 1, yOut, 1);
  } else if(p >= 1.0) {
    cblas_dcopy(count, y1, 1, yOut, 1);
  }

  for(int i = 0; i < count; i++) {
    yOut[i] = y1[i] - y0[i];
  }

  cblas_dscal(count, p, yOut, 1);

  cblas_daxpy(count, 1.0, y0, 1, yOut, 1);

}


/*!
@brief Function for performing modified Gram-Schmidt orthonormalization

This function implements a compact version of the Gram-Schmidt
algorithm to orthonormalize the columns of a matrix which has
full column rank. The matrix, whose columns are to be
orthonormalized, is passed in as A[m][n] where m and n are the
respective row and column dimensions of A[][]. The
orthonormalized columns are passed back in A[][].

@param m row dimension
@param n column dimension
@param A[m][n] a rectangular matrix to be orthonormalized, the orthonormalized
               matrix is returned in A
*/
void bksl_gramSchmidt(
  const uint32_t m,
  const uint32_t n,
  double* __restrict__ A) {

  /* @TODO get rid of blas calls here */
  for(int k = 0; k < n; k++) {
    /* normalize column k */
    double R = 1.0 / cblas_dnrm2(m, A + k, n);
    cblas_dscal(
      m,
      R,
      A + k,
      n);
    for(int j = k + 1; j < n; j++) {
      R = 0;
      for(int i = 0; i < m; i++) {
        R += A[i * n + k] * A[i * n + j];
      }
      for(int i = 0; i < m; i++) {
        A[i * n + j] -= A[i * n + k] * R;
      }
    }
  }
}

/*!
@brief used to determine if two double arrays are close to each other
within an error tolerance of 1e-6

@param count [in] number of entries in array
@param a1 [in] first double array
@param a2 [in] second double array
@returns true if values are all close, false if values are not all close
*/
bool bksl_all_close(
  const uint32_t count,
  double* __restrict__ a1,
  double* __restrict__ a2) {

  for(uint32_t i = 0; i < count; i++) {
    double scaleFactor = fmax(fabs(a1[i]), 1.0);
    if(fabs(a1[i] - a2[i]) / scaleFactor > 1e-6) {
      fprintf(stdout, "match failed at index: %d\n", i);
      return false;
    }
  }
  return true;
}


/*!
@brief get axis and angle from a rotation matrix

if sin is close to 0, the axis of rotation is not well defined.
*/
inline void bbksl_mat3x3_getAxisAngle(
  bksl_mat3x3_t* __restrict__ r,
  bksl_vec3_t* __restrict__ axis,
  double* __restrict__ angle) {

  *angle = acos(0.5 * (r->m00 + r->m11 + r->m22 - 1.0));
  if(fabs(sin(*angle)) < 1e-9) {
    /* if m22 == -1, return {1, 0, 0} */
    if(fabs(r->m22 + 1) < 1e-9) {
      axis->x = 1.0;
      axis->y = 0.0;
      axis->z = 0.0;
      return;
    } else {
      axis->x = 0.0;
      axis->y = 0.0;
      axis->z = 1.0;
      return;
    }
  }
  /*u_tilde = (1 / 2 sin \theta) * (r - r^T) */
  axis->x = r->m21 - r->m12;
  axis->y = r->m02 - r->m20;
  axis->z = r->m10 - r->m01;
  bksl_vec3_scale(1 / (2 * sin(*angle)), axis);
}


/*!
@brief get axis, angle and pitch of screw motion
from an SE3 displacement
*/
inline void bksl_displacement_getScrewAxis(
  bksl_displacement_t* __restrict__ d,
  bksl_screw_t* __restrict__ h) {

  bksl_vec3_t rotationAxis;
  double angle;
  bbksl_mat3x3_getAxisAngle(
    &d->R,
    &rotationAxis,
    &angle);



}
