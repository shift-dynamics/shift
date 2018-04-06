/**
@file
@brief A vectorized math library for kinematics and dynamics
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage

This library contains a collection of functions for translating,
rotating and displacing (translation and rotation) of vectors and
spatial quantities or screws. The following quantities can be
translated, rotated or generally displaced:

S = 6x1 s2_screw_t

V = 3x1 Vector


Displacements consisting of only translations are denoted as:

t = General Translation

tx = Translation along x axis

ty = Translation along y axis

tz = Translation along z axis


Likewise, displacements consisting of only rotations are denoted as

r = General rotation

rx = rotation about x axis

ry = rotation about y axis

rz = rotation about z axis


s2_displacement_ts, consisting of both rotation and translation are
abbreviated using a "d"

d = General s2_displacement_t


d = General s2_displacement_t


d = General s2_displacement_t


d = General s2_displacement_t

d = General s2_displacement_t
d = General s2_displacement_t

d = General s2_displacement_t


d = General s2_displacement_t

d = General s2_displacement_t
d = General s2_displacement_t

d = General s2_displacement_t

d = General s2_displacement_t


d = General s2_displacement_t

d = General s2_displacement_t
Functions for computing products using the inverse of d, r, and t are
are also provided. When the inverse operation is specified, the term
inverse appears in the function declaration AFTER d, r or t. For example:

The inverse of d is noted as "dinverse".

When transforming screw quantities the d, r and t operators take on
different forms depending on whether a screw or coscrew quantities
is being processed, therefore different functions
are provided for the two spaces.  Screws are
spatial quantities that can be measured using a ruler or clock (i.e. position,
velocity, and acceleration). Coscrews are linear operators on screws, also
called the dual or function space. Examples of coscrews include forces,
torques and spatial momentum.

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
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THqoRY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include "kinematics.h"

/*!
@name Utilities for initializing basic datastructures

Utilities to  vectors, screws, SO3 (mat3) rotation matrices and SE3 (mat4)
transformations
@{
*/

/*!
@brief  initialize a vector_t datastructure on the heap (an array of doubles of
length 3)
*/
s2_vector_t* s2_vector() {
  return calloc(1, sizeof(s2_vector_t));
}

/*!
@brief initialize a  vectorf_t datastructure on the heap (an array of floats of
length 3)

*/
s2_vectorf_t* s2_vectorf() {
  return calloc(1, sizeof(s2_vectorf_t));
}

/*! @brief
initialize a vectori_t datastructure on the heap (an array of ints of length 3),
must be freed later
*/
s2_vectori_t* s2_vectori() {
  return calloc(1, sizeof(s2_vectori_t));
}

/*!
@brief compute the norm of a double precision vector
*/
inline double s2_vector_l2norm(s2_vector_t* restrict v) {
  return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

/*!
@brief compute the norm of a single precision vector
*/
inline float s2_vectorf_l2norm(s2_vectorf_t* restrict v) {
  return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

/*!
@brief Normalize a double precision vector

@param v [in/out] vector to normalize
*/
inline void s2_vector_normalize(s2_vector_t* restrict v) {

  double norm = s2_vector_l2norm(v);
  s2_vector_scale(1.0 / norm, v);
}

/*!
@brief scale a double precision vector
@param [in/out] vector to scale
*/
inline void s2_vector_scale(double k, s2_vector_t* restrict v) {
  v->x *= k;
  v->y *= k;
  v->z *= k;
}

/*!
@brief Normalize a double precision vector

@param v [in/out] vector to normalize
*/
inline void s2_vectorf_normalize(s2_vectorf_t* restrict v) {

  float norm = s2_vectorf_l2norm(v);
  s2_vectorf_scale(1.0 / norm, v);
}

/*!
@brief scale a double precision vector
@param [in/out] vector to scale
*/
inline void s2_vectorf_scale(float k, s2_vectorf_t* restrict v) {

  v->x *= k;
  v->y *= k;
  v->z *= k;
}

/*! @brief initializes a general s2_displacement_t datastructure

A general displacement is an array of doubles of length 12, consisting of a
rotation matrix in column major order followed by a translation vector.

@returns A displacement with the rotation component set to identity and
translation set to zero.
*/
s2_displacement_t* s2_displacement() {
  s2_displacement_t* D = calloc(1, sizeof(s2_displacement_t));
  D->R.m00 = 1.0;
  D->R.m11 = 1.0;
  D->R.m22 = 1.0;
  return D;
}

/*! @brief s a general s2_displacementf_t datastructure

A general displacement is an array of float of length 12, consisting of a
rotation matrix in column major order followed by a translation vector
*/
s2_displacementf_t* s2_displacementf() {
  s2_displacementf_t* D = calloc(1, sizeof(s2_displacementf_t));
  D->R.m00 = 1.f;
  D->R.m11 = 1.f;
  D->R.m22 = 1.f;
  return D;
}

/*! @brief s a s2_screw_t datastructure (an array of doubles of length 6)

*/
s2_screw_t* s2_screw() {
  return calloc(1, sizeof(s2_screw_t));
}

/*! @brief s a s2_screw_t datastructure (an array of floats of length 6)

*/
s2_screwf_t* s2_screwf() {
  return calloc(1, sizeof(s2_screwf_t));
}

/*! @brief s a double precision s2_mat3x3_t datastructure for a general
rotation (i.e. a SO3 matrix). The matrix is specified in column major order.

@returns A s2_mat3x3_t rotation matrix set to identity.
*/
s2_mat3x3_t* s2_mat3x3() {
  s2_mat3x3_t* mat3 = calloc(1, sizeof(s2_mat3x3_t));
  mat3->m00 = 1.0;
  mat3->m11 = 1.0;
  mat3->m22 = 1.0;
  return mat3;
}

/*! @brief s a single precision s2_mat3x3f_t datastructure for a general
rotation (i.e. a SO3 matrix). The matrix is specified in column major order.

@returns A s2_mat3x3f_t rotation matrix set to identity.
*/
s2_mat3x3f_t* s2_mat3x3f() {
  s2_mat3x3f_t* mat3 = calloc(1, sizeof(s2_mat3x3f_t));
  mat3->m00 = 1.f;
  mat3->m11 = 1.f;
  mat3->m22 = 1.f;
  return mat3;
}

/*! @brief s a double precision s2_Mat4 datastructure for a general
transformation (i.e. a SE3 matrix). The matrix is specified in column major
order.

@returns A s2_Mat4 SE3 matrix set to identity.
*/
s2_mat4x4_t* s2_mat4x4() {
  s2_mat4x4_t* mat4 = calloc(1, sizeof(s2_mat4x4_t));
  mat4->m00 = 1.0;
  mat4->m11 = 1.0;
  mat4->m22 = 1.0;
  mat4->m33 = 1.0;
  return mat4;
}

/*! @brief s a single precision s2_Mat4f datastructure for a general
transformation (i.e. a SE3 matrix). The matrix is specified in column major
order.

@returns A s2_Mat4f SE3 matrix set to identity.
*/
s2_mat4x4f_t* s2_mat4x4f() {

  s2_mat4x4f_t* mat4 = calloc(1, sizeof(s2_mat4x4f_t));
  mat4->m00 = 1.f;
  mat4->m11 = 1.f;
  mat4->m22 = 1.f;
  mat4->m33 = 1.f;
  return mat4;
}

/*! @brief converts a double precision s2_displacement_t datastructure to
s2_Mat4 format.

*/
inline void s2_displacement_toMat4x4(s2_displacement_t* restrict D,
                                     s2_mat4x4_t* restrict mat4) {

  memset(mat4, 0, sizeof(s2_mat4x4_t));
  memcpy(&mat4->m00, &D->R.m00, sizeof(s2_vector_t));
  memcpy(&mat4->m01, &D->R.m01, sizeof(s2_vector_t));
  memcpy(&mat4->m02, &D->R.m02, sizeof(s2_vector_t));
  memcpy(&mat4->m03, &D->T, sizeof(s2_vector_t));
  mat4->m33 = 1.0;
}

/*! @brief converts a single precision s2_displacementf_t datastructure to
s2_Mat4f format.

*/
inline void s2_displacementf_toMat4x4f(s2_displacementf_t* restrict D,
                                       s2_mat4x4f_t* restrict mat4) {

  memset(mat4, 0, sizeof(s2_mat4x4f_t));
  memcpy(&mat4->m00, &D->R.m00, sizeof(s2_vectorf_t));
  memcpy(&mat4->m01, &D->R.m01, sizeof(s2_vectorf_t));
  memcpy(&mat4->m02, &D->R.m02, sizeof(s2_vectorf_t));
  memcpy(&mat4->m03, &D->T, sizeof(s2_vectorf_t));
  mat4->m33 = 1.0;
}

/*! @brief converts a double precision s2_displacement_t datastructure to a
single precision s2_Mat4f format.

*/
inline void s2_displacement_toMat4x4f(s2_displacement_t* restrict D,
                                      s2_mat4x4f_t* restrict mat4) {

  double* Di = (double*)D;
  float* mat4i = (float*)mat4;
  memset(mat4, 0, 15 * sizeof(float));
  for(int i = 0; i < 3; i++) {
    mat4i[i + 0] = (float)Di[i];
  }
  for(int i = 3; i < 6; i++) {
    mat4i[i + 1] = (float)Di[i];
  }
  for(int i = 6; i < 9; i++) {
    mat4i[i + 2] = (float)Di[i];
  }
  for(int i = 9; i < 12; i++) {
    mat4i[i + 3] = (float)Di[i];
  }
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
void s2_vector_print(FILE* f, s2_vector_t* ti) {

  fprintf(f, "%f %f %f\n", ti->x, ti->y, ti->z);
}

/*!
@brief Utility to print a quaternion

quaternion is printed following the x,y,z,w convention

@param f [in] file pointer to direct output to
@param q [in] quaternion to print
*/
void s2_quaternion_print(FILE* f, s2_quaternion_t* q) {
  fprintf(f, "%f %f %f %f\n", q->x, q->y, q->z, q->w);
}

/*!
@brief Utility to print a 6x1 screw vector

prints a 6x1 column vector as a row vector

@param si 6x1 column vector
*/
void s2_screw_print(FILE* f, s2_screw_t* si) {

  fprintf(f, "%g,%g,%g,%g,%g,%g,", si->m0, si->m1, si->m2, si->m3, si->m4,
          si->m5);
}

/*!
@brief Utility to print a 3x3 rotation matrix

prints a rotation matrix

@param ri 3x3 rotation matrix
*/
void s2_mat3x3_print(FILE* f, s2_mat3x3_t* ri) {

  fprintf(f, "%f %f %f\n", ri->m00, ri->m01, ri->m02);
  fprintf(f, "%f %f %f\n", ri->m10, ri->m11, ri->m12);
  fprintf(f, "%f %f %f\n", ri->m20, ri->m21, ri->m22);
}

/*!
@brief Utility to print a 3x3 rotation matrix

prints a rotation matrix

@param ri 3x3 rotation matrix
*/
void s2_mat3x3f_print(FILE* f, s2_mat3x3f_t* ri) {
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
void s2_displacement_print(FILE* f, s2_displacement_t* Di) {

  fprintf(f, "  % 9.9f % 9.9f % 9.9f % 9.9f\n", Di->R.m00, Di->R.m01, Di->R.m02,
          Di->T.x);
  fprintf(f, "  % 9.9f % 9.9f % 9.9f % 9.9f\n", Di->R.m10, Di->R.m11, Di->R.m12,
          Di->T.y);
  fprintf(f, "  % 9.9f % 9.9f % 9.9f % 9.9f\n", Di->R.m20, Di->R.m21, Di->R.m22,
          Di->T.z);
  fprintf(f, "  ");
  for(int i = 0; i < 3; i++) {
    fprintf(f, " 0.000000000 ");
  }
  fprintf(f, " 1.000000000\n");
}

/*!
@brief Utility to print a SE3 mat4 matrix

prints a s2_Mat4f as a 4x4 matrix for transforming
homogeneous coordinates. Note that the displacement is stored in
memory as a 12 x 1 contiguous array with terms of the rotation matrix
in the first 9 entries of the array (in row major order), followed by
translation terms in the last 3 entries of the array.

@param Di A general displacement, \f$D_i\f$
*/
void s2_mat4x4_print(FILE* f, s2_mat4x4_t* Di) {

  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m00, Di->m01, Di->m02, Di->m03);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m10, Di->m11, Di->m12, Di->m13);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m20, Di->m21, Di->m22, Di->m23);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m30, Di->m31, Di->m32, Di->m33);
}

/*!
@brief Utility to print a SE3 mat4 matrix

prints a s2_Mat4f as a 4x4 matrix for transforming
homogeneous coordinates. Note that the displacement is stored in
memory as a 12 x 1 contiguous array with terms of the rotation matrix
in the first 9 entries of the array (in row major order), followed by
translation terms in the last 3 entries of the array.

@param Di A general displacement, \f$D_i\f$
*/
void s2_mat4x4f_print(FILE* f, s2_mat4x4f_t* Di) {

  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m00, Di->m01, Di->m02, Di->m03);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m10, Di->m11, Di->m12, Di->m13);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m20, Di->m21, Di->m22, Di->m23);
  fprintf(f, "%0.4f %0.4f %0.4f %0.4f\n", Di->m30, Di->m31, Di->m32, Di->m33);
}

/*!
@brief utility to print a general array of double of length n on one line
*/
void s2_array_print(FILE* f, int n, double* a) {

  for(int i = 0; i < n; i++) {
    fprintf(f, "%0.6g ", a[i]);
  }
  fprintf(f, "\n");
}

/*!
@brief utility to print a 2D array stored in a single pointer
with dimension rowDim * colDim
*/
void s2_array_print2D(FILE* f, int rowDim, int colDim, bool isRowMajor,
                      double* a) {

  if(isRowMajor) {
    for(int i = 0; i < rowDim; i++) {
      fprintf(f, "  [");
      for(int j = 0; j < colDim; j++) {
        fprintf(f, "% 0.10f", a[i * colDim + j]);
        if(j != colDim - 1) {
          fprintf(f, ", ");
        }
      }
      fprintf(f, "]\n");
    }
  } else {
    for(int i = 0; i < rowDim; i++) {
      fprintf(f, "  ");
      for(int j = 0; j < colDim; j++) {
        fprintf(f, "% 0.10f ", a[i + j * rowDim]);
      }
      fprintf(f, "\n");
    }
  }
}

/*!
@brief utility to print a general array of ints of length n on one line
*/
void s2_arrayi_print(FILE* f, int n, int* a) {

  for(int i = 0; i < n; i++) {
    fprintf(f, "%d, ", a[i]);
  }
  fprintf(f, "\n");
}

/*!
@brief utility to print a lower triangular matrix of doubles stored in a
linear array
*/
void s2_triang_print(FILE* f, int n, double* a) {

  for(int i = 0; i < n; i++) {
    int i_s = (i * (i + 1) / 2);
    for(int j = 0; j < i + 1; j++) {
      fprintf(f, "% 9.4f,", a[i_s + j]);
    }
    fprintf(f, "\n");
  }
}

/*!
@brief utility to print a lower triangular matrix of integers stored in a
linear array
*/
void s2_triangi_print(FILE* f, int n, int* a) {

  for(int i = 0; i < n; i++) {
    int i_s = (i * (i + 1) / 2);
    for(int j = 0; j < i + 1; j++) {
      fprintf(f, "% d,", a[i_s + j]);
    }
    fprintf(f, "\n");
  }
}

/*!
@brief utility to print a lower triangular matrix of integers stored in a
linear array
*/
void s2_triangb_print(FILE* f, int n, bool* a) {

  for(int i = 0; i < n; i++) {
    int i_s = (i * (i + 1) / 2);
    for(int j = 0; j < i + 1; j++) {
      fprintf(f, "%s,", a[i_s + j] ? "1" : "0");
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
inline void s2_dc(double* restrict thetai, double dc[2]) {

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

@param D an SE3 matrix to set to identity, the resulting matrix will be
overwritten
*/

inline void s2_displacement_setIdentity(s2_displacement_t* restrict D) {
  memset(D, 0, sizeof(s2_displacement_t));
  D->R.m00 = 1.0;
  D->R.m11 = 1.0;
  D->R.m22 = 1.0;
}

/*!
@brief Set a rotation matrix to identity

Overwrites a rotation matrix with the identity matrix.

@param R an SE3 matrix to set to identity, the resulting matrix will be
overwritten
*/

inline void s2_mat3x3_setIdentity(s2_mat3x3_t* restrict R) {
  memset(R, 0, sizeof(s2_mat3x3_t));
  R->m00 = 1.0;
  R->m11 = 1.0;
  R->m22 = 1.0;
}

/*!
@brief Set a rotation matrix to identity

Overwrites a rotation matrix with the identity matrix.

@param R an SE3 matrix to set to identity, the resulting matrix will be
overwritten
*/

inline void s2_mat4x4f_setIdentity(s2_mat4x4f_t* restrict R) {
  memset(R, 0, sizeof(s2_mat4x4f_t));
  ((float*)R)[0 * 4 + 0] = 1.f;
  ((float*)R)[1 * 4 + 1] = 1.f;
  ((float*)R)[2 * 4 + 2] = 1.f;
  ((float*)R)[3 * 4 + 3] = 1.f;
}

/*!
@}
@name utilities to set and get quantities by index

@{
*/

/*!
@brief set a value of vector
*/
inline void s2_vector_set(s2_vector_t* restrict v, int index, double value) {

  // if(index > 2 || index < 0) {
  //   fprintf(stderr, "s2_vector_set: out of bounds\n");
  //   return;
  // }
  ((double*)v)[index] = value;
}

/*!
@brief set a value of quaternion at specified index
*/
inline void s2_quaternion_set(s2_quaternion_t* restrict q, int index,
                              double value) {
  ((double*)q)[index] = value;
}

/*!
@brief l2 norm of double precision screw
*/
inline double s2_screw_l2norm(s2_screw_t* restrict v) {
  register double sum = 0;
  sum += v->m0 * v->m0;
  sum += v->m1 * v->m1;
  sum += v->m2 * v->m2;
  sum += v->m3 * v->m3;
  sum += v->m4 * v->m4;
  sum += v->m5 * v->m5;
  return sqrt(sum);
}

/*!
@brief l2 norm of single precision screw
*/
inline float s2_screwf_l2norm(s2_screwf_t* restrict v) {
  register double sum = 0;
  sum += v->m0 * v->m0;
  sum += v->m1 * v->m1;
  sum += v->m2 * v->m2;
  sum += v->m3 * v->m3;
  sum += v->m4 * v->m4;
  sum += v->m5 * v->m5;
  return sqrt(sum);
}

/*!
@brief set a value of a screw
*/
inline void s2_screw_set(s2_screw_t* restrict s, int index, double value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 5 || index < 0) {
    fprintf(stderr, "s2_screw_set: out of bounds\n");
    return;
  }
#endif
  ((double*)s)[index] = value;
}

/*!
@brief set a value of a mat3x3

note that mat3x3 is stored in column major order
*/
inline void s2_mat3x3_set(s2_mat3x3_t* restrict R, int row, int column,
                          double value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 2 || row < 0 || column < 0 || column > 2) {
    fprintf(stderr, "s2_mat3x3_set: out of bounds\n");
    return;
  }
#endif
  ((double*)R)[3 * column + row] = value;
}

/*!
@brief set a mat3x3 rotation matrix from three vectors

note that mat3x3 is stored in column major order
*/
inline void s2_mat3x3_setFromVectors(s2_mat3x3_t* restrict R,
                                     s2_vector_t* restrict x,
                                     s2_vector_t* restrict y,
                                     s2_vector_t* restrict z) {

  memcpy(&(R->m00), x, sizeof(s2_vector_t));
  memcpy(&(R->m01), y, sizeof(s2_vector_t));
  memcpy(&(R->m02), z, sizeof(s2_vector_t));
}

/*!
@brief set a value of a mat3x3

note that mat3x3 is stored in column major order
*/
inline void s2_mat4x4_set(s2_mat4x4_t* restrict R, int row, int column,
                          double value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 3) {
    fprintf(stderr, "s2_mat4x4_set: out of bounds\n");
    return;
  }
#endif
  ((double*)R)[4 * column + row] = value;
}

/*!
@brief set a value of a displacement
*/
inline void s2_displacement_set(s2_displacement_t* restrict D, int row,
                                int column, double value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 3) {
    fprintf(stderr, "s2_displacement_set: out of bounds\n");
    return;
  }
#endif
  if(column == 3) {
    s2_vector_set(&D->T, row, value);
  } else {
    s2_mat3x3_set(&D->R, row, column, value);
  }
}

/*!
@brief set a value of vectorf
*/
inline void s2_vectorf_set(s2_vectorf_t* restrict v, int index, float value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 2 || index < 0) {
    fprintf(stderr, "s2_vector_set: out of bounds\n");
    return;
  }
#endif
  ((float*)v)[index] = value;
}

/*!
@brief set a value of a screwf
*/
inline void s2_screwf_set(s2_screwf_t* restrict s, int index, float value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 5 || index < 0) {
    fprintf(stderr, "s2_screw_set: out of bounds\n");
    return;
  }
#endif
  ((float*)s)[index] = value;
}

/*!
@brief set a value of a mat3x3f

note that mat3x3 is stored in column major order
*/
inline void s2_mat3x3f_set(s2_mat3x3f_t* restrict R, int row, int column,
                           float value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 2) {
    fprintf(stderr, "s2_mat3x3_set: out of bounds\n");
    return;
  }
#endif
  ((float*)R)[row + column * 3] = value;
}

/*!
@brief set a value of a mat4x4f

note that mat3x3 is stored in column major order
*/
inline void s2_mat4x4f_set(s2_mat4x4f_t* restrict R, int row, int column,
                           float value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 3) {
    fprintf(stderr, "s2_mat4x4f_set: out of bounds\n");
    return;
  }
#endif
  ((float*)R)[4 * column + row] = value;
}

/*!
@brief set a value of a displacementf
*/
inline void s2_displacementf_set(s2_displacementf_t* restrict D, int row,
                                 int column, float value) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 3) {
    fprintf(stderr, "s2_displacement_set: out of bounds\n");
    return;
  }
#endif
  if(column == 3) {
    s2_vectorf_set(&D->T, row, value);
  } else {
    s2_mat3x3f_set(&D->R, row, column, value);
  }
}

/*!
@brief get a value of vector
*/
inline double s2_vector_get(s2_vector_t* restrict v, int index) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 2 || index < 0) {
    fprintf(stderr, "s2_vector_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((double*)v)[index];
}

/*!
@brief get a value of vectori
*/
inline int s2_vectori_get(s2_vectori_t* restrict v, int index) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 2 || index < 0) {
    fprintf(stderr, "s2_vector_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((int*)v)[index];
}

/*!
@brief set a value of a screw
*/
inline double s2_screw_get(s2_screw_t* restrict s, int index) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 5 || index < 0) {
    fprintf(stderr, "s2_screw_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((double*)s)[index];
}

/*!
@brief set a value of a mat3x3

note that mat3x3 is stored in column major order
*/
inline double s2_mat3x3_get(s2_mat3x3_t* restrict R, int row, int column) {
#ifdef _SHIFT_BOUNDS_CHECKING
  assert(row < 3 && row > -1 && column < 3 && column > -1);
  if(row > 3 || row < 0 || column < 0 || column > 2) {
    fprintf(stderr, "s2_mat3x3_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((double*)R)[3 * column + row];
}

/*!
@brief set a value of a displacement
*/
inline double s2_displacement_get(s2_displacement_t* restrict D, int row,
                                  int column) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 3) {
    fprintf(stderr, "s2_displacement_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  if(column == 3) {
    return s2_vector_get(&D->T, row);
  } else {
    return s2_mat3x3_get(&D->R, row, column);
  }
}

/*!
@brief set a value of vectorf
*/
inline float s2_vectorf_get(s2_vectorf_t* restrict v, int index) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 2 || index < 0) {
    fprintf(stderr, "s2_vector_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((float*)v)[index];
}

/*!
@brief set a value of a screwf
*/
inline float s2_screwf_get(s2_screwf_t* restrict s, int index) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(index > 5 || index < 0) {
    fprintf(stderr, "s2_screw_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((float*)s)[index];
}

/*!
@brief set a value of a mat3x3f

note that mat3x3 is stored in column major order
*/
inline float s2_mat3x3f_get(s2_mat3x3f_t* restrict R, int row, int column) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 2) {
    fprintf(stderr, "s2_mat3x3_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  return ((float*)R)[3 * column + row];
}

/*!
@brief set a value of a displacementf
*/
inline float s2_displacementf_get(s2_displacementf_t* restrict D, int row,
                                  int column) {
#ifdef _SHIFT_BOUNDS_CHECKING
  if(row > 3 || row < 0 || column < 0 || column > 3) {
    fprintf(stderr, "s2_displacement_set: out of bounds\n");
    return -INFINITY;
  }
#endif
  if(column == 3) {
    return s2_vectorf_get(&D->T, row);
  } else {
    return s2_mat3x3f_get(&D->R, row, column);
  }
}

/*!
@brief get translation component of a general displacement

*/
inline void s2_displacement_getTranslation(s2_displacement_t* restrict d,
                                           s2_vector_t* restrict t) {

  s2_vector_copy(&(d->T), t);
}

/*!
@}
@name 3x1 vector algebra utilities
Utilities for carrying out Vector/Vector and Vector/Scalar operations
@{
*/

/*!
@brief Copy a double precision Vector vi into Vector vo.


\f$V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void s2_vector_copy(s2_vector_t* restrict vi, s2_vector_t* restrict vo) {
  memcpy(vo, vi, sizeof(s2_vector_t));
}

/*!
@brief Copy a single precision Vectorf vi into Vectorf vo.

\f$V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void s2_vectorf_copy(s2_vectorf_t* restrict vi,
                            s2_vectorf_t* restrict vo) {
  memcpy(vo, vi, sizeof(s2_vectorf_t));
}

/*!
@brief Swap double precision Vectors vio1 and vio2.

\f$V_io1 \rightarrow V_io2\f$
\f$V_io2 \rightarrow V_io1\f$

@param vio1 input/output vector \f$V_io1\f$
@param vio2 input/output vector \f$V_io2\f$
*/
inline void s2_vector_swap(s2_vector_t* restrict vio1,
                           s2_vector_t* restrict vio2) {
  s2_vector_t temp;
  s2_vector_copy(vio1, &temp);
  s2_vector_copy(vio2, vio1);
  s2_vector_copy(&temp, vio2);
}

/*!
@brief Swap single precision Vectors vio1 and vio2.


\f$V_io1 \rightarrow V_io2\f$
\f$V_io2 \rightarrow V_io1\f$


@param vio1 input/output vector \f$V_io1\f$
@param vio2 input/output vector \f$V_io2\f$
*/
inline void s2_vectorf_swap(s2_vectorf_t* restrict vio1,
                            s2_vectorf_t* restrict vio2) {
  s2_vectorf_t temp;
  s2_vectorf_copy(vio1, &temp);
  s2_vectorf_copy(vio2, vio1);
  s2_vectorf_copy(&temp, vio2);
}

/*!
@brief Copy inverse of double precision Vector vi into vo.

\f$-V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void s2_vector_invert(s2_vector_t* restrict vi,
                             s2_vector_t* restrict vo) {
  vo->x = -vi->x;
  vo->y = -vi->y;
  vo->z = -vi->z;
}

/*!
@brief invert a double precision vector.

\f$-V_i \rightarrow V_i\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void s2_vector_invertInPlace(s2_vector_t* restrict vi) {
  vi->x = -vi->x;
  vi->y = -vi->y;
  vi->z = -vi->z;
}

/*!
@brief Copy a single prevision Vectorf vi into vo.

\f$-V_i \rightarrow V_o\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void s2_vectorf_invert(s2_vectorf_t* restrict vi,
                              s2_vectorf_t* restrict vo) {
  vo->x = -vi->x;
  vo->y = -vi->y;
  vo->z = -vi->z;
}

/*!
@brief invert a single precision vector in place

\f$-V_i \rightarrow V_i\f$

@param vi input vector \f$V_i\f$
@param vo output vector \f$V_o\f$
*/
inline void s2_vectorf_invertInPlace(s2_vectorf_t* restrict vi) {
  vi->x = -vi->x;
  vi->y = -vi->y;
  vi->z = -vi->z;
}

/*!
@brief double precision Dot product of two vectors

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline double s2_dot_vv(s2_vector_t* restrict vi0, s2_vector_t* restrict vi1) {

  register double sum = 0;
  sum += vi0->x * vi1->x;
  sum += vi0->y * vi1->y;
  sum += vi0->z * vi1->z;
  return sum;
}

/*!
@brief single precision Dot product of two vectors

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline float s2_dot_vvf(s2_vectorf_t* restrict vi0,
                        s2_vectorf_t* restrict vi1) {

  register float sum = 0;
  sum += vi0->x * vi1->x;
  sum += vi0->y * vi1->y;
  sum += vi0->z * vi1->z;
  return sum;
}

/*!
@brief double precision Dot product of two screws

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline double s2_dot_ss(s2_screw_t* restrict vi0, s2_screw_t* restrict vi1) {
  register double sum = 0;
  sum += vi0->m0 * vi1->m0;
  sum += vi0->m1 * vi1->m1;
  sum += vi0->m2 * vi1->m2;
  sum += vi0->m3 * vi1->m3;
  sum += vi0->m4 * vi1->m4;
  sum += vi0->m5 * vi1->m5;
  return sum;
}

/*!
@brief single precision Dot product of two screws

\f$ V_i0 \cdot V_i1 \rightarrow V_o\f$

@param vi0 first input vector
@param vi1 second input vector
*/
inline float s2_dot_ssf(s2_screwf_t* restrict vi0, s2_screwf_t* restrict vi1) {

  register double sum = 0;
  sum += vi0->m0 * vi1->m0;
  sum += vi0->m1 * vi1->m1;
  sum += vi0->m2 * vi1->m2;
  sum += vi0->m3 * vi1->m3;
  sum += vi0->m4 * vi1->m4;
  sum += vi0->m5 * vi1->m5;
  return sum;
}

/*!
@brief Computes the product of a vector and a scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void s2_product_vk(s2_vector_t* restrict vi, double* restrict ki,
                          s2_vector_t* restrict vo) {

  vo->x = vi->x * *ki;
  vo->y = vi->y * *ki;
  vo->z = vi->z * *ki;
}

/*!
@brief Computes the product of a vector and 1/scalar

\f$V_i * 1 / k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void s2_product_vkinverse(s2_vector_t* restrict vi, double* restrict ki,
                                 s2_vector_t* restrict vo) {
  register double inv = 1.0 / *ki;
  vo->x = vi->x * inv;
  vo->y = vi->y * inv;
  vo->z = vi->z * inv;

  // cblas_dcopy(3, (double*)vi, 1, (double*)vo, 1);
  // cblas_dscal(3, 1. / *ki, (double*)vo, 1);
}

/*!
@brief Computes the product of a vector and a 1/scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void s2_product_vinversek(s2_vector_t* restrict vi, double* restrict ki,
                                 s2_vector_t* restrict vo) {
  vo->x = -vi->x * *ki;
  vo->y = -vi->y * *ki;
  vo->z = -vi->z * *ki;
}

/*!
@brief Computes the product of a vector and a scalar

\f$V_i * k_i \rightarrow V_o\f$

@param vi[3] A 3x1 input vector
@param *ki a pointer to a scalar
@param vo[3] the results are stored in vo[3]
*/
inline void s2_product_vkf(s2_vectorf_t* restrict vi, float* restrict ki,
                           s2_vectorf_t* restrict vo) {
  vo->x = vi->x * *ki;
  vo->y = vi->y * *ki;
  vo->z = vi->z * *ki;
}

/*!
@brief Adds two double precision vectors

Add 3 by 1 column matrices v1i and v2i and store results in vo.

\f$V_{1i} + V_{2i} \rightarrow V_o\f$

@param v1i a first column vector input
@param v2i second column vector
@param vo the results of v1i + v2i are stored in vo
*/
inline void s2_add_vv(s2_vector_t* restrict v1i, s2_vector_t* restrict v2i,
                      s2_vector_t* restrict vo) {
  vo->x = v1i->x + v2i->x;
  vo->y = v1i->y + v2i->y;
  vo->z = v1i->z + v2i->z;
}

/*!
@brief Adds two single precision vectors

Add 3 by 1 column matrices v1i and v2i and store results in vo.

\f$V_{1i} + V_{2i} \rightarrow V_o\f$

@param v1i a first column vector input
@param v2i second column vector
@param vo the results of v1i + v2i are stored in vo
*/
inline void s2_add_vvf(s2_vectorf_t* restrict v1i, s2_vectorf_t* restrict v2i,
                       s2_vectorf_t* restrict vo) {
  vo->x = v1i->x + v2i->x;
  vo->y = v1i->y + v2i->y;
  vo->z = v1i->z + v2i->z;
}

/*!
@brief Adds two double precision vectors

Add 3 by 1 column matrix v1i to v2i

\f$V_{2i} += V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void s2_addequal_vv(s2_vector_t* restrict v1i,
                           s2_vector_t* restrict v2i) {
  v2i->x += v1i->x;
  v2i->y += v1i->y;
  v2i->z += v1i->z;
}

/*!
@brief Adds two single precision vectors

Add 3 by 1 column matrix v1i to v2i

\f$V_{2i} += V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void s2_addequal_vvf(s2_vectorf_t* restrict v1i,
                            s2_vectorf_t* restrict v2i) {

  v2i->x += v1i->x;
  v2i->y += v1i->y;
  v2i->z += v1i->z;
}

/*!
@brief Adds two double precision vectors

subtract 3 by 1 column matrix v1i from v2i

\f$V_{2i} -= V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void s2_subtractequal_vv(s2_vector_t* restrict v1i,
                                s2_vector_t* restrict v2i) {
  v2i->x -= v1i->x;
  v2i->y -= v1i->y;
  v2i->z -= v1i->z;
}

/*!
@brief Adds two single precision vectors

Subtract 3 by 1 column matrix v1i from v2i

\f$V_{2i} -= V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void s2_subtractequal_vvf(s2_vectorf_t* restrict v1i,
                                 s2_vectorf_t* restrict v2i) {
  v2i->x -= v1i->x;
  v2i->y -= v1i->y;
  v2i->z -= v1i->z;
}

/*!
@brief axpy (ax plus y stored in y) for double precision vectors

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void s2_axpy_vv(double k, s2_vector_t* restrict v1x,
                       s2_vector_t* restrict v2y) {
  v2y->x += v1x->x * k;
  v2y->y += v1x->y * k;
  v2y->z += v1x->z * k;
}

/*!
@brief axpy (ax plus y stored in y) for single precision vectors

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void s2_axpy_vvf(float k, s2_vectorf_t* restrict v1x,
                        s2_vectorf_t* restrict v2y) {
  v2y->x += v1x->x * k;
  v2y->y += v1x->y * k;
  v2y->z += v1x->z * k;
}

/*!
@brief axpy (ax plus y stored in y) for double precision screws

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i a first column vector input
@param v2i second column vector
*/
inline void s2_axpy_ss(double k, s2_screw_t* restrict v1x,
                       s2_screw_t* restrict v2y) {
  v2y->m0 += v1x->m0 * k;
  v2y->m1 += v1x->m1 * k;
  v2y->m2 += v1x->m2 * k;
  v2y->m3 += v1x->m3 * k;
  v2y->m4 += v1x->m4 * k;
  v2y->m5 += v1x->m5 * k;
}

/*!
@brief axpy (ax plus y stored in y) for single precision screws

add 3 by 1 column matrix v1i scaled by k to v2i

\f$V_{2i} += k * V_{1i} \f$

@param v1i [in] a first column vector input
@param v2i [in/out] second column vector
*/
inline void s2_axpy_ssf(float k, s2_screwf_t* restrict v1x,
                        s2_screwf_t* restrict v2y) {
  v2y->m0 += v1x->m0 * k;
  v2y->m1 += v1x->m1 * k;
  v2y->m2 += v1x->m2 * k;
  v2y->m3 += v1x->m3 * k;
  v2y->m4 += v1x->m4 * k;
  v2y->m5 += v1x->m5 * k;
}

/*!
@brief Subtract two double precision vectors

Subtract 3 by 1 column matrix v2i from v1i and store results in vo.

\f$V_{1i} - V_{2i} \rightarrow V_o\f$

@param v1i [in] a first 3x1 vector
@param V21 [in] a second 3x1 vector
@param vo [out] the results of v1i - v2i are returned in vo
*/
inline void s2_subtract_vv(s2_vector_t* restrict v1i, s2_vector_t* restrict v2i,
                           s2_vector_t* restrict vo) {
  vo->x = v1i->x - v2i->x;
  vo->y = v1i->y - v2i->y;
  vo->z = v1i->z - v2i->z;
}

/*!
@brief Subtract two single precision vectors

Subtract 3 by 1 column matrix v2i from v1i and store results in vo.

\f$V_{1i} - V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param V21 a second 3x1 vector
@param vo the results of v1i - v2i are returned in vo
*/
inline void s2_subtract_vvf(s2_vectorf_t* restrict v1i,
                            s2_vectorf_t* restrict v2i,
                            s2_vectorf_t* restrict vo) {
  vo->x = v1i->x - v2i->x;
  vo->y = v1i->y - v2i->y;
  vo->z = v1i->z - v2i->z;
}

/*!
@brief Compute double precision cross product of v1i with v2i

\f$V_{1i} \times V_{2i} \rightarrow V_o\f$

@param v1i a first 3x1 vector
@param v2i a second 3x1 vector
@param vo the results are stored in vo
*/
inline void s2_cross_vv(s2_vector_t* restrict v1i, s2_vector_t* restrict v2i,
                        s2_vector_t* restrict vo) {

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
inline void s2_cross_vvinverse(s2_vector_t* restrict v1i,
                               s2_vector_t* restrict v2i,
                               s2_vector_t* restrict vo) {

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
inline void s2_cross_vvf(s2_vectorf_t* restrict v1i, s2_vectorf_t* restrict v2i,
                         s2_vectorf_t* restrict vo) {

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
inline void s2_cross_vvinversef(s2_vectorf_t* restrict v1i,
                                s2_vectorf_t* restrict v2i,
                                s2_vectorf_t* restrict vo) {

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
@brief Copy double precision s2_screw_t si to so.

\f$S_i \rightarrow S_o\f$

@param si 6x1 screw input
@param so si is copied to so
*/
inline void s2_screw_copy(s2_screw_t* restrict si, s2_screw_t* restrict so) {
  memcpy(so, si, sizeof(s2_screw_t));
}

/*!
@brief Copy single precision s2_screw_t si to so.

\f$S_i \rightarrow S_o\f$

@param si 6x1 screw input
@param so si is copied to so
*/
inline void s2_screwf_copy(s2_screwf_t* restrict si, s2_screwf_t* restrict so) {
  memcpy(so, si, sizeof(s2_screwf_t));
}

/*!
@brief Invert a double precision s2_screw_t si, store the results in so.

\f$-S_i \rightarrow S_o\f$

@param si input vector \f$S_i\f$
@param so output vector \f$S_o\f$
*/
inline void s2_screw_invert(s2_screw_t* restrict si, s2_screw_t* restrict so) {
  so->m0 = -si->m0;
  so->m1 = -si->m1;
  so->m2 = -si->m2;
  so->m3 = -si->m3;
  so->m4 = -si->m4;
  so->m5 = -si->m5;
}

/*!
@brief Invert a single precision s2_screwf_t si, store the results in so.

\f$-S_i \rightarrow S_o\f$

@param si input vector \f$S_i\f$
@param so output vector \f$S_o\f$
*/
inline void s2_screwf_invert(s2_screwf_t* restrict si,
                             s2_screwf_t* restrict so) {
  so->m0 = -si->m0;
  so->m1 = -si->m1;
  so->m2 = -si->m2;
  so->m3 = -si->m3;
  so->m4 = -si->m4;
  so->m5 = -si->m5;
}

/*!
@brief Computes the product of a a double precision s2_screw_t and a scalar

\f$S_i * k_i \rightarrow S_o\f$

@param si[6] A 3x1 input vector
@param *ki a pointer to a scalar
@param so[6] the results are stored in so[6]
*/
inline void s2_product_sk(s2_screw_t* restrict si, const double* ki,
                          s2_screw_t* restrict so) {

  register double k = *ki;
  so->m0 = si->m0 * k;
  so->m1 = si->m1 * k;
  so->m2 = si->m2 * k;
  so->m3 = si->m3 * k;
  so->m4 = si->m4 * k;
  so->m5 = si->m5 * k;
}

/*!
@brief Computes the product of a single precision s2_screwf_t and a scalar

\f$S_i * k_i \rightarrow S_o\f$

@param si[6] A 3x1 input vector
@param *ki a pointer to a scalar
@param so[6] the results are stored in so[6]
*/
inline void s2_product_skf(s2_screwf_t* restrict si, const float* restrict ki,
                           s2_screwf_t* restrict so) {

  register float k = *ki;
  so->m0 = si->m0 * k;
  so->m1 = si->m1 * k;
  so->m2 = si->m2 * k;
  so->m3 = si->m3 * k;
  so->m4 = si->m4 * k;
  so->m5 = si->m5 * k;
}

/*!
@brief Add double precision s2_screw_ts s1i and s2i and store results in so.

\f$S_{1i} + S_{2i} \rightarrow S_o\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
inline void s2_add_ss(s2_screw_t* restrict s1i, s2_screw_t* s2i,
                      s2_screw_t* restrict so) {

  so->m0 = s1i->m0 + s2i->m0;
  so->m1 = s1i->m1 + s2i->m1;
  so->m2 = s1i->m2 + s2i->m2;
  so->m3 = s1i->m3 + s2i->m3;
  so->m4 = s1i->m4 + s2i->m4;
  so->m5 = s1i->m5 + s2i->m5;
}

/*!
@brief Add double precision s2_screw_ts s1i to s2i.

\f$S_{2i} += S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
inline void s2_addequal_ss(s2_screw_t* restrict s1i, s2_screw_t* restrict s2i) {

  s2i->m0 += s1i->m0;
  s2i->m1 += s1i->m1;
  s2i->m2 += s1i->m2;
  s2i->m3 += s1i->m3;
  s2i->m4 += s1i->m4;
  s2i->m5 += s1i->m5;
}

/*!
@brief Add double precision s2_screw_ts s1i to s2i.

\f$S_{2i} += S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
inline void s2_addequal_ssf(s2_screwf_t* restrict s1i,
                            s2_screwf_t* restrict s2i) {

  s2i->m0 += s1i->m0;
  s2i->m1 += s1i->m1;
  s2i->m2 += s1i->m2;
  s2i->m3 += s1i->m3;
  s2i->m4 += s1i->m4;
  s2i->m5 += s1i->m5;
}

/*!
@brief Add single precision s2_screw_ts s1i and s2i and store results in so.

\f$S_{1i} + S_{2i} \rightarrow S_o\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
inline void s2_add_ssf(s2_screwf_t* restrict s1i, s2_screwf_t* restrict s2i,
                       s2_screwf_t* restrict so) {

  so->m0 = s1i->m0 + s2i->m0;
  so->m1 = s1i->m1 + s2i->m1;
  so->m2 = s1i->m2 + s2i->m2;
  so->m3 = s1i->m3 + s2i->m3;
  so->m4 = s1i->m4 + s2i->m4;
  so->m5 = s1i->m5 + s2i->m5;
}

/*!
@brief Subtract double precision s2_screw_t s2i from s1i and store results in
so.

\f$S_{1i} - S_{2i} \rightarrow S_o\f$

@param s1i a first s2_screw_t
@param s2i a second s2_screw_t
@param so The results of s1i - s2i are stored in so
*/
inline void s2_subtract_ss(s2_screw_t* restrict s1i, s2_screw_t* restrict s2i,
                           s2_screw_t* restrict so) {

  so->m0 = s1i->m0 - s2i->m0;
  so->m1 = s1i->m1 - s2i->m1;
  so->m2 = s1i->m2 - s2i->m2;
  so->m3 = s1i->m3 - s2i->m3;
  so->m4 = s1i->m4 - s2i->m4;
  so->m5 = s1i->m5 - s2i->m5;
}

/*!
@brief Subtract single precision s2_screw_t s2i from s1i and store results in
so.

\f$S_{1i} - S_{2i} \rightarrow S_o\f$

@param s1i a first s2_screw_t
@param s2i a second s2_screw_t
@param so The results of s1i - s2i are stored in so
*/
inline void s2_subtract_ssf(s2_screwf_t* restrict s1i,
                            s2_screwf_t* restrict s2i,
                            s2_screwf_t* restrict so) {

  so->m0 = s1i->m0 - s2i->m0;
  so->m1 = s1i->m1 - s2i->m1;
  so->m2 = s1i->m2 - s2i->m2;
  so->m3 = s1i->m3 - s2i->m3;
  so->m4 = s1i->m4 - s2i->m4;
  so->m5 = s1i->m5 - s2i->m5;
}

/*!
@brief Add double precision s2_screw_ts s1i to s2i.

\f$S_{2i} -= S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
inline void s2_subtractequal_ss(s2_screw_t* restrict s1i,
                                s2_screw_t* restrict s2i) {

  s2i->m0 -= s1i->m0;
  s2i->m1 -= s1i->m1;
  s2i->m2 -= s1i->m2;
  s2i->m3 -= s1i->m3;
  s2i->m4 -= s1i->m4;
  s2i->m5 -= s1i->m5;
}

/*!
@brief Add double precision s2_screw_ts s1i to s2i.

\f$S_{2i} += S_{1i}\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param so The results of s1i + s2i are stored in so
*/
inline void s2_subtractequal_ssf(s2_screwf_t* restrict s1i,
                                 s2_screwf_t* restrict s2i) {

  s2i->m0 -= s1i->m0;
  s2i->m1 -= s1i->m1;
  s2i->m2 -= s1i->m2;
  s2i->m3 -= s1i->m3;
  s2i->m4 -= s1i->m4;
  s2i->m5 -= s1i->m5;
}

/*!
@}
@name Mixed 3x1 vector and 6x1 screw utilities

Utilities for performing operations on a combination of 3x1 vectors and
6x1 screws.
@{
*/

/*!
@brief Adds a double prevision s2_screw_t and Vector

\f$S_i[0:3] + V_i \rightarrow S_o[0:3]\f$

\f$S_i[3:6] \rightarrow S_o[3:6]\f$

@param si a 6x1 screw
@param vi a 3x1 vector
@param so the results of the operation are returned in so
*/
inline void s2_add_sst(s2_screw_t* restrict si1, s2_screw_t* restrict si2,
                       s2_screw_t* restrict so) {

  s2_screw_copy(si1, so);
  so->m0 += si2->m0;
  so->m1 += si2->m1;
  so->m2 += si2->m2;
}

/*!
@brief Adds a single prevision s2_screwf_t and Vectorf

\f$S_i[0:3] + V_i \rightarrow S_o[0:3]\f$

\f$S_i[3:6] \rightarrow S_o[3:6]\f$

@param si a 6x1 screw
@param vi a 3x1 vector
@param so the results of the operation are returned in so
*/
inline void s2_add_sstf(s2_screwf_t* restrict si1, s2_screwf_t* restrict si2,
                        s2_screwf_t* restrict so) {

  s2_screwf_copy(si1, so);
  so->m0 += si2->m0;
  so->m1 += si2->m1;
  so->m2 += si2->m2;
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
@brief Copy a double precision s2_mat3x3_t direction cosine matrix ri into ro.

\f$R_i \rightarrow R_o\f$

@param ri rotation matrix input
@param ro rotation matrix output
*/
inline void s2_mat3x3_copy(s2_mat3x3_t* restrict ri, s2_mat3x3_t* restrict ro) {
  memcpy(ro, ri, sizeof(s2_mat3x3_t));
}

/*!
@brief Copy a single precision s2_mat3x3f_t direction cosine matrix ri into ro.

\f$R_i \rightarrow R_o\f$

@param ri rotation matrix input
@param ro rotation matrix output
*/
inline void s2_mat3x3f_copy(s2_mat3x3f_t* restrict ri,
                            s2_mat3x3f_t* restrict ro) {
  memcpy(ro, ri, sizeof(s2_mat3x3f_t));
}

/*!
@brief Transposes a double precision s2_mat3x3_t matrix

\f$R_i^T \rightarrow R_o\f$

@param ri rotation matrix to be inverted
@param ro transposed rotation matrix returned in ro
*/
inline void s2_mat3x3_transpose(s2_mat3x3_t* restrict ri,
                                s2_mat3x3_t* restrict ro) {

  cblas_dcopy(3, (double*)ri, 1, (double*)ro, 3);
  cblas_dcopy(3, ((double*)ri) + 3, 1, ((double*)ro) + 1, 3);
  cblas_dcopy(3, ((double*)ri) + 6, 1, ((double*)ro) + 2, 3);
  // ro[0 * 3 + 0] = ri[0][0]; ro[0][1] = ri[1][0]; ro[0][2] = ri[2][0];
  // ro[1][0] = ri[0][1]; ro[1][1] = ri[1][1]; ro[1][2] = ri[2][1];
  // ro[2][0] = ri[0][2]; ro[2][1] = ri[1][2]; ro[2][2] = ri[2][2];
}

/*!
@brief Inverts (transposes) a single precision s2_mat3x3f_t rotation matrix

\f$R_i^T \rightarrow R_o\f$

@param ri rotation matrix to be inverted
@param ro transposed rotation matrix returned in ro
*/
inline void s2_mat3x3f_transpose(s2_mat3x3f_t* restrict ri,
                                 s2_mat3x3f_t* restrict ro) {
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
inline void s2_product_rv(s2_mat3x3_t* restrict ri, s2_vector_t* restrict vi,
                          s2_vector_t* restrict vo) {

  vo->x = ri->m00 * vi->x + ri->m01 * vi->y + ri->m02 * vi->z;
  vo->y = ri->m10 * vi->x + ri->m11 * vi->y + ri->m12 * vi->z;
  vo->z = ri->m20 * vi->x + ri->m21 * vi->y + ri->m22 * vi->z;
}

/*!
@brief Store product of direction cosine matrix ri with vector vi in vo.

\f$R_i * V_i \rightarrow V_o\f$
*/
inline void s2_product_rvf(s2_mat3x3f_t* restrict ri, s2_vectorf_t* vi,
                           s2_vectorf_t* restrict vo) {

  cblas_sgemv(CblasColMajor, CblasNoTrans, 3, 3, 1.0, (float*)ri, 3, (float*)vi,
              1, 0.0, (float*)vo, 1);
  // vo[0] =  ri[0][0] * vi[0] + ri[0][1] * vi[1] + ri[0][2] * vi[2];
  // vo[1] =  ri[1][0] * vi[0] + ri[1][1] * vi[1] + ri[1][2] * vi[2];
  // vo[2] =  ri[2][0] * vi[0] + ri[2][1] * vi[1] + ri[2][2] * vi[2];
}

/*!
@brief Store product of a double precision direction cosine matrix ri with
vector -vi in vo.

\f$R_i * -V_i \rightarrow V_o\f$
*/
inline void s2_product_rvinverse(s2_mat3x3_t* restrict ri,
                                 s2_vector_t* restrict vi,
                                 s2_vector_t* restrict vo) {

  cblas_dgemv(CblasColMajor, CblasNoTrans, 3, 3, -1.0, (double*)ri, 3,
              (double*)vi, 1, 0.0, (double*)vo, 1);
  // vo[0] = -ri[0][0] * vi[0] - ri[0][1] * vi[1] - ri[0][2] * vi[2];
  // vo[1] = -ri[1][0] * vi[0] - ri[1][1] * vi[1] - ri[1][2] * vi[2];
  // vo[2] = -ri[2][0] * vi[0] - ri[2][1] * vi[1] - ri[2][2] * vi[2];
}

/*!
@brief Store product of a single precision direction cosine matrix ri with
vector -vi in vo.

\f$R_i * -V_i \rightarrow V_o\f$
*/
inline void s2_product_rvinversef(s2_mat3x3f_t* restrict ri,
                                  s2_vectorf_t* restrict vi,
                                  s2_vectorf_t* restrict vo) {

  cblas_sgemv(CblasColMajor, CblasNoTrans, 3, 3, -1.0, (float*)ri, 3,
              (float*)vi, 1, 0.0, (float*)vo, 1);
  // vo[0] = -ri[0][0] * vi[0] - ri[0][1] * vi[1] - ri[0][2] * vi[2];
  // vo[1] = -ri[1][0] * vi[0] - ri[1][1] * vi[1] - ri[1][2] * vi[2];
  // vo[2] = -ri[2][0] * vi[0] - ri[2][1] * vi[1] - ri[2][2] * vi[2];
}

/*!
@brief Store product of a double precision direction cosine matrix ri^-1 with
vector vi in vo.

\f$R_i^T * V_i \rightarrow V_o\f$
*/
inline void s2_product_rinversev(s2_mat3x3_t* restrict ri,
                                 s2_vector_t* restrict vi,
                                 s2_vector_t* restrict vo) {

  cblas_dgemv(CblasColMajor, CblasTrans, 3, 3, 1.0, (double*)ri, 3, (double*)vi,
              1, 0.0, (double*)vo, 1);
  // vo[0] =  ri[0][0] * vi[0] + ri[1][0] * vi[1] + ri[2][0] * vi[2];
  // vo[1] =  ri[0][1] * vi[0] + ri[1][1] * vi[1] + ri[2][1] * vi[2];
  // vo[2] =  ri[0][2] * vi[0] + ri[1][2] * vi[1] + ri[2][2] * vi[2];
}

/*!
@brief Store product of a single precision direction cosine matrix ri^-1 with
vector vi in vo.

\f$R_i^T * V_i \rightarrow V_o\f$
*/
inline void s2_product_rinversevf(s2_mat3x3f_t* restrict ri,
                                  s2_vectorf_t* restrict vi,
                                  s2_vectorf_t* restrict vo) {

  cblas_sgemv(CblasColMajor, CblasTrans, 3, 3, 1.0, (float*)ri, 3, (float*)vi,
              1, 0.0, (float*)vo, 1);
  // vo[0] =  ri[0][0] * vi[0] + ri[1][0] * vi[1] + ri[2][0] * vi[2];
  // vo[1] =  ri[0][1] * vi[0] + ri[1][1] * vi[1] + ri[2][1] * vi[2];
  // vo[2] =  ri[0][2] * vi[0] + ri[1][2] * vi[1] + ri[2][2] * vi[2];
}

/*!
@brief Store product of a double precision direction cosine matrix ri^-1 with
vector -vi in vo.

\f$R_i^T * -V_i \rightarrow V_o\f$
*/
inline void s2_product_rinversevinverse(s2_mat3x3_t* restrict ri,
                                        s2_vector_t* restrict vi,
                                        s2_vector_t* restrict vo) {

  cblas_dgemv(CblasColMajor, CblasTrans, 3, 3, -1.0, (double*)ri, 3,
              (double*)vi, 1, 0.0, (double*)vo, 1);
}

/*!
@brief Store product of a double precision direction cosine matrix ri^-1 with
vector -vi in vo.

\f$R_i^T * -V_i \rightarrow V_o\f$
*/
inline void s2_product_rinversevinversef(s2_mat3x3f_t* restrict ri,
                                         s2_vectorf_t* restrict vi,
                                         s2_vectorf_t* restrict vo) {

  cblas_sgemv(CblasColMajor, CblasTrans, 3, 3, -1.0, (float*)ri, 3, (float*)vi,
              1, 0.0, (float*)vo, 1);
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
inline void s2_product_rs(s2_mat3x3_t* restrict ri, s2_screw_t* restrict si,
                          s2_screw_t* restrict so) {

  s2_product_rv(ri, (s2_vector_t*)si, (s2_vector_t*)so);
  s2_product_rv(ri, (s2_vector_t*)(((double*)si) + 3),
                (s2_vector_t*)(((double*)so) + 3));
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
inline void s2_product_rsf(s2_mat3x3f_t* restrict ri, s2_screwf_t* restrict si,
                           s2_screwf_t* restrict so) {

  s2_product_rvf(ri, (s2_vectorf_t*)si, (s2_vectorf_t*)so);
  s2_product_rvf(ri, (s2_vectorf_t*)(((float*)si) + 3),
                 (s2_vectorf_t*)(((float*)so) + 3));
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
inline void s2_product_rsinverse(s2_mat3x3_t* restrict ri,
                                 s2_screw_t* restrict si,
                                 s2_screw_t* restrict so) {

  s2_product_rvinverse(ri, (s2_vector_t*)si, (s2_vector_t*)so);
  s2_product_rvinverse(ri, (s2_vector_t*)(((double*)si) + 3),
                       (s2_vector_t*)(((double*)so) + 3));
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
inline void s2_product_rsinversef(s2_mat3x3f_t* restrict ri,
                                  s2_screwf_t* restrict si,
                                  s2_screwf_t* restrict so) {

  s2_product_rvinversef(ri, (s2_vectorf_t*)si, (s2_vectorf_t*)so);
  s2_product_rvinversef(ri, (s2_vectorf_t*)(((float*)si) + 3),
                        (s2_vectorf_t*)(((float*)so) + 3));
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
inline void s2_product_rinverses(s2_mat3x3_t* restrict ri,
                                 s2_screw_t* restrict si,
                                 s2_screw_t* restrict so) {

  s2_product_rinversev(ri, (s2_vector_t*)si, (s2_vector_t*)so);
  s2_product_rinversev(ri, (s2_vector_t*)(((double*)si) + 3),
                       (s2_vector_t*)(((double*)so) + 3));
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
inline void s2_product_rinversesf(s2_mat3x3f_t* restrict ri,
                                  s2_screwf_t* restrict si,
                                  s2_screwf_t* restrict so) {

  s2_product_rinversevf(ri, (s2_vectorf_t*)si, (s2_vectorf_t*)so);
  s2_product_rinversevf(ri, (s2_vectorf_t*)(((float*)si) + 3),
                        (s2_vectorf_t*)(((float*)so) + 3));
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
inline void s2_product_rrx(s2_mat3x3_t* restrict ri, double dc[2],
                           s2_mat3x3_t* restrict ro) {

  memset(ro, 0, 9 * sizeof(double));

  cblas_dcopy(3, (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 6, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 3, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0  0.0
  // 0.0| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0
  // dc[1] -dc[0]| |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|
  // |0.0  dc[0] dc[1]|

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
inline void s2_product_rrxf(s2_mat3x3f_t* restrict ri, float dc[2],
                            s2_mat3x3f_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3f_t));
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 6, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 3, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);
  cblas_scopy(3, (float*)ri, 1, (float*)ro, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0  0.0
  // 0.0| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0
  // dc[1] -dc[0]| |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|
  // |0.0  dc[0] dc[1]|

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
inline void s2_product_rrxinverse(s2_mat3x3_t* restrict ri, double dc[2],
                                  s2_mat3x3_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3_t));
  cblas_dcopy(3, (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 6, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 3, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0   0.0
  // 0.0
  // |
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0  dc[1]
  // dc[0]| |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0
  // -dc[0] dc[1]|

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
inline void s2_product_rrxinversef(s2_mat3x3f_t* restrict ri, float dc[2],
                                   s2_mat3x3f_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3f_t));
  cblas_scopy(3, (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 6, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 3, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |1.0   0.0
  // 0.0
  // |
  // |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |0.0  dc[1]
  // dc[0]| |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |0.0
  // -dc[0] dc[1]|

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
inline void s2_product_rry(s2_mat3x3_t* restrict ri, double dc[2],
                           s2_mat3x3_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 6, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, dc[0], (double*)ri, 1, ((double*)ro) + 6, 1);
  cblas_dcopy(3, ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   | dc[1] 0.0
  // dc[0]| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |
  // 0.0  1.0   0.0
  // |
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |-dc[0] 0.0
  // dc[1]|

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
inline void s2_product_rryf(s2_mat3x3f_t* restrict ri, float dc[2],
                            s2_mat3x3f_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 6, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, dc[0], (float*)ri, 1, ((float*)ro) + 6, 1);
  cblas_scopy(3, ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   | dc[1] 0.0
  // dc[0]| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |
  // 0.0  1.0   0.0
  // |
  // |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |-dc[0] 0.0
  // dc[1]|

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
inline void s2_product_rryinverse(s2_mat3x3_t* restrict ri, double dc[2],
                                  s2_mat3x3_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 6, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);
  cblas_daxpy(3, -dc[0], (double*)ri, 1, ((double*)ro) + 6, 1);
  cblas_dcopy(3, ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] 0.0
  // -dc[0]| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |
  // 0.0  1.0   0.0 | |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1]
  // ri[2][2]|   |dc[0] 0.0  dc[1]|

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
inline void s2_product_rryinversef(s2_mat3x3f_t* restrict ri, float dc[2],
                                   s2_mat3x3f_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 6, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);
  cblas_saxpy(3, -dc[0], (float*)ri, 1, ((float*)ro) + 6, 1);
  cblas_scopy(3, ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] 0.0
  // -dc[0]| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |
  // 0.0  1.0   0.0 | |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1]
  // ri[2][2]|   |dc[0] 0.0  dc[1]|

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
inline void s2_product_rrz(s2_mat3x3_t* restrict ri, double dc[2],
                           s2_mat3x3_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[0], ((double*)ri) + 3, 1, (double*)ro, 1);
  cblas_daxpy(3, -dc[0], (double*)ri, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_dcopy(3, ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] -dc[0]
  // 0.0| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |dc[0]
  // dc[1] 0.0| |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |
  // 0.0    0.0  1.0|

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
inline void s2_product_rrzf(s2_mat3x3f_t* restrict ri, float dc[2],
                            s2_mat3x3f_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[0], ((float*)ri) + 3, 1, (float*)ro, 1);
  cblas_saxpy(3, -dc[0], (float*)ri, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_scopy(3, ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);

  // |ro[0][0] ro[0][1] ro[0][2]|   |ri[0][0] ri[0][1] ri[0][2]|   |dc[1] -dc[0]
  // 0.0| |ro[1][0] ro[1][1] ro[1][2]| = |ri[1][0] ri[1][1] ri[1][2]| * |dc[0]
  // dc[1] 0.0| |ro[2][0] ro[2][1] ro[2][2]|   |ri[2][0] ri[2][1] ri[2][2]|   |
  // 0.0    0.0  1.0|

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
inline void s2_product_rrzinverse(s2_mat3x3_t* restrict ri, double dc[2],
                                  s2_mat3x3_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3_t));
  cblas_daxpy(3, dc[1], (double*)ri, 1, (double*)ro, 1);
  cblas_daxpy(3, -dc[0], ((double*)ri) + 3, 1, (double*)ro, 1);
  cblas_daxpy(3, dc[0], (double*)ri, 1, ((double*)ro) + 3, 1);
  cblas_daxpy(3, dc[1], ((double*)ri) + 3, 1, ((double*)ro) + 3, 1);
  cblas_dcopy(3, ((double*)ri) + 6, 1, ((double*)ro) + 6, 1);

  // |ro[0 * 3 + 0] ro[0 * 3 + 1] ro[0 * 3 + 2]|   |ri[0 * 3 + 0] ri[0 * 3 + 1]
  // ri[0
  // * 3 + 2]|   | dc[1] dc[0] 0.0|
  // |ro[1 * 3 + 0] ro[1 * 3 + 1] ro[1 * 3 + 2]| = |ri[1 * 3 + 0] ri[1 * 3 + 1]
  // ri[1
  // * 3 + 2]| * |-dc[0] dc[1] 0.0|
  // |ro[2 * 3 + 0] ro[2 * 3 + 1] ro[2 * 3 + 2]|   |ri[2 * 3 + 0] ri[2 * 3 + 1]
  // ri[2
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
inline void s2_product_rrzinversef(s2_mat3x3f_t* restrict ri, float dc[2],
                                   s2_mat3x3f_t* restrict ro) {

  memset(ro, 0, sizeof(s2_mat3x3f_t));
  cblas_saxpy(3, dc[1], (float*)ri, 1, (float*)ro, 1);
  cblas_saxpy(3, -dc[0], ((float*)ri) + 3, 1, (float*)ro, 1);
  cblas_saxpy(3, dc[0], (float*)ri, 1, ((float*)ro) + 3, 1);
  cblas_saxpy(3, dc[1], ((float*)ri) + 3, 1, ((float*)ro) + 3, 1);
  cblas_scopy(3, ((float*)ri) + 6, 1, ((float*)ro) + 6, 1);

  // |ro[0 * 3 + 0] ro[0 * 3 + 1] ro[0 * 3 + 2]|   |ri[0 * 3 + 0] ri[0 * 3 + 1]
  // ri[0
  // * 3 + 2]|   | dc[1] dc[0] 0.0|
  // |ro[1 * 3 + 0] ro[1 * 3 + 1] ro[1 * 3 + 2]| = |ri[1 * 3 + 0] ri[1 * 3 + 1]
  // ri[1
  // * 3 + 2]| * |-dc[0] dc[1] 0.0|
  // |ro[2 * 3 + 0] ro[2 * 3 + 1] ro[2 * 3 + 2]|   |ri[2 * 3 + 0] ri[2 * 3 + 1]
  // ri[2
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
inline void s2_product_rr(s2_mat3x3_t* restrict R1i, s2_mat3x3_t* restrict R2i,
                          s2_mat3x3_t* restrict ro) {

  cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0,
              (double*)R1i, 3, (double*)R2i, 3, 0.0, (double*)ro, 3);
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
inline void s2_product_rrf(s2_mat3x3f_t* restrict R1i,
                           s2_mat3x3f_t* restrict R2i,
                           s2_mat3x3f_t* restrict ro) {

  cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0,
              (float*)R1i, 3, (float*)R2i, 3, 0.0, (float*)ro, 3);
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
inline void s2_product_rrinverse(s2_mat3x3_t* restrict R1i,
                                 s2_mat3x3_t* restrict R2i,
                                 s2_mat3x3_t* restrict ro) {

  cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 3, 1.0,
              (double*)R1i, 3, (double*)R2i, 3, 0.0, (double*)ro, 3);

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
inline void s2_product_rrinversef(s2_mat3x3f_t* restrict R1i,
                                  s2_mat3x3f_t* restrict R2i,
                                  s2_mat3x3f_t* restrict ro) {

  cblas_sgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 3, 1.0,
              (float*)R1i, 3, (float*)R2i, 3, 0.0, (float*)ro, 3);

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
@brief Double precision product of the inverse of a general rotation matrix and
a general rotation matrix

Computes the product of the inverse of a general rotation matrix, R1i, and
a second rotation matrix, R2i.

\f$R_{1i}^T * R_{2i} \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void s2_product_rinverser(s2_mat3x3_t* restrict R1i,
                                 s2_mat3x3_t* restrict R2i,
                                 s2_mat3x3_t* restrict ro) {

  cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, 3, 3, 3, 1.0,
              (double*)R1i, 3, (double*)R2i, 3, 0.0, (double*)ro, 3);

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
@brief single precision product of the inverse of a general rotation matrix and
a general rotation matrix

Computes the product of the inverse of a general rotation matrix, R1i, and
a second rotation matrix, R2i.

\f$R_{1i}^T * R_{2i} \rightarrow R_o\f$

@param R1i a pointer to a general rotation matrix
@param R2i a pointer to a general rotation matrix
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void s2_product_rinverserf(s2_mat3x3f_t* restrict R1i,
                                  s2_mat3x3f_t* restrict R2i,
                                  s2_mat3x3f_t* restrict ro) {

  cblas_sgemm(CblasColMajor, CblasTrans, CblasNoTrans, 3, 3, 3, 1.0,
              (float*)R1i, 3, (float*)R2i, 3, 0.0, (float*)ro, 3);

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
@brief Double precision product of the a general rotation matrix and a primitive
translation along the x axis

Computes the product of a general rotation matrix and a primitive translation
along the x axis using sparse algebra.

\f$R_{i} * t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param to a pointer to a vector where result is stored.
*/
inline void s2_product_rtx(s2_mat3x3_t* restrict ri, const double* restrict ti,
                           s2_vector_t* restrict to) {

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
@brief single precision product of the a general rotation matrix and a primitive
translation along the x axis

Computes the product of a general rotation matrix and a primitive translation
along the x axis using sparse algebra.

\f$R_{i} * t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void s2_product_rtxf(s2_mat3x3f_t* restrict ri, const float* restrict ti,
                            s2_vectorf_t* restrict to) {

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
@brief Double precision product of the a general rotation matrix and the inverse
of a primitive translation along the x axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * -t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void s2_product_rtxinverse(s2_mat3x3_t* restrict ri,
                                  const double* restrict ti,
                                  s2_vector_t* restrict to) {

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
@brief single precision product of the a general rotation matrix and the inverse
of a primitive translation along the x axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{x} \rightarrow t_o\f$

is equivalent to:

\f$R_i[0,3,6] * -t_x \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void s2_product_rtxinversef(s2_mat3x3f_t* restrict ri,
                                   const float* restrict ti,
                                   s2_vectorf_t* restrict to) {

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
inline void s2_product_rty(s2_mat3x3_t* restrict ri, const double* restrict ti,
                           s2_vector_t* restrict to) {

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
inline void s2_product_rtyf(s2_mat3x3f_t* restrict ri, const float* restrict ti,
                            s2_vectorf_t* restrict to) {

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
inline void s2_product_rtyinverse(s2_mat3x3_t* restrict ri,
                                  const double* restrict ti,
                                  s2_vector_t* restrict to) {

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
@brief single precision product of a general rotation matrix and the inverse of
a primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a
primitive translation along the x axis using sparse algebra.

\f$R_{i} * -t_{y} \rightarrow t_o\f$

is equivalent to:

\f$R_i[1,4,7] * -t_y \rightarrow t_o[0:2]\f$

@param ri a pointer to a general rotation matrix
@param ti a pointer to a double
@param ro a pointer to a rotation matrix where the result is stored.
*/
inline void s2_product_rtyinversef(s2_mat3x3f_t* restrict ri,
                                   const float* restrict ti,
                                   s2_vectorf_t* restrict to) {

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
inline void s2_product_rtz(s2_mat3x3_t* restrict ri, const double* restrict ti,
                           s2_vector_t* restrict to) {

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
inline void s2_product_rtzf(s2_mat3x3f_t* restrict ri, const float* restrict ti,
                            s2_vectorf_t* restrict to) {

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
inline void s2_product_rtzinverse(s2_mat3x3_t* restrict ri,
                                  const double* restrict ti,
                                  s2_vector_t* restrict to) {

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
inline void s2_product_rtzinversef(s2_mat3x3f_t* restrict ri,
                                   const float* restrict ti,
                                   s2_vectorf_t* restrict to) {

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
inline void s2_add_ttx(s2_vector_t* restrict t1i, const double* restrict t2i,
                       s2_vector_t* restrict to) {

  s2_vector_copy(t1i, to);
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
inline void s2_add_ttxf(s2_vectorf_t* restrict t1i, const float* restrict t2i,
                        s2_vectorf_t* restrict to) {

  s2_vectorf_copy(t1i, to);
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
inline void s2_add_ttxinverse(s2_vector_t* restrict t1i,
                              const double* restrict t2i,
                              s2_vector_t* restrict to) {

  s2_vector_copy(t1i, to);
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
inline void s2_add_ttxinversef(s2_vectorf_t* restrict t1i,
                               const float* restrict t2i,
                               s2_vectorf_t* restrict to) {

  s2_vectorf_copy(t1i, to);
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
inline void s2_add_tty(s2_vector_t* restrict t1i, const double* restrict t2i,
                       s2_vector_t* restrict to) {

  s2_vector_copy(t1i, to);
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
inline void s2_add_ttyf(s2_vectorf_t* restrict t1i, const float* restrict t2i,
                        s2_vectorf_t* restrict to) {

  s2_vectorf_copy(t1i, to);
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
inline void s2_add_ttyinverse(s2_vector_t* restrict t1i,
                              const double* restrict t2i,
                              s2_vector_t* restrict to) {

  s2_vector_copy(t1i, to);
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
inline void s2_add_ttyinversef(s2_vectorf_t* restrict t1i,
                               const float* restrict t2i,
                               s2_vectorf_t* restrict to) {

  s2_vectorf_copy(t1i, to);
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
inline void s2_add_ttz(s2_vector_t* restrict t1i, const double* restrict t2i,
                       s2_vector_t* restrict to) {

  s2_vector_copy(t1i, to);
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
inline void s2_add_ttzf(s2_vectorf_t* restrict t1i, const float* restrict t2i,
                        s2_vectorf_t* restrict to) {

  s2_vectorf_copy(t1i, to);
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
inline void s2_add_ttzinverse(s2_vector_t* restrict t1i,
                              const double* restrict t2i,
                              s2_vector_t* restrict to) {

  s2_vector_copy(t1i, to);
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
inline void s2_add_ttzinversef(s2_vectorf_t* restrict t1i,
                               const float* restrict t2i,
                               s2_vectorf_t* restrict to) {

  s2_vectorf_copy(t1i, to);
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
@brief Compute double precision inverse of a s2_displacement_t

\f$D_i^{-1} \rightarrow D_o\f$

@param Di a general displacement to take the inverse of
@param Do inverse of Di is returned in Do
*/
inline void s2_displacement_invert(s2_displacement_t* restrict Di,
                                   s2_displacement_t* restrict Do) {

  s2_mat3x3_invert(&(Di->R), &(Do->R));
  s2_product_rinversevinverse(&(Di->R), &(Di->T), &(Do->T));
}

/*!
@brief Compute single precision inverse of a s2_displacement_t

\f$D_i^{-1} \rightarrow D_o\f$

@param Di a general displacement to take the inverse of
@param Do inverse of Di is returned in Do
*/
inline void s2_displacementf_invert(s2_displacementf_t* restrict Di,
                                    s2_displacementf_t* restrict Do) {

  s2_mat3x3f_invert(&(Di->R), &(Do->R));
  s2_product_rinversevinversef(&(Di->R), &(Di->T), &(Do->T));
}

/*!
@brief Copy a double precision s2_displacement_t consisting of a rotation and
translation

\f$D_i \rightarrow D_o\f$

@param Di a general displacement input
@param Do Di is copied to Do
*/
inline void s2_displacement_copy(s2_displacement_t* restrict Di,
                                 s2_displacement_t* restrict Do) {

  memcpy(Do, Di, sizeof(s2_displacement_t));
  // cblas_dcopy(12, (double*)Di, 1, (double*)Do, 1);
}

/*!
@brief Copy a single precision s2_displacementf_t consisting of a rotation and
translation

\f$D_i \rightarrow D_o\f$

@param Di a general displacement input
@param Do Di is copied to Do
*/
inline void s2_displacementf_copy(s2_displacementf_t* restrict Di,
                                  s2_displacementf_t* restrict Do) {

  memcpy(Do, Di, sizeof(s2_displacementf_t));
  // cblas_scopy(12, (float*)Di, 1, (float*)Do, 1);
}

/*!
@brief Double precision product of a s2_displacement_t with a primitive rotation
about the x-axis

\f$R_i * R_x \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a pointer to a general displacement
@param dc direction cosines characterizing a primitive rotation
@param the result of the computation is stored in Do
*/
inline void s2_product_drx(s2_displacement_t* restrict Di, double dc[2],
                           s2_displacement_t* restrict Do) {

  s2_product_rrx(&(Di->R), dc, &(Do->R));
  s2_vector_copy(&(Di->T), &(Do->T));
}

/*!
@brief single precision product of a s2_displacement_t with a primitive rotation
about the x-axis

\f$R_i * R_x \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a pointer to a general displacement
@param dc direction cosines characterizing a primitive rotation
@param the result of the computation is stored in Do
*/
inline void s2_product_drxf(s2_displacementf_t* restrict Di, float dc[2],
                            s2_displacementf_t* restrict Do) {

  s2_product_rrxf(&(Di->R), dc, &(Do->R));
  s2_vectorf_copy(&(Di->T), &(Do->T));
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
inline void s2_product_drxinverse(s2_displacement_t* restrict Di, double dc[2],
                                  s2_displacement_t* restrict Do) {

  s2_product_rrxinverse(&(Di->R), dc, &(Do->R));
  s2_vector_copy(&(Di->T), &(Do->T));
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
inline void s2_product_drxinversef(s2_displacementf_t* restrict Di, float dc[2],
                                   s2_displacementf_t* restrict Do) {

  s2_product_rrxinversef(&(Di->R), dc, &(Do->R));
  s2_vectorf_copy(&(Di->T), &(Do->T));
}

/*!
@brief Double precision product of a s2_displacement_t, \f$D\f$, and a rotation
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
inline void s2_product_dry(s2_displacement_t* restrict Di, double dc[2],
                           s2_displacement_t* restrict Do) {

  s2_product_rry(&(Di->R), dc, &(Do->R));
  s2_vector_copy(&(Di->T), &(Do->T));
}

/*!
@brief single precision product of a s2_displacementf_t, \f$D\f$, and a rotation
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
inline void s2_product_dryf(s2_displacementf_t* restrict Di, float dc[2],
                            s2_displacementf_t* restrict Do) {

  s2_product_rryf(&(Di->R), dc, &(Do->R));
  s2_vectorf_copy(&(Di->T), &(Do->T));
}

/*!
@brief Double precision product of a s2_displacement_t, \f$D\f$, and the inverse
of a rotation about the y axis, \f$R_y^T\f$

The following operation:
\f$D_i * R_y^T \rightarrow D_o\f$

Can be split in to:

\f$R_i * R_y^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Y axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void s2_product_dryinverse(s2_displacement_t* restrict Di, double dc[2],
                                  s2_displacement_t* restrict Do) {

  s2_product_rryinverse(&(Di->R), dc, &(Do->R));
  s2_vector_copy(&(Di->T), &(Do->T));
}

/*!
@brief single precision product of a s2_displacementf_t, \f$D\f$, and the
inverse of a rotation about the y axis, \f$R_y^T\f$

The following operation:
\f$D_i * R_y^T \rightarrow D_o\f$

Can be split in to:

\f$R_i * R_y^T \rightarrow R_o\f$

\f$T_i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param dc[2] direction cosines for a rotation about the Y axis.
@param Do the results are stored in a general displacement, \f$D_o\f$
*/
inline void s2_product_dryinversef(s2_displacementf_t* restrict Di, float dc[2],
                                   s2_displacementf_t* restrict Do) {

  s2_product_rryinversef(&(Di->R), dc, &(Do->R));
  s2_vectorf_copy(&(Di->T), &(Do->T));
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
inline void s2_product_drz(s2_displacement_t* restrict Di, double dc[2],
                           s2_displacement_t* restrict Do) {

  s2_product_rrz(&(Di->R), dc, &(Do->R));
  s2_vector_copy(&(Di->T), &(Do->T));
}

/*!
@brief single precision product of a s2_displacementf_t, \f$D\f$, and a
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
inline void s2_product_drzf(s2_displacementf_t* restrict Di, float dc[2],
                            s2_displacementf_t* restrict Do) {

  s2_product_rrzf(&(Di->R), dc, &(Do->R));
  s2_vectorf_copy(&(Di->T), &(Do->T));
}

/*!
@brief Double precision product of a s2_displacement_t, \f$D\f$, and an inverse
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
inline void s2_product_drzinverse(s2_displacement_t* restrict Di, double dc[2],
                                  s2_displacement_t* restrict Do) {

  s2_product_rrzinverse(&(Di->R), dc, &(Do->R));
  s2_vector_copy(&(Di->T), &(Do->T));
}

/*!
@brief single precision product of a s2_displacement_t, \f$D\f$, and an inverse
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
inline void s2_product_drzinversef(s2_displacementf_t* restrict Di, float dc[2],
                                   s2_displacementf_t* restrict Do) {

  s2_product_rrzinversef(&(Di->R), dc, &(Do->R));
  s2_vectorf_copy(&(Di->T), &(Do->T));
}

/*!
@brief Double precision product of a general s2_displacement_t with a vector

@param Di a general displacement, \f$D_i\f$
@param vi a general 3-d Vector
@param vo the results are stored in vector, \f$v_o\f$
*/
inline void s2_product_dv(s2_displacement_t* restrict Di,
                          s2_vector_t* restrict vi, s2_vector_t* restrict vo) {

  s2_product_rv(&(Di->R), vi, vo);

  vo->x += Di->T.x;
  vo->y += Di->T.y;
  vo->z += Di->T.z;
  // cblas_daxpy(
  //   3,
  //   1.0,
  //   ((double*)Di) + 9,
  //   1,
  //   (double*)vo,
  //   1);
}

/*!
@brief Double precision product of a general s2_displacement_t inverse
with a vector

@param Di a general displacement, \f$D\f$
@param vi a general 3-d Vector
@param vo the results are stored in vector, \f$v_o\f$
*/
inline void s2_product_dinversev(s2_displacement_t* restrict Di,
                                 s2_vector_t* restrict vi,
                                 s2_vector_t* restrict vo) {

  s2_vector_t temp;

  /* R^-1 * t -> temp*/
  s2_product_rinversev(&(Di->R), &(Di->T), &temp);

  /* R^-1 * vi -> vo */
  s2_product_rinversev(&(Di->R), vi, vo);

  /* vo -= temp */
  cblas_daxpy(3, -1.0, (double*)&temp, 1, (double*)vo, 1);
}

/*!
@brief Double precision product of a general s2_displacement_t, followed by
a 3D translation

Computes the result of a general displacement, followed by a general
3-d translation.

\f$R_i \rightarrow R_o\f$

\f$v1i + ri*v2i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param vi[3] a general 3-d Vector
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dt(s2_displacement_t* restrict Di,
                          s2_vector_t* restrict vi,
                          s2_displacement_t* restrict Do) {

  cblas_dcopy(9, (double*)Di, 1, (double*)Do, 1);
  s2_product_rv(&(Di->R), vi, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief Double precision product of a general s2_displacement_t, followed by
a 3D translation

Computes the result of a general displacement, followed by the inverse of
a general 3-d translation.

\f$R_i \rightarrow R_o\f$

\f$v1i + ri*v2i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param vi[3] a general 3-d Vector
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtinverse(s2_displacement_t* restrict Di,
                                 s2_vector_t* restrict vi,
                                 s2_displacement_t* restrict Do) {

  cblas_dcopy(9, (double*)Di, 1, (double*)Do, 1);
  s2_product_rvinverse(&(Di->R), vi, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief Product of a general s2_displacement_t and a displacement consisting
of only a translation component

Computes the result of a general displacement, followed by a general
3-d translation.

\f$R_i \rightarrow R_o\f$

\f$v1i + ri*v2i \rightarrow T_o\f$

@param Di a general displacement, \f$D_i\f$
@param vi[3] a general 3-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtf(s2_displacementf_t* restrict Di,
                           s2_vectorf_t* restrict vi,
                           s2_displacementf_t* restrict Do) {

  cblas_scopy(9, (float*)Di, 1, (float*)Do, 1);
  s2_product_rvf(&(Di->R), vi, &(Do->T));
  cblas_saxpy(3, 1.0, ((float*)Di) + 9, 1, ((float*)Do) + 9, 1);
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
inline void s2_product_dinverset(s2_displacement_t* restrict Di,
                                 s2_vector_t* restrict vi,
                                 s2_displacement_t* restrict Do) {

  s2_subtract_vv(vi, &(Di->T), &(Do->T));
  s2_mat3x3_invert(&(Di->R), &(Do->R));
  s2_product_rinversev(&(Di->R), &(Do->T), &(Do->T));
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
inline void s2_product_dinversetf(s2_displacementf_t* restrict Di,
                                  s2_vectorf_t* restrict vi,
                                  s2_displacementf_t* restrict Do) {

  s2_subtract_vvf(vi, &(Di->T), &(Do->T));
  s2_mat3x3f_invert(&(Di->R), &(Do->R));
  s2_product_rinversevf(&(Di->R), &(Do->T), &(Do->T));
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
inline void s2_product_dtx(s2_displacement_t* restrict Di,
                           const double* restrict ti,
                           s2_displacement_t* restrict Do) {

  s2_mat3x3_copy(&(Di->R), &(Do->R));
  s2_product_rtx(&(Di->R), ti, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief Double precision product of a s2_displacement_t and a primitive
translation

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_x \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtxinverse(s2_displacement_t* restrict Di,
                                  const double* restrict ti,
                                  s2_displacement_t* restrict Do) {

  s2_mat3x3_copy(&(Di->R), &(Do->R));
  s2_product_rtxinverse(&(Di->R), ti, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief single precision product of a s2_displacement_t and a primitive
translation

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_x \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtxinversef(s2_displacementf_t* restrict Di,
                                   const float* restrict ti,
                                   s2_displacementf_t* restrict Do) {

  s2_mat3x3f_copy(&(Di->R), &(Do->R));
  s2_product_rtxinversef(&(Di->R), ti, &(Do->T));
  cblas_saxpy(3, 1.0, ((float*)Di) + 9, 1, ((float*)Do) + 9, 1);
}

/*!
@brief Double precision general s2_displacement_t multiplied with a primitive
translation along the y-axis

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dty(s2_displacement_t* restrict Di,
                           const double* restrict ti,
                           s2_displacement_t* restrict Do) {

  s2_mat3x3_copy(&(Di->R), &(Do->R));
  s2_product_rty(&(Di->R), ti, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief single precision s2_displacementf_t multiplied with a primitive
translation along the y-axis

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtyf(s2_displacementf_t* restrict Di,
                            const float* restrict ti,
                            s2_displacementf_t* restrict Do) {

  s2_mat3x3f_copy(&(Di->R), &(Do->R));
  s2_product_rtyf(&(Di->R), ti, &(Do->T));
  cblas_saxpy(3, 1.0, ((float*)Di) + 9, 1, ((float*)Do) + 9, 1);
}

/*!
@brief Double precision s2_displacement_t multiplied with the inverse of a
primitive translation along y axis

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtyinverse(s2_displacement_t* restrict Di,
                                  const double* restrict ti,
                                  s2_displacement_t* restrict Do) {

  s2_mat3x3_copy(&(Di->R), &(Do->R));
  s2_product_rtyinverse(&(Di->R), ti, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief single precision s2_displacementf_t multiplied with the inverse of a
primitive translation along y axis

Computes the result of a general displacement, followed by a primitive
1-d translation.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_y \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtyinversef(s2_displacementf_t* restrict Di,
                                   const float* restrict ti,
                                   s2_displacementf_t* restrict Do) {

  s2_mat3x3f_copy(&(Di->R), &(Do->R));
  s2_product_rtyinversef(&(Di->R), ti, &(Do->T));
  cblas_saxpy(3, 1.0, ((float*)Di) + 9, 1, ((float*)Do) + 9, 1);
}

/*!
@brief Double precision s2_displacement_t multiplied with a primitive
translation along z axis

Computes the result of a general displacement, followed by a primitive
1-d translation along z axis.

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_z \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtz(s2_displacement_t* restrict Di,
                           const double* restrict ti,
                           s2_displacement_t* restrict Do) {

  s2_mat3x3_copy(&(Di->R), &(Do->R));
  s2_product_rtz(&(Di->R), ti, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
}

/*!
@brief single precision s2_displacementf_t multiplied with a primitive
translation along z axis

Computes the result of a general displacement, followed by a primitive
1-d translation along z axis.

\f$R_i \rightarrow R_o\f$

\f$t_i + R_i * t_z \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtzf(s2_displacementf_t* restrict Di,
                            const float* restrict ti,
                            s2_displacementf_t* restrict Do) {

  s2_mat3x3f_copy(&(Di->R), &(Do->R));
  s2_product_rtzf(&(Di->R), ti, &(Do->T));
  cblas_saxpy(3, 1.0, ((float*)Di) + 9, 1, ((float*)Do) + 9, 1);
}

/*!
@brief Double precision s2_displacement_t, multiplied with the inverse of a
primitive translation along z axis

Computes the result of a general displacement, followed by the inverse of a
primitive 1-d translation along the z axis.

\f$R_i \rightarrow R_o\f$

\f$t_i - R_i * t_z \rightarrow t_o\f$

@param Di a general displacement, \f$D_i\f$
@param ti a double representing a primitive 1-d translation
@param Do the results are stored in the general displacement, \f$D_o\f$
*/
inline void s2_product_dtzinverse(s2_displacement_t* restrict Di,
                                  const double* restrict ti,
                                  s2_displacement_t* restrict Do) {

  s2_mat3x3_copy(&(Di->R), &(Do->R));
  s2_product_rtzinverse(&(Di->R), ti, &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)Di) + 9, 1, ((double*)Do) + 9, 1);
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
inline void s2_product_dd(s2_displacement_t* restrict D1i,
                          s2_displacement_t* restrict D2i,
                          s2_displacement_t* restrict Do) {

  s2_product_rr(&(D1i->R), &(D2i->R), &(Do)->R);
  s2_product_rv(&(D1i->R), &(D2i->T), &(Do->T));
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
inline void s2_product_ddf(s2_displacementf_t* restrict D1i,
                           s2_displacementf_t* restrict D2i,
                           s2_displacementf_t* restrict Do) {

  s2_product_rrf(&(D1i->R), &(D2i->R), &(Do)->R);
  s2_product_rvf(&(D1i->R), &(D2i->T), &(Do->T));
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
inline void s2_product_dr(s2_displacement_t* restrict Di,
                          s2_mat3x3_t* restrict Ri,
                          s2_displacement_t* restrict Do) {

  s2_product_rr(&Di->R, Ri, &Do->R);
  s2_vector_copy(&Di->T, &Do->T);
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
inline void s2_product_drf(s2_displacementf_t* restrict Di,
                           s2_mat3x3f_t* restrict Ri,
                           s2_displacementf_t* restrict Do) {

  s2_product_rrf(&(Di->R), Ri, &(Do)->R);
  s2_vectorf_copy(&Di->T, &Do->T);
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
inline void s2_product_drinverse(s2_displacement_t* restrict Di,
                                 s2_mat3x3_t* restrict Ri,
                                 s2_displacement_t* restrict Do) {

  s2_product_rrinverse(&(Di->R), Ri, &(Do)->R);
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
inline void s2_product_drinversef(s2_displacementf_t* restrict Di,
                                  s2_mat3x3f_t* restrict Ri,
                                  s2_displacementf_t* restrict Do) {

  s2_product_rrinversef(&(Di->R), Ri, &(Do)->R);
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
inline void s2_product_ddinverse(s2_displacement_t* restrict D1i,
                                 s2_displacement_t* restrict D2i,
                                 s2_displacement_t* restrict Do) {

  s2_product_rrinverse(&(D1i->R), &(D2i->R), &(Do->R));
  s2_product_rvinverse(&(Do->R), &(D2i->T), &(Do->T));
  cblas_daxpy(3, 1.0, ((double*)D1i) + 9, 1, ((double*)Do) + 9, 1);
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
inline void s2_product_ddinversef(s2_displacementf_t* restrict D1i,
                                  s2_displacementf_t* restrict D2i,
                                  s2_displacementf_t* restrict Do) {

  s2_product_rrinversef(&(D1i->R), &(D2i->R), &(Do->R));
  s2_product_rvinversef(&(D1i->R), &(D2i->T), &(Do->T));
  cblas_saxpy(3, 1.0, ((float*)D1i) + 9, 1, ((float*)Do) + 9, 1);
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
inline void s2_htx(s2_displacement_t* restrict Di, s2_screw_t* restrict ho) {

  cblas_dcopy(3, (double*)Di, 1, (double*)ho, 1);
  // memset(ho + 3, 0, 3 * sizeof(double));
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
inline void s2_htxf(s2_displacementf_t* restrict Di, s2_screwf_t* restrict ho) {

  cblas_scopy(3, (float*)Di, 1, (float*)ho, 1);
  // memset(ho + 3, 0, 3 * sizeof(double));
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
inline void s2_hty(s2_displacement_t* restrict Di, s2_screw_t* restrict ho) {

  cblas_dcopy(3, ((double*)Di) + 3, 1, (double*)ho, 1);
  // memset(ho + 3, 0, 3 * sizeof(double));
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
inline void s2_htyf(s2_displacementf_t* restrict Di, s2_screwf_t* restrict ho) {

  cblas_scopy(3, ((float*)Di) + 3, 1, (float*)ho, 1);
  // memset(ho + 3, 0, 3 * sizeof(double));
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
inline void s2_htz(s2_displacement_t* restrict ri, s2_screw_t* restrict ho) {

  cblas_dcopy(3, ((double*)ri) + 6, 1, (double*)ho, 1);
  // memset(ho + 3, 0, 3 * sizeof(double));
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
inline void s2_htzf(s2_displacementf_t* restrict ri, s2_screwf_t* restrict ho) {

  cblas_scopy(3, ((float*)ri) + 6, 1, (float*)ho, 1);
  // memset(ho + 3, 0, 3 * sizeof(double));
}

/*!
@brief Double precision influence coefficient matrix for a
primitive rotation about x-axis

Computes the influence coefficient matrix for a rotation about x-axis.

\f$D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void s2_hrx(s2_displacement_t* restrict Di, s2_screw_t* restrict ho) {

  s2_htx(Di, (s2_screw_t*)(((double*)ho) + 3));
  s2_cross_vv(&(Di->T), (s2_vector_t*)(((double*)ho) + 3), (s2_vector_t*)ho);
}

/*!
@brief single precision influence coefficient matrix for a
primitive rotation about x-axis

Computes the influence coefficient matrix for a rotation about x-axis.

\f$D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void s2_hrxf(s2_displacementf_t* restrict Di, s2_screwf_t* restrict ho) {

  s2_htxf(Di, (s2_screwf_t*)(((float*)ho) + 3));
  s2_cross_vvf(&(Di->T), (s2_vectorf_t*)(((float*)ho) + 3), (s2_vectorf_t*)ho);
}

/*!
@brief Double precision influence coefficient matrix for a primitive
rotation about y-axis

Computes the influence coefficient matrix for a rotation about y-axis.

\f$D[1,4,7] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void s2_hry(s2_displacement_t* restrict Di, s2_screw_t* restrict ho) {

  s2_hty(Di, (s2_screw_t*)(((double*)ho) + 3));
  s2_cross_vv(&(Di->T), (s2_vector_t*)(((double*)ho) + 3), (s2_vector_t*)ho);
}

/*!
@brief Double precision influence coefficient matrix for a primitive rotation
about z-axis

Computes the influence coefficient matrix for a rotation about z-axis.

\f$D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void s2_hrz(s2_displacement_t* restrict Di, s2_screw_t* restrict ho) {

  s2_htz(Di, (s2_screw_t*)(((double*)ho) + 3));
  s2_cross_vv(&(Di->T), (s2_vector_t*)(((double*)ho) + 3), (s2_vector_t*)ho);
}

/*!
@brief single precision influence coefficient matrix for a primitive rotation
about z-axis

Computes the influence coefficient matrix for a rotation about z-axis.

\f$D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void s2_hrzf(s2_displacementf_t* restrict Di, s2_screwf_t* restrict ho) {

  s2_htzf(Di, (s2_screwf_t*)(((float*)ho) + 3));
  s2_cross_vvf(&(Di->T), (s2_vectorf_t*)(((float*)ho) + 3), (s2_vectorf_t*)ho);
}

/*!
@brief Double precision influence coefficient matrix for inverse primitive
translation along x-axis

Computes the influence coefficient matrix for an inverse primitive translation
along the x-axis.

\f$-D[0,3,6] \rightarrow h_{tx}[0:2]\f$

Note that the last three elements of ho will always be zero, so the following
operation need not be computed if ho is d to zero.

\f$0 \rightarrow h_{tx}[3:5]\f$
*/
inline void s2_htxinverse(s2_displacement_t* restrict Di,
                          s2_screw_t* restrict ho) {

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
inline void s2_htxinversef(s2_displacementf_t* restrict Di,
                           s2_screwf_t* restrict ho) {

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
inline void s2_htyinverse(s2_displacement_t* restrict Di,
                          s2_screw_t* restrict ho) {

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
inline void s2_htyinversef(s2_displacementf_t* restrict Di,
                           s2_screwf_t* restrict ho) {

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
inline void s2_htzinverse(s2_displacement_t* restrict Di,
                          s2_screw_t* restrict ho) {

  memset(ho, 0, 3 * sizeof(double));
  cblas_daxpy(3, -1, ((double*)Di) + 6, 1, (double*)ho, 1);
  // ho[0] = -ri[0 * 3 + 2];
  // ho[1] = -ri[1 * 3 + 2];
  // ho[2] = -ri[2 * 3 + 2];
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
inline void s2_htzinversef(s2_displacementf_t* restrict Di,
                           s2_screwf_t* restrict ho) {

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
inline void s2_hrxinverse(s2_displacement_t* restrict Di,
                          s2_screw_t* restrict ho) {

  s2_htxinverse(Di, (s2_screw_t*)(((double*)ho) + 3));
  s2_cross_vv(&(Di->T), (s2_vector_t*)(((double*)ho) + 3), (s2_vector_t*)ho);
}

/*!
@brief single precision influence coefficient matrix for inverse of a primitive
rotation about x-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about x-axis.

\f$-D[0,3,6] \rightarrow h_{tx}[3:5]\f$

\f$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]\f$
*/
inline void s2_hrxinversef(s2_displacementf_t* restrict Di,
                           s2_screwf_t* restrict ho) {

  s2_htxinversef(Di, (s2_screwf_t*)(((float*)ho) + 3));
  s2_cross_vvf(&(Di->T), (s2_vectorf_t*)(((float*)ho) + 3), (s2_vectorf_t*)ho);
}

/*!
@brief Double precision influence coefficient matrix for inverse of a primitive
rotation about y-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about y-axis.

\f$-D[1,4,7] \rightarrow h_{ty}[3:5]\f$

\f$T_i \times h_{ty}[3:5] \rightarrow h_{ry}[0:2]\f$
*/
inline void s2_hryinverse(s2_displacement_t* restrict Di,
                          s2_screw_t* restrict ho) {

  s2_htyinverse(Di, (s2_screw_t*)((double*)ho + 3));
  s2_cross_vv(&(Di->T), (s2_vector_t*)(((double*)ho) + 3), (s2_vector_t*)ho);
}

/*!
@brief single precision influence coefficient matrix for inverse of a primitive
rotation about y-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about y-axis.

\f$-D[1,4,7] \rightarrow h_{ty}[3:5]\f$

\f$T_i \times h_{ty}[3:5] \rightarrow h_{ry}[0:2]\f$
*/
inline void s2_hryinversef(s2_displacementf_t* restrict Di,
                           s2_screwf_t* restrict ho) {

  s2_htyinversef(Di, (s2_screwf_t*)(((float*)ho) + 3));
  s2_cross_vvf(&(Di->T), (s2_vectorf_t*)(((float*)ho) + 3), (s2_vectorf_t*)ho);
}

/*!
@brief Double precision influence coefficient matrix for inverse of a
primitive rotation about z-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about z-axis.

\f$-D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tz}[3:5] \rightarrow h_{rz}[0:2]\f$
*/
inline void s2_hrzinverse(s2_displacement_t* restrict Di,
                          s2_screw_t* restrict ho) {

  s2_htzinverse(Di, (s2_screw_t*)(((double*)ho) + 3));
  s2_cross_vv(&(Di->T), (s2_vector_t*)(((double*)ho) + 3), (s2_vector_t*)ho);
}

/*!
@brief single precision influence coefficient matrix for inverse of a
primitive rotation about z-axis

Computes the influence coefficient matrix for the inverse of a primitive
rotation about z-axis.

\f$-D[2,5,8] \rightarrow h_{tz}[3:5]\f$

\f$T_i \times h_{tz}[3:5] \rightarrow h_{rz}[0:2]\f$
*/
inline void s2_hrzinversef(s2_displacementf_t* restrict Di,
                           s2_screwf_t* restrict ho) {

  s2_htzinversef(Di, (s2_screwf_t*)(((float*)ho) + 3));
  s2_cross_vvf(&(Di->T), (s2_vectorf_t*)(((float*)ho) + 3), (s2_vectorf_t*)ho);
}

/*!
@}
@name Utilities for transforming quantities in configuration space
Quantities in configuration space include spatial quantities that can be
be measured with a ruler or clock. s2_screw_t quantities in configuration space
are often referred to as screws/twists.
@{
*/

/*!
@brief Compute double precision s2_screw_t Cross product in configuration space

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$S_{1i}[0:2] \times S_{2i}[0:2] + S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow
S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void s2_screwcross_ss(s2_screw_t* restrict s1i, s2_screw_t* restrict s2i,
                             s2_screw_t* restrict so) {

  /* vo <- w1i x v2i */
  s2_cross_vv((s2_vector_t*)(((double*)s1i) + 3), (s2_vector_t*)s2i,
              (s2_vector_t*)(((double*)so)));

  /* wo <- w1i x w2i */
  s2_cross_vv((s2_vector_t*)(((double*)s1i) + 3),
              (s2_vector_t*)(((double*)s2i) + 3),
              (s2_vector_t*)(((double*)so) + 3));

  /* vo += v1i x w2i */
  so->m0 += s1i->m1 * s2i->m5 - s1i->m2 * s2i->m4;
  so->m1 += s1i->m2 * s2i->m3 - s1i->m0 * s2i->m5;
  so->m2 += s1i->m0 * s2i->m4 - s1i->m1 * s2i->m3;
}

/*!
@brief Compute single precision s2_screwf_t Cross product in configuration space

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$S_{1i}[0:2] \times S_{2i}[0:2] + S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow
S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void s2_screwcross_ssf(s2_screwf_t* restrict s1i,
                              s2_screwf_t* restrict s2i,
                              s2_screwf_t* restrict so) {

  /* vo <- w1i x v2i */
  s2_cross_vvf((s2_vectorf_t*)(((float*)s1i) + 3), (s2_vectorf_t*)s2i,
               (s2_vectorf_t*)(((float*)so)));

  /* wo <- w1i x w2i */
  s2_cross_vvf((s2_vectorf_t*)(((float*)s1i) + 3),
               (s2_vectorf_t*)(((float*)s2i) + 3),
               (s2_vectorf_t*)(((float*)so) + 3));

  /* vo += v1i x w2i */
  so->m0 += s1i->m1 * s2i->m5 - s1i->m2 * s2i->m4;
  so->m1 += s1i->m2 * s2i->m3 - s1i->m0 * s2i->m5;
  so->m2 += s1i->m0 * s2i->m4 - s1i->m1 * s2i->m3;
}

/*!
@brief Compute a double precision s2_screw_t Cross product in configuration
space Compute s2_screw_t Cross product in configuration space, where the second
screw consists of only a translation component.

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$0 \rightarrow S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void s2_screwcross_sst(s2_screw_t* restrict s1i,
                              s2_screw_t* restrict s2i,
                              s2_screw_t* restrict so) {

  s2_cross_vv((s2_vector_t*)(((double*)s1i) + 3), (s2_vector_t*)s2i,
              (s2_vector_t*)so);
  memset(((double*)so) + 3, 0, 3 * sizeof(double));
}

/*!
@brief Compute a single precision s2_screwf_t Cross product in configuration
space Compute s2_screw_t Cross product in configuration space, where the second
screw consists of only a translation component.

\f$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]\f$

\f$0 \rightarrow S_o[3:5]\f$

@param s1i a first 6x1 screw
@param s2i a second 6x1 screw
@param The results of s1i crossed with s2i are stored in so
*/
inline void s2_screwcross_sstf(s2_screwf_t* restrict s1i,
                               s2_screwf_t* restrict s2i,
                               s2_screwf_t* restrict so) {

  s2_cross_vvf((s2_vectorf_t*)(((float*)s1i) + 3), (s2_vectorf_t*)s2i,
               (s2_vectorf_t*)so);
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
inline void s2_screwproduct_dts(s2_vector_t* restrict ti,
                                s2_screw_t* restrict si,
                                s2_screw_t* restrict so) {

  s2_screw_copy(si, so);

  /* vo += ti x w2i */
  so->m0 += ti->y * si->m5 - ti->z * si->m4;
  so->m1 += ti->z * si->m3 - ti->x * si->m5;
  so->m2 += ti->x * si->m4 - ti->y * si->m3;
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
inline void s2_screwproduct_dtsf(s2_vectorf_t* restrict ti,
                                 s2_screwf_t* restrict si,
                                 s2_screwf_t* restrict so) {

  s2_screwf_copy(si, so);

  /* vo += ti x w2i */
  so->m0 += ti->y * si->m5 - ti->z * si->m4;
  so->m1 += ti->z * si->m3 - ti->x * si->m5;
  so->m2 += ti->x * si->m4 - ti->y * si->m3;
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
inline void s2_screwproduct_dtinverses(s2_vector_t* restrict ti,
                                       s2_screw_t* restrict si,
                                       s2_screw_t* restrict so) {

  s2_screw_copy(si, so);

  /* vo -= ti x w2i */
  so->m0 -= ti->y * si->m5 - ti->z * si->m4;
  so->m1 -= ti->z * si->m3 - ti->x * si->m5;
  so->m2 -= ti->x * si->m4 - ti->y * si->m3;
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
inline void s2_screwproduct_dtinversesf(s2_vectorf_t* restrict ti,
                                        s2_screwf_t* restrict si,
                                        s2_screwf_t* restrict so) {

  s2_screwf_copy(si, so);

  /* vo -= ti x w2i */
  so->m0 -= ti->y * si->m5 - ti->z * si->m4;
  so->m1 -= ti->z * si->m3 - ti->x * si->m5;
  so->m2 -= ti->x * si->m4 - ti->y * si->m3;
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
inline void s2_screwproduct_dtsinverse(s2_vector_t* restrict ti,
                                       s2_screw_t* restrict si,
                                       s2_screw_t* restrict so) {

  s2_screw_invert(si, so);

  /* vo -= ti x w2i */
  so->m0 -= ti->y * si->m5 - ti->z * si->m4;
  so->m1 -= ti->z * si->m3 - ti->x * si->m5;
  so->m2 -= ti->x * si->m4 - ti->y * si->m3;
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
inline void s2_screwproduct_dtsinversef(s2_vectorf_t* restrict ti,
                                        s2_screwf_t* restrict si,
                                        s2_screwf_t* restrict so) {

  s2_screwf_invert(si, so);

  /* vo -= ti x w2i */
  so->m0 -= ti->y * si->m5 - ti->z * si->m4;
  so->m1 -= ti->z * si->m3 - ti->x * si->m5;
  so->m2 -= ti->x * si->m4 - ti->y * si->m3;
}

/*!
@brief Double precision displace screw si by general displacement
and store in so.

\f$D_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void s2_screwproduct_ds(s2_displacement_t* restrict Di,
                               s2_screw_t* restrict si,
                               s2_screw_t* restrict so) {

  s2_product_rs(&(Di->R), si, so);

  so->m0 += Di->T.y * so->m5 - Di->T.z * so->m4;
  so->m1 += Di->T.z * so->m3 - Di->T.x * so->m5;
  so->m2 += Di->T.x * so->m4 - Di->T.y * so->m3;
}

/*!
@brief Double precision displace negative screw si by general displacement
and store in so.

\f$D_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void s2_screwproduct_dsinverse(s2_displacement_t* restrict Di,
                                      s2_screw_t* restrict si,
                                      s2_screw_t* restrict so) {

  s2_product_rsinverse(&(Di->R), si, so);
  so->m0 += Di->T.y * so->m5 - Di->T.z * so->m4;
  so->m1 += Di->T.z * so->m3 - Di->T.x * so->m5;
  so->m2 += Di->T.x * so->m4 - Di->T.y * so->m3;
}

/*!
@brief single precision displace screw si by general displacement
and store in Do.

\f$D_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void s2_screwproduct_dsf(s2_displacementf_t* restrict Di,
                                s2_screwf_t* restrict si,
                                s2_screwf_t* restrict so) {

  s2_product_rsf(&(Di->R), si, so);

  so->m0 += Di->T.y * so->m5 - Di->T.z * so->m4;
  so->m1 += Di->T.z * so->m3 - Di->T.x * so->m5;
  so->m2 += Di->T.x * so->m4 - Di->T.y * so->m3;
}

/*!
@brief In configuration space, double precision displace screw si by ti^-1
& ri^-1 and store in so.

\f$D_i^{-1} * S_i \rightarrow S_o\f$
@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void s2_screwproduct_dinverses(s2_displacement_t* restrict Di,
                                      s2_screw_t* restrict si,
                                      s2_screw_t* restrict so) {

  s2_screw_t temp;
  s2_screw_copy(si, &temp);

  temp.m0 -= Di->T.y * temp.m5 - Di->T.z * temp.m4;
  temp.m1 -= Di->T.z * temp.m3 - Di->T.x * temp.m5;
  temp.m2 -= Di->T.x * temp.m4 - Di->T.y * temp.m3;

  s2_product_rinverses(&Di->R, &temp, so);
}

/*!
@brief In configuration space, single precision displace screw si by ti^-1
& ri^-1 and store in so.

\f$D_i^{-1} * S_i \rightarrow S_o\f$
@param Di a general displacement
@param si a screw to transform
@param so the results are stored in so
*/
inline void s2_screwproduct_dinversesf(s2_displacementf_t* restrict Di,
                                       s2_screwf_t* restrict si,
                                       s2_screwf_t* restrict so) {
  s2_screwf_t temp;
  s2_screwf_copy(si, &temp);

  temp.m0 -= Di->T.y * temp.m5 - Di->T.z * temp.m4;
  temp.m1 -= Di->T.z * temp.m3 - Di->T.x * temp.m5;
  temp.m2 -= Di->T.x * temp.m4 - Di->T.y * temp.m3;

  s2_product_rinversesf(&Di->R, &temp, so);
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

\f$ S_{1i}[3:5]\times S_{2i}[0:2] + S_{1i}[0:2] \times S_{2i}[3:5] \rightarrow
S_o[0:2]\f$

\f$ S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]\f$

@param s1i a first coscrew
@param s2i a second coscrew
@param so the results are stored in so
*/
inline void s2_coscrewcross_ss(s2_screw_t* restrict s1i,
                               s2_screw_t* restrict s2i,
                               s2_screw_t* restrict so) {

  /* vo <- w1 x v2 */
  s2_cross_vv((s2_vector_t*)(((double*)s1i) + 3), (s2_vector_t*)s2i,
              (s2_vector_t*)so);

  /* wo <- v1 x v2 */
  s2_cross_vv((s2_vector_t*)s1i, (s2_vector_t*)((double*)s2i),
              (s2_vector_t*)(((double*)so) + 3));

  /* wo += w1 x w2 */
  so->m3 += s1i->m4 * s2i->m5 - s1i->m5 * s2i->m4;
  so->m4 += s1i->m5 * s2i->m3 - s1i->m3 * s2i->m5;
  so->m5 += s1i->m3 * s2i->m4 - s1i->m4 * s2i->m3;
}

/*!
@brief In function space, store single precision spatial cross product of s1i
with s2i in so.

\f$ S_{1i}[3:5]\times S_{2i}[0:2] + S_{1i}[0:2] \times S_{2i}[3:5] \rightarrow
S_o[0:2]\f$

\f$ S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]\f$

@param s1i a first coscrew
@param s2i a second coscrew
@param so the results are stored in so
*/
inline void s2_coscrewcross_ssf(s2_screwf_t* restrict s1i,
                                s2_screwf_t* restrict s2i,
                                s2_screwf_t* restrict so) {

  /* vo <- w1 x v2 */
  s2_cross_vvf((s2_vectorf_t*)(((float*)s1i) + 3), (s2_vectorf_t*)s2i,
               (s2_vectorf_t*)so);

  /* wo <- v1 x v2 */
  s2_cross_vvf((s2_vectorf_t*)s1i, (s2_vectorf_t*)s2i,
               (s2_vectorf_t*)(((float*)so) + 3));

  /* wo += w1 x w2 */
  so->m3 += s1i->m4 * s2i->m5 - s1i->m5 * s2i->m4;
  so->m4 += s1i->m5 * s2i->m3 - s1i->m3 * s2i->m5;
  so->m5 += s1i->m3 * s2i->m4 - s1i->m4 * s2i->m3;
}

/*!
@brief In function space displace spatial vector si by ti^T and store in so.

\f$S_i[3:5] - T_i \times S_i[0:2] \rightarrow S_o[3:5] \f$

\f$S_i[0:2] \rightarrow S_o[0:2]\f$

@param ti a 3-d translation
@param si a coscrew
@param so the results are stored in so
*/
inline void s2_coscrewproduct_dts(s2_vector_t* restrict ti,
                                  s2_screw_t* restrict si,
                                  s2_screw_t* restrict so) {

  s2_screw_copy(si, so);

  /* wo += w1 x w2 */
  so->m3 += ti->y * si->m2 - ti->z * si->m1;
  so->m4 += ti->z * si->m0 - ti->x * si->m2;
  so->m5 += ti->x * si->m1 - ti->y * si->m0;
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
inline void s2_coscrewproduct_dtsf(s2_vectorf_t* restrict ti,
                                   s2_screwf_t* restrict si,
                                   s2_screwf_t* restrict so) {

  s2_screwf_copy(si, so);

  /* wo += w1 x w2 */
  so->m3 += ti->y * si->m2 - ti->z * si->m1;
  so->m4 += ti->z * si->m0 - ti->x * si->m2;
  so->m5 += ti->x * si->m1 - ti->y * si->m0;
}

/*!
@brief In function space displace spatial vector si by ti^-T and store in so.

\f$ S_i[3:5] + T_i \times S_i[0:2] \rightarrow S_o[3:5]\f$

\f$ S_i[0:2] \rightarrow S_o[0:2] \f$
*/
inline void s2_coscrewproduct_dtsinverse(s2_vector_t* restrict ti,
                                         s2_screw_t* restrict si,
                                         s2_screw_t* restrict so) {

  s2_screw_t temp;
  s2_screw_invert(si, &temp);
  s2_coscrewproduct_dts(ti, &temp, so);

  /* wo -= w1 x w2 */
  // so->m3 -= ti->y * si->m2 - ti->z * si->m1;
  // so->m4 -= ti->z * si->m0 - ti->x * si->m2;
  // so->m5 -= ti->x * si->m1 - ti->y * si->m0;
}

/*!
@brief In function space displace single precision spatial vector si by ti^-T
and store in so.

\f$ S_i[3:5] + T_i \times S_i[0:2] \rightarrow S_o[3:5]\f$

\f$ S_i[0:2] \rightarrow S_o[0:2] \f$
*/
inline void s2_coscrewproduct_dtsinversef(s2_vectorf_t* restrict ti,
                                          s2_screwf_t* restrict si,
                                          s2_screwf_t* restrict so) {

  s2_screwf_t temp;
  s2_screwf_invert(si, &temp);
  s2_coscrewproduct_dtsf(ti, &temp, so);

  /* previously was */
  // s2_screwf_copy(si, so);
  //
  // /* wo -= w1 x w2 */
  // so->m3 -= ti->y * si->m2 - ti->z * si->m1;
  // so->m4 -= ti->z * si->m0 - ti->x * si->m2;
  // so->m5 -= ti->x * si->m1 - ti->y * si->m0;
}

/*!
@brief In function space displace spatial vector si by ti^T and ri^-1 and store
in so.

\f$ R_i^{-1} * T_i^T * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void s2_coscrewproduct_ds(s2_displacement_t* restrict Di,
                                 s2_screw_t* restrict si,
                                 s2_screw_t* restrict so) {

  s2_product_rs(&(Di->R), si, so);

  so->m3 += Di->T.y * so->m2 - Di->T.z * so->m1;
  so->m4 += Di->T.z * so->m0 - Di->T.x * so->m2;
  so->m5 += Di->T.x * so->m1 - Di->T.y * so->m0;
}

/*!
@brief In function space displace single precision spatial vector si by ti^T
and ri^-1 and store in so.

\f$ R_i^{-1} * T_i^T * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void s2_coscrewproduct_dsf(s2_displacementf_t* restrict Di,
                                  s2_screwf_t* restrict si,
                                  s2_screwf_t* restrict so) {

  s2_product_rsf(&(Di->R), si, so);

  so->m3 += Di->T.y * so->m2 - Di->T.z * so->m1;
  so->m4 += Di->T.z * so->m0 - Di->T.x * so->m2;
  so->m5 += Di->T.x * so->m1 - Di->T.y * so->m0;
}

/*!
@brief In function space, displace spatial vector si by ri and ti^-T and store
in so.

\f$ T_i^{-T} * R_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void s2_coscrewproduct_dinverses(s2_displacement_t* restrict Di,
                                        s2_screw_t* restrict si,
                                        s2_screw_t* restrict so) {

  s2_screw_t temp;
  s2_screw_copy(si, &temp);

  temp.m3 -= Di->T.y * temp.m2 - Di->T.z * temp.m1;
  temp.m4 -= Di->T.z * temp.m0 - Di->T.x * temp.m2;
  temp.m5 -= Di->T.x * temp.m1 - Di->T.y * temp.m0;

  s2_product_rinverses(&(Di->R), &temp, so);
}

/*!
@brief In function space, displace single precision spatial vector si by ri
and ti^-T and store in so.

\f$ T_i^{-T} * R_i * S_i \rightarrow S_o\f$

@param Di a general displacement
@param si a coscrew
@param so the results are stored in so
*/
inline void s2_coscrewproduct_dinversesf(s2_displacementf_t* restrict Di,
                                         s2_screwf_t* restrict si,
                                         s2_screwf_t* restrict so) {

  s2_screwf_t temp;
  s2_screwf_copy(si, &temp);

  temp.m3 -= Di->T.y * temp.m2 - Di->T.z * temp.m1;
  temp.m4 -= Di->T.z * temp.m0 - Di->T.x * temp.m2;
  temp.m5 -= Di->T.x * temp.m1 - Di->T.y * temp.m0;

  s2_product_rinversesf(&(Di->R), &temp, so);
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
void s2_quaternion_normalize(s2_quaternion_t* restrict qi) {

  double norm = sqrt(cblas_ddot(4, (double*)qi, 1, (double*)qi, 1));
  cblas_dscal(4, 1.0 / norm, (double*)qi, 1);
}

/*!
@brief Normalize single precision quaternion

Normalize a quaternion and overwrite.

@param qi euler parameters to normalize
*/
void s2_quaternionf_normalize(s2_quaternionf_t* restrict qi) {

  float norm = sqrt(cblas_sdot(4, (float*)qi, 1, (float*)qi, 1));
  cblas_sscal(4, 1.0 / norm, (float*)qi, 1);
}

/*!
@brief Double precision function to convert s2_mat3x3_t rotation matrix to
quaternion representation

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 &
e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 +
e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 &
e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}\f$

@todo Supply *ER*, a reference Euler Parameter. If necessary, flip the sign of
*(E) so that each *(E+i) is the closest match to ER[i]. For this algorithm to
work, *(E) must be a close follow-on to ER. This function would then prove more
useful when continuous Euler Parameters are desired for interpolation purposes.
Matrix R is quadratic in *(E) so *(E+i) and -*(E+i) give the same R.

@param ri a rotation matrix in a 9x1 column format
@param qo the resulting euler parameters are stored in the 4x1 array, qo
*/
void s2_mat3x3_toQuaternion(s2_mat3x3_t* restrict ri,
                            s2_quaternion_t* restrict qo) {

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

      qo->x = (((double*)ri)[2 + 1 * 3] - ((double*)ri)[1 + 2 * 3]) /
              p; // e1 = 4e0e1/4e0
      qo->y = (((double*)ri)[0 + 2 * 3] - ((double*)ri)[2 + 0 * 3]) /
              p; // e2 = 4e0e2/4e0
      qo->z = (((double*)ri)[1 + 0 * 3] - ((double*)ri)[0 + 1 * 3]) /
              p; // e3 = 4e0e3/4e0
    }

    // 4e1^2-1 > 0 or e1^2 > 1/4 or |e1| > 1/2
    else if((p = ri->m00 - ri->m11 - ri->m22) > 0) {
      // Now p = 2|e1| > 1
      p = sqrt(p + 1);

      // If (2|e1| - 2er1) > 1, then p = 2|e1| has the wrong sign.
      // if ( p - ER[1] - ER[1] > 1 ) p = -p; // 2e1

      qo->x = p / 2; // e1
      p = p + p;     // 4e1

      qo->w = (((double*)ri)[2 + 1 * 3] - ((double*)ri)[1 + 2 * 3]) /
              p; // e0 = 4e0e1/4e1
      qo->y = (((double*)ri)[1 + 0 * 3] + ((double*)ri)[0 + 1 * 3]) /
              p; // e2 = 4e1e2/4e1
      qo->z = (((double*)ri)[2 + 0 * 3] + ((double*)ri)[0 + 2 * 3]) /
              p; // e3 = 4e3e1/4e1
    }

    // 4e2^2-1 > 0 or e2^2 > 1/4 or |e2| > 1/2
    else if((p = ri->m11 - ri->m00 - ri->m22) > 0) {
      // Now p = 2|e2| > 1
      p = sqrt(p + 1);

      // If (2|e2| - 2er2) > 1, then p = 2|e2| has the wrong sign.
      // if ( p - ER[2] - ER[2] > 1 ) p = -p; // 2e2

      qo->y = p / 2; // e2
      p = p + p;     // 4e2

      qo->w = (((double*)ri)[0 + 2 * 3] - ((double*)ri)[2 + 0 * 3]) /
              p; // e0 = 4e0e2/4e2
      qo->x = (((double*)ri)[0 + 1 * 3] + ((double*)ri)[1 + 0 * 3]) /
              p; // e1 = 4e1e2/4e2
      qo->z = (((double*)ri)[1 + 2 * 3] + ((double*)ri)[2 + 1 * 3]) /
              p; // e3 = 4e2e3/4e2
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

      qo->w = (((double*)ri)[1 + 0 * 3] - ((double*)ri)[0 + 1 * 3]) /
              p; // e0 = 4e0e3/4e3
      qo->x = (((double*)ri)[0 + 2 * 3] + ((double*)ri)[2 + 0 * 3]) /
              p; // e1 = 4e3e1/4e3
      qo->y = (((double*)ri)[2 + 1 * 3] + ((double*)ri)[1 + 2 * 3]) /
              p; // e2 = 4e2e3/4e3
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
@brief single precision function to convert s2_mat3x3_t rotation matrix to
single precision quaternion representation

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 &
e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 +
e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 &
e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}\f$

@todo Supply *ER*, a reference Euler Parameter. If necessary, flip the sign of
*(E) so that each *(E+i) is the closest match to ER[i]. For this algorithm to
work, *(E) must be a close follow-on to ER. This function would then prove more
useful when continuous Euler Parameters are desired for interpolation purposes.
Matrix R is quadratic in *(E) so *(E+i) and -*(E+i) give the same R.

@param ri [in] a rotation matrix in a 9x1 column format
@param qo [out] resulting euler parameters
*/
inline void s2_mat3x3f_toQuaternion(s2_mat3x3f_t* restrict ri,
                                    s2_quaternionf_t* restrict qo) {

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

      qo->x = (((float*)ri)[2 + 1 * 3] - ((float*)ri)[1 + 2 * 3]) /
              p; // e1 = 4e0e1/4e0
      qo->y = (((float*)ri)[0 + 2 * 3] - ((float*)ri)[2 + 0 * 3]) /
              p; // e2 = 4e0e2/4e0
      qo->z = (((float*)ri)[1 + 0 * 3] - ((float*)ri)[0 + 1 * 3]) /
              p; // e3 = 4e0e3/4e0
    }

    // 4e1^2-1 > 0 or e1^2 > 1/4 or |e1| > 1/2
    else if((p = ri->m00 - ri->m11 - ri->m22) > 0) {
      // Now p = 2|e1| > 1
      p = sqrt(p + 1);

      // If (2|e1| - 2er1) > 1, then p = 2|e1| has the wrong sign.
      // if ( p - ER[1] - ER[1] > 1 ) p = -p; // 2e1

      qo->x = p / 2; // e1
      p = p + p;     // 4e1

      qo->w = (((float*)ri)[2 + 1 * 3] - ((float*)ri)[1 + 2 * 3]) /
              p; // e0 = 4e0e1/4e1
      qo->y = (((float*)ri)[1 + 0 * 3] + ((float*)ri)[0 + 1 * 3]) /
              p; // e2 = 4e1e2/4e1
      qo->z = (((float*)ri)[2 + 0 * 3] + ((float*)ri)[0 + 2 * 3]) /
              p; // e3 = 4e3e1/4e1
    }

    // 4e2^2-1 > 0 or e2^2 > 1/4 or |e2| > 1/2
    else if((p = ri->m11 - ri->m00 - ri->m22) > 0) {
      // Now p = 2|e2| > 1
      p = sqrt(p + 1);

      // If (2|e2| - 2er2) > 1, then p = 2|e2| has the wrong sign.
      // if ( p - ER[2] - ER[2] > 1 ) p = -p; // 2e2

      qo->y = p / 2; // e2
      p = p + p;     // 4e2

      qo->w = (((float*)ri)[0 + 2 * 3] - ((float*)ri)[2 + 0 * 3]) /
              p; // e0 = 4e0e2/4e2
      qo->x = (((float*)ri)[0 + 1 * 3] + ((float*)ri)[1 + 0 * 3]) /
              p; // e1 = 4e1e2/4e2
      qo->z = (((float*)ri)[1 + 2 * 3] + ((float*)ri)[2 + 1 * 3]) /
              p; // e3 = 4e2e3/4e2
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

      qo->w = (((float*)ri)[1 + 0 * 3] - ((float*)ri)[0 + 1 * 3]) /
              p; // e0 = 4e0e3/4e3
      qo->x = (((float*)ri)[0 + 2 * 3] + ((float*)ri)[2 + 0 * 3]) /
              p; // e1 = 4e3e1/4e3
      qo->y = (((float*)ri)[2 + 1 * 3] + ((float*)ri)[1 + 2 * 3]) /
              p; // e2 = 4e2e3/4e3
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
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 &
e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 +
e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 &
e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}\f$

The Quaternion will be normalized during conversion.

@param qi [in] euler parameters to convert in a 4x1 array, in the form [w,x,y,z]
@param ro [out] the resulting rotation matrix is stored in ro
*/
inline void s2_quaternion_toMat3x3(s2_quaternion_t* restrict qi,
                                   s2_mat3x3_t* restrict ro) {

  s2_quaternion_normalize(qi);

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
matrix (s2_mat3x3f_t) representation

The following formula is used to perform conversion:

\f$R = \begin{bmatrix}
e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 &
e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\
e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 +
e_2e_3 - e_0e_1 - e_0e_1 \\
e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 &
e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}\f$

The Quaternion will be normalized during conversion.

@param qi euler parameters to convert in a 4x1 array, in the form [w,x,y,z]
@param ro the resulting rotation matrix is stored in ro
*/
inline void s2_quaternionf_toMat3x3f(s2_quaternionf_t* restrict qi,
                                     s2_mat3x3f_t* restrict ro) {

  s2_quaternionf_normalize(qi);

  float w2 = qi->w * qi->w;
  float x2 = qi->x * qi->x;
  float y2 = qi->y * qi->y;
  float z2 = qi->z * qi->z;

  ro->m00 = (x2 - y2 - z2 + w2);
  ro->m11 = (-x2 + y2 - z2 + w2);
  ro->m22 = (-x2 - y2 + z2 + w2);

  float tmp1 = qi->x * qi->y;
  float tmp2 = qi->z * qi->y;
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
Linear Interpolation algorithm

Refer to [Shoemake, Animating rotation with Quaternion Curves, SIGGRAPH, vol 19,
No. 3, 1985](http://run.usc.edu/cs520-s12/assign2/p245-shoemake.pdf) for
specific information on the algorithm.

@param q1i a first quaternion in the form [w,x,y,z] to interpolate from
@param q2i a second quaternion in the form [w,x,y,z] to interpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
inline void s2_quaternion_slerp(s2_quaternion_t* restrict q1i,
                                s2_quaternion_t* restrict q2i,
                                double* restrict t,
                                s2_quaternion_t* restrict qo) {

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
Linear Interpolation algorithm

Refer to [Shoemake, Animating rotation with Quaternion Curves, SIGGRAPH, vol 19,
No. 3, 1985](http://run.usc.edu/cs520-s12/assign2/p245-shoemake.pdf) for
specific information on the algorithm.

@param q1i a first quaternion in the form [w,x,y,z] to interpolate from
@param q2i a second quaternion in the form [w,x,y,z] to interpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
inline void s2_quaternionf_slerp(s2_quaternionf_t* restrict q1i,
                                 s2_quaternionf_t* restrict q2i,
                                 float* restrict t,
                                 s2_quaternionf_t* restrict qo) {

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
@brief Vectorized version of quaternion Linear Interpolation (Lerp)

Less expensive numerically than Shoemake's Slerp algorithm (s2_slerp)
but does not interpolate with constant velocity

@param q1i a first quaternion in the form [w,x,y,z] to interpolate from
@param q2i a second quaternion in the form [w,x,y,z] to interpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
inline void s2_quaternion_nlerp(s2_quaternion_t* restrict q1i,
                                s2_quaternion_t* restrict q2i,
                                double* restrict t,
                                s2_quaternion_t* restrict qo) {

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
    simple interpolation doesn't yield quaternions of unit length,
    so must normalize before returning to user
  */
  memset(qo, 0, 4 * sizeof(double));
  cblas_daxpy(4, alpha, (double*)q1i, 1, (double*)qo, 1);
  cblas_daxpy(4, omega, (double*)q2i, 1, (double*)qo, 1);
  s2_quaternion_normalize(qo);
}

/*!
@brief Vectorized version of quaternion Linear Interpolation (Lerp)

Less expensive numerically than Shoemake's Slerp algorithm (s2_Slerp)
but does not interpolate with constant velocity

@param q1i a first quaternion in the form [w,x,y,z] to interpolate from
@param q2i a second quaternion in the form [w,x,y,z] to interpolate to
@param t fraction from q1i to q2i (0<t<1)
@param qo the resulting quaternion is returned in qo
*/
inline void s2_quaternionf_nlerp(s2_quaternionf_t* restrict q1i,
                                 s2_quaternionf_t* restrict q2i,
                                 float* restrict t,
                                 s2_quaternionf_t* restrict qo) {

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
  s2_quaternionf_normalize(qo);
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
inline double s2_normalizeDegrees(double angle) {

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
inline float s2_normalizeDegreesf(float angle) {

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
double s2_normalizeRadians(double angle) {
  const double twopi = 2.0 * M_PI;
  while(angle < 0) {
    angle += twopi;
  }
  while(angle > 2 * M_PI) {
    angle -= twopi;
  }
  return angle;
}

/*!
@brief Normalize angle in radians

@param normalizes single precision radians to be between 0 and 2 * PI radians
*/
float s2_normalizeRadiansf(float angle) {
  const float twopi = 2.0 * M_PI;
  while(angle < 0) {
    angle += twopi;
  }
  while(angle > 2 * M_PI) {
    angle -= twopi;
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
double catan2pi(double y, double x, double theta) {

  double temp;
  double a;

  /*
    temp holds the exact current angle, but it may be off by multiples of 2*pi.
    so it is brought closer to theta by adding multiples of 2*pi obtained from
    theta.
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
double catan1pi(double y, double x, double theta) {

  double temp;
  double a;

  /*
    temp holds the exact current angle, but it may be off by multiples of pi.
    so it is brought closer to theta by adding multiples of pi obtained from
    theta.
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
@brief convert s2_axis_enum_t to a vector of ints
*/
s2_vectori_t s2_axis_getVector(s2_axis_enum_t axisType) {

  switch(axisType) {
    case S2_AXIS_XYZ: {
      s2_vectori_t axis = {0, 1, 2};
      return axis;
    }
    case S2_AXIS_YZX: {
      s2_vectori_t axis = {1, 2, 0};
      return axis;
    }
    case S2_AXIS_ZXY: {
      s2_vectori_t axis = {2, 1, 2};
      return axis;
    }
    case S2_AXIS_XZY: {
      s2_vectori_t axis = {0, 2, 1};
      return axis;
    }
    case S2_AXIS_YXZ: {
      s2_vectori_t axis = {1, 0, 2};
      return axis;
    }
    case S2_AXIS_ZYX: {
      s2_vectori_t axis = {2, 1, 0};
      return axis;
    }
    case S2_AXIS_XYX: {
      s2_vectori_t axis = {0, 1, 0};
      return axis;
    }
    case S2_AXIS_XZX: {
      s2_vectori_t axis = {0, 2, 0};
      return axis;
    }
    case S2_AXIS_YXY: {
      s2_vectori_t axis = {1, 0, 1};
      return axis;
    }
    case S2_AXIS_YZY: {
      s2_vectori_t axis = {1, 2, 1};
      return axis;
    }
    case S2_AXIS_ZXZ: {
      s2_vectori_t axis = {2, 0, 2};
      return axis;
    }
    case S2_AXIS_ZYZ: {
      s2_vectori_t axis = {2, 1, 2};
      return axis;
    }
    default: {
      s2_vectori_t axis = {0, 1, 2};
      return axis;
    }
  }
}

/*!
@brief extract Euler angles from a s2_mat3x3_t rotation matrix

This function decomposes a general direction cosine matrix into three primitive
direction cosine matrices, whose axes are determined by the integers passed
in through axis (see below). The direction cosine matrix is passed in through
ri, and the three computed angles are updated and returned in angle. The
previous values assigned to angle are passed in to RtoA in angle. RtoA calls
catan2, which uses the values stored in angle to maintain continuity if at all
possible.

This function does not check for bad axis indices. These checks could easily
be added. For actual reporting purposes, especially when a large
number of user output requests are to be processed, it would be more efficient
to program four or twelve different functions and have a preprocessor
generate pointers to the appropriate functions as needed. That way, no logical
tests would have to be performed at runtime, except in the function catan2().

If four functions were coded, each would handle one of the four Cases below.
In this case, the if statements would be eliminated, and only the axis indices
would be needed. If twelve functions were coded, each would handle one of the
twelve Cases. Again, the if statements would be eliminated, and the axis indices
would be hard coded, eliminating the need to pass them in or to perform any
other tests.

Used to determine if the next axis is forward '+' or reverse '-' sequence.
It also identifies index locations in the direction cosine matrix when
axes are repeated, such as in the sequence 1 --> 2 --> 1.

The function can be modified to decompose a matrix in any sequence of angles by
by changing "axis", but presently the sequence [rx=0, ry=1, rz=2] is hard coded
in to the function.

*/
void s2_mat3x3_getEulerAngles(s2_mat3x3_t* restrict rin,
                              s2_vector_t* restrict angles,
                              s2_axis_enum_t axisType) {

  const int f[3] = {1, 2, 0};

  s2_vectori_t axis = s2_axis_getVector(axisType);

  /* Short notation for the axis identifiers. */
  int i = axis.x;
  int j = axis.y;
  int k = axis.z;

  /*
    The value of k will be changed in Cases 3 and 4.
    Temp variables to hold true sin and cos values of third angle.
  */
  double c;
  double s;

  /* convert column major s2_mat3x3_t matrix to row major */
  double ri[3][3] = {{rin->m00, rin->m01, rin->m02},
                     {rin->m10, rin->m11, rin->m12},
                     {rin->m20, rin->m21, rin->m22}};

  if(j == f[i]) { // Identifies Cases 1 and 3: forward cycle i --> j
    if(k != i) {  // forward cycle j --> k
      // Case 1: 0 --> 1 --> 2; 1 --> 2 --> 0; 2 --> 0 --> 1
      angles->y = catan1pi(-ri[i][j], ri[i][i], angles->y);
      c = cos(angles->y);
      s = sin(angles->y);
      angles->x = catan2pi(ri[k][j] * c + ri[k][i] * s,
                           ri[j][j] * c + ri[j][i] * s, angles->x);
      angles->y = catan2pi(ri[i][k], ri[i][i] * c - ri[i][j] * s, angles->y);
    } else { // k == i; reverse cycle j --> i
      // Case 3: 0 --> 1 --> 0; 1 --> 2 --> 1; 2 --> 0 --> 2
      // Reset k for correct index into direction cosine matrix.
      k = f[j];
      angles->z = catan1pi(ri[i][j], ri[i][k], angles->z);
      c = cos(angles->z);
      s = sin(angles->z);
      angles->x = catan2pi(ri[k][j] * c - ri[k][k] * s,
                           ri[j][j] * c - ri[j][k] * s, angles->x);
      angles->y = catan2pi(ri[i][k] * c + ri[i][j] * s, ri[i][i], angles->y);
    }
  } else {       // Cases 2 and 4: reverse cycle i --> j
    if(k != i) { // reverse cycle j --> k
      // Case 2: 0 --> 2 --> 1; 1 --> 0 --> 2; 2 --> 1 --> 0
      angles->z = catan1pi(ri[i][j], ri[i][i], angles->z);
      c = cos(angles->z);
      s = sin(angles->z);
      angles->x = catan2pi(-(ri[k][j] * c - ri[k][i] * s),
                           ri[j][j] * c - ri[j][i] * s, angles->x);
      angles->y = catan2pi(-ri[i][k], ri[i][i] * c + ri[i][j] * s, angles->y);
    } else { // k == i; forward cycle j --> i
      // Case 4: 0 --> 2 --> 0; 1 --> 0 --> 1; 2 --> 1 --> 2
      // Reset k for correct index into direction cosine matrix.
      k = f[i];
      angles->z = catan1pi(-ri[i][j], ri[i][k], angles->z);
      c = cos(angles->z);
      s = sin(angles->z);
      angles->x = catan2pi(-(ri[k][j] * c + ri[k][k] * s),
                           ri[j][j] * c + ri[j][k] * s, angles->x);
      angles->y = catan2pi(-(ri[i][k] * c - ri[i][j] * s), ri[i][i], angles->y);
    }
  }
}

/*!
@brief convert sequence of euler angles to mat3x3 rotation matrix
@param [in] sequence of three angles specified in radians
@param [in] an s2_axis_enum_t specifying sequence of axes
@param [out] a 3x3 rotation matrix, results are returned in the rotation matrix
*/
void s2_axis_getMat3x3(s2_vector_t* restrict angles, s2_axis_enum_t axisType,
                       s2_mat3x3_t* restrict R) {

  s2_vectori_t axis = s2_axis_getVector(axisType);
  s2_mat3x3_t temp;
  s2_mat3x3_setIdentity(R);

  double dc[2] = {0.0, 0.0};
  for(int i = 0; i < 3; i++) {
    double angle = s2_vector_get(angles, i);
    s2_dc(&angle, dc);
    s2_mat3x3_setIdentity(&temp);
    switch(s2_vectori_get(&axis, i)) {
      case 0: {
        s2_product_rrx(R, dc, &temp);
        break;
      }
      case 1: {
        s2_product_rry(R, dc, &temp);
        break;
      }
      case 2: {
        s2_product_rrz(R, dc, &temp);
        break;
      }
    }
    s2_mat3x3_copy(&temp, R);
  }
}

/*! @brief Extract scalar from a uniformly scaled s2_mat3x3_t matrix

The norm of each column or row of an orthonormal rotation matrix is one.
Therefore, to find the scale factor for a rotation matrix that has been
<uniformly> scaled, simply find the norm of any row or column of the rotation
matrix.

Warning, this approach will not work for a nonuniformly scaled matrix..
For nonuniform scaling, it is necesssary to find the eigenvectors of the
rotation matrix.
*/
inline double s2_mat3x3_getScalar(s2_mat3x3_t* restrict ri) {

  return sqrt(ri->m00 * ri->m00 + ri->m11 * ri->m11 + ri->m22 * ri->m22);
}

// /*! @brief Extract scalar from a uniformly scaled single precision
// s2_mat3x3f_t matrix
//
// The norm of each column or row of an orthonormal rotation matrix is one.
//
// Therefore, to find the scale factor for a rotation matrix that has been
// <uniformly> scaled, simply find the norm of any row or column of the
// rotation matrix. Warning, this approach will not work for nonuniform
// scaling. For nonuniform scaling, one needs to find the eigenvectors using
// SVD or other means.
//
// */
// float s2_mat3x3f_getScalar(
//   s2_mat3x3f_t* ri) {
//
//   return sqrt(ri->m00 * ri->m00 + ri->m11 * ri->m11 + ri->m22 * ri->m22);
// }

/*!
@brief extract translation components of a s2_Mat4 matrix
*/
inline void s2_mat4x4_getTranslation(s2_mat4x4_t* restrict Mi,
                                     s2_vector_t* restrict to) {
  memcpy(to, &Mi->m12, sizeof(s2_vector_t));
}

/*!
@brief extract translation component of a s2_mat4x4f matrix
*/
inline void s2_mat4x4f_getTranslation(s2_mat4x4f_t* restrict Mi,
                                      s2_vectorf_t* restrict to) {
  memcpy(to, &Mi->m12, sizeof(s2_vectorf_t));
}

/*!
@brief utility to convert dpi, an array of length 3, to deltaq (an incremental
change in quaternion), add incremental change to an input quaternion
*/
inline void s2_add_dpiToq(s2_vector_t* restrict dpi,
                          s2_quaternion_t* restrict qi) {
  register s2_quaternion_t dq;
  dq.x = -qi->w * dpi->x + qi->z * dpi->y - qi->y * dpi->z;
  dq.y = -qi->z * dpi->x - qi->w * dpi->y + qi->x * dpi->z;
  dq.z = qi->y * dpi->x - qi->x * dpi->y - qi->w * dpi->z;
  dq.w = -qi->x * dpi->x - qi->y * dpi->y - qi->z * dpi->z;
  qi->x += 0.5 * dq.x;
  qi->y += 0.5 * dq.y;
  qi->z += 0.5 * dq.z;
  qi->w += 0.5 * dq.w;
}

/*!
@brief utility to convert dpi, an array of length 3, to deltaq (an incremental
change in quaternion), subtract incremental change from an input quaternion
*/
void s2_subtract_dpiFromq(s2_vector_t* restrict dpi,
                          s2_quaternion_t* restrict qi) {

  /*
  qi -= 1/2 * L_c(e_{ij} d\underline{\pi}_{ij}^j)

  qi -= 1 / 2 * [ e_w  -e_z   e_y] [dpi_x]
                [ e_z   e_w  -e_x] [dpi_y]
                [-e_y   e_x   e_w] [dpi_z]
                [-e_x  -e_y  -e_z]
  */

  /* dq \leftarrow L_c(e_{ij} d\underline{\pi}_{ij}^j) */
  register s2_quaternion_t dq;
  dq.x = qi->w * dpi->x - qi->z * dpi->y + qi->y * dpi->z;
  dq.y = qi->z * dpi->x + qi->w * dpi->y - qi->x * dpi->z;
  dq.z = -qi->y * dpi->x + qi->x * dpi->y + qi->w * dpi->z;
  dq.w = -qi->x * dpi->x - qi->y * dpi->y - qi->z * dpi->z;

  /* qi -= 1/2 * dq */
  qi->x -= 0.5 * dq.x;
  qi->y -= 0.5 * dq.y;
  qi->z -= 0.5 * dq.z;
  qi->w -= 0.5 * dq.w;
}

/*!
@brief utility to convert angular velocity to time derivative of quaternion

\f$ \underline{\dot{e}}^{ij} \leftarrow 1/2 \mathscr{R}(\underline{e})
\underline{\omega}_{ij}^i\f$

The result is then normalized by adding a feedback term

\f$ \underline{\dot{e}}^{ij} - \frac{0.5}{\tau}(1 - \underline{e}^{ijT}
\underline{e}^{ij})\underline{e}^{ij} \f$

where \f$\tau=0.5\f$ is hard coded

@param w [in] omega, angular velocity
@param qi [in] reference quaternion
@param dqo [out] output quaternion time derivative
*/
void s2_omega_to_dquaternion(s2_vector_t* restrict w,
                             s2_quaternion_t* restrict qi,
                             s2_quaternion_t* restrict dqo) {

  /*
    1/2 * [ qi->w  qi->z -qi->y] * {w->x}
          [-qi->z  qi->w  qi->x]   {w->y}
          [ qi->y -qi->x  qi->w]   {w->z}
          [-qi->x -qi->y -qi->z]
  */
  dqo->x = qi->w * w->x + qi->z * w->y - qi->y * w->z;
  dqo->y = -qi->z * w->x + qi->w * w->y + qi->x * w->z;
  dqo->z = qi->y * w->x - qi->x * w->y + qi->w * w->z;
  dqo->w = -qi->x * w->x - qi->y * w->y - qi->z * w->z;

  dqo->x *= 0.5;
  dqo->y *= 0.5;
  dqo->z *= 0.5;
  dqo->w *= 0.5;

  /* add feedback term to normalize quaternion */

  /* if tau is 0.5 */
  double alpha = 1.0 - cblas_ddot(4, (double*)qi, 1, (double*)qi, 1);

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
@brief linear interpolation nlerp

@param count [in] number of entries to interpolate
@param p [in] interpolant value, ranges from 0 to 1
@param y0 [in] value of function at y=0
@param y1 [in] value of function at y=1
@param yOut [out] interpolated data returned in yOut

*/
void s2_array_lerp(int count, double p, double* restrict y0,
                   double* restrict y1, double* restrict yOut) {

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
void s2_gramSchmidt(int m, int n, double* restrict A) {
  for(int k = 0; k < n; k++) {
    /* normalize column k */
    double R = 1.0 / cblas_dnrm2(m, A + k, n);
    cblas_dscal(m, R, A + k, n);
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
bool s2_all_close(int count, double* restrict a1, double* restrict a2) {
  for(int i = 0; i < count; i++) {
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
inline void s2_mat3x3_getAxisAngle(s2_mat3x3_t* restrict r,
                                   s2_vector_t* restrict axis, double* angle) {

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
  /* u_tilde = (1 / 2 sin \theta) * (r - r^T) */
  axis->x = r->m21 - r->m12;
  axis->y = r->m02 - r->m20;
  axis->z = r->m10 - r->m01;
  s2_vector_scale(1 / (2 * sin(*angle)), axis);
}

/*!
@brief get axis, angle and pitch of screw motion
from an SE3 displacement, do not use, not yet implemented
@TODO finish / verify this
*/
inline void s2_displacement_getScrewAxis(s2_displacement_t* restrict d,
                                         s2_screw_t* restrict h) {

  (void)h;
  s2_vector_t rotationAxis;
  double angle;
  s2_mat3x3_getAxisAngle(&d->R, &rotationAxis, &angle);
}

/*!
@brief compute determinant of 3x3 matrix
*/
inline double s2_mat3x3_determinant(s2_mat3x3_t* restrict R) {
  return (R->m00 * (R->m11 * R->m22 - R->m21 * R->m12) -
          R->m10 * (R->m01 * R->m22 - R->m21 * R->m02) +
          R->m20 * (R->m01 * R->m12 - R->m11 * R->m02));
}

/*!
@brief obtain s2_axis_enum_t from const char*
*/
s2_axis_enum_t s2_axis_enumFromChar(const char* sequence) {
  if(!strcmp(sequence, "XYZ")) {
    return S2_AXIS_XYZ;
  } else if(!strcmp(sequence, "YZX")) {
    return S2_AXIS_YZX;
  } else if(!strcmp(sequence, "ZXY")) {
    return S2_AXIS_ZXY;
  } else if(!strcmp(sequence, "XZY")) {
    return S2_AXIS_XZY;
  } else if(!strcmp(sequence, "YXZ")) {
    return S2_AXIS_YXZ;
  } else if(!strcmp(sequence, "ZYX")) {
    return S2_AXIS_ZYX;
  } else if(!strcmp(sequence, "XYX")) {
    return S2_AXIS_XYX;
  } else if(!strcmp(sequence, "XZX")) {
    return S2_AXIS_XZX;
  } else if(!strcmp(sequence, "YXY")) {
    return S2_AXIS_YXY;
  } else if(!strcmp(sequence, "YZY")) {
    return S2_AXIS_YZY;
  } else if(!strcmp(sequence, "ZXZ")) {
    return S2_AXIS_ZXZ;
  } else if(!strcmp(sequence, "ZYZ")) {
    return S2_AXIS_ZYZ;
  } else {
    return S2_AXIS_INVALID;
  }
}
