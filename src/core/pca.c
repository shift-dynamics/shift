/**
@file
@brief Functions used to set up, factor and solve equations of motion for one
time step
@author Kristopher Wehage
@date 2016
@version 1.0
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
#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "kinematics.h"
#include "pca.h"

void s2_symSchur2(s2_mat3x3_t* A, int p, int q, double* cs);

/*!
@brief 2x2 symmetric schur decomposition
Given an nxn symmetric matrix, A, and indices p,q such that 1 <= p < q <= n,
compute a sine-cosine pair (s,c) that forms a Jacobi rotation matrix
*/
inline void s2_symSchur2(s2_mat3x3_t* A, int p, int q, double* cs) {
  if(fabs(s2_mat3x3_get(A, p, q)) > 1e-9) {
    double r = (s2_mat3x3_get(A, q, q) - s2_mat3x3_get(A, p, p)) /
               (2.0 * s2_mat3x3_get(A, p, q));
    double t;
    if(r >= 0.0) {
      t = 1.0f / (r + sqrt(1.0f + r * r));
    } else {
      t = -1.0f / (-r + sqrt(1.0f + r * r));
    }
    cs[0] = 1.0 / sqrt(1.0 + t * t);
    cs[1] = t * cs[0];
  } else {
    cs[0] = 1.0;
    cs[1] = 0.0;
  }
}

/*!
@brief compute covariance matrix of point set
@param vertices [in] array of double precision 3-d vertices to process
@param vertexCount [in] number of vertices to process
@return cov 3x3 covariance matrix
*/
s2_mat3x3f_t s2_pca_covariancef(const float* vertices,
                                const size_t vertexCount) {

  s2_mat3x3f_t cov;
  float oon = 1.0f / (float)vertexCount;
  s2_vectorf_t c = {0.0f, 0.0f, 0.0f};

  /* compute center of mass */
  for(int i = 0; i < vertexCount; i++) {
    s2_vectorf_t* pt = (s2_vectorf_t*)(&vertices[i * 3]);
    s2_addequal_vvf(pt, &c);
  }
  s2_vectorf_scale(oon, &c);

  float e00 = 0.f;
  float e11 = 0.f;
  float e22 = 0.f;
  float e01 = 0.f;
  float e02 = 0.f;
  float e12 = 0.f;

  /* compute covariance elements */
  for(int i = 0; i < vertexCount; i++) {
    /* translate points so center of mass is at origin */
    s2_vectorf_t p;
    s2_vectorf_t* pt = (s2_vectorf_t*)(&vertices[i * 3]);
    s2_subtract_vvf(pt, &c, &p);

    /* compute covariance of translated points */
    e00 += p.x * p.x;
    e11 += p.y * p.y;
    e22 += p.z * p.z;
    e01 += p.x * p.y;
    e02 += p.x * p.z;
    e12 += p.y * p.z;
  }

  /* fill in covariance elements */
  s2_mat3x3f_set(&cov, 0, 0, e00 * oon);
  s2_mat3x3f_set(&cov, 1, 1, e11 * oon);
  s2_mat3x3f_set(&cov, 2, 2, e22 * oon);

  float temp = e01 * oon;
  s2_mat3x3f_set(&cov, 0, 1, temp);
  s2_mat3x3f_set(&cov, 1, 0, temp);

  temp = e02 * oon;
  s2_mat3x3f_set(&cov, 0, 2, temp);
  s2_mat3x3f_set(&cov, 2, 0, temp);

  temp = e12 * oon;
  s2_mat3x3f_set(&cov, 1, 2, temp);
  s2_mat3x3f_set(&cov, 2, 1, temp);

  return cov;
}

/*!
@brief compute covariance matrix of point set
@param cov [out] covariance matrix
@param vertices [in] array of double precision 3-d vertices to process
@param vertexCount [in] number of vertices to process
*/
s2_mat3x3_t s2_pca_covariance(const double* vertices,
                              const size_t vertexCount) {

  s2_mat3x3_t cov;
  float oon = 1.0 / (double)vertexCount;
  s2_vector_t c = {0.0f, 0.0f, 0.0f};

  /* compute center of mass */
  for(int i = 0; i < vertexCount; i++) {
    s2_vector_t* pt = (s2_vector_t*)(&vertices[i * 3]);
    s2_addequal_vv(pt, &c);
  }
  s2_vector_scale(oon, &c);

  float e00 = 0.f;
  float e11 = 0.f;
  float e22 = 0.f;
  float e01 = 0.f;
  float e02 = 0.f;
  float e12 = 0.f;

  /* compute covariance elements */
  for(int i = 0; i < vertexCount; i++) {
    /* translate points so center of mass is at origin */
    s2_vector_t p;
    s2_vector_t* pt = (s2_vector_t*)(&vertices[i * 3]);
    s2_subtract_vv(pt, &c, &p);

    /* compute covariance of translated points */
    e00 += p.x * p.x;
    e11 += p.y * p.y;
    e22 += p.z * p.z;
    e01 += p.x * p.y;
    e02 += p.x * p.z;
    e12 += p.y * p.z;
  }

  /* fill in covariance elements */
  s2_mat3x3_set(&cov, 0, 0, e00 * oon);
  s2_mat3x3_set(&cov, 1, 1, e11 * oon);
  s2_mat3x3_set(&cov, 2, 2, e22 * oon);

  double temp = e01 * oon;
  s2_mat3x3_set(&cov, 0, 1, temp);
  s2_mat3x3_set(&cov, 1, 0, temp);

  temp = e02 * oon;
  s2_mat3x3_set(&cov, 0, 2, temp);
  s2_mat3x3_set(&cov, 2, 0, temp);

  temp = e12 * oon;
  s2_mat3x3_set(&cov, 1, 2, temp);
  s2_mat3x3_set(&cov, 2, 1, temp);
  return cov;
}

/*!
@brief compute eigenvectors and eigenvalues of a symmetric matrix A using
Jacobi method of iteratively updating A as A = J^T * A * J where J is a Jacobi
rotation matrix
*/
void s2_pca_jacobi(s2_mat3x3_t* A, s2_mat3x3_t* v) {
  s2_mat3x3_setIdentity(v);
  s2_mat3x3_t J;
  s2_mat3x3_t temp;
  const int MAX_ITERATIONS = 50;
  int p = 0;
  int q = 1;
  double cs[2];
  double prev_norm = DBL_MAX;

  for(int n = 0; n < MAX_ITERATIONS; n++) {
    /* find largest off diagonal element */
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(i == j) {
          continue;
        }
        if(fabs(s2_mat3x3_get(A, i, j)) > fabs(s2_mat3x3_get(A, p, q))) {
          p = i;
          q = j;
        }
      }
    }

    /* compute jacobi rotation matrix */
    s2_symSchur2(A, p, q, cs);
    s2_mat3x3_setIdentity(&J);
    s2_mat3x3_set(&J, p, p, cs[0]);
    s2_mat3x3_set(&J, p, q, cs[1]);
    s2_mat3x3_set(&J, q, p, -cs[1]);
    s2_mat3x3_set(&J, q, q, cs[0]);

    s2_product_rr(v, &J, &temp);
    s2_mat3x3_copy(&temp, v);

    s2_product_rinverser(&J, A, &temp);
    s2_product_rr(&temp, &J, A);

    double norm = 0.0;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(i == j) {
          continue;
        } else {
          norm += s2_mat3x3_get(A, i, j) * s2_mat3x3_get(A, i, j);
        }
      }
    }

    if(n > 2 && norm >= prev_norm) {
      return;
    }
    prev_norm = norm;
  }
}

/*!
@brief compute squared distance at specified index of a single precision vector
array
*/
inline float s2_pca_squaredDistanceAtIndex(const float* vertices,
                                           const size_t index0,
                                           const size_t index1) {

  float sq_distance = 0;
  sq_distance += vertices[3 * index0 + 0] * vertices[3 * index1 + 0];
  sq_distance += vertices[3 * index0 + 1] * vertices[3 * index1 + 1];
  sq_distance += vertices[3 * index0 + 2] * vertices[3 * index1 + 2];
  return sq_distance;
}

void s2_pca_extremePointsAlongDirection(s2_vectorf_t* restrict e,
                                        const float* restrict vertices,
                                        const int vertexCount,
                                        int* restrict imin,
                                        int* restrict imax) {

  float max_projection2 = -FLT_MAX;
  float min_projection2 = FLT_MAX;
  for(int i = 0; i < vertexCount; i++) {
    float projection2 = 0;
    projection2 += e->x * vertices[i * 3 + 0];
    projection2 += e->y * vertices[i * 3 + 1];
    projection2 += e->z * vertices[i * 3 + 2];
    if(projection2 > max_projection2) {
      *imax = i;
      max_projection2 = projection2;
    }
    if(projection2 < max_projection2) {
      *imin = i;
      min_projection2 = projection2;
    }
  }
}

/*!
@brief get a double precision vector from array of single precision vectors
stored in a float array
*/
inline s2_vector_t s2_pca_vectorFromFloatArrayAtIndex(const float* vertices,
                                                      const size_t index) {

  s2_vector_t vector;
  s2_vector_set(&vector, 0, (double)vertices[3 * index + 0]);
  s2_vector_set(&vector, 1, (double)vertices[3 * index + 1]);
  s2_vector_set(&vector, 2, (double)vertices[3 * index + 2]);
  return vector;
}
