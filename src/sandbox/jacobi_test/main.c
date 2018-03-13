#include "../../core/ellipsoid.h"
#include "../../core/factor.h"
#include "../../core/kinematics.h"
#include "../../core/pca.h"

#include <float.h>
#include <stdio.h>

int main() {

  s2_mat3x3_t A = {1620.0, -675.0, -30.0, -675.0, 300.0,
                   60.0,   -30.0,  60.0,  4.0};

  printf("A:\n");
  s2_mat3x3_print(stdout, &A);

  s2_mat3x3_t v;
  s2_pca_jacobi(&A, &v);

  printf("\nD:\n");
  s2_mat3x3_print(stdout, &A);

  printf("\nv:\n");
  s2_mat3x3_print(stdout, &v);

  printf("L matrix:\n");
  double L[16] = {5.0, 0.0, 0.0, 0.0, 3.0, 4.0, 0.0,  0.0,
                  5.0, 6.0, 3.0, 0.0, 8.0, 9.0, 10.0, 2.0};
  print_A(L, 4, 4);

  /* printed result for LD should be:
  [[ 25.  15.  25.  40.]
   [ 15.  25.  39.  60.]
   [ 25.  39.  70. 124.]
   [ 40.  60. 124. 249.]]
  */
  s2_factor_verify_cholesky(L, 4);

  double B[16] = {25., 15., 25., 40.,  15., 25., 39.,  60.,
                  25., 39., 70., 124., 40., 60., 124., 249.};

  // double B[16] = {2.0,  0.0,  0.0,   0.0, 6.0,  22.0, 0.0,   0.0,
  //                 10.0, 54.0, 201.0, 0.0, 16.0, 84.0, 366.0, 1163.0};

  printf("B:\n");
  print_A(B, 4, 4);

  s2_factor_cholesky(B, 4);

  printf("composite B matrix after cholesky\n");
  print_A(B, 4, 4);

  printf("verify decomposition:\n");
  s2_factor_verify_cholesky(B, 4);

  double Q[44] = {4., 3.,  7.,  9.,  11., 16., 12., 15., 17., 19., 13.,
                  7., 13., 23., 17., 9.,  11., 17., 3.,  2.,  5.,  6.,
                  9., 2.,  17., 11., 5.,  8.,  13., 4.,  3.,  2.,  1.,
                  1., 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.};

  double d[11] = {1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11.};

  double X[16];
  memset(X, 0, 16 * sizeof(double));

  product_QDQt(Q, d, X, 11);

  print_A(X, 4, 4);

  return 0;
}
