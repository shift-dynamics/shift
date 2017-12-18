/**
@file
@author Kristopher Wehage
@date 5/27/2015
@copyright Kristopher Wehage 2014,2015
@section DESCRIPTION
This function library is used to test star functionality

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

#include "../shift.h"



/*!
@brief LU Decomposition with complete row and column pivoting

mbdl)factor_LU_full factors a double precision matrix, A[rowDim * colDim],
stored in row major order using full row and column pivoting. Matrix A need
not have full row or column rank. The integer variable 'rank' represents the
matrix rank and internally is always one less than the true rank to be
consistent with C's indexing from zero. Upon return from the function,
rank will be set to the correct value.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity
diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper
triangular (rank+1) by (rank+1) Ur matrix is stored on and above the
diagonal in A[0:rank][0:rank].

Following the first major matrix decomposition step, A[0:rank][rank+1:colDim]
stores the residual matrix UR and A[rank+1:rowDim][0:rank] stores the residual
matrix LR. The product C=LR*inverse(Lr) is then computed and stored back in
A[rank+1:rowDim][0:rank] and the product B=-inverse(Ur)*UR is
computed and stored back in A[0:rank][rank+1:colDim].

The remaining A[rank+1:rowDim][rank+1:colDim] submatrix contains numbers whose
absolute values are all smaller than eps times the absolutely largest
entry in the original matrix. This part of the matrix is not used.

@param rowDim [in] row dimension of matrix A.
@param colDim [in] column dimension of matrix A.
@param *A [in/out] matrix to be factored with dimensions
A[0:rowDim-1][0:colDim-1], LU factors are overwritten in original matrix
@param eps [in] input tolerance on the order of machine roundoff.
@param pr [out] output row permutation array with dimensions pr[0:rowDim-1]
@param pc [out] output column permutation array with dimensions pc[0:colDim-1]
@return rank of matrix A
*/
int LU_full_colMajor(
  int rowDim,
  int colDim,
  double* A,
  double eps,
  int* pr,
  int* pc,
  int* pr_inv,
  int* pc_inv) {

  // printf("Jacobian matrix in fullFactor_LU\n");
  // for(int i = 0; i < rowDim; i++) {
  //   for(int j = 0; j < colDim; j++)
  //     printf("% 2.4f ", A[i * colDim + j]);
  //   printf("\n");
  // }

  int rank;     /* (rank+1) is the matrix rank */
  int pivotRow; /*  row with current pivotal element */
  int pivotCol; /* column with current pivotal element */
  double pivot = 0.0;
  /* current pivotal element, holds the current pivotal element
     value */
  double tol;   /* tolerance for checking residual matrix
                              infinity norm against */
  double save; /* variable for holding intermediate results*/
  double size = rowDim * colDim; /*overall size of matrix */

  /*
    Return failure if a bad row or column dimension was found.
  */
  if((rowDim <= 0) || (colDim <= 0)) {
    return (-1);
  }

  /*
    Consistent with C-indexing, rank always has a value of
    one less than the true value. rank+1 is returned.
  */
  rank = -1; /* initialize for indexing */

  /*
    Search through the entire A matrix to find the
    absolutely largest element for assigning to pivot.
    Save that row and column number in pivotRow and
    pivotCol. Also intialize the row permutation array pr.
  */
  int index = cblas_idamax(size, A, 1);
  div_t result = div(index, rowDim);
  pivotCol = result.quot;
  pivotRow = result.rem;
  pivot = A[index];
  // printf("pivotRow: %d, pivotCol: %d, pivot: %f\n", pivotRow, pivotCol, pivot);


  /* Initialize the column permutation array. */
  for(int col = 0; col < colDim; col++) {
    pc[col] = col;
  }
  for(int row = 0; row < rowDim; row++) {
    pr[row] = row;
  }

  /* Set tolerance to check all zero-elements against. */
  tol = fabs(eps * pivot);

  /*
    Major loop to permute and factor the matrix.
    Generate factors column by column, so outer loop
    sweeps over columns.
  */
  for(int col = 0; col < colDim; col++) {
    /*
      If the current pivotal element is <= tol, the
      remaining submatrix is zero and factorization
      is complete. Need to add an else statement to
      break out of this loop so not to waste time
      going through remaining columns?
    */
    if(fabs(pivot) > tol) {
      /* Increase rank for the current column. */
      ++rank;

      /*
        pivotRow can never be less than rank.
        If pivotRow==rank, then no permutation is needed.
        If pivotRow>rank, then need to swap rows.
      */
      // fprintf(stdout, "\nbefore swap:\n");
      // s2_array_print2D(stdout, rowDim, colDim, false, A);
      if(pivotRow > rank) {
        /*
          Need to swap entire row A[rank][:] and A[pivotRow][:].
          This also swaps rows in the Lr matrix to this point.

          swapping row:
            N = colDim
            inc = 1

          number of entries for column swap: colDim
          stride =
        */

        // fprintf(stdout, "\n\nbefore row swap\n");
        // s2_array_print2D(stdout, rowDim, colDim, false, A);

        // fprintf(stdout, "swapping row %d and %d:\n", rank, pivotRow);
        cblas_dswap(
          colDim,
          A + rank,
          rowDim,
          A + pivotRow,
          rowDim);

        // fprintf(stdout, "after row swap\n");
        // s2_array_print2D(stdout, rowDim, colDim, false, A);
        /*
          Swap row permutation entries pr[rank] and
          pr[pivotRow] to reflect row swaps in A.
        */
        s2_iswap(pr + rank, pr + pivotRow);
      }

      /*
        pivotCol can never be less than rank.
        If pivotCol==rank, then no permutation is needed.
        If pivotCol>rank, then need to swap columns.
      */
      if(pivotCol > rank) {
        // printf("rank: %d, pivotCol: %d\n", rank, pivotCol);
        // printf("colDim %d\n", colDim);
        /*
          entire columns A[:][rank] and A[:][pivotCol].

          swapping column:
            N = rowDim
            stride = colDim
        */
        // fprintf(stdout, "swapping column %d and %d:\n", rank, pivotCol);

        cblas_dswap(
          rowDim,
          A + rank * rowDim,
          1,
          A + pivotCol * rowDim,
          1);

        // fprintf(stdout, "after column swap\n");
        // s2_array_print2D(stdout, rowDim, colDim, false, A);
        /*
          Swap column permutation entries pc[rank] and
          pr[pivotCol] to reflect column swaps.
        */
        s2_iswap(pc + rank, pc + pivotCol);
      }

      // fprintf(stdout, "after swap:\n");
      // s2_array_print2D(stdout, rowDim, colDim, false, A);

      /*
        rank cannot be > rowDim-1. If rank < rowDim-1
        there is still some processing left to do. If
        rank == rowDim-1, there is only a 1 on the
        diagonal of this column of Lr. This loop factors
        the matrix and searches for a new pivotal element
        for the next factorization step.
      */
      if(rank < rowDim - 1) {

        /*
          Copy the current pivotal element into save so new
          pivotal element can be stored back in pivot.
        */
        save = pivot;
        pivot = 0;

        /* Need only process the rows from rank+1 to rowDim. */
        for(int i = rank + 1; i < rowDim; i++) {

          /* Evaluate the current entry in the L matrix.*/
          A[i + rank * rowDim] /= save;

          /*
            rank cannot be >= colDim. If rank < colDim
            there is still some processing left to do. If
            rank == colDim-1, only a pivot remains on the
            diagonal.
          */
          if(rank < colDim) {

            /*
              This next loop computes the remaining U matrix,
              searches for the largest pivotal element, and
              remembers the new pivotal element row and colum.
            */
            for(int j = rank + 1; j < colDim; j++) {
              A[i + j * rowDim] -= A[i + rank * rowDim] * A[rank + j * rowDim];
              if(fabs(A[i + j * rowDim]) > fabs(pivot)) {
                pivot = A[i + j * rowDim];
                pivotRow = i;
                pivotCol = j;
              } /*End if*/
            }   /*End for j*/
          }     /*End if rank*/
        }       /*Endfor i = rank+1*/
      }         /*Endif (rank<rowDim-1)*/
      else {
        break;  /*No more rows to process.*/
      }
    }          /*End if*/
    else {
      break;  /*Done if remainder of matrix is at noise level.*/
    }
  }          /*End for col*/

  /*
    This block computes LR*inverse(Lr)
    Here rank>0 means true rank is greater than 1.
    If rank==0, the Lr matrix is a 1 by 1 identity
    matrix, so there is nothing to do here. This loop
    computes A[rank+1:rowDim-1][0:rank]=LR*inverse(Lr)
    where LR is stored in A[rank+1:rowDim-1][0:rank]
    and Lr is stored in A[0:rank][0:rank].
    j is the column number in LR. It ends at 1
    because the diagonal entry in row 0 of Lr is 1.
    i is the row number in LR.
    k is the column number in Lr
  */
  if(rank > 0) {
    for(int j = rank; j > 0; j--) {
      for(int i = rowDim - 1; i > rank; i--) {
        for(int k = 0; k < j; k++)
          A[i + k * rowDim] -= A[i + j * rowDim] *
                               A[j + k * rowDim];
      }
    }
  }

  /*
    This block computes -inverse(Ur)*UR.
    Here rank > -1 means the true rank is greater than 0.
    If rank==0, nothing to process. This loop computes
    A[0:rank][rank+1:colDim-1]=-inverse(Ur)*UR
    where UR is stored in A[0:rank][rank+1:colDim-1]
    and Ur is stored in upper part of A[0:rank][0:rank].
    i is the row number in Ur.
    j is the column number in Ur & row number in UR.
    k is the column number in UR.
  */
  // if(rank > -1 && rank < colDim - 1) {
  //   for(i = rank; i >= 0; i--) {
  //     for(k = rank + 1; k < colDim; k++) {
  //       save = 0;
  //       for(j = rank; j > i; j--) {
  //         save -= A[i + j * rowDim] * A[j + k * rowDim];
  //       }
  //       A[i + k * rowDim] = (save - A[i + k * rowDim]) / A[i + i * rowDim];
  //     }
  //   }
  // }
  for(int i = 0; i < rowDim; i++) {
    pr_inv[pr[i]] = i;
  }
  for(int i = 0; i < colDim; i++) {
    pc_inv[pc[i]] = i;
  }
  return (rank + 1);
}


/*!
@brief LU Decomposition with no row or column pivoting

factor LU factors a double precision matrix, A[rowDim][colDim],
using no row or column pivoting. Only the square rank x colDim submatrix of
matrix A will be processed.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity
diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper
triangular (rank+1) by (rank+1) Ur matrix is stored on and above the
diagonal in A[0:rank][0:rank].

Note that this minimal algorithm variation does not compute the product
C=LR*inverse(Lr)

@param rowDim [in] row dimension of matrix A.
@param colDim [in] column dimension of matrix A.
@param rank [in] only the square rank x colDim submatrix of A will be processed
@param *A [in/out] matrix to be factored with dimensions A[0:rowDim-1][0:colDim-1]:
*/
void LU_colMajor(
  int rowDim,
  int colDim,
  int rank,
  double* A) {

  /*
    Major loop to factor the matrix.
    Generate factors column by column
  */
  for(int row = 0; row < rank; row++) {

    /* i iterates over rows of A, up to rank,
    previously rank was rowDim */
    for(int i = row + 1; i < rank; i++) {

      /* Evaluate the current entry in the L matrix.*/
      A[i + row * rowDim] /= A[row + row * rowDim];

      /* Compute U matrix */
      for(int j = row + 1; j < colDim; j++) {
        A[i + j * rowDim] -= A[i + row * rowDim] * A[row + j * rowDim];
      }
    }
  }

  if(rank > 0 && rank < colDim) {
    for(int i = rank - 1; i > -1; i--) { /* rows of B */
      for(int k = rank; k < colDim; k++) { /* columns of B */
        double save = 0;
        for(int j = rank - 1; j > i; j--) { /* columns of Ur */
          save += A[i + j * rowDim] * A[j + k * rowDim];
        }
        A[i * colDim + k] =
          (A[i + k * rowDim] - save) / A[i + i * rowDim];
      }
    }
  }
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
void gramSchmidt_colMajor(
  int rowDim,
  int colDim,
  double* A) {

  for(int k = 0; k < colDim; k++) {
    /* normalize column k */
    double R =
      1.0 / sqrt(cblas_ddot(rowDim, A + k * rowDim, 1, A + k * rowDim, 1));
    cblas_dscal(rowDim, R, A + k * rowDim, 1);
    for(int j = k + 1; j < colDim; j++) {
      R = 0;
      for(int i = 0; i < rowDim; i++) {
        R += A[i + k * rowDim] * A[i + j * rowDim];
      }
      for(int i = 0; i < rowDim; i++) {
        A[i + j * rowDim] -= A[i + k * rowDim] * R;
      }
    }
  }
}


/*!
@brief
set jacobian for a single loop mechanism in column major order in PO2
*/
void setJacobianPO2_colMajor(
  int colDim,
  double* J_po1,
  double* J_po2,
  int* pr_12,
  int* pc_12) {

  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < colDim; j++) {
      J_po2[i + j * 6] = J_po1[pr_12[i] + pc_12[j] * 6];
    }
  }
}


int main(int argc, char* argv[]) {

  s2_context_t* context = s2_context_load(argv[1]);
  s2_context_setTopology(context);

  FILE* debug = context->modelParameters.debugFilePointer;
  // s2_context_print(debug, context);

  /* find current position of end effector */
  s2_updateDH(context);
  s2_frame_t* endEffectorFrame =
    s2_context_getFrameByName(context, "body7", "7h_plus");

  if(!endEffectorFrame) {
    fprintf(stderr, "frame not found\n");
    return(-1);
  }

  /* end effector frame not in spanning tree, need to explicitly update using dRelative */
  s2_body_t* endEffectorBody =
    s2_context_getBodyByName(context, "body7");

  s2_product_dd(
    endEffectorBody->referenceFrame->dAbsolute,
    endEffectorFrame->dWRTReference,
    endEffectorFrame->dAbsolute);

  fprintf(debug, "dAbsolute:\n");
  s2_displacement_print(debug, endEffectorFrame->dAbsolute);

  /* J is a conventional Jacobian for the end effector in row major order */
  double J[context->dqCount * 6];
  memset(J, 0, context->dqCount * 6 * sizeof(double));
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < context->dqCount; j++) {
      J[i * context->dqCount + j] = ((double*)(context->h + j))[i];
    }
  }

  fprintf(debug, "\nverifying jacobian (row major)\n");
  s2_array_print2D(debug, 6, context->dqCount, true, J);

  /* J_a is an augmented column major ordered
   Jacobian matrix */
  double J_a[(context->dqCount) * 6];
  memset(J_a, 0, (context->dqCount) * 6 * sizeof(double));

  fprintf(debug, "h:\n");
  for(int i = 0; i < context->dqCount; i++) {
    s2_screw_print(debug, context->h + i);
  }

  /* evaluate initial jacobian in po1 */

  double* J_po1 = (double*)context->h;

  fprintf(debug, "\nverifying jacobian (column major)\n");
  s2_array_print2D(debug, 6, context->dqCount, false, J_po1);

  fprintf(debug, "\nverifying jacobian (row major)\n");
  s2_array_print2D(debug, 6, context->dqCount, true, J);

  int pr_12[6] = {0, 1, 2, 3, 4, 5};
  int pr_21[6] = {0, 1, 2, 3, 4, 5};
  int pc_12[context->dqCount];
  int pc_21[context->dqCount];
  memset(pc_12, 0, context->dqCount * sizeof(int));
  memset(pc_21, 0, context->dqCount * sizeof(int));

  int rank = LU_full_colMajor(
               6, context->dqCount, J_po1, 1e-9, pr_12, pc_12, pr_21, pc_21);

  fprintf(
    debug, "\nverifying jacobian LU factors (column major): rank = %d\n", rank);
  s2_array_print2D(debug, 6, context->dqCount, false, J_po1);

  fprintf(debug, "\northonormalizing last column\n");
  gramSchmidt_colMajor(6, context->dqCount - rank, J_po1 + rank * 6);
  s2_array_print2D(debug, 6, context->dqCount, false, J_po1);

  fprintf(debug, "\nverifying jacobian LU factors (row major)\n");
  s2_factor_LU_full(6, context->dqCount, J, 1e-9, pr_12, pc_12);
  s2_array_print2D(debug, 6, context->dqCount, true, J);

  fprintf(debug, "interpolate through 5 key frames:\n");
  s2_displacement_t frames[6];
  memcpy(frames, endEffectorFrame->dAbsolute, sizeof(s2_displacement_t));

  double angle = 0.01;
  double dc[2];
  dc[0] = sin(angle);
  dc[1] = cos(angle);
  double stepSize = 0.05;

  /* set frame 1 */
  s2_vector_t t01 = {0.0, 1.0, 1.0};
  s2_vector_normalize(&t01);
  s2_vector_scale(stepSize, &t01);
  s2_product_drx(frames, dc, frames + 1);
  s2_add_vv(&frames[0].T, &t01, &frames[1].T);

  /* set frame 2 */
  s2_vector_t t12 = {0.1, 1.0, 1.0};
  s2_vector_normalize(&t12);
  s2_vector_scale(stepSize, &t12);
  s2_product_drx(frames + 1, dc, frames + 2);
  s2_add_vv(&frames[1].T, &t12, &frames[2].T);

  /* set frame 3 */
  s2_vector_t t23 = {0.2, 1.0, 1.0};
  s2_vector_normalize(&t23);
  s2_vector_scale(stepSize, &t23);
  s2_product_drx(frames + 2, dc, frames + 3);
  s2_add_vv(&frames[2].T, &t23, &frames[3].T);

  /* set frame 4 */
  s2_vector_t t34 = {0.3, 1.0, 1.0};
  s2_vector_normalize(&t34);
  s2_vector_scale(stepSize, &t34);
  s2_product_dry(frames + 3, dc, frames + 4);
  s2_add_vv(&frames[3].T, &t34, &frames[4].T);

  /* set frame 5 */
  s2_vector_t t45 = {0.4, 1.0, 1.0};
  s2_vector_normalize(&t45);
  s2_vector_scale(stepSize, &t45);
  s2_product_dry(frames + 4, dc, frames + 5);
  s2_add_vv(&frames[4].T, &t45, &frames[5].T);

  s2_quaternion_t q[6];
  for(int i = 0; i < 6; i++) {
    s2_mat3x3_toQuaternion(&frames[i].R, q + i);
  }

  double pitch = 0.1;
  s2_screw_t hLocal =
  {0.0, 0.0, 1.0, 0.0, 0.0, fabs(pitch) < 1e-9 ? 0 : 1.0 / pitch};

  s2_screw_print(debug, &hLocal);

  s2_screwproduct_ds(
    endEffectorFrame->dAbsolute,
    &hLocal,
    &context->h[context->dqCount]);





  // for


  // s2_displacement_print(debug, )
  //
  // double J[context->dqCount + 1 * 6];
  // for(int i = 0; i < context->dqCount; i++) {
  //
  // }














  return 0;
}
