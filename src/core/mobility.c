/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
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

#include "mobility.h"
#include "context.h"
#include "jacobian.h"
#include "kinematics.h"
#include "stdlib.h"
#include "substructure.h"
#include "topology.h"
#include "utility.h"
#include <math.h>

/*!
  computes mobility numbers, i.e. sum of squares of entries in A[m][:] and
  returns result in a[m]

*/
void s2_mobilityNumbers(int m, int n, double* A, double* a) {

  /* m is row dimension, i iterates over all rows */
  for(int i = 0; i < m; i++) {
    a[i] = cblas_ddot(n, A + i * n, 1, A + i * n, 1);
  }
}

/*!
@brief prints mobility numbers for one substructure
*/
void s2_substructure_printMobility(FILE* f, s2_substructure_t* substructure) {

  int substructureIndex = substructure->index;
  s2_context_t* context = substructure->context;
  fprintf(f,
          "\nsubstructure [%d], rank: %d, dqOffset: %d, dqCount, %d, "
          "constraintEqnCount: %d\n",
          substructureIndex, substructure->rank, substructure->dqOffset,
          substructure->dqCount, substructure->constraintEquationCount);

  fprintf(f, "mobility numbers: \n");

  int i = substructure->dqOffset;
  for(int j = 1; j < context->substructureMembersCount[substructureIndex];
      j++) {
    s2_joint_t* joint =
      context
        ->jointPointers[context->substructureMembers[substructureIndex][j]];
    double mobility = 0;
    switch(joint->jointType) {
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ:
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {
        mobility += context->mobilityNumbers[i++];
        break;
      }
      case S2_JOINT_FLOAT: {
        mobility += context->mobilityNumbers[i++];
        mobility += context->mobilityNumbers[i++];
        mobility += context->mobilityNumbers[i++];
        __attribute__((fallthrough));
      }
      case S2_JOINT_SPHERICAL: {
        mobility += context->mobilityNumbers[i++];
        mobility += context->mobilityNumbers[i++];
        mobility += context->mobilityNumbers[i++];
        break;
      }
      default: { break; }
    }
    fprintf(f, "  m=%f2.2 ", mobility);

    s2_stackstr_print(f, joint->name);
  }
}

/*!
@brief print mobility information,
*/
void s2_context_printMobility(FILE* f, s2_context_t* context) {

  fprintf(f, "Mobility:\n");
  fprintf(f, "Total mobility: %0.1f\n", context->mobility);
  fprintf(f,
          "----------------------------------------------------------------\n");

  for(int i = 0; i < context->dqCount; i++) {
    bool inSubstructure = false;
    s2_substructure_t* substructure = context->substructures;
    for(int j = 0; j < context->substructureCount; j++, substructure++) {
      if(i >= substructure->dqOffset &&
         i < substructure->dqOffset + substructure->dqCount) {
        s2_substructure_printMobility(f, substructure);
        inSubstructure = true;
        break;
      }
    }
    // if(!inSubstructure) {
    //   printf("  [%d] m=%0.9f ", i, context->mobilityNumbers[i]);
    //   int jointIndex = context->jointLookup[i];
    //   s2_stackstr_print(context->jointPointers[jointIndex]->name);
    // }
  }
  fprintf(f, "\n");
}

/*!
@brief Initialize quantities necessary to compute mobility numbers

This function evaluates the variable mobility numbers in
permuted order. It first forms an n by n-r matrix from the
submatrix J[0:r-1][r:n-1] stacked above an (n-r) by (n-r)
identity matrix. Then it calls GramSchmidt() to
orthonormalize the columns and return them in the same
matrix. Finally it returns the square of row norms of
this matrix as mobility numbers in the mn[] array.

@param m row dimension
@param n column dimension
@param** J Jacobian matrix
@param *mn square of row norms of submatrix J[0:r-1][r:n-1] are returned in the
mn[] array
*/
void s2_context_initializeMobility(s2_context_t* context) {

  context->mobility = 0;
  s2_array_free(context->mobilityNumbers);
  context->mobilityNumbers = malloc(context->dqCount * sizeof(double));

  for(int i = 0; i < context->dqCount; i++) {
    context->mobilityNumbers[i] = 1.0;
  }

  for(int i = 0; i < context->activeJointCount; i++) {
    s2_joint_t* joint = context->jointPointers[i];
    if(joint->jointType != S2_JOINT_RIGID) {
      joint->mobilityNumber = context->mobilityNumbers + joint->dqIndex;
    } else {
      joint->mobilityNumber = NULL;
    }
  }

  /* get count of size for single right null space array */
  context->rightNullSpaceCount = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    context->rightNullSpaceCount +=
      substructure->dqCount * substructure->dqCount;
  }

  /* malloc memory for rightNullSpace, allows additional storage for null space
     up to dqCount * dqCount in the event that the null space gains
     rank */
  s2_array_free(context->rightNullSpace);
  context->rightNullSpace =
    calloc(context->rightNullSpaceCount, sizeof(double));

  int rightNullSpaceCount = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    substructure->rightNullSpace =
      context->rightNullSpace + rightNullSpaceCount;

    substructure->mobilityNumbers =
      context->mobilityNumbers + substructure->dqOffset;

    rightNullSpaceCount += substructure->dqCount * substructure->dqCount;
  }
}

/*!
@brief free quantities related to processing mobility
*/
void s2_context_freeMobility(s2_context_t* context) {

  for(int i = 0; i < context->activeJointCount; i++) {
    context->jointPointers[i]->mobilityNumber = NULL;
  }
  s2_array_free(context->mobilityNumbers);
  s2_array_free(context->rightNullSpace);
  context->rightNullSpaceCount = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    context->substructures[i].rightNullSpace = NULL;
    context->substructures[i].mobilityNumbers = NULL;
  }
}

/*!
  @brief compute global instantaneous mobility, to be called after
  mobility has been computed within each substructure
*/
void s2_context_computeGlobalMobility(s2_context_t* context) {

  context->mobility =
    cblas_dasum(context->dqCount, context->mobilityNumbers, 1);
}

/*!
  @brief vector copy B matrix into null space array

  @param rank [in] rank of Jacobian matrix
  @param dqCount [in] count of variables in substructure matrix
  @param pc [in] column permutation array
  @param J [in] pointer to start of Jacobian matrix
  @param Z [out] pointer to start of null space array
*/
void s2_substructure_copyNullSpace(s2_substructure_t* substructure) {

  /* dqCount - rank is the column dimension of null space */
  int colDim = substructure->dqCount - substructure->rank;
  for(int i = 0; i < substructure->rank; i++) {

    cblas_dcopy(
      colDim,
      substructure->J + (i * substructure->dqCount + substructure->rank), 1,
      substructure->rightNullSpace + (substructure->pc_12[i] * colDim), 1);
  }

  /* set the remaining matrix to identity */
  int j = 0;
  for(int i = substructure->rank; i < substructure->dqCount; i++) {
    substructure->rightNullSpace[substructure->pc_12[i] * colDim + j++] = 1.0;
  }
}

/*!
@brief Function for computing mobility numbers, s2_jacobian_setPO1
must be called prior to this function

This function evaluates the variable mobility numbers in
PO2. It first forms an n by n-r matrix from the
submatrix J[0:r-1][r:n-1] stacked above an (n-r) by (n-r)
identity matrix. Then it calls GramSchmidt() to
orthonormalize the columns and return them in the same
matrix. Finally it returns the square of row norms of
this matrix as mobility numbers in the mn[] array.

@param context [in]
*/
void s2_context_computeMobility(s2_context_t* context) {

  s2_context_updateDH(context);

  s2_substructure_t* substructure = context->substructures;
  for(int i = 0; i < context->substructureCount; i++, substructure++) {
    s2_substructure_setJacobianPO2(substructure);
    s2_substructure_factorJacobianPO2(substructure);
    s2_substructure_setBMatrix(substructure);
    s2_substructure_computeMobility(substructure);
  }
  s2_context_computeGlobalMobility(context);
}

/*!
  @brief minimal version of computing mobility numbers, assumes
  jacobian of substructre has already been factored using full factor
  and mobility numbers have been initialized

  this function requires that s2_jacobian_setPO2_substructure,
  s2_jacobian_factorPO2_substructure and s2_jacobian_setBMatrix_substructure has
  been called prior to execution
*/
void s2_substructure_computeMobility(s2_substructure_t* substructure) {

  s2_substructure_copyNullSpace(substructure);

  if(substructure->dqCount - substructure->rank == 0) {
    printf("zero mobility detected for substructure\n");
    memset(substructure->mobilityNumbers, 0,
           substructure->dqCount * sizeof(double));
    return;
  }

  s2_gramSchmidt(substructure->dqCount,
                 substructure->dqCount - substructure->rank,
                 substructure->rightNullSpace);

  s2_mobilityNumbers(
    substructure->dqCount, substructure->dqCount - substructure->rank,
    substructure->rightNullSpace, substructure->mobilityNumbers);

  // printf("mobility numbers: ");
  // s2_array_print(substructure->dqCount, substructure->mobilityNumbers);
  // printf("rank: %d\n", substructure->rank);
}
