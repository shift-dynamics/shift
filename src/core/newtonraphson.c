/**
@file
@author Kristopher Wehage
@copyright Kristopher Wehage 2016
@section DESCRIPTION

Utilities to perform newton raphson iteration to remove constraint violations
*/

#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "context.h"
#include "jacobian.h"
#include "joint.h"
#include "mobility.h"
#include "substructure.h"
#include "topology.h"

void s2_printNewtonRaphsonError(s2_substructure_t* substructure,
                                double initialError, double finalError,
                                double* qCopy);

void s2_substructure_firstOrderUpdate(s2_substructure_t* substructure) {

  /* rows in B matrix */
  for(int i = 0; i < substructure->rank; i++) {
    /* columns in B matrix */
    for(int j = substructure->rank; j < substructure->dqCount; j++) {

      substructure->q[substructure->pc_12[i]] -=
        substructure->J[i * substructure->dqCount + j] *
        substructure->deltaq[substructure->pc_12[j]];
    }
  }
}

/*!
@brief computes constraint error in a substructure

error = sqrt(cs[] . cs[]) / constraintEquationCount
*/
double
s2_substructure_evaluateConstraintError(s2_substructure_t* substructure) {

  return sqrt(cblas_ddot(substructure->constraintEquationCount,
                         substructure->cs, 1, substructure->cs, 1)) /
         substructure->constraintEquationCount;
}

/*!
@brief computes error in change of delta q for a substructure

error = sqrt(deltaq[] . deltaq[]) / dqCount
*/
double
s2_newtonRaphson_evaluateqError_substructure(s2_substructure_t* substructure) {

  return sqrt(cblas_ddot(substructure->dqCount, substructure->deltaq, 1,
                         substructure->deltaq, 1)) /
         substructure->dqCount;
}

/*!
@brief Utility for solving a system of equations L*D*U*dxd = -cp

Based on a flag set by the calling function, EquationSolver() will
either solve L*U*dxd = -cp or dxd = -B*dxi where B = U^(-1)*UR
was stored in J[0:r-1][r:n-1]. The L, D^(-1), and U matrices are
stored in J[0:r-1][0:r-1]. In the first case, dxi is set to zero
and Newton-Raphson iteration will be used to update xd. In the second
case dxi represents a small increment in xi, and dxd represents a
small change in xd consistent, to first order, with the primary
constraint equations cp(x).

Entries in the column matrices dx[0:n-1] and c[0:m-1] are in original order.
Entries in J[0:m-1][0:n-1] are in permuted order. The first r entries of
pr[0:m-1] will be used to access the permuted primary entries in cp[0:r-1].
The first r entries of pc[0:n-1] will be used to access the permuted
dependent entries in dx[0:n-1]. And the remaining n-r entries of pc[0:n-1]
will be used to access the permuted independent entries in dx[0:n-1].

@param m row dimension
@param n column dimension
@param **J Jacobian matrix
@param *pr row permutation array
@param *pc column permutation array
@param r matrix rank
@param *dx incremental change in variables
@param *c column matrix of constraint equations
*/
void s2_substructure_newtonRaphsonSolver(s2_substructure_t* substructure) {

  memset(substructure->deltaq, 0, substructure->dqCount * sizeof(double));

  /* cs <-  L^-1 cs */
  for(int i = 0; i < substructure->rank; i++) {
    for(int j = 0; j < i; j++) {
      substructure->cs[substructure->pr_12[i]] -=
        substructure->J[i * substructure->dqCount + j] *
        substructure->cs[substructure->pr_12[j]];
    }
  }

  /* deltaq <- U^-1 * (L^-1 * cs) */
  for(int i = substructure->rank - 1; i > -1; i--) {
    for(int j = substructure->rank - 1; j > i; j--) {
      substructure->cs[substructure->pr_12[i]] -=
        substructure->J[i * substructure->dqCount + j] *
        substructure->deltaq[substructure->pc_12[j]];
    }
    substructure->deltaq[substructure->pc_12[i]] =
      substructure->cs[substructure->pr_12[i]] /
      substructure->J[i * substructure->dqCount + i];
  }
}

/*!
@brief update q array
@param a kinematic substructure to process
*/
void s2_substructure_updateq(s2_substructure_t* substructure) {
  s2_context_t* context = substructure->context;
  int substructureIndex = substructure->index;
  int dqIndex = substructure->dqOffset;
  int qIndex = substructure->qOffset;
  // printf("\n");
  for(int j = 1; j < context->substructureMembersCount[substructureIndex];
      j++) {
    // printf("qIndex: %d/%d dqIndex: %d/%d\n", qIndex, context->qCount,
    // dqIndex, context->dqCount);
    switch(
      context->jointPointers[context->substructureMembers[substructureIndex][j]]
        ->jointType) {
      case S2_JOINT_RIGID: {
        printf("rigid joint detected!\n");
        break;
      }
      case S2_JOINT_FLOAT: {
        context->q[qIndex++] -= context->deltaq[dqIndex++];
        context->q[qIndex++] -= context->deltaq[dqIndex++];
        context->q[qIndex++] -= context->deltaq[dqIndex++];
        /* falls through to spherical case to process rotational dof */
      }
      case S2_JOINT_SPHERICAL: {
        // bool anyNonZero = false;
        // for(int i = 0; i < 3; i++) {
        //   if(fabs(context->deltaq[dqIndex + i]) > 1e-9) {
        //     anyNonZero = true;
        //     break;
        //   }
        // }
        // if(anyNonZero) {
        //   printf(
        //     "  nonzero dq: %g %g %g\n",
        //     context->deltaq[dqIndex + 0],
        //     context->deltaq[dqIndex + 1],
        //     context->deltaq[dqIndex + 2]
        //   );
        // }
        /* store 4 terms deltaq at substructure->deltaq[qIndex]
          take 3 terms from substructure->deltaq[dqIndex] */

        // printf("[%d] processing spherical joint in newton raphson: \n  ", j);
        // s2_stackstr_print(stdout, context->jointPointers[
        //   context->substructureMembers[substructureIndex][j]]->name);
        // printf("  deltaq: ");
        // s2_vector_print(stdout, (s2_vector_t*)(context->deltaq + dqIndex));
        // printf("  qi: ");
        // s2_quaternion_print(stdout, (s2_quaternion_t*)(context->q + qIndex));
        //
        // double q_prev[4];
        // memcpy(&q_prev, context->q + qIndex, 4 * sizeof(double));
        s2_subtract_dpiFromq((s2_vector_t*)(context->deltaq + dqIndex),
                             (s2_quaternion_t*)(context->q + qIndex));
        s2_quaternion_normalize((s2_quaternion_t*)(context->q + qIndex));
        // printf("  qo: ");
        // s2_quaternion_print(stdout, (s2_quaternion_t*)(context->q + qIndex));
        // printf("  updated: ");
        // for(int i = 0; i < 4; i++) {
        //   printf("%s ", fabs(q_prev[i] - context->q[qIndex + i]) > 1e-9 ?
        //   "true" : "false");
        // }
        // printf("\n");
        qIndex += 4;
        dqIndex += 3;
        break;
      }
      default: {
        if(fabs(context->deltaq[dqIndex]) > 1e-9) {
          printf("nonzero dq: %g\n", context->deltaq[dqIndex]);
        }
        context->q[qIndex++] -= context->deltaq[dqIndex++];
        break;
      }
    }
  }
}

/*!
@brief This function removes constraints for a substructure
using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)

on exit, sets the sparsity pattern of J in the Jb boolean matrix

@param substructure [in]
@return int -1 indicates Newton-Raphson was not successful, 0 indicates
  Newton-Raphson completed succesfully and converged to the error tolerance
required for rank determination
*/
int s2_substructure_removeConstraintViolations(
  s2_substructure_t* substructure) {

  s2_context_t* context = substructure->context;
  /*
  Iteratively update x to satisfy the constraint equations.
  dx[colDim] contains changes in x[colDim] for current N-R iteration.
  error = sqrt(cs[] . cs[]) / constraintEquationCount
  < eps when converged.
  */
  int maxStep = 100;
  double qCopy[context->qCount];
  memcpy(qCopy, context->q, context->qCount * sizeof(double));
  // FILE* debug = context->modelParameters->debugFilePointer;
  // fprintf(debug, "\nq before NR for substructure [%d]:\n",
  // substructureIndex);
  //
  // printf("jointCount: %d\n", substructure->jointCount);
  // for(int i = 1; i < substructure->jointCount; i++) {
  //   int jointIndex = i + substructure->jointOffset;
  //   int qOffset = substructure->qOffset;
  //
  //   s2_joint_t* joint = context->jointPointers[jointIndex];
  //   int count;
  //   switch(joint->jointType) {
  //     case S2_JOINT_RIGID: {
  //       count = 0;
  //       break;
  //     }
  //     case S2_JOINT_FLOAT: {
  //       count = 7;
  //       break;
  //     }
  //     case S2_JOINT_SPHERICAL: {
  //       count = 4;
  //       break;
  //     } default: {
  //       count = 1;
  //       break;
  //     }
  //   }
  //   fprintf(stdout, "[%d] ", jointIndex);
  //   s2_stackstr_print(stdout, joint->name);
  //   for(int i = 0; i < count; i++) {
  //     fprintf(stdout, " % 0.12g ", context->q[qOffset++]);
  //   }
  //   printf("\n");
  // }
  // s2_array_print(stdout, context->qCount, context->q);
  //
  // fprintf(stdout, "   qOffset: %d\n", substructure->qOffset);

  s2_context_updateDH(context);
  s2_substructure_evaluateConstraints(substructure);
  double initialError = s2_substructure_evaluateConstraintError(substructure);
  fprintf(stdout, "  initial error: %g\n", initialError);

  int step = 0;
  double error = context->epsNewtonRaphsonRankDetermination + 1;
  while(error > context->epsNewtonRaphsonRankDetermination && step < maxStep) {
    s2_substructure_setJacobianPO1(substructure);
    s2_substructure_newtonRaphsonSolver(substructure);
    s2_substructure_updateq(substructure);
    s2_context_updateDH(context);
    s2_substructure_evaluateConstraints(substructure);
    error = s2_substructure_evaluateConstraintError(substructure);
    ++step;
    fprintf(stdout, "  [%d] error: %g\n", step, error);
  }
  s2_context_updateDH(context);
  s2_substructure_setJacobianBooleanPO2(substructure);
  if(step == maxStep) {
    s2_printNewtonRaphsonError(substructure, initialError, error, qCopy);
    return -1;
  } else {
    return 0;
  }
}

/*!
@brief This function removes constraints for the entire mechanism
using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)

@param context [in] an s2_context_t datastructure
*/
int s2_context_removeConstraintViolations(s2_context_t* self) {

  bool constraintsSatisfied = true;
  for(int i = 0; i < self->substructureCount; i++) {
    s2_substructure_t* substructure = self->substructures + i;
    int returnVal = s2_substructure_removeConstraintViolations(substructure);
    if(returnVal) {
      constraintsSatisfied = false;
    }
  }
  s2_context_updateGLReferenceFrames(self);
  if(constraintsSatisfied) {
    return 0;
  } else {
    return -1;
  }
}

void s2_printNewtonRaphsonError(s2_substructure_t* substructure,
                                double initialError, double finalError,
                                double* qCopy) {

  s2_context_t* context = substructure->context;
  FILE* f = s2_context_getDebugPointer(context, 0);
  fprintf(f,
          "\n\n>>> Error: could not resolve constraint violations for "
          "substructure [%d]\n",
          substructure->index);
  fprintf(f, "initial error: %f, final error: %f\n", initialError, finalError);
  fprintf(f, "  context->qCount: %d\n", context->qCount);
  fprintf(f, "  substructure->qCount: %d\n", substructure->qCount);
  fprintf(f, "  qOffset: %d\n", substructure->qOffset);
  assert(context->q + substructure->qOffset == substructure->q);
  fprintf(f, " states before Newton-Raphson:\n   ");
  s2_array_print(f, context->qCount, qCopy);
  fprintf(f, " states after Newton-Raphson:\n    ");
  s2_array_print(f, context->qCount, context->q);
  fprintf(f, " values that have changed:\n       ");
  for(int i = 0; i < context->qCount; i++) {
    fprintf(f, "%s ", fabs(context->q[i] - qCopy[i]) > 1e-9 ? "true" : "false");
  }
  printf(">>>\n\n");
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables.

Does not set the jacobian boolean matrix, which is necessary for solving
the equations of motion.

@param context [in] s2_context_t datastructure
*/
int s2_substructure_removeConstraintViolationsFast(
  s2_substructure_t* substructure, bool useTightTolerance) {

  s2_context_t* context = substructure->context;
  s2_context_updateDH(context);
  s2_substructure_evaluateConstraints(substructure);
  double initialError = s2_substructure_evaluateConstraintError(substructure);
  printf("initial error: %f\n", initialError);

  double qCopy[context->qCount];
  memcpy(qCopy, context->q, context->qCount * sizeof(double));
  const int maxStep = 100;
  double tol;
  if(useTightTolerance) {
    tol = context->epsNewtonRaphsonRankDetermination;
  } else {
    tol = context->epsNewtonRaphson;
  }
  s2_substructure_setJacobianPO2(substructure);
  s2_substructure_factorJacobianPO2(substructure);
  s2_substructure_evaluateConstraints(substructure);
  int step = 0;
  double error = tol + 1;
  while(error > tol && step < maxStep) {
    s2_substructure_newtonRaphsonSolver(substructure);
    s2_substructure_updateq(substructure);
    s2_context_updateDH(context);
    s2_substructure_evaluateConstraints(substructure);
    error = s2_substructure_evaluateConstraintError(substructure);
    step++;
    // printf("  [%d] error: %g\n", step, error);
  }

  // printf("  q after NR for substructure [%d]: ", substructureIndex);
  // s2_array_print(substructure->qCount, substructure->q);
  if(step == maxStep) {
    s2_printNewtonRaphsonError(substructure, initialError, error, qCopy);
    return -1;
  } else {
    return 0;
  }
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables. This function
requires that setJacobianPO1 has been called in every substructure.

@param context [in] an s2_context_t datastructure
*/
void s2_context_removeConstraintViolationsFast(s2_context_t* context,
                                               bool useTightTolerance) {

  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    s2_substructure_removeConstraintViolationsFast(substructure,
                                                   useTightTolerance);
  }
}

/*!
@brief drive down constraint violations, then perform an initial full factor
to set up slider for repositioning mechanism

removes constraint violations and performs a specified
full factor in PO1, permutations are used for subsequent
fast factorization
*/
void s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
  s2_substructure_t* substructure, int specifiedCoordinateIndex) {

  s2_context_t* context = substructure->context;
  s2_context_updateDH(context);
  s2_substructure_setJacobianPO1(substructure);
  s2_substructure_removeConstraintViolationsFast(substructure, true);
  // FILE* debug = context->modelParameters->debugFilePointer;
  // fprintf(debug, "rank after set specified: %d\n", substructure->rank);
  s2_substructure_setJacobianPO1(substructure);
  s2_substructure_computeMobility(substructure);
  s2_substructure_setJacobianPO1Specified(substructure,
                                          specifiedCoordinateIndex);
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one of its independent variables.

Full factor with a specified coordinate must be called prior to calling this
function. A lower error tolerance (context->epsNewtonRaphson) is used to
advance the position of the mechanism for visualization purposes, but a
higher tolerance (context->epsNewtonRaphsonRankDetermination) is used
for the purpose of determining the mechanism rank.

@param substructure [in] pointer to substructure to process
@param specifiedCoordinateIndex [in] global index of coordinate to set as
specified
*/
int s2_substructure_incrementSpecifiedCoordinate(
  s2_substructure_t* substructure, int specifiedCoordinateIndex, double value,
  s2_joint_enum_t jointType, s2_move_enum_t moveType) {

  s2_context_t* context = substructure->context;
  memset(substructure->deltaq, 0, substructure->dqCount * sizeof(double));

  switch(moveType) {
    case S2_JOINT_INCREMENT: {
      context->deltaq[specifiedCoordinateIndex] = value;
      context->q[specifiedCoordinateIndex] += value;
      break;
    }
    case S2_JOINT_TARGET: {
      context->deltaq[specifiedCoordinateIndex] =
        value - context->q[specifiedCoordinateIndex];
      context->q[specifiedCoordinateIndex] = value;
      break;
    }
  }
  s2_context_updateDH(context);
  s2_substructure_setJacobianPO1Specified(substructure,
                                          specifiedCoordinateIndex);
  s2_substructure_firstOrderUpdate(substructure);
  s2_substructure_removeConstraintViolationsFast(substructure, true);
  s2_substructure_setBMatrix(substructure);
  return 0;
}
