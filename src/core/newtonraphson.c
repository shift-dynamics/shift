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
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#include "context.h"
#include "substructure.h"
#include "joint.h"



void s2_newtonRaphson_firstOrderUpdate(
  s2_substructure_t* substructure) {

  /* rows in B matrix */
  for(int i = 0; i < substructure->rank; i++) {

    /* columns in B matrix */
    for(
      int j = substructure->rank;
      j < substructure->dqCount;
      j++) {

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
double s2_newtonRaphson_evaluateConstraintError_substructure(
  s2_substructure_t* substructure) {

  return sqrt(cblas_ddot(
    substructure->constraintEquationCount,
    substructure->cs,
    1,
    substructure->cs,
    1)) / substructure->constraintEquationCount;
}


/*!
@brief computes error in change of delta q for a substructure

error = sqrt(deltaq[] . deltaq[]) / dqCount
*/
double s2_newtonRaphson_evaluateqError_substructure(
  s2_substructure_t* substructure) {

  return sqrt(cblas_ddot(
    substructure->dqCount,
    substructure->deltaq,
    1,
    substructure->deltaq,
    1)) / substructure->dqCount;
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
void s2_newtonRaphson_solver(
  s2_context_t* context,
  int substructureIndex) {

  s2_substructure_t* substructure =
    context->substructures + substructureIndex;

  memset(substructure->deltaq, 0, substructure->dqCount * sizeof(double));

  /*
    cs <-  L^-1 cs
  */
  for(int i = 0; i < substructure->rank; i++) {
    for(int j = 0; j < i; j++) {
      substructure->cs[substructure->pr_12[i]] -=
        substructure->J[i * substructure->dqCount + j] *
        substructure->cs[substructure->pr_12[j]];
    }
  }

  /*
    deltaq <- U^-1 * (L^-1 * cs)
  */
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

*/
void s2_newtonRaphson_updateq(
  s2_context_t* context,
  int substructureIndex) {

  s2_substructure_t* substructure =
    context->substructures + substructureIndex;
  // printf("  deltaq: ");
  // s2_array_print(substructure->dqCount, substructure->deltaq);
  // for(int i = 0; i < substructure->dqCount; i++) {
  //   printf("%e ", substructure->deltaq[i]);
  // }
  // printf("\n");

  int dqIndex = substructure->dqOffset;
  int qIndex = substructure->qOffset;
  for(int j = 1; j < context->substructureMembersCount[substructureIndex]; j++) {

    switch(
      context->jointPointers[
        context->substructureMembers[substructureIndex][j]]->jointType) {
      case S2_JOINT_FLOAT: {
        /* take 6 entries at position dqIndex, expand to 7 entries and
           store at position dqIndex */
        cblas_daxpy(3, -1, context->deltaq + dqIndex, 1, context->q + qIndex, 1);
        s2_subtract_dpiFromq(
          (s2_vector_t*)(context->deltaq + dqIndex + 3),
          (s2_quaternion_t*)(context->q + qIndex + 3));
        s2_quaternion_normalize(
          (s2_quaternion_t*)(context->q + qIndex + 3));
        qIndex += 7;
        dqIndex += 6;

        /* store 4 terms deltaq at substructure->deltaq[qIndex + 3]
          take 3 terms from substructure->deltaq[dqIndex + 3] */
      } break;
      case S2_JOINT_SPHERICAL: {
        /* take 6 entries at position dqIndex, expand to 7 entries and
            store at position dqIndex */
        s2_subtract_dpiFromq(
          (s2_vector_t*)(context->deltaq + dqIndex),
          (s2_quaternion_t*)(context->q + qIndex));
        s2_quaternion_normalize((s2_quaternion_t*)(context->q + qIndex));
        qIndex += 4;
        dqIndex += 3;

        /* store 4 terms deltaq at substructure->deltaq[qIndex]
          take 3 terms from substructure->deltaq[dqIndex] */
      } break;
      default: {
        context->q[qIndex++] -= context->deltaq[dqIndex++];
      } break;
    }
  }
}


/*!
@brief This function removes constraints for the entire mechanism
using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)

on exit, sets the sparsity pattern of J in the Jb boolean matrix

@param context [in] an s2_context_t datastructure
*/
int s2_newtonRaphson_removeConstraintViolations_substructure(
  s2_context_t* context,
  int substructureIndex) {

  /*
  Iteratively update x to satisfy the constraint equations.
  dx[colDim] contains changes in x[colDim] for current N-R iteration.
  error = sqrt(cs[] . cs[]) / constraintEquationCount
  < eps when converged.
  */
  int maxStep = 100;

  s2_substructure_t* substructure =
    context->substructures + substructureIndex;

  s2_updateDH(context);

  // FILE* debug = context->modelParameters->debugFilePointer;
  // fprintf(debug, "\nq before NR for substructure [%d]:\n", substructureIndex);
  //
  // for(int i = 0; i < substructure->qCount; i++) {
  //   int jointIndex = i + substructure->qOffset;
  //   fprintf(debug, "[%d] ", jointIndex);
  //   fprintf(debug, " % 0.12g ", context->q[jointIndex]);
  //   s2_stackstr_print(
  //     debug,
  //     context->jointPointers[jointIndex]->name);
  // }
  // s2_array_print(context->qCount, context->q);

  // fprintf(debug, "   qOffset: %d\n", substructure->qOffset);

  s2_jacobian_evaluateConstraints_substructure(context, substructure);

  int step = 0;
  double error = context->epsNewtonRaphsonRankDetermination + 1;
  while(error > context->epsNewtonRaphsonRankDetermination && step < maxStep) {
    s2_jacobian_setPO1_substructure(context, substructureIndex);
    s2_newtonRaphson_solver(context, substructureIndex);
    s2_newtonRaphson_updateq(context, substructureIndex);
    s2_updateDH(context);
    s2_jacobian_evaluateConstraints_substructure(context, substructure);
    error = s2_newtonRaphson_evaluateConstraintError_substructure(
      substructure);
    ++step;
    // fprintf(debug, "  [%d] error: %g\n", step, error);
  }

  // fprintf(
  //   debug,
  //   "  q after NR for substructure [%d]:\n",
  //   substructureIndex);
  // for(int i = 0; i < substructure->qCount; i++) {
  //   int jointIndex = i + substructure->qOffset;
  //   printf("[%d] ", jointIndex);
  //   printf(" % 0.12g ", context->q[jointIndex]);
  //   s2_stackstr_print(
  //     debug,
  //     context->jointPointers[jointIndex]->name);
  // }


  s2_updateDH(context);
  // s2_jacobian_setBMatrix_substructure(substructure);
  s2_jacobian_setJacobianBooleanPO2_substructure(
    context,
    substructureIndex);
  if(step == maxStep) {
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
int s2_newtonRaphson_removeConstraintViolations(
  s2_context_t* context) {

  bool constraintsSatisfied = true;
  for(int i = 0; i < context->substructureCount; i++) {
    int returnVal =
      s2_newtonRaphson_removeConstraintViolations_substructure(
        context,
        i);
    if(returnVal) {
      constraintsSatisfied = false;
    }
  }
  s2_updateGLReferenceFrames(context);
  if(constraintsSatisfied) {
    return 0;
  } else {
    return -1;
  }
}


/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables.

Does not set the jacobian boolean matrix, which is necessary for solving
the equations of motion.

@param context [in] s2_context_t datastructure
*/
int s2_newtonRaphson_removeConstraintViolationsFast_substructure(
  s2_context_t* context,
  int substructureIndex,
  bool useTightTolerance) {

  s2_substructure_t* substructure =
    context->substructures + substructureIndex;

  // exit(1);
  // printf("  q before NR for substructure [%d]: ", substructureIndex);
  // s2_array_print(context->qCount, context->q);

  // printf("cs (PO1): ");
  // s2_array_print(
  //   substructure->constraintEquationCount,
  //   substructure->cs);

  s2_updateDH(context);

  // printf("  q before NR for substructure [%d]: ", substructureIndex);
  // s2_array_print(context->qCount, context->q);

  s2_jacobian_evaluateConstraints_substructure(
    context,
    substructure);

  // printf("cs (PO1): ");
  // s2_array_print(
  //   substructure->constraintEquationCount,
  //   substructure->cs);

  // printf(
  //   "initial error: %f\n", s2_newtonRaphson_evaluateConstraintError_substructure(
  //     substructure));

  double initialError =
    s2_newtonRaphson_evaluateConstraintError_substructure(substructure);

  double qCopy[context->qCount];
  memcpy(qCopy, context->q, context->qCount * sizeof(double));
  int step = 0;
  int maxStep = 100;
  double tol;
  if(useTightTolerance) {
    tol = context->epsNewtonRaphsonRankDetermination;
  } else {
    tol = context->epsNewtonRaphson;
  }
  double error = tol + 1;

  s2_jacobian_setPO2_substructure(
    context, substructureIndex);

  // printf("set jacobian in po2:\n");
  // s2_array_print2D(
  //   substructure->constraintEquationCount,
  //   substructure->dqCount,
  //   true,
  //   substructure->J);

  s2_jacobian_factorPO2_substructure(
    substructure);

  // printf("factored jacobian:\n");
  // s2_array_print2D(
  //   substructure->constraintEquationCount,
  //   substructure->dqCount,
  //   true,
  //   substructure->J);

  s2_jacobian_evaluateConstraints_substructure(
    context,
    substructure);

  while(error > tol && step < maxStep) {
    s2_newtonRaphson_solver(context, substructureIndex);

    // printf("deltaq: ");
    // s2_array_print(substructure->dqCount, substructure->deltaq);

    s2_newtonRaphson_updateq(context, substructureIndex);
    s2_updateDH(context);
    s2_jacobian_evaluateConstraints_substructure(
      context,
      substructure);
    error = s2_newtonRaphson_evaluateConstraintError_substructure(
      substructure);
    // s2_jacobian_setPO2_substructure(
    //   context, substructureIndex);
    //
    // s2_jacobian_factorPO2_substructure(
    //   substructure);
    step++;
    // printf("  [%d] error: %g\n", step, error);
  }

  // printf("  q after NR for substructure [%d]: ", substructureIndex);
  // s2_array_print(substructure->qCount, substructure->q);
  if(step == maxStep) {
    FILE* debug = context->modelParameters->debugFilePointer;
    fprintf(
      debug,
      "\n\n>>> Error: could not resolve constraint violations for substructure [%d]\n",
      substructureIndex);
    fprintf(
      debug, "initial error: %f, final error: %f\n", initialError, error);
    fprintf(debug, "  qCount: %d\n", substructure->qCount);
    fprintf(debug, "  qOffset: %d\n", substructure->qOffset);
    assert(
      context->q + substructure->qOffset ==
        substructure->q);
    fprintf(debug, " states before newton raphson:\n   ");
    s2_array_print(debug, context->dqCount, qCopy);
    fprintf(debug, " states after newton raphson:\n    ");
    s2_array_print(debug, context->qCount, context->q);
    printf(">>>\n\n");

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
void s2_newtonRaphson_removeConstraintViolationsFast(
  s2_context_t* context,
  bool useTightTolerance) {

  for(int i = 0; i < context->substructureCount; i++) {
    s2_newtonRaphson_removeConstraintViolationsFast_substructure(
      context,
      i,
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
void s2_newtonRaphson_setup_specified(
  s2_context_t* context,
  int substructureIndex,
  int specifiedCoordinateIndex) {

  s2_substructure_t* substructure =
    context->substructures + substructureIndex;

  s2_updateDH(context);
  s2_jacobian_setPO1_substructure(
    context,
    substructureIndex);

  s2_newtonRaphson_removeConstraintViolationsFast_substructure(
    context,
    substructureIndex,
    true);

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "rank after set specified: %d\n", substructure->rank);

  s2_jacobian_setPO1_substructure(context, substructureIndex);
  s2_mobility_compute_substructure(substructure);

  // fprintf(debug, "mobility numbers: ");
  // s2_array_print(
  //   debug,
  //   substructure->dqCount,
  //   substructure->mobilityNumbers);
  // s2_array_print2D(
  //   debug,
  //   substructure->constraintEquationCount,
  //   substructure->dqCount,
  //   true,
  //   substructure->J);

  s2_jacobian_setPO1Specified_substructure(
    context,
    substructureIndex,
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

@param context [in] an s2_context_t datastructure
@param substructureIndex [in] index of substructure to process
@param specifiedCoordinateIndex [in] global index of coordinate to set as specified
*/
int s2_newtonRaphson_increment_specified(
  s2_context_t* context,
  int substructureIndex,
  int specifiedCoordinateIndex,
  double value,
  s2_joint_enum_t jointType,
  s2_move_enum_t moveType) {

  s2_substructure_t* substructure =
    context->substructures + substructureIndex;
  memset(
    substructure->deltaq,
    0,
    substructure->dqCount * sizeof(double));

  switch(moveType) {
    case S2_JOINT_INCREMENT: {
      context->deltaq[specifiedCoordinateIndex] = value;
      context->q[specifiedCoordinateIndex] += value;
    } break;
    case S2_JOINT_TARGET: {
      context->deltaq[specifiedCoordinateIndex] =
        value - context->q[specifiedCoordinateIndex];
      context->q[specifiedCoordinateIndex] = value;
    } break;
  }

  s2_updateDH(context);
  s2_jacobian_setPO1Specified_substructure(
    context,
    substructureIndex,
    specifiedCoordinateIndex);
  s2_newtonRaphson_firstOrderUpdate(substructure);
  s2_newtonRaphson_removeConstraintViolationsFast_substructure(
    context,
    substructureIndex,
    true);

  s2_jacobian_setBMatrix_substructure(substructure);

  return 0;

}
