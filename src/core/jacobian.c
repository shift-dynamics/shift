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

#include <math.h>
#include <stdio.h>

#include "utility.h"
#include "stdlib.h"
#include "kinematics.h"
#include "jacobian.h"
#include "substructure.h"
#include "body.h"
#include "context.h"
#include "frame.h"
#include "chordconstraint.h"
#include "joint.h"
#include "memoize.h"
#include "topology.h"



/*!
@brief pivot ratio, performed on a Jacobian that has been factored

returns the ratio of the largest to the smallest pivot
*/
double s2_jacobian_pivotRatio(
  int colDim,
  int rank,
  double* J) {

  double min = fabs(J[0]);
  double max = fabs(J[0]);
  for(int i = 1; i < rank; i++) {
    min = fmin(min, fabs(J[i * colDim + i]));
  }
  for(int i = 1; i < rank; i++) {
    max = fmax(max, fabs(J[i * colDim + i]));
  }
  return(max / min);
}


/*!
@brief returns infinity norm of B matrix

*/
double s2_jacobian_infinityNormB(
  int rowDim,
  int colDim,
  int rank,
  double* J) {

  // printf("\nevaluating infinity norm B\n");
  // printf("rank: %d\n", rank);
  // printf("J in infinity norm B check:\n");
  // s2_array_print2D(rowDim, colDim, true, J);
  double max = 0;
  for(int i = 0; i < rank; i++) {
    for(int j = rank; j < colDim; j++) {
      max = fmax(max, fabs(J[i * colDim + j]));
    }
  }
  // printf("infinityNormB: %f\n\n", max);
  // assert(!isinf(max));
  return max;

}


/*!
@brief set inverse permutation

*/
void s2_jacobian_setInversePermutation(
  int n,
  int* p,
  int* pInverse) {

  for(int i = 0; i < n; i++) {
    pInverse[p[i]] = i;
  }
}


/*!
@brief set lookup tables for variables and joints, converts expanded list
to compressed list, removes "rigid" variables from compressed list,
*/
void s2_context_setJacobianLookup(
  s2_context_t* context) {

  if(context->jointLookup) {
    free(context->jointLookup);
  }
  context->jointLookup = calloc(context->activeJointCount, sizeof(int));

  if(context->dqLookup) {
    free(context->dqLookup);
  }
  context->dqLookup = calloc(context->activeJointCount, sizeof(int));

  if(context->qLookup) {
    free(context->qLookup);
  }
  context->qLookup = calloc(context->activeJointCount, sizeof(int));

  if(context->dqLookupCompressed) {
    free(context->dqLookupCompressed);
  }
  context->dqLookupCompressed =
    calloc(context->nonRigidJointCount, sizeof(int));

  int jointCount = 0;
  int dqCount = 0;
  int qCount = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    s2_joint_t* joint = context->jointPointers[i];
    switch(joint->jointType) {
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ:
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ: {
        context->jointLookup[i] = jointCount++;
        context->dqLookup[i] = dqCount++;
        context->qLookup[i] = qCount++;
      }
      break;
      case S2_JOINT_SPHERICAL: {
        context->jointLookup[i] = jointCount++;
        context->dqLookup[i] = dqCount;
        context->qLookup[i] = qCount;
        dqCount += 3;
        qCount += 4;
      }
      break;
      case S2_JOINT_FLOAT: {
        context->jointLookup[i] = jointCount++;
        context->dqLookup[i] = dqCount;
        context->qLookup[i] = qCount;
        dqCount += 6;
        qCount += 7;
      }
      break;
      case S2_JOINT_RIGID: {
        context->jointLookup[i] = -1;
        context->dqLookup[i] = -1;
        context->qLookup[i] = -1;
      }
      break;
      case S2_JOINT_INVALID:
      case S2_JOINT_PRIMITIVE:
        break;
    }
  }

  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointLookup[i] > -1) {
      context->dqLookupCompressed[context->jointLookup[i]] =
        context->dqLookup[i];
    }
  }
}


/*!
@brief Allocates memory for Jacobian matrices associated with each substructure

*/
int s2_context_mallocEquations(s2_context_t* context) {

  FILE* debug = context->modelParameters.debugFilePointer;

  s2_array_free(context->J);
  s2_arrayi_free(context->Jxx);
  s2_arrayb_free(context->Jb);
  s2_array_free(context->cs);
  s2_array_free(context->dcs);

  if(context->substructures) {
    free(context->substructures);
  }
  context->substructures =
    malloc(context->substructureCount * sizeof(s2_substructure_t));

  /*
    determine size of Jacobian
  */
  context->jacobianSize = 0;
  int constraintEquationCount = 0;

  // fprintf(debug, "allocating memory for equations:\n");
  for(int i = 0; i < context->substructureCount; i++) {

    // fprintf(debug, "  allocating memory for substructure [%d]\n", i);
    context->substructures[i].context = context;
    context->substructures[i].qCount = 0;
    context->substructures[i].dqCount = 0;
    context->substructures[i].jointCount = 0;
    context->substructures[i].index = i;

    // fprintf(debug, "processing substructure [%d]\n", i);
    // fprintf(debug, "  with members: ");
    // s2_arrayi_print(
    //   debug,
    //   context->substructureMembersCount[i],
    //   context->substructureMembers[i]);
    // fprintf(debug, "  levels: ");
    // for(int j = 1; j < context->substructureMembersCount[i]; j++) {
    //   fprintf(debug, "%d ", context->levels[context->substructureMembers[i][j]]);
    // }
    // fprintf(debug, "\n");

    /*
      substructure members are enumerated by nodes (i.e. bodies), but the
      jacobian is populated with graph edges (i.e. joints). At this point, the
      bodies and joints are ordered such that each body's unique parent joint
      is at the same index in the list of bodies and joints. The parent joint
      of the base body is not in the substructure, therefore start indexing at
      1, to include only those joints which are parents of other loop
      members
    */
    for(int j = 1; j < context->substructureMembersCount[i]; j++) {

      s2_joint_t* joint =
        context->jointPointers[context->substructureMembers[i][j]];

      switch(joint->jointType) {
        case S2_JOINT_TX:
        case S2_JOINT_TY:
        case S2_JOINT_TZ:
        case S2_JOINT_RX:
        case S2_JOINT_RY:
        case S2_JOINT_RZ: {
          context->substructures[i].qCount++;
          context->substructures[i].dqCount++;
          context->substructures[i].jointCount++;
          break;
        }
        case S2_JOINT_FLOAT: {
          context->substructures[i].qCount += 7;
          context->substructures[i].dqCount += 6;
          context->substructures[i].jointCount++;
          break;
        }
        case S2_JOINT_SPHERICAL: {
          context->substructures[i].qCount += 4;
          context->substructures[i].dqCount += 3;
          context->substructures[i].jointCount++;
          break;
        }
        case S2_JOINT_RIGID:
        case S2_JOINT_INVALID:
        case S2_JOINT_PRIMITIVE:
          break;
      }
    }
    // fprintf(debug, "\n");

    /* store total number of joints and constraints, constraint equations
       in each substructures */
    context->substructures[i].chordConstraintCount =
      context->substructureLoopCount[i];

    context->substructures[i].constraintEquationCount =
      context->substructures[i].chordConstraintCount * 6;

    constraintEquationCount +=
      context->substructures[i].constraintEquationCount;

    context->substructures[i].jointOffset = context->jacobianSize;

    context->jacobianSize +=
      context->substructures[i].constraintEquationCount *
      context->substructures[i].dqCount;

    context->substructures[i].pr_12 =
      calloc(context->substructures[i].constraintEquationCount, sizeof(int));

    context->substructures[i].pc_12 =
      calloc(context->substructures[i].dqCount, sizeof(int));

    context->substructures[i].pr_21 =
      calloc(context->substructures[i].constraintEquationCount, sizeof(int));

    context->substructures[i].pc_21 =
      calloc(context->substructures[i].dqCount, sizeof(int));

  }

  context->J = calloc(context->jacobianSize, sizeof(double));
  context->Js = calloc(context->jacobianSize, sizeof(double));
  context->Jxx = calloc(context->jacobianSize, sizeof(int));
  context->Jb = calloc(context->jacobianSize, sizeof(bool));
  context->cs = calloc(constraintEquationCount, sizeof(double));
  context->dcs = calloc(constraintEquationCount, sizeof(double));
  context->deltaq = calloc(context->dqCount, sizeof(double));

  int jacobianSize = 0;
  constraintEquationCount = 0;
  int qCount = 0;
  int dqCount = 0;

  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    substructure->J = context->J + jacobianSize;
    substructure->Jxx = context->Jxx + jacobianSize;
    substructure->Jb = context->Jb + jacobianSize;

    jacobianSize +=
      substructure->constraintEquationCount *
      substructure->dqCount;

    substructure->cs = context->cs + constraintEquationCount;
    substructure->dcs = context->dcs + constraintEquationCount;

    qCount += substructure->qCount;
    dqCount += substructure->dqCount;

    constraintEquationCount += substructure->constraintEquationCount;
  }

  if(context->isDependentPO1) {
    free(context->isDependentPO1);
  }
  context->isDependentPO1 = calloc(context->dqCount, sizeof(bool));

  if(context->isDependentPO3) {
    free(context->isDependentPO3);
  }
  context->isDependentPO3 = calloc(context->dqCount, sizeof(bool));

  if(context->hasMass) {
    free(context->hasMass);
  }
  context->hasMass = calloc(context->nonRigidJointCount, sizeof(bool));

  if(context->hasMassdq) {
    free(context->hasMassdq);
  }
  context->hasMassdq = calloc(context->dqCount, sizeof(bool));

  if(context->gamma_d) {
    free(context->gamma_d);
  }
  context->gamma_d = calloc(context->dqCount, sizeof(double));

  if(context->pv_12) {
    free(context->pv_12);
  }
  context->pv_12 = calloc(context->dqCount, sizeof(int));

  if(context->pv_21) {
    free(context->pv_21);
  }
  context->pv_21 = calloc(context->dqCount, sizeof(int));

  if(context->pv_13) {
    free(context->pv_13);
  }
  context->pv_13 = calloc(context->dqCount, sizeof(int));

  if(context->pv_31) {
    free(context->pv_31);
  }
  context->pv_31 = calloc(context->dqCount, sizeof(int));

  if(context->pv_23) {
    free(context->pv_23);
  }
  context->pv_23 = calloc(context->dqCount, sizeof(int));

  if(context->pv_32) {
    free(context->pv_32);
  }
  context->pv_32 = calloc(context->dqCount, sizeof(int));

  context->Ml_size = ((context->dqCount + 1) * (context->dqCount + 2)) / 2;

  if(context->Ml) {
    free(context->Ml);
  }
  context->Ml = calloc(context->Ml_size, sizeof(double));

  if(context->Mls) {
    free(context->Mls);
  }
  context->Mls = calloc(context->Ml_size, sizeof(double));

  if(context->Mlb) {
    free(context->Mlb);
  }
  context->Mlb = calloc(context->Ml_size, sizeof(bool));

  if(context->Mlbs) {
    free(context->Mlbs);
  }
  context->Mlbs = calloc(context->Ml_size, sizeof(bool));

  if(context->hasMass) {
    free(context->hasMass);
  }
  context->hasMass = calloc(context->nonRigidJointCount, sizeof(bool));

  if(context->hasMassdq) {
    free(context->hasMassdq);
  }
  context->hasMassdq = calloc(context->dqCount, sizeof(bool));

  if(context->jointDOF) {
    free(context->jointDOF);
  }
  context->jointDOF = calloc(context->activeJointCount, sizeof(int));

  // fprintf(debug, "dqCount: %d\n", context->dqCount);
  for(int i = 0; i < context->activeJointCount; i++) {
    if(
      !(context->bodyPointers[i]->isVirtual ||
        context->bodyPointers[i]->inertia == NULL)) {
      if(context->jointLookup[i] > -1) {
        // printf("context->jointLookup[i]: %d\n", context->jointLookup[i]);
        context->hasMass[context->jointLookup[i]] = true;
        // printf("context->dqLookup[i]: %d\n", context->dqLookup[i]);
        context->hasMassdq[context->dqLookup[i]] = true;
      }
    }
    switch(context->jointPointers[i]->jointType) {
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ:
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ: {
        context->jointDOF[i] = 1;
        break;
      }
      case S2_JOINT_SPHERICAL: {
        context->jointDOF[i] = 3;
        break;
      }
      case S2_JOINT_FLOAT: {
        context->jointDOF[i] = 6;
        break;
      }
      case S2_JOINT_RIGID: {
        context->jointDOF[i] = 0;
        break;
      }
      case S2_JOINT_INVALID:
      case S2_JOINT_PRIMITIVE:
        break;
    }
  }


  /* create compressed parent child list */
  if(context->pclCompressed) {
    free(context->pclCompressed);
  }
  context->pclCompressed = calloc(context->nonRigidJointCount, sizeof(int));

  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
      int parent = context->pcl[i];
      while(true) {
        if(parent == -1) {
          break;
        }
        if(context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
          break;
        }
        parent = context->pcl[parent];
      }
      //printf("i: %d\n", i);
      //printf("parent: %d\n", parent);
      // fprintf(debug, "parent: %d\n", parent);
      if(parent == -1) {
        context->pclCompressed[context->jointLookup[i]] = -1;
      } else {
        context->pclCompressed[context->jointLookup[i]] =
          context->jointLookup[parent];
      }
    }
  }


  /* create parent child list for dq */
  if(context->pcldq) {
    free(context->pcldq);
  }
  context->pcldq = calloc(context->dqCount, sizeof(int));

  for(int i = 0; i < context->activeJointCount; i++) {
    int parentIndex = context->pcl[i];
    int variableIndex = context->dqLookup[i];
    if(variableIndex != -1) {
      if(parentIndex != -1) {
        context->pcldq[variableIndex] = context->dqLookup[parentIndex];
      } else {
        context->pcldq[variableIndex] = -1;
      }
    }
    int jointDOF = context->jointDOF[i];
    if(jointDOF > 1) {
      for(int j = 1; j < jointDOF; j++) {
        context->pcldq[variableIndex + j] = variableIndex + j - 1;
      }
    }
  }

  // fprintf(debug, "jointDOF: ");
  // s2_arrayi_print(debug, context->activeJointCount, context->jointDOF);
  // fprintf(debug, "jointLookup: ");
  // s2_arrayi_print(debug, context->activeJointCount, context->jointLookup);
  // fprintf(debug, "dqLookup: ");
  // s2_arrayi_print(debug, context->activeJointCount, context->dqLookup);
  // fprintf(debug, "pcl: ");
  // s2_arrayi_print(debug, context->activeJointCount, context->pcl);
  // fprintf(debug, "pclCompressed: ");
  // s2_arrayi_print(debug, context->nonRigidJointCount, context->pclCompressed);
  // fprintf(debug, "pcldq: ");
  // s2_arrayi_print(debug, context->dqCount, context->pcldq);

  /* set isRotation */
  if(context->isRotation) {
    free(context->isRotation);
  }
  context->isRotation = calloc(context->dqCount, sizeof(bool));
  for(int i = 0; i < context->activeJointCount; i++) {
    int baseIndex = context->dqLookup[i];
    switch(context->jointPointers[i]->jointType) {
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ:
      case S2_JOINT_RIGID:
      case S2_JOINT_PRIMITIVE:
      case S2_JOINT_INVALID:
        break;
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {
        context->isRotation[baseIndex] = true;
        break;
      }
      case S2_JOINT_SPHERICAL: {
        for(int j = 0; j < 3; j++) {
          context->isRotation[baseIndex + j] = true;
        }
        break;
      }
      case S2_JOINT_FLOAT: {
        for(int j = 3; j < 6; j++) {
          context->isRotation[baseIndex + j] = true;
        }
        break;
      }
    }
  }

  return 0;
}


/*! @brief compress rigid joints out of positive and negative loop member
lists and compute offsets in jacobian

*/
void s2_context_compressLoopMembers(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  context->positiveLoopMemberOffsets =
    malloc(context->chordConstraintCount * sizeof(int*));

  context->negativeLoopMemberOffsets =
    malloc(context->chordConstraintCount * sizeof(int*));

  context->positiveLoopMembersCompressedCount =
    calloc(context->chordConstraintCount, sizeof(int));

  context->negativeLoopMembersCompressedCount =
    calloc(context->chordConstraintCount, sizeof(int));

  int positiveLoopMemberCount = 0;
  int negativeLoopMemberCount = 0;

  /*
    compute total size
  */

  /* for each substructure
  j is index into local list of loops for substructure
  k is index into global list of loops */
  int k = 0;
  for(int i = 0; i < context->substructureCount; i++) {

    context->substructures[i].loopOffset = k;

    /* for each loop in substructure */
    for(
      int j = 0;
      j < context->substructureLoopCount[i];
      j++, k++) {


      /* for each positive loop members */
      for(int l = 0; l < context->positiveLoopMembersCount[k]; l++) {
        int positiveLoopIndex = context->positiveLoopMembers[k][l];
        int dof;
        switch(context->jointPointers[positiveLoopIndex]->jointType) {
          case S2_JOINT_RIGID: {
            dof = 0;
            break;
          }
          case S2_JOINT_FLOAT: {
            dof = 6;
            break;
          }
          case S2_JOINT_SPHERICAL: {
            dof = 3;
            break;
          }
          default: {
            dof = 1;
          }
        }
        for(int m = 0; m < dof; m++) {
          context->positiveLoopMembersCompressedCount[k]++;
          positiveLoopMemberCount++;
        }
      }

      /* process negative loop members */
      for(int l = 0; l < context->negativeLoopMembersCount[k]; l++) {
        int negativeLoopIndex = context->negativeLoopMembers[k][l];
        int dof;
        switch(context->jointPointers[negativeLoopIndex]->jointType) {
          case S2_JOINT_RIGID: {
            dof = 0;
            break;
          }
          case S2_JOINT_FLOAT: {
            dof = 6;
            break;
          }
          case S2_JOINT_SPHERICAL: {
            dof = 3;
            break;
          }
          default: {
            dof = 1;
          }
        }
        for(int m = 0; m < dof; m++) {
          context->negativeLoopMembersCompressedCount[k]++;
          negativeLoopMemberCount++;
        }
      }
    }
  }

  if(context->chordConstraintCount > 0) {
    // fprintf(debug, "positiveLoopMemberCount: %d\n", positiveLoopMemberCount);
    context->positiveLoopMemberOffsets[0] =
      malloc(positiveLoopMemberCount * sizeof(int));

    // fprintf(debug, "negativeLoopMemberCount: %d\n", negativeLoopMemberCount);
    context->negativeLoopMemberOffsets[0] =
      malloc(negativeLoopMemberCount * sizeof(int));
  }

  /*
    store offsets in positiveLoopMemberOffsets and
    negativeLoopMembersOffsets
  */

  /* for each substructure [i] */
  k = 0; /* loop counter */
  positiveLoopMemberCount = 0;
  negativeLoopMemberCount = 0;
  for(int i = 0; i < context->substructureCount; i++) {

    /* set offset into global variable list */
    // fprintf(debug, "all substructure members:");
    // s2_arrayi_print(
    //   debug,
    //   context->substructureMembersCount[i],
    //   context->substructureMembers[i]);
    // for(int j = 0; i < context->substructureLoopCount[i]; j++) {
    //   printf("  loop [%d]: ", j);
    //   s2_arrayi_print(
    //     context->positiveLoopMembersCompressedCount[i],
    //     context->positiveLoopMembersCompressedCount);
    // }


    int substructureOffset =
      fmin(
        context->substructureMembers[i][1],
        context->substructureMembers[i][2]);
    // int substructureOffset = context->substructureMembers[i][1];

    // fprintf(
    //   debug,
    //   "substructure offset: %d\n", substructureOffset);
    // fprintf(
    //   debug,
    //   "dqLookup[%d]: %d\n",
    //   substructureOffset,
    //   context->dqLookup[substructureOffset]);

    context->substructures[i].dqOffset =
      context->dqLookup[substructureOffset];
    context->substructures[i].dq =
      context->dq + context->substructures[i].dqOffset;
    // fprintf(
    //   debug,
    //   "context->substructures[%i].dqOffset: %d\n",
    //   i,
    //   context->substructures[i].dqOffset);

    context->substructures[i].qOffset =
      context->qLookup[substructureOffset];
    context->substructures[i].q =
      context->q + context->substructures[i].qOffset;

    // assert(
    //   context->q + context->substructures[i].qOffset ==
    //   context->substructures[i].q);
    // fprintf(
    //   debug,
    //   "context->substructures[%d].qOffset: %d\n",
    //   i,
    //   context->substructures[i].qOffset);

    context->substructures[i].deltaq =
      context->deltaq + context->substructures[i].dqOffset;

    /* for each loop in substructure [j] local loop index, [k] global loop index */
    for(int j = 0; j < context->substructureLoopCount[i]; j++, k++) {

      context->positiveLoopMemberOffsets[k] =
        context->positiveLoopMemberOffsets[0] + positiveLoopMemberCount;
      context->negativeLoopMemberOffsets[k] =
        context->negativeLoopMemberOffsets[0] + negativeLoopMemberCount;

      /* for each positive loop member [l] */
      // fprintf(
      //   debug,
      //   "positive loop [%d]\n", k);
      for(int l = 0; l < context->positiveLoopMembersCount[k]; l++) {
        // fprintf(
        //   debug,
        //   "  loop member [%d/%d]: %d ",
        //   l,
        //   positiveLoopMemberCount,
        //   context->positiveLoopMembers[k][l]);
        //
        // s2_stackstr_print(
        //   debug,
        //   context->jointPointers[
        //   context->positiveLoopMembers[k][l]]->name);

        int positiveLoopMember = context->positiveLoopMembers[k][l];
        int dof;
        switch(context->jointPointers[positiveLoopMember]->jointType) {
          case S2_JOINT_RIGID: {
            dof = 0;
            break;
          }
          case S2_JOINT_FLOAT: {
            dof = 6;
            break;
          }
          case S2_JOINT_SPHERICAL: {
            dof = 3;
            break;
          }
          default: {
            dof = 1;
          }
        }
        int dqOffset = context->dqLookup[positiveLoopMember];
        for(int m = 0; m < dof; m++) {
          context->positiveLoopMemberOffsets[0][positiveLoopMemberCount++] =
            dqOffset++;
        }
      }

      /* for each negative loop member [l]*/
      // fprintf(
      //   debug,
      //   "negative loop [%d]\n", k);
      for(int l = 0; l < context->negativeLoopMembersCount[k]; l++) {
        // fprintf(
        //   debug,
        //   "  loop member [%d/%d]: %d ",
        //   l,
        //   negativeLoopMemberCount,
        //   context->negativeLoopMembers[k][l]);
        //
        // s2_stackstr_print(
        //   debug,
        //   context->jointPointers[
        //   context->negativeLoopMembers[k][l]]->name);

        int negativeLoopMember = context->negativeLoopMembers[k][l];
        int dof;
        switch(context->jointPointers[negativeLoopMember]->jointType) {
          case S2_JOINT_RIGID: {
            dof = 0;
            break;
          }
          case S2_JOINT_FLOAT: {
            dof = 6;
            break;
          }
          case S2_JOINT_SPHERICAL: {
            dof = 3;
            break;
          }
          default: {
            dof = 1;
          }
        }
        int dqOffset = context->dqLookup[negativeLoopMember];
        for(int m = 0; m < dof; m++) {
          context->negativeLoopMemberOffsets[0][negativeLoopMemberCount++] =
            dqOffset++;
        }
      }
    }
  }

  /* subtract offset off each positive / negative loopMemberOffset */
  // k = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   for(int j = 0; j < context->substructureLoopCount[i]; j++, k++) {
  //     for(int l = 0; l < context->positiveLoopMembersCompressedCount[k]; l++) {
  //       context->positiveLoopMemberOffsets[k][l] -= substructure->dqOffset;
  //     }
  //     for(int l = 0; l < context->negativeLoopMembersCompressedCount[k]; l++) {
  //       context->negativeLoopMemberOffsets[k][l] -= substructure->dqOffset;
  //     }
  //     printf("\n");
  //   }
  // }

  /*
    verify result
  */
  // k = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   fprintf(
  //     debug,
  //     "column indices for substructure [%d]\n", i);
  //   for(int j = 0; j < context->substructureLoopCount[i]; j++, k++) {
  //     fprintf(
  //       debug,
  //       "positive loop members [%d]: ",
  //       context->positiveLoopMembersCompressedCount[k]);
  //     for(int l = 0; l < context->positiveLoopMembersCompressedCount[k]; l++) {
  //       fprintf(
  //         debug,
  //         "%d ", context->positiveLoopMemberOffsets[k][l]);
  //     }
  //     fprintf(
  //       debug,
  //       "\nnegative loop members [%d]: ",
  //       context->negativeLoopMembersCompressedCount[k]);
  //     for(int l = 0; l < context->negativeLoopMembersCompressedCount[k]; l++) {
  //       fprintf(
  //         debug,
  //         "%d ", context->negativeLoopMemberOffsets[k][l]);
  //     }
  //     fprintf(debug, "\n");
  //   }
  // }
}


/*! @brief set initial jacobian in permutation order 1 and perform an initial
call to GCP using s2_factor_LU_full

a global Jacobian is stored in the linear array context->J
a pointer to the local Jacobian is stored in context->substructure[i]->J

note the Jacobian is stored in row-major order
*/
void s2_context_setJacobianPO1(s2_context_t* context) {
  s2_substructure_t* substructure = context->substructures;
  for(int i = 0; i < context->substructureCount; i++, substructure++) {
    s2_substructure_setJacobianPO1(substructure);
  }
}


/*! @brief set initial jacobian in permutation order 1 and perform an initial
call to GCP using s2_factor_LU_full on the basis of one substructure

a global Jacobian is stored in the linear array context->J
a pointer to the local Jacobian is stored in context->substructure[i]->J

the Jacobian is stored in row-major order
*/
void s2_substructure_setJacobianPO1(s2_substructure_t* substructure) {

  int substructureIndex = substructure->index;
  s2_context_t* context = substructure->context;

  memset(
    substructure->J,
    0,
    substructure->dqCount *
    substructure->constraintEquationCount * sizeof(double));

  memset(
    substructure->Jb,
    0,
    substructure->dqCount *
    substructure->constraintEquationCount * sizeof(bool));

  /* j is local loop index, k is global loop index */
  int k = substructure->loopOffset;
  for(
    int j = 0;
    j < context->substructureLoopCount[substructureIndex];
    j++, k++) {

    for(
      int l = 0;
      l < context->positiveLoopMembersCompressedCount[k];
      l++) {

      int variableIndex = context->positiveLoopMemberOffsets[k][l];

      int columnIndex =
        variableIndex - substructure->dqOffset;

      // assert(columnIndex >= 0);
      // printf(
      //   "    processing variable: %d, column: [%d], dqOffset: %d, \n     h: ",
      //   variableIndex,
      //   columnIndex,
      //   substructure->dqOffset);
      // s2_screw_print(context->h + variableIndex);

      cblas_dcopy(
        6,
        (double*)(context->h + variableIndex),
        1,
        substructure->J + (j * 6 * substructure->dqCount + columnIndex),
        substructure->dqCount);
    }

    /* negative loop members [l]*/
    // printf("  setting negative loop members:\n");

    for(int l = 0; l < context->negativeLoopMembersCompressedCount[k]; l++) {

      int variableIndex = context->negativeLoopMemberOffsets[k][l];
      int columnIndex = variableIndex - substructure->dqOffset;

      // printf(
      //   "    processing variable: %d, column: [%d], dqOffset: %d, \n     h: ",
      //   variableIndex,
      //   columnIndex,
      //   substructure->dqOffset);
      // s2_screw_print(context->h + variableIndex);

      cblas_daxpy(
        6,
        -1.0,
        (double*)(context->h + variableIndex),
        1,
        substructure->J + (j * 6 * substructure->dqCount + columnIndex),
        substructure->dqCount);
    }
  }
  // printf("after setting J in PO1: \n");
  // s2_array_print2D(
  //   substructure->constraintEquationCount,
  //   substructure->dqCount,
  //   true,
  //   substructure->J);

  substructure->rank =
    s2_factor_LU_full(
      substructure->constraintEquationCount,
      substructure->dqCount,
      substructure->J,
      context->epsRankDetermination,
      substructure->pr_12,
      substructure->pc_12);

  s2_jacobian_setInversePermutation(
    substructure->constraintEquationCount,
    substructure->pr_12,
    substructure->pr_21);

  // printf("\n\n%d\n", substructure->dqCount);
  // s2_arrayi_print(substructure->dqCount, substructure->pc);

  s2_jacobian_setInversePermutation(
    substructure->dqCount,
    substructure->pc_12,
    substructure->pc_21);

  s2_factor_setBMatrix(
    substructure->constraintEquationCount,
    substructure->dqCount,
    substructure->rank,
    substructure->J);

  substructure->scaledInitialInfinityNormB =
    context->modelParameters.pivotRatioMultiple *
    s2_jacobian_infinityNormB(
      substructure->constraintEquationCount,
      substructure->dqCount,
      substructure->rank,
      substructure->J);

  /*! @TODO remove duplicate code for setting up PO1 in a
  substructure and remove pivotRatio */

  substructure->pivotRatio =
    context->modelParameters.pivotRatioMultiple *
    s2_jacobian_pivotRatio(
      substructure->dqCount,
      substructure->rank,
      substructure->J);

  s2_substructure_setJacobianBooleanPO2(substructure);
}


/*! @brief set initial jacobian in permutation order 1 and perform an initial
call to GCP using s2_factor_LU_full on the basis of one substructure with one
specified coordinate

@param context [in] s2_context_t data structure
@param substructureIndex [in] index of substructure
@param dqIndex [in] global dqIndex of specified joint

note the Jacobian is stored in row-major order
*/
void s2_substructure_setJacobianPO1Specified(
  s2_substructure_t* substructure,
  int dqIndex) {

  s2_context_t* context = substructure->context;
  int substructureIndex = substructure->index;

  memset(
    substructure->J,
    0,
    substructure->dqCount *
    substructure->constraintEquationCount *
    sizeof(double));

  int k = substructure->loopOffset;

  /* j is local loop index, k is global loop index */
  // printf("\n updating substructure [%d]\n", substructureIndex);
  for(
    int j = 0;
    j < context->substructureLoopCount[substructureIndex];
    j++, k++) {

    /* positive loop members [l]*/
    // printf("  loop [%d]\n", j);
    // printf("  setting positive loop members:\n");
    /* l is current loop member being processed */
    for(int l = 0; l < context->positiveLoopMembersCompressedCount[k]; l++) {
      int variableIndex = context->positiveLoopMemberOffsets[k][l];
      int columnIndex = variableIndex - substructure->dqOffset;
      assert(columnIndex > -1);

      // printf(
      //   "    processing variable: %d, column: [%d], dqOffset: %d, \n     h: ",
      //   variableIndex,
      //   columnIndex,
      //   substructure->dqOffset);
      // s2_screw_print(context->h + variableIndex);
      cblas_dcopy(
        6,
        (double*)(context->h + variableIndex),
        1,
        substructure->J + (j * 6 * substructure->dqCount + columnIndex),
        substructure->dqCount);
    }

    /* negative loop members [l]*/
    // printf("  setting negative loop members:\n");
    for(int l = 0; l < context->negativeLoopMembersCompressedCount[k]; l++) {
      int variableIndex = context->negativeLoopMemberOffsets[k][l];
      int columnIndex = variableIndex - substructure->dqOffset;
      assert(columnIndex > -1);
      // printf(
      //   "    processing variable: %d, column: [%d], dqOffset: %d, \n     h: ",
      //   variableIndex,
      //   columnIndex,
      //   substructure->dqOffset);
      // s2_screw_print(context->h + variableIndex);

      cblas_daxpy(
        6,
        -1.0,
        (double*)(context->h + variableIndex),
        1,
        substructure->J + (j * 6 * substructure->dqCount + columnIndex),
        substructure->dqCount);
    }
  }
  // printf("specified full factor: after setting J in PO1: \n");
  // s2_array_print2D(
  //   substructure->constraintEquationCount,
  //   substructure->dqCount,
  //   true,
  //   substructure->J);

  /* convert global index to local index */
  int localSpecifiedIndex = dqIndex - substructure->dqOffset;
  assert(localSpecifiedIndex > -1);

  substructure->rank =
    s2_factor_LU_full_specified(
      substructure->constraintEquationCount,
      substructure->dqCount,
      substructure->J,
      context->epsRankDetermination,
      substructure->pr_12,
      substructure->pc_12,
      localSpecifiedIndex);
  // printf("rank inside of fast factor specified: %d\n", substructure->rank);

  s2_jacobian_setInversePermutation(
    substructure->constraintEquationCount,
    substructure->pr_12,
    substructure->pr_21);

  s2_jacobian_setInversePermutation(
    substructure->dqCount,
    substructure->pc_12,
    substructure->pc_21);

  s2_factor_setBMatrix(
    substructure->constraintEquationCount,
    substructure->dqCount,
    substructure->rank,
    substructure->J);

  substructure->scaledInitialInfinityNormB =
    context->modelParameters.pivotRatioMultiple *
    s2_jacobian_infinityNormB(
      substructure->constraintEquationCount,
      substructure->dqCount,
      substructure->rank,
      substructure->J);

  /*! @TODO remove this once infinity norm is verified to work */
  substructure->pivotRatio =
    context->modelParameters.pivotRatioMultiple *
    s2_jacobian_pivotRatio(
      substructure->dqCount,
      substructure->rank,
      substructure->J);

  s2_substructure_setJacobianBooleanPO2(substructure);
}


/*!
  @brief translate local permutations to global permutations

*/
void s2_context_translatePermutations(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  memset(context->isDependentPO1, 0, context->dqCount * sizeof(bool));

  /* initialize permutation arrays */
  for(int i = 0; i < context->dqCount; i++) {
    context->pv_12[i] = i;
    context->pv_21[i] = i;
    context->pv_13[i] = i;
    context->pv_31[i] = i;
  }

  /* set pv_21 */
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    for(int j = 0; j < substructure->dqCount; j++) {

      context->pv_12[substructure->dqOffset + j] =
        substructure->pc_12[j] + substructure->dqOffset;

    }
  }

  /* set inverse permutation for pv_21 */
  s2_jacobian_setInversePermutation(
    context->dqCount,
    context->pv_12,
    context->pv_21);

  /* determine which variables are independent / dependent */
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    for(int j = 0; j < substructure->rank; j++) {
      context->isDependentPO1[
      context->pv_12[
      substructure->dqOffset + j]] = true;
    }

    // fprintf(
    //   debug,
    //   "substructure: [%d], dqOffset: %d\n", i, substructure->dqOffset);
    // for(int j = 0; j < substructure->dqCount; j++) {
    //   printf(
    //     "%s ",
    //     context->isDependentPO1[substructure->dqOffset + j] ? "true":"false");
    // }
    // printf("\n");
  }

  /* verify p_v_12 and p_v_21 */
  // fprintf(debug, "pv_12: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_12);
  //
  // fprintf(debug, "pv_21: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_21);

  /* set p_v_13 */
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    // fprintf(
    //   debug,
    //   "rank for substructure [%d]: %d\n", i, substructure->rank);
    int k = substructure->dqOffset;
    for(
      int j = substructure->dqOffset;
      j < substructure->dqOffset + substructure->dqCount;
      j++) {

      if(!context->isDependentPO1[j]) {
        context->pv_13[k++] = j;
      }
    }

    for(
      int j = substructure->dqOffset;
      j < substructure->dqOffset + substructure->dqCount;
      j++) {

      if(context->isDependentPO1[j]) {
        context->pv_13[k++] = j;
      }
    }
  }

  /* set p_v_31 */
  s2_jacobian_setInversePermutation(
    context->dqCount,
    context->pv_13,
    context->pv_31);

  /* set isDependentPO3 */
  for(int i = 0; i < context->dqCount; i++) {
    context->isDependentPO3[
    context->pv_31[i]] =
        context->isDependentPO1[i];
  }

  /* set pv_23 */
  for(int i = 0; i < context->dqCount; i++) {
    context->pv_23[i] = context->pv_21[context->pv_13[i]];
  }

  /* set pv_32 */
  s2_jacobian_setInversePermutation(
    context->dqCount,
    context->pv_23,
    context->pv_32);

  /* verify pv_13 and pv_31 */
  // printf("pv_13: ");
  // s2_arrayi_print(context->dqCount, context->pv_13);
  //
  // printf("pv_31: ");
  // s2_arrayi_print(context->dqCount, context->pv_31);
  //
  // fprintf(debug, "isDependentPO1: ");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, "%s ", context->isDependentPO1[i] ? "true":"false");
  // }
  // fprintf(debug, "\n");

  // fprintf(debug, "isDependentPO3: ");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, "%s ", context->isDependentPO3[i] ? "true":"false");
  // }
  // fprintf(debug, "\n");

}

/*!
@brief evaluate and set a compressed sparsity pattern for preconditioned
constraints in linear array
in PO3
*/
void s2_context_evaluateJacobianSparsity(s2_context_t* context) {

  int bOffset = 0;
  context->nnzb = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    /* record offset into linear b array */
    substructure->bOffset = bOffset;

    /* record number of nonzero entries in B matrix */
    substructure->nnzb = 0;
    for(int j = 0; j < substructure->rank; j++) {
      for(int k = substructure->rank; k < substructure->dqCount; k++) {
        if(substructure->Jb[j * substructure->dqCount + k]) {
          (substructure->nnzb)++;
        }
      }
    }

    bOffset +=
      (substructure->dqCount - substructure->rank) * substructure->rank;

    /* record total entries in B matrix */
    substructure->nb = bOffset - substructure->bOffset;
    context->nnzb += substructure->nnzb;
  }

  if(context->ijMapping) {
    s2_arrayi_free(context->ijMapping);
  }
  // printf("jacobianSize: %d\n", context->jacobianSize);
  //
  // printf("bOffset: %d\n", bOffset);
  // printf("nnzb: %d\n", context->nnzb);
  // debugging statements:

  // int nnzbSum = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;

  // printf("%d ", substructure->nnzb);
  // nnzbSum += substructure->nnzb;
  // }
  // printf("nnzb: %d\n", context->nnzb);
  // printf("nnzbSum = nnzb: %s\n", nnzbSum == context->nnzb ? "true":"false");

  context->ijMapping =
    calloc(context->jacobianSize, sizeof(int));

  int jacobianOffset = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;

    substructure->ijMapping =
      context->ijMapping + jacobianOffset;
    jacobianOffset +=
      substructure->dqCount *
      substructure->constraintEquationCount;
  }

  if(context->b3) {
    free(context->b3);
  }
  context->b3 = calloc(bOffset, sizeof(double));

  if(context->pb_23) {
    free(context->pb_23);
  }
  context->pb_23 = calloc(bOffset, sizeof(int));

  if(context->pb_32) {
    free(context->pb_32);
  }
  context->pb_32 = calloc(bOffset, sizeof(int));

  s2_arrayi_free2D(context->rb2);
  context->rb2 = s2_arrayi_malloc2D(context->dqCount, 2);

  s2_arrayi_free2D(context->rb3_temp);
  context->rb3_temp = s2_arrayi_malloc2D(context->dqCount, 2);

  s2_arrayi_free2D(context->rb3);
  context->rb3 = s2_arrayi_malloc2D(context->dqCount, 2);

  if(context->cb2) {
    free(context->cb2);
  }
  context->cb2 = calloc(bOffset, sizeof(int));

  if(context->cb3_temp) {
    free(context->cb3_temp);
  }
  context->cb3_temp = calloc(bOffset, sizeof(int));

  if(context->cb3) {
    free(context->cb3);
  }
  context->cb3 = calloc(bOffset, sizeof(int));

  // int sizecb3 = bOffset;

  for(int sub = 0; sub < context->substructureCount; sub++) {
    s2_substructure_t* substructure = context->substructures + sub;

    int jacobianSize =
      substructure->chordConstraintCount * substructure->dqCount;

    int ijMapping_temp_po3[substructure->dqCount * substructure->dqCount];

    for(int i = 0; i < substructure->dqCount * substructure->dqCount; i++) {
      ijMapping_temp_po3[i] = -1;
    }
    for(int i = 0; i < jacobianSize; i++) {
      substructure->ijMapping[i] = -1;
    }

    /* generate a local pv_local_23 array */
    int pv_local_23[substructure->dqCount];
    for(int i = 0; i < substructure->dqCount; i++) {
      pv_local_23[i] = context->pv_23[i + substructure->dqOffset] - substructure->dqOffset;
    }

    /* set ij mapping temp po3 indices */
    int k = substructure->bOffset;
    for(int i3 = substructure->dqCount - substructure->rank; i3 < substructure->dqCount; i3++) {
      int i2 = pv_local_23[i3];
      // assert(i2 > -1 && i2 < substructure->dqCount);
      int rb3Index = i3 + substructure->dqOffset;
      context->rb3[rb3Index][0] = k;
      for(int j3 = 0; j3 < substructure->dqCount - substructure->rank; j3++) {
        int j2 = pv_local_23[j3];

        // assert(j2 > -1 && j2 < substructure->constraintEquationCount);

        if(substructure->Jb[i2 * substructure->dqCount + j2]) {

          ijMapping_temp_po3[i3 * substructure->dqCount + j3] = k;
          context->cb3[k] = j3 + substructure->dqOffset;
          k++;
        }
      }
      context->rb3[rb3Index][1] = k;
    }

    // FILE* debug = stdout;
    // fprintf(debug, "substructure [%d] ijMapping_temp_po3:\n", sub);
    // for(int i = 0; i < substructure->dqCount; i++) {
    //   fprintf(debug, "  ");
    //   for(int j = 0; j < substructure->dqCount; j++) {
    //     fprintf(debug, "%d ", ijMapping_temp_po3[i * substructure->dqCount + j]);
    //   }
    //   fprintf(debug, "\n");
    // }
    assert(substructure->ijMapping != NULL);
    /* permute ij mapping temp to po2 */
    for(int i3 = substructure->dqCount - substructure->rank; i3 < substructure->dqCount; i3++) {

      int i2 = pv_local_23[i3];
      assert(i2 > -1 && i2 < substructure->dqCount);

      // fprintf(debug, "i2: %d\n", i2);
      for(int j3 = 0; j3 < substructure->dqCount - substructure->rank; j3++) {
        int j2 = pv_local_23[j3];// + substructure->dqCount - substructure->rank;
        // fprintf(stdout, "i2: %d\n", i2);
        // fprintf(stdout, "j2: %d\n", j2);

        assert(j2 > -1 && j2 < substructure->dqCount);

        substructure->ijMapping[i2 * substructure->dqCount + j2] =
          ijMapping_temp_po3[i3 * substructure->dqCount + j3];

      }
    }

    // fprintf(debug, "substructure [%d] ijMapping:\n", sub);
    // for(int i = 0; i < substructure->constraintEquationCount; i++) {
    //   fprintf(debug, "  ");
    //   for(int j = 0; j < substructure->dqCount; j++) {
    //     fprintf(debug, "%d ", substructure->ijMapping[i * substructure->dqCount + j]);
    //   }
    //   fprintf(debug, "\n");
    // }

  }



  //
  //
  //
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   s2_substructure_t* substructure = context->substructures + sub;
  //
  //   FILE* debug = context->modelParameters->debugFilePointer;
  //
  //   fprintf(debug, "\nB Matrix in PO2: \n");
  //   for(int i = 0; i < substructure->rank; i++) {
  //     fprintf(debug, "[");
  //     for(int j = substructure->rank; j < substructure->dqCount; j++) {
  //       fprintf(debug, "%0.9f ", substructure->J[i * substructure->dqCount + j]);
  //     }
  //     fprintf(debug, "]\n");
  //   }
  //
  //   /* print local jacobian in PO2 */
  //
  //
  //   int k = substructure->bOffset;
  //
  //   /* i loops over "global" rows */
  //   for(
  //     int i = substructure->dqOffset;
  //     i < (substructure->dqOffset + substructure->rank);
  //     i++) {
  //
  //     context->rb2[i][0] = k;
  //
  //     /* loop over local rows in each substructure */
  //     for(
  //       int j = substructure->rank;
  //       j < substructure->dqCount;
  //       j++) {
  //
  //       /*
  //         when indexing into Jb, i must be converted
  //         to a "local" index.  j is already a local index
  //       */
  //       if(substructure->Jb[
  //         (i - substructure->dqOffset) * substructure->dqCount + j]) {
  //
  //         /* cb2 contains columns of B matrix in PO2 shifted to
  //         start at zero indexing. */
  //         context->cb2[k] = j - substructure->rank;
  //
  //         /* add dqOffset to determine correct permutation */
  //         context->cb3_temp[k] =
  //           context->pv_32[j + substructure->dqOffset];
  //         k++;
  //       }
  //     }
  //     context->rb2[i][1] = k;
  //
  //     /* rb3_temp rows are permuted to follow PO3, but columns are still
  //     in PO2 */
  //     context->rb3_temp[context->pv_32[i]][0] = context->rb2[i][0];
  //     context->rb3_temp[context->pv_32[i]][1] = context->rb2[i][1];
  //   }
  //
  //   fprintf(debug, "\ncb2:\n");
  //   for(
  //     int i = substructure->dqOffset;
  //     i < (substructure->dqOffset + substructure->rank);
  //     i++) {
  //
  //     fprintf(debug, "[%d] ", i);
  //     for(int j = context->rb2[i][0]; j < context->rb2[i][1]; j++) {
  //       fprintf(debug, "cb2(%d): %d, ", j, context->cb2[j]);
  //     }
  //     fprintf(debug, "\n");
  //   }
  //
  //   fprintf(debug, "\ncb3_temp:\n");
  //   for(
  //     int i = substructure->dqOffset;
  //     i < (substructure->dqOffset + substructure->rank);
  //     i++) {
  //
  //     fprintf(debug, "[%d] ", i);
  //     for(int j = context->rb2[i][0]; j < context->rb2[i][1]; j++) {
  //       fprintf(debug, "cb3_temp(%d): %d, ", j, context->cb3_temp[j]);
  //     }
  //     fprintf(debug, "\n");
  //   }
  //
  //   fprintf(debug, "\ncb3_temp with rows in PO3:\n");
  //   for(
  //     int i = substructure->dqOffset + substructure->dqCount - substructure->rank;
  //     i < substructure->dqOffset + substructure->dqCount;
  //     i++) {
  //
  //     fprintf(debug, "[%d] ", i);
  //     for(int j = context->rb3_temp[i][0]; j < context->rb3_temp[i][1]; j++) {
  //       fprintf(debug, "cb3_temp(%d): %d, ", j, context->cb3_temp[j]);
  //     }
  //     fprintf(debug, "\n");
  //   }



  // fprintf(debug, "\ncb3_temp with rows in PO3:\n");
  // for(
  //   int i = substructure->dqOffset + substructure->dqCount - substructure->rank;
  //   i < substructure->dqOffset + substructure->dqCount;
  //   i++) {
  //
  //   fprintf(debug, "[%d] ", i);
  //   for(int j = context->rb3_temp[i][0]; j < context->rb3_temp[i][1]; j++) {
  //     fprintf(debug, "b3(%d): %d, ", j, context->cb3_temp[j]);
  //   }
  //   fprintf(debug, "\n");
  // }



  //
  //
  //
  //
  //
  //   k = substructure->bOffset;
  //   bOffset = substructure->bOffset;
  //   int independentCount = substructure->dqCount - substructure->rank;
  //   int indices[substructure->nb];
  //   for(int ind = 0; ind < substructure->nb; ind++) {
  //     indices[ind] = ind;
  //   }
  //
  //   for(
  //     int i = substructure->dqOffset + independentCount;
  //     i < (substructure->dqOffset + substructure->dqCount);
  //     i++) {
  //
  //     context->rb3[i][0] = bOffset;
  //     bOffset += (context->rb3_temp[i][1] - context->rb3_temp[i][0]);
  //     context->rb3[i][1] = bOffset;
  //
  //     for(
  //       int j = context->rb3_temp[i][0];
  //       j < context->rb3_temp[i][1];
  //       j++) {
  //
  //       int minIndex = j;
  //       int minVal = context->cb3_temp[j];
  //
  //       for(int l = j + 1; l < context->rb3_temp[i][1]; l++) {
  //         if(context->cb3_temp[l] < minVal) {
  //           minIndex = l;
  //           minVal = context->cb3_temp[l];
  //         }
  //       }
  //
  //       if(minIndex != j) {
  //         s2_iswap(
  //           context->cb3_temp + j,
  //           context->cb3_temp + minIndex);
  //         s2_iswap(
  //           indices + (j - substructure->bOffset),
  //           indices + (minIndex - substructure->bOffset));
  //       }
  //
  //       context->pb_23[k] =
  //         indices[j - substructure->bOffset] + substructure->bOffset;
  //       context->cb3[k] = minVal;
  //       k++;
  //     }
  //   }
  //
  //   for(
  //     int i = substructure->bOffset;
  //     i < substructure->bOffset + substructure->nnzb;
  //     i++) {
  //
  //     context->pb_32[context->pb_23[i]] = i;
  //   }
  //
  //   k = substructure->bOffset;
  //   for(
  //     int i = 0;
  //     i < substructure->rank;
  //     i++) {
  //
  //     for(
  //       int j = substructure->rank;
  //       j < substructure->dqCount;
  //       j++) {
  //
  //       /* set indices of Jxx */
  //       if(substructure->Jb[i * substructure->dqCount + j]) {
  //         substructure->Jxx[i * substructure->dqCount + j] =
  //           context->pb_32[k++];
  //       }
  //     }
  //   }
  // }
}


/*!
@brief evaluate permuted Jacobian boolean matrix in PO2
for a single substructure
*/
void s2_substructure_setJacobianBooleanPO2(s2_substructure_t* substructure) {

  s2_context_t* context = substructure->context;
  int substructureIndex = substructure->index;

  memset(
    substructure->Jb,
    0,
    substructure->dqCount *
    substructure->constraintEquationCount *
    sizeof(bool));

  int k = substructure->loopOffset;

  /* j iterates over local list of loops in substructure,
     k iterates over global list of loops */
  for(
    int j = 0;
    j < context->substructureLoopCount[substructureIndex];
    j++,
    k++) {

    /* positive loop members [l]*/
    for(int l = 0; l < context->positiveLoopMembersCompressedCount[k]; l++) {
      int variableIndex = context->positiveLoopMemberOffsets[k][l];

      // printf("   variableIndex: %d\n", variableIndex);
      // printf("   substructure->dqOffset: %d\n", substructure->dqOffset);
      // printf("   substructure->dqCount: %d\n", substructure->dqCount);
      // printf("   variableIndex: %d\n", variableIndex);
      // printf("   substructure->dqOffset: %d\n", substructure->dqOffset);
      // printf("   pc_21: ");
      // s2_arrayi_print(substructure->dqCount, substructure->pc_21);

      assert(variableIndex - substructure->dqOffset >= 0);

      int columnIndex =
        substructure->pc_21[variableIndex - substructure->dqOffset];

      // printf("   columnIndex: %d\n", columnIndex);

      for(int m = 0; m < 6; m++) {
        int rowIndex = substructure->pr_21[j * 6 + m];
        substructure->Jb[
        rowIndex * substructure->dqCount + columnIndex] = true;
      }
    }

    /* negative loop members [l]*/
    for(int l = 0; l < context->negativeLoopMembersCompressedCount[k]; l++) {
      int variableIndex = context->negativeLoopMemberOffsets[k][l];
      int columnIndex =
        substructure->pc_21[variableIndex - substructure->dqOffset];
      for(int m = 0; m < 6; m++) {
        int rowIndex = substructure->pr_21[j * 6 + m];
        substructure->Jb[
        rowIndex * substructure->dqCount + columnIndex] = true;
      }
    }
  }

  // printf("jacobian boolean in setJacobianBoolean (PO2) before factorization:\n");
  // for(int j = 0; j < substructure->constraintEquationCount; j++) {
  //   for(int k = 0; k < substructure->dqCount; k++) {
  //     if(substructure->Jb[j * substructure->dqCount + k]) {
  //       printf("1 ");
  //     } else {
  //       printf("0 ");
  //     }
  //   }
  //   printf("\n");
  // }
  // printf("\n");


  /* Major Boolean LU factors */
  for(int j = 0; j < substructure->rank; j++) {
    for(int k = j + 1; k < substructure->rank; k++) {
      if(substructure->Jb[k * substructure->dqCount + j]) {
        for(int l = j + 1; l < substructure->dqCount; l++) {
          if(context->Jb[j * substructure->dqCount + l]) {
            substructure->Jb[k * substructure->dqCount + l] = true;
          }
        }
      }
    }
  }

  /* Record fills in Boolean B matrix */
  for(int j = substructure->rank - 1; j > -1; j--) {
    for(int k = substructure->rank - 1; k > j; k--) {
      if(substructure->Jb[j * substructure->dqCount + k]) {
        for(int l = substructure->rank; l < substructure->dqCount; l++) {
          if(substructure->Jb[k * substructure->dqCount + l]) {
            substructure->Jb[j * substructure->dqCount + l] = true;
          }
        }
      }
    }
  }

  // printf("jacobian boolean in setJacobianBoolean after factorization:\n");
  // for(int j = 0; j < substructure->constraintEquationCount; j++) {
  //   for(int k = 0; k < substructure->dqCount; k++) {
  //     if(substructure->Jb[j * substructure->dqCount + k]) {
  //       printf("1 ");
  //     } else {
  //       printf("0 ");
  //     }
  //   }
  //   printf("\n");
  // }
  // printf("\n");
  //
  //
  // printf("verifying Jb after factorization (PO2)\n");
  // for(int i = 0; i < substructure->constraintEquationCount; i++) {
  //   for(int j = 0; j < substructure->dqCount; j++) {
  //     printf("%d ", substructure->Jb[i * substructure->dqCount + j] ? 1:0);
  //   }
  //   printf("\n");
  // }
}


/*!
@brief set jacobian boolean matrix for all substructures in PO2
*/
void s2_context_setJacobianBooleanPO2(s2_context_t* self) {
  s2_substructure_t* substructure = self->substructures;
  for(int i = 0; i < self->substructureCount; i++, substructure++) {
    s2_substructure_setJacobianBooleanPO2(substructure);
  }
}


/*!
  @brief set ij mapping array for substructure
*/
void s2_substructure_setJacobianIJMapping(s2_substructure_t* substructure) {

  // memset(
  //   substructure->ijMapping,
  //   0,
  //   substructure->dqCount *
  //     substructure->constraintEquationCount *
  //     sizeof(int));
  //
  // int k = substructure->bOffset + substructure->nnzb - 1;
  // int rank = substructure->rank;
  // int colDim = substructure->dqCount;
  // for(int i = rank - 1; i > -1; i--) {
  //   for(int j = rank; j < colDim; j++) {
  //     if(substructure->Jb[i * colDim + j]) {
  //       // printf("k: %d\n",  k);
  //       substructure->ijMapping[i * colDim + j] =
  //         context->pb_32[k--];
  //     }
  //   }
  // }
}


/*!
  @brief records row and column position in ijMapping array
*/
void s2_context_setJacobianIJMapping(s2_context_t* context) {
  s2_substructure_t* substructure = context->substructures;
  for(int i = 0; i < context->substructureCount; i++, substructure++) {
    s2_substructure_setJacobianIJMapping(substructure);
  }
}


/*!
@brief evaluate permuted Jacobian matrix for all substructures in PO2
*/
void s2_context_setJacobianPO2(s2_context_t* context) {
  s2_substructure_t* substructure = context->substructures;
  for(int i = 0; i < context->substructureCount; i++, substructure++) {
    s2_substructure_setJacobianPO2(substructure);
  }
}


/*!
@brief evaluate permuted Jacobian matrix for one substructure in PO2

note: does not zero out the jacobian matrix for the substructure. zeroing the
Jacobian is performed in s2_context_setJacobianPO2. If it is desired to set the
substructure jacobian without setting other jacobians, the array should
explicitly be zeroed before this function is called.
*/
void s2_substructure_setJacobianPO2(s2_substructure_t* substructure) {

  s2_context_t* context = substructure->context;
  int substructureIndex = substructure->index;
  memset(
    substructure->J,
    0,
    substructure->dqCount *
    substructure->constraintEquationCount * sizeof(double));

  /* j is local loop index, k is global loop index */
  int k = substructure->loopOffset;
  for(
    int j = 0;
    j < context->substructureLoopCount[substructureIndex];
    j++, k++) {

    /* positive loop members [l]*/
    for(
      int l = 0;
      l < context->positiveLoopMembersCompressedCount[k];
      l++) {

      int variableIndex = context->positiveLoopMemberOffsets[k][l];

      int columnIndex_2 =
        substructure->pc_21[variableIndex - substructure->dqOffset];

      for(int m = 0; m < 6; m++) {

        int rowIndex_2 = substructure->pr_21[j * 6 + m];

        substructure->J[
        rowIndex_2 * substructure->dqCount + columnIndex_2] =
            ((double*)(context->h + variableIndex))[m];
      }
    }

    /* negative loop members [l]*/
    for(
      int l = 0;
      l < context->negativeLoopMembersCompressedCount[k];
      l++) {

      int variableIndex = context->negativeLoopMemberOffsets[k][l];

      int columnIndex_2 =
        substructure->pc_21[variableIndex - substructure->dqOffset];

      for(int m = 0; m < 6; m++) {

        int rowIndex_2 = substructure->pr_21[j * 6 + m];

        substructure->J[
        rowIndex_2 * substructure->dqCount + columnIndex_2] =
            -((double*)(context->h + variableIndex))[m];
      }
    }
  }

}


/*!
@brief wrapper function for s2_factor_LU to evaluate the jacobian
for a substructure that has been evaluated in PO2 order
*/
void s2_substructure_factorJacobianPO2(
  s2_substructure_t* substructure) {

  s2_factor_LU(
    substructure->constraintEquationCount,
    substructure->dqCount,
    substructure->rank,
    substructure->J);
}


/*!
@brief factors LU Decomposition with no row or column pivoting,
stores preconditioned constraints in PO3

fullFactor LU factors a double precision matrix, A[rowDim][colDim],
using no row or column pivoting. Only the square rank x rank submatrix of
matrix A will be processed.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity
diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper
triangular (rank+1) by (rank+1) Ur matrix is stored on and above the
diagonal in A[0:rank][0:rank].

Note that this minimal algorithm variation does not compute the product
C=LR*inverse(Lr) or B=-inverse(Ur)*UR.

@param rowDim input row dimension of matrix A.
@param colDim input column dimension of matrix A.
@param only the square rank x rank submatrix of A will be factored
@param *A input/output matrix to be factored with dimensions
A[0:rowDim-1][0:colDim-1]:
*/
int s2_context_factorJacobianPO2(
  s2_context_t* context) {

  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_factorJacobianPO2(context->substructures + i);
  }
  return 0;
}


/*!
@brief record location of velocity and position level constraints

*/
void s2_context_setConstraintPointers(
  s2_context_t* context) {

  FILE* debug = context->modelParameters.debugFilePointer;

  if(context->constraints) {
    free(context->constraints);
  }
  if(context->chordConstraintCount > 0) {
    context->constraints =
      s2_arrayi_malloc2D(context->chordConstraintCount, 4);
  }

  // int m = 0;
  // for(int k = 0; k < context->activeJointCount; k++) {
  /* identify a virtual body */
  // if(context->bodyPointers[k]->isVirtual) {

  // fprintf(debug, "setting chord constraints:\n");

  /* find chord constraint that contains the virtual body */

  int k = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    // fprintf(debug, " [%d] substructure:\n", i);
    substructure->constraints = context->constraints + k;

    for(int j = 0; j < substructure->chordConstraintCount; j++, k++) {

      // fprintf(debug, "   [%d] chord constraint\n", k);
      // fprintf(debug, "        parent Body: ");
      // s2_stackstr_print(
      //   debug,
      //   context->chordConstraints[k].parentBodyPointer->name);
      // fprintf(
      //   debug,
      //   "        child Body: ");
      // s2_stackstr_print(
      //   debug,
      //   context->chordConstraints[k].childBodyPointer->name);
      // fprintf(
      //   debug,
      //   "        original parent Body: ");
      // s2_stackstr_print(
      //   debug,
      //   context->chordConstraints[k].originalParentBodyPointer->name);
      // if(
      //     context->chordConstraints[j].parentBodyPointer ==
      //     context->bodyPointers[k]) {

      /* value at first location is the index of the original embedded
         frame, on the child (-) side */
      context->constraints[k][0] =
        context->chordConstraints[k].childBodyPointer->referenceFrame->vaIndex;

      /* the value at the second location is the index of the
         virtual (detached) frame, on the parent (+) side */
      context->constraints[k][1] =
        context->chordConstraints[k].parentBodyPointer->referenceFrame->vaIndex;

      /* value at first location is the index of the original embedded
         frame, on the child (-) side*/
      context->constraints[k][2] =
        context->chordConstraints[k].childFramePointer->dIndex;

      /* the value at the second location is the index of the
         virtual (detached) frame, on the parent (+) side */
      context->constraints[k][3] =
        context->chordConstraints[k].parentFramePointer->dIndex;
    }
  }
  // m++;
  // break;
  //   }
  // }
  // }

  // int m = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   substructure->constraints = context->constraints + m;
  //   m += substructure->chordConstraintCount;
  // }
}


/*!
  evaluate position level constraints for one substructure
*/
void s2_substructure_evaluateConstraints(s2_substructure_t* substructure) {
  s2_context_t* context = substructure->context;
  for(int i = 0; i < substructure->chordConstraintCount; i++) {
    s2_displacement_t* parentFrame = context->d +
      substructure->constraints[i][3];
    s2_displacement_t* childFrame = context->d +substructure->constraints[i][2];
    s2_displacement_t temp;
    s2_product_ddinverse(parentFrame, childFrame, &temp);

    // s2_frame_t* parentFramePointer = findFrame(context, parentFrame);
    // printf("\nparent frame [%d]: ", substructure->constraints[i][3]);
    // s2_stackstr_print(stdout, parentFramePointer->name);
    // s2_displacement_print(stdout, parentFrame);
    //
    // s2_frame_t* childFramePointer = findFrame(context, childFrame);
    // printf("child Frame [%d]: ", substructure->constraints[i][2]);
    // s2_stackstr_print(stdout, childFramePointer->name);
    // s2_displacement_print(stdout, childFrame);
    //
    // printf("parent * child^(-1): \n");
    // s2_displacement_print(stdout, &temp);

    memcpy(substructure->cs + i * 6, &temp.T, sizeof(s2_vector_t));
    double trace = temp.R.m00 + temp.R.m11 + temp.R.m22 - 3;
    substructure->cs[i * 6 + 3] = 0.5 * (temp.R.m21 - temp.R.m12) + trace;
    substructure->cs[i * 6 + 4] = 0.5 * (temp.R.m02 - temp.R.m20) + trace;
    substructure->cs[i * 6 + 5] = 0.5 * (temp.R.m10 - temp.R.m01) + trace;
  }
}


/*!
  evaluates position level constraints
*/
void s2_context_evaluateConstraints(s2_context_t* context) {
  for(int i = 0; i < context->chordConstraintCount; i++) {
    s2_displacement_t* parentFrame = context->d + context->constraints[i][3];
    s2_displacement_t* childFrame = context->d + context->constraints[i][2];
    s2_displacement_t temp;
    s2_product_ddinverse(parentFrame, childFrame, &temp);
    memcpy(context->cs + i * 6, &temp.T, sizeof(s2_vector_t));
    double trace = temp.R.m00 + temp.R.m11 + temp.R.m22 - 3;
    context->cs[i * 6 + 3] = 0.5 * (temp.R.m21 - temp.R.m12) + trace;
    context->cs[i * 6 + 4] = 0.5 * (temp.R.m02 - temp.R.m20) + trace;
    context->cs[i * 6 + 5] = 0.5 * (temp.R.m10 - temp.R.m01) + trace;
  }
}


/*!
  sparse version of set B matrix evaluates B matrix
  into compressed b3 array in PO3, to align with the
  permutation order of the coefficient matrix. Evaluates infinityNormB.

  to be called after evaluate sparsity
*/
void s2_substructure_setBMatrixSparse(s2_substructure_t* substructure) {

  // printf("Jb:\n");
  // for(int i = 0; i < substructure->constraintEquationCount; i++) {
  //   for(int j = 0; j < substructure->dqCount; j++) {
  //     printf("%d ", substructure->Jb[i * substructure->dqCount + j] ? 1:0);
  //   }
  //   printf("\n");
  // }
  s2_context_t* context = substructure->context;
  double* J = substructure->J;
  // double J[substructure->constraintEquationCount * substructure->dqCount];
  // memcpy(
  //   J,
  //   substructure->J,
  //   substructure->constraintEquationCount * substructure->dqCount *
  //   sizeof(double));

  // printf("Factored jacobian before setBMatrixSparse\n");
  // s2_array_print2D(
  //   substructure->constraintEquationCount,
  //   substructure->dqCount,
  //   true,
  //   J);

  if(substructure->rank > 0 && substructure->rank < substructure->dqCount) {
    substructure->infinityNormB = 0;
    double pivots[substructure->rank];
    for(int i = 0; i < substructure->rank; i++) {
      pivots[i] = J[i * substructure->dqCount + i];
    }
    // printf("substructure->dqCount: %d\n", substructure->dqCount);
    for(int i = substructure->rank - 1; i > -1; i--) { /* rows of B */
      // printf("\n[%d] pivot value: %f, original pivot value: %f\n",
      //   i, J[i * substructure->dqCount + i], pivots[i]);
      for(
        int k = substructure->rank;
        k < substructure->dqCount;
        k++)  {

        /* columns of B */
        // printf("index [%d][%d]\n", i, k);
        // printf("i: %d\n", i);
        assert(J[i * substructure->dqCount + i] == pivots[i]);
        if(substructure->Jb[i * substructure->dqCount + k]) {
          double save = 0;
          assert(J[i * substructure->dqCount + i] == pivots[i]);
          for(int j = substructure->rank - 1; j > i; j--) {
            save +=
              J[i * substructure->dqCount + j] *
              J[j * substructure->dqCount + k];
          }
          assert(J[i * substructure->dqCount + i] == pivots[i]);
          int b3Index =
            substructure->ijMapping[
          i * substructure->dqCount + k];

          // printf("i * dqCount + k: %d\n", i * substructure->dqCount + k);
          // printf("i * dqCount + i: %d\n", i * substructure->dqCount + i);
          context->b3[b3Index] =
            (J[i * substructure->dqCount + k] - save) /
            J[i * substructure->dqCount + i];

          // printf(
          //   "b3[%d] = %0.9f\n",
          //   b3Index, context->b3[b3Index]);
          // assert(k != i);
          // for(int p = 0; p < substructure->rank; p++) {
          //   assert(
          //     J + (i * substructure->dqCount + k) !=
          //     J + (p * substructure->dqCount + p));
          // }
          J[i * substructure->dqCount + k] = context->b3[b3Index];
          assert(J[i * substructure->dqCount + i] == pivots[i]);
          // printf(
          //   " b3[%d/%d], J[%d, %d]: %f\n", b3Index, substructure->nnzb, i, k, context->b3[b3Index]);
          assert(!isinf(context->b3[b3Index]));
          assert(!isnan(context->b3[b3Index]));
          substructure->infinityNormB =
            fmax(substructure->infinityNormB, fabs(context->b3[b3Index]));
        }
      }

      // printf("b matrix in (po2):");
      // for(int i = 0; i < substructure->rank; i++) {
      //   for(
      //     int j = substructure->rank;
      //     j < substructure->dqCount;
      //     j++) {
      //       int b3index =
      //         substructure->ijMapping[
      //           i * substructure->dqCount + j];
      //       if(b3index != -1) {
      //         printf("%f ", context->b3[b3index]);
      //       } else {
      //         printf("0.0 ");
      //       }
      //   }
      // }
      // printf("\n");


      // printf(
      //   "infinity norm computed in setBMatrixSparse: %f\n",
      //   substructure->infinityNormB);
    }
  }
}


/*!
  sparse version of set B matrix evaluates B matrix into compressed b3 array
  in PO3 for all substructures

  to be called after evaluate sparsity
*/
void s2_context_setBMatrixSparse(s2_context_t* self) {
  s2_substructure_t* substructure = self->substructures;
  for(int i = 0; i < self->substructureCount; i++, substructure++) {
    s2_substructure_setBMatrixSparse(substructure);
  }
}


/*!
  @brief wrapper function for s2_factor_setBMatrix for a substructure
*/
void s2_substructure_setBMatrix(s2_substructure_t* self) {
  s2_factor_setBMatrix(
    self->constraintEquationCount,
    self->dqCount,
    self->rank,
    self->J);
}


/*!
  @brief sets B matrix for all substructures
*/
void s2_context_setBMatrix(s2_context_t* self) {
  for(int i = 0; i < self->substructureCount; i++) {
    s2_substructure_setBMatrix(self->substructures + i);
  }
}


/*!
@brief used to print properties of substructure datastructures related
to Jacobians
*/
void s2_context_printJacobian(
  FILE* f,
  s2_context_t* context) {

  if(context->mobilityNumbers == NULL) {
    s2_context_initializeMobility(context);
  }

  /* backup screws before doing anything */
  int k = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    fprintf(
      f,
      "\n substructure [%d] with dimension [%d, %d]\n",
      i,
      substructure->constraintEquationCount,
      substructure->dqCount);
    fprintf(f, "  rank: %d\n", substructure->rank);
    fprintf(f, "pr_12: ");
    s2_arrayi_print(f, substructure->constraintEquationCount, substructure->pr_12);
    fprintf(f, "pc_12: ");
    s2_arrayi_print(f, substructure->dqCount, substructure->pc_12);

    fprintf(f, "pr_21: ");
    s2_arrayi_print(f, substructure->constraintEquationCount, substructure->pr_21);
    fprintf(f, "pc_21: ");
    s2_arrayi_print(f, substructure->dqCount, substructure->pc_21);
    fprintf(f, "  dqOffset: %d\n", substructure->dqOffset);
    fprintf(f, "  qOffset: %d\n", substructure->qOffset);
    fprintf(f, "  Jacobian members:\n");
    for(int j = 1; j < context->substructureMembersCount[i]; j++) {
      s2_joint_t* joint =
        context->jointPointers[context->substructureMembers[i][j]];
      if(joint->jointType != S2_JOINT_RIGID) {
        fprintf(f, "   ");
        s2_stackstr_print(f, joint->name);
      }
    }

    for(int j = 0; j < context->substructureLoopCount[i]; j++, k++) {
      fprintf(f, "   loop [%d], positive loop members:\n", k);
      for(int l = 0; l < context->positiveLoopMembersCount[k]; l++) {
        s2_joint_t* joint =
          context->jointPointers[context->positiveLoopMembers[k][l]];
        if(joint->jointType != S2_JOINT_RIGID) {
          fprintf(f, "   ");
          s2_stackstr_print(f, joint->name);
        }
      }
      fprintf(f, "   loop [%d], negative loop members:\n", k);
      for(int l = 0; l < context->negativeLoopMembersCount[k]; l++) {
        s2_joint_t* joint =
          context->jointPointers[context->negativeLoopMembers[k][l]];
        if(joint->jointType != S2_JOINT_RIGID) {
          fprintf(f, "   ");
          s2_stackstr_print(f, joint->name);
        }
      }
    }

    memset(
      substructure->J,
      0,
      substructure->dqCount *
      substructure->constraintEquationCount * sizeof(double));

    memset(
      substructure->Jb,
      0,
      substructure->dqCount *
      substructure->constraintEquationCount * sizeof(bool));

    int k = substructure->loopOffset;
    for(
      int j = 0;
      j < context->substructureLoopCount[i];
      j++, k++) {

      /* positive loop members [l]*/
      // printf("  loop [%d]\n", j);
      // printf("  setting positive loop members:\n");
      for(
        int l = 0;
        l < context->positiveLoopMembersCompressedCount[k];
        l++) {

        int variableIndex = context->positiveLoopMemberOffsets[k][l];
        int columnIndex = variableIndex - substructure->dqOffset;

        assert(columnIndex >= 0);
        printf(
          "    processing variable: %d, column: [%d], dqOffset: %d, \n     h: ",
          variableIndex,
          columnIndex,
          substructure->dqOffset);
        s2_screw_print(stdout, context->h + variableIndex);

        cblas_dcopy(
          6,
          (double*)(context->h + variableIndex),
          1,
          substructure->J + (j * 6 * substructure->dqCount + columnIndex),
          substructure->dqCount);
      }

      /* negative loop members [l]*/
      // printf("  setting negative loop members:\n");
      for(int l = 0; l < context->negativeLoopMembersCompressedCount[k]; l++) {

        int variableIndex = context->negativeLoopMemberOffsets[k][l];
        int columnIndex = variableIndex - substructure->dqOffset;

        // printf(
        //   "    processing variable: %d, column: [%d], dqOffset: %d, \n     h: ",
        //   variableIndex,
        //   columnIndex,
        //   substructure->dqOffset);
        // s2_screw_print(context->h + variableIndex);

        cblas_daxpy(
          6,
          -1.0,
          (double*)(context->h + variableIndex),
          1,
          substructure->J + (j * 6 * substructure->dqCount + columnIndex),
          substructure->dqCount);
      }
    }


    FILE* outfile = fopen("jacobian.csv", "w");
    fprintf(f, "Jacobian in PO1:\n");
    for(int m = 0; m < substructure->constraintEquationCount; m++) {
      for(int n = 0; n < substructure->dqCount; n++) {
        fprintf(
          outfile,
          "% 0.9f,",
          substructure->J[m * substructure->dqCount + n]);
      }
      fprintf(outfile, "\n");
    }
    fclose(outfile);

    fprintf(f, "\n");

    int jacobianSize =
      substructure->dqCount * substructure->constraintEquationCount;

    double J1[jacobianSize]; /* copy of jacobian in PO1 */
    memcpy(J1, substructure->J, jacobianSize * sizeof(double));

    s2_substructure_setJacobianPO1(substructure);
    // printf("pr_12: ");
    // s2_arrayi_print(substructure->constraintEquationCount, substructure->pr_12);
    // printf("pc_12: ");
    // s2_arrayi_print(substructure->dqCount, substructure->pc_12);
    //
    // printf("pr_21: ");
    // s2_arrayi_print(substructure->constraintEquationCount, substructure->pr_21);
    // printf("pc_21: ");
    // s2_arrayi_print(substructure->dqCount, substructure->pc_21);
    //
    //
    // printf("factored Jacobian in PO1 (B matrix is set):\n");
    // for(int m = 0; m < substructure->constraintEquationCount; m++) {
    //   printf("   [ ");
    //   for(int n = 0; n < substructure->dqCount; n++) {
    //     printf(
    //       "% 0.9f ",
    //       substructure->J[m * substructure->dqCount + n]);
    //   }
    //   printf("]\n");
    // }
    // printf("\n");

    // double J2[jacobianSize];
    // memcpy(J2, substructure->J, jacobianSize * sizeof(double));

    s2_context_translatePermutations(context);
    fprintf(f, "  isDependentPO1: ");
    for(int j = 0; j < substructure->dqCount; j++) {
      fprintf(
        f,
        "%s ",
        context->isDependentPO1[
      j + substructure->dqOffset] ? "true" : "false");
    }
    fprintf(f, "\n");


    s2_substructure_computeMobility(substructure);
    s2_substructure_printMobility(f, substructure);

    // double m1[substructure->dqCount];
    // memcpy(
    //   m1,
    //   substructure->mobilityNumbers,
    //   substructure->dqCount * sizeof(double));


    // s2_substructure_setJacobianPO2(context, i);
    // s2_context_factorJacobianPO2_substructure(substructure);
    // s2_jacobian_setBMatrix_substructure(substructure);
    // s2_mobility_compute_substructure(substructure);
    // s2_mobility_print_substructure(context, i);
    //
    // double m2[substructure->dqCount];
    // memcpy(
    //   m2,
    //   substructure->mobilityNumbers,
    //   substructure->dqCount * sizeof(double));
    //
    //
    // s2_context_translatePermutations(context);
    // printf("  isDependentPO1: ");
    // for(int j = 0; j < substructure->dqCount; j++) {
    //   printf(
    //     "%s ",
    //     context->isDependentPO1[
    //       j + substructure->dqOffset] ? "true":"false");
    // }
    // assert(s2_all_close(substructure->dqCount, m1, m2));
    // printf("\n");
  }
}


/*!
@brief called after s2_context_initializeJacobian
*/
void s2_context_analyzeJacobian(s2_context_t* context) {
  s2_context_setJacobianPO1(context);
  s2_context_translatePermutations(context);
  s2_context_setJacobianBooleanPO2(context);
  s2_context_evaluateJacobianSparsity(context);
  s2_context_setJacobianIJMapping(context);
  s2_context_setJacobianPO2(context);
  s2_context_factorJacobianPO2(context);
  s2_context_setBMatrixSparse(context);
}


/*!
  @brief set pointer to substructure within each joint datastructure
*/
void s2_context_setSubstructurePointers(s2_context_t* context) {
  FILE* debug = context->modelParameters.debugFilePointer;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    for(int j = 0; j < context->activeJointCount; j++) {
      if(
        context->jointPointers[j]->dqIndex >= substructure->dqOffset &&
        context->jointPointers[j]->dqIndex < substructure->dqOffset + substructure->dqCount &&
        context->jointPointers[j]->jointType != S2_JOINT_RIGID) {
        s2_joint_t* joint = context->jointPointers[j];
        joint->substructurePointer = substructure;
      }
    }
  }
}


/*!
@brief check pivot ratios in all jacobians and call full factor if
pivot ratio has changed a specified threshold from initial starting value
*/
int s2_context_infinityNormBCheck(s2_context_t* context) {

  bool convergence = true;
  bool pass = true;
  context->iterationsSinceLastConstraintError++;
  s2_substructure_t* substructure = context->substructures;
  for(int i = 0; i < context->substructureCount; i++, substructure++) {
    // printf(
    //   "testing if %f > %f\n",
    //   substructure->infinityNormB,
    //   substructure->scaledInitialInfinityNormB);

    bool infinityNormFail = substructure->infinityNormB >
                            substructure->scaledInitialInfinityNormB;

    // double eSubstructure =
    //   s2_substructure_evaluateConstraintError(
    //     context->substructures + sub);

    // bool constraintFail =
    //   context->iterationsSinceLastConstraintError > 100 &&
    //   eSubstructure > 0.01;

    if(infinityNormFail) {
      // if(infinityNormFail || constraintFail) {

      context->iterationsSinceLastConstraintError = 0;

      FILE* debug = context->modelParameters.debugFilePointer;

      fprintf(debug, "performing full factor on substructure [%d] ", substructure->index);
      fprintf(debug, "infinityNormB: %f ", substructure->infinityNormB);
      fprintf(
        debug,
        "scaledInitialInfinityNormB: %f ", substructure->scaledInitialInfinityNormB);


      if(context->numericalIntegrator != NULL) {

        switch(context->modelParameters.integrationType) {
          case S2_NUMERICALINTEGRATOR_ODE113: {
            /* get current index */
            s2_numericalintegrator_ode113_t* numericalIntegrator =
              (s2_numericalintegrator_ode113_t*) context->numericalIntegrator;

            s2_stack_append(
              numericalIntegrator->reconfigurations,
              (int)(
                numericalIntegrator->tout / context->modelParameters.timeStep)
              - 1);

          }
          break;
          case S2_NUMERICALINTEGRATOR_ODE113B: {
            /* get current index */
            s2_numericalintegrator_ode113b_t* numericalIntegrator =
              (s2_numericalintegrator_ode113b_t*) context->numericalIntegrator;

            s2_stack_append(
              numericalIntegrator->reconfigurations,
              numericalIntegrator->step);

            // fprintf(debug, "y before nr: ");
            // s2_array_print(
            //   debug,
            //   numericalIntegrator->neqn,
            //   numericalIntegrator->yy);
            // fprintf(debug, "p before nr: ");
            // s2_array_print(
            //   debug,
            //   numericalIntegrator->neqn,
            //   numericalIntegrator->p);
          }
          break;
        }
      }

      double e = s2_substructure_evaluateConstraintError(substructure);
      fprintf(debug, "e before nr: % 0.9g ", e);

      pass = false;

      /* backup q array to qCopy */
      memcpy(
        context->qCopy + substructure->qOffset,
        context->q + substructure->qOffset,
        substructure->qCount * sizeof(double));
      // printf("substructure->qOffset: %d\n", substructure->qOffset);

      // fprintf(debug, "q before: ");
      // s2_array_print(
      //   debug,
      //   substructure->qCount,
      //   context->q + substructure->qOffset);

      // s2_substructure_setJacobianPO2(context, sub);
      // s2_context_factorJacobianPO2_substructure(substructure);


      int returnVal =
        s2_substructure_removeConstraintViolationsFast(substructure, true);

      e = s2_substructure_evaluateConstraintError(substructure);
      fprintf(debug, "e after nr: % 0.9g ", e);

      s2_substructure_setJacobianPO1(substructure);

      fprintf(debug, "rank: %d ", substructure->rank);

      if(returnVal) {
        fprintf(debug, "NR FAILED!\n");
        convergence = false;
        break;
      } else {
        fprintf(debug, "\n");
      }

      /* restore states */
      memcpy(
        context->q + substructure->qOffset,
        context->qCopy + substructure->qOffset,
        substructure->qCount * sizeof(double));

      s2_context_updateDHVA(context);
      s2_substructure_evaluateConstraints(substructure);
    }
  }
  if(!pass) {

    s2_context_updateDHVA(context);
    s2_context_setJacobianPO2(context);
    s2_context_factorJacobianPO2(context);
    s2_context_translatePermutations(context);
    s2_context_setJacobianBooleanPO2(context);
    s2_context_evaluateJacobianSparsity(context);
    s2_context_setJacobianIJMapping(context);
    s2_context_setJacobianPO2(context);
    s2_context_factorJacobianPO2(context);
    // printf("setBMatrixSparse in infinity norm b check:\n");
    s2_context_setBMatrixSparse(context);
    s2_context_evaluateConstraints(context);
    s2_context_setRHS(context);
    s2_context_setMassMatrix(context);
    s2_context_memoizeEquations(context);

    // s2_context_setJacobianPO2(context);
    // s2_context_factorJacobianPO2(context);
  }

  if(!convergence) {
    return -1;
  } else {
    return 0;
  }
}


/*!
@brief initialize and set initial jacobian in PO1

initializes and set ups an initial jacobian in PO1

*/
void s2_context_initializeJacobian(s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "\n# setting variable and joint lookup tables\n");
  s2_context_setJacobianLookup(context);

  // fprintf(debug, "\n# initializing storage for jacobian and equations of motion\n");
  s2_context_mallocEquations(context);

  // fprintf(debug, "\n# compressing loop members\n");
  s2_context_compressLoopMembers(context);

  // fprintf(debug, "\n# setting initial Jacobian in PO1\n");
  s2_context_setJacobianPO1(context);

  // fprintf(debug, "\n# translate local permutations to global permutations\n");
  s2_context_translatePermutations(context);

  // fprintf(debug, "\n# evaluate jacobian boolean and sparsity pattern in PO2\n");
  s2_context_setJacobianBooleanPO2(context);

  /* print jacobian boolean */
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   for(int j = 0; j < substructure->constraintEquationCount; j++) {
  //     for(int k = 0; k < substructure->dqCount; k++) {
  //       if(substructure->Jb[i * substructure->dqCount + k]) {
  //         fprintf(debug, "1 ");
  //       } else {
  //         fprintf(debug, "0 ");
  //       }
  //     }
  //     fprintf(debug, "\n");
  //   }
  //   fprintf(debug, "\n");
  // }

  // fprintf(debug, "verify jacobian boolean po2:\n");
  // fprintf(debug, "\n# evaluating sparsity\n");
  s2_context_evaluateJacobianSparsity(context);

  // fprintf(debug, "\n# setting ijmapping\n");
  s2_context_setJacobianIJMapping(context);

  s2_context_setJacobianPO2(context);

  // fprintf(debug, "\nJacobian before fast factor:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   fprintf(debug, "\nsubstructure [%d]:\n", i);
  //   s2_array_print2D(
  //     debug,
  //     substructure->constraintEquationCount,  substructure->dqCount,
  //     true,
  //     substructure->J);
  // }

  s2_context_factorJacobianPO2(context);

  // fprintf(debug, "\nJacobian after fast factor:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   fprintf(debug, "\nsubstructure [%d]:\n", i);
  //   s2_array_print2D(
  //     debug,
  //     substructure->constraintEquationCount,  substructure->dqCount,
  //     true,
  //     substructure->J);
  // }

  // fprintf(debug, "\n# setting sparse b matrix\n");
  s2_context_setBMatrixSparse(context);

  // fprintf(debug, "\n# initialize constraints\n");
  s2_context_setConstraintPointers(context);

  // fprintf(debug, "\n# setting joint->substructure pointers\n");
  s2_context_setSubstructurePointers(context);

  // printf("\n# printing jacobian\n");
  // s2_jacobian_print(context);

}
