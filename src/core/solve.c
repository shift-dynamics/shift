/**
@file
@brief Utilities to solve and numerically integrate the equations of motion
and other quantities requested by the user
@author Kristopher Wehage
@date 2016
@version 1.1
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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include "solve.h"
#include "context.h"
#include "numericalintegrator.h"
#include "substructure.h"
#include "jacobian.h"
#include "../force_modules/service.h"


/*!
@brief generic function to solve equations of motion for one time step
of numerical integration

*/
void s2_equation_fcn(
  double t,
  double y[],
  double dy[],
  s2_context_t* context) {

  /*
    copy y to context->q and context->dq

    this copy by value would not be necessary if the setEquations
    function was designed to work with relative memory pointers or indices

    candidate for future speed improvement
  */

  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      s2_numericalintegrator_ode113_t* numericalIntegrator =
        context->numericalIntegrator;
      numericalIntegrator->iterationCount++;
    }
    break;
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      s2_numericalintegrator_ode113b_t* numericalIntegrator =
        context->numericalIntegrator;
      numericalIntegrator->iterationCount++;
    }
    break;
  }
  memcpy(
    context->q,
    y,
    (context->qCount + context->dqCount) * sizeof(double));

  s2_updateDHVA(context);
  s2_jacobian_evaluateConstraints(context);
  s2_service_setForces(context, y);
  s2_equation_setEquations(context);

  /* make a copy of Ml and fl to verify equations of motion, remove this
  once equations of motion verified */
  // double Ml[context->dqCount * context->dqCount];
  // memset(Ml, 0, context->dqCount * context->dqCount  * sizeof(double));
  // for(int i = 0; i < context->nonRigidJointCount; i++) {
  //   int istart = i * (i + 1) / 2;
  //   for(int j = 0; j < i + 1; j++) {
  //     Ml[i * context->nonRigidJointCount + j] = context->Ml[istart + j];
  //     if(i != j) {
  //       Ml[j * context->nonRigidJointCount + i] = context->Ml[istart + j];
  //     }
  //   }
  // }

  /* copy fl to a second array to verify solution of
  equations of motion, uncomment for debugging */
  // double fl[context->dqCount];
  // memcpy(fl, context->fl, context->dqCount * sizeof(double));

  s2_equation_factorAndSolve(context);

  /* verify that multiplying coefficient matrix with lhs gets back the
  right hand side, comment out once verified */
  // s2_solve_verify(context, Ml, fl);

  /*
    convert angular velocities to time derivative of quaternions
    copy by value all other dq terms to dy array
  */
  s2_equation_dq_to_dy(context, dy);

  /*
    copy context->ddq to second half of dy, starting at qCount
  */
  memcpy(
    dy + context->qCount,
    context->ddq,
    context->dqCount * sizeof(double));
}


/*!
@brief writes headers for output file to stdout and to the output file
pointer specified in context->modelParameters->outFilePointer
*/
void s2_solve_writeHeaders(
  s2_context_t* context) {

  int neqn = 0;
  double* y = NULL;
  double t = 0;
  FILE* debug = context->modelParameters.debugFilePointer;

  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      s2_numericalintegrator_ode113_t* numericalIntegrator =
        (s2_numericalintegrator_ode113_t*)context->numericalIntegrator;
      neqn = numericalIntegrator->neqn;
      y = numericalIntegrator->y;
      t = numericalIntegrator->t;
    }
    break;
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      s2_numericalintegrator_ode113b_t* numericalIntegrator =
        (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
      neqn = numericalIntegrator->neqn;
      y = numericalIntegrator->y;
      t = numericalIntegrator->x;
    }
    break;
    default: {
      exit(-1);
    }
    break;
  }

  fprintf(debug, "T");
  for(int i = 0; i < context->qCount; i++) {
    fprintf(debug, "\tq[%d]", i);
  }
  for(int i = 0; i < context->dqCount; i++) {
    fprintf(debug, "\tdq[%d]", i);
  }
  for(int i = 0; i < context->dqCount; i++) {
    fprintf(debug, "\tddq[%d]", i);
  }
  fprintf(debug, "\n");

  fprintf(debug, "[0]\t%g", 0.0);
  for(int i = 0; i < neqn; i++) {
    fprintf(debug, "\t% 6.4f", y[i]);
  }
  fprintf(debug, "\n");

  FILE* f = context->modelParameters.outFilePointer;
  if(f != NULL) {

    fprintf(f, "time");
    for(int i = 0; i < context->qCount; i++) {
      fprintf(f, "%sq[%d]", context->modelParameters.delimiter, i);
    }
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(f, "%sdq[%d]", context->modelParameters.delimiter, i);
    }
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(f, "%sddq[%d]", context->modelParameters.delimiter, i);
    }

    fprintf(
      f,
      "%sKE",
      context->modelParameters.delimiter);
    fprintf(
      f,
      "%sPE",
      context->modelParameters.delimiter);
    fprintf(
      f,
      "%siterations",
      context->modelParameters.delimiter);

    fprintf(
      f,
      "%se",
      context->modelParameters.delimiter);
    fprintf(
      f,
      "%somega_n",
      context->modelParameters.delimiter);
    fprintf(f, "\n");

    /* write initial conditions to first row of file */
    fprintf(f, "%g", t);
    for(int i = 0; i < context->qCount + context->dqCount; i++) {
      fprintf(
        f,
        "%s%g",
        context->modelParameters.delimiter,
        y[i]);
    }

    s2_solve_kineticEnergy(context);
    fprintf(
      f,
      "%s%f",
      context->modelParameters.delimiter,
      context->kineticEnergy);

    s2_solve_gravitationalPotentialEnergy(context);
    fprintf(
      f,
      "%s%f",
      context->modelParameters.delimiter,
      context->gravitationalPotentialEnergy);

    fprintf(
      f,
      "%s%d",
      context->modelParameters.delimiter,
      0);

    double e = 0;
    for(int j = 0; j < context->substructureCount; j++) {
      s2_substructure_t* substructure = context->substructures + j;
      e += s2_newtonRaphson_evaluateConstraintError_substructure(
             substructure);
    }

    fprintf(
      f,
      "%s%e",
      context->modelParameters.delimiter,
      e);

    fprintf(
      f,
      "%s%f",
      context->modelParameters.delimiter,
      context->modelParameters.baumgarte2OmegaN / 2.0);
    fprintf(f, "\n");
  }
}


/*!
@brief writes states to output file, ydata array and/or stdout
*/
void s2_solve_writeStates(
  s2_context_t* context,
  int step) {

  int neqn = 0;
  double* ydata = NULL;
  double* y = NULL;
  double t = 0;
  FILE* debug = context->modelParameters.debugFilePointer;
  int iterationCount = 0;
  double omegaN = 0;
  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      s2_numericalintegrator_ode113_t* numericalIntegrator =
        context->numericalIntegrator;
      neqn = numericalIntegrator->neqn;
      ydata = numericalIntegrator->ydata;
      y = numericalIntegrator->y;
      t = numericalIntegrator->t;
      iterationCount = numericalIntegrator->iterationCount;
      omegaN = context->modelParameters.baumgarte2OmegaN / 2.0;
    }
    break;
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      s2_numericalintegrator_ode113b_t* numericalIntegrator =
        context->numericalIntegrator;
      neqn = numericalIntegrator->neqn;
      ydata = numericalIntegrator->ydata;
      y = numericalIntegrator->yy;
      t = numericalIntegrator->x;
      iterationCount = numericalIntegrator->iterationCount;
      /* write to tdata array if it exists */
      if(numericalIntegrator->tdata) {
        numericalIntegrator->tdata[step] = t;
      }
      iterationCount = numericalIntegrator->iterationCount;
      omegaN = numericalIntegrator->baumgarte2OmegaN / 2.0;
    }
    break;
    default: {
      exit(-1);
    }
    break;
  }

  /* copy states to ydata array if ydata is specified */
  if(ydata != NULL) {
    memcpy(
      ydata + neqn * step,
      y,
      neqn * sizeof(double));
  }

  /* do post processing and report requested information to user */
  // fprintf(debug, ">>> [%d]\t%0.4f", step, t);
  // for(int j = 0; j < context->qCount; j++) {
  //   fprintf(debug, "\t% 6.4f", y[j]);
  // }
  // fprintf(debug, "\n\tdq:");
  // for(int j = context->qCount; j < context->dqCount + context->qCount; j++) {
  //   fprintf(debug, "\t% 6.4g", y[j]);
  // }
  //
  // if(context->qCount + context->dqCount < neqn) {
  //   fprintf(debug, "\n\ty[n]:");
  //   for(int j = context->qCount + context->dqCount; j < neqn; j++) {
  //     fprintf(debug, "\t% 6.4g", y[j]);
  //   }
  // }
  //
  // fprintf(debug, "\n\tddq:");
  // for(int j = 0; j < context->dqCount; j++) {
  //   fprintf(debug, "\t% 6.4g", context->ddq[j]);
  // }

  double e = 0;
  for(int j = 0; j < context->substructureCount; j++) {
    s2_substructure_t* substructure = context->substructures + j;
    e += s2_newtonRaphson_evaluateConstraintError_substructure(
           substructure);
  }

  s2_solve_kineticEnergy(context);
  s2_solve_gravitationalPotentialEnergy(context);

  /* solve for absolute displacement of error */
// for(int j= 0; j < context->constraintEquationCount; j++)

  // fprintf(debug, "\te:% 6.6g", e);
  // fprintf(debug, " KE:% 6.6g", context->kineticEnergy);
  // fprintf(debug, " PE:% 6.4g", context->gravitationalPotentialEnergy);
  // fprintf(debug,
  //  " E:% 6.6g",
  //  context->kineticEnergy + context->gravitationalPotentialEnergy);
  //fprintf(debug, " i: %d", iterationCount);
  // fprintf(debug, " infNormB: ");

  //fprintf(debug, " omega_n: %0.9f ", omegaN);

  // for(int i = 0; i < context->substructureCount; i++) {
  //
  //   double infNormB =
  //     context->substructures[i].infinityNormB;
  //   fprintf(debug, "[%d] %0.9f ", i, infNormB);
  // }
  //fprintf(debug, "\n");

  // if(context->modelParameters->integrationType == ODE113B) {
  //   s2_numericalintegrator_ode113b_t* numericalIntegrator =
  //     (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
  //   printf("  yerr: ");
  //   for(int i = 0; i < numericalIntegrator->neqn; i++) {
  //     printf("% 0.7g ", numericalIntegrator->yerr[i]);
  //   }
  //   printf("\n");
  // }


  context->modelParameters.sumIterationCount += iterationCount;

  if(e > fabs(context->modelParameters.maxConstraintError)) {
    context->modelParameters.maxConstraintError = fabs(e);
    context->modelParameters.maxConstraintErrorStep = step;
  }

  if(
    iterationCount >
    context->modelParameters.maxIterationCount) {

    context->modelParameters.maxIterationCount = iterationCount;
    context->modelParameters.maxIterationCountStep = step;
  }
  context->modelParameters.sumConstraintError += e;

  FILE* f = context->modelParameters.outFilePointer;

  /* write to file if output file is specified */
  if(f != NULL) {
    /* write time */
    fprintf(
      f,
      "%g",
      t);

    /* write q / ddq */
    for(int i = 0; i < neqn; i++) {
      fprintf(
        f,
        "%s%g",
        context->modelParameters.delimiter,
        y[i]);
    }

    /* write ddq */
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(
        f,
        "%s%g",
        context->modelParameters.delimiter,
        context->ddq[i]);
    }

    /* write kinetic energy */
    fprintf(
      f,
      "%s%f",
      context->modelParameters.delimiter,
      context->kineticEnergy);

    /* write gravitational potential energy */
    fprintf(
      f,
      "%s%f",
      context->modelParameters.delimiter,
      context->gravitationalPotentialEnergy);

    /* write number of evaluations required for reporting interval */
    fprintf(
      f,
      "%s%d",
      context->modelParameters.delimiter,
      iterationCount);

    /* write constraint error */
    fprintf(
      f,
      "%s%e",
      context->modelParameters.delimiter,
      e);

    /* write omega N */
    fprintf(
      f,
      "%s%e",
      context->modelParameters.delimiter,
      omegaN);


    /* write velocity and acceleration for chassis */
    // s2_body_t* chassis = s2_context_getBodyByName(context, "chassis");
    // if(chassis) {
    //   // fprintf(debug, "found chassis\n");
    //   s2_displacement_t* d = chassis->referenceFrame->dAbsolute;
    //
    //   s2_screw_t* vGlobal = context->v + chassis->referenceFrame->vaIndex;
    //   s2_screw_t vLocal;
    //   s2_screwproduct_dinverses(d, vGlobal, &vLocal);
    //   for(int i = 0; i < 6; i++) {
    //     fprintf(
    //       f,
    //       "%s%e",
    //       context->modelParameters->delimiter,
    //       s2_screw_get(&vLocal, i));
    //   }
    //
    //   int i = chassis->index;
    //   int dqIndex = context->jointPointers[i]->dqIndex;
    //   s2_screw_t aGlobal;
    //   s2_screw_t* aq = context->aq + dqIndex;
    //
    //   s2_addequal_ss(aq, &aGlobal);
    //
    //   /* correct for gravitational term */
    //   for(int j = 0; j < 3; j++) {
    //     ((double*)&aGlobal)[j] +=
    //       s2_vector_get(context->modelParameters->gravity, j);
    //   }
    //
    //   while(i > -1) {
    //     int dqIndex = context->jointPointers[i]->dqIndex;
    //
    //     s2_screw_t* h = context->h + dqIndex;
    //     double ddq = context->ddq[dqIndex];
    //
    //     for(int j = 0; j < 6; j++) {
    //       ((double*)&aGlobal)[j] += ddq * s2_screw_get(h, j);
    //     }
    //     i = context->pcldq[i];
    //   }
    //
    //   /* transform acceleration to local body frame */
    //   s2_screw_t aLocal;
    //   s2_screwproduct_dinverses(d, &aGlobal, &aLocal);
    //
    //   /* add omega cross v in body frame to translational component of acceleration */
    //   s2_vector_t omegaCrossV;
    //   s2_vector_t* omegaLocal = (s2_vector_t*)(((double*)&vLocal) + 3);
    //   s2_cross_vv(omegaLocal, (s2_vector_t*)&vLocal, &omegaCrossV);
    //   for(int i = 0; i < 3; i++) {
    //     ((double*)&aLocal)[i] += ((double*)&omegaCrossV)[i];
    //   }
    //   for(int i = 0; i < 6; i++) {
    //     fprintf(
    //       f,
    //       "%s%e",
    //       context->modelParameters->delimiter,
    //       s2_screw_get(&aLocal, i));
    //   }
    // }
    fprintf(f, "\n");

  }
}


/*!
@brief wrapper function to ode integrator
*/
void s2_solve_integrateEquations(
  s2_context_t* context) {

  FILE* debug = context->modelParameters.debugFilePointer;

  s2_numericalintegrator_allocate(context);

  context->modelParameters.sumConstraintError = 0;
  context->modelParameters.maxConstraintError = 0;
  context->modelParameters.maxConstraintErrorStep = 0;
  context->modelParameters.sumIterationCount = 0;
  context->modelParameters.maxIterationCount = 0;
  context->modelParameters.maxIterationCountStep = 0;

  if(context->modelParameters.outFile) {
    context->modelParameters.outFilePointer =
      fopen(
        context->modelParameters.outFile, "w");
  }

  s2_solve_writeHeaders(context);


  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    assert(
      context->q + substructure->qOffset ==
      substructure->q);
  }

  int stepCount = 0;
  s2_stack_t* reconfigurations = NULL;
  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      clock_t begin = clock();
      s2_ode113_integrateEquations(context);
      clock_t end = clock();
      fprintf(debug, "simulation time: %f\n", (double)(end - begin) / CLOCKS_PER_SEC);
      s2_numericalintegrator_ode113_t* numericalIntegrator =
        (s2_numericalintegrator_ode113_t*)context->numericalIntegrator;
      stepCount = numericalIntegrator->stepCount;
      reconfigurations = numericalIntegrator->reconfigurations;

    }
    break;
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      clock_t begin = clock();
      s2_ode113b_integrateEquations(context);
      clock_t end = clock();
      fprintf(debug, "simulation time: %f\n", (double)(end - begin) / CLOCKS_PER_SEC);
      s2_numericalintegrator_ode113b_t* numericalIntegrator =
        (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
      stepCount = numericalIntegrator->stepCount;
      reconfigurations = numericalIntegrator->reconfigurations;

    }
    break;
    default: {
      exit(-1);
    }
    break;
  }

  if(context->modelParameters.outFilePointer) {
    fclose(context->modelParameters.outFilePointer);
  }

  fprintf(debug,
          "max constraint error: %e at time interval [%d]\n",
          context->modelParameters.maxConstraintError,
          context->modelParameters.maxConstraintErrorStep);
  fprintf(debug,
          "max number of iterations: %d at time interval [%d]\n",
          context->modelParameters.maxIterationCount,
          context->modelParameters.maxIterationCountStep);
  fprintf(debug,
          "average constraint error: %e\n",
          context->modelParameters.sumConstraintError / (double)stepCount);
  fprintf(debug,
          "average iteration count: %f\n",
          (double)context->modelParameters.sumIterationCount / (double)stepCount);
  fprintf(debug, "full factor performed at indices: ");
  s2_stack_print(debug, reconfigurations);
  fprintf(debug, "\n");

  FILE* f2 = fopen("reconfigurations.dat", "w");
  if(f2 != NULL) {
    s2_stacknode_t* stackHead = reconfigurations->head;
    while(stackHead != NULL) {
      fprintf(f2, "%d", stackHead->value);
      if(stackHead->next) {
        fprintf(f2, "%s", context->modelParameters.delimiter);
      }
      stackHead = stackHead->next;
    }
    fclose(f2);
  }
}


/*!
@brief compute total spatial momentum for system
*/
void s2_solve_momentum(s2_context_t* context) {

  memset(&(context->spatialMomentum), 0, sizeof(s2_screw_t));
  for(int i = 0; i < context->nonRigidJointCount; i++) {
    cblas_daxpy(
      6,
      1,
      (double*)(context->p + i),
      1,
      (double*)(&(context->spatialMomentum)),
      1);
  }
}


/*!
@brief compute kinetic energy for system

*/
void s2_solve_kineticEnergy(s2_context_t* context) {

  context->kineticEnergy =
    cblas_ddot(
      6 * context->nonRigidJointCount,
      (double*)(context->p),
      1,
      (double*)(context->v + 1),
      1) / 2.0;
}


/*!
@brief computes total gravitational potential energy for system

result is stored in context->gravitationalPotentialEnergy

@param context [in] context datastructure
*/
void s2_solve_gravitationalPotentialEnergy(s2_context_t* context) {

  context->gravitationalPotentialEnergy =
    -(cblas_ddot(
        3,
        (double*) & (context->inertiaP->mt),
        1,
        (double*)context->modelParameters.gravity,
        1));

}





/*!
@brief set coefficient matrix and multiply with answer to verify we get the
right hand side back
*/
bool s2_solve_verify(
  s2_context_t* context,
  double* Ml,
  double* fl) {

  FILE* debug = context->modelParameters.debugFilePointer;
  // printf("joint names in PO1:\n");
  // for(int i = 0; i < context->activeJointCount; i++) {
  //   printf("   [%d] ", i);
  //   s2_stackstr_print(context->jointPointers[i]->name);
  // }

  // printf("isDependent in po1:\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("  [%d] %s\n", i, context->isDependentPO1[i] ? "true":"false");
  // }
  // printf("\n");
  //
  // printf("isDependent in po3:\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("  [%d] %s\n", i, context->isDependentPO3[i] ? "true":"false");
  // }

  // printf("pv_12: ");
  // s2_arrayi_print(context->dqCount, context->pv_12);
  //
  // printf("pv_21: ");
  // s2_arrayi_print(context->dqCount, context->pv_21);
  //
  // printf("pv_13: ");
  // s2_arrayi_print(context->dqCount, context->pv_13);
  //
  // printf("pv_31: ");
  // s2_arrayi_print(context->dqCount, context->pv_31);
  //
  // printf("pv_32: ");
  // s2_arrayi_print(context->dqCount, context->pv_32);
  //
  // printf("pv_23: ");
  // s2_arrayi_print(context->dqCount, context->pv_23);

  /* get dim of jacobian */
  // int rank = 0;
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   rank += context->substructures[sub].rank;
  // }

  /* initialize coefficient matrix to zero */
  // double* Ml =
  //   calloc(
  //     context->nonRigidJointCount *
  //     context->nonRigidJointCount,
  //     sizeof(double));

  // printf("# updating dhva\n");
  // s2_updateDHVA(context);

  // printf("# verifying aq terms:\n");
  // for(int i = 0; i < context->dqCount + 1; i++) {
  //   printf("   [%d] ", i);
  //   s2_screw_print(context->aq + i);
  // }

  // printf("# evaluating constraints\n");
  // s2_jacobian_evaluateConstraints(context);

  // printf("# evaluating equations\n");
  // s2_equation_setEquations(context);

  // printf("b3 array after set equation: \n");
  // s2_array_print(context->nnzb, context->b3);

  // printf("\nMl after set equation: \n");
  // s2_triang_print(context->dqCount, context->Ml);

  /* copy Ml to full dense array */


  // printf("\nMl (full dense version) after set equation:\n");
  // s2_array_print2D(context->dqCount, context->dqCount, true, Ml);
  //
  // printf("\nfl after set equations: \n   ");
  // s2_array_print(context->dqCount, context->fl);

  // printf("\n# factoring and solving equations\n");
  // s2_equation_factorAndSolve(context);
  //
  // printf("b3 array after factor / solve equation: ");
  // s2_array_print(context->nnzb, context->b3);
  //
  // printf("\nddq (PO1): \n   ");
  // s2_array_print(context->dqCount, context->ddq);

  /* copy ddq to PO3 */
  double ddq_po3[context->dqCount];
  memset(ddq_po3, 0, context->dqCount * sizeof(double));
  for(int i = 0; i < context->dqCount; i++) {
    ddq_po3[i] = context->ddq[context->pv_13[i]];
  }

  // printf("ddq (po1):\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("[%d] %f\n", i, context->ddq[i]);
  // }

  // printf("context->pv_13:\n");
  // s2_arrayi_print(context->dqCount, context->pv_13);

  // printf("\nddq (PO3): \n   ");
  // s2_array_print(context->dqCount, ddq_po3);
  //
  // printf("\nla (PO2):\n   ");
  // s2_array_print(context->dqCount, context->la);

  double la_po3[context->dqCount];
  memset(la_po3, 0, context->dqCount * sizeof(double));
  for(int i = 0; i < context->dqCount; i++) {
    la_po3[i] = context->la[context->pv_23[i]];
  }

  double fl_po3[context->dqCount];
  memset(fl_po3, 0, context->dqCount * sizeof(double));
  // printf("\nempty fl_po3 array:\n  ");
  // s2_array_print(context->dqCount, fl_po3);
  //
  // printf("\nijMapping arrays:\n");
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   printf("   substructure [%d]\n", sub);
  //   s2_substructure_t* substructure = context->substructures + sub;
  //
  //   for(int i = 0; i < substructure->constraintEquationCount; i++) {
  //     printf("   ");
  //     for(int j = 0; j < substructure->dqCount; j++) {
  //       printf("%d ", substructure->ijMapping[i * substructure->dqCount + j]);
  //     }
  //     printf("\n");
  //   }
  // }



  cblas_dgemv(
    CblasRowMajor,
    CblasNoTrans,
    context->dqCount,
    context->dqCount,
    1,
    Ml,
    context->dqCount,
    ddq_po3,
    1,
    0.0,
    fl_po3,
    1);

  // printf("fl_po3 after adding Ml * ddq:\n");
  // s2_array_print(context->dqCount, fl_po3);


  // printf("\nfl_po3 <- Ml * ddq: \n   ");
  // s2_array_print(context->dqCount, fl_po3);

  /* set full dense b matrix for each substructure */
  // s2_jacobian_setBMatrix(context);

  /* print full dense B matrix for each substructure */
  // printf("full dense b matrix in PO2\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //
  //   printf("substructure [%d]\n", i);
  //   for(int i = 0; i < substructure->rank; i++) {
  //     for(int j = substructure->rank; j < substructure->dqCount; j++) {
  //       printf("%f ", substructure->J[i * substructure->dqCount + j]);
  //     }
  //     printf("\n");
  //   }
  //   printf("\n");
  // }

  double B[context->dqCount * context->dqCount];
  memset(B, 0, context->dqCount * context->dqCount * sizeof(double));
  int row = 0;
  for(int i = 0; i < context->dqCount; i++) {
    if(context->isDependentPO3[i]) {
      for(
        int j = context->rb3[i][0];
        j < context->rb3[i][1];
        j++) {

        int col = context->cb3[j];
        B[i * context->dqCount + col] = context->b3[j];
        B[i * context->dqCount + i] = 1.0;
      }
      row++;
    }
  }

  // printf("\nB matrix:\n");
  // s2_array_print2D(
  //   context->dqCount, context->dqCount, true, B);

  // printf("\nij mapping:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //
  //   for(int i = 0; i < substructure->constraintEquationCount; i++) {
  //     for(int j = 0; j < substructure->dqCount; j++) {
  //       printf("%d ", substructure->ijMapping[i * context->dqCount + j]);
  //     }
  //     printf("\n");
  //   }
  // }

  // printf("\nla_po3: ");
  // s2_array_print(context->dqCount, la_po3);
  // s2_array_print(context->dqCount, fl_po3);


  cblas_dgemv(
    CblasRowMajor,
    CblasTrans,
    context->dqCount,
    context->dqCount,
    1.0,
    B,
    context->dqCount,
    la_po3,
    1,
    1.0,
    fl_po3,
    1);

  bool verify = true;

  // printf("\nfl_po3 <- fl_po3 + B^T * la_po3\n   ");
  // printf("context->dqCount: %d\n", context->dqCount);
  // s2_array_print(context->dqCount, fl_po3);

  if(!s2_all_close(context->dqCount, fl_po3, fl)) {

    verify = false;

    fprintf(debug, "error: context->fl (po3): ");
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(debug, "%0.5g, ", fl[i]);
    }
    fprintf(debug, "\n");

    fprintf(debug, "         != fl_po3 (po3): ");
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(debug, "%0.5g, ", fl_po3[i]);
    }
    fprintf(debug, "\n");

  }

  double gamma_d_po3[context->dqCount];
  memset(gamma_d_po3, 0, context->dqCount * sizeof(double));
  cblas_dgemv(
    CblasRowMajor,
    CblasNoTrans,
    context->dqCount,
    context->dqCount,
    1.0,
    B,
    context->dqCount,
    ddq_po3,
    1,
    0.0,
    gamma_d_po3,
    1);

  /* permute gamma_d po3 to po2 */
  double gamma_d_po2[context->dqCount];
  memset(gamma_d_po2, 0, context->dqCount * sizeof(double));
  for(int i = 0; i < context->dqCount; i++) {
    gamma_d_po2[context->pv_23[i]] = gamma_d_po3[i];
  }

  if(!s2_all_close(context->dqCount, gamma_d_po2, context->gamma_d)) {

    fprintf(debug, "\n>>> Error, equations of motion not solved correctly.\n");

    verify = false;

    fprintf(debug, "\nerror: context->gamma_d (po2): \n");
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(debug, "%0.5g, ", context->gamma_d[i]);
    }
    fprintf(debug, "\n");

    fprintf(debug, "\n             != gamma_d (po2): \n");
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(debug, "%0.5g, ", gamma_d_po2[i]);
    }
    fprintf(debug, "\n");
  }

  // s2_solve_kineticEnergy(context);
  // printf("  KE: %f ", context->kineticEnergy);
  //
  // s2_solve_gravitationalPotentialEnergy(context);
  // printf("  PE: %f ", context->gravitationalPotentialEnergy);
  //
  // printf(
  //   "  E: %f",
  //   context->kineticEnergy + context->gravitationalPotentialEnergy);

  // double e = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   e += s2_newtonRaphson_evaluateConstraintError_substructure(
  //     substructure);
  // }
  // printf("  e: %f\n", e);

  // s2_inertia_print(context->inertiaP);
  // printf("verifying aq:\n");
  // for(int i = 0; i < context->dqCount + 1; i++) {
  //   printf("[%d] ", i);
  //   s2_screw_print(context->aq + i);
  // }
  // printf("cs (PO1): ");


  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   s2_array_print(
  //     substructure->constraintEquationCount,
  //     substructure->cs);
  //   printf(
  //     "pivotRatio: %f, ",
  //     s2_jacobian_pivotRatio(
  //       substructure->dqCount,
  //       substructure->rank,
  //       substructure->J));
  //   printf(
  //     "infNormB: %f\n",
  //     s2_jacobian_infinityNormB(
  //       substructure->constraintEquationCount,
  //       substructure->dqCount,
  //       substructure->rank,
  //       substructure->J));
  // }

  if(!verify) {

    fprintf(debug, "\nJoint Names:\n");
    for(int i = 0; i < context->activeJointCount; i++) {
      fprintf(debug, "   ");
      s2_stackstr_print(debug, context->jointPointers[i]->name);
    }
    fprintf(debug, "\nis dependent PO1: ");
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(debug, "%s, ", context->isDependentPO1[i] ? "true" : "false");
    }
    fprintf(debug, "\n");

    fprintf(debug, "\nSubstructures:\n");
    for(int i = 0; i < context->substructureCount; i++) {
      fprintf(debug,
              "rank: %d\n",
              context->substructures[i].rank);
    }

    fprintf(debug, "\nis dependent PO3: ");
    for(int i = 0; i < context->dqCount; i++) {
      fprintf(debug, "%s, ", context->isDependentPO3[i] ? "true" : "false");
    }
    fprintf(debug, "\n");

    fprintf(debug, "\npv_12: ");
    s2_arrayi_print(debug, context->dqCount, context->pv_12);

    fprintf(debug, "\npv_21: ");
    s2_arrayi_print(debug, context->dqCount, context->pv_21);

    fprintf(debug, "\npv_13: ");
    s2_arrayi_print(debug, context->dqCount, context->pv_13);

    fprintf(debug, "\npv_31: ");
    s2_arrayi_print(debug, context->dqCount, context->pv_31);

    fprintf(debug, "\npv_23: ");
    s2_arrayi_print(debug, context->dqCount, context->pv_23);

    fprintf(debug, "\npv_32: ");
    s2_arrayi_print(debug, context->dqCount, context->pv_32);

    exit(-1);
  }
  return verify;
}
