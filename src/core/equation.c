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
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include "context.h"
#include "newtonraphson.h"
#include "substructure.h"

/*!
  @brief initializes quantities needed to solve system of equations
*/
void s2_context_initializeEquations(s2_context_t* context) {

  s2_context_initializeInertia(context);
  if(context->mhProduct) {
    free(context->mhProduct);
  }
  context->mhProduct = calloc(context->dqCount, sizeof(s2_screw_t));
  if(context->fl) {
    free(context->fl);
  }
  context->fl = calloc(context->dqCount, sizeof(double));
  if(context->g) {
    free(context->g);
  }
  context->g = calloc(context->nonRigidJointCount, sizeof(s2_screw_t));
  if(context->p) {
    free(context->p);
  }
  context->p = calloc(context->nonRigidJointCount, sizeof(s2_screw_t));
  if(context->gamma_d) {
    free(context->gamma_d);
  }
  context->gamma_d = calloc(context->dqCount, sizeof(double));
  if(context->la) {
    free(context->la);
  }
  context->la = calloc(context->dqCount, sizeof(double));

  s2_context_removeConstraintViolations(context);
  s2_context_analyzeJacobian(context);
  s2_context_updateDHVA(context);
  s2_context_setEquations(context);
  s2_context_memoizeEquations(context);
  s2_context_memoizeRestraints(context);
}

/*!
  @brief evaluate right hand side of equations of motion
  Step 1 is carried out in PO1

  f^{l} = f + H^T * C^{-T} (g + \tilde{v}^T(MV) - M \gamma)

  "aq" (accelerations that are quadratic in velocity)
  is equivalent to "\gamma"

  setRHS must be called before setMassMatrix
*/
void s2_context_setRHS(s2_context_t* context) {

  s2_context_updateDHVA(context);
  s2_context_evaluateConstraints(context);

  /* g and f are cleared in s2_service_setForces in service.c */
  memset(context->fl, 0, context->dqCount * sizeof(double));
  memset(context->p, 0, context->nonRigidJointCount * sizeof(s2_screw_t));
  s2_screw_t* aq = context->aq + 1;
  s2_screw_t* v = context->v + 1;

#if _SHIFT_VERBOSE
  FILE* dReference = fopen("dReference.csv", "w");
  printf("inertia before mass projection / local:\n");
  for(int i = 0; i < context->nonRigidJointCount; i++) {
    printf("[%d] hasmass %s ", i, context->hasMass[i] ? "true" : "false");
    s2_inertia_print(stdout, context->inertia + i);
  }
#endif

  /* transform inertias to fixed reference frame, carried out in PO1*/
  memset(context->inertiaP, 0,
         (context->nonRigidJointCount + 1) * sizeof(s2_inertia_t));
  s2_inertia_t* inertiaP = context->inertiaP + 1;

  for(int i = 0; i < context->nonRigidJointCount; i++) {
#if _SHIFT_VERBOSE
    fprintf(dReference, "[%d]\n", i);
    s2_displacement_print(dReference, context->dReferenceFrames[i]);
    fprintf(dReference, "\n");
#endif
    if(context->hasMass[i]) {

      s2_inertia_transform(context->inertia + i, context->dReferenceFrames[i],
                           inertiaP + i);

#if _SHIFT_VERBOSE
      printf("[%d] inertia in body frame:\n", i);
      s2_inertia_print(stdout, context->inertia + i);

      printf("absolute displacement:\n");
      s2_displacement_print(stdout, context->dReferenceFrames[i]);

      printf("inertia transformed to fixed reference frame:\n");
      s2_inertia_print(stdout, inertiaP + i);
#endif
    }
  }

#if _SHIFT_VERBOSE
  fclose(dReference);
#endif

/* g <- g - M \gamma, (carried out in PO1) */
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(int j = 0; j < context->nonRigidJointCount; j++) {

    context->g[j].m0 -= inertiaP[j].m * aq[j].m0;
    context->g[j].m1 -= inertiaP[j].m * aq[j].m1;
    context->g[j].m2 -= inertiaP[j].m * aq[j].m2;

    context->g[j].m4 -= inertiaP[j].mt.z * aq[j].m0;
    context->g[j].m5 += inertiaP[j].mt.y * aq[j].m0;

    context->g[j].m3 += inertiaP[j].mt.z * aq[j].m1;
    context->g[j].m5 -= inertiaP[j].mt.x * aq[j].m1;

    context->g[j].m3 -= inertiaP[j].mt.y * aq[j].m2;
    context->g[j].m4 += inertiaP[j].mt.x * aq[j].m2;

    context->g[j].m1 += inertiaP[j].mt.z * aq[j].m3;
    context->g[j].m2 -= inertiaP[j].mt.y * aq[j].m3;

    context->g[j].m3 -= inertiaP[j].Ixx * aq[j].m3;
    context->g[j].m4 -= inertiaP[j].Ixy * aq[j].m3;
    context->g[j].m5 -= inertiaP[j].Izx * aq[j].m3;

    context->g[j].m0 -= inertiaP[j].mt.z * aq[j].m4;
    context->g[j].m2 += inertiaP[j].mt.x * aq[j].m4;

    context->g[j].m3 -= inertiaP[j].Ixy * aq[j].m4;
    context->g[j].m4 -= inertiaP[j].Iyy * aq[j].m4;
    context->g[j].m5 -= inertiaP[j].Iyz * aq[j].m4;

    context->g[j].m0 += inertiaP[j].mt.y * aq[j].m5;
    context->g[j].m1 -= inertiaP[j].mt.x * aq[j].m5;

    context->g[j].m3 -= inertiaP[j].Izx * aq[j].m5;
    context->g[j].m4 -= inertiaP[j].Iyz * aq[j].m5;
    context->g[j].m5 -= inertiaP[j].Izz * aq[j].m5;
#ifndef _SHIFT_USE_OPENMP
  }

  /* p = Mv, (carried out in PO1) */
  for(int j = 0; j < context->nonRigidJointCount; j++) {
#endif
    context->p[j].m0 += inertiaP[j].m * v[j].m0;
    context->p[j].m1 += inertiaP[j].m * v[j].m1;
    context->p[j].m2 += inertiaP[j].m * v[j].m2;

    context->p[j].m4 += inertiaP[j].mt.z * v[j].m0;
    context->p[j].m5 -= inertiaP[j].mt.y * v[j].m0;

    context->p[j].m3 -= inertiaP[j].mt.z * v[j].m1;
    context->p[j].m5 += inertiaP[j].mt.x * v[j].m1;

    context->p[j].m3 += inertiaP[j].mt.y * v[j].m2;
    context->p[j].m4 -= inertiaP[j].mt.x * v[j].m2;

    context->p[j].m1 -= inertiaP[j].mt.z * v[j].m3;
    context->p[j].m2 += inertiaP[j].mt.y * v[j].m3;

    context->p[j].m3 += inertiaP[j].Ixx * v[j].m3;
    context->p[j].m4 += inertiaP[j].Ixy * v[j].m3;
    context->p[j].m5 += inertiaP[j].Izx * v[j].m3;

    context->p[j].m0 += inertiaP[j].mt.z * v[j].m4;
    context->p[j].m2 -= inertiaP[j].mt.x * v[j].m4;

    context->p[j].m3 += inertiaP[j].Ixy * v[j].m4;
    context->p[j].m4 += inertiaP[j].Iyy * v[j].m4;
    context->p[j].m5 += inertiaP[j].Iyz * v[j].m4;

    context->p[j].m0 -= inertiaP[j].mt.y * v[j].m5;
    context->p[j].m1 += inertiaP[j].mt.x * v[j].m5;

    context->p[j].m3 += inertiaP[j].Izx * v[j].m5;
    context->p[j].m4 += inertiaP[j].Iyz * v[j].m5;
    context->p[j].m5 += inertiaP[j].Izz * v[j].m5;
  }

  /* g += cross(v)^T * p (carried out in PO1)*/
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(int j = 0; j < context->nonRigidJointCount; j++) {

    // context->g[j][0:3] -= cross(context->v[j][3:6], context->p[j][0:3]);
    // x1 = m3, y1 = m4, z1 = m5; x2 = m0, y2 = m1, z2 = m2
    context->g[j].m0 -= v[j].m4 * context->p[j].m2;
    context->g[j].m1 -= v[j].m5 * context->p[j].m0;
    context->g[j].m2 -= v[j].m3 * context->p[j].m1;

    // context->g[j][3:6] -= cross(context->v[j][0:3], context->p[j][0:3]);
    // x1 = m0, y1 = m1, z1 = m2; x2 = m0, y2 = m1, z2 = m2
    context->g[j].m3 -= v[j].m1 * context->p[j].m2;
    context->g[j].m4 -= v[j].m2 * context->p[j].m0;
    context->g[j].m5 -= v[j].m0 * context->p[j].m1;

    // context->g[j][3:6] -= cross(context->v[j][3:6], context->p[j][3:6]);
    // x1 = m3, y1 = m4, z1 = m5; x2 = m3, y2 = m4, z2 = m5
    context->g[j].m3 -= v[j].m4 * context->p[j].m5;
    context->g[j].m4 -= v[j].m5 * context->p[j].m3;
    context->g[j].m5 -= v[j].m3 * context->p[j].m4;
  }

  for(int j = 0; j < context->nonRigidJointCount; j++) {
    context->g[j].m0 += v[j].m5 * context->p[j].m1;
    context->g[j].m1 += v[j].m3 * context->p[j].m2;
    context->g[j].m2 += v[j].m4 * context->p[j].m0;

    context->g[j].m3 += v[j].m2 * context->p[j].m1;
    context->g[j].m4 += v[j].m0 * context->p[j].m2;
    context->g[j].m5 += v[j].m1 * context->p[j].m0;

    context->g[j].m3 += v[j].m5 * context->p[j].m4;
    context->g[j].m4 += v[j].m3 * context->p[j].m5;
    context->g[j].m5 += v[j].m4 * context->p[j].m3;
  }

  /* g <- C{-T} * (g + \tilde{v}^T p - m\gamma) (carried out in PO1) */
  for(int j = context->nonRigidJointCount - 1; j > -1; j--) {
    int i = context->pclCompressed[j];
    if(i > -1) {
      context->g[i].m0 += context->g[j].m0;
      context->g[i].m1 += context->g[j].m1;
      context->g[i].m2 += context->g[j].m2;
      context->g[i].m3 += context->g[j].m3;
      context->g[i].m4 += context->g[j].m4;
      context->g[i].m5 += context->g[j].m5;
    }
  }
// for(int i = 0; i < context->dqCount; i++) {
//   printf("g[%d] (after): ", i);
//   s2_screw_print(context->g + i);
// }

/*
  f_l = f + H^T * C{-T} * (g + \tilde{v}^Tp - m\gamma)
  this step is evaluated in PO1 and stored in PO3,
  use the permutation array p_v31 to assign to f_l

  at this point the quantity g contains C{-T} (g + \tilde{v}^Tp - m\gamma)
  which has length nonRigidJointCount and is stored in PO1.

  Expand each multi DOF joint to expand the quantity in g with length
  nonRigidJointCount in PO1 to fl of length dqCount in PO3
*/
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(int j = 0; j < context->nonRigidJointCount; j++) {
    /* j is index into compressed list of joints and g in PO1,
    increment l and k for each additional degree of freedom
    for composite joints */
    for(int m = 0; m < context->jointDOFCompressed[j]; m++) {
      /* l = index into dq array in PO1 */
      int l = context->dqLookupCompressed[j] + m;

      /* k is index into dq array in PO3 */
      int k = context->pv_31[l];

      /* f <- f_l */
      context->fl[k] = context->f[l];
      if(context->isRotation[l]) {
        /* f_l += h^T * g yields the final f_l*/
        context->fl[k] += s2_dot_ss(context->h + l, context->g + j);
      } else {
        context->fl[k] += context->h[l].m0 * context->g[j].m0;
        context->fl[k] += context->h[l].m1 * context->g[j].m1;
        context->fl[k] += context->h[l].m2 * context->g[j].m2;
        // context->fl[k] += cblas_ddot(3, (double*)(context->h + l), 1,
        // (double*)(context->g + j), 1);
      }
    }
  }

  /* evaluate primary constraints (\bar{\gamma}_p) in PO(1), store in PO(2) */
  double baumgarte2OmegaN = context->modelParameters.baumgarte2OmegaN;
  double baumgarteOmegaN2 = context->modelParameters.baumgarteOmegaN2;

  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      if(context->numericalIntegrator) {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

        baumgarte2OmegaN = numericalIntegrator->baumgarte2OmegaN;
        baumgarteOmegaN2 = numericalIntegrator->baumgarteOmegaN2;
      }
      break;
    }
    default:
      break;
  }

  int chordConstraintOffset = 0;
  for(int i = 0; i < context->substructureCount; i++) {

    s2_substructure_t* substructure = context->substructures + i;
    int offset = substructure->dqOffset;
    // printf("offset for substructure [%d]: %d\n", i, substructure->dqOffset);

    /* number of constraints to evaluate */
    for(int k = 0; k < substructure->chordConstraintCount;
        k++, chordConstraintOffset++) {

      // FILE* debug = context->modelParameters->debugFilePointer;
      // fprintf(debug, "\n   [%d]\n", chordConstraintOffset);

      // printf(
      //   "constraints[chordConstraintOffset][1]: %d\n",
      //   context->constraints[chordConstraintOffset][1]);
      // printf(
      //   "constraints[chordConstraintOffset][0]: %d\n",
      //   context->constraints[chordConstraintOffset][0]);

      // fprintf(debug, "   body[1]: ");
      // s2_stackstr_print(
      //   debug,
      //   context->bodyPointers[
      //     context->constraints[chordConstraintOffset][1]]->name);
      //
      // fprintf(debug, "   body[0]: ");
      // s2_stackstr_print(
      //   debug,
      //   context->bodyPointers[
      //     context->constraints[chordConstraintOffset][0]]->name);

      // for(int i = 0; i < context->activeJointCount; i++) {
      //   s2_stackstr_print(debug, context->bodyPointers[i]->name);
      //   fprintf(debug, "aq: ");
      //   s2_screw_print(
      //     debug,
      //     context->aq + context->bodyPointers[i]->referenceFrame->vaIndex);
      //   fprintf(debug, "v: ");
      //   s2_screw_print(
      //     debug,
      //     context->v + context->bodyPointers[i]->referenceFrame->vaIndex);
      // }

      // fprintf(debug, "   v[1] (po1): ");
      // s2_screw_print(
      //   debug,
      //   context->v + context->constraints[chordConstraintOffset][1]);
      //
      // fprintf(debug, "   v[0] (po1): ");
      // s2_screw_print(
      //   debug,
      //   context->v + context->constraints[chordConstraintOffset][0]);
      //
      //
      // fprintf(debug, "   aq[1] (po1): ");
      // s2_screw_print(
      //   debug,
      //   context->aq + context->constraints[chordConstraintOffset][1]);
      //
      // fprintf(debug, "   aq[0] (po1): ");
      // s2_screw_print(
      //   debug,
      //   context->aq + context->constraints[chordConstraintOffset][0]);
      //
      // fprintf(debug, "   phi (po1): ");
      // s2_array_print(
      //   debug,
      //   6,
      //   substructure->cs + 6 * k);

      // double phi_po1[substructure->constraintEquationCount];
      // double phi_po2[substructure->constraintEquationCount];
      // memset(phi_po1, 0, substructure->constraintEquationCount *
      // sizeof(double)); memset(phi_po2, 0,
      // substructure->constraintEquationCount * sizeof(double)); memcpy(
      //   phi_po1 + 6 * k,
      //   context->cs + 6 * chordConstraintOffset,
      //   sizeof(s2_screw_t));
      // for(int q = 0; q < substructure->constraintEquationCount; q++) {
      //   phi_po2[q] = phi_po1[substructure->pr_12[q]];
      // }
      // fprintf(debug, "   phi (po2):\t\t\t");
      // for(int q = 0; q < substructure->rank; q++) {
      //   fprintf(debug, "% 0.9g\t", phi_po2[q]);
      // }
      // fprintf(debug, "\n");
      //
      // s2_screw_t phi_dot;
      // memset(&phi_dot, 0, sizeof(s2_screw_t));
      // s2_subtract_ss(
      //   context->v + context->constraints[chordConstraintOffset][1],
      //   context->v + context->constraints[chordConstraintOffset][0],
      //   &phi_dot);

      // double phi_dot_po1[substructure->constraintEquationCount];
      // double phi_dot_po2[substructure->constraintEquationCount];
      // memset(phi_dot_po1, 0, substructure->constraintEquationCount *
      // sizeof(double)); memset(phi_dot_po2, 0,
      // substructure->constraintEquationCount * sizeof(double));
      // memcpy(phi_dot_po1 + 6 * k, &phi_dot, sizeof(s2_screw_t));
      // for(int q = 0; q < substructure->constraintEquationCount; q++) {
      //   phi_dot_po2[q] = phi_dot_po1[substructure->pr_12[q]];
      // }
      // fprintf(debug, "   phi_dot (po2):\t\t");
      // for(int q = 0; q < substructure->rank; q++) {
      //   fprintf(debug, "% 0.9g\t", phi_dot_po2[q]);
      // }
      // fprintf(debug, "\n");

      assert(substructure->constraints[k] ==
             context->constraints[chordConstraintOffset]);

      /* set gamma_p */
      for(int l = 0; l < 6; l++) {
        if(substructure->pr_21[k * 6 + l] < substructure->rank) {
          /*
            first constraint is embedded (host) body
            second is the virtual body (released frame)
          */

          /*
            prInverse = pr_21
            pr = p12
          */
          context->gamma_d[offset + substructure->pr_21[k * 6 + l]] =
            /* 0 is the index of the velocity of the child (-) side */
            ((double*)(context->aq +
                       context->constraints[chordConstraintOffset][0]))[l] -
            /* 1 is the index of the velocity of the parent (+) side */
            ((double*)(context->aq +
                       context->constraints[chordConstraintOffset][1]))[l];
        }
      }

      // fprintf(debug, "   gamma_p (po2) initial:\t");
      // for(int q = offset; q < offset + substructure->rank; q++) {
      //   fprintf(
      //     debug,
      //     "% 0.9g\t",
      //     context->gamma_d[q]);
      // }
      // fprintf(debug, "\n");

      /* subtract 2 * omega_n * \dot{phi} from gamma_p */
      for(int l = 0; l < 6; l++) {
        if(substructure->pr_21[k * 6 + l] < substructure->rank) {

          /* -2omega N * \dot{\phi} velocity part of baumgarte
          constraint stabilization */
          context->gamma_d[offset + substructure->pr_21[k * 6 + l]] -=
            baumgarte2OmegaN *
            (((double*)(context->v +
                        context->constraints[chordConstraintOffset][1]))[l] -
             ((double*)(context->v +
                        context->constraints[chordConstraintOffset][0]))[l]);
        }
      }

      // fprintf(debug, "   gamma_p (po2) -= 2w phi_dot:\t");
      // for(int q = offset; q < offset + substructure->rank; q++) {
      //   fprintf(debug, "% 0.9g\t", context->gamma_d[q]);
      // }
      // fprintf(debug, "\n");

      /* subtract omega_n^2 * \dot{phi} from gamma_p */
      for(int l = 0; l < 6; l++) {
        if(substructure->pr_21[k * 6 + l] < substructure->rank) {

          /* -omegaN^2 * phi, position part of baumgarte
          constraint stabilization */
          context->gamma_d[offset + substructure->pr_21[k * 6 + l]] -=
            baumgarteOmegaN2 * substructure->cs[k * 6 + l];

          assert(substructure->cs + k * 6 ==
                 context->cs + chordConstraintOffset * 6);
        }
      }

      /* substract integral error term from gamma_p  for adaptive constraint
       * stabilization*/
      if(context->modelParameters.integrationType ==
           S2_NUMERICALINTEGRATOR_ODE113B &&
         context->numericalIntegrator != NULL) {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

        double* integralErrorTerm = numericalIntegrator->integralError;

        for(int l = 0; l < 6; l++) {
          if(substructure->pr_21[k * 6 + l] < substructure->rank) {

            context->gamma_d[offset + substructure->pr_21[k * 6 + l]] -=
              integralErrorTerm[k * 6 + l];
          }
        }
      }
    }
  }

  /*
    precondition primary constraints in PO(2)
    \bar{\gamma}_d = U^{-1} L^{-1} \bar{\gamma}_p
  */
  for(int sub = 0; sub < context->substructureCount; sub++) {
    s2_substructure_t* substructure = context->substructures + sub;

    int offset = substructure->dqOffset;

    /*
      forward elimination
      \bar{\gamma} <- L^{-1} * \bar\{\gamma}
    */
    for(int i = 0; i < substructure->rank; i++) {
      for(int j = 0; j < i; j++) {
        context->gamma_d[i + offset] -=
          substructure->J[i * substructure->dqCount + j] *
          context->gamma_d[j + offset];
      }
    }

    /*
      back substitution
      \bar{\gamma} <- U^{-1} * \bar\{\gamma}
    */
    for(int i = substructure->rank - 1; i > -1; i--) {
      double save = 0;
      for(int j = substructure->rank - 1; j > i; j--) {
        save += substructure->J[i * substructure->dqCount + j] *
                context->gamma_d[j + offset];
      }
      context->gamma_d[i + offset] =
        (context->gamma_d[i + offset] - save) /
        substructure->J[i * substructure->dqCount + i];
    }
  }
}

void s2_inertia_print2(FILE* f, s2_inertia_t* inertia, int index,
                       const char* name) {

  fprintf(f, "    i%d = np.matrix([\n", index);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertia->m, 0.f, 0.f, 0.f, inertia->mt.z, -inertia->mt.y);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, inertia->m, 0.f, -inertia->mt.z, 0.f, inertia->mt.x);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, inertia->m, inertia->mt.y, -inertia->mt.x, 0.f);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, -inertia->mt.z, inertia->mt.y, inertia->Ixx, inertia->Ixy,
          inertia->Izx);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertia->mt.z, 0.f, -inertia->mt.x, inertia->Ixy, inertia->Iyy,
          inertia->Iyz);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g]])\n",
          -inertia->mt.y, inertia->mt.x, 0.f, inertia->Izx, inertia->Iyz,
          inertia->Izz);

  fprintf(f, "    if i%d.any() and not np.all(np.linalg.eigvals(i%d) > 0):\n",
          index, index);
  fprintf(f, "        print(\"[%d] %s not positive definite\")\n", index, name);
  fprintf(f, "        print(i%d)\n", index);
}

void s2_inertia_print2Original(FILE* f, s2_inertia_t* inertia, int index,
                               const char* name) {

  fprintf(f, "    i%d = np.matrix([\n", index);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertia->m, 0.f, 0.f, 0.f, 0.f, 0.f);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, inertia->m, 0.f, 0.f, 0.f, 0.f);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, inertia->m, 0.f, 0.f, 0.f);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, 0.f, inertia->Ixx, inertia->Ixy, inertia->Izx);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, 0.f, inertia->Ixy, inertia->Iyy, inertia->Iyz);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g]])\n",
          0.f, 0.f, 0.f, inertia->Izx, inertia->Iyz, inertia->Izz);

  fprintf(f, "    if i%d.any() and not np.all(np.linalg.eigvals(i%d) > 0):\n",
          index, index);
  fprintf(f, "        print(\"[%d] %s not positive definite\")\n", index, name);
  fprintf(f, "        print(i%d)\n", index);
}

void s2_inertia_print2Reference(FILE* f, s2_inertia_t* inertiaReference,
                                s2_inertia_t* inertiaFixed, int index,
                                const char* name, s2_displacement_t* d) {

  s2_vector_t t;
  if(inertiaReference->m > 1e-9) {
    double invMass = 1 / inertiaReference->m;
    s2_product_vk(&inertiaReference->mt, &invMass, &t);
  }

  s2_vector_t mt;
  s2_vector_copy(&inertiaReference->mt, &mt);

  fprintf(f, "    i%d = np.matrix([\n", index);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertiaReference->m, 0.f, 0.f, 0.f, inertiaReference->mt.z,
          -inertiaReference->mt.y);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, inertiaReference->m, 0.f, -inertiaReference->mt.z, 0.f,
          inertiaReference->mt.x);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, inertiaReference->m, inertiaReference->mt.y,
          -inertiaReference->mt.x, 0.f);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, -inertiaReference->mt.z, inertiaReference->mt.y,
          inertiaReference->Ixx - (-mt.y * t.y - mt.z * t.z),
          inertiaReference->Ixy - (mt.y * t.x),
          inertiaReference->Izx - (mt.z * t.x));
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertiaReference->mt.z, 0.f, -inertiaReference->mt.x,
          inertiaReference->Ixy - (mt.x * t.y),
          inertiaReference->Iyy - (-mt.z * t.z - mt.x * t.x),
          inertiaReference->Iyz - (mt.z * t.y));
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g]])\n",
          -inertiaReference->mt.y, inertiaReference->mt.x, 0.f,
          inertiaReference->Izx - (mt.x * t.z),
          inertiaReference->Iyz - (mt.y * t.z),
          inertiaReference->Izz - (-mt.x * t.x - mt.y * t.y));

  fprintf(f, "    r = np.matrix([\n");
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          d->R.m00, d->R.m01, d->R.m02, 0.f, 0.f, 0.f);

  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          d->R.m10, d->R.m11, d->R.m12, 0.f, 0.f, 0.f);

  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          d->R.m20, d->R.m21, d->R.m22, 0.f, 0.f, 0.f);

  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, 0.f, d->R.m00, d->R.m01, d->R.m02);

  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, 0.f, d->R.m10, d->R.m11, d->R.m12);

  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g]])\n",
          0.f, 0.f, 0.f, d->R.m20, d->R.m21, d->R.m22);

  fprintf(f, "    t = np.matrix(np.eye(6))\n");
  fprintf(f, "    t[0, 4] = %16.12g\n", -d->T.z);
  fprintf(f, "    t[0, 5] = %16.12g\n", d->T.y);
  fprintf(f, "    t[1, 3] = %16.12g\n", d->T.z);
  fprintf(f, "    t[1, 5] = %16.12g\n", -d->T.x);
  fprintf(f, "    t[2, 3] = %16.12g\n", -d->T.y);
  fprintf(f, "    t[2, 4] = %16.12g\n", d->T.x);
  fprintf(f, "    d = t * r\n");
  fprintf(f, "    dinv = np.linalg.inv(d)\n");
  fprintf(f, "    dinvT = dinv.transpose()\n");
  fprintf(f, "    iFixed = dinvT * i%d * dinv\n", index);
  fprintf(f, "    i%dFixed = np.matrix([\n", index);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertiaFixed->m, 0.f, 0.f, 0.f, inertiaFixed->mt.z,
          -inertiaFixed->mt.y);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, inertiaFixed->m, 0.f, -inertiaFixed->mt.z, 0.f,
          inertiaFixed->mt.x);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, 0.f, inertiaFixed->m, inertiaFixed->mt.y, -inertiaFixed->mt.x,
          0.f);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          0.f, -inertiaFixed->mt.z, inertiaFixed->mt.y, inertiaFixed->Ixx,
          inertiaFixed->Ixy, inertiaFixed->Izx);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g],\n",
          inertiaFixed->mt.z, 0.f, -inertiaFixed->mt.x, inertiaFixed->Ixy,
          inertiaFixed->Iyy, inertiaFixed->Iyz);
  fprintf(f, "        ");
  fprintf(f, "[% 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g, % 16.12g]])\n",
          -inertiaFixed->mt.y, inertiaFixed->mt.x, 0.f, inertiaFixed->Izx,
          inertiaFixed->Iyz, inertiaFixed->Izz);

  fprintf(f, "    if not np.allclose(i%dFixed, iFixed):\n", index);
  fprintf(f, "        print(\"[%d] %s does not match\")\n", index, name);
  fprintf(f, "        print(i%dFixed)\n", index);
  fprintf(f, "        print(iFixed)\n");

  fprintf(f, "    if i%d.any() and not np.all(np.linalg.eigvals(i%d) > 0):\n",
          index, index);
  fprintf(f, "        print(\"[%d] %s not positive definite\")\n", index, name);
  fprintf(f, "        print(i%d)\n", index);

  fprintf(f,
          "    if i%dFixed.any() and not np.all(np.linalg.eigvals(i%d) > 0):\n",
          index, index);
  fprintf(f,
          "        print(\"[%d] %s transformed to fixed frame is not positive "
          "definite\")\n",
          index, name);
  fprintf(f, "        print(i%dFixed)\n", index);
}

/*!
@brief evaluate generalized mass matrix

*/
void s2_context_setMassMatrix(s2_context_t* context) {

  memset(context->mhProduct, 0, context->dqCount * sizeof(s2_screw_t));
  memset(context->Ml, 0, context->Ml_size * sizeof(double));
  memset(context->Mlb, 0, context->Ml_size * sizeof(bool));
  s2_inertia_t* inertiaP = context->inertiaP + 1;

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "inertia:\n");
  //
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, " body [%d]: ", i);
  //   s2_stackstr_print(
  //     debug,
  //     context->bodyPointers[i]->name);
  //
  //   fprintf(debug, " transformation (Ad_01):\n");
  //   s2_displacement_print(debug, context->dReferenceFrames[i]);
  //
  //   fprintf(debug, " inertia transformed to fixed frame:\n");
  //   s2_inertia_print(debug, inertiaP + i);
  //
  //   fprintf(debug, " inertia in reference frame:\n");
  //   s2_inertia_print(debug, context->inertia + i);
  //
  // }

  /* create a python script to check for positive definite on all matrices */
  // FILE* outFile = fopen("inertia.py", "w");
  // fprintf(outFile, "#!/usr/bin/env python\n\n");
  // fprintf(outFile, "import numpy as np\n");
  // fprintf(outFile, "if __name__ == \"__main__\":\n");

  // fprintf(outFile, "    print(\"inertia in original body reference
  // frame:\")\n"); for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(outFile, "    # body [%d]: ", i);
  //   s2_stackstr_print(
  //     outFile,
  //     context->bodyPointers[i]->name);
  //   s2_inertia_t* bodyInertia = context->bodyPointers[i]->inertia;
  //   const char* name = s2_stackstr_getRoot(context->bodyPointers[i]->name);
  //   s2_inertia_print2Original(outFile, context->inertia + i, i, name);
  // }

  // fprintf(outFile, "    print(\"inertia in new reference frame:\")\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(outFile, "    # body [%d]: ", i);
  //   s2_stackstr_print(
  //     outFile,
  //     context->bodyPointers[i]->name);
  //   const char* name = s2_stackstr_getRoot(context->bodyPointers[i]->name);
  //   s2_inertia_print2Reference(
  //     outFile,
  //     context->inertia + i,
  //     inertiaP + i,
  //     i,
  //     name,
  //     context->bodyPointers[i]->referenceFrame->dAbsolute);
  // }
  //
  // fprintf(outFile, "    print(\"inertia in fixed frame:\")\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(outFile, "    # body [%d]: ", i);
  //   s2_stackstr_print(
  //     outFile,
  //     context->bodyPointers[i]->name);
  //   const char* name = s2_stackstr_getRoot(context->bodyPointers[i]->name);
  //   s2_inertia_print2(outFile, context->inertia + i, i, name);
  // }

  /*
    compute ``accumulated'' inertia matrices, project child inertia onto parent
  */
  for(int j = context->nonRigidJointCount - 1; j > -1; j--) {
    int i = context->pclCompressed[j];
    cblas_daxpy(10, 1.0, (double*)(inertiaP + j), 1, (double*)(inertiaP + i),
                1);
  }

// for(int i = context->nonRigidJointCount - 1; i > -1; i--) {
//   const char* name = s2_stackstr_getRoot(context->bodyPointers[i]->name);
//   s2_inertia_print2(outFile, inertiaP + i, i, name);
// }
#if _SHIFT_VERBOSE
  FILE* f = fopen("massmatrix.csv", "w");
  fprintf(f, "inertia:\nindex,m,mt.x,mt.y,mt.z,Ixx,Iyy,Izz,Ixy,Iyz,Izx,\n");
  for(int i = 0; i < context->nonRigidJointCount; i++) {
    fprintf(f, "%d,", i);
    s2_inertia_print(f, context->inertia + i);
  }

  fprintf(f, "inertiaP:\nindex,m,mt.x,mt.y,mt.z,Ixx,Iyy,Izz,Ixy,Iyz,Izx,\n");
  for(int i = 0; i < context->nonRigidJointCount; i++) {
    // s2_stackstr_print(debug, context->bodyPointers[context->pv_13[i]]->name);
    fprintf(f, "%d,", i);
    s2_inertia_print(f, inertiaP + i);
  }
#endif
  /* compute mh product */

#if _SHIFT_VERBOSE
  printf("processing dqIndex in mhProduct: ");
#endif
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(int j = 0; j < context->nonRigidJointCount; j++) {
    for(int dof = 0; dof < context->jointDOFCompressed[j]; dof++) {
      int k = context->dqLookupCompressed[j] + dof;

      /* update quantities that are processed for both rotation and
      translation */
      context->mhProduct[k].m0 += inertiaP[j].m * context->h[k].m0;
      context->mhProduct[k].m1 += inertiaP[j].m * context->h[k].m1;
      context->mhProduct[k].m2 += inertiaP[j].m * context->h[k].m2;

      /* this cblas call could replace the three += calls above */
      // cblas_daxpy(3, inertiaP[j].m, (double*)(context->h + j), 1,
      //             (double*)(context->mhProduct + j), 1);

      context->mhProduct[k].m4 += inertiaP[j].mt.z * context->h[k].m0;
      context->mhProduct[k].m5 -= inertiaP[j].mt.y * context->h[k].m0;

      context->mhProduct[k].m3 -= inertiaP[j].mt.z * context->h[k].m1;
      context->mhProduct[k].m5 += inertiaP[j].mt.x * context->h[k].m1;

      context->mhProduct[k].m3 += inertiaP[j].mt.y * context->h[k].m2;
      context->mhProduct[k].m4 -= inertiaP[j].mt.x * context->h[k].m2;

      /* do stuff just for rotation */
      if(context->isRotation[k]) {
        context->mhProduct[k].m1 -= inertiaP[j].mt.z * context->h[k].m3;
        context->mhProduct[k].m2 += inertiaP[j].mt.y * context->h[k].m3;
        context->mhProduct[k].m3 += inertiaP[j].Ixx * context->h[k].m3;
        context->mhProduct[k].m4 += inertiaP[j].Ixy * context->h[k].m3;
        context->mhProduct[k].m5 += inertiaP[j].Izx * context->h[k].m3;

        context->mhProduct[k].m0 += inertiaP[j].mt.z * context->h[k].m4;
        context->mhProduct[k].m2 -= inertiaP[j].mt.x * context->h[k].m4;
        context->mhProduct[k].m3 += inertiaP[j].Ixy * context->h[k].m4;
        context->mhProduct[k].m4 += inertiaP[j].Iyy * context->h[k].m4;
        context->mhProduct[k].m5 += inertiaP[j].Iyz * context->h[k].m4;

        context->mhProduct[k].m0 -= inertiaP[j].mt.y * context->h[k].m5;
        context->mhProduct[k].m1 += inertiaP[j].mt.x * context->h[k].m5;
        context->mhProduct[k].m3 += inertiaP[j].Izx * context->h[k].m5;
        context->mhProduct[k].m4 += inertiaP[j].Iyz * context->h[k].m5;
        context->mhProduct[k].m5 += inertiaP[j].Izz * context->h[k].m5;
      }
    }
  }

#if _SHIFT_VERBOSE
  printf("\n");
  fprintf(f, "index,hasMass,");
  for(int i = 0; i < 6; i++) {
    fprintf(f, "mhProduct.m%d,", i);
  }
  for(int i = 0; i < 6; i++) {
    fprintf(f, "h.m%d,", i);
  }
  fprintf(f, "\n");
  for(int i = 0; i < context->dqCount; i++) {
    fprintf(f, "%d,%s,", i, context->hasMassdq[i] ? "true" : "false");
    s2_screw_print(f, context->mhProduct + i);
    s2_screw_print(f, context->h + i);
    fprintf(f, "\n");
  }
#endif

  /* use inverse permutation to save terms computed in PO1 order and
  store in PO3 */
  int m_3;
  int ks_3;
  int k_3;
  for(int j = context->dqCount - 1; j > -1; j--) {
    if(context->hasMassdq[j]) {
      m_3 = context->pv_31[j];
      // m_1 = j;
      // ks_1 = (m_1 * (m_1 + 1)) / 2;
      ks_3 = (m_3 * (m_3 + 1)) / 2;

      // k_1 = ks_1 + m_1;
      k_3 = ks_3 + m_3;
      context->Mlb[k_3] = true;

      if(context->isRotation[j]) {
        context->Ml[k_3] += s2_dot_ss(context->mhProduct + j, context->h + j);
      } else {
        context->Ml[k_3] += context->mhProduct[j].m0 * context->h[j].m0;
        context->Ml[k_3] += context->mhProduct[j].m1 * context->h[j].m1;
        context->Ml[k_3] += context->mhProduct[j].m2 * context->h[j].m2;
        /* the cblas call could replace the above function calls */
        // context->Ml[k_3] += cblas_ddot(3, (double*)(context->mhProduct + j),
        // 1,
        //                                (double*)(context->h + j), 1);
      }

      int i = context->pcldq[j];
      while(i > -1) {
        int i_3 = context->pv_31[i];
        if(m_3 > i_3) {
          k_3 = ks_3 + i_3;
        } else {
          k_3 = (i_3 * (i_3 + 1)) / 2 + context->pv_31[j];
        }
        context->Mlb[k_3] = true;

        if(context->isRotation[i]) {
          context->Ml[k_3] += s2_dot_ss(context->mhProduct + j, context->h + j);
        } else {

          context->Ml[k_3] += context->mhProduct[j].m0 * context->h[j].m0;
          context->Ml[k_3] += context->mhProduct[j].m1 * context->h[j].m1;
          context->Ml[k_3] += context->mhProduct[j].m2 * context->h[j].m2;

          // context->Ml[k_3] += cblas_ddot(3, (double*)(context->mhProduct +
          // j),
          //                                1, (double*)(context->h + i), 1);
        }
        i = context->pcldq[i];
      }
    }
  }

#if _SHIFT_VERBOSE
  printf("mass matrix boolean before factorization Mlb:\n");
  s2_triangb_print(f, context->dqCount, context->Mlb);
  printf("\n");

  printf("\ngeneralized mass matrix before factorization:\n");
  s2_triang_print(f, context->dqCount, context->Ml);
  fclose(f);
  exit(0);
#endif
}

/*!
@brief convert angular velocities to time derivative of quaternions
copy by value other terms from dq to dY array
*/
void s2_context_dqTody(s2_context_t* context, double* dy) {

  /* convert angular velocities to time derivative of quaternions */
  for(int i = 0; i < context->quaternionCount; i++) {
    s2_omega_to_dquaternion(
      (s2_vector_t*)(context->dq + context->dqQuaternionIndices[i]),
      (s2_quaternion_t*)(context->q + context->qQuaternionIndices[i]),
      (s2_quaternion_t*)(dy + context->qQuaternionIndices[i]));
  }

  /* copy primitive variables */
  for(int i = 0; i < context->nonQuaternionCount; i++) {
    dy[context->qNonQuaternionIndices[i]] =
      context->dq[context->dqNonQuaternionIndices[i]];
  }
}

/*!
  @brief evaluates Jacobians in PO2, factors Jacobian in PO2, sets mass matrix
  and RHS in PO3.
*/
void s2_context_setEquations(s2_context_t* context) {

  /* save a copy of the Jacobian to Js for comparison */
  memcpy(context->Js, context->J, context->jacobianSize * sizeof(double));

  s2_context_setJacobianPO2(context);

  // FILE* debug = context->modelParameters->debugFilePointer;
  // printf("\nJacobian after setPO2:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   printf("\nsubstructure [%d]:\n", i);
  //   s2_array_print2D(
  //     debug,
  //     substructure->constraintEquationCount,  substructure->dqCount,
  //     true,
  //     substructure->J);
  // }
  s2_context_factorJacobianPO2(context);

  // printf("\nJacobian after factorPO2:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   printf("\nsubstructure [%d]:\n", i);
  //   s2_array_print2D(
  //     substructure->constraintEquationCount,  substructure->dqCount,
  //     true,
  //     substructure->J);
  //   printf("infinity norm: [%f]\n",
  //   substructure->scaledInitialInfinityNormB);
  //
  //   for(int j = 0; j < substructure->constraintEquationCount; j++) {
  //     for(int k = 0; k < substructure->dqCount; k++) {
  //       if(substructure->Jb[j * substructure->dqCount + k]) {
  //         printf("1 ");
  //       } else {
  //         printf("0 ");
  //       }
  //     }
  //     printf("\n");
  //   }
  //   printf("\n");
  // }

  s2_context_setBMatrixSparse(context);

  // if(s2_all_close(
  //   context->jacobianSize,
  //   context->Js,
  //   context->J)) {
  //
  //   fprintf(debug, "Jacobian close to previous evaluation:\n");
  // for(
  //   int i = 0; i < context->substructureCount; i++) {
  //
  //   s2_substructure_t* substructure =
  //     context->substructures + i;
  //
  //   fprintf(debug, "\nfactored Jacobian for substructure (including b matrix)
  //   [%d]\n", i); s2_array_print2D(
  //     debug,
  //     substructure->constraintEquationCount,
  //     substructure->dqCount,
  //     true,
  //     substructure->J);
  // }
  // }

  // printf("infinity norm b check in setEquations\n");
  int returnVal = s2_context_infinityNormBCheck(context);
  if(returnVal) {
    switch(context->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        exit(-1);
      } break;
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
        numericalIntegrator->newtonRaphsonFailure = true;
        return;
      } break;
    }
  }
  s2_context_setRHS(context);
  s2_context_setMassMatrix(context);
}

/*!
@brief factor and solve a system of equations using source and destination
pointer arrays set up in memoize.c

This algorithm is intended to be called after s2_equation_memoize()
*/
void s2_context_factorAndSolve(s2_context_t* context) {

  // printf("ddq on entering s2_equation_factorAndSolve:\n");
  // s2_array_print(context->dqCount, context->ddq);

  /* Access entries in context->P[] to factor matrix and access L */
  int p2 = context->dqCount;

  /* Access entries in context->P[] to process rhs and lhs */
  int p3 = context->p3Start;

  int ni = 0;
  int nd = 0;
  // int k = context->dependentCount - 1;

  /*
    Factor the matrix
  */
  for(int i = context->dqCount - 1; i > -1; i--) {
    if(context->isDependentPO3[i]) {
      for(int j = 0; j < context->n1[nd]; j++) {
        /* M[] -= M[] * B[]; */
        *(context->P[p2]) -= *(context->P[p2 + 1]) * *(context->P[p2 + 2]);
        p2 += 3;
      }
      nd++;
    } else {
      /* Invert the pivot: M[i] = 1.0 / M[i]; */
      if(*(context->P[i]) > 1e-6) {
        *(context->P[i]) = 1.0 / *(context->P[i]);
      } else {
        fprintf(stderr, "error: pivot at index [%d] has the value %0.9g\n", i,
                *(context->P[i]));
        exit(-1);
      }
      /* Copy the nonzero entries in row i of the M[]
         array to the left of the diagonal into the work array and scale the
         entries in row i to L terms.
      */
      for(int j = context->startOfWorkInM; j < context->n2[ni]; j++) {
        // Copy entry into work array: M[] = M[];
        *(context->P[p2]) = *(context->P[p2 + 1]);
        p2++;
        // Scale entry to evaluate L term: M[] *= M[];
        *(context->P[p2]) *= *(context->P[i]);
        p2++;
      }
      // Perform mass projections with one assignment statement.
      for(int j = 0; j < context->n3[ni]; j++) {
        /* Update Schur-complement matrix: M[] -= M[] * M[]; */
        *(context->P[p2]) -= *(context->P[p2 + 1]) * *(context->P[p2 + 2]);
        p2 += 3;
      }
      ni++;
    }
  }
  nd = 0;
  ni = 0;

  /* Backward elimination */
  for(int i = context->dqCount - 1; i > -1; i--) {
    if(context->isDependentPO3[i]) {
      for(int j = 0; j < context->p1[nd]; j++) {
        /*
          Part of b <-- L^{-T} b: Q[] -= B[] * Q[i];
        */
        *(context->P[p3++]) -= *(context->P[p2++]) * context->fl[i];
      }
      // Project constraint-induced inertia forces onto parent forces.
      // Does not execute if Mii = 0.
      for(int j = 0; j < context->p2[nd]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * ga[k];
        *(context->P[p3++]) -=
          *(context->P[p2++]) * context->gamma_d[context->pv_23[i]];
      }
      nd++;

      /* Part of x <-- D^{-1} b: */
      context->ddq[context->pv_13[i]] = context->gamma_d[context->pv_23[i]];
      context->la[context->pv_23[i]] = context->fl[i];

      if(context->Mlb[context->i4[i]])
        /* Part of x <-- D^{-1} b: la[k] -= M[] * ddq[i]; */
        context->la[context->pv_23[i]] -=
          *(context->P[i]) * context->ddq[context->pv_13[i]];
      // k--;
    } else {
      for(int j = 0; j < context->p3[ni]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * Q[i];
        *(context->P[p3++]) -= *(context->P[p2++]) * context->fl[i];
      }
      // Part of x <-- D^{-1} b: ddq[i] = M[] * Q[i];
      context->ddq[context->pv_13[i]] = *(context->P[i]) * context->fl[i];
      ni++;
    }
  }

  /* Reset for indexing forward through context->la[k] */
  // k = 0;

  /* P2 is one past last entry in L. Decrement for reverse direction. */
  p2--;

  /*
    Forward substitution
  */
  for(int i = 0; i < context->dqCount; i++) {
    if(context->isDependentPO3[i]) {
      nd--;
      for(int j = 0; j < context->p2[nd]; j++) {
        // Part of x <-- L^{-1} x: la[k] -= M[] * ddq[];
        context->la[context->pv_23[i]] -=
          *(context->P[p2--]) * *(context->P[p3++]);
      }
      for(int j = 0; j < context->p1[nd]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= B[] * ddq[];
        context->ddq[context->pv_13[i]] -=
          *(context->P[p2--]) * *(context->P[p3++]);
      }
      // k++;
    } else {
      ni--;
      for(int j = 0; j < context->p3[ni]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= M[] * ddq[];
        context->ddq[context->pv_13[i]] -=
          *(context->P[p2--]) * *(context->P[p3++]);
      }
    }
  }
  // printf("ddq on exit s2_equation_factorAndSolve:\n");
  // s2_array_print(context->dqCount, context->ddq);
}
