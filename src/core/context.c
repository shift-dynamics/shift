/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Context datastructure.
@date 2016
@copyright Kristopher Wehage 2016

This function library is used to initialize and free memory for
data structures defined in context.h

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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#ifdef __APPLE__
  #include <Accelerate/Accelerate.h>
#else
  #include <cblas.h>
#endif
#include "context.h"
#include "utility.h"
#include "kinematics.h"
#include "body.h"
#include "joint.h"
#include "frame.h"
#include "assembly.h"
#include "light.h"
#include "io.h"
#include "substructure.h"
#include "assembly.h"
#include "../force_modules/tire.h"
#include "../ode/numericalintegrator.h"


/*!
@brief Initialize a context datastructure

@param None

@returns a context datastructure
*/
s2_context_t* s2_context() {

  assert(sizeof(s2_screw_t) == 6 * sizeof(double));
  assert(sizeof(s2_mat3x3_t) == 9 * sizeof(double));
  assert(sizeof(s2_displacement_t) == 12 * sizeof(double));
  assert(sizeof(s2_vector_t) == 3 * sizeof(double));

  s2_context_t* context = malloc(sizeof *context);

  if (!context) {
    fprintf(stderr, "Error: failed to initialize Context.\n");
    return NULL;
  }
  context->assembly = s2_assembly();
  context->modelParameters = s2_modelparameters();

  /*!
  @name Counts of objects in Context data structure
  @{
  */
  context->epsRankDetermination = 1e-9; /*!< epsilon, tolerance used for rank determination
                            in LU factorization */
  context->epsNewtonRaphsonRankDetermination = 1e-12;
  context->epsNewtonRaphson = 1e-6;
  context->jointCount = 0; //!< total number of joints
  context->bodyCount = 0; //!< total number of bodies
  context->frameCount = 0; //!< total number of frames
  context->lightCount = 0; //!< total number of lights
  context->ioCount = 0; //!< total number of input/output devices
  context->restraintCount = 0; //<! total number of restraints
  context->nonRigidJointCount = 0; /*!< total number of non-rigid joints, equal
                                        to dof */
  context->dhvaCount = 0;
  context->activeFrameCount = 0; /*!< number of active frames in mechanism */
  context->activeFrameInTreeCount = 0; /*!< number of active frames in spanning tree */
  context->activeJointCount = 0; /*!< number of active joints */
  context->activeBodyCount = 0; /*!< number of active bodies */
  context->activeLightCount = 0; /*!< number of active lights */
  context->activeIOCount = 0; /*!< number of active devices */
  context->activeRestraintCount = 0; //!< number of active restraints
  context->qCount = 0; /*!< dof for generalized positions */
  context->dqCount = 0; /*!< dof for generalized velocities */
  context->dcCount = 0; /*!< count of direction cosines */

  context->chordConstraintCount = 0; /*!< number of chord constraints, one for
                                 each kinematic loop */
  context->dqLookup = NULL;
    /*!< lookup table used to look up a joint's position in variable
         list. float variables have 6 variables, spherical joints have
         3 variables, and rigid joints
         have 0 variables associated with them */
  context->qLookup = NULL;
  context->jointLookup = NULL;
    /*!< lookup table used to find a joint's position in compressed joint list
         rigid joints are removed from compressed list and have a value of -1
         in lookup table */
  context->dqLookupCompressed = NULL;

  /*!
  @}
  @name Data related to processing kinematic loops within mechanism
  @{
  */
  context->chordConstraints = NULL; /*!< specified chord constraints, if
                                         multiple loops are present in a
                                         mechanism */
  context->pivotRatioMultiple = 4;
  context->J = NULL; /*!< Jacobian matrix (or matrices) */
  context->Js = NULL;
  context->Jxx = NULL; /*!< instructions for evaluating Jacobian matrices */
  context->Jb = NULL; /*!< Boolean Jacobian matrix */
  context->jacobianSize = 0; /*!< total size of jacobian matrix */

  context->isDependentPO1 = NULL; /*!< equal to dof, in PO1 */
  context->isDependentPO3 = NULL; /*! equal to dof, in PO3 */
  context->hasMass = NULL; /*!< equal to active jointCount */
  context->hasMassdq = NULL; /*!< indicates whether body associated with
                                  joint variable has mass */
  context->jointDOF = NULL;

  context->substructures = NULL;
  context->substructureCount = 0; /*!< Number of kinematic substructures */
  context->substructureLoopCount = NULL; /*!< Number of loops in each
                                   kinematic substructure */
  context->substructureMembers = NULL; /*!< members of each kinematic substructure */
  context->substructureMembersCount = NULL; /*!< count of members in each substructure */

  context->positiveLoopMembers = NULL; /*!< Indicates mechanism's positive loop members */
  context->positiveLoopMembersCount = NULL; /*!< Count of positive members in each loop */
  context->positiveLoopMembersCompressedCount = NULL; /*!< containts a compressed list
                                                of loop members with rigid
                                                joints removed */
  context->positiveLoopMemberOffsets = NULL; /*!< Indicates mechanism's local positive loop
                                        member offsets */

  context->negativeLoopMembers = NULL; /*!< Indicates mechanism's negative
                                            loop members */
  context->negativeLoopMembersCompressedCount = NULL; /*!< containts a compressed list
                                                of loop members with rigid
                                                joints removed */
  context->negativeLoopMembersCount = NULL; /*!< Count of negative members in each loop */
  context->negativeLoopMemberOffsets = NULL; /*!< Indicates mechanism's negative loop
                                        member offsets */

  context->pv_12 = NULL; /*!< global compressed variable permutation array
                              going from PO2 to PO1 */
  context->pv_21 = NULL; /*!< global compressed variable permutation array
                              going from PO1 to PO2 */
  context->pv_13 = NULL; /*!< global compressed variable permutation array
                              going from PO3 to PO1 */
  context->pv_31 = NULL; /*!< global compressed variable permutation array
                              going from PO1 to PO3 */
  context->pv_32 = NULL; /*!< global compressed variable permutation array
                              going from PO2 to PO3 */
  context->pv_23 = NULL; /*!< global compressed variable permutation array
                              going from PO3 to PO2 */

  /*!
  @}
  @name Pointers to contiguous arrays and pointer arrays of bodies, joints,
  frames and other quantities used at runtime for processing kinematics
  @{
  */
  context->bodyPointers = NULL;
    /*!< Array of pointers to bodies to directly access
         body in position in assembly hierarchy */

  context->nonVirtualBodyPointers = NULL;
    /*!< Array of pointers to bodies to directly access
         body in position in assembly hierarchy, does not include
         virtual bodies */

  context->jointPointers = NULL; /*!< Array of pointers to joints to directly
                                     access body in position in assembly
                                     hierarchy */
  context->framePointers = NULL; /*!< array of pointers to frames for
                                     direct access */

  context->lightPointers = NULL; //!< array of pointers to lights

  context->dhFunctionPointers = NULL;
    /*!< function pointers to update displacements
         and influence coefficient matrices in
         spanning tree */

  context->dhvaFunctionPointers = NULL;
    /*!< function pointers to update displacements
        influence coefficient matrices,
        velocities and accelerations in spanning tree */

  context->forceModuleCount = 0;
  context->forceModulePointers = NULL;

  context->names = NULL;

  context->jointTypes = NULL; /*!< Array of joint types, equal in length to joint
                              list */
  context->pcl = NULL; /*!< Parent Child List, used to navigate mechanism
                             topology, set automatically by
                             s2_ProcessTopology */
  context->pclCompressed = NULL;
  context->pcldq = NULL;
  context->isRotation = NULL;

  context->levels = NULL; /*!< Levels list, set by s2_ProcessTopology */
  context->isReference = NULL; /*!< an array of booleans that indicate whether the
                          displacement at the current index of context->d is
                          a reference Frame */
  context->originInTree = NULL; /*< an array of booleans equal to
                          number of bodies, true if body origin is in
                          spanning tree */

  context->originPointers = NULL;
  context->referenceFramePointers = NULL;

  context->d = NULL; /*!< array of displacements equal
                               in size to number of frames */
  context->dWRTReference = NULL; /*!< array of relative displacements,
                                           equal in size to number of shape
                                           matrices + rigid joints*/
  context->dReferenceFrames = NULL;

  context->dc = NULL; //!< direction cosine arrays
  context->dcq = NULL; /*!< generalized coordinates that have direction cosine terms
                      associated with them */
  context->h = NULL; /*!< an array of influence coefficient matrices equal to the
                    6 * number of variables in the datastructure */
  context->v = NULL; /*!< an array of body velocities equal to the
                    number of variables in the datastructure */
  context->aq = NULL; /*!< an array of body accelerations equal to the
                    number of variables in the datastructure */
  context->vRelative = NULL; /*!< an array of body velocities equal to the
                    number of variables in the datastructure */
  context->aqRelative = NULL; /*!< an array of body accelerations equal to the
                    number of variables in the datastructure */
  context->dhva = NULL; /*!< pointers to quantities needed by dhFunctionPointers
                         and dhvaFunctionPointers, set by
                         topology_ProcessTopology */

  context->q = NULL; //!< primitive joint displacements
  context->qCopy = NULL;
  context->dq = NULL; //!< primitive joint velocities
  context->ddq = NULL; //!< primitive joint accelerations

  // context->gamma_p = NULL; /*!< primary loop constraints */
  context->gamma_d = NULL; /*!< preconditioned constraints */
  context->cs = NULL; /*!< constraints */
  context->dcs = NULL; /*!< constraint first derivative */
  context->ddcs = NULL; /*!< constraint second derivative */
  context->ddcsq = NULL; /*!< constraint second derivative and quadratic terms */

  context->Ml = NULL; /*!< Generalized mass matrix, linear array containing mass
                     matrix terms on and below the diagonal */
  context->Mls = NULL; /*!< Saved copy of original M */
  context->Ml_size = 0;

  context->p = NULL; /*!< momentum */
  context->g = NULL; /*!< body and inertia forces */
  context->k = NULL; /*!< joint reaction forces */
  context->mhProduct = NULL; /*!< initialize mH product */

  context->nnzm = 0; /*!< number of nonzero entries in mass matrix */
  context->nnzms = 0; /*!< original number of nonzero entries in mass matrix */
  context->nnzb = 0; /*!< number of nonzero entries in B matrix */

  context->pcl = NULL; /*!< parent-child list in PO(1) */

  context->Mlb = NULL; /*!< boolean mass matrix, equal in size to M */
  context->Mlbs = NULL; /*!< original boolean mass matrix, with no fills recorded */

  context->rm = NULL; /*!< row ranges of M matrix in PO(3) */
  context->cm = NULL; /*!< row ranges of M matrix in PO(3) */

  context->rb2 = NULL; /*!< row ranges of B matrix in PO(2) */
  context->rb3_temp = NULL; /*!< temporary array used to convert rb2 to rb3 */
  context->rb3 = NULL; /*!< row ranges of B matrix in PO(3) */

  context->cb2 = NULL; /*!< global col indices of B matrix in PO(2) */
  context->cb3_temp = NULL; /*!< temporary array used to convert cb2 to cb3*/
  context->cb3 = NULL; /*!< global col indices of B matrix in PO(3) */
  context->colb = NULL; /*!< local col indices of B matrix in PO(3) */
  context->ncb = 0; /*!< number of columns in B matrix */
  context->pb_23 = NULL; /*!< permutation of linear b matrix from 3 to 2 */
  context->pb_32 = NULL; /*!< permutation of linear b matrix from 2 to 3 */

  context->ijMapping = NULL;
  context->b3 = NULL; /*!< Linear array containing terms of preconditioned constraints
                     in PO(3) */
  context->f = NULL; /*!< working forces applied in a joint, equal in length to
                          total number of degrees of freedom */
  context->fl = NULL; /*!< Generalized forces */
  context->la = NULL; /*!< lagrange multipliers */
  context->constraints = NULL;

  /*!
  @}
  @name index and pointer arrays used to "memoize" solution of dynamics
  equations
  @{
  */
  context->i1s = 0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  context->i2s = 0; /*!< Points to Mij in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  context->i6s = 0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Mik or Mjk -= Mij*Mik */
  context->i8s = 0; /*!< Points to Mik in Mkj -= Mij*Mik or Mjk -= Mij*Mik */

  /*!< M[i1[l1]] -= M[i2[l1]] * B[i3[l1]]; */
  context->i1 = NULL;
  context->i2 = NULL;
  context->i3 = NULL;

  /*!< M[i4[i]] = 1.0/M[i4[i]];
       M[p] = M[i5[l2]];
       M[i5[l2]] *= M[i4[i]]; */
  context->i4 = NULL;
  context->i5 = NULL;

  /*!< M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */
  context->i6 = NULL;
  context->i7 = NULL;
  context->i8 = NULL;

  /*!< Q[j1[m1]] -= B[j2[m1]]*Q[i];
       ddq[i] -= B[j2[m1]]*ddq[j1[m1]]; */
  context->j1 = NULL;
  context->j2 = NULL;

  /*!< Q[j3[m2]] -= M[j4[m2]]*ga[i];
       la[i] -= M[j4[m2]]*ddq[j3[m2]]; */
  context->j3 = NULL;
  context->j4 = NULL;

  /*!< Q[j5[m3]] -= M[j6[m3]]*Q[i];
       ddq[i] -= M[j6[m3]]*ddq[j5[m3]]; */
  context->j5 = NULL;
  context->j6 = NULL;

  context->p1 = NULL; /*!< increments in l1 to process current dependent variable. */
  context->p2 = NULL; /*!< increments in m1 to process current dependent variable. */
  context->p3 = NULL; /*!< increments in m2 to process current dependent variable. */
  context->n1 = NULL; /*!< increments in l2 to process current independent variable. */
  context->n2 = NULL; /*!< increments in l3 to process current independent variable. */
  context->n3 = NULL; /*!< increments in m3 to process current independent variable. */

  /*! Single array of source and destination memory pointers.
  The dimension of this array is hard coded for initial testing.
  Eventually this decleration will be changed to double **P, and
  sufficient memory for all source and destination memory pointers
  will be malloced. This memory cannot be malloced until the size
  of P has been determined. */
  context->startOfWorkInM = 0;
  context->P = NULL;
  context->p3Start = 0;
  context->rightNullSpace = NULL;
  context->rightNullSpaceCount = 0;
  context->mobilityNumbers = NULL;
  context->mobility = 0;

  /*!
  @}
  @name properties related to input/output datastructures
  @{
  */
  context->io = NULL; /*!< pointer to input/output datastructures */

  /*!
  @}
  @name struct used for numerical integration
  @{
  */

  context->qQuaternionIndices = NULL;
  context->dqQuaternionIndices = NULL;
  context->qNonQuaternionIndices = NULL;
  context->dqNonQuaternionIndices = NULL;
  context->numericalIntegrator = NULL; /*!< pointer to properties
                                            related to numerical
                                            integration */


  /*!
  @}
  @name quantities related to processing inertia
  @{
  */
  context->inertia = NULL;
  context->inertiaP = NULL;

  /*!
  @}
  @name quantities related to processing graphics
  @{
  */
  context->glInitialized = false;
  context->viewport = NULL;
  context->glReferenceFramesf = NULL;
  context->glReferenceFrames = NULL;
  context->shaderHandleLight = NULL;

  context->programLine = -1;
  context->programFlat = -1;
  context->programGouraud = -1;
  context->programPhong = -1;

  context->lightCountUniform = 0;

  /*!
  @}
  @name quantities related to processing graphics
  @{
  */

  context->stiffnessCount = 0;
  context->stiffnessJoints = NULL;

  context->dampingCount = 0;
  context->dampingJoints = NULL;
  /*!
  @}
  */
  context->iterationsSinceLastConstraintError = 0;

  context->terrainCount = 0;
  context->terrains = NULL;

  return context;
}


/*!
@brief used to set omegaN for baumgarte constraint stabilization
*/
void s2_context_setOmegaN(
  s2_context_t* context,
  double omegaN) {

  context->modelParameters->baumgarteOmegaN = omegaN;
  context->modelParameters->baumgarte2OmegaN = 2 * omegaN;
  context->modelParameters->baumgarteOmegaN2 =
    omegaN * omegaN;
}


/*!
@brief Initialize a Units datastructure
@returns a Unit datastructure
*/
s2_units_t* s2_units() {
  s2_units_t* units = malloc(sizeof *units);
  if(!units) {
    fprintf(stderr, "Error: failed to initialize units.\n");
    return NULL;
  }
  units->angle = NULL;
  units->length = NULL;
  units->mass = NULL;
  units->time = NULL;
  return units;
}


/*!
@brief Free a Units datastructure
@param units a Units datastructure to free
*/
void s2_units_free(
  s2_units_t* units) {

  if(units) {
    if(units->angle) {
      free(units->angle);
      units->angle = NULL;
    }
    if(units->length) {
      free(units->length);
      units->length = NULL;
    }
    if(units->mass) {
      free(units->mass);
      units->mass = NULL;
    }
    if(units->time) {
      free(units->time);
      units->time = NULL;
    }
    free(units);
    units = NULL;
  }
}


/*!
@brief Initialize a ModelParameters datastructure
@returns a ModelParameters datastructure
*/
s2_modelparameters_t* s2_modelparameters() {
  s2_modelparameters_t* modelParameters = malloc(sizeof *modelParameters);

  if (!modelParameters) {
    fprintf(stderr, "Error: failed to initialize modelParameters.\n");
    return NULL;
  }
  modelParameters->version = NULL;
  modelParameters->includeDirectory = NULL;

  modelParameters->gravity = s2_vector();
  modelParameters->units = s2_units();

  modelParameters->outFile = NULL;
  modelParameters->outFilePointer = NULL;
  modelParameters->debugFile = NULL;
  modelParameters->debugFilePointer = stdout;

  /*! @TODO remove this later once dhva verified */
  modelParameters->dhvaFilePointer =
    fopen("dhva.dat", "w");
  if(!modelParameters->dhvaFilePointer) {
    fprintf(stderr, "could not open dhva debug file\n");
    modelParameters->dhvaFilePointer = NULL;
  }

  asprintf(
    &(modelParameters->delimiter),
    ","
  );
  modelParameters->monitorIterations = true;
  modelParameters->monitorConstraintError = true;


  modelParameters->sumConstraintError = 0;
  modelParameters->maxConstraintError = 0;
  modelParameters->maxConstraintErrorStep = 0;
  modelParameters->sumIterationCount = 0;
  modelParameters->maxIterationCount = 0;
  modelParameters->maxIterationCountStep = 0;

  return modelParameters;
}


/*!
@brief Free memory for ModelParameters datastructure
@param modelParameters a ModelParameters datastructure
*/
void s2_modelparameters_free(
  s2_modelparameters_t* modelParameters) {

  if(modelParameters) {
    if(modelParameters->dhvaFilePointer) {
      fclose(modelParameters->dhvaFilePointer);
    }
    if(
      modelParameters->debugFilePointer &&
      modelParameters->debugFilePointer != stdout) {

      fclose(modelParameters->debugFilePointer);
    }

    if(modelParameters->version) {
      free(modelParameters->version);
      modelParameters->version = NULL;
    }
    s2_stackstr_free(modelParameters->includeDirectory);

    if(modelParameters->gravity) {
      free(modelParameters->gravity);
      modelParameters->gravity = NULL;
    }

    s2_units_free(modelParameters->units);

    free(modelParameters);
    modelParameters = NULL;
  }
}


/*!
@brief Print a Context datastructure
@param context a Context datastructure
*/
void s2_context_print(
  FILE* f,
  s2_context_t* context) {

  fprintf(f, "\nModel Parameters:\n");
  fprintf(f, "\ts2_version: %s\n", context->modelParameters->version);
  fprintf(f, "\tgravity: ");
  s2_vector_print(f, context->modelParameters->gravity);
  fprintf(f, "\ttime: %s\n", context->modelParameters->units->time);
  fprintf(f, "\tlength: %s\n", context->modelParameters->units->length);
  fprintf(f, "\tmass: %s\n", context->modelParameters->units->mass);
  fprintf(f, "\tangle: %s\n", context->modelParameters->units->angle);
  fprintf(f, "\tinclude_dir: ");
  s2_stackstr_print(f, context->modelParameters->includeDirectory);
  s2_assembly_print(f, context->assembly);
  fprintf(f, "PC: ");
  for(int i = 0; i < context->activeJointCount; i++) {
    fprintf(f, "%d ", context->pcl[i]);
  }
  fprintf(f, "\n");
}


/*!
@brief Free influence coefficient matrices, copy values back to their original
Joint datastructures

@param context a Context datastructure
*/
void s2_context_freeH(
  s2_context_t* context) {

  if(context->h) {
    for(int i = 0; i < context->activeJointCount; i++) {
      s2_joint_t* joint = context->jointPointers[i];
      if(joint->dqIndex > -1) {
        s2_screw_copyBack(context->h + joint->dqIndex, &(joint->h));
      }
    }
    free(context->h);
    context->h = NULL;
  }
}


/*!
@brief Free Displacements, Velocities and Accelerations recursively

Not intended to be called directly, used internally by s2_FreeDVA function

@param context a Context datastructure
@param assembly an Assembly datastructure
*/
int s2_context_freeDVARecursive(
  s2_context_t* context,
  s2_assembly_t* assembly) {

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while (subAssemblyHead) {
    if(subAssemblyHead->active) {
      s2_body_t* bodyHead = subAssemblyHead->bodyHead;
      while(bodyHead) {
          s2_frame_t* frameHead = bodyHead->referenceFrame;
          /*
            velocity and acceleration are only stored for the
            reference frame (frameHead).
          */
          if(bodyHead->parentJointPointer->jointType != S2_JOINT_RIGID) {
            s2_screw_copyBack(
              context->v + frameHead->vaIndex,
              &(frameHead->velocityAbsolute));
            s2_screw_copyBack(
              context->aq + frameHead->vaIndex,
              &(frameHead->accelerationQuadraticAbsolute));
          } else {
            frameHead->velocityAbsolute = s2_screw();
            s2_screw_copy(
              context->v + frameHead->vaIndex,
              frameHead->velocityAbsolute);
            frameHead->accelerationQuadraticAbsolute = s2_screw();
            s2_screw_copy(
              context->aq + frameHead->vaIndex,
              frameHead->accelerationQuadraticAbsolute);
          }

          /*
            displacement is stored, computed for all frames
          */
          frameHead = bodyHead->frameHead;
          while(frameHead) {
            if(frameHead->dIndex > -1) {
              s2_displacement_copyBack(
                context->d + frameHead->dIndex,
                &(frameHead->dAbsolute));
            }
            frameHead = frameHead->next;
          }
        bodyHead = bodyHead->next;
      }
    }
    // s2_joint_t* jointHead = subAssemblyHead->jointHead;
    // while(jointHead) {
    //   if(jointHead->h)
    //   jointHead = jointHead->next;
    // }

    subAssemblyHead = subAssemblyHead->next;
  }
  return 0;
}


/*!
@brief Free Displacements, Velocities and Accelerations

@param context a Context datastructure
*/
void s2_context_freeDHVA(
  s2_context_t* context) {

  if(context->d) {
    s2_frame_t* frameHead = context->assembly->bodyHead->frameHead;
    s2_screw_copyBack(
      context->v,
      &(frameHead->velocityAbsolute));
    s2_screw_copyBack(
      context->aq,
      &(frameHead->accelerationQuadraticAbsolute));

    while(frameHead) {
      if(frameHead->dIndex > -1) {
        s2_displacement_copyBack(
          context->d + frameHead->dIndex,
          &(frameHead->dAbsolute));
        }
      frameHead->index = -1;
      frameHead->dIndex = -1;
      frameHead = frameHead->next;
    }
    s2_context_freeDVARecursive(context, context->assembly);

    free(context->d);
    context->d = NULL;
  }
  if(context->dWRTReference) {
    free(context->dWRTReference);
    context->dWRTReference = NULL;
  }
  if(context->v) {
    free(context->v);
    context->v = NULL;
  }
  if(context->aq) {
    free(context->aq);
    context->aq = NULL;
  }

  s2_context_freeH(context);
}


/*!
@brief Free direction cosine array and copy values back to their original
Joint datastructures, also copies q and dq back to original datastructures

@param context a Context datastructure
*/
void s2_context_freeDc(
  s2_context_t* context) {

  if(context->dc) {
    for(int i = 0; i < context->activeJointCount; i++) {
      s2_joint_t* joint = context->jointPointers[i];
      switch(joint->jointType) {
        case S2_JOINT_RX:
        case S2_JOINT_RY:
        case S2_JOINT_RZ: {
          joint->directionCosine = malloc(2 * sizeof(double));
          cblas_dcopy(
            2,
            context->dc + (2 * joint->dcIndex),
            1,
            joint->directionCosine,
            1);
        }
        case S2_JOINT_TX:
        case S2_JOINT_TY:
        case S2_JOINT_TZ: {
          joint->primitiveDisplacement = calloc(1, sizeof(double));
          joint->primitiveDisplacement[0] = context->q[joint->qIndex];
          joint->primitiveVelocity = calloc(1, sizeof(double));
          joint->primitiveVelocity[0] = context->dq[joint->dqIndex];
        } break;
        case S2_JOINT_RIGID: {

        } break;
        case S2_JOINT_SPHERICAL: {
          joint->primitiveDisplacement = calloc(4, sizeof(double));
          cblas_dcopy(
            4,
            context->dq + joint->qIndex,
            1,
            joint->primitiveVelocity,
            1);
          joint->primitiveVelocity = calloc(3, sizeof(double));
          cblas_dcopy(
            3,
            context->dq + joint->dqIndex,
            1,
            joint->primitiveVelocity,
            1);
        } break;
        case S2_JOINT_FLOAT: {
          joint->primitiveDisplacement = calloc(7, sizeof(double));
          cblas_dcopy(
            7,
            context->dq + joint->qIndex,
            1,
            joint->primitiveVelocity,
            1);
          joint->primitiveVelocity = calloc(6, sizeof(double));
          cblas_dcopy(
            6,
            context->dq + joint->dqIndex,
            1,
            joint->primitiveVelocity,
            1);
        } break;
        case S2_JOINT_PRIMITIVE:
        case S2_JOINT_INVALID: {
        } break;
      }
    }
    free(context->dc);
    context->dc = NULL;
  }
}


/*!
@brief Recursively clear Body and Joint Indices

@param assembly an s2_assembly_t datastructure
*/
void s2_clearIndices(
  s2_assembly_t* assembly) {

  s2_body_t* bodyHead = assembly->bodyHead;
  while(bodyHead) {
    bodyHead->index = -1;
      s2_frame_t* frameHead = bodyHead->frameHead;
      while(frameHead) {
        frameHead->index = -1;
        frameHead->vaIndex = -1;
        frameHead->isReference = false;
        frameHead->inTree = false;
        frameHead = frameHead->next;
      }
    bodyHead = bodyHead->next;
  }

  s2_joint_t* jointHead = assembly->jointHead;
  while (jointHead) {
    jointHead->index = -1;
    jointHead->qIndex = -1;
    jointHead->dqIndex = -1;
    jointHead = jointHead->next;
  }

  s2_light_t* lightHead = assembly->lightHead;
  while(lightHead) {
    lightHead->index = -1;
    lightHead = lightHead->next;
  }

  s2_io_t* ioHead = assembly->ioHead;
  while(ioHead) {
    ioHead->index = -1;
    ioHead = ioHead->next;
  }

  s2_restraint_t* restraintHead = assembly->restraintHead;
  while(restraintHead) {
    restraintHead->index = -1;
    restraintHead = restraintHead->next;
  }

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead) {
    s2_clearIndices(subAssemblyHead);
    subAssemblyHead = subAssemblyHead->next;
  }
}

/*!
  @brief free data related to substructures
*/
void s2_context_freeSubstructures(
  s2_context_t* context) {

  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    s2_arrayi_free(substructure->pc_12);
    s2_arrayi_free(substructure->pr_12);
    s2_arrayi_free(substructure->pc_21);
    s2_arrayi_free(substructure->pr_21);
  }
  s2_arrayi_free(context->Jxx);
  s2_array_free(context->J);
  s2_arrayb_free(context->Jb);
  if(context->substructures) free(context->substructures);
  context->substructures = NULL;
}


/*!
@brief Free run time arrays in a Context datastructure

Copy values back to original Body, Joint and Frame datastructures and free
run time arrays.

@param context an s2_context_t datastructure
*/
void s2_context_freeRunTimeArrays(
  s2_context_t *context) {

  // copy values by value back out of D,H,V,A before clearing
  // respective arrays
  // printf("Freeing dhva\n");
  s2_context_freeDHVA(context);

  // printf("Freeing direction cosines\n");
  s2_context_freeDc(context);
  //
  // printf("freed dhva\n");

  // printf("Freeing virtual bodies\n");
  for (int i = 0; i < context->activeJointCount; i++) {
    if (context->bodyPointers[i]->isVirtual) {
      s2_virtualbody_free(context->bodyPointers[i]);
    }
  }

  // printf("freeing body pointer array\n");
  if (context->bodyPointers) {
    free(context->bodyPointers);
    context->bodyPointers = NULL;
  }

  // printf("freeing joint pointer array\n");
  if (context->jointPointers) {
    free(context->jointPointers);
    context->jointPointers = NULL;
  }

  // printf("freeing frame pointer array\n");
  if (context->framePointers) {
    free(context->framePointers);
    context->framePointers = NULL;
  }

  if(context->dhFunctionPointers) {
    free(context->dhFunctionPointers);
    context->dhFunctionPointers = NULL;
  }

  if(context->dhvaFunctionPointers) {
    free(context->dhvaFunctionPointers);
    context->dhvaFunctionPointers = NULL;
  }

  if(context->dhva) {
    free(context->dhva[0]);
    free(context->dhva);
  }

  if(context->q) {
    free(context->q);
    context->q = NULL;
    context->dq = NULL;
  }

  // printf("freeing jointTypes array\n");
  if(context->jointTypes) {
    free(context->jointTypes);
    context->jointTypes = NULL;
  }

  // printf("freeing parent child list\n");
  if(context->pcl) {
    free(context->pcl);
    context->pcl = NULL;
  }

  // printf("freeing levels list\n");
  if(context->levels) {
    free(context->levels);
    context->levels = NULL;
  }

  // printf("freeing substructureMembers list\n");
  if(context->substructureMembers) {
    for (int i = 0; i < context->substructureCount; i++) {
      free(context->substructureMembers[i]);
    }
    free(context->substructureMembers);
    context->substructureMembers = NULL;
  }

  if(context->substructureMembersCount) {
    free(context->substructureMembersCount);
    context->substructureMembersCount = NULL;
  }

  // printf("freeing chordConstraints\n");
  if(context->chordConstraints) {
    free(context->chordConstraints);
    context->chordConstraints = NULL;
  }

  // printf("freeing isReference\n");
  if(context->isReference) {
    free(context->isReference);
    context->isReference = NULL;
  }

  // printf("freeing positiveLoopMembers\n");
  if(context->positiveLoopMembers) {
    free(context->positiveLoopMembers[0]);
    free(context->positiveLoopMembers);
    context->positiveLoopMembers = NULL;
  }

  // printf("freeing positiveLoopMembersCount\n");
  if(context->positiveLoopMembersCount) {
    free(context->positiveLoopMembersCount);
    context->positiveLoopMembersCount = NULL;
  }

  // printf("freeing negativeLoopMembers\n");
  if(context->negativeLoopMembers) {
    free(context->negativeLoopMembers[0]);
    free(context->negativeLoopMembers);
    context->negativeLoopMembers = NULL;
  }

  // printf("freeing negativeLoopMembersCount\n");
  if(context->negativeLoopMembersCount) {
    free(context->negativeLoopMembersCount);
    context->negativeLoopMembersCount = NULL;
  }

  // printf("freeing glReferenceFramesf\n");
  if(context->glReferenceFramesf) {
    free(context->glReferenceFramesf);
    context->glReferenceFramesf = NULL;
  }

  // printf("freeing glReferenceFrames\n");
  if(context->glReferenceFrames) {
    free(context->glReferenceFrames);
    context->glReferenceFrames = NULL;
  }

  if(context->nonVirtualBodyPointers) {
    free(context->nonVirtualBodyPointers);
    context->nonVirtualBodyPointers = NULL;
  }
  if(context->referenceFramePointers) {
    free(context->referenceFramePointers);
    context->referenceFramePointers = NULL;
  }
  if(context->lightPointers) {
    free(context->lightPointers);
    context->lightPointers = NULL;
  }
  if(context->names) {
    for(int i = 0; i < context->activeFrameInTreeCount; i++) {
      if(context->names[i]) {
        for(int j = 0; j < 4; j++) {
          if(context->names[i][j]) {
            free(context->names[i][j]);
          }
        }
        free(context->names[i]);
      }
    }
    free(context->names);
    context->names = NULL;
  }

  s2_arrayi_free(context->substructureLoopCount);

  context->activeBodyCount = 0;
  context->activeJointCount = 0;
  context->activeFrameCount = 0;
  context->chordConstraintCount = 0;
  context->activeFrameInTreeCount = 0;
  context->activeIOCount = 0;
  context->nonRigidJointCount = 0;
  context->qCount = 0;
  context->dqCount = 0;
  context->dcCount = 0;

  s2_clearIndices(context->assembly);

  s2_context_freeSubstructures(context);
  s2_arrayi_free(context->pv_12);
  s2_arrayi_free(context->pv_21);
  s2_arrayi_free(context->pv_13);
  s2_arrayi_free(context->pv_31);
  s2_arrayi_free(context->pv_32);
  s2_arrayi_free(context->pv_23);
  s2_arrayi_free(context->pcl);
  s2_arrayi_free(context->pclCompressed);
  s2_arrayi_free(context->pcldq);
  s2_arrayb_free(context->isRotation);
  s2_arrayb_free(context->isDependentPO1);
  s2_arrayb_free(context->isDependentPO3);

  s2_arrayi_free(context->cb2);
  s2_arrayi_free(context->cb3_temp);
  s2_arrayi_free(context->cb3);
  s2_arrayi_free2D(context->rb3);
  s2_arrayi_free2D(context->rb3_temp);
  s2_arrayi_free2D(context->rb2);
  s2_arrays_free(context->mhProduct);
  s2_arrayi_free(context->cm);
  s2_arrayi_free(context->colb);
  s2_arrayi_free(context->pb_23);
  s2_arrayi_free(context->pb_32);
  s2_arrayi_free(context->ijMapping);
  s2_array_free(context->b3);
  s2_array_free(context->f);
  s2_array_free(context->fl);
  s2_array_free(context->la);
  s2_arrayi_free(context->i1);

  s2_arrayi_free(context->i2);
  s2_arrayi_free(context->i3);
  s2_arrayi_free(context->i4);
  s2_arrayi_free(context->i5);
  s2_arrayi_free(context->i6);
  s2_arrayi_free(context->i7);
  s2_arrayi_free(context->i8);
  s2_arrayi_free(context->j1);
  s2_arrayi_free(context->j2);
  s2_arrayi_free(context->j3);
  s2_arrayi_free(context->j4);
  s2_arrayi_free(context->j5);
  s2_arrayi_free(context->j6);
  s2_arrayi_free(context->p1);
  s2_arrayi_free(context->p2);
  s2_arrayi_free(context->p3);
  s2_arrayi_free(context->n1);
  s2_arrayi_free(context->n2);
  s2_arrayi_free(context->n3);
  if(context->P) free(context->P);
  context->P = NULL;
  context->p3Start = 0;
  if(context->inertia) free(context->inertia);
  context->inertia = NULL;
  if(context->inertiaP) free(context->inertiaP);
  context->inertiaP = NULL;
  s2_array_free(context->Ml);
  s2_array_free(context->Mls);
  s2_arrayb_free(context->Mlb);
  s2_arrayb_free(context->Mlbs);
  s2_arrayb_free(context->hasMass);
  s2_arrayi_free(context->jointDOF);
  s2_arrayb_free(context->originInTree);
  s2_arrayb_free(context->hasMassdq);
  s2_arrayi_free(context->positiveLoopMembersCompressedCount);
  s2_arrayi_free(context->negativeLoopMembersCompressedCount);
  s2_arrayi_free(context->dqLookupCompressed);
  s2_arrayi_free(context->dqLookup);
  s2_arrayi_free(context->qLookup);
  s2_arrayi_free(context->jointLookup);
  if(context->positiveLoopMembers) {
    s2_arrayi_free2D(context->positiveLoopMemberOffsets);
  }
  if(context->negativeLoopMembers) {
    s2_arrayi_free2D(context->negativeLoopMemberOffsets);
  }
  if(context->constraints) {
    s2_arrayi_free2D(context->constraints);
  }
  s2_arrays_free(context->g);
  if(context->dcq) free(context->dcq);
  context->dcq = NULL;
  if(context->originPointers) free(context->originPointers);
  context->originPointers = NULL;
  s2_array_free(context->ddq);
  if(context->dReferenceFrames) free(context->dReferenceFrames);
  context->dReferenceFrames = NULL;
  s2_arrays_free(context->p);
  s2_array_free(context->gamma_d);
  if(context->glReferenceFramesf) free(context->glReferenceFramesf);
  context->glReferenceFramesf = NULL;

}


/*!
@brief Free frames on the *fixed* inertial reference frame
@param context a Context datastructure
*/
void s2_context_freeFixedFrames(
  s2_context_t* context) {

  s2_body_t* bodyHead = context->assembly->bodyHead;
  while(bodyHead) {
    s2_body_t* bodyNext = bodyHead->next;
    s2_body_free(bodyHead);
    bodyHead = bodyNext;
  }

  s2_light_t* lightHead = context->assembly->lightHead;
  while(lightHead) {
    s2_light_t* lightNext = lightHead->next;
    s2_light_free(lightHead);
    lightHead = lightNext;
  }

  s2_stackstr_free(context->assembly->name);
  free(context->assembly);
}


/*!
@brief Free a Context datastructure
@param context a Context datastructure
*/
void s2_context_free(
  s2_context_t* context) {

  if(context) {
    s2_numericalintegrator_free(context);
    s2_modelparameters_free(context->modelParameters);
    s2_context_freeRunTimeArrays(context);

    // printf("freeing assembly\n");
    s2_assembly_free(context->assembly);

    // printf("freeing fixed frames\n");
    s2_context_freeFixedFrames(context);

    s2_arrayi_free(context->qQuaternionIndices);
    s2_arrayi_free(context->dqQuaternionIndices);
    s2_arrayi_free(context->qNonQuaternionIndices);
    s2_arrayi_free(context->dqNonQuaternionIndices);

    free(context);
    context = NULL;
  }
}


/*!
@brief get a frame pointer by specifying root body name and root frame name
@param [in] context data structure
@param [in] root descriptive body name as specified in json file
@param [in] root descriptive frame name as specified in json file
@return frame pointer, returns NULL if none found
*/
s2_frame_t* s2_context_getFrameByName(
  s2_context_t* context,
  const char* bodyName,
  const char* frameName) {

  for(int i = 0; i < context->activeJointCount; i++) {
    s2_body_t* bodyHead = context->bodyPointers[i];
    if(!strcmp(bodyName, s2_stackstr_getRoot(bodyHead->name))) {
      s2_frame_t* frameHead = bodyHead->frameHead;
      while(frameHead) {
        if(!strcmp(frameName, s2_stackstr_getRoot(frameHead->name))) {
          return frameHead;
        }
        frameHead = frameHead->next;
      }
    }
  }
  return NULL;
}


/*!
@brief get a body pointer by specifying root body name, matches first body
with root name match searching through all assemblies
@param [in] context data structure
@param [in] root descriptive body name as specified in json file
@param [in] root descriptive frame name as specified in json file
@return frame pointer, returns NULL if none found
*/
s2_body_t* s2_context_getBodyByName(
  s2_context_t* context,
  const char* bodyName) {

  for(int i = 0; i < context->activeJointCount; i++) {
    s2_body_t* bodyHead = context->bodyPointers[i];
    if(!strcmp(bodyName, s2_stackstr_getRoot(bodyHead->name))) {
      return bodyHead;
    }
  }
  return NULL;
}


/*!
@brief get a pointer to debug file pointer

*/
FILE* s2_context_getDebugPointer(s2_context_t* context, int i) {
  if(context->modelParameters) {
    if(i == 0) {
      return context->modelParameters->debugFilePointer;
    } else if(i == 1) {
      return context->modelParameters->dhvaFilePointer;
    }
  }
  return stdout;
}


/*!
@brief get current simulation time
*/
double s2_context_getSimulationTime(s2_context_t* context) {
  if(context->modelParameters && context->numericalIntegrator) {
    switch(context->modelParameters->integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          context->numericalIntegrator;
        return numericalIntegrator->t;
      } break;
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          context->numericalIntegrator;
        return numericalIntegrator->x;
      } break;
    }
  }
  return -1;
}


/*!
@brief get current iconv value from numerical integrator
*/
int s2_context_getIConv(s2_context_t* context) {
  if(context->modelParameters && context->numericalIntegrator) {
    switch(context->modelParameters->integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          context->numericalIntegrator;
        return numericalIntegrator->iconv;
      } break;
      default: {
        return -1;
      }
    }
  }
  return -1;
}
