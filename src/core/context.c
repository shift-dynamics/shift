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
#include "utility.h"
#include "sphere.h"
#include "../force_modules/tire.h"

void s2_context_setBodyJointPointers(
  s2_context_t* context);

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
  context->modelParameters.version = NULL;
  context->modelParameters.includeDirectory = NULL;

  context->modelParameters.gravity = s2_vector();
  context->modelParameters.units.angle = NULL;
  context->modelParameters.units.length = NULL;
  context->modelParameters.units.mass = NULL;
  context->modelParameters.units.time = NULL;

  context->modelParameters.outFile = NULL;
  context->modelParameters.outFilePointer = NULL;
  context->modelParameters.debugFile = NULL;
  context->modelParameters.debugFilePointer = stdout;

  /*! @TODO remove this later once dhva verified */
  context->modelParameters.dhvaFilePointer =
    fopen("dhva.dat", "w");
  if(!context->modelParameters.dhvaFilePointer) {
    fprintf(stderr, "could not open dhva debug file\n");
    context->modelParameters.dhvaFilePointer = NULL;
  }

  asprintf(
    &(context->modelParameters.delimiter),
    ","
  );
  context->modelParameters.monitorIterations = true;
  context->modelParameters.monitorConstraintError = true;


  context->modelParameters.sumConstraintError = 0;
  context->modelParameters.maxConstraintError = 0;
  context->modelParameters.maxConstraintErrorStep = 0;
  context->modelParameters.sumIterationCount = 0;
  context->modelParameters.maxIterationCount = 0;
  context->modelParameters.maxIterationCountStep = 0;

  context->assembly = s2_assembly();

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
  context->pointCloudCount = 0;
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
  context->activePointCloudCount = 0;
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

  context->pointCloudPointers = NULL;
  /*!< Array of pointers to pointclouds for direct access
       within assembly hierarchy */

  context->boundingSpheres = NULL;

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

  context->modelParameters.baumgarteOmegaN = omegaN;
  context->modelParameters.baumgarte2OmegaN = 2 * omegaN;
  context->modelParameters.baumgarteOmegaN2 =
    omegaN * omegaN;
}


/*!
@brief Print a Context datastructure
@param context a Context datastructure
*/
void s2_context_print(
  FILE* f,
  s2_context_t* context) {

  fprintf(f, "\nModel Parameters:\n");
  fprintf(f, "\ts2_version: %s\n", context->modelParameters.version);
  fprintf(f, "\tgravity: ");
  s2_vector_print(f, context->modelParameters.gravity);
  fprintf(f, "\ttime: %s\n", context->modelParameters.units.time);
  fprintf(f, "\tlength: %s\n", context->modelParameters.units.length);
  fprintf(f, "\tmass: %s\n", context->modelParameters.units.mass);
  fprintf(f, "\tangle: %s\n", context->modelParameters.units.angle);
  fprintf(f, "\tinclude_dir: ");
  s2_stackstr_print(f, context->modelParameters.includeDirectory);
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
    s2_context_freeModelParameters(context);
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
  if(i == 0) {
    return context->modelParameters.debugFilePointer;
  } else if(i == 1) {
    return context->modelParameters.dhvaFilePointer;
  } else {
    return stdout;
  }
}


/*!
@brief get current simulation time
*/
double s2_context_getSimulationTime(s2_context_t* context) {
  if(context->numericalIntegrator) {
    switch(context->modelParameters.integrationType) {
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
  if(context->numericalIntegrator) {
    switch(context->modelParameters.integrationType) {
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


/*!
@brief loads fixed frames into an s2_context_t datastructure
*/
int s2_context_loadFixedFrames(
  s2_context_t* context,
  json_t* root) {

  context->assembly->name =
    s2_stackstr_fromString("fixed");

  s2_body_t* fixedBody =
    s2_body(s2_stackstr_fromString("fixed"));
  context->assembly->bodyHead = fixedBody;

  json_t* fixedObject = json_object_get(root, "fixed");
  if(json_is_object(fixedObject)) {
    json_t* framesObject = json_object_get(fixedObject, "frames");
    if(!json_is_null(framesObject) && json_is_array(framesObject)) {
      size_t framesIndex;
      json_t* frameObject;
      json_array_foreach(framesObject, framesIndex, frameObject) {
        s2_frame_t* frame = s2_frame();
        frame->name = s2_stackstr_copy(fixedBody->name);
        s2_stackstr_append(
          frame->name,
          json_string_value(json_object_get(frameObject, "name")));
        json_t* rotation = json_object_get(frameObject, "R");
        if (json_is_array(rotation)) {
          for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
              s2_displacement_set(
                frame->dWRTOrigin,
                j,
                k,
                json_number_value(
                  json_array_get(json_array_get(rotation, j), k)));
            }
          }
        }
        json_t* translation = json_object_get(frameObject, "T");
        if (json_is_array(translation)) {
          for (int j = 0; j < 3; j++) {
            s2_vector_set(
              &(frame->dWRTOrigin->T),
              j,
              json_number_value(json_array_get(translation, j)));
            // frame->translationWRTReference[j] = frame->translationWRTOrigin[j];
          }
        }

        s2_displacement_invert(
          frame->dWRTOrigin,
          frame->dWRTOriginInverse);
        (context->frameCount)++;
        s2_body_appendFrame(fixedBody, frame);
      }
    }
  }
  return 0;
}


/*!
@brief Load model parameters from a star2 JSON file

*/
void s2_context_loadModelParameters(
  s2_context_t* context,
  json_t* modelParametersObject,
  const char* filePath) {

  asprintf(
    &(context->modelParameters.version),
    "%s",
    json_string_value(
      json_object_get(modelParametersObject, "version")));

  asprintf(
    &(context->modelParameters.units.angle),
    "%s",
    json_string_value(
      json_object_get(
        json_object_get(modelParametersObject, "units"), "angle")));

  asprintf(
    &(context->modelParameters.units.length),
    "%s",
    json_string_value(
      json_object_get(
        json_object_get(modelParametersObject, "units"), "length")));

  asprintf(&(context->modelParameters.units.mass), "%s",
    json_string_value(json_object_get(
      json_object_get(modelParametersObject, "units"), "mass")));

  asprintf(&(context->modelParameters.units.time), "%s",
    json_string_value(json_object_get(
      json_object_get(modelParametersObject, "units"), "time")));

  for (int i = 0; i < 3; i++) {
    ((double*)(context->modelParameters.gravity))[i] = json_number_value(
      json_array_get(json_object_get(modelParametersObject, "gravity"), i));
  }

  /*
    Load simulation parameters
  */
  json_t* simulationObject =
    json_object_get(modelParametersObject, "simulation");

  /* if there is no simulationObject, then set simulation parameters to default
  values */
  if(simulationObject != NULL) {
    /*
      parse integration type
    */
    json_t* integrationObject =
      json_object_get(simulationObject, "integration_type");
    if(integrationObject != NULL) {
      const char* integrationType = json_string_value(integrationObject);
      if(!strcmp(integrationType, "ode113")) {
        context->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
      } else if(!strcmp(integrationType, "ode113b")) {
        context->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113B;
      } else {
        /*
          if other integration types are created in the future, add parser for
          them here
        */
        fprintf(
          stderr,
          "Warning, an integration type other than ode113 or ode113b was" "specified\n"
          "currently the program only supports ode113. Setting to ode113\n");
        context->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
      };
    } else {
      fprintf(
        stderr,
        "Warning, no integration type specified. ode113 will be used.\n");
      context->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
    }

    /* load omega N for baumgarte constraint stabilization */
    json_t* omegaNObject =
      json_object_get(simulationObject, "omega_n");
    double omegaN = 20;
    if(omegaNObject != NULL) {
      omegaN = json_number_value(omegaNObject);
    }
    context->modelParameters.baumgarteOmegaN = omegaN;
    context->modelParameters.baumgarte2OmegaN = 2 * omegaN;
    context->modelParameters.baumgarteOmegaN2 = omegaN * omegaN;

    /*
      parse time span
    */
    json_t* timeSpanObject = json_object_get(simulationObject, "time_span");
    if(timeSpanObject != NULL) {
      context->modelParameters.timeSpan = json_number_value(timeSpanObject);
    } else {
      fprintf(stderr, "Warning, no time span specifed in JSON file, "
        "using 1 second.\n");
      context->modelParameters.timeSpan = 1.0;
    }

    /*
      parse time step
    */
    json_t* timeStepObject = json_object_get(simulationObject, "time_step");
    if(timeStepObject != NULL) {
      context->modelParameters.timeStep = json_number_value(timeStepObject);
    } else {
      fprintf(stderr, "Warning, no time step specifed in JSON file, "
        "using 0.0001 seconds.\n");
      context->modelParameters.timeStep = 0.0001;
    }

    json_t* outFileObject =
      json_object_get(simulationObject, "out_file");
    if(outFileObject != NULL) {
      asprintf(
        &(context->modelParameters.outFile),
        "%s",
        json_string_value(outFileObject));
    }

    json_t* debugFileObject =
      json_object_get(simulationObject, "debug_file");
    if(debugFileObject != NULL) {
      if(strcmp(json_string_value(debugFileObject), "stdout")) {
        asprintf(
          &(context->modelParameters.debugFile),
          "%s",
          json_string_value(debugFileObject));
        context->modelParameters.debugFilePointer =
          fopen(context->modelParameters.debugFile, "w");
        if(!context->modelParameters.debugFilePointer) {
          fprintf(stderr, "unable to open debug file, exiting.\n");
          exit(-1);
        }
      }
    }

    /*
      parse relative error
    */
    json_t* relativeErrorObject =
      json_object_get(simulationObject, "relative_error");
    if(relativeErrorObject != NULL) {
      context->modelParameters.relativeError = json_number_value(relativeErrorObject);
    } else {
      fprintf(
        stderr,
        "Warning, no relative integration error specifed in JSON "
        "file, using 0.0 seconds.\n");
      context->modelParameters.relativeError = 0.0;
    }

    /*
      parse absolute error
    */
    json_t* absoluteErrorObject =
      json_object_get(simulationObject, "absolute_error");
    if(absoluteErrorObject != NULL) {
      context->modelParameters.absoluteError = json_number_value(absoluteErrorObject);
    } else {
      fprintf(
        stderr,
        "Warning, no absolute integration error specifed in JSON "
        "file, using 0.0001 seconds.\n");
      context->modelParameters.absoluteError = 0.0001;
    }

    /*
      parse infinity norm multiplier
    */
    json_t* infNormMultiplierObject =
      json_object_get(simulationObject, "infinity_norm_multiplier");
    if(infNormMultiplierObject != NULL) {
      context->modelParameters.pivotRatioMultiple =
        json_number_value(infNormMultiplierObject);
    } else {
      fprintf(
        stderr,
        "Warning, no infinity norm multiplier specified in "
        "file, using 4.\n");
      context->modelParameters.pivotRatioMultiple = 4;
    }

  } else {
    fprintf(stderr, "Warning, no simulation parameters specifed in JSON file, "
      "using default values\n");
    context->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
    context->modelParameters.timeSpan = 1.0;
    context->modelParameters.timeStep = 0.0001;
    context->modelParameters.relativeError = 0.0;
    context->modelParameters.absoluteError = 1e-4;
  }

  // Load include directories
  json_t* includeDirectoryObject =
    json_object_get(modelParametersObject, "include_dir");

  context->modelParameters.includeDirectory = s2_stackstr();
  char* fullFilePath = s2_getFilePath(filePath);
  if(fullFilePath){
    s2_stackstr_append(
      context->modelParameters.includeDirectory,
      fullFilePath);
    free(fullFilePath);
  }
  if(json_is_array(includeDirectoryObject)) {
    size_t index;
    json_t* value;
    json_array_foreach(includeDirectoryObject, index, value) {
      s2_stackstr_append(
        context->modelParameters.includeDirectory,
        json_string_value(value));
    }
  } else if(json_is_string(includeDirectoryObject)) {
    s2_stackstr_append(
      context->modelParameters.includeDirectory,
      json_string_value(includeDirectoryObject));
  }
}


/*!
@brief free memory associated with modelParameters substructure
*/
void s2_context_freeModelParameters(s2_context_t* context) {
  if(context->modelParameters.units.angle) {
    free(context->modelParameters.units.angle);
    context->modelParameters.units.angle = NULL;
  }
  if(context->modelParameters.units.length) {
    free(context->modelParameters.units.length);
    context->modelParameters.units.length = NULL;
  }
  if(context->modelParameters.units.mass) {
    free(context->modelParameters.units.mass);
    context->modelParameters.units.mass = NULL;
  }
  if(context->modelParameters.units.time) {
    free(context->modelParameters.units.time);
    context->modelParameters.units.time = NULL;
  }
  if(context->modelParameters.dhvaFilePointer) {
    fclose(context->modelParameters.dhvaFilePointer);
  }
  if(
    context->modelParameters.debugFilePointer &&
    context->modelParameters.debugFilePointer != stdout) {

    fclose(context->modelParameters.debugFilePointer);
  }

  if(context->modelParameters.version) {
    free(context->modelParameters.version);
    context->modelParameters.version = NULL;
  }
  s2_stackstr_free(context->modelParameters.includeDirectory);

  if(context->modelParameters.gravity) {
    free(context->modelParameters.gravity);
    context->modelParameters.gravity = NULL;
  }
}


/*!
@brief Processes light pointers from a flattened list of lights

Sets parent frame for a light for a flattened list of lights and frames

@param context a Context datastructure
*/
void s2_context_setLightPointers(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "# processing light pointers\n");
  // fprintf(debug, "%d lights\n", context->lightCount);
  for (int i = 0; i < context->lightCount; i++) {
    // fprintf(debug, "\n\nprocessing light ");
    // s2_stackstr_print(debug, context->lightPointers[i]->name);
    //
    // fprintf(debug, "\nsearching for ");
    // s2_stackstr_print(debug, context->lightPointers[i]->parentFrame);

    for (int j = 0; j < context->bodyCount; j++) {
      s2_frame_t* frameHead = context->bodyPointers[j]->frameHead;
      while(frameHead) {
        if(s2_stackstr_isEqual(
            context->lightPointers[i]->parentFrame,
            frameHead->name)) {

          // fprintf(debug, "\nmatch found!");
          context->lightPointers[i]->parentFramePointer = frameHead;

        }
        frameHead = frameHead->next;
      }
    }
  }
}


void s2_context_flattenFixedLights(
  s2_context_t* context,
  int* lightCounter) {

  s2_light_t* lightHead = context->assembly->lightHead;
  while(lightHead) {
    context->lightPointers[*lightCounter] = lightHead;
    lightHead->index = (*lightCounter);
    (*lightCounter)++;
    lightHead = lightHead->next;
  }
}


void s2_context_flattenHierarchy(
  s2_assembly_t* assembly,
  s2_context_t* context,
  int* bodyCounter,
  int* jointCounter,
  int* lightCounter) {

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  // if(subAssemblyHead){
  while(subAssemblyHead) {
    s2_body_t* bodyHead = subAssemblyHead->bodyHead;
    while(bodyHead) {
      context->bodyPointers[*bodyCounter] = bodyHead;
      bodyHead->index = (*bodyCounter);
      (*bodyCounter)++;
      bodyHead = bodyHead->next;
    }
    s2_joint_t* jointHead = subAssemblyHead->jointHead;
    while(jointHead) {
      context->jointPointers[*jointCounter] = jointHead;
      jointHead->index = (*jointCounter);
      (*jointCounter)++;
      jointHead = jointHead->next;
    }
    s2_light_t* lightHead = subAssemblyHead->lightHead;
    while(lightHead) {
      context->lightPointers[*lightCounter] = lightHead;
      lightHead->index = (*lightCounter);
      (*lightCounter)++;
      lightHead = lightHead->next;
    }

    if(subAssemblyHead->subAssemblyHead) {
      s2_context_flattenHierarchy(
        subAssemblyHead,
        context,
        bodyCounter,
        jointCounter,
        lightCounter);
    }
    subAssemblyHead = subAssemblyHead->next;
  }
}


void s2_context_setBoundingSpheres(s2_context_t* context) {

  context->boundingSpheres = calloc(
    context->bodyCount, sizeof(s2_sphere_t));
  if(context->boundingSpheres == NULL) {
    fprintf(stderr, "failed to allocate memory for bounding spheres\n");
    exit(-1);
  }
  int j = 0;
  for(int i = 0; i < context->bodyCount + context->chordConstraintCount; i++) {
    s2_body_t* body = context->bodyPointers[i];
    if(!body->isVirtual) {
      printf("[%d] ", i);
      s2_stackstr_print(stdout, body->name);
      s2_sphere_approximateMinBoundSphere(
        &context->boundingSpheres[j++],
        context->bodyPointers[i]);
    }
  }
}


/*!
@brief Loads hierarchical json file into s2_context_t data structure

Recursive function to extract all bodies and joints in
 arbitrarily nested assembly hierarchy

@returns Context
*/
s2_context_t* s2_context_loadJSON(
  json_t* root,
  const char* filePath) {

  s2_context_t* context = s2_context();

  // load modelParameters
  json_t* modelParametersObject =
    json_object_get(root, "model_parameters");
  if(!json_is_object(modelParametersObject)) {
    fprintf(stderr, "No valid model parameters object found.\n");
    return NULL;
  }

  s2_context_loadModelParameters(
    context,
    modelParametersObject,
    filePath);

  s2_context_loadFixedFrames(context, root);

  json_t* fixedObject = json_object_get(root, "fixed");

  s2_assembly_loadLights(
    context->assembly,
    fixedObject,
    context,
    context->assembly->name);

  /* modelParametersObject must be loaded before terrain
  so that include directores can be used to find terrain
  related files */
  s2_terrain_load(
    context->assembly,
    fixedObject,
    context,
    context->assembly->name);

  json_t* assembliesObject = json_object_get(root, "assemblies");

  if (!json_is_array(assembliesObject)) {
    fprintf(stderr, "No valid assembly object found.\n");
    return NULL;
  }

  s2_stackstr_t* prefix = s2_stackstr();
  s2_assembly_load(context->assembly, assembliesObject, context, prefix);
  s2_stackstr_free(prefix);

  context->bodyPointers =
    malloc(sizeof(s2_body_t *) * (context->bodyCount + 1));
  context->jointPointers =
    malloc(sizeof(s2_joint_t *) * context->jointCount);
  context->lightPointers =
    malloc(sizeof(s2_light_t*) * context->lightCount);

  for (int i = 0; i < (context->bodyCount + 1); i++) {
    context->bodyPointers[i] = NULL;
  }
  for (int i = 0; i < context->jointCount; i++) {
    context->jointPointers[i] = NULL;
  }
  for (int i = 0; i < context->lightCount; i++) {
    context->lightPointers[i] = NULL;
  }
  context->bodyPointers[0] = context->assembly->bodyHead;

  int bodyCounter = 1;
  int jointCounter = 0;
  int lightCounter = 0;

  s2_context_flattenFixedLights(
    context,
    &lightCounter);

  s2_context_flattenHierarchy(
    context->assembly,
    context,
    &bodyCounter,
    &jointCounter,
    &lightCounter);

  s2_context_setBodyJointPointers(context);
  s2_context_setLightPointers(context);
  s2_context_setTerrainPointers(context);

  return context;
}


/*!
@brief Parse a STAR2 JSON file and return in "context", a SetupContext
data structure

This function parses a STAR2 JSON file and returns a SetupContext
data structure. The JSON file is parsed using the Jansson C library.
@param fileName path to star2 JSON file.

@return a setupContext data structure.
*/
s2_context_t* s2_context_load(
  const char* fileName) {

  json_error_t error;
  json_t *root = json_load_file(fileName, 0, &error);

  /*
    get path to file name
  */

  if(!root) {
    fprintf(
      stderr,
      "Error: file loading error on line %d: %s\n",
      error.line,
      error.text);
    return NULL;
  }

  if (!json_is_object(root)) {
    fprintf(
      stderr,
      "Error: file loading error, file does not contain a valid object\n");
    json_decref(root);
    return NULL;
  }
  char* realPath = realpath(fileName, NULL);
  s2_context_t* context = s2_context_loadJSON(root, realPath);
  free(realPath);
  json_decref(root);
  return context;
}


/*!
@brief
*/
bool s2_connectFramePointer(
  s2_body_t* body,
  s2_joint_t* joint,
  bool searchParent) {

  bool found = false;
  s2_frame_t* frameHead = body->frameHead;
  if(searchParent) {
    while(frameHead) {
      if(s2_stackstr_isEqual(joint->parentFrame, frameHead->name)) {
        joint->parentFramePointer = frameHead;
        found = true;
        break;
      }
      frameHead = frameHead->next;
    }
  } else {
    while(frameHead) {
      if(s2_stackstr_isEqual(joint->childFrame, frameHead->name)) {
        joint->childFramePointer = frameHead;
        found = true;
        break;
      }
      frameHead = frameHead->next;
    }
  }
  return found;
}


/*!
@brief Processes body joint pointers from a flattened list of bodies and joints

The flattened list must be generated using s2_LoadFlattenedBodyJointList

@param context a Context datastructure
*/
void s2_context_setBodyJointPointers(
  s2_context_t* context) {

  for(int i = 0; i < context->jointCount; i++) {
    bool parentFound = false;
    for(int j = 0; j < context->bodyCount + 1; j++) {
      if(s2_stackstr_isEqual(
          context->jointPointers[i]->parentBody,
          context->bodyPointers[j]->name)) {

        context->jointPointers[i]->parentBodyPointer = context->bodyPointers[j];
        if (!s2_connectFramePointer(
              context->bodyPointers[j],
              context->jointPointers[i], true)) {
          fprintf(
            stderr,
            "Warning, parentFrame not found for joint: %s, frame: %s\n",
            context->jointPointers[i]->name->tail->value,
            context->jointPointers[i]->parentFrame->tail->value);
        } else {
          parentFound = true;
        }
        break;
      }
    }
    bool childFound = false;
    for (int j = 0; j < context->bodyCount + 1; j++) {
      if(s2_stackstr_isEqual(
          context->jointPointers[i]->childBody,
          context->bodyPointers[j]->name)) {

        context->jointPointers[i]->childBodyPointer = context->bodyPointers[j];
        if (!s2_connectFramePointer(
              context->bodyPointers[j],
              context->jointPointers[i], false)) {
          fprintf(
            stderr,
            "Warning, parentFrame not found for joint: %s, frame: %s\n",
            context->jointPointers[i]->name->tail->value,
            context->jointPointers[i]->childFrame->tail->value);
        } else {
          childFound = true;
        }
        break;
      }
    }

    if(!parentFound) {
      fprintf(
        stderr,
        "Warning, parentBody not found for joint: %s, body: %s\n",
        context->jointPointers[i]->name->tail->value,
        context->jointPointers[i]->childBody->tail->value);
    }

    if(!childFound) {
      fprintf(
        stderr,
        "Warning, childBody not found for joint: %s, body: %s\n",
        context->jointPointers[i]->name->tail->value,
        context->jointPointers[i]->childBody->tail->value);
    }
  }
}


/*!
@brief Sets parent frame and body pointers for terrains

@param context an s2_context_t datastructure
*/
void s2_context_setTerrainPointers(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  //
  // fprintf(debug, "# processing elevation map pointers\n");
  // fprintf(debug, "%d lights\n", context->lightCount);
  for (int i = 0; i < context->terrainCount; i++) {
    // fprintf(debug, "\n\nprocessing light ");
    // s2_stackstr_print(debug, context->lightPointers[i]->name);
    //
    // fprintf(debug, "\nsearching for ");
    // s2_stackstr_print(debug, context->lightPointers[i]->parentFrame);

    for (int j = 0; j < context->bodyCount; j++) {
      s2_frame_t* frameHead = context->bodyPointers[j]->frameHead;
      if(s2_stackstr_isEqual(
        context->terrains[i]->parentBody,
        context->bodyPointers[j]->name)) {
          context->terrains[i]->parentBodyPointer =
            context->bodyPointers[j];
        // s2_stackstr_print(debug, context->bodyPointers[j]->name);
      }

      while(frameHead) {
        if(s2_stackstr_isEqual(
            context->terrains[i]->parentFrame,
            frameHead->name)) {
          context->terrains[i]->parentFramePointer = frameHead;
          // s2_stackstr_print(debug, frameHead->name);
        }
        frameHead = frameHead->next;
      }
    }
  }
}


/*!
@brief update global centroid of bounding spheres
*/
void s2_context_updateBoundingSpheres(s2_context_t* context) {
  for(int i = 0; i < context->bodyCount; i++) {
    s2_sphere_updateCentroid(&context->boundingSpheres[i]);
    s2_stackstr_print(stdout, context->boundingSpheres[i].parentBody->name);
    printf("radius: %f, centerGlobal: ", context->boundingSpheres[i].radius);
    s2_vector_print(stdout, &context->boundingSpheres[i].centerGlobal);
    printf("centerLocal: ");
    s2_vector_print(stdout, &context->boundingSpheres[i].centerLocal);
  }
}
