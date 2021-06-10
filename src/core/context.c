/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Context datastructure.
@date 2018
@copyright Kristopher Wehage 2018

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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include "../force_modules/tire.h"
#include "assembly.h"
#include "body.h"
#include "context.h"
#include "frame.h"
#include "io.h"
#include "joint.h"
#include "kinematics.h"
#include "light.h"
#include "sphere.h"
#include "substructure.h"
#include "utility.h"

void s2_context_setBodyJointPointers(s2_context_t* context);

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

  s2_context_t* self = malloc(sizeof *self);

  if(!self) {
    fprintf(stderr, "Error: failed to initialize Context.\n");
    return NULL;
  }
  self->modelParameters.version = NULL;
  self->modelParameters.includeDirectory = NULL;

  self->modelParameters.gravity = s2_vector();
  self->modelParameters.units.angle = NULL;
  self->modelParameters.units.length = NULL;
  self->modelParameters.units.mass = NULL;
  self->modelParameters.units.time = NULL;

  self->modelParameters.outFile = NULL;
  self->modelParameters.outFilePointer = NULL;
  self->modelParameters.debugFile = NULL;
  self->modelParameters.debugFilePointer = stdout;

  /*! @TODO remove this once dhva verified */
  self->modelParameters.dhvaFilePointer = fopen("dhva.dat", "w");
  if(!self->modelParameters.dhvaFilePointer) {
    fprintf(stderr, "could not open dhva debug file\n");
    self->modelParameters.dhvaFilePointer = NULL;
  }

  asprintf(&(self->modelParameters.delimiter), ",");
  self->modelParameters.monitorIterations = true;
  self->modelParameters.monitorConstraintError = true;

  self->modelParameters.sumConstraintError = 0;
  self->modelParameters.maxConstraintError = 0;
  self->modelParameters.maxConstraintErrorStep = 0;
  self->modelParameters.sumIterationCount = 0;
  self->modelParameters.maxIterationCount = 0;
  self->modelParameters.maxIterationCountStep = 0;

  self->modelParameters.contactDetectionEnabled = false;
  self->assembly = s2_assembly();

  /*!
  @name Counts of objects in Context data structure
  @{
  */
  self->epsRankDetermination = 1e-9; /*!< epsilon, tolerance used for rank
                            determination in LU factorization */
  self->epsNewtonRaphsonRankDetermination = 1e-12;
  self->epsNewtonRaphson = 1e-6;
  self->jointCount = 0; //!< total number of joints
  self->bodyCount = 0;  //!< total number of bodies
  self->pointCloudCount = 0;
  self->frameCount = 0;         //!< total number of frames
  self->lightCount = 0;         //!< total number of lights
  self->ioCount = 0;            //!< total number of input/output devices
  self->restraintCount = 0;     //<! total number of restraints
  self->nonRigidJointCount = 0; /*!< total number of non-rigid joints, equal
                                        to dof */
  self->dhvaCount = 0;
  self->activeFrameCount = 0; /*!< number of active frames in mechanism */
  self->activeFrameInTreeCount =
    0;                        /*!< number of active frames in spanning tree */
  self->activeJointCount = 0; /*!< number of active joints */
  self->activeBodyCount = 0;  /*!< number of active bodies */
  self->activePointCloudCount = 0;
  self->activeLightCount = 0;     /*!< number of active lights */
  self->activeIOCount = 0;        /*!< number of active devices */
  self->activeRestraintCount = 0; //!< number of active restraints
  self->qCount = 0;               /*!< dof for generalized positions */
  self->dqCount = 0;              /*!< dof for generalized velocities */
  self->dcCount = 0;              /*!< count of direction cosines */

  self->chordConstraintCount = 0; /*!< number of chord constraints, one for
                                 each kinematic loop */
  self->dqLookup = NULL;
  /*!< lookup table used to look up a joint's position in variable
       list. float variables have 6 variables, spherical joints have
       3 variables, and rigid joints
       have 0 variables associated with them */
  self->qLookup = NULL;
  self->jointLookup = NULL;
  /*!< lookup table used to find a joint's position in compressed joint list
       rigid joints are removed from compressed list and have a value of -1
       in lookup table */
  self->dqLookupCompressed = NULL;

  /*!
  @}
  @name Data related to processing kinematic loops within mechanism
  @{
  */
  self->chordConstraints = NULL; /*!< specified chord constraints, if
                                         multiple loops are present in a
                                         mechanism */
  self->J = NULL;                /*!< Jacobian matrix (or matrices) */
  self->Js = NULL;
  self->Jxx = NULL;       /*!< instructions for evaluating Jacobian matrices */
  self->Jb = NULL;        /*!< Boolean Jacobian matrix */
  self->jacobianSize = 0; /*!< total size of jacobian matrix */

  self->isDependentPO1 = NULL; /*!< equal to dof, in PO1 */
  self->isDependentPO3 = NULL; /*! equal to dof, in PO3 */
  self->hasMass = NULL;        /*!< equal to active jointCount */
  self->hasMassdq = NULL;      /*!< indicates whether body associated with
                                       joint variable has mass itself or
                                       has mass projected onto it from
                                       a child body */
  self->jointDOF = NULL;
  self->jointDOFCompressed = NULL;

  self->substructures = NULL;
  self->substructureCount = 0;        /*!< Number of kinematic substructures */
  self->substructureLoopCount = NULL; /*!< Number of loops in each
                                   kinematic substructure */
  self->substructureMembers =
    NULL; /*!< members of each kinematic substructure */
  self->substructureMembersCount =
    NULL; /*!< count of members in each substructure */

  self->positiveLoopMembers =
    NULL; /*!< Indicates mechanism's positive loop members */
  self->positiveLoopMembersCount =
    NULL; /*!< Count of positive members in each loop */
  self->positiveLoopMembersCompressedCount = NULL; /*!< containts a compressed
                                                list of loop members with rigid
                                                joints removed */
  self->positiveLoopMemberOffsets = NULL; /*!< Indicates mechanism's local
                                        positive loop member offsets */

  self->negativeLoopMembers = NULL; /*!< Indicates mechanism's negative
                                            loop members */
  self->negativeLoopMembersCompressedCount = NULL; /*!< containts a compressed
                                                list of loop members with rigid
                                                joints removed */
  self->negativeLoopMembersCount =
    NULL; /*!< Count of negative members in each loop */
  self->negativeLoopMemberOffsets = NULL; /*!< Indicates mechanism's negative
                                        loop member offsets */

  self->pv_12 = NULL; /*!< global compressed variable permutation array
                              going from PO2 to PO1 */
  self->pv_21 = NULL; /*!< global compressed variable permutation array
                              going from PO1 to PO2 */
  self->pv_13 = NULL; /*!< global compressed variable permutation array
                              going from PO3 to PO1 */
  self->pv_31 = NULL; /*!< global compressed variable permutation array
                              going from PO1 to PO3 */
  self->pv_32 = NULL; /*!< global compressed variable permutation array
                              going from PO2 to PO3 */
  self->pv_23 = NULL; /*!< global compressed variable permutation array
                              going from PO3 to PO2 */

  /*!
  @}
  @name Pointers to contiguous arrays and pointer arrays of bodies, joints,
  frames and other quantities used at runtime for processing kinematics
  @{
  */
  self->bodyPointers = NULL;
  /*!< Array of pointers to bodies to directly access
       body in position in assembly hierarchy */

  self->pointCloudPointers = NULL;
  /*!< Array of pointers to pointclouds for direct access
       within assembly hierarchy */

  // context->boundingSpheres = NULL;

  self->nonVirtualBodyPointers = NULL;
  /*!< Array of pointers to bodies to directly access
       body in position in assembly hierarchy, does not include
       virtual bodies */

  self->jointPointers = NULL; /*!< Array of pointers to joints to directly
                                     access body in position in assembly
                                     hierarchy */
  self->framePointers = NULL; /*!< array of pointers to frames for
                                     direct access */

  self->lightPointers = NULL; //!< array of pointers to lights

  self->dhFunctionPointers = NULL;
  /*!< function pointers to update displacements
       and influence coefficient matrices in
       spanning tree */

  self->dhvaFunctionPointers = NULL;
  /*!< function pointers to update displacements
      influence coefficient matrices,
      velocities and accelerations in spanning tree */

  self->forceModuleCount = 0;
  self->forceModulePointers = NULL;

  self->names = NULL;

  self->jointTypes = NULL; /*!< Array of joint types, equal in length to joint
                              list */
  self->pcl = NULL;        /*!< Parent Child List, used to navigate mechanism
                                    topology, set automatically by
                                    s2_ProcessTopology */
  self->pclCompressed = NULL;
  self->pcldq = NULL;
  self->isRotation = NULL;

  self->levels = NULL;       /*!< Levels list, set by s2_ProcessTopology */
  self->isReference = NULL;  /*!< an array of booleans that indicate whether the
                           displacement at the current index of context->d is
                           a reference Frame */
  self->originInTree = NULL; /*< an array of booleans equal to
                          number of bodies, true if body origin is in
                          spanning tree */

  self->originPointers = NULL;
  self->referenceFramePointers = NULL;

  self->dWRTReference = NULL; /*!< array of relative displacements,
                                           equal in size to number of shape
                                           matrices + rigid joints*/
  self->dReferenceFrames = NULL;

  self->dcq = NULL; /*!< index of generalized coordinates that have
                      direction cosine terms associated with them */

  self->dhva.dc = NULL; //!< direction cosine arrays
  self->dhva.d = NULL;  /*!< array of displacements equal
                                in size to number of frames */
  self->dhva.h =
    NULL;                       /*!< an array of influence coefficient matrices
                                     equal to the 6 * number of variables in the datastructure */
  self->dhva.v = NULL;          /*!< an array of body velocities equal to the
                             number of variables in the datastructure */
  self->dhva.aq = NULL;         /*!< an array of body accelerations equal to the
                            number of variables in the datastructure */
  // self->dhva.vRelative = NULL;  /*!< an array of body velocities equal to
  //                    the number of variables in the datastructure */
  // self->dhva.aqRelative = NULL; /*!< an array of body accelerations
  //                   equal to the number of variables in the datastructure */
  self->dhvaOffsets = NULL;     /*!< pointers to quantities needed by
                             dhFunctionPointers             and
                             dhvaFunctionPointers, set by     topology_ProcessTopology
                           */

  self->q = NULL; //!< primitive joint displacements
  self->qCopy = NULL;
  self->dq = NULL;  //!< primitive joint velocities
  self->ddq = NULL; //!< primitive joint accelerations

  // context->gamma_p = NULL; /*!< primary loop constraints */
  self->gamma_d = NULL; /*!< preconditioned constraints */
  self->cs = NULL;      /*!< constraints */
  self->dcs = NULL;     /*!< constraint first derivative */
  self->ddcs = NULL;    /*!< constraint second derivative */
  self->ddcsq = NULL;   /*!< constraint second derivative and quadratic terms */

  self->Ml = NULL;  /*!< Generalized mass matrix, linear array containing mass
                      matrix terms on and below the diagonal */
  self->Mls = NULL; /*!< Saved copy of original M */
  self->Ml_size = 0;

  self->p = NULL;         /*!< momentum */
  self->g = NULL;         /*!< body and inertia forces */
  self->k = NULL;         /*!< joint reaction forces */
  self->mhProduct = NULL; /*!< initialize mH product */

  self->nnzm = 0;  /*!< number of nonzero entries in mass matrix */
  self->nnzms = 0; /*!< original number of nonzero entries in mass matrix */
  self->nnzb = 0;  /*!< number of nonzero entries in B matrix */

  self->pcl = NULL; /*!< parent-child list in PO(1) */

  self->Mlb = NULL; /*!< boolean mass matrix, equal in size to M */
  self->Mlbs =
    NULL; /*!< original boolean mass matrix, with no fills recorded */

  self->rm = NULL; /*!< row ranges of M matrix in PO(3) */
  self->cm = NULL; /*!< row ranges of M matrix in PO(3) */

  self->rb2 = NULL;      /*!< row ranges of B matrix in PO(2) */
  self->rb3_temp = NULL; /*!< temporary array used to convert rb2 to rb3 */
  self->rb3 = NULL;      /*!< row ranges of B matrix in PO(3) */

  self->cb2 = NULL;      /*!< global col indices of B matrix in PO(2) */
  self->cb3_temp = NULL; /*!< temporary array used to convert cb2 to cb3*/
  self->cb3 = NULL;      /*!< global col indices of B matrix in PO(3) */
  self->colb = NULL;     /*!< local col indices of B matrix in PO(3) */
  self->ncb = 0;         /*!< number of columns in B matrix */
  self->pb_23 = NULL;    /*!< permutation of linear b matrix from 3 to 2 */
  self->pb_32 = NULL;    /*!< permutation of linear b matrix from 2 to 3 */

  self->ijMapping = NULL;
  self->b3 = NULL; /*!< Linear array containing terms of preconditioned
                     constraints in PO(3) */
  self->f = NULL;  /*!< working forces applied in a joint, equal in length to
                           total number of degrees of freedom */
  self->fl = NULL; /*!< Generalized forces */
  self->la = NULL; /*!< lagrange multipliers */
  self->constraints = NULL;

  /*!
  @}
  @name index and pointer arrays used to "memoize" solution of dynamics
  equations
  @{
  */
  self->i1s =
    0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  self->i2s = 0; /*!< Points to Mij in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  self->i6s =
    0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Mik or Mjk -= Mij*Mik */
  self->i8s = 0; /*!< Points to Mik in Mkj -= Mij*Mik or Mjk -= Mij*Mik */

  /*!< M[i1[l1]] -= M[i2[l1]] * B[i3[l1]]; */
  self->i1 = NULL;
  self->i2 = NULL;
  self->i3 = NULL;

  /*!< M[i4[i]] = 1.0/M[i4[i]];
       M[p] = M[i5[l2]];
       M[i5[l2]] *= M[i4[i]]; */
  self->i4 = NULL;
  self->i5 = NULL;

  /*!< M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */
  self->i6 = NULL;
  self->i7 = NULL;
  self->i8 = NULL;

  /*!< Q[j1[m1]] -= B[j2[m1]]*Q[i];
       ddq[i] -= B[j2[m1]]*ddq[j1[m1]]; */
  self->j1 = NULL;
  self->j2 = NULL;

  /*!< Q[j3[m2]] -= M[j4[m2]]*ga[i];
       la[i] -= M[j4[m2]]*ddq[j3[m2]]; */
  self->j3 = NULL;
  self->j4 = NULL;

  /*!< Q[j5[m3]] -= M[j6[m3]]*Q[i];
       ddq[i] -= M[j6[m3]]*ddq[j5[m3]]; */
  self->j5 = NULL;
  self->j6 = NULL;

  self->p1 =
    NULL; /*!< increments in l1 to process current dependent variable. */
  self->p2 =
    NULL; /*!< increments in m1 to process current dependent variable. */
  self->p3 =
    NULL; /*!< increments in m2 to process current dependent variable. */
  self->n1 =
    NULL; /*!< increments in l2 to process current independent variable. */
  self->n2 =
    NULL; /*!< increments in l3 to process current independent variable. */
  self->n3 =
    NULL; /*!< increments in m3 to process current independent variable. */

  /*! Single array of source and destination memory pointers.
  The dimension of this array is hard coded for initial testing.
  Eventually this decleration will be changed to double **P, and
  sufficient memory for all source and destination memory pointers
  will be malloced. This memory cannot be malloced until the size
  of P has been determined. */
  self->startOfWorkInM = 0;
  self->P = NULL;
  self->p3Start = 0;
  self->rightNullSpace = NULL;
  self->rightNullSpaceCount = 0;
  self->mobilityNumbers = NULL;
  self->mobility = 0;

  /*!
  @}
  @name properties related to input/output datastructures
  @{
  */
  self->io = NULL; /*!< pointer to input/output datastructures */

  /*!
  @}
  @name struct used for numerical integration
  @{
  */

  self->qQuaternionIndices = NULL;
  self->dqQuaternionIndices = NULL;
  self->qNonQuaternionIndices = NULL;
  self->dqNonQuaternionIndices = NULL;
  self->numericalIntegrator = NULL; /*!< pointer to properties
                                            related to numerical
                                            integration */

  /*!
  @}
  @name quantities related to processing inertia
  @{
  */
  self->inertia = NULL;
  self->inertiaP = NULL;

  /*!
  @}
  @name quantities related to processing graphics
  @{
  */
  self->glInitialized = false;
  self->viewport = NULL;
  self->glReferenceFramesf = NULL;
  self->glReferenceFrames = NULL;
  self->shaderHandleLight = NULL;

  self->programLine = -1;
  self->programFlat = -1;
  self->programGouraud = -1;
  self->programPhong = -1;

  self->lightCountUniform = 0;

  /*!
  @}
  @name quantities related to processing graphics
  @{
  */

  self->stiffnessCount = 0;
  self->stiffnessJoints = NULL;

  self->dampingCount = 0;
  self->dampingJoints = NULL;
  /*!
  @}
  */
  self->iterationsSinceLastConstraintError = 0;

  self->terrainCount = 0;
  self->terrains = NULL;

  self->convexhullCount = 0;
  self->convexhulls = NULL;

  self->sphereCount = 0;
  self->spheres = NULL;

  self->ellipsoidCount = 0;
  self->ellipsoids = NULL;

  self->materialCount = 0;
  self->materials = NULL;

  self->collisionObjectCount = 0;
  self->collisionObjects = NULL;

  self->simplexExistsBitfield = NULL;
  self->simplexExistsBitfieldSize = 0;
  self->simplexHead = NULL;
  self->simplexTail = NULL;

  s2_context_setOmegaN(self, 50);

  return self;
}

/*!
@brief used to set omegaN for baumgarte constraint stabilization
*/
void s2_context_setOmegaN(s2_context_t* self, double omegaN) {
  self->modelParameters.baumgarteOmegaN = omegaN;
  self->modelParameters.baumgarte2OmegaN = 2 * omegaN;
  self->modelParameters.baumgarteOmegaN2 = omegaN * omegaN;
}

/*!
@brief Print a Context datastructure
@param context a Context datastructure
*/
void s2_context_print(FILE* f, s2_context_t* context) {
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
@brief Free Displacements, Velocities and Accelerations recursively

Not intended to be called directly, used internally by s2_FreeDVA function

@param context a Context datastructure
@param assembly an Assembly datastructure
*/
int s2_context_freeDVARecursive(s2_context_t* self, s2_assembly_t* assembly) {

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead) {
    if(subAssemblyHead->active) {
      s2_body_t* bodyHead = subAssemblyHead->bodyHead;
      while(bodyHead) {
        s2_frame_t* frameHead = bodyHead->referenceFrame;
        /*
          velocity and acceleration are only stored for the
          reference frame (frameHead).
        */
        if(bodyHead->parentJointPointer->jointType != S2_JOINT_RIGID) {
          s2_screw_copyBack(self->dhva.v + frameHead->vaIndex,
                            &(frameHead->velocityAbsolute));
          s2_screw_copyBack(self->dhva.aq + frameHead->vaIndex,
                            &(frameHead->accelerationQuadraticAbsolute));
        } else {
          frameHead->velocityAbsolute = s2_screw();
          s2_screw_copy(self->dhva.v + frameHead->vaIndex,
                        frameHead->velocityAbsolute);
          frameHead->accelerationQuadraticAbsolute = s2_screw();
          s2_screw_copy(self->dhva.aq + frameHead->vaIndex,
                        frameHead->accelerationQuadraticAbsolute);
        }

        /*
          displacement is stored, computed for all frames
        */
        frameHead = bodyHead->frameHead;
        while(frameHead) {
          if(frameHead->dIndex > -1) {
            s2_displacement_copyBack(self->dhva.d + frameHead->dIndex,
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
void s2_context_freeDHVA(s2_context_t* self) {

  if(self->dhva.dc) {
    s2_frame_t* frameHead = self->assembly->bodyHead->frameHead;
    s2_screw_copyBack(self->dhva.v, &(frameHead->velocityAbsolute));
    s2_screw_copyBack(self->dhva.aq,
                      &(frameHead->accelerationQuadraticAbsolute));

    while(frameHead) {
      if(frameHead->dIndex > -1) {
        s2_displacement_copyBack(self->dhva.d + frameHead->dIndex,
                                 &(frameHead->dAbsolute));
      }
      frameHead->index = -1;
      frameHead->dIndex = -1;
      frameHead = frameHead->next;
    }
    s2_context_freeDVARecursive(self, self->assembly);

    for(int i = 0; i < self->activeJointCount; i++) {
      s2_joint_t* joint = self->jointPointers[i];
      if(joint->dqIndex > -1) {
        s2_screw_copyBack(self->dhva.h + joint->dqIndex, &(joint->h));
      }
    }

    free(self->dhva.dc);
    self->dhva.d = NULL;
    self->dhva.h = NULL;
    self->dhva.v = NULL;
    self->dhva.aq = NULL;

    if(self->dWRTReference) {
      free(self->dWRTReference);
    }
    self->dWRTReference = NULL;
  }
}

/*!
@brief Free direction cosine array and copy values back to their original
Joint datastructures, also copies q and dq back to original datastructures

@param context a Context datastructure
*/
void s2_context_copyBackDc(s2_context_t* self) {

  if(self->dhva.dc) {
    for(int i = 0; i < self->activeJointCount; i++) {
      s2_joint_t* joint = self->jointPointers[i];
      switch(joint->jointType) {
        case S2_JOINT_RX:
        case S2_JOINT_RY:
        case S2_JOINT_RZ: {
          joint->directionCosine = malloc(2 * sizeof(double));
          memcpy(joint->directionCosine, self->dhva.dc + 2 * joint->dcIndex,
                 2 * sizeof(double));
          __attribute__((fallthrough));
        }
        case S2_JOINT_TX:
        case S2_JOINT_TY:
        case S2_JOINT_TZ: {
          joint->q = calloc(1, sizeof(double));
          joint->q[0] = self->q[joint->qIndex];
          joint->dq = calloc(1, sizeof(double));
          joint->dq[0] = self->dq[joint->dqIndex];
        } break;
        case S2_JOINT_RIGID: {

        } break;
        case S2_JOINT_SPHERICAL: {
          joint->q = calloc(4, sizeof(double));
          memcpy(joint->q, self->q + joint->qIndex, 4 * sizeof(double));
          joint->dq = calloc(3, sizeof(double));
          memcpy(joint->dq, self->dq + joint->dqIndex, 3 * sizeof(double));
          break;
        }
        case S2_JOINT_FLOAT: {
          joint->q = calloc(7, sizeof(double));
          memcpy(joint->q, self->q + joint->qIndex, 7 * sizeof(double));
          joint->dq = calloc(6, sizeof(double));
          memcpy(joint->dq, self->dq + joint->dqIndex, 6 * sizeof(double));
          break;
        }
        case S2_JOINT_PRIMITIVE:
        case S2_JOINT_INVALID: {
          break;
        }
      }
    }
  }
}

/*!
@brief Recursively clear Body and Joint Indices

@param assembly an s2_assembly_t datastructure
*/
void s2_clearIndices(s2_assembly_t* assembly) {

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
  while(jointHead) {
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
void s2_context_freeSubstructures(s2_context_t* context) {
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
  if(context->substructures) {
    free(context->substructures);
  }
  context->substructures = NULL;
}

/*!
@brief Free run time arrays in a Context datastructure

Copy values back to original Body, Joint and Frame datastructures and free
run time arrays.

@param context an s2_context_t datastructure
*/
void s2_context_freeRunTimeArrays(s2_context_t* self) {

  // printf("Freeing direction cosines, q and dq\n");
  s2_context_copyBackDc(self);

  // copy values by value back out of D,H,V,A before clearing
  // respective arrays
  // printf("Freeing dhva\n");
  s2_context_freeDHVA(self);

  // printf("Freeing virtual bodies\n");
  for(int i = 0; i < self->activeJointCount; i++) {
    if(self->bodyPointers[i]->isVirtual) {
      s2_virtualbody_free(self->bodyPointers[i]);
    }
  }

  // printf("freeing body pointer array\n");
  if(self->bodyPointers) {
    free(self->bodyPointers);
    self->bodyPointers = NULL;
  }

  // printf("freeing joint pointer array\n");
  if(self->jointPointers) {
    free(self->jointPointers);
    self->jointPointers = NULL;
  }

  // printf("freeing frame pointer array\n");
  if(self->framePointers) {
    free(self->framePointers);
    self->framePointers = NULL;
  }

  if(self->dhFunctionPointers) {
    free(self->dhFunctionPointers);
    self->dhFunctionPointers = NULL;
  }

  if(self->dhvaFunctionPointers) {
    free(self->dhvaFunctionPointers);
    self->dhvaFunctionPointers = NULL;
  }

  if(self->q) {
    free(self->q);
    self->q = NULL;
    self->dq = NULL;
  }

  // printf("freeing jointTypes array\n");
  if(self->jointTypes) {
    free(self->jointTypes);
    self->jointTypes = NULL;
  }

  // printf("freeing parent child list\n");
  if(self->pcl) {
    free(self->pcl);
    self->pcl = NULL;
  }

  // printf("freeing levels list\n");
  if(self->levels) {
    free(self->levels);
    self->levels = NULL;
  }

  // printf("freeing substructureMembers list\n");
  if(self->substructureMembers) {
    for(int i = 0; i < self->substructureCount; i++) {
      free(self->substructureMembers[i]);
    }
    free(self->substructureMembers);
    self->substructureMembers = NULL;
  }

  if(self->substructureMembersCount) {
    free(self->substructureMembersCount);
    self->substructureMembersCount = NULL;
  }

  // printf("freeing chordConstraints\n");
  if(self->chordConstraints) {
    free(self->chordConstraints);
    self->chordConstraints = NULL;
  }

  // printf("freeing isReference\n");
  if(self->isReference) {
    free(self->isReference);
    self->isReference = NULL;
  }

  // printf("freeing positiveLoopMembers\n");
  if(self->positiveLoopMembers) {
    free(self->positiveLoopMembers[0]);
    free(self->positiveLoopMembers);
    self->positiveLoopMembers = NULL;
  }

  // printf("freeing positiveLoopMembersCount\n");
  if(self->positiveLoopMembersCount) {
    free(self->positiveLoopMembersCount);
    self->positiveLoopMembersCount = NULL;
  }

  // printf("freeing negativeLoopMembers\n");
  if(self->negativeLoopMembers) {
    free(self->negativeLoopMembers[0]);
    free(self->negativeLoopMembers);
    self->negativeLoopMembers = NULL;
  }

  // printf("freeing negativeLoopMembersCount\n");
  if(self->negativeLoopMembersCount) {
    free(self->negativeLoopMembersCount);
    self->negativeLoopMembersCount = NULL;
  }

  // printf("freeing glReferenceFramesf\n");
  if(self->glReferenceFramesf) {
    free(self->glReferenceFramesf);
    self->glReferenceFramesf = NULL;
  }

  // printf("freeing glReferenceFrames\n");
  if(self->glReferenceFrames) {
    free(self->glReferenceFrames);
    self->glReferenceFrames = NULL;
  }

  if(self->nonVirtualBodyPointers) {
    free(self->nonVirtualBodyPointers);
    self->nonVirtualBodyPointers = NULL;
  }
  if(self->referenceFramePointers) {
    free(self->referenceFramePointers);
    self->referenceFramePointers = NULL;
  }
  if(self->lightPointers) {
    free(self->lightPointers);
    self->lightPointers = NULL;
  }
  if(self->names) {
    for(int i = 0; i < self->activeFrameInTreeCount; i++) {
      if(self->names[i]) {
        for(int j = 0; j < 4; j++) {
          if(self->names[i][j]) {
            free(self->names[i][j]);
          }
        }
        free(self->names[i]);
      }
    }
    free(self->names);
    self->names = NULL;
  }

  s2_arrayi_free(self->substructureLoopCount);

  self->activeBodyCount = 0;
  self->activeJointCount = 0;
  self->activeFrameCount = 0;
  self->chordConstraintCount = 0;
  self->activeFrameInTreeCount = 0;
  self->activeIOCount = 0;
  self->nonRigidJointCount = 0;
  self->qCount = 0;
  self->dqCount = 0;
  self->dcCount = 0;

  s2_clearIndices(self->assembly);

  s2_context_freeSubstructures(self);
  s2_arrayi_free(self->pv_12);
  s2_arrayi_free(self->pv_21);
  s2_arrayi_free(self->pv_13);
  s2_arrayi_free(self->pv_31);
  s2_arrayi_free(self->pv_32);
  s2_arrayi_free(self->pv_23);
  s2_arrayi_free(self->pcl);
  s2_arrayi_free(self->pclCompressed);
  s2_arrayi_free(self->pcldq);
  s2_arrayb_free(self->isRotation);
  s2_arrayb_free(self->isDependentPO1);
  s2_arrayb_free(self->isDependentPO3);

  s2_arrayi_free(self->cb2);
  s2_arrayi_free(self->cb3_temp);
  s2_arrayi_free(self->cb3);
  s2_arrayi_free2D(self->rb3);
  s2_arrayi_free2D(self->rb3_temp);
  s2_arrayi_free2D(self->rb2);
  s2_arrays_free(self->mhProduct);
  s2_arrayi_free(self->cm);
  s2_arrayi_free(self->colb);
  s2_arrayi_free(self->pb_23);
  s2_arrayi_free(self->pb_32);
  s2_arrayi_free(self->ijMapping);
  s2_array_free(self->b3);
  s2_array_free(self->f);
  s2_array_free(self->fl);
  s2_array_free(self->la);
  s2_arrayi_free(self->i1);

  s2_arrayi_free(self->i2);
  s2_arrayi_free(self->i3);
  s2_arrayi_free(self->i4);
  s2_arrayi_free(self->i5);
  s2_arrayi_free(self->i6);
  s2_arrayi_free(self->i7);
  s2_arrayi_free(self->i8);
  s2_arrayi_free(self->j1);
  s2_arrayi_free(self->j2);
  s2_arrayi_free(self->j3);
  s2_arrayi_free(self->j4);
  s2_arrayi_free(self->j5);
  s2_arrayi_free(self->j6);
  s2_arrayi_free(self->p1);
  s2_arrayi_free(self->p2);
  s2_arrayi_free(self->p3);
  s2_arrayi_free(self->n1);
  s2_arrayi_free(self->n2);
  s2_arrayi_free(self->n3);
  if(self->P) {
    free(self->P);
  }
  self->P = NULL;
  self->p3Start = 0;
  if(self->inertia) {
    free(self->inertia);
  }
  self->inertia = NULL;
  if(self->inertiaP) {
    free(self->inertiaP);
  }
  self->inertiaP = NULL;
  s2_array_free(self->Ml);
  s2_array_free(self->Mls);
  s2_arrayb_free(self->Mlb);
  s2_arrayb_free(self->Mlbs);
  s2_arrayb_free(self->hasMass);
  s2_arrayi_free(self->jointDOF);
  s2_arrayb_free(self->originInTree);
  s2_arrayb_free(self->hasMassdq);
  s2_arrayi_free(self->positiveLoopMembersCompressedCount);
  s2_arrayi_free(self->negativeLoopMembersCompressedCount);
  s2_arrayi_free(self->dqLookupCompressed);
  s2_arrayi_free(self->dqLookup);
  s2_arrayi_free(self->qLookup);
  s2_arrayi_free(self->jointLookup);
  if(self->positiveLoopMembers) {
    s2_arrayi_free2D(self->positiveLoopMemberOffsets);
  }
  if(self->negativeLoopMembers) {
    s2_arrayi_free2D(self->negativeLoopMemberOffsets);
  }
  if(self->constraints) {
    s2_arrayi_free2D(self->constraints);
  }
  s2_arrays_free(self->g);
  if(self->dcq) {
    free(self->dcq);
  }
  self->dcq = NULL;
  if(self->originPointers) {
    free(self->originPointers);
  }
  self->originPointers = NULL;
  s2_array_free(self->ddq);
  if(self->dReferenceFrames) {
    free(self->dReferenceFrames);
  }
  self->dReferenceFrames = NULL;
  s2_arrays_free(self->p);
  s2_array_free(self->gamma_d);
  if(self->glReferenceFramesf) {
    free(self->glReferenceFramesf);
  }
  self->glReferenceFramesf = NULL;
}

/*!
@brief Free frames on the *fixed* inertial reference frame
@param context a Context datastructure
*/
void s2_context_freeFixedFrames(s2_context_t* self) {

  s2_body_t* bodyHead = self->assembly->bodyHead;
  while(bodyHead) {
    s2_body_t* bodyNext = bodyHead->next;
    s2_body_free(bodyHead);
    bodyHead = bodyNext;
  }

  s2_light_t* lightHead = self->assembly->lightHead;
  while(lightHead) {
    s2_light_t* lightNext = lightHead->next;
    s2_light_free(lightHead);
    lightHead = lightNext;
  }

  s2_stackstr_free(self->assembly->name);
  free(self->assembly);
}

/*!
@brief Free a Context datastructure
@param context a Context datastructure
*/
void s2_context_free(s2_context_t* self) {

  if(self) {
    s2_context_freeNumericalIntegrator(self);
    s2_context_freeModelParameters(self);
    s2_context_freeRunTimeArrays(self);

    // printf("freeing assembly\n");
    s2_assembly_free(self->assembly);

    // printf("freeing fixed frames\n");
    s2_context_freeFixedFrames(self);

    s2_arrayi_free(self->qQuaternionIndices);
    s2_arrayi_free(self->dqQuaternionIndices);
    s2_arrayi_free(self->qNonQuaternionIndices);
    s2_arrayi_free(self->dqNonQuaternionIndices);

    free(self);
    self = NULL;
  }
}

/*!
@brief get a frame pointer by specifying root body name and root frame name
@param [in] context data structure
@param [in] root descriptive body name as specified in json file
@param [in] root descriptive frame name as specified in json file
@return frame pointer, returns NULL if none found
*/
s2_frame_t* s2_context_getFrameByName(s2_context_t* self, const char* bodyName,
                                      const char* frameName) {

  for(int i = 0; i < self->activeJointCount; i++) {
    s2_body_t* bodyHead = self->bodyPointers[i];
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
s2_body_t* s2_context_getBodyByName(s2_context_t* self, const char* bodyName) {

  for(int i = 0; i < self->activeJointCount; i++) {
    s2_body_t* bodyHead = self->bodyPointers[i];
    if(!strcmp(bodyName, s2_stackstr_getRoot(bodyHead->name))) {
      return bodyHead;
    }
  }
  return NULL;
}

/*!
@brief get a pointer to debug file pointer

*/
FILE* s2_context_getDebugPointer(s2_context_t* self, int i) {
  if(i == 0) {
    return self->modelParameters.debugFilePointer;
  } else if(i == 1) {
    return self->modelParameters.dhvaFilePointer;
  } else {
    return stdout;
  }
}

/*!
@brief get current simulation time
*/
double s2_context_getSimulationTime(s2_context_t* self) {
  if(self->numericalIntegrator) {
    switch(self->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          self->numericalIntegrator;
        return numericalIntegrator->t;
      } break;
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          self->numericalIntegrator;
        return numericalIntegrator->x;
      } break;
    }
  }
  return -1;
}

/*!
@brief get current iconv value from numerical integrator
*/
int s2_context_getIConv(s2_context_t* self) {
  if(self->numericalIntegrator) {
    switch(self->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          self->numericalIntegrator;
        return numericalIntegrator->iconv;
      } break;
      default: { return -1; }
    }
  }
  return -1;
}

/*!
@brief loads fixed frames into an s2_context_t datastructure
*/
int s2_context_loadFixedFrames(s2_context_t* self, json_t* root) {

  self->assembly->name = s2_stackstr_fromString("fixed");

  s2_body_t* fixedBody = s2_body(s2_stackstr_fromString("fixed"));
  self->assembly->bodyHead = fixedBody;

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
          frame->name, json_string_value(json_object_get(frameObject, "name")));
        json_t* rotation = json_object_get(frameObject, "R");
        if(json_is_array(rotation)) {
          for(int j = 0; j < 3; j++) {
            for(int k = 0; k < 3; k++) {
              s2_displacement_set(frame->dWRTOrigin, j, k,
                                  json_number_value(json_array_get(
                                    json_array_get(rotation, j), k)));
            }
          }
        }
        json_t* translation = json_object_get(frameObject, "T");
        if(json_is_array(translation)) {
          for(int j = 0; j < 3; j++) {
            s2_vector_set(&(frame->dWRTOrigin->T), j,
                          json_number_value(json_array_get(translation, j)));
            // frame->translationWRTReference[j] =
            // frame->translationWRTOrigin[j];
          }
        }

        s2_displacement_invert(frame->dWRTOrigin, frame->dWRTOriginInverse);
        (self->frameCount)++;
        s2_body_appendFrame(fixedBody, frame);
      }
    }
  }
  return 0;
}

/*!
@brief Load model parameters from a Shift JSON file

*/
void s2_context_loadModelParameters(s2_context_t* self,
                                    json_t* modelParametersObject,
                                    const char* filePath) {

  asprintf(
    &(self->modelParameters.version), "%s",
    json_string_value(json_object_get(modelParametersObject, "version")));

  asprintf(&(self->modelParameters.units.angle), "%s",
           json_string_value(json_object_get(
             json_object_get(modelParametersObject, "units"), "angle")));

  asprintf(&(self->modelParameters.units.length), "%s",
           json_string_value(json_object_get(
             json_object_get(modelParametersObject, "units"), "length")));

  asprintf(&(self->modelParameters.units.mass), "%s",
           json_string_value(json_object_get(
             json_object_get(modelParametersObject, "units"), "mass")));

  asprintf(&(self->modelParameters.units.time), "%s",
           json_string_value(json_object_get(
             json_object_get(modelParametersObject, "units"), "time")));

  for(int i = 0; i < 3; i++) {
    ((double*)(self->modelParameters.gravity))[i] = json_number_value(
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
        self->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
      } else if(!strcmp(integrationType, "ode113b")) {
        self->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113B;
      } else {
        /*
          if other integration types are created in the future, add parser for
          them here
        */
        fprintf(
          stderr,
          "Warning, an integration type other than ode113 or ode113b was"
          "specified\n"
          "currently the program only supports ode113. Setting to ode113\n");
        self->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
      };
    } else {
      fprintf(stderr,
              "Warning, no integration type specified. ode113 will be used.\n");
      self->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
    }

    /* load omega N for baumgarte constraint stabilization */
    json_t* omegaNObject = json_object_get(simulationObject, "omega_n");
    double omegaN = 20;
    if(omegaNObject != NULL) {
      omegaN = json_number_value(omegaNObject);
    }
    self->modelParameters.baumgarteOmegaN = omegaN;
    self->modelParameters.baumgarte2OmegaN = 2 * omegaN;
    self->modelParameters.baumgarteOmegaN2 = omegaN * omegaN;

    /* parse time span */
    json_t* timeSpanObject = json_object_get(simulationObject, "time_span");
    if(timeSpanObject != NULL) {
      self->modelParameters.timeSpan = json_number_value(timeSpanObject);
    } else {
      fprintf(stderr, "Warning, no time span specifed in JSON file, "
                      "using 1 second.\n");
      self->modelParameters.timeSpan = 1.0;
    }

    /* parse time step */
    json_t* timeStepObject = json_object_get(simulationObject, "time_step");
    if(timeStepObject != NULL) {
      self->modelParameters.timeStep = json_number_value(timeStepObject);
    } else {
      fprintf(stderr, "Warning, no time step specifed in JSON file, "
                      "using 0.0001 seconds.\n");
      self->modelParameters.timeStep = 0.0001;
    }

    json_t* outFileObject = json_object_get(simulationObject, "out_file");
    if(outFileObject != NULL) {
      asprintf(&(self->modelParameters.outFile), "%s",
               json_string_value(outFileObject));
    }

    json_t* debugFileObject = json_object_get(simulationObject, "debug_file");
    if(debugFileObject != NULL) {
      if(strcmp(json_string_value(debugFileObject), "stdout")) {
        asprintf(&(self->modelParameters.debugFile), "%s",
                 json_string_value(debugFileObject));
        self->modelParameters.debugFilePointer =
          fopen(self->modelParameters.debugFile, "w");
        if(!self->modelParameters.debugFilePointer) {
          fprintf(stderr, "unable to open debug file, exiting.\n");
          exit(-1);
        }
      }
    }

    /* parse relative error */
    json_t* relativeErrorObject =
      json_object_get(simulationObject, "relative_error");
    if(relativeErrorObject != NULL) {
      self->modelParameters.relativeError =
        json_number_value(relativeErrorObject);
    } else {
      fprintf(stderr, "Warning, no relative integration error specifed in JSON "
                      "file, using 0.0 seconds.\n");
      self->modelParameters.relativeError = 0.0;
    }

    /* parse absolute error */
    json_t* absoluteErrorObject =
      json_object_get(simulationObject, "absolute_error");
    if(absoluteErrorObject != NULL) {
      self->modelParameters.absoluteError =
        json_number_value(absoluteErrorObject);
    } else {
      fprintf(stderr, "Warning, no absolute integration error specifed in JSON "
                      "file, using 0.0001 seconds.\n");
      self->modelParameters.absoluteError = 0.0001;
    }

    /* parse infinity norm multiplier */
    json_t* infNormMultiplierObject =
      json_object_get(simulationObject, "infinity_norm_multiplier");
    if(infNormMultiplierObject != NULL) {
      self->modelParameters.pivotRatioMultiple =
        json_number_value(infNormMultiplierObject);
    } else {
      fprintf(stderr, "Warning, no infinity norm multiplier specified in "
                      "file, using 4.\n");
      self->modelParameters.pivotRatioMultiple = 4;
    }

    /* check if contact detection enabled for simulation */
    json_t* contactDetectionObject =
      json_object_get(simulationObject, "contact_detection");
    if(contactDetectionObject) {
      self->modelParameters.contactDetectionEnabled =
        json_boolean_value(contactDetectionObject);
    }

  } else {
    fprintf(stderr, "Warning, no simulation parameters specifed in JSON file, "
                    "using default values\n");
    self->modelParameters.integrationType = S2_NUMERICALINTEGRATOR_ODE113;
    self->modelParameters.timeSpan = 1.0;
    self->modelParameters.timeStep = 0.0001;
    self->modelParameters.relativeError = 0.0;
    self->modelParameters.absoluteError = 1e-4;
  }

  // Load include directories
  json_t* includeDirectoryObject =
    json_object_get(modelParametersObject, "include_dir");

  self->modelParameters.includeDirectory = s2_stackstr();
  char* fullFilePath = s2_getFilePath(filePath);
  if(fullFilePath) {
    s2_stackstr_append(self->modelParameters.includeDirectory, fullFilePath);
    free(fullFilePath);
  }
  if(json_is_array(includeDirectoryObject)) {
    size_t index;
    json_t* value;
    json_array_foreach(includeDirectoryObject, index, value) {
      s2_stackstr_append(self->modelParameters.includeDirectory,
                         json_string_value(value));
    }
  } else if(json_is_string(includeDirectoryObject)) {
    s2_stackstr_append(self->modelParameters.includeDirectory,
                       json_string_value(includeDirectoryObject));
  }
}

/*!
@brief free memory associated with modelParameters substructure
*/
void s2_context_freeModelParameters(s2_context_t* self) {
  if(self->modelParameters.units.angle) {
    free(self->modelParameters.units.angle);
    self->modelParameters.units.angle = NULL;
  }
  if(self->modelParameters.units.length) {
    free(self->modelParameters.units.length);
    self->modelParameters.units.length = NULL;
  }
  if(self->modelParameters.units.mass) {
    free(self->modelParameters.units.mass);
    self->modelParameters.units.mass = NULL;
  }
  if(self->modelParameters.units.time) {
    free(self->modelParameters.units.time);
    self->modelParameters.units.time = NULL;
  }
  if(self->modelParameters.dhvaFilePointer) {
    fclose(self->modelParameters.dhvaFilePointer);
  }
  if(self->modelParameters.debugFilePointer &&
     self->modelParameters.debugFilePointer != stdout) {

    fclose(self->modelParameters.debugFilePointer);
  }

  if(self->modelParameters.version) {
    free(self->modelParameters.version);
    self->modelParameters.version = NULL;
  }
  s2_stackstr_free(self->modelParameters.includeDirectory);

  if(self->modelParameters.gravity) {
    free(self->modelParameters.gravity);
    self->modelParameters.gravity = NULL;
  }
}

/*!
@brief Processes light pointers from a flattened list of lights

Sets parent frame for a light for a flattened list of lights and frames

@param context a Context datastructure
*/
void s2_context_setLightPointers(s2_context_t* self) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "# processing light pointers\n");
  // fprintf(debug, "%d lights\n", context->lightCount);
  for(int i = 0; i < self->lightCount; i++) {
    // fprintf(debug, "\n\nprocessing light ");
    // s2_stackstr_print(debug, context->lightPointers[i]->name);
    //
    // fprintf(debug, "\nsearching for ");
    // s2_stackstr_print(debug, context->lightPointers[i]->parentFrame);

    for(int j = 0; j < self->bodyCount; j++) {
      s2_frame_t* frameHead = self->bodyPointers[j]->frameHead;
      while(frameHead) {
        if(s2_stackstr_isEqual(self->lightPointers[i]->parentFrame,
                               frameHead->name)) {

          // fprintf(debug, "\nmatch found!");
          self->lightPointers[i]->parentFramePointer = frameHead;
        }
        frameHead = frameHead->next;
      }
    }
  }
}

void s2_context_flattenFixedLights(s2_context_t* self, int* lightCounter) {

  s2_light_t* lightHead = self->assembly->lightHead;
  while(lightHead) {
    self->lightPointers[*lightCounter] = lightHead;
    lightHead->index = (*lightCounter);
    (*lightCounter)++;
    lightHead = lightHead->next;
  }
}

void s2_context_flattenHierarchy(s2_assembly_t* assembly, s2_context_t* context,
                                 int* bodyCounter, int* jointCounter,
                                 int* lightCounter) {

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;

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
      s2_context_flattenHierarchy(subAssemblyHead, context, bodyCounter,
                                  jointCounter, lightCounter);
    }
    subAssemblyHead = subAssemblyHead->next;
  }
}

/*!
@brief Loads hierarchical json file into s2_context_t data structure

Recursive function to extract all bodies and joints in
 arbitrarily nested assembly hierarchy

@returns Context
*/
s2_context_t* s2_context_loadJSON(json_t* root, const char* filePath) {

  /* load modelParameters */
  json_t* modelParametersObject = json_object_get(root, "model_parameters");
  if(!json_is_object(modelParametersObject)) {
    fprintf(stderr, "No valid model parameters object found.\n");
    return NULL;
  }

  s2_context_t* context = s2_context();

  s2_context_loadModelParameters(context, modelParametersObject, filePath);

  s2_context_loadFixedFrames(context, root);

  json_t* fixedObject = json_object_get(root, "fixed");

  s2_assembly_loadLights(context->assembly, fixedObject, context,
                         context->assembly->name);

  /* modelParametersObject must be loaded before terrain
  so that include directores can be used to find terrain
  related files */
  s2_terrain_load(context->assembly, fixedObject, context,
                  context->assembly->name);

  json_t* assembliesObject = json_object_get(root, "assemblies");

  if(!json_is_array(assembliesObject)) {
    fprintf(stderr, "No valid assembly object found.\n");
    return NULL;
  }

  s2_stackstr_t* prefix = s2_stackstr();
  s2_assembly_load(context->assembly, assembliesObject, context, prefix);
  s2_stackstr_free(prefix);

  context->bodyPointers = malloc(sizeof(s2_body_t*) * (context->bodyCount + 1));
  context->jointPointers = malloc(sizeof(s2_joint_t*) * context->jointCount);
  if(context->lightCount > 0) {
    context->lightPointers = malloc(sizeof(s2_light_t*) * context->lightCount);
  } else {
    context->lightPointers = NULL;
  }

  for(int i = 0; i < (context->bodyCount + 1); i++) {
    context->bodyPointers[i] = NULL;
  }
  for(int i = 0; i < context->jointCount; i++) {
    context->jointPointers[i] = NULL;
  }
  for(int i = 0; i < context->lightCount; i++) {
    context->lightPointers[i] = NULL;
  }
  context->bodyPointers[0] = context->assembly->bodyHead;

  int bodyCounter = 1;
  int jointCounter = 0;
  int lightCounter = 0;

  s2_context_flattenFixedLights(context, &lightCounter);

  s2_context_flattenHierarchy(context->assembly, context, &bodyCounter,
                              &jointCounter, &lightCounter);

  s2_context_setBodyJointPointers(context);
  s2_context_setLightPointers(context);
  s2_context_setTerrainPointers(context);

  return context;
}

/*!
@brief Parse a Shift JSON file and return in "context", an s2_context_t
datastructure

This function parses a Shift JSON file and returns an s2_context_t
data structure. The JSON file is parsed using the Jansson C library.

@param [in] fileName path to Shift JSON file.
@return a context data structure.
*/
s2_context_t* s2_context_load(const char* fileName) {

  json_error_t error;
  json_t* root = json_load_file(fileName, 0, &error);

  /*
    get path to file name
  */

  if(!root) {
    fprintf(stderr, "Error: file loading error on line %d: %s\n", error.line,
            error.text);
    return NULL;
  }

  if(!json_is_object(root)) {
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
@brief compares names in frames to set pointers
*/
bool s2_connectFramePointer(s2_body_t* body, s2_joint_t* joint,
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
@brief Processes body and joint pointers from a flattened list of bodies and
joints

The flattened list must be generated using s2_LoadFlattenedBodyJointList

@param context a Context datastructure
*/
void s2_context_setBodyJointPointers(s2_context_t* self) {

  for(int i = 0; i < self->jointCount; i++) {
    bool parentFound = false;
    for(int j = 0; j < self->bodyCount + 1; j++) {
      if(s2_stackstr_isEqual(self->jointPointers[i]->parentBody,
                             self->bodyPointers[j]->name)) {

        self->jointPointers[i]->parentBodyPointer = self->bodyPointers[j];
        if(!s2_connectFramePointer(self->bodyPointers[j],
                                   self->jointPointers[i], true)) {
          fprintf(stderr,
                  "Warning, parentFrame not found for joint: %s, frame: %s\n",
                  self->jointPointers[i]->name->tail->value,
                  self->jointPointers[i]->parentFrame->tail->value);
        } else {
          parentFound = true;
        }
        break;
      }
    }
    bool childFound = false;
    for(int j = 0; j < self->bodyCount + 1; j++) {
      if(s2_stackstr_isEqual(self->jointPointers[i]->childBody,
                             self->bodyPointers[j]->name)) {

        self->jointPointers[i]->childBodyPointer = self->bodyPointers[j];
        if(!s2_connectFramePointer(self->bodyPointers[j],
                                   self->jointPointers[i], false)) {
          fprintf(stderr,
                  "Warning, parentFrame not found for joint: %s, frame: %s\n",
                  self->jointPointers[i]->name->tail->value,
                  self->jointPointers[i]->childFrame->tail->value);
        } else {
          childFound = true;
        }
        break;
      }
    }

    if(!parentFound) {
      fprintf(stderr, "Warning, parentBody not found for joint: %s, body: %s\n",
              self->jointPointers[i]->name->tail->value,
              self->jointPointers[i]->childBody->tail->value);
    }

    if(!childFound) {
      fprintf(stderr, "Warning, childBody not found for joint: %s, body: %s\n",
              self->jointPointers[i]->name->tail->value,
              self->jointPointers[i]->childBody->tail->value);
    }
  }
}

/*!
@brief Sets parent frame and body pointers for terrains

@param context an s2_context_t datastructure
*/
void s2_context_setTerrainPointers(s2_context_t* self) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  // fprintf(debug, "# processing elevation map pointers\n");
  for(int i = 0; i < self->terrainCount; i++) {

    for(int j = 0; j < self->bodyCount; j++) {
      s2_frame_t* frameHead = self->bodyPointers[j]->frameHead;
      if(s2_stackstr_isEqual(self->terrains[i]->parentBody,
                             self->bodyPointers[j]->name)) {
        self->terrains[i]->parentBodyPointer = self->bodyPointers[j];
      }

      while(frameHead) {
        if(s2_stackstr_isEqual(self->terrains[i]->parentFrame,
                               frameHead->name)) {
          self->terrains[i]->parentFramePointer = frameHead;
        }
        frameHead = frameHead->next;
      }
    }
  }
}

/*!
@param Evaluate constraint error for each substructure and store sum in
context->modelParameters.sumConstraintError
*/
void s2_context_evaluateConstraintError(s2_context_t* self) {
  self->modelParameters.sumConstraintError = 0;
  for(int j = 0; j < self->substructureCount; j++) {
    s2_substructure_t* substructure = self->substructures + j;
    self->modelParameters.sumConstraintError +=
      s2_substructure_evaluateConstraintError(substructure);
  }
}
