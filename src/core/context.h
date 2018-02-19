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
#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif
#include <jansson.h>

#include "../ode/numericalintegrator.h"
#include "collisionobject.h"
#include "joint.h"
#include "kinematics.h"
#include "stack.h"

typedef struct s2_forcemodule_t s2_forcemodule_t;
typedef struct s2_context_t s2_context_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_inertia_t s2_inertia_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_substructure_t s2_substructure_t;
typedef struct s2_kinematicdrivers_t s2_kinematicdrivers_t;
typedef struct s2_light_t s2_light_t;
typedef struct s2_restraint_t s2_restraint_t;
typedef struct s2_units_t s2_units_t;
typedef struct s2_driver_t s2_driver_t;
typedef struct s2_chordconstraint_t s2_chordconstraint_t;
typedef struct s2_jacobian_t s2_jacobian_t;
typedef struct s2_io_t s2_io_t;
typedef struct s2_sensor_t s2_sensor_t;
typedef struct s2_camera_t s2_camera_t;
typedef struct s2_material_t s2_material_t;
typedef struct s2_shaderhandle_t s2_shaderhandle_t;
typedef struct s2_terrain_t s2_terrain_t;
typedef struct s2_pointcloud_t s2_pointcloud_t;
typedef struct s2_convexhull_t s2_convexhull_t;
typedef struct s2_sphere_t s2_sphere_t;
typedef struct s2_ellipsoid_t s2_ellipsoid_t;
typedef struct s2_collisionobject_t s2_collisionobject_t;
typedef struct s2_supportmapping_t s2_supportmapping_t;

/*!
@brief shift context datastructure

This datastructure holds all of the values associated with a model. An empty
Context data structure can be initialized using the
s2_InitializeContext function or loaded from a file using the
s2_LoadJSON function.
*/
typedef struct s2_context_t {

  struct modelParameters {
    char* version;                   /*!< specify shift file version number */
    s2_stackstr_t* includeDirectory; /*!< specify directories to search for
                                            external files */
    s2_vector_t*
      gravity; /*!< specify gravity force field, a 3x1 vector with
                      units as specified in the Units datastructure */
    struct units {
      char* angle;
      char* length;
      char* mass;
      char* time;
    } units;

    s2_numericalintegrator_enum_t integrationType; /*!< type of numerical
                                             integrator to use, currently only
                                             ODE113 Adams-Bashforth-Molton is
                                             supported */
    int iflag; /*!< interpolation flag for numerical integration, used by
                    ODE113 integrator. The default value is set to
                    -1, which reports simulation results at possibly
                    irregular time intervals (whatever the largest time step
                    ode113 can take). To interpolate simulation results at
                    fixed time intervals (slower), set to 1 */

    double timeSpan;        /*!< span of simulation time used for numerical
                               integration.        if not specified in JSON file, the
                               default        is 1 second */
    double timeStep;        /*!< nominal time step used for simulation, if not
                               specified        in JSON file, the default is 0.001
                               seconds */
    double relativeError;   /*!< relative error convergence tolerance term,
                                 used by ode113. default value if not provided
                                 in JSON file is 0.0*/
    double absoluteError;   /*!< absolute error convergence tolerance term, used
                               by   ode113. default value if not provided in JSON
                                 fle is 1e-4 */
    char* outFile;          /*!< filename to write resuts to */
    FILE* outFilePointer;   /*!< file pointer to write results to */
    char* debugFile;        /*!< filename to write debug information to */
    FILE* debugFilePointer; /*!< file pointer to write debug information to */
    FILE*
      dhvaFilePointer; /*!< file pointer to write dhva debug information to */
    char* delimiter;   /*!< delimiter used in writing to file */

    double
      baumgarteOmegaN; /*!< natural frequency for baumgarte stabilization */
    double baumgarte2OmegaN; /*!< 2 * natural freq for baumgarte constraint
                                stabilization */
    double baumgarteOmegaN2; /*!< omega_N ^2 for baumgarte constraint
                                stabilization */

    bool monitorIterations;      /*!< monitor iterations in de */
    bool monitorConstraintError; /*!< monitor constraint error */

    double sumConstraintError;
    double maxConstraintError;
    int maxConstraintErrorStep;
    int sumIterationCount;
    int maxIterationCount;
    int maxIterationCountStep;
    double pivotRatioMultiple;

  } modelParameters;

  s2_assembly_t* assembly;

  /*!
  @name Counts of objects in Context data structure
  @{
  */
  int jointCount;         //!< total number of joints
  int bodyCount;          //!< total number of bodies
  int pointCloudCount;    //!< total point cloud count
  int frameCount;         //!< total number of frames
  int lightCount;         //!< total number of lights
  int ioCount;            //!< total number of input/output devices
  int restraintCount;     //<! total number of restraints
  int nonRigidJointCount; /*!< total number of non-rigid joints - includes 1
                             entry for each multi degree of freedom joint and
                             zero entries for each rigid joint*/
  int dhvaCount;
  int activeFrameCount;       //!< number of active frames in mechanism
  int activeFrameInTreeCount; //!< number of active frames in spanning tree
  int activeJointCount;       //!< number of active joints
  int activeBodyCount;        //!< number of active bodies
  int activePointCloudCount;
  int activeLightCount;     //!< number of active lights
  int activeIOCount;        //!< number of active devices
  int activeRestraintCount; //!< number of active restraints
  int qCount;  /*!< total number of generalized coordinates in system, 7 for
                    each FLOAT joint, 4 for each spherical joint
                    0 dof for RIGID joints, and 1 otherwise */
  int dqCount; /*!< total number of generalized velocites in system, 6 for
                   each FLOAT joint, 3 for each spherical joint
                   0 dof for RIGID joints, and 1 otherwise */
  int dcCount; //!< count of direction cosines

  int chordConstraintCount; /*!< number of chord constraints, one for
                                 each kinematic loop */
  int* dqLookup;
  /*!< lookup table used to look up a joint's position in variable
       list. float variables have 6 variables, spherical joints have
       3 variables, and rigid joints
       have 0 variables associated with them */
  int* qLookup;
  int* jointLookup;
  /*!< lookup table used to find a joint's position in compressed joint list
       rigid joints are removed from compressed list and have a value of -1
       in lookup table */
  int* dqLookupCompressed;
  /*!< provides mapping of nonrigid joints to
       index into dq array */

  /*!
  @}
  @name Data related to processing kinematic loops within mechanism
  @{
  */
  double epsRankDetermination; /*!< tolerance for rank determination in LU
                                  decomposition */
  double epsNewtonRaphson;     /*!< general tolerance for
      resolving constraint violation */
  double epsNewtonRaphsonRankDetermination; /*!< tighter newton raphson
                                                 tolerance for the purpose of
                                                 determining mechanism rank */

  s2_chordconstraint_t* chordConstraints; /*!< specified chord constraints, if
                                                 multiple loops are present in a
                                                 mechanism */

  int** constraints; /*!< compressed table of constraints, the value at the
                          first position is the index of the embedded (host)
                          body in aq, the second value is the index of the
                          virtual (released frame) */
  double* J;         /*!< Jacobian matrix (or matrices) */
  double*
    Js;     /*!< saved copy of jacobian matrices used for debugging purposes */
  int* Jxx; /*!< instructions for evaluating Jacobian matrices */
  bool* Jb; /*!< Boolean Jacobian matrix */
  int jacobianSize; /*!< total size of jacobian matrix */

  bool* isDependentPO1; /*!< equal to dof, in PO1 */
  bool* isDependentPO3; /*! equal to dof, in PO3 */
  bool* hasMass;        /*!< size equal to nonRigidJointCount */
  bool* hasMassdq;      /*!< size equal to dqCount */
  int dependentCount;   /*!< number of dependent variables */

  int* jointDOF;           /*!< size equal to number of active joints */
  int* jointDOFCompressed; /*!< size equal to number of non rigid joints */

  s2_substructure_t* substructures;
  int substructureCount;         /*!< Number of kinematic substructures */
  int* substructureLoopCount;    /*!< Number of loops in each
                                      kinematic substructure */
  int** substructureMembers;     /*!< members of each kinematic substructure */
  int* substructureMembersCount; /*!< count of members in each substructure */

  int** positiveLoopMembers; /*!< Indicates mechanism's positive loop members */
  int* positiveLoopMembersCount; /*!< Count of positive members in each loop */
  int* positiveLoopMembersCompressedCount; /*!< containts a compressed list
                                                of loop members with rigid
                                                joints removed */
  int** positiveLoopMemberOffsets; /*!< Indicates mechanism's local positive
                                      loop member offsets */

  int** negativeLoopMembers; /*!< Indicates mechanism's negative loop members */
  int* negativeLoopMembersCompressedCount; /*!< containts a compressed list
                                                of loop members with rigid
                                                joints removed */
  int* negativeLoopMembersCount; /*!< Count of negative members in each loop */
  int** negativeLoopMemberOffsets; /*!< Indicates mechanism's negative loop
                                        member offsets */

  int* pv_12; /*!< global compressed variable permutation array going from
                    PO2 to PO1 */
  int* pv_21; /*!< global compressed variable permutation array going from
                    PO1 to PO2 */
  int* pv_13; /*!< global compressed variable permutation array going from
                    PO3 to PO1 */
  int* pv_31; /*!< global compressed variable permutation array going from
                    PO1 to PO3 */
  int* pv_32; /*!< global compressed variable permutation array going from
                   PO2 to PO3 */
  int* pv_23; /*!< global compressed variable permutation array going from
                   PO3 to PO2 */

  /*!
  @}
  @name Pointers to contiguous arrays and pointer arrays of bodies, joints,
  frames and other quantities used at runtime for processing kinematics
  @{
  */
  s2_body_t** bodyPointers;
  /*!< Array of pointers to bodies to directly access
       body in position in assembly hierarchy */
  s2_pointcloud_t** pointCloudPointers;
  // s2_sphere_t* boundingSpheres; /*! array of bounding spheres */

  s2_body_t** nonVirtualBodyPointers;
  /*!< Array of pointers to bodies to directly access
       body in position in assembly hierarchy, does not include
       virtual bodies */

  s2_joint_t** jointPointers; /*!< Array of pointers to joints to directly
                                     access body in position in assembly
                                     hierarchy */
  s2_frame_t** framePointers; /*!< array of pointers to frames for
                                     direct access */

  int forceModuleCount;
  s2_forcemodule_t**
    forceModulePointers; /*!< Array of pointers to force modules */

  s2_light_t** lightPointers; /*!< array of pointers to lights */

  void (**dhFunctionPointers)(void** dhva);
  /*!< function pointers to update displacements
       and influence coefficient matrices in
       spanning tree */

  void (**dhvaFunctionPointers)(void** dhva);
  /*!< function pointers to update displacements
      influence coefficient matrices,
      velocities and accelerations in spanning tree */

  char*** names;

  s2_joint_enum_t* jointTypes; /*!< Array of joint types, equal in length to
                                      joint list */
  int* pcl;           /*!< Parent Child List, used to navigate mechanism
                                       topology, set automatically by
                                       s2_ProcessTopology */
  int* pclCompressed; /*!< parent child list, with rigid joints
                           compressed out */
  int* pcldq;       /*!< parent child list, with rigid joints compressed out and
                         including multi-dof joints */
  bool* isRotation; /*!< Boolean matrix, equal in length to dqCount, true if
                         primitive joint is a rotation, false
                         otherwise */
  int* levels;      //!< Levels list, set automatically by s2_ProcessTopology

  bool* isReference;  /*!< an array of booleans that indicate whether the
                           displacement at the current index of context->d is
                           a reference Frame */
  bool* originInTree; /*< an array of booleans equal to
                          number of bodies, true if body origin is in
                          spanning tree */

  s2_frame_t** originPointers;
  s2_frame_t** referenceFramePointers;

  s2_displacement_t* d;                 /*!< array of displacements equal
                                               in size to number of frames */
  s2_displacement_t* dWRTReference;     /*!< array of relative displacements,
                                               equal in size to number of shape
                                               matrices + rigid joints*/
  s2_displacement_t** dReferenceFrames; /*!< absolute displacement of
                                               reference frames */
  double* dc;                           //!< direction cosine arrays
  double** dcq;   /*!< generalized coordinates that have direction cosine terms
                      associated with them */
  s2_screw_t* h;  /*!< an array of influence coefficient matrices equal to the
                   6 * number of variables in the datastructure */
  s2_screw_t* v;  /*!< an array of body velocities equal to the
                   number of variables in the datastructure */
  s2_screw_t* aq; /*!< an array of body accelerations equal to the
                  number of variables in the datastructure */
  s2_screw_t* vRelative;  /*!< an array of body velocities equal to the
                   number of variables in the datastructure */
  s2_screw_t* aqRelative; /*!< an array of body accelerations equal to the
                  number of variables in the datastructure */
  void*** dhva; /*!< pointers to quantities needed by dhFunctionPointers
                       and dhvaFunctionPointers, set by
                       topology_ProcessTopology */

  double* q;       /*!< primitive joint displacements */
  double* qCopy;   /*!< copy of primitive joint displacements */
  double* dq;      /*!< primitive joint velocities */
  double* ddq;     /*!< primitive joint accelerations */
  double* deltaq;  /*!< incremental change in q, used in Newton Raphson
                        iteration */
  double* gamma_d; /*!< preconditioned constraints */
  double* cs;      /*!< constraints */
  double* dcs;     /*!< constraint first derivative */
  double* ddcs;    /*!< constraint second derivative */
  double* ddcsq;   /*!< constraint second derivative and quadratic terms */

  int Ml_size; /*!< size of Ml array = ((2 + n) * (1 + n)) / 2*/

  double* Ml;  /*!< Generalized mass matrix, linear array containing mass
                    matrix terms on and below the diagonal */
  double* Mls; /*!< Saved copy of original M */

  bool* Mlb;  /*!< boolean mass matrix, equal in size to M */
  bool* Mlbs; /*!< original boolean mass matrix, with no fills recorded */

  s2_screw_t* p; /*!< momentum */
  s2_screw_t* g; /*!< body and inertial forces, length equal to dq count */
  s2_screw_t* k; /*!< joint reaction forces */
  s2_screw_t* mhProduct; /*!< product of mass matrix and influence
                                coefficient matrices in PO(1) */

  s2_screw_t spatialMomentum; /*!< total spatial momentum expressed in fixed
                                     frame */
  s2_screw_t barycentricSpatialMomentum; /*! total spatial momentum expressed
                                               at origin of barycentric frame */
  double kineticEnergy; /*!< total kinetic energy of the system */
  double gravitationalPotentialEnergy; /*!< total gravitational potential
                                            energy of the system, currently
                                            does not compute potential energy
                                            due to springs, etc. */
  int nnzm;  /*!< number of nonzero entries in mass matrix */
  int nnzms; /*!< original number of nonzero entries in mass matrix */
  int nnzb;  /*!< number of nonzero entries in B matrix */

  int** rm; /*!< row ranges of M matrix in PO(3) */
  int* cm;  /*!< row ranges of M matrix in PO(3) */

  int** rb2;      /*!< row ranges of B matrix in PO(2) */
  int** rb3_temp; /*!< temporary array used to convert rb2 to rb3 */
  int** rb3;      /*!< row ranges of B matrix in PO(3) */

  int* cb2;      /*!< global col indices of B matrix in PO(2) */
  int* cb3_temp; /*!< temporary array used to convert cb2 to cb3*/
  int* cb3;      /*!< global col indices of B matrix in PO(3) */
  int* colb;     /*!< local col indices of B matrix in PO(3) */
  int ncb;       /*!< number of columns in B matrix */

  int* pb_23; /*!< permutation of linear b matrix from 3 to 2 */
  int* pb_32; /*!< permutation of linear b matrix from 2 to 3 */

  int* ijMapping; /*!< row and column indices in B matrix to process */
  double* b3; /*!< Linear array containing terms of preconditioned constraints
                   in PO(3) */
  double*
    f; /*!< Working forces within a joint, such as from friction or damping */
  double* fl; /*!< Generalized forces */

  // double* ga; /*!< quadratic acceleration terms */
  // double* gas; /*!< Saved copy of original ga */
  double* la; /*!< lagrange multipliers */

  int i1s; /*!< Points to Mkj or Mjk in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i2s; /*!< Points to Mij in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i6s; /*!< Points to Mkj or Mjk in Mkj -= Mij*Mik or Mjk -= Mij*Mik */
  int i8s; /*!< Points to Mik in Mkj -= Mij*Mik or Mjk -= Mij*Mik */

  /*!< M[i1[l1]] -= M[i2[l1]] * B[i3[l1]]; */
  int* i1;
  int* i2;
  int* i3;

  /*!< M[i4[i]] = 1.0/M[i4[i]];
       M[p] = M[i5[l2]];
       M[i5[l2]] *= M[i4[i]]; */
  int* i4;
  int* i5;

  /*!< M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */
  int* i6;
  int* i7;
  int* i8;

  /*!< Q[j1[m1]] -= B[j2[m1]]*Q[i];
       ddq[i] -= B[j2[m1]]*ddq[j1[m1]]; */
  int* j1;
  int* j2;

  /*!< Q[j3[m2]] -= M[j4[m2]]*ga[i];
       la[i] -= M[j4[m2]]*ddq[j3[m2]]; */
  int* j3;
  int* j4;

  /*!< Q[j5[m3]] -= M[j6[m3]]*Q[i];
       ddq[i] -= M[j6[m3]]*ddq[j5[m3]]; */
  int* j5;
  int* j6;

  int* p1; /*!< increments in l1 to process current dependent variable. */
  int* p2; /*!< increments in m1 to process current dependent variable. */
  int* p3; /*!< increments in m2 to process current dependent variable. */
  int* n1; /*!< increments in l2 to process current independent variable. */
  int* n2; /*!< increments in l3 to process current independent variable. */
  int* n3; /*!< increments in m3 to process current independent variable. */

  int startOfWorkInM; /*!< n(n+1)/2 points to first entry in temporary storage
                           part of Ml[] */
  double** P; /*!< Single array of source and destination memory pointers. */
  double p3Start; /*!< start of p3 in P */

  /*!
  @}
  @name quantities related to mobility numbers
  @{
  */
  double* rightNullSpace;  /*!< single array of memory allocated for right
                                Null space, equal to sum of dqCount * dqCount
                                for each substructure */
  int rightNullSpaceCount; /*!< total number of entries in rightNullSpace */
  double* mobilityNumbers; /*!< mobility associated with each variable */
  double mobility;         /*!< total mobility of system */

  s2_io_t* io; /*!< pointer to input/output datastructures */

  /*!
  @}
  @name quantities related to updating inertia
  @{
  */
  s2_inertia_t* inertia;  /*!< inertia expressed at COM WRT body reference
                                 frame, this inertia may differ from
                                 inertia on a body, as any bodies connected by
                                 rigid joints are merged together and expressed
                                 in terms of their common centroidal
                                 inertial reference frame with respect to the
                                 reference frame of the parent body */
  s2_inertia_t* inertiaP; /*! accumulated or "projected" inertia matrices
                                used in solving dynamics equations */

  /*!
  @}
  @name quantities related to processing graphics
  @{
  */
  bool glInitialized;
  s2_camera_t* viewport;
  s2_mat4x4f_t* glReferenceFramesf;
  s2_mat4x4_t* glReferenceFrames;
  s2_shaderhandle_t* shaderHandleLight;

  GLuint programLine;
  GLuint programFlat;
  GLuint programGouraud;
  GLuint programPhong;

  GLuint lightCountUniform;

  /*!
  @}
  @name quantities related to converting angular velocities to time derivative
  of quaternions
  @{
  */

  int quaternionCount;      /*!< number of quaternions included in generalized
                                 coordinates */
  int* qQuaternionIndices;  /*! indices where quaternions start in q array */
  int* dqQuaternionIndices; /*! indices where angular velocities start in dq
                               array */

  int nonQuaternionCount; /*! number of indices in q and dq to process that are
                              not associated with quaternions */
  int*
    qNonQuaternionIndices; /*! indices where non quaternions start in q array */
  int* dqNonQuaternionIndices; /*! indices where non quaternions start in dq
                                  array */

  /*!
  @}
  @name quantities related to numerical integration
  @{
  */

  void* numericalIntegrator; /*!< pointer to properties
                                                  related to numerical
                                                  integration */
  int iterationsSinceLastConstraintError;

  /*!
  @}
  @name data related to setting restraints
  @{
  */
  int dampingCount;
  s2_joint_t** dampingJoints;

  int stiffnessCount;
  s2_joint_t** stiffnessJoints;

  /*!
  @}
  @name data related to terrain maps
  @{
  */

  int terrainCount;
  s2_terrain_t** terrains;

  /*!
  @}
  */

  /*!
  @name data related to collision geometry
  */
  int sphereCount;
  s2_sphere_t* spheres;

  int convexhullCount;
  s2_convexhull_t* convexhulls;

  int ellipsoidCount;
  s2_ellipsoid_t* ellipsoids;

  int collisionObjectCount;
  s2_collisionobject_t* collisionObjects; /*!< contains pointers to a fixed
                                             number of collision objects */

  s2_supportmapping_t*
    supportmappingHead; /*!< head of suportmapping doubly linked list */
  s2_supportmapping_t*
    supportmappingTail; /*!< tail of supportmapping doubly linked list */

  int materialCount;
  s2_material_t* materials;
  /*!
  @}
  */

} s2_context_t;

s2_context_t* s2_context();

s2_units_t* s2_units();

void s2_context_free(s2_context_t* context);

void s2_context_freeRunTimeArrays(s2_context_t* context);

void s2_context_print(FILE* f, s2_context_t* context);

s2_frame_t* s2_context_getFrameByName(s2_context_t* context,
                                      const char* bodyName,
                                      const char* frameName);

s2_body_t* s2_context_getBodyByName(s2_context_t* context,
                                    const char* bodyName);

int s2_context_getIConv(s2_context_t* context);

double s2_context_getSimulationTime(s2_context_t* context);

FILE* s2_context_getDebugPointer(s2_context_t* context, int i);

void s2_context_setOmegaN(s2_context_t* context, double omegaN);

int s2_context_loadFixedFrames(s2_context_t* context, json_t* root);

void s2_context_loadModelParameters(s2_context_t* context,
                                    json_t* modelParametersObject,
                                    const char* filePath);

void s2_context_freeModelParameters(s2_context_t*);

s2_context_t* s2_context_load(const char*);

void s2_context_setTerrainPointers(s2_context_t*);

void s2_context_setBoundingSpheres(s2_context_t*);

void s2_context_updateBoundingSpheres(s2_context_t*);

void s2_context_pushCollisionGeometry(s2_context_t*, void*,
                                      s2_collisionobject_enum_t);

#endif
