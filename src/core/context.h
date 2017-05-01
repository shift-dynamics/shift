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
#ifndef _STAR2_CONTEXT_H_
#define _STAR2_CONTEXT_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __APPLE__
  #include <OpenGL/OpenGL.h>
#else
  #include <GL/gl.h>
#endif

#include "stack.h"
#include "kinematics.h"

typedef struct s2_forcemodule_t s2_forcemodule_t;
typedef struct s2_context_t s2_context_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_inertia_t s2_inertia_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_substructure_t s2_substructure_t;
typedef struct s2_modelparameters_t s2_modelparameters_t;
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
typedef struct s2_elevationmap_t s2_elevationmap_t;


/*!
@brief Current supported joint types
*/
typedef enum {
  TX,
  TY,
  TZ,
  RX,
  RY,
  RZ,
  RIGID,
  SPHERICAL,
  FLOAT,
  INVALID,
  PRIMITIVE
} s2_joint_enum_t;


/*!
@brief Current supported joint types
*/
typedef enum {
  INCREMENT,
  TARGET
} s2_move_enum_t;


/*!
@brief Current supported numerical integration types

Currently only the Adams-Bashforth-Molton variable time step predictor
correction numerical integrator is supported
*/
typedef enum {
  ODE113, /*!< variable time step Adams-Bashforth-Molton, output is
               interpolated at user specified time intervals */
  ODE113B /*!< variable time step Adams-Bashforth-Molton, output
               is reported at time interval closest to requested time
               interval. */
} s2_numericalintegrator_enum_t;


/*!
@brief types of force modules currently supported
*/
typedef enum {
  TIRE,
  PID
} s2_forcemodule_enum_t;


s2_context_t* s2_context();


s2_assembly_t* s2_assembly();


s2_body_t* s2_body();


s2_joint_t* s2_joint(const char* jointTypeStr);


s2_inertia_t* s2_inertia();


s2_frame_t* s2_frame();


s2_io_t* s2_io();


s2_sensor_t* s2_sensor();


s2_chordconstraint_t* s2_chordconstraint();


s2_jacobian_t* s2_jacobian();


s2_substructure_t* s2_substructure();


s2_units_t* s2_units();


s2_modelparameters_t* s2_modelparameters();


s2_restraint_t* s2_restraint();


s2_light_t* s2_light();


s2_forcemodule_t* s2_forcemodule(s2_forcemodule_enum_t);


s2_material_t* s2_materials(
  int materialCount);


void s2_restraint_append(
  s2_assembly_t* assembly,
  s2_restraint_t* restraint);


void s2_joint_append(
  s2_assembly_t* assembly,
  s2_joint_t* joint);


void s2_body_append(
  s2_assembly_t* assembly,
  s2_body_t* body);


void s2_frame_append(
  s2_body_t* body,
  s2_frame_t* frame);


void s2_assembly_append(
  s2_assembly_t* assembly,
  s2_assembly_t* subAssembly);


void s2_forcemodule_append(
  s2_assembly_t* assembly,
  s2_forcemodule_t* forceModule);


void s2_light_append(
  s2_assembly_t* assembly,
  s2_light_t* light);


void s2_context_free(
  s2_context_t* context);


void s2_modelparameters_free(
  s2_modelparameters_t* modelparameters);


void s2_freeRunTimeArrays(
  s2_context_t* context);


void s2_body_free(
  s2_body_t* body);


void s2_virtualbody_free(
  s2_body_t* body);


void s2_context_print(
  FILE* f,
  s2_context_t* context);


s2_frame_t* s2_context_getFrameByName(
  s2_context_t* context,
  const char* bodyName,
  const char* frameName);


s2_body_t* s2_context_getBodyByName(
  s2_context_t* context,
  const char* bodyName);


int s2_context_getIConv(s2_context_t* context);


double s2_context_getSimulationTime(s2_context_t* context);


FILE* s2_context_getDebugPointer(s2_context_t* context, int i);


/*!
@brief Types of tires
*/
typedef enum {
  PAC1996, /*!< not supported */
  PAC2002, /*!< under development */
  FIALA, /*!< under development */
  PAC2012, /*!< under development */
  LUGRE    /*!< not supported */
} s2_tire_enum_t;


/*!
@brief tire side type
*/
typedef enum {
  LEFT,
  RIGHT
} s2_tireside_enum_t;


typedef struct s2_tire_t s2_tire_t;


typedef struct s2_tire_t {
  s2_tire_enum_t tireType;
  void* properties;
} s2_tire_t;


/*!
@brief Types of distances
*/
typedef enum {
  EUCLIDIAN,
  XDIRECTION,
  YDIRECTION,
  ZDIRECTION
} s2_distance_enum_t;


/*!
@brief Restraint types
*/
typedef enum  {
  SPRING,
  DAMPER
} s2_restraint_enum_t;


/*!
@brief Type of coordinate for specifying restraints
*/
typedef enum {
  FIXED,
  LOCAL
} s2_coordinate_enum_t;


/*!
@brief Light types
*/
typedef enum {
  POINT,
  DIRECTIONAL
} s2_light_enum_t;


typedef enum {
  AMBIENT,
  DIFFUSE,
  SPECULAR,
  EMISSIVE
} s2_color_enum_t;


typedef enum  {
  WIREFRAME,
  FLAT,
  GOURAUD,
  PHONG,
  LINE
} s2_shading_enum_t;


typedef enum {
  PERSPECTIVE,
  ORTHOGRAPHIC
} s2_perspective_enum_t;


typedef enum s2_interpolation_enum_t {
  BILINEAR, /*!< bilinear interpolation is the simplest technique for 2D
  interpolation. The resulting surface is C0 continuous (position continuity is maintained between surface patches, but not tangency and curvature). The surface intersects the control points. */
  BICUBIC,  /*!< bicubic interpolation uses knot points as control points, surface passes near control point, but may not intersect control point, surface is C0, C1 and C2 continuous (position, tangency and curvature continuity are maintained between surface patches) */
  CATMULROM /*!< catmul-rom interpolation, surface passes through control points, but is only C0 and C1 continuous (position and tangency continuity are maintained between surface patches, but not curvature) */
} s2_interpolation_enum_t;


typedef enum s2_coordinate_convention_enum_t {
  ISO, /*!< x is left ordinate of image, y is top ordinate of image, z is pointing up out of height map */
  SAE /*!< x is top ordinate of image, y is left ordinate of image, z is pointing down into height map */
} s2_coordinate_convention_enum_t;


typedef struct s2_elevationmap_t {
  s2_interpolation_enum_t interpolationType; /*!< technique used for 2D interpolation */
  s2_coordinate_convention_enum_t coordinateConvention; /*!< ISO or SAE coordinate system */
  size_t columns;        /*!< number of columns of elevation map */
  size_t rows;           /*!< number of rows in elevation map */
  double xOffset;        /*!< x offset of origin of elevation map */
  double yOffset;        /*!< y offset of origin of elevation map */
  double xScale;         /*!< uniform scale factor for x dimension */
  double yScale;         /*!< uniform scale factor for y dimension */
  double zMin;           /*!< minimum z height in surface */
  double zMax;           /*!< maximum z height in surface */
  double* controlPoints; /*!< control points loaded from image */
  char* texture;         /*!< rgba texture */
  size_t textureRows;    /*!< number of rows in texture */
  size_t textureColumns; /*!< number of columns in texture */
  char* propertyMap;         /*!< rgba texture */
  size_t propertyMapRows;    /*!< number of rows in texture */
  size_t propertyMapColumns; /*!< number of columns in texture */
  double (*getZ)(
    struct s2_elevationmap_t* elevationMap,
    double x,
    double y,
    s2_vector_t* normal); /*!< function used to obtain elevation at specified x, y location. This function pointer is set based on type of interpolation specified when elevationmap is created */
  s2_stackstr_t* name;
  s2_stackstr_t* parentBody;
  s2_body_t* parentBodyPointer;
  s2_stackstr_t* parentFrame;
  s2_frame_t* parentFramePointer;
} s2_elevationmap_t;

/*!
@brief mbdl context datastructure

This datastructure holds all of the values associated with a model. An empty
Context data structure can be initialized using the
s2_InitializeContext function or loaded from a file using the
s2_LoadJSON function.
*/
typedef struct s2_context_t {

  s2_modelparameters_t* modelParameters;
  s2_assembly_t* assembly;

  /*!
  @name Counts of objects in Context data structure
  @{
  */
  int jointCount; //!< total number of joints
  int bodyCount; //!< total number of bodies
  int frameCount; //!< total number of frames
  int lightCount; //!< total number of lights
  int ioCount; //!< total number of input/output devices
  int restraintCount; //<! total number of restraints
  int nonRigidJointCount; /*!< total number of non-rigid joints - includes 1 entry for each multi degree of freedom joint and zero entries for each rigid joint*/
  int dhvaCount;
  int activeFrameCount; //!< number of active frames in mechanism
  int activeFrameInTreeCount; //!< number of active frames in spanning tree
  int activeJointCount; //!< number of active joints
  int activeBodyCount; //!< number of active bodies
  int activeLightCount; //!< number of active lights
  int activeIOCount; //!< number of active devices
  int activeRestraintCount; //!< number of active restraints
  int qCount; /*!< total number of generalized coordinates in system, 7 for
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
  double epsRankDetermination; /*!< tolerance for rank determination in LU decomposition */
  double epsNewtonRaphson; /*!< general tolerance for
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
  double pivotRatioMultiple; /*!< if ratio of pivots increases greater
                                  than this value, LU with full row and
                                  column pivoting will be called again, set
                                  to a default value of 10 */
  double* J; /*!< Jacobian matrix (or matrices) */
  double* Js; /*!< saved copy of jacobian matrices used for debugging purposes */
  int* Jxx; /*!< instructions for evaluating Jacobian matrices */
  bool* Jb; /*!< Boolean Jacobian matrix */
  int jacobianSize; /*!< total size of jacobian matrix */

  bool* isDependentPO1; /*!< equal to dof, in PO1 */
  bool* isDependentPO3; /*! equal to dof, in PO3 */
  bool* hasMass; /*!< size equal to nonRigidJointCount */
  bool* hasMassdq; /*!< size equal to dqCount */
  int dependentCount; /*!< number of dependent variables */

  int* jointDOF; /*!< size equal to number of active joints */

  s2_substructure_t* substructures;
  int substructureCount; /*!< Number of kinematic substructures */
  int* substructureLoopCount; /*!< Number of loops in each
                                   kinematic substructure */
  int** substructureMembers; /*!< members of each kinematic substructure */
  int* substructureMembersCount; /*!< count of members in each substructure */

  int** positiveLoopMembers; /*!< Indicates mechanism's positive loop members */
  int* positiveLoopMembersCount; /*!< Count of positive members in each loop */
  int* positiveLoopMembersCompressedCount; /*!< containts a compressed list
                                                of loop members with rigid
                                                joints removed */
  int** positiveLoopMemberOffsets; /*!< Indicates mechanism's local positive loop
                                        member offsets */

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
  s2_forcemodule_t** forceModulePointers; /*!< Array of pointers to force modules */

  s2_light_t** lightPointers; /*!< array of pointers to lights */

  void (**dhFunctionPointers) (void** dhva);
    /*!< function pointers to update displacements
         and influence coefficient matrices in
         spanning tree */

  void (**dhvaFunctionPointers) (void** dhva);
    /*!< function pointers to update displacements
        influence coefficient matrices,
        velocities and accelerations in spanning tree */

  char*** names;

  s2_joint_enum_t* jointTypes; /*!< Array of joint types, equal in length to
                                      joint list */
  int* pcl; /*!< Parent Child List, used to navigate mechanism
                             topology, set automatically by
                             s2_ProcessTopology */
  int* pclCompressed; /*!< parent child list, with rigid joints
                           compressed out */
  int* pcldq; /*!< parent child list, with rigid joints compressed out and
                   including multi-dof joints */
  bool* isRotation; /*!< Boolean matrix, equal in length to dqCount, true if
                         primitive joint is a rotation, false
                         otherwise */
  int* levels; //!< Levels list, set automatically by s2_ProcessTopology

  bool* isReference; /*!< an array of booleans that indicate whether the
                          displacement at the current index of context->d is
                          a reference Frame */
  bool* originInTree; /*< an array of booleans equal to
                          number of bodies, true if body origin is in
                          spanning tree */

  s2_frame_t** originPointers;
  s2_frame_t** referenceFramePointers;

  s2_displacement_t* d; /*!< array of displacements equal
                               in size to number of frames */
  s2_displacement_t* dWRTReference; /*!< array of relative displacements,
                                           equal in size to number of shape
                                           matrices + rigid joints*/
  s2_displacement_t** dReferenceFrames; /*!< absolute displacement of
                                               reference frames */
  double* dc; //!< direction cosine arrays
  double** dcq; /*!< generalized coordinates that have direction cosine terms
                    associated with them */
  s2_screw_t* h; /*!< an array of influence coefficient matrices equal to the
                  6 * number of variables in the datastructure */
  s2_screw_t* v; /*!< an array of body velocities equal to the
                  number of variables in the datastructure */
  s2_screw_t* aq; /*!< an array of body accelerations equal to the
                  number of variables in the datastructure */
  s2_screw_t* vRelative; /*!< an array of body velocities equal to the
                  number of variables in the datastructure */
  s2_screw_t* aqRelative; /*!< an array of body accelerations equal to the
                  number of variables in the datastructure */
  void*** dhva; /*!< pointers to quantities needed by dhFunctionPointers
                       and dhvaFunctionPointers, set by
                       topology_ProcessTopology */

  double* q; /*!< primitive joint displacements */
  double* qCopy; /*!< copy of primitive joint displacements */
  double* dq; /*!< primitive joint velocities */
  double* ddq; /*!< primitive joint accelerations */

  double* deltaq; /*!< incremental change in q, used in Newton Raphson
                       iteration */
  // double* gamma_p; /*!< loop constraints */
  double* gamma_d; /*!< preconditioned constraints */
  double* cs; /*!< constraints */
  double* dcs; /*!< constraint first derivative */
  double* ddcs; /*!< constraint second derivative */
  double* ddcsq; /*!< constraint second derivative and quadratic terms */

  int Ml_size; /*!< size of Ml array = ((2 + n) * (1 + n)) / 2*/

  double* Ml; /*!< Generalized mass matrix, linear array containing mass
                   matrix terms on and below the diagonal */
  double* Mls; /*!< Saved copy of original M */

  bool* Mlb; /*!< boolean mass matrix, equal in size to M */
  bool* Mlbs; /*!< original boolean mass matrix, with no fills recorded */

  s2_screw_t* p; /*!< momentum */
  s2_screw_t* g; /*!< body and inertia forces */
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
  int nnzm; /*!< number of nonzero entries in mass matrix */
  int nnzms; /*!< original number of nonzero entries in mass matrix */
  int nnzb; /*!< number of nonzero entries in B matrix */

  int** rm; /*!< row ranges of M matrix in PO(3) */
  int* cm; /*!< row ranges of M matrix in PO(3) */

  int** rb2; /*!< row ranges of B matrix in PO(2) */
  int** rb3_temp; /*!< temporary array used to convert rb2 to rb3 */
  int** rb3; /*!< row ranges of B matrix in PO(3) */

  int* cb2; /*!< global col indices of B matrix in PO(2) */
  int* cb3_temp; /*!< temporary array used to convert cb2 to cb3*/
  int* cb3; /*!< global col indices of B matrix in PO(3) */
  int* colb; /*!< local col indices of B matrix in PO(3) */
  int ncb; /*!< number of columns in B matrix */

  int* pb_23; /*!< permutation of linear b matrix from 3 to 2 */
  int* pb_32; /*!< permutation of linear b matrix from 2 to 3 */

  int* ijMapping; /*!< row and column indices in B matrix to process */
  double* b3; /*!< Linear array containing terms of preconditioned constraints
                   in PO(3) */
  double* f; /*!< Working forces within a joint, such as from friction or damping */
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
  double** P;   /*!< Single array of source and destination memory pointers. */
  double p3Start; /*!< start of p3 in P */

  /*!
  @}
  @name quantities related to mobility numbers
  @{
  */
  double* rightNullSpace; /*!< single array of memory allocated for right
                               Null space, equal to sum of dqCount * dqCount
                               for each substructure */
  int rightNullSpaceCount; /*!< total number of entries in rightNullSpace */
  double* mobilityNumbers; /*!< mobility associated with each variable */
  double mobility; /*!< total mobility of system */

  s2_kinematicdrivers_t* kinematicDrivers; /*!< pointer to kinematic drivers,
                                            if present in the mechanism */

  /*!
  @}
  @name quantities related to updating inertia
  @{
  */
  s2_inertia_t* inertia; /*!< inertia expressed at COM WRT body reference
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

  int quaternionCount; /*!< number of quaternions included in generalized
                            coordinates */
  int* qQuaternionIndices; /*! indices where quaternions start in q array */
  int* dqQuaternionIndices; /*! indices where angular velocities start in dq array */

  int nonQuaternionCount; /*! number of indices in q and dq to process that are
                              not associated with quaternions */
  int* qNonQuaternionIndices; /*! indices where non quaternions start in q array */
  int* dqNonQuaternionIndices; /*! indices where non quaternions start in dq array */

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
  @name data related to elevationmaps
  @{
  */

  int elevationMapCount;
  s2_elevationmap_t** elevationmaps;


  /*!
  @}
  */

} s2_context_t;



typedef struct s2_forcemodule_t {

  s2_forcemodule_enum_t forceModuleType;
  void* data;
  void (*update)(s2_context_t* context, void** parameters, double* y);
  void (*saveHistory)(void* parameters);
  void** parameters;
  int yOffset;
  int neqn; /*!< number of states that need to be numerically
    integrated */
  double* y;
  bool useTransient;
  s2_stackstr_t* name;
  s2_forcemodule_t* next;
} s2_forcemodule_t;



/*!
@brief Describes properties related to an Assembly

Each assembly contains a descriptive name, a flag to specify whether the
Assembly is active/inactive and a linked list of:
  - Subassemblies
  - Bodies
  - Joints
  - Lights
  - Restraints
*/
typedef struct s2_assembly_t {
  s2_assembly_t* next; /*!< pointer to next assembly in linked list */
  s2_assembly_t* subAssemblyHead; /*!< pointer to head of subAssembly linked
                                         list datastructure */
  s2_assembly_t* subAssemblyTail; /*!< pointer to tail of subAssembly linked
                                         list datastructure */

  s2_stackstr_t* name;  /*!< Assembly descriptive name */
  bool active; /*!< specifies whether current assembly is active, i.e.
                    true for "switched on", false for "switched off" */
  s2_body_t* bodyHead; /*!< head of Body linked list */
  s2_body_t* bodyTail; /*!< tail of Body linked list */

  s2_joint_t* jointHead; /*!< head of Joint linked list */
  s2_joint_t* jointTail; /*!< tail of Joint linked list */

  s2_light_t* lightHead; /*!< head of Light linked list */
  s2_light_t* lightTail; /*!< tail of Light linked list */

  s2_restraint_t* restraintHead; /*!< head of Restraint linked list */
  s2_restraint_t* restraintTail; /*!< tail of Restraint linked list */

  s2_io_t* ioHead; /*!< head of Device linked list */
  s2_io_t* ioTail; /*!< tail of Device linked list */

  s2_forcemodule_t* forceModuleHead; /*!< head of forcemodule linked list */
  s2_forcemodule_t* forceModuleTail; /*!< tail of forcemodule linked list */
} s2_assembly_t;




/*!
@brief data structure defining a material

*/
typedef struct s2_material_t {
  char* name;
  int index;
  char* filePath;
  s2_shading_enum_t shadingType;

  GLfloat Ka[4]; /*!< ambient color properties */
  GLfloat Kd[4]; /*!< diffuse color properties */
  GLfloat Ks[4]; /*!< specular color properties */
  GLfloat Ke[4]; /*!< emissive color properties */
  GLint Ns; /*!< specular highlight */

  int vertexCount;
  int faceCount;

  GLfloat* vertices;
  GLfloat* vertexNormals;
  GLuint* faces;

  GLuint vertexBuffer; /*!< vertex buffer objects */
  GLuint vertexNormalBuffer;
  GLuint vao; /*!< vertex array objects */
  GLuint ibo; /*!< index buffer objects */

} s2_material_t;


/*!
@brief Describes properties related to a rigid body

A Body datastructure contains:
  - A descriptive name
  - A Frame linked list
  - Pointer to Frame defined as origin

The following attributes are assigned by the s2_ProcessTopology() function:
  - An index, which is used to identify position in minimal *run-time* arrays
  - A level, which indicates level in the spanning tree
  - The unique reference frame, which may not be the origin
*/
typedef struct s2_body_t {
  s2_stackstr_t* name; /*!< Descriptive name of body */

  s2_stackstr_t* definition; /*!< Path to files that define body definitions,
                                    i.e. obj and mtl files */

  s2_assembly_t* assembly; /*!< Pointer to assembly that body is a
                                  member of. */

  s2_frame_t* frameHead; /*!< head of Frame linked list */
  s2_frame_t* frameTail; /*!< tail of Frame linked list */

  s2_frame_t* referenceFrame; /*! pointer to Body's unique reference frame,
                                    which  may not be the Frame's origin.
                                    Assigned automatically by
                                    s2_ProcessTopology */
  s2_frame_t* origin; /*!< A pointer to a body's unique *origin* coordinate
                             system, which may not be the reference frame */

  s2_joint_t* parentJointPointer; /*!< Pointer to Body's unique *parent*
                                         Joint datastructure */

  s2_body_t* parentBodyPointer; /*!< A pointer to the unique *parent* Body,
                                       set by s2_ProcessTopology */

  s2_frame_t* parentFramePointer; /*!< parent frame of body reference frame */
  s2_body_t* next; /*!< Pointer to next Body in linked list datastructure */

  s2_material_t* materials; /*!< s2_material_t datastructure */

  int frameCount; /*!< count of total number of frames */

  int index; /*!< index in contiguous *run-time* array of Bodies, set
                  by s2_ProcessTopology */

  int level; /*!< level in the spanning tree, set
                  by s2_ProcessTopology */

  int materialCount; /*!< number of materials assigned to a body */

  bool isVirtual; /*!< indicates whether the Body is *virtual*. By default,
                       all user-defined bodies are not virtual. Massless
                       virtual bodies are added to simplify processing
                       kinematics if multiple loops are present in the
                       mechanism */

  s2_inertia_t* inertia; /*!< centroidal Mass and moment of Inertia
                                properties in origin coordinate system */

  s2_vector_t* centerOfMassWRTOrigin; /*!< vector from body origin to
                                             center of mass */

  s2_vector_t* centerOfMassWRTReference; /*!< vector from body reference
                                                frame to center of mass */
  s2_mat4x4_t* glReferenceFrame;
  s2_mat4x4f_t* glReferenceFramef;
  s2_mat4x4f_t* glModelView;
  s2_mat4x4f_t* glModelViewProjection;

} s2_body_t;


/*!
@brief properties related to Joints
*/
typedef struct s2_joint_t {

  s2_stackstr_t* name; //!< descriptive name
  int dof; //!< number of degrees of freedom
  int index; //!< index in joint array, set by s2_ProcessTopology
  int qIndex; //!< index into q array
  int dqIndex; //!< index into dq array
  int dcIndex; //!< index into dc array
  int indexSubstructure; //!< local index into substructure
  int sense; /*!< indicates whether joint is positively aligned with spanning
                 tree (+1), or negatively aligned with spanning tree (-1).
                 Automatically set by s2_ProcessTopology */

  int level; /*!< Indicates Joint's level in the spanning tree, automatically
                  set by s2_ProcessTopology */

  s2_joint_enum_t jointType; //!< specifies type of joint
  char* jointTypeStr; //!< descriptive name of joint type

  s2_stackstr_t* parentBody; //!< descriptive name of parent Body
  s2_body_t* parentBodyPointer; //!< pointer to parent Body

  s2_stackstr_t* parentFrame; //!< descriptive name of parent Frame
  s2_frame_t* parentFramePointer; //!< pointer to parent Frame

  s2_stackstr_t* childBody; //!< descriptive name of child Body
  s2_body_t* childBodyPointer; //!< pointer to child Body

  s2_stackstr_t* childFrame; //!< descriptive name of child frame
  s2_frame_t* childFramePointer; //!< pointer to child Frame

  double* initialPrimitiveDisplacement; /*!< initial primitive displacement
                                             specified by user */
  double* initialPrimitiveVelocity; /*!< initial primitive velocity specified
                                         by user */

  double* primitiveDisplacement; /*!< current primitive displacement in
                                     model */
  double* primitiveVelocity; /*!< current primitive velocity in model */

  s2_displacement_t* generalDisplacement; /*!< used for rigid joints and to
                                              set initial conditions for
                                              float and spherical joints */

  double* directionCosine; /*!< array of direction cosines */

  s2_screw_t* velocityRelative;
  s2_screw_t* accelerationQuadraticRelative;
  s2_screw_t* h;

  double* mobilityNumber;
  bool independent; /*!< boolean to indicate whether joint is
                         independent (true) or dependent (false) as
                         determined by fullFactor_LU algorithm */

  s2_joint_t* parentJoint; /*!< pointer to unique parentJoint */
  s2_joint_t* next; /*!< pointer to next Joint in linked-list data structure */

  s2_substructure_t* substructurePointer; /*!< pointer to substructure assignment */

  double* dampingCoefficients; /*!< damping coefficients, c0 + c1*x + c2 * x**2
                                    + ... + cn * x**n */
  int dampingCoefficientCount; /*! number of damping coefficients */

  double* stiffnessCoefficients; /*!< spring coefficients, k0 + k1 * x +
                                   k2 * x**2 + ... + kn * x**n */
  int stiffnessCoefficientCount; /*!< number of spring coefficients */

} s2_joint_t;


/*!
@brief mass and inertia properties
*/
typedef struct s2_inertia_t {
  double m;         /*!< mass */
  s2_vector_t mt; /*!< mass times a vector, t, from centroid to reference
                         frame*/
  double Ixx;       /*!< Ixx centroidal inertia term, expressed in reference
                         frame coordinates */
  double Iyy;       /*!< Ixy centroidal inertia term, expressed in reference
                         frame coordinates */
  double Izz;       /*!< Izz centroidal inertia term, expressed in reference
                         frame coordinates */
  double Ixy;       /*!< Ixy centroidal inertia term, expressed in reference
                         frame coordinates */
  double Iyz;       /*!< Iyz centroidal inertia term, expressed in reference
                         frame coordinates */
  double Izx;       /*!< Izx centroidal inertia term, expressed in reference
                         frame coordinates */
} s2_inertia_t;


/*!
@brief Frame or coordinate system
*/
typedef struct s2_frame_t {
  s2_stackstr_t* name;

  s2_frame_t* referenceFramePointer; /*!< reference frame of the body
                                      set to null if bool isParent = true,
                                      i.e. this frame is the reference
                                      frame */

  s2_frame_t* next; /*!< next frame in linked list */
  s2_frame_t* childFrame;

  s2_displacement_t* dWRTOrigin; /*!< rotation with respect to origin */

  s2_displacement_t* dWRTOriginInverse; /*!< inverse of rotation with respect */

  s2_displacement_t* dWRTReference; /*!< displacement with respect to reference
                                           frame */

  s2_displacement_t* dWRTReferenceInverse; /*!< rotation with respect to
                                                  reference frame inverse */

  s2_displacement_t* dAbsolute; /*!< absolute rotation in inertial reference
                                       frame */

  s2_screw_t* velocityAbsolute; /*!< for a rigid body, the spatial velocity is
                                   constant for all frames on the body, for
                                   flexible bodies, the spatial velocity is
                                   not constant */
  s2_screw_t* accelerationQuadraticAbsolute; /*! for a rigid body, the spatial
                                   acceleration is constant for all frames on
                                   the body, for flexible bodies, the spatial
                                   acceleration is
                                   not constant */

  bool isReference; /*!< designates if the current frame is the reference
                        frame - or is the child of a Joint */

  bool isOrigin; /*!< designates whether the current frame is the origin */
  bool isParent; /*!< designates if the current frame is the parent of a joint */
  bool inTree; /*!< designates if the current frame is either the parent or */
              /*!< child of a joint */
  int index;
  int dIndex; /*!< index into d and dWRTReference arrays*/
  int vaIndex; /*!< index into velocity and acceleration arrays */
} s2_frame_t;


/*!
@brief a sensor datastructure

*/
typedef struct s2_sensor_t {

  s2_sensor_t* next;
} s2_sensor_t;


typedef struct s2_camera_t {

  s2_perspective_enum_t perspectiveType;

  float fieldOfView;
  float aspectRatio;
  float nearClippingPlane;
  float farClippingPlane;

  s2_mat4x4f_t view;
  s2_mat4x4f_t projection;

  s2_camera_t* next;
} s2_camera_t;


/*!
@brief input/output data structure

This structure stores data related to sensors, i.e. cameras, lidar, radar, touch
*/
typedef struct s2_io_t {
  s2_stackstr_t* name;

  int numberSteps;
  int index;
  double deltaDisplacement;

  s2_displacement_t* displacement;

  s2_sensor_t* sensorHead;
  s2_sensor_t* sensorTail;

  s2_camera_t* cameraHead;
  s2_camera_t* cameraTail;

  s2_io_t* next;
} s2_io_t;


/*!
@brief chordConstraintNode properties

If chord loops are detected, a virtual massless body is inserted and a
chordConstraintNode data structure is created for each chord loop
*/
typedef struct s2_chordconstraint_t {
  int level;
  int index;
  int substructure;

  char* name;
  char* parentBody; //!< name of parentBody (a virtual body)

  char* originalParentBody; //!< name of original bodyFrom

  char* parentFrame; //!< name of frameFrom

  char* originalParentFrame; //!< name of original frameFrom

  char* childBody; //!< name of child body
  char* childFrame; //!< name of child frame

  s2_body_t* parentBodyPointer; /*!< pointer to virtual body */
  s2_body_t* originalParentBodyPointer; /*!< pointer to original bodyFrom
                                             (on the nonvirtual body) */
  s2_frame_t* parentFramePointer; /*!< pointer to parentFrame
                                       (on the virtual body) */
  s2_frame_t* originalParentFramePointer; /*!< pointer to original
                                               frameFrom (on the
                                               nonvirtual body) */

  s2_body_t* childBodyPointer; /*!< pointer to body data structure */
  s2_frame_t* childFramePointer; /*!< pointer to frame data structure */

  s2_joint_t* parentJointPointer;
  s2_joint_t* childJointPointer;

  s2_chordconstraint_t* next;
} s2_chordconstraint_t;


/*!
@brief Defines properties related to kinematic substructure or module

At the time of model loading, the topology of a complex mechanism is analyzed
to determine if it would benefit from kinematic substructuring. Kinematic
substructuring effectively block diagonalizes the Jacobian matrix, which
significantly reduces the number of floating point operations required for
generalized coordinate partitioning. If the mechanism is an open loop chain,
there are no chord constraints to process,  and therefore no substructures
are returned. If the mechanism does not benefit from substructuring,
only one substructure is created (containing the entire mechanism.)
A jacobian matrix, a jacobian <i>mapping</i> matrix, and an array of constraint
violations are computed for each kinematic substructure.
*/
typedef struct s2_substructure_t {
  double* J; /*!< pointer to offset in context->J array */
  int* Jxx; /*!< pointer to offset in context->Jxx array */
  bool* Jb; /*!< pointer to offset in context->Jb array */

  int substructureIndex; /*!< index of this substructure */
  int rank;
  int qOffset; /*!< offset in compressed list of generalized positions */
  int dqOffset; /*!< offset in compressed lists of generalized velocities */
  int jointOffset; /*!< offset into compressed list of joints */
  int loopOffset; /*!< offset into global list of loops*/
  int chordConstraintOffset;
  int chordConstraintCount; /*!< number of chord loops
                                 constraint equations associated with
                                 substructure */
  int constraintEquationCount; /*!< total number of constraint equations =
                                    6 * numChordLoops + any additional
                                    constraints imposed by the user */
  int qCount; /*!< number of generalized coordinates in substructure */
  int dqCount; /*!< number of generalized velocities in substructure */
  int jointCount; /*!< number of joints in a substructure, not necessarily
                       equal to the number of variables if a joint has more
                       than one DOF */
  double pivotRatio; /*!< ratio of max / min pivot */
  double scaledInitialInfinityNormB; /*!< context->pivotRatioMultiplier * initial infinity norm of B matrix from an initial factorization of Jacobian */
  double infinityNormB; /*! current infinity norm of B matrix */
  int bOffset; /*!< offset into linear b array */
  int nnzb; /*!< number of nonzero entries in B matrix */
  int nb; /*!< total number of entries in full, dense B matrix */
  int* pc_12; /*!< local column permutation array, permutation goes from PO(1) to
                PO(2) */
  int* pr_12; /*!< local row permutation array, goes from PO(1) to PO(2) */

  int* pc_21; /*!< (pcInverse) local column (variable) permutation array, permutation
                       goes from PO2 to PO1 */
  int* pr_21; /*!< (prInverse) local row (constraint) permutation array, permutation
                       goes from PO2 to PO1 */
  int** constraints; /*!< compressed table of constraints, the value at the
                          first position is the index of the embedded (host)
                          body in aq, the second value is the index of the
                          virtual (released frame) */
  int* ijMapping; /*!< maps positions in right null space of full dense
                        Jacobian matrix to compressed sparse location in b3 */
  double* rightNullSpace; /*!< full right null space matrix */
  double* mobilityNumbers; /*!< sum of squares of each row of orthonormalized
                                null space matrix */

  double* q; /*!< pointer to start of q in global q array */
  double* dq; /*!< pointer to start of dq in global dq array */
  double* cs; /*!< pointer to start of constraints in global constraints array
                   context->cs */
  double* dcs; /*!< pointer to start of time derivative of global constraints
                    array context->dcs */
  double* deltaq; /*!< pointer to start of array in global deltaq, used in
                       newton raphson iteration */

} s2_substructure_t;


/*!
@brief attributes associated with units
*/
typedef struct s2_units_t {
  char* angle; /*!< internally, radians will always be used, valid values
                    are "radians" or "degrees". */
  char* length; /*!< specify distance units to report to user (default is
                     "meters") */
  char* mass; /*!< specify mass units to report to user (default is
                   "kilograms") */
  char* time; /*!< specify time units to report to user (default is
                   "seconds") */
} s2_units_t;


/*!
@brief Attributes associated with force fields, file version,
numerical integration and include directories.
*/
typedef struct s2_modelparameters_t {
  char* version; /*!< specify mbdl file version number */
  s2_stackstr_t* includeDirectory; /*!< specify directories to search for
                                          external files */
  s2_vector_t* gravity; /*!< specify gravity force field, a 3x1 vector with
                               units as specified in the Units datastructure */
  s2_units_t* units; /*!< Refer to the Units data structure definition
                            for more information */

  s2_numericalintegrator_enum_t integrationType; /*!< type of numerical integrator
                                           to use, currently only ODE113
                                           Adams-Bashforth-Molton is
                                           supported */
  int iflag; /*!< interpolation flag for numerical integration, used by
                  ODE113 integrator. The default value is set to
                  -1, which reports simulation results at possibly
                  irregular time intervals (whatever the largest time step
                  ode113 can take). To interpolate simulation results at
                  fixed time intervals (slower), set to 1 */

  double timeSpan; /*!< span of simulation time used for numerical integration.
                         if not specified in JSON file, the default is 1 second */
  double timeStep; /*!< nominal time step used for simulation, if not specified
                         in JSON file, the default is 0.001 seconds */
  double relativeError; /*!< relative error convergence tolerance term,
                             used by ode113. default value if not provided
                             in JSON file is 0.0*/
  double absoluteError; /*!< absolute error convergence tolerance term, used by
                             ode113. default value if not provided in JSON
                             fle is 1e-4 */
  char* outFile; /*!< filename to write resuts to */
  FILE* outFilePointer; /*!< file pointer to write results to */
  char* debugFile; /*!< filename to write debug information to */
  FILE* debugFilePointer; /*!< file pointer to write debug information to */
  FILE* dhvaFilePointer; /*!< file pointer to write dhva debug information to */
  char* delimiter; /*!< delimiter used in writing to file */

  double baumgarteOmegaN; /*!< natural frequency for baumgarte stabilization */
  double baumgarte2OmegaN; /*!< 2 * natural freq for baumgarte constraint stabilization */
  double baumgarteOmegaN2; /*!< omega_N ^2 for baumgarte constraint stabilization */

  bool monitorIterations; /*!< monitor iterations in de */
  bool monitorConstraintError; /*!< monitor constraint error

  */

  double sumConstraintError;
  double maxConstraintError;
  int maxConstraintErrorStep;
  int sumIterationCount;
  int maxIterationCount;
  int maxIterationCountStep;

} s2_modelparameters_t;


/*!
@brief Attributes associated with kinematic drivers, currently not used
*/
typedef struct s2_kinematicdrivers_t {

  struct s2_kinematicdrivers_t* next;
} s2_kinematicdrivers_t;


/*!
@brief Defines a Light
*/
typedef struct s2_light_t {
  s2_stackstr_t* name; /*!< descriptive name of Light */
  s2_stackstr_t* parentFrame; /*!< frame that Light is attached to */
  s2_frame_t* parentFramePointer; /*!< pointer to frame that Light is
                                         attached to */
  s2_stackstr_t* parentBody; /*!< descriptive name of body that Light is
                                    attached to note, set to "fixed" for
                                    inertial reference frame */
  s2_body_t* parentBodyPointer; /*!< pointer to parent body that Light is
                                       light is attached to */
  int index;
  GLuint lightUniform;
  s2_light_enum_t lightType;
  struct glLight {
    GLfloat position[3];
    GLfloat Ka[3];
    GLfloat Kd[3];
    GLfloat Ks[3];
    int directional;
  } glLight;
  struct s2_light_t* next;
} s2_light_t;


/*!
@brief defines a Restraint
*/
typedef struct s2_restraint_t {

  s2_stackstr_t* name;
  double* knots;
  double** coefficients;
  int index;

  s2_restraint_enum_t restraintType;
  s2_coordinate_enum_t coordinateType;
  s2_distance_enum_t distanceType;

  s2_frame_t* parentFrame;
  s2_frame_t* childFrame;

  s2_restraint_t* next;
} s2_restraint_t;

s2_joint_enum_t s2_getJointType(
  const char* jointTypeStr);

void s2_context_setOmegaN(
  s2_context_t* context,
  double omegaN);

#endif
