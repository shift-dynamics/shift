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

#ifndef _STAR2_JOINT_H_
#define _STAR2_JOINT_H_

typedef struct s2_body_t s2_body_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_substructure_t s2_substructure_t;
typedef struct s2_context_t s2_context_t;
typedef struct s2_assembly_t s2_assembly_t;
#include "stack.h"
#include "kinematics.h"

/*!
@brief Current supported joint types
*/
typedef enum {
  S2_JOINT_TX,
  S2_JOINT_TY,
  S2_JOINT_TZ,
  S2_JOINT_RX,
  S2_JOINT_RY,
  S2_JOINT_RZ,
  S2_JOINT_RIGID,
  S2_JOINT_SPHERICAL,
  S2_JOINT_FLOAT,
  S2_JOINT_INVALID,
  S2_JOINT_PRIMITIVE
} s2_joint_enum_t;


/*!
@brief Current supported joint types
*/
typedef enum {
  S2_JOINT_INCREMENT,
  S2_JOINT_TARGET
} s2_move_enum_t;


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


s2_joint_t* s2_joint(const char* jointTypeStr);


s2_joint_enum_t s2_getJointType(
  const char* jointTypeStr);


void s2_joint_append(
  s2_assembly_t* assembly,
  s2_joint_t* joint);


void s2_joints_print(
  FILE* f,
  s2_assembly_t* assembly);


void s2_joint_tx(void** dhva);


void s2_joint_txinverse(void** dhva);


void s2_joint_ty(void** dhva);


void s2_joint_tyinverse(void** dhva);


void s2_joint_tz(void** dhva);


void s2_joint_tzinverse(void** dhva);


void s2_joint_tx_dhva(void** dhva);


void s2_joint_txinverse_dhva(void** dhva);


void s2_joint_ty_dhva(void** dhva);


void s2_joint_tyinverse_dhva(void** dhva);


void s2_joint_tz_dhva(void** dhva);


void s2_joint_tzinverse_dhva(void** dhva);


void s2_joint_rx(void** dhva);


void s2_joint_rxinverse(void** dhva);


void s2_joint_ry(void** dhva);


void s2_joint_ryinverse(void** dhva);


void s2_joint_rz(void** dhva);


void s2_joint_rzinverse(void** dhva);


void s2_joint_rx_dhva(void** dhva);


void s2_joint_rxinverse_dhva(void** dhva);


void s2_joint_ry_dhva(void** dhva);


void s2_joint_ryinverse_dhva(void** dhva);


void s2_joint_rz_dhva(void** dhva);


void s2_joint_rzinverse_dhva(void** dhva);


void s2_joint_rigid(void** dhva);


void s2_joint_rigidinverse(void** dhva);


void s2_joint_rigid_dhva(void** dhva);


void s2_joint_rigidinverse_dhva(void** dhva);


void s2_joint_spherical(void** dhva);


void s2_joint_sphericalinverse(void** dhva);


void s2_joint_spherical_dhva(void** dhva);


void s2_joint_sphericalinverse_dhva(void** dhva);


void s2_joint_float(void** dhva);


void s2_joint_floatinverse(void** dhva);


void s2_joint_float_dhva(void** dhva);


void s2_joint_floatinverse_dhva(void** dhva);


void s2_joint_float(void** dhva);


void s2_joint_floatinverse(void** dhva);


void s2_joint_floatfixed(void** dhva);


void s2_joint_floatfixed_dhva(void** dhva);


void s2_joint_free(s2_joint_t* joint);

#endif
