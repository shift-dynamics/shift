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
#include "joint.h"
#include "assembly.h"
#include "body.h"
#include "context.h"
#include "kinematics.h"

/*!
@brief Initialize a Joint datastructure

@param None
@returns a Joint datastructure
*/
s2_joint_t* s2_joint(const char* jointTypeStr, s2_stackstr_t* name) {

  s2_joint_t* joint = malloc(sizeof(s2_joint_t));

  if(!joint) {
    fprintf(stderr, "Error: failed to initialize joint.\n");
    return NULL;
  }
  joint->jointTypeStr = strdup(jointTypeStr);
  joint->jointType = s2_getJointType(jointTypeStr);

  joint->name = name;

  joint->index = -1;
  joint->parentBody = NULL;
  joint->parentFrame = NULL;
  joint->childBody = NULL;
  joint->childFrame = NULL;
  joint->sense = 0;
  joint->level = -1;
  joint->stiffnessCoefficients = NULL;
  joint->stiffnessCoefficientCount = 0;
  joint->dampingCoefficients = NULL;
  joint->dampingCoefficientCount = 0;
  joint->isVisible = false;

  switch(joint->jointType) {
    case S2_JOINT_TX:
    case S2_JOINT_TY:
    case S2_JOINT_TZ: {
      joint->dof = 1;
      joint->initialPrimitiveDisplacement = calloc(1, sizeof(double));
      joint->initialPrimitiveVelocity = calloc(1, sizeof(double));

      joint->primitiveDisplacement = calloc(1, sizeof(double));
      joint->primitiveVelocity = calloc(1, sizeof(double));

      joint->generalDisplacement = NULL;
      joint->directionCosine = NULL;

      joint->velocityRelative = s2_screw();
      joint->accelerationQuadraticRelative = s2_screw();

      joint->h = s2_screw();
      break;
    }
    case S2_JOINT_RX:
    case S2_JOINT_RY:
    case S2_JOINT_RZ: {
      joint->dof = 1;
      joint->initialPrimitiveDisplacement = calloc(1, sizeof(double));
      joint->initialPrimitiveVelocity = calloc(1, sizeof(double));

      joint->primitiveDisplacement = calloc(1, sizeof(double));
      joint->primitiveVelocity = calloc(1, sizeof(double));

      joint->generalDisplacement = NULL;
      joint->directionCosine = calloc(2, sizeof(double));

      joint->velocityRelative = s2_screw();
      joint->accelerationQuadraticRelative = s2_screw();

      joint->h = s2_screw();
      break;
    }
    case S2_JOINT_RIGID: {
      joint->dof = 0;
      joint->initialPrimitiveDisplacement = NULL;
      joint->initialPrimitiveVelocity = NULL;

      joint->primitiveDisplacement = NULL;
      joint->primitiveVelocity = NULL;
      joint->directionCosine = NULL;

      joint->generalDisplacement = s2_displacement();

      joint->velocityRelative = s2_screw();
      joint->accelerationQuadraticRelative = s2_screw();

      joint->h = NULL;
      break;
    }
    case S2_JOINT_SPHERICAL: {
      joint->dof = 3;
      joint->initialPrimitiveDisplacement = calloc(4, sizeof(double));
      joint->initialPrimitiveVelocity = calloc(3, sizeof(double));

      joint->primitiveDisplacement = calloc(4, sizeof(double));
      joint->primitiveVelocity = calloc(3, sizeof(double));

      joint->generalDisplacement = s2_displacement();
      joint->directionCosine = NULL;

      joint->velocityRelative = s2_screw();
      joint->accelerationQuadraticRelative = s2_screw();

      joint->h = calloc(3, sizeof(s2_screw_t));
      break;
    }
    case S2_JOINT_FLOAT: {
      joint->dof = 6;
      joint->initialPrimitiveDisplacement = calloc(7, sizeof(double));
      joint->initialPrimitiveVelocity = calloc(6, sizeof(double));

      /* a float joint has primitive displacement specified as
      translation in the first 3 terms, followed by orientation specified
      as quaternions in the last four terms */
      joint->primitiveDisplacement = calloc(7, sizeof(double));

      /* velocity is specified as translational velocity in the first three
      terms and angular velocity in the last three terms */
      joint->primitiveVelocity = calloc(6, sizeof(double));

      joint->generalDisplacement = s2_displacement();
      joint->directionCosine = NULL;

      joint->velocityRelative = s2_screw();
      joint->accelerationQuadraticRelative = s2_screw();

      joint->h = calloc(6, sizeof(s2_screw_t));
      break;
    }
    case S2_JOINT_INVALID:
    case S2_JOINT_PRIMITIVE: {
      fprintf(stderr, "invalid joint type found\n");
      break;
    }
  }

  joint->mobilityNumber = NULL;
  joint->independent = false;

  joint->parentJoint = NULL;
  joint->next = NULL;

  joint->substructurePointer = NULL;
  return joint;
}

/*!
@brief Converts jointType descriptive name to JOINTTYPE enum

@param jointType descriptive name
@return JOINTTYPE enum
*/
s2_joint_enum_t s2_getJointType(const char* jointTypeStr) {

  if(!strcmp(jointTypeStr, "Tx")) {
    return S2_JOINT_TX;
  } else if(!strcmp(jointTypeStr, "Ty")) {
    return S2_JOINT_TY;
  } else if(!strcmp(jointTypeStr, "Tz")) {
    return S2_JOINT_TZ;
  } else if(!strcmp(jointTypeStr, "Rx")) {
    return S2_JOINT_RX;
  } else if(!strcmp(jointTypeStr, "Ry")) {
    return S2_JOINT_RY;
  } else if(!strcmp(jointTypeStr, "Rz")) {
    return S2_JOINT_RZ;
  } else if(!strcmp(jointTypeStr, "rigid")) {
    return S2_JOINT_RIGID;
  } else if(!strcmp(jointTypeStr, "spherical")) {
    return S2_JOINT_SPHERICAL;
  } else if(!strcmp(jointTypeStr, "float")) {
    return S2_JOINT_FLOAT;
  }
  return S2_JOINT_INVALID;
}

/*!
@brief Prismatic joint along x-axis (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tx(void** dhva) {
  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtx(dParent, dPrimitive, dChild);
  s2_htx(dChild, h);
}

/*!
@brief @brief Prismatic joint along x-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

The dhva array contains pointers to the following quantities:

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tx_dhva(void** dhva) {
  s2_joint_tx(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_sst(vParent, vRelative, vChild);
  s2_screwcross_sst(vParent, vRelative, aqRelative);
  s2_add_sst(aqParent, aqRelative, aqChild);
}

/*!
@brief Prismatic Joint aligned with x-axis - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_txinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtxinverse(dParent, dPrimitive, dChild);
  s2_htxinverse(dChild, h);
}

/*!
@brief Prismatic joint along x-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_txinverse_dhva(void** dhva) {

  s2_joint_txinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_sst(vParent, vRelative, vChild);
  s2_screwcross_sst(vParent, vRelative, aqRelative);
  s2_add_sst(aqParent, aqRelative, aqChild);
}

/*!
@brief Prismatic Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_ty(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dty(dParent, dPrimitive, dChild);
  s2_hty(dChild, h);
}

/*!
@brief Prismatic joint along y-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_ty_dhva(void** dhva) {

  s2_joint_ty(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_sst(vParent, vRelative, vChild);
  s2_screwcross_sst(vParent, vRelative, aqRelative);
  s2_add_sst(aqParent, aqRelative, aqChild);
}

/*!
@brief Prismatic Joint aligned with y-axis - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tyinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtyinverse(dParent, dPrimitive, dChild);
  s2_htyinverse(dChild, h);
}

/*!
@brief @brief Prismatic joint along y-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tyinverse_dhva(void** dhva) {

  s2_joint_tyinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_sst(vParent, vRelative, vChild);
  s2_screwcross_sst(vParent, vRelative, aqRelative);
  s2_add_sst(aqParent, aqRelative, aqChild);
}

/*!
@brief Prismatic Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tz(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtz(dParent, dPrimitive, dChild);
  s2_htz(dChild, h);
}

/*!
@brief Prismatic joint along z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tz_dhva(void** dhva) {

  s2_joint_tz(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_sst(vParent, vRelative, vChild);
  s2_screwcross_sst(vParent, vRelative, aqRelative);
  s2_add_sst(aqParent, aqRelative, aqChild);
}

/*!
@brief Prismatic Joint aligned with z-axis - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tzinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtzinverse(dParent, dPrimitive, dChild);
  s2_htzinverse(dChild, h);
}

/*!
@brief Prismatic joint along z-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_tzinverse_dhva(void** dhva) {

  s2_joint_tzinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_sst(vParent, vRelative, vChild);
  s2_screwcross_sst(vParent, vRelative, aqRelative);
  s2_add_sst(aqParent, aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with x-axis (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dc = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rx(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drx(dParent, dc, dChild);
  s2_hrx(dChild, h);
}

/*!
@brief Revolute joint aligned with x-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rx_dhva(void** dhva) {

  s2_joint_rx(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with x-axis - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dc = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rxinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drxinverse(dParent, dc, dChild);
  s2_hrxinverse(dChild, h);
}

/*!
@brief Revolute joint aligned with x-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rxinverse_dhva(void** dhva) {

  s2_joint_rxinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dc = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_ry(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dry(dParent, dc, dChild);
  s2_hry(dChild, h);
}

/*!
@brief Revolute joint aligned with y-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_ry_dhva(void** dhva) {

  s2_joint_ry(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with y-axis - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dc = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_ryinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];

  s2_screw_t* h = dhva[3];

  s2_product_dryinverse(dParent, dc, dChild);
  s2_hryinverse(dChild, h);
}

/*!
@brief Revolute joint aligned with y-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_ryinverse_dhva(void** dhva) {

  s2_joint_ryinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dc = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rz(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drz(dParent, dc, dChild);
  s2_hrz(dChild, h);
}

/*!
@brief Revolute joint aligned with z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rz_dhva(void** dhva) {
  s2_joint_rz(dhva);

  s2_screw_t* h = dhva[3];
  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];
  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with z-axis - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    double* dc = dhva[1];
    s2_displacement_t* dChild = dhva[2];
    s2_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rzinverse(void** dhva) {
  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drzinverse(dParent, dc, dChild);
  s2_hrzinverse(dChild, h);
}

/*!
@brief Revolute joint aligned with z-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* h = dhva[3];

    s2_screw_t* vParent = dhva[4];
    double* vPrimitive = dhva[5];
    s2_screw_t* vRelative = dhva[6];
    s2_screw_t* vChild = dhva[7];

    s2_screw_t* aqParent = dhva[8];
    s2_screw_t* aqRelative = dhva[9];
    s2_screw_t* aqChild = dhva[10];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rzinverse_dhva(void** dhva) {

  s2_joint_rzinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(h, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Rigid Joint (dh)

Updates displacement and influence coefficient matrices

    Displacement dChild = dhva[0];

    Displacement dParent = dhva[1];

    Displacement dPrimitive = dhva[2];

    double* dc = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vChild = dhva[6];

    double* vPrimitive = dhva[7];

    s2_screw_t* vRelative = dhva[8];

    s2_screw_t* aqChild = dhva[9];

    s2_screw_t* aqParent = dhva[10];

    s2_screw_t* aqRelative = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rigid(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_displacement_t* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];

  s2_product_dd(dParent, dPrimitive, dChild);
}

/*!
@brief Rigid Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* vParent = dhva[3];
    s2_screw_t* vChild = dhva[4];
    s2_screw_t* aqChild = dhva[5];
    s2_screw_t* aqParent = dhva[6];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rigid_dhva(void** dhva) {

  s2_joint_rigid(dhva);

  s2_screw_t* vParent = dhva[3];
  s2_screw_t* vChild = dhva[4];
  s2_screw_t* aqChild = dhva[5];
  s2_screw_t* aqParent = dhva[6];

  s2_screw_copy(vParent, vChild);
  s2_screw_copy(aqParent, aqChild);
}

/*!
@brief Rigid Joint - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];
    s2_displacement_t* dPrimitive = dhva[1];
    s2_displacement_t* dChild = dhva[2];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rigidinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_displacement_t* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];

  s2_product_ddinverse(dParent, dPrimitive, dChild);
}

/*!
@brief Rigid Joint - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_screw_t* vParent = dhva[3];
    s2_screw_t* vChild = dhva[4];
    s2_screw_t* aqChild = dhva[5];
    s2_screw_t* aqParent = dhva[6];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_rigidinverse_dhva(void** dhva) {

  s2_joint_rigidinverse(dhva);

  s2_screw_t* vParent = dhva[3];
  s2_screw_t* vChild = dhva[4];
  s2_screw_t* aqChild = dhva[5];
  s2_screw_t* aqParent = dhva[6];
  s2_screw_copy(vParent, vChild);
  s2_screw_copy(aqParent, aqChild);
}

/*!
@brief Rigid Joint (dh)

Updates displacement and influence coefficient matrices

s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_float(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_vector_t* t = dhva[1];
  s2_quaternion_t* q = (s2_quaternion_t*)(((double*)dhva[1]) + 3);
  s2_displacement_t* dWRTReference = dhva[2];
  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));
  s2_product_rr(&(dParent->R), &(dWRTReference->R), &(dChild->R));
  s2_product_rv(&(dParent->R), t, &(dChild->T));
  cblas_daxpy(3, 1.0, ((double*)dParent) + 9, 1, ((double*)dChild) + 9, 1);
  s2_htx(dChild, h);
  s2_hty(dChild, h + 1);
  s2_htz(dChild, h + 2);
  s2_hrx(dChild, h + 3);
  s2_hry(dChild, h + 4);
  s2_hrz(dChild, h + 5);
}

/*!
@brief Float Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_float_dhva(void** dhva) {
  s2_joint_float(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_screw_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_screwproduct_ds(dChild, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Float Joint - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_floatinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_vector_t* t = dhva[1];
  s2_quaternion_t* q = (s2_quaternion_t*)(((double*)dhva[1]) + 3);
  s2_displacement_t* dWRTReference = dhva[2];
  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));
  s2_product_rrinverse(&(dParent->R), &(dWRTReference->R), &(dChild->R));
  s2_product_rvinverse(&(dParent->R), t, &(dChild->T));
  cblas_daxpy(3, 1.0, ((double*)dParent) + 9, 1, ((double*)dChild) + 9, 1);
  s2_htxinverse(dChild, h);
  s2_htyinverse(dChild, h + 1);
  s2_htzinverse(dChild, h + 2);
  s2_hrxinverse(dChild, h + 3);
  s2_hryinverse(dChild, h + 4);
  s2_hrzinverse(dChild, h + 5);
}

/*!
@brief Float Joint - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_floatinverse_dhva(void** dhva) {
  s2_joint_floatinverse(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_screw_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_screwproduct_dsinverse(dChild, vPrimitive, vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Spherical Joint (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_spherical(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_quaternion_t* q = dhva[1];
  s2_displacement_t* dWRTReference = dhva[2];
  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));
  s2_product_dr(dParent, &(dWRTReference->R), dChild);
  s2_vector_copy(&dParent->T, &dChild->T);

  s2_hrx(dChild, h);
  s2_hry(dChild, h + 1);
  s2_hrz(dChild, h + 2);
}

/*!
@brief Spherical Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
@todo add spherical joint
*/
void s2_joint_spherical_dhva(void** dhva) {

  s2_joint_spherical(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_vector_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_product_rv(&(dChild->R), vPrimitive,
                (s2_vector_t*)(((double*)vRelative) + 3));
  s2_cross_vv(&(dChild->T), (s2_vector_t*)(((double*)vRelative) + 3),
              (s2_vector_t*)vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
@brief Spherical Joint - inverse (dh)

Updates displacement and influence coefficient matrices

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_sphericalinverse(void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_quaternion_t* q = dhva[1];
  s2_displacement_t* dWRTReference = dhva[2];
  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));
  s2_product_drinverse(dParent, &(dWRTReference->R), dChild);
  s2_vector_copy(&dParent->T, &dChild->T);

  s2_hrxinverse(dChild, h);
  s2_hryinverse(dChild, h + 1);
  s2_hrzinverse(dChild, h + 2);
}

/*!
@brief Spherical Joint - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

    s2_displacement_t* dParent = dhva[0];

    s2_displacement_t* dPrimitive = dhva[1];

    s2_displacement_t* dWRTReference = dhva[2];

    s2_displacement_t* dChild = dhva[3];

    s2_screw_t* h = dhva[4];

    s2_screw_t* vParent = dhva[5];

    s2_screw_t* vPrimitive = dhva[6];

    s2_screw_t* vRelative = dhva[7];

    s2_screw_t* vChild = dhva[8];

    s2_screw_t* aqParent = dhva[9];

    s2_screw_t* aqRelative = dhva[10];

    s2_screw_t* aqChild = dhva[11];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void s2_joint_sphericalinverse_dhva(void** dhva) {

  s2_joint_sphericalinverse(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_vector_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_product_rvinverse(&(dChild->R), vPrimitive,
                       (s2_vector_t*)(((double*)vRelative) + 3));
  s2_cross_vv(&(dChild->T), (s2_vector_t*)(((double*)vRelative) + 3),
              (s2_vector_t*)vRelative);
  s2_add_ss(vParent, vRelative, vChild);
  s2_screwcross_ss(vChild, vRelative, aqRelative);
  s2_add_ss(aqParent, aqRelative, aqChild);
}

/*!
  @brief free a s2_joint_t data structure
*/
void s2_joint_free(s2_joint_t* joint) {

  if(joint) {
    if(joint->name) {
      s2_stackstr_free(joint->name);
    }
    if(joint->parentBody) {
      s2_stackstr_free(joint->parentBody);
      joint->parentBody = NULL;
    }
    if(joint->parentFrame) {
      s2_stackstr_free(joint->parentFrame);
      joint->parentFrame = NULL;
    }
    if(joint->childBody) {
      s2_stackstr_free(joint->childBody);
      joint->childBody = NULL;
    }
    if(joint->childFrame) {
      s2_stackstr_free(joint->childFrame);
      joint->childFrame = NULL;
    }
    if(joint->generalDisplacement) {
      free(joint->generalDisplacement);
      joint->generalDisplacement = NULL;
    }
    if(joint->directionCosine) {
      free(joint->directionCosine);
      joint->directionCosine = NULL;
    }
    if(joint->velocityRelative) {
      free(joint->velocityRelative);
      joint->velocityRelative = NULL;
    }
    if(joint->accelerationQuadraticRelative) {
      free(joint->accelerationQuadraticRelative);
      joint->accelerationQuadraticRelative = NULL;
    }
    if(joint->h) {
      free(joint->h);
      joint->h = NULL;
    }
    if(joint->primitiveVelocity) {
      free(joint->primitiveVelocity);
      joint->primitiveVelocity = NULL;
    }
    if(joint->primitiveDisplacement) {
      free(joint->primitiveDisplacement);
      joint->primitiveDisplacement = NULL;
    }

    if(joint->initialPrimitiveVelocity) {
      free(joint->initialPrimitiveVelocity);
      joint->initialPrimitiveVelocity = NULL;
    }

    if(joint->initialPrimitiveDisplacement) {
      free(joint->initialPrimitiveDisplacement);
      joint->initialPrimitiveDisplacement = NULL;
    }
    if(joint->jointTypeStr) {
      free(joint->jointTypeStr);
      joint->jointTypeStr = NULL;
    }
    free(joint);
    joint = NULL;
  }
}

/*!
@brief loads joint from shift json file
*/
int s2_joint_load(s2_context_t* context, s2_assembly_t* assembly,
                  s2_stackstr_t* prefix, json_t* jointObject) {

  s2_stackstr_t* name = s2_stackstr_copy(prefix);
  s2_stackstr_append(name,
                     json_string_value(json_object_get(jointObject, "name")));

  s2_joint_t* joint =
    s2_joint(json_string_value(json_object_get(jointObject, "type")), name);

  json_t* bodyFramePairs = json_object_get(jointObject, "body_frame_pairs");

  /* if the format of the parent or child body is an array, the user is
    specifying a full path to the body. the parent or child may exist
    within another assembly. */
  json_t* parentBodyObject =
    json_array_get(json_array_get(bodyFramePairs, 0), 0);

  if(json_is_array(parentBodyObject)) {
    joint->parentBody = s2_stackstr();
    size_t parentBodyIndex;
    json_t* value;
    json_array_foreach(parentBodyObject, parentBodyIndex, value) {
      s2_stackstr_append(joint->parentBody, json_string_value(value));
    }
  } else if(json_is_string(parentBodyObject)) {
    const char* parentBody = json_string_value(parentBodyObject);

    /* if parentBody == "fixed", don't append assemblyName */
    if(!strcmp(parentBody, "fixed")) {
      joint->parentBody = s2_stackstr();
      s2_stackstr_append(joint->parentBody, "fixed");
    } else {
      joint->parentBody = s2_stackstr_copy(assembly->name);
      s2_stackstr_append(joint->parentBody,
                         json_string_value(parentBodyObject));
    }
  }

  json_t* childBodyObject =
    json_array_get(json_array_get(bodyFramePairs, 1), 0);

  if(json_is_array(childBodyObject)) {
    joint->childBody = s2_stackstr();
    size_t childBodyIndex;
    json_t* value;
    json_array_foreach(childBodyObject, childBodyIndex, value) {
      s2_stackstr_append(joint->childBody, json_string_value(value));
    }
  } else if(json_is_string(childBodyObject)) {
    const char* childBody = json_string_value(childBodyObject);
    if(!strcmp(childBody, "fixed")) {
      joint->childBody = s2_stackstr();
      s2_stackstr_append(joint->childBody, "fixed");
    } else {
      joint->childBody = s2_stackstr_copy(assembly->name);
      s2_stackstr_append(joint->childBody, json_string_value(childBodyObject));
    }
  }

  joint->parentFrame = s2_stackstr_copy(joint->parentBody);
  s2_stackstr_append(
    joint->parentFrame,
    json_string_value(json_array_get(json_array_get(bodyFramePairs, 0), 1)));

  joint->childFrame = s2_stackstr_copy(joint->childBody);
  s2_stackstr_append(
    joint->childFrame,
    json_string_value(json_array_get(json_array_get(bodyFramePairs, 1), 1)));

  /* get spring coefficients */
  json_t* stiffnessCoefficients = json_object_get(jointObject, "stiffness");
  if(json_is_array(stiffnessCoefficients)) {

    size_t size = json_array_size(stiffnessCoefficients);
    joint->stiffnessCoefficientCount = (int)size;
    joint->stiffnessCoefficients =
      calloc(joint->stiffnessCoefficientCount, sizeof(double));

    for(int i = 0; i < joint->stiffnessCoefficientCount; i++) {
      joint->stiffnessCoefficients[i] =
        json_real_value(json_array_get(stiffnessCoefficients, i));
      // printf("stiffness coefficient [%d]: %f\n", i,
      // joint->stiffnessCoefficients[i]);
    }
  }

  /* get damping coefficients */
  json_t* dampingCoefficients = json_object_get(jointObject, "damping");
  if(json_is_array(dampingCoefficients)) {

    size_t size = json_array_size(dampingCoefficients);
    joint->dampingCoefficientCount = (int)size;
    joint->dampingCoefficients =
      calloc(joint->dampingCoefficientCount, sizeof(double));

    for(int i = 0; i < joint->dampingCoefficientCount; i++) {
      joint->dampingCoefficients[i] =
        json_real_value(json_array_get(dampingCoefficients, i));
      // printf("damping coefficient [%d]: %f\n", i,
      // joint->dampingCoefficients[i]);
    }
  }

  /* get initial values of joints */
  switch(joint->jointType) {
    case S2_JOINT_TX:
    case S2_JOINT_TY:
    case S2_JOINT_TZ:
    case S2_JOINT_RX:
    case S2_JOINT_RY:
    case S2_JOINT_RZ: {
      // printf("loading primitive\n");
      joint->initialPrimitiveDisplacement[0] =
        json_number_value(json_object_get(jointObject, "initial_displacement"));
      joint->primitiveDisplacement[0] = joint->initialPrimitiveDisplacement[0];
      json_t* initialValueVelocity =
        json_object_get(jointObject, "initial_velocity");
      if(!json_is_null(initialValueVelocity))
        joint->initialPrimitiveVelocity[0] =
          json_number_value(initialValueVelocity);
      joint->primitiveVelocity[0] = joint->initialPrimitiveVelocity[0];
      break;
    }
    case S2_JOINT_RIGID: {
      // printf("loading rigid/float\n");
      json_t* initialValueObj =
        json_object_get(jointObject, "initial_displacement");
      json_t* rotation = json_object_get(initialValueObj, "R");
      json_t* translation = json_object_get(initialValueObj, "T");
      if(rotation) {
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++)
            s2_displacement_set(joint->generalDisplacement, i, j,
                                json_number_value(json_array_get(
                                  json_array_get(rotation, i), j)));
        }
        s2_gramSchmidt(3, 3, (double*)joint->generalDisplacement);
      }
      if(translation) {
        for(int i = 0; i < 3; i++) {
          s2_vector_set(&(joint->generalDisplacement->T), i,
                        json_number_value(json_array_get(translation, i)));
        }
      }
      break;
    }
    case S2_JOINT_FLOAT: {
      printf("loading float\n");
      json_t* initialValueObj =
        json_object_get(jointObject, "initial_displacement");
      json_t* rotation = json_object_get(initialValueObj, "R");
      json_t* translation = json_object_get(initialValueObj, "T");
      if(rotation) {
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++)
            s2_displacement_set(joint->generalDisplacement, i, j,
                                json_number_value(json_array_get(
                                  json_array_get(rotation, i), j)));
        }
      }
      if(translation) {
        for(int i = 0; i < 3; i++) {
          s2_vector_set(&(joint->generalDisplacement->T), i,
                        json_number_value(json_array_get(translation, i)));
          joint->initialPrimitiveDisplacement[i] =
            s2_vector_get(&(joint->generalDisplacement->T), i);
        }
      }
      s2_mat3x3_toQuaternion(
        &(joint->generalDisplacement->R),
        (s2_quaternion_t*)(joint->primitiveDisplacement + 3));
      break;
    }
    case S2_JOINT_SPHERICAL: {
      json_t* initialValueObj =
        json_object_get(jointObject, "initial_displacement");
      json_t* rotation = json_object_get(initialValueObj, "R");
      if(rotation) {
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++)
            s2_displacement_set(joint->generalDisplacement, i, j,
                                json_number_value(json_array_get(
                                  json_array_get(rotation, i), j)));
        }
      }
      s2_mat3x3_toQuaternion(&(joint->generalDisplacement->R),
                             (s2_quaternion_t*)joint->primitiveDisplacement);
      printf("loading spherical: q=");
      for(int i = 0; i < 4; i++) {
        printf("%f,", joint->primitiveDisplacement[i]);
      }
      printf("\n");
      break;
    }
    case S2_JOINT_PRIMITIVE:
    case S2_JOINT_INVALID: {
      fprintf(stderr, "Invalid Joint Type Found in %s\n",
              joint->name->tail->value);
      break;
    }
  }

  (context->jointCount)++;
  s2_assembly_appendJoint(assembly, joint);
  return 0;
}

s2_body_t* s2_joint_getParentBody(s2_joint_t* self) {
  if(self->sense == 1) {
    return self->parentBodyPointer;
  } else {
    return self->childBodyPointer;
  }
}

s2_body_t* s2_joint_getChildBody(s2_joint_t* self) {
  if(self->sense == 1) {
    return self->childBodyPointer;
  } else {
    return self->parentBodyPointer;
  }
}
