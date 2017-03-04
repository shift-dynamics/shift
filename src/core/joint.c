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
#include "kinematics.h"
#include "joint.h"

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
void s2_joint_tx(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtx(
    dParent,
    dPrimitive,
    dChild);
  s2_htx(
    dChild,
    h);
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
void s2_joint_tx_dhva(
  void** dhva) {

  s2_joint_tx(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_sst(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_sst(
    vParent,
    vRelative,
    aqRelative);
  s2_add_sst(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_txinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtxinverse(
    dParent,
    dPrimitive,
    dChild);
  s2_htxinverse(
    dChild,
    h);
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
void s2_joint_txinverse_dhva(
  void** dhva) {

  s2_joint_txinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_sst(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_sst(
    vParent,
    vRelative,
    aqRelative);
  s2_add_sst(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_ty(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dty(
    dParent,
    dPrimitive,
    dChild);
  s2_hty(
    dChild,
    h);
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
void s2_joint_ty_dhva(
  void** dhva) {

  s2_joint_ty(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_sst(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_sst(
    vParent,
    vRelative,
    aqRelative);
  s2_add_sst(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_tyinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtyinverse(
    dParent,
    dPrimitive,
    dChild);
  s2_htyinverse(
    dChild,
    h);
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
void s2_joint_tyinverse_dhva(
  void** dhva) {

  s2_joint_tyinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_sst(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_sst(
    vParent,
    vRelative,
    aqRelative);
  s2_add_sst(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_tz(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtz(
    dParent,
    dPrimitive,
    dChild);
  s2_htz(
    dChild,
    h);
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
void s2_joint_tz_dhva(
  void **dhva) {

  s2_joint_tz(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_sst(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_sst(
    vParent,
    vRelative,
    aqRelative);
  s2_add_sst(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_tzinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dtzinverse(
    dParent,
    dPrimitive,
    dChild);
  s2_htzinverse(
    dChild,
    h);
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
void s2_joint_tzinverse_dhva(
  void** dhva) {

  s2_joint_tzinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_sst(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_sst(
    vParent,
    vRelative,
    aqRelative);
  s2_add_sst(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_rx(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drx(
    dParent,
    dc,
    dChild);
  s2_hrx(
    dChild,
    h);
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
void s2_joint_rx_dhva(
  void** dhva) {

  s2_joint_rx(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_rxinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drxinverse(
    dParent,
    dc,
    dChild);
  s2_hrxinverse(
    dChild,
    h);
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
void s2_joint_rxinverse_dhva(
  void** dhva) {

  s2_joint_rxinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_ry(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_dry(
    dParent,
    dc,
    dChild);
  s2_hry(
    dChild,
    h);
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
void s2_joint_ry_dhva(
  void** dhva) {

  s2_joint_ry(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_ryinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];

  s2_screw_t* h = dhva[3];

  s2_product_dryinverse(
    dParent,
    dc,
    dChild);
  s2_hryinverse(
    dChild,
    h);
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
void s2_joint_ryinverse_dhva(
  void** dhva) {

  s2_joint_ryinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_rz(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drz(
    dParent,
    dc,
    dChild);
  s2_hrz(
    dChild,
    h);
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
void s2_joint_rz_dhva(
  void** dhva) {

  s2_joint_rz(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_rzinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  double* dc = dhva[1];
  s2_displacement_t* dChild = dhva[2];
  s2_screw_t* h = dhva[3];

  s2_product_drzinverse(
    dParent,
    dc,
    dChild);
  s2_hrzinverse(
    dChild,
    h);
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
void s2_joint_rzinverse_dhva(
  void** dhva) {

  s2_joint_rzinverse(dhva);

  s2_screw_t* h = dhva[3];

  s2_screw_t* vParent = dhva[4];
  double* vPrimitive = dhva[5];
  s2_screw_t* vRelative = dhva[6];
  s2_screw_t* vChild = dhva[7];

  s2_screw_t* aqParent = dhva[8];
  s2_screw_t* aqRelative = dhva[9];
  s2_screw_t* aqChild = dhva[10];

  s2_product_sk(
    h,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_rigid(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_displacement_t* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];

  s2_product_dd(
    dParent,
    dPrimitive,
    dChild);
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
void s2_joint_rigid_dhva(
  void** dhva) {

  s2_joint_rigid(dhva);

  s2_screw_t* vParent = dhva[3];
  s2_screw_t* vChild = dhva[4];
  s2_screw_t* aqChild = dhva[5];
  s2_screw_t* aqParent = dhva[6];

  s2_screw_copy(
    vParent,
    vChild);
  s2_screw_copy(
    aqParent,
    aqChild);
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
void s2_joint_rigidinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_displacement_t* dPrimitive = dhva[1];
  s2_displacement_t* dChild = dhva[2];

  s2_product_ddinverse(
    dParent,
    dPrimitive,
    dChild);
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
void s2_joint_rigidinverse_dhva(
  void** dhva) {

  s2_joint_rigidinverse(dhva);

  s2_screw_t* vParent = dhva[3];
  s2_screw_t* vChild = dhva[4];
  s2_screw_t* aqChild = dhva[5];
  s2_screw_t* aqParent = dhva[6];
  s2_screw_copy(
    vParent,
    vChild);
  s2_screw_copy(
    aqParent,
    aqChild);
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
void s2_joint_float(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];

  s2_vector_t* t = dhva[1];
  s2_quaternion_t* q = (s2_quaternion_t*)(((double*)dhva[1]) + 3);

  s2_displacement_t* dWRTReference = dhva[2];

  s2_displacement_t* dChild = dhva[3];

  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));

  s2_product_rr(
    &(dParent->R),
    &(dWRTReference->R),
    &(dChild)->R);
  s2_product_rv(
    &(dParent->R),
    t,
    &(dChild ->T));
  cblas_daxpy(3, 1.0, ((double*)dParent) + 9, 1, ((double*)dChild) + 9, 1);

  s2_htx(
    dChild,
    h);

  s2_hty(
    dChild,
    h + 1);

  s2_htz(
    dChild,
    h + 2);

  s2_hrx(
    dChild,
    h + 3);

  s2_hry(
    dChild,
    h + 4);

  s2_hrz(
    dChild,
    h + 5);

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
void s2_joint_float_dhva(
  void** dhva) {

  s2_joint_float(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_screw_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_screwproduct_ds(
    dChild,
    vPrimitive,
    vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_floatinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];

  s2_vector_t* t = dhva[1];
  s2_quaternion_t* q = (s2_quaternion_t*)(((double*)dhva[1]) + 3);

  s2_displacement_t* dWRTReference = dhva[2];

  s2_displacement_t* dChild = dhva[3];

  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));

  s2_product_rrinverse(
    &(dParent->R),
    &(dWRTReference->R),
    &(dChild)->R);
  s2_product_rvinverse(
    &(dParent->R),
    t,
    &(dChild ->T));
  cblas_daxpy(3, 1.0, ((double*)dParent) + 9, 1, ((double*)dChild) + 9, 1);

  s2_htxinverse(
    dChild,
    h);

  s2_htyinverse(
    dChild,
    h + 1);

  s2_htzinverse(
    dChild,
    h + 2);

  s2_hrxinverse(
    dChild,
    h + 3);

  s2_hryinverse(
    dChild,
    h + 4);

  s2_hrzinverse(
    dChild,
    h + 5);
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
void s2_joint_floatinverse_dhva(
  void** dhva) {

  s2_joint_floatinverse(dhva);

  s2_displacement_t* dChild = dhva[3];

  s2_screw_t* vParent = dhva[5];

  s2_screw_t* vPrimitive = dhva[6];

  s2_screw_t* vRelative = dhva[7];

  s2_screw_t* vChild = dhva[8];

  s2_screw_t* aqParent = dhva[9];

  s2_screw_t* aqRelative = dhva[10];

  s2_screw_t* aqChild = dhva[11];

  s2_screwproduct_dsinverse(
   dChild,
   vPrimitive,
   vRelative);

  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
void s2_joint_spherical(
  void** dhva) {

    s2_displacement_t* dParent = dhva[0];
    s2_quaternion_t* q = dhva[1];
    s2_displacement_t* dWRTReference = dhva[2];
    s2_displacement_t* dChild = dhva[3];
    s2_screw_t* h = dhva[4];

    s2_quaternion_toMat3x3(q, &(dWRTReference->R));
    s2_product_dr(
      dParent,
      &(dWRTReference->R),
      dChild);

    s2_hrx(dChild, h);
    s2_hry(dChild, h + 1);
    s2_hrz(dChild, h + 2);
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
@todo add spherical joint
*/
void s2_joint_spherical_dhva(
  void** dhva) {

  s2_joint_spherical(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_vector_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_product_rv(
    &(dChild->R),
    vPrimitive,
    (s2_vector_t*)(((double*)vRelative) + 3));
  s2_cross_vv(
    &(dChild->T),
    (s2_vector_t*)(((double*)vRelative) + 3),
    (s2_vector_t*)vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
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
@todo add sphericalinverse
*/
void s2_joint_sphericalinverse(
  void** dhva) {

  s2_displacement_t* dParent = dhva[0];
  s2_quaternion_t* q = dhva[1];
  s2_displacement_t* dWRTReference = dhva[2];
  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* h = dhva[4];

  s2_quaternion_toMat3x3(q, &(dWRTReference->R));

  s2_product_drinverse(
    dParent,
    &(dWRTReference->R),
    dChild);

  s2_hrxinverse(dChild, h);
  s2_hryinverse(dChild, h + 1);
  s2_hrzinverse(dChild, h + 2);
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
void s2_joint_sphericalinverse_dhva(
  void** dhva) {

  s2_joint_sphericalinverse(dhva);

  s2_joint_spherical(dhva);

  s2_displacement_t* dChild = dhva[3];
  s2_screw_t* vParent = dhva[5];
  s2_vector_t* vPrimitive = dhva[6];
  s2_screw_t* vRelative = dhva[7];
  s2_screw_t* vChild = dhva[8];
  s2_screw_t* aqParent = dhva[9];
  s2_screw_t* aqRelative = dhva[10];
  s2_screw_t* aqChild = dhva[11];

  s2_product_rvinverse(
    &(dChild->R),
    vPrimitive,
    (s2_vector_t*)(((double*)vRelative) + 3));
  s2_cross_vv(
    &(dChild->T),
    (s2_vector_t*)(((double*)vRelative) + 3),
    (s2_vector_t*)vRelative);
  s2_add_ss(
    vParent,
    vRelative,
    vChild);
  s2_screwcross_ss(
    vChild,
    vRelative,
    aqRelative);
  s2_add_ss(
    aqParent,
    aqRelative,
    aqChild);
}
