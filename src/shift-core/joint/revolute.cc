#include "shift-core/joint/revolute.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"

bool Shift::Joint::Revolute::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "revolute", Shift::Joint::Revolute::create);

std::unique_ptr<Shift::Joint::Revolute> Shift::Joint::Revolute::create() {
  return std::make_unique<Shift::Joint::Revolute>();
}

/*!
@brief Revolute Joint aligned with x-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rx_dh(const Shift::Context&,
                         Shift::Joint::DHVA& dhvaPointers,
                         const int* __restrict__ dhvaOffsets,
                         const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddrx(dParent, dc, dChild);
  ksl_hsrx(dChild, h);
}

/*!
@brief Revolute joint aligned with x-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rx_dhva(const Shift::Context& context,
                           Shift::Joint::DHVA& dhvaPointers,
                           const int* __restrict__ dhvaOffsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::rx_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with x-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rxinv_dh(const Shift::Context&,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddrxinv(dParent, dc, dChild);
  ksl_hsrxinv(dChild, h);
}

/*!
@brief Revolute joint aligned with x-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rxinv_dhva(const Shift::Context& context,
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::rxinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ry_dh(const Shift::Context&,
                         Shift::Joint::DHVA& dhvaPointers,
                         const int* __restrict__ dhvaOffsets,
                         const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddry(dParent, dc, dChild);
  ksl_hsry(dChild, h);
}

/*!
@brief Revolute joint aligned with y-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ry_dhva(const Shift::Context& context,
                           Shift::Joint::DHVA& dhvaPointers,
                           const int* __restrict__ dhvaOffsets,

                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::ry_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with y-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ryinv_dh(const Shift::Context&,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddryinv(dParent, dc, dChild);
  ksl_hsryinv(dChild, h);
}

/*!
@brief Revolute joint aligned with y-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ryinv_dhva(const Shift::Context& context,
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::ryinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rz_dh(const Shift::Context&,
                         Shift::Joint::DHVA& dhvaPointers,
                         const int* __restrict__ dhvaOffsets,
                         const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddrz(dParent, dc, dChild);
  ksl_hsrz(dChild, h);
}

/*!
@brief Revolute joint aligned with z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rz_dhva(const Shift::Context& context,
                           Shift::Joint::DHVA& dhvaPointers,
                           const int* __restrict__ dhvaOffsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  Shift::Joint::rz_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with z-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rzinv_dh(const Shift::Context&,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddrzinv(dParent, dc, dChild);
  ksl_hsrzinv(dChild, h);
}

/*!
@brief Revolute joint aligned with z-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rzinv_dhva(const Shift::Context& context,
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::rzinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute joint aligned with arbitrary axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ra_dh(const Shift::Context& context, DHVA& dhvaPointers,
                         const int* dhvaOffsets, const double*) {

  const ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const ksl_vec3_t* axis =
    (ksl_vec3_t*)(context.getJointData() + dhvaOffsets[4]);
  ksl_mat3x3_t temp;

  ksl_mat3x3_setFromAxisDC(&temp, axis, dc);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
  ksl_hsra(dChild, axis, h);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rainv_dh(const Shift::Context& context, DHVA& dhvaPointers,
                            const int* dhvaOffsets, const double*) {
  const ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dc = dhvaPointers.dc + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const ksl_vec3_t* axis =
    (ksl_vec3_t*)(context.getJointData() + dhvaOffsets[4]);
  ksl_vec3_t neg_axis = ksl_vec3(-axis->x, -axis->y, -axis->z);
  ksl_mat3x3_t temp;

  ksl_mat3x3_setFromAxisDC(&temp, &neg_axis, dc);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
  ksl_hsrainv(dChild, axis, h);
}

/*!
@brief Revolute joint aligned with arbitrary axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ra_dhva(const Shift::Context& context, DHVA& dhvaPointers,
                           const int* dhvaOffsets, const double* q,
                           const double* dq) {
  Shift::Joint::ra_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rainv_dhva(const Shift::Context& context, DHVA& dhvaPointers,
                              const int* dhvaOffsets, const double* q,
                              const double* dq) {
  Shift::Joint::rainv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

void Shift::Joint::Revolute::addKeys() {
  Shift::Joint::Base::addKeys();

  SerialEntry axis("axis", SerialType::KSL_VEC3, &axis_);
  axis.setRequired(false);
  axis.setDescription(
    "Specifies axis of rotation expressed in body frame, defaults to z-axis if "
    " no axis is provided.");
  addSerialEntry(axis);

  SerialEntry q0("initial_displacement", SerialType::DOUBLE,
                 &initialDisplacement_);
  q0.setRequired(false);
  q0.setDescription("Initial joint displacement.");
  addSerialEntry(q0);

  SerialEntry dq0("initial_velocity", SerialType::DOUBLE, &initialVelocity_);
  dq0.setRequired(false);
  dq0.setDescription("Initial joint velocity.");
  addSerialEntry(dq0);
}

void Shift::Joint::Revolute::updateD() {}

void Shift::Joint::Revolute::updateDH() {}

void Shift::Joint::Revolute::updateDHVA() {}
