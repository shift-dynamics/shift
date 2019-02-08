#include "shift-core/joint/spherical.h"
#include "shift-core/factory.h"

bool Shift::Joint::Spherical::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "spherical", Shift::Joint::Spherical::create);

std::unique_ptr<Shift::Joint::Spherical> Shift::Joint::Spherical::create() {
  return std::make_unique<Shift::Joint::Spherical>();
}

/*!
@brief Spherical Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::spherical_dh(const Shift::Context&,
                                Shift::Joint::DHVA& dhvaPointers,
                                const int* __restrict__ dhvaOffsets,
                                const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + dhvaOffsets[2]);
  ksl_mat3x3_t temp;
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
  ksl_hsrx(dChild, h);
  ksl_hsry(dChild, h + 1);
  ksl_hsrz(dChild, h + 2);
}

/*!
@brief Spherical Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
@todo add spherical joint
*/
void Shift::Joint::spherical_dhva(const Shift::Context& context,
                                  Shift::Joint::DHVA& dhvaPointers,
                                  const int* __restrict__ dhvaOffsets,
                                  const double* __restrict__ q,
                                  const double* __restrict__ dq) {

  Shift::Joint::spherical_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  ksl_vec3_t* vPrimitive = (ksl_vec3_t*)(dq + dhvaOffsets[3]);
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_drv(&(dChild->R), vPrimitive,
                  (ksl_vec3_t*)(((double*)&vRelative) + 3));
  ksl_cross_vv(&(dChild->t), (ksl_vec3_t*)(((double*)&vRelative) + 3),
               (ksl_vec3_t*)&vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Spherical Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::sphericalinv_dh(const Shift::Context&,
                                   Shift::Joint::DHVA& dhvaPointers,
                                   const int* __restrict__ dhvaOffsets,
                                   const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + dhvaOffsets[2]);
  ksl_mat3x3_t temp;
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddrinv(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
  ksl_hsrxinv(dChild, h);
  ksl_hsryinv(dChild, h + 1);
  ksl_hsrzinv(dChild, h + 2);
}

/*!
@brief Spherical Joint - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::sphericalinv_dhva(const Shift::Context& context,
                                     Shift::Joint::DHVA& dhvaPointers,
                                     const int* __restrict__ dhvaOffsets,
                                     const double* __restrict__ q,
                                     const double* __restrict__ dq) {

  Shift::Joint::sphericalinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  ksl_vec3_t* vPrimitive = (ksl_vec3_t*)(dq + dhvaOffsets[3]);
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_drvinv(&(dChild->R), vPrimitive,
                     (ksl_vec3_t*)(((double*)&vRelative) + 3));
  ksl_cross_vv(&(dChild->t), (ksl_vec3_t*)(((double*)&vRelative) + 3),
               (ksl_vec3_t*)&vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

void Shift::Joint::Spherical::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Spherical::updateD() {}

void Shift::Joint::Spherical::updateDH() {}

void Shift::Joint::Spherical::updateDHVA() {}
