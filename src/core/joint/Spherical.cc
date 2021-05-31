#include "shift/core/joint/Spherical.h"
#include "shift/core/Factory.h"

bool shift::core::joint::Spherical::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "spherical", shift::core::joint::Spherical::create);

std::unique_ptr<shift::core::joint::Spherical> shift::core::joint::Spherical::create() {
  return std::make_unique<shift::core::joint::Spherical>();
}

/*!
@brief Spherical Joint (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::spherical_d(const shift::core::Context&,
                               shift::core::topology::DHVA& dhva,
                               const shift::core::topology::Offsets& offsets,
                               const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + offsets.q);
  ksl_mat3x3_t temp;

  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
}

/*!
@brief Spherical Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::spherical_dh(const shift::core::Context&,
                                shift::core::topology::DHVA& dhva,
                                const shift::core::topology::Offsets& offsets,
                                const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + offsets.q);
  ksl_mat3x3_t temp;
  ksl_screw_t* h = dhva.h + offsets.dq;

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
void shift::core::joint::spherical_dhva(const shift::core::Context& context,
                                  shift::core::topology::DHVA& dhva,
                                  const shift::core::topology::Offsets& offsets,
                                  const double* __restrict__ q,
                                  const double* __restrict__ dq) {

  shift::core::joint::spherical_dh(context, dhva, offsets, q);

  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_vec3_t* vPrimitive = (ksl_vec3_t*)(dq + offsets.dq);
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_drvinv(&dChild->R, vPrimitive, &vRelative.ang);
  ksl_cross_vv(&dChild->t, &vRelative.ang, &vRelative.lin);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Spherical Joint - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::sphericalinv_d(const shift::core::Context&,
                                  shift::core::topology::DHVA& dhva,
                                  const shift::core::topology::Offsets& offsets,
                                  const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + offsets.q);
  ksl_mat3x3_t temp;
  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddrinv(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
}

/*!
@brief Spherical Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::sphericalinv_dh(const shift::core::Context&,
                                   shift::core::topology::DHVA& dhva,
                                   const shift::core::topology::Offsets& offsets,
                                   const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + offsets.q);
  ksl_mat3x3_t temp;
  ksl_screw_t* h = dhva.h + offsets.dq;

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
void shift::core::joint::sphericalinv_dhva(const shift::core::Context& context,
                                     shift::core::topology::DHVA& dhva,
                                     const shift::core::topology::Offsets& offsets,
                                     const double* __restrict__ q,
                                     const double* __restrict__ dq) {

  shift::core::joint::sphericalinv_dh(context, dhva, offsets, q);

  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_vec3_t* vPrimitive = (ksl_vec3_t*)(dq + offsets.dq);
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_drvinv(&dChild->R, vPrimitive, &vRelative.ang);
  ksl_cross_vv(&dChild->t, &vRelative.ang, &vRelative.lin);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

void shift::core::joint::Spherical::DeclareParameters() {}

shift::core::topology::dh_fptr shift::core::joint::Spherical::Dfptr() {
  if(sense_ >= 0) {
    return spherical_d;
  } else {
    return sphericalinv_d;
  }
}

shift::core::topology::dh_fptr shift::core::joint::Spherical::DHfptr() {
  if(sense_ >= 0) {
    return spherical_dh;
  } else {
    return sphericalinv_dh;
  }
}

shift::core::topology::dhva_fptr shift::core::joint::Spherical::DHVAfptr() {
  if(sense_ >= 0) {
    return spherical_dhva;
  } else {
    return sphericalinv_dhva;
  }
}
