#include "shift-core/joint/spherical.h"
#include "shift-core/factory.h"

bool Shift::Joint::Spherical::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "spherical", Shift::Joint::Spherical::create);

std::unique_ptr<Shift::Joint::Spherical> Shift::Joint::Spherical::create() {
  return std::make_unique<Shift::Joint::Spherical>();
}

/*!
@brief Spherical Joint (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::spherical_d(const Shift::Context&,
                               Shift::Topology::DHVA& dhva,
                               const Shift::Topology::Offsets& offsets,
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
void Shift::Joint::spherical_dh(const Shift::Context&,
                                Shift::Topology::DHVA& dhva,
                                const Shift::Topology::Offsets& offsets,
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
void Shift::Joint::spherical_dhva(const Shift::Context& context,
                                  Shift::Topology::DHVA& dhva,
                                  const Shift::Topology::Offsets& offsets,
                                  const double* __restrict__ q,
                                  const double* __restrict__ dq) {

  Shift::Joint::spherical_dh(context, dhva, offsets, q);

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
void Shift::Joint::sphericalinv_d(const Shift::Context&,
                                  Shift::Topology::DHVA& dhva,
                                  const Shift::Topology::Offsets& offsets,
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
void Shift::Joint::sphericalinv_dh(const Shift::Context&,
                                   Shift::Topology::DHVA& dhva,
                                   const Shift::Topology::Offsets& offsets,
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
void Shift::Joint::sphericalinv_dhva(const Shift::Context& context,
                                     Shift::Topology::DHVA& dhva,
                                     const Shift::Topology::Offsets& offsets,
                                     const double* __restrict__ q,
                                     const double* __restrict__ dq) {

  Shift::Joint::sphericalinv_dh(context, dhva, offsets, q);

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

void Shift::Joint::Spherical::addParameters() {}

Shift::Topology::dh_fptr Shift::Joint::Spherical::Dfptr() {
  if(sense_ >= 0) {
    return spherical_d;
  } else {
    return sphericalinv_d;
  }
}

Shift::Topology::dh_fptr Shift::Joint::Spherical::DHfptr() {
  if(sense_ >= 0) {
    return spherical_dh;
  } else {
    return sphericalinv_dh;
  }
}

Shift::Topology::dhva_fptr Shift::Joint::Spherical::DHVAfptr() {
  if(sense_ >= 0) {
    return spherical_dhva;
  } else {
    return sphericalinv_dhva;
  }
}
