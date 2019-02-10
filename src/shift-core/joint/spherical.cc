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
void Shift::Joint::spherical_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                                const Shift::Joint::Offsets& offsets,
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
                                  Shift::Joint::DHVA& dhva,
                                  const Shift::Joint::Offsets& offsets,
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
@brief Spherical Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::sphericalinv_dh(const Shift::Context&,
                                   Shift::Joint::DHVA& dhva,
                                   const Shift::Joint::Offsets& offsets,
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
                                     Shift::Joint::DHVA& dhva,
                                     const Shift::Joint::Offsets& offsets,
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

void Shift::Joint::Spherical::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Spherical::readJSON(Json::Value& value) {
  Shift::Joint::Base::readJSON(value);
  frame_.setNameRequired(false);
  if(value["initial_displacement"]) {
    frame_.readJSON(value["initial_displacement"]);
  }
  const ksl_SE3_t* temp = frame_.getTransform();
  ksl_mat3x3_toQuaternion(&temp->R, (ksl_quaternion_t*)&q_0_);
}

Shift::Joint::dh_fptr Shift::Joint::Spherical::getDH() {
  if(sense_ >= 0) {
    return spherical_dh;
  } else {
    return sphericalinv_dh;
  }
}

Shift::Joint::dhva_fptr Shift::Joint::Spherical::getDHVA() {
  if(sense_ >= 0) {
    return spherical_dhva;
  } else {
    return sphericalinv_dhva;
  }
}
