#include "shift-core/joint/float.h"
#include "shift-core/factory.h"

bool Shift::Joint::Float::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "float", Shift::Joint::Float::create);

std::unique_ptr<Shift::Joint::Float> Shift::Joint::Float::create() {
  return std::make_unique<Shift::Joint::Float>();
}

/*!
@brief Float Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::float_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                            const Shift::Joint::Offsets& offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_vec3_t* t = (ksl_vec3_t*)(q + offsets.q);
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + offsets.q + 3);
  ksl_SE3_t temp;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_quaternion_toMat3x3(quat, &temp.R);
  ksl_product_drdr(&dParent->R, &temp.R, &dChild->R);
  ksl_product_drv(&dParent->R, t, &dChild->t);
  ksl_xpy_vv(&dParent->t, &dChild->t);
  ksl_hstx(dChild, h);
  ksl_hsty(dChild, h + 1);
  ksl_hstz(dChild, h + 2);
  ksl_hsrx(dChild, h + 3);
  ksl_hsry(dChild, h + 4);
  ksl_hsrz(dChild, h + 5);
}

/*!
@brief Float Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

Float
[0] dParent      offset into dhva.dAbsolute
[1] dChild       offset into dhva.dAbsolute
[2] q[7]         offset into q
[3] dqIndex      offset into dhva.h, dq
[4] vaParent     offset into dhva.v, dhva.aq
[5] vaChild      offset into dhva.v, dhva.aq
Total: 6

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::float_dhva(const Shift::Context& context,
                              Shift::Joint::DHVA& dhva,
                              const Shift::Joint::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::float_dh(context, dhva, offsets, q);

  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_screw_t* vPrimitive = (ksl_screw_t*)(dq + offsets.dq);
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t* vChild = dhva.v + offsets.child.v;

  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_Ads(dChild, vPrimitive, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Float Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::floatinv_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                               const Shift::Joint::Offsets& offsets,
                               const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_vec3_t* t = (ksl_vec3_t*)(q + offsets.q);
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + offsets.q + 3);
  ksl_SE3_t temp;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_quaternion_toMat3x3(quat, &temp.R);
  ksl_product_drdrinv(&dParent->R, &temp.R, &dChild->R);
  ksl_product_drvinv(&dParent->R, t, &dChild->t);
  ksl_xpy_vv(&dParent->t, &dChild->t);
  ksl_hstxinv(dChild, h);
  ksl_hstyinv(dChild, h + 1);
  ksl_hstzinv(dChild, h + 2);
  ksl_hsrxinv(dChild, h + 3);
  ksl_hsryinv(dChild, h + 4);
  ksl_hsrzinv(dChild, h + 5);
}

/*!
@brief Float Joint - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::floatinv_dhva(const Shift::Context& context,
                                 Shift::Joint::DHVA& dhva,
                                 const Shift::Joint::Offsets& offsets,
                                 const double* __restrict__ q,
                                 const double* __restrict__ dq) {
  Shift::Joint::floatinv_dh(context, dhva, offsets, q);

  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  ksl_screw_t* vPrimitive = (ksl_screw_t*)(dq + offsets.dq);
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t* vChild = dhva.v + offsets.child.v;

  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_Adsinv(dChild, vPrimitive, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

void Shift::Joint::Float::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Float::readJSON(Json::Value& value) {
  Shift::Joint::Base::readJSON(value);
  frame_.setNameRequired(false);
  if(value["initial_displacement"]) {
    frame_.readJSON(value["initial_displacement"]);
  }
  const ksl_SE3_t* temp = frame_.getTransform();
  ksl_mat3x3_toQuaternion(&temp->R, (ksl_quaternion_t*)(&q_0_ + 3));
  ksl_vec3_copy(&temp->t, (ksl_vec3_t*)(&q_0_));
}

Shift::Joint::dh_fptr Shift::Joint::Float::getDH() {
  if(sense_ >= 0) {
    return float_dh;
  } else {
    return floatinv_dh;
  }
}

Shift::Joint::dhva_fptr Shift::Joint::Float::getDHVA() {
  if(sense_ >= 0) {
    return float_dhva;
  } else {
    return floatinv_dhva;
  }
}
