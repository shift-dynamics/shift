#include "shift/core/joint/spherical.h"
#include "shift/core/factory.h"
#include "ksl/quaternion.h"

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
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(&q[offsets.q]);
  ksl_mat3x3_t temp;

  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddr(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
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
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(&q[offsets.q]);
  ksl_mat3x3_t temp;
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddr(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
  ksl_hsrx(d_child, h);
  ksl_hsry(d_child, h + 1);
  ksl_hsrz(d_child, h + 2);
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

  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_vec3_t* v_primitive = (ksl_vec3_t*)(&dq[offsets.dq]);
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  /* omega = R * dq, consistent with the influence coefficients ksl_hsrx,
  ksl_hsry and ksl_hsrz (the inverse joint uses -R * dq with ksl_hsrxinv) */
  ksl_product_drv(&d_child->R, v_primitive, &v_relative.ang);
  ksl_cross_vv(&d_child->t, &v_relative.ang, &v_relative.lin);
  ksl_add_ss(v_parent, &v_relative, v_child);
  ksl_cross_ss(v_parent, &v_relative, &aq_relative);
  ksl_add_ss(aq_parent, &aq_relative, aq_child);
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
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(&q[offsets.q]);
  ksl_mat3x3_t temp;
  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddrinv(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
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
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(&q[offsets.q]);
  ksl_mat3x3_t temp;
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_quaternion_toMat3x3(quat, &temp);
  ksl_product_ddrinv(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
  ksl_hsrxinv(d_child, h);
  ksl_hsryinv(d_child, h + 1);
  ksl_hsrzinv(d_child, h + 2);
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

  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_vec3_t* v_primitive = (ksl_vec3_t*)(&dq[offsets.dq]);
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_drvinv(&d_child->R, v_primitive, &v_relative.ang);
  ksl_cross_vv(&d_child->t, &v_relative.ang, &v_relative.lin);
  ksl_add_ss(v_parent, &v_relative, v_child);
  ksl_cross_ss(v_parent, &v_relative, &aq_relative);
  ksl_add_ss(aq_parent, &aq_relative, aq_child);
}

void shift::core::joint::Spherical::DeclareParameters() {
  // the initial displacement across the joint is an unnamed frame
  frame_["name"].set_required(false);
  DeclareParameter("initial_displacement", &frame_,
                   "initial rotation of the child frame with respect to the "
                   "parent frame across the joint, defaults to identity if not "
                   "specified",
                   false);
  /* dq = omega, the angular velocity of the child frame with respect to the
  parent frame, expressed in the child frame */
  DeclareParameter("initial_velocity", reinterpret_cast<ksl_vec3_t*>(dq_0_),
                   "initial angular velocity [wx, wy, wz] of the child frame "
                   "across the joint, expressed in the child frame, defaults "
                   "to zero",
                   false);
}

void shift::core::joint::Spherical::ReadJSON(Json::Value& value) {
  shift::core::joint::Base::ReadJSON(value);
  /* q = quaternion [x, y, z, w], see spherical_d */
  ksl_mat3x3_toQuaternion(&frame_.get_d_wrt_origin()->R,
                          (ksl_quaternion_t*)(q_0_));
}

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
