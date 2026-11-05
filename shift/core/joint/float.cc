#include "shift/core/joint/float.h"
#include "shift/core/factory.h"
#include "ksl/quaternion.h"
#include "ksl/vec3.h"

bool shift::core::joint::Float::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "float", shift::core::joint::Float::create);

std::unique_ptr<shift::core::joint::Float> shift::core::joint::Float::create() {
  return std::make_unique<shift::core::joint::Float>();
}

/*!
@brief Float Joint (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::float_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_vec3_t* t = (ksl_vec3_t*)(&q[offsets.q]);
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(&q[offsets.q] + 3);
  ksl_SE3_t temp;
  ksl_quaternion_toMat3x3(quat, &temp.R);
  ksl_product_drdr(&d_parent->R, &temp.R, &d_child->R);
  ksl_product_drv(&d_parent->R, t, &d_child->t);
  ksl_xpy_vv(&d_parent->t, &d_child->t);
}

/*!
@brief Float Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::float_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_vec3_t* t = (ksl_vec3_t*)(&q[offsets.q]);
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(&q[offsets.q] + 3);
  ksl_SE3_t temp;
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_quaternion_toMat3x3(quat, &temp.R);
  ksl_product_drdr(&d_parent->R, &temp.R, &d_child->R);
  ksl_product_drv(&d_parent->R, t, &d_child->t);
  ksl_xpy_vv(&d_parent->t, &d_child->t);
  ksl_hstx(d_child, h);
  ksl_hsty(d_child, h + 1);
  ksl_hstz(d_child, h + 2);
  ksl_hsrx(d_child, h + 3);
  ksl_hsry(d_child, h + 4);
  ksl_hsrz(d_child, h + 5);
}

/*!
@brief Float Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::float_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  shift::core::joint::float_dh(context, dhva, offsets, q);

  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  ksl_screw_t* v_primitive = (ksl_screw_t*)(dq + offsets.dq);
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_parent = dhva.v + offsets.parent.v;
  ksl_screw_t* v_child = dhva.v + offsets.child.v;

  ksl_screw_t* aq_parent = dhva.aq + offsets.parent.v;
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = dhva.aq + offsets.child.v;

  ksl_product_Ads(d_child, v_primitive, &v_relative);
  ksl_add_ss(v_parent, &v_relative, v_child);
  ksl_cross_ss(v_child, &v_relative, &aq_relative);
  ksl_add_ss(aq_parent, &aq_relative, aq_child);
}

void shift::core::joint::floatinv_pose(const ksl_SE3_t* d_parent,
                                       const double* q, ksl_SE3_t* d_child) {
  const ksl_vec3_t* t = (const ksl_vec3_t*)(q);
  ksl_quaternion_t* quat = (ksl_quaternion_t*)(q + 3);
  ksl_mat3x3_t R;
  ksl_quaternion_toMat3x3(quat, &R);
  ksl_product_drdrinv(&d_parent->R, &R, &d_child->R);
  ksl_product_drvinv(&d_child->R, t, &d_child->t);
  ksl_xpy_vv(&d_parent->t, &d_child->t);
}

void shift::core::joint::floatinv_h(const ksl_SE3_t* d_parent,
                                    const ksl_SE3_t* d_child, ksl_screw_t* h) {
  ksl_hstxinv(d_child, h);
  ksl_hstyinv(d_child, h + 1);
  ksl_hstzinv(d_child, h + 2);
  for(int k = 0; k < 3; k++) {
    ksl_vec3_inverted(&d_child->R.v[k], &h[3 + k].ang);
    ksl_cross_vv(&d_parent->t, &h[3 + k].ang, &h[3 + k].lin);
  }
}

/*!
@brief Float Joint - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::floatinv_d(const shift::core::Context&,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q) {
  shift::core::joint::floatinv_pose(&dhva.d[offsets.parent.d], q + offsets.q,
                   &dhva.d[offsets.child.d]);
}

/*!
@brief Float Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::floatinv_dh(const shift::core::Context&,
                               shift::core::topology::DHVA& dhva,
                               const shift::core::topology::Offsets& offsets,
                               const double* __restrict__ q) {
  const ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  shift::core::joint::floatinv_pose(d_parent, q + offsets.q, d_child);
  shift::core::joint::floatinv_h(d_parent, d_child, &dhva.h[offsets.dq]);
}

/*!
@brief Float Joint - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::floatinv_dhva(const shift::core::Context& context,
                                 shift::core::topology::DHVA& dhva,
                                 const shift::core::topology::Offsets& offsets,
                                 const double* __restrict__ q,
                                 const double* __restrict__ dq) {
  shift::core::joint::floatinv_dh(context, dhva, offsets, q);

  const ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  const ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* dq_joint = dq + offsets.dq;
  const ksl_screw_t* v_parent = dhva.v + offsets.parent.v;
  ksl_screw_t* v_child = dhva.v + offsets.child.v;
  const ksl_screw_t* aq_parent = dhva.aq + offsets.parent.v;
  ksl_screw_t* aq_child = dhva.aq + offsets.child.v;

  /* v_child = v_parent + sum_k h_k dq_k */
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  for(int k = 0; k < 6; k++) {
    for(int m = 0; m < 6; m++) {
      v_relative.at[m] += h[k].at[m] * dq_joint[k];
    }
  }
  ksl_add_ss(v_parent, &v_relative, v_child);

  /* aq_child = aq_parent + sum_k dh_k/dt dq_k. The axes a_k = R_C e_k turn
  with C, da_k/dt = omega_C x a_k, the rotation axes pass through the origin
  of P, which moves with velocity dt_P/dt = v_P.lin + omega_P x t_P */
  const ksl_vec3_t* omega_child = &v_child->ang;
  ksl_vec3_t dt_parent;
  ksl_cross_vv(&v_parent->ang, &d_parent->t, &dt_parent);
  ksl_xpy_vv(&v_parent->lin, &dt_parent);

  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  for(int k = 0; k < 3; k++) {
    ksl_vec3_t da; /* d(-a_k)/dt = omega_C x -a_k */
    ksl_vec3_t minus_a;
    ksl_vec3_inverted(&d_child->R.v[k], &minus_a);
    ksl_cross_vv(omega_child, &minus_a, &da);

    /* translation, h = [-a_k, 0] */
    for(int m = 0; m < 3; m++) {
      aq_relative.lin.at[m] += da.at[m] * dq_joint[k];
    }

    /* rotation, h = [t_P x -a_k, -a_k] */
    ksl_vec3_t dlin, temp;
    ksl_cross_vv(&dt_parent, &minus_a, &dlin);
    ksl_cross_vv(&d_parent->t, &da, &temp);
    ksl_xpy_vv(&temp, &dlin);
    for(int m = 0; m < 3; m++) {
      aq_relative.lin.at[m] += dlin.at[m] * dq_joint[3 + k];
      aq_relative.ang.at[m] += da.at[m] * dq_joint[3 + k];
    }
  }
  ksl_add_ss(aq_parent, &aq_relative, aq_child);
}

void shift::core::joint::Float::DeclareParameters() {
  // the initial displacement across the joint is an unnamed frame
  frame_["name"].set_required(false);
  DeclareParameter("initial_displacement", &frame_,
                   "initial displacement of the child frame with respect to "
                   "the parent frame across the joint, defaults to identity if "
                   "not specified",
                   false);
  /* dq = [v, omega], the primitive velocity of the child frame with respect
  to the parent frame across the joint, laid out as a screw */
  DeclareParameter("initial_velocity", reinterpret_cast<ksl_screw_t*>(dq_0_),
                   "initial velocity [vx, vy, vz, wx, wy, wz] of the child "
                   "frame across the joint, expressed in the child frame, "
                   "defaults to zero",
                   false);
}

void shift::core::joint::Float::ReadJSON(Json::Value& value) {
  shift::core::joint::Base::ReadJSON(value);
  /* q = [t, quaternion], see float_d */
  const ksl_SE3_t* d = frame_.get_d_wrt_origin();
  ksl_vec3_copy(&d->t, (ksl_vec3_t*)(q_0_));
  ksl_mat3x3_toQuaternion(&d->R, (ksl_quaternion_t*)(q_0_ + 3));
}

shift::core::topology::dh_fptr shift::core::joint::Float::Dfptr() {
  if(sense_ >= 0) {
    return float_d;
  } else {
    return floatinv_d;
  }
}

shift::core::topology::dh_fptr shift::core::joint::Float::DHfptr() {
  if(sense_ >= 0) {
    return float_dh;
  } else {
    return floatinv_dh;
  }
}

shift::core::topology::dhva_fptr shift::core::joint::Float::DHVAfptr() {
  if(sense_ >= 0) {
    return float_dhva;
  } else {
    return floatinv_dhva;
  }
}
