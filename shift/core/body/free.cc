#include "shift/core/body/free.h"

#include "shift/core/exception.h"
#include "shift/core/factory.h"

#include "ksl/inertia.h"
#include "ksl/mat3x3.h"
#include "ksl/vec3.h"

bool shift::core::body::Free::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "free_body", shift::core::body::Free::Create);

std::unique_ptr<shift::core::body::Free> shift::core::body::Free::Create() {
  return std::make_unique<shift::core::body::Free>();
}

void shift::core::body::Free::DeclareParameters() {
  // the initial displacement is an unnamed frame
  frame_["name"].set_required(false);
  DeclareParameter("initial_displacement", &frame_,
                   "initial displacement (R, T) of the body origin with "
                   "respect to the fixed frame, defaults to identity",
                   false);
  DeclareParameter("initial_velocity", &initial_velocity_,
                   "initial velocity [vx, vy, vz, wx, wy, wz] of the body "
                   "origin: its linear velocity and the angular velocity of "
                   "the body, in fixed coordinates, defaults to zero",
                   false);
}

void shift::core::body::Free::SetupInertia() {
  const ksl_inertia_t& I = inertia_;
  if(!(I.m > 0.0)) {
    throw shift::core::Exception("Free body '" + name_ +
                                 "' needs a positive mass");
  }
  inertia_wrt_origin_ = ksl_inertia(I.m, center_of_mass_wrt_origin_, I.Ixx,
                                    I.Iyy, I.Izz, I.Ixy, I.Iyz, I.Izx);
  ksl_mat3x3_t J;
  J.at[0][0] = I.Ixx;
  J.at[1][1] = I.Iyy;
  J.at[2][2] = I.Izz;
  J.at[0][1] = J.at[1][0] = I.Ixy;
  J.at[1][2] = J.at[2][1] = I.Iyz;
  J.at[0][2] = J.at[2][0] = I.Izx;
  /* positive definite: the leading principal minors are positive */
  const double minor2 = J.at[0][0] * J.at[1][1] - J.at[0][1] * J.at[0][1];
  ksl_mat3x3_t inverse;
  if(!(J.at[0][0] > 0.0) || !(minor2 > 0.0) ||
     ksl_mat3x3_inverted(&J, &inverse) != 0 || !(inverse.at[0][0] > 0.0)) {
    throw shift::core::Exception(
      "Free body '" + name_ +
      "' needs a positive definite centroidal moment of inertia");
  }
  inverse_inertia_ = ksl_inertia(1.0 / I.m, {{0.0, 0.0, 0.0}},
                                 inverse.at[0][0], inverse.at[1][1],
                                 inverse.at[2][2], inverse.at[0][1],
                                 inverse.at[1][2], inverse.at[0][2]);
}

ksl_screw_t shift::core::body::Free::get_initial_v() const {
  /* v.lin is the velocity of the body point at the fixed origin,
  v_origin - omega x t */
  const ksl_vec3_t& t = get_initial_d().t;
  ksl_vec3_t omega_t;
  ksl_cross_vv(&initial_velocity_.ang, &t, &omega_t);
  ksl_screw_t v = initial_velocity_;
  ksl_subtract_vv(&initial_velocity_.lin, &omega_t, &v.lin);
  return v;
}

void shift::core::body::Free::set_initial_dv(const ksl_SE3_t& d,
                                            const ksl_screw_t& v) {
  frame_.set_d_wrt_origin(d);
  ksl_vec3_t omega_t;
  ksl_cross_vv(&v.ang, &d.t, &omega_t);
  initial_velocity_ = v;
  ksl_add_vv(&v.lin, &omega_t, &initial_velocity_.lin);
}

/* the product of a symmetric 3x3 matrix, stored as the moment of inertia
terms of a ksl_inertia_t, and a vector */
static ksl_vec3_t ProductSymmetric(const ksl_inertia_t& J, const ksl_vec3_t& v) {
  return {{J.Ixx * v.x + J.Ixy * v.y + J.Izx * v.z,
           J.Ixy * v.x + J.Iyy * v.y + J.Iyz * v.z,
           J.Izx * v.x + J.Iyz * v.y + J.Izz * v.z}};
}

void shift::core::body::Free::Evaluate(const ksl_SE3_t& d,
                                       const ksl_screw_t& V,
                                       const ksl_coscrew_t& load,
                                       const ksl_vec3_t& gravity,
                                       ksl_screw_t& a) const {
  const ksl_vec3_t& v = V.lin;
  const ksl_vec3_t& omega = V.ang;

  /* spatial inertia about the fixed origin, in fixed coordinates: mass,
  m c (c the center of mass) and the moment of inertia about the origin */
  ksl_inertia_t M;
  ksl_inertia_transformed(&inertia_wrt_origin_, &d, &M);

  /* momentum h = M V: h.lin = m v + omega x m c,
  h.ang = m c x v + I_O omega */
  ksl_vec3_t h_lin, h_ang, temp;
  ksl_cross_vv(&omega, &M.mt, &h_lin);
  ksl_cross_vv(&M.mt, &v, &h_ang);
  const ksl_vec3_t I_omega = ProductSymmetric(M, omega);
  for(int k = 0; k < 3; k++) {
    h_lin.at[k] += M.m * v.at[k];
    h_ang.at[k] += I_omega.at[k];
  }

  /* f - V x* h: (V x* h).lin = omega x h.lin,
  (V x* h).ang = omega x h.ang + v x h.lin */
  ksl_vec3_t f_lin, f_ang;
  ksl_cross_vv(&omega, &h_lin, &f_lin);
  ksl_cross_vv(&omega, &h_ang, &f_ang);
  ksl_cross_vv(&v, &h_lin, &temp);
  for(int k = 0; k < 3; k++) {
    f_lin.at[k] = load.lin.at[k] - f_lin.at[k];
    f_ang.at[k] = load.ang.at[k] - f_ang.at[k] - temp.at[k];
  }

  /* a = M^-1 f by the centroidal inertia: shift the load to the center of
  mass, invert the block diagonal centroidal inertia, shift the
  acceleration back to the fixed origin */
  const double m_inverse = 1.0 / M.m;
  ksl_vec3_t c;
  ksl_product_av(m_inverse, &M.mt, &c);
  ksl_vec3_t c_f;
  ksl_cross_vv(&c, &f_lin, &c_f);
  ksl_vec3_t moment_c;
  ksl_subtract_vv(&f_ang, &c_f, &moment_c);
  ksl_inertia_t inverse; // R I_b^-1 R^T
  ksl_inertia_rotated(&inverse_inertia_, &d.R, &inverse);
  a.ang = ProductSymmetric(inverse, moment_c);
  ksl_vec3_t alpha_c;
  ksl_cross_vv(&a.ang, &c, &alpha_c);
  for(int k = 0; k < 3; k++) {
    a.lin.at[k] = f_lin.at[k] * m_inverse - alpha_c.at[k] + gravity.at[k];
  }
}
