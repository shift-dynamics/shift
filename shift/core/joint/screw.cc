#include "shift/core/joint/screw.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

#include <math.h>

bool shift::core::joint::Screw::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "screw", shift::core::joint::Screw::Create);

std::unique_ptr<shift::core::joint::Screw> shift::core::joint::Screw::Create() {
  return std::make_unique<shift::core::joint::Screw>();
}

void shift::core::joint::Screw::Initialize() {
  dof_ = 1;
  q_count_ = 1;
  dq_count_ = 1;
  dc_count_ = 0; // the rotation is computed from q, the advance needs q too
  q_0_ = new double[q_count_]();
  dq_0_ = new double[dq_count_]();
  rotation_indices_.push_back(0);
  is_rotation_ = true;
  DeclareParameters();
}

void shift::core::joint::Screw::Displace(const ksl_SE3_t* d_parent,
                                         ksl_SE3_t* d_child, const double* data,
                                         double sign, double q) {
  const ksl_vec3_t axis = ksl_vec3(sign * data[0], sign * data[1], sign * data[2]);
  ksl_mat3x3_t rotation;
  ksl_mat3x3_setFromAxisAngle(&rotation, &axis, q);
  ksl_vec3_t advance;
  ksl_product_av(data[3] * q, &axis, &advance);
  ksl_product_ddt(d_parent, &advance, d_child);
  ksl_product_drdr(&d_parent->R, &rotation, &d_child->R);
}

void shift::core::joint::Screw::JointScrew(const ksl_SE3_t* d_child,
                                           const double* data, double sign,
                                           ksl_screw_t* h) {
  const ksl_vec3_t axis = ksl_vec3(sign * data[0], sign * data[1], sign * data[2]);
  ksl_vec3_t moment;
  ksl_vec3_t advance;
  ksl_product_drv(&d_child->R, &axis, &h->ang);
  ksl_cross_vv(&d_child->t, &h->ang, &moment);
  ksl_product_av(data[3], &h->ang, &advance);
  ksl_add_vv(&moment, &advance, &h->lin);
}

void shift::core::joint::screw_d(const shift::core::Context& context,
                                 topology::DHVA& dhva,
                                 const topology::Offsets& offsets,
                                 const double* q) {
  shift::core::joint::Screw::Displace(
    &dhva.d[offsets.parent.d], &dhva.d[offsets.child.d],
    &context.get_joint_data()[offsets.data], 1.0, q[offsets.q]);
}

void shift::core::joint::screwinv_d(const shift::core::Context& context,
                                    topology::DHVA& dhva,
                                    const topology::Offsets& offsets,
                                    const double* q) {
  shift::core::joint::Screw::Displace(
    &dhva.d[offsets.parent.d], &dhva.d[offsets.child.d],
    &context.get_joint_data()[offsets.data], -1.0, q[offsets.q]);
}

void shift::core::joint::screw_dh(const shift::core::Context& context,
                                  topology::DHVA& dhva,
                                  const topology::Offsets& offsets,
                                  const double* q) {
  const double* data = &context.get_joint_data()[offsets.data];
  shift::core::joint::Screw::Displace(&dhva.d[offsets.parent.d],
                                      &dhva.d[offsets.child.d], data, 1.0,
                                      q[offsets.q]);
  shift::core::joint::Screw::JointScrew(&dhva.d[offsets.child.d], data, 1.0,
                                        &dhva.h[offsets.dq]);
}

void shift::core::joint::screwinv_dh(const shift::core::Context& context,
                                     topology::DHVA& dhva,
                                     const topology::Offsets& offsets,
                                     const double* q) {
  const double* data = &context.get_joint_data()[offsets.data];
  shift::core::joint::Screw::Displace(&dhva.d[offsets.parent.d],
                                      &dhva.d[offsets.child.d], data, -1.0,
                                      q[offsets.q]);
  shift::core::joint::Screw::JointScrew(&dhva.d[offsets.child.d], data, -1.0,
                                        &dhva.h[offsets.dq]);
}

/* The screw of the joint is constant in the parent frame, so the
velocity and the quadratic acceleration follow as for any joint with a fixed
screw. */
static void ScrewVelocity(shift::core::topology::DHVA& dhva,
                          const shift::core::topology::Offsets& offsets,
                          const double* dq) {
  ksl_screw_t* h = &dhva.h[offsets.dq];
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_product_as(dq[offsets.dq], h, &v_relative);
  ksl_add_ss(v_parent, &v_relative, v_child);
  ksl_cross_ss(v_child, &v_relative, &aq_relative);
  ksl_add_ss(aq_parent, &aq_relative, aq_child);
}

void shift::core::joint::screw_dhva(const shift::core::Context& context,
                                    topology::DHVA& dhva,
                                    const topology::Offsets& offsets,
                                    const double* q, const double* dq) {
  shift::core::joint::screw_dh(context, dhva, offsets, q);
  ScrewVelocity(dhva, offsets, dq);
}

void shift::core::joint::screwinv_dhva(const shift::core::Context& context,
                                       topology::DHVA& dhva,
                                       const topology::Offsets& offsets,
                                       const double* q, const double* dq) {
  shift::core::joint::screwinv_dh(context, dhva, offsets, q);
  ScrewVelocity(dhva, offsets, dq);
}

void shift::core::joint::Screw::DeclareParameters() {
  DeclareParameter("axis", &axis_,
                   "Axis of the screw, expressed in the parent frame, "
                   "defaults to the z axis if none is given.",
                   false);
  DeclareParameter("pitch", &pitch_,
                   "Pitch of the screw: the distance the child advances along "
                   "the axis per radian it turns about it, in length units per "
                   "radian (a lead of L per turn is a pitch of L / 2 pi). "
                   "The sign sets the handedness: a positive pitch is a "
                   "right-handed screw.",
                   true);
  DeclareParameter("initial_displacement", q_0_,
                   "Initial angle of the screw, in radians.", false);
  DeclareParameter("initial_velocity", dq_0_,
                   "Initial angular velocity of the screw.", false);
  DeclareParameter(
    "damping_coefficients", &damping_coefficients_,
    "specify damping coefficients a, where the force due to damping is "
    "computed as a_0 + a_1 * \\dot{x}^1 + a_2 * \\dot{x}^2 + .. a_n * "
    "\\dot{x}^n\\",
    false)
    .set_row_dim(-1);
  DeclareParameter("stiffness_coefficients", &stiffness_coefficients_,
                   "specify stiffness coefficients a, where the force due to "
                   "stiffness is computed as a_0 + a_1 * x^1 + a_2 * x^2 + .. "
                   "a_n * x^n\\",
                   false)
    .set_row_dim(-1);
}

const double* shift::core::joint::Screw::get_data() {
  ksl_vec3_normalize(&axis_);
  data_[0] = axis_.x;
  data_[1] = axis_.y;
  data_[2] = axis_.z;
  data_[3] = pitch_;
  return data_;
}

unsigned int shift::core::joint::Screw::get_data_count() {
  return 4;
}

shift::core::topology::dh_fptr shift::core::joint::Screw::Dfptr() {
  return sense_ >= 0 ? screw_d : screwinv_d;
}

shift::core::topology::dh_fptr shift::core::joint::Screw::DHfptr() {
  return sense_ >= 0 ? screw_dh : screwinv_dh;
}

shift::core::topology::dhva_fptr shift::core::joint::Screw::DHVAfptr() {
  return sense_ >= 0 ? screw_dhva : screwinv_dhva;
}

void shift::core::joint::Screw::Service(const std::vector<double>& y) {
  std::vector<double>& f = context_->get_joint_working_forces();
  const double q = y[q_index_];
  for(unsigned int i = 0; i < stiffness_coefficients_.size(); i++) {
    f[dq_index_] -= stiffness_coefficients_[i] * pow(q, i);
  }
  const double dq = y[context_->get_q_count() + dq_index_];
  for(unsigned int i = 0; i < damping_coefficients_.size(); i++) {
    f[dq_index_] -= damping_coefficients_[i] * pow(dq, i);
  }
}
