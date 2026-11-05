#include "shift/core/joint/gear.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

#include <math.h>

bool shift::core::joint::Gear::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "gear", shift::core::joint::Gear::Create);

std::unique_ptr<shift::core::joint::Gear> shift::core::joint::Gear::Create() {
  return std::make_unique<shift::core::joint::Gear>();
}

void shift::core::joint::Gear::Initialize() {
  dof_ = 1;
  q_count_ = 1;
  dq_count_ = 1;
  dc_count_ = 0;
  q_0_ = new double[q_count_]();
  dq_0_ = new double[dq_count_]();
  rotation_indices_.push_back(0);
  is_rotation_ = true;
  DeclareParameters();
}

/* The child frame is at the position
  [separation cos(q), separation sin(q), 0]
of the parent frame, turned by (1 + ratio) q about its z axis. Seen from the
child, the parent frame is turned by -(1 + ratio) q, at
  [-separation cos(ratio q), separation sin(ratio q), 0]. */
void shift::core::joint::Gear::Displace(const ksl_SE3_t* d_parent,
                                        ksl_SE3_t* d_child, const double* data,
                                        bool inverse, double q) {
  const double separation = data[0], ratio = data[1];
  const ksl_vec3_t z_axis = ksl_vec3(0.0, 0.0, 1.0);
  ksl_vec3_t offset;
  ksl_mat3x3_t rotation;
  if(!inverse) {
    offset = ksl_vec3(separation * cos(q), separation * sin(q), 0.0);
    ksl_mat3x3_setFromAxisAngle(&rotation, &z_axis, (1.0 + ratio) * q);
  } else {
    offset = ksl_vec3(-separation * cos(ratio * q),
                      separation * sin(ratio * q), 0.0);
    ksl_mat3x3_setFromAxisAngle(&rotation, &z_axis, -(1.0 + ratio) * q);
  }
  ksl_product_ddt(d_parent, &offset, d_child);
  ksl_product_drdr(&d_parent->R, &rotation, &d_child->R);
}

/* In the frame of the body that is the parent in the traversal, the child
turns about the z axis at (1 + ratio) q', and the point of the child at the
origin of the parent frame has the velocity
  q' * ratio * separation * [sin(q), -cos(q), 0]
(inverted: -(1 + ratio) q' and -q' * separation * [sin(ratio q),
cos(ratio q), 0]). The derivative of that screw with respect to q gives the
quadratic acceleration. */
void shift::core::joint::Gear::JointScrews(const ksl_SE3_t* d_parent,
                                           const double* data, bool inverse,
                                           double q, ksl_screw_t* h,
                                           ksl_screw_t* dh_dq) {
  const double separation = data[0], ratio = data[1];
  ksl_vec3_t ang_local, lin_local, dlin_local;
  if(!inverse) {
    ang_local = ksl_vec3(0.0, 0.0, 1.0 + ratio);
    lin_local = ksl_vec3(ratio * separation * sin(q),
                         -ratio * separation * cos(q), 0.0);
    dlin_local = ksl_vec3(ratio * separation * cos(q),
                          ratio * separation * sin(q), 0.0);
  } else {
    ang_local = ksl_vec3(0.0, 0.0, -(1.0 + ratio));
    lin_local = ksl_vec3(-separation * sin(ratio * q),
                         -separation * cos(ratio * q), 0.0);
    dlin_local = ksl_vec3(-ratio * separation * cos(ratio * q),
                          ratio * separation * sin(ratio * q), 0.0);
  }
  ksl_vec3_t rotated, moment;
  ksl_product_drv(&d_parent->R, &ang_local, &h->ang);
  ksl_product_drv(&d_parent->R, &lin_local, &rotated);
  ksl_cross_vv(&d_parent->t, &h->ang, &moment);
  ksl_add_vv(&rotated, &moment, &h->lin);
  dh_dq->ang = ksl_vec3(0.0, 0.0, 0.0);
  ksl_product_drv(&d_parent->R, &dlin_local, &dh_dq->lin);
}

void shift::core::joint::gear_d(const shift::core::Context& context,
                                topology::DHVA& dhva,
                                const topology::Offsets& offsets,
                                const double* q) {
  shift::core::joint::Gear::Displace(
    &dhva.d[offsets.parent.d], &dhva.d[offsets.child.d],
    &context.get_joint_data()[offsets.data], false, q[offsets.q]);
}

void shift::core::joint::gearinv_d(const shift::core::Context& context,
                                   topology::DHVA& dhva,
                                   const topology::Offsets& offsets,
                                   const double* q) {
  shift::core::joint::Gear::Displace(
    &dhva.d[offsets.parent.d], &dhva.d[offsets.child.d],
    &context.get_joint_data()[offsets.data], true, q[offsets.q]);
}

static void GearDh(const shift::core::Context& context,
                   shift::core::topology::DHVA& dhva,
                   const shift::core::topology::Offsets& offsets,
                   const double* q, bool inverse, ksl_screw_t* dh_dq) {
  const double* data = &context.get_joint_data()[offsets.data];
  shift::core::joint::Gear::Displace(&dhva.d[offsets.parent.d],
                                     &dhva.d[offsets.child.d], data, inverse,
                                     q[offsets.q]);
  shift::core::joint::Gear::JointScrews(&dhva.d[offsets.parent.d], data,
                                        inverse, q[offsets.q],
                                        &dhva.h[offsets.dq], dh_dq);
}

void shift::core::joint::gear_dh(const shift::core::Context& context,
                                 topology::DHVA& dhva,
                                 const topology::Offsets& offsets,
                                 const double* q) {
  ksl_screw_t dh_dq;
  GearDh(context, dhva, offsets, q, false, &dh_dq);
}

void shift::core::joint::gearinv_dh(const shift::core::Context& context,
                                    topology::DHVA& dhva,
                                    const topology::Offsets& offsets,
                                    const double* q) {
  ksl_screw_t dh_dq;
  GearDh(context, dhva, offsets, q, true, &dh_dq);
}

/* The velocity of the child is that of the parent plus h q', and the
quadratic acceleration that of the parent plus V x h q' (the screw h
moves with the parent) plus (dh/dq) q'^2 (h changes as the child moves). */
static void GearDhva(const shift::core::Context& context,
                     shift::core::topology::DHVA& dhva,
                     const shift::core::topology::Offsets& offsets,
                     const double* q, const double* dq, bool inverse) {
  ksl_screw_t dh_dq;
  GearDh(context, dhva, offsets, q, inverse, &dh_dq);
  const double rate = dq[offsets.dq];
  ksl_screw_t* h = &dhva.h[offsets.dq];
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t aq_change = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t aq_sum = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_product_as(rate, h, &v_relative);
  ksl_add_ss(v_parent, &v_relative, v_child);
  ksl_cross_ss(v_child, &v_relative, &aq_relative);
  ksl_product_as(rate * rate, &dh_dq, &aq_change);
  ksl_add_ss(&aq_relative, &aq_change, &aq_sum);
  ksl_add_ss(aq_parent, &aq_sum, aq_child);
}

void shift::core::joint::gear_dhva(const shift::core::Context& context,
                                   topology::DHVA& dhva,
                                   const topology::Offsets& offsets,
                                   const double* q, const double* dq) {
  GearDhva(context, dhva, offsets, q, dq, false);
}

void shift::core::joint::gearinv_dhva(const shift::core::Context& context,
                                      topology::DHVA& dhva,
                                      const topology::Offsets& offsets,
                                      const double* q, const double* dq) {
  GearDhva(context, dhva, offsets, q, dq, true);
}

void shift::core::joint::Gear::DeclareParameters() {
  DeclareParameter("separation", &separation_,
                   "Distance between the axis of the parent gear, the z axis "
                   "of the parent frame, and the axis of the child gear: the "
                   "sum of the pitch radii of an external pair, their "
                   "difference for a gear inside a ring.",
                   true);
  DeclareParameter("gear_ratio", &ratio_,
                   "Radius of the parent gear over the radius of the child "
                   "gear (the ratio of their numbers of teeth). It is "
                   "negative when the child runs inside the parent, a ring "
                   "gear. As the line of centers turns by an angle q about "
                   "the parent frame, the child turns relative to the parent "
                   "frame by (1 + gear_ratio) q.",
                   true);
  DeclareParameter("initial_displacement", q_0_,
                   "Initial angle of the line of centers from the x axis of "
                   "the parent frame, in radians.",
                   false);
  DeclareParameter("initial_velocity", dq_0_,
                   "Initial angular velocity of the line of centers.", false);
  DeclareParameter(
    "damping_coefficients", &damping_coefficients_,
    "specify damping coefficients a, where the torque due to damping is "
    "computed as a_0 + a_1 * \\dot{x}^1 + a_2 * \\dot{x}^2 + .. a_n * "
    "\\dot{x}^n\\",
    false)
    .set_row_dim(-1);
  DeclareParameter("stiffness_coefficients", &stiffness_coefficients_,
                   "specify stiffness coefficients a, where the torque due to "
                   "stiffness is computed as a_0 + a_1 * x^1 + a_2 * x^2 + .. "
                   "a_n * x^n\\",
                   false)
    .set_row_dim(-1);
}

const double* shift::core::joint::Gear::get_data() {
  data_[0] = separation_;
  data_[1] = ratio_;
  return data_;
}

unsigned int shift::core::joint::Gear::get_data_count() {
  return 2;
}

shift::core::topology::dh_fptr shift::core::joint::Gear::Dfptr() {
  return sense_ >= 0 ? gear_d : gearinv_d;
}

shift::core::topology::dh_fptr shift::core::joint::Gear::DHfptr() {
  return sense_ >= 0 ? gear_dh : gearinv_dh;
}

shift::core::topology::dhva_fptr shift::core::joint::Gear::DHVAfptr() {
  return sense_ >= 0 ? gear_dhva : gearinv_dhva;
}

void shift::core::joint::Gear::Service(const std::vector<double>& y) {
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
