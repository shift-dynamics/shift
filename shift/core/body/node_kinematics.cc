#include "shift/core/body/node_kinematics.h"

#include "ksl/SE3.h"
#include "ksl/matrix.h"
#include "ksl/screw.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"

/* the slot's coordinate and rate of its sliding joint, s = sense q */
static double SlideAt(const shift::core::topology::NodeSlot& slot,
                      const double* x) {
  return slot.slide_q >= 0 ? slot.slide_sense * x[slot.slide_q] : 0.0;
}

void shift::core::body::NodeD(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* q) {
  const auto& slot = context.get_node_slots()[offsets.data];
  ksl_SE3_t d_rk;
  if(slot.slide_q >= 0) {
    slot.body->EvaluateSlidingNode(slot.node, SlideAt(slot, q), 0.0,
                                   q + offsets.q, nullptr, &d_rk, nullptr,
                                   nullptr, nullptr, nullptr);
  } else {
    slot.body->EvaluateNode(slot.node, q + offsets.q, nullptr, &d_rk, nullptr,
                            nullptr);
  }
  ksl_product_dd(&dhva.d[offsets.parent.d], &d_rk, &dhva.d[offsets.child.d]);
}

void shift::core::body::NodeDh(const shift::core::Context& context,
                               shift::core::topology::DHVA& dhva,
                               const shift::core::topology::Offsets& offsets,
                               const double* q) {
  const auto& slot = context.get_node_slots()[offsets.data];
  const unsigned int n_e = slot.body->get_elastic_count();
  ksl_screw_t* he = dhva.he.data() + slot.he_offset;
  ksl_SE3_t d_rk;
  ksl_screw_t h_s;
  if(slot.slide_q >= 0) {
    slot.body->EvaluateSlidingNode(slot.node, SlideAt(slot, q), 0.0,
                                   q + offsets.q, nullptr, &d_rk, he, nullptr,
                                   &h_s, nullptr);
  } else {
    slot.body->EvaluateNode(slot.node, q + offsets.q, nullptr, &d_rk, he,
                            nullptr);
  }
  const ksl_SE3_t* d_fr = &dhva.d[offsets.parent.d];
  ksl_product_dd(d_fr, &d_rk, &dhva.d[offsets.child.d]);
  /* the screws in r, transformed to fixed coordinates in place */
  for(unsigned int m = 0; m < n_e; m++) {
    const ksl_screw_t h_r = he[m];
    ksl_product_Ads(d_fr, &h_r, &he[m]);
  }
  if(slot.slide_q >= 0) {
    ksl_screw_t h_f;
    ksl_product_Ads(d_fr, &h_s, &h_f);
    ksl_product_as(slot.slide_sense, &h_f,
                   &dhva.slide[size_t(slot.slide_offset)]);
  }
}

void shift::core::body::NodeDhva(const shift::core::Context& context,
                                 shift::core::topology::DHVA& dhva,
                                 const shift::core::topology::Offsets& offsets,
                                 const double* q, const double* dq) {
  const auto& slot = context.get_node_slots()[offsets.data];
  const unsigned int n_e = slot.body->get_elastic_count();
  const double* eta_dot = dq + offsets.dq;
  ksl_screw_t* he = dhva.he.data() + slot.he_offset;
  ksl_SE3_t d_rk;
  ksl_screw_t gamma_r, h_s, h_s_dot;
  if(slot.slide_q >= 0) {
    slot.body->EvaluateSlidingNode(slot.node, SlideAt(slot, q),
                                   SlideAt(slot, dq), q + offsets.q, eta_dot,
                                   &d_rk, he, &gamma_r, &h_s, &h_s_dot);
  } else {
    slot.body->EvaluateNode(slot.node, q + offsets.q, eta_dot, &d_rk, he,
                            &gamma_r);
  }
  const ksl_SE3_t* d_fr = &dhva.d[offsets.parent.d];
  ksl_product_dd(d_fr, &d_rk, &dhva.d[offsets.child.d]);

  /* elastic part of the node's velocity, in fixed coordinates */
  ksl_screw_t v_e = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  for(unsigned int m = 0; m < n_e; m++) {
    const ksl_screw_t h_r = he[m];
    ksl_product_Ads(d_fr, &h_r, &he[m]);
    ksl_axpy_ss(eta_dot[m], &he[m], &v_e);
  }

  const ksl_screw_t& v_r = dhva.v[offsets.parent.v];
  ksl_add_ss(&v_r, &v_e, &dhva.v[offsets.child.v]);

  /* aq_k = aq_r + v_r x v_e + Ad_fr gamma_r */
  ksl_screw_t v_r_x_v_e;
  ksl_cross_ss(&v_r, &v_e, &v_r_x_v_e);
  ksl_screw_t gamma;
  ksl_product_Ads(d_fr, &gamma_r, &gamma);
  ksl_screw_t& aq_k = dhva.aq[offsets.child.v];
  ksl_add_ss(&dhva.aq[offsets.parent.v], &v_r_x_v_e, &aq_k);
  ksl_xpy_ss(&gamma, &aq_k);

  /* the sliding joint's H = sense Ad_fr h_s and its rate,
  H_dot = v_r x H + sense Ad_fr h_s_dot */
  if(slot.slide_q >= 0) {
    ksl_screw_t& H = dhva.slide[size_t(slot.slide_offset)];
    ksl_screw_t& H_dot = dhva.slide[size_t(slot.slide_offset) + 1];
    ksl_screw_t h_f;
    ksl_product_Ads(d_fr, &h_s, &h_f);
    ksl_product_as(slot.slide_sense, &h_f, &H);
    ksl_screw_t rate;
    ksl_product_Ads(d_fr, &h_s_dot, &rate);
    ksl_cross_ss(&v_r, &H, &H_dot);
    ksl_axpy_ss(slot.slide_sense, &rate, &H_dot);
  }
}

void shift::core::body::SlidingJointDh(
  const shift::core::Context& context, shift::core::topology::DHVA& dhva,
  const shift::core::topology::Offsets& offsets, const double* q) {
  const auto& slot =
    context.get_node_slots()[size_t(context.GetNodeSlotAt(offsets.parent.v))];
  slot.joint_dh(context, dhva, offsets, q);
  dhva.h[offsets.dq] = dhva.slide[size_t(slot.slide_offset)];
}

void shift::core::body::SlidingJointDhva(
  const shift::core::Context& context, shift::core::topology::DHVA& dhva,
  const shift::core::topology::Offsets& offsets, const double* q,
  const double* dq) {
  const auto& slot =
    context.get_node_slots()[size_t(context.GetNodeSlotAt(offsets.parent.v))];
  slot.joint_dhva(context, dhva, offsets, q, dq);
  /* the joint's displacement as it is; its screw, velocity and quadratic
  acceleration with the sliding: v = v_slot + H q_dot, aq = aq_slot +
  H_dot q_dot */
  const double q_dot = dq[offsets.dq];
  const ksl_screw_t& H = dhva.slide[size_t(slot.slide_offset)];
  const ksl_screw_t& H_dot = dhva.slide[size_t(slot.slide_offset) + 1];
  dhva.h[offsets.dq] = H;
  ksl_screw_t& v = dhva.v[offsets.child.v];
  v = dhva.v[offsets.parent.v];
  ksl_axpy_ss(q_dot, &H, &v);
  ksl_screw_t& aq = dhva.aq[offsets.child.v];
  aq = dhva.aq[offsets.parent.v];
  ksl_axpy_ss(q_dot, &H_dot, &aq);
}

void shift::core::body::RigidEntryD(
  const shift::core::Context& context, shift::core::topology::DHVA& dhva,
  const shift::core::topology::Offsets& offsets, const double*) {
  ksl_product_dd(&dhva.d[offsets.parent.d],
                 &context.get_d_wrt_reference()[offsets.q],
                 &dhva.d[offsets.child.d]);
}

void shift::core::body::RigidEntryDhva(
  const shift::core::Context& context, shift::core::topology::DHVA& dhva,
  const shift::core::topology::Offsets& offsets, const double* q,
  const double*) {
  RigidEntryD(context, dhva, offsets, q);
  dhva.v[offsets.child.v] = dhva.v[offsets.parent.v];
  dhva.aq[offsets.child.v] = dhva.aq[offsets.parent.v];
}
