#include "shift/core/body/node_kinematics.h"

#include "ksl/SE3.h"
#include "ksl/matrix.h"
#include "ksl/screw.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"

void shift::core::body::NodeD(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* q) {
  const auto& slot = context.get_node_slots()[offsets.data];
  ksl_SE3_t d_rk;
  slot.body->EvaluateNode(slot.node, q + offsets.q, nullptr, &d_rk, nullptr,
                          nullptr);
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
  slot.body->EvaluateNode(slot.node, q + offsets.q, nullptr, &d_rk, he,
                          nullptr);
  const ksl_SE3_t* d_fr = &dhva.d[offsets.parent.d];
  ksl_product_dd(d_fr, &d_rk, &dhva.d[offsets.child.d]);
  /* the screws in r, transformed to fixed coordinates in place */
  for(unsigned int m = 0; m < n_e; m++) {
    const ksl_screw_t h_r = he[m];
    ksl_product_Ads(d_fr, &h_r, &he[m]);
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
  ksl_screw_t gamma_r;
  slot.body->EvaluateNode(slot.node, q + offsets.q, eta_dot, &d_rk, he,
                          &gamma_r);
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
