#ifndef SHIFT_BODY_NODE_KINEMATICS_H_
#define SHIFT_BODY_NODE_KINEMATICS_H_

#include "shift/core/topology.h"

namespace shift {
namespace core {
namespace body {

/*!
\name Kinematics of the node slots of a flexible body
The operations of a node slot (topology::NodeSlot), run in the DHVA sequence
after the operation of the flexible body's parent joint. Their offsets are:
parent.d the body's reference frame r, child.d the node's frame k, parent.v
the body's velocity index, child.v the node slot's index, q and dq the first
elastic coordinate and velocity of the body, data the node slot's index in
Context::get_node_slots(). With the node's deformed displacement
\f$D_{rk}\f$ and influence screws \f$h_{km}\f$ (in r, from
body::Base::EvaluateNode):
\f[
D_{fk} = D_{fr} D_{rk}, \qquad
\underline{\mathbf{h}}^e_{km} = [Ad]_{fr}\, h_{km}, \qquad
\underline{\mathbf{v}}_k = \underline{\mathbf{v}}_r +
  \sum_m \underline{\mathbf{h}}^e_{km}\dot{\eta}_m
\f]
\f[
\underline{\mathbf{a}}^q_k = \underline{\mathbf{a}}^q_r +
  \underline{\mathbf{v}}_r \times \sum_m \underline{\mathbf{h}}^e_{km}
  \dot{\eta}_m + [Ad]_{fr}\,\underline{\gamma}_k
\f]
the counterpart, for the elastic coordinates, of the joints'
\f$\underline{\mathbf{a}}^q_{child} = \underline{\mathbf{a}}^q_{parent} +
\underline{\mathbf{v}}_{parent} \times H\underline{\dot{q}}\f$, with the
change of the node's screws in r, \f$\underline{\gamma}_k\f$, added.
*/
///@{

/*! \brief displacement of the node frame */
void NodeD(const shift::core::Context&, topology::DHVA&,
           const topology::Offsets&, const double* q);

/*! \brief displacement and elastic influence screws of the node */
void NodeDh(const shift::core::Context&, topology::DHVA&,
            const topology::Offsets&, const double* q);

/*! \brief displacement, elastic influence screws, velocity and quadratic
 * acceleration of the node */
void NodeDhva(const shift::core::Context&, topology::DHVA&,
              const topology::Offsets&, const double* q, const double* dq);
///@}

/*!
\name Kinematics of a flexible body on a rigid joint
A flexible body keeps its own entry in the compressed tree even on a rigid
joint. Its reference frame (child.d) is placed from the joint's parent frame
(parent.d) by the joint's constant displacement, kept at offsets.q in
Context::get_d_wrt_reference(), and has the velocity and quadratic
acceleration of the parent (parent.v, child.v).
*/
///@{
void RigidEntryD(const shift::core::Context&, topology::DHVA&,
                 const topology::Offsets&, const double* q);

void RigidEntryDhva(const shift::core::Context&, topology::DHVA&,
                    const topology::Offsets&, const double* q,
                    const double* dq);
///@}

} // namespace body
} // namespace core
} // namespace shift

#endif
