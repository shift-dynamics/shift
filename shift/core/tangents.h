#ifndef SHIFT_TANGENTS_H_
#define SHIFT_TANGENTS_H_

#include <vector>

#include "ksl/vec3.h"

namespace shift {
namespace core {

/*!
@brief The derivatives of the generalized forces with respect to the
generalized coordinates and velocities, which the implicit integrators use in
their iteration matrix (see integrator::GeneralizedAlpha)

The tangent stiffness and damping of the applied forces
\f$\underline{Q}(\underline{q}, \underline{\dot{q}}, t)\f$ are
\f[
K = -\frac{\partial \underline{Q}}{\partial \underline{q}}, \qquad
C = -\frac{\partial \underline{Q}}{\partial \underline{\dot{q}}}
\f]
with one column per generalized velocity: a rotation of a spherical or float
joint is perturbed by a small rotation about the axis of its angular velocity,
as in Linearization, not by changing its quaternion.

A force module or controller adds its contribution in one of two forms:

- between generalized velocities, AddJointStiffness and AddJointDamping, for
  a force that acts on joint coordinates directly (a joint spring, a motor
  torque that depends on the joint rate, a PID on a joint);
- between bodies, AddBodyStiffness and AddBodyDamping, for a wrench on a body
  that depends on the displacement and twist of the same or another body (a
  bushing, a spring-damper, a contact, a tire). The blocks are \f$6 \times
  6\f$, \f$-\partial \mathbf{g}_a / \partial \underline{\delta}_b\f$ and
  \f$-\partial \mathbf{g}_a / \partial \mathbf{v}_b\f$, with the wrench
  \f$\mathbf{g}_a\f$ on body a (as passed to Context::AddBodyLoad) and the
  small displacement \f$\underline{\delta}_b\f$ and twist
  \f$\mathbf{v}_b\f$ of body b, all in fixed coordinates referred to the
  fixed origin. The solver maps them to generalized coordinates by the
  influence coefficients of the joints, \f$K_{jk} \mathrel{+}=
  \mathbf{h}_j^T K_{ab} \mathbf{h}_k\f$ for j on the path of a and k on the
  path of b.

The blocks are those of the loads for a displacement or a change of twist of
the bodies; the change of a body's twist with the derivatives of the
influence coefficients at a perturbed configuration
(\f$\partial \mathbf{h} / \partial \underline{q}\, \underline{\dot{q}}\f$) is
left out, as are the inertia forces and gravity. A module that differences
its own load (tire::Base) keeps the velocity of the body's reference point
(the hub) and its angular velocity.

A module with internal states \f$\underline{z}\f$ also gives the derivatives
of their rates with respect to the generalized coordinates and velocities,
\f$\partial \dot{z}_s / \partial \underline{q}\f$ and
\f$\partial \dot{z}_s / \partial \underline{\dot{q}}\f$, not negated: between
a state and a generalized velocity (AddStateStiffness, AddStateDamping), or
between a state and a body (AddStateBodyStiffness, AddStateBodyDamping), as a
row of six, \f$\partial \dot{z}_s / \partial \underline{\delta}_b\f$ and
\f$\partial \dot{z}_s / \partial \mathbf{v}_b\f$, mapped as the blocks are
(Context::GetLoadPath). These couple the states to the motion in the
iteration matrix (the hydraulic stiffness of the oil in a cylinder, the
twist of a driveshaft); without them a matrix from the tangents leaves the
coupling out.

The derivatives with respect to the internal states themselves are found by
finite differences, one evaluation of the equations of motion per state,
unless the module that owns the states gives them
(force_module::Base::AddStateTangents): the derivative of the load it
applies to a body, \f$\partial \underline{w}_b / \partial z_s\f$ (fixed
coordinates, referred to the fixed origin, lin first, not negated;
AddStateLoad), or of a generalized force (AddStateForce), and of the rates
of its states, \f$\partial \dot{z}_r / \partial z_s\f$ (AddStateRate),
after which AddGivenStates marks its states as given. A module with many
states (track_soil has one per station of each shoe) is then no longer
differenced state by state. The forces and the rates of the other modules
must not depend on those states.

For a force \f$\underline{f}\f$ at a point \f$\underline{p}\f$ of a body, the
small displacement of the point is \f$E \underline{\delta}\f$ with
\f$E = [I, -[\underline{p}\times]]\f$ (PointMap), and the load is
\f$E^T \underline{f}\f$: a force that changes with the displacements of the
points \f$\underline{p}_a\f$ and \f$\underline{p}_b\f$ by
\f$X = \partial \underline{f}_a / \partial \underline{p}_b\f$ gives the block
\f$-E_a^T X E_b\f$ (AddPointBlock), and the moment of the force about the
fixed origin changes with the point's own displacement by
\f$-[\underline{f}\times] E\f$ (AddMomentArmBlock).

The entries are only accumulated here, as triplets; the solver assembles
them into its iteration matrix (solver::Base::FactorIterationMatrix). A module that leaves terms out makes
Newton converge more slowly, not to a different state, since the residual is
always the full equations of motion (solver::Base::Residual).
*/
class Tangents {
public:
  /*! \brief a 6x6 block between the bodies of two load indices
   * (Context::GetBodyLoadIndex), row major, wrench rows by motion columns */
  struct BodyBlock {
    int load_index_a = -1; //!< body the wrench acts on
    int load_index_b = -1; //!< body whose motion it depends on
    double block[36] = {};
  };

  /*! \brief an entry between two generalized velocities */
  struct JointEntry {
    int dq_a = -1; //!< row, the generalized velocity the force acts on
    int dq_b = -1; //!< column, the coordinate or velocity it depends on
    double value = 0.0;
  };

  /*! \brief a derivative of the rate of an internal state with respect to
   * a generalized coordinate or velocity */
  struct StateEntry {
    int state = -1; //!< index of the internal state in y
    int dq = -1;    //!< the generalized velocity
    double value = 0.0;
  };

  /*! \brief the derivatives of the rate of an internal state with respect
   * to the displacement or the twist of a body (fixed coordinates, referred
   * to the fixed origin, lin first) */
  struct StateBodyRow {
    int state = -1;      //!< index of the internal state in y
    int load_index = -1; //!< the body (Context::GetBodyLoadIndex)
    double row[6] = {};
  };

  /*! \brief the derivative of the load on a body with respect to an
   * internal state (fixed coordinates, referred to the fixed origin, lin
   * first) */
  struct StateLoad {
    int state = -1;      //!< index of the internal state in y
    int load_index = -1; //!< the body (Context::GetBodyLoadIndex)
    double load[6] = {};
  };

  /*! \brief the derivative of the rate of an internal state with respect
   * to an internal state */
  struct StateRate {
    int state = -1; //!< index in y of the state whose rate it is
    int other = -1; //!< index in y of the state it depends on
    double value = 0.0;
  };

  /*! \brief a range of internal states whose derivatives are given */
  struct StateRange {
    int first = -1; //!< index in y of the first state
    int count = 0;
  };

  void Clear() {
    joint_stiffness_.clear();
    joint_damping_.clear();
    body_stiffness_.clear();
    body_damping_.clear();
    state_stiffness_.clear();
    state_damping_.clear();
    state_body_stiffness_.clear();
    state_body_damping_.clear();
    state_loads_.clear();
    state_forces_.clear();
    state_rates_.clear();
    given_states_.clear();
  }

  void AddJointStiffness(int dq_a, int dq_b, double k) {
    joint_stiffness_.push_back({dq_a, dq_b, k});
  }
  void AddJointDamping(int dq_a, int dq_b, double c) {
    joint_damping_.push_back({dq_a, dq_b, c});
  }
  void AddBodyStiffness(int load_index_a, int load_index_b,
                        const double block[36]) {
    body_stiffness_.push_back(MakeBlock(load_index_a, load_index_b, block));
  }
  void AddBodyDamping(int load_index_a, int load_index_b,
                      const double block[36]) {
    body_damping_.push_back(MakeBlock(load_index_a, load_index_b, block));
  }

  /*! \brief \f$\partial \dot{z}_{state} / \partial q_{dq}\f$ and
   * \f$\partial \dot{z}_{state} / \partial \dot{q}_{dq}\f$ */
  void AddStateStiffness(int state, int dq, double value) {
    state_stiffness_.push_back({state, dq, value});
  }
  void AddStateDamping(int state, int dq, double value) {
    state_damping_.push_back({state, dq, value});
  }
  /*! \brief \f$\partial \dot{z}_{state} / \partial \underline{\delta}_b\f$
   * and \f$\partial \dot{z}_{state} / \partial \mathbf{v}_b\f$ */
  void AddStateBodyStiffness(int state, int load_index, const double row[6]) {
    state_body_stiffness_.push_back(MakeRow(state, load_index, row));
  }
  void AddStateBodyDamping(int state, int load_index, const double row[6]) {
    state_body_damping_.push_back(MakeRow(state, load_index, row));
  }

  /*! \brief \f$\partial \underline{w}_b / \partial z_{state}\f$, the
   * load on the body of \p load_index, lin first, not negated */
  void AddStateLoad(int state, int load_index, const double load[6]) {
    StateLoad entry;
    entry.state = state;
    entry.load_index = load_index;
    for(int i = 0; i < 6; i++) {
      entry.load[i] = load[i];
    }
    state_loads_.push_back(entry);
  }
  /*! \brief \f$\partial f_{dq} / \partial z_{state}\f$, a generalized
   * force, not negated */
  void AddStateForce(int state, int dq, double value) {
    state_forces_.push_back({state, dq, value});
  }
  /*! \brief \f$\partial \dot{z}_{state} / \partial z_{other}\f$ */
  void AddStateRate(int state, int other, double value) {
    state_rates_.push_back({state, other, value});
  }
  /*! \brief the \p count states from index \p first of y have all their
   * derivatives given (AddStateLoad, AddStateForce, AddStateRate): the
   * integrator does not difference them */
  void AddGivenStates(int first, int count) {
    given_states_.push_back({first, count});
  }

  /*! \brief the map \f$E = [I, -[\underline{p}\times]]\f$, 3 x 6 row major,
   * from the small displacement (or twist) of a body to the displacement (or
   * velocity) of its point p */
  static void PointMap(const ksl_vec3_t& p, double e[18]) {
    const double cross[9] = {0.0,     -p.at[2], p.at[1], p.at[2], 0.0,
                             -p.at[0], -p.at[1], p.at[0], 0.0};
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++) {
        e[r * 6 + c] = r == c ? 1.0 : 0.0;
        e[r * 6 + 3 + c] = -cross[r * 3 + c];
      }
    }
  }
  /*! \brief block += scale E_a^T X E_b, with X 3 x 3 row major */
  static void AddPointBlock(const ksl_vec3_t& p_a, const double x[9],
                            const ksl_vec3_t& p_b, double scale,
                            double block[36]) {
    double e_a[18], e_b[18], x_e[18];
    PointMap(p_a, e_a);
    PointMap(p_b, e_b);
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 6; c++) {
        x_e[r * 6 + c] = 0.0;
        for(int m = 0; m < 3; m++) {
          x_e[r * 6 + c] += x[r * 3 + m] * e_b[m * 6 + c];
        }
      }
    }
    for(int r = 0; r < 6; r++) {
      for(int c = 0; c < 6; c++) {
        double value = 0.0;
        for(int m = 0; m < 3; m++) {
          value += e_a[m * 6 + r] * x_e[m * 6 + c];
        }
        block[r * 6 + c] += scale * value;
      }
    }
  }
  /*! \brief block += scale [0; [f x] E]: the stiffness of the moment about
   * the fixed origin of a force f at a point p of the body, for the point's
   * displacement */
  static void AddMomentArmBlock(const ksl_vec3_t& p, const ksl_vec3_t& f,
                                double scale, double block[36]) {
    double e[18];
    PointMap(p, e);
    const double cross[9] = {0.0,     -f.at[2], f.at[1], f.at[2], 0.0,
                             -f.at[0], -f.at[1], f.at[0], 0.0};
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 6; c++) {
        double value = 0.0;
        for(int m = 0; m < 3; m++) {
          value += cross[r * 3 + m] * e[m * 6 + c];
        }
        block[(r + 3) * 6 + c] += scale * value;
      }
    }
  }
  /*! \brief row += scale r^T E, the derivative of a scalar with the
   * gradient r (3) with respect to the displacement of the point p */
  static void AddPointRow(const double r[3], const ksl_vec3_t& p, double scale,
                          double row[6]) {
    double e[18];
    PointMap(p, e);
    for(int c = 0; c < 6; c++) {
      for(int m = 0; m < 3; m++) {
        row[c] += scale * r[m] * e[m * 6 + c];
      }
    }
  }

  const std::vector<JointEntry>& get_joint_stiffness() const {
    return joint_stiffness_;
  }
  const std::vector<JointEntry>& get_joint_damping() const {
    return joint_damping_;
  }
  const std::vector<BodyBlock>& get_body_stiffness() const {
    return body_stiffness_;
  }
  const std::vector<BodyBlock>& get_body_damping() const {
    return body_damping_;
  }

  const std::vector<StateEntry>& get_state_stiffness() const {
    return state_stiffness_;
  }
  const std::vector<StateEntry>& get_state_damping() const {
    return state_damping_;
  }
  const std::vector<StateBodyRow>& get_state_body_stiffness() const {
    return state_body_stiffness_;
  }
  const std::vector<StateBodyRow>& get_state_body_damping() const {
    return state_body_damping_;
  }
  const std::vector<StateLoad>& get_state_loads() const {
    return state_loads_;
  }
  const std::vector<StateEntry>& get_state_forces() const {
    return state_forces_;
  }
  const std::vector<StateRate>& get_state_rates() const {
    return state_rates_;
  }
  const std::vector<StateRange>& get_given_states() const {
    return given_states_;
  }

private:
  static StateBodyRow MakeRow(int state, int load_index, const double row[6]) {
    StateBodyRow state_row;
    state_row.state = state;
    state_row.load_index = load_index;
    for(int i = 0; i < 6; i++) {
      state_row.row[i] = row[i];
    }
    return state_row;
  }

  static BodyBlock MakeBlock(int load_index_a, int load_index_b,
                             const double block[36]) {
    BodyBlock body_block;
    body_block.load_index_a = load_index_a;
    body_block.load_index_b = load_index_b;
    for(int i = 0; i < 36; i++) {
      body_block.block[i] = block[i];
    }
    return body_block;
  }

  std::vector<JointEntry> joint_stiffness_;
  std::vector<JointEntry> joint_damping_;
  std::vector<BodyBlock> body_stiffness_;
  std::vector<BodyBlock> body_damping_;
  std::vector<StateEntry> state_stiffness_;
  std::vector<StateEntry> state_damping_;
  std::vector<StateBodyRow> state_body_stiffness_;
  std::vector<StateBodyRow> state_body_damping_;
  std::vector<StateLoad> state_loads_;
  std::vector<StateEntry> state_forces_;
  std::vector<StateRate> state_rates_;
  std::vector<StateRange> given_states_;
};

} // namespace core
} // namespace shift

#endif
