#ifndef SHIFT_LINEARIZATION_H_
#define SHIFT_LINEARIZATION_H_

#include <complex>
#include <string>
#include <vector>

namespace shift {
namespace core {

class Context;

/*!
@brief The equations of motion of a model linearized about a state, and their
modes: which motions are fast, and which coordinates take part in them

The state of the linearization is
\f$\underline{x} = [\delta\underline{q}, \delta\dot{\underline{q}},
\text{free body states}, \text{auxiliary states}]\f$, with one displacement
\f$\delta q_j\f$ per velocity \f$\dot{q}_j\f$: a rotation of a spherical or
float joint is perturbed by a small rotation about the axis of its angular
velocity, not by changing its quaternion. The state matrix
\f$\mathbf{A} = \partial \dot{\underline{x}} / \partial \underline{x}\f$ has
\f$\delta\dot{\underline{q}}\f$ as the rate of \f$\delta\underline{q}\f$, and
its other rows are central differences of the solver's state derivative,
which includes the forces of the force modules and controllers and the
rates of their internal states.

A model with kinematic loops is linearized with Baumgarte constraint
stabilization off, so that the constraints hold at the acceleration level
alone: a perturbation that breaks a constraint then drifts at a constant rate
(eigenvalues 0), and the modes of the motions that keep the constraints are
those of the model. The rates of a rotating body's small rotations neglect
the body's own angular velocity, which shifts the eigenvalues by about that
much; it matters only for the slow modes.

The eigenvalues \f$\lambda\f$ of \f$\mathbf{A}\f$ are the modes: \f$\lambda
= \sigma \pm i \omega\f$ oscillates at \f$\omega\f$ (rad/s) and decays at the
rate \f$-\sigma\f$ (1/s). An explicit integrator such as ODE113 is stable for
steps \f$h\f$ with \f$h |\lambda|\f$ below about 1 for every mode, so the mode
of the largest \f$|\lambda|\f$ sets the step when the step is limited by
stability rather than accuracy. The participation of state \f$k\f$ in mode
\f$i\f$ is \f$|w_{ki} v_{ki}|\f$, with the right and left eigenvectors
\f$\underline{v}_i\f$ and \f$\underline{w}_i\f$ (\f$\underline{w}_i^T
\underline{v}_i = 1\f$); it is unitless, and the participations of a mode
add to about 1.
*/
class Linearization {
public:
  /*! \brief a mode of the linearized model */
  struct Mode {
    std::complex<double> eigenvalue;
    /*! \brief participation of each coordinate and auxiliary state
     * (get_participant_names), summed over the displacement and the velocity
     * of a coordinate */
    std::vector<double> participation;
  };

  explicit Linearization(Context& context);

  /*! \brief the state matrix at time t and state y (the integrator's state
   * vector), with differences of relative size step */
  void Compute(double t, const std::vector<double>& y, double step = 1e-6);

  /*! \brief number of states of the linearization, get_matrix() is its
   * square */
  unsigned int get_size() const {
    return size_;
  }
  /*! \brief the state matrix A, row major */
  const std::vector<double>& get_matrix() const {
    return a_;
  }
  /*! \brief names of the states of the linearization: <joint>__dq_<i> for
   * the displacements and the velocities of the coordinates, then the free
   * body and auxiliary states as <component>__state_<i> */
  const std::vector<std::string>& get_state_names() const {
    return state_names_;
  }
  /*! \brief names of the participants of the modes: the coordinates,
   * <joint>__dq_<i>, then the free body and auxiliary states */
  const std::vector<std::string>& get_participant_names() const {
    return participant_names_;
  }

  /*! \brief the modes of the state matrix, the largest |eigenvalue| first;
   * a complex pair is one mode, with the eigenvalue of positive imaginary
   * part */
  std::vector<Mode> Modes() const;

  /*! \brief a table of the count fastest modes: eigenvalue, frequency and
   * damping ratio of oscillations, the step size h = 1 / |eigenvalue|, and
   * the states that take part most */
  std::string Report(const std::vector<Mode>& modes, int count) const;

private:
  void NameStates();
  /*! \brief the state derivative at t, y, in dy */
  void Evaluate(double t, const std::vector<double>& y,
                std::vector<double>& dy);

  Context& context_;
  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  unsigned int other_count_ = 0; //!< free body and auxiliary states
  unsigned int size_ = 0;
  std::vector<double> a_;
  std::vector<std::string> state_names_;
  std::vector<std::string> participant_names_;
};

} // namespace core
} // namespace shift

#endif
