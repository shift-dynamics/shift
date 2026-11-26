#ifndef SHIFT_INTEGRATOR_ALPHA_H_
#define SHIFT_INTEGRATOR_ALPHA_H_

#include <cmath>
#include <string>
#include <vector>

#include "shift/core/integrator/base.h"
#include "shift/core/tangents.h"

namespace shift {
namespace core {
namespace integrator {

/*!
@brief Implicit integrators of the generalized-\f$\alpha\f$ family on the
equations of motion in all the generalized coordinates; GeneralizedAlpha and
HHT choose the coefficients

All the generalized coordinates \f$\underline{q}\f$, independent and
dependent, advance by the generalized-\f$\alpha\f$ method of Chung and
Hulbert in the form of Arnold and Brüls for constrained multibody systems:
\f[
\begin{aligned}
\underline{q}_{n+1} &= \underline{q}_n \oplus \underline{\Delta}, \qquad
\underline{\Delta} = h \underline{\dot{q}}_n + h^2 (\tfrac{1}{2} - \beta)
\underline{a}_n + h^2 \beta \underline{a}_{n+1} \\
\underline{\dot{q}}_{n+1} &= \underline{\dot{q}}_n + h (1 - \gamma)
\underline{a}_n + h \gamma \underline{a}_{n+1} \\
(1 - \alpha_m) \underline{a}_{n+1} + \alpha_m \underline{a}_n &= (1 -
\alpha_f) \underline{\ddot{q}}_{n+1} + \alpha_f \underline{\ddot{q}}_n
\end{aligned}
\f]
with the pseudo-accelerations \f$\underline{a}\f$ and \f$\oplus\f$ the
increment of Context::IncrementPositions (a rotation for a quaternion). The
other states, of the free bodies and the internal states of the force
modules and controllers, advance with pseudo-rates \f$\underline{w}\f$ by
the same recurrence (Brüls and Arnold),
\f[
\begin{aligned}
\underline{z}_{n+1} &= \underline{z}_n + h ((1 - \gamma)
\underline{w}_n + \gamma \underline{w}_{n+1}) \\
(1 - \alpha_m) \underline{w}_{n+1} + \alpha_m \underline{w}_n &= (1 -
\alpha_f) \underline{\dot{z}}_{n+1} + \alpha_f \underline{\dot{z}}_n
\end{aligned}
\f]
so that they are second order and their stiff modes are damped as the
mechanical ones are; the trapezoidal rule with weight \f$\gamma\f$ on
\f$\underline{\dot{z}}\f$ is first order for \f$\gamma \ne \tfrac{1}{2}\f$, and
made the coordinates first order through the forces of the states.

The unknowns of a step are \f$\underline{\ddot{q}}_{n+1}\f$, the
multipliers \f$\underline{\lambda}_{n+1}\f$ and
\f$\underline{\dot{z}}_{n+1}\f$. Newton's method solves the equations of
motion and the velocity constraints at the end of the step in the solver's
form (solver::Base::Residual with solver::Base::set_velocity_constraints),
\f[
M \underline{\ddot{q}}_{n+1} + B^T \underline{\lambda}_{n+1} -
\underline{f} = \underline{0}, \qquad
\frac{1}{\gamma' h} \tilde{\underline{\dot{\Phi}}}(\underline{q}_{n+1},
\underline{\dot{q}}_{n+1}) = \underline{0}
\f]
with the constraints scaled so that the iteration matrix is
\f[
S = \begin{bmatrix} M + \gamma' h C + \beta' h^2 K & B^T \\ B & 0
\end{bmatrix}, \qquad
\beta' = \beta \frac{1 - \alpha_f}{1 - \alpha_m}, \quad
\gamma' = \gamma \frac{1 - \alpha_f}{1 - \alpha_m}
\f]
(solver::Base::FactorIterationMatrix). C and K are the derivatives of the
generalized forces with respect to the velocities and the coordinates. When
the controllers and force modules give their own (Context::CollectTangents:
the joints' springs and dampers, PID on a joint coordinate or velocity,
schedules and maps, bushings, contact, terrain contact, spring-dampers,
tires, hydraulic cylinders, powertrains), S is formed from those tangents,
which leave out
the derivatives of the inertia forces, of gravity and of M and B. The columns of the generalized velocities on which a force module that
gives no tangents depends, those of the paths of the bodies it loads and the
joints it drives, are found by forward differences of the residual of the
equations of motion in place of the tangents, one column per velocity; each
holds all the terms of its column. When a controller gives no tangents (it
may read sensors anywhere), or those columns are all of them, C and K are
found by differences in every column. S is kept across iterations and
steps (modified Newton) until Newton is in trouble. When the step size
changes by more than a factor of 1.5, S is factored again for it, from the
derivatives it was formed with (C, K and those of the other states, scaled
by the new \f$\gamma' h\f$ and \f$\beta' h^2\f$) and the mass matrix and B
at the predictor, without evaluating residuals.

The rates \f$\underline{\dot{z}}\f$ of the other states are unknowns of
Newton too, with the residual \f$\underline{\dot{z}} -
\underline{f}(\underline{y})\f$: the derivatives of the equations of motion
with respect to the other states and those of f with respect to them are
found by differences, one column per state, but for the states whose
modules give them (force_module::Base::AddStateTangents, in a matrix from
the tangents), and those of f with respect to
the coordinates and velocities with the columns of a matrix by differences,
or, for a matrix from the tangents, from the derivatives of the rates the
modules give with them (Tangents::AddStateStiffness and the others). With
\f$U\f$ the derivatives of the equations of motion with respect to the
other states (times \f$\gamma' h\f$), \f$V\f$ those of their residual with
respect to the accelerations and \f$D = I - \gamma' h\, \partial
\underline{f} / \partial \underline{z}\f$, the Newton system is
\f[
\begin{bmatrix} S & U \\ V & D \end{bmatrix}
\begin{Bmatrix} \underline{x} \\ \underline{w} \end{Bmatrix} =
\begin{Bmatrix} \underline{r} \\ \underline{g} \end{Bmatrix}.
\f]
It is solved by eliminating either the coordinates and velocities or the
other states first, whichever the count of operations favors, so the other
states may be stiff (hydraulic pressures, drivelines) and many (the stations
of the shoes of a track). The first factors the Schur complement
\f$Z = D - V S^{-1} U\f$ densely, after one solve with \f$S\f$ per other
state. The second factors \f$D\f$ in its blocks of states that depend on
one another (each state alone, when every rate depends only on its own
state), and then \f$S - U D^{-1} V = S (I - S^{-1} U D^{-1} V)\f$, whose
second factor differs from \f$I\f$ only in the columns of the velocities,
after one solve with \f$S\f$ per velocity: \f$\underline{x} = (I - S^{-1} U
D^{-1} V)^{-1} S^{-1} (\underline{r} - U D^{-1} \underline{g})\f$ and
\f$\underline{w} = D^{-1} (\underline{g} - V \underline{x})\f$.

When Newton diverges, converges slowly (a correction more than 0.9 times
the last) or does not converge in max_iterations, a matrix of an earlier step
is formed again at this one, and one formed from the tangents is formed again
by differences while that has rescued steps (a running average of at least
one half, tried again at every twentieth failure otherwise); then the step is
rejected and repeated with a quarter of its size. A matrix formed again
starts from the predictor. When the partition changes, the matrix is formed
again.

The position constraints hold at the end of every step by a projection:
at each Newton iteration the dependent coordinates of the trial state are
corrected by Newton-Raphson until \f$\underline{\Phi}(\underline{q}_{n+1})
= \underline{0}\f$, with the independent ones held, before the residual is
evaluated there (ProjectPositions). The coordinates are those of the formulas
above plus a correction of the dependent ones, as in the stabilized index-2
form of Gear, Gupta and Leimkuhler applied to the generalized-alpha method
(Brüls, Acary and Cardona; Yen, Petzold and Raha for the correction of the
dependent coordinates of a partition); the iteration matrix is that of the
velocity constraints alone. There is no Baumgarte stabilization. With the
position constraints in Newton instead (the index-3 form, Arnold and Brüls),
the method is second order only for \f$\rho_\infty < 1\f$: at
\f$\rho_\infty = 1\f$ the error of the constraints left by Newton, divided
by \f$h^2\f$, enters the accelerations of closed loops undamped, and the
steps fall by orders of magnitude. The partition into independent and dependent coordinates
does not change what is integrated: Residual does not check it, and the
solver checks it, and refactors the Jacobian when a check fails, in the Solve
at the state of each accepted step, whose multipliers start the next step.

The pseudo-acceleration \f$\underline{a}_n\f$ approximates the
accelerations at \f$t_n + (\alpha_m - \alpha_f) h_n\f$, \f$h_n\f$ the size of
the step that gave it. For a step of another size \f$h\f$ its difference
from the accelerations is scaled (Jay and Negrut, eq. 2.3),
\f[
\underline{a}_n \leftarrow \underline{\ddot{q}}_n + \frac{h}{h_n}
(\underline{a}_n - \underline{\ddot{q}}_n)
\f]
so that the method stays second order when the step size changes; with
\f$\underline{a}_n\f$ unscaled it is first order on variable steps
(Brüls and Arnold).

The local error of a step is estimated by the a posteriori estimate of
Zienkiewicz and Xie for the Newmark family,
\f$e_i = |\beta - \tfrac{1}{6}|\, h^2\, |a_{n+1,i} - a_{n,i}|\f$, in each
displacement, weighted by absolute_error + relative_error \f$|\Delta_i|\f$,
and in each other state by the analogue for the pseudo-rates (from the
second step on),
\f[
e_j = c\, h\, |(w_{n+1,j} - w_{n,j}) - \tfrac{h}{h_n} (w_{n,j} - w_{n-1,j})|,
\quad
c = \left|\tfrac{1}{2}(1 - \gamma)\delta^2 + \tfrac{1}{2}\gamma(1 + \delta)^2 -
\tfrac{1}{6}\right|, \quad \delta = \alpha_m - \alpha_f
\f]
(the local error \f$c\, h^3 \dddot{z}_j\f$ of the update of \f$z\f$ with
\f$\underline{w}_n\f$ the rates at \f$t_n + \delta h_n\f$; \f$c =
\tfrac{1}{12}\f$, the trapezoidal rule's, at \f$\delta = 0\f$), weighted by
absolute_error + relative_error \f$|z_j|\f$; the step is accepted when the
root mean square of the weighted estimates is at most 1, and the next step size is \f$h \min(2, \max(0.2, 0.9\,
\epsilon^{-1/3}))\f$. A step is also rejected, and repeated with a quarter of
its size, when Newton does not converge in max_iterations with a newly formed
\f$S\f$, or when the solver fails. With the displacements alone the steps
did not follow the hydraulic pressures of the excavator example, which ended
16% off at a tolerance of 1e-4.

S is not symmetric where the forces have no potential (friction, tires,
controllers that read one joint and drive another); RNCA factors it as it is
(RNCA::FactorIterationMatrix), by a factorization of its own that leaves the
one of the explicit solve unchanged. The articulated-body solvers (ABA, DCA),
which do not form M, give the residual of the accelerations and the matrix
\f$I + M^{-1}(\gamma' h C + \beta' h^2 K)\f$, the same equations multiplied
by \f$M^{-1}\f$ (see solver::ArticulatedBody); solver::Auto couples the two
forms.

The parameters follow from the spectral radius at infinite frequency
\f$\rho_\infty \in [0, 1]\f$ (GeneralizedAlpha):
\f[
\alpha_m = \frac{2 \rho_\infty - 1}{\rho_\infty + 1}, \quad
\alpha_f = \frac{\rho_\infty}{\rho_\infty + 1}, \quad
\gamma = \tfrac{1}{2} - \alpha_m + \alpha_f, \quad
\beta = \tfrac{1}{4} (1 - \alpha_m + \alpha_f)^2
\f]
HHT-\f$\alpha\f$ (HHT) is \f$\alpha_m = 0\f$, \f$\alpha_f = -\alpha\f$ with
\f$\alpha \in [-1/3, 0]\f$. Both are second order and unconditionally stable
for linear systems. \f$\rho_\infty = 1\f$ (or \f$\alpha = 0\f$) is the
trapezoidal rule, without numerical damping.

References:
  - Hilber H. M., Hughes T. J. R., Taylor R. L., Improved numerical
    dissipation for time integration algorithms in structural dynamics,
    Earthquake Engineering and Structural Dynamics 5:283-292, 1977
  - Chung J., Hulbert G. M., A time integration algorithm for structural
    dynamics with improved numerical dissipation: the generalized-alpha
    method, Journal of Applied Mechanics 60:371-375, 1993
  - Zienkiewicz O. C., Xie Y. M., A simple error estimator and adaptive time
    stepping procedure for dynamic analysis, Earthquake Engineering and
    Structural Dynamics 20:871-887, 1991
  - Arnold M., Brüls O., Convergence of the generalized-alpha scheme for
    constrained mechanical systems, Multibody System Dynamics 18:185-202,
    2007
  - Brüls O., Arnold M., The generalized-alpha scheme as a linear multistep
    integrator: toward a general mechatronic simulator, Journal of
    Computational and Nonlinear Dynamics 3:041007, 2008
  - Jay L. O., Negrut D., Extensions of the HHT-alpha method to
    differential-algebraic equations in mechanics, Electronic Transactions
    on Numerical Analysis 26:190-208, 2007
  - Brüls O., Acary V., Cardona A., Simultaneous enforcement of constraints
    at position and velocity levels in the nonsmooth generalized-alpha
    scheme, Computer Methods in Applied Mechanics and Engineering
    281:131-161, 2014
  - Yen J., Petzold L., Raha S., A time integration algorithm for flexible
    mechanism dynamics: the DAE alpha-method, Computer Methods in Applied
    Mechanics and Engineering 158:341-355, 1998
*/
class Alpha : public Base {

public:
  Alpha(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  Alpha()
    : Base("") {
    DeclareParameters();
  }
  virtual ~Alpha() override {}

  virtual void get_current_time(double& time, double& delta_time) override;

  virtual void Initialize() override;
  virtual void Connect() override;
  virtual void Integrate() override;
  virtual void Reset();

  /*! \brief the number of accepted and rejected steps of the last
   * Integrate, and the number of times the iteration matrix was formed */
  unsigned int get_accepted_steps() const {
    return accepted_steps_;
  }
  /*! \brief the iteration matrices factored with the other states
   * eliminated first (see the class description) */
  unsigned int get_states_first_count() const {
    return states_first_count_;
  }
  unsigned int get_rejected_steps() const {
    return rejected_steps_;
  }
  unsigned int get_iteration_matrix_count() const {
    return iteration_matrix_count_;
  }
  unsigned int get_newton_iterations() const {
    return newton_iterations_;
  }
  unsigned int get_tangent_matrix_count() const {
    return tangent_matrix_count_;
  }
  /*! \brief of the matrices from the tangents, those with the columns of
   * the modules that give no tangents found by differences */
  unsigned int get_mixed_matrix_count() const {
    return mixed_matrix_count_;
  }
  /*! \brief the times a matrix was factored again, scaled for a new step
   * size, without being formed again */
  unsigned int get_rescaled_matrix_count() const {
    return rescaled_matrix_count_;
  }

protected:
  virtual void DeclareParameters() override;

  /*! \brief set alpha_m_, alpha_f_, gamma_ and beta_ from the parameters of
   * the method */
  virtual void SetCoefficients() = 0;

  double alpha_m_ = 0.0;
  double alpha_f_ = 0.0;
  double gamma_ = 0.5;
  double beta_ = 0.25;

private:
  /*! \brief attempt one step of size h from (t_, y_); \p truncated when h
   * was shortened to land on a reporting time. On success advance the state
   * and return true; on failure leave it, reduce h_ and return false. */
  bool Step(double h, bool truncated);

  /*! \brief the trial state y_trial_ at the end of the step of size h for
   * the unknowns ddq_trial_, la_trial_ and dz_trial_, by the formulas of the
   * method with the coordinates projected onto the position constraints
   * (ProjectPositions), and the residual r_ and the state derivatives dy_ at
   * time t from solver::Base::Residual. Returns true on solver failure. */
  bool EvaluateTrial(double t, double h);

  /*! \brief the dependent coordinates of y_trial_ corrected by Newton-Raphson
   * until the position constraints hold (Jacobian::ProjectPositions at the
   * tolerance epsilon_rank_determination), with the independent ones held; the context's
   * coordinates are left as they were. Throws shift::core::Exception when
   * Newton-Raphson fails. */
  void ProjectPositions();

  /*! \brief the constant c of the local error of the other states, c h^3
   * times their third derivative (see the class documentation) */
  double StateErrorCoefficient() const;

  /*! \brief how an iteration matrix is formed: from the tangents of the
   * joints, controllers and force modules (Context::CollectTangents), when
   * each gives its own, or by differences of the residual */
  enum class MatrixKind { kTangent, kDifferenced };

  /*! \brief form the derivatives of the iteration matrix at the trial state
   * (the tangents, the columns by differences and those of the other
   * states) and factor it for the step size h (FactorIterationMatrix);
   * leaves r_ and dy_ those of the trial state. Returns true on failure. */
  bool FormIterationMatrix(double t, double h, MatrixKind kind);

  /*! \brief factor the iteration matrix for the step size h from the
   * derivatives of the last FormIterationMatrix, with the mass matrix and B
   * of the last Residual, and the Schur complement of the other states; no
   * residuals are evaluated. Returns true on failure. */
  bool FactorIterationMatrix(double h);

  /*! \brief one Newton correction: x holds the negative residual of the
   * equations of motion and the scaled constraints, w that of the other
   * states' rates, dz - f; on return the corrections of the accelerations
   * and multipliers, and of the rates. Returns true when the iteration matrix
   * no longer fits the solver's equations (the partition changed). */
  bool SolveNewton(std::vector<double>& x, std::vector<double>& w);
  /*! \brief factor \f$D\f$ in its blocks and the matrix \f$I - S^{-1}
   * U D^{-1} V\f$ of the coordinates and velocities, for the elimination of
   * the other states first (see the class description) */
  bool FactorStatesFirst(double gamma_h);
  /*! \brief \f$D^{-1}\f$ applied to \p data, other_count_ rows of \p
   * columns each (row major), by the blocks of FactorStatesFirst */
  bool SolveStateBlocks(double* data, int columns) const;

  /*! \brief the weight of the displacement of velocity i in the error and
   * convergence norms */
  double Weight(unsigned int i) const {
    return absolute_error_ + relative_error_ * std::fabs(delta_[i]);
  }

  /* parameters */
  double absolute_error_ = 1e-6;
  double relative_error_ = 1e-4;
  double hmax_ = 0.0; //!< 0: time_step_
  /*! the longest step: hmax_, or time_step_, and no longer than the terrain
   * allows (Context::GetStepLimit) */
  double step_max_ = 0.0;
  int max_iterations_ = 6;

  /* constants of the method */
  static constexpr double kNewtonTolerance = 1e-2; //!< of the error weights
  static constexpr double kDivergenceRate = 0.9;
  static constexpr double kMinStepSize = 1e-12;
  /*! the iteration matrix is factored again, scaled for the new step
   * size from the derivatives it was formed with, when the step size has
   * changed by more than this factor since it was factored */
  static constexpr double kReformFactor = 1.5;
  /*! after Newton fails with a matrix from the tangents, the step is
   * tried with one by differences while such rescues have succeeded at
   * least at this rate (a running average with the weight below), and
   * otherwise rejected, except at every kRescueProbe-th failure, which tries
   * again: a model whose tangents leave out terms that matter (spinning
   * wheels) is rescued, one where both fail (contact that begins or ends
   * within the step) is not charged for differences */
  static constexpr double kRescueRate = 0.5;
  static constexpr double kRescueWeight = 0.2;
  static constexpr unsigned int kRescueProbe = 20;

  /* state */
  double t_ = 0.0;
  double h_ = 0.0;
  unsigned int neqn_ = 0;
  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  unsigned int other_offset_ = 0; //!< index of the other states in y
  unsigned int other_count_ = 0;
  std::vector<double> y_;      //!< accepted state at t_
  std::vector<double> ddq_;    //!< accelerations at t_
  std::vector<double> a_;      //!< pseudo-accelerations at t_
  /*! a_ scaled to the size of the step being taken (see Step) */
  std::vector<double> a_step_;
  std::vector<double> w_;       //!< pseudo-rates of the other states at t_
  std::vector<double> w_step_;  //!< w_ scaled to the step being taken
  std::vector<double> w_trial_;
  /*! w_ less w_step_ of the last accepted step, of size a_step_size_ */
  std::vector<double> dw_;
  bool has_dw_ = false;
  std::vector<double> projection_q_; //!< the context's q, kept by ProjectPositions
  double a_step_size_ = 0.0; //!< size of the step that gave a_; 0 at t = 0
  std::vector<double> la_;     //!< multipliers at t_ (starting values)
  std::vector<double> dz_;     //!< rates of the other states at t_
  std::vector<double> y_trial_;
  std::vector<double> ddq_trial_;
  std::vector<double> la_trial_;
  std::vector<double> dz_trial_;
  std::vector<double> a_trial_;
  std::vector<double> delta_;  //!< displacements of the step
  std::vector<double> r_;      //!< residual at the trial state
  std::vector<double> dy_;     //!< state derivatives at the trial state
  std::vector<double> rhs_;
  /* the derivatives the iteration matrix is formed from, without the
  factors of the step size: the columns of K and C by differences (dq_count
  by matrix_columns_.size(), row major) and their velocities, with the
  tangents_ in the other columns; the derivatives of the rates of the other
  states with respect to the coordinates and the velocities (other_count by
  dq_count); and those of the residual of the equations of motion (dq_count
  per state, by state) and of the rates (other_count by other_count) with
  respect to the other states */
  std::vector<double> k_columns_;
  std::vector<double> c_columns_;
  std::vector<int> matrix_columns_;
  std::vector<double> state_stiffness_;
  std::vector<double> state_damping_;
  std::vector<double> force_state_columns_;
  std::vector<double> rate_state_block_;
  std::vector<int> all_velocities_;
  bool s_current_ = false;     //!< an iteration matrix is factored
  double s_step_size_ = 0.0;   //!< the step size it was formed for
  MatrixKind matrix_kind_ = MatrixKind::kTangent; //!< how it was formed
  double rescue_rate_ = 1.0;
  unsigned int tangent_failures_ = 0;
  Tangents tangents_;
  /* the other states in the Newton system: W = S^-1 U (column by column, of
  coupled_size_ each), the rows V (other_count_ by dq_count_), and the LU
  factors of the Schur complement D - V W */
  std::vector<double> coupling_;
  std::vector<double> state_rows_;
  std::vector<double> schur_;
  std::vector<int> pivots_;
  size_t coupled_size_ = 0;
  /* the elimination of the other states first (FactorStatesFirst): the
  states by blocks of D (block k from block_begin_[k] of block_states_), the
  LU factors of the blocks from block_lu_begin_[k] of block_lu_ with their
  pivots (a block of one state holds its single entry), D^-1 V (other_count_
  by dq_count_), and the LU factors of I - S^-1 U D^-1 V in schur_ and
  pivots_; block_buffer_ holds the rows of a block while it is solved */
  bool states_first_ = false;
  double gamma_h_ = 0.0;
  std::vector<int> block_states_;
  std::vector<size_t> block_begin_;
  std::vector<size_t> block_lu_begin_;
  std::vector<double> block_lu_;
  std::vector<int> block_pivots_;
  std::vector<double> solved_rows_;
  mutable std::vector<double> block_buffer_;
  std::vector<double> state_rhs_;

  unsigned int accepted_steps_ = 0;
  unsigned int rejected_steps_ = 0;
  unsigned int iteration_matrix_count_ = 0;
  unsigned int newton_iterations_ = 0;
  unsigned int tangent_matrix_count_ = 0;
  unsigned int differenced_matrix_count_ = 0;
  unsigned int states_first_count_ = 0;
  /*! the matrices from the tangents with some columns by differences, of
   * the modules that give no tangents */
  unsigned int mixed_matrix_count_ = 0;
  /*! the times a matrix was factored again for a new step size */
  unsigned int rescaled_matrix_count_ = 0;
  /*! the generalized velocities of the last matrix whose columns were
   * found by differences in place of the tangents (Context::CollectTangents) */
  std::vector<int> mixed_columns_;
};

} // namespace integrator
} // namespace core
} // namespace shift

#endif
