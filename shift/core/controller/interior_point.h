#ifndef SHIFT_CONTROLLER_INTERIOR_POINT_H_
#define SHIFT_CONTROLLER_INTERIOR_POINT_H_

#include <cmath>
#include <utility>
#include <vector>

namespace shift {
namespace core {
namespace controller {

/*!
@brief a primal-dual interior point method for optimal control problems in
stages, by multiple shooting, after A. Wächter and L. T. Biegler (On the
implementation of an interior-point filter line-search algorithm for
large-scale nonlinear programming, Mathematical Programming 106, 2006), its
Newton steps by the Riccati recursion of C. V. Rao, S. J. Wright and J. B.
Rawlings (Application of interior-point methods to model predictive control,
Journal of Optimization Theory and Applications 99, 1998)

The problem (Problem) has N steps, the states \f$\underline{x}_k\f$ (n)
at the stations k = 0 to N, the first given, and the inputs
\f$\underline{u}_k\f$ (m) over each step:

\f[
\min \sum_{k=0}^{N-1} l_k(\underline{x}_k, \underline{u}_k) +
l_N(\underline{x}_N) \quad \text{such that} \quad
\underline{x}_{k+1} = \underline{f}_k(\underline{x}_k, \underline{u}_k),
\quad \underline{c}_k(\underline{x}_k, \underline{u}_k) \le 0, \quad
\underline{c}_N(\underline{x}_N) \le 0.
\f]

Each state is a variable, the steps \f$\underline{f}_k\f$ constraints on
them (multiple shooting, H. G. Bock and K. J. Plitt, 1984), so that an
unstable motion need not be run from the start. With the slacks
\f$\underline{s} \ge 0\f$, \f$\underline{c} + \underline{s} = 0\f$, the
barrier problem of \f$\mu\f$ adds \f$-\mu \sum \ln s_i\f$ to the cost, and
its conditions of optimality, with the multipliers \f$\underline{\lambda}_k\f$
of the steps and \f$\underline{z} \ge 0\f$ of the constraints, are

\f[
\nabla l_k + J_k^T \underline{\lambda}_{k+1} - [\underline{\lambda}_k; 0]
+ C_k^T \underline{z}_k = 0, \quad
\underline{f}_k - \underline{x}_{k+1} = 0, \quad
\underline{c} + \underline{s} = 0, \quad s_i z_i = \mu,
\f]

\f$J_k = [A_k \; B_k]\f$ and \f$C_k\f$ the Jacobians of the step and of the
constraints. Newton's method for them, the slacks' and the multipliers'
steps eliminated, leaves at each stage the Hessian
\f$\tilde H_k = H_k + C_k^T \Sigma_k C_k\f$, \f$\Sigma = Z S^{-1}\f$, and the
gradient \f$\tilde{\underline{q}}_k = \nabla l_k + C_k^T S^{-1} (\mu +
Z (\underline{c} + \underline{s}))\f$ of a linear-quadratic problem whose
dynamics are the steps' linearizations with their defects
\f$\underline{d}_k = \underline{f}_k - \underline{x}_{k+1}\f$; the Riccati
recursion solves it backward,

\f[
Q_{uu} = \tilde H_{uu} + B^T P B, \;
Q_{ux} = \tilde H_{ux} + B^T P A, \;
\underline{q}_u = \tilde{\underline{q}}_u + B^T (\underline{p} + P
\underline{d}), \;
K = -Q_{uu}^{-1} Q_{ux}, \;
\underline{k} = -Q_{uu}^{-1} \underline{q}_u,
\f]
\f[
P_k = Q_{xx} + Q_{ux}^T K, \quad \underline{p}_k = \underline{q}_x +
Q_{ux}^T \underline{k},
\f]

and forward \f$\delta\underline{u}_k = \underline{k}_k + K_k
\delta\underline{x}_k\f$, \f$\delta\underline{x}_{k+1} = A_k
\delta\underline{x}_k + B_k \delta\underline{u}_k + \underline{d}_k\f$, the
new multipliers \f$\underline{\lambda}_k = \underline{p}_k + P_k
\delta\underline{x}_k\f$. Its \f$Q_{uu}\f$ all positive definite is the
Hessian's right inertia; if one is not, \f$\delta_w\f$ is added to the
Hessian's diagonal (in the variables' scales) until they are. \f$H_k\f$ is
the Hessian of the Lagrangian \f$l_k + \underline{\lambda}_{k+1}^T
\underline{f}_k + \underline{z}_k^T \underline{c}_k\f$ by differences, or,
with gauss_newton, only the diagonal of the cost's. With project, each
stage's Hessian, in the variables' scales, has its eigenvalues below
min_curvature raised to it (by Jacobi's method), so that it is positive
definite and the Riccati recursion needs no \f$\delta_w\f$ (the
convexification of the stages' Hessians of R. Verschueren, M. Zanon, R.
Quirynen and M. Diehl, A sparsity preserving convexification procedure for
indefinite quadratic programs arising in direct optimal control, SIAM
Journal on Optimization 27, 2017, is its exact form).

The steps keep a share \f$\tau = \max(0.99, 1 - \mu)\f$ of the slacks and
the multipliers \f$\underline{z}\f$ (fraction to the boundary), and the
filter line search takes the primal step that lowers the constraints'
violation \f$\theta\f$ (the defects' and \f$\underline{c} + \underline{s}\f$'s
1-norm) or the barrier cost \f$\varphi\f$ enough, with Armijo's condition on
\f$\varphi\f$ where \f$\theta\f$ is small. The barrier's \f$\mu\f$ falls
superlinearly, \f$\mu \leftarrow \max(\epsilon / 10, \min(\kappa_\mu \mu,
\mu^{1.5}))\f$, once the barrier problem's error is below \f$10 \mu\f$, until
the error of the problem is within tolerance. The derivatives are by
central differences in the variables' scales, the stages in parallel.
*/
class InteriorPoint {
public:
  /*! \brief an optimal control problem in stages */
  class Problem {
  public:
    virtual ~Problem() = default;
    virtual int get_states() const = 0;
    virtual int get_inputs() const = 0;
    virtual int get_steps() const = 0;
    virtual int get_constraints() const = 0;     //!< of each step
    virtual int get_end_constraints() const = 0; //!< of the last state
    /*! \brief the state after step k from x with the inputs u, the step's
     * cost and its constraints (at most 0) */
    virtual void Stage(int k, const double* x, const double* u, double* next,
                       double* cost, double* c) const = 0;
    /*! \brief the last state's cost and constraints */
    virtual void End(const double* x, double* cost, double* c) const = 0;
    /*! \brief the scales of the states and the inputs, for the
     * differences and the regularization */
    virtual double StateScale(int i) const = 0;
    virtual double InputScale(int i) const = 0;
  };

  int max_iterations = 300;
  double tolerance = 1.0e-6;   //!< of the scaled error of optimality
  /*! the error, with the constraints held, at which 15 iterations in a row
   * end the solution (IPOPT's acceptable_tol), the derivatives' differences
   * limiting how small the error gets */
  double acceptable_tolerance = 1.0e-3;
  double constraint_tolerance = 1.0e-4; //!< of the constraints' violation
  double initial_barrier = 0.1;
  double difference = 1.0e-3;  //!< of the scales
  bool gauss_newton = false;   //!< the cost's diagonal only, see above
  bool project = true;         //!< each stage's Hessian made positive
  double min_curvature = 1.0e-8; //!< of a projected Hessian, scaled
  /*! the proximal term added to the Hessian's diagonal in the scales, as
   * the Levenberg-Marquardt method's: from min_proximal, five times more
   * after a step of less than 0.3 of the longest, a third after a full
   * one, to max_proximal; it changes the steps, not the solution */
  double min_proximal = 0.0;
  double max_proximal = 1.0e4;
  /*! the filter's greatest violation, over that of the first guess or 1
   * (Wächter and Biegler's 10^4, which lets a long horizon's steps stray
   * far from the motion) */
  double max_violation = 1.0;
  bool verbose = false;        //!< a line of each iteration to stderr

  /*! \brief solves from the states xs ((N + 1) n, the first the start's)
   * and the inputs us (N m) of a first guess; true if it converged */
  bool Solve(const Problem& problem, std::vector<double> xs,
             std::vector<double> us);

  const std::vector<double>& get_states() const {
    return x_;
  }
  const std::vector<double>& get_inputs() const {
    return u_;
  }
  /*! \brief the constraints at the solution, the steps' then the end's */
  const std::vector<double>& get_constraints() const {
    return c_;
  }
  double get_cost() const {
    return cost_;
  }
  /*! \brief the greatest violation of a step or a constraint */
  double get_violation() const {
    return violation_;
  }
  double get_error() const {
    return error_;
  }
  double get_barrier() const {
    return mu_;
  }
  int get_iterations() const {
    return iterations_;
  }

private:
  /*! \brief the steps, the costs and the constraints at x, u */
  void Evaluate(const Problem& p, const std::vector<double>& x,
                const std::vector<double>& u, std::vector<double>* next,
                std::vector<double>* costs, std::vector<double>* c) const;
  /*! \brief the Jacobians, the cost's gradients and the Lagrangian's
   * Hessians at the current point */
  void Linearize(const Problem& p);
  /*! \brief the Newton step for the barrier mu with the regularization
   * delta; false if a Q_uu is not positive definite */
  bool Newton(double delta);
  /*! \brief the violation (1-norm) and the barrier cost of a point */
  std::pair<double, double> Measure(const std::vector<double>& x,
                                    const std::vector<double>& next,
                                    const std::vector<double>& costs,
                                    const std::vector<double>& c,
                                    const std::vector<double>& s) const;
  /*! \brief the error of the barrier problem of mu (0: of the problem) */
  double Error(double mu) const;

  int n_ = 0, m_ = 0, steps_ = 0, mc_ = 0, me_ = 0;
  std::vector<double> state_scales_, input_scales_;
  std::vector<double> x_, u_, lambda_, s_, z_;
  std::vector<double> next_, costs_, c_;
  /* the derivatives: A, B, the costs' gradients (n + m of each step, n of
  the end), the constraints' Jacobians, the Lagrangian's Hessians */
  std::vector<double> a_, b_, g_, cj_, h_;
  /* the Newton step */
  std::vector<double> dx_, du_, ds_, dz_, lambda_new_;
  double cost_ = 0.0, violation_ = INFINITY, error_ = INFINITY, mu_ = 0.1;
  int iterations_ = 0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
