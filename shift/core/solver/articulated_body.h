#ifndef SHIFT_SOLVER_ARTICULATED_BODY_H_
#define SHIFT_SOLVER_ARTICULATED_BODY_H_

#include <string>
#include <vector>

#include "ksl/coscrew.h"
#include "ksl/inertia.h"
#include "ksl/screw.h"

#include "shift/core/solver/base.h"
#include "shift/core/topology.h"

namespace shift {
namespace core {
namespace solver {

/*!
@brief Base class of the articulated-body solvers for kinematic trees (solver::ABA,
solver::DCA)

A tree has no closed loops, so its equations of motion need no constraint
Jacobian: they follow from the spatial equations of motion of each body and the
joints that connect them. All quantities are expressed in the coordinates of the
fixed frame \f$f\f$ and referred to its origin, as in the DHVA arrays.

For compressed body \f$j\f$ with parent \f$i\f$, the influence coefficient
matrix \f$H_j\f$ (\f$6 \times n_j\f$, the columns
\f$\underline{\mathbf{h}}\f$ of its joint) maps the joint velocities to the
relative velocity across the joint. The acceleration of the body is the
quadratic acceleration \f$\underline{\mathbf{a}}^q_j\f$ (computed with the
velocities, \f$\underline{\ddot{q}} = \underline{0}\f$, and including the
acceleration \f$-\underline{g}\f$ of the fixed frame that accounts for gravity)
plus the part produced by the joint accelerations,
\f[
\underline{\dot{\mathbf{v}}}_j = \underline{\mathbf{a}}^q_j +
\underline{\mathbf{a}}_j, \qquad
\underline{\mathbf{a}}_j = \underline{\mathbf{a}}_i + H_j\underline{\ddot{q}}_j,
\qquad \underline{\mathbf{a}}_f = \underline{\mathbf{0}}
\f]
The spatial equations of motion of the body then read
\f[
M_j\underline{\mathbf{a}}_j - \underline{\mathbf{z}}^*_j =
\underline{\mathbf{k}}^*_j - \sum_{c \in \mathcal{C}(j)}\underline{\mathbf{k}}^*_c,
\qquad H_j^T\underline{\mathbf{k}}^*_j = \underline{f}_j
\f]
where \f$\underline{\mathbf{k}}^*_j\f$ is the wrench that joint \f$j\f$ applies
to body \f$j\f$, \f$\mathcal{C}(j)\f$ are the child bodies of \f$j\f$,
\f$\underline{f}_j\f$ the working forces of joint \f$j\f$, and the bias wrench
\f[
\underline{\mathbf{z}}^*_j = \underline{\mathbf{g}}^*_j -
M_j\underline{\mathbf{a}}^q_j - [ad]^*_j M_j\underline{\mathbf{v}}_j
\f]
collects the applied wrench \f$\underline{\mathbf{g}}^*_j\f$, the inertia
wrench of the quadratic acceleration and the gyroscopic wrench, with
\f$[ad]^*_j\f$ formed from \f$\underline{\mathbf{v}}_j\f$.
*/
class ArticulatedBody : public Base {

public:
  ArticulatedBody(std::string name)
    : Base(name) {}
  virtual ~ArticulatedBody() override {}

  virtual void Initialize() override;

  virtual bool Solve(const double t, const std::vector<double>& y,
                     std::vector<double>& dy) override;

  virtual double
  GravitationalPotentialEnergy(shift::core::topology::DHVA& dhva) override;
  virtual double KineticEnergy(shift::core::topology::DHVA& dhva) override;

  /*! \brief restrict the solver to the trees of Context::get_trees() with
   * these indices, as solver::Auto does for the trees without closed loops;
   * by default it solves every tree. Call before Initialize(). */
  void set_trees(const std::vector<int>& trees) {
    tree_indices_ = trees;
    all_trees_ = false;
  }

  /*! \brief evaluate the bias wrenches and solve the joint accelerations of
   * the solver's trees, for the current DHVA and forces, writing them to
   * Context::get_ddq(); return true on failure */
  bool SolveAccelerations();

protected:
  /*! \brief check that the solver can solve every tree of the model; throw
   * shift::core::Exception otherwise */
  virtual void CheckTrees() = 0;

  /*! \brief allocate the arrays of the solver, called once the trees pass
   * CheckTrees() */
  virtual void Setup() = 0;

  /*! \brief solve for the joint accelerations, given the inertias, bias
   * wrenches and influence coefficients of the current state, and write them
   * to Context::get_ddq(); return true on failure */
  virtual bool SolveTrees() = 0;

  /*! \brief evaluate \f$M_j\f$, \f$\underline{\mathbf{p}}^*_j\f$ and
   * \f$\underline{\mathbf{z}}^*_j\f$ for each compressed body */
  void SetupBodyWrenches();

  /*! \brief \f$M_j\f$, \f$\underline{\mathbf{p}}^*_j\f$ and
   * \f$\underline{\mathbf{z}}^*_j\f$ of compressed body j */
  void SetupBodyWrench(int j);

  /*! \brief copy the influence coefficients of compressed joint j into the
   * columns of H (column-major, 6 x n_j) and return n_j. The angular part of
   * a translational degree of freedom is zero. */
  int JointColumns(int j, double H[36]) const;

  /*! \brief the name of the root joint of a tree, for messages */
  std::string RootName(const shift::core::topology::Tree& tree) const;

  /*! \brief the dense 6 x 6 matrix (row-major) of a spatial inertia, acting
   * on a screw \f$\{\underline{v};\ \underline{\omega}\}\f$ to give a coscrew
   * \f$\{\underline{f};\ \underline{\tau}\}\f$ */
  static void InertiaMatrix(const ksl_inertia_t& inertia, double M[36]);

  /*! \brief in-place Cholesky factorization \f$A = LL^T\f$ of the symmetric
   * positive definite n x n matrix A (row-major, leading dimension n); L
   * overwrites the lower triangle. Return false if A is not positive
   * definite relative to the largest diagonal term. */
  static bool CholeskyFactor(int n, double* A);

  /*! \brief solve \f$LL^T\underline{x} = \underline{b}\f$ in place, with L
   * from CholeskyFactor */
  static void CholeskySolve(int n, const double* L, double* x);

  /*! \brief the inverse of a symmetric positive definite 6 x 6 matrix;
   * return false if it is not positive definite */
  static bool InvertSymmetric6(const double A[36], double inverse[36]);

  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  unsigned int body_count_ = 0; /*!< compressed body count */

  std::vector<ksl_inertia_t> inertia_; /*!< \f$M_j\f$, in fixed coordinates */
  std::vector<ksl_coscrew_t> p_;       /*!< spatial momentum */
  std::vector<ksl_coscrew_t> z_;       /*!< bias wrenches */

  /*! the trees this solver solves; the other entries of inertia_, p_ and
   * z_ stay zero */
  std::vector<shift::core::topology::Tree> trees_;

  /*! the bodies of trees_, tree after tree, each in level-increasing order */
  std::vector<int> bodies_;

private:
  std::vector<int> tree_indices_;
  bool all_trees_ = true;
};

} // namespace solver
} // namespace core
} // namespace shift

#endif
