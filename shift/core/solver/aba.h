#ifndef SHIFT_SOLVER_ABA_H_
#define SHIFT_SOLVER_ABA_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/solver/articulated_body.h"

namespace shift {
namespace core {
namespace solver {

/*!
@brief Articulated-body algorithm (ABA) of Featherstone for kinematic trees

Solves the equations of motion of each tree of the model (see
solver::ArticulatedBody) in two recursions over its bodies, without forming the
generalized mass matrix. Every tree must be free of closed loops: serial
chains (topology::Tree::Kind::kSerialChain) and branched trees
(topology::Tree::Kind::kBranched).

The first recursion runs from the tips of each tree to its root. It forms the
articulated-body inertia \f$M^A_j\f$ and bias wrench
\f$\underline{\mathbf{z}}^{A*}_j\f$ of the subtree from body \f$j\f$ outward,
which relate the wrench of joint \f$j\f$ to the acceleration of body \f$j\f$,
\f$\underline{\mathbf{k}}^*_j = M^A_j\underline{\mathbf{a}}_j +
\underline{\mathbf{z}}^{A*}_j\f$:
\f[
\begin{aligned}
M^A_j &= M_j + \sum_{c \in \mathcal{C}(j)}\left(M^A_c - U_c D_c^{-1}
U_c^T\right) \\
\underline{\mathbf{z}}^{A*}_j &= -\underline{\mathbf{z}}^*_j +
\sum_{c \in \mathcal{C}(j)}\left(\underline{\mathbf{z}}^{A*}_c + U_c
D_c^{-1}\underline{u}_c\right) \\
U_j &= M^A_j H_j, \qquad D_j = H_j^T U_j, \qquad
\underline{u}_j = \underline{f}_j - H_j^T\underline{\mathbf{z}}^{A*}_j
\end{aligned}
\f]
where \f$\mathcal{C}(j)\f$ are the child bodies of \f$j\f$ (none at a
tip). The second recursion runs from the root to the tips:
\f[
\underline{\ddot{q}}_j = D_j^{-1}\left(\underline{u}_j -
U_j^T\underline{\mathbf{a}}_i\right), \qquad
\underline{\mathbf{a}}_j = \underline{\mathbf{a}}_i + H_j\underline{\ddot{q}}_j
\f]
with \f$\underline{\mathbf{a}}_f = \underline{\mathbf{0}}\f$ for the fixed
body.
*/
class ABA : public ArticulatedBody {

public:
  ABA(std::string name)
    : ArticulatedBody(name) {
    DeclareParameters();
  }
  ABA()
    : ArticulatedBody("") {
    DeclareParameters();
  }
  virtual ~ABA() override {}

  static bool is_registered() {
    return registered_;
  }

  const std::string& get_description() {
    return description_;
  }

protected:
  virtual void CheckTrees() override;
  virtual void Setup() override;
  virtual bool SolveTrees() override;

private:
  static bool registered_;
  static std::unique_ptr<ABA> Create();

  /*! \brief C += M, with M given by its ten terms */
  static void AddInertia(const ksl_inertia_t& inertia, double C[36]);

  /*! \brief u += C h for the dense row-major C; only the linear part of a
   * translational column h is read */
  static void AddProduct(const double C[36], const ksl_screw_t& h,
                         bool is_rotation, ksl_coscrew_t* u);

  /*! \f$M^A_j - M_j\f$, what the children add to the articulated-body
   * inertia, 36 per body, row-major; \f$M_j\f$ itself is kept as its ten
   * terms in inertia_ */
  std::vector<double> inertia_a_;
  std::vector<ksl_coscrew_t> bias_a_; /*!< \f$\underline{\mathbf{z}}^{A*}_j\f$ */
  std::vector<ksl_screw_t> h_;        /*!< columns of \f$H_j\f$, by dq */
  std::vector<ksl_coscrew_t> u_matrix_; /*!< columns of \f$U_j\f$, by dq */
  /*! Cholesky factor of \f$D_j\f$, or \f$1/D_j\f$ for one degree of
   * freedom */
  std::vector<double> d_factor_;
  std::vector<double> u_;      /*!< \f$\underline{u}_j\f$, by dq */
  std::vector<ksl_screw_t> a_; /*!< \f$\underline{\mathbf{a}}_j\f$ */
  std::vector<char> has_children_;
  std::vector<int> dof_;          /*!< \f$n_j\f$ */

  const std::string description_ =
    "Articulated-body algorithm (ABA):\n"
    "Featherstone's O(n) forward dynamics algorithm for kinematic trees. "
    "Each tree is solved with two passes over its bodies, without forming "
    "the generalized mass matrix: from the tips to the root to form the "
    "articulated-body inertias, then from the root to the tips to obtain the "
    "joint accelerations. Every tree of the model must be free of closed "
    "loops (serial chains and branched trees).\n"
    "\n"
    "Refer to the following publications for additional information:\n"
    "  * Featherstone R., The calculation of robot dynamics using "
    "articulated-body inertias, Int. J. Robotics Research 2(1):13-30, 1983 "
    "`<https://doi.org/10.1177/027836498300200102>`_\n"
    "\n"
    "  * Featherstone R., Rigid Body Dynamics Algorithms, Springer, 2008 "
    "`<https://doi.org/10.1007/978-1-4899-7560-7>`_";
};

} // namespace solver
} // namespace core
} // namespace shift

#endif
