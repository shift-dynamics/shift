#ifndef SHIFT_SOLVER_DCA_H_
#define SHIFT_SOLVER_DCA_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/solver/articulated_body.h"

namespace shift {
namespace core {
namespace solver {

/*!
@brief Divide-and-conquer algorithm (DCA) of Featherstone for kinematic trees

Solves the equations of motion of each tree of the model (see solver::ArticulatedBody)
by assembling its bodies pairwise, through one joint at a time, into ever
larger subassemblies, then disassembling them in the reverse order. Every tree
must be free of closed loops (serial chains and branched trees).

A subassembly \f$A\f$ is described by its handles, the bodies of \f$A\f$ that
have joints to bodies outside it (or to the fixed body). Since all quantities
are expressed in the coordinates of the fixed frame and referred to its
origin, the acceleration of every point of a body is described by the same
screw, and the handles of a body coincide. The accelerations of the handles
are linear in the wrenches applied to them:
\f[
\underline{\mathbf{a}}_m = \sum_{n} \Psi^A_{mn}\underline{\mathbf{k}}^*_n +
\underline{\mathbf{b}}^A_m
\f]
where the \f$\Psi^A_{mn}\f$ are the \f$6 \times 6\f$ blocks of the inverse
articulated-body inertia (the mobility) of \f$A\f$, and the
\f$\underline{\mathbf{b}}^A_m\f$ its bias accelerations. A single body has
\f$\Psi = M_j^{-1}\f$ and \f$\underline{\mathbf{b}} =
M_j^{-1}\underline{\mathbf{z}}^*_j\f$.

Joint \f$j\f$ connects handle \f$\alpha\f$ of \f$A\f$ (the parent body) and
handle \f$\beta\f$ of \f$B\f$ (the child body). It applies
\f$\underline{\mathbf{k}}^*_j\f$ to \f$\beta\f$ and its reaction to
\f$\alpha\f$, with \f$\underline{\mathbf{a}}_\beta - \underline{\mathbf{a}}_\alpha
= H_j\underline{\ddot{q}}_j\f$ and \f$H_j^T\underline{\mathbf{k}}^*_j =
\underline{f}_j\f$. With
\f[
W = \Psi^A_{\alpha\alpha} + \Psi^B_{\beta\beta}, \quad
D = H_j^TW^{-1}H_j, \quad
K = W^{-1} - W^{-1}H_jD^{-1}H_j^TW^{-1}, \quad
\underline{\mathbf{k}}^*_0 = W^{-1}H_jD^{-1}\underline{f}_j -
K\left(\underline{\mathbf{b}}^B_\beta - \underline{\mathbf{b}}^A_\alpha\right)
\f]
the assembly \f$C = A \cup B\f$ has, for handles \f$m, n\f$ of \f$A\f$ and
\f$r, s\f$ of \f$B\f$,
\f[
\begin{aligned}
\Psi^C_{mn} &= \Psi^A_{mn} - \Psi^A_{m\alpha}K\Psi^A_{\alpha n}, &
\Psi^C_{ms} &= \Psi^A_{m\alpha}K\Psi^B_{\beta s}, &
\underline{\mathbf{b}}^C_m &= \underline{\mathbf{b}}^A_m -
\Psi^A_{m\alpha}\underline{\mathbf{k}}^*_0 \\
\Psi^C_{rs} &= \Psi^B_{rs} - \Psi^B_{r\beta}K\Psi^B_{\beta s}, &
\Psi^C_{rn} &= \Psi^B_{r\beta}K\Psi^A_{\alpha n}, &
\underline{\mathbf{b}}^C_r &= \underline{\mathbf{b}}^B_r +
\Psi^B_{r\beta}\underline{\mathbf{k}}^*_0
\end{aligned}
\f]
The whole tree is the final assembly; its one handle is the root body, whose
joint to the fixed body is solved with \f$\Psi^A = 0\f$ and
\f$\underline{\mathbf{b}}^A = \underline{\mathbf{0}}\f$. Disassembly then
recovers, from the handle wrenches of each assembly, the wrench and
accelerations of the joint that formed it:
\f[
\underline{\ddot{q}}_j = D^{-1}\left(\underline{f}_j +
H_j^TW^{-1}\underline{\mathbf{a}}^0_{\beta\alpha}\right), \qquad
\underline{\mathbf{k}}^*_j = W^{-1}\left(H_j\underline{\ddot{q}}_j -
\underline{\mathbf{a}}^0_{\beta\alpha}\right)
\f]
where \f$\underline{\mathbf{a}}^0_{\beta\alpha}\f$ is the relative
acceleration across the joint with \f$\underline{\mathbf{k}}^*_j =
\underline{\mathbf{0}}\f$.

The assembly order is fixed when the solver is initialized: each tree is split
recursively at the joint that divides it into the two most nearly equal parts,
so the hierarchy has a depth of about \f$\log_2 n\f$ for a chain of \f$n\f$
bodies. Every body must have a positive definite spatial inertia.

On a serial computer, solver::ABA solves the same trees in fewer operations
and with better accuracy; the divide-and-conquer algorithm pays off when the
independent assemblies of each level are evaluated in parallel.
*/
class DCA : public ArticulatedBody {

public:
  DCA(std::string name)
    : ArticulatedBody(name) {
    DeclareParameters();
  }
  DCA()
    : ArticulatedBody("") {
    DeclareParameters();
  }
  virtual ~DCA() override {}

  static bool is_registered() {
    return registered_;
  }

  const std::string& get_description() {
    return description_;
  }

  /*! \brief the depth of the assembly hierarchy of each tree, a leaf
   * (a single body) having depth 0 */
  std::vector<int> get_assembly_depths() const;

protected:
  virtual void CheckTrees() override;
  virtual void Setup() override;
  virtual bool SolveTrees() override;

private:
  /*!
  @brief A body (leaf) or the assembly of two subassemblies through one joint
  */
  struct Node {
    int body = -1;  /*!< compressed index of the body of a leaf, else -1 */
    int joint = -1; /*!< compressed index of the joint that joins a and b
                         (and of its child body, in b) */
    int a = -1;     /*!< subassembly holding the parent body of joint */
    int b = -1;     /*!< subassembly holding the child body of joint */
    int a_handle = -1; /*!< position of the parent body in a's handles */
    int b_handle = -1; /*!< position of the child body in b's handles */
    int depth = 0;
    int dof = 0; /*!< degrees of freedom of joint */
    std::vector<int> handles;
    std::vector<int> a_map; /*!< position in handles of each handle of a,
                                 -1 if internal to this assembly */
    std::vector<int> b_map;
    std::vector<double> psi;    /*!< handles^2 blocks of 6 x 6, row-major */
    std::vector<double> bias;   /*!< 6 per handle */
    std::vector<double> wrench; /*!< 6 per handle, from disassembly */
    double inverse_w[36];       /*!< \f$W^{-1}\f$ */
    double h[36];               /*!< \f$H_j\f$, column-major */
    double y[36];               /*!< \f$W^{-1}H_j\f$, column-major */
    double d_factor[36];        /*!< Cholesky factor of \f$D\f$ */
    double q_f[6];              /*!< \f$D^{-1}\underline{f}_j\f$ */
    double beta[6]; /*!< \f$\underline{\mathbf{b}}^B_\beta -
                         \underline{\mathbf{b}}^A_\alpha\f$ */
  };

  static bool registered_;
  static std::unique_ptr<DCA> Create();

  /*! \brief build the assembly hierarchy of a connected piece of a tree
   * (compressed bodies in level-increasing order); return its node */
  int Build(const std::vector<int>& piece);

  /*! \brief solve joint j between a subassembly with mobility block psi_a
   * and bias b_a at its handle and one with psi_b, b_b at its handle; fills
   * inverse_w, h, y, d_factor, q_f and beta of node, and K */
  bool SolveJoint(Node& node, const double* psi_a, const double* b_a,
                  const double* psi_b, const double* b_b, double K[36]);

  /*! \brief joint accelerations and wrench of node, given the relative
   * acceleration \f$\underline{\mathbf{a}}^0_{\beta\alpha}\f$ across its
   * joint with no joint wrench */
  void Disassemble(const Node& node, const double r[6], double k[6]);

  bool Assemble(Node& node);

  std::vector<Node> nodes_;     /*!< post-order: subassemblies first */
  std::vector<int> tops_;       /*!< the final assembly of each tree */
  std::vector<int> root_joint_; /*!< the root joint of each tree */
  std::vector<std::vector<int>> children_;

  const std::string description_ =
    "Divide-and-conquer algorithm (DCA):\n"
    "Featherstone's recursive forward dynamics algorithm for kinematic trees. "
    "Bodies are assembled pairwise through their joints into ever larger "
    "subassemblies, each described by the inverse articulated-body inertias "
    "and bias accelerations of its handles (the bodies with joints to the "
    "rest of the tree), then disassembled in reverse order to recover the "
    "joint accelerations. The assembly hierarchy is balanced, about log2(n) "
    "levels deep for n bodies, so that the assemblies of each level are "
    "independent of each other. Every tree of the model must be free of "
    "closed loops, and every body must have mass and rotational inertia.\n"
    "\n"
    "Refer to the following publications for additional information:\n"
    "  * Featherstone R., A divide-and-conquer articulated-body algorithm for "
    "parallel O(log(n)) calculation of rigid-body dynamics. Part 1: Basic "
    "algorithm, Int. J. Robotics Research 18(9):867-875, 1999\n"
    "\n"
    "  * Featherstone R., A divide-and-conquer articulated-body algorithm for "
    "parallel O(log(n)) calculation of rigid-body dynamics. Part 2: Trees, "
    "loops, and accuracy, Int. J. Robotics Research 18(9):876-892, 1999 "
    "`<https://doi.org/10.1177/02783649922066619>`_\n"
    "\n"
    "  * Mukherjee R. M., Anderson K. S., A logarithmic complexity "
    "divide-and-conquer algorithm for multi-flexible articulated body "
    "dynamics, J. Computational and Nonlinear Dynamics 2(1):10-21, 2007 "
    "`<https://doi.org/10.1115/1.2389038>`_";
};

} // namespace solver
} // namespace core
} // namespace shift

#endif
