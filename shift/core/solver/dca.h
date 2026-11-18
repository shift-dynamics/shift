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
articulated-body inertia of \f$A\f$, and the
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
\f$W\f$ is never inverted. With the QR factorization \f$H_j = Q_1R\f$ (by
Householder reflections, \f$Q = [Q_1\ N]\f$ orthogonal), the columns of \f$N\f$
span the wrenches the joint transmits, \f$H_j^TN = 0\f$, and
\f[
K = N\left(N^TWN\right)^{-1}N^T = ZZ^T, \qquad Z = NL^{-T}, \qquad
LL^T = N^TWN
\f]
of order \f$6 - n_j\f$ for a joint of \f$n_j\f$ degrees of freedom, so that
\f$\Psi^A_{m\alpha}K\Psi^A_{\alpha n} = V_mV_n^T\f$ with \f$V_m =
\Psi^A_{m\alpha}Z\f$. The wrench of the joint forces is
\f$W^{-1}H_jD^{-1}\underline{f}_j = Q\{\underline{u};\ \underline{\lambda}\}\f$
with \f$R^T\underline{u} = \underline{f}_j\f$ and \f$\underline{\lambda} =
-(N^TWN)^{-1}N^TWQ_1\underline{u}\f$. \f$K\f$ is symmetric positive
semidefinite and \f$H_j^TK = 0\f$ by construction, where the difference of
the form above loses the digits of \f$W^{-1}\f$ along the joint's motions.

The whole tree is the final assembly; its one handle is the root body, whose
joint to the fixed body is solved with \f$\Psi^A = 0\f$ and
\f$\underline{\mathbf{b}}^A = \underline{\mathbf{0}}\f$. Disassembly then
recovers, from the handle wrenches of each assembly, the wrench and
accelerations of the joint that formed it:
\f[
\underline{\mathbf{k}}^*_j = W^{-1}H_jD^{-1}\underline{f}_j -
K\underline{\mathbf{a}}^0_{\beta\alpha}, \qquad
\underline{\ddot{q}}_j = R^{-1}Q_1^T\left(W\underline{\mathbf{k}}^*_j +
\underline{\mathbf{a}}^0_{\beta\alpha}\right)
\f]
where \f$\underline{\mathbf{a}}^0_{\beta\alpha}\f$ is the relative
acceleration across the joint with \f$\underline{\mathbf{k}}^*_j =
\underline{\mathbf{0}}\f$.

A flexible body (body::Flex) and its node slots (topology::NodeSlot) form one
leaf, whose handles are the body and the node slots that carry joints to bodies
outside it. The accelerations of the leaf follow from
\f$\underline{x} = \{\underline{\mathbf{a}}_j;\ \underline{\ddot{\eta}}_j\}\f$,
the acceleration of the body's frame and of its elastic coordinates: the frame
has \f$\underline{\mathbf{a}}_j = P_j\underline{x}\f$, \f$P_j = [I\ 0]\f$,
and node slot \f$k\f$ has \f$\underline{\mathbf{a}}_k = P_k\underline{x}\f$,
\f$P_k = [I\ E_k]\f$, the columns of \f$E_k\f$ the node's influence
coefficients DHVA::he. With the inertia \f$M_k\f$ and the bias wrench
\f$\underline{\mathbf{z}}^*_k\f$ of what is welded at each node slot, the
equations of the leaf are \f$\mathcal{M}\underline{x} = \underline{\zeta} +
\sum_n P_n^T\underline{\mathbf{k}}^*_n\f$, with
\f[
\mathcal{M} = \begin{bmatrix}
M_{bb} + \sum_k M_k & M_{be} + \sum_k M_k E_k \\
\left(M_{be} + \sum_k M_k E_k\right)^T & M_{ee} + \sum_k E_k^T M_k E_k
\end{bmatrix}, \qquad
\underline{\zeta} = \begin{Bmatrix}
\underline{\mathbf{z}}^*_j + \sum_k\underline{\mathbf{z}}^*_k \\
\underline{f}_e + \underline{b}_e - M_{be}^T\underline{\mathbf{a}}^q_j +
\sum_k E_k^T\underline{\mathbf{z}}^*_k
\end{Bmatrix}
\f]
(the terms of solver::ABA), so that \f$\Psi_{mn} = P_m\mathcal{M}^{-1}P_n^T\f$
and \f$\underline{\mathbf{b}}_m = P_m\mathcal{M}^{-1}\underline{\zeta}\f$.
Once the wrenches of its handles are known from the disassembly,
\f$\underline{\ddot{\eta}}_j\f$ is the last part of
\f$\mathcal{M}^{-1}(\underline{\zeta} + \sum_n
P_n^T\underline{\mathbf{k}}^*_n)\f$. This is the two-handle form of a
flexible body in the divide-and-conquer algorithm of Mukherjee and Anderson
(2007), with any number of handles.

The assembly order is fixed when the solver is initialized: each tree is split
recursively at the joint that divides it into the two most nearly equal parts,
so the hierarchy has a depth of about \f$\log_2 n\f$ for a chain of \f$n\f$
bodies. Every body must have a positive definite spatial inertia.

A flexible body with nothing at its node slots (no mass, no children) has
only its frame as a handle and a constant \f$M_{ee}\f$, which is factored once
when the solver is initialized; its leaf is then the Schur complement
\f$\Psi = (M_{bb} - M_{be}M_{ee}^{-1}M_{eb})^{-1}\f$, with seven solves with
\f$M_{ee}\f$ at each evaluation in place of factoring \f$\mathcal{M}\f$.

The assemblies of each depth are independent of each other: they are
evaluated by several threads (OpenMP) when the estimated work of the level is
enough to pay for starting them, about 50 us per thread, and the results do
not depend on the number of threads. On a serial computer, solver::ABA solves
the same trees in fewer operations. The two agree to rounding: over a chain
of 512 bodies their state derivatives differ by about 1e-9 of the largest
(RNCA's and ABA's by 3e-8), over 64 by 2e-12.
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

  /*! \brief the threads that share the assemblies of each depth, from the
   * leaves up (1: one thread) */
  const std::vector<int>& get_level_threads() const {
    return level_threads_;
  }

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
    std::vector<double> psi;    /*!< handles^2 blocks of 6 x 6, row-major;
                                     symmetric, block mn the transpose of
                                     block nm */
    std::vector<double> bias;   /*!< 6 per handle */
    std::vector<double> wrench; /*!< 6 per handle, from disassembly */
    double h[36];               /*!< \f$H_j\f$, column-major */
    double w[36];               /*!< \f$W\f$, row-major */
    double householder[36];     /*!< the vectors \f$\underline{v}_k\f$ of
                                     the reflections \f$P_k = I -
                                     \tau_k\underline{v}_k\underline{v}_k^T\f$
                                     of \f$H_j = Q R\f$, 6 for each joint
                                     coordinate */
    double tau[6];              /*!< \f$\tau_k\f$ */
    double r_factor[36];        /*!< \f$R\f$, dof x dof, row-major */
    double z[36];               /*!< \f$Z = N L^{-T}\f$, 6 rows of 6 -
                                     dof, row-major with stride 6, \f$N\f$
                                     the last 6 - dof columns of \f$Q\f$ and
                                     \f$L L^T = N^T W N\f$; \f$K = Z Z^T\f$ */
    double k_f[6];              /*!< the joint wrench of the joint forces
                                     alone, \f$W^{-1}H_jD^{-1}\underline{f}_j\f$ */
    double beta[6]; /*!< \f$\underline{\mathbf{b}}^B_\beta -
                         \underline{\mathbf{b}}^A_\alpha\f$ */
    std::vector<double> g; /*!< scratch of an assembly: \f$G_m\f$, 6 x 6
                                for each handle of a and b */
    /* a leaf of a rigid body */
    bool has_inverse_inertia = false; /*!< inverse_inertia is cached */
    ksl_inertia_t inertia = {}; /*!< the body's inertia in its reference
                                    frame, from which inverse_inertia was made */
    ksl_inertia_t inverse_inertia = {}; /*!< 1/m in m, and the inverse of the
                                    centroidal inertia in the reference
                                    frame in the moment terms */
    /* a leaf of a flexible body */
    std::vector<int> slots;    /*!< its node slots */
    std::vector<double> mass;  /*!< Cholesky factor of \f$\mathcal{M}\f$ */
    std::vector<double> zeta;  /*!< \f$\underline{\zeta}\f$ */
    /* a flexible body without node slots, whose only handle is its frame */
    std::vector<double> ee_factor; /*!< Cholesky factor of the constant
                                        \f$M_{ee}\f$, factored once */
    std::vector<double> y_e; /*!< \f$M_{ee}^{-1}M_{eb}\f$, n_e x 6 */
    std::vector<double> x_e; /*!< \f$M_{ee}^{-1}\underline{\zeta}_e\f$ */
  };

  static bool registered_;
  static std::unique_ptr<DCA> Create();

  /*! \brief build the assembly hierarchy of a connected piece of a tree
   * (compressed bodies in level-increasing order); return its node */
  int Build(const std::vector<int>& piece);

  /*! \brief solve joint j between a subassembly with inverse inertia block psi_a
   * and bias b_a at its handle and one with psi_b, b_b at its handle, in
   * the wrenches the joint transmits: fills h, w, householder, tau,
   * r_factor, z, k_f and beta of node */
  bool SolveJoint(Node& node, const double* psi_a, const double* b_a,
                  const double* psi_b, const double* b_b);

  /*! \brief joint accelerations and wrench of node, given the relative
   * acceleration \f$\underline{\mathbf{a}}^0_{\beta\alpha}\f$ across its
   * joint with no joint wrench */
  void Disassemble(const Node& node, const double r[6], double k[6]);

  bool Assemble(Node& node);

  /*! \brief fill the h x h blocks of psi below the diagonal, and the lower
   * triangles of its diagonal blocks, from those above */
  static void MirrorBlocks(int h, double* psi);

  /*! \brief x = P x for the reflection \f$P = I - \tau\underline{v}
   * \underline{v}^T\f$ */
  static void Reflect(const double v[6], double tau, double x[6]);

  /*! \brief the inverse of the centroidal inertia of the rigid body of leaf
   * node in its reference frame, once; false (and node unchanged) if it is
   * singular */
  bool CacheInverseInertia(Node& node) const;

  /*! \brief \f$\Psi = M^{-1}\f$ of the rigid body of leaf node in fixed
   * coordinates, from its cached centroidal inverse \f$\bar{J}\f$ rotated to
   * \f$J = R\bar{J}R^T\f$ and referred to the fixed origin, c being the
   * mass centre: \f$\Psi = \begin{bmatrix} I/m + \tilde{c}J\tilde{c}^T &
   * \tilde{c}J \\ J\tilde{c}^T & J\end{bmatrix}\f$ */
  void LeafInverseInertia(const Node& node, double psi[36]) const;

  /*! \brief the wrenches of the handles of node's subassemblies, from its
   * own (a leaf: the elastic accelerations of a flexible body) */
  void DisassembleNode(Node& node);

  /*! \brief an estimate of the floating-point operations of assembling and
   * disassembling node, which sets the threads that share a level */
  double NodeWork(const Node& node) const;

  /*! \brief \f$\Psi\f$ and \f$\underline{\mathbf{b}}\f$ of the leaf of a
   * flexible body and its node slots; return true on failure */
  bool AssembleFlexible(Node& node);

  /*! \brief AssembleFlexible for a flexible body without node slots: with
   * \f$M_{ee}\f$ factored once, \f$\Psi = S^{-1}\f$ of the Schur
   * complement \f$S = M_{bb} - M_{be}M_{ee}^{-1}M_{eb}\f$ and
   * \f$\underline{\mathbf{b}} = S^{-1}(\underline{\zeta}_b -
   * M_{be}M_{ee}^{-1}\underline{\zeta}_e)\f$; return true on failure */
  bool AssembleFlexibleFrame(Node& node);

  /*! \brief \f$P_n^T\f$ of handle x of the leaf of flexible body j, a
   * \f$(6 + n_e) \times 6\f$ matrix (row-major) */
  void HandleMap(int j, int x, double* p) const;

  /*! \brief the elastic accelerations of the leaf of a flexible body, from
   * the wrenches of its handles */
  void SolveElastic(const Node& node);

  /* by compressed entry */
  std::vector<int> elastic_count_; /*!< \f$n_e\f$, 0 for a rigid body */
  std::vector<int> elastic_dq_;    /*!< index of the first elastic velocity */
  std::vector<int> slot_he_; /*!< a node slot's first DHVA::he, else -1 */
  std::vector<const shift::core::body::Base*> elastic_body_;

  std::vector<Node> nodes_;     /*!< post-order: subassemblies first */
  std::vector<std::vector<int>> levels_; /*!< the nodes of each depth */
  std::vector<int> level_threads_; /*!< threads that share each level */
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
