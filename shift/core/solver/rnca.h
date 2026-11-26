#ifndef SHIFT_SOLVER_RNCA_H_
#define SHIFT_SOLVER_RNCA_H_

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "shift/core/array2d.h"
#include "shift/core/topology.h"
#include "shift/core/solver/base.h"
#include "shift/core/substructure.h"

namespace shift {
namespace core {
namespace solver {

/*!
@brief Abstract base class for shift::Solver C++ objects
*/
class RNCA : public Base {

public:
  RNCA(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  RNCA()
    : Base("") {
    DeclareParameters();
  }
  virtual ~RNCA() override {}

  static bool is_registered() {
    return registered_;
  }

  const std::string& get_description() {
    return description_;
  }

  /*! @brief get time derivative of constraint equations */
  const std::vector<ksl_screw_t>& get_de() {
    return de_;
  }

  void PrintStates(std::stringstream&);
  virtual double
  GravitationalPotentialEnergy(shift::core::topology::DHVA& dhva) override;
  virtual double KineticEnergy(shift::core::topology::DHVA& dhva) override;

  /*! memoize equations of motion in array of source and destination pointers */
  void Memoize();

  /*! set generalized mass matrix for equations of motion */
  void SetupMassMatrixAndRecordFills();
  void SetupMassMatrixAndRecordFills(const double y[]);

  /*! set generalized mass matrix for equations of motion without setting /
   * evaluating the boolean mass matrix Mlb_*/
  void SetupMassMatrix();

  /*! set right hand side of equations of motion - generalized forces and
  preconditioned quadratic acceleration terms */
  void SetupRHS();

  bool SetupEquations();

  /*! generate mass matrix and right hand side of preconditioned
  special equations of motion
   */
  bool SetupEquations(const double t, const std::vector<double>& y);
  bool SetupEquationsAndRecordFills();
  bool SetupEquationsAndRecordFills(const double t,
                                    const std::vector<double>& y);

  /*! \brief factor the block-diagonalized preconditioned spatial equations
   * of motion using run-time optimized natural constraint algorithm
   * \f[
   * \begin{bmatrix}
   *   M^{\ell}_{ii} & M^{\ell}_{id} & B_{di}^T \\
   *   M^{\ell}_{di} & M^{\ell}_{dd} & I \\
   *   B_{di} & I & 0
   * \end{bmatrix}
   * \begin{Bmatrix} \underline{\ddot{q}}_i \\ \underline{\ddot{q}}_d \\
   * \underline{\lambda}^d \end{Bmatrix} =
   * \begin{Bmatrix} \underline{f}^{\ell}_i \\ \underline{f}^{\ell}_d \\
   * \underline{\gamma}^d \end{Bmatrix}
   * \f]
   * in place in Ml_, by the operations recorded by Memoize;
   * ForwardBackwardSubstitution then solves for the right-hand sides fl_
   * and gamma_d_, as many times as needed before the next Factor
   */
  void Factor();

  /*! \brief solve the equations factored by the last Factor for the
   * right-hand sides fl_ (permutation order 3) and gamma_d_, which are
   * overwritten, into the accelerations (Context::get_ddq) and the
   * multipliers la_: backward elimination \f$\underline{b} \leftarrow
   * L^{-T} \underline{b}\f$, the diagonal \f$\underline{x} \leftarrow
   * D^{-1} \underline{b}\f$, then forward substitution \f$\underline{x}
   * \leftarrow L^{-1} \underline{x}\f$ */
  void ForwardBackwardSubstitution();

  virtual bool Solve(const double t, const std::vector<double>& y,
                     std::vector<double>& dy) override;

  virtual void Initialize() override;

  virtual void set_velocity_constraints(bool velocity) override {
    velocity_constraints_ = velocity;
  }

  /*! \brief implicit integration is supported by RNCA alone for models
   * without variables delegated to another solver; solver::Auto supports
   * those */
  virtual bool supports_implicit() const override {
    return std::find(delegated_dq_.begin(), delegated_dq_.end(), true) ==
           delegated_dq_.end();
  }

  /*! \brief the number of dependent coordinates of the current partition */
  virtual unsigned int get_multiplier_count() const override {
    return dependent_count_;
  }

  /*! \brief the multipliers la_ of the last Solve, one per dependent
   * coordinate in permutation order 3 */
  virtual void Multipliers(std::vector<double>& la) const override;

  /*! \brief the residual of the preconditioned equations of motion (see
   * Factor) at t and y for ddq (velocity order) and la (as Multipliers):
   * \f$M^{\ell} \underline{\ddot{q}} + B^T \underline{\lambda}^d -
   * \underline{f}^{\ell}\f$ in the rows of the generalized velocities
   * (velocity order), then the Newton-Raphson corrections of the dependent
   * coordinates (Jacobian::EvaluateNewtonRaphsonCorrections) in the order of
   * the multipliers, or with set_velocity_constraints the preconditioned
   * velocity constraints \f$\dot{q}_d + B_{di}\dot{q}_i\f$ in that order. The equations are set up as in SetupEquations, without
   * its condition check, so that the partition does not change; they are
   * not factored. The rows of variables delegated to another solver are
   * zero, for it to fill (solver::Auto). Returns true on failure. */
  virtual bool Residual(const double t, const std::vector<double>& y,
                        const std::vector<double>& ddq,
                        const std::vector<double>& la, std::vector<double>& r,
                        std::vector<double>& dy) override;

  /*! \brief add c C + k K (see solver::Base) to the mass matrix of the
   * last Residual and factor it with the matrix B of that Residual, by the
   * non-symmetric factorization of the iteration matrix (see Iteration
   * matrix), which leaves the memoized factorization of Solve as it is. The entries between generalized velocities are added where
   * they fall; the 6x6 blocks between bodies are mapped by the influence
   * coefficients of the joints on the paths of the two bodies to the fixed
   * body, \f$K_{lk} \mathrel{+}= \mathbf{h}_l^T K_{ab} \mathbf{h}_k\f$ (the
   * blocks of free bodies and of the node slots of flexible bodies are left
   * out). The factors are kept apart from the arrays of Residual, so that
   * SolveIterationMatrix may follow further Residuals. The rows and
   * columns of variables delegated to another solver are left out, with a 1
   * on the diagonal, for solver::Auto to couple to its own. Returns true on
   * failure. */
  virtual bool FactorIterationMatrix(const Tangents& tangents, double c,
                                     double k,
                                     const std::vector<int>& differenced,
                                     const std::vector<double>& k_columns,
                                     const std::vector<double>& c_columns)
    override;

  /*! \brief solve with the factors saved by FactorIterationMatrix (see
   * solver::Base); the accelerations of the context and the multipliers la_
   * are left as they were */
  virtual bool SolveIterationMatrix(std::vector<double>& rhs) override;

protected:
  virtual void DeclareParameters() override;

  /*! \brief solve the joint accelerations of the delegated trees (see
   * delegated_body_), after RNCA has solved the rest; return true on
   * failure. RNCA itself delegates nothing. */
  virtual bool SolveDelegated() {
    return false;
  }

  /*! \brief the compressed bodies, and the generalized velocities in
   * permutation orders 1 and 3, of the trees that another solver handles
   * (see solver::Auto). Their rows and columns are left out of the mass
   * matrix, so they add no operations to the factorization, and their
   * accelerations are not written. All false for RNCA. */
  std::vector<bool> delegated_body_;
  std::vector<bool> delegated_dq_;
  std::vector<bool> delegated_po3_;
  /*! the bodies (compressed indices) of the flexible bodies RNCA solves,
   * those not delegated, when some are delegated; the elastic kinetic
   * energy of the delegated ones is the tree solver's */
  std::vector<int> elastic_entries_;
  bool has_delegated_elastic_ = false;
  /*! the rows of the multipliers of Residual are the velocity constraints
   * (see Base::set_velocity_constraints) */
  bool velocity_constraints_ = false;

private:
  static bool registered_;
  static std::unique_ptr<RNCA> Create();

  /*! \brief the entries of the pattern of the mass matrix before the fills
   * of the factorization (Mlbs_), as lists, for products with the mass
   * matrix (Residual); called by Memoize after CountOperations */
  void RecordMassPattern();
  void CountOperations();
  void RecordOperations();
  void CountPointers();
  void RecordPointers();
  bool Verify(const double t, const std::vector<double>& y, double Ml[],
              double fl[]);

  /*! \name Flexible bodies
  The elastic coordinates of a flexible body b (body::Flex) are further
  columns of the joint-space equations. A generalized velocity below b moves
  b's elastic coordinate m through the influence screw
  \f$\mathbf{h}_{km}\f$ of the node k (node slot) through which it hangs
  from b, so in the walk up pcl_dq its entry in that coordinate's row is
  \f$\mathbf{h}_{km}^T (M^c_j\mathbf{h}_j)\f$. The column of elastic
  coordinate m holds the momentum \f$M_{be,m} + \sum_k M^c_k
  \mathbf{h}_{km}\f$, with \f$M^c_k\f$ the inertia accumulated below node
  slot k, and the elastic-elastic block of b is \f$M_{ee} + \sum_k
  \mathbf{h}_k^T M^c_k \mathbf{h}_k\f$. The right-hand side of elastic
  coordinate m is \f$b_{e,m} - M_{be,m}^T \mathbf{a}^q_b + \sum_k
  \mathbf{h}_{km}^T \mathbf{g}_k\f$, with \f$\mathbf{g}_k\f$ the wrench
  accumulated below node slot k, and b's bias wrench is added to its own.
  All quantities are in fixed coordinates, referred to the fixed origin. For
  a model of rigid bodies these functions do nothing.
  */
  ///@{
  /*! \brief the lookups below; throws for a closed loop through a flexible
   * body, whose chord Jacobian does not have the elastic columns yet */
  void SetupElasticLookup();
  /*! \brief the momenta of the elastic columns, after the accumulated
   * inertias, before the walk up pcl_dq */
  void SetElasticMomenta();
  /*! \brief the mass matrix entries in the rows of elastic coordinates,
   * after the walk up pcl_dq, which leaves them zero (the screws dhva.h of
   * elastic coordinates are zero) */
  void AddElasticMassTerms(bool record_fills);
  /*! \brief the right-hand sides of the elastic coordinates, after the
   * wrenches are accumulated */
  void AddElasticForces();
  /*! \brief the node slot through which the walk up pcl_dq, at velocity
   * \p c, enters the elastic coordinates of another body at its parent
   * velocity \p i; -1 if it does not enter one */
  int EnteredNodeSlot(int c, int i) const;
  /*! \brief index in Ml_ (permutation order 3, lower triangle) of the entry
   * of the generalized velocities \p a and \p b (permutation order 1) */
  int MassIndex(int a, int b) const;

  bool has_elastic_ = false;
  /*! per generalized velocity: the index in Context::get_elastic_bodies() of
   * the body whose elastic coordinate it is, or -1; and that coordinate's
   * index in the body */
  std::vector<int> elastic_body_of_dq_;
  std::vector<int> elastic_mode_of_dq_;
  /*! per generalized velocity: the compressed joint it belongs to */
  std::vector<int> entry_of_dq_;
  /*! per compressed joint: its index in Context::get_node_slots(), or -1 */
  std::vector<int> slot_of_entry_;
  /*! per flexible body: its node slots */
  std::vector<std::vector<int>> slots_of_body_;
  /*! \f$M^c_k \mathbf{h}_{km}\f$, laid out as topology::DHVA::he */
  std::vector<ksl_coscrew_t> mhe_;
  ///@}

  double infinity_norm_multiplier_ = 2.0;
  bool infinity_norm_failure_ = false;
  /* what refactors the Jacobian (Substructure::Refactor), as named in the
  model file, and the mobility number of an independent velocity divided by
  the mobility below which Refactor::MOBILITY_EPSILON does so */
  std::string refactor_criterion_name_ = "infinity_norm";
  Substructure::Refactor refactor_criterion_ =
    Substructure::Refactor::INFINITY_NORM;
  double epsilon_mobility_ = 0.01;

  unsigned int dependent_count_ = 0;
  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;

  std::vector<double>
    gamma_d_; /*!< preconditioned constraints, this quantity contains both
                 \f$\underline{\gamma}^p\f$ and \f$\underline{\gamma}^d\f$ */

  int Ml_size_ = 0; /*!< size of Ml array = \f$(2 + n)(1 + n) / 2\f$ */

  std::vector<double> Ml_; /*!< Generalized mass matrix, linear array containing
                    mass  matrix terms on and below the diagonal */
  std::vector<double> Mls_; /*!< Saved copy of original M */
  std::vector<bool>
    Mlb_; /*!< boolean mass matrix with fills rescorded, equal in size to M */
  std::vector<bool>
    Mlbs_; /*!< original boolean mass matrix, with no fills recorded */
  /*! the entries of Mlbs_, recorded by RecordMassPattern: the index in Ml_,
   * and the row and column (permutation order 3, row >= column) */
  std::vector<int> mass_pattern_index_;
  std::vector<int> mass_pattern_row_;
  std::vector<int> mass_pattern_column_;

  /*! \name Iteration matrix
  FactorIterationMatrix factors \f$S\f$ by a sparse LU factorization of its
  own, separate from the memoized symmetric factorization of the equations
  of motion (Factor), of which it reads only the mass matrix of the last
  Residual (the entries of Mlbs_) and the matrix B. The pivots are those of
  Factor, in permutation order 3 from the last variable to the first: an
  independent variable i is eliminated by its diagonal entry,
  \f$A_{jk} \mathrel{-}= (A_{ji} / A_{ii}) A_{ik}\f$, and a dependent one
  by the identity in its constraint row and in its own row of
  \f$B^T\f$, which forms the rows and the columns of the variables
  \f$j\f$ of its row of B,
  \f$A_{jk} \mathrel{-}= B_{ij} A_{ik}\f$ (with \f$k = i\f$), then
  \f$A_{kj} \mathrel{-}= A_{ki} B_{ij}\f$. The fill-in follows from the
  pattern of the entries and is the same in the upper and the lower
  triangle, so the pattern is kept symmetric and an entry (r, c) and its
  transpose (c, r) each have a value. The pattern and the operations are
  recorded (RecordIterationPattern) when the partition changes (Memoize)
  or an entry falls outside the pattern, which then grows by the entries
  of all the matrices since the last partition change.
  */
  ///@{
  /*! \brief add the entry (a, b) of A = cC + kK (velocity order) to the
   * entries of the iteration matrix */
  void AddIterationEntry(int a, int b, double value);
  /*! \brief the pattern, the value indices and the operations of the
   * factorization of the iteration matrix */
  void RecordIterationPattern();
  /*! \brief the index in iteration_factors_ of the entry (r, c),
   * permutation order 3, of the pattern; -1 outside it */
  int IterationSlot(int r, int c) const;
  /*! \brief assemble M + A and factor it; returns true for a zero or
   * not finite pivot */
  bool FactorIterationEntries();

  /*! the entries of A of the last FactorIterationMatrix, permutation
   * order 3 */
  std::vector<int> iteration_row_;
  std::vector<int> iteration_column_;
  std::vector<double> iteration_value_;
  /*! the off-diagonal entries since the last partition change, as
   * max(r, c) * dq_count_ + min(r, c) */
  std::unordered_set<int64_t> iteration_pattern_entries_;
  /*! the value index of each off-diagonal entry of the pattern, keyed by
   * r * dq_count_ + c; the diagonal entry of variable i has index i */
  std::unordered_map<int64_t, int> iteration_slot_;
  /*! per variable i (permutation order 3), the variables k < i of the
   * pattern of its row, from iteration_neighbor_start_[i] to
   * iteration_neighbor_start_[i + 1], with the value indices of (k, i) and
   * (i, k) */
  std::vector<int> iteration_neighbor_start_;
  std::vector<int> iteration_neighbor_;
  std::vector<int> iteration_lower_slot_;
  std::vector<int> iteration_upper_slot_;
  /*! the value indices of the entries of mass_pattern_index_, and of their
   * transposes */
  std::vector<int> iteration_mass_slot_;
  std::vector<int> iteration_mass_transpose_slot_;
  /*! the updates V[x] -= V[y] * V[z] of the elimination of variable i, as
   * triples from iteration_operation_start_[i] to
   * iteration_operation_end_[i] */
  std::vector<int> iteration_operation_;
  std::vector<int> iteration_operation_start_;
  std::vector<int> iteration_operation_end_;
  /*! the values: the entries of the pattern, then B (as b3), factored in
   * place: L (scaled) below the diagonal, U and the inverted pivots of the
   * independent variables on and above it */
  std::vector<double> iteration_factors_;
  int iteration_slot_count_ = 0;
  size_t iteration_b_count_ = 0;
  bool has_iteration_pattern_ = false;
  unsigned int iteration_pattern_memoize_count_ = 0;
  ///@}

  /*! incremented by Memoize: the pattern of the iteration matrix and its
   * factors are those of one record of the operations */
  unsigned int memoize_count_ = 0;
  unsigned int iteration_memoize_count_ = 0;
  bool has_iteration_matrix_ = false;
  std::vector<ksl_coscrew_t> p_;          /*!< spatial momentum */
  std::vector<ksl_coscrew_t> mh_product_; /*!< product of body mass matrix and
                                influence coefficient matrices in PO(1) */
  std::vector<ksl_inertia_t> inertia_p_;  /*!< projected spatial inertias */
  std::vector<ksl_screw_t> de_; /*!< time derivative of constraint equations */
  int nnzm_ = 0;                /*!< number of nonzero entries in mass matrix */
  int nnzms_ = 0; /*!< original number of nonzero entries in mass matrix */
  int nnzb_ = 0;  /*!< number of nonzero entries in B matrix */

  shift::core::Array2d<int> rm_;
  std::vector<int> cm_; /*!< row ranges of M matrix in PO(3) */

  // shift::core::Array2d<int> rb2_; /*!< row ranges of B matrix in PO(2) */
  // shift::core::Array2d<int>
  //   rb3_temp_; /*!< temporary array used to convert rb2 to rb3 */
  // // shift::core::Array2d<int> rb3_; /*!< row ranges of B matrix in PO(3) */
  //
  // std::vector<int> cb2_;      /*!< global col indices of B matrix in PO(2) */
  // std::vector<int> cb3_temp_; /*!< temporary array used to convert cb2 to
  // cb3*/ std::vector<int> cb3_;      /*!< global col indices of B matrix in
  // PO(3) */ std::vector<int> colb_;     /*!< local col indices of B matrix in
  // PO(3) */ int ncb_ = 0;               /*!< number of columns in B matrix */
  //
  // std::vector<int> pb_23_; /*!< permutation of linear b matrix from 3 to 2 */
  // std::vector<int> pb_32_; /*!< permutation of linear b matrix from 2 to 3 */

  // std::vector<int> ij_mapping_;
  // std::vector<double> b3_; /*!< Linear array containing terms of
  // preconditioned
  //                 constraints in PO(3) */
  std::vector<double> fl_; /*!< generalized forces */
  std::vector<ksl_coscrew_t>
    g_; /*!< externally applied forces TODO: move to Context data structure */
  std::vector<double> la_; /*!< lagrange multipliers */

  int i1s_ = 0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i2s_ = 0; /*!< Points to Mij in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i6s_ = 0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Mik or Mjk -= Mij*Mik */
  int i8s_ = 0; /*!< Points to Mik in Mkj -= Mij*Mik or Mjk -= Mij*Mik */

  /*!< M[i1[l1]] -= M[i2[l1]] * B[i3[l1]]; */
  std::vector<int> i1_;
  std::vector<int> i2_;
  std::vector<int> i3_;

  /*!< M[i4[i]] = 1.0/M[i4[i]];
       M[p] = M[i5[l2]];
       M[i5[l2]] *= M[i4[i]]; */
  std::vector<int> i4_;
  std::vector<int> i5_;

  /*!< M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */
  std::vector<int> i6_;
  std::vector<int> i7_;
  std::vector<int> i8_;

  /*!< Q[j1[m1]] -= B[j2[m1]]*Q[i];
       ddq[i] -= B[j2[m1]]*ddq[j1[m1]]; */
  std::vector<int> j1_;
  std::vector<int> j2_;

  /*!< Q[j3[m2]] -= M[j4[m2]]*ga[i];
       la[i] -= M[j4[m2]]*ddq[j3[m2]]; */
  std::vector<int> j3_;
  std::vector<int> j4_;

  /*!< Q[j5[m3]] -= M[j6[m3]]*Q[i];
       ddq[i] -= M[j6[m3]]*ddq[j5[m3]]; */
  std::vector<int> j5_;
  std::vector<int> j6_;

  std::vector<int>
    p1_; /*!< increments in l1 to process current dependent variable. */
  std::vector<int>
    p2_; /*!< increments in m1 to process current dependent variable. */
  std::vector<int>
    p3_; /*!< increments in m2 to process current dependent variable. */
  std::vector<int>
    n1_; /*!< increments in l2 to process current independent variable. */
  std::vector<int>
    n2_; /*!< increments in l3 to process current independent variable. */
  std::vector<int>
    n3_; /*!< increments in m3 to process current independent variable. */

  /*! index in memoized_pointers_ one past the factorization's operations,
   * where the entries of L used by ForwardBackwardSubstitution start; set by
   * RecordPointers */
  int factored_pointer_end_ = 0;
  int start_of_work_in_M_ = 0; /*!< n(n+1)/2 points to first entry in temporary
                           storage part of Ml[] */
  std::vector<double*> memoized_pointers_;

  double p3_start_ = 0; /*!< start of p3 in P */

  const std::string description_ =
    "Run-time optimized Natural Constraint Algorithm (RNCA):\n"
    "The RNCA algorithm is suited to set up and solve the equations "
    "of motion for complex, constrained mechanisms with multiple kinematic "
    "chord loops. Chord loops are identified automatically from the JSON "
    "input file. Overlapping clusters of chord loops (i.e. kinematic "
    "substructures), are identified automatically if present in the model, "
    "thereby allowing the constraint Jacobian matrix to be block-diagonalized. "
    "The optimal split of independent and dependent joint variables is "
    "obtained automatically using a heuristic application of Generalized "
    "Coordinate Partitioning (GCP). The equations of motion are solved "
    "using a custom sparse matrix factor and solve algorithm  \n"
    "tailored to the unique structure of multibody dynamics equations.\n"
    "\n"
    "Key features include:\n"
    "  * Block-diagonalization via kinematic substructuring\n"
    "  * Heuristic application of GCP via monitoring infinity norm\n"
    "  * Custom sparse factor and solve routine tailored to structure of \n"
    "    multibody dynamics equations\n"
    "  * Minimizes logic and overhead using arrays of function pointers\n"
    "  * Automatically reconfigures model at run-time as necessary\n"
    "\n"
    "Best suited for:\n"
    "  * Complex constrained mechanisms\n"
    "  * Tree topologies\n"
    "\n"
    "Refer to the folowing publications for additional information:\n"
    "  * Wehage K., Computational Methods for Dynamics and Mobility Analysis "
    "of Multiloop Mechanisms and Robotic Systems, Thesis, December 2016 "
    "`<https://search.proquest.com/openview/"
    "2b96ee91ebc96f840bdada69a21b7cbe>`_\n"
    "\n"
    "  * Wehage K. Ravani B., A Computational Method for Formulation and "
    "Solution of Dynamical Equations for Complex Mechanisms and Multibody "
    "Systems, Proceedings of IDETC/DIE at Cleveland Ohio, August 2017, "
    "`<https://doi.org/10.1115/DETC2017-67766>`_\n"
    "\n"
    "  * Wehage K., Wehage R., Ravani B., Generalized coordinate partitioning "
    "for complex mechanisms based on kinematic substructuring, Mechanism and "
    "Machine Theory 92:464-483, October 2015 "
    "`<https://doi.org/10.1016/j.mechmachtheory.2015.06.006>`_";
};

} // namespace solver
} // namespace core
} // namespace shift

#endif
