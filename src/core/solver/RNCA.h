#ifndef SHIFT_SOLVER_RNCA_H_
#define SHIFT_SOLVER_RNCA_H_

#include <string>

#include "shift/core/Array2d.h"
#include "shift/core/solver/Base.h"
#include "shift/core/Topology.h"

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
  
  virtual double GravitationalPotentialEnergy(shift::core::topology::DHVA& dhva) override;
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
  bool SetupEquationsAndRecordFills(const double t, const std::vector<double>& y);

  /*! factor and solve the block-diagonalized preconditioned spatial equations
   * of motion using run-time optimized natural constraint algorithm
   * [Ml_ii Ml_id  B_id] {ddq_i} = {fl_i}
   * [Ml_di Ml_ii  I   ] {ddq_d} = {fl_d}
   * [B_di 0       0   ] {la_d}  = {ga_d}
   */
  void FactorAndSolve();

  virtual bool Solve(const double t, const std::vector<double>& y,
                     std::vector<double>& dy) override;

  virtual void Initialize() override;

private:

  static bool registered_;
  static std::unique_ptr<RNCA> Create();

  virtual void DeclareParameters() override;
  void CountOperations();
  void RecordOperations();
  void CountPointers();
  void RecordPointers();
  bool Verify(const double t, const std::vector<double>& y, double Ml[],
              double fl[]);

  double infinity_norm_multiplier_ = 2.0;
  bool infinity_norm_failure_ = false;

  unsigned int dependent_count_ = 0;
  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;

  std::vector<double> gamma_d_; /*!< preconditioned constraints */
  std::vector<double> dcs_;     /*!< constraint first derivative */
  std::vector<double> ddcs_;    /*!< constraint second derivative */
  std::vector<double>
    ddcsq_; /*!< constraint second derivative and quadratic terms */

  int Ml_size_ = 0; /*!< size of Ml array = ((2 + n) * (1 + n)) / 2*/

  std::vector<double> Ml_; /*!< Generalized mass matrix, linear array containing
                    mass  matrix terms on and below the diagonal */
  std::vector<double> Mls_; /*!< Saved copy of original M */
  std::vector<bool>
    Mlb_; /*!< boolean mass matrix with fills rescorded, equal in size to M */
  std::vector<bool>
    Mlbs_; /*!< original boolean mass matrix, with no fills recorded */
  std::vector<ksl_coscrew_t> p_;          /*!< spatial momentum */
  std::vector<ksl_coscrew_t> mh_product_; /*!< product of body mass matrix and
                                influence coefficient matrices in PO(1) */
  std::vector<ksl_inertia_t> inertia_p_;  /*!< projected spatial inertias */

  int nnzm_ = 0;  /*!< number of nonzero entries in mass matrix */
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
  std::vector<ksl_coscrew_t> g_;
  // double* ga; /*!< quadratic acceleration terms */
  // double* gas; /*!< Saved copy of original ga */
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
}
} // namespace shift

#endif
