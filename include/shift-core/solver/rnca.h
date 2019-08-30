#ifndef SHIFT_SOLVER_RNCA_H_
#define SHIFT_SOLVER_RNCA_H_

#include <string>

#include "shift-core/solver/base.h"

namespace Shift {

namespace Solver {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class RNCA : public Base {

public:
  RNCA(std::string name)
    : Base(name) {
    addParameters();
  }
  RNCA()
    : Base("") {
    addParameters();
  }
  virtual ~RNCA() override {
    if(gamma_d_) {
      delete[] gamma_d_;
      gamma_d_ = nullptr;
    }
    if(dcs_) {
      delete[] dcs_;
      dcs_ = nullptr;
    }
    if(ddcs_) {
      delete[] ddcs_;
      ddcs_ = nullptr;
    }
    if(ddcsq_) {
      delete[] ddcsq_;
      ddcsq_ = nullptr;
    }
    if(Ml_) {
      delete[] Ml_;
      Ml_ = nullptr;
    }
    if(Mls_) {
      delete[] Mls_;
      Mls_ = nullptr;
    }
    if(Mlb_) {
      delete[] Mlb_;
      Mlb_ = nullptr;
    }
    if(Mlbs_) {
      delete[] Mlbs_;
      Mlbs_ = nullptr;
    }
    if(p_) {
      free(p_);
      p_ = nullptr;
    }
    if(mhProduct_) {
      free(mhProduct_);
      mhProduct_ = nullptr;
    }
    if(inertiaP_) {
      free(inertiaP_);
      inertiaP_ = nullptr;
    }

    if(rm_) {
      free(*rm_);
      rm_ = nullptr;
    }

    if(cm_) {
      delete[] cm_;
      cm_ = nullptr;
    }

    if(rb2_) {
      free(*rb2_);
      rb2_ = nullptr;
    }
    if(rb3_temp_) {
      free(*rb3_temp_);
      rb3_temp_ = nullptr;
    }
    if(rb3_) {
      free(*rb3_);
      rb3_ = nullptr;
    }

    if(cb2_) {
      delete[] cb2_;
      cb2_ = nullptr;
    }
    if(cb3_temp_) {
      delete[] cb3_temp_;
      cb3_temp_ = nullptr;
    }
    if(cb3_) {
      delete[] cb3_;
      cb3_ = nullptr;
    }
    if(colb_) {
      delete[] colb_;
      colb_ = nullptr;
    }
    if(pb_23_) {
      delete[] pb_23_;
      pb_23_ = nullptr;
    }
    if(pb_32_) {
      delete[] pb_32_;
      pb_32_ = nullptr;
    }

    if(ijMapping_) {
      delete[] ijMapping_;
      ijMapping_ = nullptr;
    }

    if(b3_) {
      delete[] b3_;
      b3_ = nullptr;
    }
    if(fl_) {
      delete[] fl_;
      fl_ = nullptr;
    }
    if(la_) {
      delete[] la_;
      la_ = nullptr;
    }

    if(i1_) {
      delete[] i1_;
      i1_ = nullptr;
    }
    if(i2_) {
      delete[] i2_;
      i2_ = nullptr;
    }
    if(i3_) {
      delete[] i3_;
      i3_ = nullptr;
    }
    if(i4_) {
      delete[] i4_;
      i4_ = nullptr;
    }
    if(i5_) {
      delete[] i5_;
      i5_ = nullptr;
    }
    if(i6_) {
      delete[] i6_;
      i6_ = nullptr;
    }
    if(i7_) {
      delete[] i7_;
      i7_ = nullptr;
    }
    if(i8_) {
      delete[] i8_;
      i8_ = nullptr;
    }
    if(j1_) {
      delete[] j1_;
      j1_ = nullptr;
    }
    if(j2_) {
      delete[] j2_;
      j2_ = nullptr;
    }
    if(j3_) {
      delete[] j3_;
      j3_ = nullptr;
    }
    if(j4_) {
      delete[] j4_;
      j4_ = nullptr;
    }
    if(j5_) {
      delete[] j5_;
      j5_ = nullptr;
    }
    if(j6_) {
      delete[] j6_;
      j6_ = nullptr;
    }
    if(p1_) {
      delete[] p1_;
      p1_ = nullptr;
    }
    if(p2_) {
      delete[] p2_;
      p2_ = nullptr;
    }
    if(p3_) {
      delete[] p3_;
      p3_ = nullptr;
    }
    if(n1_) {
      delete[] n1_;
      n1_ = nullptr;
    }
    if(n2_) {
      delete[] n2_;
      n2_ = nullptr;
    }
    if(n3_) {
      delete[] n3_;
      n3_ = nullptr;
    }

    if(P_) {
      free(*P_);
      P_ = nullptr;
    }
  }

  const std::string& getDescription() {
    return description_;
  }

  void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

  virtual void initialize() override;

private:
  std::vector<bool> hasMass_;
  std::vector<bool> hasMassdq_;

  double* gamma_d_ = nullptr; /*!< preconditioned constraints */
  double* dcs_ = nullptr;     /*!< constraint first derivative */
  double* ddcs_ = nullptr;    /*!< constraint second derivative */
  double* ddcsq_ =
    nullptr; /*!< constraint second derivative and quadratic terms */

  int Ml_size_ = 0; /*!< size of Ml array = ((2 + n) * (1 + n)) / 2*/

  double* Ml_ = nullptr;  /*!< Generalized mass matrix, linear array containing
                    mass  matrix terms on and below the diagonal */
  double* Mls_ = nullptr; /*!< Saved copy of original M */

  bool* Mlb_ = nullptr; /*!< boolean mass matrix, equal in size to M */
  bool* Mlbs_ =
    nullptr; /*!< original boolean mass matrix, with no fills recorded */

  ksl_screw_t* p_ = nullptr;          /*!< momentum */
  ksl_screw_t* mhProduct_ = nullptr;  /*!< product of mass matrix and influence
                                 coefficient matrices in PO(1) */
  ksl_inertia_t* inertiaP_ = nullptr; /*!< accumulated "projected" inertia
                                matrices used in solving dynamics equations */

  int nnzm_ = 0;  /*!< number of nonzero entries in mass matrix */
  int nnzms_ = 0; /*!< original number of nonzero entries in mass matrix */
  int nnzb_ = 0;  /*!< number of nonzero entries in B matrix */

  int** rm_ = nullptr; /*!< row ranges of M matrix in PO(3) */
  int* cm_ = nullptr;  /*!< row ranges of M matrix in PO(3) */

  int** rb2_ = nullptr;      /*!< row ranges of B matrix in PO(2) */
  int** rb3_temp_ = nullptr; /*!< temporary array used to convert rb2 to rb3 */
  int** rb3_ = nullptr;      /*!< row ranges of B matrix in PO(3) */

  int* cb2_ = nullptr;      /*!< global col indices of B matrix in PO(2) */
  int* cb3_temp_ = nullptr; /*!< temporary array used to convert cb2 to cb3*/
  int* cb3_ = nullptr;      /*!< global col indices of B matrix in PO(3) */
  int* colb_ = nullptr;     /*!< local col indices of B matrix in PO(3) */
  int ncb_ = 0;             /*!< number of columns in B matrix */

  int* pb_23_ = nullptr; /*!< permutation of linear b matrix from 3 to 2 */
  int* pb_32_ = nullptr; /*!< permutation of linear b matrix from 2 to 3 */

  int* ijMapping_ =
    nullptr;             /*!< row and column indices in B matrix to process */
  double* b3_ = nullptr; /*!< Linear array containing terms of preconditioned
                   constraints     in PO(3) */

  double* fl_ = nullptr; /*!< Generalized forces */

  // double* ga; /*!< quadratic acceleration terms */
  // double* gas; /*!< Saved copy of original ga */
  double* la_ = nullptr; /*!< lagrange multipliers */

  int i1s_ = 0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i2s_ = 0; /*!< Points to Mij in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i6s_ = 0; /*!< Points to Mkj or Mjk in Mkj -= Mij*Mik or Mjk -= Mij*Mik */
  int i8s_ = 0; /*!< Points to Mik in Mkj -= Mij*Mik or Mjk -= Mij*Mik */

  /*!< M[i1[l1]] -= M[i2[l1]] * B[i3[l1]]; */
  int* i1_ = nullptr;
  int* i2_ = nullptr;
  int* i3_ = nullptr;

  /*!< M[i4[i]] = 1.0/M[i4[i]];
       M[p] = M[i5[l2]];
       M[i5[l2]] *= M[i4[i]]; */
  int* i4_ = nullptr;
  int* i5_ = nullptr;

  /*!< M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */
  int* i6_ = nullptr;
  int* i7_ = nullptr;
  int* i8_ = nullptr;

  /*!< Q[j1[m1]] -= B[j2[m1]]*Q[i];
       ddq[i] -= B[j2[m1]]*ddq[j1[m1]]; */
  int* j1_ = nullptr;
  int* j2_ = nullptr;

  /*!< Q[j3[m2]] -= M[j4[m2]]*ga[i];
       la[i] -= M[j4[m2]]*ddq[j3[m2]]; */
  int* j3_ = nullptr;
  int* j4_ = nullptr;

  /*!< Q[j5[m3]] -= M[j6[m3]]*Q[i];
       ddq[i] -= M[j6[m3]]*ddq[j5[m3]]; */
  int* j5_ = nullptr;
  int* j6_ = nullptr;

  int* p1_ =
    nullptr; /*!< increments in l1 to process current dependent variable. */
  int* p2_ =
    nullptr; /*!< increments in m1 to process current dependent variable. */
  int* p3_ =
    nullptr; /*!< increments in m2 to process current dependent variable. */
  int* n1_ =
    nullptr; /*!< increments in l2 to process current independent variable. */
  int* n2_ =
    nullptr; /*!< increments in l3 to process current independent variable. */
  int* n3_ =
    nullptr; /*!< increments in m3 to process current independent variable. */

  int startOfWorkInM_ = 0; /*!< n(n+1)/2 points to first entry in temporary
                           storage part of Ml[] */
  double** P_ =
    nullptr; /*!< Single array of source and destination memory pointers. */
  double p3Start_ = 0; /*!< start of p3 in P */

  const std::string description_ =
    "Run-time optimized Natural Constraint Algorithm (RNCA):\n"
    "The RNCA algorithm is well suited to set up and solve the equations "
    "of motion for complex constrained mechanisms with multiple kinematic "
    "chord loops. Chord loops are identified automatically from the JSON "
    "input file. Overlapping clusters of chord loops (i.e. kinematic "
    "substructures), are identified automatically if present in the model, "
    "thereby allowing the constraint Jacobian matrix to be block-diagonalized. "
    "The optimal split of independent and dependent joint variables is "
    "obtained automatically using a heuristic application of Generalized "
    "Coordinate Partitioning (GCP). The equations of motion are solved "
    "using a custom sparse matrix factor and solve algorithm that is \n"
    "tailored to the unique structure of multibody dynamics equations.\n"
    "\n"
    "Key features include:\n"
    "  * Block diagonalization via kinematic substructuring\n"
    "  * Heuristic application of GCP via monitoring infinity norm\n"
    "  * Custom sparse factor and solve routine tailored to MBD\n"
    "  * Minimize logic and overhead using arrays of function pointers\n"
    "  * Run-time reconfiguration\n"
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

  static bool registered_;
  static std::unique_ptr<RNCA> create();
};

} // namespace Solver

} // namespace Shift

#endif
