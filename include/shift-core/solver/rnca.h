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
    addKeys();
  }
  RNCA()
    : Base("") {
    addKeys();
  }
  virtual ~RNCA() override {
    if(gamma_d_) {
      delete[] gamma_d_;
      gamma_d_ = NULL;
    }
    if(dcs_) {
      delete[] dcs_;
      dcs_ = NULL;
    }
    if(ddcs_) {
      delete[] ddcs_;
      ddcs_ = NULL;
    }
    if(ddcsq_) {
      delete[] ddcsq_;
      ddcsq_ = NULL;
    }
    if(Ml_) {
      delete[] Ml_;
      Ml_ = NULL;
    }
    if(Mls_) {
      delete[] Mls_;
      Mls_ = NULL;
    }
    if(Mlb_) {
      delete[] Mlb_;
      Mlb_ = NULL;
    }
    if(Mlbs_) {
      delete[] Mlbs_;
      Mlbs_ = NULL;
    }
    if(p_) {
      free(p_);
      p_ = NULL;
    }
    if(mhProduct_) {
      free(mhProduct_);
      mhProduct_ = NULL;
    }
    if(inertiaP_) {
      free(inertiaP_);
      inertiaP_ = NULL;
    }

    if(rm_) {
      free(*rm_);
      rm_ = NULL;
    }

    if(cm_) {
      delete[] cm_;
      cm_ = NULL;
    }

    if(rb2_) {
      free(*rb2_);
      rb2_ = NULL;
    }
    if(rb3_temp_) {
      free(*rb3_temp_);
      rb3_temp_ = NULL;
    }
    if(rb3_) {
      free(*rb3_);
      rb3_ = NULL;
    }

    if(cb2_) {
      delete[] cb2_;
      cb2_ = NULL;
    }
    if(cb3_temp_) {
      delete[] cb3_temp_;
      cb3_temp_ = NULL;
    }
    if(cb3_) {
      delete[] cb3_;
      cb3_ = NULL;
    }
    if(colb_) {
      delete[] colb_;
      colb_ = NULL;
    }
    if(pb_23_) {
      delete[] pb_23_;
      pb_23_ = NULL;
    }
    if(pb_32_) {
      delete[] pb_32_;
      pb_32_ = NULL;
    }

    if(ijMapping_) {
      delete[] ijMapping_;
      ijMapping_ = NULL;
    }

    if(b3_) {
      delete[] b3_;
      b3_ = NULL;
    }
    if(fl_) {
      delete[] fl_;
      fl_ = NULL;
    }
    if(la_) {
      delete[] la_;
      la_ = NULL;
    }

    if(i1_) {
      delete[] i1_;
      i1_ = NULL;
    }
    if(i2_) {
      delete[] i2_;
      i2_ = NULL;
    }
    if(i3_) {
      delete[] i3_;
      i3_ = NULL;
    }
    if(i4_) {
      delete[] i4_;
      i4_ = NULL;
    }
    if(i5_) {
      delete[] i5_;
      i5_ = NULL;
    }
    if(i6_) {
      delete[] i6_;
      i6_ = NULL;
    }
    if(i7_) {
      delete[] i7_;
      i7_ = NULL;
    }
    if(i8_) {
      delete[] i8_;
      i8_ = NULL;
    }
    if(j1_) {
      delete[] j1_;
      j1_ = NULL;
    }
    if(j2_) {
      delete[] j2_;
      j2_ = NULL;
    }
    if(j3_) {
      delete[] j3_;
      j3_ = NULL;
    }
    if(j4_) {
      delete[] j4_;
      j4_ = NULL;
    }
    if(j5_) {
      delete[] j5_;
      j5_ = NULL;
    }
    if(j6_) {
      delete[] j6_;
      j6_ = NULL;
    }
    if(p1_) {
      delete[] p1_;
      p1_ = NULL;
    }
    if(p2_) {
      delete[] p2_;
      p2_ = NULL;
    }
    if(p3_) {
      delete[] p3_;
      p3_ = NULL;
    }
    if(n1_) {
      delete[] n1_;
      n1_ = NULL;
    }
    if(n2_) {
      delete[] n2_;
      n2_ = NULL;
    }
    if(n3_) {
      delete[] n3_;
      n3_ = NULL;
    }

    if(P_) {
      free(*P_);
      P_ = NULL;
    }
  }

  const std::string& getDescription() {
    return description_;
  }

  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

private:
  std::vector<bool> hasMass_;
  std::vector<bool> hasMassdq_;

  double* gamma_d_ = NULL; /*!< preconditioned constraints */
  double* dcs_ = NULL;     /*!< constraint first derivative */
  double* ddcs_ = NULL;    /*!< constraint second derivative */
  double* ddcsq_ =
    NULL; /*!< constraint second derivative and quadratic terms */

  int Ml_size_; /*!< size of Ml array = ((2 + n) * (1 + n)) / 2*/

  double* Ml_ = NULL; /*!< Generalized mass matrix, linear array containing mass
                   matrix terms on and below the diagonal */
  double* Mls_ = NULL; /*!< Saved copy of original M */

  bool* Mlb_ = NULL; /*!< boolean mass matrix, equal in size to M */
  bool* Mlbs_ =
    NULL; /*!< original boolean mass matrix, with no fills recorded */

  ksl_screw_t* p_ = NULL;          /*!< momentum */
  ksl_screw_t* mhProduct_ = NULL;  /*!< product of mass matrix and influence
                                 coefficient matrices in PO(1) */
  ksl_inertia_t* inertiaP_ = NULL; /*!< accumulated "projected" inertia
                                matrices used in solving dynamics equations */

  int nnzm_;  /*!< number of nonzero entries in mass matrix */
  int nnzms_; /*!< original number of nonzero entries in mass matrix */
  int nnzb_;  /*!< number of nonzero entries in B matrix */

  int** rm_ = NULL; /*!< row ranges of M matrix in PO(3) */
  int* cm_ = NULL;  /*!< row ranges of M matrix in PO(3) */

  int** rb2_ = NULL;      /*!< row ranges of B matrix in PO(2) */
  int** rb3_temp_ = NULL; /*!< temporary array used to convert rb2 to rb3 */
  int** rb3_ = NULL;      /*!< row ranges of B matrix in PO(3) */

  int* cb2_ = NULL;      /*!< global col indices of B matrix in PO(2) */
  int* cb3_temp_ = NULL; /*!< temporary array used to convert cb2 to cb3*/
  int* cb3_ = NULL;      /*!< global col indices of B matrix in PO(3) */
  int* colb_ = NULL;     /*!< local col indices of B matrix in PO(3) */
  int ncb_;              /*!< number of columns in B matrix */

  int* pb_23_ = NULL; /*!< permutation of linear b matrix from 3 to 2 */
  int* pb_32_ = NULL; /*!< permutation of linear b matrix from 2 to 3 */

  int* ijMapping_ = NULL; /*!< row and column indices in B matrix to process */
  double* b3_ = NULL;     /*!< Linear array containing terms of preconditioned
                       constraints     in PO(3) */

  double* fl_ = NULL; /*!< Generalized forces */

  // double* ga; /*!< quadratic acceleration terms */
  // double* gas; /*!< Saved copy of original ga */
  double* la_ = NULL; /*!< lagrange multipliers */

  int i1s_; /*!< Points to Mkj or Mjk in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i2s_; /*!< Points to Mij in Mkj -= Mij*Bik or Mjk -= Mij*Bik */
  int i6s_; /*!< Points to Mkj or Mjk in Mkj -= Mij*Mik or Mjk -= Mij*Mik */
  int i8s_; /*!< Points to Mik in Mkj -= Mij*Mik or Mjk -= Mij*Mik */

  /*!< M[i1[l1]] -= M[i2[l1]] * B[i3[l1]]; */
  int* i1_ = NULL;
  int* i2_ = NULL;
  int* i3_ = NULL;

  /*!< M[i4[i]] = 1.0/M[i4[i]];
       M[p] = M[i5[l2]];
       M[i5[l2]] *= M[i4[i]]; */
  int* i4_ = NULL;
  int* i5_ = NULL;

  /*!< M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */
  int* i6_ = NULL;
  int* i7_ = NULL;
  int* i8_ = NULL;

  /*!< Q[j1[m1]] -= B[j2[m1]]*Q[i];
       ddq[i] -= B[j2[m1]]*ddq[j1[m1]]; */
  int* j1_ = NULL;
  int* j2_ = NULL;

  /*!< Q[j3[m2]] -= M[j4[m2]]*ga[i];
       la[i] -= M[j4[m2]]*ddq[j3[m2]]; */
  int* j3_ = NULL;
  int* j4_ = NULL;

  /*!< Q[j5[m3]] -= M[j6[m3]]*Q[i];
       ddq[i] -= M[j6[m3]]*ddq[j5[m3]]; */
  int* j5_ = NULL;
  int* j6_ = NULL;

  int* p1_ =
    NULL; /*!< increments in l1 to process current dependent variable. */
  int* p2_ =
    NULL; /*!< increments in m1 to process current dependent variable. */
  int* p3_ =
    NULL; /*!< increments in m2 to process current dependent variable. */
  int* n1_ =
    NULL; /*!< increments in l2 to process current independent variable. */
  int* n2_ =
    NULL; /*!< increments in l3 to process current independent variable. */
  int* n3_ =
    NULL; /*!< increments in m3 to process current independent variable. */

  int startOfWorkInM_; /*!< n(n+1)/2 points to first entry in temporary storage
                           part of Ml[] */
  double** P_ =
    NULL; /*!< Single array of source and destination memory pointers. */
  double p3Start_; /*!< start of p3 in P */

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

  double infinity_norm_multiplier_ = 2.0;
  static bool registered_;
  static std::unique_ptr<RNCA> create();
};

} // namespace Solver

} // namespace Shift

#endif
