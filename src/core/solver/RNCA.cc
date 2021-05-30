#include <assert.h>

#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/solver/RNCA.h"

#include "ksl/array.h"
#include "ksl/print.h"

#ifdef SHIFT_VERIFY_EQUATIONS_
#include <cblas.h>
#endif

bool shift::core::solver::RNCA::registered_ =
  shift::core::Factory<shift::core::solver::Base>::get()->registerModule(
    "rnca", shift::core::solver::RNCA::create);

std::unique_ptr<shift::core::solver::RNCA> shift::core::solver::RNCA::create() {
  return std::make_unique<shift::core::solver::RNCA>();
}

void shift::core::solver::RNCA::addParameters() {
  shift::core::solver::Base::addParameters();

  addParameter(
    "infinity_norm_multiplier", &infinity_norm_multiplier_,
    "The infinity norm of each Jacobian matrix's *B* matrix is used as a "
    "heuristic to determine when to reapply Gaussian elimination with complete "
    "row and column pivoting on as-needed basis. GECP will be reapplied when "
    "the infinity norm reaches this multiple of the initial starting value -- "
    "default: 2.0.",
    false);
}

void shift::core::solver::RNCA::initialize() {
  assert(context_);

  q_count_ = context_->qCount();
  dq_count_ = context_->dqCount();

  gamma_d_.resize(dq_count_);
  std::fill(gamma_d_.begin(), gamma_d_.end(), 0);

  mh_product_.resize(dq_count_);
  std::fill(mh_product_.begin(), mh_product_.end(),
            ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  fl_.resize(dq_count_);
  std::fill(fl_.begin(), fl_.end(), 0);

  Ml_size_ = ((dq_count_ + 1) * (dq_count_ + 2)) / 2;

  Ml_.resize(Ml_size_);
  std::fill(Ml_.begin(), Ml_.end(), 0);

  Mls_.resize(Ml_size_);
  std::fill(Mls_.begin(), Mls_.end(), 0);

  Mlb_.resize(Ml_size_);
  std::fill(Mlb_.begin(), Mlb_.end(), false);

  Mlbs_.resize(Ml_size_);
  std::fill(Mlbs_.begin(), Mlbs_.end(), false);

  inertia_p_.resize(context_->nonRigidJointCount() + 1);
  std::fill(inertia_p_.begin(), inertia_p_.end(),
            ksl_inertia(0.0, {{0.0, 0.0, 0.0}}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  p_.resize(context_->nonRigidJointCount());
  std::fill(p_.begin(), p_.end(), ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  g_.resize(context_->nonRigidJointCount());
  std::fill(g_.begin(), g_.end(), ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  la_.resize(context_->dqCount());
  std::fill(la_.begin(), la_.end(), 0);

  infinity_norm_failure_ = false;

  context_->jacobian().removeConstraintViolations();
  context_->jacobian().analyze();
  context_->updateDHVA();
  context_->evaluateConstraints();
  setEquationsAndRecordFills();
  memoize();
}

void shift::core::solver::RNCA::memoize() {
  countOperations();
  recordOperations();
  countPointers();
  recordPointers();
}

void shift::core::solver::RNCA::countOperations() {
  assert(context_);
  dq_count_ = context_->dqCount();
  auto ddq = context_->ddq();
  const auto& jacobian = context_->jacobian();
  const auto& rb3 = jacobian.rb3();
  const auto& cb3 = jacobian.cb3();
  // const auto& b3 = jacobian.b3();
  const auto& is_dependent_po3 = jacobian.isDependentPO3();
  // const auto& pv_13 = jacobian.pv13();

  // COUNT OPERATIONS BLOCK
  Mlbs_.clear();
  Mlbs_.assign(Mlb_.begin(), Mlb_.end());

  /* Indices for matrix factorization */
  int l1 = 0; /*!< index for M[i1[l1]] -= M[i2[l1]]*B[i3[l1]]; */
  int l2 = 0; /*!< Index for M[p] = M[i5[l2]];
                         M[i5[l2]] *= M[i4[i]]; */
  int l3 = 0; /*!< Index for M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */

  /* Indices for backward elimination/forward substitution. */
  int m1 = 0; /*!< Index for Q[j1[m1]] -= B[j2[m1]] * Q[i];
                         ddq[i] -= B[j2[m1]] * ddq[j1[m1]]; */
  int m2 = 0; /*!< Index for Q[j3[m2]] -= M[j4[m2]] * ga[i];
                         la[i] -= M[j4[m2]] * ddq[j3[m2]]; */
  int m3 = 0; /*!< Index for Q[j5[m3]] -= M[j6[m3]] * Q[i];
                         ddq[i] -= M[j6[m3]] * ddq[j5[m3]]; */

  /*! indices of independent variables in kinematic substructure that
  are coupled to the current dependent variable being eliminated */
  int coupled_list[dq_count_];

  /*! indices of independent variables in kinematic substructure that
  are not coupled to the current dependent variable being eliminated */
  int non_coupled_list[dq_count_];

  /*! Pointer to nonzero entry in row i, col j of the L matrix. */
  int cl1 = 0; /*!< Counter for size of i1[], i2[], i3[] */
  int cl2 = 0; /*!< Counter for size of i5[] */
  int cl3 = 0; /*!< Counter for size of i6[], i7[], i8[] */
  int cm1 = 0; /*!< Counter for size of j1[], j2[] */
  int cm2 = 0; /*!< Counter for size of j3[], j4[] */
  int cm3 = 0; /*!< Counter for size of j5[], j6[] */

  /*! n(n+1)/2 points to first entry in temp storage part of M[] */
  start_of_work_in_M_ = (dq_count_ * (dq_count_ + 1)) / 2;
  i4_.resize(dq_count_);
  std::fill(i4_.begin(), i4_.end(), 0);

  /* Index for the dependent variable currently being processed. */
  int ni = 0; /*!< counter for number of independent variables */
  int nd = 0; /*!< counter for number of dependent variables */
  for(size_t i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      nd++;
    } else {
      ni++;
    }
  }

  // Allocate arrays based on ni and nd to be used later.
  n1_.resize(nd);
  std::fill(n1_.begin(), n1_.end(), 0);

  n2_.resize(ni);
  std::fill(n2_.begin(), n2_.end(), 0);

  n3_.resize(ni);
  std::fill(n3_.begin(), n3_.end(), 0);

  p1_.resize(nd);
  std::fill(p1_.begin(), p1_.end(), 0);

  p2_.resize(nd);
  std::fill(p2_.begin(), p2_.end(), 0);

  p3_.resize(ni);
  std::fill(p3_.begin(), p3_.end(), 0);

  ni = 0;
  nd = 0;

  /*
    MATRIX FACTORIZATION
  */
  for(int i = (int)dq_count_ - 1; i > -1; i--) {
    // i points to the current variable
    // being processed.

    // Pointer to first column entry in the ith row of M, Mi0; it may be 0.
    // The ith row of M forms the current row in the L-matrix.
    int start_of_row_i_in_M = i * (i + 1) / 2;

    // Pointer to the diagonal entry in the ith row of M, Mii.
    // Points to the pivotal element if this is an independent variable.
    int end_of_row_i_in_M = start_of_row_i_in_M + i;

    i4_[i] = end_of_row_i_in_M;

    // Process a dependent variable when true.
    if(is_dependent_po3[i]) {
      // Initialize incremental count of l1 for current variable.
      cl1 = 0;

      // Initialize incremental count of m1 for current variable.
      cm1 = 0;

      // Initialize incremental count of m2 for current variable.
      cm2 = 0;

      /*! number of independent variables in kinematic substructure that
      are coupled to the current dependent variable being eliminated */
      int coupled_dq_count = 0;

      /*! number of independent variables in kinematic substructure that
      are not coupled to the current dependent variable being eliminated */
      int noncoupled_dq_count = 0;

      // Classify all independent variables.
      /* here k is the index into the cb3 and b3 arrays */
      for(int k = rb3[i][0]; k < rb3[i][1]; k++) {

        int column_index = cb3[k];

        // If Mik != 0, variable k is coupled to variable i--add to
        // coupled_list.
        if(Mlb_[start_of_row_i_in_M + column_index]) {
          coupled_list[coupled_dq_count] = column_index;
          // coupled_index_list[coupled_dq_count] = k;
          coupled_dq_count++;
        } else {
          // If Mik = 0, variable k is not coupled to variable i--add to
          // non_coupled_list.
          non_coupled_list[noncoupled_dq_count] = column_index;
          // non_coupled_index_list[noncoupled_dq_count] = k;
          noncoupled_dq_count++;
        }
      }

      /*! As each dependent variable within a kinematic substructure is
      eliminated, assignment equations are generated and executed, based on a
      particular sequence of independent variables in that kinematic
      substructure. As each independent variable is accessed or processed, this
      variable is decremented. */
      int unprocessed_dq_count = coupled_dq_count + noncoupled_dq_count;

      /*
        Start the factorization process for this dependent variable.
        Loop until all independent variables are processed.
        here k is the value found at cb3, the index of an independent variable
        associated with a particular column of the b matrix and differs from the
        k used in the for loop above */
      int k;
      while(unprocessed_dq_count) {

        /* Process a coupled variable if any left. */
        // int k_index_in_B; /*!< points to the current index into b3
        if(coupled_dq_count) {
          coupled_dq_count--;
          k = coupled_list[coupled_dq_count];
          // k_index_in_B = coupled_index_list[coupled_dq_count];
        } else {
          // Otherwise process a non-coupled variable--
          // there must be at least one.
          noncoupled_dq_count--;
          k = non_coupled_list[noncoupled_dq_count];
          // k_index_in_B = non_coupled_index_list[noncoupled_dq_count];
        }

        // Reduce count because current variable will be processed.
        unprocessed_dq_count--;

        // Pointer to first entry in kth row of M, Mk0.
        int start_of_row_k_in_M = k * (k + 1) / 2;

        // The mass matrix term on the diagonal is nonzero.
        // Other terms in row/column i may be nonzero when true.
        // All terms in row/column i are zero when false, so skip projections
        // involving mass terms.
        if(Mlb_[end_of_row_i_in_M]) {
          // Process pointers for if (k >= j)Mkj -= Mij*Bik else Mjk -= Mij*Bik.

          // Index over the row i entries in M, starting left of diagonal.
          for(int j = i - 1; j > -1; j--) {
            // i2[l1] = start_of_row_i_in_M + j; Mij.
            int i2s = start_of_row_i_in_M + j;

            // if (im[i2[l1]]) If Mij != 0 then process.
            if(Mlb_[i2s]) {
              // if (k >= j) i1[l1] = start_of_row_k_in_M + j; Mkj.
              int i1s;
              if(k >= j) {
                i1s = start_of_row_k_in_M + j;
              } else {
                //  else i1[l1] = j*(j+1)/2 + k; //Mjk.
                i1s = j * (j + 1) / 2 + k;
              }
              //  im[i1[l1]] = 1; //Record assignment to Mjk or Mkj.
              Mlb_[i1s] = true;

              //  i3[l1] = k_index_in_B; //Bik.
              l1++;

              // Advance the count of l1.
              cl1++;
            } // end if (im[i2[l1]]).
          }   // end for (j = i-1; j >= 0; j--).

          // Process pointers for Mik -= Mii*Bik.
          //  i1[l1] = start_of_row_i_in_M + k; //Mik.
          int i1s = start_of_row_i_in_M + k;

          //  im[i1[l1]] = 1; //Record assignment to Mik.
          Mlb_[i1s] = true;

          //  i2[l1] = start_of_row_i_in_M + i; Mii.
          // int i2s = start_of_row_i_in_M + i;

          //  i3[l1] = k_index_in_B; //Bik.
          l1++;

          // Advance the count of l1.
          cl1++;

          // Process pointers for Mkk -= Mik*Bik.
          //  i1[l1] = start_of_row_k_in_M + k; //Mkk.
          i1s = start_of_row_k_in_M + k;

          //  im[i1[l1]] = 1; //Record assignment to Mkk.
          Mlb_[i1s] = true;

          //  i2[l1] = start_of_row_i_in_M + k; //Mik.

          //  i3[l1] = k_index_in_B; //Bik.
          l1++;

          // Advance the count of l1.
          cl1++;

        } // end if (im[end_of_row_i_in_M]).

        // Process pointers for Qk -= Bik*Qi and ddqi -= Bik*gak.
        //  j1[m1] = k;.

        //  j2[m1] = k_index_in_B; //Points to current entry in the L-matrix.

        m1++;  // Points to next entry in the L-matrix.
        cm1++; // Advance the count of m1.

        // Check to see if any fills have turned non-coupled into coupled.
        int j = 0;

        // Search through the non-coupled list.
        while(j < noncoupled_dq_count) {
          // Get the index of a non-coupled variable.
          k = non_coupled_list[j];

          // If true, a fill has coupled this variable.
          if(Mlb_[start_of_row_i_in_M + k]) {
            // Add to end of coupled list.
            coupled_list[coupled_dq_count] = k;
            coupled_dq_count++;

            /* Remove this variable from the non-coupled list and compress
               the list. */
            for(k = j; k < noncoupled_dq_count - 1; k++) {
              non_coupled_list[k] = non_coupled_list[k + 1];
            }

            /* Decrement size of non-coupled list. */
            noncoupled_dq_count--;

            /* Need this to counter the j++; below, because list size was
               decremented. */
            j--;
          } // end if (im[start_of_row_i_in_M + k]).

          // Move on to next variable in list.
          j++;
        } // end while (j < noncoupled_dq_count).
      }   // end while(numberOfUnprocessedVariables).

      /* FORWARD ELIMINATION / BACKWARD SUBSTITUTION */

      /* This block sets pointers for forward elimination/back substitution.

      Index over the row i entries in M, starting left of diagonal. */
      for(int j = i - 1; j > -1; j--) {
        // Mij.
        int pMij = start_of_row_i_in_M + j;

        // If Mij != 0, some operations are required.
        if(Mlb_[pMij]) {
          // Process pointers for Qj -= Mij * gai and lai -= Mij * ddqj.
          //  j3[m2] = j;.

          //  j4[m2] = pMij; //Points to current entry in the L-matrix.

          // Points to next entry in the L-matrix.
          m2++;

          // Advance the count of m2.
          cm2++;
        } // end if (im[pMij]).
      }   // end for (j = i-1; j >= 0; j--).

      // Save the operations count for this dependent variable.
      n1_[nd] = cl1;
      p1_[nd] = cm1;
      p2_[nd] = cm2;

      // Move forward for the next dependent variable.
      nd++;
    } // end if (depend[i]).

    // if (!depend[i]) -- Process an independent variable.
    else {
      // Initialize incremental count of l2 for current variable.
      cl2 = start_of_work_in_M_;

      // Initialize incremental count of l3 for current variable.
      cl3 = 0;

      // Initialize incremental count of m3 for current variable.
      cm3 = 0;

      //  p = startOfWorkInM; //Index into the M work array.

      // Index over the row i entries in M, starting left of the diagonal.
      for(int j = i - 1; j > -1; j--) {
        // If Mij != 0 then process.
        if(Mlb_[start_of_row_i_in_M + j]) {

          //  i5[l2] = start_of_row_i_in_M + j; Mij.
          int start_of_row_j_in_M = j * (j + 1) / 2;

          // Index over the row i entries in M, starting at j.
          for(int k = j; k > -1; k--) {
            // Save pointer to Mik, but Mik may be zero.
            //  i8[l3] = start_of_row_i_in_M + k;
            int i8s = start_of_row_i_in_M + k;

            //  if (im[i8[l3]]) //If Mik != 0 then process.
            if(Mlb_[i8s]) {
              //  i6[l3] = start_of_row_j_in_M + k; Mjk.
              int i6s = start_of_row_j_in_M + k;
              Mlb_[i6s] = true;
              //  i7[l3] = p; //Mij}.
              l3++;

              // Advance the count of l3.
              cl3++;
            } // end if (im[i7[l3]]).
          }   // end for (k = i-1; k <= j; k--).

          // Advance the index into the M work array for the next term.
          //  p++;

          // This block sets pointers for forward elimination/back substitution.
          // Process pointers for Qj -= Mij * Qi and ddqi -= Mij*ddqj.
          //  j5[m3] = j;

          // Points to entry in the L-matrix.
          //  j6[m3] = i5[l2];
          m3++;

          // Advance the count of m3.
          cm3++;

          // Now done with i5[l2] so can move on to next term.
          l2++;

          // Advance the count of l2.
          cl2++;
        } // end if (im[start_of_row_i_in_M + j]).
      }   // end for (j = i-1; j >=0; j--).

      // Save the operations count for this independent variable.
      n2_[ni] = cl2;
      n3_[ni] = cl3;
      p3_[ni] = cm3;

      // Move forward for the next independent variable.
      ni++;
    } // end if (!depend[i]).
  }   // end for (i = n-1; i >= 0; i--).

  /*
    Allocate arrays here
  */
  i1_.resize(l1);
  i2_.resize(l1);
  i3_.resize(l1);

  i5_.resize(l2);

  i6_.resize(l3);
  i7_.resize(l3);
  i8_.resize(l3);

  j1_.resize(m1);
  j2_.resize(m1);

  j3_.resize(m2);
  j4_.resize(m2);

  j5_.resize(m3);
  j6_.resize(m3);

  // Restore Mlb for subsequent processing.
  Mlb_.assign(Mlbs_.begin(), Mlbs_.end());
}

void shift::core::solver::RNCA::recordOperations() {

  assert(context_);

  const auto& jacobian = context_->jacobian();
  const auto& is_dependent_po3 = jacobian.isDependentPO3();
  const auto& rb3 = jacobian.rb3();
  const auto& cb3 = jacobian.cb3();

  int l1 = 0;
  int l2 = 0;
  int l3 = 0;

  int m1 = 0;
  int m2 = 0;
  int m3 = 0;

  // RECORD OPERATIONS BLOCK
  int coupled_list[dq_count_];
  int coupled_index_list[dq_count_];
  int non_coupled_list[dq_count_];
  int non_coupled_index_list[dq_count_];
  //
  /*! As each dependent variable within a kinematic substructure is eliminated,
  assignment equations are generated and executed based on a particular
  sequence of independent variables in that kinematic substructure. As each
  independent variable is accessed or processed, this variable is decremented.
  */

  // Backward recursion for matrix factorization.
  // i points to the current variable being processed.
  for(int i = dq_count_ - 1; i > -1; i--) {

    // Pointer to first column entry in the ith row of M, Mi0.
    // The ith row forms the current row in the L-matrix.
    int start_of_row_i_in_M = i * (i + 1) / 2;

    // Pointer to diagonal entry in the ith row of M, Mii.
    // Points to pivotal element if an independent variable.
    int end_of_row_i_in_M = start_of_row_i_in_M + i;

    // Process a dependent variable.
    if(is_dependent_po3[i]) {

      // Initialize number of coupled independent variables.
      int coupled_dq_count = 0;

      // Initialize number of non-coupled independent variables.
      int noncoupled_dq_count = 0;

      // Classify all independent variables.
      for(int k = rb3[i][0]; k < rb3[i][1]; k++) {
        int column_index = cb3[k];

        // If Mik != 0, variable k is coupled to variable i--add to
        // coupled_list.
        if(Mlb_[start_of_row_i_in_M + column_index]) {
          coupled_list[coupled_dq_count] = column_index;
          coupled_index_list[coupled_dq_count] = k;
          coupled_dq_count++;
        } else {
          // If Mik = 0, variable k is not coupled to variable i--add to
          // non_coupled_list.
          non_coupled_list[noncoupled_dq_count] = column_index;
          non_coupled_index_list[noncoupled_dq_count] = k;
          noncoupled_dq_count++;
        }
      }

      /* Set the remaining number of independent variables in substructure to
        be processed. */
      int unprocessed_dq_count = coupled_dq_count + noncoupled_dq_count;

      // Start the factorization process for this dependent variable.
      int k;
      while(unprocessed_dq_count) {
        /* Process a coupled variable if any left. */
        int k_index_in_B; /*!< points to the current index into b3 */
        if(coupled_dq_count) {
          coupled_dq_count--;
          k = coupled_list[coupled_dq_count];
          k_index_in_B = coupled_index_list[coupled_dq_count];
        } else {
          // Otherwise process a non-coupled variable--
          // there must be at least one.
          noncoupled_dq_count--;
          k = non_coupled_list[noncoupled_dq_count];
          k_index_in_B = non_coupled_index_list[noncoupled_dq_count];
        }

        // Reduce count because current variable will be processed.
        unprocessed_dq_count--;

        // Pointer to first entry in kth row of M, Mk0.
        int start_of_row_k_in_M = k * (k + 1) / 2;

        // The mass matrix term on the diagonal is nonzero.
        // Other terms in row/column i may be nonzero when true.
        // All terms in row/column i are zero when false, so skip projections
        // involving mass terms.
        if(Mlb_[end_of_row_i_in_M]) {
          // Process pointers for if (k >= j)Mkj += Mij*Bik else Mjk += Mij*Bik.
          // Index over the row i entries in M, starting left of diagonal.
          for(int j = i - 1; j > -1; j--) {
            // Mij.
            i2_[l1] = start_of_row_i_in_M + j;
            // If Mij != 0 then process.
            if(Mlb_[i2_[l1]]) {
              // Mkj.
              if(k >= j) {
                i1_[l1] = start_of_row_k_in_M + j;
              }
              // Mjk.
              else {
                i1_[l1] = j * (j + 1) / 2 + k;
              }

              // Record assignment to Mjk or Mkj.
              Mlb_[i1_[l1]] = true;

              // Bik.
              i3_[l1] = k_index_in_B;
              l1++;
            } // end if (im[i2[l1]]).
          }   // end for (j = i-1; j >= 0; j--).

          // Process pointers for Mik += Mii*Bik.

          // Mik.
          i1_[l1] = start_of_row_i_in_M + k;

          // Record assignment to Mik.
          Mlb_[i1_[l1]] = true;

          // Mii.
          i2_[l1] = start_of_row_i_in_M + i;

          // Bik.
          i3_[l1] = k_index_in_B;
          l1++;

          // Process pointers for Mkk += Mik*Bik.
          // Mkk.
          i1_[l1] = start_of_row_k_in_M + k;

          // Record assignment to Mkk.
          Mlb_[i1_[l1]] = true;

          // Mik.
          i2_[l1] = start_of_row_i_in_M + k;

          // Bik.
          i3_[l1] = k_index_in_B;

          l1++;

        } // end if (im[end_of_row_i_in_M]).

        // Process pointers for Qk += Bik * Qi and ddqi += Bik * gak.
        j1_[m1] = k;

        // Points to entry in the L-matrix.
        j2_[m1] = k_index_in_B;
        m1++;

        // Check to see if any fills have turned non-coupled into coupled.
        int j = 0;

        // Search through the non-coupled list.
        while(j < noncoupled_dq_count) {
          // Get the number of a non-coupled variable.
          k = non_coupled_list[j];

          // If true, a fill has turned variable into coupled.
          if(Mlb_[start_of_row_i_in_M + k]) {
            // Add to end of coupled list.
            coupled_list[coupled_dq_count] = k;
            coupled_dq_count++;

            // Remove this variable from the non-coupled list and compress the
            // list.
            for(k = j; k < noncoupled_dq_count - 1; k++) {
              non_coupled_list[k] = non_coupled_list[k + 1];
            }

            // Decrement size of non-coupled list.
            noncoupled_dq_count--;

            // Need this to counter the j++; below, because list size was
            // decremented.
            j--;

          } // end if(im[start_of_row_i_in_M + k]).

          // Move on to next variable in list.
          j++;

        } // end while (j < noncoupled_dq_count).

      } // end while (numberOfUnprocessedVariables).

      // This block sets pointers for forward elimination/back substitution.
      // Index over the row i entries in M, starting left of diagonal.
      for(int j = i - 1; j > -1; j--) {
        // Mij.
        int pMij = start_of_row_i_in_M + j;

        // If Mij != 0, some operations are required.
        if(Mlb_[pMij]) {
          // Process pointers for Qj -= Mij*gai and lai -= Mij*ddqj.
          j3_[m2] = j;

          // Points to entry in the L-matrix.
          j4_[m2] = pMij;
          m2++;
        } // end if (im[pMij]).
      }   // end for (j = i-1; j >= 0; j--).
    } else {
      // if (!depend[i])--Process an independent variable.

      // Index into the M work array.
      int p = start_of_work_in_M_;

      // Index over the row i entries in M, starting left of the diagonal.
      for(int j = i - 1; j >= 0; j--) {
        // If Mij != 0 then process.
        if(Mlb_[start_of_row_i_in_M + j]) {
          // Mij.
          i5_[l2] = start_of_row_i_in_M + j;
          int start_of_row_j_in_M = j * (j + 1) / 2;

          // Index over the row j entries in M, starting at j.
          for(int k = j; k > -1; k--) {
            // Save pointer to Mik, but Mik may be zero.
            i8_[l3] = start_of_row_i_in_M + k;
            // If Mik != 0 then process.
            if(Mlb_[i8_[l3]]) {
              // Mjk.
              i6_[l3] = start_of_row_j_in_M + k;
              Mlb_[i6_[l3]] = true;

              // Mij.
              i7_[l3] = p;
              l3++;
            } // end if(im[i6[l3]]).
          }   // end for(k = j; k >= 0; k--).

          // Advance the index into the M work array for the next term.
          p++;

          // This block sets pointers for forward elimination/back substitution.
          // Process pointers for Qj -= Mij*Qi and ddqi -=Mij*ddqj.
          j5_[m3] = j;

          // Points to entry in the L-matrix.
          j6_[m3] = i5_[l2];
          m3++;

          // Now done with i5[l2] so can move on to next term.
          l2++;
        } // end if (im[start_of_row_i_in_M + j]).
      }   // end for (j = i-1; j >=0; j--).
    }     // end if (!depend[i]).
  }       // end for (i = n-1; i >= 0; i--).
}

void shift::core::solver::RNCA::countPointers() {
  assert(context_);
  const auto& jacobian = context_->jacobian();
  const auto& is_dependent_po3 = jacobian.isDependentPO3();

  /* Currently on entry to this function, context_->nd = 0. Normally
  k = context_->nd - 1; The following effectively sets k = context_->nd - 1; */
  int ni = 0;
  int nd = 0;
  int k = -1;

  /* Currently matrix factoring starts at memoized_pointers_[0].
  Eventually matrix factoring will start at some offset determined by
  other processes before it. Assuming that processing starts at
  memoized_pointers_[0], the first dq_count entries in memoized_pointers_[0]
  point to the diagonal entries in context_->M[], in reverse order.
  Thus p2 starts at dq_count.*/
  int p2 = dq_count_;

  /* Count the total number of operations
  required to factor the matrix and access the L matrix. */
  int p3 = 0;

  for(size_t i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      k++;
    }
  }

  /* Factor the matrix */
  for(int i = (int)dq_count_ - 1; i > -1; i--) {
    if(is_dependent_po3[i]) {
      for(int j = 0; j < n1_[nd]; j++) {
        // M[] -= M[] * B[];
        // *(memoized_pointers_[p2]) -= *(memoized_pointers_[p2 + 1]) *
        // *(memoized_pointers_[p2 + 2]);
        p2 += 3;
      }
      nd++;
    } else {
      // Invert the pivot: M[i] = 1.0 / M[i];
      // *(memoized_pointers_[i]) = 1.0 / *(memoized_pointers_[i]);

      // Copy the nonzero entries in row i of the M[]
      // array to the left of the diagonal into the work array
      // and scale the entries in row i to L terms.
      for(int j = start_of_work_in_M_; j < n2_[ni]; j++) {
        // Copy entry into work array: M[] = M[];
        // *(memoized_pointers_[p2]) = *(memoized_pointers_[p2 + 1]);
        p2++;
        // Scale entry to evaluate L term: M[] *= M[];
        // *(memoized_pointers_[p2]) *= *(memoized_pointers_[i]);
        p2++;
      }
      // Perform mass projections with one assignment statement.
      for(int j = 0; j < n3_[ni]; j++) {
        // Update Schur-complement matrix: M[] -= M[] * M[];
        // *(memoized_pointers_[p2]) -= *(memoized_pointers_[p2 + 1]) *
        // *(memoized_pointers_[p2 + 2]);
        p2 += 3;
      }
      ni++;
    }
  }

  k = nd - 1;
  nd = 0;
  ni = 0;

  // Backward elimination
  for(int i = (int)dq_count_ - 1; i > -1; i--) {
    if(is_dependent_po3[i]) {
      for(int j = 0; j < p1_[nd]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= B[] * Q[i];
        // *(memoized_pointers_[p3++]) -= *(memoized_pointers_[p2++]) *
        // context_->fl[i];
        p3++;
        p2++;
      }
      // Project constraint-induced inertia forces onto parent forces.
      // Does not execute if Mii = 0.
      for(int j = 0; j < p2_[nd]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * ga[k];
        // *(memoized_pointers_[p3++]) -= *(memoized_pointers_[p2++]) *
        // g_amma_d[k];
        p3++;
        p2++;
      }
      nd++;

      // Part of x <-- D^{-1} b:
      // context_->ddq[i] = g_amma_d[k];
      // context_->la[k] = context_->fl[i];

      if(Mlb_[i4_[i]])
      // Part of x <-- D^{-1} b: la[k] -= M[] * ddq[i];
      // context_->la[k] -= *(memoized_pointers_[i]) * context_->ddq[i];
      {
        k--;
      }
    } else {
      for(int j = 0; j < p3_[ni]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * Q[i];
        // *(memoized_pointers_[p3++]) -= *(memoized_pointers_[p2++]) *
        // context_->fl[i];
        p3++;
        p2++;
      }
      // Part of x <-- D^{-1} b: ddq[i] = M[] * Q[i];
      // context_->ddq[i] = *(memoized_pointers_[i]) * context_->fl[i];
      ni++;
    }
  }

  /* Reset for indexing forward through context_->la[k] */
  k = 0;

  /* P2 is one past last entry in L. Decrement for reverse direction. */
  // p2--;

  // Forward substitution
  for(size_t i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      nd--;
      for(int j = 0; j < p2_[nd]; j++) {
        // Part of x <-- L^{-1} x: la[k] -= M[] * ddq[];
        // context_->la[k] -= *(memoized_pointers_[p2--]) *
        // *(memoized_pointers_[p3++]);
        p3++;
      }
      for(int j = 0; j < p1_[nd]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= B[] * ddq[];
        // context_->ddq[i] -= *(memoized_pointers_[p2--]) *
        // *(memoized_pointers_[p3++]);
        p3++;
      }
      k++;
    } else {
      ni--;
      for(int j = 0; j < p3_[ni]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= M[] * ddq[];
        // context_->ddq[i] -= *(memoized_pointers_[p2--]) *
        // *(memoized_pointers_[p3++]);
        p3++;
      }
    }
  }

  memoized_pointers_.clear();
  memoized_pointers_.resize(p3 + p2);
  std::fill(memoized_pointers_.begin(), memoized_pointers_.end(), nullptr);

  p3_start_ = p2;
}

void shift::core::solver::RNCA::recordPointers() {
  assert(context_);
  const auto& jacobian = context_->jacobian();
  const auto& is_dependent_po3 = jacobian.isDependentPO3();
  // const auto& rb3 = jacobian.rb3();
  // const auto& cb3 = jacobian.cb3();
  const auto& b3 = jacobian.b3();
  // const auto& pv_23 = jacobian.pv23();
  const auto& pv_13 = jacobian.pv13();

  auto& ddq = context_->ddq();

  int ni = 0;
  int nd = 0;
  int l1 = 0;
  int l2 = 0;
  int l3 = 0;
  int m1 = 0;
  int m2 = 0;
  int m3 = 0;

  /* Currently matrix factoring starts at memoized_pointers_[0].
  Eventually matrix factoring will start at some offset determined by
  other processes before it. Assuming that processing starts at
  memoized_pointers_[0], the first context_->n entries in memoized_pointers_[0]
  point to the diagonal entries in context_->M[], in reverse order.
  Thus p2 starts at dq_count_.*/
  int p2 = dq_count_;

  /* Subsequently the AOP that sets up memoized_pointers_[0] for matrix
  factorization and solution will determine in advance the total number of
  operations required to factor the matrix and access the L matrix. It will then
  set an offset value for initializing p3. */
  int p3 = p3_start_;

  for(int i = (int)dq_count_ - 1; i > -1; i--) {
    /* Point to the current diagonal entry in context_->M[] */
    memoized_pointers_[i] = &Ml_[i4_[i]];
    if(is_dependent_po3[i]) {
      // Processing dependent variable
      for(int j = 0; j < n1_[nd]; j++) {
        /* M[] -= M[] * B[]; */
        memoized_pointers_[p2++] = &Ml_[i1_[l1]];
        memoized_pointers_[p2++] = &Ml_[i2_[l1]];
        memoized_pointers_[p2++] = const_cast<double*>(&b3[i3_[l1]]);
        l1++;
      }
      nd++;
    } else {
      // Processing independent variable
      /* M[] = 1.0 / M[]; This pointer already set above*/
      for(int j = start_of_work_in_M_; j < n2_[ni]; j++) {
        /* M[] = M[]; Copy row i, col j of context_->M[] into work area. */
        memoized_pointers_[p2++] = &Ml_[j];
        /* M[] *= M[]; Scale row i, col j of context_->M[] into L matrix. */
        memoized_pointers_[p2++] = &Ml_[i5_[l2]];
        l2++;
      }
      // Update Schur-complement
      for(int j = 0; j < n3_[ni]; j++) {
        /* M[] -= M[] * M[]; */
        memoized_pointers_[p2++] = &Ml_[i6_[l3]];
        memoized_pointers_[p2++] = &Ml_[i7_[l3]];
        memoized_pointers_[p2++] = &Ml_[i8_[l3]];
        l3++;
      }
      ni++;
    }
  }

  dependent_count_ = nd;
  int k = nd - 1;

  ni = 0;
  nd = 0;
  l1 = 0;
  l2 = 0;
  l3 = 0;
  m1 = 0;
  m2 = 0;
  m3 = 0;

  // Start of Backward Elimination
  for(int i = (int)dq_count_ - 1; i > -1; i--) {
    if(is_dependent_po3[i]) {
      /* Processing dependent variable: b <- L^{-T} b */
      for(int j = 0; j < p1_[nd]; j++) {
        /* Q[] -= B[] * Q[i]; */
        memoized_pointers_[p3++] = &fl_[j1_[m1]];
        memoized_pointers_[p2++] = const_cast<double*>(&b3[j2_[m1]]);
        m1++;
      }
      for(int j = 0; j < p2_[nd]; j++) {
        /* Q[] -= M[] * ga[k]; */
        memoized_pointers_[p3++] = &fl_[j3_[m2]];
        memoized_pointers_[p2++] = &Ml_[j4_[m2]];
        m2++;
      }
      nd++;

      /* x <- D^{-1} b: No additional pointers required here
         ddq[i] = ga[k];
         la[k] = Q[i];
         if(context_->Mlb[context_->i4[i]]){
           la[k] -= M[] * ddq[i]; */
      k--;
    } else {
      /* Processing independent variable: b <- L^{-T} b */
      for(int j = 0; j < p3_[ni]; j++) {
        /* Q[] -= M[] * Q[i] */
        memoized_pointers_[p3++] = &fl_[j5_[m3]];
        memoized_pointers_[p2++] = &Ml_[j6_[m3]];
        m3++;
      }
      /* x <- D^{-1} b: No additional pointers required here
      ddq[i] = M[] * Q[i]; */
      ni++;
    }
  }

  // Start of Forward Substitution
  k = 0;
  for(size_t i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      /* Processing dependent variable: x <- L^{-1} x */
      nd--;
      for(int j = 0; j < p2_[nd]; j++) {
        --(m2);
        /* la[k] -= M[] * ddq[]; */
        memoized_pointers_[p3++] = &ddq[pv_13[j3_[m2]]];
      }
      for(int j = 0; j < p1_[nd]; j++) {
        --(m1);
        /* ddq[i] -= B[] * ddq[]; */
        memoized_pointers_[p3++] = &ddq[pv_13[j1_[m1]]];
      }
      k++;
    } else {
      /* Processing independent variable x <- L^{-1} x */
      ni--;
      for(int j = 0; j < p3_[ni]; j++) {
        --(m3);
        memoized_pointers_[p3++] = &ddq[pv_13[j5_[m3]]];
      }
    }
  }
}

bool shift::core::solver::RNCA::setEquations() {
  assert(context_);
  return setEquations(0.0, context_->q());
}

bool shift::core::solver::RNCA::setEquationsAndRecordFills() {
  assert(context_);
  setEquationsAndRecordFills(0.0, context_->q());
  return false;
}

bool shift::core::solver::RNCA::setEquationsAndRecordFills(
  const double t, const std::vector<double>& y) {
  const double* q = y.data();
  const double* dq = q + q_count_;
  assert(context_);
  auto& jacobian = context_->jacobian();
  context_->updateDHVA(q, dq);
  jacobian.save();
  jacobian.setPO2();
  jacobian.factorPO2();
  jacobian.setBMatrixSparse();
  unsigned int infinity_norm_check = jacobian.infinityNormCheck(
    t, const_cast<double*>(q), dq, infinity_norm_multiplier_);
  if(infinity_norm_check) {
    if(infinity_norm_check == 2) {
      std::cerr << "Newton-raphson failure, exiting simulation." << std::endl;
      return true;
    }
    std::cerr << "Failed infinity norm check, but GECP was successfully "
                 "reapplied and simulation will attempt to continue"
              << std::endl;
    jacobian.analyze();
    setRHS();
    setMassMatrixAndRecordFills();
    memoize();
  } else {
    setRHS();
    setMassMatrixAndRecordFills();
  }
  return false;
}

bool shift::core::solver::RNCA::setEquations(const double t,
                                       const std::vector<double>& y) {
  const double* q = y.data();
  const double* dq = q + q_count_;
  assert(context_);
  auto& jacobian = context_->jacobian();
  context_->updateDHVA(q, dq);
  jacobian.save();
  jacobian.setPO2();
  jacobian.factorPO2();
  jacobian.setBMatrixSparse();
  unsigned int infinity_norm_check = jacobian.infinityNormCheck(
    t, const_cast<double*>(q), dq, infinity_norm_multiplier_);
  if(infinity_norm_check) {
    if(infinity_norm_check == 2) {
      std::cerr << "Newton-raphson failure, exiting simulation." << std::endl;
      return true;
    }
    std::cerr << "Failed infinity norm check, but GECP was successfully "
                 "reapplied and simulation will attempt to continue"
              << std::endl;
    jacobian.analyze();
    setRHS();
    setMassMatrixAndRecordFills();
    memoize();
  } else {
    setRHS();
    setMassMatrix();
  }
  return false;
}

void shift::core::solver::RNCA::setRHS() {

  const topology::DHVA& dhva = context_->dhva();
  unsigned int non_rigid_joint_count = context_->nonRigidJointCount();
  const auto& dq_lookup_compressed = context_->dqLookupCompressed();
  const auto& pcl_compressed = context_->pclCompressed();
  const auto& has_mass = context_->hasMassArray();

  const auto& d_reference_frames = context_->dReferenceFrames();
  const auto& inertia = context_->inertiaArray();

  context_->evaluateConstraints();

  std::fill(fl_.begin(), fl_.end(), 0);
  std::fill(p_.begin(), p_.end(), ksl_coscrew(0, 0, 0, 0, 0, 0));
  std::fill(g_.begin(), g_.end(), ksl_coscrew(0, 0, 0, 0, 0, 0));

  const ksl_screw_t* aq = dhva.aq;
  const ksl_screw_t* v = dhva.v;

  /* transform inertias to fixed reference frame, carried out in PO1*/
  std::fill(inertia_p_.begin(), inertia_p_.end(),
            ksl_inertia(0, {{0, 0, 0}}, 0, 0, 0, 0, 0, 0));
  ksl_inertia_t* inertia_p = inertia_p_.data() + 1;

#ifdef SHIFT_USE_OPENMP_
#pragma omp parallel for
#endif
  for(size_t i = 0; i < non_rigid_joint_count; i++) {
    if(has_mass[i]) {
      ksl_inertia_transformed(&inertia[i], dhva.d + d_reference_frames[i],
                              &inertia_p[i]);
    }
  }

/* g <- g - M \gamma, (carried out in PO1) */
#ifdef SHIFT_USE_OPENMP_
#pragma omp parallel for
#endif
  for(size_t j = 0; j < non_rigid_joint_count; j++) {

    g_[j].m0 -= inertia_p[j].m * aq[j].m0;
    g_[j].m1 -= inertia_p[j].m * aq[j].m1;
    g_[j].m2 -= inertia_p[j].m * aq[j].m2;

    g_[j].m4 -= inertia_p[j].mt.z * aq[j].m0;
    g_[j].m5 += inertia_p[j].mt.y * aq[j].m0;

    g_[j].m3 += inertia_p[j].mt.z * aq[j].m1;
    g_[j].m5 -= inertia_p[j].mt.x * aq[j].m1;

    g_[j].m3 -= inertia_p[j].mt.y * aq[j].m2;
    g_[j].m4 += inertia_p[j].mt.x * aq[j].m2;

    g_[j].m1 += inertia_p[j].mt.z * aq[j].m3;
    g_[j].m2 -= inertia_p[j].mt.y * aq[j].m3;

    g_[j].m3 -= inertia_p[j].Ixx * aq[j].m3;
    g_[j].m4 -= inertia_p[j].Ixy * aq[j].m3;
    g_[j].m5 -= inertia_p[j].Izx * aq[j].m3;

    g_[j].m0 -= inertia_p[j].mt.z * aq[j].m4;
    g_[j].m2 += inertia_p[j].mt.x * aq[j].m4;

    g_[j].m3 -= inertia_p[j].Ixy * aq[j].m4;
    g_[j].m4 -= inertia_p[j].Iyy * aq[j].m4;
    g_[j].m5 -= inertia_p[j].Iyz * aq[j].m4;

    g_[j].m0 += inertia_p[j].mt.y * aq[j].m5;
    g_[j].m1 -= inertia_p[j].mt.x * aq[j].m5;

    g_[j].m3 -= inertia_p[j].Izx * aq[j].m5;
    g_[j].m4 -= inertia_p[j].Iyz * aq[j].m5;
    g_[j].m5 -= inertia_p[j].Izz * aq[j].m5;
  }

#ifdef SHIFT_USE_OPENMP_
#pragma omp parallel for
#endif
  for(size_t j = 0; j < non_rigid_joint_count; j++) {
    /* p = Mv, (carried out in PO1) */

    p_[j].m0 += inertia_p[j].m * v[j].m0;
    p_[j].m1 += inertia_p[j].m * v[j].m1;
    p_[j].m2 += inertia_p[j].m * v[j].m2;

    p_[j].m4 += inertia_p[j].mt.z * v[j].m0;
    p_[j].m5 -= inertia_p[j].mt.y * v[j].m0;

    p_[j].m3 -= inertia_p[j].mt.z * v[j].m1;
    p_[j].m5 += inertia_p[j].mt.x * v[j].m1;

    p_[j].m3 += inertia_p[j].mt.y * v[j].m2;
    p_[j].m4 -= inertia_p[j].mt.x * v[j].m2;

    p_[j].m1 -= inertia_p[j].mt.z * v[j].m3;
    p_[j].m2 += inertia_p[j].mt.y * v[j].m3;

    p_[j].m3 += inertia_p[j].Ixx * v[j].m3;
    p_[j].m4 += inertia_p[j].Ixy * v[j].m3;
    p_[j].m5 += inertia_p[j].Izx * v[j].m3;

    p_[j].m0 += inertia_p[j].mt.z * v[j].m4;
    p_[j].m2 -= inertia_p[j].mt.x * v[j].m4;

    p_[j].m3 += inertia_p[j].Ixy * v[j].m4;
    p_[j].m4 += inertia_p[j].Iyy * v[j].m4;
    p_[j].m5 += inertia_p[j].Iyz * v[j].m4;

    p_[j].m0 -= inertia_p[j].mt.y * v[j].m5;
    p_[j].m1 += inertia_p[j].mt.x * v[j].m5;

    p_[j].m3 += inertia_p[j].Izx * v[j].m5;
    p_[j].m4 += inertia_p[j].Iyz * v[j].m5;
    p_[j].m5 += inertia_p[j].Izz * v[j].m5;
  }

  /* g += cross(v)^T * p (carried out in PO1)*/
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(size_t j = 0; j < non_rigid_joint_count; j++) {

    // g_[j][0:3] -= cross(context_->v[j][3:6], p_[j][0:3]);
    // x1 = m3, y1 = m4, z1 = m5; x2 = m0, y2 = m1, z2 = m2
    g_[j].m0 -= v[j].m4 * p_[j].m2;
    g_[j].m1 -= v[j].m5 * p_[j].m0;
    g_[j].m2 -= v[j].m3 * p_[j].m1;

    // g_[j][3:6] -= cross(context_->v[j][0:3], p_[j][0:3]);
    // x1 = m0, y1 = m1, z1 = m2; x2 = m0, y2 = m1, z2 = m2
    g_[j].m3 -= v[j].m1 * p_[j].m2;
    g_[j].m4 -= v[j].m2 * p_[j].m0;
    g_[j].m5 -= v[j].m0 * p_[j].m1;

    // g_[j][3:6] -= cross(context_->v[j][3:6], p_[j][3:6]);
    // x1 = m3, y1 = m4, z1 = m5; x2 = m3, y2 = m4, z2 = m5
    g_[j].m3 -= v[j].m4 * p_[j].m5;
    g_[j].m4 -= v[j].m5 * p_[j].m3;
    g_[j].m5 -= v[j].m3 * p_[j].m4;
  }

  for(size_t j = 0; j < non_rigid_joint_count; j++) {
    g_[j].m0 += v[j].m5 * p_[j].m1;
    g_[j].m1 += v[j].m3 * p_[j].m2;
    g_[j].m2 += v[j].m4 * p_[j].m0;

    g_[j].m3 += v[j].m2 * p_[j].m1;
    g_[j].m4 += v[j].m0 * p_[j].m2;
    g_[j].m5 += v[j].m1 * p_[j].m0;

    g_[j].m3 += v[j].m5 * p_[j].m4;
    g_[j].m4 += v[j].m3 * p_[j].m5;
    g_[j].m5 += v[j].m4 * p_[j].m3;
  }

  /* g <- C{-T} * (g + \tilde{v}^T p - m\gamma) (carried out in PO1) */
  for(int j = (int)non_rigid_joint_count - 1; j > -1; j--) {
    int i = pcl_compressed[j];
    if(i > -1) {
      g_[i].m0 += g_[j].m0;
      g_[i].m1 += g_[j].m1;
      g_[i].m2 += g_[j].m2;
      g_[i].m3 += g_[j].m3;
      g_[i].m4 += g_[j].m4;
      g_[i].m5 += g_[j].m5;
    }
  }

  /* f_l = f + H^T * C{-T} * (g + \tilde{v}^Tp - m\gamma)
    this step is evaluated in PO1 and stored in PO3,
    use the permutation array p_v31 to assign to f_l

    at this point the quantity g contains C{-T} (g + \tilde{v}^Tp - m\gamma)
    which has length nonRigidJointCount and is stored in PO1.

    Expand each multi DOF joint to expand the quantity in g with length
    nonRigidJointCount in PO1 to fl of length dqCount in PO3 */
  assert(context_);
  const auto& f = context_->jointWorkingForces();
  const auto& joint_dof_compressed = context_->jointDOFCompressed();
  const auto& is_rotation = context_->isRotation();

  const auto& jacobian = context_->jacobian();
  const auto& pv_31 = jacobian.pv31();

  assert(integrator_);
  const double baumgarte_2_omega_n = integrator_->baumgarte2OmegaN();
  const double baumgarte_omega_n_2 = integrator_->baumgarteOmegaN2();

  std::cout << "baumgarte_2_omega_n: " << baumgarte_2_omega_n << std::endl;
  std::cout << "baumgarte_omega_n_2: " << baumgarte_omega_n_2 << std::endl;

  const auto& integral_constraint_error =
    integrator_->integralConstraintError();

#ifdef SHIFT_USE_OPENMP_
#pragma omp parallel for
#endif
  for(size_t j = 0; j < non_rigid_joint_count; j++) {
    /* j is index into compressed list of joints and g in PO1,
    increment l and k for each additional degree of freedom
    for composite joints */
    for(size_t m = 0; m < joint_dof_compressed[j]; m++) {

      /* l = index into dq array in PO1 */
      int l = dq_lookup_compressed[j] + m;

      /* k is index into dq array in PO3 */
      int k = pv_31[l];

      /* f_l <- f */
      fl_[k] = f[l];
      if(is_rotation[l]) {
        /* f_l += h^T * g yields the final f_l*/
        fl_[k] += ksl_dot_cs(&g_[j], &dhva.h[l]);
      } else {
        fl_[k] += dhva.h[l].m0 * g_[j].m0;
        fl_[k] += dhva.h[l].m1 * g_[j].m1;
        fl_[k] += dhva.h[l].m2 * g_[j].m2;
      }
    }
  }

  /* evaluate primary constraints (\bar{\gamma}_p) in PO(1), store in PO(2) */
  int chord_constraint_offset = 0;
  for(auto& substructure : jacobian.substructures()) {

    const auto& pr_21 = substructure.pr21();
    int offset = substructure.dqOffset();
    const double* cs = substructure.cs();
    const auto& constraints = substructure.constraintIndices();

    std::cout << "pr_21: ";
    for(auto& i : pr_21) {
      std::cout << i << " ";
    }
    std::cout << std::endl;

    /* number of constraints to evaluate */
    for(size_t k = 0; k < substructure.chordConstraintCount();
        k++, chord_constraint_offset++) {

      ksl_screw_print(stdout, &dhva.aq[constraints[k].child.v], "aq child: ");
      ksl_screw_print(stdout, &dhva.aq[constraints[k].parent.v], "aq parent: ");

      ksl_screw_print(stdout, &dhva.v[constraints[k].child.v], "v child: ");
      ksl_screw_print(stdout, &dhva.v[constraints[k].parent.v], "v parent: ");

      /* set gamma_p */

      // acceleration level
      for(size_t l = 0; l < 6; l++) {
        if(pr_21[k * 6 + l] < substructure.rank()) {
          /*
            first constraint is embedded (host) body
            second is the virtual body (released frame)

            prInverse = pr_21
            pr = p12
          */
          gamma_d_[offset + pr_21[k * 6 + l]] =
            dhva.aq[constraints[k].child.v].at[l] -
            dhva.aq[constraints[k].parent.v].at[l];
        }
      }

      /* subtract 2 * omega_n * \dot{phi} from gamma_p */

      // velocity level
      for(int l = 0; l < 6; l++) {
        if(pr_21[k * 6 + l] < substructure.rank()) {

          /* -(2 * omega_N) * \dot{\phi} velocity part of baumgarte
          constraint stabilization */
          gamma_d_[offset + pr_21[k * 6 + l]] -=
            baumgarte_2_omega_n * (dhva.v[constraints[k].parent.v].at[l] -
                                   dhva.v[constraints[k].child.v].at[l]);

          double v_term =
            baumgarte_2_omega_n * (dhva.v[constraints[k].child.v].at[l] -
                                   dhva.v[constraints[k].parent.v].at[l]);
          std::cout << v_term << " ";
        }
      }
      std::cout << std::endl;
      std::cout << "baumgarte_2_omega_n: " << baumgarte_2_omega_n << std::endl;

      /* subtract omega_n^2 * \dot{phi} from gamma_p */

      // position level
      for(int l = 0; l < 6; l++) {
        if(pr_21[k * 6 + l] < substructure.rank()) {

          /* -(omega_N^2) * phi, position part of baumgarte
          constraint stabilization */
          gamma_d_[offset + pr_21[k * 6 + l]] -=
            baumgarte_omega_n_2 * cs[k * 6 + l];
        }
      }

      /* substract integral error term from gamma_p  for adaptive constraint
       * stabilization*/

      // integral gain
#if 1
      for(int l = 0; l < 6; l++) {
        if(pr_21[k * 6 + l] < substructure.rank()) {
          gamma_d_[offset + pr_21[k * 6 + l]] -=
            integral_constraint_error[chord_constraint_offset * 6 + l];
        }
      }
#endif
    }
  }

  /* precondition primary constraints in PO(2)
  \bar{\gamma}_d = U^{-1} L^{-1} \bar{\gamma}_p */
  for(auto& substructure : jacobian.substructures()) {

    int offset = substructure.dqOffset();
    const int dq_count = substructure.dqCount();
    const double* J = substructure.jacobianMatrix();
    const int rank = substructure.rank();

    /* forward elimination:
    \bar{\gamma} <- L^{-1} * \bar\{\gamma} */
    for(int i = 0; i < rank; i++) {
      for(int j = 0; j < i; j++) {
        gamma_d_[i + offset] -= J[i * dq_count + j] * gamma_d_[j + offset];
      }
    }

    /* back substitution
    \bar{\gamma} <- U^{-1} * \bar\{\gamma) */
    for(int i = rank - 1; i > -1; i--) {
      double save = 0;
      for(int j = rank - 1; j > i; j--) {
        save += J[i * dq_count + j] * gamma_d_[j + offset];
      }
      gamma_d_[i + offset] =
        (gamma_d_[i + offset] - save) / J[i * dq_count + i];
    }
  }

  std::cout << "gamma_d_: ";
  for(auto& val : gamma_d_) {
    std::cout << val << " ";
  }
  std::cout << std::endl;
}

/*! set mass matrix without setting boolean mass matrix */
void shift::core::solver::RNCA::setMassMatrix() {
  assert(context_);
  const auto& dhva = context_->dhva();
  const size_t non_rigid_joint_count = context_->nonRigidJointCount();
  const auto& pcl_compressed = context_->pclCompressed();
  const auto& joint_dof_compressed = context_->jointDOFCompressed();
  const auto& dq_lookup_compressed = context_->dqLookupCompressed();
  const auto& is_rotation = context_->isRotation();
  const auto& has_mass_dq = context_->hasMassdq();
  const auto& pcl_dq = context_->pcldq();

  const auto& jacobian = context_->jacobian();
  const auto& pv_31 = jacobian.pv31();

  std::fill(mh_product_.begin(), mh_product_.end(),
            ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  std::fill(Ml_.begin(), Ml_.end(), 0);
  ksl_inertia_t* inertia_p = inertia_p_.data() + 1;

  /* compute ``accumulated'' inertia matrices, project child inertia onto
   * parent
   */
  for(int j = non_rigid_joint_count - 1; j > -1; j--) {
    int i = pcl_compressed[j];

#ifdef SHIFT_WITH_BLAS_
    cblas_daxpy(10, 1.0, (double*)(inertia_p + j), 1, (double*)(inertia_p + i),
                1);
#else
    inertia_p[i].m += inertia_p[j].m;
    inertia_p[i].mt.x += inertia_p[j].mt.x;
    inertia_p[i].mt.y += inertia_p[j].mt.y;
    inertia_p[i].mt.z += inertia_p[j].mt.z;
    inertia_p[i].Ixx += inertia_p[j].Ixx;
    inertia_p[i].Iyy += inertia_p[j].Iyy;
    inertia_p[i].Izz += inertia_p[j].Izz;
    inertia_p[i].Ixy += inertia_p[j].Ixy;
    inertia_p[i].Iyz += inertia_p[j].Iyz;
    inertia_p[i].Izx += inertia_p[j].Izx;
#endif
  }

  /* compute mh product */
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(size_t j = 0; j < non_rigid_joint_count; j++) {
    for(size_t dof = 0; dof < joint_dof_compressed[j]; dof++) {
      int k = dq_lookup_compressed[j] + dof;

      /* update quantities that are processed for both rotation and
      translation */
      mh_product_[k].m0 += inertia_p[j].m * dhva.h[k].m0;
      mh_product_[k].m1 += inertia_p[j].m * dhva.h[k].m1;
      mh_product_[k].m2 += inertia_p[j].m * dhva.h[k].m2;

      mh_product_[k].m4 += inertia_p[j].mt.z * dhva.h[k].m0;
      mh_product_[k].m5 -= inertia_p[j].mt.y * dhva.h[k].m0;

      mh_product_[k].m3 -= inertia_p[j].mt.z * dhva.h[k].m1;
      mh_product_[k].m5 += inertia_p[j].mt.x * dhva.h[k].m1;

      mh_product_[k].m3 += inertia_p[j].mt.y * dhva.h[k].m2;
      mh_product_[k].m4 -= inertia_p[j].mt.x * dhva.h[k].m2;

      /* perform extra operations only for generalized coordinates with
       * rotational degrees of freedom */
      if(is_rotation[k]) {
        mh_product_[k].m1 -= inertia_p[j].mt.z * dhva.h[k].m3;
        mh_product_[k].m2 += inertia_p[j].mt.y * dhva.h[k].m3;
        mh_product_[k].m3 += inertia_p[j].Ixx * dhva.h[k].m3;
        mh_product_[k].m4 += inertia_p[j].Ixy * dhva.h[k].m3;
        mh_product_[k].m5 += inertia_p[j].Izx * dhva.h[k].m3;

        mh_product_[k].m0 += inertia_p[j].mt.z * dhva.h[k].m4;
        mh_product_[k].m2 -= inertia_p[j].mt.x * dhva.h[k].m4;
        mh_product_[k].m3 += inertia_p[j].Ixy * dhva.h[k].m4;
        mh_product_[k].m4 += inertia_p[j].Iyy * dhva.h[k].m4;
        mh_product_[k].m5 += inertia_p[j].Iyz * dhva.h[k].m4;

        mh_product_[k].m0 -= inertia_p[j].mt.y * dhva.h[k].m5;
        mh_product_[k].m1 += inertia_p[j].mt.x * dhva.h[k].m5;
        mh_product_[k].m3 += inertia_p[j].Izx * dhva.h[k].m5;
        mh_product_[k].m4 += inertia_p[j].Iyz * dhva.h[k].m5;
        mh_product_[k].m5 += inertia_p[j].Izz * dhva.h[k].m5;
      }
    }
  }

  /* use inverse permutation to save terms computed in PO1 order and
  store in PO3 */
  int m_3;
  int ks_3;
  int k_3;
  for(int j = (int)dq_count_ - 1; j > -1; j--) {
    if(has_mass_dq[j]) {
      m_3 = pv_31[j];
      // m_1 = j;
      // ks_1 = (m_1 * (m_1 + 1)) / 2;
      ks_3 = (m_3 * (m_3 + 1)) / 2;

      // k_1 = ks_1 + m_1;
      k_3 = ks_3 + m_3;
      if(is_rotation[j]) {
        Ml_[k_3] += ksl_dot_cs(&mh_product_[j], &dhva.h[j]);
      } else {
        Ml_[k_3] += mh_product_[j].m0 * dhva.h[j].m0;
        Ml_[k_3] += mh_product_[j].m1 * dhva.h[j].m1;
        Ml_[k_3] += mh_product_[j].m2 * dhva.h[j].m2;
      }

      int i = pcl_dq[j];
      while(i > -1) {
        int i_3 = pv_31[i];
        if(m_3 > i_3) {
          k_3 = ks_3 + i_3;
        } else {
          k_3 = (i_3 * (i_3 + 1)) / 2 + pv_31[j];
        }
        if(is_rotation[i]) {
          Ml_[k_3] += ksl_dot_cs(&mh_product_[j], &dhva.h[i]);
        } else {
          Ml_[k_3] += mh_product_[j].m0 * dhva.h[i].m0;
          Ml_[k_3] += mh_product_[j].m1 * dhva.h[i].m1;
          Ml_[k_3] += mh_product_[j].m2 * dhva.h[i].m2;
        }
        i = pcl_dq[i];
      }
    }
  }
}

void shift::core::solver::RNCA::setMassMatrixAndRecordFills() {
  assert(context_);
  const auto& dhva = context_->dhva();
  const size_t non_rigid_joint_count = context_->nonRigidJointCount();
  const auto& pcl_compressed = context_->pclCompressed();
  const auto& joint_dof_compressed = context_->jointDOFCompressed();
  const auto& dq_lookup_compressed = context_->dqLookupCompressed();
  const auto& is_rotation = context_->isRotation();
  const auto& has_mass_dq = context_->hasMassdq();
  const auto& pcl_dq = context_->pcldq();

  const auto& jacobian = context_->jacobian();
  const auto& pv_31 = jacobian.pv31();

  std::fill(mh_product_.begin(), mh_product_.end(),
            ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  std::fill(Ml_.begin(), Ml_.end(), 0);
  ksl_inertia_t* inertia_p = inertia_p_.data() + 1;
  std::fill(Mlb_.begin(), Mlb_.end(), 0);

  /* compute ``accumulated'' inertia matrices, project child inertia onto
   * parent
   */
  for(int j = non_rigid_joint_count - 1; j > -1; j--) {
    int i = pcl_compressed[j];

#ifdef SHIFT_WITH_BLAS_
    cblas_daxpy(10, 1.0, (double*)(inertia_p + j), 1, (double*)(inertia_p + i),
                1);
#else
    inertia_p[i].m += inertia_p[j].m;
    inertia_p[i].mt.x += inertia_p[j].mt.x;
    inertia_p[i].mt.y += inertia_p[j].mt.y;
    inertia_p[i].mt.z += inertia_p[j].mt.z;
    inertia_p[i].Ixx += inertia_p[j].Ixx;
    inertia_p[i].Iyy += inertia_p[j].Iyy;
    inertia_p[i].Izz += inertia_p[j].Izz;
    inertia_p[i].Ixy += inertia_p[j].Ixy;
    inertia_p[i].Iyz += inertia_p[j].Iyz;
    inertia_p[i].Izx += inertia_p[j].Izx;
#endif
  }

  /* compute mh product */
#ifdef _SHIFT_USE_OPENMP
#pragma omp parallel for
#endif
  for(size_t j = 0; j < non_rigid_joint_count; j++) {
    for(size_t dof = 0; dof < joint_dof_compressed[j]; dof++) {
      int k = dq_lookup_compressed[j] + dof;

      /* update quantities that are processed for both rotation and
      translation */
      mh_product_[k].m0 += inertia_p[j].m * dhva.h[k].m0;
      mh_product_[k].m1 += inertia_p[j].m * dhva.h[k].m1;
      mh_product_[k].m2 += inertia_p[j].m * dhva.h[k].m2;

      mh_product_[k].m4 += inertia_p[j].mt.z * dhva.h[k].m0;
      mh_product_[k].m5 -= inertia_p[j].mt.y * dhva.h[k].m0;

      mh_product_[k].m3 -= inertia_p[j].mt.z * dhva.h[k].m1;
      mh_product_[k].m5 += inertia_p[j].mt.x * dhva.h[k].m1;

      mh_product_[k].m3 += inertia_p[j].mt.y * dhva.h[k].m2;
      mh_product_[k].m4 -= inertia_p[j].mt.x * dhva.h[k].m2;

      /* perform extra operations only for generalized coordinates with
       * rotational degrees of freedom */
      if(is_rotation[k]) {
        mh_product_[k].m1 -= inertia_p[j].mt.z * dhva.h[k].m3;
        mh_product_[k].m2 += inertia_p[j].mt.y * dhva.h[k].m3;
        mh_product_[k].m3 += inertia_p[j].Ixx * dhva.h[k].m3;
        mh_product_[k].m4 += inertia_p[j].Ixy * dhva.h[k].m3;
        mh_product_[k].m5 += inertia_p[j].Izx * dhva.h[k].m3;

        mh_product_[k].m0 += inertia_p[j].mt.z * dhva.h[k].m4;
        mh_product_[k].m2 -= inertia_p[j].mt.x * dhva.h[k].m4;
        mh_product_[k].m3 += inertia_p[j].Ixy * dhva.h[k].m4;
        mh_product_[k].m4 += inertia_p[j].Iyy * dhva.h[k].m4;
        mh_product_[k].m5 += inertia_p[j].Iyz * dhva.h[k].m4;

        mh_product_[k].m0 -= inertia_p[j].mt.y * dhva.h[k].m5;
        mh_product_[k].m1 += inertia_p[j].mt.x * dhva.h[k].m5;
        mh_product_[k].m3 += inertia_p[j].Izx * dhva.h[k].m5;
        mh_product_[k].m4 += inertia_p[j].Iyz * dhva.h[k].m5;
        mh_product_[k].m5 += inertia_p[j].Izz * dhva.h[k].m5;
      }
    }
  }

  /* use inverse permutation to save terms computed in PO1 order and
  store in PO3 */
  int m_3;
  int ks_3;
  int k_3;
  for(int j = (int)dq_count_ - 1; j > -1; j--) {
    if(has_mass_dq[j]) {
      m_3 = pv_31[j];
      // m_1 = j;
      // ks_1 = (m_1 * (m_1 + 1)) / 2;
      ks_3 = (m_3 * (m_3 + 1)) / 2;

      // k_1 = ks_1 + m_1;
      k_3 = ks_3 + m_3;
      Mlb_[k_3] = true;

      if(is_rotation[j]) {
        Ml_[k_3] += ksl_dot_cs(&mh_product_[j], &dhva.h[j]);
      } else {
        Ml_[k_3] += mh_product_[j].m0 * dhva.h[j].m0;
        Ml_[k_3] += mh_product_[j].m1 * dhva.h[j].m1;
        Ml_[k_3] += mh_product_[j].m2 * dhva.h[j].m2;
      }

      int i = pcl_dq[j];
      while(i > -1) {
        int i_3 = pv_31[i];
        if(m_3 > i_3) {
          k_3 = ks_3 + i_3;
        } else {
          k_3 = (i_3 * (i_3 + 1)) / 2 + pv_31[j];
        }
        Mlb_[k_3] = true;

        if(is_rotation[i]) {
          Ml_[k_3] += ksl_dot_cs(&mh_product_[j], &dhva.h[i]);
        } else {
          Ml_[k_3] += mh_product_[j].m0 * dhva.h[i].m0;
          Ml_[k_3] += mh_product_[j].m1 * dhva.h[i].m1;
          Ml_[k_3] += mh_product_[j].m2 * dhva.h[i].m2;
        }
        i = pcl_dq[i];
      }
    }
  }
}

void shift::core::solver::RNCA::factorAndSolve() {

  assert(context_);
  auto& ddq = context_->ddq();

  shift::core::Jacobian& jacobian = context_->jacobian();
  const auto& pv_23 = jacobian.pv23();
  const auto& pv_13 = jacobian.pv13();
  const auto& is_dependent_po3 = jacobian.isDependentPO3();

  /* Access entries in P_[] to factor matrix and access L */
  int p2 = dq_count_;

  /* Access entries in P_[] to process rhs and lhs */
  int p3 = p3_start_;

  int ni = 0;
  int nd = 0;
  // int k = context_->dependentCount - 1;

  /* Factor the matrix */
  for(int i = dq_count_ - 1; i > -1; i--) {
    if(is_dependent_po3[i]) {
      for(int j = 0; j < n1_[nd]; j++) {
        /* M[] -= M[] * B[]; */
        *(memoized_pointers_[p2]) -=
          *(memoized_pointers_[p2 + 1]) * *(memoized_pointers_[p2 + 2]);
        p2 += 3;
      }
      nd++;
    } else {
      /* Invert the pivot: M[i] = 1.0 / M[i]; */
      assert(fabs(*(memoized_pointers_[i])) > 1e-9);
      *(memoized_pointers_[i]) = 1.0 / *(memoized_pointers_[i]);

      /* Copy the nonzero entries in row i of the M[]
         array to the left of the diagonal into the work array and scale the
         entries in row i to L terms. */
      for(int j = start_of_work_in_M_; j < n2_[ni]; j++) {
        // Copy entry into work array: M[] = M[];
        *(memoized_pointers_[p2]) = *(memoized_pointers_[p2 + 1]);
        p2++;
        // Scale entry to evaluate L term: M[] *= M[];
        *(memoized_pointers_[p2]) *= *(memoized_pointers_[i]);
        p2++;
      }
      // Perform mass projections with one assignment statement.
      for(int j = 0; j < n3_[ni]; j++) {
        /* Update Schur-complement matrix: M[] -= M[] * M[]; */
        *(memoized_pointers_[p2]) -=
          *(memoized_pointers_[p2 + 1]) * *(memoized_pointers_[p2 + 2]);
        p2 += 3;
      }
      ni++;
    }
  }
  nd = 0;
  ni = 0;

  /* Backward elimination */
  for(int i = dq_count_ - 1; i > -1; i--) {
    if(is_dependent_po3[i]) {
      for(int j = 0; j < p1_[nd]; j++) {
        /* Part of b <-- L^{-T} b: Q[] -= B[] * Q[i]; */
        *(memoized_pointers_[p3++]) -= *(memoized_pointers_[p2++]) * fl_[i];
      }
      // Project constraint-induced inertia forces onto parent forces.
      // Does not execute if Mii = 0.
      for(int j = 0; j < p2_[nd]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * ga[k];
        *(memoized_pointers_[p3++]) -=
          *(memoized_pointers_[p2++]) * gamma_d_[pv_23[i]];
      }
      nd++;

      /* Part of x <-- D^{-1} b: */
      ddq[pv_13[i]] = gamma_d_[pv_23[i]];
      la_[pv_23[i]] = fl_[i];

      if(Mlb_[i4_[i]]) {
        /* Part of x <-- D^{-1} b: la[k] -= M[] * ddq[i]; */
        la_[pv_23[i]] -= *(memoized_pointers_[i]) * ddq[pv_13[i]];
      }
      // k--;
    } else {
      for(int j = 0; j < p3_[ni]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * Q[i];
        *(memoized_pointers_[p3++]) -= *(memoized_pointers_[p2++]) * fl_[i];
      }
      // Part of x <-- D^{-1} b: ddq[i] = M[] * Q[i];
      ddq[pv_13[i]] = *(memoized_pointers_[i]) * fl_[i];
      ni++;
    }
  }

  /* Reset for indexing forward through context_->la[k] */
  // k = 0;

  /* P2 is one past last entry in L. Decrement for reverse direction. */
  p2--;

  /*
    Forward substitution
  */
  for(size_t i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      nd--;
      for(int j = 0; j < p2_[nd]; j++) {
        // Part of x <-- L^{-1} x: la[k] -= M[] * ddq[];
        la_[pv_23[i]] -=
          *(memoized_pointers_[p2--]) * *(memoized_pointers_[p3++]);
      }
      for(int j = 0; j < p1_[nd]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= B[] * ddq[];
        ddq[pv_13[i]] -=
          *(memoized_pointers_[p2--]) * *(memoized_pointers_[p3++]);
      }
      // k++;
    } else {
      ni--;
      for(int j = 0; j < p3_[ni]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= M[] * ddq[];
        ddq[pv_13[i]] -=
          *(memoized_pointers_[p2--]) * *(memoized_pointers_[p3++]);
      }
    }
  }
}

/*! setup and solve the equations of motion given a vector of states y[]

y[] contains the generalized coordinates q[], the first time derivatives dq[],
and any states required for force modules

y = [q, dq, q_f]
dy = [dq, ddq, dq_f]
*/
bool shift::core::solver::RNCA::solve(const double t, const std::vector<double>& y,
                                std::vector<double>& dy) {

  assert(context_);
  assert(q_count_ == context_->qCount());

  const double* q = y.data();
  const double* dq = q + q_count_;

  context_->updateDHVA(q, dq);
  context_->evaluateConstraints();
  context_->setForces(t, y);

  if(setEquations(t, y)) {
    return true;
  }

#ifdef SHIFT_VERIFY_EQUATIONS_
  /* make a copy of Ml and fl to verify equations of motion, remove this
  once equations of motion verified */
  double Ml[dq_count_ * dq_count_];
  for(unsigned int i = 0; i < context_->nonRigidJointCount(); i++) {
    int istart = i * (i + 1) / 2;
    for(unsigned int j = 0; j < i + 1; j++) {
      Ml[i * context_->nonRigidJointCount() + j] = Ml_[istart + j];
      if(i != j) {
        Ml[j * context_->nonRigidJointCount() + i] = Ml_[istart + j];
      }
    }
  }
#endif

#ifdef SHIFT_VERIFY_EQUATIONS_
  /* copy fl to a second array to verify solution of
  equations of motion */
  double fl[dq_count_];
  memcpy(fl, fl_.data(), dq_count_ * sizeof(double));

  std::cout << "fl (before factor): ";
  for(unsigned int i = 0; i < dq_count_; i++) {
    std::cout << fl_[i] << " ";
  }
  std::cout << std::endl;
#endif

  factorAndSolve();

#if SHIFT_VERIFY_EQUATIONS_
  std::cout << "fl (after factor): ";
  for(unsigned int i = 0; i < dq_count_; i++) {
    std::cout << fl_[i] << " ";
  }
  std::cout << std::endl;

  /* verify that multiplying coefficient matrix with lhs gets back the
  right hand side */
  // std::cout << "(" << t << ") ";
  // std::cout << " y: ";
  // for(unsigned int i = 0; i < q_count_ + dq_count_; i++) {
  //   std::cout << y[i] << " ";
  // }
  // std::cout << std::endl;
  verify(t, y, Ml, fl);
#endif

  /* convert angular velocities to time derivative of quaternions
    copy by value all other dq terms to dy array */
  context_->dqTody(y, dy);

#if SHIFT_VERIFY_EQUATIONS_
  std::cout << "y: ";
  for(size_t i = 0; i < q_count_ + dq_count_; i++) {
    std::cout << y[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "dy: ";
  for(size_t i = 0; i < dq_count_ + dq_count_; i++) {
    std::cout << dy[i] << " ";
  }
  std::cout << std::endl;
#endif

  return false;
}

bool shift::core::solver::RNCA::verify(const double t,
                                 const std::vector<double>& /*dy*/, double Ml[],
                                 double fl[]) {
  bool verified = true;
  (void)t;

#ifdef SHIFT_VERIFY_EQUATIONS_

  // std::cout << "verifying equations: ";
  // printf("joint names in PO1:\n");
  // for(int i = 0; i < context->activeJointCount; i++) {
  //   printf("   [%d] ", i);
  //   s2_stackstr_print(context->jointPointers[i]->name);
  // }

  // printf("isDependent in po1:\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("  [%d] %s\n", i, context->isDependentPO1[i] ? "true":"false");
  // }
  // printf("\n");
  //
  // printf("isDependent in po3:\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("  [%d] %s\n", i, context->isDependentPO3[i] ? "true":"false");
  // }

  // printf("pv_12: ");
  // s2_arrayi_print(context->dqCount, context->pv_12);
  //
  // printf("pv_21: ");
  // s2_arrayi_print(context->dqCount, context->pv_21);
  //
  // printf("pv_13: ");
  // s2_arrayi_print(context->dqCount, context->pv_13);
  //
  // printf("pv_31: ");
  // s2_arrayi_print(context->dqCount, context->pv_31);
  //
  // printf("pv_32: ");
  // s2_arrayi_print(context->dqCount, context->pv_32);
  //
  // printf("pv_23: ");
  // s2_arrayi_print(context->dqCount, context->pv_23);

  /* get dim of jacobian */
  // int rank = 0;
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   rank += context->substructures[sub].rank;
  // }

  /* initialize coefficient matrix to zero */
  // double* Ml =
  //   calloc(
  //     context->nonRigidJointCount *
  //     context->nonRigidJointCount,
  //     sizeof(double));

  // printf("# updating dhva\n");
  // s2_updateDHVA(context);

  // printf("# verifying aq terms:\n");
  // for(int i = 0; i < context->dqCount + 1; i++) {
  //   printf("   [%d] ", i);
  //   s2_screw_print(context->aq + i);
  // }

  // printf("# evaluating constraints\n");
  // s2_jacobian_evaluateConstraints(context);

  // printf("# evaluating equations\n");
  // s2_equation_setEquations(context);

  // printf("b3 array after set equation: \n");
  // s2_array_print(context->nnzb, context->b3);

  // printf("\nMl after set equation: \n");
  // s2_triang_print(context->dqCount, context->Ml);

  /* copy Ml to full dense array */

  // printf("\nMl (full dense version) after set equation:\n");
  // s2_array_print2D(context->dqCount, context->dqCount, true, Ml);
  //
  // printf("\nfl after set equations: \n   ");
  // s2_array_print(context->dqCount, context->fl);

  // printf("\n# factoring and solving equations\n");
  // s2_equation_factorAndSolve(context);
  //
  // printf("b3 array after factor / solve equation: ");
  // s2_array_print(context->nnzb, context->b3);
  //
  // printf("\nddq (PO1): \n   ");
  // s2_array_print(context->dqCount, context->ddq);

  /* copy ddq to PO3 */
  auto& jacobian = context_->jacobian();
  const auto& pv_13 = jacobian.pv13();
  const auto& pv_23 = jacobian.pv23();

  const auto& ddq = context_->ddq();

  double ddq_po3[dq_count_];
  memset(ddq_po3, 0, dq_count_ * sizeof(double));
  for(unsigned int i = 0; i < dq_count_; i++) {
    ddq_po3[i] = ddq[pv_13[i]];
  }

  // printf("ddq (po1):\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("[%d] %f\n", i, context->ddq[i]);
  // }

  // printf("context->pv_13:\n");
  // s2_arrayi_print(context->dqCount, context->pv_13);

  // printf("\nddq (PO3): \n   ");
  // s2_array_print(context->dqCount, ddq_po3);
  //
  // printf("\nla (PO2):\n   ");
  // s2_array_print(context->dqCount, context->la);

  double la_po3[dq_count_];
  memset(la_po3, 0, dq_count_ * sizeof(double));
  for(unsigned int i = 0; i < dq_count_; i++) {
    la_po3[i] = la_[pv_23[i]];
  }

  double fl_po3[dq_count_];
  memset(fl_po3, 0, dq_count_ * sizeof(double));
  // printf("\nempty fl_po3 array:\n  ");
  // s2_array_print(context->dqCount, fl_po3);
  //
  // printf("\nijMapping arrays:\n");
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   printf("   substructure [%d]\n", sub);
  //   s2_substructure_t* substructure = context->substructures + sub;
  //
  //   for(int i = 0; i < substructure->constraintEquationCount; i++) {
  //     printf("   ");
  //     for(int j = 0; j < substructure->dqCount; j++) {
  //       printf("%d ", substructure->ijMapping[i * substructure->dqCount +
  //       j]);
  //     }
  //     printf("\n");
  //   }
  // }

  cblas_dgemv(CblasRowMajor, CblasNoTrans, dq_count_, dq_count_, 1, Ml,
              dq_count_, ddq_po3, 1, 0.0, fl_po3, 1);

  // printf("fl_po3 after adding Ml * ddq:\n");
  // s2_array_print(context->dqCount, fl_po3);

  // printf("\nfl_po3 <- Ml * ddq: \n   ");
  // s2_array_print(context->dqCount, fl_po3);

  /* set full dense b matrix for each substructure */
  // s2_jacobian_setBMatrix(context);

  /* print full dense B matrix for each substructure */
  // printf("full dense b matrix in PO2\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //
  //   printf("substructure [%d]\n", i);
  //   for(int i = 0; i < substructure->rank; i++) {
  //     for(int j = substructure->rank; j < substructure->dqCount; j++) {
  //       printf("%f ", substructure->J[i * substructure->dqCount + j]);
  //     }
  //     printf("\n");
  //   }
  //   printf("\n");
  // }

  const auto& rb3 = jacobian.rb3();
  const auto& cb3 = jacobian.cb3();
  const auto& b3 = jacobian.b3();

  const auto& is_dependent_po3 = jacobian.isDependentPO3();

  double B[dq_count_ * dq_count_];
  memset(B, 0, dq_count_ * dq_count_ * sizeof(double));
  int row = 0;
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {

        int col = cb3[j];
        B[i * dq_count_ + col] = b3[j];
        B[i * dq_count_ + i] = 1.0;
      }
      row++;
    }
  }

  cblas_dgemv(CblasRowMajor, CblasTrans, dq_count_, dq_count_, 1.0, B,
              dq_count_, la_po3, 1, 1.0, fl_po3, 1);

  if(!ksl_allclose(dq_count_, fl_po3, fl)) {

    std::cerr << ">>> Error, equations of motion not solved correctly."
              << std::endl;

    verified = false;
    std::cerr << "verification failed" << std::endl;

    std::cerr << "  context->fl (original): ";
    for(unsigned int i = 0; i < dq_count_; i++) {
      printf("%0.5g, ", fl[i]);
    }
    std::cerr << std::endl << "  context->fl (recovered): ";
    for(unsigned int i = 0; i < dq_count_; i++) {
      printf("%0.5g, ", fl_po3[i]);
    }
    printf("\n");

    auto& jacobian = context_->jacobian();
    std::cout << "infinity norm failures: " << std::endl;
    for(auto& infinity_norm_failure : jacobian.infinityNormFailures()) {
      std::cout << "t: " << infinity_norm_failure.time << " ";
      for(auto& index : infinity_norm_failure.substructure_index) {
        std::cout << index << " ";
      }
      std::cout << std::endl;
    }
    exit(1);
  }

  double gamma_d_po3[dq_count_];
  memset(gamma_d_po3, 0, dq_count_ * sizeof(double));
  cblas_dgemv(CblasRowMajor, CblasNoTrans, dq_count_, dq_count_, 1.0, B,
              dq_count_, ddq_po3, 1, 0.0, gamma_d_po3, 1);

  /* permute gamma_d po3 to po2 */
  double gamma_d_po2[dq_count_];
  memset(gamma_d_po2, 0, dq_count_ * sizeof(double));
  for(unsigned int i = 0; i < dq_count_; i++) {
    gamma_d_po2[pv_23[i]] = gamma_d_po3[i];
  }

  if(!ksl_allclose(dq_count_, gamma_d_po2, gamma_d_.data())) {

    std::cerr << ">>> Error, equations of motion not solved correctly."
              << std::endl;

    verified = false;

    fprintf(stderr, "\nerror: gamma_d (po2): \n");
    for(unsigned int i = 0; i < dq_count_; i++) {
      fprintf(stderr, "%0.5g, ", gamma_d_[i]);
    }
    fprintf(stderr, "\n");

    fprintf(stderr, "\n             != gamma_d (po2): \n");
    for(unsigned int i = 0; i < dq_count_; i++) {
      fprintf(stderr, "%0.5g, ", gamma_d_po2[i]);
    }
    fprintf(stderr, "\n");
    exit(1);
  }
  // std::cout << "verified: " << (verified ? "true" : "false") << std::endl;
#endif

  return verified;
}
