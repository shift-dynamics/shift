#include <assert.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"
#include "shift/core/solver/rnca.h"
#include "shift/core/tangents.h"

#include "ksl/array.h"
#include "ksl/inertia.h"
#include "ksl/print.h"

#ifdef SHIFT_VERIFY_EQUATIONS_
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif

/*!
@brief compares a computed array against a recovered array using a per-entry
scale equal to the sum of absolute values of the terms that formed that entry,
so that roundoff from large cancelling terms (e.g. large constraint forces near
a singular configuration) is not mistaken for a solver failure
*/
static bool VerificationClose(const int count, const double* computed,
                              const double* recovered, const double* scale) {
  for(int i = 0; i < count; i++) {
    const double scale_factor =
      fmax(fmax(scale[i], fabs(computed[i])), 1.0);
    if(fabs(computed[i] - recovered[i]) / scale_factor > 1e-6) {
      SHIFT_ERROR("Verification failed at index " + std::to_string(i) +
                  ": |difference| / scale = " +
                  std::to_string(fabs(computed[i] - recovered[i]) /
                                 scale_factor));
      return false;
    }
  }
  return true;
}
#endif

bool shift::core::solver::RNCA::registered_ =
  shift::core::Factory<shift::core::solver::Base>::get()->RegisterModule(
    "rnca", shift::core::solver::RNCA::Create);

std::unique_ptr<shift::core::solver::RNCA> shift::core::solver::RNCA::Create() {
  return std::make_unique<shift::core::solver::RNCA>();
}

void shift::core::solver::RNCA::DeclareParameters() {
  shift::core::solver::Base::DeclareParameters();

  DeclareParameter(
    "infinity_norm_multiplier", &infinity_norm_multiplier_,
    "The infinity norm of each Jacobian matrix's *B* matrix is used as a "
    "heuristic to determine when to reapply Gaussian elimination with complete "
    "row and column pivoting on as-needed basis. GECP will be reapplied when "
    "the infinity norm reaches this multiple of the initial starting value -- "
    "default: 2.0.",
    false);

  DeclareParameter(
    "refactor_criterion", &refactor_criterion_name_,
    "What reapplies Gaussian elimination with complete row and column "
    "pivoting (GECP): \"infinity_norm\", the infinity norm of B reaching "
    "infinity_norm_multiplier times its value at the last GECP, or "
    "\"mobility_epsilon\", the mobility number of an independent velocity divided by the mobility falling below "
    "epsilon_mobility. The pivot ratio is checked whichever is chosen -- "
    "default: \"infinity_norm\".",
    false);

  DeclareParameter(
    "epsilon_mobility", &epsilon_mobility_,
    "With refactor_criterion \"mobility_epsilon\", GECP is reapplied when "
    "the mobility number of an independent velocity divided by the mobility "
    "falls below this value -- default: 0.01.",
    false);
}

void shift::core::solver::RNCA::Initialize() {
  assert(context_);

  if(refactor_criterion_name_ == "infinity_norm") {
    refactor_criterion_ = Substructure::Refactor::INFINITY_NORM;
  } else if(refactor_criterion_name_ == "mobility_epsilon") {
    refactor_criterion_ = Substructure::Refactor::MOBILITY_EPSILON;
  } else {
    throw shift::core::Exception(
      "Unknown refactor_criterion \"" + refactor_criterion_name_ +
      "\"; expected \"infinity_norm\" or \"mobility_epsilon\"");
  }

  q_count_ = context_->get_q_count();
  dq_count_ = context_->get_dq_count();

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

  inertia_p_.resize(context_->get_non_rigid_joint_count() + 1);
  std::fill(inertia_p_.begin(), inertia_p_.end(),
            ksl_inertia(0.0, {{0.0, 0.0, 0.0}}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  p_.resize(context_->get_non_rigid_joint_count());
  std::fill(p_.begin(), p_.end(), ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  g_.resize(context_->get_non_rigid_joint_count());
  std::fill(g_.begin(), g_.end(), ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  la_.resize(context_->get_dq_count());
  std::fill(la_.begin(), la_.end(), 0);

  infinity_norm_failure_ = false;

  if(delegated_body_.size() != context_->get_non_rigid_joint_count()) {
    delegated_body_.assign(context_->get_non_rigid_joint_count(), false);
  }
  if(delegated_dq_.size() != dq_count_) {
    delegated_dq_.assign(dq_count_, false);
  }
  delegated_po3_.assign(dq_count_, false);

  elastic_entries_.clear();
  has_delegated_elastic_ = false;
  for(int i : context_->get_elastic_bodies()) {
    const int j = context_->get_active_articulated_bodies()[i]->get_va_index();
    if(delegated_body_[j]) {
      has_delegated_elastic_ = true;
    } else {
      elastic_entries_.push_back(j);
    }
  }

  SetupElasticLookup();

  context_->get_jacobian().RemoveConstraintViolations();
  /* the initial velocities of a model file seldom satisfy the velocity
  constraints (a rate given to a crank and none to the rest of its loop) */
  context_->get_jacobian().MakeVelocitiesConsistent();
  context_->get_jacobian().Analyze();
  context_->UpdateDHVA();
  context_->EvaluateConstraints();
  SetupEquationsAndRecordFills();
  Memoize();
}

inline void PrintVector(const std::string& label, const std::vector<double>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(auto& i : v) {
    ss << i << " ";
  }
  ss << "\n";
}

inline void PrintVector(const std::string& label, const std::vector<int>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(auto& i : v) {
    ss << i << " ";
  }
  ss << "\n";
}

inline void PrintVector(const std::string& label, const std::vector<unsigned int>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(auto& i : v) {
    ss << i << " ";
  }
  ss << "\n";
}


inline void PrintVector(const std::string& label, const std::vector<bool>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(size_t i = 0; i < v.size(); i++) {
    ss << v[i] << " ";
  }
  ss << "\n";
}

inline void PrintVector(const std::string& label,
                        const std::vector<ksl_coscrew_t>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(auto& val : v) {
    for(size_t i = 0; i < 6; i++) {
      ss << val.at[i] << " ";
    }
  }
  ss << "\n";
}

inline void PrintVector(const std::string& label,
                        const std::vector<ksl_screw_t>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(auto& val : v) {
    for(size_t i = 0; i < 6; i++) {
      ss << val.at[i] << " ";
    }
  }
  ss << "\n";
}

inline void PrintVector(const std::string& label,
                        const std::vector<ksl_inertia_t>& v,
                        std::stringstream& ss) {
  ss << "  " << label << ": ";
  for(auto& val : v) {
    for(size_t i = 0; i < 10; i++) {
      ss << val.at[i] << " ";
    }
  }
  ss << "\n";
}


void shift::core::solver::RNCA::PrintStates(std::stringstream& ss) {
  double t;
  double dt;
  const auto& f = context_->get_joint_working_forces();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& is_rotation = context_->get_is_rotation();
  const auto& jacobian = context_->get_jacobian();
  const auto& pv_31 = jacobian.get_pv31();
  const double baumgarte_2_omega_n = integrator_->get_baumgarte_2_omega_n();
  const auto& integral_constraint_error = integrator_->get_integral_error();

  context_->get_current_time(t, dt);
  ss << "  t: " << t << "\n";
  ss << "  dt: " << dt << "\n";
  ss << "  infinity_norm_failure_: " << infinity_norm_failure_ << "\n";
  ss << "  dependent_count_: " << dependent_count_ << "\n";
  ss << "  q_count_: " << q_count_ << "\n";
  ss << "  dq_count_: " << dq_count_ << "\n";
  PrintVector("gamma_d_", gamma_d_, ss);
  PrintVector("Ml_", Ml_, ss);
  ss << "  Ml_size_: " << Ml_size_ << "\n";
  PrintVector("Mls_", Mls_, ss);
  PrintVector("p_", p_, ss);
  PrintVector("mh_product_", mh_product_, ss);
  PrintVector("inertia_p_", inertia_p_, ss);
  PrintVector("fl_", fl_, ss);
  PrintVector("g_", g_, ss);
  PrintVector("la_", la_, ss);
  ss << "  omega_n: " << baumgarte_2_omega_n / 2.0 << "\n";
  PrintVector("f", f, ss);
  PrintVector("joint_dof_compressed", joint_dof_compressed, ss);
  PrintVector("is_rotation", is_rotation, ss);
  PrintVector("pv_31", pv_31, ss);
  PrintVector("e", jacobian.get_constraints(), ss);
  PrintVector("de", jacobian.get_de(), ss);
  PrintVector("integral_error", integral_constraint_error, ss);
  context_->get_dhva().Print(ss);
}

double shift::core::solver::RNCA::GravitationalPotentialEnergy(
  shift::core::topology::DHVA& dhva) {
  if(!inertia_p_.empty()) {
    return ksl_dot_vv(&inertia_p_[0].mt, &dhva.aq[-1].lin);
  } else {
    return 0.0;
  }
}

double
shift::core::solver::RNCA::KineticEnergy(shift::core::topology::DHVA& dhva) {
#if SHIFT_WITH_BLAS_
  return cblas_ddot(6 * non_rigid_joint_count_, (double*)(p_.data()), 1,
                    (double*)(dhva.v), 1) /
           2.0 +
         ElasticKineticEnergy(dhva, has_delegated_elastic_ ? &elastic_entries_
                                                           : nullptr);
#else
  double sum = 0.0;
  for(size_t i = 0; i < context_->get_non_rigid_joint_count(); i++) {
    sum += ksl_dot_cs(&(p_[i]), &dhva.v[i]);
  }
  return sum / 2.0 +
         ElasticKineticEnergy(dhva, has_delegated_elastic_ ? &elastic_entries_
                                                           : nullptr);
#endif
}

void shift::core::solver::RNCA::Memoize() {
  /* the permutation to order 3 changes when the Jacobian is analyzed again */
  const auto& pv_31 = context_->get_jacobian().get_pv31();
  for(unsigned int l = 0; l < dq_count_; l++) {
    delegated_po3_[pv_31[l]] = delegated_dq_[l];
  }
  CountOperations();
  RecordOperations();
  CountPointers();
  RecordPointers();
  RecordMassPattern();
  memoize_count_++;
}

void shift::core::solver::RNCA::RecordMassPattern() {
  mass_pattern_index_.clear();
  mass_pattern_row_.clear();
  mass_pattern_column_.clear();
  for(unsigned int i = 0; i < dq_count_; i++) {
    const int row_start = i * (i + 1) / 2;
    for(unsigned int j = 0; j <= i; j++) {
      if(Mlbs_[row_start + j]) {
        mass_pattern_index_.push_back(row_start + j);
        mass_pattern_row_.push_back(i);
        mass_pattern_column_.push_back(j);
      }
    }
  }
}

void shift::core::solver::RNCA::CountOperations() {
  assert(context_);
  dq_count_ = context_->get_dq_count();
  auto ddq = context_->get_ddq();
  const auto& jacobian = context_->get_jacobian();
  const auto& rb3 = jacobian.get_rb3();
  const auto& cb3 = jacobian.get_cb3();
  // const auto& b3 = jacobian.b3();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
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

void shift::core::solver::RNCA::RecordOperations() {

  assert(context_);

  const auto& jacobian = context_->get_jacobian();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  const auto& rb3 = jacobian.get_rb3();
  const auto& cb3 = jacobian.get_cb3();

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
            // Pointer to Mik, but Mik may be zero. Only store it when it is
            // processed, i8_ is sized by CountOperations for nonzero Mik only.
            const int pMik = start_of_row_i_in_M + k;
            // If Mik != 0 then process.
            if(Mlb_[pMik]) {
              i8_[l3] = pMik;
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

void shift::core::solver::RNCA::CountPointers() {
  assert(context_);
  const auto& jacobian = context_->get_jacobian();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();

  int ni = 0;
  int nd = 0;

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

      // if(Mlb_[i4_[i]])
      // Part of x <-- D^{-1} b: la[k] -= M[] * ddq[i];
      // context_->la[k] -= *(memoized_pointers_[i]) * context_->ddq[i];
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

void shift::core::solver::RNCA::RecordPointers() {
  assert(context_);
  const auto& jacobian = context_->get_jacobian();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  // const auto& rb3 = jacobian.rb3();
  // const auto& cb3 = jacobian.cb3();
  const auto& b3 = jacobian.get_b3();
  // const auto& pv_23 = jacobian.pv23();
  const auto& pv_13 = jacobian.get_pv13();

  auto& ddq = context_->get_ddq();

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

  /* the substitutions (ForwardBackwardSubstitution) read the entries of L
  from here on */
  factored_pointer_end_ = p2;

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

bool shift::core::solver::RNCA::SetupEquations() {
  assert(context_);
  return SetupEquations(0.0, context_->get_q());
}

bool shift::core::solver::RNCA::SetupEquationsAndRecordFills() {
  assert(context_);
  SetupEquationsAndRecordFills(0.0, context_->get_q());
  return false;
}

bool shift::core::solver::RNCA::SetupEquationsAndRecordFills(
  const double t, const std::vector<double>& y) {
  const double* q = y.data();
  const double* dq = q + q_count_;
  assert(context_);
  auto& jacobian = context_->get_jacobian();
  context_->UpdateDHVA(q, dq);
  context_->EvaluateElasticBodies(context_->get_dhva(), q, dq);
  jacobian.Save();
  jacobian.SetupPO2();
  jacobian.FactorPO2();
  jacobian.SetupBMatrixSparse();
  unsigned int condition_check = 
    jacobian.CheckCondition(
      t, const_cast<double*>(q), dq, infinity_norm_multiplier_,
      refactor_criterion_, epsilon_mobility_);

  if(condition_check) {

    if(condition_check & shift::core::Jacobian::Failure::NEWTON_RAPHSON) {
      SHIFT_ERROR("Newton-raphson failure, exiting simulation.");
      return true;
    }

    jacobian.Analyze();
    SetupRHS();
    SetupMassMatrixAndRecordFills();
    Memoize();

#if SHIFT_LOG_LEVEL > 3
    {
      std::stringstream ss;
      double t;
      double dt;
      context_->get_current_time(t, dt);
      ss << "Failed a refactor check at t = " << t
         << ", but GECP was successfully reapplied and simulation "
         << "will attempt to continue";
      SHIFT_DEBUG(ss.str());
    }
#endif

  } else {
    SetupRHS();
    SetupMassMatrixAndRecordFills();
  }
  return false;
}

bool shift::core::solver::RNCA::SetupEquations(const double t,
                                               const std::vector<double>& y) {
  const double* q = y.data();
  const double* dq = q + q_count_;
  assert(context_);
  auto& jacobian = context_->get_jacobian();
  context_->UpdateDHVA(q, dq);
  context_->EvaluateElasticBodies(context_->get_dhva(), q, dq);
  jacobian.Save();
  jacobian.SetupPO2();
  jacobian.FactorPO2();
  jacobian.SetupBMatrixSparse();
  int failure = jacobian.CheckCondition(
    t, const_cast<double*>(q), dq, infinity_norm_multiplier_,
      refactor_criterion_, epsilon_mobility_);
  
  if(failure) {
    
    if(failure & shift::core::Jacobian::Failure::NEWTON_RAPHSON) {
      SHIFT_ERROR("Newton-raphson failure; exiting simulation.");
      return true;
    }

#if SHIFT_LOG_LEVEL > 3
    {
      std::stringstream ss;
      double t;
      double dt;
      context_->get_current_time(t, dt);
      ss << "Failed ";
      if(failure & shift::core::Jacobian::Failure::INFINITY_NORM_B) {
        ss << "(infinity norm check) ";
      }
      if(failure & shift::core::Jacobian::Failure::PIVOT_RATIO) {
        ss << "(pivot ratio check) ";
      }
      if(failure & shift::core::Jacobian::Failure::MOBILITY_EPSILON) {
        ss << "(mobility epsilon check) ";
      }
      ss << "at t = " << t
         << ", but GECP was successfully reapplied and simulation "
         << "will attempt to continue";
      SHIFT_DEBUG(ss.str());
    }
#endif

    jacobian.Analyze();
    SetupRHS();
    SetupMassMatrixAndRecordFills();
    Memoize();
  } else {
    SetupRHS();
    SetupMassMatrix();
  }
  return false;
}

void shift::core::solver::RNCA::SetupRHS() {

  const topology::DHVA& dhva = context_->get_dhva();
  unsigned int non_rigid_joint_count = context_->get_non_rigid_joint_count();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& pcl_compressed = context_->get_pcl_compressed();
  const auto& has_mass = context_->get_has_mass_array();

  const auto& d_reference_frames = context_->get_d_reference_frames();
  const auto& inertia = context_->get_inertia_vector();

  context_->EvaluateConstraints();

  std::fill(fl_.begin(), fl_.end(), 0);
  std::fill(p_.begin(), p_.end(), ksl_coscrew(0, 0, 0, 0, 0, 0));
  /* external loads on the bodies (e.g. tire forces), set in
  Context::SetForces */
  const auto& body_loads = context_->get_body_loads();
  if(body_loads.size() == g_.size()) {
    std::copy(body_loads.begin(), body_loads.end(), g_.begin());
  } else {
    std::fill(g_.begin(), g_.end(), ksl_coscrew(0, 0, 0, 0, 0, 0));
  }

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
    if(has_mass[i] && !delegated_body_[i]) {
      ksl_inertia_transformed(&inertia[i], &dhva.d[d_reference_frames[i]],
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

  /* the bias wrenches of the flexible bodies, which the gyroscopic wrench
  of their frame-frame blocks above leaves out */
  if(has_elastic_) {
    const auto& bias_b = context_->get_elastic_bias_b();
    for(int i : context_->get_elastic_bodies()) {
      const int j = context_->get_active_articulated_bodies()[i]->get_va_index();
      for(int k = 0; k < 6; k++) {
        g_[j].at[k] += bias_b[j].at[k];
      }
    }
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
    which has length non_rigid_joint_count and is stored in PO1.

    Expand each multi DOF joint to expand the quantity in g with length
    non_rigid_joint_count in PO1 to fl of length dqCount in PO3 */

  assert(context_);
  const auto& f = context_->get_joint_working_forces();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& is_rotation = context_->get_is_rotation();

  const auto& jacobian = context_->get_jacobian();
  const auto& pv_31 = jacobian.get_pv31();

  assert(integrator_);
  const double baumgarte_2_omega_n = integrator_->get_baumgarte_2_omega_n();
  const double baumgarte_omega_n_2 = integrator_->get_baumgarte_omega_n_2();
  const auto& integral_constraint_error = integrator_->get_integral_error();

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
  AddElasticForces();

  const auto& e = jacobian.get_constraints();
  const auto& de = jacobian.get_de();

  /* evaluate primary constraints (\bar{\gamma}_p) in PO(1), store in PO(2) */
  size_t chord_constraint_offset = 0;
  for(auto& substructure : jacobian.get_substructures()) {

    SHIFT_TRACE("Processing substructure: " +
                std::to_string(substructure.get_index()));

    const auto& pr_21 = substructure.get_pr21();
    const int dq_offset = substructure.get_dq_offset();
    const auto& constraint_indices = substructure.get_constraint_indices();

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "dq_offset: " << dq_offset;
      SHIFT_TRACE(ss.str());
    }

    {
      std::stringstream ss;
      ss << "pr_21: ";
      for(auto& i : pr_21) {
        ss << i << " ";
      }
      ss << std::endl;
      SHIFT_TRACE(ss.str());
    }
#endif

    for(size_t k = 0; k < substructure.get_chord_constraint_count();
        k++, chord_constraint_offset++) {

      for(size_t l = 0; l < 6; l++) {

        // \gamma_p(po2) <- \gamma_p(po2) - 2 * \omega_n \dot{e}
        //                  - \omega_n^2 * e - K_I * \int{e}
        if(pr_21[k * 6 + l] < substructure.get_rank()) {
          gamma_d_[dq_offset + pr_21[k * 6 + l]] =

            (dhva.aq[constraint_indices[k].child.v].at[l] -
             dhva.aq[constraint_indices[k].parent.v].at[l])

            /* -(2 * omega_N) * \dot{e} velocity part of baumgarte
            constraint stabilization */
            - baumgarte_2_omega_n * de[chord_constraint_offset * 6 + l]

            /* subtract omega_n^2 * e from gamma_p */
            - baumgarte_omega_n_2 * e[chord_constraint_offset * 6 + l]

            // integral error
            - integral_constraint_error[chord_constraint_offset * 6 + l];

        }
      }
    }
  }

  /* precondition primary constraints in PO(2)
  \bar{\gamma}_d = U^{-1} L^{-1} \bar{\gamma}_p */
  for(auto& substructure : jacobian.get_substructures()) {

    SHIFT_TRACE("substructure: " + std::to_string(substructure.get_index()) +
                " dq_offset:" + std::to_string(substructure.get_dq_offset()));

    int dq_offset = substructure.get_dq_offset();
    const int dq_count = substructure.get_dq_count();
    const double* J = substructure.get_jacobian_matrix();
    const int rank = substructure.get_rank();

    /* forward elimination:
    \bar{\gamma} <- L^{-1} * \bar\{\gamma} */
    for(int i = 0; i < rank; i++) {
      for(int j = 0; j < i; j++) {
        gamma_d_[i + dq_offset] -=
          J[i * dq_count + j] * gamma_d_[j + dq_offset];
      }
    }

    /* back substitution
    \bar{\gamma} <- U^{-1} * \bar\{\gamma) */
    for(int i = rank - 1; i > -1; i--) {
      double save = 0;
      for(int j = rank - 1; j > i; j--) {
        save += J[i * dq_count + j] * gamma_d_[j + dq_offset];
      }
      gamma_d_[i + dq_offset] =
        (gamma_d_[i + dq_offset] - save) / J[i * dq_count + i];
    }
  }

#if SHIFT_LOG_LEVEL > 4
  std::stringstream ss;
  ss << "gamma_d_: ";
  for(auto& val : gamma_d_) {
    ss << val << " ";
  }
  SHIFT_TRACE(ss.str());
#endif
}

/*! set mass matrix without setting boolean mass matrix */
void shift::core::solver::RNCA::SetupMassMatrix() {
  assert(context_);
  const auto& dhva = context_->get_dhva();
  const size_t non_rigid_joint_count = context_->get_non_rigid_joint_count();
  const auto& pcl_compressed = context_->get_pcl_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& is_rotation = context_->get_is_rotation();
  const auto& has_mass_dq = context_->get_has_mass_dq();
  const auto& pcl_dq = context_->get_pcldq();

  const auto& jacobian = context_->get_jacobian();
  const auto& pv_31 = jacobian.get_pv31();

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

  SetElasticMomenta();

  /* use inverse permutation to save terms computed in PO1 order and
  store in PO3 */
  int m_3;
  int ks_3;
  int k_3;
  for(int j = (int)dq_count_ - 1; j > -1; j--) {
    if(has_mass_dq[j] && !delegated_dq_[j]) {
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
  AddElasticMassTerms(false);
}

void shift::core::solver::RNCA::SetupMassMatrixAndRecordFills() {
  assert(context_);
  const auto& dhva = context_->get_dhva();
  const size_t non_rigid_joint_count = context_->get_non_rigid_joint_count();
  const auto& pcl_compressed = context_->get_pcl_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& is_rotation = context_->get_is_rotation();
  const auto& has_mass_dq = context_->get_has_mass_dq();
  const auto& pcl_dq = context_->get_pcldq();

  const auto& jacobian = context_->get_jacobian();
  const auto& pv_31 = jacobian.get_pv31();

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

  SetElasticMomenta();

  /* use inverse permutation to save terms computed in PO1 order and
  store in PO3 */
  int m_3;
  int ks_3;
  int k_3;
  for(int j = (int)dq_count_ - 1; j > -1; j--) {
    if(has_mass_dq[j] && !delegated_dq_[j]) {
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
  AddElasticMassTerms(true);
}

void shift::core::solver::RNCA::Factor() {

  assert(context_);
  const auto& is_dependent_po3 =
    context_->get_jacobian().get_is_dependent_po3();

  /* Access entries in P_[] to factor matrix and access L */
  int p2 = dq_count_;

  int ni = 0;
  int nd = 0;

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
    } else if(delegated_po3_[i]) {
      /* a delegated variable has an empty row and column: no operations */
      ni++;
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
}

void shift::core::solver::RNCA::ForwardBackwardSubstitution() {

  assert(context_);
  auto& ddq = context_->get_ddq();

  shift::core::Jacobian& jacobian = context_->get_jacobian();
  const auto& pv_23 = jacobian.get_pv23();
  const auto& pv_13 = jacobian.get_pv13();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();

  /* Access entries in P_[] to access L, after the factorization's */
  int p2 = factored_pointer_end_;

  /* Access entries in P_[] to process rhs and lhs */
  int p3 = p3_start_;

  int ni = 0;
  int nd = 0;

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
      if(!delegated_po3_[i]) {
        ddq[pv_13[i]] = *(memoized_pointers_[i]) * fl_[i];
      }
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
bool shift::core::solver::RNCA::Solve(const double t,
                                      const std::vector<double>& y,
                                      std::vector<double>& dy) {

  assert(context_);
  assert(q_count_ == context_->get_q_count());

  const double* q = y.data();
  const double* dq = q + q_count_;

  context_->UpdateDHVA(q, dq);
  context_->EvaluateConstraints();
  context_->SetForces(t, y, dy);

  if(SetupEquations(t, y)) {
    return true;
  }

#ifdef SHIFT_VERIFY_EQUATIONS_
  /* make a copy of Ml and fl to verify equations of motion, remove this
  once equations of motion verified */
  /* heap allocated, a dense dq_count^2 array overflows the stack for large
  models (e.g. 500 free bodies) */
  std::vector<double> Ml(dq_count_ * dq_count_);
  for(unsigned int i = 0; i < dq_count_; i++) {
    int istart = i * (i + 1) / 2;
    for(unsigned int j = 0; j < i + 1; j++) {
      Ml[i * dq_count_ + j] = Ml_[istart + j];
      if(i != j) {
        Ml[j * dq_count_ + i] = Ml_[istart + j];
      }
    }
  }
#endif

#ifdef SHIFT_VERIFY_EQUATIONS_
  /* copy fl to a second array to verify solution of
  equations of motion */
  double fl[dq_count_];
  memcpy(fl, fl_.data(), dq_count_ * sizeof(double));

#if SHIFT_LOG_LEVEL > 4
  {
    std::stringstream ss;
    ss << "fl (before factor): ";
    for(unsigned int i = 0; i < dq_count_; i++) {
      ss << fl_[i] << " ";
    }
    SHIFT_TRACE(ss.str());
  }
#endif
#endif

  Factor();
  ForwardBackwardSubstitution();
  if(SolveDelegated()) {
    return true;
  }

#if SHIFT_VERIFY_EQUATIONS_
#if SHIFT_LOG_LEVEL > 4
  {
    std::stringstream ss;
    ss << "fl (after factor): ";
    for(unsigned int i = 0; i < dq_count_; i++) {
      ss << fl_[i] << " ";
    }
    SHIFT_TRACE(ss.str());
  }
#endif

  /* verify that multiplying coefficient matrix with lhs gets back the
  right hand side */
  // std::cout << "(" << t << ") ";
  // std::cout << " y: ";
  // for(unsigned int i = 0; i < q_count_ + dq_count_; i++) {
  //   std::cout << y[i] << " ";
  // }
  // std::cout << std::endl;
  Verify(t, y, Ml.data(), fl);
#endif

  /* convert angular velocities to time derivative of quaternions
    copy by value all other dq terms to dy array */
  context_->ConvertdqTody(y, dy);

#if SHIFT_LOG_LEVEL > 4
  {
    std::stringstream ss;
    ss << "RNCA States\n";
    PrintStates(ss);
    PrintVector("y", y, ss);
    PrintVector("dy", dy, ss);
    SHIFT_TRACE(ss.str());
  }
#endif

#if SHIFT_VERIFY_EQUATIONS_
#if SHIFT_LOG_LEVEL > 4
  {
    std::stringstream ss;
    ss << "y: ";
    for(unsigned int i = 0; i < dq_count_ + dq_count_; i++) {
      ss << y[i] << " ";
    }
    SHIFT_TRACE(ss.str());
  }

  {
    std::stringstream ss;
    ss << "dy: ";
    for(unsigned int i = 0; i < dq_count_ + dq_count_; i++) {
      ss << dy[i] << " ";
    }
    SHIFT_TRACE(ss.str());
  }
#endif
#endif

  return false;
}

bool shift::core::solver::RNCA::Verify(const double t,
                                       const std::vector<double>& /*dy*/,
                                       double Ml[], double fl[]) {
  bool verified = true;
  (void)t;
  (void)Ml;
  (void)fl;

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
  //     context->non_rigid_joint_count *
  //     context->non_rigid_joint_count,
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
  auto& jacobian = context_->get_jacobian();
  const auto& pv_13 = jacobian.get_pv13();
  const auto& pv_23 = jacobian.get_pv23();

  const auto& ddq = context_->get_ddq();

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

  const auto& rb3 = jacobian.get_rb3();
  const auto& cb3 = jacobian.get_cb3();
  const auto& b3 = jacobian.get_b3();

  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();

  std::vector<double> B_dense(dq_count_ * dq_count_, 0.0);
  double* B = B_dense.data();
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {

        int col = cb3[j];
        B[i * dq_count_ + col] = b3[j];
        B[i * dq_count_ + i] = 1.0;
      }
    }
  }

  cblas_dgemv(CblasRowMajor, CblasTrans, dq_count_, dq_count_, 1.0, B,
              dq_count_, la_po3, 1, 1.0, fl_po3, 1);

  /* the rows of delegated variables are empty; another solver checks them */
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(delegated_po3_[i]) {
      fl_po3[i] = fl[i];
    }
  }

  /* per-entry scale: sum_j |Ml_ij * ddq_j| + sum_j |B_ji * la_j| */
  double fl_scale[dq_count_];
  for(unsigned int i = 0; i < dq_count_; i++) {
    fl_scale[i] = 0.0;
    for(unsigned int j = 0; j < dq_count_; j++) {
      fl_scale[i] += fabs(Ml[i * dq_count_ + j] * ddq_po3[j]) +
                     fabs(B[j * dq_count_ + i] * la_po3[j]);
    }
  }

  if(!VerificationClose(dq_count_, fl_po3, fl, fl_scale)) {

    SHIFT_ERROR("Error, equations of motion not solved correctly at time t = " +
        std::to_string(t));

    verified = false;

#if SHIFT_LOG_LEVEL > 0
    {
      std::stringstream ss;
      ss << "Verification failed for fl array:\n";
      ss << "  fl (original): ";
      for(unsigned int i = 0; i < dq_count_; i++) {
        ss << std::setprecision(5) << fl[i] << " ";
      }
      ss << "\n  != fl (recovered): ";
      for(unsigned int i = 0; i < dq_count_; i++) {
        ss << std::setprecision(5) << fl_po3[i] << " ";
      }
      SHIFT_ERROR(ss.str());
    }
#endif

    auto& jacobian = context_->get_jacobian();

#if SHIFT_LOG_LEVEL > 0
    std::stringstream ss;
    ss << "infinity norm failures:\n";
    for(auto& infinity_norm_failure : jacobian.get_infinity_norm_failures()) {
      ss << "t: " << infinity_norm_failure.time << ", substructure indices: ";
      for(auto& index : infinity_norm_failure.substructure_index) {
        ss << index << " ";
      }
      ss << "\n";
    }
    SHIFT_ERROR(ss.str());
#endif

    throw shift::core::Exception("Failed to solve equations of motion");
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

  /* per-entry scale: sum_j |B_ij * ddq_j|, permuted to po2 */
  double gamma_d_scale[dq_count_];
  for(unsigned int i = 0; i < dq_count_; i++) {
    double sum = 0.0;
    for(unsigned int j = 0; j < dq_count_; j++) {
      sum += fabs(B[i * dq_count_ + j] * ddq_po3[j]);
    }
    gamma_d_scale[pv_23[i]] = sum;
  }

  if(!VerificationClose(dq_count_, gamma_d_po2, gamma_d_.data(),
                        gamma_d_scale)) {

    std::cerr << ">>> Error, equations of motion not solved correctly."
              << std::endl;

    verified = false;

#if SHIFT_LOG_LEVEL > 0
    {
      std::stringstream ss;
      ss << "Verification failed for gamma_d array:\n";
      ss << "  gamma_d (original): ";
      for(unsigned int i = 0; i < dq_count_; i++) {
        ss << std::setprecision(5) << gamma_d_[i] << " ";
      }
      ss << "\n  != gamma_d (recovered): ";
      for(unsigned int i = 0; i < dq_count_; i++) {
        ss << std::setprecision(5) << gamma_d_po2[i] << " ";
      }
      SHIFT_ERROR(ss.str());
    }
#endif

    throw shift::core::Exception("Failed to solve equations of motion");
  }
  // std::cout << "verified: " << (verified ? "true" : "false") << std::endl;
#endif

  return verified;
}

void shift::core::solver::RNCA::SetupElasticLookup() {
  const auto& elastic_bodies = context_->get_elastic_bodies();
  const auto& bodies = context_->get_active_articulated_bodies();
  const auto& slots = context_->get_node_slots();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& elastic_count_compressed =
    context_->get_elastic_count_compressed();
  const size_t entry_count = context_->get_non_rigid_joint_count();

  has_elastic_ = !elastic_bodies.empty();
  elastic_body_of_dq_.assign(dq_count_, -1);
  elastic_mode_of_dq_.assign(dq_count_, -1);
  entry_of_dq_.assign(dq_count_, -1);
  slot_of_entry_.assign(entry_count, -1);
  slots_of_body_.assign(elastic_bodies.size(), {});
  mhe_.clear();
  if(!has_elastic_) {
    return;
  }

  for(size_t k = 0; k < slots.size(); k++) {
    slot_of_entry_[slots[k].index] = (int)k;
  }
  for(size_t j = 0; j < entry_count; j++) {
    if(slot_of_entry_[j] > -1) {
      continue;
    }
    const unsigned int count =
      joint_dof_compressed[j] + elastic_count_compressed[j];
    for(unsigned int k = 0; k < count; k++) {
      entry_of_dq_[dq_lookup_compressed[j] + k] = (int)j;
    }
  }
  for(size_t e = 0; e < elastic_bodies.size(); e++) {
    const auto body = bodies[elastic_bodies[e]];
    const int dq_index = body->get_elastic_dq_index();
    for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
      elastic_body_of_dq_[dq_index + m] = (int)e;
      elastic_mode_of_dq_[dq_index + m] = (int)m;
    }
    for(size_t k = 0; k < slots.size(); k++) {
      if(slots[k].body == body) {
        slots_of_body_[e].push_back((int)k);
      }
    }
  }
  mhe_.assign(context_->get_dhva().he.size(),
              ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

int shift::core::solver::RNCA::MassIndex(int a, int b) const {
  const auto& pv_31 = context_->get_jacobian().get_pv31();
  const int a_3 = pv_31[a];
  const int b_3 = pv_31[b];
  const int high = a_3 > b_3 ? a_3 : b_3;
  const int low = a_3 > b_3 ? b_3 : a_3;
  return (high * (high + 1)) / 2 + low;
}

int shift::core::solver::RNCA::EnteredNodeSlot(int c, int i) const {
  const int body = elastic_body_of_dq_[i];
  if(body < 0 || elastic_body_of_dq_[c] == body) {
    return -1;
  }
  /* c is the first velocity of its compressed joint (or the first elastic
  coordinate of a flexible body on a rigid joint), whose parent in the
  compressed tree is the node slot it hangs from */
  const int parent = context_->get_pcl_compressed()[entry_of_dq_[c]];
  return parent > -1 ? slot_of_entry_[parent] : -1;
}

void shift::core::solver::RNCA::SetElasticMomenta() {
  if(!has_elastic_) {
    return;
  }
  const auto& dhva = context_->get_dhva();
  const auto& slots = context_->get_node_slots();
  const auto& mass_be = context_->get_elastic_mass_be();
  const auto& elastic_bodies = context_->get_elastic_bodies();
  const auto& bodies = context_->get_active_articulated_bodies();
  const ksl_inertia_t* inertia_p = inertia_p_.data() + 1;

  /* M^c_k h_km, with the inertia accumulated below each node slot */
  for(const auto& slot : slots) {
    for(unsigned int m = 0; m < slot.body->get_elastic_count(); m++) {
      ksl_product_is(&inertia_p[slot.index], &dhva.he[slot.he_offset + m],
                     &mhe_[slot.he_offset + m]);
    }
  }
  /* the momentum of each elastic column: M_be,m + sum_k M^c_k h_km */
  for(size_t e = 0; e < elastic_bodies.size(); e++) {
    const auto body = bodies[elastic_bodies[e]];
    const int dq_index = body->get_elastic_dq_index();
    for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
      ksl_coscrew_t& mh = mh_product_[dq_index + m];
      mh = mass_be[dq_index + m];
      for(int k : slots_of_body_[e]) {
        const ksl_coscrew_t& c = mhe_[slots[k].he_offset + m];
        for(int r = 0; r < 6; r++) {
          mh.at[r] += c.at[r];
        }
      }
    }
  }
}

void shift::core::solver::RNCA::AddElasticMassTerms(bool record_fills) {
  if(!has_elastic_) {
    return;
  }
  const auto& dhva = context_->get_dhva();
  const auto& slots = context_->get_node_slots();
  const auto& elastic_bodies = context_->get_elastic_bodies();
  const auto& bodies = context_->get_active_articulated_bodies();
  const auto& has_mass_dq = context_->get_has_mass_dq();
  const auto& pcl_dq = context_->get_pcldq();

  auto add = [&](int a, int b, double value) {
    const int k = MassIndex(a, b);
    Ml_[k] += value;
    if(record_fills) {
      Mlb_[k] = true;
    }
  };

  for(int j = (int)dq_count_ - 1; j > -1; j--) {
    if(!has_mass_dq[j] || delegated_dq_[j]) {
      continue;
    }
    int c = j;
    int i = pcl_dq[j];
    const int e = elastic_body_of_dq_[j];
    if(e > -1) {
      /* the block of the body's own elastic coordinates, M_ee + sum_k
      h_k^T M^c_k h_k: the diagonal, then up the chain of its coordinates */
      const auto body = bodies[elastic_bodies[e]];
      const int n = (int)body->get_elastic_count();
      const double* mass_ee = body->get_mass_ee();
      const int m = elastic_mode_of_dq_[j];
      auto block = [&](int l) {
        double value = mass_ee[l * n + m];
        for(int k : slots_of_body_[e]) {
          value += ksl_dot_cs(&mhe_[slots[k].he_offset + m],
                              &dhva.he[slots[k].he_offset + l]);
        }
        return value;
      };
      add(j, j, block(m));
      while(i > -1 && elastic_body_of_dq_[i] == e) {
        add(i, j, block(elastic_mode_of_dq_[i]));
        c = i;
        i = pcl_dq[i];
      }
    }
    /* the elastic coordinates of the bodies above, each through the node
    slot the walk entered it by */
    int slot = -1;
    while(i > -1) {
      if(elastic_body_of_dq_[i] > -1) {
        if(elastic_body_of_dq_[c] != elastic_body_of_dq_[i]) {
          slot = EnteredNodeSlot(c, i);
        }
        if(slot > -1) {
          add(i, j,
              ksl_dot_cs(&mh_product_[j],
                         &dhva.he[slots[slot].he_offset +
                                  elastic_mode_of_dq_[i]]));
        }
      }
      c = i;
      i = pcl_dq[i];
    }
  }
}

void shift::core::solver::RNCA::AddElasticForces() {
  if(!has_elastic_) {
    return;
  }
  const auto& dhva = context_->get_dhva();
  const auto& slots = context_->get_node_slots();
  const auto& elastic_bodies = context_->get_elastic_bodies();
  const auto& bodies = context_->get_active_articulated_bodies();
  const auto& mass_be = context_->get_elastic_mass_be();
  const auto& bias_e = context_->get_elastic_bias_e();
  const auto& f = context_->get_joint_working_forces();
  const auto& pv_31 = context_->get_jacobian().get_pv31();

  for(size_t e = 0; e < elastic_bodies.size(); e++) {
    const auto body = bodies[elastic_bodies[e]];
    const int j = body->get_va_index();
    const int dq_index = body->get_elastic_dq_index();
    for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
      const int l = dq_index + (int)m;
      if(delegated_dq_[l]) {
        continue; /* solved by another solver */
      }
      /* b_e - M_be^T aq_b + sum_k h_km^T g_k */
      double value = f[l] + bias_e[l] - ksl_dot_cs(&mass_be[l], &dhva.aq[j]);
      for(int k : slots_of_body_[e]) {
        value += ksl_dot_cs(&g_[slots[k].index], &dhva.he[slots[k].he_offset + m]);
      }
      fl_[pv_31[l]] = value;
    }
  }
}

void shift::core::solver::RNCA::Multipliers(std::vector<double>& la) const {
  assert(context_);
  const auto& jacobian = context_->get_jacobian();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  const auto& pv_23 = jacobian.get_pv23();
  la.clear();
  la.reserve(dependent_count_);
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      la.push_back(la_[pv_23[i]]);
    }
  }
}

bool shift::core::solver::RNCA::Residual(const double t,
                                         const std::vector<double>& y,
                                         const std::vector<double>& ddq,
                                         const std::vector<double>& la,
                                         std::vector<double>& r,
                                         std::vector<double>& dy) {
  assert(context_);
  assert(ddq.size() == dq_count_);
  assert(la.size() == dependent_count_);

  const double* q = y.data();
  const double* dq = q + q_count_;

  context_->UpdateDHVA(q, dq);
  context_->EvaluateConstraints();
  context_->SetForces(t, y, dy);
  /* the mass matrix and the bias forces of the flexible bodies at y */
  context_->EvaluateElasticBodies(context_->get_dhva(), q, dq);

  /* the equations at y as SetupEquations sets them up, without its
  condition check, which could refactor the Jacobian and change the
  partition */
  auto& jacobian = context_->get_jacobian();
  jacobian.Save();
  jacobian.SetupPO2();
  jacobian.FactorPO2();
  jacobian.SetupBMatrixSparse();
  SetupRHS();
  SetupMassMatrix();

  const auto& pv_13 = jacobian.get_pv13();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  const auto& rb3 = jacobian.get_rb3();
  const auto& cb3 = jacobian.get_cb3();
  const auto& b3 = jacobian.get_b3();

  /* M ddq - f in permutation order 3, over the entries of the pattern of M
  recorded by Memoize (the lower triangle of Ml_) */
  std::vector<double> ddq_po3(dq_count_);
  for(unsigned int i = 0; i < dq_count_; i++) {
    ddq_po3[i] = ddq[pv_13[i]];
  }
  std::vector<double> r_po3(dq_count_);
  for(unsigned int i = 0; i < dq_count_; i++) {
    r_po3[i] = -fl_[i];
  }
  for(size_t l = 0; l < mass_pattern_index_.size(); l++) {
    const int i = mass_pattern_row_[l];
    const int j = mass_pattern_column_[l];
    const double m_ij = Ml_[mass_pattern_index_[l]];
    r_po3[i] += m_ij * ddq_po3[j];
    if(i != j) {
      r_po3[j] += m_ij * ddq_po3[i];
    }
  }

  /* + B^T la: each dependent row of B is a 1 at its own variable and the
  entries of b3 at the independent variables */
  unsigned int m = 0;
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      const double la_m = la[m++];
      r_po3[i] += la_m;
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        r_po3[cb3[j]] += b3[j] * la_m;
      }
    }
  }

  /* the rows of variables delegated to another solver are zero, for it to
  fill (solver::Auto) */
  r.assign(dq_count_ + dependent_count_, 0.0);
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(!delegated_po3_[i]) {
      r[pv_13[i]] = r_po3[i];
    }
  }

  /* the velocity constraints, [B_di I] dq, each dependent row of B a 1 at
  its own variable and the entries of b3 at the independent variables */
  if(velocity_constraints_) {
    m = 0;
    for(unsigned int i = 0; i < dq_count_; i++) {
      if(is_dependent_po3[i]) {
        double rate = dq[pv_13[i]];
        for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
          rate += b3[j] * dq[pv_13[cb3[j]]];
        }
        r[dq_count_ + m++] = rate;
      }
    }
    return false;
  }

  /* the position constraints, as the Newton-Raphson corrections of the
  dependent coordinates */
  jacobian.EvaluateNewtonRaphsonCorrections();
  const auto& delta_q = context_->get_delta_q();
  m = 0;
  for(unsigned int i = 0; i < dq_count_; i++) {
    if(is_dependent_po3[i]) {
      r[dq_count_ + m++] = delta_q[pv_13[i]];
    }
  }

  /* the constraints as Solve leaves them */
  context_->EvaluateConstraints();
  return false;
}

void shift::core::solver::RNCA::AddIterationEntry(int a, int b,
                                                  double value) {
  if(value == 0.0) {
    return;
  }
  const auto& pv_31 = context_->get_jacobian().get_pv31();
  iteration_row_.push_back(pv_31[a]);
  iteration_column_.push_back(pv_31[b]);
  iteration_value_.push_back(value);
}

int shift::core::solver::RNCA::IterationSlot(int r, int c) const {
  if(r == c) {
    return r;
  }
  const auto found =
    iteration_slot_.find(static_cast<int64_t>(r) * dq_count_ + c);
  return found == iteration_slot_.end() ? -1 : found->second;
}

void shift::core::solver::RNCA::RecordIterationPattern() {
  const int n = static_cast<int>(dq_count_);
  const auto& jacobian = context_->get_jacobian();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  const auto& rb3 = jacobian.get_rb3();
  const std::vector<int> cb3 = jacobian.get_cb3();

  /* the pattern below the diagonal, row by row: the mass matrix, the
  entries of A since the last partition change, then the fill-in of the
  elimination from the last variable to the first */
  std::vector<std::set<int>> lower(n);
  for(size_t l = 0; l < mass_pattern_index_.size(); l++) {
    if(mass_pattern_row_[l] != mass_pattern_column_[l]) {
      lower[mass_pattern_row_[l]].insert(mass_pattern_column_[l]);
    }
  }
  for(const int64_t key : iteration_pattern_entries_) {
    lower[key / n].insert(static_cast<int>(key % n));
  }
  auto connect = [&](int r, int c) {
    if(r != c) {
      lower[std::max(r, c)].insert(std::min(r, c));
    }
  };
  for(int i = n - 1; i > -1; i--) {
    if(is_dependent_po3[i]) {
      /* the rows and columns of the variables of row i of B take the
      entries of row and column i */
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        assert(cb3[j] < i);
        lower[i].insert(cb3[j]);
      }
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        for(int c : lower[i]) {
          connect(cb3[j], c);
        }
      }
    } else {
      for(int a : lower[i]) {
        for(int b : lower[i]) {
          if(b < a) {
            lower[a].insert(b);
          }
        }
      }
    }
  }

  /* the value indices: the diagonal first, then (i, k) and (k, i) */
  iteration_slot_.clear();
  iteration_neighbor_start_.assign(n + 1, 0);
  iteration_neighbor_.clear();
  iteration_lower_slot_.clear();
  iteration_upper_slot_.clear();
  int slot_count = n;
  for(int i = 0; i < n; i++) {
    iteration_neighbor_start_[i] = static_cast<int>(iteration_neighbor_.size());
    for(int k : lower[i]) {
      iteration_neighbor_.push_back(k);
      iteration_upper_slot_.push_back(slot_count);
      iteration_slot_[static_cast<int64_t>(i) * n + k] = slot_count++;
      iteration_lower_slot_.push_back(slot_count);
      iteration_slot_[static_cast<int64_t>(k) * n + i] = slot_count++;
    }
  }
  iteration_neighbor_start_[n] = static_cast<int>(iteration_neighbor_.size());
  iteration_slot_count_ = slot_count;

  iteration_mass_slot_.resize(mass_pattern_index_.size());
  iteration_mass_transpose_slot_.resize(mass_pattern_index_.size());
  for(size_t l = 0; l < mass_pattern_index_.size(); l++) {
    const int r = mass_pattern_row_[l];
    const int c = mass_pattern_column_[l];
    iteration_mass_slot_[l] = IterationSlot(r, c);
    iteration_mass_transpose_slot_[l] = r == c ? -1 : IterationSlot(c, r);
  }

  /* the operations of each elimination; B follows the entries */
  const int b_offset = slot_count;
  iteration_operation_.clear();
  iteration_operation_start_.assign(n, 0);
  iteration_operation_end_.assign(n, 0);
  auto push = [&](int x, int y, int z) {
    assert(x > -1 && y > -1 && z > -1);
    iteration_operation_.push_back(x);
    iteration_operation_.push_back(y);
    iteration_operation_.push_back(z);
  };
  for(int i = n - 1; i > -1; i--) {
    iteration_operation_start_[i] = static_cast<int>(iteration_operation_.size());
    if(is_dependent_po3[i]) {
      /* the rows of the variables of row i of B: A_rc -= B_ir A_ic, the
      entries of row i (with c = i) */
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        const int r = cb3[j];
        push(IterationSlot(r, i), b_offset + j, IterationSlot(i, i));
        for(int c : lower[i]) {
          push(IterationSlot(r, c), b_offset + j, IterationSlot(i, c));
        }
      }
      /* then their columns: A_rc -= A_ri B_ic */
      for(int r : lower[i]) {
        for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
          push(IterationSlot(r, cb3[j]), IterationSlot(r, i), b_offset + j);
        }
      }
    } else {
      /* A_ab -= L_ai U_ib, after the column of L is scaled */
      for(int a : lower[i]) {
        for(int b : lower[i]) {
          push(IterationSlot(a, b), IterationSlot(a, i), IterationSlot(i, b));
        }
      }
    }
    iteration_operation_end_[i] = static_cast<int>(iteration_operation_.size());
  }

  iteration_b_count_ = jacobian.get_b3().size();
  iteration_pattern_memoize_count_ = memoize_count_;
  has_iteration_pattern_ = true;
  SHIFT_DEBUG("RNCA: iteration matrix pattern of " +
              std::to_string(slot_count) + " entries, " +
              std::to_string(iteration_operation_.size() / 3) + " operations");
}

bool shift::core::solver::RNCA::FactorIterationEntries() {
  const int n = static_cast<int>(dq_count_);
  const auto& jacobian = context_->get_jacobian();
  const auto& b3 = jacobian.get_b3();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();

  /* the pattern is recorded again for a new partition, or for an entry
  outside it */
  bool record = !has_iteration_pattern_ ||
                iteration_pattern_memoize_count_ != memoize_count_ ||
                b3.size() != iteration_b_count_;
  if(iteration_pattern_memoize_count_ != memoize_count_) {
    iteration_pattern_entries_.clear();
  }
  for(size_t e = 0; e < iteration_row_.size(); e++) {
    const int r = iteration_row_[e];
    const int c = iteration_column_[e];
    if(r == c) {
      continue;
    }
    const int64_t key = static_cast<int64_t>(std::max(r, c)) * n +
                        std::min(r, c);
    if(iteration_pattern_entries_.insert(key).second && !record &&
       IterationSlot(r, c) < 0) {
      record = true;
    }
  }
  if(record) {
    RecordIterationPattern();
  }

  /* M + A, then B */
  const int b_offset = iteration_slot_count_;
  iteration_factors_.assign(b_offset + b3.size(), 0.0);
  double* value = iteration_factors_.data();
  for(size_t l = 0; l < mass_pattern_index_.size(); l++) {
    const double m = Ml_[mass_pattern_index_[l]];
    value[iteration_mass_slot_[l]] += m;
    if(iteration_mass_transpose_slot_[l] > -1) {
      value[iteration_mass_transpose_slot_[l]] += m;
    }
  }
  for(size_t e = 0; e < iteration_row_.size(); e++) {
    value[IterationSlot(iteration_row_[e], iteration_column_[e])] +=
      iteration_value_[e];
  }
  std::copy(b3.begin(), b3.end(), value + b_offset);

  /* the elimination from the last variable to the first */
  const int* operation = iteration_operation_.data();
  for(int i = n - 1; i > -1; i--) {
    if(!is_dependent_po3[i]) {
      const double pivot = value[i];
      if(pivot == 0.0 || !std::isfinite(pivot)) {
        SHIFT_DEBUG("RNCA: pivot " + std::to_string(i) +
                    " of the iteration matrix is " + std::to_string(pivot));
        return true;
      }
      value[i] = 1.0 / pivot;
      for(int m = iteration_neighbor_start_[i];
          m < iteration_neighbor_start_[i + 1]; m++) {
        value[iteration_lower_slot_[m]] *= value[i];
      }
    }
    for(int o = iteration_operation_start_[i]; o < iteration_operation_end_[i];
        o += 3) {
      value[operation[o]] -= value[operation[o + 1]] * value[operation[o + 2]];
    }
  }
  return false;
}

bool shift::core::solver::RNCA::FactorIterationMatrix(
  const shift::core::Tangents& tangents, double c, double k,
  const std::vector<int>& differenced, const std::vector<double>& k_columns,
  const std::vector<double>& c_columns) {
  assert(context_);
  has_iteration_matrix_ = false;

  iteration_row_.clear();
  iteration_column_.clear();
  iteration_value_.clear();

  /* a column found by differences holds all its terms, and takes the place
  of the tangents in it; the rows and columns of variables delegated to
  another solver are left out, with a 1 on the diagonal (solver::Auto) */
  std::vector<bool> is_differenced(dq_count_, false);
  for(const int l : differenced) {
    is_differenced[l] = true;
  }
  auto is_delegated = [&](int l) {
    return l < static_cast<int>(delegated_dq_.size()) && delegated_dq_[l];
  };
  for(unsigned int l = 0; l < dq_count_; l++) {
    if(is_delegated(l)) {
      AddIterationEntry(l, l, 1.0);
    }
  }

  /* the tangents, between velocities and between bodies (as SetupRHS
  projects the body loads, Context::GetLoadPath) */
  ForEachTangentEntry(
    *context_, tangents, c, k, [&](int row, int column, double value) {
      if(row < 0 || column < 0 || is_differenced[column] ||
         is_delegated(row) || is_delegated(column)) {
        return;
      }
      AddIterationEntry(row, column, value);
    });

  /* the columns found by finite differences */
  const size_t differenced_count = differenced.size();
  assert(k_columns.empty() ||
         k_columns.size() == dq_count_ * differenced_count);
  assert(c_columns.empty() ||
         c_columns.size() == dq_count_ * differenced_count);
  for(size_t n = 0; n < differenced_count; n++) {
    for(unsigned int i = 0; i < dq_count_; i++) {
      double value = 0.0;
      if(!k_columns.empty()) {
        value += k * k_columns[i * differenced_count + n];
      }
      if(!c_columns.empty()) {
        value += c * c_columns[i * differenced_count + n];
      }
      if(is_delegated(static_cast<int>(i)) || is_delegated(differenced[n])) {
        continue;
      }
      AddIterationEntry(i, differenced[n], value);
    }
  }

  if(FactorIterationEntries()) {
    return true;
  }

  iteration_memoize_count_ = memoize_count_;
  has_iteration_matrix_ = true;
  return false;
}

bool shift::core::solver::RNCA::SolveIterationMatrix(
  std::vector<double>& rhs) {
  assert(context_);
  if(!has_iteration_matrix_ || iteration_memoize_count_ != memoize_count_) {
    return true;
  }
  assert(rhs.size() == dq_count_ + dependent_count_);

  const int n = static_cast<int>(dq_count_);
  const auto& jacobian = context_->get_jacobian();
  const auto& pv_13 = jacobian.get_pv13();
  const auto& is_dependent_po3 = jacobian.get_is_dependent_po3();
  const auto& rb3 = jacobian.get_rb3();
  const std::vector<int> cb3 = jacobian.get_cb3();
  const double* value = iteration_factors_.data();
  const double* b = value + iteration_slot_count_;

  /* the right-hand sides in permutation order 3: f of the rows of the
  velocities, g of the constraint rows of the dependent variables */
  std::vector<double> f(n);
  std::vector<double> g(n, 0.0);
  unsigned int m = 0;
  for(int i = 0; i < n; i++) {
    f[i] = rhs[pv_13[i]];
    if(is_dependent_po3[i]) {
      g[i] = rhs[dq_count_ + m++];
    }
  }

  /* the operations of the factorization on the right-hand sides, from the
  last variable to the first */
  for(int i = n - 1; i > -1; i--) {
    const int start = iteration_neighbor_start_[i];
    const int end = iteration_neighbor_start_[i + 1];
    if(is_dependent_po3[i]) {
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        f[cb3[j]] -= b[j] * f[i];
      }
      for(int k = start; k < end; k++) {
        f[iteration_neighbor_[k]] -= value[iteration_lower_slot_[k]] * g[i];
      }
    } else {
      for(int k = start; k < end; k++) {
        f[iteration_neighbor_[k]] -= value[iteration_lower_slot_[k]] * f[i];
      }
    }
  }

  /* the back substitution from the first variable to the last: a dependent
  variable from its constraint row, its multiplier from its own row */
  std::vector<double> x(n);
  std::vector<double> la(n, 0.0);
  for(int i = 0; i < n; i++) {
    const int start = iteration_neighbor_start_[i];
    const int end = iteration_neighbor_start_[i + 1];
    if(is_dependent_po3[i]) {
      double x_i = g[i];
      for(int j = rb3[i][0]; j < rb3[i][1]; j++) {
        x_i -= b[j] * x[cb3[j]];
      }
      x[i] = x_i;
      double la_i = f[i] - value[i] * x_i;
      for(int k = start; k < end; k++) {
        la_i -= value[iteration_upper_slot_[k]] * x[iteration_neighbor_[k]];
      }
      la[i] = la_i;
    } else {
      double v = f[i];
      for(int k = start; k < end; k++) {
        v -= value[iteration_upper_slot_[k]] * x[iteration_neighbor_[k]];
      }
      x[i] = value[i] * v;
    }
  }

  m = 0;
  for(int i = 0; i < n; i++) {
    rhs[pv_13[i]] = x[i];
    if(is_dependent_po3[i]) {
      rhs[dq_count_ + m++] = la[i];
    }
  }
  return false;
}
