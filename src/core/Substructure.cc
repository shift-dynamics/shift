#include <cfloat>
#include <cmath>
#include <stdio.h>

#ifdef SHIFT_WITH_BLAS_
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#endif

#include "shift/core/Context.h"
#include "shift/core/Exception.h"
#include "shift/core/Substructure.h"
#include "shift/core/Topology.h"
#include "shift/core/log.h"
#include "shift/core/Jacobian.h"

#include "ksl/linalg.h"
#include "ksl/print.h"

void shift::core::Substructure::set_context(shift::core::Context* context) {
  context_ = context;
  assert(context_);

  epsilon_rank_determination_ =
    (*context_)["model_parameters"]["epsilon_rank_determination"].get<double>();

  infinity_norm_multiplier_ =
    context_->get_model_parameters().get_infinity_norm_multiplier();

  epsilon_pivot_ratio_ =
    (*context_)["model_parameters"]["epsilon_pivot_ratio"].get<double>();
}

void shift::core::Substructure::SetupPO1(
  const shift::core::topology::DHVA& dhva) {
  memset(J_, 0, dq_count_ * constraint_equation_count_ * sizeof(double));
  /* j is local loop index */
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  SHIFT_TRACE("positive loop members");
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto variable_index : *it) {
      SHIFT_TRACE(std::to_string(variable_index));
      assert(variable_index >= 0 && variable_index - dq_offset_ < dq_count_);
      ksl_screw_copy_rmo(&dhva.h[variable_index], J_, j * 6,
                         variable_index - dq_offset_, dq_count_);
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  SHIFT_TRACE("negative loop members");
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto variable_index : *it) {
      SHIFT_TRACE(std::to_string(variable_index));
      assert(variable_index >= 0 && variable_index - dq_offset_ < dq_count_);
      ksl_screw_ncopy_rmo(&dhva.h[variable_index], J_, j * 6,
                          variable_index - dq_offset_, dq_count_);
    }
  }

  // SHIFT_TRACE("Jacobian:");
  // ksl_array2D_print(stdout, constraint_equation_count_, dq_count_, J_);

  rank_ = ksl_linalg_lu_full_rmo(constraint_equation_count_, dq_count_, J_,
                                 epsilon_rank_determination_, pr_12_.data(),
                                 pc_12_.data());

  // SHIFT_TRACE("Factored Jacobian:");
  // ksl_array2D_print(stdout, constraint_equation_count_, dq_count_, J_);
  // SHIFT_TRACE("Rank: " + std::to_string(rank_));
  // assert(rank_ != -1);

  shift::core::topology::SetInversePermutation(pr_12_, pr_21_);
  shift::core::topology::SetInversePermutation(pc_12_, pc_21_);

  // std::cout << "rank: " << rank_ << std::endl;
  // std::cout << "dq_count: " << dq_count_ << std::endl;

  if(rank_ < (int)dq_count_) {
    ksl_linalg_lu_setBMatrix_rmo(rank_, dq_count_, J_);
    initial_infinity_norm_b_ =
      ksl_linalg_lu_infinityNormB_rmo(rank_, dq_count_, J_);
  }
}

/*!
\brief evaluate permuted Jacobian matrix in PO2
for a single substructure
*/
void shift::core::Substructure::SetupPO2(
  const shift::core::topology::DHVA& dhva) {
  memset(J_, 0, dq_count_ * constraint_equation_count_ * sizeof(double));

  /* j is local loop index*/
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variable_index : *it) {
      int column_index_2 = pc_21_[variable_index - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int row_index_2 = pr_21_[j * 6 + m];
        J_[row_index_2 * dq_count_ + column_index_2] =
          dhva.h[variable_index].at[m];
      }
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variable_index : *it) {
      int column_index_2 = pc_21_[variable_index - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int row_index_2 = pr_21_[j * 6 + m];
        J_[row_index_2 * dq_count_ + column_index_2] =
          -dhva.h[variable_index].at[m];
      }
    }
  }
}

/*!
\brief evaluate permuted Jacobian boolean matrix in PO2
for a single substructure
*/
void shift::core::Substructure::SetupBooleanPO2() {
  std::fill(Jb_.begin(), Jb_.end(), 0);

  /* j is local loop index */
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variable_index : *it) {
      int column_index = pc_21_[variable_index - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int row_index = pr_21_[j * 6 + m];
        Jb_[row_index * dq_count_ + column_index] = true;
      }
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variable_index : *it) {
      assert((int)variable_index - (int)dq_offset_ >= 0);
      int column_index = pc_21_[variable_index - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int row_index = pr_21_[j * 6 + m];
        Jb_[row_index * dq_count_ + column_index] = true;
      }
    }
  }

  /* Record fills in LU factorization */
  for(int j = 0; j < rank_; j++) {
    for(int k = j + 1; k < rank_; k++) {
      if(Jb_[k * dq_count_ + j]) {
        for(unsigned int l = j + 1; l < dq_count_; l++) {
          if(Jb_[j * dq_count_ + l]) {
            Jb_[k * dq_count_ + l] = true;
          }
        }
      }
    }
  }

  /* Record fills in Boolean B matrix */
  for(int j = rank_ - 1; j > -1; j--) {
    for(int k = rank_ - 1; k > j; k--) {
      if(Jb_[j * dq_count_ + k]) {
        for(unsigned int l = rank_; l < dq_count_; l++) {
          if(Jb_[k * dq_count_ + l]) {
            Jb_[j * dq_count_ + l] = true;
          }
        }
      }
    }
  }
}

void shift::core::Substructure::FactorPO2() {
  ksl_linalg_lu_rmo(rank_, dq_count_, J_);
}

void shift::core::Substructure::SetupBMatrixSparse(double* __restrict__ b3) {
  if(rank_ > 0 && rank_ < (int)dq_count_) {
    infinity_norm_b_ = 0;
    for(int i = rank_ - 1; i > -1; i--) { /* rows of B */
      for(unsigned int k = rank_; k < dq_count_; k++) {
        // assert(J_[i * dq_count_ + i] == pivots[i]);
        if(Jb_[i * dq_count_ + k]) {
          double save = 0;
          for(int j = rank_ - 1; j > i; j--) {
            save += J_[i * dq_count_ + j] * J_[j * dq_count_ + k];
          }
          int b3_index = ij_mapping_[i * dq_count_ + k];
          assert(b3_index > -1);
          b3[b3_index] = (J_[i * dq_count_ + k] - save) / J_[i * dq_count_ + i];
          J_[i * dq_count_ + k] = b3[b3_index];
          assert(!std::isinf(b3[b3_index]));
          assert(!std::isnan(b3[b3_index]));
          infinity_norm_b_ = std::max(infinity_norm_b_, std::abs(b3[b3_index]));
        }
      }
    }
  }
}

void shift::core::Substructure::EvaluateSparsity(shift::core::Array2d<int>& rb3,
                                                 std::vector<int>& cb3,
                                                 std::vector<int>& pv_23) {
  int ij_mapping_temp_po3[dq_count_ * dq_count_];

  for(unsigned int i = 0; i < dq_count_ * dq_count_; i++) {
    ij_mapping_temp_po3[i] = -1;
  }

  /* generate a local pv_local_23 array */
  int pv_local_23[dq_count_];
  for(unsigned int i = 0; i < dq_count_; i++) {
    pv_local_23[i] = pv_23[i + dq_offset_] - dq_offset_;
  }

  /* set ij mapping temp po3 indices */
  int k = b_offset_;
  for(unsigned int i3 = dq_count_ - rank_; i3 < dq_count_; i3++) {
    int i2 = pv_local_23[i3];
    assert(i2 > -1 && i2 < (int)dq_count_);
    int rb3_index = i3 + dq_offset_;
    assert(rb3_index > 0);
    rb3[rb3_index][0] = k;
    for(unsigned int j3 = 0; j3 < dq_count_ - rank_; j3++) {
      int j2 = pv_local_23[j3];
      if(Jb_[i2 * dq_count_ + j2]) {
        ij_mapping_temp_po3[i3 * dq_count_ + j3] = k;
        cb3[k] = j3 + dq_offset_;
        k++;
      }
    }
    rb3[rb3_index][1] = k;
  }

  /* permute ij mapping temp to po2 */
  for(unsigned int i3 = dq_count_ - rank_; i3 < dq_count_; i3++) {
    int i2 = pv_local_23[i3];
    assert(i2 > -1 && i2 < (int)dq_count_);
    for(unsigned int j3 = 0; j3 < dq_count_ - rank_; j3++) {
      int j2 = pv_local_23[j3];
      assert(j2 > -1 && j2 < (int)dq_count_);
      ij_mapping_[i2 * dq_count_ + j2] =
        ij_mapping_temp_po3[i3 * dq_count_ + j3];
    }
  }
}

void shift::core::Substructure::Print() const {
  std::cout << "Substructure [" << index_ << "]" << std::endl;

  std::cout << "  compressed positive members:" << std::endl;
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    std::cout << "  [" + std::to_string(j) + "]:";
    for(auto& variable_index : *it) {
      std::cout << " " + std::to_string(variable_index);
    }
    std::cout << std::endl;
  }

  std::cout << "  compressed negative members:" << std::endl;
  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    std::cout << "  [" + std::to_string(j) + "]:";
    for(auto& variable_index : *it) {
      std::cout << " " + std::to_string(variable_index);
    }
    std::cout << std::endl;
  }

  std::cout << "  dq_count: " << dq_count_ << std::endl;
  std::cout << "  rank: " << rank_ << std::endl;
  std::cout << "  dq_offset: " << dq_offset_ << std::endl;
  std::cout << "  chord_constraint_count: " << chord_constraint_count_
            << std::endl;
  std::cout << "  constraint_equation_count: " << constraint_equation_count_
            << std::endl
            << std::endl;
}

void shift::core::Substructure::FirstOrderUpdate() {
  /* rows in B matrix */
  for(int i = 0; i < rank_; i++) {
    /* columns in B matrix */
    for(unsigned int j = rank_; j < dq_count_; j++) {
      q_[pc_12_[i]] -= J_[i * dq_count_ + j] * delta_q_[pc_12_[j]];
    }
  }
}

/*!
@brief update q array
@param a kinematic substructure to process
*/
void shift::core::Substructure::Updateq() {
  unsigned int q_index = q_offset_;
  unsigned int dq_index = dq_offset_;

  for(unsigned int j = 1;
      j < context_->get_substructure_members()[index_].size(); j++) {
    int index = context_->get_substructure_members()[index_][j];
    auto joint = context_->get_active_joints()[index];
    auto q_count = joint->get_q_count();
    auto dq_count = joint->get_dq_count();
    if(q_count == dq_count) {
      for(unsigned int i = 0; i < q_count; i++) {
        context_->get_q()[q_index++] -= context_->get_delta_q()[dq_index++];
      }
    } else if(q_count == 4 && dq_count == 3) {
      // spherical case
      ksl_subtract_dpiFromq((ksl_vec3_t*)(&context_->get_delta_q()[dq_index]),
                            (ksl_quaternion_t*)(&context_->get_q()[q_index]));
      ksl_quaternion_normalize(
        (ksl_quaternion_t*)(&context_->get_q()[q_index]));
      q_index += 4;
      dq_index += 3;

    } else if(q_count == 7 && dq_count == 6) {
      // float case
      context_->get_q()[q_index++] -= context_->get_delta_q()[dq_index++];
      context_->get_q()[q_index++] -= context_->get_delta_q()[dq_index++];
      context_->get_q()[q_index++] -= context_->get_delta_q()[dq_index++];
      ksl_subtract_dpiFromq((ksl_vec3_t*)(&context_->get_delta_q()[dq_index]),
                            (ksl_quaternion_t*)(&context_->get_q()[q_index]));
      ksl_quaternion_normalize(
        (ksl_quaternion_t*)(&context_->get_q()[q_index]));
      q_index += 4;
      dq_index += 3;
    } else {
      assert(0);
    }
  }
}

/*!
@brief computes RMS error in change of delta q for a substructure

error = sqrt(deltaq[] . deltaq[]) / dq_count
*/
double shift::core::Substructure::ComputeDeltaqError() {
#if SHIFT_WITH_BLAS_
  return sqrt(cblas_ddot(dq_count_, delta_q_, 1, delta_q_, 1)) / dq_count_;
#else
  double sum = 0;
  for(unsigned int i = 0; i < dq_count_; i++) {
    sum += delta_q_[i] * delta_q_[i];
  }
  return sqrt(sum) / dq_count_;
#endif
}

/*!

*/
void shift::core::Substructure::NewtonRaphsonSolver() {

  // assert(context_->get_jacobian().get_constraints().data() + loop_offset_ * 6 ==
  //       e_);
  memset(delta_q_, 0, dq_count_ * sizeof(double));
  /* cs <-  L^-1 cs */
  for(int i = 0; i < rank_; i++) {
    for(int j = 0; j < i; j++) {
      e_[pr_12_[i]] -= J_[i * dq_count_ + j] * e_[pr_12_[j]];
    }
  }
  /* deltaq <- U^-1 * (L^-1 * cs) */
  for(int i = rank_ - 1; i > -1; i--) {
    for(int j = rank_ - 1; j > i; j--) {
      e_[pr_12_[i]] -= J_[i * dq_count_ + j] * delta_q_[pc_12_[j]];
    }
    delta_q_[pc_12_[i]] = e_[pr_12_[i]] / J_[i * dq_count_ + i];
  }
}

/*!
\brief evaluate position level constraints for one substructure
\return RMS constraint error
*/
double shift::core::Substructure::EvaluateConstraintsNewtonRaphson(
  const shift::core::topology::DHVA& dhva) {
  assert(constraints_.size() == chord_constraint_count_);
  for(unsigned int i = 0; i < chord_constraint_count_; i++) {
    const ksl_SE3_t* d_parent = &dhva.d[constraints_[i].parent.d];
    const ksl_SE3_t* d_child = &dhva.d[constraints_[i].child.d];
    ksl_SE3_t temp;

    ksl_product_ddinv(d_parent, d_child, &temp);

    memcpy(e_ + i * 6, &temp.t, sizeof(ksl_vec3_t));

    double trace = temp.R.m00 + temp.R.m11 + temp.R.m22 - 3.0;
    e_[i * 6 + 3] = 0.5 * (temp.R.m21 - temp.R.m12) + trace;
    e_[i * 6 + 4] = 0.5 * (temp.R.m02 - temp.R.m20) + trace;
    e_[i * 6 + 5] = 0.5 * (temp.R.m10 - temp.R.m01) + trace;

#if 1
    {
      double tol = 1.0e-12;
      assert(fabs(ksl_mat3x3_determinant(&d_parent->R) - 1.0) < tol);
      assert(fabs(ksl_mat3x3_determinant(&d_child->R) - 1.0) < tol);
      assert(fabs(ksl_mat3x3_determinant(&temp.R) - 1.0) < tol);
    }
#endif

#if 0
    {
      std::cout << "e_: ";
      for(unsigned int j = 0; j < 6; j++) {
        std::cout << e_[i * 6 + j] << " ";
      }
      std::cout << std::endl;

      double rms_constraint_error = 0.0;
      for(size_t j = 0; j < 6; j++) {
        rms_constraint_error += e_[i * 6 + j] * e_[i * 6 + j];
      }
      rms_constraint_error = sqrt(rms_constraint_error) / 6.0;
      std::cout << "rms constraint error: " << rms_constraint_error << std::endl;

      tol = 1.e-2;
      if(rms_constraint_error > tol) {
        context_->DigraphFrames();
        throw shift::core::Exception(
          "Simulation failed when constraint error tolerance was exceeded (" +
          std::to_string(fabs(trace)) + " > " + std::to_string(tol) + ")");
      }
    }
#endif
  }

#if SHIFT_WITH_BLAS_
  return sqrt(cblas_ddot(constraint_equation_count_, e_, 1, e_, 1)) /
         constraint_equation_count_;
#else
  double sum = 0;
  for(unsigned int i = 0; i < constraint_equation_count_; i++) {
    sum += e_[i] * e_[i];
  }
  return sqrt(sum) / constraint_equation_count_;
#endif

}

double shift::core::Substructure::EvaluateConstraints(
  const shift::core::topology::DHVA& dhva) {
  assert(constraints_.size() == chord_constraint_count_);
  for(unsigned int i = 0; i < chord_constraint_count_; i++) {
    const ksl_SE3_t* d_parent = &dhva.d[constraints_[i].parent.d];
    const ksl_SE3_t* d_child = &dhva.d[constraints_[i].child.d];
    ksl_SE3_t temp;

    ksl_product_ddinv(d_parent, d_child, &temp);

    //memcpy(e_ + i * 6, &temp.t, sizeof(ksl_vec3_t));

    e_[i * 6 + 0] = temp.t.at[0];
    e_[i * 6 + 1] = temp.t.at[1];
    e_[i * 6 + 2] = temp.t.at[2];
    e_[i * 6 + 3] = 0.5 * (temp.R.m21 - temp.R.m12);
    e_[i * 6 + 4] = 0.5 * (temp.R.m02 - temp.R.m20);
    e_[i * 6 + 5] = 0.5 * (temp.R.m10 - temp.R.m01);

    for(size_t j = 0; j < 6; j++) {
      de_[i * 6 + j] =
        dhva.v[constraints_[i].parent.v].at[j] -
        dhva.v[constraints_[i].child.v].at[j];
    }


#if 1
    {
      double tol = 1.0e-12;
      assert(fabs(ksl_mat3x3_determinant(&d_parent->R) - 1.0) < tol);
      assert(fabs(ksl_mat3x3_determinant(&d_child->R) - 1.0) < tol);
      assert(fabs(ksl_mat3x3_determinant(&temp.R) - 1.0) < tol);
    }
#endif

#if 0
    {
      std::cout << "e_: ";
      for(unsigned int j = 0; j < 6; j++) {
        std::cout << e_[i * 6 + j] << " ";
      }
      std::cout << std::endl;

      double rms_constraint_error = 0.0;
      for(size_t j = 0; j < 6; j++) {
        rms_constraint_error += e_[i * 6 + j] * e_[i * 6 + j];
      }
      rms_constraint_error = sqrt(rms_constraint_error) / 6.0;
      std::cout << "rms constraint error: " << rms_constraint_error << std::endl;

      tol = 1.e-2;
      if(rms_constraint_error > tol) {
        context_->DigraphFrames();
        throw shift::core::Exception(
          "Simulation failed when constraint error tolerance was exceeded (" +
          std::to_string(fabs(trace)) + " > " + std::to_string(tol) + ")");
      }
    }
#endif
  }

#if SHIFT_WITH_BLAS_
  return sqrt(cblas_ddot(constraint_equation_count_, e_, 1, e_, 1)) /
         constraint_equation_count_;
#else
  double sum = 0;
  for(unsigned int i = 0; i < constraint_equation_count_; i++) {
    sum += e_[i] * e_[i];
  }
  return sqrt(sum) / constraint_equation_count_;
#endif
}

unsigned int shift::core::Substructure::RemoveConstraintViolations() {
  assert(context_);

  /* Iteratively update x to satisfy constraint equations.

  dx[col_dim] contains changes in x[col_dim] for current N-R iteration.

  error = sqrt(e[] . e[]) / constraint_equation_count < eps when converged.
  */
  int max_step = 1000;
  double q_copy[q_count_];
  memcpy(q_copy, q_, q_count_ * sizeof(double));

  assert(context_);
  context_->UpdateDH();
  double initial_error = EvaluateConstraints(context_->get_dhva());

  SHIFT_DEBUG("Initial constraint error for substructure " +
              std::to_string(index_) + ": " + std::to_string(initial_error));

  int step = 0;
  double eps_rank_determination =
    context_->get_model_parameters().get_epsilon_rank_determination();
  double error = eps_rank_determination + 1;
  while(error > eps_rank_determination && step < max_step) {
    SetupPO1(context_->get_dhva());
    NewtonRaphsonSolver();
    Updateq();
    context_->UpdateDH();
    error = EvaluateConstraints(context_->get_dhva());

    SHIFT_DEBUG("[" << step << "] error: " + std::to_string(error));
    ++step;
  }
  context_->UpdateDH();
  SetupBooleanPO2();
  if(step == max_step) {
    PrintError(initial_error, error, q_copy);
    context_->DigraphFrames();
    throw shift::core::Exception("Failed to remove constraint violations");
  }
  return shift::core::Jacobian::Failure::OK;
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables.

Does not set the jacobian boolean matrix, which is necessary for solving
the equations of motion.
*/
unsigned int shift::core::Substructure::RemoveConstraintViolationsFast(
  bool use_tight_tolerance) {

  context_->UpdateDH();
  double initial_error = EvaluateConstraintsNewtonRaphson(context_->get_dhva());
  double q_copy[q_count_];
  memcpy(q_copy, q_, q_count_ * sizeof(double));
  const int max_step = 100;
  double tol;
  if(use_tight_tolerance) {
    tol = context_->get_model_parameters().get_epsilon_rank_determination();
  } else {
    tol = context_->get_model_parameters().get_epsilon_newton_raphson();
  }
  SetupPO2(context_->get_dhva());
  FactorPO2();

  EvaluateConstraintsNewtonRaphson(context_->get_dhva());
  int step = 0;
  double error = tol + 1;
  while(error > tol && step < max_step) {
    NewtonRaphsonSolver();
    Updateq();
    context_->UpdateDH();
    error = EvaluateConstraints(context_->get_dhva());
    step++;
    // printf("  [%d] error: %g\n", step, error);
  }

  if(step == max_step) {
    PrintError(initial_error, error, q_copy);
    throw shift::core::Exception("Failed to remove constraint violations");
  }
  return shift::core::Jacobian::Failure::OK;
}

void shift::core::Substructure::PrintError(double initial_error,
                                           double final_error, double* q_copy) {

  FILE* f = context_->get_model_parameters().get_debug_file_pointer();
  if(!f) {
    f = stderr;
  }
  fprintf(f,
          "\n\n>>> Error: could not resolve constraint violations for "
          "substructure [%d]\n",
          index_);
  fprintf(f, "initial error: %f, final error: %f\n", initial_error,
          final_error);
  fprintf(f, "  q_count: %d\n", q_count_);
  fprintf(f, "  q_offset: %d\n", q_offset_);
  assert(context_->get_q().data() + q_offset_ == q_);
  fprintf(f, " states before Newton-Raphson:\n   ");
  ksl_array_print(f, q_count_, q_copy);
  fprintf(f, " states after Newton-Raphson:\n    ");
  ksl_array_print(f, q_count_, q_);
}

/*!
@brief drive down constraint violations, then perform an initial full factor
to set up slider for repositioning mechanism

removes constraint violations and performs a specified
full factor in PO1, permutations are used for subsequent
fast factorization
*/
void shift::core::Substructure::SetupNewtonRaphsonSpecifiedCoordinate(
  shift::core::topology::DHVA& dhva, int specified_coordinate_index) {
  context_->UpdateDH();
  SetupPO1(context_->get_dhva());
  RemoveConstraintViolationsFast(true);
  SetupPO1(context_->get_dhva());
  ComputeMobility();
  SetupPO1Specified(dhva, specified_coordinate_index);
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one of its independent variables.

Full factor with a specified coordinate must be called prior to calling this
function. A lower error tolerance (context->epsNewtonRaphson) is used to
advance the position of the mechanism for visualization purposes, but a
higher tolerance (context->epsNewtonRaphsonRankDetermination) is used
for the purpose of determining the mechanism rank.

@param substructure [in] pointer to substructure to process
@param specified_coordinate_index [in] global index of coordinate to set as
specified
*/
void shift::core::Substructure::IncrementSpecifiedCoordinate(
  shift::core::topology::DHVA& dhva, int specified_coordinate_index,
  double value, MoveType move_type) {

  /* currently the joint type is not used, but may be used in the future, for
  example to set all 3 dof as specified for a spherical joint */
  memset(delta_q_, 0, dq_count_ * sizeof(double));
  switch(move_type) {
    case INCREMENT: {
      context_->get_delta_q()[specified_coordinate_index] = value;
      context_->get_q()[specified_coordinate_index] += value;
      break;
    }
    case TARGET: {
      context_->get_delta_q()[specified_coordinate_index] =
        value - context_->get_q()[specified_coordinate_index];
      context_->get_q()[specified_coordinate_index] = value;
      break;
    }
    default: {
      throw shift::core::Exception(
        "Encountered case not handled in IncrementSpecifiedCoordinate()");
    }
  }
  context_->UpdateDH();
  SetupPO1Specified(dhva, specified_coordinate_index);
  FirstOrderUpdate();
  RemoveConstraintViolationsFast(true);
  SetupBMatrix();
}

/*!
  @brief wrapper function for ksl_linalg_lu_SetupBMatrix_rmo for a substructure
*/
void shift::core::Substructure::SetupBMatrix() {
  ksl_linalg_lu_setBMatrix_rmo(rank_, dq_count_, J_);
}

/*! @brief set initial row-major ordered jacobian in permutation order 1 and
perform an initial call to GCP using ksl_linalg_lu_full for one substructure
with a specified independent coordinate

@param context [in] s2_context_t data structure
@param substructure_index [in] index of substructure
@param dq_index [in] global dq_index of specified joint

the Jacobian is stored in row-major order
*/
void shift::core::Substructure::SetupPO1Specified(
  const shift::core::topology::DHVA& dhva, int dq_index) {

  memset(J_, 0, dq_count_ * constraint_equation_count_ * sizeof(double));

  /* j is local loop index */
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variable_index : *it) {
      // std::cout << variable_index << std::endl;
      // ksl_screw_print(stdout, &dhva.h[variable_index]);
      ksl_screw_copy_rmo(&dhva.h[variable_index], J_, j * 6,
                         variable_index - dq_offset_, dq_count_);
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variable_index : *it) {
      // std::cout << variable_index << std::endl;
      // ksl_screw_print(stdout, &dhva.h[variable_index]);
      ksl_screw_ncopy_rmo(&dhva.h[variable_index], J_, j * 6,
                          variable_index - dq_offset_, dq_count_);
    }
  }

  /* convert global index to local index */
  int local_specified_index = dq_index - dq_offset_;
  assert(local_specified_index > -1);

  rank_ = ksl_linalg_lu_full_specified_rmo(
    constraint_equation_count_, dq_count_, J_,
    context_->get_model_parameters().get_epsilon_rank_determination(),
    pr_12_.data(), pc_12_.data(), local_specified_index);
  topology::SetInversePermutation(pr_12_, pr_21_);
  topology::SetInversePermutation(pc_12_, pc_21_);

  SetupBMatrix();

  initial_infinity_norm_b_ = InfinityNormB();

  SetupBooleanPO2();
}

double shift::core::Substructure::InfinityNormB() {
  return ksl_linalg_lu_infinityNormB_rmo(rank_, dq_count_, J_);
}

double shift::core::Substructure::PivotRatio() {
  double pivot_min = fabs(J_[0]);
  double pivot_max = pivot_min;
  for(ssize_t i = 1; i < rank_; i++) {
    double pivot = fabs(J_[i * dq_count_ + i]);
    pivot_min = fmin(pivot_min, pivot);
    pivot_max = fmax(pivot_max, pivot);
  }
  return pivot_min / pivot_max;
}

unsigned int shift::core::Substructure::CheckCondition() {
  return CheckCondition(infinity_norm_multiplier_);
}

unsigned int shift::core::Substructure::CheckCondition(
  const double infinity_norm_multiplier) {

  unsigned int return_value = shift::core::Jacobian::Failure::OK;

  const double infinity_norm_b = InfinityNormB();
  if(infinity_norm_b > (infinity_norm_multiplier * initial_infinity_norm_b_)) {
    SHIFT_DEBUG("Infinity norm B check failed with value " +
                std::to_string(infinity_norm_b));
    return_value |= shift::core::Jacobian::Failure::INFINITY_NORM_B;
  }

  const double pivot_ratio = PivotRatio();
  double time, dt;
  context_->get_current_time(time, dt);
  
  // std::cout << time << " " << pivot_ratio << std::endl;
  if(pivot_ratio < epsilon_pivot_ratio_) {
    SHIFT_DEBUG("Pivot ratio check failed with value " +
                std::to_string(pivot_ratio));
    return_value |= shift::core::Jacobian::Failure::PIVOT_RATIO;
  }
  
  return return_value;
}

unsigned int shift::core::Substructure::RemoveConstraintViolationsRunTime(
  double* q, const double* dq) {

  SHIFT_DEBUG(
    "Performing full factor on substructure [" + std::to_string(index_) + "]"
  );

  /* backup q array to q_copy */
  double* q_copy = context_->get_q_copy().data();
  memcpy(q_copy + q_offset_, q + q_offset_, q_count_ * sizeof(double));

  unsigned int return_value = RemoveConstraintViolations();
    
  double nr_error = EvaluateConstraints(context_->get_dhva());

  SHIFT_DEBUG("Error after newton raphson: " + std::to_string(nr_error));

  /* restore states */
  memcpy(q + q_offset_, q_copy + q_offset_, q_count_ * sizeof(double));
  
  context_->UpdateDHVA(q, dq);
  SetupPO1(context_->get_dhva());

  SHIFT_DEBUG("rank: " + std::to_string(rank_));

  EvaluateConstraints(context_->get_dhva());
  return return_value;
}

/*!
@brief vector copy B matrix into null space array
*/
void shift::core::Substructure::CopyNullSpace() {
  memset(right_null_space_, 0, dq_count_ * dq_count_ * sizeof(double));
  /* dq_count - rank is the column dimension of null space */
  assert((int)dq_count_ >= rank_ && rank_ != -1);
  int col_dim = dq_count_ - rank_;
  for(int i = 0; i < rank_; i++) {
#if SHIFT_WITH_BLAS_
    cblas_dcopy(col_dim, J_ + (i * dq_count_ + rank_), 1,
                right_null_space_ + (pc_12_[i] * col_dim), 1);
#else
    for(int j = 0; j < col_dim; j++) {
      right_null_space_[pc_12_[i] * col_dim + j] =
        J_[i * dq_count_ + rank_ + j];
    }
#endif
  }

  /* set the remaining matrix to identity */
  unsigned int j = 0;
  for(unsigned int i = rank_; i < dq_count_; i++) {
    right_null_space_[pc_12_[i] * col_dim + j++] = 1.0;
  }
}

/* m is row dimension, i iterates over all rows */

/*!
  @brief minimal version of computing mobility numbers, assumes
  jacobian of substructre has already been factored using full factor
  and mobility numbers have been initialized

  this function requires that SetupPO2, FactorPO2 and SetupBMatrix
  have been called prior to execution
*/
void shift::core::Substructure::ComputeMobility() {
  if(dq_count_ - rank_ == 0) {
    printf("zero mobility detected for substructure\n");
    assert(mobility_numbers_);
    memset(mobility_numbers_, 0, dq_count_ * sizeof(double));
    return;
  }
  CopyNullSpace();
  ksl_linalg_gramSchmidt(right_null_space_, dq_count_, dq_count_ - rank_);
  assert(mobility_numbers_);
  assert((int)dq_count_ >= rank_ && rank_ != -1);
  int n = dq_count_ - rank_;
  for(unsigned int i = 0; i < dq_count_; i++) {
#if SHIFT_WITH_BLAS_
    mobility_numbers_[i] =
      cblas_ddot(n, right_null_space_ + i * n, 1, right_null_space_ + i * n, 1);
#else
    mobility_numbers_[i] = 0;
    for(int j = 0; j < n; j++) {
      mobility_numbers_[i] +=
        right_null_space_[i * n + j] * right_null_space_[i * n + j];
    }
#endif
  }
}

void shift::core::Substructure::Connect() {
  print_constraint_error_ =
    (*context_)["model_parameters"]["print_constraint_error"].get<bool>();
}
