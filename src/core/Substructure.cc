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

#include "ksl/linalg.h"
#include "ksl/print.h"

void shift::core::Substructure::setContext(shift::core::Context* context) {
  context_ = context;
  assert(context_);
  epsilon_rank_determination_ =
    context->modelParameters().epsilonRankDetermination();
  infinity_norm_multiplier_ =
    context->modelParameters().infinityNormMultiplier();
}

void shift::core::Substructure::setPO1(
  const shift::core::topology::DHVA& dhva) {
  memset(J_, 0, dq_count_ * constraint_equation_count_ * sizeof(double));
  /* j is local loop index */
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto variableIndex : *it) {
      assert(variableIndex >= 0 && variableIndex - dq_offset_ < dq_count_);
      ksl_screw_copy_rmo(dhva.h + variableIndex, J_, j * 6,
                         variableIndex - dq_offset_, dq_count_);
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto variableIndex : *it) {
      assert(variableIndex >= 0 && variableIndex - dq_offset_ < dq_count_);
      ksl_screw_ncopy_rmo(dhva.h + variableIndex, J_, j * 6,
                          variableIndex - dq_offset_, dq_count_);
    }
  }

  // ksl_array2D_print(stdout, constraint_equation_count_, dq_count_, J_);

  rank_ = ksl_linalg_lu_full_rmo(constraint_equation_count_, dq_count_, J_,
                                 epsilon_rank_determination_, pr_12_.data(),
                                 pc_12_.data());
  assert(rank_ != -1);

  shift::core::topology::setInversePermutation(pr_12_, pr_21_);
  shift::core::topology::setInversePermutation(pc_12_, pc_21_);

  // std::cout << "rank: " << rank_ << std::endl;
  // std::cout << "dqCount: " << dq_count_ << std::endl;

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
void shift::core::Substructure::setPO2(
  const shift::core::topology::DHVA& dhva) {
  memset(J_, 0, dq_count_ * constraint_equation_count_ * sizeof(double));

  /* j is local loop index*/
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variableIndex : *it) {
      int columnIndex_2 = pc_21_[variableIndex - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex_2 = pr_21_[j * 6 + m];
        J_[rowIndex_2 * dq_count_ + columnIndex_2] =
          dhva.h[variableIndex].at[m];
      }
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variableIndex : *it) {
      int columnIndex_2 = pc_21_[variableIndex - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex_2 = pr_21_[j * 6 + m];
        J_[rowIndex_2 * dq_count_ + columnIndex_2] =
          -dhva.h[variableIndex].at[m];
      }
    }
  }
}

/*!
\brief evaluate permuted Jacobian boolean matrix in PO2
for a single substructure
*/
void shift::core::Substructure::setBooleanPO2() {
  std::fill(Jb_.begin(), Jb_.end(), 0);

  /* j is local loop index */
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variableIndex : *it) {
      int columnIndex = pc_21_[variableIndex - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex = pr_21_[j * 6 + m];
        Jb_[rowIndex * dq_count_ + columnIndex] = true;
      }
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variableIndex : *it) {
      assert((int)variableIndex - (int)dq_offset_ >= 0);
      int columnIndex = pc_21_[variableIndex - dq_offset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex = pr_21_[j * 6 + m];
        Jb_[rowIndex * dq_count_ + columnIndex] = true;
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

void shift::core::Substructure::factorPO2() {
  ksl_linalg_lu_rmo(rank_, dq_count_, J_);
}

void shift::core::Substructure::setBMatrixSparse(double* __restrict__ b3) {
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
          int b3Index = ij_mapping_[i * dq_count_ + k];
          assert(b3Index > -1);
          b3[b3Index] = (J_[i * dq_count_ + k] - save) / J_[i * dq_count_ + i];
          J_[i * dq_count_ + k] = b3[b3Index];
          assert(!std::isinf(b3[b3Index]));
          assert(!std::isnan(b3[b3Index]));
          infinity_norm_b_ = std::max(infinity_norm_b_, std::abs(b3[b3Index]));
        }
      }
    }
  }
}

void shift::core::Substructure::evaluateSparsity(shift::core::Array2d<int>& rb3,
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
    int rb3Index = i3 + dq_offset_;
    assert(rb3Index > 0);
    rb3[rb3Index][0] = k;
    for(unsigned int j3 = 0; j3 < dq_count_ - rank_; j3++) {
      int j2 = pv_local_23[j3];
      if(Jb_[i2 * dq_count_ + j2]) {
        ij_mapping_temp_po3[i3 * dq_count_ + j3] = k;
        cb3[k] = j3 + dq_offset_;
        k++;
      }
    }
    rb3[rb3Index][1] = k;
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

void shift::core::Substructure::print() {
  std::cout << "Substructure [" << index_ << "]" << std::endl;
  std::cout << "  members:" << std::endl;
  std::cout << "  dqCount: " << dq_count_ << std::endl;
  std::cout << "  rank: " << rank_ << std::endl;
  std::cout << "  dqOffset: " << dq_offset_ << std::endl;
  std::cout << "  chordConstraintCount: " << chord_constraint_count_
            << std::endl;
  std::cout << "  constraintEquationCount: " << constraint_equation_count_
            << std::endl
            << std::endl;
}

void shift::core::Substructure::firstOrderUpdate() {
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
void shift::core::Substructure::updateq() {
  unsigned int qIndex = q_offset_;
  unsigned int dqIndex = dq_offset_;

  for(unsigned int j = 1; j < context_->substructureMembers()[index_].size();
      j++) {
    int index = context_->substructureMembers()[index_][j];
    auto joint = context_->activeJoints()[index];
    auto qCount = joint->qCount();
    auto dqCount = joint->dqCount();
    if(qCount == dqCount) {
      for(unsigned int i = 0; i < qCount; i++) {
        context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      }
    } else if(qCount == 4 && dqCount == 3) {
      // spherical case
      ksl_subtract_dpiFromq((ksl_vec3_t*)(&context_->deltaq()[dqIndex]),
                            (ksl_quaternion_t*)(&context_->q()[qIndex]));
      ksl_quaternion_normalize((ksl_quaternion_t*)(&context_->q()[qIndex]));
      qIndex += 4;
      dqIndex += 3;

    } else if(qCount == 7 && dqCount == 6) {
      // float case
      context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      ksl_subtract_dpiFromq((ksl_vec3_t*)(&context_->deltaq()[dqIndex]),
                            (ksl_quaternion_t*)(&context_->q()[qIndex]));
      ksl_quaternion_normalize((ksl_quaternion_t*)(&context_->q()[qIndex]));
      qIndex += 4;
      dqIndex += 3;
    } else {
      assert(0);
    }
  }
}

/*!
@brief computes RMS error in change of delta q for a substructure

error = sqrt(deltaq[] . deltaq[]) / dqCount
*/
double shift::core::Substructure::qError() {
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
void shift::core::Substructure::newtonRaphsonSolver() {
  memset(delta_q_, 0, dq_count_ * sizeof(double));
  /* cs <-  L^-1 cs */
  for(int i = 0; i < rank_; i++) {
    for(int j = 0; j < i; j++) {
      cs_[pr_12_[i]] -= J_[i * dq_count_ + j] * cs_[pr_12_[j]];
    }
  }
  /* deltaq <- U^-1 * (L^-1 * cs) */
  for(int i = rank_ - 1; i > -1; i--) {
    for(int j = rank_ - 1; j > i; j--) {
      cs_[pr_12_[i]] -= J_[i * dq_count_ + j] * delta_q_[pc_12_[j]];
    }
    delta_q_[pc_12_[i]] = cs_[pr_12_[i]] / J_[i * dq_count_ + i];
  }
}

/*!
\brief evaluate position level constraints for one substructure
\return RMS constraint error
*/
double shift::core::Substructure::evaluateConstraints(
  const shift::core::topology::DHVA& dhva) {
  assert(constraints_.size() == chord_constraint_count_);

  for(unsigned int i = 0; i < chord_constraint_count_; i++) {
    ksl_SE3_t* parent_frame = dhva.d + constraints_[i].parent.d;
    ksl_SE3_t* child_frame = dhva.d + constraints_[i].child.d;
    ksl_SE3_t temp;

    ksl_product_ddinv(parent_frame, child_frame, &temp);

    memcpy(cs_ + i * 6, &temp.t, sizeof(ksl_vec3_t));
    double trace = temp.R.m00 + temp.R.m11 + temp.R.m22 - 3.0;

    cs_[i * 6 + 3] = 0.5 * (temp.R.m21 - temp.R.m12) + trace;
    cs_[i * 6 + 4] = 0.5 * (temp.R.m02 - temp.R.m20) + trace;
    cs_[i * 6 + 5] = 0.5 * (temp.R.m10 - temp.R.m01) + trace;

#if 0
      ksl_SE3_print(stdout, parent_frame, "parent_frame: ");
      std::cout << "determinant: " << ksl_mat3x3_determinant(&parent_frame->R)
                << std::endl
                << std::endl;

      ksl_SE3_print(stdout, child_frame, "child_frame: ");
      std::cout << "determinant: " << ksl_mat3x3_determinant(&child_frame->R)
                << std::endl
                << std::endl;

      ksl_SE3_print(stdout, &temp, "ddinv: ");
      std::cout << "determinant: " << ksl_mat3x3_determinant(&temp.R)
                << std::endl
                << std::endl;
      
      std::cout << "trace - 3: " << trace << std::endl;
      std::cout << "cs_: ";
      for(unsigned int j = 0; j < 6; j++) {
        std::cout << cs_[i * 6 + j] << " ";
      }
      std::cout << std::endl;

    double substructure_error = 0.0;
    for(size_t j = 0; j < 6; j++) {
      substructure_error += cs_[i * 6 + j] * cs_[i * 6 + j];
  
    substructure_error = sqrt(substructure_error) / 6.0;

    std::cout << "chord loop error: " << substructure_error << std::endl;
    
    double tol = 1e-2;
    if(substructure_error > tol) {
      context_->digraphFrames();
      throw shift::core::Exception(
        "Simulation failed when constraint error tolerance was exceeded (" +
        std::to_string(fabs(trace)) + " > " + std::to_string(tol) + ")");
    };
#endif
  }

#if SHIFT_WITH_BLAS_
  return sqrt(cblas_ddot(constraint_equation_count_, cs_, 1, cs_, 1)) /
         constraint_equation_count_;
#else
  double sum = 0;
  for(unsigned int i = 0; i < constraint_equation_count_; i++) {
    sum += cs_[i] * cs_[i];
  }
  return sqrt(sum) / constraint_equation_count_;
#endif
}

int shift::core::Substructure::removeConstraintViolations() {
  assert(context_);

  /* Iteratively update x to satisfy constraint equations.

  dx[colDim] contains changes in x[colDim] for current N-R iteration.

  error = sqrt(cs[] . cs[]) / constraintEquationCount < eps when converged.
  */
  int max_step = 100;
  double q_copy[q_count_];
  memcpy(q_copy, q_, q_count_ * sizeof(double));

  assert(context_);
  context_->updateDH();
  double initial_error = evaluateConstraints(context_->dhva());

  if(print_constraint_error_) {
    std::cout << std::endl << "initial error: " << initial_error << std::endl;
  }

  int step = 0;
  double eps_rank_determination =
    context_->modelParameters().epsilonRankDetermination();
  double error = eps_rank_determination + 1;
  while(error > eps_rank_determination && step < max_step) {
    setPO1(context_->dhva());
    newtonRaphsonSolver();
    updateq();
    context_->updateDH();
    error = evaluateConstraints(context_->dhva());
    
    if(print_constraint_error_) {
      for(size_t j = 0; j < q_count_; j++) {
        std::cout << q_[j] << " ";
      }
      std::cout << std::endl;
      std::cout << "  [" << step << "] error: " << error << std::endl;
    }
    ++step;
  }
  context_->updateDH();
  setBooleanPO2();
  if(step == max_step) {
    printError(initial_error, error, q_copy);
    throw shift::core::Exception("Failed to remove constraint violations");
  } 
  return 0;
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables.

Does not set the jacobian boolean matrix, which is necessary for solving
the equations of motion.
*/
int shift::core::Substructure::removeConstraintViolationsFast(
  bool useTightTolerance) {

  context_->updateDH();
  double initialError = evaluateConstraints(context_->dhva());
  double qCopy[q_count_];
  memcpy(qCopy, q_, q_count_ * sizeof(double));
  const int maxStep = 100;
  double tol;
  if(useTightTolerance) {
    tol = context_->modelParameters().epsilonRankDetermination();
  } else {
    tol = context_->modelParameters().epsilsonNewtonRaphson();
  }
  setPO2(context_->dhva());
  factorPO2();
  evaluateConstraints(context_->dhva());
  int step = 0;
  double error = tol + 1;
  while(error > tol && step < maxStep) {
    newtonRaphsonSolver();
    updateq();
    context_->updateDH();
    error = evaluateConstraints(context_->dhva());
    step++;
    // printf("  [%d] error: %g\n", step, error);
  }

  if(step == maxStep) {
    printError(initialError, error, qCopy);
    throw shift::core::Exception("Failed to remove constraint violations");
  } 
  return 0;
}

void shift::core::Substructure::printError(double initialError,
                                           double finalError, double* qCopy) {

  FILE* f = context_->modelParameters().debugFilePointer();
  if(!f) {
    f = stderr;
  }
  fprintf(f,
          "\n\n>>> Error: could not resolve constraint violations for "
          "substructure [%d]\n",
          index_);
  fprintf(f, "initial error: %f, final error: %f\n", initialError, finalError);
  fprintf(f, "  qCount: %d\n", q_count_);
  fprintf(f, "  qOffset: %d\n", q_offset_);
  assert(context_->q().data() + q_offset_ == q_);
  fprintf(f, " states before Newton-Raphson:\n   ");
  ksl_array_print(f, q_count_, qCopy);
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
void shift::core::Substructure::setupNewtonRaphsonSpecifiedCoordinate(
  shift::core::topology::DHVA& dhva, int specifiedCoordinateIndex) {
  context_->updateDH();
  setPO1(context_->dhva());
  removeConstraintViolationsFast(true);
  setPO1(context_->dhva());
  computeMobility();
  setPO1Specified(dhva, specifiedCoordinateIndex);
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
@param specifiedCoordinateIndex [in] global index of coordinate to set as
specified
*/
void shift::core::Substructure::incrementSpecifiedCoordinate(
  shift::core::topology::DHVA& dhva, int specifiedCoordinateIndex, double value,
  MoveType moveType) {

  /* currently the joint type is not used, but may be used in the future, for
  example to set all 3 dof as specified for a spherical joint */
  memset(delta_q_, 0, dq_count_ * sizeof(double));
  switch(moveType) {
    case INCREMENT: {
      context_->deltaq()[specifiedCoordinateIndex] = value;
      context_->q()[specifiedCoordinateIndex] += value;
      break;
    }
    case TARGET: {
      context_->deltaq()[specifiedCoordinateIndex] =
        value - context_->q()[specifiedCoordinateIndex];
      context_->q()[specifiedCoordinateIndex] = value;
      break;
    }
    default: {
      throw shift::core::Exception("Encountered case not handled in incrementSpecifiedCoordinate()");
    }
  }
  context_->updateDH();
  setPO1Specified(dhva, specifiedCoordinateIndex);
  firstOrderUpdate();
  removeConstraintViolationsFast(true);
  setBMatrix();
}

/*!
  @brief wrapper function for ksl_linalg_lu_setBMatrix_rmo for a substructure
*/
void shift::core::Substructure::setBMatrix() {
  ksl_linalg_lu_setBMatrix_rmo(rank_, dq_count_, J_);
}

/*! @brief set initial row-major ordered jacobian in permutation order 1 and
perform an initial call to GCP using ksl_linalg_lu_full for one substructure
with a specified independent coordinate

@param context [in] s2_context_t data structure
@param substructureIndex [in] index of substructure
@param dqIndex [in] global dqIndex of specified joint

the Jacobian is stored in row-major order
*/
void shift::core::Substructure::setPO1Specified(
  const shift::core::topology::DHVA& dhva, int dqIndex) {

  memset(J_, 0, dq_count_ * constraint_equation_count_ * sizeof(double));

  /* j is local loop index */
  auto it =
    positive_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variableIndex : *it) {
      // std::cout << variableIndex << std::endl;
      // ksl_screw_print(stdout, dhva.h + variableIndex);
      ksl_screw_copy_rmo(dhva.h + variableIndex, J_, j * 6,
                         variableIndex - dq_offset_, dq_count_);
    }
  }

  it = negative_loop_members_compressed_->begin() + chord_constraint_offset_;
  for(unsigned int j = 0; j < chord_constraint_count_; j++, it++) {
    for(auto& variableIndex : *it) {
      // std::cout << variableIndex << std::endl;
      // ksl_screw_print(stdout, dhva.h + variableIndex);
      ksl_screw_ncopy_rmo(dhva.h + variableIndex, J_, j * 6,
                          variableIndex - dq_offset_, dq_count_);
    }
  }

  /* convert global index to local index */
  int localSpecifiedIndex = dqIndex - dq_offset_;
  assert(localSpecifiedIndex > -1);

  rank_ = ksl_linalg_lu_full_specified_rmo(
    constraint_equation_count_, dq_count_, J_,
    context_->modelParameters().epsilonRankDetermination(), pr_12_.data(),
    pc_12_.data(), localSpecifiedIndex);
  topology::setInversePermutation(pr_12_, pr_21_);
  topology::setInversePermutation(pc_12_, pc_21_);

  setBMatrix();

  initial_infinity_norm_b_ = infinityNormB();

  setBooleanPO2();
}

double shift::core::Substructure::infinityNormB() {
  return ksl_linalg_lu_infinityNormB_rmo(rank_, dq_count_, J_);
}

unsigned int shift::core::Substructure::infinityNormCheck(double* q,
                                                          const double* dq) {
  return infinityNormCheck(q, dq, infinity_norm_multiplier_);
}

unsigned int shift::core::Substructure::infinityNormCheck(
  double* q, const double* dq, const double infinity_norm_multiplier) {

  int return_value = 0;
  const double infinity_norm_b = infinityNormB();
  bool infinity_norm_fail =
    infinity_norm_b > (infinity_norm_multiplier * initial_infinity_norm_b_);

  std::cout << infinity_norm_b << " " << initial_infinity_norm_b_ << std::endl;

  if(infinity_norm_fail) {

    return_value++;

    std::cerr << "performing full factor on substructure [" << index_
              << "]: infinity norm: " << infinity_norm_b << " > "
              << infinity_norm_b * infinity_norm_multiplier << std::endl;

    double* q_copy = context_->qCopy().data();

    /* backup q array to qCopy */
    memcpy(q_copy + q_offset_, q + q_offset_, q_count_ * sizeof(double));

    int newton_raphson_failure = removeConstraintViolationsFast(true);

    std::cerr << "e after nr: " << evaluateConstraints(context_->dhva())
              << std::endl;

    setPO1(context_->dhva());
    std::cerr << "rank: " << rank_ << std::endl;

    if(newton_raphson_failure) {
      return_value++;
      std::cerr << "Newton-Raphson failed!" << std::endl;
      return return_value;
    }

    /* restore states */
    memcpy(q + q_offset_, q_copy + q_offset_, q_count_ * sizeof(double));

    context_->updateDHVA(q, dq);
    evaluateConstraints(context_->dhva());
  }
  return return_value;
}

/*!
@brief vector copy B matrix into null space array
*/
void shift::core::Substructure::copyNullSpace() {
  memset(right_null_space_, 0, dq_count_ * dq_count_ * sizeof(double));
  /* dqCount - rank is the column dimension of null space */
  assert((int)dq_count_ >= rank_ && rank_ != -1);
  int colDim = dq_count_ - rank_;
  for(int i = 0; i < rank_; i++) {
#if SHIFT_WITH_BLAS_
    cblas_dcopy(colDim, J_ + (i * dq_count_ + rank_), 1,
                right_null_space_ + (pc_12_[i] * colDim), 1);
#else
    for(int j = 0; j < colDim; j++) {
      right_null_space_[pc_12_[i] * colDim + j] = J_[i * dq_count_ + rank_ + j];
    }
#endif
  }

  /* set the remaining matrix to identity */
  unsigned int j = 0;
  for(unsigned int i = rank_; i < dq_count_; i++) {
    right_null_space_[pc_12_[i] * colDim + j++] = 1.0;
  }
}

/* m is row dimension, i iterates over all rows */

/*!
  @brief minimal version of computing mobility numbers, assumes
  jacobian of substructre has already been factored using full factor
  and mobility numbers have been initialized

  this function requires that setPO2, factorPO2 and setBMatrix
  have been called prior to execution
*/
void shift::core::Substructure::computeMobility() {
  if(dq_count_ - rank_ == 0) {
    printf("zero mobility detected for substructure\n");
    assert(mobility_numbers_);
    memset(mobility_numbers_, 0, dq_count_ * sizeof(double));
    return;
  }
  copyNullSpace();
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


void shift::core::Substructure::connect() {
  print_constraint_error_ = 
    (*context_)["model_parameters"]["print_constraint_error"].get<bool>();
}
