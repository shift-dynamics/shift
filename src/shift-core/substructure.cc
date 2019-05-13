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

#include "shift-core/context.h"
#include "shift-core/substructure.h"
#include "shift-core/topology.h"

#include "ksl/linalg.h"
#include "ksl/print.h"

void Shift::Substructure::setContext(Shift::Context* context) {
  context_ = context;
  assert(context_);
  epsilonRankDetermination_ =
    context->modelParameters().epsilonRankDetermination();
  infinityNormMultiplier_ = context->modelParameters().infinityNormMultiplier();
}

void Shift::Substructure::setPO1(const Shift::Topology::DHVA& dhva) {
  memset(J_, 0, dqCount_ * constraintEquationCount_ * sizeof(double));
  /* j is local loop index */
  auto it = positiveLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto variableIndex : *it) {
      assert(variableIndex >= 0 && variableIndex - dqOffset_ < dqCount_);
      ksl_screw_copy_rmo(dhva.h + variableIndex, J_, j * 6,
                         variableIndex - dqOffset_, dqCount_);
    }
  }

  it = negativeLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto variableIndex : *it) {
      assert(variableIndex >= 0 && variableIndex - dqOffset_ < dqCount_);
      ksl_screw_ncopy_rmo(dhva.h + variableIndex, J_, j * 6,
                          variableIndex - dqOffset_, dqCount_);
    }
  }

  // ksl_array2D_print(stdout, constraintEquationCount_, dqCount_, J_);

  rank_ = ksl_linalg_lu_full_rmo(constraintEquationCount_, dqCount_, J_,
                                 epsilonRankDetermination_, pr_12_.data(),
                                 pc_12_.data());
  assert(rank_ != -1);

  Shift::Topology::setInversePermutation(pr_12_, pr_21_);
  Shift::Topology::setInversePermutation(pc_12_, pc_21_);

  // std::cout << "rank: " << rank_ << std::endl;
  // std::cout << "dqCount: " << dqCount_ << std::endl;

  if(rank_ < (int)dqCount_) {
    ksl_linalg_lu_setBMatrix_rmo(rank_, dqCount_, J_);
    scaledInitialInfinityNormB_ =
      infinityNormMultiplier_ *
      ksl_linalg_lu_infinityNormB_rmo(rank_, dqCount_, J_);
  }
}

/*!
\brief evaluate permuted Jacobian matrix in PO2
for a single substructure
*/
void Shift::Substructure::setPO2(const Shift::Topology::DHVA& dhva) {
  memset(J_, 0, dqCount_ * constraintEquationCount_ * sizeof(double));

  /* j is local loop index*/
  auto it = positiveLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto& variableIndex : *it) {
      int columnIndex_2 = pc_21_[variableIndex - dqOffset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex_2 = pr_21_[j * 6 + m];
        J_[rowIndex_2 * dqCount_ + columnIndex_2] = dhva.h[variableIndex].at[m];
      }
    }
  }

  it = negativeLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto& variableIndex : *it) {
      int columnIndex_2 = pc_21_[variableIndex - dqOffset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex_2 = pr_21_[j * 6 + m];
        J_[rowIndex_2 * dqCount_ + columnIndex_2] =
          -dhva.h[variableIndex].at[m];
      }
    }
  }
}

/*!
\brief evaluate permuted Jacobian boolean matrix in PO2
for a single substructure
*/
void Shift::Substructure::setBooleanPO2() {
  memset(Jb_, 0, dqCount_ * constraintEquationCount_ * sizeof(bool));

  /* j is local loop index */
  auto it = positiveLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto& variableIndex : *it) {
      int columnIndex = pc_21_[variableIndex - dqOffset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex = pr_21_[j * 6 + m];
        Jb_[rowIndex * dqCount_ + columnIndex] = true;
      }
    }
  }

  it = negativeLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto& variableIndex : *it) {
      assert((int)variableIndex - (int)dqOffset_ >= 0);
      int columnIndex = pc_21_[variableIndex - dqOffset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex = pr_21_[j * 6 + m];
        Jb_[rowIndex * dqCount_ + columnIndex] = true;
      }
    }
  }

  /* Record fills in LU factorization */
  for(int j = 0; j < rank_; j++) {
    for(int k = j + 1; k < rank_; k++) {
      if(Jb_[k * dqCount_ + j]) {
        for(unsigned int l = j + 1; l < dqCount_; l++) {
          if(Jb_[j * dqCount_ + l]) {
            Jb_[k * dqCount_ + l] = true;
          }
        }
      }
    }
  }

  /* Record fills in Boolean B matrix */
  for(int j = rank_ - 1; j > -1; j--) {
    for(int k = rank_ - 1; k > j; k--) {
      if(Jb_[j * dqCount_ + k]) {
        for(unsigned int l = rank_; l < dqCount_; l++) {
          if(Jb_[k * dqCount_ + l]) {
            Jb_[j * dqCount_ + l] = true;
          }
        }
      }
    }
  }
}

void Shift::Substructure::factorPO2() {
  ksl_linalg_lu_rmo(rank_, dqCount_, J_);
}

void Shift::Substructure::setBMatrixSparse(double* __restrict__ b3) {
  if(rank_ > 0 && rank_ < (int)dqCount_) {
    infinityNormB_ = 0;
    for(int i = rank_ - 1; i > -1; i--) { /* rows of B */
      for(unsigned int k = rank_; k < dqCount_; k++) {
        // assert(J_[i * dqCount_ + i] == pivots[i]);
        if(Jb_[i * dqCount_ + k]) {
          double save = 0;
          for(int j = rank_ - 1; j > i; j--) {
            save += J_[i * dqCount_ + j] * J_[j * dqCount_ + k];
          }
          int b3Index = ijMapping_[i * dqCount_ + k];
          assert(b3Index > -1);
          b3[b3Index] = (J_[i * dqCount_ + k] - save) / J_[i * dqCount_ + i];
          J_[i * dqCount_ + k] = b3[b3Index];
          assert(!std::isinf(b3[b3Index]));
          assert(!std::isnan(b3[b3Index]));
          infinityNormB_ = std::max(infinityNormB_, std::abs(b3[b3Index]));
        }
      }
    }
  }
}

void Shift::Substructure::evaluateSparsity(int** __restrict__ rb3,
                                           int* __restrict__ cb3,
                                           std::vector<int>& pv_23) {
  int ijMapping_temp_po3[dqCount_ * dqCount_];

  for(unsigned int i = 0; i < dqCount_ * dqCount_; i++) {
    ijMapping_temp_po3[i] = -1;
  }

  /* generate a local pv_local_23 array */
  int pv_local_23[dqCount_];
  for(unsigned int i = 0; i < dqCount_; i++) {
    pv_local_23[i] = pv_23[i + dqOffset_] - dqOffset_;
  }

  /* set ij mapping temp po3 indices */
  int k = bOffset_;
  for(unsigned int i3 = dqCount_ - rank_; i3 < dqCount_; i3++) {
    int i2 = pv_local_23[i3];
    assert(i2 > -1 && i2 < (int)dqCount_);
    int rb3Index = i3 + dqOffset_;
    assert(rb3Index > 0);
    assert(rb3);
    rb3[rb3Index][0] = k;
    for(unsigned int j3 = 0; j3 < dqCount_ - rank_; j3++) {
      int j2 = pv_local_23[j3];
      if(Jb_[i2 * dqCount_ + j2]) {
        ijMapping_temp_po3[i3 * dqCount_ + j3] = k;
        cb3[k] = j3 + dqOffset_;
        k++;
      }
    }
    rb3[rb3Index][1] = k;
  }

  /* permute ij mapping temp to po2 */
  for(unsigned int i3 = dqCount_ - rank_; i3 < dqCount_; i3++) {
    int i2 = pv_local_23[i3];
    assert(i2 > -1 && i2 < (int)dqCount_);
    for(unsigned int j3 = 0; j3 < dqCount_ - rank_; j3++) {
      int j2 = pv_local_23[j3];
      assert(j2 > -1 && j2 < (int)dqCount_);
      ijMapping_[i2 * dqCount_ + j2] = ijMapping_temp_po3[i3 * dqCount_ + j3];
    }
  }
}

void Shift::Substructure::print() {
  std::cout << "Substructure [" << index_ << "]" << std::endl;
  std::cout << "  members:" << std::endl;
  std::cout << "  dqCount: " << dqCount_ << std::endl;
  std::cout << "  rank: " << rank_ << std::endl;
  std::cout << "  dqOffset: " << dqOffset_ << std::endl;
  std::cout << "  chordConstraintCount: " << chordConstraintCount_ << std::endl;
  std::cout << "  constraintEquationCount: " << constraintEquationCount_
            << std::endl
            << std::endl;
}

void Shift::Substructure::firstOrderUpdate() {
  /* rows in B matrix */
  for(int i = 0; i < rank_; i++) {
    /* columns in B matrix */
    for(unsigned int j = rank_; j < dqCount_; j++) {
      q_[pc_12_[i]] -= J_[i * dqCount_ + j] * deltaq_[pc_12_[j]];
    }
  }
}

/*!
@brief update q array
@param a kinematic substructure to process
*/
void Shift::Substructure::updateq() {
  unsigned int qIndex = qOffset_;
  unsigned int dqIndex = dqOffset_;
  for(unsigned int j = 1; j < context_->substructureMembers()[index_].size();
      j++) {
    auto joint = context_->activeJoints()[j];
    auto qCount = joint->qCount();
    auto dqCount = joint->dqCount();
    if(qCount == dqCount) {
      for(unsigned int i = 0; i < qCount; i++) {
        context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      }
    } else if(qCount == 4 && dqCount == 3) {
      // spherical case
      ksl_subtract_dpiFromq((ksl_vec3_t*)(context_->deltaq() + dqIndex),
                            (ksl_quaternion_t*)(context_->q() + qIndex));
      ksl_quaternion_normalize((ksl_quaternion_t*)(context_->q() + qIndex));
      qIndex += 4;
      dqIndex += 3;

    } else if(qCount == 7 && dqCount == 6) {
      // float case
      context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      context_->q()[qIndex++] -= context_->deltaq()[dqIndex++];
      ksl_subtract_dpiFromq((ksl_vec3_t*)(context_->deltaq() + dqIndex),
                            (ksl_quaternion_t*)(context_->q() + qIndex));
      ksl_quaternion_normalize((ksl_quaternion_t*)(context_->q() + qIndex));
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
double Shift::Substructure::qError() {
#if SHIFT_WITH_BLAS_
  return sqrt(cblas_ddot(dqCount_, deltaq_, 1, deltaq_, 1)) / dqCount_;
#else
  double sum = 0;
  for(unsigned int i = 0; i < dqCount_; i++) {
    sum += deltaq_[i] * deltaq_[i];
  }
  return sqrt(sum) / dqCount_;
#endif
}

/*!

*/
void Shift::Substructure::newtonRaphsonSolver() {
  memset(deltaq_, 0, dqCount_ * sizeof(double));
  /* cs <-  L^-1 cs */
  for(int i = 0; i < rank_; i++) {
    for(int j = 0; j < i; j++) {
      cs_[pr_12_[i]] -= J_[i * dqCount_ + j] * cs_[pr_12_[j]];
    }
  }
  /* deltaq <- U^-1 * (L^-1 * cs) */
  for(int i = rank_ - 1; i > -1; i--) {
    for(int j = rank_ - 1; j > i; j--) {
      cs_[pr_12_[i]] -= J_[i * dqCount_ + j] * deltaq_[pc_12_[j]];
    }
    deltaq_[pc_12_[i]] = cs_[pr_12_[i]] / J_[i * dqCount_ + i];
  }
}

/*!
\brief evaluate position level constraints for one substructure
\return RMS constraint error
*/
double
Shift::Substructure::evaluateConstraints(const Shift::Topology::DHVA& dhva) {

  assert(constraints_.size() == chordConstraintCount_);
  for(unsigned int i = 0; i < chordConstraintCount_; i++) {
    ksl_SE3_t* parentFrame = dhva.d + constraints_[i].parent.d;
    ksl_SE3_t* childFrame = dhva.d + constraints_[i].child.d;
    ksl_SE3_t temp;
    ksl_product_ddinv(parentFrame, childFrame, &temp);
    memcpy(cs_ + i * 6, &temp.t, sizeof(ksl_vec3_t));
    double trace = temp.R.m00 + temp.R.m11 + temp.R.m22 - 3;
    cs_[i * 6 + 3] = 0.5 * (temp.R.m21 - temp.R.m12) + trace;
    cs_[i * 6 + 4] = 0.5 * (temp.R.m02 - temp.R.m20) + trace;
    cs_[i * 6 + 5] = 0.5 * (temp.R.m10 - temp.R.m01) + trace;
  }

#if SHIFT_WITH_BLAS_
  return sqrt(cblas_ddot(constraintEquationCount_, cs_, 1, cs_, 1)) /
         constraintEquationCount_;
#else
  double sum = 0;
  for(unsigned int i = 0; i < constraintEquationCount_; i++) {
    sum += cs_[i] * cs_[i];
  }
  return sqrt(sum) / constraintEquationCount_;
#endif
}

int Shift::Substructure::removeConstraintViolations() {
  /* Iteratively update x to satisfy constraint equations.

  dx[colDim] contains changes in x[colDim] for current N-R iteration.

  error = sqrt(cs[] . cs[]) / constraintEquationCount < eps when converged.
  */
  int maxStep = 100;
  double qCopy[qCount_];
  memcpy(qCopy, q_, qCount_ * sizeof(double));

  assert(context_);
  context_->updateDH();
  double initialError = evaluateConstraints(context_->dhva());
  fprintf(stdout, "  initial error: %g\n", initialError);

  int step = 0;
  double epsRankDetermination =
    context_->modelParameters().epsilonRankDetermination();
  double error = epsRankDetermination + 1;
  while(error > epsRankDetermination && step < maxStep) {
    setPO1(context_->dhva());
    newtonRaphsonSolver();
    updateq();
    context_->updateDH();
    error = evaluateConstraints(context_->dhva());
    ++step;
    fprintf(stdout, "  [%d] error: %g\n", step, error);
  }
  context_->updateDH();
  setBooleanPO2();
  if(step == maxStep) {
    printError(initialError, error, qCopy);
    return -1;
  } else {
    return 0;
  }
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables.

Does not set the jacobian boolean matrix, which is necessary for solving
the equations of motion.
*/
int Shift::Substructure::removeConstraintViolationsFast(
  bool useTightTolerance) {

  context_->updateDH();
  double initialError = evaluateConstraints(context_->dhva());
  double qCopy[qCount_];
  memcpy(qCopy, q_, qCount_ * sizeof(double));
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
    return -1;
  } else {
    return 0;
  }
}

void Shift::Substructure::printError(double initialError, double finalError,
                                     double* qCopy) {

  FILE* f = context_->modelParameters().debugFilePointer();
  if(!f) {
    f = stderr;
  }
  fprintf(f,
          "\n\n>>> Error: could not resolve constraint violations for "
          "substructure [%d]\n",
          index_);
  fprintf(f, "initial error: %f, final error: %f\n", initialError, finalError);
  fprintf(f, "  qCount: %d\n", qCount_);
  fprintf(f, "  qOffset: %d\n", qOffset_);
  assert(context_->q() + qOffset_ == q_);
  fprintf(f, " states before Newton-Raphson:\n   ");
  ksl_array_print(f, qCount_, qCopy);
  fprintf(f, " states after Newton-Raphson:\n    ");
  ksl_array_print(f, qCount_, q_);
}

/*!
@brief drive down constraint violations, then perform an initial full factor
to set up slider for repositioning mechanism

removes constraint violations and performs a specified
full factor in PO1, permutations are used for subsequent
fast factorization
*/
void Shift::Substructure::setupNewtonRaphsonSpecifiedCoordinate(
  Shift::Topology::DHVA& dhva, int specifiedCoordinateIndex) {
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
void Shift::Substructure::incrementSpecifiedCoordinate(
  Shift::Topology::DHVA& dhva, int specifiedCoordinateIndex, double value,
  MoveType moveType) {

  /* currently the joint type is not used, but may be used in the future, for
  example to set all 3 dof as specified for a spherical joint */
  memset(deltaq_, 0, dqCount_ * sizeof(double));
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
      std::cerr << "case not handled in incrementSpecifiedCoordinate()"
                << std::endl;
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
void Shift::Substructure::setBMatrix() {
  ksl_linalg_lu_setBMatrix_rmo(rank_, dqCount_, J_);
}

/*! @brief set initial row-major ordered jacobian in permutation order 1 and
perform an initial call to GCP using ksl_linalg_lu_full for one substructure
with a specified independent coordinate

@param context [in] s2_context_t data structure
@param substructureIndex [in] index of substructure
@param dqIndex [in] global dqIndex of specified joint

the Jacobian is stored in row-major order
*/
void Shift::Substructure::setPO1Specified(const Shift::Topology::DHVA& dhva,
                                          int dqIndex) {

  memset(J_, 0, dqCount_ * constraintEquationCount_ * sizeof(double));

  /* j is local loop index */
  auto it = positiveLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto& variableIndex : *it) {
      // std::cout << variableIndex << std::endl;
      // ksl_screw_print(stdout, dhva.h + variableIndex);
      ksl_screw_copy_rmo(dhva.h + variableIndex, J_, j * 6,
                         variableIndex - dqOffset_, dqCount_);
    }
  }

  it = negativeLoopMembersCompressed_->begin() + chordConstraintOffset_;
  for(unsigned int j = 0; j < chordConstraintCount_; j++, it++) {
    for(auto& variableIndex : *it) {
      // std::cout << variableIndex << std::endl;
      // ksl_screw_print(stdout, dhva.h + variableIndex);
      ksl_screw_ncopy_rmo(dhva.h + variableIndex, J_, j * 6,
                          variableIndex - dqOffset_, dqCount_);
    }
  }

  /* convert global index to local index */
  int localSpecifiedIndex = dqIndex - dqOffset_;
  assert(localSpecifiedIndex > -1);

  rank_ = ksl_linalg_lu_full_specified_rmo(
    constraintEquationCount_, dqCount_, J_,
    context_->modelParameters().epsilonRankDetermination(), pr_12_.data(),
    pc_12_.data(), localSpecifiedIndex);
  Topology::setInversePermutation(pr_12_, pr_21_);
  Topology::setInversePermutation(pc_12_, pc_21_);

  setBMatrix();

  scaledInitialInfinityNormB_ =
    context_->modelParameters().infinityNormMultiplier() * infinityNormB();

  setBooleanPO2();
}

double Shift::Substructure::infinityNormB() {
  return ksl_linalg_lu_infinityNormB_rmo(rank_, dqCount_, J_);
}

/*!
@brief vector copy B matrix into null space array
*/
void Shift::Substructure::copyNullSpace() {
  memset(rightNullSpace_, 0, dqCount_ * dqCount_ * sizeof(double));
  /* dqCount - rank is the column dimension of null space */
  assert((int)dqCount_ >= rank_ && rank_ != -1);
  int colDim = dqCount_ - rank_;
  for(int i = 0; i < rank_; i++) {
#if SHIFT_WITH_BLAS_
    cblas_dcopy(colDim, J_ + (i * dqCount_ + rank_), 1,
                rightNullSpace_ + (pc_12_[i] * colDim), 1);
#else
    for(int j = 0; j < colDim; j++) {
      rightNullSpace_[pc_12_[i] * colDim + j] = J_[i * dqCount_ + rank_ + j];
    }
#endif
  }

  /* set the remaining matrix to identity */
  unsigned int j = 0;
  for(unsigned int i = rank_; i < dqCount_; i++) {
    rightNullSpace_[pc_12_[i] * colDim + j++] = 1.0;
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
void Shift::Substructure::computeMobility() {
  if(dqCount_ - rank_ == 0) {
    printf("zero mobility detected for substructure\n");
    assert(mobilityNumbers_);
    memset(mobilityNumbers_, 0, dqCount_ * sizeof(double));
    return;
  }
  copyNullSpace();
  ksl_linalg_gramSchmidt(rightNullSpace_, dqCount_, dqCount_ - rank_);
  assert(mobilityNumbers_);
  assert((int)dqCount_ >= rank_ && rank_ != -1);
  int n = dqCount_ - rank_;
  for(unsigned int i = 0; i < dqCount_; i++) {
#if SHIFT_WITH_BLAS_
    mobilityNumbers_[i] =
      cblas_ddot(n, rightNullSpace_ + i * n, 1, rightNullSpace_ + i * n, 1);
#else
    mobilityNumbers_[i] = 0;
    for(int j = 0; j < n; j++) {
      mobilityNumbers_[i] +=
        rightNullSpace_[i * n + j] * rightNullSpace_[i * n + j];
    }
#endif
  }
}
