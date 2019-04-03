#include <cfloat>
#include <cmath>

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
  int j = 0;
  for(auto it = positiveLoopMembersCompressedBegin_;
      it != positiveLoopMembersCompressedEnd_; it++, j++) {
    for(auto& variableIndex : *it) {
      std::cout << variableIndex << std::endl;
      ksl_screw_print(stdout, dhva.h + variableIndex);
      ksl_screw_copy_rmo(dhva.h + variableIndex, J_, j * 6,
                         variableIndex - dqOffset_, dqCount_);
    }
  }

  j = 0;
  for(auto it = negativeLoopMembersCompressedBegin_;
      it != negativeLoopMembersCompressedEnd_; it++, j++) {
    for(auto& variableIndex : *it) {
      std::cout << variableIndex << std::endl;
      ksl_screw_print(stdout, dhva.h + variableIndex);
      ksl_screw_ncopy_rmo(dhva.h + variableIndex, J_, j * 6,
                          variableIndex - dqOffset_, dqCount_);
    }
  }

  ksl_array2D_print(stdout, constraintEquationCount_, dqCount_, J_);

  rank_ = ksl_linalg_lu_full_rmo(constraintEquationCount_, dqCount_, J_,
                                 epsilonRankDetermination_, pr_12_.data(),
                                 pc_12_.data());
  assert(rank_ != -1);

  Shift::Topology::setInversePermutation(pr_12_, pr_21_);
  Shift::Topology::setInversePermutation(pc_12_, pc_21_);

  std::cout << "rank: " << rank_ << std::endl;
  std::cout << "dqCount: " << dqCount_ << std::endl;

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

  int j = 0; /* j is local loop index*/
  for(auto it = positiveLoopMembersCompressedBegin_;
      it != positiveLoopMembersCompressedEnd_; it++, j++) {
    for(auto& variableIndex : *it) {
      int columnIndex_2 = pc_21_[variableIndex - dqOffset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex_2 = pr_21_[j * 6 + m];
        J_[rowIndex_2 * dqCount_ + columnIndex_2] = dhva.h[variableIndex].at[m];
      }
    }
  }

  j = 0;
  for(auto it = negativeLoopMembersCompressedBegin_;
      it != negativeLoopMembersCompressedEnd_; it++, j++) {
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
  int j = 0;
  for(auto it = positiveLoopMembersCompressedBegin_;
      it != positiveLoopMembersCompressedEnd_; it++, j++) {
    for(auto& variableIndex : *it) {
      int columnIndex = pc_21_[variableIndex - dqOffset_];
      for(int m = 0; m < 6; m++) {
        int rowIndex = pr_21_[j * 6 + m];
        Jb_[rowIndex * dqCount_ + columnIndex] = true;
      }
    }
  }

  j = 0;
  for(auto it = negativeLoopMembersCompressedBegin_;
      it != negativeLoopMembersCompressedEnd_; it++, j++) {
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
  std::cout << "setBMatrixSparse() " << std::endl;
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
          std::cout << "  b3Index: " << b3Index << std::endl;
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
    std::cout << "dqCount: " << dqCount_ << " dqOffset:" << dqOffset_
              << " rb3Index: " << rb3Index << std::endl;
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
  std::cout << "i2: " << std::flush;
  for(unsigned int i3 = dqCount_ - rank_; i3 < dqCount_; i3++) {
    int i2 = pv_local_23[i3];
    std::cout << i2 << " " << std::flush;
    assert(i2 > -1 && i2 < (int)dqCount_);
    for(unsigned int j3 = 0; j3 < dqCount_ - rank_; j3++) {
      int j2 = pv_local_23[j3];
      assert(j2 > -1 && j2 < (int)dqCount_);
      ijMapping_[i2 * dqCount_ + j2] = ijMapping_temp_po3[i3 * dqCount_ + j3];
    }
  }
  std::cout << std::endl;
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
