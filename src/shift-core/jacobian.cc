#include <assert.h>
#include <string.h>

#include "shift-core/context.h"
#include "shift-core/jacobian.h"
#include "shift-core/topology.h"

void Shift::Jacobian::initialize() {
  assert(context_);
  setLookup();
  mallocEquations();
  compressLoopMembers();
  setPO1(context_->getDHVA());
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2(context_->getDHVA());
  factorPO2();
  setBMatrixSparse();
  setConstraintPointers();
  setSubstructurePointers();
}

void Shift::Jacobian::analyze() {
  setPO1(context_->getDHVA());
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2(context_->getDHVA());
  factorPO2();
  setBMatrixSparse();
}

void Shift::Jacobian::setLookup() {
  jointLookup_.clear();
  jointLookup_.resize(context_->getActiveJointCount());

  dqLookup_.clear();
  dqLookup_.resize(context_->getActiveJointCount());

  qLookup_.clear();
  qLookup_.resize(context_->getActiveJointCount());

  dqLookupCompressed_.clear();
  dqLookupCompressed_.resize(context_->getNonRigidJointCount());

  int jointCount = 0;
  int dqCount = 0;
  int qCount = 0;
  unsigned int i = 0;
  for(auto& joint : context_->getActiveJoints()) {
    if(joint->getqCount() > 0) {
      jointLookup_[i] = jointCount;
      dqLookup_[i] = dqCount;
      qLookup_[i] = qCount;
      jointCount++;
    } else {
      jointLookup_[i] = -1;
      dqLookup_[i] = -1;
      qLookup_[i] = -1;
    }
    i++;
    dqCount += joint->getdqCount();
    qCount += joint->getqCount();
  }

  for(unsigned int j = 0; j < context_->getActiveJointCount(); j++) {
    if(jointLookup_[j] > -1) {
      dqLookupCompressed_[jointLookup_[j]] = dqLookup_[j];
    }
  }
}

void Shift::Jacobian::mallocEquations() {

  if(J_) {
    delete[] J_;
    J_ = NULL;
  }

  if(Jxx_) {
    delete[] Jxx_;
    Jxx_ = NULL;
  }

  if(Jb_) {
    delete[] Jb_;
    Jb_ = NULL;
  }

  if(cs_) {
    delete[] cs_;
    cs_ = NULL;
  }

  if(dcs_) {
    delete[] dcs_;
    dcs_ = NULL;
  }

  auto substructureMembers = context_->getSubstructureMembers();
  auto substructureIndices = context_->getSubstructureIndices();
  substructures_.clear();
  substructures_.resize(substructureMembers.size());

  // get total size of Jacobian
  jacobianSize_ = 0;
  constraintEquationCount_ = 0;
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  unsigned int jointOffset = 0;
  unsigned int constraintEquationCount = 0;
  for(unsigned int i = 0; i < substructureMembers.size(); i++) {
    substructures_[i].setIndex(i);
    unsigned int qCount = 0;
    unsigned int dqCount = 0;
    unsigned int jointCount = 0;
    unsigned int chordConstraintCount = 0;
    for(unsigned int j = 1; j < substructureMembers[i].size(); j++) {
      auto joint = context_->getActiveJoints()[substructureMembers[i][j]];
      qCount += joint->getqCount();
      dqCount += joint->getdqCount();
      jointCount += (joint->getqCount() > 0) ? 1 : 0;
    }

    chordConstraintCount += substructureIndices[i].size();
    substructures_[i].setqCount(qCount);
    substructures_[i].setdqCount(dqCount);
    substructures_[i].setJointCount(jointCount);
    substructures_[i].setChordConstraintCount(chordConstraintCount);
    substructures_[i].setJointOffset(jointOffset);

    // theses offsets do not account for joints that are not a member of any
    // substructure
    qOffset += qCount;
    dqOffset += dqCount;
    jointOffset += jointCount;
    jacobianSize_ += chordConstraintCount * 6 * dqCount;
  }

  J_ = new double[jacobianSize_]{};
  Jxx_ = new int[jacobianSize_]{};
  Jb_ = new bool[jacobianSize_]{};
  cs_ = new double[constraintEquationCount]{};
  dcs_ = new double[constraintEquationCount]{};

  int jacobianOffset = 0;
  unsigned int constraintEquationOffset = 0;

  for(auto& substructure : substructures_) {
    substructure.setJ(J_ + jacobianOffset);
    substructure.setJxx(Jxx_ + jacobianOffset);
    substructure.setJb(Jb_ + jacobianOffset);

    jacobianOffset +=
      substructure.getConstraintEquationCount() * substructure.getdqCount();

    substructure.setcs(cs_ + constraintEquationOffset);
    substructure.setdcs(dcs_ + constraintEquationOffset);

    constraintEquationOffset += substructure.getConstraintEquationCount();
  }

  unsigned int dqCount = context_->getdqCount();
  isDependentPO1_.resize(dqCount);
  std::fill(isDependentPO1_.begin(), isDependentPO1_.end(), false);
  isDependentPO3_.resize(dqCount);
  std::fill(isDependentPO3_.begin(), isDependentPO3_.end(), false);
  pv_12_.resize(dqCount);
  pv_21_.resize(dqCount);
  pv_13_.resize(dqCount);
  pv_31_.resize(dqCount);
  pv_23_.resize(dqCount);
}

void Shift::Jacobian::compressLoopMembers() {

  unsigned int chordConstraintCount = context_->getChordConstraintCount();

  /* j is local loop index for substructure
  k is global loop index */
  int k = 0;
  auto& positiveLoopMembers = context_->getPositiveLoopMembers();
  auto& negativeLoopMembers = context_->getNegativeLoopMembers();
  auto& activeJoints = context_->getActiveJoints();
  positiveLoopMembersCompressed_.clear();
  positiveLoopMembersCompressed_.resize(chordConstraintCount);
  negativeLoopMembersCompressed_.clear();
  negativeLoopMembersCompressed_.resize(chordConstraintCount);

  /* record offsets in positiveLoopMemberOffsets and
    negativeLoopMembersOffsets */
  k = 0; /* loop counter */
  auto& substructureMembers = context_->getSubstructureMembers();

  unsigned int i = 0;
  for(auto& substructure : substructures_) {
    /* set offset into global variable list */
    unsigned int begin = k;
    int substructureOffset =
      std::min(substructureMembers[i][1], substructureMembers[i][2]);
    substructure.setdqOffset(dqLookup_[substructureOffset], context_->getdq(),
                             context_->getDeltaq());
    substructure.setqOffset(qLookup_[substructureOffset], context_->getq());

    for(unsigned int j = 0; j < substructure.getChordConstraintCount();
        j++, k++) {
      for(auto positiveLoopMember : positiveLoopMembers[k]) {
        unsigned int dqCount = activeJoints[positiveLoopMember]->getdqCount();
        int dqOffset = dqLookup_[positiveLoopMember];
        for(unsigned int m = 0; m < dqCount; m++) {
          positiveLoopMembersCompressed_[k].push_back(dqOffset++);
        }
      }

      for(auto negativeLoopMember : negativeLoopMembers[k]) {
        unsigned int dqCount = activeJoints[negativeLoopMember]->getdqCount();
        int dqOffset = dqLookup_[negativeLoopMember];
        for(unsigned int m = 0; m < dqCount; m++) {
          negativeLoopMembersCompressed_[k].push_back(dqOffset++);
        }
      }
    }
    substructure.setPositiveLoopMembersCompressed(
      positiveLoopMembersCompressed_.begin() + begin,
      positiveLoopMembersCompressed_.begin() + k);
    substructure.setNegativeLoopMembersCompressed(
      negativeLoopMembersCompressed_.begin() + begin,
      negativeLoopMembersCompressed_.begin() + k);
    i++;
  }
}

void Shift::Jacobian::setPO1(const Shift::Topology::DHVA& dhva) {
  for(auto& substructure : substructures_) {
    substructure.setPO1(dhva);
  }
}

void Shift::Jacobian::translatePermutations() {
  unsigned int dqCount = context_->getdqCount();
  isDependentPO1_.assign(isDependentPO1_.size(), false);
  isDependentPO3_.assign(isDependentPO3_.size(), false);
  /* initialize permutation arrays */
  for(unsigned int i = 0; i < dqCount; i++) {
    pv_12_[i] = i;
    pv_21_[i] = i;
    pv_13_[i] = i;
    pv_31_[i] = i;
  }

  /* set pv_21 */
  for(auto& substructure : substructures_) {
    for(unsigned int j = 0; j < substructure.getdqCount(); j++) {
      std::cout << "substructure.getdqCount(): " << substructure.getdqCount()
                << std::endl;
      std::cout << "substructure.getdqOffset(): " << substructure.getdqOffset()
                << std::endl;
      pv_12_[substructure.getdqOffset() + j] =
        substructure.getpc_12()[j] + substructure.getdqOffset();
    }
  }

  std::cout << "pv_12_: ";
  for(auto i : pv_12_) {
    std::cout << i << " ";
  }
  std::cout << std::endl;
  Shift::Topology::setInversePermutation(pv_12_, pv_21_);
  std::cout << "pv_21_: ";
  for(auto i : pv_21_) {
    std::cout << i << " ";
  }
  std::cout << std::endl;
  /* determine which variables are independent / dependent */
  for(auto& substructure : substructures_) {
    std::cout << "substructure.dqOffset: " << substructure.getdqOffset()
              << std::endl;
    std::cout << "rank: " << substructure.getRank() << std::endl;
    for(int j = 0; j < substructure.getRank(); j++) {
      isDependentPO1_[pv_12_[substructure.getdqOffset() + j]] = true;
    }
  }

  std::cout << "isDependentPO1: ";
  for(auto i : isDependentPO1_) {
    std::cout << (i ? "1 " : "0 ");
  }
  std::cout << std::endl;

  /* set pv_13 */
  for(auto& substructure : substructures_) {
    int k = substructure.getdqOffset();
    for(unsigned int j = substructure.getdqOffset();
        j < substructure.getdqOffset() + substructure.getdqCount(); j++) {
      if(!isDependentPO1_[j]) {
        pv_13_[k++] = j;
      }
    }

    for(unsigned int j = substructure.getdqOffset();
        j < substructure.getdqOffset() + substructure.getdqCount(); j++) {
      if(isDependentPO1_[j]) {
        pv_13_[k++] = j;
      }
    }
  }

  std::cout << "pv_13_:" << std::endl;
  for(auto i : pv_13_) {
    std::cout << i << " ";
  }
  std::cout << std::endl;

  /* set p_v_31 */
  Shift::Topology::setInversePermutation(pv_13_, pv_31_);

  /* set isDependentPO3 */
  for(unsigned int i = 0; i < dqCount; i++) {
    isDependentPO3_[pv_31_[i]] = isDependentPO1_[i];
  }

  /* set pv_23 */
  for(unsigned int i = 0; i < dqCount; i++) {
    pv_23_[i] = pv_21_[pv_13_[i]];
  }

  /* set pv_32 */
  Shift::Topology::setInversePermutation(pv_23_, pv_32_);
}

void Shift::Jacobian::setBooleanPO2() {
  for(auto& substructure : substructures_) {
    substructure.setBooleanPO2();
  }
}

void Shift::Jacobian::evaluateSparsity() {

  int bOffset = 0;
  // int nnzb = 0; // number of nonzero entries in B matrix
  int dqCount = context_->getqCount();
  for(auto& substructure : substructures_) {
    substructure.setBOffset(bOffset);
    // nnzb += substructure.getNonZeroElementsInB();
    bOffset += (substructure.getdqCount() - substructure.getRank()) *
               substructure.getRank();
  }

  if(ijMapping_) {
    delete[] ijMapping_;
  }
  ijMapping_ = new int[jacobianSize_]();

  int jacobianOffset = 0;
  for(auto& substructure : substructures_) {
    substructure.setijMapping(ijMapping_ + jacobianOffset);
    jacobianOffset +=
      substructure.getdqCount() * substructure.getConstraintEquationCount();
  }

  if(b3_) {
    delete[] b3_;
  }
  b3_ = new double[bOffset]();

  if(cb3_) {
    delete[] cb3_;
  }
  cb3_ = new int[bOffset]();

  ksl_array2Di_free(rb3_);
  rb3_ = ksl_array2Di_alloc(dqCount, 2);

  for(auto& substructure : substructures_) {
    substructure.print();
  }
  std::cout << "pv_23_: ";
  for(auto i : pv_23_) {
    std::cout << i << " ";
  }
  std::cout << std::endl;

  for(auto& substructure : substructures_) {
    substructure.evaluateSparsity(rb3_, cb3_, pv_23_);
  }
}

void Shift::Jacobian::setPO2(const Shift::Topology::DHVA& dhva) {
  for(auto& substructure : substructures_) {
    substructure.setPO2(dhva);
  }
}

void Shift::Jacobian::factorPO2() {
  for(auto& substructure : substructures_) {
    substructure.factorPO2();
  }
}

void Shift::Jacobian::setBMatrixSparse() {
  for(auto& substructure : substructures_) {
    substructure.setBMatrixSparse(b3_);
  }
}

void Shift::Jacobian::setConstraintPointers() {}

void Shift::Jacobian::setSubstructurePointers() {}
