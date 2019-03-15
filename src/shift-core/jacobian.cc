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

  auto substructureIndices = context_->getSubstructureIndices();
  substructures_.clear();
  substructures_.resize(substructureIndices.size());

  // get total size of Jacobian
  jacobianSize_ = 0;
  constraintEquationCount_ = 0;
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  unsigned int jointOffset = 0;
  unsigned int jacobianSize = 0;
  unsigned int constraintEquationCount = 0;
  for(unsigned int i = 0; i < substructureIndices.size(); i++) {
    substructures_[i].setIndex(i);
    unsigned int qCount = 0;
    unsigned int dqCount = 0;
    unsigned int jointCount = 0;
    unsigned int chordConstraintCount = 0;
    for(unsigned int j = 0; j < substructureIndices[i].size(); j++) {
      /* iterate over loops in substructure */
      auto loopMembers = context_->getLoopMembers();
      for(unsigned int k = 1; k < loopMembers[j].size(); k++) {
        /* iterate over loop members in loop */
        auto joint = context_->getActiveJoints().at(loopMembers[j][k]);
        qCount += joint->getqCount();
        dqCount += joint->getdqCount();
        jointCount += (joint->getqCount() > 0) ? 1 : 0;
      }
      chordConstraintCount++;
    }
    substructures_[i].setJointOffset(jointOffset);
    qOffset += qCount;
    dqOffset += dqCount;
    jointOffset += jointCount;
    jacobianSize += chordConstraintCount * 6 * dqCount;
    constraintEquationCount += 6 * chordConstraintCount;
    substructures_[i].setqCount(qCount);
    substructures_[i].setdqCount(dqCount);
    substructures_[i].setJointCount(jointCount);
    substructures_[i].setChordConstraintCount(chordConstraintCount);
  }

  J_ = new double[jacobianSize]{};
  Jxx_ = new int[jacobianSize]{};
  Jb_ = new bool[jacobianSize]{};
  cs_ = new double[constraintEquationCount]{};
  dcs_ = new double[constraintEquationCount]{};

  jacobianSize = 0;
  unsigned int constraintEquationOffset = 0;

  for(auto& substructure : substructures_) {

    substructure.setJ(J_ + jacobianSize);
    substructure.setJxx(Jxx_ + jacobianSize);
    substructure.setJb(Jb_ + jacobianSize);

    jacobianSize +=
      substructure.getConstraintEquationCount() * substructure.getdqCount();

    substructure.setcs(cs_ + constraintEquationOffset);
    substructure.setdcs(dcs_ + constraintEquationOffset);

    constraintEquationOffset += substructure.getConstraintEquationCount();
  }

  isDependentPO1_.resize(dqOffset);
  isDependentPO3_.resize(dqOffset);

  pv_12_.resize(dqOffset);
  pv_21_.resize(dqOffset);
  pv_13_.resize(dqOffset);
  pv_31_.resize(dqOffset);
  pv_23_.resize(dqOffset);
  pv_32_.resize(dqOffset);
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
      pv_12_[substructure.getdqOffset() + j] =
        substructure.getpc_12()[j] + substructure.getdqOffset();
    }
  }

  Shift::Topology::setInversePermutation(pv_12_, pv_21_);

  /* determine which variables are independent / dependent */
  for(auto& substructure : substructures_) {
    for(int j = 0; j < substructure.getRank(); j++) {
      isDependentPO1_[pv_12_[substructure.getdqOffset() + j]] = true;
    }
  }

  /* set p_v_13 */
  for(auto& substructure : substructures_) {
    int k = substructure.getdqOffset();
    for(unsigned int j = substructure.getdqOffset();
        j < substructure.getdqOffset() + substructure.getdqCount(); j++) {
      if(!isDependentPO1_[j]) {
        pv_13_[k++] = j;
      }
    }
  }

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
  int nnzb = 0; // number of nonzero entries in B matrix
  int dqCount = 0;
  for(auto& substructure : substructures_) {
    substructure.setBOffset(bOffset);
    nnzb += substructure.getNonZeroElementsInB();
    bOffset += (substructure.getdqCount() - substructure.getRank()) *
               substructure.getRank();
    dqCount += substructure.getdqCount();
  }

  if(ijMapping_) {
    delete[] ijMapping_;
  }
  ijMapping_ = new int[jacobianSize_]{};

  int jacobianOffset = 0;
  for(auto& substructure : substructures_) {
    substructure.setijMapping(ijMapping_ + jacobianOffset);
    jacobianOffset +=
      substructure.getdqCount() * substructure.getConstraintEquationCount();
  }

  if(b3_) {
    delete[] b3_;
  }
  b3_ = new double[bOffset]{};

  if(cb3_) {
    delete[] cb3_;
  }
  cb3_ = new int[bOffset]{};

  ksl_array2Di_free(rb3_);
  rb3_ = ksl_array2Di_alloc(dqCount, 2);

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
    substructure.setBMatrixSparse();
  }
}

void Shift::Jacobian::setConstraintPointers() {}

void Shift::Jacobian::setSubstructurePointers() {}
