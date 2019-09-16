#include <assert.h>
#include <string.h>

#include "shift-core/constraint/chord.h"
#include "shift-core/context.h"
#include "shift-core/jacobian.h"
#include "shift-core/topology.h"

void Shift::Jacobian::initialize() {
  newton_raphson_failure_ = false;
  assert(context_);
  mallocEquations();
  compressLoopMembers();
  setPO1(context_->dhva());
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2(context_->dhva());
  factorPO2();
  setBMatrixSparse();
  setConstraintPointers();
  setSubstructurePointers();

  if(removeConstraintViolations()) {
    std::cerr << "Error: constraints not satisfied." << std::endl;
  };
}

void Shift::Jacobian::analyze() {
  setPO1(context_->dhva());
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2(context_->dhva());
  factorPO2();
  setBMatrixSparse();
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

  auto substructureMembers = context_->substructureMembers();
  auto substructureIndices = context_->substructureIndices();
  substructures_.clear();
  substructures_.resize(substructureMembers.size());

  // get total size of Jacobian
  jacobian_size_ = 0;
  constraint_equation_count_ = 0;
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
      auto joint = context_->activeJoints()[substructureMembers[i][j]];
      qCount += joint->qCount();
      dqCount += joint->qCount();
      jointCount += (joint->qCount() > 0) ? 1 : 0;
    }

    chordConstraintCount += substructureIndices[i].size();
    substructures_[i].setqCount(qCount);
    substructures_[i].setdqCount(dqCount);
    substructures_[i].setJointCount(jointCount);
    substructures_[i].setChordConstraintCount(chordConstraintCount);
    substructures_[i].setJointOffset(jointOffset);
    substructures_[i].setContext(context_);

    // theses offsets do not account for joints that are not a member of any
    // substructure
    qOffset += qCount;
    dqOffset += dqCount;
    jointOffset += jointCount;
    jacobian_size_ += chordConstraintCount * 6 * dqCount;
    constraintEquationCount += chordConstraintCount * 6;
  }

  J_ = new double[jacobian_size_]{};
  Jxx_ = new int[jacobian_size_]{};
  Jb_ = new bool[jacobian_size_]{};
  cs_ = new double[constraintEquationCount]{};
  dcs_ = new double[constraintEquationCount]{};

  int jacobianOffset = 0;
  unsigned int constraintEquationOffset = 0;

  for(auto& substructure : substructures_) {
    substructure.setJ(J_ + jacobianOffset);
    substructure.setJxx(Jxx_ + jacobianOffset);
    substructure.setJb(Jb_ + jacobianOffset);

    jacobianOffset +=
      substructure.constraintEquationCount() * substructure.qCount();

    substructure.setcs(cs_ + constraintEquationOffset);
    substructure.setdcs(dcs_ + constraintEquationOffset);

    constraintEquationOffset += substructure.constraintEquationCount();
  }

  unsigned int dqCount = context_->qCount();
  is_dependent_po1_.resize(dqCount);
  std::fill(is_dependent_po1_.begin(), is_dependent_po1_.end(), false);
  is_dependent_po3_.resize(dqCount);
  std::fill(is_dependent_po3_.begin(), is_dependent_po3_.end(), false);
  pv_12_.resize(dqCount);
  pv_21_.resize(dqCount);
  pv_13_.resize(dqCount);
  pv_31_.resize(dqCount);
  pv_23_.resize(dqCount);
}

void Shift::Jacobian::compressLoopMembers() {

  unsigned int chordConstraintCount = context_->chordConstraintCount();

  /* j is local loop index for substructure
  k is global loop index */
  int k = 0;
  auto& positiveLoopMembers = context_->positiveLoopMembers();
  auto& negativeLoopMembers = context_->negativeLoopMembers();
  auto& activeJoints = context_->activeJoints();
  positive_loop_members_compressed_.clear();
  positive_loop_members_compressed_.resize(chordConstraintCount);
  negative_loop_members_compressed_.clear();
  negative_loop_members_compressed_.resize(chordConstraintCount);
  auto& dqLookup = context_->dqLookup();
  auto& qLookup = context_->qLookup();

  /* record offsets in positiveLoopMemberOffsets and
    negativeLoopMembersOffsets */
  k = 0; /* loop counter */
  auto& substructureMembers = context_->substructureMembers();

  unsigned int i = 0;
  for(auto& substructure : substructures_) {
    /* set offset into global variable list */
    int substructureOffset =
      std::min(substructureMembers[i][1], substructureMembers[i][2]);
    substructure.setdqOffset(dqLookup[substructureOffset], context_->dq(),
                             context_->deltaq().data());
    substructure.setqOffset(qLookup[substructureOffset], context_->q().data());

    for(unsigned int j = 0; j < substructure.chordConstraintCount(); j++, k++) {
      for(auto positiveLoopMember : positiveLoopMembers[k]) {
        unsigned int dqCount = activeJoints[positiveLoopMember]->qCount();
        int dqOffset = dqLookup[positiveLoopMember];
        for(unsigned int m = 0; m < dqCount; m++) {
          positive_loop_members_compressed_[k].push_back(dqOffset++);
        }
      }

      for(auto negativeLoopMember : negativeLoopMembers[k]) {
        unsigned int dqCount = activeJoints[negativeLoopMember]->qCount();
        int dqOffset = dqLookup[negativeLoopMember];
        for(unsigned int m = 0; m < dqCount; m++) {
          negative_loop_members_compressed_[k].push_back(dqOffset++);
        }
      }
    }

    i++;
  }

  unsigned int j = 0;
  for(auto& substructure : substructures_) {
    substructure.setChordConstraintOffset(j);
    substructure.setPositiveLoopMembersCompressed(
      &positive_loop_members_compressed_);
    substructure.setNegativeLoopMembersCompressed(
      &negative_loop_members_compressed_);
    j += substructure.chordConstraintCount();
  }
}

void Shift::Jacobian::setPO1(const Shift::Topology::DHVA& dhva) {
  for(auto& substructure : substructures_) {
    substructure.setPO1(dhva);
  }
}

void Shift::Jacobian::translatePermutations() {
  unsigned int dqCount = context_->qCount();
  is_dependent_po1_.assign(is_dependent_po1_.size(), false);
  is_dependent_po3_.assign(is_dependent_po3_.size(), false);
  /* initialize permutation arrays */
  for(unsigned int i = 0; i < dqCount; i++) {
    pv_12_[i] = i;
    pv_21_[i] = i;
    pv_13_[i] = i;
    pv_31_[i] = i;
  }

  /* set pv_21 */
  for(auto& substructure : substructures_) {
    for(unsigned int j = 0; j < substructure.qCount(); j++) {
      // std::cout << "substructure.qCount(): " << substructure.qCount()
      //           << std::endl;
      // std::cout << "substructure.dqOffset(): " << substructure.dqOffset()
      //           << std::endl;
      pv_12_[substructure.dqOffset() + j] =
        substructure.pc_12()[j] + substructure.dqOffset();
    }
  }

  Shift::Topology::setInversePermutation(pv_12_, pv_21_);

  /* determine which variables are independent / dependent */
  for(auto& substructure : substructures_) {
    // std::cout << "substructure.dqOffset: " << substructure.dqOffset()
    //           << std::endl;
    // std::cout << "rank: " << substructure.rank() << std::endl;
    for(int j = 0; j < substructure.rank(); j++) {
      is_dependent_po1_[pv_12_[substructure.dqOffset() + j]] = true;
    }
  }

  /* set pv_13 */
  for(auto& substructure : substructures_) {
    int k = substructure.dqOffset();
    for(unsigned int j = substructure.dqOffset();
        j < substructure.dqOffset() + substructure.qCount(); j++) {
      if(!is_dependent_po1_[j]) {
        pv_13_[k++] = j;
      }
    }

    for(unsigned int j = substructure.dqOffset();
        j < substructure.dqOffset() + substructure.qCount(); j++) {
      if(is_dependent_po1_[j]) {
        pv_13_[k++] = j;
      }
    }
  }

  /* set p_v_31 */
  Shift::Topology::setInversePermutation(pv_13_, pv_31_);

  /* set isDependentPO3 */
  for(unsigned int i = 0; i < dqCount; i++) {
    is_dependent_po3_[pv_31_[i]] = is_dependent_po1_[i];
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
  int dqCount = context_->qCount();
  for(auto& substructure : substructures_) {
    substructure.setBOffset(bOffset);
    // nnzb += substructure.getNonZeroElementsInB();
    bOffset +=
      (substructure.qCount() - substructure.rank()) * substructure.rank();
  }

  if(ij_mapping_) {
    delete[] ij_mapping_;
  }
  ij_mapping_ = new int[jacobian_size_]();

  int jacobianOffset = 0;
  for(auto& substructure : substructures_) {
    substructure.setijMapping(ij_mapping_ + jacobianOffset);
    jacobianOffset +=
      substructure.qCount() * substructure.constraintEquationCount();
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

  // for(auto& substructure : substructures_) {
  //   substructure.print();
  // }

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

void Shift::Jacobian::setConstraintPointers() {
  unsigned int k = 0;
  for(auto& substructure : substructures_) {
    substructure.clearConstraintOffsets();
    for(unsigned int i = 0; i < substructure.chordConstraintCount(); i++, k++) {
      auto chordConstraint = context_->chordConstraints()[k];
      Shift::Substructure::Constraint constraintOffset;
      constraintOffset.parent.d = chordConstraint->parentFrame()->index();
      constraintOffset.parent.v = chordConstraint->parentBody()->vaIndex();
      constraintOffset.child.d = chordConstraint->childFrame()->index();
      constraintOffset.child.v = chordConstraint->childBody()->vaIndex();
      substructure.appendConstraintOffset(constraintOffset);
    }
  }
}

void Shift::Jacobian::setSubstructurePointers() {
  for(auto& joint : context_->activeJoints()) {
    joint->clearSubstructure();
  }
  for(auto& substructure : substructures_) {
    for(auto& joint : context_->activeJoints()) {
      if(joint->dqIndex() >= (int)substructure.dqOffset() &&
         joint->dqIndex() <
           (int)(substructure.dqOffset() + substructure.dqCount()) &&
         joint->dqCount() > 0) {
        joint->setSubstructure(&substructure);
      }
    }
  }
}

/*!
@brief This function removes constraints for the entire mechanism
using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)

@param context [in] an s2_context_t datastructure
*/
int Shift::Jacobian::removeConstraintViolations() {
  bool newton_raphson_failure_ = false;
  for(auto& substructure : substructures_) {
    if(substructure.removeConstraintViolations()) {
      newton_raphson_failure_ = true;
    }
  }
  // @TODO update GL reference frames once graphics are added back in
  // context_->updateGLReferenceFrames()
  return newton_raphson_failure_ ? -1 : 0;
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables. This function
requires that setJacobianPO1 has been called for every substructure.

@param bool useTightTolerance [in] indicates whether to use a tight error
tolerance for removing constraint violations.
*/
void Shift::Jacobian::removeConstraintViolationsFast(bool useTightTolerance) {
  for(auto& substructure : substructures_) {
    substructure.removeConstraintViolationsFast(useTightTolerance);
  }
}

void Shift::Jacobian::initializeMobility() {
  /* get count of size for single right null space array */
  unsigned int rightNullSpaceCount = 0;
  for(auto& substructure : substructures_) {
    rightNullSpaceCount += substructure.dqCount() * substructure.dqCount();
  }

  if(rightNullSpaceCount) {
    /* allocate memory for rightNullSpace, allows additional storage for null
       space up to dqCount * dqCount in the event that the null space increases
       rank from initial starting configuration
     */
    if(right_null_space_) {
      delete[] right_null_space_;
    }
    right_null_space_ = new double[rightNullSpaceCount]{};
    rightNullSpaceCount = 0;
    for(auto& substructure : substructures_) {
      substructure.setRightNullSpace(right_null_space_ + rightNullSpaceCount);
      substructure.setMobilityNumbers(context_->mobilityNumbers().data() +
                                      substructure.dqOffset());
      rightNullSpaceCount += substructure.dqCount() * substructure.dqCount();
    }
  }
}

void Shift::Jacobian::evaluateConstraints() {
  for(auto& substructure : substructures_) {
    substructure.evaluateConstraints(context_->dhva());
  }
}
