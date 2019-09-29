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
  setPO1(context_->dhva()); // records initial infinity norm of B
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2(context_->dhva());
  factorPO2();
  setBMatrixSparse();
}

void Shift::Jacobian::mallocEquations() {

  auto substructure_members = context_->substructureMembers();
  auto substructure_indices = context_->substructureIndices();
  substructures_.clear();
  substructures_.resize(substructure_members.size());

  // get total size of Jacobian
  jacobian_size_ = 0;
  constraint_equation_count_ = 0;
  unsigned int q_offset = 0;
  unsigned int dq_offset = 0;
  unsigned int joint_offset = 0;
  for(unsigned int i = 0; i < substructure_members.size(); i++) {
    substructures_[i].setIndex(i);
    unsigned int q_count = 0;
    unsigned int dq_count = 0;
    unsigned int joint_count = 0;
    unsigned int chord_constraint_count = 0;
    for(unsigned int j = 1; j < substructure_members[i].size(); j++) {
      auto joint = context_->activeJoints()[substructure_members[i][j]];
      q_count += joint->qCount();
      dq_count += joint->dqCount();
      joint_count += (joint->qCount() > 0) ? 1 : 0;
    }

    chord_constraint_count += substructure_indices[i].size();
    substructures_[i].setqCount(q_count);
    substructures_[i].setdqCount(dq_count);
    substructures_[i].setJointCount(joint_count);
    substructures_[i].setChordConstraintCount(chord_constraint_count);
    substructures_[i].setJointOffset(joint_offset);
    substructures_[i].setContext(context_);

    // theses offsets do not account for joints that are not a member of any
    // substructure
    q_offset += q_count;
    dq_offset += dq_count;
    joint_offset += joint_count;
    jacobian_size_ += chord_constraint_count * 6 * dq_count;
    constraint_equation_count_ += chord_constraint_count * 6;
  }

  J_.resize(jacobian_size_);
  std::fill(J_.begin(), J_.end(), 0);

  Js_.resize(jacobian_size_);
  std::fill(Js_.begin(), Js_.end(), 0);

  Jxx_.resize(jacobian_size_);
  std::fill(Jxx_.begin(), Jxx_.end(), 0);

  Jb_.resize(jacobian_size_);
  std::fill(Jb_.begin(), Jb_.end(), 0);

  cs_.resize(constraint_equation_count_);
  std::fill(cs_.begin(), cs_.end(), 0);

  dcs_.resize(constraint_equation_count_);
  std::fill(dcs_.begin(), dcs_.end(), 0);

  int jacobian_offset = 0;
  unsigned int constraint_equation_offset = 0;

  for(auto& substructure : substructures_) {
    substructure.setJ(J_.data() + jacobian_offset);
    substructure.setJxx(Jxx_.data() + jacobian_offset);
    substructure.setJb();

    jacobian_offset +=
      substructure.constraintEquationCount() * substructure.qCount();

    substructure.setcs(cs_.data() + constraint_equation_offset);
    substructure.setdcs(dcs_.data() + constraint_equation_offset);

    constraint_equation_offset += substructure.constraintEquationCount();
  }

  unsigned int dq_count = context_->dqCount();
  is_dependent_po1_.resize(dq_count);
  std::fill(is_dependent_po1_.begin(), is_dependent_po1_.end(), false);
  is_dependent_po3_.resize(dq_count);
  std::fill(is_dependent_po3_.begin(), is_dependent_po3_.end(), false);
  pv_12_.resize(dq_count);
  pv_21_.resize(dq_count);
  pv_13_.resize(dq_count);
  pv_31_.resize(dq_count);
  pv_23_.resize(dq_count);
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
        unsigned int dqCount = activeJoints[positiveLoopMember]->dqCount();
        int dqOffset = dqLookup[positiveLoopMember];
        for(unsigned int m = 0; m < dqCount; m++) {
          positive_loop_members_compressed_[k].push_back(dqOffset++);
        }
      }

      for(auto negativeLoopMember : negativeLoopMembers[k]) {
        unsigned int dqCount = activeJoints[negativeLoopMember]->dqCount();
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

void Shift::Jacobian::setPO1() {
  for(auto& substructure : substructures_) {
    substructure.setPO1(context_->dhva());
  }
}

void Shift::Jacobian::translatePermutations() {
  unsigned int dq_count = context_->dqCount();
  is_dependent_po1_.assign(is_dependent_po1_.size(), false);
  is_dependent_po3_.assign(is_dependent_po3_.size(), false);
  /* initialize permutation arrays */
  for(unsigned int i = 0; i < dq_count; i++) {
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
  for(unsigned int i = 0; i < dq_count; i++) {
    is_dependent_po3_[pv_31_[i]] = is_dependent_po1_[i];
  }

  /* set pv_23 */
  for(unsigned int i = 0; i < dq_count; i++) {
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

  int b_offset = 0;
  // int nnzb = 0; // number of nonzero entries in B matrix
  int dq_count = context_->dqCount();
  for(auto& substructure : substructures_) {
    substructure.setBOffset(b_offset);
    // nnzb += substructure.getNonZeroElementsInB();
    b_offset +=
      (substructure.qCount() - substructure.rank()) * substructure.rank();
  }

  ij_mapping_.resize(jacobian_size_);
  std::fill(ij_mapping_.begin(), ij_mapping_.end(), 0);

  int jacobian_offset = 0;
  for(auto& substructure : substructures_) {
    substructure.setijMapping(ij_mapping_.data() + jacobian_offset);
    jacobian_offset +=
      substructure.qCount() * substructure.constraintEquationCount();
  }

  b3_.resize(b_offset);
  std::fill(b3_.begin(), b3_.end(), 0);

  cb3_.resize(b_offset);
  std::fill(cb3_.begin(), cb3_.end(), 0);

  rb3_.resize(dq_count, 2);
  rb3_.zero();

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

void Shift::Jacobian::setPO2() {
  for(auto& substructure : substructures_) {
    substructure.setPO2(context_->dhva());
  }
}

void Shift::Jacobian::factorPO2() {
  for(auto& substructure : substructures_) {
    substructure.factorPO2();
  }
}

void Shift::Jacobian::setBMatrixSparse() {
  for(auto& substructure : substructures_) {
    substructure.setBMatrixSparse(b3_.data());
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
  unsigned int right_null_space_count = 0;
  for(auto& substructure : substructures_) {
    right_null_space_count += substructure.dqCount() * substructure.dqCount();
  }

  if(right_null_space_count) {
    /* allocate memory for rightNullSpace, allows additional storage for null
       space up to dqCount * dqCount in the event that the null space increases
       rank from initial starting configuration
     */
    right_null_space_.resize(right_null_space_count);
    std::fill(right_null_space_.begin(), right_null_space_.end(), 0);

    right_null_space_count = 0;
    for(auto& substructure : substructures_) {
      substructure.setRightNullSpace(right_null_space_.data() +
                                     right_null_space_count);
      substructure.setMobilityNumbers(context_->mobilityNumbers().data() +
                                      substructure.dqOffset());
      right_null_space_count += substructure.dqCount() * substructure.dqCount();
    }
  }
}

void Shift::Jacobian::evaluateConstraints() {
  for(auto& substructure : substructures_) {
    substructure.evaluateConstraints(context_->dhva());
  }
}

bool Shift::Jacobian::infinityNormCheck() {
  for(auto& substructure : substructures_) {
    if(substructure.infinityNormCheck()) {
      return true;
    }
  }
  return false;
}

bool Shift::Jacobian::infinityNormCheck(double infinity_norm_multiplier) {
  for(auto& substructure : substructures_) {
    if(substructure.infinityNormCheck(infinity_norm_multiplier)) {
      return true;
    }
  }
  return false;
}
