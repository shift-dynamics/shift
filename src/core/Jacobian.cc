#include <assert.h>
#include <string.h>

#include "shift/core/constraint/Chord.h"
#include "shift/core/Context.h"
#include "shift/core/Jacobian.h"
#include "shift/core/Topology.h"

void shift::core::Jacobian::Initialize() {
  newton_raphson_failure_ = false;
  iterations_since_last_infinity_norm_failure_ = 0;
  infinity_norm_failures_.clear();
  assert(context_);
  MallocEquations();
  CompressLoopMembers();
  SetupPO1(context_->get_dhva());
  TranslatePermutations();
  SetupBooleanPO2();
  EvaluateSparsity();
  SetupPO2(context_->get_dhva());
  FactorPO2();
  SetupBMatrixSparse();
  SetupConstraintPointers();
  SetupSubstructurePointers();
  for(auto& substructure : substructures_) {
    substructure.Connect();
  }
  if(RemoveConstraintViolations()) {
    std::cerr << "Error: constraints not satisfied." << std::endl;
  }
}

void shift::core::Jacobian::Analyze() {
  SetupPO1(context_->get_dhva()); // records initial infinity norm of B
  TranslatePermutations();
  SetupBooleanPO2();
  EvaluateSparsity();
  SetupPO2(context_->get_dhva());
  FactorPO2();
  SetupBMatrixSparse();
}

void shift::core::Jacobian::MallocEquations() {
  auto substructure_members = context_->get_substructure_members();
  auto substructure_indices = context_->get_substructure_indices();
  substructures_.clear();
  substructures_.resize(substructure_members.size());

  // get total size of Jacobian
  jacobian_size_ = 0;
  constraint_equation_count_ = 0;
  unsigned int q_offset = 0;
  unsigned int dq_offset = 0;
  unsigned int joint_offset = 0;
  for(unsigned int i = 0; i < substructure_members.size(); i++) {
    substructures_[i].set_index(i);
    unsigned int q_count = 0;
    unsigned int dq_count = 0;
    unsigned int joint_count = 0;
    unsigned int chord_constraint_count = 0;
    for(unsigned int j = 1; j < substructure_members[i].size(); j++) {
      auto joint = context_->get_active_joints()[substructure_members[i][j]];
      q_count += joint->get_q_count();
      dq_count += joint->get_dq_count();
      joint_count += (joint->get_q_count() > 0) ? 1 : 0;
    }

    chord_constraint_count += substructure_indices[i].size();
    substructures_[i].set_q_count(q_count);
    substructures_[i].set_dq_count(dq_count);
    substructures_[i].set_joint_count(joint_count);
    substructures_[i].set_chord_constraint_count(chord_constraint_count);
    substructures_[i].set_joint_offset(joint_offset);
    substructures_[i].set_context(context_);

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
    substructure.set_J(J_.data() + jacobian_offset);
    substructure.set_Jxx(Jxx_.data() + jacobian_offset);
    substructure.set_Jb();

    jacobian_offset +=
      substructure.get_constraint_equation_count() * substructure.get_q_count();

    substructure.set_cs(cs_.data() + constraint_equation_offset);
    substructure.set_dcs(dcs_.data() + constraint_equation_offset);

    constraint_equation_offset += substructure.get_constraint_equation_count();
  }

  unsigned int dq_count = context_->get_dq_count();
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

void shift::core::Jacobian::CompressLoopMembers() {

  unsigned int chord_constraint_count = context_->get_chord_constraint_count();

  /* j is local loop index for substructure
  k is global loop index */
  int k = 0;
  auto& positive_loop_members = context_->get_positive_loop_members();
  auto& negative_loop_members = context_->get_negative_loop_members();
  auto& active_joints = context_->get_active_joints();
  positive_loop_members_compressed_.clear();
  positive_loop_members_compressed_.resize(chord_constraint_count);
  negative_loop_members_compressed_.clear();
  negative_loop_members_compressed_.resize(chord_constraint_count);
  auto& dq_lookup = context_->get_dq_lookup();
  auto& q_lookup = context_->get_q_lookup();

  /* record offsets in positiveLoopMemberOffsets and
    negative_loop_membersOffsets */
  k = 0; /* loop counter */
  auto& substructure_members = context_->get_substructure_members();

  unsigned int i = 0;
  for(auto& substructure : substructures_) {
    /* set offset into global variable list */
    int substructure_offset =
      std::min(substructure_members[i][1], substructure_members[i][2]);
    substructure.set_dq_offset(
      dq_lookup[substructure_offset], context_->get_dq(),
      context_->get_delta_q().data());
    substructure.set_q_offset(
      q_lookup[substructure_offset], context_->get_q().data());

    for(unsigned int j = 0; j < substructure.get_chord_constraint_count(); 
        j++, k++) {
      for(auto positive_loop_member : positive_loop_members[k]) {
        unsigned int dq_count = 
          active_joints[positive_loop_member]->get_dq_count();
        int dq_offset = dq_lookup[positive_loop_member];
        for(unsigned int m = 0; m < dq_count; m++) {
          positive_loop_members_compressed_[k].push_back(dq_offset++);
        }
      }

      for(auto negative_loop_member : negative_loop_members[k]) {
        unsigned int dq_count = 
          active_joints[negative_loop_member]->get_dq_count();
        int dq_offset = dq_lookup[negative_loop_member];
        for(unsigned int m = 0; m < dq_count; m++) {
          negative_loop_members_compressed_[k].push_back(dq_offset++);
        }
      }
    }

    i++;
  }

  unsigned int j = 0;
  for(auto& substructure : substructures_) {
    substructure.set_chord_constraint_offset(j);
    substructure.set_positive_loop_members_compressed(
      &positive_loop_members_compressed_);
    substructure.set_negative_loop_members_compressed(
      &negative_loop_members_compressed_);
    j += substructure.get_chord_constraint_count();
  }
}

void shift::core::Jacobian::SetupPO1(const shift::core::topology::DHVA& dhva) {
  for(auto& substructure : substructures_) {
    substructure.SetupPO1(dhva);
  }
}

void shift::core::Jacobian::SetupPO1() {
  for(auto& substructure : substructures_) {
    substructure.SetupPO1(context_->get_dhva());
  }
}

void shift::core::Jacobian::TranslatePermutations() {
  unsigned int dq_count = context_->get_dq_count();
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
    for(unsigned int j = 0; j < substructure.get_q_count(); j++) {
      pv_12_[substructure.get_dq_offset() + j] =
        substructure.get_pc12()[j] + substructure.get_dq_offset();
    }
  }

  shift::core::topology::SetInversePermutation(pv_12_, pv_21_);

  /* determine which variables are independent / dependent */
  for(auto& substructure : substructures_) {
    for(int j = 0; j < substructure.get_rank(); j++) {
      is_dependent_po1_[pv_12_[substructure.get_dq_offset() + j]] = true;
    }
  }

  /* set pv_13 */
  for(auto& substructure : substructures_) {
    int k = substructure.get_dq_offset();
    for(unsigned int j = substructure.get_dq_offset();
        j < substructure.get_dq_offset() + substructure.get_q_count(); j++) {
      if(!is_dependent_po1_[j]) {
        pv_13_[k++] = j;
      }
    }

    for(unsigned int j = substructure.get_dq_offset();
        j < substructure.get_dq_offset() + substructure.get_q_count(); j++) {
      if(is_dependent_po1_[j]) {
        pv_13_[k++] = j;
      }
    }
  }

  /* set p_v_31 */
  shift::core::topology::SetInversePermutation(pv_13_, pv_31_);

  /* set isDependentPO3 */
  for(unsigned int i = 0; i < dq_count; i++) {
    is_dependent_po3_[pv_31_[i]] = is_dependent_po1_[i];
  }

  /* set pv_23 */
  for(unsigned int i = 0; i < dq_count; i++) {
    pv_23_[i] = pv_21_[pv_13_[i]];
  }

  /* set pv_32 */
  shift::core::topology::SetInversePermutation(pv_23_, pv_32_);
}

void shift::core::Jacobian::SetupBooleanPO2() {
  for(auto& substructure : substructures_) {
    substructure.SetupBooleanPO2();
  }
}

void shift::core::Jacobian::EvaluateSparsity() {
  int b_offset = 0;
  // int nnzb = 0; // number of nonzero entries in B matrix
  int dq_count = context_->get_dq_count();
  for(auto& substructure : substructures_) {
    substructure.set_b_offset(b_offset);
    // nnzb += substructure.getNonZeroElementsInB();
    b_offset +=
      (substructure.get_q_count() - substructure.get_rank()) * 
      substructure.get_rank();
  }

  ij_mapping_.resize(jacobian_size_);
  std::fill(ij_mapping_.begin(), ij_mapping_.end(), 0);

  int jacobian_offset = 0;
  for(auto& substructure : substructures_) {
    substructure.set_ij_mapping(ij_mapping_.data() + jacobian_offset);
    jacobian_offset +=
      substructure.get_q_count() * substructure.get_constraint_equation_count();
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
    substructure.EvaluateSparsity(rb3_, cb3_, pv_23_);
  }
}

void shift::core::Jacobian::SetupPO2(const shift::core::topology::DHVA& dhva) {
  for(auto& substructure : substructures_) {
    substructure.SetupPO2(dhva);
  }
}

void shift::core::Jacobian::SetupPO2() {
  for(auto& substructure : substructures_) {
    substructure.SetupPO2(context_->get_dhva());
  }
}

void shift::core::Jacobian::FactorPO2() {
  for(auto& substructure : substructures_) {
    substructure.FactorPO2();
  }
}

void shift::core::Jacobian::SetupBMatrixSparse() {
  for(auto& substructure : substructures_) {
    substructure.SetupBMatrixSparse(b3_.data());
  }
}

void shift::core::Jacobian::SetupConstraintPointers() {
  unsigned int k = 0;
  for(auto& substructure : substructures_) {
    substructure.ClearConstraintOffsets();
    for(unsigned int i = 0; i < substructure.get_chord_constraint_count(); 
        i++, k++) {
      auto chord_constraint = context_->get_chord_constraints()[k];
      shift::core::Substructure::Constraint constraint_offset;
      constraint_offset.parent.d = 
        chord_constraint->get_parent_frame()->get_index();
      constraint_offset.parent.v = 
        chord_constraint->get_parent_body()->get_va_index();

      // std::cout << "parent: " << chordConstraint->parentFrame()->name()
      //           << " d: " << constraintOffset.parent.d
      //           << " v: " << constraintOffset.parent.v << std::endl;

      constraint_offset.child.d = 
        chord_constraint->get_child_frame()->get_index();
      constraint_offset.child.v = 
        chord_constraint->get_child_body()->get_va_index();

      // std::cout << "child: " << chordConstraint->childFrame()->name()
      //           << " d: " << constraintOffset.child.d
      //           << " v: " << constraintOffset.child.v << std::endl;

      substructure.AppendConstraintOffset(constraint_offset);
    }
  }
}

void shift::core::Jacobian::SetupSubstructurePointers() {
  for(auto& joint : context_->get_active_joints()) {
    joint->ClearSubstructure();
  }
  for(auto& substructure : substructures_) {
    for(auto& joint : context_->get_active_joints()) {
      if(joint->get_dq_index() >= (int)substructure.get_dq_offset() &&
         joint->get_dq_index() <
           (int)(substructure.get_dq_offset() + substructure.get_dq_count()) &&
         joint->get_dq_count() > 0) {
        joint->set_substructure(&substructure);
      }
    }
  }
}

/*!
@brief This function removes constraints for the entire mechanism
using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)

@param context [in] an s2_context_t datastructure
*/
int shift::core::Jacobian::RemoveConstraintViolations() {
  bool newton_raphson_failure_ = false;
  for(auto& substructure : substructures_) {
    if(substructure.RemoveConstraintViolations()) {
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
void shift::core::Jacobian::RemoveConstraintViolationsFast(bool useTightTolerance) {
  for(auto& substructure : substructures_) {
    substructure.RemoveConstraintViolationsFast(useTightTolerance);
  }
}

void shift::core::Jacobian::InitializeMobility() {
  /* get count of size for single right null space array */
  unsigned int right_null_space_count = 0;
  for(auto& substructure : substructures_) {
    right_null_space_count += substructure.get_dq_count() * substructure.get_dq_count();
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
      substructure.set_right_null_space(right_null_space_.data() +
                                        right_null_space_count);
      substructure.set_mobility_numbers(context_->get_mobility_numbers().data() +
                                        substructure.get_dq_offset());
      right_null_space_count += 
        substructure.get_dq_count() * substructure.get_dq_count();
    }
  }
}

double shift::core::Jacobian::EvaluateConstraints() {
  double e = 0;
  for(auto& substructure : substructures_) {
    e += substructure.EvaluateConstraints(context_->get_dhva());
  }
  return e;
}

unsigned int shift::core::Jacobian::InfinityNormCheck(double time, double* q,
                                                      const double* dq) {
  iterations_since_last_infinity_norm_failure_++;

  bool infinity_norm_pass = true;
  std::vector<unsigned int> substructure_failure_indices;
  for(auto& substructure : substructures_) {

    /* 0: infinity norm check passed
       1: infinity norm check failed, but newton raphson iteration was
       successfully applied to determine correct substructure rank. simulation
       can continue
       2: infinity norm check failed, and application of newton
       raphson iteration failed, simulation should terminate.
    */
    int return_value = substructure.InfinityNormCheck(q, dq);
    if(return_value == 1) {
      infinity_norm_pass = false;
      substructure_failure_indices.push_back(substructure.get_index());
    } else if(return_value == 2) {
      substructure_failure_indices.push_back(substructure.get_index());
      infinity_norm_failures_.push_back({time, substructure_failure_indices});
      return 2;
    }
  }

  if(!infinity_norm_pass) {
    infinity_norm_failures_.push_back({time, substructure_failure_indices});
    iterations_since_last_infinity_norm_failure_ = 0;
    return 1;
  }

  return 0;
}

unsigned int
shift::core::Jacobian::InfinityNormCheck(double time, double* q, const double* dq,
                                   double infinity_norm_multiplier) {
  iterations_since_last_infinity_norm_failure_++;

  bool infinity_norm_pass = true;
  std::vector<unsigned int> substructure_failure_indices;
  for(auto& substructure : substructures_) {
    /* 0: infinity norm check passed
       1: infinity norm check failed, but newton raphson iteration was
       successfully applied to determine correct substructure rank. simulation
       can continue
       2: infinity norm check failed, and application of newton
       raphson iteration failed, simulation should terminate.
    */
    int return_value =
      substructure.InfinityNormCheck(q, dq, infinity_norm_multiplier);
    if(return_value == 1) {
      infinity_norm_pass = false;
      substructure_failure_indices.push_back(substructure.get_index());
    } else if(return_value == 2) {
      substructure_failure_indices.push_back(substructure.get_index());
      infinity_norm_failures_.push_back({time, substructure_failure_indices});
      return 2;
    }
  }

  if(!infinity_norm_pass) {
    infinity_norm_failures_.push_back({time, substructure_failure_indices});
    iterations_since_last_infinity_norm_failure_ = 0;
    return 1;
  }

  return 0;
}
