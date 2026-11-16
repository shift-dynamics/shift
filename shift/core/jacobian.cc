#include <assert.h>
#include <string.h>

#include <algorithm>

#include "shift/core/constraint/chord.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/jacobian.h"
#include "shift/core/topology.h"
#include "shift/core/log.h"

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
    substructure.Print();
  }
  if(RemoveConstraintViolations()) {
    SHIFT_ERROR("Error: constraints not satisfied.");
  }
  MakeVelocitiesConsistent();
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
  auto& substructure_members = context_->get_substructure_members();
  auto& substructure_indices = context_->get_substructure_indices();
  substructures_.clear();
  substructures_.resize(substructure_members.size());

  // get total size of Jacobian
  jacobian_size_ = 0;
  constraint_equation_count_ = 0;
  unsigned int joint_offset = 0;

  for(size_t i = 0; i < substructure_members.size(); i++) {
    substructures_[i].set_index(i);
    unsigned int q_count = 0;
    unsigned int dq_count = 0;
    unsigned int joint_count = 0;
    unsigned int chord_constraint_count = 0;
    for(size_t j = 1; j < substructure_members[i].size(); j++) {
      auto& joint = context_->get_active_joints()[substructure_members[i][j]];
      /* the elastic coordinates of a flexible child body follow the
      joint's, inside the substructure's range */
      const unsigned int elastic_count =
        context_->GetElasticCount(substructure_members[i][j]);
      q_count += joint->get_q_count() + elastic_count;
      dq_count += joint->get_dq_count() + elastic_count;
      joint_count += (joint->get_q_count() > 0) ? 1 : 0;
    }

    /* the elastic coordinates of a flexible head that its loops leave by
    nodes, just below the members' */
    const unsigned int head_elastic_count = HeadElasticCount(i);
    q_count += head_elastic_count;
    dq_count += head_elastic_count;
    substructures_[i].set_head_elastic_count(head_elastic_count);

    chord_constraint_count += substructure_indices[i].size();
    substructures_[i].set_q_count(q_count);
    substructures_[i].set_dq_count(dq_count);
    substructures_[i].set_joint_count(joint_count);
    substructures_[i].set_chord_constraint_count(chord_constraint_count);
    substructures_[i].set_joint_offset(joint_offset);
    substructures_[i].set_context(context_);

    // this offset does not account for joints that are not a member of any
    // substructure
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

  e_.resize(constraint_equation_count_);
  std::fill(e_.begin(), e_.end(), 0);

  de_.resize(constraint_equation_count_);
  std::fill(de_.begin(), de_.end(), 0);

  int jacobian_offset = 0;
  unsigned int constraint_equation_offset = 0;

  for(auto& substructure : substructures_) {
    substructure.set_J(J_.data() + jacobian_offset);
    substructure.set_Jxx(Jxx_.data() + jacobian_offset);
    substructure.set_Jb();

    substructure.set_e(e_.data() + constraint_equation_offset);
    substructure.set_de(de_.data() + constraint_equation_offset);

    jacobian_offset +=
      substructure.get_constraint_equation_count() * substructure.get_dq_count();
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

  auto& positive_loop_members = context_->get_positive_loop_members();
  auto& negative_loop_members = context_->get_negative_loop_members();
  positive_loop_members_compressed_.clear();
  positive_loop_members_compressed_.resize(chord_constraint_count);
  negative_loop_members_compressed_.clear();
  negative_loop_members_compressed_.resize(chord_constraint_count);
  positive_loop_columns_compressed_.clear();
  positive_loop_columns_compressed_.resize(chord_constraint_count);
  negative_loop_columns_compressed_.clear();
  negative_loop_columns_compressed_.resize(chord_constraint_count);
  auto& dq_lookup = context_->get_dq_lookup();
  auto& q_lookup = context_->get_q_lookup();

  int k = 0; // global loop index
             // j is local loop index for substructure
  
  auto& substructure_members = context_->get_substructure_members();

  for(size_t i = 0; i < substructures_.size(); i++) {
    auto& substructure = substructures_[i];
    
    /* set offset into global variable list, the first generalized
    coordinate of any member; members without generalized coordinates (rigid
    joints) have a lookup of -1 */
    int dq_offset = -1;
    int q_offset = -1;
    auto lowest = [](int index, int& offset) {
      if(index >= 0 && (offset < 0 || index < offset)) {
        offset = index;
      }
    };
    const auto& bodies = context_->get_active_articulated_bodies();
    for(size_t j = 1; j < substructure_members[i].size(); j++) {
      const int member = substructure_members[i][j];
      lowest(dq_lookup[member], dq_offset);
      lowest(q_lookup[member], q_offset);
      /* a flexible body on a rigid joint has elastic coordinates only */
      if(context_->GetElasticCount(member)) {
        lowest(bodies[member]->get_elastic_dq_index(), dq_offset);
        lowest(bodies[member]->get_elastic_q_index(), q_offset);
      }
    }
    if(substructure.get_head_elastic_count()) {
      const int head = substructure_members[i][0];
      const int head_dq = bodies[head]->get_elastic_dq_index();
      const int head_q = bodies[head]->get_elastic_q_index();
      /* the head's elastic coordinates must lie just below the members' */
      if((dq_offset >= 0 &&
          head_dq + (int)substructure.get_head_elastic_count() != dq_offset) ||
         (q_offset >= 0 &&
          head_q + (int)substructure.get_head_elastic_count() != q_offset)) {
        throw shift::core::Exception(
          "Jacobian: substructure " + std::to_string(i) +
          " hangs from nodes of flexible body '" + bodies[head]->get_name() +
          "', whose elastic coordinates are not next to its members' (more "
          "than one substructure hanging from the body by nodes is not "
          "handled yet)");
      }
      dq_offset = head_dq;
      q_offset = head_q;
    }
    assert(dq_offset >= 0 && q_offset >= 0);

    SHIFT_INFO("substructure: " + std::to_string(i));
    SHIFT_INFO("dq_offset: " + std::to_string(dq_offset));
    substructure.set_dq_offset(dq_offset, context_->get_dq(),
                               context_->get_delta_q().data());

    SHIFT_INFO("q_offset: " + std::to_string(q_offset));
    substructure.set_q_offset(q_offset, context_->get_q().data());

    assert(
      context_->get_q().data() + context_->get_q_count() == 
      context_->get_dq());

    for(unsigned int j = 0; j < substructure.get_chord_constraint_count(); 
        j++, k++) {
      
      CompressLoopSide(k, positive_loop_members[k],
                       context_->get_chord_constraints()[k]->get_parent_frame(),
                       positive_loop_members_compressed_[k],
                       positive_loop_columns_compressed_[k]);
      CompressLoopSide(k, negative_loop_members[k],
                       context_->get_chord_constraints()[k]->get_child_frame(),
                       negative_loop_members_compressed_[k],
                       negative_loop_columns_compressed_[k]);
      std::vector<int>* members[2] = {&positive_loop_members_compressed_[k],
                                      &negative_loop_members_compressed_[k]};
      std::vector<int>* columns[2] = {&positive_loop_columns_compressed_[k],
                                      &negative_loop_columns_compressed_[k]};
      std::vector<int> ancestor_members[2], ancestor_columns[2];
      AppendAncestorColumns(k, ancestor_members, ancestor_columns);
      for(int side = 0; side < 2; side++) {
        members[side]->insert(members[side]->end(),
                              ancestor_members[side].begin(),
                              ancestor_members[side].end());
        columns[side]->insert(columns[side]->end(),
                              ancestor_columns[side].begin(),
                              ancestor_columns[side].end());
        /* every column inside the substructure's range */
        for(int variable : *members[side]) {
          if(variable < dq_offset ||
             variable >= dq_offset + (int)substructure.get_dq_count()) {
            throw shift::core::Exception(
              "Jacobian: loop " + std::to_string(k) + " moves velocity " +
              std::to_string(variable) + ", outside substructure " +
              std::to_string(i));
          }
        }
      }
    }
  }

  unsigned int j = 0;
  for(auto& substructure : substructures_) {
    substructure.set_chord_constraint_offset(j);
    substructure.set_positive_loop_members_compressed(
      &positive_loop_members_compressed_);
    substructure.set_negative_loop_members_compressed(
      &negative_loop_members_compressed_);
    substructure.set_loop_columns_compressed(&positive_loop_columns_compressed_,
                                             &negative_loop_columns_compressed_);
    j += substructure.get_chord_constraint_count();
  }

#if SHIFT_LOG_LEVEL > 1 
  {
    std::stringstream ss;
    ss << "compressed loop members:\n";
    ss << "  negative loop members compressed:\n";
    int j = 0;
    for(auto& loop : negative_loop_members_compressed_) {
      ss << "    [" << j++ << "]: ";
      for(auto& i : loop) {
        ss << i << " ";
      }
      ss << std::endl;
    }

    ss << "  positive loop members compressed:\n";
    j = 0;
    for(auto& loop : positive_loop_members_compressed_) {
      ss << "   [" << j++ << "]: ";
      for(auto& i : loop) {
        ss << i << " ";
      }
      ss << std::endl;
    }

    SHIFT_TRACE(ss.str());
  }
#endif

}

const shift::core::Frame* shift::core::Jacobian::LoopExitFrame(
  int member, const std::vector<int>& side, const Frame* end_frame) const {
  const auto& pcl = context_->get_pcl();
  const auto& joints = context_->get_active_joints();
  for(int other : side) {
    if(pcl[other] == member) {
      return joints[other]->get_system_parent_frame();
    }
  }
  return end_frame;
}

void shift::core::Jacobian::CompressLoopSide(int loop,
                                             const std::vector<int>& side,
                                             const Frame* end_frame,
                                             std::vector<int>& members,
                                             std::vector<int>& columns) const {
  const auto& joints = context_->get_active_joints();
  const auto& dq_lookup = context_->get_dq_lookup();
  const auto& slots = context_->get_node_slots();
  const auto& bodies = context_->get_active_articulated_bodies();
  for(int member : side) {
    int dq_offset = dq_lookup[member];
    for(unsigned int m = 0; m < joints[member]->get_dq_count(); m++) {
      members.push_back(dq_offset++);
      columns.push_back(-1);
    }
    /* a flexible child body: its elastic coordinates move the loop by the
    influence coefficients of the node it leaves the body by (the frame of
    the next member down this side, or the chord's frame at the end of the
    side); a frame that is not a node moves with the reference frame, and
    the columns are zero */
    const unsigned int elastic_count = context_->GetElasticCount(member);
    if(!elastic_count) {
      continue;
    }
    const int slot = context_->GetNodeSlot(LoopExitFrame(member, side,
                                                         end_frame));
    if(slot < 0) {
      continue;
    }
    if(slots[slot].body != bodies[member]) {
      throw shift::core::Exception(
        "Jacobian: loop " + std::to_string(loop) + " leaves flexible body '" +
        bodies[member]->get_name() + "' by a node of another body");
    }
    const int elastic_dq_index = bodies[member]->get_elastic_dq_index();
    for(unsigned int m = 0; m < elastic_count; m++) {
      members.push_back(elastic_dq_index + (int)m);
      columns.push_back(slots[slot].he_offset + (int)m);
    }
  }
}

int shift::core::Jacobian::LoopAncestor(int loop,
                                        const Frame* exits[2]) const {
  const auto& pcl = context_->get_pcl();
  const auto& joints = context_->get_active_joints();
  const auto& bodies = context_->get_active_articulated_bodies();
  const auto chord = context_->get_chord_constraints()[loop];
  const std::vector<int>* sides[2] = {
    &context_->get_positive_loop_members()[loop],
    &context_->get_negative_loop_members()[loop]};
  const Frame* ends[2] = {chord->get_parent_frame(),
                          chord->get_child_frame()};
  int ancestor = -2;
  for(int side = 0; side < 2; side++) {
    exits[side] = ends[side];
    int side_ancestor = -1;
    if(sides[side]->empty()) {
      /* the chord's frame is on the ancestor itself */
      const auto body = ends[side]->get_parent_body();
      for(size_t j = 0; j < bodies.size(); j++) {
        if(bodies[j] == body) {
          side_ancestor = (int)j;
        }
      }
    } else {
      /* the member of the side that hangs from a body off the side */
      for(int member : *sides[side]) {
        if(std::find(sides[side]->begin(), sides[side]->end(), pcl[member]) ==
           sides[side]->end()) {
          side_ancestor = pcl[member];
          exits[side] = joints[member]->get_system_parent_frame();
        }
      }
    }
    if(ancestor == -2) {
      ancestor = side_ancestor;
    } else if(ancestor != side_ancestor) {
      throw shift::core::Exception("Jacobian: the sides of loop " +
                                   std::to_string(loop) +
                                   " do not hang from one body");
    }
  }
  return ancestor;
}

void shift::core::Jacobian::AppendAncestorColumns(
  int loop, std::vector<int> members[2], std::vector<int> columns[2]) const {
  const Frame* exits[2];
  const int ancestor = LoopAncestor(loop, exits);
  if(ancestor < 0 || !context_->GetElasticCount(ancestor)) {
    return;
  }
  const auto body = context_->get_active_articulated_bodies()[ancestor];
  const auto& slots = context_->get_node_slots();
  for(int side = 0; side < 2; side++) {
    /* a frame that is not a node moves with the reference frame */
    const int slot = context_->GetNodeSlot(exits[side]);
    if(slot < 0) {
      continue;
    }
    assert(slots[slot].body == body);
    for(unsigned int m = 0; m < body->get_elastic_count(); m++) {
      members[side].push_back(body->get_elastic_dq_index() + (int)m);
      columns[side].push_back(slots[slot].he_offset + (int)m);
    }
  }
}

unsigned int shift::core::Jacobian::HeadElasticCount(
  size_t substructure) const {
  const auto& substructure_members = context_->get_substructure_members();
  const auto& substructure_indices = context_->get_substructure_indices();
  const int head = substructure_members[substructure][0];
  if(head < 0 || !context_->GetElasticCount(head)) {
    return 0;
  }
  /* the loops are ordered by substructure */
  size_t first = 0;
  for(size_t i = 0; i < substructure; i++) {
    first += substructure_indices[i].size();
  }
  for(size_t k = first; k < first + substructure_indices[substructure].size();
      k++) {
    const Frame* exits[2];
    if(LoopAncestor((int)k, exits) != head) {
      continue;
    }
    for(const Frame* frame : exits) {
      if(context_->GetNodeSlot(frame) > -1) {
        return context_->GetElasticCount(head);
      }
    }
  }
  return 0;
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
    for(unsigned int j = 0; j < substructure.get_dq_count(); j++) {
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
        j < substructure.get_dq_offset() + substructure.get_dq_count(); j++) {
      if(!is_dependent_po1_[j]) {
        pv_13_[k++] = j;
      }
    }

    for(unsigned int j = substructure.get_dq_offset();
        j < substructure.get_dq_offset() + substructure.get_dq_count(); j++) {
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
      (substructure.get_dq_count() - substructure.get_rank()) * 
      substructure.get_rank();
  }

  ij_mapping_.resize(jacobian_size_);
  std::fill(ij_mapping_.begin(), ij_mapping_.end(), 0);

  int jacobian_offset = 0;
  for(auto& substructure : substructures_) {
    substructure.set_ij_mapping(ij_mapping_.data() + jacobian_offset);
    jacobian_offset +=
      substructure.get_dq_count() * substructure.get_constraint_equation_count();
  }

  b3_.resize(b_offset);
  std::fill(b3_.begin(), b3_.end(), 0);

  cb3_.resize(b_offset);
  std::fill(cb3_.begin(), cb3_.end(), 0);

  rb3_.resize(dq_count, 2);
  rb3_.zero();

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
      /* a chord frame at a node of a flexible body moves with its node
      slot */
      constraint_offset.parent.v =
        context_->GetFrameVaIndex(chord_constraint->get_parent_frame());

      constraint_offset.child.d = 
        chord_constraint->get_child_frame()->get_index();
      constraint_offset.child.v =
        context_->GetFrameVaIndex(chord_constraint->get_child_frame());

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
*/
unsigned int shift::core::Jacobian::RemoveConstraintViolations() {
  unsigned int failure = Jacobian::OK;
  for(auto& substructure : substructures_) {
    failure |= substructure.RemoveConstraintViolations();
  }
  // @TODO update GL reference frames once graphics are added back in
  // context_->updateGLReferenceFrames()
  newton_raphson_failure_ = 
    failure & shift::core::Jacobian::Failure::NEWTON_RAPHSON;
  return failure;
}

/*!
@brief This function advances the solution of a set of constraints
under user control of one or more of its independent variables. This function
requires that SetupJacobianPO1 has been called for every substructure.

@param bool use_tight_tolerance [in] indicates whether to use a tight error
tolerance for removing constraint violations.
*/
void shift::core::Jacobian::MakeVelocitiesConsistent() {
  for(auto& substructure : substructures_) {
    substructure.MakeVelocitiesConsistent(context_->get_dq());
  }
  context_->UpdateDHVA();
}

void shift::core::Jacobian::RemoveConstraintViolationsFast(bool use_tight_tolerance) {
  for(auto& substructure : substructures_) {
    substructure.RemoveConstraintViolationsFast(use_tight_tolerance);
  }
}

void shift::core::Jacobian::InitializeMobility() {
  /* get count of size for single right null space array */
  unsigned int right_null_space_count = 0;
  for(auto& substructure : substructures_) {
    right_null_space_count += substructure.get_dq_count() * substructure.get_dq_count();
  }

  if(right_null_space_count) {
    /* allocate memory for right_null_space_, allows additional storage for null
       space up to dq_count * dq_count in the event that the null space increases
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
  double e = 0.0;
  for(auto& substructure : substructures_) {
    e += substructure.EvaluateConstraints(context_->get_dhva());
  }
  return e;
}

double shift::core::Jacobian::EvaluateConstraintsNewtonRaphson() {
  double e = 0.0;
  for(auto& substructure : substructures_) {
    e += substructure.EvaluateConstraintsNewtonRaphson(context_->get_dhva());
  }
  return e;
}

unsigned int shift::core::Jacobian::CheckCondition(
    double time, double* q, const double* dq) {

  iterations_since_last_infinity_norm_failure_++;

  unsigned int failure = shift::core::Jacobian::Failure::OK;
  std::vector<unsigned int> substructure_failure_indices;
  for(auto& substructure : substructures_) {

    unsigned int substructure_failure = substructure.CheckCondition();
    if(substructure_failure & (
        shift::core::Jacobian::Failure::INFINITY_NORM_B | 
        shift::core::Jacobian::Failure::PIVOT_RATIO)) {
      substructure_failure_indices.push_back(substructure.get_index());
    }
    if(substructure_failure) {
      substructure_failure |= 
        substructure.RemoveConstraintViolationsRunTime(q, dq);
    }
    failure |= substructure_failure;
  }

  if(failure & shift::core::Jacobian::Failure::INFINITY_NORM_B) {
    infinity_norm_failures_.push_back({time, substructure_failure_indices});
    iterations_since_last_infinity_norm_failure_ = 0;
  }

  return failure;
}

unsigned int shift::core::Jacobian::CheckCondition(
    double time, double* q, const double* dq, const double infinity_norm_multiplier) {

  iterations_since_last_infinity_norm_failure_++;

  unsigned int failure = shift::core::Jacobian::Failure::OK;
  std::vector<unsigned int> substructure_failure_indices;
  for(auto& substructure : substructures_) {

    unsigned int substructure_failure = 
      substructure.CheckCondition(infinity_norm_multiplier);

    if(substructure_failure & (
        shift::core::Jacobian::Failure::INFINITY_NORM_B | 
        shift::core::Jacobian::Failure::PIVOT_RATIO)) {
      substructure_failure_indices.push_back(substructure.get_index());
    }
    if(substructure_failure) {
      substructure_failure |= 
        substructure.RemoveConstraintViolationsRunTime(q, dq);
    }
    failure |= substructure_failure;
  }

  if(failure & shift::core::Jacobian::Failure::INFINITY_NORM_B) {
    infinity_norm_failures_.push_back({time, substructure_failure_indices});
    iterations_since_last_infinity_norm_failure_ = 0;
  }

  return failure;
}

