#include "shift/core/solver/auto.h"

#include <assert.h>

#include <sstream>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"
#include "shift/core/solver/aba.h"
#include "shift/core/solver/dca.h"

bool shift::core::solver::Auto::registered_ =
  shift::core::Factory<shift::core::solver::Base>::get()->RegisterModule(
    "auto", shift::core::solver::Auto::Create);

std::unique_ptr<shift::core::solver::Auto>
shift::core::solver::Auto::Create() {
  return std::make_unique<shift::core::solver::Auto>();
}

void shift::core::solver::Auto::DeclareParameters() {
  DeclareParameter("tree_solver", &tree_solver_type_,
                   "Solver for the trees without closed loops: 'aba', the "
                   "articulated-body algorithm, or 'dca', the "
                   "divide-and-conquer algorithm -- default: 'aba'.",
                   false);
}

void shift::core::solver::Auto::Initialize() {
  assert(context_);
  const auto& trees = context_->get_trees();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();

  delegated_trees_.clear();
  delegated_body_.assign(context_->get_non_rigid_joint_count(), false);
  delegated_dq_.assign(context_->get_dq_count(), false);
  for(size_t t = 0; t < trees.size(); t++) {
    if(trees[t].kind == topology::Tree::Kind::kClosedLoop) {
      continue;
    }
    delegated_trees_.push_back(static_cast<int>(t));
    for(int j : trees[t].bodies) {
      delegated_body_[j] = true;
      for(unsigned int k = 0; k < joint_dof_compressed[j]; k++) {
        delegated_dq_[dq_lookup_compressed[j] + k] = true;
      }
    }
  }

  tree_solver_.reset();
  if(!delegated_trees_.empty()) {
    if(tree_solver_type_ == "aba") {
      tree_solver_ = std::make_unique<ABA>();
    } else if(tree_solver_type_ == "dca") {
      tree_solver_ = std::make_unique<DCA>();
    } else {
      throw shift::core::Exception("Solver 'auto': unknown tree_solver '" +
                                   tree_solver_type_ +
                                   "', expected 'aba' or 'dca'");
    }
    tree_solver_->set_context(context_);
    tree_solver_->set_trees(delegated_trees_);
    tree_solver_->Initialize();
  }

  {
    std::stringstream ss;
    ss << "Solver 'auto': " << delegated_trees_.size() << " of "
       << trees.size() << " trees solved by '" << tree_solver_type_
       << "', the others by 'rnca'";
    SHIFT_DEBUG(ss.str());
  }

  RNCA::Initialize();
}

bool shift::core::solver::Auto::SolveDelegated() {
  return tree_solver_ ? tree_solver_->SolveAccelerations() : false;
}

double shift::core::solver::Auto::GravitationalPotentialEnergy(
  shift::core::topology::DHVA& dhva) {
  return RNCA::GravitationalPotentialEnergy(dhva) +
         (tree_solver_ ? tree_solver_->GravitationalPotentialEnergy(dhva)
                       : 0.0);
}

double
shift::core::solver::Auto::KineticEnergy(shift::core::topology::DHVA& dhva) {
  return RNCA::KineticEnergy(dhva) +
         (tree_solver_ ? tree_solver_->KineticEnergy(dhva) : 0.0);
}
