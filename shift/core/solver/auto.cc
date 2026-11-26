#include "shift/core/solver/auto.h"

#include <assert.h>

#include <cmath>
#include <sstream>

#include <lapacke.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"
#include "shift/core/solver/aba.h"
#include "shift/core/solver/dca.h"
#include "shift/core/tangents.h"

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
                   "divide-and-conquer algorithm -- default: 'aba'.", false,
                   {"aba", "dca"});
}

void shift::core::solver::Auto::Initialize() {
  assert(context_);
  const auto& trees = context_->get_trees();

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
    /* the velocities of the delegated trees, those of the joints and the
    elastic coordinates of their flexible bodies: RNCA leaves their rows and
    columns empty, and the tree solver solves them */
    for(const int l : tree_solver_->get_tree_dq()) {
      delegated_dq_[l] = true;
    }
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

bool shift::core::solver::Auto::supports_implicit() const {
  return true;
}

bool shift::core::solver::Auto::Residual(const double t,
                                         const std::vector<double>& y,
                                         const std::vector<double>& ddq,
                                         const std::vector<double>& la,
                                         std::vector<double>& r,
                                         std::vector<double>& dy) {
  if(RNCA::Residual(t, y, ddq, la, r, dy)) {
    return true;
  }
  if(!tree_solver_) {
    return false;
  }
  /* the forces and the motion of the state are those RNCA's Residual set */
  if(tree_solver_->SolveAccelerations()) {
    return true;
  }
  tree_solver_->TreeResidual(ddq, r);
  return false;
}

bool shift::core::solver::Auto::FactorIterationMatrix(
  const Tangents& tangents, double c, double k,
  const std::vector<int>& differenced, const std::vector<double>& k_columns,
  const std::vector<double>& c_columns) {
  assert(context_);
  has_coupled_matrix_ = false;
  if(RNCA::FactorIterationMatrix(tangents, c, k, differenced, k_columns,
                                 c_columns)) {
    return true;
  }
  implicit_dq_.clear();
  if(!tree_solver_) {
    has_coupled_matrix_ = true;
    return false;
  }
  implicit_dq_ = tree_solver_->get_tree_dq();
  const unsigned int n = context_->get_dq_count();
  const size_t nd = implicit_dq_.size();
  std::vector<int> position(n, -1);
  for(size_t m = 0; m < nd; m++) {
    position[implicit_dq_[m]] = static_cast<int>(m);
  }
  std::vector<bool> is_differenced(n, false);
  for(const int l : differenced) {
    is_differenced[l] = true;
  }

  /* c C + k K in the rows of the delegated variables (force_rows, nd by n)
  and in the rows of RNCA's variables and the columns of the delegated ones
  (a_rd, n by nd), from the tangents in the columns not differenced */
  std::vector<double> force_rows(nd * n, 0.0);
  std::vector<double> a_rd(size_t(n) * nd, 0.0);
  ForEachTangentEntry(
    *context_, tangents, c, k, [&](int row, int column, double value) {
      if(row < 0 || column < 0 || row >= static_cast<int>(n) ||
         column >= static_cast<int>(n) || is_differenced[column]) {
        return;
      }
      if(position[row] > -1) {
        force_rows[size_t(position[row]) * n + column] += value;
      } else if(position[column] > -1) {
        a_rd[size_t(row) * nd + position[column]] += value;
      }
    });

  /* G = M_d^-1 (c C + k K) + [I 0] in the rows of the delegated variables,
  the differenced columns from the differences of the residual (the
  accelerations' in these rows, the forces' in RNCA's) */
  std::vector<double> inverse_mass;
  if(tree_solver_->InverseMass(inverse_mass)) {
    return true;
  }
  std::vector<double> g(nd * n, 0.0);
  for(size_t i = 0; i < nd; i++) {
    for(size_t m = 0; m < nd; m++) {
      const double inverse = inverse_mass[i * nd + m];
      if(inverse == 0.0) {
        continue;
      }
      const double* row = &force_rows[m * n];
      for(unsigned int j = 0; j < n; j++) {
        g[i * n + j] += inverse * row[j];
      }
    }
    g[i * n + implicit_dq_[i]] += 1.0;
  }
  const size_t differenced_count = differenced.size();
  for(size_t m = 0; m < differenced_count; m++) {
    const int j = differenced[m];
    for(unsigned int i = 0; i < n; i++) {
      double value = 0.0;
      if(!k_columns.empty()) {
        value += k * k_columns[size_t(i) * differenced_count + m];
      }
      if(!c_columns.empty()) {
        value += c * c_columns[size_t(i) * differenced_count + m];
      }
      if(position[i] > -1) {
        g[size_t(position[i]) * n + j] =
          (static_cast<int>(i) == j ? 1.0 : 0.0) + value;
      } else if(position[j] > -1) {
        a_rd[size_t(i) * nd + position[j]] = value;
      }
    }
  }

  /* W = S_r^-1 A_rd, a column for each delegated variable with entries in
  RNCA's rows */
  const size_t size = n + get_multiplier_count();
  coupling_.assign(size * nd, 0.0);
  std::vector<double> column(size);
  for(size_t m = 0; m < nd; m++) {
    bool any = false;
    std::fill(column.begin(), column.end(), 0.0);
    for(unsigned int i = 0; i < n; i++) {
      column[i] = a_rd[size_t(i) * nd + m];
      any = any || column[i] != 0.0;
    }
    if(!any) {
      continue;
    }
    if(RNCA::SolveIterationMatrix(column)) {
      return true;
    }
    std::copy(column.begin(), column.end(), coupling_.begin() + m * size);
  }

  /* G_dr (the columns of RNCA's variables) and Z = G_dd - G_dr W */
  rows_dr_.assign(nd * n, 0.0);
  schur_.assign(nd * nd, 0.0);
  for(size_t i = 0; i < nd; i++) {
    for(unsigned int j = 0; j < n; j++) {
      if(position[j] < 0) {
        rows_dr_[i * n + j] = g[i * n + j];
      } else {
        schur_[i * nd + position[j]] = g[i * n + j];
      }
    }
  }
  for(size_t i = 0; i < nd; i++) {
    for(size_t m = 0; m < nd; m++) {
      double sum = 0.0;
      const double* w = &coupling_[m * size];
      for(unsigned int j = 0; j < n; j++) {
        sum += rows_dr_[i * n + j] * w[j];
      }
      schur_[i * nd + m] -= sum;
    }
  }
  schur_pivots_.resize(nd);
  if(nd > 0 &&
     LAPACKE_dgetrf(LAPACK_ROW_MAJOR, static_cast<int>(nd),
                    static_cast<int>(nd), schur_.data(), static_cast<int>(nd),
                    schur_pivots_.data()) != 0) {
    return true;
  }
  has_coupled_matrix_ = true;
  return false;
}

bool shift::core::solver::Auto::SolveIterationMatrix(
  std::vector<double>& rhs) {
  if(!has_coupled_matrix_) {
    return true;
  }
  const size_t nd = implicit_dq_.size();
  if(nd == 0) {
    return RNCA::SolveIterationMatrix(rhs);
  }
  const unsigned int n = context_->get_dq_count();
  const size_t size = rhs.size();
  if(size != n + get_multiplier_count() || coupling_.size() != size * nd) {
    return true;
  }
  /* y = S_r^-1 b_r, with the delegated rows of b_r zero */
  std::vector<double> b_d(nd);
  for(size_t m = 0; m < nd; m++) {
    b_d[m] = rhs[implicit_dq_[m]];
    rhs[implicit_dq_[m]] = 0.0;
  }
  if(RNCA::SolveIterationMatrix(rhs)) {
    return true;
  }
  /* Z x_d = b_d - G_dr y */
  for(size_t i = 0; i < nd; i++) {
    double sum = 0.0;
    for(unsigned int j = 0; j < n; j++) {
      sum += rows_dr_[i * n + j] * rhs[j];
    }
    b_d[i] -= sum;
  }
  if(LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'N', static_cast<int>(nd), 1,
                    schur_.data(), static_cast<int>(nd), schur_pivots_.data(),
                    b_d.data(), 1) != 0) {
    return true;
  }
  /* x_r = y - W x_d, and x_d */
  for(size_t m = 0; m < nd; m++) {
    const double x = b_d[m];
    if(x != 0.0) {
      const double* w = &coupling_[m * size];
      for(size_t i = 0; i < size; i++) {
        rhs[i] -= w[i] * x;
      }
    }
  }
  for(size_t m = 0; m < nd; m++) {
    rhs[implicit_dq_[m]] = b_d[m];
  }
  return false;
}
