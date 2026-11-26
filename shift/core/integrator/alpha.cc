#include "shift/core/integrator/alpha.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <lapacke.h>
#include <sstream>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/log.h"
#include "shift/core/solver/base.h"
#include "shift/core/tangents.h"

void shift::core::integrator::Alpha::DeclareParameters() {
  DeclareParameter("absolute_error", &absolute_error_,
                   "absolute error of the displacement of a step, defaults to "
                   "1e-6 if not provided",
                   false);
  DeclareParameter("relative_error", &relative_error_,
                   "error of the displacement of a step relative to the "
                   "displacement, defaults to 1e-4 if not provided",
                   false);
  DeclareParameter("hmax", &hmax_,
                   "largest internal time step, defaults to time_step if not "
                   "provided",
                   false);
  DeclareParameter("max_iterations", &max_iterations_,
                   "largest number of Newton iterations of a step, defaults "
                   "to 6 if not provided",
                   false);
}

void shift::core::integrator::Alpha::Connect() {
  assert(context_);
  Base::Connect();
  assert(solver_);
}

void shift::core::integrator::Alpha::Reset() {
  assert(context_);
  q_count_ = context_->get_q_count();
  dq_count_ = context_->get_dq_count();
  const unsigned int state_count = context_->get_state_count();
  neqn_ = state_count + context_->get_auxiliary_state_count();
  other_offset_ = q_count_ + dq_count_;
  other_count_ = neqn_ - other_offset_;

  y_.assign(neqn_, 0.0);
  memcpy(y_.data(), context_->get_q().data(), state_count * sizeof(double));
  context_->InitializeAuxiliaryStates(y_.data() + state_count);

  /* the position constraints hold by projection: no Baumgarte
  stabilization */
  integral_error_.assign(context_->get_constraint_equation_count(), 0.0);
  baumgarte_omega_n_ = 0.0;
  baumgarte_omega_n_prev_ = 0.0;

  SetCoefficients();
  if(std::fabs(1.0 - alpha_m_) < 1e-12) {
    throw shift::core::Exception(
      "Alpha integrator: alpha_m = 1 leaves the method undefined");
  }

  for(auto* v : {&ddq_, &a_, &a_step_, &ddq_trial_, &a_trial_, &delta_}) {
    v->assign(dq_count_, 0.0);
  }
  for(auto* v : {&dz_, &dz_trial_, &w_, &w_step_, &w_trial_, &dw_}) {
    v->assign(other_count_, 0.0);
  }
  for(auto* v : {&y_trial_, &dy_}) {
    v->assign(neqn_, 0.0);
  }
  all_velocities_.resize(dq_count_);
  for(unsigned int i = 0; i < dq_count_; i++) {
    all_velocities_[i] = static_cast<int>(i);
  }
  la_.clear();
  a_step_size_ = 0.0;
  has_dw_ = false;

  t_ = 0.0;
  /* no longer than hmax and than the terrain allows (Context::GetStepLimit) */
  step_max_ = std::min(hmax_ > 0.0 ? hmax_ : time_step_,
                       context_->GetStepLimit());
  h_ = 0.01 * step_max_;
  s_current_ = false;
  s_step_size_ = 0.0;
  matrix_kind_ = MatrixKind::kTangent;
  rescue_rate_ = 1.0;
  tangent_failures_ = 0;
  tangent_matrix_count_ = 0;
  differenced_matrix_count_ = 0;
  states_first_count_ = 0;
  mixed_matrix_count_ = 0;
  rescaled_matrix_count_ = 0;
  mixed_columns_.clear();
  accepted_steps_ = 0;
  rejected_steps_ = 0;
  iteration_matrix_count_ = 0;
  newton_iterations_ = 0;
}

void shift::core::integrator::Alpha::Initialize() {
  Reset();
}

void shift::core::integrator::Alpha::get_current_time(double& time,
                                                      double& delta_time) {
  time = t_;
  delta_time = h_;
}

bool shift::core::integrator::Alpha::EvaluateTrial(double t, double h) {
  const double one_minus_alpha_m = 1.0 - alpha_m_;
  for(unsigned int i = 0; i < dq_count_; i++) {
    a_trial_[i] = ((1.0 - alpha_f_) * ddq_trial_[i] + alpha_f_ * ddq_[i] -
                   alpha_m_ * a_step_[i]) /
                  one_minus_alpha_m;
    delta_[i] = h * y_[q_count_ + i] +
                h * h * ((0.5 - beta_) * a_step_[i] + beta_ * a_trial_[i]);
    y_trial_[q_count_ + i] =
      y_[q_count_ + i] +
      h * ((1.0 - gamma_) * a_step_[i] + gamma_ * a_trial_[i]);
  }
  context_->IncrementPositions(y_.data(), delta_.data(), y_trial_.data());
  if(solver_->get_multiplier_count() > 0) {
    ProjectPositions();
  }
  for(unsigned int j = 0; j < other_count_; j++) {
    w_trial_[j] = ((1.0 - alpha_f_) * dz_trial_[j] + alpha_f_ * dz_[j] -
                   alpha_m_ * w_step_[j]) /
                  one_minus_alpha_m;
    y_trial_[other_offset_ + j] =
      y_[other_offset_ + j] +
      h * ((1.0 - gamma_) * w_step_[j] + gamma_ * w_trial_[j]);
  }
  return solver_->Residual(t, y_trial_, ddq_trial_, la_trial_, r_, dy_);
}

void shift::core::integrator::Alpha::ProjectPositions() {
  std::vector<double>& q = context_->get_q();
  projection_q_.assign(q.begin(), q.begin() + q_count_);
  memcpy(q.data(), y_trial_.data(), q_count_ * sizeof(double));
  try {
    context_->get_jacobian().ProjectPositions(
      context_->get_model_parameters().get_epsilon_rank_determination());
  } catch(...) {
    memcpy(q.data(), projection_q_.data(), q_count_ * sizeof(double));
    throw;
  }
  memcpy(y_trial_.data(), q.data(), q_count_ * sizeof(double));
  memcpy(q.data(), projection_q_.data(), q_count_ * sizeof(double));
}

double shift::core::integrator::Alpha::StateErrorCoefficient() const {
  const double delta = alpha_m_ - alpha_f_;
  return std::fabs((1.0 - gamma_) * delta * delta / 2.0 +
                   gamma_ * (1.0 + delta) * (1.0 + delta) / 2.0 - 1.0 / 6.0);
}

bool shift::core::integrator::Alpha::FormIterationMatrix(double t, double h,
                                                        MatrixKind kind) {
  /* the trial state, with the modules recording what their tangents use */
  context_->set_collecting_tangents(true);
  const bool failed = EvaluateTrial(t, h);
  context_->set_collecting_tangents(false);
  if(failed) {
    return true;
  }
  tangents_.Clear();
  const bool complete =
    context_->CollectTangents(y_trial_, tangents_, &mixed_columns_);
  /* a matrix from the tangents with the columns of the modules that give
  none by differences, unless those are all of them */
  if(kind == MatrixKind::kTangent && !complete &&
     mixed_columns_.size() >= dq_count_) {
    kind = MatrixKind::kDifferenced;
  }
  if(kind == MatrixKind::kDifferenced) {
    mixed_columns_.clear();
  }
  const std::vector<int>& columns =
    kind == MatrixKind::kDifferenced ? all_velocities_ : mixed_columns_;
  const size_t column_count = columns.size();
  if(kind == MatrixKind::kTangent) {
    tangent_matrix_count_++;
    if(column_count > 0) {
      mixed_matrix_count_++;
    }
  } else {
    differenced_matrix_count_++;
    tangents_.Clear();
  }
  iteration_matrix_count_++;
  if(iteration_matrix_count_ == 1 || (kind == MatrixKind::kTangent &&
                                      column_count > 0 &&
                                      mixed_matrix_count_ == 1)) {
    SHIFT_DEBUG("Alpha integrator: " + std::to_string(dq_count_) +
                " velocities, " + std::to_string(other_count_) +
                " other states, tangents " +
                (complete ? "complete"
                          : "incomplete (" + std::to_string(column_count) +
                              " columns by differences)"));
  }

  const std::vector<double> r_base(r_.begin(), r_.begin() + dq_count_);
  const std::vector<double> y_base(y_trial_);
  const std::vector<double> rates_base(dy_.begin() + other_offset_,
                                       dy_.begin() + other_offset_ +
                                         other_count_);
  const double root_epsilon = std::sqrt(DBL_EPSILON);
  std::vector<double> y_perturbed(neqn_);
  std::vector<double> dy_perturbed(neqn_);
  std::vector<double> r_perturbed;

  /* the derivatives of the rates of the other states with respect to the
  coordinates and to the velocities, one row per state */
  state_stiffness_.assign(size_t(other_count_) * dq_count_, 0.0);
  state_damping_.assign(size_t(other_count_) * dq_count_, 0.0);

  /* the columns of the differenced velocities, all of them or those of the
  modules that give no tangents, and whether a velocity is among them */
  matrix_columns_ = columns;
  std::vector<bool> is_differenced(dq_count_, false);
  for(const int l : columns) {
    is_differenced[l] = true;
  }
  k_columns_.assign(size_t(dq_count_) * column_count, 0.0);
  c_columns_.assign(size_t(dq_count_) * column_count, 0.0);
  {
    /* the derivatives of the residual of the equations of motion with
    respect to the coordinates (by an increment along each velocity, as the
    method increments them) and to the velocities, by forward differences:
    each column holds all the terms, those of the tangents too */
    std::vector<double> unit(dq_count_, 0.0);
    for(size_t n = 0; n < column_count; n++) {
      const int l = columns[n];
      for(int velocity = 0; velocity < 2; velocity++) {
        double step;
        y_perturbed = y_base;
        if(velocity) {
          step = root_epsilon * std::max(1.0, std::fabs(y_base[q_count_ + l]));
          y_perturbed[q_count_ + l] += step;
        } else {
          step = root_epsilon;
          unit[l] = step;
          context_->IncrementPositions(y_base.data(), unit.data(),
                                       y_perturbed.data());
          unit[l] = 0.0;
        }
        if(solver_->Residual(t, y_perturbed, ddq_trial_, la_trial_,
                             r_perturbed, dy_perturbed)) {
          return true;
        }
        std::vector<double>& found = velocity ? c_columns_ : k_columns_;
        for(unsigned int i = 0; i < dq_count_; i++) {
          found[size_t(i) * column_count + n] =
            (r_perturbed[i] - r_base[i]) / step;
        }
        std::vector<double>& rows = velocity ? state_damping_ : state_stiffness_;
        for(unsigned int j = 0; j < other_count_; j++) {
          rows[size_t(j) * dq_count_ + l] =
            (dy_perturbed[other_offset_ + j] - rates_base[j]) / step;
        }
      }
    }
  }
  if(kind == MatrixKind::kTangent) {
    /* the rows of the other states from the derivatives of their rates the
    modules give (see Tangents), as the differences above would find them,
    in the columns not found by differences */
    auto add_state = [&](std::vector<double>& rows, int state, int l,
                         double value) {
      const int j = state - static_cast<int>(other_offset_);
      if(j > -1 && j < static_cast<int>(other_count_) && l > -1 &&
         l < static_cast<int>(dq_count_) && !is_differenced[l]) {
        rows[size_t(j) * dq_count_ + l] += value;
      }
    };
    for(const auto& entry : tangents_.get_state_stiffness()) {
      add_state(state_stiffness_, entry.state, entry.dq, entry.value);
    }
    for(const auto& entry : tangents_.get_state_damping()) {
      add_state(state_damping_, entry.state, entry.dq, entry.value);
    }
    std::vector<int> path;
    std::vector<ksl_screw_t> screws;
    auto add_rows = [&](const std::vector<Tangents::StateBodyRow>& body_rows,
                        std::vector<double>& rows) {
      for(const auto& state_row : body_rows) {
        context_->GetLoadPath(state_row.load_index, path, screws);
        for(size_t m = 0; m < path.size(); m++) {
          double value = 0.0;
          for(int c = 0; c < 6; c++) {
            value += state_row.row[c] * screws[m].at[c];
          }
          add_state(rows, state_row.state, path[m], value);
        }
      }
    };
    add_rows(tangents_.get_state_body_stiffness(), state_stiffness_);
    add_rows(tangents_.get_state_body_damping(), state_damping_);
  }

  /* the derivatives of the residual of the equations of motion and of the
  rates with respect to the other states: by differences, but for the
  states whose modules give them (see Tangents) */
  force_state_columns_.assign(size_t(dq_count_) * other_count_, 0.0);
  rate_state_block_.assign(size_t(other_count_) * other_count_, 0.0);
  std::vector<bool> is_given(other_count_, false);
  for(const auto& range : tangents_.get_given_states()) {
    for(int state = range.first; state < range.first + range.count;
        state++) {
      const int j = state - static_cast<int>(other_offset_);
      if(j > -1 && j < static_cast<int>(other_count_)) {
        is_given[j] = true;
      }
    }
  }
  for(unsigned int j = 0; j < other_count_; j++) {
    if(is_given[j]) {
      continue;
    }
    const double z = y_base[other_offset_ + j];
    const double step = root_epsilon * std::max(1.0, std::fabs(z));
    y_perturbed = y_base;
    y_perturbed[other_offset_ + j] += step;
    if(solver_->Residual(t, y_perturbed, ddq_trial_, la_trial_, r_perturbed,
                         dy_perturbed)) {
      return true;
    }
    for(unsigned int i = 0; i < dq_count_; i++) {
      force_state_columns_[size_t(j) * dq_count_ + i] =
        (r_perturbed[i] - r_base[i]) / step;
    }
    for(unsigned int i = 0; i < other_count_; i++) {
      rate_state_block_[size_t(i) * other_count_ + j] =
        (dy_perturbed[other_offset_ + i] - rates_base[i]) / step;
    }
  }
  {
    /* the given states: the residual changes by minus the generalized
    forces of the loads, h . dw / dz */
    auto given = [&](int state) {
      const int j = state - static_cast<int>(other_offset_);
      return j > -1 && j < static_cast<int>(other_count_) && is_given[j]
               ? j
               : -1;
    };
    std::vector<int> path;
    std::vector<ksl_screw_t> screws;
    for(const auto& entry : tangents_.get_state_loads()) {
      const int j = given(entry.state);
      if(j < 0) {
        continue;
      }
      context_->GetLoadPath(entry.load_index, path, screws);
      for(size_t m = 0; m < path.size(); m++) {
        double value = 0.0;
        for(int c = 0; c < 6; c++) {
          value += entry.load[c] * screws[m].at[c];
        }
        force_state_columns_[size_t(j) * dq_count_ + path[m]] -= value;
      }
    }
    for(const auto& entry : tangents_.get_state_forces()) {
      const int j = given(entry.state);
      if(j > -1 && entry.dq > -1 && entry.dq < static_cast<int>(dq_count_)) {
        force_state_columns_[size_t(j) * dq_count_ + entry.dq] -= entry.value;
      }
    }
    for(const auto& entry : tangents_.get_state_rates()) {
      const int j = given(entry.other);
      const int i = entry.state - static_cast<int>(other_offset_);
      if(j > -1 && i > -1 && i < static_cast<int>(other_count_)) {
        rate_state_block_[size_t(i) * other_count_ + j] += entry.value;
      }
    }
  }

  /* the equations at the trial state again, whose mass matrix and B the
  iteration matrix takes */
  if(EvaluateTrial(t, h)) {
    return true;
  }
  matrix_kind_ = kind;
  return FactorIterationMatrix(h);
}

bool shift::core::integrator::Alpha::FactorIterationMatrix(double h) {
  s_current_ = false;
  const double ratio = (1.0 - alpha_f_) / (1.0 - alpha_m_);
  /* the derivative of the other states with respect to their rates,
  gamma' h, through the pseudo-rates */
  const double gamma_h = gamma_ * ratio * h;
  const double c_scale = gamma_ * ratio * h;
  const double k_scale = beta_ * ratio * h * h;

  if(solver_->FactorIterationMatrix(tangents_, c_scale, k_scale,
                                    matrix_columns_, k_columns_, c_columns_)) {
    return true;
  }

  /* the rows of the other states' residual dz - f(y) for the accelerations:
  f changes with q by beta' h^2 and with dq by gamma' h per acceleration */
  const size_t row_size = size_t(other_count_) * dq_count_;
  state_rows_.resize(row_size);
  for(size_t e = 0; e < row_size; e++) {
    state_rows_[e] =
      -(k_scale * state_stiffness_[e] + c_scale * state_damping_[e]);
  }

  /* the other states by the Schur complement: with S the iteration matrix
  of the solver, U the columns of z and V the rows of its residual,
  W = S^-1 U and Z = D - V W; the residual of the equations of motion and the
  rates change with z by gamma h per rate. Or, when it takes fewer
  operations, the other states first (FactorStatesFirst) */
  const unsigned int multiplier_count = solver_->get_multiplier_count();
  const size_t n = dq_count_ + multiplier_count;
  states_first_ = false;
  if(other_count_ > n) {
    if(FactorStatesFirst(gamma_h)) {
      return true;
    }
    if(states_first_) {
      coupled_size_ = n;
      s_step_size_ = h;
      s_current_ = true;
      return false;
    }
  }
  coupling_.assign(n * other_count_, 0.0);
  std::vector<double> column(n);
  for(unsigned int j = 0; j < other_count_; j++) {
    std::fill(column.begin(), column.end(), 0.0);
    for(unsigned int i = 0; i < dq_count_; i++) {
      column[i] = gamma_h * force_state_columns_[size_t(j) * dq_count_ + i];
    }
    if(solver_->SolveIterationMatrix(column)) {
      return true;
    }
    std::copy(column.begin(), column.end(), coupling_.begin() + j * n);
  }
  schur_.resize(size_t(other_count_) * other_count_);
  for(unsigned int i = 0; i < other_count_; i++) {
    for(unsigned int j = 0; j < other_count_; j++) {
      double sum = 0.0;
      for(unsigned int l = 0; l < dq_count_; l++) {
        sum += state_rows_[size_t(i) * dq_count_ + l] * coupling_[j * n + l];
      }
      schur_[size_t(i) * other_count_ + j] =
        (i == j ? 1.0 : 0.0) -
        gamma_h * rate_state_block_[size_t(i) * other_count_ + j] - sum;
    }
  }
  pivots_.resize(other_count_);
  if(other_count_ > 0 &&
     LAPACKE_dgetrf(LAPACK_ROW_MAJOR, other_count_, other_count_,
                    schur_.data(), other_count_, pivots_.data()) != 0) {
    return true;
  }
  coupled_size_ = n;
  s_step_size_ = h;
  s_current_ = true;
  return false;
}

bool shift::core::integrator::Alpha::FactorStatesFirst(double gamma_h) {
  const unsigned int m = other_count_;
  const size_t n = dq_count_ + solver_->get_multiplier_count();
  /* the blocks of D = I - gamma' h df/dz: the states joined by an entry */
  std::vector<int> root(m);
  for(unsigned int i = 0; i < m; i++) {
    root[i] = static_cast<int>(i);
  }
  auto find = [&](int i) {
    while(root[i] != i) {
      root[i] = root[root[i]];
      i = root[i];
    }
    return i;
  };
  for(unsigned int i = 0; i < m; i++) {
    for(unsigned int j = 0; j < m; j++) {
      if(i != j && rate_state_block_[size_t(i) * m + j] != 0.0) {
        const int a = find(static_cast<int>(i));
        const int b = find(static_cast<int>(j));
        if(a != b) {
          root[std::max(a, b)] = std::min(a, b);
        }
      }
    }
  }
  std::vector<std::vector<int>> blocks;
  std::vector<int> block_of(m, -1);
  for(unsigned int i = 0; i < m; i++) {
    const int r = find(static_cast<int>(i));
    if(block_of[r] < 0) {
      block_of[r] = static_cast<int>(blocks.size());
      blocks.emplace_back();
    }
    blocks[block_of[r]].push_back(static_cast<int>(i));
  }

  /* the operations of each way, with a solve with S counted as n^2 */
  double blocks_cost = 0.0;
  for(const auto& block : blocks) {
    const double b = static_cast<double>(block.size());
    blocks_cost += b * b * b / 3.0 + b * b * dq_count_;
  }
  const double dn = static_cast<double>(n), dm = static_cast<double>(m);
  const double dq = static_cast<double>(dq_count_);
  const double motion_first = dm * dn * dn + dm * dm * dq + dm * dm * dm / 3.0;
  const double states_first =
    blocks_cost + dm * dq * dq + dq * dn * dn + dn * dn * dn / 3.0;
  if(!(states_first < motion_first)) {
    return false;
  }

  /* the LU factors of the blocks */
  block_states_.clear();
  block_begin_.assign(1, 0);
  block_lu_begin_.assign(1, 0);
  block_lu_.clear();
  block_pivots_.clear();
  for(const auto& block : blocks) {
    const size_t b = block.size();
    const size_t lu = block_lu_.size();
    block_lu_.resize(lu + b * b);
    for(size_t r = 0; r < b; r++) {
      for(size_t c = 0; c < b; c++) {
        block_lu_[lu + r * b + c] =
          (r == c ? 1.0 : 0.0) -
          gamma_h * rate_state_block_[size_t(block[r]) * m + block[c]];
      }
    }
    block_pivots_.resize(block_states_.size() + b);
    if(b == 1) {
      /* a state alone: its own factor, without LAPACK */
      if(block_lu_[lu] == 0.0) {
        return true;
      }
      block_pivots_[block_states_.size()] = 1;
    } else if(LAPACKE_dgetrf(LAPACK_ROW_MAJOR, static_cast<int>(b),
                      static_cast<int>(b), block_lu_.data() + lu,
                      static_cast<int>(b),
                      block_pivots_.data() + block_states_.size()) != 0) {
      return true;
    }
    block_states_.insert(block_states_.end(), block.begin(), block.end());
    block_begin_.push_back(block_states_.size());
    block_lu_begin_.push_back(block_lu_.size());
  }

  /* D^-1 V, then the columns of S^-1 U D^-1 V, one solve per velocity */
  solved_rows_ = state_rows_;
  if(SolveStateBlocks(solved_rows_.data(), static_cast<int>(dq_count_))) {
    return true;
  }
  schur_.assign(n * n, 0.0);
  for(size_t i = 0; i < n; i++) {
    schur_[i * n + i] = 1.0;
  }
  std::vector<double> column(n);
  for(unsigned int l = 0; l < dq_count_; l++) {
    std::fill(column.begin(), column.end(), 0.0);
    for(unsigned int j = 0; j < m; j++) {
      const double v = solved_rows_[size_t(j) * dq_count_ + l];
      if(v == 0.0) {
        continue;
      }
      const double* u = &force_state_columns_[size_t(j) * dq_count_];
      for(unsigned int i = 0; i < dq_count_; i++) {
        column[i] += gamma_h * u[i] * v;
      }
    }
    if(solver_->SolveIterationMatrix(column) || column.size() != n) {
      return true;
    }
    for(size_t i = 0; i < n; i++) {
      schur_[i * n + l] -= column[i];
    }
  }
  pivots_.resize(n);
  if(LAPACKE_dgetrf(LAPACK_ROW_MAJOR, static_cast<int>(n),
                    static_cast<int>(n), schur_.data(), static_cast<int>(n),
                    pivots_.data()) != 0) {
    return true;
  }
  gamma_h_ = gamma_h;
  states_first_ = true;
  states_first_count_++;
  return false;
}

bool shift::core::integrator::Alpha::SolveStateBlocks(double* data,
                                                      int columns) const {
  std::vector<double>& buffer = block_buffer_;
  for(size_t k = 0; k + 1 < block_begin_.size(); k++) {
    const size_t begin = block_begin_[k];
    const size_t b = block_begin_[k + 1] - begin;
    if(b == 1) {
      /* a state alone: a division, without the copies and checks of LAPACKE
      that cost more than the solve */
      const double d = block_lu_[block_lu_begin_[k]];
      double* row = data + size_t(block_states_[begin]) * columns;
      for(int c = 0; c < columns; c++) {
        row[c] /= d;
      }
      continue;
    }
    buffer.resize(b * columns);
    for(size_t r = 0; r < b; r++) {
      std::copy(data + size_t(block_states_[begin + r]) * columns,
                data + size_t(block_states_[begin + r] + 1) * columns,
                buffer.begin() + r * columns);
    }
    if(LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'N', static_cast<int>(b), columns,
                      block_lu_.data() + block_lu_begin_[k],
                      static_cast<int>(b), block_pivots_.data() + begin,
                      buffer.data(), columns) != 0) {
      return true;
    }
    for(size_t r = 0; r < b; r++) {
      std::copy(buffer.begin() + r * columns,
                buffer.begin() + (r + 1) * columns,
                data + size_t(block_states_[begin + r]) * columns);
    }
  }
  return false;
}

bool shift::core::integrator::Alpha::SolveNewton(std::vector<double>& x,
                                                 std::vector<double>& w) {
  if(states_first_ && other_count_ > 0) {
    if(x.size() != coupled_size_ || w.size() < other_count_) {
      return true;
    }
    /* t = D^-1 g; x = (I - S^-1 U D^-1 V)^-1 S^-1 (r - U t);
    w = t - D^-1 V x */
    if(SolveStateBlocks(w.data(), 1)) {
      return true;
    }
    for(unsigned int j = 0; j < other_count_; j++) {
      if(w[j] == 0.0) {
        continue;
      }
      const double* u = &force_state_columns_[size_t(j) * dq_count_];
      for(unsigned int i = 0; i < dq_count_; i++) {
        x[i] -= gamma_h_ * u[i] * w[j];
      }
    }
    if(solver_->SolveIterationMatrix(x) || x.size() != coupled_size_) {
      return true;
    }
    const int n = static_cast<int>(coupled_size_);
    if(LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'N', n, 1, schur_.data(), n,
                      pivots_.data(), x.data(), 1) != 0) {
      return true;
    }
    for(unsigned int j = 0; j < other_count_; j++) {
      double sum = 0.0;
      for(unsigned int l = 0; l < dq_count_; l++) {
        sum += solved_rows_[size_t(j) * dq_count_ + l] * x[l];
      }
      w[j] -= sum;
    }
    return false;
  }
  if(solver_->SolveIterationMatrix(x) || x.size() != coupled_size_) {
    return true;
  }
  if(other_count_ == 0) {
    return false;
  }
  /* (D - V W) dw = g - V x0, dx = x0 - W dw */
  for(unsigned int i = 0; i < other_count_; i++) {
    double sum = 0.0;
    for(unsigned int l = 0; l < dq_count_; l++) {
      sum += state_rows_[size_t(i) * dq_count_ + l] * x[l];
    }
    w[i] -= sum;
  }
  if(LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'N', other_count_, 1, schur_.data(),
                    other_count_, pivots_.data(), w.data(), 1) != 0) {
    return true;
  }
  const size_t n = coupled_size_;
  for(unsigned int j = 0; j < other_count_; j++) {
    const double* column = &coupling_[j * n];
    for(size_t i = 0; i < n; i++) {
      x[i] -= column[i] * w[j];
    }
  }
  return false;
}

bool shift::core::integrator::Alpha::Step(double h, bool truncated) {
  const double t1 = t_ + h;
  const double ratio = (1.0 - alpha_f_) / (1.0 - alpha_m_);
  const double beta_h2 = beta_ * ratio * h * h;
  /* the velocity constraints, whose derivative with respect to the
  accelerations is gamma' h [B_di I] */
  const double constraint_scale = 1.0 / (gamma_ * ratio * h);
  const double hmax = step_max_;

  auto reject = [&](double factor, const std::string& reason) {
    rejected_steps_++;
    h_ = h * factor;
    SHIFT_DEBUG("Alpha integrator: step of " + std::to_string(h) +
                " s at t = " + std::to_string(t_) + " rejected (" + reason +
                ")");
    if(h_ < kMinStepSize * std::max(1.0, t_)) {
      std::stringstream ss;
      ss << "Alpha integrator: step size " << h_ << " below the smallest at t = "
         << t_ << " (" << reason << ")";
      SHIFT_ERROR(ss.str());
      throw shift::core::Exception(ss.str());
    }
    return false;
  };

  /* the pseudo-accelerations for a step of size h: a_n approximates the
  accelerations at t_n + (alpha_m - alpha_f) h_n, h_n the size of the step
  that gave it, so that its difference from the accelerations is scaled to
  this step (Jay and Negrut, eq. 2.3); without the scaling a step size
  change leaves the method first order */
  for(unsigned int i = 0; i < dq_count_; i++) {
    a_step_[i] = a_step_size_ > 0.0
                   ? ddq_[i] + h / a_step_size_ * (a_[i] - ddq_[i])
                   : a_[i];
  }
  /* the pseudo-rates of the other states likewise */
  for(unsigned int j = 0; j < other_count_; j++) {
    w_step_[j] = a_step_size_ > 0.0
                   ? dz_[j] + h / a_step_size_ * (w_[j] - dz_[j])
                   : w_[j];
  }

  /* the predictor: the accelerations and rates of the last step, the
  multipliers of the Solve at its state */
  const unsigned int multiplier_count = solver_->get_multiplier_count();
  auto predict = [&]() {
    ddq_trial_ = ddq_;
    dz_trial_ = dz_;
    la_trial_ = la_;
    if(la_trial_.size() != multiplier_count) {
      la_trial_.assign(multiplier_count, 0.0);
    }
  };
  predict();
  rhs_.resize(dq_count_ + multiplier_count);
  state_rhs_.resize(other_count_);

  /* the matrix of an earlier step is scaled for a new step size, from the
  derivatives it was formed with, and factored again at the predictor */
  bool form = !s_current_;
  bool rescale = !form && (h > s_step_size_ * kReformFactor ||
                           h < s_step_size_ / kReformFactor);
  MatrixKind kind = MatrixKind::kTangent;
  bool rescuing = false;
  bool formed = false;
  bool converged = false;
  double previous_norm = 0.0;
  int iteration = 0;

  /* Newton is in trouble (diverging, too slow, or out of iterations): a
  matrix of an earlier step is formed again at this one, one from the
  tangents is formed again by differences while that rescues steps (see
  kRescueRate), both from the predictor; otherwise the step is rejected.
  Returns false to go on. */
  std::string trouble;
  auto record_rescue = [&](bool success) {
    rescue_rate_ = (1.0 - kRescueWeight) * rescue_rate_ +
                   kRescueWeight * (success ? 1.0 : 0.0);
  };
  auto escalate = [&](const std::string& reason) {
    if(formed && matrix_kind_ == MatrixKind::kTangent) {
      tangent_failures_++;
    }
    if(formed && (matrix_kind_ == MatrixKind::kDifferenced ||
                  (rescue_rate_ < kRescueRate &&
                   tangent_failures_ % kRescueProbe != 0))) {
      if(rescuing) {
        record_rescue(false);
      }
      trouble = reason;
      return true;
    }
    SHIFT_DEBUG("Alpha integrator: " + reason + " at t = " +
                std::to_string(t_) + ", h = " + std::to_string(h) +
                ", with a matrix " +
                (formed ? "from the tangents" : "of an earlier step") +
                ": formed again " +
                (formed ? "by differences" : "for this step"));
    if(formed) {
      kind = MatrixKind::kDifferenced;
      rescuing = true;
    } else {
      kind = MatrixKind::kTangent;
    }
    predict();
    form = true;
    rescale = false;
    iteration = 0;
    previous_norm = 0.0;
    converged = false;
    return false;
  };

  try {
    while(true) {
      if(form) {
        if(FormIterationMatrix(t1, h, kind)) {
          return reject(0.25, "solver failure");
        }
        form = false;
        formed = true;
      } else if(EvaluateTrial(t1, h)) {
        return reject(0.25, "solver failure");
      } else if(rescale) {
        rescale = false;
        rescaled_matrix_count_++;
        if(FactorIterationMatrix(h)) {
          /* formed again at this step */
          form = true;
          continue;
        }
      }
      if(converged) {
        break;
      }
      if(iteration >= max_iterations_) {
        if(escalate("Newton iterations did not converge")) {
          return reject(0.25, trouble);
        }
        continue;
      }

      for(unsigned int i = 0; i < dq_count_; i++) {
        rhs_[i] = -r_[i];
      }
      for(unsigned int m = 0; m < multiplier_count; m++) {
        rhs_[dq_count_ + m] = -r_[dq_count_ + m] * constraint_scale;
      }
      for(unsigned int j = 0; j < other_count_; j++) {
        state_rhs_[j] = dy_[other_offset_ + j] - dz_trial_[j];
      }
      if(SolveNewton(rhs_, state_rhs_)) {
        /* the partition changed since the iteration matrix was factored */
        if(escalate("no iteration matrix")) {
          return reject(0.25, trouble);
        }
        continue;
      }

      /* the corrections, weighted by the displacements they cause */
      double sum = 0.0;
      for(unsigned int i = 0; i < dq_count_; i++) {
        const double weighted = beta_h2 * rhs_[i] / Weight(i);
        sum += weighted * weighted;
        ddq_trial_[i] += rhs_[i];
      }
      for(unsigned int m = 0; m < multiplier_count; m++) {
        la_trial_[m] += rhs_[dq_count_ + m];
      }
      for(unsigned int j = 0; j < other_count_; j++) {
        const double change = gamma_ * ratio * h * state_rhs_[j];
        const double weight =
          absolute_error_ +
          relative_error_ * std::fabs(y_trial_[other_offset_ + j]);
        sum += (change / weight) * (change / weight);
        dz_trial_[j] += state_rhs_[j];
      }
      const unsigned int count = dq_count_ + other_count_;
      const double norm = count > 0 ? std::sqrt(sum / count) : 0.0;
      iteration++;
      newton_iterations_++;

      if(iteration > 1 && norm > kDivergenceRate * previous_norm &&
         (!formed || norm > previous_norm)) {
        if(escalate("Newton iterations diverged")) {
          return reject(0.25, trouble);
        }
        continue;
      }
      converged = norm <= kNewtonTolerance;
      previous_norm = norm;
    }
  } catch(const shift::core::Exception& exception) {
    return reject(0.25, exception.what());
  }
  if(rescuing) {
    record_rescue(true);
  }

  /* the local error estimate of Zienkiewicz and Xie */
  double sum = 0.0;
  const double error_coefficient = std::fabs(beta_ - 1.0 / 6.0) * h * h;
  for(unsigned int i = 0; i < dq_count_; i++) {
    const double weighted =
      error_coefficient * std::fabs(a_trial_[i] - a_step_[i]) / Weight(i);
    sum += weighted * weighted;
  }
  /* and of the other states, c h^3 times their third derivative, h^2 times
  which is the second difference of the pseudo-rates (see the class
  documentation); none on the first step */
  unsigned int count = dq_count_;
  if(has_dw_) {
    const double state_coefficient = StateErrorCoefficient() * h;
    const double scale = h / a_step_size_;
    for(unsigned int j = 0; j < other_count_; j++) {
      const double second_difference =
        (w_trial_[j] - w_step_[j]) - scale * dw_[j];
      const double weight =
        absolute_error_ +
        relative_error_ * std::fabs(y_trial_[other_offset_ + j]);
      const double weighted =
        state_coefficient * std::fabs(second_difference) / weight;
      sum += weighted * weighted;
    }
    count += other_count_;
  }
  const double error = count > 0 ? std::sqrt(sum / count) : 0.0;
  const double factor =
    error > 0.0
      ? std::min(2.0, std::max(0.2, 0.8 * std::pow(error, -1.0 / 3.0)))
      : 2.0;
  if(error > 1.0) {
    return reject(factor, "error estimate " + std::to_string(error));
  }

  /* accept */
  y_ = y_trial_;
  ddq_ = ddq_trial_;
  a_ = a_trial_;
  for(unsigned int j = 0; j < other_count_; j++) {
    dw_[j] = w_trial_[j] - w_step_[j];
  }
  has_dw_ = true;
  w_ = w_trial_;
  a_step_size_ = h;
  for(unsigned int j = 0; j < other_count_; j++) {
    dz_[j] = dy_[other_offset_ + j];
  }
  t_ = t1;
  accepted_steps_++;
  context_->AcceptStep(y_);

  /* the solver checks the partition at the accepted state, and refactors the
  Jacobian when a check fails; its multipliers start the next step */
  if(solver_->Solve(t_, y_, dy_)) {
    std::stringstream ss;
    ss << "Newton-Raphson failure at t = " << t_
       << ", integration stopped. Last successfully integrated time: " << t_;
    SHIFT_ERROR(ss.str());
    throw shift::core::Exception(ss.str());
  }
  solver_->Multipliers(la_);

  const double proposal = h * factor;
  if(!truncated || proposal < h_) {
    h_ = proposal;
  }
  h_ = std::min(h_, hmax);
  return true;
}

void shift::core::integrator::Alpha::Integrate() {
  Reset();
  next_progress_percent_ = 10;
  solver_->Initialize();
  /* the stabilized index-2 form: the velocity constraints in Newton, the
  position constraints by projection (see ProjectPositions) */
  solver_->set_velocity_constraints(true);

  if(!solver_->supports_implicit()) {
    throw shift::core::Exception(
      "integrator " + type_ +
      ": the solver does not support implicit integration (rnca, aba, dca "
      "and auto do)");
  }
  if(neqn_ < 1 || time_span_ <= 0.0 || time_step_ <= 0.0 || h_ <= 0.0 ||
     max_iterations_ < 1) {
    std::stringstream ss;
    ss << "Improper simulation parameters detected: neqn = " << neqn_
       << ", time_span = " << time_span_ << ", time_step = " << time_step_
       << ", hmax = " << hmax_ << ", max_iterations = " << max_iterations_;
    SHIFT_ERROR(ss.str());
    throw shift::core::Exception(ss.str());
  }

  /* the coordinates as the solver left them at its initialization, with
  the position constraints satisfied to the tightest tolerance, that of the
  projection of each step (ProjectPositions) */
  memcpy(y_.data(), context_->get_q().data(),
         context_->get_state_count() * sizeof(double));
  context_->get_jacobian().RemoveConstraintViolationsFast(true);
  memcpy(y_.data(), context_->get_q().data(), q_count_ * sizeof(double));

  /* the accelerations, multipliers and rates at the initial state */
  if(solver_->Solve(t_, y_, dy_)) {
    throw shift::core::Exception(
      "Newton-Raphson failure at the initial state, integration stopped");
  }
  for(unsigned int i = 0; i < dq_count_; i++) {
    ddq_[i] = dy_[q_count_ + i];
  }
  a_ = ddq_;
  for(unsigned int j = 0; j < other_count_; j++) {
    dz_[j] = dy_[other_offset_ + j];
  }
  w_ = dz_;
  solver_->Multipliers(la_);
  context_->WriteStates(t_, y_);

  /* report every time_step_, the last internal step of each report
  shortened to land on it */
  const unsigned int report_count = ceil(time_span_ / time_step_ - 1e-9);
  for(unsigned int report = 1; report <= report_count; report++) {
    const double t_report = std::min(report * time_step_, time_span_);
    while(t_report - t_ > 1e-12 * std::max(1.0, t_report)) {
      const double remaining = t_report - t_;
      const bool truncated = remaining < h_;
      Step(truncated ? remaining : h_, truncated);
    }
    context_->WriteStates(t_, y_);
    ReportProgress(t_);
  }

  SHIFT_INFO("Simulation finished t=" + std::to_string(t_) + " (" +
             std::to_string(accepted_steps_) + " steps, " +
             std::to_string(rejected_steps_) + " rejected, " +
             std::to_string(iteration_matrix_count_) + " iteration matrices (" +
             std::to_string(tangent_matrix_count_) + " from tangents, " +
             std::to_string(mixed_matrix_count_) +
             " of them with columns by differences), " +
             std::to_string(rescaled_matrix_count_) +
             " factored again for a new step size, " +
             std::to_string(newton_iterations_) + " Newton iterations)");
}
