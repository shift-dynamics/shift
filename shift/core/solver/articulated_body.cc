#include "shift/core/solver/articulated_body.h"

#include <assert.h>

#include <algorithm>
#include <cmath>

#include <lapacke.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/joint/base.h"
#include "shift/core/log.h"
#include "shift/core/tangents.h"

void shift::core::solver::ArticulatedBody::Initialize() {
  assert(context_);
  q_count_ = context_->get_q_count();
  dq_count_ = context_->get_dq_count();
  body_count_ = context_->get_non_rigid_joint_count();

  const auto& trees = context_->get_trees();
  trees_.clear();
  if(all_trees_) {
    trees_ = trees;
  } else {
    for(int index : tree_indices_) {
      trees_.push_back(trees.at(index));
    }
  }
  bodies_.clear();
  for(const auto& tree : trees_) {
    bodies_.insert(bodies_.end(), tree.bodies.begin(), tree.bodies.end());
  }

  /* the generalized velocities of the trees: each joint's, then the elastic
  coordinates of its child body */
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& elastic_count_compressed =
    context_->get_elastic_count_compressed();
  tree_dq_.clear();
  for(int j : bodies_) {
    const unsigned int count =
      joint_dof_compressed[j] +
      (static_cast<size_t>(j) < elastic_count_compressed.size()
         ? elastic_count_compressed[j]
         : 0u);
    for(unsigned int k = 0; k < count; k++) {
      tree_dq_.push_back(dq_lookup_compressed[j] + static_cast<int>(k));
    }
  }
  std::sort(tree_dq_.begin(), tree_dq_.end());
  tree_dq_.erase(std::unique(tree_dq_.begin(), tree_dq_.end()),
                 tree_dq_.end());
  has_iteration_matrix_ = false;

  inertia_.assign(body_count_,
                  ksl_inertia(0.0, {{0.0, 0.0, 0.0}}, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0));
  p_.assign(body_count_, ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  z_.assign(body_count_, ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  CheckTrees();
  Setup();
  context_->UpdateDHVA();
  context_->EvaluateElasticBodies();
}

bool shift::core::solver::ArticulatedBody::Solve(const double t,
                                      const std::vector<double>& y,
                                      std::vector<double>& dy) {
  assert(context_);
  assert(q_count_ == context_->get_q_count());
  const double* q = y.data();
  const double* dq = q + q_count_;

  context_->UpdateDHVA(q, dq);
  context_->SetForces(t, y, dy);
  context_->EvaluateElasticBodies(context_->get_dhva(), q, dq);
  if(SolveAccelerations()) {
    return true;
  }
  context_->ConvertdqTody(y, dy);
  return false;
}

bool shift::core::solver::ArticulatedBody::SolveAccelerations() {
  SetupBodyWrenches();
  return SolveTrees();
}

void shift::core::solver::ArticulatedBody::SetupBodyWrenches() {
  for(int j : bodies_) {
    SetupBodyWrench(j);
  }
}

void shift::core::solver::ArticulatedBody::SetupBodyWrench(int j) {
  const topology::DHVA& dhva = context_->get_dhva();
  const auto& body_loads = context_->get_body_loads();

  ksl_inertia_t& m = inertia_[j];
  if(context_->get_has_mass_array()[j]) {
    ksl_inertia_transformed(
      &context_->get_inertia_vector()[j],
      &dhva.d[context_->get_d_reference_frames()[j]], &m);
  } else {
    m = ksl_inertia(0.0, {{0.0, 0.0, 0.0}}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  const ksl_screw_t& v = dhva.v[j];
  const ksl_screw_t& aq = dhva.aq[j];

  /* p = M v, z = g - M aq - [ad]^* p */
  ksl_coscrew_t& p = p_[j];
  ksl_coscrew_t& z = z_[j];
  ksl_coscrew_t ma;
  ksl_coscrew_t ad_p;
  ksl_product_is(&m, &v, &p);
  ksl_product_is(&m, &aq, &ma);
  ksl_cross_sc(&v, &p, &ad_p);
  const bool has_loads = body_loads.size() == body_count_;
  for(int r = 0; r < 6; r++) {
    z.at[r] = (has_loads ? body_loads[j].at[r] : 0.0) - ma.at[r] - ad_p.at[r];
  }
  /* a flexible body's bias wrench on its frame, which the gyroscopic wrench
  of M_bb above leaves out */
  const auto& bias_b = context_->get_elastic_bias_b();
  if(bias_b.size() == body_count_) {
    for(int r = 0; r < 6; r++) {
      z.at[r] += bias_b[j].at[r];
    }
  }
}

int shift::core::solver::ArticulatedBody::JointColumns(int j, double H[36]) const {
  const topology::DHVA& dhva = context_->get_dhva();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const auto& is_rotation = context_->get_is_rotation();
  const int n = static_cast<int>(joint_dof_compressed[j]);
  const int l0 = dq_lookup_compressed[j];
  for(int k = 0; k < n; k++) {
    const ksl_screw_t& h = dhva.h[l0 + k];
    for(int r = 0; r < 3; r++) {
      H[k * 6 + r] = h.at[r];
      H[k * 6 + 3 + r] = is_rotation[l0 + k] ? h.at[3 + r] : 0.0;
    }
  }
  return n;
}

std::string shift::core::solver::ArticulatedBody::RootName(
  const shift::core::topology::Tree& tree) const {
  return context_->get_active_joints()[tree.root_joint]->get_name();
}

double shift::core::solver::ArticulatedBody::GravitationalPotentialEnergy(
  shift::core::topology::DHVA& dhva) {
  double sum = 0.0;
  for(const auto& m : inertia_) {
    sum += ksl_dot_vv(&m.mt, &dhva.aq[-1].lin);
  }
  return sum;
}

double
shift::core::solver::ArticulatedBody::KineticEnergy(shift::core::topology::DHVA& dhva) {
  double sum = 0.0;
  for(size_t j = 0; j < p_.size(); j++) {
    sum += ksl_dot_cs(&p_[j], &dhva.v[j]);
  }
  return sum / 2.0 + ElasticKineticEnergy(dhva, &bodies_);
}

void shift::core::solver::ArticulatedBody::InertiaMatrix(const ksl_inertia_t& m,
                                              double M[36]) {
  /* M = [m I, -m t~; m t~, I_f], with m t~ the skew-symmetric matrix of mt */
  const double x = m.mt.x;
  const double y = m.mt.y;
  const double z = m.mt.z;
  const double matrix[36] = {
    m.m, 0.0, 0.0, 0.0, z,     -y,    //
    0.0, m.m, 0.0, -z,  0.0,   x,     //
    0.0, 0.0, m.m, y,   -x,    0.0,   //
    0.0, -z,  y,   m.Ixx, m.Ixy, m.Izx, //
    z,   0.0, -x,  m.Ixy, m.Iyy, m.Iyz, //
    -y,  x,   0.0, m.Izx, m.Iyz, m.Izz};
  for(int i = 0; i < 36; i++) {
    M[i] = matrix[i];
  }
}

bool shift::core::solver::ArticulatedBody::CholeskyFactor(int n, double* A) {
  double scale = 0.0;
  for(int i = 0; i < n; i++) {
    scale = std::fmax(scale, std::fabs(A[i * n + i]));
  }
  const double tolerance = 1e-14 * scale;
  for(int j = 0; j < n; j++) {
    double d = A[j * n + j];
    for(int k = 0; k < j; k++) {
      d -= A[j * n + k] * A[j * n + k];
    }
    if(!(d > tolerance)) {
      return false;
    }
    d = std::sqrt(d);
    A[j * n + j] = d;
    for(int i = j + 1; i < n; i++) {
      double s = A[i * n + j];
      for(int k = 0; k < j; k++) {
        s -= A[i * n + k] * A[j * n + k];
      }
      A[i * n + j] = s / d;
    }
  }
  return true;
}

void shift::core::solver::ArticulatedBody::CholeskySolve(int n, const double* L,
                                              double* x) {
  for(int i = 0; i < n; i++) {
    double s = x[i];
    for(int k = 0; k < i; k++) {
      s -= L[i * n + k] * x[k];
    }
    x[i] = s / L[i * n + i];
  }
  for(int i = n - 1; i > -1; i--) {
    double s = x[i];
    for(int k = i + 1; k < n; k++) {
      s -= L[k * n + i] * x[k];
    }
    x[i] = s / L[i * n + i];
  }
}

bool shift::core::solver::ArticulatedBody::InvertSymmetric6(const double A[36],
                                                 double inverse[36]) {
  double L[36];
  for(int i = 0; i < 36; i++) {
    L[i] = A[i];
  }
  if(!CholeskyFactor(6, L)) {
    return false;
  }
  for(int c = 0; c < 6; c++) {
    double x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    x[c] = 1.0;
    CholeskySolve(6, L, x);
    for(int r = 0; r < 6; r++) {
      inverse[r * 6 + c] = x[r];
    }
  }
  return true;
}

bool shift::core::solver::ArticulatedBody::Residual(
  const double t, const std::vector<double>& y, const std::vector<double>& ddq,
  const std::vector<double>& la, std::vector<double>& r,
  std::vector<double>& dy) {
  assert(context_);
  (void)la;
  const double* q = y.data();
  const double* dq = q + q_count_;
  context_->UpdateDHVA(q, dq);
  context_->SetForces(t, y, dy);
  context_->EvaluateElasticBodies(context_->get_dhva(), q, dq);
  if(SolveAccelerations()) {
    return true;
  }
  /* a velocity of no tree (there is none in a model the solver accepts)
  keeps the row of the accelerations */
  r.assign(ddq.begin(), ddq.end());
  TreeResidual(ddq, r);
  return false;
}

void shift::core::solver::ArticulatedBody::TreeResidual(
  const std::vector<double>& ddq, std::vector<double>& r) const {
  const auto& solved = context_->get_ddq();
  for(const int l : tree_dq_) {
    r[l] = ddq[l] - solved[l];
  }
}

bool shift::core::solver::ArticulatedBody::InverseMass(
  std::vector<double>& inverse_mass) {
  const size_t n = tree_dq_.size();
  inverse_mass.assign(n * n, 0.0);
  auto& f = context_->get_joint_working_forces();
  auto& solved = context_->get_ddq();
  const std::vector<double> base(solved.begin(), solved.end());
  /* a change large against the forces, so that the accelerations it causes
  are well above the rounding of those of the state; the solver is linear
  in it */
  double largest = 1.0;
  for(const double value : f) {
    largest = std::max(largest, std::fabs(value));
  }
  const double change = 1e8 * largest;
  bool failed = false;
  for(size_t m = 0; m < n && !failed; m++) {
    const int l = tree_dq_[m];
    const double saved = f[l];
    f[l] = saved + change;
    failed = SolveTrees();
    f[l] = saved;
    for(size_t i = 0; i < n && !failed; i++) {
      const int row = tree_dq_[i];
      inverse_mass[i * n + m] = (solved[row] - base[row]) / change;
    }
  }
  std::copy(base.begin(), base.end(), solved.begin());
  return failed;
}

bool shift::core::solver::ArticulatedBody::FactorIterationMatrix(
  const Tangents& tangents, double c, double k,
  const std::vector<int>& differenced, const std::vector<double>& k_columns,
  const std::vector<double>& c_columns) {
  assert(context_);
  has_iteration_matrix_ = false;
  const int n = static_cast<int>(dq_count_);
  std::vector<double> inverse_mass;
  if(InverseMass(inverse_mass)) {
    return true;
  }
  /* the position of each velocity among those of the trees */
  std::vector<int> position(dq_count_, -1);
  for(size_t m = 0; m < tree_dq_.size(); m++) {
    position[tree_dq_[m]] = static_cast<int>(m);
  }
  std::vector<bool> is_differenced(dq_count_, false);
  for(const int l : differenced) {
    is_differenced[l] = true;
  }

  /* c C + k K of the tangents, dense, in the columns not found by
  differences */
  std::vector<double> a(size_t(n) * n, 0.0);
  ForEachTangentEntry(*context_, tangents, c, k,
                      [&](int row, int column, double value) {
                        if(row > -1 && column > -1 && row < n && column < n &&
                           !is_differenced[column]) {
                          a[size_t(row) * n + column] += value;
                        }
                      });

  /* S = I + M^-1 A in the rows of the trees; a velocity of no tree keeps
  its row of A */
  iteration_lu_.assign(size_t(n) * n, 0.0);
  const size_t tree_count = tree_dq_.size();
  for(int i = 0; i < n; i++) {
    const int p = position[i];
    for(int j = 0; j < n; j++) {
      double value = i == j ? 1.0 : 0.0;
      if(p < 0) {
        value += a[size_t(i) * n + j];
      } else {
        for(size_t m = 0; m < tree_count; m++) {
          value += inverse_mass[size_t(p) * tree_count + m] *
                   a[size_t(tree_dq_[m]) * n + j];
        }
      }
      iteration_lu_[size_t(i) * n + j] = value;
    }
  }
  const size_t differenced_count = differenced.size();
  for(size_t m = 0; m < differenced_count; m++) {
    const int j = differenced[m];
    for(int i = 0; i < n; i++) {
      double value = i == j ? 1.0 : 0.0;
      if(!k_columns.empty()) {
        value += k * k_columns[size_t(i) * differenced_count + m];
      }
      if(!c_columns.empty()) {
        value += c * c_columns[size_t(i) * differenced_count + m];
      }
      iteration_lu_[size_t(i) * n + j] = value;
    }
  }

  iteration_pivots_.resize(n);
  if(n > 0 && LAPACKE_dgetrf(LAPACK_ROW_MAJOR, n, n, iteration_lu_.data(), n,
                             iteration_pivots_.data()) != 0) {
    return true;
  }
  has_iteration_matrix_ = true;
  return false;
}

bool shift::core::solver::ArticulatedBody::SolveIterationMatrix(
  std::vector<double>& rhs) {
  const int n = static_cast<int>(dq_count_);
  if(!has_iteration_matrix_ || rhs.size() != dq_count_) {
    return true;
  }
  if(n > 0 && LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'N', n, 1,
                             iteration_lu_.data(), n, iteration_pivots_.data(),
                             rhs.data(), 1) != 0) {
    return true;
  }
  return false;
}
