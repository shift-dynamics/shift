#include "shift/core/solver/articulated_body.h"

#include <assert.h>

#include <cmath>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/joint/base.h"

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

  inertia_.assign(body_count_,
                  ksl_inertia(0.0, {{0.0, 0.0, 0.0}}, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0));
  p_.assign(body_count_, ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  z_.assign(body_count_, ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  CheckTrees();
  Setup();
  context_->UpdateDHVA();
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
  return sum / 2.0;
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
