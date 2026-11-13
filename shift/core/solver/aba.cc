#include "shift/core/solver/aba.h"

#include <assert.h>

#include <algorithm>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"

bool shift::core::solver::ABA::registered_ =
  shift::core::Factory<shift::core::solver::Base>::get()->RegisterModule(
    "aba", shift::core::solver::ABA::Create);

std::unique_ptr<shift::core::solver::ABA> shift::core::solver::ABA::Create() {
  return std::make_unique<shift::core::solver::ABA>();
}

void shift::core::solver::ABA::CheckTrees() {
  for(const auto& tree : trees_) {
    if(tree.kind == topology::Tree::Kind::kClosedLoop) {
      throw shift::core::Exception(
        "Solver 'aba' solves kinematic trees only, but the tree with root "
        "joint '" +
        RootName(tree) + "' has closed loops; use the 'rnca' solver");
    }
  }
}

void shift::core::solver::ABA::Setup() {
  const size_t n = body_count_;
  const auto& pcl_compressed = context_->get_pcl_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  const ksl_coscrew_t zero_coscrew = ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const ksl_screw_t zero_screw = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  inertia_a_.assign(36 * n, 0.0);
  bias_a_.assign(n, zero_coscrew);
  h_.assign(dq_count_, zero_screw);
  u_matrix_.assign(dq_count_, zero_coscrew);
  u_.assign(dq_count_, 0.0);
  d_factor_.assign(36 * n, 0.0);
  a_.assign(n, zero_screw);
  dof_.resize(n);
  has_children_.assign(n, 0);
  for(size_t j = 0; j < n; j++) {
    dof_[j] = static_cast<int>(joint_dof_compressed[j]);
  }
  for(int j : bodies_) {
    if(pcl_compressed[j] > -1) {
      has_children_[pcl_compressed[j]] = 1;
    }
  }
}

void shift::core::solver::ABA::AddInertia(const ksl_inertia_t& m,
                                          double C[36]) {
  /* the terms of InertiaMatrix(): [m I, -m t~; m t~, I] */
  const double x = m.mt.x;
  const double y = m.mt.y;
  const double z = m.mt.z;
  C[0] += m.m;
  C[7] += m.m;
  C[14] += m.m;
  C[4] += z;
  C[5] -= y;
  C[9] -= z;
  C[11] += x;
  C[15] += y;
  C[16] -= x;
  C[19] -= z;
  C[20] += y;
  C[24] += z;
  C[26] -= x;
  C[30] -= y;
  C[31] += x;
  C[21] += m.Ixx;
  C[22] += m.Ixy;
  C[23] += m.Izx;
  C[27] += m.Ixy;
  C[28] += m.Iyy;
  C[29] += m.Iyz;
  C[33] += m.Izx;
  C[34] += m.Iyz;
  C[35] += m.Izz;
}

void shift::core::solver::ABA::AddProduct(const double C[36],
                                          const ksl_screw_t& h,
                                          bool is_rotation,
                                          ksl_coscrew_t* u) {
  /* u += C h; a translational column has no angular part */
  const int columns = is_rotation ? 6 : 3;
  for(int r = 0; r < 6; r++) {
    double s = 0.0;
    for(int c = 0; c < columns; c++) {
      s += C[r * 6 + c] * h.at[c];
    }
    u->at[r] += s;
  }
}

bool shift::core::solver::ABA::SolveTrees() {
  const auto& pcl_compressed = context_->get_pcl_compressed();
  const auto& dq_lookup_compressed = context_->get_dq_lookup_compressed();
  const auto& is_rotation = context_->get_is_rotation();
  const auto& f = context_->get_joint_working_forces();
  const topology::DHVA& dhva = context_->get_dhva();
  auto& ddq = context_->get_ddq();

  /* M^A_j and z^A_j start from the body alone; the dense part of M^A_j holds
  only what the children add, so a tip uses the ten terms of M_j */
  for(int j : bodies_) {
    ksl_coscrew_inverted(&z_[j], &bias_a_[j]);
    if(has_children_[j]) {
      std::fill(&inertia_a_[36 * j], &inertia_a_[36 * j] + 36, 0.0);
    }
  }

  /* tips to root: articulated-body inertias and bias wrenches; a child
  follows its parent in permutation order 1, so each body has received the
  contributions of all its children when it is reached */
  for(auto it = bodies_.rbegin(); it != bodies_.rend(); it++) {
    const int j = *it;
    const int nd = dof_[j];
    const int l0 = dq_lookup_compressed[j];
    const double* C = &inertia_a_[36 * j];
    const ksl_coscrew_t& zA = bias_a_[j];
    double* D = &d_factor_[36 * j];

    /* U = M^A H, D = H^T U, u = f - H^T z^A */
    for(int k = 0; k < nd; k++) {
      const int l = l0 + k;
      ksl_screw_t& h = h_[l];
      ksl_coscrew_t& U = u_matrix_[l];
      h = dhva.h[l];
      if(!is_rotation[l]) {
        h.m3 = 0.0;
        h.m4 = 0.0;
        h.m5 = 0.0;
      }
      ksl_product_is(&inertia_[j], &h, &U);
      if(has_children_[j]) {
        AddProduct(C, h, is_rotation[l], &U);
      }
      u_[l] = f[l] - ksl_dot_cs(&zA, &h);
    }
    for(int k = 0; k < nd; k++) {
      for(int m = 0; m <= k; m++) {
        const double d = ksl_dot_cs(&u_matrix_[l0 + m], &h_[l0 + k]);
        D[k * nd + m] = d;
        D[m * nd + k] = d;
      }
    }
    if(nd == 1) {
      if(!(D[0] > 0.0)) {
        D[0] = 0.0;
      } else {
        D[0] = 1.0 / D[0];
      }
    }
    if((nd == 1 && D[0] == 0.0) || (nd > 1 && !CholeskyFactor(nd, D))) {
      SHIFT_ERROR("ABA: the articulated-body inertia of joint " +
                  std::to_string(j) +
                  " is singular along its axes (no mass outboard of the "
                  "joint?)");
      return true;
    }

    const int i = pcl_compressed[j];
    if(i < 0) {
      continue;
    }

    /* M^A_i += M^A_j - U D^{-1} U^T, z^A_i += z^A_j + U D^{-1} u */
    double y[6 * 6]; /* rows D^{-1} U^T, nd x 6 */
    double x[6];
    if(nd == 1) {
      for(int c = 0; c < 6; c++) {
        y[c] = D[0] * u_matrix_[l0].at[c];
      }
      x[0] = D[0] * u_[l0];
    } else {
      for(int c = 0; c < 6; c++) {
        double t[6];
        for(int k = 0; k < nd; k++) {
          t[k] = u_matrix_[l0 + k].at[c];
        }
        CholeskySolve(nd, D, t);
        for(int k = 0; k < nd; k++) {
          y[k * 6 + c] = t[k];
        }
      }
      for(int k = 0; k < nd; k++) {
        x[k] = u_[l0 + k];
      }
      CholeskySolve(nd, D, x);
    }

    double* Ci = &inertia_a_[36 * i];
    AddInertia(inertia_[j], Ci);
    ksl_coscrew_t& zAi = bias_a_[i];
    for(int r = 0; r < 6; r++) {
      double s = zA.at[r];
      for(int k = 0; k < nd; k++) {
        s += u_matrix_[l0 + k].at[r] * x[k];
      }
      zAi.at[r] += s;
      /* the update is symmetric: form the lower triangle, mirror it */
      for(int c = 0; c <= r; c++) {
        double m = has_children_[j] ? C[r * 6 + c] : 0.0;
        for(int k = 0; k < nd; k++) {
          m -= u_matrix_[l0 + k].at[r] * y[k * 6 + c];
        }
        Ci[r * 6 + c] += m;
        if(c != r) {
          Ci[c * 6 + r] += m;
        }
      }
    }
  }

  /* root to tips: joint accelerations */
  const ksl_screw_t zero = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  for(int j : bodies_) {
    const int nd = dof_[j];
    const int i = pcl_compressed[j];
    const int l0 = dq_lookup_compressed[j];
    const ksl_screw_t& ai = i > -1 ? a_[i] : zero;
    const double* D = &d_factor_[36 * j];
    double x[6];
    for(int k = 0; k < nd; k++) {
      x[k] = u_[l0 + k] - ksl_dot_cs(&u_matrix_[l0 + k], &ai);
    }
    if(nd == 1) {
      x[0] *= D[0];
    } else {
      CholeskySolve(nd, D, x);
    }
    ksl_screw_t& aj = a_[j];
    aj = ai;
    for(int k = 0; k < nd; k++) {
      ddq[l0 + k] = x[k];
      ksl_axpy_ss(x[k], &h_[l0 + k], &aj);
    }
  }
  return false;
}
