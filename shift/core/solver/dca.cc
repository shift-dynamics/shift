#include "shift/core/solver/dca.h"

#include <assert.h>

#include <algorithm>

#include <omp.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"

bool shift::core::solver::DCA::registered_ =
  shift::core::Factory<shift::core::solver::Base>::get()->RegisterModule(
    "dca", shift::core::solver::DCA::Create);

std::unique_ptr<shift::core::solver::DCA> shift::core::solver::DCA::Create() {
  return std::make_unique<shift::core::solver::DCA>();
}

void shift::core::solver::DCA::CheckTrees() {
  for(const auto& tree : trees_) {
    if(tree.kind == topology::Tree::Kind::kClosedLoop) {
      throw shift::core::Exception(
        "Solver 'dca' solves kinematic trees only, but the tree with root "
        "joint '" +
        RootName(tree) + "' has closed loops; use the 'rnca' solver");
    }
  }
  const auto& has_mass = context_->get_has_mass_array();
  const auto& joint_lookup = context_->get_joint_lookup();
  const auto& bodies = context_->get_active_articulated_bodies();
  std::vector<bool> solved(body_count_, false);
  for(int j : bodies_) {
    solved[j] = true;
  }
  for(size_t i = 0; i < joint_lookup.size(); i++) {
    const int j = joint_lookup[i];
    if(j > -1 && solved[j] && !has_mass[j]) {
      throw shift::core::Exception(
        "Solver 'dca' needs the mass and inertia of every body, but body '" +
        bodies[i]->get_name() +
        "' (with the bodies rigidly attached to it) has none; use 'aba' or "
        "'rnca'");
    }
  }
}

/* each thread that shares a level of the assembly takes at least this many
floating-point operations (about 50 us): starting threads costs more than it
saves for less (Context's kElasticGrain) */
static constexpr double kLevelGrain = 1e5;

void shift::core::solver::DCA::Setup() {
  const auto& pcl_compressed = context_->get_pcl_compressed();
  const auto& joint_dof_compressed = context_->get_joint_dof_compressed();
  children_.assign(body_count_, {});
  for(size_t j = 0; j < body_count_; j++) {
    if(pcl_compressed[j] > -1) {
      children_[pcl_compressed[j]].push_back(static_cast<int>(j));
    }
  }

  /* the flexible bodies and their node slots */
  const auto& bodies = context_->get_active_articulated_bodies();
  elastic_count_.assign(body_count_, 0);
  elastic_dq_.assign(body_count_, -1);
  slot_he_.assign(body_count_, -1);
  elastic_body_.assign(body_count_, nullptr);
  for(int i : context_->get_elastic_bodies()) {
    const auto body = bodies[i];
    const int j = body->get_va_index();
    assert(j > -1 && j < (int)body_count_);
    elastic_body_[j] = body;
    elastic_count_[j] = (int)body->get_elastic_count();
    elastic_dq_[j] = body->get_elastic_dq_index();
  }
  for(const auto& slot : context_->get_node_slots()) {
    slot_he_[slot.index] = slot.he_offset;
  }

  nodes_.clear();
  tops_.clear();
  root_joint_.clear();
  for(const auto& tree : trees_) {
    tops_.push_back(Build(tree.bodies));
    root_joint_.push_back(tree.bodies[0]);
  }
  for(auto& node : nodes_) {
    const size_t h = node.handles.size();
    node.psi.assign(36 * h * h, 0.0);
    node.bias.assign(6 * h, 0.0);
    node.wrench.assign(6 * h, 0.0);
    node.dof = node.joint > -1
                 ? static_cast<int>(joint_dof_compressed[node.joint])
                 : 0;
    if(node.body < 0) {
      node.g.assign(
        36 * (nodes_[node.a].handles.size() + nodes_[node.b].handles.size()),
        0.0);
    } else if(elastic_count_[node.body] == 0) {
      node.has_inverse_inertia = CacheInverseInertia(node);
    }
    if(node.body > -1 && elastic_count_[node.body] > 0) {
      const size_t size = 6 + elastic_count_[node.body];
      node.mass.assign(size * size, 0.0);
      node.zeta.assign(size, 0.0);
      /* when nothing hangs at its node slots (no mass, no children), M_ee
      alone is factored, and only once */
      bool empty = true;
      for(int k : node.slots) {
        empty = empty && !context_->get_has_mass_array()[k] &&
                children_[k].empty();
      }
      if(empty) {
        const int n = elastic_count_[node.body];
        const double* mass_ee = elastic_body_[node.body]->get_mass_ee();
        node.ee_factor.assign(mass_ee, mass_ee + n * n);
        if(CholeskyFactor(n, node.ee_factor.data())) {
          node.y_e.assign(6 * n, 0.0);
          node.x_e.assign(n, 0.0);
        } else {
          node.ee_factor.clear(); // AssembleFlexible reports it
        }
      }
    }
  }

  /* the nodes of each depth are independent of each other: they share
  threads when the work of the level pays for starting them */
  levels_.clear();
  for(size_t i = 0; i < nodes_.size(); i++) {
    const size_t d = static_cast<size_t>(nodes_[i].depth);
    if(levels_.size() <= d) {
      levels_.resize(d + 1);
    }
    levels_[d].push_back(static_cast<int>(i));
  }
  level_threads_.assign(levels_.size(), 1);
  for(size_t d = 0; d < levels_.size(); d++) {
    double work = 0.0;
    for(int i : levels_[d]) {
      work += NodeWork(nodes_[i]);
    }
    level_threads_[d] = std::max(
      1, std::min({omp_get_max_threads(), static_cast<int>(levels_[d].size()),
                   static_cast<int>(work / kLevelGrain)}));
  }

  {
    std::stringstream ss;
    ss << "DCA assembly depth of each tree:";
    for(int depth : get_assembly_depths()) {
      ss << " " << depth;
    }
    SHIFT_DEBUG(ss.str());
  }
}

std::vector<int> shift::core::solver::DCA::get_assembly_depths() const {
  std::vector<int> depths;
  for(int top : tops_) {
    depths.push_back(nodes_[top].depth);
  }
  return depths;
}

double shift::core::solver::DCA::NodeWork(const Node& node) const {
  const double h = static_cast<double>(node.handles.size());
  if(node.body < 0) {
    /* V_m and the h(h + 1)/2 blocks of Psi on and above the diagonal,
    products of 6 x 6 and 6 x (6 - dof) blocks, and the joint's reflections
    and factor */
    const double nr = 6.0 - node.dof;
    return 72.0 * nr * (h + h * (h + 1.0) / 2.0) + 1000.0;
  }
  const double n = elastic_count_[node.body];
  if(n == 0.0) {
    return node.has_inverse_inertia ? 250.0 : 500.0;
  }
  if(!node.ee_factor.empty()) {
    /* seven solves with M_ee, M_be Y */
    return 26.0 * n * n + 72.0 * n + 500.0;
  }
  /* the factor of M, its 6h + 1 solves and the h^2 blocks of Psi */
  const double size = 6.0 + n;
  return size * size * size / 3.0 + 2.0 * size * size * (6.0 * h + 1.0) +
         72.0 * size * h * h;
}

int shift::core::solver::DCA::Build(const std::vector<int>& piece) {
  const auto& pcl = context_->get_pcl_compressed();
  const int n = static_cast<int>(body_count_);
  std::vector<char> in_piece(n, 0);
  for(int x : piece) {
    in_piece[x] = 1;
  }

  /* handles: bodies with a joint to a body outside the piece, the fixed body
  included */
  std::vector<int> handles;
  for(int x : piece) {
    bool handle = pcl[x] < 0 || !in_piece[pcl[x]];
    for(int c : children_[x]) {
      handle = handle || !in_piece[c];
    }
    if(handle) {
      handles.push_back(x);
    }
  }

  /* a body alone, or a flexible body with its node slots, is a leaf */
  bool leaf = true;
  for(size_t k = 1; k < piece.size(); k++) {
    leaf = leaf && slot_he_[piece[k]] > -1 && pcl[piece[k]] == piece[0];
  }
  if(leaf) {
    Node node;
    node.body = piece[0];
    node.slots.assign(piece.begin() + 1, piece.end());
    node.handles = handles;
    nodes_.push_back(node);
    return static_cast<int>(nodes_.size()) - 1;
  }

  /* split at the joint whose child subtree (within the piece) is closest to
  half of the piece; piece[0], the body of lowest level, is its top */
  std::vector<int> size(n, 0);
  for(int x : piece) {
    size[x] = 1;
  }
  for(auto it = piece.rbegin(); it != piece.rend(); it++) {
    if(pcl[*it] > -1 && in_piece[pcl[*it]]) {
      size[pcl[*it]] += size[*it];
    }
  }
  const int total = static_cast<int>(piece.size());
  int split = -1;
  int best = total + 1;
  for(size_t k = 1; k < piece.size(); k++) {
    const int x = piece[k];
    if(slot_he_[x] > -1) {
      continue; // a node slot stays with its flexible body
    }
    const int larger = std::max(size[x], total - size[x]);
    if(larger < best) {
      best = larger;
      split = x;
    }
  }
  assert(split > -1);

  std::vector<char> in_b(n, 0);
  std::vector<int> piece_a;
  std::vector<int> piece_b;
  for(int x : piece) {
    if(x == split || (pcl[x] > -1 && in_piece[pcl[x]] && in_b[pcl[x]])) {
      in_b[x] = 1;
      piece_b.push_back(x);
    } else {
      piece_a.push_back(x);
    }
  }

  const int a = Build(piece_a);
  const int b = Build(piece_b);

  Node node;
  node.joint = split;
  node.a = a;
  node.b = b;
  node.handles = handles;
  node.depth = 1 + std::max(nodes_[a].depth, nodes_[b].depth);
  auto position = [](const std::vector<int>& list, int x) {
    auto it = std::find(list.begin(), list.end(), x);
    return it == list.end() ? -1 : static_cast<int>(it - list.begin());
  };
  node.a_handle = position(nodes_[a].handles, pcl[split]);
  node.b_handle = position(nodes_[b].handles, split);
  assert(node.a_handle > -1 && node.b_handle > -1);
  for(int x : nodes_[a].handles) {
    node.a_map.push_back(position(handles, x));
  }
  for(int x : nodes_[b].handles) {
    node.b_map.push_back(position(handles, x));
  }
  nodes_.push_back(node);
  return static_cast<int>(nodes_.size()) - 1;
}

void shift::core::solver::DCA::Reflect(const double v[6], double tau,
                                       double x[6]) {
  double s = 0.0;
  for(int i = 0; i < 6; i++) {
    s += v[i] * x[i];
  }
  s *= tau;
  for(int i = 0; i < 6; i++) {
    x[i] -= s * v[i];
  }
}

bool shift::core::solver::DCA::SolveJoint(Node& node, const double* psi_a,
                                          const double* b_a,
                                          const double* psi_b,
                                          const double* b_b) {
  const int nd = node.dof;
  const int nr = 6 - nd; /* the wrenches the joint transmits */
  for(int i = 0; i < 36; i++) {
    node.w[i] = (psi_a ? psi_a[i] : 0.0) + psi_b[i];
  }
  JointColumns(node.joint, node.h);

  /* H = Q R by Householder reflections, Q = P_0 ... P_{nd-1}: the last nr
  columns of Q, N, span the wrenches with H^T k = 0 */
  double a[36];
  std::copy(node.h, node.h + 6 * nd, a);
  for(int k = 0; k < nd; k++) {
    double* v = &node.householder[6 * k];
    const double* x = &a[6 * k];
    double norm = 0.0;
    double column = 0.0;
    for(int i = 0; i < 6; i++) {
      column += node.h[6 * k + i] * node.h[6 * k + i];
      norm += i < k ? 0.0 : x[i] * x[i];
    }
    norm = std::sqrt(norm);
    if(!(norm > 1e-12 * std::sqrt(column))) {
      SHIFT_ERROR("DCA: the joint columns of joint " +
                  std::to_string(node.joint) + " are dependent");
      return false;
    }
    const double alpha = x[k] > 0.0 ? -norm : norm;
    for(int i = 0; i < 6; i++) {
      v[i] = i < k ? 0.0 : x[i];
    }
    v[k] -= alpha;
    double vv = 0.0;
    for(int i = k; i < 6; i++) {
      vv += v[i] * v[i];
    }
    node.tau[k] = 2.0 / vv;
    node.r_factor[k * nd + k] = alpha;
    for(int j = k + 1; j < nd; j++) {
      Reflect(v, node.tau[k], &a[6 * j]);
      node.r_factor[k * nd + j] = a[6 * j + k];
      node.r_factor[j * nd + k] = 0.0;
    }
  }

  /* Q^T W Q, one reflection at a time: P W P = W - v z^T - z v^T with
  y = W v and z = tau y - (tau^2 v^T y / 2) v */
  double wq[36];
  std::copy(node.w, node.w + 36, wq);
  for(int k = 0; k < nd; k++) {
    const double* v = &node.householder[6 * k];
    const double tau = node.tau[k];
    double y[6];
    double s = 0.0;
    for(int r = 0; r < 6; r++) {
      double sum = 0.0;
      for(int c = k; c < 6; c++) {
        sum += wq[r * 6 + c] * v[c];
      }
      y[r] = sum;
      s += v[r] * sum;
    }
    double z[6];
    for(int r = 0; r < 6; r++) {
      z[r] = tau * y[r] - 0.5 * tau * tau * s * v[r];
    }
    for(int r = 0; r < 6; r++) {
      for(int c = 0; c < 6; c++) {
        wq[r * 6 + c] -= v[r] * z[c] + z[r] * v[c];
      }
    }
  }

  /* L L^T = N^T W N, the trailing block */
  double l[36];
  for(int r = 0; r < nr; r++) {
    for(int c = 0; c < nr; c++) {
      l[r * nr + c] = wq[(nd + r) * 6 + nd + c];
    }
  }
  if(nr > 0 && !CholeskyFactor(nr, l)) {
    SHIFT_ERROR("DCA: singular inverse articulated-body inertia across "
                "joint " +
                std::to_string(node.joint));
    return false;
  }

  /* Z = N L^-T: row i of Z solves L z = (row i of N)^T; its last dof
  columns are zero, so that the products with Z run over 6 columns */
  std::fill(node.z, node.z + 36, 0.0);
  for(int c = 0; c < nr; c++) {
    double e[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    e[nd + c] = 1.0;
    for(int k = nd - 1; k > -1; k--) {
      Reflect(&node.householder[6 * k], node.tau[k], e);
    }
    for(int i = 0; i < 6; i++) {
      node.z[i * 6 + c] = e[i];
    }
  }
  for(int i = 0; i < 6; i++) {
    double* row = &node.z[i * 6];
    for(int c = 0; c < nr; c++) {
      double s = row[c];
      for(int k = 0; k < c; k++) {
        s -= l[c * nr + k] * row[k];
      }
      row[c] = s / l[c * nr + c];
    }
  }

  /* the wrench of the joint forces alone, in Q's coordinates [u; lambda]:
  R^T u = f, and lambda = -(N^T W N)^-1 (Q^T W Q)_{21} u */
  const auto& f = context_->get_joint_working_forces();
  const int l0 = context_->get_dq_lookup_compressed()[node.joint];
  double x[6];
  for(int k = 0; k < nd; k++) {
    double s = f[l0 + k];
    for(int m = 0; m < k; m++) {
      s -= node.r_factor[m * nd + k] * x[m];
    }
    x[k] = s / node.r_factor[k * nd + k];
  }
  if(nr > 0) {
    double t[6];
    for(int r = 0; r < nr; r++) {
      double s = 0.0;
      for(int m = 0; m < nd; m++) {
        s -= wq[(nd + r) * 6 + m] * x[m];
      }
      t[r] = s;
    }
    CholeskySolve(nr, l, t);
    for(int r = 0; r < nr; r++) {
      x[nd + r] = t[r];
    }
  }
  for(int k = nd - 1; k > -1; k--) {
    Reflect(&node.householder[6 * k], node.tau[k], x);
  }
  std::copy(x, x + 6, node.k_f);

  for(int r = 0; r < 6; r++) {
    node.beta[r] = b_b[r] - (b_a ? b_a[r] : 0.0);
  }
  return true;
}

void shift::core::solver::DCA::Disassemble(const Node& node, const double r[6],
                                           double k[6]) {
  const int nd = node.dof;

  /* k = k_f - Z Z^T r, in locals (k may alias nothing of node, but the
  compiler cannot tell) */
  double a[6];
  std::copy(r, r + 6, a);
  double t[6];
  for(int c = 0; c < 6; c++) {
    double s = 0.0;
    for(int i = 0; i < 6; i++) {
      s += node.z[i * 6 + c] * a[i];
    }
    t[c] = s;
  }
  double wrench[6];
  for(int i = 0; i < 6; i++) {
    double s = node.k_f[i];
    for(int c = 0; c < 6; c++) {
      s -= node.z[i * 6 + c] * t[c];
    }
    wrench[i] = s;
  }

  /* H qdd = W k + r: R qdd = the first dof terms of Q^T (W k + r) */
  double x[6];
  for(int i = 0; i < 6; i++) {
    double s = a[i];
    for(int c = 0; c < 6; c++) {
      s += node.w[i * 6 + c] * wrench[c];
    }
    x[i] = s;
  }
  std::copy(wrench, wrench + 6, k);
  for(int m = 0; m < nd; m++) {
    Reflect(&node.householder[6 * m], node.tau[m], x);
  }
  auto& ddq = context_->get_ddq();
  const int l0 = context_->get_dq_lookup_compressed()[node.joint];
  for(int m = nd - 1; m > -1; m--) {
    double s = x[m];
    for(int c = m + 1; c < nd; c++) {
      s -= node.r_factor[m * nd + c] * x[c];
    }
    x[m] = s / node.r_factor[m * nd + m];
    ddq[l0 + m] = x[m];
  }
}

void shift::core::solver::DCA::HandleMap(int j, int x, double* p) const {
  const int n = elastic_count_[j];
  std::fill(p, p + 6 * (6 + n), 0.0);
  for(int c = 0; c < 6; c++) {
    p[c * 6 + c] = 1.0;
  }
  if(x != j) {
    /* a node slot: P_k^T = [I; E_k^T] */
    const ksl_screw_t* E = &context_->get_dhva().he[slot_he_[x]];
    for(int m = 0; m < n; m++) {
      for(int c = 0; c < 6; c++) {
        p[(6 + m) * 6 + c] = E[m].at[c];
      }
    }
  }
}

bool shift::core::solver::DCA::AssembleFlexibleFrame(Node& node) {
  const int j = node.body;
  const int n = elastic_count_[j];
  const int e = elastic_dq_[j];
  const auto& dhva = context_->get_dhva();
  const auto& mass_be = context_->get_elastic_mass_be();
  const auto& bias_e = context_->get_elastic_bias_e();
  const auto& f = context_->get_joint_working_forces();
  assert(node.handles.size() == 1 && node.handles[0] == j);

  /* x_e = M_ee^-1 zeta_e, Y = M_ee^-1 M_eb */
  double* x = node.x_e.data();
  double* y = node.y_e.data();
  for(int m = 0; m < n; m++) {
    x[m] = f[e + m] + bias_e[e + m] - ksl_dot_cs(&mass_be[e + m], &dhva.aq[j]);
  }
  /* the loads at the empty node slots: P_k^T z_k */
  ksl_coscrew_t slot_load = ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  for(int k : node.slots) {
    const ksl_screw_t* E = &dhva.he[slot_he_[k]];
    for(int m = 0; m < n; m++) {
      x[m] += ksl_dot_cs(&z_[k], &E[m]);
    }
    for(int r = 0; r < 6; r++) {
      slot_load.at[r] += z_[k].at[r];
    }
  }
  CholeskySolve(n, node.ee_factor.data(), x);
  double t[64];
  std::vector<double> big;
  double* column = t;
  if(n > 64) {
    big.resize(n);
    column = big.data();
  }
  for(int c = 0; c < 6; c++) {
    for(int m = 0; m < n; m++) {
      column[m] = mass_be[e + m].at[c];
    }
    CholeskySolve(n, node.ee_factor.data(), column);
    for(int m = 0; m < n; m++) {
      y[m * 6 + c] = column[m];
    }
  }

  /* S = M_bb - M_be Y, zeta_b - M_be x_e */
  double S[36];
  InertiaMatrix(inertia_[j], S);
  double zeta[6];
  for(int r = 0; r < 6; r++) {
    double s = z_[j].at[r] + slot_load.at[r];
    for(int m = 0; m < n; m++) {
      s -= mass_be[e + m].at[r] * x[m];
    }
    zeta[r] = s;
    for(int c = 0; c < 6; c++) {
      double sum = 0.0;
      for(int m = 0; m < n; m++) {
        sum += mass_be[e + m].at[r] * y[m * 6 + c];
      }
      S[r * 6 + c] -= sum;
    }
  }
  if(!InvertSymmetric6(S, node.psi.data())) {
    SHIFT_ERROR("DCA: the mass matrix of flexible body '" +
                elastic_body_[j]->get_name() +
                "' free of its joints (its frame's motion with its elastic "
                "coordinates) is singular: its shapes repeat a rigid motion "
                "of its reference frame, which DCA cannot solve (the shapes "
                "must hold the frame still); use 'aba' or 'rnca'");
    return true;
  }
  for(int r = 0; r < 6; r++) {
    double s = 0.0;
    for(int c = 0; c < 6; c++) {
      s += node.psi[r * 6 + c] * zeta[c];
    }
    node.bias[r] = s;
  }
  return false;
}

bool shift::core::solver::DCA::AssembleFlexible(Node& node) {
  if(!node.ee_factor.empty()) {
    return AssembleFlexibleFrame(node);
  }
  const int j = node.body;
  const int n = elastic_count_[j];
  const int size = 6 + n;
  const int e = elastic_dq_[j];
  const auto& dhva = context_->get_dhva();
  const auto& mass_be = context_->get_elastic_mass_be();
  const auto& bias_e = context_->get_elastic_bias_e();
  const auto& f = context_->get_joint_working_forces();
  double* M = node.mass.data();
  double* zeta = node.zeta.data();

  /* the frame's M_bb, M_be, M_ee, its bias wrench and the elastic forces */
  std::fill(M, M + size * size, 0.0);
  double block[36];
  InertiaMatrix(inertia_[j], block);
  for(int r = 0; r < 6; r++) {
    for(int c = 0; c < 6; c++) {
      M[r * size + c] = block[r * 6 + c];
    }
    zeta[r] = z_[j].at[r];
  }
  const double* mass_ee = elastic_body_[j]->get_mass_ee();
  for(int m = 0; m < n; m++) {
    for(int r = 0; r < 6; r++) {
      M[r * size + 6 + m] = mass_be[e + m].at[r];
      M[(6 + m) * size + r] = mass_be[e + m].at[r];
    }
    for(int l = 0; l < n; l++) {
      M[(6 + m) * size + 6 + l] = mass_ee[m * n + l];
    }
    zeta[6 + m] =
      f[e + m] + bias_e[e + m] - ksl_dot_cs(&mass_be[e + m], &dhva.aq[j]);
  }

  /* what is welded at the node slots: P_k^T M_k P_k and P_k^T z_k */
  std::vector<double> p(6 * size), mp(6 * size);
  for(int k : node.slots) {
    if(inertia_[k].m <= 0.0) {
      /* nothing hangs there but its loads */
      HandleMap(j, k, p.data());
      for(int r = 0; r < size; r++) {
        for(int d = 0; d < 6; d++) {
          zeta[r] += p[r * 6 + d] * z_[k].at[d];
        }
      }
      continue;
    }
    HandleMap(j, k, p.data());
    InertiaMatrix(inertia_[k], block);
    for(int r = 0; r < size; r++) {
      for(int c = 0; c < 6; c++) {
        double s = 0.0;
        for(int d = 0; d < 6; d++) {
          s += p[r * 6 + d] * block[d * 6 + c];
        }
        mp[r * 6 + c] = s;
      }
      for(int d = 0; d < 6; d++) {
        zeta[r] += p[r * 6 + d] * z_[k].at[d];
      }
    }
    for(int r = 0; r < size; r++) {
      for(int c = 0; c < size; c++) {
        double s = 0.0;
        for(int d = 0; d < 6; d++) {
          s += mp[r * 6 + d] * p[c * 6 + d];
        }
        M[r * size + c] += s;
      }
    }
  }
  if(!CholeskyFactor(size, M)) {
    SHIFT_ERROR("DCA: the mass matrix of flexible body '" +
                elastic_body_[j]->get_name() +
                "' free of its joints (its frame's motion with its elastic "
                "coordinates) is singular: its shapes repeat a rigid motion "
                "of its reference frame, which DCA cannot solve (the shapes "
                "must hold the frame still); use 'aba' or 'rnca'");
    return true;
  }

  /* b_m = P_m M^-1 zeta, Psi_mn = P_m M^-1 P_n^T */
  const int h = static_cast<int>(node.handles.size());
  std::vector<double> x(size);
  std::copy(zeta, zeta + size, x.begin());
  CholeskySolve(size, M, x.data());
  std::vector<double> maps(6 * size * h), solved(6 * size * h);
  for(int a = 0; a < h; a++) {
    double* pa = &maps[6 * size * a];
    HandleMap(j, node.handles[a], pa);
    for(int r = 0; r < 6; r++) {
      double s = 0.0;
      for(int d = 0; d < size; d++) {
        s += pa[d * 6 + r] * x[d];
      }
      node.bias[6 * a + r] = s;
    }
    /* the columns of M^-1 P_a^T */
    double* xa = &solved[6 * size * a];
    for(int c = 0; c < 6; c++) {
      std::vector<double> column(size);
      for(int d = 0; d < size; d++) {
        column[d] = pa[d * 6 + c];
      }
      CholeskySolve(size, M, column.data());
      for(int d = 0; d < size; d++) {
        xa[d * 6 + c] = column[d];
      }
    }
  }
  /* the blocks on and above the diagonal, mirrored below */
  for(int a = 0; a < h; a++) {
    const double* pa = &maps[6 * size * a];
    for(int b = a; b < h; b++) {
      const double* xb = &solved[6 * size * b];
      double* psi = &node.psi[36 * (a * h + b)];
      for(int r = 0; r < 6; r++) {
        for(int c = a == b ? r : 0; c < 6; c++) {
          double s = 0.0;
          for(int d = 0; d < size; d++) {
            s += pa[d * 6 + r] * xb[d * 6 + c];
          }
          psi[r * 6 + c] = s;
        }
      }
    }
  }
  MirrorBlocks(h, node.psi.data());
  return false;
}

void shift::core::solver::DCA::SolveElastic(const Node& node) {
  const int j = node.body;
  const int n = elastic_count_[j];
  if(!node.ee_factor.empty()) {
    /* the frame's acceleration b + Psi k, then x_e - Y of it */
    double a[6];
    for(int r = 0; r < 6; r++) {
      double s = node.bias[r];
      for(int c = 0; c < 6; c++) {
        s += node.psi[r * 6 + c] * node.wrench[c];
      }
      a[r] = s;
    }
    auto& ddq = context_->get_ddq();
    const int e = elastic_dq_[j];
    for(int m = 0; m < n; m++) {
      double s = node.x_e[m];
      for(int c = 0; c < 6; c++) {
        s -= node.y_e[m * 6 + c] * a[c];
      }
      ddq[e + m] = s;
    }
    return;
  }
  const int size = 6 + n;
  std::vector<double> x(node.zeta);
  std::vector<double> p(6 * size);
  for(size_t a = 0; a < node.handles.size(); a++) {
    HandleMap(j, node.handles[a], p.data());
    for(int d = 0; d < size; d++) {
      for(int c = 0; c < 6; c++) {
        x[d] += p[d * 6 + c] * node.wrench[6 * a + c];
      }
    }
  }
  CholeskySolve(size, node.mass.data(), x.data());
  auto& ddq = context_->get_ddq();
  const int e = elastic_dq_[j];
  for(int m = 0; m < n; m++) {
    ddq[e + m] = x[6 + m];
  }
}

bool shift::core::solver::DCA::CacheInverseInertia(Node& node) const {
  const ksl_inertia_t& local = context_->get_inertia_vector()[node.body];
  if(!(local.m > 0.0)) {
    return false;
  }
  /* the centroidal inertia in the reference frame (its moment terms are the
  entries of the matrix), inverted by its Cholesky factor */
  double L[9] = {local.Ixx, local.Ixy, local.Izx, //
                 local.Ixy, local.Iyy, local.Iyz, //
                 local.Izx, local.Iyz, local.Izz};
  if(!CholeskyFactor(3, L)) {
    return false;
  }
  double J[9];
  for(int c = 0; c < 3; c++) {
    double x[3] = {0.0, 0.0, 0.0};
    x[c] = 1.0;
    CholeskySolve(3, L, x);
    for(int r = 0; r < 3; r++) {
      J[r * 3 + c] = x[r];
    }
  }
  /* the lower triangle, so that the inverse is symmetric */
  node.inverse_inertia = ksl_inertia(1.0 / local.m, ksl_vec3(0.0, 0.0, 0.0),
                                     J[0], J[4], J[8], J[3], J[7], J[6]);
  node.inertia = local;
  return true;
}

void shift::core::solver::DCA::LeafInverseInertia(const Node& node,
                                                  double psi[36]) const {
  /* J = R Jbar R^T, by the rotation of an inertia's moment terms */
  const ksl_SE3_t& d =
    context_->get_dhva().d[context_->get_d_reference_frames()[node.body]];
  ksl_inertia_t rotated;
  ksl_inertia_rotated(&node.inverse_inertia, &d.R, &rotated);
  const double J[3][3] = {{rotated.Ixx, rotated.Ixy, rotated.Izx},
                          {rotated.Ixy, rotated.Iyy, rotated.Iyz},
                          {rotated.Izx, rotated.Iyz, rotated.Izz}};

  /* the mass centre in fixed coordinates (SetupBodyWrench set inertia_) */
  const ksl_inertia_t& m = inertia_[node.body];
  ksl_vec3_t c;
  ksl_product_av(1.0 / m.m, &m.mt, &c);

  /* A = c~ J, column by column */
  double A[3][3];
  for(int s = 0; s < 3; s++) {
    const ksl_vec3_t column = ksl_vec3(J[0][s], J[1][s], J[2][s]);
    ksl_vec3_t a;
    ksl_cross_vv(&c, &column, &a);
    for(int r = 0; r < 3; r++) {
      A[r][s] = a.at[r];
    }
  }
  for(int s = 0; s < 3; s++) {
    /* c~ J c~^T = c~ A^T: its column s is c x (row s of A); the upper
    triangle, mirrored */
    const ksl_vec3_t row = ksl_vec3(A[s][0], A[s][1], A[s][2]);
    ksl_vec3_t b;
    ksl_cross_vv(&c, &row, &b);
    for(int r = 0; r <= s; r++) {
      const double value = (r == s ? node.inverse_inertia.m : 0.0) + b.at[r];
      psi[r * 6 + s] = value;
      psi[s * 6 + r] = value;
    }
    for(int r = 0; r < 3; r++) {
      psi[r * 6 + 3 + s] = A[r][s];
      psi[(3 + s) * 6 + r] = A[r][s];
      psi[(3 + r) * 6 + 3 + s] = J[r][s];
    }
  }
}

bool shift::core::solver::DCA::Assemble(Node& node) {
  if(node.body > -1 && elastic_count_[node.body] > 0) {
    return AssembleFlexible(node);
  }
  if(node.body > -1) {
    /* the inverse cached from the body's inertia in its reference frame,
    while that inertia is the one it was made from */
    const ksl_inertia_t& local = context_->get_inertia_vector()[node.body];
    if(node.has_inverse_inertia &&
       std::equal(local.at, local.at + 10, node.inertia.at)) {
      LeafInverseInertia(node, node.psi.data());
    } else {
      double M[36];
      InertiaMatrix(inertia_[node.body], M);
      if(!InvertSymmetric6(M, node.psi.data())) {
        SHIFT_ERROR("DCA: the spatial inertia of body " +
                    std::to_string(node.body) + " is not positive definite");
        return true;
      }
    }
    for(int r = 0; r < 6; r++) {
      double s = 0.0;
      for(int c = 0; c < 6; c++) {
        s += node.psi[r * 6 + c] * z_[node.body].at[c];
      }
      node.bias[r] = s;
    }
    return false;
  }

  const Node& A = nodes_[node.a];
  const Node& B = nodes_[node.b];
  const int ha = static_cast<int>(A.handles.size());
  const int h = static_cast<int>(node.handles.size());
  const int pa = node.a_handle;
  const int pb = node.b_handle;
  auto block = [](const Node& s, int m, int n) {
    return &s.psi[36 * (m * static_cast<int>(s.handles.size()) + n)];
  };

  if(!SolveJoint(node, block(A, pa, pa), &A.bias[6 * pa], block(B, pb, pb),
                 &B.bias[6 * pb])) {
    return true;
  }

  /* a local copy of Z: the compiler cannot tell that the stores into g and
  psi below leave node.z unchanged */
  double Z[36];
  std::copy(node.z, node.z + 36, Z);

  /* k0 = k_f - K beta, K = Z Z^T */
  double k0[6];
  {
    double t[6];
    for(int c = 0; c < 6; c++) {
      double s = 0.0;
      for(int i = 0; i < 6; i++) {
        s += Z[i * 6 + c] * node.beta[i];
      }
      t[c] = s;
    }
    for(int i = 0; i < 6; i++) {
      double s = node.k_f[i];
      for(int c = 0; c < 6; c++) {
        s -= Z[i * 6 + c] * t[c];
      }
      k0[i] = s;
    }
  }

  /* V_m = Psi_{m alpha} Z for the handles of A, Psi_{m beta} Z for B, kept
  transposed (6 x 6, the rows past 6 - dof zero), and the bias accelerations
  of the assembly */
  double* g = node.g.data();
  for(int side = 0; side < 2; side++) {
    const Node& S = side == 0 ? A : B;
    const std::vector<int>& map = side == 0 ? node.a_map : node.b_map;
    const int p = side == 0 ? pa : pb;
    const double sign = side == 0 ? -1.0 : 1.0;
    const int offset = side == 0 ? 0 : ha;
    for(size_t m = 0; m < map.size(); m++) {
      if(map[m] < 0) {
        continue;
      }
      const double* psi = block(S, static_cast<int>(m), p);
      double* V = &g[36 * (offset + m)];
      for(int r = 0; r < 6; r++) {
        for(int c = 0; c < 6; c++) {
          double s = 0.0;
          for(int e = 0; e < 6; e++) {
            s += psi[r * 6 + e] * Z[e * 6 + c];
          }
          V[c * 6 + r] = s;
        }
        double s = S.bias[6 * m + r];
        for(int e = 0; e < 6; e++) {
          s += sign * psi[r * 6 + e] * k0[e];
        }
        node.bias[6 * map[m] + r] = s;
      }
    }
  }

  /* Psi^C_{mn}: the block of the own side less Psi_{m p} K Psi_{p n} =
  V_m V_n^T, or V_m V_n^T across the two sides. Psi^C is symmetric: only
  the blocks on and above the diagonal are formed, and of a diagonal block
  only its upper triangle; the rest are mirrored below */
  for(int side_m = 0; side_m < 2; side_m++) {
    const std::vector<int>& map_m = side_m == 0 ? node.a_map : node.b_map;
    const Node& Sm = side_m == 0 ? A : B;
    const int offset_m = side_m == 0 ? 0 : ha;
    for(size_t m = 0; m < map_m.size(); m++) {
      if(map_m[m] < 0) {
        continue;
      }
      const double* Vm = &g[36 * (offset_m + m)];
      for(int side_n = 0; side_n < 2; side_n++) {
        const std::vector<int>& map_n = side_n == 0 ? node.a_map : node.b_map;
        const int offset_n = side_n == 0 ? 0 : ha;
        for(size_t n = 0; n < map_n.size(); n++) {
          if(map_n[n] < map_m[m]) {
            continue; // below the diagonal, or internal (-1)
          }
          const double* Vn = &g[36 * (offset_n + n)];
          double* C = &node.psi[36 * (map_m[m] * h + map_n[n])];
          const bool same = side_m == side_n;
          const double* own =
            same ? block(Sm, static_cast<int>(m), static_cast<int>(n)) : nullptr;
          const bool diagonal = map_m[m] == map_n[n];
          for(int r = 0; r < 6; r++) {
            for(int c = diagonal ? r : 0; c < 6; c++) {
              double s = 0.0;
              for(int e = 0; e < 6; e++) {
                s += Vm[e * 6 + r] * Vn[e * 6 + c];
              }
              C[r * 6 + c] = same ? own[r * 6 + c] - s : s;
            }
          }
        }
      }
    }
  }
  MirrorBlocks(h, node.psi.data());
  return false;
}

void shift::core::solver::DCA::MirrorBlocks(int h, double* psi) {
  for(int m = 0; m < h; m++) {
    double* D = &psi[36 * (m * h + m)];
    for(int r = 1; r < 6; r++) {
      for(int c = 0; c < r; c++) {
        D[r * 6 + c] = D[c * 6 + r];
      }
    }
    for(int n = m + 1; n < h; n++) {
      const double* U = &psi[36 * (m * h + n)];
      double* L = &psi[36 * (n * h + m)];
      for(int r = 0; r < 6; r++) {
        for(int c = 0; c < 6; c++) {
          L[r * 6 + c] = U[c * 6 + r];
        }
      }
    }
  }
}

bool shift::core::solver::DCA::SolveTrees() {
  /* assembly, from the bodies up, level by level */
  for(size_t d = 0; d < levels_.size(); d++) {
    const std::vector<int>& level = levels_[d];
    const int count = static_cast<int>(level.size());
    const int threads = level_threads_[d];
    bool failed = false;
#pragma omp parallel for schedule(dynamic, 1) num_threads(threads) \
  if(threads > 1) reduction(|| : failed)
    for(int i = 0; i < count; i++) {
      if(Assemble(nodes_[level[i]])) {
        failed = true;
      }
    }
    if(failed) {
      return true;
    }
  }

  /* the joint of each tree to the fixed body */
  for(size_t t = 0; t < tops_.size(); t++) {
    Node& top = nodes_[tops_[t]];
    assert(top.handles.size() == 1);
    Node root;
    root.joint = root_joint_[t];
    root.dof =
      static_cast<int>(context_->get_joint_dof_compressed()[root.joint]);
    if(!SolveJoint(root, nullptr, nullptr, top.psi.data(), top.bias.data())) {
      return true;
    }
    Disassemble(root, root.beta, top.wrench.data());
  }

  /* disassembly, from the trees down, level by level */
  for(size_t d = levels_.size(); d-- > 0;) {
    const std::vector<int>& level = levels_[d];
    const int count = static_cast<int>(level.size());
    const int threads = level_threads_[d];
#pragma omp parallel for schedule(dynamic, 1) num_threads(threads) \
  if(threads > 1)
    for(int i = 0; i < count; i++) {
      DisassembleNode(nodes_[level[i]]);
    }
  }
  return false;
}

void shift::core::solver::DCA::DisassembleNode(Node& node) {
  if(node.body > -1) {
    /* a leaf has the wrenches of its handles: the elastic accelerations
    of a flexible body follow */
    if(elastic_count_[node.body] > 0) {
      SolveElastic(node);
    }
    return;
  }
  Node& A = nodes_[node.a];
  Node& B = nodes_[node.b];
  const int pa = node.a_handle;
  const int pb = node.b_handle;
  const int ha = static_cast<int>(A.handles.size());
  const int hb = static_cast<int>(B.handles.size());
  for(int m = 0; m < ha; m++) {
    for(int r = 0; r < 6; r++) {
      A.wrench[6 * m + r] =
        node.a_map[m] < 0 ? 0.0 : node.wrench[6 * node.a_map[m] + r];
    }
  }
  for(int m = 0; m < hb; m++) {
    for(int r = 0; r < 6; r++) {
      B.wrench[6 * m + r] =
        node.b_map[m] < 0 ? 0.0 : node.wrench[6 * node.b_map[m] + r];
    }
  }

  /* r = beta + sum_n Psi^B_{beta n} k_n - sum_n Psi^A_{alpha n} k_n */
  double r[6];
  for(int c = 0; c < 6; c++) {
    r[c] = node.beta[c];
  }
  for(int n = 0; n < hb; n++) {
    const double* psi = &B.psi[36 * (pb * hb + n)];
    for(int c = 0; c < 6; c++) {
      for(int e = 0; e < 6; e++) {
        r[c] += psi[c * 6 + e] * B.wrench[6 * n + e];
      }
    }
  }
  for(int n = 0; n < ha; n++) {
    const double* psi = &A.psi[36 * (pa * ha + n)];
    for(int c = 0; c < 6; c++) {
      for(int e = 0; e < 6; e++) {
        r[c] -= psi[c * 6 + e] * A.wrench[6 * n + e];
      }
    }
  }

  double k[6];
  Disassemble(node, r, k);
  for(int c = 0; c < 6; c++) {
    A.wrench[6 * pa + c] -= k[c];
    B.wrench[6 * pb + c] += k[c];
  }
}

