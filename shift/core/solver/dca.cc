#include "shift/core/solver/dca.h"

#include <assert.h>

#include <algorithm>

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
    if(node.body > -1 && elastic_count_[node.body] > 0) {
      const size_t size = 6 + elastic_count_[node.body];
      node.mass.assign(size * size, 0.0);
      node.zeta.assign(size, 0.0);
    }
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

bool shift::core::solver::DCA::SolveJoint(Node& node, const double* psi_a,
                                          const double* b_a,
                                          const double* psi_b,
                                          const double* b_b, double K[36]) {
  const int nd = node.dof;
  double W[36];
  for(int i = 0; i < 36; i++) {
    W[i] = (psi_a ? psi_a[i] : 0.0) + psi_b[i];
  }
  if(!InvertSymmetric6(W, node.inverse_w)) {
    SHIFT_ERROR("DCA: singular mobility across joint " +
                std::to_string(node.joint));
    return false;
  }
  JointColumns(node.joint, node.h);

  /* Y = W^{-1} H, D = H^T Y */
  for(int k = 0; k < nd; k++) {
    for(int r = 0; r < 6; r++) {
      double s = 0.0;
      for(int c = 0; c < 6; c++) {
        s += node.inverse_w[r * 6 + c] * node.h[k * 6 + c];
      }
      node.y[k * 6 + r] = s;
    }
  }
  for(int k = 0; k < nd; k++) {
    for(int m = 0; m < nd; m++) {
      double s = 0.0;
      for(int r = 0; r < 6; r++) {
        s += node.h[k * 6 + r] * node.y[m * 6 + r];
      }
      node.d_factor[k * nd + m] = s;
    }
  }
  if(!CholeskyFactor(nd, node.d_factor)) {
    SHIFT_ERROR("DCA: singular joint-space inertia at joint " +
                std::to_string(node.joint));
    return false;
  }

  /* K = W^{-1} - Y D^{-1} Y^T */
  double x[6 * 6]; /* D^{-1} Y^T, nd x 6 */
  for(int c = 0; c < 6; c++) {
    double t[6];
    for(int k = 0; k < nd; k++) {
      t[k] = node.y[k * 6 + c];
    }
    CholeskySolve(nd, node.d_factor, t);
    for(int k = 0; k < nd; k++) {
      x[k * 6 + c] = t[k];
    }
  }
  for(int r = 0; r < 6; r++) {
    for(int c = 0; c < 6; c++) {
      double s = node.inverse_w[r * 6 + c];
      for(int k = 0; k < nd; k++) {
        s -= node.y[k * 6 + r] * x[k * 6 + c];
      }
      K[r * 6 + c] = s;
    }
  }

  const auto& f = context_->get_joint_working_forces();
  const int l0 = context_->get_dq_lookup_compressed()[node.joint];
  for(int k = 0; k < nd; k++) {
    node.q_f[k] = f[l0 + k];
  }
  CholeskySolve(nd, node.d_factor, node.q_f);
  for(int r = 0; r < 6; r++) {
    node.beta[r] = b_b[r] - (b_a ? b_a[r] : 0.0);
  }
  return true;
}

void shift::core::solver::DCA::Disassemble(const Node& node, const double r[6],
                                           double k[6]) {
  const int nd = node.dof;
  double x[6];
  for(int m = 0; m < nd; m++) {
    double s = 0.0;
    for(int c = 0; c < 6; c++) {
      s += node.y[m * 6 + c] * r[c];
    }
    x[m] = s;
  }
  CholeskySolve(nd, node.d_factor, x);

  auto& ddq = context_->get_ddq();
  const int l0 = context_->get_dq_lookup_compressed()[node.joint];
  double t[6];
  for(int c = 0; c < 6; c++) {
    t[c] = -r[c];
  }
  for(int m = 0; m < nd; m++) {
    const double q = node.q_f[m] + x[m];
    ddq[l0 + m] = q;
    for(int c = 0; c < 6; c++) {
      t[c] += node.h[m * 6 + c] * q;
    }
  }
  for(int c = 0; c < 6; c++) {
    double s = 0.0;
    for(int e = 0; e < 6; e++) {
      s += node.inverse_w[c * 6 + e] * t[e];
    }
    k[c] = s;
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

bool shift::core::solver::DCA::AssembleFlexible(Node& node) {
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
  for(int a = 0; a < h; a++) {
    const double* pa = &maps[6 * size * a];
    for(int b = 0; b < h; b++) {
      const double* xb = &solved[6 * size * b];
      double* psi = &node.psi[36 * (a * h + b)];
      for(int r = 0; r < 6; r++) {
        for(int c = 0; c < 6; c++) {
          double s = 0.0;
          for(int d = 0; d < size; d++) {
            s += pa[d * 6 + r] * xb[d * 6 + c];
          }
          psi[r * 6 + c] = s;
        }
      }
    }
  }
  return false;
}

void shift::core::solver::DCA::SolveElastic(const Node& node) {
  const int j = node.body;
  const int n = elastic_count_[j];
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

bool shift::core::solver::DCA::Assemble(Node& node) {
  if(node.body > -1 && elastic_count_[node.body] > 0) {
    return AssembleFlexible(node);
  }
  if(node.body > -1) {
    double M[36];
    InertiaMatrix(inertia_[node.body], M);
    if(!InvertSymmetric6(M, node.psi.data())) {
      SHIFT_ERROR("DCA: the spatial inertia of body " +
                  std::to_string(node.body) + " is not positive definite");
      return true;
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
  const int hb = static_cast<int>(B.handles.size());
  const int h = static_cast<int>(node.handles.size());
  const int pa = node.a_handle;
  const int pb = node.b_handle;
  auto block = [](const Node& s, int m, int n) {
    return &s.psi[36 * (m * static_cast<int>(s.handles.size()) + n)];
  };

  double K[36];
  if(!SolveJoint(node, block(A, pa, pa), &A.bias[6 * pa], block(B, pb, pb),
                 &B.bias[6 * pb], K)) {
    return true;
  }

  /* k0 = W^{-1} H D^{-1} f - K beta */
  double k0[6];
  for(int r = 0; r < 6; r++) {
    double s = 0.0;
    for(int m = 0; m < node.dof; m++) {
      s += node.y[m * 6 + r] * node.q_f[m];
    }
    for(int c = 0; c < 6; c++) {
      s -= K[r * 6 + c] * node.beta[c];
    }
    k0[r] = s;
  }

  /* G_m = Psi_{m alpha} K for the handles of A, Psi_{m beta} K for B, and
  the bias accelerations of the assembly */
  std::vector<double> g(36 * (ha + hb), 0.0);
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
      double* G = &g[36 * (offset + m)];
      for(int r = 0; r < 6; r++) {
        for(int c = 0; c < 6; c++) {
          double s = 0.0;
          for(int e = 0; e < 6; e++) {
            s += psi[r * 6 + e] * K[e * 6 + c];
          }
          G[r * 6 + c] = s;
        }
        double s = S.bias[6 * m + r];
        for(int e = 0; e < 6; e++) {
          s += sign * psi[r * 6 + e] * k0[e];
        }
        node.bias[6 * map[m] + r] = s;
      }
    }
  }

  /* Psi^C_{mn}: the block of the own side less G_m Psi_{p n}, or G_m times
  the row of the joint handle of the other side */
  for(int side_m = 0; side_m < 2; side_m++) {
    const std::vector<int>& map_m = side_m == 0 ? node.a_map : node.b_map;
    const Node& Sm = side_m == 0 ? A : B;
    const int offset_m = side_m == 0 ? 0 : ha;
    for(size_t m = 0; m < map_m.size(); m++) {
      if(map_m[m] < 0) {
        continue;
      }
      const double* G = &g[36 * (offset_m + m)];
      for(int side_n = 0; side_n < 2; side_n++) {
        const std::vector<int>& map_n = side_n == 0 ? node.a_map : node.b_map;
        const Node& Sn = side_n == 0 ? A : B;
        const int pn = side_n == 0 ? pa : pb;
        for(size_t n = 0; n < map_n.size(); n++) {
          if(map_n[n] < 0) {
            continue;
          }
          const double* row = block(Sn, pn, static_cast<int>(n));
          double* C = &node.psi[36 * (map_m[m] * h + map_n[n])];
          const bool same = side_m == side_n;
          const double* own =
            same ? block(Sm, static_cast<int>(m), static_cast<int>(n)) : nullptr;
          for(int r = 0; r < 6; r++) {
            for(int c = 0; c < 6; c++) {
              double s = 0.0;
              for(int e = 0; e < 6; e++) {
                s += G[r * 6 + e] * row[e * 6 + c];
              }
              C[r * 6 + c] = same ? own[r * 6 + c] - s : s;
            }
          }
        }
      }
    }
  }
  return false;
}

bool shift::core::solver::DCA::SolveTrees() {
  /* assembly, from the bodies up */
  for(auto& node : nodes_) {
    if(Assemble(node)) {
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
    double K[36];
    if(!SolveJoint(root, nullptr, nullptr, top.psi.data(), top.bias.data(),
                   K)) {
      return true;
    }
    Disassemble(root, root.beta, top.wrench.data());
  }

  /* disassembly, from the trees down; a parent precedes its subassemblies */
  for(auto it = nodes_.rbegin(); it != nodes_.rend(); it++) {
    Node& node = *it;
    if(node.body > -1) {
      /* a leaf has the wrenches of its handles: the elastic accelerations
      of a flexible body follow */
      if(elastic_count_[node.body] > 0) {
        SolveElastic(node);
      }
      continue;
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
  return false;
}
