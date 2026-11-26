#include "shift/core/fea/reduction.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SymShiftInvert.h>
#include <Spectra/SymGEigsShiftSolver.h>

#include "shift/core/exception.h"
#include "shift/core/fea/trap_suspension.h"

/* [a]x, the matrix of the cross product a x */
static Eigen::Matrix3d Skew(const Eigen::Vector3d& a) {
  Eigen::Matrix3d s;
  s << 0.0, -a(2), a(1), a(2), 0.0, -a(0), -a(1), a(0), 0.0;
  return s;
}

static Eigen::Vector3d Vec(const ksl_vec3_t& v) {
  return Eigen::Vector3d(v.x, v.y, v.z);
}

/* the averaging (RBE3) attachment a: G u = r_a (6 x 3m) of its m nodes'
translations u (their 3m degrees of freedom dof), the least-squares mean
motion about its position */
static Eigen::MatrixXd
AveragingMatrix(const shift::core::fea::TetMesh& mesh,
                const shift::core::fea::Attachment& at, size_t a,
                std::vector<Eigen::Index>* dof) {
  const size_t m = at.nodes.size();
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for(int node : at.nodes) {
    c += Vec(mesh.nodes[node]) / static_cast<double>(m);
  }
  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  for(int node : at.nodes) {
    const Eigen::Vector3d d = Vec(mesh.nodes[node]) - c;
    S += d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(S);
  if(eigen.eigenvalues()(0) <= 1e-9 * eigen.eigenvalues()(2) || m < 3) {
    throw shift::core::Exception(
      "Reduce: the nodes of attachment " + std::to_string(a) +
      " lie on a line, so their mean rotation is not defined");
  }
  const Eigen::Matrix3d S_inverse = S.inverse();
  const Eigen::Matrix3d E = Skew(Vec(at.position) - c);
  Eigen::MatrixXd G(6, 3 * m);
  dof->clear();
  for(size_t i = 0; i < m; i++) {
    const Eigen::Matrix3d rotation =
      S_inverse * Skew(Vec(mesh.nodes[at.nodes[i]]) - c);
    G.block<3, 3>(0, 3 * i) =
      Eigen::Matrix3d::Identity() / static_cast<double>(m) - E * rotation;
    G.block<3, 3>(3, 3 * i) = rotation;
    for(int d = 0; d < 3; d++) {
      dof->push_back(3 * Eigen::Index(at.nodes[i]) + d);
    }
  }
  return G;
}

/* six columns of an averaging matrix G (positions in its columns) to
eliminate, chosen by Gaussian elimination with complete pivoting */
static std::vector<int> AveragingPivots(const Eigen::MatrixXd& G, size_t a) {
  const Eigen::Index columns = G.cols();
  Eigen::MatrixXd W = G;
  std::vector<char> used(size_t(columns), 0);
  std::vector<int> pivot;
  for(int k = 0; k < 6; k++) {
    Eigen::Index best_r = -1, best_c = -1;
    double best = 0.0;
    for(Eigen::Index r = k; r < 6; r++) {
      for(Eigen::Index col = 0; col < columns; col++) {
        if(!used[col] && std::fabs(W(r, col)) > best) {
          best = std::fabs(W(r, col));
          best_r = r;
          best_c = col;
        }
      }
    }
    if(best_c < 0 || best <= 1e-12 * G.cwiseAbs().maxCoeff()) {
      throw shift::core::Exception("Reduce: the motions of attachment " +
                                   std::to_string(a) +
                                   " are not set by its nodes");
    }
    W.row(k).swap(W.row(best_r));
    for(Eigen::Index r = k + 1; r < 6; r++) {
      W.row(r) -= W(r, best_c) / W(k, best_c) * W.row(k);
    }
    used[best_c] = 1;
    pivot.push_back(static_cast<int>(best_c));
  }
  return pivot;
}

/* (K_oo - sigma M_oo)^-1 for Spectra's shift and invert at sigma = 0,
held by the constraints Gh u_o = 0: K_oo^-1 - X C^-1 X^T, X = K_oo^-1 Gh^T
and C = Gh X; its results satisfy the constraints */
namespace shift {
namespace core {
namespace fea {
class HeldInverse {
public:
  using Scalar = double;
  HeldInverse(const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& ldlt,
              const Eigen::MatrixXd& X, const Eigen::LDLT<Eigen::MatrixXd>& c)
    : ldlt_(ldlt), X_(X), c_(c) {}
  Eigen::Index rows() const {
    return X_.rows();
  }
  Eigen::Index cols() const {
    return X_.rows();
  }
  void set_shift(double sigma) {
    if(sigma != 0.0) {
      throw shift::core::Exception("Reduce: the modes are found at shift 0");
    }
  }
  void perform_op(const double* x_in, double* y_out) const {
    const Eigen::Map<const Eigen::VectorXd> x(x_in, X_.rows());
    Eigen::Map<Eigen::VectorXd> y(y_out, X_.rows());
    y = ldlt_.solve(x);
    if(X_.cols() > 0) {
      y -= X_ * c_.solve(X_.transpose() * x);
    }
  }

private:
  const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& ldlt_;
  const Eigen::MatrixXd& X_;
  const Eigen::LDLT<Eigen::MatrixXd>& c_;
};
} // namespace fea
} // namespace core
} // namespace shift

/* Reduce, keeping modes_kept or, if keep_all, every mode found */
static shift::core::fea::Reduction ReduceModes(
  const shift::core::fea::TetMesh& mesh,
  const shift::core::fea::Assembled& assembled,
  const std::vector<shift::core::fea::Attachment>& attachments,
  unsigned int reference, const std::vector<int>& retained_nodes,
  unsigned int modes_computed, std::vector<unsigned int> modes_kept,
  bool keep_all) {
  /* Eigen's vectorized solves divide by zero in unused lanes (see
  DivideByZeroTrapSuspension); the results are checked below */
  const shift::core::fea::DivideByZeroTrapSuspension suspension;
  const Eigen::Index node_count = static_cast<Eigen::Index>(mesh.nodes.size());
  const Eigen::Index dofs = 3 * node_count;
  if(reference >= attachments.size()) {
    throw shift::core::Exception(
      "Reduce: the reference attachment is not among the attachments");
  }
  for(unsigned int k : modes_kept) {
    if(k >= modes_computed) {
      throw shift::core::Exception("Reduce: mode " + std::to_string(k) +
                                   " is kept but only " +
                                   std::to_string(modes_computed) +
                                   " are computed");
    }
  }

  /* each node belongs to at most one attachment or retained node */
  std::vector<int> owner(mesh.nodes.size(), -1);
  auto own = [&](int node, int who) {
    if(node < 0 || node >= node_count) {
      throw shift::core::Exception("Reduce: node " + std::to_string(node) +
                                   " is not a node of the mesh");
    }
    if(owner[node] >= 0) {
      throw shift::core::Exception(
        "Reduce: node " + std::to_string(node) +
        " is tied to two attachments (or retained and tied)");
    }
    owner[node] = who;
  };
  for(size_t a = 0; a < attachments.size(); a++) {
    if(attachments[a].nodes.empty()) {
      throw shift::core::Exception("Reduce: attachment " + std::to_string(a) +
                                   " is tied to no nodes");
    }
    for(int node : attachments[a].nodes) {
      own(node, static_cast<int>(a));
    }
  }
  for(int node : retained_nodes) {
    own(node, static_cast<int>(attachments.size()));
  }

  /* the interface coordinates r: six for each attachment but the
  reference, three for each retained node */
  std::vector<int> column(attachments.size(), -1);
  Eigen::Index r_count = 0;
  for(size_t a = 0; a < attachments.size(); a++) {
    if(a != reference) {
      column[a] = static_cast<int>(r_count);
      r_count += 6;
    }
  }
  const Eigen::Index retained_column = r_count;
  r_count += 3 * static_cast<Eigen::Index>(retained_nodes.size());

  /* the kind of each degree of freedom: free (in u_o), or set by r (RBE2
  and retained nodes), or a pivot of an averaging attachment */
  enum class Kind { kFree, kSet, kPivot };
  std::vector<Kind> kind(dofs, Kind::kFree);
  for(size_t a = 0; a < attachments.size(); a++) {
    if(attachments[a].spider == shift::core::fea::Attachment::Spider::kRbe2) {
      for(int node : attachments[a].nodes) {
        for(int d = 0; d < 3; d++) {
          kind[3 * node + d] = Kind::kSet;
        }
      }
    }
  }
  for(int node : retained_nodes) {
    for(int d = 0; d < 3; d++) {
      kind[3 * node + d] = Kind::kSet;
    }
  }

  /* the averaging attachments: G u = r_a, 6 x 3m; six pivots chosen by
  Gaussian elimination with complete pivoting */
  struct Average {
    std::vector<Eigen::Index> dof;      // 3m
    std::vector<int> pivot;             // 6, positions in dof
    Eigen::Matrix<double, 6, 6> p_inverse;
    Eigen::MatrixXd rest;               // 6 x 3m, zero at the pivots
  };
  std::vector<Average> averages(attachments.size());
  /* the other averaging attachments are constraints G_c u = r_g (rows of
  G_c in the order of their columns of r): eliminating their coordinates
  as the reference's would couple all their nodes densely */
  std::vector<Eigen::Triplet<double>> gc;
  std::vector<Eigen::Index> g_column;
  for(size_t a = 0; a < attachments.size(); a++) {
    const shift::core::fea::Attachment& at = attachments[a];
    if(at.spider != shift::core::fea::Attachment::Spider::kRbe3) {
      continue;
    }
    Average& avg = averages[a];
    const Eigen::MatrixXd G = AveragingMatrix(mesh, at, a, &avg.dof);
    if(a != reference) {
      const Eigen::Index row = static_cast<Eigen::Index>(g_column.size());
      for(int k = 0; k < 6; k++) {
        g_column.push_back(column[a] + k);
        for(size_t i = 0; i < avg.dof.size(); i++) {
          if(G(k, Eigen::Index(i)) != 0.0) {
            gc.emplace_back(row + k, avg.dof[i], G(k, Eigen::Index(i)));
          }
        }
      }
      continue;
    }
    avg.pivot = AveragingPivots(G, a);
    Eigen::Matrix<double, 6, 6> P;
    for(int k = 0; k < 6; k++) {
      P.col(k) = G.col(avg.pivot[k]);
    }
    avg.p_inverse = P.inverse();
    avg.rest = G;
    for(int k = 0; k < 6; k++) {
      avg.rest.col(avg.pivot[k]).setZero();
      kind[avg.dof[avg.pivot[k]]] = Kind::kPivot;
    }
  }

  /* u = T_r r + T_o u_o */
  std::vector<Eigen::Index> o_index(dofs, -1);
  Eigen::Index o_count = 0;
  for(Eigen::Index d = 0; d < dofs; d++) {
    if(kind[d] == Kind::kFree) {
      o_index[d] = o_count++;
    }
  }
  std::vector<Eigen::Triplet<double>> tr, to;
  for(Eigen::Index d = 0; d < dofs; d++) {
    if(kind[d] == Kind::kFree) {
      to.emplace_back(d, o_index[d], 1.0);
    }
  }
  for(size_t a = 0; a < attachments.size(); a++) {
    const shift::core::fea::Attachment& at = attachments[a];
    if(at.spider == shift::core::fea::Attachment::Spider::kRbe2) {
      if(column[a] < 0) {
        continue; /* the reference: held */
      }
      const Eigen::Index c = column[a];
      for(int node : at.nodes) {
        const Eigen::Vector3d r = Vec(mesh.nodes[node]) - Vec(at.position);
        /* u = t + theta x r = t - [r]x theta */
        const Eigen::Matrix3d minus_skew = -Skew(r);
        for(int d = 0; d < 3; d++) {
          tr.emplace_back(3 * node + d, c + d, 1.0);
          for(int e = 0; e < 3; e++) {
            if(minus_skew(d, e) != 0.0) {
              tr.emplace_back(3 * node + d, c + 3 + e, minus_skew(d, e));
            }
          }
        }
      }
      continue;
    }
    if(static_cast<unsigned int>(a) != reference) {
      continue; /* a constraint */
    }
    const Average& avg = averages[a];
    /* u_p = P^-1 (r_a - Rest u_rest) */
    const Eigen::MatrixXd coupling = avg.p_inverse * avg.rest;
    for(int k = 0; k < 6; k++) {
      const Eigen::Index row = avg.dof[avg.pivot[k]];
      if(column[a] >= 0) {
        for(int j = 0; j < 6; j++) {
          tr.emplace_back(row, column[a] + j, avg.p_inverse(k, j));
        }
      }
      for(size_t i = 0; i < avg.dof.size(); i++) {
        const Eigen::Index o = o_index[avg.dof[i]];
        if(o >= 0 && coupling(k, i) != 0.0) {
          to.emplace_back(row, o, -coupling(k, i));
        }
      }
    }
  }
  for(size_t i = 0; i < retained_nodes.size(); i++) {
    for(int d = 0; d < 3; d++) {
      tr.emplace_back(3 * retained_nodes[i] + d,
                      retained_column + 3 * static_cast<Eigen::Index>(i) + d,
                      1.0);
    }
  }
  Eigen::SparseMatrix<double> T_r(dofs, r_count), T_o(dofs, o_count);
  T_r.setFromTriplets(tr.begin(), tr.end());
  T_o.setFromTriplets(to.begin(), to.end());

  Eigen::SparseMatrix<double> M(dofs, dofs);
  {
    std::vector<Eigen::Triplet<double>> m;
    for(Eigen::Index n = 0; n < node_count; n++) {
      for(int d = 0; d < 3; d++) {
        m.emplace_back(3 * n + d, 3 * n + d, assembled.masses[n]);
      }
    }
    M.setFromTriplets(m.begin(), m.end());
  }
  const Eigen::SparseMatrix<double>& K = assembled.stiffness;
  const Eigen::SparseMatrix<double> K_o = K * T_o;
  const Eigen::SparseMatrix<double> K_oo = T_o.transpose() * K_o;
  const Eigen::SparseMatrix<double> M_oo = T_o.transpose() * (M * T_o);

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(K_oo);
  if(ldlt.info() != Eigen::Success ||
     ldlt.vectorD().minCoeff() <= 1e-12 * ldlt.vectorD().maxCoeff()) {
    throw shift::core::Exception(
      "Reduce: the stiffness with the reference attachment held is singular "
      "(does the reference attachment hold the body still?)");
  }

  /* the constraints on u_o, Gh = G_c T_o; X = K_oo^-1 Gh^T and the
  compliance C = Gh X at the averaging attachments */
  const Eigen::Index g_count = static_cast<Eigen::Index>(g_column.size());
  Eigen::SparseMatrix<double> G_c(g_count, dofs);
  G_c.setFromTriplets(gc.begin(), gc.end());
  const Eigen::SparseMatrix<double> Gh = G_c * T_o;
  Eigen::MatrixXd X(o_count, g_count);
  Eigen::LDLT<Eigen::MatrixXd> c_ldlt;
  if(g_count > 0) {
    X = ldlt.solve(Eigen::MatrixXd(Gh.transpose()));
    const Eigen::MatrixXd C = Gh * X;
    c_ldlt.compute(0.5 * (C + C.transpose()));
    if(c_ldlt.info() != Eigen::Success || !c_ldlt.isPositive()) {
      throw shift::core::Exception(
        "Reduce: the attachments' compliance is not positive definite");
    }
  }

  /* the static modes, the least-energy deformations with the interface
  r: u = T_r r_e + T_o u_o with G_c u = r_g; with B = T_o^T K T_r, Y =
  K_oo^-1 B and E_g the rows of r_g, u_o = -Y r + X C^-1 (E_g - G_c T_r +
  Gh Y) r */
  Eigen::MatrixXd psi(dofs, 0);
  if(r_count > 0) {
    const Eigen::MatrixXd Y =
      ldlt.solve(Eigen::MatrixXd(T_o.transpose() * (K * T_r)));
    Eigen::MatrixXd u_o = -Y;
    if(g_count > 0) {
      Eigen::MatrixXd W = Gh * Y - Eigen::MatrixXd(G_c * T_r);
      for(Eigen::Index g = 0; g < g_count; g++) {
        W(g, g_column[size_t(g)]) += 1.0;
      }
      u_o += X * c_ldlt.solve(W);
    }
    psi = Eigen::MatrixXd(T_r) + T_o * u_o;
  }

  /* the fixed-interface modes: K_oo v = lambda M_oo v with Gh v = 0, by
  shift and invert at 0 with the constrained inverse K_oo^-1 - X C^-1 X^T
  (HeldInverse), so that the one factorization serves */
  shift::core::fea::Reduction reduction;
  Eigen::MatrixXd phi(dofs, 0);
  if(modes_computed > 0) {
    const Eigen::Index free = o_count - g_count;
    const Eigen::Index nev = std::min<Eigen::Index>(modes_computed, free - 1);
    const Eigen::Index ncv =
      std::min<Eigen::Index>(free, std::max<Eigen::Index>(2 * nev + 1, 20));
    shift::core::fea::HeldInverse op(ldlt, X, c_ldlt);
    Spectra::SparseSymMatProd<double> bop(M_oo);
    Spectra::SymGEigsShiftSolver<shift::core::fea::HeldInverse,
                                 Spectra::SparseSymMatProd<double>,
                                 Spectra::GEigsMode::ShiftInvert>
      solver(op, bop, nev, ncv, 0.0);
    solver.init();
    solver.compute(Spectra::SortRule::LargestMagn, 1000, 1e-10);
    if(solver.info() != Spectra::CompInfo::Successful) {
      throw shift::core::Exception(
        "Reduce: the fixed-interface modes did not converge");
    }
    const Eigen::VectorXd values = solver.eigenvalues();
    const Eigen::MatrixXd vectors = solver.eigenvectors();
    std::vector<Eigen::Index> order(values.size());
    for(Eigen::Index i = 0; i < values.size(); i++) {
      order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](Eigen::Index a, Eigen::Index b) {
      return values(a) < values(b);
    });
    const Eigen::Index computed = static_cast<Eigen::Index>(order.size());
    reduction.computed_shapes.assign(3 * node_count * computed, 0.0);
    for(Eigen::Index k = 0; k < computed; k++) {
      const Eigen::Index i = order[k];
      reduction.frequencies.push_back(
        std::sqrt(std::max(values(i), 0.0)) / (2.0 * M_PI));
      Eigen::VectorXd v = vectors.col(i);
      v /= std::sqrt(v.dot(M_oo * v));
      const Eigen::VectorXd u = T_o * v;
      for(Eigen::Index n = 0; n < node_count; n++) {
        for(int d = 0; d < 3; d++) {
          reduction.computed_shapes[3 * (n * computed + k) + d] =
            u(3 * n + d);
        }
      }
    }
    if(keep_all) {
      modes_kept.clear();
      for(Eigen::Index k = 0; k < computed; k++) {
        modes_kept.push_back(static_cast<unsigned int>(k));
      }
    }
    for(unsigned int k : modes_kept) {
      if(k >= order.size()) {
        throw shift::core::Exception(
          "Reduce: mode " + std::to_string(k) + " is kept but the mesh has "
          "only " + std::to_string(order.size()) + " fixed-interface modes");
      }
    }
    phi.resize(dofs, static_cast<Eigen::Index>(modes_kept.size()));
    for(size_t k = 0; k < modes_kept.size(); k++) {
      Eigen::VectorXd v = vectors.col(order[modes_kept[k]]);
      v /= std::sqrt(v.dot(M_oo * v));
      phi.col(static_cast<Eigen::Index>(k)) = T_o * v;
    }
  }

  const Eigen::Index n_e = r_count + phi.cols();
  Eigen::MatrixXd Psi(dofs, n_e);
  Psi << psi, phi;
  reduction.elastic_count = static_cast<unsigned int>(n_e);
  reduction.static_count = static_cast<unsigned int>(r_count);

  /* the points: the nodes, then the attachments */
  const size_t points = mesh.nodes.size() + attachments.size();
  reduction.positions = mesh.nodes;
  reduction.masses = assembled.masses;
  reduction.shapes.assign(points * n_e, ksl_screw_t{});
  for(Eigen::Index n = 0; n < node_count; n++) {
    for(Eigen::Index j = 0; j < n_e; j++) {
      ksl_screw_t& s = reduction.shapes[n * n_e + j];
      s.lin = {Psi(3 * n, j), Psi(3 * n + 1, j), Psi(3 * n + 2, j)};
      s.ang = {0.0, 0.0, 0.0};
    }
  }
  for(size_t a = 0; a < attachments.size(); a++) {
    const unsigned int p = static_cast<unsigned int>(mesh.nodes.size() + a);
    reduction.attachment_points.push_back(p);
    reduction.positions.push_back(attachments[a].position);
    reduction.masses.push_back(0.0);
    if(column[a] >= 0) {
      for(int j = 0; j < 6; j++) {
        ksl_screw_t& s = reduction.shapes[p * n_e + column[a] + j];
        s.at[j] = 1.0;
      }
    }
  }

  const Eigen::MatrixXd k_eta = Psi.transpose() * (K * Psi);
  const Eigen::MatrixXd m_eta = Psi.transpose() * (M * Psi);
  if(!Psi.allFinite() || !k_eta.allFinite() || !m_eta.allFinite()) {
    throw shift::core::Exception(
      "Reduce: the shapes, stiffness or mass are not finite");
  }
  reduction.stiffness.resize(n_e * n_e);
  reduction.mass.resize(n_e * n_e);
  for(Eigen::Index i = 0; i < n_e; i++) {
    for(Eigen::Index j = 0; j < n_e; j++) {
      reduction.stiffness[i * n_e + j] = 0.5 * (k_eta(i, j) + k_eta(j, i));
      reduction.mass[i * n_e + j] = 0.5 * (m_eta(i, j) + m_eta(j, i));
    }
  }
  return reduction;
}

shift::core::fea::Reduction shift::core::fea::Reduce(
  const TetMesh& mesh, const Assembled& assembled,
  const std::vector<Attachment>& attachments, unsigned int reference,
  const std::vector<int>& retained_nodes, unsigned int modes_computed,
  const std::vector<unsigned int>& modes_kept) {
  return ReduceModes(mesh, assembled, attachments, reference, retained_nodes,
                     modes_computed, modes_kept, false);
}

shift::core::fea::HeldCompliance::HeldCompliance(
  const TetMesh& mesh, const Assembled& assembled, const Attachment& reference)
  : mesh_(&mesh) {
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(mesh.nodes.size());
  /* the reference holds its nodes (kRbe2) or eliminates six of their
  coordinates, u_p = -P^-1 Rest u_rest, so that its mean motion is zero */
  std::vector<char> held(size_t(dofs), 0);
  std::vector<Eigen::Index> dof;
  std::vector<int> pivot;
  Eigen::MatrixXd coupling;
  if(reference.spider == Attachment::Spider::kRbe2) {
    for(int node : reference.nodes) {
      for(int d = 0; d < 3; d++) {
        held[size_t(3 * node + d)] = 1;
      }
    }
  } else {
    const Eigen::MatrixXd G = AveragingMatrix(mesh, reference, 0, &dof);
    pivot = AveragingPivots(G, 0);
    Eigen::Matrix<double, 6, 6> P;
    Eigen::MatrixXd rest = G;
    for(int k = 0; k < 6; k++) {
      P.col(k) = G.col(pivot[k]);
      rest.col(pivot[k]).setZero();
      held[size_t(dof[size_t(pivot[k])])] = 1;
    }
    coupling = P.inverse() * rest;
  }
  std::vector<Eigen::Index> o_index(size_t(dofs), -1);
  Eigen::Index o_count = 0;
  std::vector<Eigen::Triplet<double>> t;
  for(Eigen::Index d = 0; d < dofs; d++) {
    if(!held[size_t(d)]) {
      o_index[size_t(d)] = o_count;
      t.emplace_back(d, o_count++, 1.0);
    }
  }
  for(size_t k = 0; k < pivot.size(); k++) {
    for(size_t i = 0; i < dof.size(); i++) {
      const Eigen::Index o = o_index[size_t(dof[i])];
      if(o >= 0 && coupling(Eigen::Index(k), Eigen::Index(i)) != 0.0) {
        t.emplace_back(dof[size_t(pivot[k])], o,
                       -coupling(Eigen::Index(k), Eigen::Index(i)));
      }
    }
  }
  T_.resize(dofs, o_count);
  T_.setFromTriplets(t.begin(), t.end());
  const Eigen::SparseMatrix<double> K_oo =
    T_.transpose() * (assembled.stiffness * T_);
  ldlt_.compute(K_oo);
  if(ldlt_.info() != Eigen::Success ||
     ldlt_.vectorD().minCoeff() <= 1e-12 * ldlt_.vectorD().maxCoeff()) {
    throw shift::core::Exception(
      "Compliance: the stiffness with the reference attachment held is "
      "singular (does the reference attachment hold the body still?)");
  }
}

std::vector<double> shift::core::fea::HeldCompliance::Compliance(
  const std::vector<Attachment>& attachments) const {
  const Eigen::Index n = 6 * static_cast<Eigen::Index>(attachments.size());
  std::vector<Eigen::Triplet<double>> g;
  for(size_t a = 0; a < attachments.size(); a++) {
    if(attachments[a].spider != Attachment::Spider::kRbe3) {
      throw shift::core::Exception(
        "Compliance: attachment " + std::to_string(a) + " is not averaging");
    }
    std::vector<Eigen::Index> dof;
    const Eigen::MatrixXd G = AveragingMatrix(*mesh_, attachments[a], a, &dof);
    for(Eigen::Index r = 0; r < 6; r++) {
      for(size_t i = 0; i < dof.size(); i++) {
        if(G(r, Eigen::Index(i)) != 0.0) {
          g.emplace_back(6 * Eigen::Index(a) + r, dof[i],
                         G(r, Eigen::Index(i)));
        }
      }
    }
  }
  Eigen::SparseMatrix<double> G_all(n, T_.rows());
  G_all.setFromTriplets(g.begin(), g.end());
  const Eigen::SparseMatrix<double> GT = G_all * T_;
  const Eigen::MatrixXd rhs = Eigen::MatrixXd(GT.transpose());
  const Eigen::MatrixXd x = ldlt_.solve(rhs);
  const Eigen::MatrixXd c = GT * x;
  std::vector<double> out(size_t(n * n));
  for(Eigen::Index i = 0; i < n; i++) {
    for(Eigen::Index j = 0; j < n; j++) {
      out[size_t(i * n + j)] = 0.5 * (c(i, j) + c(j, i));
    }
  }
  return out;
}

shift::core::fea::Reduction shift::core::fea::ReduceKeepingAll(
  const TetMesh& mesh, const Assembled& assembled,
  const std::vector<Attachment>& attachments, unsigned int reference,
  const std::vector<int>& retained_nodes, unsigned int modes_computed) {
  return ReduceModes(mesh, assembled, attachments, reference, retained_nodes,
                     modes_computed, {}, true);
}

std::vector<double> shift::core::fea::ModalDamping(
  const Reduction& reduction, const std::vector<double>& ratios) {
  const Eigen::Index n = reduction.elastic_count;
  if(ratios.empty() || n == 0) {
    return {};
  }
  const Eigen::MatrixXd K =
    Eigen::Map<const Eigen::MatrixXd>(reduction.stiffness.data(), n, n);
  const Eigen::MatrixXd M =
    Eigen::Map<const Eigen::MatrixXd>(reduction.mass.data(), n, n);
  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eigen(K, M);
  if(eigen.info() != Eigen::Success) {
    throw shift::core::Exception(
      "ModalDamping: the modes of the reduced body were not found");
  }
  const Eigen::MatrixXd& Phi = eigen.eigenvectors();
  Eigen::VectorXd d(n);
  for(Eigen::Index k = 0; k < n; k++) {
    const double zeta =
      ratios[std::min<size_t>(static_cast<size_t>(k), ratios.size() - 1)];
    d(k) = 2.0 * zeta * std::sqrt(std::max(eigen.eigenvalues()(k), 0.0));
  }
  const Eigen::MatrixXd MPhi = M * Phi;
  const Eigen::MatrixXd C = MPhi * d.asDiagonal() * MPhi.transpose();
  std::vector<double> c(n * n);
  for(Eigen::Index i = 0; i < n; i++) {
    for(Eigen::Index j = 0; j < n; j++) {
      c[i * n + j] = 0.5 * (C(i, j) + C(j, i));
    }
  }
  return c;
}
