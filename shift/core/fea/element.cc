#include "shift/core/fea/element.h"

#include <cmath>
#include <string>
#include <unordered_map>

#include "ksl/vec3.h"

#include "shift/core/exception.h"

/* the derivatives of the 10 shape functions with respect to the natural
coordinates (xi, eta, zeta) at (L1, L2, L3) = (xi, eta, zeta), L0 = 1 - xi -
eta - zeta: corners L_i (2 L_i - 1), mid-sides 4 L_i L_j on the edges 01, 12,
20, 03, 13, 23 */
static void Tet10Derivatives(double xi, double eta, double zeta,
                             double dn[10][3]) {
  const double L[4] = {1.0 - xi - eta - zeta, xi, eta, zeta};
  /* dL_i / d(xi, eta, zeta) */
  static const double kDL[4][3] = {
    {-1.0, -1.0, -1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  static const int kEdges[6][2] = {{0, 1}, {1, 2}, {2, 0},
                                   {0, 3}, {1, 3}, {2, 3}};
  for(int i = 0; i < 4; i++) {
    for(int c = 0; c < 3; c++) {
      dn[i][c] = (4.0 * L[i] - 1.0) * kDL[i][c];
    }
  }
  for(int k = 0; k < 6; k++) {
    const int i = kEdges[k][0], j = kEdges[k][1];
    for(int c = 0; c < 3; c++) {
      dn[4 + k][c] = 4.0 * (L[j] * kDL[i][c] + L[i] * kDL[j][c]);
    }
  }
}

/* the derivatives of the 10 shape functions with respect to x, y, z,
g[n][r], at the natural coordinates (xi, eta, zeta); false if the Jacobian
there is not positive, else det the Jacobian's determinant */
static bool Tet10Gradients(const std::array<ksl_vec3_t, 10>& nodes,
                           double xi, double eta, double zeta,
                           double g[10][3], double* det) {
  double dn[10][3];
  Tet10Derivatives(xi, eta, zeta, dn);
  /* the columns of the Jacobian, d x / d xi_c */
  ksl_vec3_t column[3] = {};
  for(int n = 0; n < 10; n++) {
    for(int c = 0; c < 3; c++) {
      ksl_axpy_vv(dn[n][c], &nodes[n], &column[c]);
    }
  }
  /* its inverse by the cross products of its columns (no pivots): row c,
  d xi_c / d x, is column[c + 1] x column[c + 2] / det */
  ksl_vec3_t row[3];
  for(int c = 0; c < 3; c++) {
    ksl_cross_vv(&column[(c + 1) % 3], &column[(c + 2) % 3], &row[c]);
  }
  *det = ksl_dot_vv(&column[0], &row[0]);
  if(!(*det > 0.0)) {
    return false;
  }
  /* dN/dx_r = sum_c dN/dxi_c dxi_c/dx_r */
  for(int n = 0; n < 10; n++) {
    for(int r = 0; r < 3; r++) {
      g[n][r] = (dn[n][0] * row[0].at[r] + dn[n][1] * row[1].at[r] +
                 dn[n][2] * row[2].at[r]) /
                *det;
    }
  }
  return true;
}

std::vector<double> shift::core::fea::NodalStresses(
  const TetMesh& mesh, const Material& material, const std::vector<int>& nodes,
  const std::vector<ksl_screw_t>& shapes, size_t columns) {
  const double E = material.youngs_modulus, nu = material.poisson_ratio;
  const double lambda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
  const double mu = E / (2.0 * (1.0 + nu));
  std::unordered_map<int, size_t> index;
  for(size_t i = 0; i < nodes.size(); i++) {
    index.emplace(nodes[i], i);
  }
  std::vector<double> stresses(6 * nodes.size() * columns, 0.0);
  std::vector<double> weights(nodes.size(), 0.0);
  /* the natural coordinates of the nodes, in the order of TetMesh */
  static const double kNodes[10][3] = {
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
    {0.5, 0.0, 0.0}, {0.5, 0.5, 0.0}, {0.0, 0.5, 0.0}, {0.0, 0.0, 0.5},
    {0.5, 0.0, 0.5}, {0.0, 0.5, 0.5}};
  for(const std::array<int, 10>& t : mesh.tets) {
    std::array<ksl_vec3_t, 10> x;
    for(int a = 0; a < 10; a++) {
      x[a] = mesh.nodes[t[a]];
    }
    /* the element's weight: its volume, so that a small badly shaped
    element (whose stresses are the least accurate) counts the least */
    double centre[10][3], volume;
    if(!Tet10Gradients(x, 0.25, 0.25, 0.25, centre, &volume)) {
      continue;
    }
    for(int a = 0; a < 10; a++) {
      auto it = index.find(t[a]);
      if(it == index.end()) {
        continue;
      }
      double g[10][3], det;
      if(!Tet10Gradients(x, kNodes[a][0], kNodes[a][1], kNodes[a][2], g,
                         &det)) {
        continue;
      }
      const size_t i = it->second;
      weights[i] += volume;
      for(size_t j = 0; j < columns; j++) {
        /* the displacement gradient H[r][c] = d u_r / d x_c */
        double H[3][3] = {};
        for(int b = 0; b < 10; b++) {
          const ksl_vec3_t& u = shapes[size_t(t[b]) * columns + j].lin;
          for(int r = 0; r < 3; r++) {
            for(int c = 0; c < 3; c++) {
              H[r][c] += u.at[r] * g[b][c];
            }
          }
        }
        const double trace = H[0][0] + H[1][1] + H[2][2];
        double* s = stresses.data() + 6 * (i * columns + j);
        for(int r = 0; r < 3; r++) {
          s[r] += volume * (lambda * trace + 2.0 * mu * H[r][r]);
        }
        s[3] += volume * mu * (H[0][1] + H[1][0]);
        s[4] += volume * mu * (H[1][2] + H[2][1]);
        s[5] += volume * mu * (H[2][0] + H[0][2]);
      }
    }
  }
  for(size_t i = 0; i < nodes.size(); i++) {
    if(weights[i] > 0.0) {
      for(size_t k = 6 * i * columns; k < 6 * (i + 1) * columns; k++) {
        stresses[k] /= weights[i];
      }
    }
  }
  return stresses;
}

bool shift::core::fea::Tet10Stiffness(const std::array<ksl_vec3_t, 10>& nodes,
                                      const Material& material,
                                      double* stiffness, double* volume) {
  const double E = material.youngs_modulus, nu = material.poisson_ratio;
  const double lambda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
  const double mu = E / (2.0 * (1.0 + nu));
  for(int i = 0; i < 900; i++) {
    stiffness[i] = 0.0;
  }
  *volume = 0.0;
  /* the 4-point rule of degree 2: weights 1/24 */
  const double a = 0.5854101966249685, b = 0.1381966011250105;
  const double points[4][3] = {{b, b, b}, {a, b, b}, {b, a, b}, {b, b, a}};
  for(const auto& q : points) {
    double dn[10][3];
    Tet10Derivatives(q[0], q[1], q[2], dn);
    /* J[r][c] = d x_r / d xi_c */
    double J[3][3] = {};
    for(int n = 0; n < 10; n++) {
      const double x[3] = {nodes[n].x, nodes[n].y, nodes[n].z};
      for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 3; c++) {
          J[r][c] += x[r] * dn[n][c];
        }
      }
    }
    const double det =
      J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) -
      J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
      J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
    if(!(det > 0.0)) {
      return false;
    }
    /* inverse: Ji[c][r] = d xi_c / d x_r */
    double Ji[3][3];
    Ji[0][0] = (J[1][1] * J[2][2] - J[1][2] * J[2][1]) / det;
    Ji[0][1] = (J[0][2] * J[2][1] - J[0][1] * J[2][2]) / det;
    Ji[0][2] = (J[0][1] * J[1][2] - J[0][2] * J[1][1]) / det;
    Ji[1][0] = (J[1][2] * J[2][0] - J[1][0] * J[2][2]) / det;
    Ji[1][1] = (J[0][0] * J[2][2] - J[0][2] * J[2][0]) / det;
    Ji[1][2] = (J[0][2] * J[1][0] - J[0][0] * J[1][2]) / det;
    Ji[2][0] = (J[1][0] * J[2][1] - J[1][1] * J[2][0]) / det;
    Ji[2][1] = (J[0][1] * J[2][0] - J[0][0] * J[2][1]) / det;
    Ji[2][2] = (J[0][0] * J[1][1] - J[0][1] * J[1][0]) / det;
    /* dN/dx_r = sum_c dN/dxi_c dxi_c/dx_r */
    double g[10][3];
    for(int n = 0; n < 10; n++) {
      for(int r = 0; r < 3; r++) {
        g[n][r] = dn[n][0] * Ji[0][r] + dn[n][1] * Ji[1][r] +
                  dn[n][2] * Ji[2][r];
      }
    }
    const double w = det / 24.0;
    *volume += w;
    /* B^T D B with D = lambda m m^T + mu diag(2, 2, 2, 1, 1, 1): for the
    node pair (m, n) the 3 x 3 block K_mn[i][j] = lambda g_m,i g_n,j +
    mu (g_m,j g_n,i + delta_ij g_m . g_n) */
    for(int m = 0; m < 10; m++) {
      for(int n = 0; n < 10; n++) {
        const double dot =
          g[m][0] * g[n][0] + g[m][1] * g[n][1] + g[m][2] * g[n][2];
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++) {
            double k = lambda * g[m][i] * g[n][j] + mu * g[m][j] * g[n][i];
            if(i == j) {
              k += mu * dot;
            }
            stiffness[(3 * m + i) * 30 + 3 * n + j] += w * k;
          }
        }
      }
    }
  }
  return true;
}

std::array<double, 10> shift::core::fea::Tet10LumpedMass(double volume,
                                                         double density) {
  std::array<double, 10> m;
  const double mass = density * volume;
  for(int i = 0; i < 4; i++) {
    m[i] = mass / 36.0;
  }
  for(int i = 4; i < 10; i++) {
    m[i] = 4.0 * mass / 27.0;
  }
  return m;
}

shift::core::fea::Assembled shift::core::fea::Assemble(
  const TetMesh& mesh, const Material& material) {
  if(!(material.youngs_modulus > 0.0) || !(material.density > 0.0) ||
     !(material.poisson_ratio > -1.0 && material.poisson_ratio < 0.5)) {
    throw shift::core::Exception(
      "Assemble: the material needs a positive Young's modulus and density "
      "and a Poisson's ratio between -1 and 0.5");
  }
  const size_t n = mesh.nodes.size();
  Assembled assembled;
  assembled.masses.assign(n, 0.0);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(mesh.tets.size() * 900);
  double k[900];
  for(size_t e = 0; e < mesh.tets.size(); e++) {
    const std::array<int, 10>& t = mesh.tets[e];
    std::array<ksl_vec3_t, 10> x;
    for(int a = 0; a < 10; a++) {
      x[a] = mesh.nodes[t[a]];
    }
    double volume = 0.0;
    if(!Tet10Stiffness(x, material, k, &volume)) {
      throw shift::core::Exception(
        "Assemble: element " + std::to_string(e) +
        " of the mesh is turned inside out (its Jacobian is not positive)");
    }
    assembled.volume += volume;
    const std::array<double, 10> m = Tet10LumpedMass(volume, material.density);
    for(int a = 0; a < 10; a++) {
      assembled.masses[t[a]] += m[a];
      for(int b = 0; b < 10; b++) {
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++) {
            triplets.emplace_back(3 * t[a] + i, 3 * t[b] + j,
                                  k[(3 * a + i) * 30 + 3 * b + j]);
          }
        }
      }
    }
  }
  assembled.stiffness.resize(static_cast<Eigen::Index>(3 * n),
                             static_cast<Eigen::Index>(3 * n));
  assembled.stiffness.setFromTriplets(triplets.begin(), triplets.end());
  return assembled;
}
