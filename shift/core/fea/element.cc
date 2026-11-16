#include "shift/core/fea/element.h"

#include <cmath>
#include <string>

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
