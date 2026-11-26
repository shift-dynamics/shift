/* shift::core::fea::Reduce on a steel cantilever of 10-node tetrahedra,
1 m long and 0.1 m square, its end at x = 0 held by a rigid attachment (the
reference) and its tip tied to an averaging one:

- the fixed-interface modes with only the root attachment are the
  cantilever's: the lowest bending frequencies against Euler-Bernoulli
  (within the shear and rotary inertia of a beam 10 times longer than deep);
- the stiffness reduced to the tip is the cantilever's, PL^3 / 3EI plus the
  shear deflection PL / (k G A);
- Craig-Bampton's coordinates: unit modal mass, K^eta block diagonal (the
  static modes do no work on the fixed-interface modes);
- Guyan's reduction (no modes) gives frequencies above Craig-Bampton's;
- attachments that share nodes, or a reference that does not hold the body,
  are refused;
- the stress recovered from the static modes (fea::NodalStresses) under a
  tip load converges to the beam's bending stress as the mesh is refined,
  away from the attachments' patches;
- the divide-by-zero trap is suspended while the reduction runs and
  restored after it (fea::DivideByZeroTrapSuspension). */

#include <array>
#include <cfenv>
#include <cmath>
#include <map>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"

#if SHIFT_HAVE_FEA

#include <Eigen/Dense>

#include "shift/core/exception.h"
#include "shift/core/fea/element.h"
#include "shift/core/fea/reduction.h"
#include "shift/core/fea/tet_mesh.h"
#include "shift/core/fea/trap_suspension.h"

class FeaReduction : public ::testing::Test {
protected:
  static constexpr double kLength = 1.0;
  static constexpr double kDepth = 0.1;

  static shift::core::fea::Material Steel() {
    shift::core::fea::Material steel;
    steel.youngs_modulus = 2.1e11;
    steel.poisson_ratio = 0.3;
    steel.density = 7850.0;
    return steel;
  }

  /* the box [0, L] x [0, b] x [0, b], its sides grids of squares of side
  h cut into triangles, normals out */
  static shift::core::fea::ClosedSurface Box(int nx, int nyz) {
    shift::core::fea::ClosedSurface s;
    const int n[3] = {nx, nyz, nyz};
    const double size[3] = {kLength, kDepth, kDepth};
    std::map<std::array<int, 3>, int> index;
    auto point = [&](std::array<int, 3> g) {
      auto found = index.find(g);
      if(found != index.end()) {
        return found->second;
      }
      const int i = static_cast<int>(s.points.size());
      s.points.push_back({size[0] * g[0] / n[0], size[1] * g[1] / n[1],
                          size[2] * g[2] / n[2]});
      index.emplace(g, i);
      return i;
    };
    for(int f = 0; f < 6; f++) {
      const int axis = f / 2;
      const bool high = f % 2 == 1;
      const int u = (axis + 1) % 3, v = (axis + 2) % 3;
      for(int i = 0; i < n[u]; i++) {
        for(int j = 0; j < n[v]; j++) {
          int q[4];
          const int du[4] = {0, 1, 1, 0}, dv[4] = {0, 0, 1, 1};
          for(int k = 0; k < 4; k++) {
            std::array<int, 3> g;
            g[axis] = high ? n[axis] : 0;
            g[u] = i + du[k];
            g[v] = j + dv[k];
            q[k] = point(g);
          }
          if(high) {
            s.triangles.push_back({q[0], q[1], q[2]});
            s.triangles.push_back({q[0], q[2], q[3]});
          } else {
            s.triangles.push_back({q[0], q[2], q[1]});
            s.triangles.push_back({q[0], q[3], q[2]});
          }
          s.face.push_back(f);
          s.face.push_back(f);
        }
      }
    }
    return s;
  }

  static const shift::core::fea::TetMesh& Mesh() {
    static const shift::core::fea::TetMesh mesh =
      shift::core::fea::MeshVolume(Box(20, 2), 0.05);
    return mesh;
  }

  static const shift::core::fea::Assembled& Matrices() {
    static const shift::core::fea::Assembled assembled =
      shift::core::fea::Assemble(Mesh(), Steel());
    return assembled;
  }

  /* the nodes on the end x = at */
  static shift::core::fea::Attachment End(
    double at, shift::core::fea::Attachment::Spider spider) {
    shift::core::fea::Attachment a;
    a.position = {at, 0.5 * kDepth, 0.5 * kDepth};
    a.spider = spider;
    for(size_t i = 0; i < Mesh().nodes.size(); i++) {
      if(std::fabs(Mesh().nodes[i].x - at) < 1e-9) {
        a.nodes.push_back(static_cast<int>(i));
      }
    }
    return a;
  }

  static shift::core::fea::Attachment Root() {
    return End(0.0, shift::core::fea::Attachment::Spider::kRbe2);
  }
  static shift::core::fea::Attachment Tip() {
    return End(kLength, shift::core::fea::Attachment::Spider::kRbe3);
  }

  static double SecondMoment() {
    return std::pow(kDepth, 4) / 12.0;
  }
};

TEST_F(FeaReduction, CantileverFrequencies) {
  const shift::core::fea::Reduction r =
    shift::core::fea::Reduce(Mesh(), Matrices(), {Root()}, 0, {}, 6, {0, 1});
  ASSERT_EQ(r.frequencies.size(), 6u);
  const shift::core::fea::Material steel = Steel();
  const double area = kDepth * kDepth;
  /* (beta L)^2 / (2 pi) sqrt(EI / (rho A L^4)), beta L = 1.8751, 4.6941;
  the square section bends alike about y and z, so the lowest four modes
  are two pairs */
  const double root = std::sqrt(steel.youngs_modulus * SecondMoment() /
                                (steel.density * area * std::pow(kLength, 4)));
  const double f1 = 1.8751 * 1.8751 / (2.0 * M_PI) * root;
  const double f2 = 4.6941 * 4.6941 / (2.0 * M_PI) * root;
  EXPECT_NEAR(r.frequencies[0] / f1, 1.0, 0.02);
  EXPECT_NEAR(r.frequencies[1] / f1, 1.0, 0.02);
  /* the second bending pair is lowered more by shear and rotary inertia;
  the torsion mode lies between */
  int second = 0;
  for(double f : r.frequencies) {
    second += std::fabs(f / f2 - 1.0) < 0.06 ? 1 : 0;
  }
  EXPECT_EQ(second, 2);

  /* two kept modes, unit modal mass, K^eta = diag(omega^2) */
  ASSERT_EQ(r.elastic_count, 2u);
  EXPECT_EQ(r.static_count, 0u);
  for(int i = 0; i < 2; i++) {
    const double w = 2.0 * M_PI * r.frequencies[i];
    EXPECT_NEAR(r.mass[i * 2 + i], 1.0, 1e-8);
    EXPECT_NEAR(r.stiffness[i * 2 + i] / (w * w), 1.0, 1e-8);
  }
  EXPECT_NEAR(r.mass[1], 0.0, 1e-8);
}

TEST_F(FeaReduction, TipStiffnessIsTheCantilevers) {
  const shift::core::fea::Reduction r = shift::core::fea::Reduce(
    Mesh(), Matrices(), {Root(), Tip()}, 0, {}, 4, {0, 1, 2, 3});
  ASSERT_EQ(r.static_count, 6u);
  ASSERT_EQ(r.elastic_count, 10u);
  const Eigen::Index n = r.elastic_count;
  const Eigen::MatrixXd K =
    Eigen::Map<const Eigen::MatrixXd>(r.stiffness.data(), n, n);
  /* the static modes do no work on the fixed-interface modes */
  EXPECT_LT(K.block(0, 6, 6, 4).cwiseAbs().maxCoeff(),
            1e-6 * K.block(0, 0, 6, 6).cwiseAbs().maxCoeff());

  /* the tip's compliance in y: bending plus shear (k = 5/6) */
  const Eigen::MatrixXd compliance = K.block(0, 0, 6, 6).inverse();
  const shift::core::fea::Material steel = Steel();
  const double G =
    steel.youngs_modulus / (2.0 * (1.0 + steel.poisson_ratio));
  const double expected =
    std::pow(kLength, 3) / (3.0 * steel.youngs_modulus * SecondMoment()) +
    kLength / (5.0 / 6.0 * G * kDepth * kDepth);
  EXPECT_NEAR(compliance(1, 1) / expected, 1.0, 0.03);
  EXPECT_NEAR(compliance(2, 2) / expected, 1.0, 0.03);
  /* the tip's point moves with its static modes */
  const unsigned int tip = r.attachment_points[1];
  for(int j = 0; j < 6; j++) {
    EXPECT_EQ(r.shapes[tip * n + j].at[j], 1.0);
  }
  /* the root's point and the nodes it holds do not move */
  const unsigned int root = r.attachment_points[0];
  for(Eigen::Index j = 0; j < n; j++) {
    for(int d = 0; d < 6; d++) {
      EXPECT_EQ(r.shapes[root * n + j].at[d], 0.0);
    }
  }
}

/* Guyan's reduction to the tip and three nodes along the beam bounds the
lowest frequency from above, and adding the fixed-interface modes brings it
down to the full model's */
TEST_F(FeaReduction, GuyanBoundsCraigBampton) {
  std::vector<int> retained;
  for(double x : {0.25, 0.5, 0.75}) {
    for(size_t i = 0; i < Mesh().nodes.size(); i++) {
      const ksl_vec3_t& p = Mesh().nodes[i];
      if(std::fabs(p.x - x) < 1e-9 && std::fabs(p.y - 0.05) < 1e-9 &&
         std::fabs(p.z - 0.05) < 1e-9) {
        retained.push_back(static_cast<int>(i));
      }
    }
  }
  /* the centre line may not be a mesh line; then the nearest nodes */
  if(retained.size() < 3) {
    retained.clear();
    for(double x : {0.25, 0.5, 0.75}) {
      double best = 1e300;
      int nearest = -1;
      for(size_t i = 0; i < Mesh().nodes.size(); i++) {
        const ksl_vec3_t& p = Mesh().nodes[i];
        const double d = std::hypot(p.x - x, p.y - 0.05, p.z - 0.05);
        if(d < best) {
          best = d;
          nearest = static_cast<int>(i);
        }
      }
      retained.push_back(nearest);
    }
  }
  auto lowest = [](const shift::core::fea::Reduction& r) {
    const Eigen::Index n = r.elastic_count;
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
      Eigen::Map<const Eigen::MatrixXd>(r.stiffness.data(), n, n),
      Eigen::Map<const Eigen::MatrixXd>(r.mass.data(), n, n));
    return std::sqrt(eigen.eigenvalues()(0)) / (2.0 * M_PI);
  };
  const shift::core::fea::Reduction guyan = shift::core::fea::Reduce(
    Mesh(), Matrices(), {Root(), Tip()}, 0, retained, 0, {});
  EXPECT_EQ(guyan.elastic_count, 6u + 9u);
  const shift::core::fea::Reduction full =
    shift::core::fea::Reduce(Mesh(), Matrices(), {Root()}, 0, {}, 1, {0});
  const shift::core::fea::Reduction cb = shift::core::fea::Reduce(
    Mesh(), Matrices(), {Root(), Tip()}, 0, retained, 4, {0, 1, 2, 3});
  const double f = full.frequencies[0];
  EXPECT_GT(lowest(guyan), lowest(cb) * (1.0 - 1e-9));
  EXPECT_GT(lowest(cb), f * (1.0 - 1e-6));
  EXPECT_LT(lowest(cb) / f - 1.0, 1e-3);
  EXPECT_LT(lowest(guyan) / f - 1.0, 2e-2);
}

/* modal damping: the reduced body's modes are damped at the ratios given */
TEST_F(FeaReduction, ModalDampingRatios) {
  const shift::core::fea::Reduction r = shift::core::fea::Reduce(
    Mesh(), Matrices(), {Root(), Tip()}, 0, {}, 2, {0, 1});
  const std::vector<double> c = shift::core::fea::ModalDamping(r, {0.01, 0.05});
  const Eigen::Index n = r.elastic_count;
  const Eigen::MatrixXd K =
    Eigen::Map<const Eigen::MatrixXd>(r.stiffness.data(), n, n);
  const Eigen::MatrixXd M =
    Eigen::Map<const Eigen::MatrixXd>(r.mass.data(), n, n);
  const Eigen::MatrixXd C = Eigen::Map<const Eigen::MatrixXd>(c.data(), n, n);
  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eigen(K, M);
  for(Eigen::Index k = 0; k < n; k++) {
    const Eigen::VectorXd v = eigen.eigenvectors().col(k);
    const double zeta = v.dot(C * v) /
                        (2.0 * std::sqrt(eigen.eigenvalues()(k)) * v.dot(M * v));
    EXPECT_NEAR(zeta, k == 0 ? 0.01 : 0.05, 1e-9);
  }
}

TEST_F(FeaReduction, RefusesBadAttachments) {
  shift::core::fea::Attachment twice = Root();
  twice.spider = shift::core::fea::Attachment::Spider::kRbe3;
  EXPECT_THROW(shift::core::fea::Reduce(Mesh(), Matrices(), {Root(), twice},
                                        0, {}, 0, {}),
               shift::core::Exception);
  /* a reference tied to a single node holds no rotation */
  shift::core::fea::Attachment point = Root();
  point.nodes.resize(1);
  EXPECT_THROW(
    shift::core::fea::Reduce(Mesh(), Matrices(), {point}, 0, {}, 1, {0}),
    shift::core::Exception);
}

/* under a tip load P in y, the bending stress sigma_xx = P (L - x) (b / 2) /
I in magnitude at the top and bottom faces, at 2 and 5 depths from the root
(rigid, RBE2) attachment, from the stress of the static modes of the
reduction: closer as the mesh is refined (0.28 %, 0.018 % and 0.009 % at
x = 0.5 for 2, 3 and 4 elements through the depth). At the root itself the
rigid patch holds the end's section from contracting, and its stress moves
away from the beam's as the mesh is refined (3.3 %, 8.2 % and 9.3 %), while
half a depth away it is already converging (1.6 %, 1.4 % and 0.6 %); those
are not checked */
TEST_F(FeaReduction, StressAwayFromTheAttachmentsConverges) {
  const double load = 1000.0;
  const double stations[2] = {0.2, 0.5};
  double errors[3][2] = {};
  const int divisions[3] = {2, 3, 4}; /* through the depth */
  for(int m = 0; m < 3; m++) {
    const int nyz = divisions[m];
    const double h = kDepth / nyz;
    const shift::core::fea::TetMesh mesh = shift::core::fea::MeshVolume(
      Box(int(std::lround(kLength / h)), nyz), h);
    const shift::core::fea::Assembled assembled =
      shift::core::fea::Assemble(mesh, Steel());
    std::vector<shift::core::fea::Attachment> ends(2);
    for(int e = 0; e < 2; e++) {
      ends[e].position = {e * kLength, 0.5 * kDepth, 0.5 * kDepth};
      ends[e].spider = e == 0 ? shift::core::fea::Attachment::Spider::kRbe2
                              : shift::core::fea::Attachment::Spider::kRbe3;
      for(size_t i = 0; i < mesh.nodes.size(); i++) {
        if(std::fabs(mesh.nodes[i].x - e * kLength) < 1e-9) {
          ends[e].nodes.push_back(static_cast<int>(i));
        }
      }
    }
    const shift::core::fea::Reduction r =
      shift::core::fea::Reduce(mesh, assembled, ends, 0, {}, 0, {});
    ASSERT_EQ(r.static_count, 6u);
    const Eigen::Index n = r.elastic_count;
    const Eigen::MatrixXd K =
      Eigen::Map<const Eigen::MatrixXd>(r.stiffness.data(), n, n);
    /* the tip's motion under the load: u = K^-1 f */
    Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
    f(1) = load;
    const Eigen::VectorXd u = K.ldlt().solve(f);
    for(int k = 0; k < 2; k++) {
      std::vector<int> nodes;
      for(size_t i = 0; i < mesh.nodes.size(); i++) {
        const ksl_vec3_t& x = mesh.nodes[i];
        if(std::fabs(x.x - stations[k]) < 1e-9 &&
           (std::fabs(x.y) < 1e-9 || std::fabs(x.y - kDepth) < 1e-9)) {
          nodes.push_back(static_cast<int>(i));
        }
      }
      ASSERT_FALSE(nodes.empty());
      const std::vector<double> stresses = shift::core::fea::NodalStresses(
        mesh, Steel(), nodes, r.shapes, size_t(n));
      double sum = 0.0;
      for(size_t i = 0; i < nodes.size(); i++) {
        double sigma = 0.0;
        for(Eigen::Index j = 0; j < n; j++) {
          sigma += u(j) * stresses[6 * (i * size_t(n) + size_t(j))];
        }
        sum += std::fabs(sigma);
      }
      const double expected =
        load * (kLength - stations[k]) * 0.5 * kDepth / SecondMoment();
      errors[m][k] = std::fabs(sum / double(nodes.size()) / expected - 1.0);
    }
  }
  for(int k = 0; k < 2; k++) {
    EXPECT_LT(errors[2][k], errors[0][k]) << "x = " << stations[k];
    EXPECT_LT(errors[2][k], 1e-3) << "x = " << stations[k];
  }
}

/* the trap that shift-cli enables (SHIFT_FP_TRAP): suspended for the
lifetime of the object, a division by zero inside raises no signal and
leaves no flag, and the trap is enabled again after it. Eigen built for AVX
divides by zero in the unused lanes of the last rows of a SimplicialLDLT
solve, which trapped the reduction of the flexible slider-crank's
connecting rod in the release build. */
#if defined(SHIFT_FP_TRAP) && defined(__linux__)
TEST(FeaTrapSuspension, SuspendsAndRestoresTheTrap) {
  const int before = fegetexcept();
  feenableexcept(FE_DIVBYZERO);
  {
    const shift::core::fea::DivideByZeroTrapSuspension suspension;
    EXPECT_EQ(fegetexcept() & FE_DIVBYZERO, 0);
    volatile double zero = 0.0;
    volatile double quotient = 1.0 / zero;
    EXPECT_TRUE(std::isinf(quotient));
  }
  EXPECT_NE(fegetexcept() & FE_DIVBYZERO, 0);
  EXPECT_EQ(fetestexcept(FE_DIVBYZERO), 0);
  fedisableexcept(FE_ALL_EXCEPT);
  feenableexcept(before);
}
#endif

#endif
