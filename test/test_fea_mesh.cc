/* shift::core::fea: Netgen fills a closed triangulated surface with
tetrahedra, keeping the surface as the mesh boundary (MeshVolume); the
stiffness of 10-node tetrahedra stores no energy in rigid motions and passes
the patch test (a linear displacement field gives the exact strain energy
and no forces inside); the lumped masses add up to the body's; and a mesh of
a STEP solid with its boundary's mid-side nodes on the exact surfaces
(MeshSolid) has the solid's volume. A triangle mesh's split corners are
joined (WeldSurface) and Netgen remakes its triangles (RemeshSurface);
points inside are kept as nodes. */

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <map>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"

#if SHIFT_HAVE_FEA

#include "shift/core/exception.h"
#include "shift/core/cad/repair.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/fea/element.h"
#include "shift/core/fea/solid_mesh.h"
#include "shift/core/fea/tet_mesh.h"

/* the cube [0, a]^3, each side an n x n grid of squares cut into two
triangles, normals out; face = side */
static shift::core::fea::ClosedSurface Cube(double a, int n) {
  shift::core::fea::ClosedSurface s;
  std::map<std::array<int, 3>, int> index;
  auto point = [&](std::array<int, 3> g) {
    auto found = index.find(g);
    if(found != index.end()) {
      return found->second;
    }
    const int i = static_cast<int>(s.points.size());
    s.points.push_back({a * g[0] / n, a * g[1] / n, a * g[2] / n});
    index.emplace(g, i);
    return i;
  };
  /* side f: the axis it is normal to, and whether it is at a */
  for(int f = 0; f < 6; f++) {
    const int axis = f / 2;
    const bool high = f % 2 == 1;
    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
    for(int i = 0; i < n; i++) {
      for(int j = 0; j < n; j++) {
        std::array<int, 3> c[4];
        const int du[4] = {0, 1, 1, 0}, dv[4] = {0, 0, 1, 1};
        for(int k = 0; k < 4; k++) {
          c[k][axis] = high ? n : 0;
          c[k][u] = i + du[k];
          c[k][v] = j + dv[k];
        }
        int q[4];
        for(int k = 0; k < 4; k++) {
          q[k] = point(c[k]);
        }
        /* (u, v, axis) is right-handed: counterclockwise in (u, v) faces
        +axis */
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

/* the volume of the cube, every tetrahedron positive, the boundary kept, and
mid-side nodes at the middle of each edge */
TEST(FeaMesh, FillsACube) {
  const shift::core::fea::TetMesh mesh =
    shift::core::fea::MeshVolume(Cube(1.0, 4), 0.25);
  ASSERT_GT(mesh.tets.size(), 0u);
  EXPECT_NEAR(mesh.Volume(), 1.0, 1e-12);
  for(const std::array<int, 10>& t : mesh.tets) {
    const ksl_vec3_t& p0 = mesh.nodes[t[0]];
    ksl_vec3_t a, b, c, axb;
    ksl_subtract_vv(&mesh.nodes[t[1]], &p0, &a);
    ksl_subtract_vv(&mesh.nodes[t[2]], &p0, &b);
    ksl_subtract_vv(&mesh.nodes[t[3]], &p0, &c);
    ksl_cross_vv(&a, &b, &axb);
    EXPECT_GT(ksl_dot_vv(&axb, &c), 0.0);
    const ksl_vec3_t& m01 = mesh.nodes[t[4]];
    EXPECT_NEAR(m01.x, 0.5 * (p0.x + mesh.nodes[t[1]].x), 1e-15);
  }
  EXPECT_EQ(mesh.boundary.size(), 6u * 2 * 16);
  for(size_t f = 0; f < mesh.boundary.size(); f++) {
    EXPECT_EQ(mesh.boundary_face[f], static_cast<int>(f / 32));
  }
  /* interior nodes were added (the surface has 98 points) */
  EXPECT_GT(mesh.corner_count, 98u);
}

TEST(FeaMesh, RefusesAnOpenSurface) {
  shift::core::fea::ClosedSurface open = Cube(1.0, 2);
  open.triangles.pop_back();
  open.face.pop_back();
  EXPECT_THROW(shift::core::fea::MeshVolume(open, 0.5),
               shift::core::Exception);
}

/* the corners of a cube's triangles, each triangle with its own copies (as
a mesh file splits them), joined again: 8 points; remade by Netgen to a
finer size, still closed (MeshVolume fills it), with the cube's volume and
its six sides as faces */
TEST(FeaMesh, WeldsAndRemeshesASurface) {
  const shift::core::fea::ClosedSurface cube = Cube(1.0, 1);
  std::vector<ksl_vec3_t> corners;
  std::vector<std::array<int, 3>> split;
  for(const std::array<int, 3>& t : cube.triangles) {
    const int base = static_cast<int>(corners.size());
    for(int k : t) {
      ksl_vec3_t p = cube.points[size_t(k)];
      p.x += 1e-9; /* within the tolerance */
      corners.push_back(p);
    }
    split.push_back({base, base + 1, base + 2});
  }
  const shift::core::fea::ClosedSurface welded =
    shift::core::fea::WeldSurface(corners, split, 1e-6);
  EXPECT_EQ(welded.points.size(), 8u);
  EXPECT_EQ(welded.triangles.size(), 12u);

  const shift::core::fea::ClosedSurface remeshed =
    shift::core::fea::RemeshSurface(welded, 0.25, 0.0);
  EXPECT_GT(remeshed.triangles.size(), 6u * 2 * 16 / 2);
  int faces = 0;
  for(int f : remeshed.face) {
    faces = std::max(faces, f + 1);
  }
  EXPECT_EQ(faces, 6);
  const shift::core::fea::TetMesh mesh =
    shift::core::fea::MeshVolume(remeshed, 0.25);
  EXPECT_NEAR(mesh.Volume(), 1.0, 1e-9);
}

/* a point inside, clear of the surface, is a node of the mesh exactly; one
outside is not */
TEST(FeaMesh, KeepsInteriorPointsAsNodes) {
  const std::vector<ksl_vec3_t> points = {{0.37, 0.41, 0.53},
                                          {1.5, 0.5, 0.5}};
  const shift::core::fea::TetMesh mesh =
    shift::core::fea::MeshVolume(Cube(1.0, 4), 0.25, 0.0, points);
  ASSERT_EQ(mesh.interior_nodes.size(), 2u);
  ASSERT_GE(mesh.interior_nodes[0], 0);
  const ksl_vec3_t& node = mesh.nodes[size_t(mesh.interior_nodes[0])];
  EXPECT_EQ(node.x, 0.37);
  EXPECT_EQ(node.y, 0.41);
  EXPECT_EQ(node.z, 0.53);
  EXPECT_LT(mesh.interior_nodes[1], 0);
  EXPECT_NEAR(mesh.Volume(), 1.0, 1e-12);
}

/* steel */
static shift::core::fea::Material Steel() {
  shift::core::fea::Material steel;
  steel.youngs_modulus = 2.1e11;
  steel.poisson_ratio = 0.3;
  steel.density = 7850.0;
  return steel;
}

/* K u = 0 for the six rigid motions; and for u = A x, the strain energy
1/2 u^T K u of the constant strain eps = (A + A^T) / 2 over the volume, and
no force at the nodes inside */
TEST(FeaElement, RigidMotionsAndThePatchTest) {
  const shift::core::fea::TetMesh mesh =
    shift::core::fea::MeshVolume(Cube(0.1, 3), 0.04);
  const shift::core::fea::Material steel = Steel();
  const shift::core::fea::Assembled a =
    shift::core::fea::Assemble(mesh, steel);
  const Eigen::Index n = static_cast<Eigen::Index>(mesh.nodes.size());
  EXPECT_NEAR(a.volume, 1e-3, 1e-15);
  double mass = 0.0;
  for(double m : a.masses) {
    EXPECT_GT(m, 0.0);
    mass += m;
  }
  EXPECT_NEAR(mass, steel.density * 1e-3, 1e-12);

  const double scale = a.stiffness.norm();
  for(int motion = 0; motion < 6; motion++) {
    Eigen::VectorXd u(3 * n);
    for(Eigen::Index i = 0; i < n; i++) {
      const ksl_vec3_t& x = mesh.nodes[i];
      const double p[3] = {x.x, x.y, x.z};
      for(int d = 0; d < 3; d++) {
        u(3 * i + d) = motion == d ? 1.0 : 0.0;
      }
      if(motion >= 3) {
        /* the rotation about axis motion - 3: w x p */
        double w[3] = {0, 0, 0};
        w[motion - 3] = 1.0;
        u(3 * i + 0) = w[1] * p[2] - w[2] * p[1];
        u(3 * i + 1) = w[2] * p[0] - w[0] * p[2];
        u(3 * i + 2) = w[0] * p[1] - w[1] * p[0];
      }
    }
    EXPECT_LT((a.stiffness * u).norm(), 1e-9 * scale * u.norm()) << motion;
  }

  const double A[3][3] = {{1e-3, 2e-4, -3e-4}, {5e-4, -2e-4, 1e-4},
                          {-1e-4, 3e-4, 4e-4}};
  Eigen::VectorXd u(3 * n);
  for(Eigen::Index i = 0; i < n; i++) {
    const ksl_vec3_t& x = mesh.nodes[i];
    const double p[3] = {x.x, x.y, x.z};
    for(int r = 0; r < 3; r++) {
      u(3 * i + r) = A[r][0] * p[0] + A[r][1] * p[1] + A[r][2] * p[2];
    }
  }
  const double E = steel.youngs_modulus, nu = steel.poisson_ratio;
  const double lambda = E * nu / ((1 + nu) * (1 - 2 * nu));
  const double mu = E / (2 * (1 + nu));
  double trace = 0.0, eps2 = 0.0;
  for(int i = 0; i < 3; i++) {
    trace += A[i][i];
    for(int j = 0; j < 3; j++) {
      const double e = 0.5 * (A[i][j] + A[j][i]);
      eps2 += e * e;
    }
  }
  const double energy = 0.5 * 1e-3 * (lambda * trace * trace + 2 * mu * eps2);
  const Eigen::VectorXd f = a.stiffness * u;
  EXPECT_NEAR(0.5 * u.dot(f), energy, 1e-10 * energy);
  /* nodes inside the cube carry no force */
  double inner = 0.0, any = 0.0;
  for(Eigen::Index i = 0; i < n; i++) {
    const ksl_vec3_t& x = mesh.nodes[i];
    const double force = f.segment<3>(3 * i).norm();
    any = std::fmax(any, force);
    const double margin = 1e-9;
    if(x.x > margin && x.x < 0.1 - margin && x.y > margin &&
       x.y < 0.1 - margin && x.z > margin && x.z < 0.1 - margin) {
      inner = std::fmax(inner, force);
    }
  }
  EXPECT_LT(inner, 1e-9 * any);

  /* the stress of the uniform strain at every node, inside and on the
  boundary: sigma = lambda tr(e) I + 2 mu e, and of a rigid rotation none
  (the second shape) */
  std::vector<ksl_screw_t> shapes(2 * static_cast<size_t>(n));
  std::vector<int> nodes(static_cast<size_t>(n));
  for(Eigen::Index i = 0; i < n; i++) {
    nodes[size_t(i)] = static_cast<int>(i);
    const ksl_vec3_t& x = mesh.nodes[i];
    shapes[2 * size_t(i)].lin =
      ksl_vec3(u(3 * i), u(3 * i + 1), u(3 * i + 2));
    shapes[2 * size_t(i) + 1].lin = ksl_vec3(-1e-3 * x.y, 1e-3 * x.x, 0.0);
  }
  const std::vector<double> stresses =
    shift::core::fea::NodalStresses(mesh, steel, nodes, shapes, 2);
  ASSERT_EQ(stresses.size(), 12 * size_t(n));
  double expected[6];
  for(int r = 0; r < 3; r++) {
    expected[r] = lambda * trace + 2.0 * mu * A[r][r];
  }
  expected[3] = mu * (A[0][1] + A[1][0]);
  expected[4] = mu * (A[1][2] + A[2][1]);
  expected[5] = mu * (A[2][0] + A[0][2]);
  const double size = std::fabs(expected[0]);
  for(size_t i = 0; i < size_t(n); i++) {
    for(int c = 0; c < 6; c++) {
      EXPECT_NEAR(stresses[12 * i + size_t(c)], expected[c], 1e-6 * size);
      EXPECT_NEAR(stresses[12 * i + 6 + size_t(c)], 0.0, 1e-6 * size);
    }
  }
}

/* a cylinder 1 inch in diameter: the mesh with its boundary's mid-side
nodes on the exact surfaces has the solid's volume much more closely than
its straight-sided tetrahedra, and no element is turned inside out */
TEST(FeaElement, CurvedBoundaryHasTheSolidsVolume) {
  if(!shift::core::cad::StepSupported()) {
    GTEST_SKIP() << "shift was built without STEP support";
  }
  shift::core::cad::Model model = shift::core::cad::ReadStep(
    std::string(SHIFT_EXAMPLES_DIR) + "/cad/cylinder_inch.step");
  shift::core::cad::Repair(model);
  const double exact = 1.030e-4; /* from test_cad_triangulate.cc */
  const double h = 0.012;
  const shift::core::fea::TetMesh mesh =
    shift::core::fea::MeshSolid(model, 0, h);
  const shift::core::fea::Assembled a =
    shift::core::fea::Assemble(mesh, Steel());
  const double straight = mesh.Volume();
  EXPECT_LT(std::fabs(a.volume / exact - 1.0),
            0.25 * std::fabs(straight / exact - 1.0));
  for(size_t e = 0; e < mesh.tets.size(); e++) {
    EXPECT_GT(shift::core::fea::Tet10JacobianRatio(mesh, e), 0.2);
  }
}

#endif
