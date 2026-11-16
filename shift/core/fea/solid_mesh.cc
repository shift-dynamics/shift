#include "shift/core/fea/solid_mesh.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/triangulate.h"

/* the Jacobian determinant of a 10-node tetrahedron at (xi, eta, zeta) */
static double Tet10Determinant(const shift::core::fea::TetMesh& mesh,
                               const std::array<int, 10>& t, double xi,
                               double eta, double zeta) {
  const double L[4] = {1.0 - xi - eta - zeta, xi, eta, zeta};
  static const double kDL[4][3] = {
    {-1.0, -1.0, -1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  static const int kEdges[6][2] = {{0, 1}, {1, 2}, {2, 0},
                                   {0, 3}, {1, 3}, {2, 3}};
  double dn[10][3];
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
  double J[3][3] = {};
  for(int n = 0; n < 10; n++) {
    const ksl_vec3_t& p = mesh.nodes[t[n]];
    const double x[3] = {p.x, p.y, p.z};
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++) {
        J[r][c] += x[r] * dn[n][c];
      }
    }
  }
  return J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) -
         J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
         J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
}

double shift::core::fea::Tet10JacobianRatio(const TetMesh& mesh,
                                            size_t element) {
  const std::array<int, 10>& t = mesh.tets[element];
  ksl_vec3_t a, b, c, axb;
  ksl_subtract_vv(&mesh.nodes[t[1]], &mesh.nodes[t[0]], &a);
  ksl_subtract_vv(&mesh.nodes[t[2]], &mesh.nodes[t[0]], &b);
  ksl_subtract_vv(&mesh.nodes[t[3]], &mesh.nodes[t[0]], &c);
  ksl_cross_vv(&a, &b, &axb);
  const double straight = ksl_dot_vv(&axb, &c);
  const double p = 0.5854101966249685, q = 0.1381966011250105;
  const double points[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
                               {q, q, q}, {p, q, q}, {q, p, q}, {q, q, p}};
  double smallest = 1e300;
  for(const auto& x : points) {
    smallest = std::min(smallest, Tet10Determinant(mesh, t, x[0], x[1], x[2]));
  }
  return smallest / straight;
}

shift::core::fea::TetMesh shift::core::fea::MeshSolid(
  const cad::Model& model, int body, double element_size,
  double minimum_element_size) {
  cad::TriangulateOptions options;
  options.element_size = element_size;
  options.minimum_element_size = minimum_element_size;
  const cad::Triangulation triangulation =
    cad::Triangulate(model, body, options);
  ClosedSurface surface;
  surface.points.reserve(triangulation.points.size());
  for(const cad::Vec3& p : triangulation.points) {
    surface.points.push_back({p[0], p[1], p[2]});
  }
  surface.triangles = triangulation.triangles;
  surface.face = triangulation.face;
  TetMesh mesh = MeshVolume(surface, element_size, minimum_element_size);

  /* the boundary's mid-side nodes onto the exact geometry */
  std::vector<char> moved(mesh.nodes.size(), 0);
  std::vector<ksl_vec3_t> straight(mesh.nodes);
  for(size_t k = 0; k < mesh.boundary.size(); k++) {
    const std::array<int, 6>& tri = mesh.boundary[k];
    const cad::Face& face = model.faces[triangulation.face[k]];
    const cad::Surface& s = model.surfaces[face.surface];
    for(int e = 0; e < 3; e++) {
      const int i = tri[e], j = tri[(e + 1) % 3], mid = tri[3 + e];
      if(moved[mid]) {
        continue;
      }
      const ksl_vec3_t& m = mesh.nodes[mid];
      const cad::Vec3 p{m.x, m.y, m.z};
      cad::Vec3 exact = p;
      bool found = false;
      auto edge = triangulation.edge_of.find(std::minmax(i, j));
      if(edge != triangulation.edge_of.end() &&
         model.edges[edge->second].curve >= 0) {
        const cad::Curve3& curve =
          model.curves[model.edges[edge->second].curve];
        const ksl_vec3_t& pi = mesh.nodes[i];
        double guess = 0.0;
        if(cad::ProjectToCurve(curve, {pi.x, pi.y, pi.z}, guess)) {
          double t = guess;
          if(cad::ProjectToCurve(curve, p, t, &guess)) {
            exact = cad::CurvePoint<3>(curve, t);
            found = true;
          }
        }
      }
      if(!found) {
        const cad::Vec2& a = triangulation.uv[k][e];
        const cad::Vec2& b = triangulation.uv[k][(e + 1) % 3];
        const cad::Vec2 guess{0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])};
        cad::Vec2 uv = guess;
        if(cad::ProjectToSurface(model, s, p, uv, &guess)) {
          exact = cad::SurfacePoint(model, s, uv[0], uv[1]);
          found = true;
        }
      }
      /* a move much longer than the sag is a failed projection */
      if(found && cad::Distance(exact, p) <= 0.25 * element_size) {
        mesh.nodes[mid] = {exact[0], exact[1], exact[2]};
        moved[mid] = 1;
      }
    }
  }

  /* an element turned inside out, or nearly, keeps its straight mid-side
  nodes */
  for(int pass = 0; pass < 4; pass++) {
    bool reverted = false;
    for(size_t e = 0; e < mesh.tets.size(); e++) {
      if(Tet10JacobianRatio(mesh, e) > 0.2) {
        continue;
      }
      for(int a = 4; a < 10; a++) {
        const int node = mesh.tets[e][a];
        if(moved[node]) {
          mesh.nodes[node] = straight[node];
          moved[node] = 0;
          reverted = true;
        }
      }
    }
    if(!reverted) {
      break;
    }
  }
  return mesh;
}
