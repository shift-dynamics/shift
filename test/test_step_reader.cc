/* The STEP reader (cad::ReadStep) on the hand-written files of
examples/cad: units, geometry, topology, pcurves, colors and assemblies */

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/cad/brep.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/exception.h"

using shift::core::cad::Coedge;
using shift::core::cad::Curve2;
using shift::core::cad::Curve3;
using shift::core::cad::Model;
using shift::core::cad::NurbsSurface;
using shift::core::cad::Surface;
using shift::core::cad::Vec3;

class StepReader : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }

  static Model Read(const std::string& name) {
    return shift::core::cad::ReadStep(std::string(SHIFT_EXAMPLES_DIR) +
                                      "/cad/" + name);
  }

  template <typename T>
  static int IndexOf(const std::vector<T>& items, int step_id) {
    for(size_t i = 0; i < items.size(); ++i) {
      if(items[i].step_id == step_id) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  static int StartOf(const Model& m, const Coedge& c) {
    const auto& e = m.edges[c.edge];
    return c.orientation ? e.start : e.end;
  }

  static int EndOf(const Model& m, const Coedge& c) {
    const auto& e = m.edges[c.edge];
    return c.orientation ? e.end : e.start;
  }

  /* every loop is a closed chain: each coedge ends where the next starts */
  static void ExpectConnectedLoops(const Model& m) {
    for(const auto& f : m.faces) {
      for(const auto& l : f.loops) {
        const size_t n = l.coedges.size();
        for(size_t i = 0; i < n; ++i) {
          EXPECT_EQ(EndOf(m, l.coedges[i]), StartOf(m, l.coedges[(i + 1) % n]))
            << "face #" << f.step_id << ", coedge " << i;
        }
      }
    }
  }

  /* in a closed shell, every edge is used twice, in opposite directions, and
   * vertices - edges + faces = 2 (a shell without handles) */
  static void ExpectClosedShell(const Model& m, int shell) {
    std::map<int, std::vector<bool>> uses;
    std::map<int, int> vertices;
    const auto& s = m.shells[shell];
    for(int f : s.faces) {
      for(const auto& l : m.faces[f].loops) {
        for(const auto& c : l.coedges) {
          uses[c.edge].push_back(c.orientation);
          vertices[m.edges[c.edge].start]++;
          vertices[m.edges[c.edge].end]++;
        }
      }
    }
    for(const auto& [edge, o] : uses) {
      ASSERT_EQ(o.size(), 2u) << "edge #" << m.edges[edge].step_id;
      EXPECT_NE(o[0], o[1]) << "edge #" << m.edges[edge].step_id;
    }
    EXPECT_EQ(static_cast<int>(vertices.size()) -
                static_cast<int>(uses.size()) +
                static_cast<int>(s.faces.size()),
              2);
  }

  static void ExpectNear(const Vec3& a, const Vec3& b, double tolerance) {
    for(int i = 0; i < 3; ++i) {
      EXPECT_NEAR(a[i], b[i], tolerance) << "coordinate " << i;
    }
  }

  /* B-spline basis functions of degree p at t (Cox-de Boor) */
  static std::vector<double> Basis(const std::vector<double>& knots, int p,
                                   int count, double t) {
    std::vector<double> n(knots.size() - 1, 0.0);
    for(size_t i = 0; i + 1 < knots.size(); ++i) {
      bool last = t == knots.back() && knots[i] < knots[i + 1] &&
                  knots[i + 1] == knots.back();
      n[i] = (knots[i] <= t && t < knots[i + 1]) || last ? 1.0 : 0.0;
    }
    for(int k = 1; k <= p; ++k) {
      for(size_t i = 0; i + k + 1 < knots.size(); ++i) {
        double a = 0.0, b = 0.0;
        if(knots[i + k] > knots[i]) {
          a = (t - knots[i]) / (knots[i + k] - knots[i]) * n[i];
        }
        if(knots[i + k + 1] > knots[i + 1]) {
          b = (knots[i + k + 1] - t) / (knots[i + k + 1] - knots[i + 1]) *
              n[i + 1];
        }
        n[i] = a + b;
      }
    }
    n.resize(count);
    return n;
  }

  static Vec3 Evaluate(const NurbsSurface& s, double u, double v) {
    auto nu = Basis(s.u_knots, s.u_degree, s.u_count, u);
    auto nv = Basis(s.v_knots, s.v_degree, s.v_count, v);
    Vec3 p{0.0, 0.0, 0.0};
    double w = 0.0;
    for(int i = 0; i < s.u_count; ++i) {
      for(int j = 0; j < s.v_count; ++j) {
        const int k = i * s.v_count + j;
        const double b =
          nu[i] * nv[j] * (s.rational() ? s.weights[k] : 1.0);
        for(int d = 0; d < 3; ++d) {
          p[d] += b * s.points[k][d];
        }
        w += b;
      }
    }
    for(int d = 0; d < 3; ++d) {
      p[d] /= w;
    }
    return p;
  }
};

TEST_F(StepReader, BoxInMillimeters) {
  Model m = Read("box_mm.step");
  EXPECT_TRUE(m.warnings.empty()) << m.warnings.front();
  ASSERT_EQ(m.roots.size(), 1u);
  EXPECT_EQ(m.shapes[m.roots[0]].name, "box");
  ASSERT_EQ(m.bodies.size(), 1u);
  EXPECT_TRUE(m.bodies[0].solid);
  EXPECT_EQ(m.bodies[0].name, "box");
  ASSERT_EQ(m.shells.size(), 1u);
  EXPECT_TRUE(m.shells[0].closed);
  EXPECT_EQ(m.faces.size(), 6u);
  EXPECT_EQ(m.edges.size(), 12u);
  EXPECT_EQ(m.vertices.size(), 8u);
  EXPECT_EQ(m.surfaces.size(), 6u);
  for(const auto& s : m.surfaces) {
    EXPECT_EQ(s.type, Surface::Type::kPlane);
  }

  // millimeters to meters
  int v = IndexOf(m.vertices, 117);
  ASSERT_GE(v, 0);
  ExpectNear(m.vertices[v].point, {0.01, 0.02, 0.03}, 1e-15);
  int e = IndexOf(m.edges, 121);
  ASSERT_GE(e, 0);
  const Curve3& line = m.curves[m.edges[e].curve];
  EXPECT_EQ(line.type, Curve3::Type::kLine);
  ExpectNear(line.direction, {0.01, 0.0, 0.0}, 1e-15);

  // the plane of the left face: its z axis along -x, its x axis along y
  int left = IndexOf(m.faces, 44);
  ASSERT_GE(left, 0);
  const auto& plane = m.surfaces[m.faces[left].surface].position;
  ExpectNear(plane.z, {-1.0, 0.0, 0.0}, 1e-15);
  ExpectNear(plane.x, {0.0, 1.0, 0.0}, 1e-15);
  ExpectNear(plane.y, {0.0, 0.0, -1.0}, 1e-15);

  // colors: the solid red, the top face green, the others not set
  EXPECT_TRUE(m.bodies[0].color.set);
  EXPECT_EQ(m.bodies[0].color.r, 1.0f);
  EXPECT_EQ(m.bodies[0].color.g, 0.0f);
  for(const auto& f : m.faces) {
    if(f.step_id == 41) {
      EXPECT_TRUE(f.color.set);
      EXPECT_EQ(f.color.g, 1.0f);
      EXPECT_EQ(f.color.r, 0.0f);
    } else {
      EXPECT_FALSE(f.color.set) << "face #" << f.step_id;
    }
  }

  ExpectConnectedLoops(m);
  ExpectClosedShell(m, 0);
  for(const auto& f : m.faces) {
    for(const auto& l : f.loops) {
      EXPECT_TRUE(l.outer);
      for(const auto& c : l.coedges) {
        EXPECT_EQ(c.pcurve, -1);
      }
    }
  }
}

TEST_F(StepReader, CylinderInInchesAndDegrees) {
  Model m = Read("cylinder_inch.step");
  EXPECT_TRUE(m.warnings.empty()) << m.warnings.front();
  ASSERT_EQ(m.bodies.size(), 1u);
  EXPECT_EQ(m.faces.size(), 3u);
  EXPECT_EQ(m.edges.size(), 3u);
  EXPECT_EQ(m.vertices.size(), 2u);
  const double inch = 0.0254;

  int side = IndexOf(m.faces, 50);
  ASSERT_GE(side, 0);
  const Surface& cylinder = m.surfaces[m.faces[side].surface];
  EXPECT_EQ(cylinder.type, Surface::Type::kCylinder);
  EXPECT_NEAR(cylinder.radius, inch, 1e-15);
  int v = IndexOf(m.vertices, 73);
  ASSERT_GE(v, 0);
  ExpectNear(m.vertices[v].point, {inch, 0.0, 2.0 * inch}, 1e-15);

  // the bottom circle: in meters, its pcurve on the cylinder in radians,
  // sharing the circle's parameter (radians, where the file has degrees for
  // both)
  int bottom = IndexOf(m.edges, 100);
  ASSERT_GE(bottom, 0);
  const Curve3& circle = m.curves[m.edges[bottom].curve];
  EXPECT_EQ(circle.type, Curve3::Type::kConic);
  ExpectNear(circle.a, {inch, 0.0, 0.0}, 1e-15);
  ExpectNear(circle.b, {0.0, inch, 0.0}, 1e-15);
  ASSERT_EQ(m.edges[bottom].pcurves.size(), 1u);
  const Curve2& on_cylinder = m.edges[bottom].pcurves[0].curve;
  EXPECT_EQ(on_cylinder.type, Curve2::Type::kLine);
  EXPECT_NEAR(on_cylinder.direction[0], 1.0, 1e-15);
  EXPECT_NEAR(on_cylinder.direction[1], 0.0, 1e-15);

  // the top circle: pcurves on the cylinder (v = 2 in) and on the top plane
  int top = IndexOf(m.edges, 102);
  ASSERT_GE(top, 0);
  ASSERT_EQ(m.edges[top].pcurves.size(), 2u);
  EXPECT_NEAR(m.edges[top].pcurves[0].curve.origin[1], 2.0 * inch, 1e-15);
  const Curve2& on_plane = m.edges[top].pcurves[1].curve;
  EXPECT_EQ(on_plane.type, Curve2::Type::kConic);
  EXPECT_NEAR(on_plane.a[0], inch, 1e-15);
  EXPECT_NEAR(on_plane.b[1], inch, 1e-15);

  // the seam: the coedge along it on its pcurve at u = 360 degrees (2 pi),
  // the one against it at u = 0
  int seam = IndexOf(m.edges, 106);
  ASSERT_GE(seam, 0);
  int along = 0, against = 0;
  for(const auto& c : m.faces[side].loops[0].coedges) {
    ASSERT_GE(c.pcurve, 0);
    if(c.edge != seam) {
      continue;
    }
    const double u = m.edges[seam].pcurves[c.pcurve].curve.origin[0];
    if(c.orientation) {
      EXPECT_NEAR(u, 2.0 * M_PI, 1e-12);
      ++along;
    } else {
      EXPECT_NEAR(u, 0.0, 1e-15);
      ++against;
    }
  }
  EXPECT_EQ(along, 1);
  EXPECT_EQ(against, 1);

  // the bottom face's coedge has no pcurve in the file
  int bottom_face = IndexOf(m.faces, 51);
  ASSERT_GE(bottom_face, 0);
  EXPECT_EQ(m.faces[bottom_face].loops[0].coedges[0].pcurve, -1);
  int top_face = IndexOf(m.faces, 52);
  ASSERT_GE(top_face, 0);
  EXPECT_EQ(m.faces[top_face].loops[0].coedges[0].pcurve, 1);

  ExpectConnectedLoops(m);
  ExpectClosedShell(m, 0);
}

TEST_F(StepReader, RationalBSplinePatch) {
  Model m = Read("rational_patch.step");
  EXPECT_TRUE(m.warnings.empty()) << m.warnings.front();
  ASSERT_EQ(m.bodies.size(), 1u);
  EXPECT_FALSE(m.bodies[0].solid);
  ASSERT_EQ(m.shells.size(), 1u);
  EXPECT_FALSE(m.shells[0].closed);
  ASSERT_EQ(m.faces.size(), 1u);
  EXPECT_EQ(m.edges.size(), 4u);

  const Surface& s = m.surfaces[m.faces[0].surface];
  ASSERT_EQ(s.type, Surface::Type::kNurbs);
  const NurbsSurface& n = s.nurbs;
  EXPECT_EQ(n.u_degree, 2);
  EXPECT_EQ(n.v_degree, 1);
  EXPECT_EQ(n.u_count, 3);
  EXPECT_EQ(n.v_count, 2);
  EXPECT_EQ(n.u_knots, (std::vector<double>{0, 0, 0, 1, 1, 1}));
  EXPECT_EQ(n.v_knots, (std::vector<double>{0, 0, 1, 1}));
  ASSERT_TRUE(n.rational());
  EXPECT_NEAR(n.weights[1 * 2 + 0], std::sqrt(0.5), 1e-15);
  ExpectNear(n.points[1 * 2 + 1], {1.0, 1.0, 1.0}, 1e-15);
  // the surface is a quarter of the cylinder of radius 1
  for(double u : {0.0, 0.25, 0.5, 0.8, 1.0}) {
    Vec3 p = Evaluate(n, u, 0.5);
    EXPECT_NEAR(std::hypot(p[0], p[1]), 1.0, 1e-14) << "u " << u;
    EXPECT_NEAR(p[2], 0.5, 1e-14) << "u " << u;
  }

  // edges: rational arcs and lines, all with pcurves; B-C's is a B-spline
  int arcs = 0;
  for(const auto& e : m.edges) {
    ASSERT_GE(e.curve, 0);
    const Curve3& c = m.curves[e.curve];
    if(c.type == Curve3::Type::kNurbs) {
      EXPECT_TRUE(c.nurbs.rational());
      EXPECT_EQ(c.nurbs.degree, 2);
      ++arcs;
    }
    ASSERT_EQ(e.pcurves.size(), 1u);
  }
  EXPECT_EQ(arcs, 2);
  int bc = IndexOf(m.edges, 111);
  ASSERT_GE(bc, 0);
  const Curve2& p = m.edges[bc].pcurves[0].curve;
  EXPECT_EQ(p.type, Curve2::Type::kNurbs);
  EXPECT_EQ(p.nurbs.degree, 1);
  EXPECT_EQ(p.nurbs.points.size(), 2u);
  for(const auto& c : m.faces[0].loops[0].coedges) {
    EXPECT_EQ(c.pcurve, 0);
  }
  ExpectConnectedLoops(m);
}

TEST_F(StepReader, AssemblyPlacements) {
  Model m = Read("assembly_mm.step");
  EXPECT_TRUE(m.warnings.empty()) << m.warnings.front();
  ASSERT_EQ(m.roots.size(), 1u);
  EXPECT_EQ(m.shapes[m.roots[0]].name, "assembly");
  ASSERT_EQ(m.bodies.size(), 1u); // one solid, used three times

  auto placed = m.Flatten();
  ASSERT_EQ(placed.size(), 3u);
  // the cube's corner at (10, 0, 0) mm, in the assembly
  std::vector<Vec3> corners;
  for(const auto& p : placed) {
    EXPECT_EQ(p.path, "assembly/cube");
    corners.push_back(p.transform.Apply({0.01, 0.0, 0.0}));
  }
  std::sort(corners.begin(), corners.end());
  ExpectNear(corners[0], {0.0, 0.06, 0.0}, 1e-15);   // turned, at (0, 50, 0)
  ExpectNear(corners[1], {0.01, 0.0, 0.2}, 1e-15);   // mapped, at (0, 0, 200)
  ExpectNear(corners[2], {0.11, 0.0, 0.0}, 1e-15);   // at (100, 0, 0)
  ExpectClosedShell(m, 0);
}

TEST_F(StepReader, MissingFile) {
  EXPECT_THROW(Read("no_such_file.step"), shift::core::Exception);
}
