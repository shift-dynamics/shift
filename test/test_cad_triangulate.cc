/* cad::Triangulate: closed triangulations of the solids of the STEP parts in
examples/cad. Each surface is closed (every segment used once in each
direction, so neighbouring faces share their points along the edges), it
encloses the solid's volume to within the sag of its triangles over its
area, its points
lie on their faces' surfaces, and no triangle is longer than the element
size. */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/cad/brep.h"
#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/cad/triangulate.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_FEA
#include "shift/core/fea/tet_mesh.h"
#endif

class CadTriangulate : public ::testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }

  static shift::core::cad::Model Read(const std::string& name) {
    shift::core::cad::Model m = shift::core::cad::ReadStep(
      std::string(SHIFT_EXAMPLES_DIR) + "/cad/" + name);
    shift::core::cad::Repair(m);
    return m;
  }

  /* the exact volume of a solid body, by the divergence theorem over its
  faces' trims (as in test_cad_repair.cc) */
  static double ExactVolume(const shift::core::cad::Model& m, int body) {
    static const double x[5] = {-0.906179845938664, -0.538469310105683, 0.0,
                                0.538469310105683, 0.906179845938664};
    static const double w[5] = {0.236926885056189, 0.478628670499366,
                                0.568888888888889, 0.478628670499366,
                                0.236926885056189};
    double volume = 0.0;
    for(int shell : m.bodies[body].shells) {
      for(int fi : m.shells[shell].faces) {
        const shift::core::cad::Face& f = m.faces[fi];
        const shift::core::cad::Surface& s = m.surfaces[f.surface];
        auto integrand = [&](double u, double v) {
          shift::core::cad::Vec3 du, dv;
          shift::core::cad::Vec3 p =
            shift::core::cad::SurfacePoint(m, s, u, v, &du, &dv);
          return p[0] * (du[1] * dv[2] - du[2] * dv[1]) +
                 p[1] * (du[2] * dv[0] - du[0] * dv[2]) +
                 p[2] * (du[0] * dv[1] - du[1] * dv[0]);
        };
        auto along_u = [&](double u, double v) {
          double sum = 0.0;
          for(int piece = 0; piece < 4; ++piece) {
            const double a = u * piece / 4, b = u * (piece + 1) / 4;
            for(int k = 0; k < 5; ++k) {
              sum += w[k] * 0.5 * (b - a) *
                     integrand(0.5 * (a + b) + 0.5 * (b - a) * x[k], v);
            }
          }
          return sum;
        };
        double sum = 0.0;
        for(const shift::core::cad::TrimLoop& loop : f.trims) {
          std::vector<shift::core::cad::Vec2> poly =
            shift::core::cad::TrimPolygon(m, loop, 1024);
          for(size_t i = 0; i < poly.size(); ++i) {
            const shift::core::cad::Vec2& a = poly[i];
            const shift::core::cad::Vec2& b = poly[(i + 1) % poly.size()];
            for(int k = 0; k < 5; ++k) {
              const double t = 0.5 + 0.5 * x[k];
              sum += 0.5 * w[k] * (b[1] - a[1]) *
                     along_u(a[0] + t * (b[0] - a[0]),
                             a[1] + t * (b[1] - a[1]));
            }
          }
        }
        volume += (f.same_sense ? sum : -sum) / 3.0;
      }
    }
    return volume;
  }

  struct Check {
    bool closed = true;
    double volume = 0.0;
    double area = 0.0;
    double longest = 0.0;
    double farthest = 0.0; //!< of a point from its face's surface
    double smallest_angle = 180.0;
  };

  static Check Inspect(const shift::core::cad::Model& m,
                       const shift::core::cad::Triangulation& t) {
    Check c;
    std::map<std::pair<int, int>, int> directed;
    for(size_t k = 0; k < t.triangles.size(); k++) {
      const std::array<int, 3>& tri = t.triangles[k];
      const shift::core::cad::Vec3 p[3] = {t.points[tri[0]], t.points[tri[1]],
                                           t.points[tri[2]]};
      for(int i = 0; i < 3; i++) {
        directed[{tri[i], tri[(i + 1) % 3]}]++;
        c.longest = std::max(
          c.longest, shift::core::cad::Distance(p[i], p[(i + 1) % 3]));
        const shift::core::cad::Vec3 q = shift::core::cad::SurfacePoint(
          m, m.surfaces[m.faces[t.face[k]].surface], t.uv[k][i][0],
          t.uv[k][i][1]);
        c.farthest = std::max(c.farthest, shift::core::cad::Distance(q, p[i]));
        double e1[3], e2[3];
        for(int d = 0; d < 3; d++) {
          e1[d] = p[(i + 1) % 3][d] - p[i][d];
          e2[d] = p[(i + 2) % 3][d] - p[i][d];
        }
        const double n1 = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] +
                                    e1[2] * e1[2]);
        const double n2 = std::sqrt(e2[0] * e2[0] + e2[1] * e2[1] +
                                    e2[2] * e2[2]);
        const double cosine =
          (e1[0] * e2[0] + e1[1] * e2[1] + e1[2] * e2[2]) / (n1 * n2);
        c.smallest_angle = std::min(
          c.smallest_angle,
          std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / M_PI);
      }
      const shift::core::cad::Vec3& a = p[0];
      const shift::core::cad::Vec3& b = p[1];
      const shift::core::cad::Vec3& d = p[2];
      const double n[3] = {
        (b[1] - a[1]) * (d[2] - a[2]) - (b[2] - a[2]) * (d[1] - a[1]),
        (b[2] - a[2]) * (d[0] - a[0]) - (b[0] - a[0]) * (d[2] - a[2]),
        (b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0])};
      c.area += 0.5 * std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      c.volume += (a[0] * (b[1] * d[2] - b[2] * d[1]) +
                   a[1] * (b[2] * d[0] - b[0] * d[2]) +
                   a[2] * (b[0] * d[1] - b[1] * d[0])) /
                  6.0;
    }
    for(const auto& [edge, count] : directed) {
      auto other = directed.find({edge.second, edge.first});
      if(count != 1 || other == directed.end() || other->second != 1) {
        c.closed = false;
      }
    }
    return c;
  }
};

/* the solids of parts with planes, holes, cylinders, a sphere (poles), a
cone (apex), a spherical bowl (seams), an extruded involute (a cusp), a
NURBS cam and an assembly of five bodies: closed, the solid's volume to
within the sag, every point on its face's surface, no triangle much longer
than the element size, and every segment between two faces on an edge of
the B-rep */
TEST_F(CadTriangulate, SolidsAreClosed) {
  for(const std::string name :
      {"box_mm.step", "plate_hole.step", "cylinder_inch.step", "ball.step",
       "cone_apex.step", "bowl.step", "gear_tooth.step", "cam_nurbs.step",
       "carrier_roller.step"}) {
    SCOPED_TRACE(name);
    const shift::core::cad::Model m = Read(name);
    for(size_t b = 0; b < m.bodies.size(); b++) {
      ASSERT_TRUE(m.bodies[b].solid);
      const double exact = ExactVolume(m, static_cast<int>(b));
      shift::core::cad::TriangulateOptions options;
      options.element_size = 0.2 * std::cbrt(std::fabs(exact));
      const shift::core::cad::Triangulation t =
        shift::core::cad::Triangulate(m, static_cast<int>(b), options);
      const Check c = Inspect(m, t);
      EXPECT_TRUE(c.closed);
      /* within the sag (element_size / 20) over the area */
      EXPECT_NEAR(c.volume, std::fabs(exact),
                  c.area * options.element_size / 20.0);
      EXPECT_LT(c.longest, 1.4 * options.element_size);
      EXPECT_LT(c.farthest, 1e-5);
      EXPECT_GT(c.smallest_angle, 0.1);
      /* a segment between triangles of two faces lies on an edge */
      std::map<std::pair<int, int>, int> face_of;
      for(size_t k = 0; k < t.triangles.size(); k++) {
        for(int i = 0; i < 3; i++) {
          face_of[{t.triangles[k][i], t.triangles[k][(i + 1) % 3]}] =
            t.face[k];
        }
      }
      int missing = 0;
      for(const auto& [segment, face] : face_of) {
        auto other = face_of.find({segment.second, segment.first});
        if(other != face_of.end() && other->second != face &&
           !t.edge_of.count(std::minmax(segment.first, segment.second))) {
          missing++;
        }
      }
      EXPECT_EQ(missing, 0);
    }
  }
}

/* a shell whose faces meet along edges they do not share (the bent ring is
one face bounded by four edges, each used once) is refused, by name */
TEST_F(CadTriangulate, RefusesAnOpenShell) {
  for(const std::string name : {"ring_sector.step", "bolt.step"}) {
    SCOPED_TRACE(name);
    const shift::core::cad::Model m = Read(name);
    shift::core::cad::TriangulateOptions options;
    options.element_size = 0.005;
    try {
      shift::core::cad::Triangulate(m, 0, options);
      ADD_FAILURE() << "an open shell was triangulated";
    } catch(const shift::core::Exception& e) {
      EXPECT_NE(std::string(e.what()).find("is not closed"),
                std::string::npos)
        << e.what();
    }
  }
}

#if SHIFT_HAVE_FEA
/* Netgen fills the triangulated plate with a hole: the tetrahedra fill the
volume the surface encloses (MeshVolume checks it) */
TEST_F(CadTriangulate, NetgenFillsATriangulatedPart) {
  const shift::core::cad::Model m = Read("plate_hole.step");
  const double exact = ExactVolume(m, 0);
  shift::core::cad::TriangulateOptions options;
  options.element_size = 0.25 * std::cbrt(exact);
  const shift::core::cad::Triangulation t =
    shift::core::cad::Triangulate(m, 0, options);
  shift::core::fea::ClosedSurface s;
  for(const shift::core::cad::Vec3& p : t.points) {
    s.points.push_back({p[0], p[1], p[2]});
  }
  s.triangles = t.triangles;
  s.face = t.face;
  const shift::core::fea::TetMesh mesh =
    shift::core::fea::MeshVolume(s, options.element_size);
  EXPECT_GT(mesh.tets.size(), t.points.size());
  EXPECT_NEAR(mesh.Volume() / exact, 1.0, 1e-2);
}
#endif
