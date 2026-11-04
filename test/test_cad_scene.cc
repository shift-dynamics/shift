/* CAD geometry packed for the viewer's tessellation shaders (MakeSceneCad):
every face of the hand-written files of examples/cad packed, its trims
tested as the fragment shader tests them (CadInside) against the trim loops
of the repair, seams drawn without edge lines, and patches without
T-junctions, which would leave cracks between them; the grids of the
triangles for ray tracing (cad_triangulate.comp), and the trims of several
geometries merged for the ray tracer */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/cad/brep.h"
#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"
#include "shift/core/cad/step_reader.h"
#include "shift/gui/cad_scene.h"

using shift::core::cad::Model;
using shift::core::cad::PlacedBody;
using shift::core::cad::Vec2;
using shift::gui::CadGpuFace;
using shift::gui::CadGpuPatch;
using shift::gui::CadInside;
using shift::gui::CadTrimTables;
using shift::gui::MergeCadTrims;
using shift::gui::MakeSceneCad;
using shift::gui::SceneCad;

class CadScene : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::cad::StepSupported()) {
      GTEST_SKIP() << "shift was built without STEP support";
    }
  }

  static Model Read(const std::string& name) {
    Model m = shift::core::cad::ReadStep(std::string(SHIFT_EXAMPLES_DIR) +
                                         "/cad/" + name);
    shift::core::cad::Repair(m);
    return m;
  }

  /* the model's faces in the order they are packed */
  static std::vector<int> PackedFaces(const Model& m,
                                      const std::vector<PlacedBody>& bodies) {
    std::vector<int> faces;
    for(const auto& placed : bodies) {
      for(int shell : m.bodies[placed.body].shells) {
        for(int f : m.shells[shell].faces) {
          faces.push_back(f);
        }
      }
    }
    return faces;
  }

  /* winding number of the trim loops of a face about (u, v), and the
   * distance of (u, v) from them */
  static int Winding(const std::vector<std::vector<Vec2>>& loops, double u,
                     double v, double& distance) {
    int winding = 0;
    distance = 1e300;
    for(const auto& loop : loops) {
      for(size_t i = 0; i < loop.size(); ++i) {
        const Vec2& a = loop[i];
        const Vec2& b = loop[(i + 1) % loop.size()];
        if((a[1] <= v) != (b[1] <= v)) {
          const double x = a[0] + (v - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
          if(x > u) {
            winding += b[1] > a[1] ? 1 : -1;
          }
        }
        const double eu = b[0] - a[0], ev = b[1] - a[1];
        const double t = std::clamp(
          ((u - a[0]) * eu + (v - a[1]) * ev) / std::max(eu * eu + ev * ev,
                                                         1e-300),
          0.0, 1.0);
        distance = std::min(distance, std::hypot(u - a[0] - t * eu,
                                                 v - a[1] - t * ev));
      }
    }
    return winding;
  }

  static const std::vector<std::string>& Files() {
    static const std::vector<std::string> files{
      "box_mm.step",     "cylinder_inch.step",  "cone_apex.step",
      "tube_dome.step",  "rational_patch.step", "assembly_mm.step"};
    return files;
  }
};

/* every face packed, with its patches, segments and band lists in range */
TEST_F(CadScene, PacksEveryFace) {
  for(const std::string& file : Files()) {
    SCOPED_TRACE(file);
    const Model m = Read(file);
    const std::vector<PlacedBody> bodies = m.Flatten();
    const SceneCad cad = MakeSceneCad(m, bodies, 1.0);
    EXPECT_EQ(cad.faces_skipped, 0);
    ASSERT_EQ(cad.faces.size(), PackedFaces(m, bodies).size());
    ASSERT_FALSE(cad.patches.empty());
    const int segments = static_cast<int>(cad.segments.size() / 4);
    for(const CadGpuFace& f : cad.faces) {
      ASSERT_GE(f.trim[1], 1);
      for(int band = 0; band < f.trim[1]; ++band) {
        const int first = cad.trim_index[f.trim[0] + 2 * band];
        const int count = cad.trim_index[f.trim[0] + 2 * band + 1];
        ASSERT_LE(size_t(first + count), cad.trim_index.size());
        for(int k = 0; k < count; ++k) {
          int index = cad.trim_index[first + k];
          index = index < 0 ? ~index : index;
          EXPECT_GE(index, 0);
          EXPECT_LT(index, segments);
        }
      }
    }
    for(const CadGpuPatch& p : cad.patches) {
      ASSERT_GE(p.info[0], 0);
      ASSERT_LT(p.info[0], static_cast<int>(cad.faces.size()));
      EXPECT_LT(p.uv[0], p.uv[2]);
      EXPECT_LT(p.uv[1], p.uv[3]);
    }
  }
}

/* the bounds scale with the geometry's scale */
TEST_F(CadScene, ScalesBounds) {
  const Model m = Read("box_mm.step");
  const SceneCad one = MakeSceneCad(m, m.Flatten(), 1.0);
  const SceneCad two = MakeSceneCad(m, m.Flatten(), 2.0);
  EXPECT_FLOAT_EQ(two.scale, 2.f);
  for(int k = 0; k < 3; ++k) {
    EXPECT_NEAR(two.bounds_min[k], 2.f * one.bounds_min[k], 1e-6f);
    EXPECT_NEAR(two.bounds_max[k], 2.f * one.bounds_max[k], 1e-6f);
  }
  // 10 mm x 20 mm x 30 mm
  EXPECT_NEAR(one.bounds_max[0] - one.bounds_min[0], 0.01f, 1e-6f);
  EXPECT_NEAR(one.bounds_max[1] - one.bounds_min[1], 0.02f, 1e-6f);
  EXPECT_NEAR(one.bounds_max[2] - one.bounds_min[2], 0.03f, 1e-6f);
}

/* the inside test of the packed segments agrees with the trim loops of the
repair on a grid over each face, away from the loops (the segments are
within the sampling tolerance of them) */
TEST_F(CadScene, InsideMatchesTrimLoops) {
  for(const std::string& file : Files()) {
    SCOPED_TRACE(file);
    const Model m = Read(file);
    const std::vector<PlacedBody> bodies = m.Flatten();
    const SceneCad cad = MakeSceneCad(m, bodies, 1.0);
    const std::vector<int> faces = PackedFaces(m, bodies);
    for(size_t i = 0; i < faces.size(); ++i) {
      SCOPED_TRACE("face " + std::to_string(i));
      std::vector<std::vector<Vec2>> loops;
      double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
      for(const auto& loop : m.faces[faces[i]].trims) {
        loops.push_back(shift::core::cad::TrimPolygon(m, loop, 256));
        for(const Vec2& p : loops.back()) {
          for(int k = 0; k < 2; ++k) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
          }
        }
      }
      // the loops in the face's box scaled to a unit square, where the
      // margin is measured
      for(auto& loop : loops) {
        for(Vec2& p : loop) {
          for(int k = 0; k < 2; ++k) {
            p[k] = (p[k] - lo[k]) / (hi[k] - lo[k]);
          }
        }
      }
      const double margin = 0.01;
      const int n = 40;
      int tested = 0, inside = 0;
      for(int a = 0; a <= n; ++a) {
        for(int b = 0; b <= n; ++b) {
          // a grid offset from the box's edges, where the loops run
          const double x = (a + 0.37) / (n + 1), y = (b + 0.61) / (n + 1);
          const double u = lo[0] + (hi[0] - lo[0]) * x;
          const double v = lo[1] + (hi[1] - lo[1]) * y;
          double distance;
          const bool expected = Winding(loops, x, y, distance) != 0;
          if(distance < margin) {
            continue;
          }
          ++tested;
          inside += expected;
          EXPECT_EQ(CadInside(cad, static_cast<int>(i), float(u), float(v)),
                    expected)
            << "at (" << u << ", " << v << ")";
        }
      }
      EXPECT_GT(tested, 0);
      EXPECT_GT(inside, 0);
    }
  }
}

/* seams of periodic surfaces are drawn without edge lines (~index in the
band lists); the edges of a box all have them */
TEST_F(CadScene, SeamsWithoutEdgeLines) {
  auto without_lines = [](const SceneCad& cad) {
    int count = 0;
    for(const CadGpuFace& f : cad.faces) {
      for(int band = 0; band < f.trim[1]; ++band) {
        const int first = cad.trim_index[f.trim[0] + 2 * band];
        const int n = cad.trim_index[f.trim[0] + 2 * band + 1];
        for(int k = 0; k < n; ++k) {
          count += cad.trim_index[first + k] < 0;
        }
      }
    }
    return count;
  };
  const Model box = Read("box_mm.step");
  EXPECT_EQ(without_lines(MakeSceneCad(box, box.Flatten(), 1.0)), 0);
  const Model tube = Read("tube_dome.step");
  EXPECT_GT(without_lines(MakeSceneCad(tube, tube.Flatten(), 1.0)), 0);
  const Model cylinder = Read("cylinder_inch.step");
  EXPECT_GT(without_lines(MakeSceneCad(cylinder, cylinder.Flatten(), 1.0)),
            0);
}

/* no corner of a patch lies inside an edge of another patch of its face:
the tessellations of patches sharing an edge then meet exactly, while a
corner in the middle of a neighbor's edge (a T-junction) leaves a crack */
TEST_F(CadScene, PatchesWithoutTJunctions) {
  for(const std::string& file : Files()) {
    SCOPED_TRACE(file);
    const Model m = Read(file);
    const SceneCad cad = MakeSceneCad(m, m.Flatten(), 1.0);
    std::vector<std::vector<const CadGpuPatch*>> by_face(cad.faces.size());
    for(const CadGpuPatch& p : cad.patches) {
      by_face[p.info[0]].push_back(&p);
    }
    for(const auto& patches : by_face) {
      for(const CadGpuPatch* p : patches) {
        const float corners[4][2] = {{p->uv[0], p->uv[1]},
                                     {p->uv[2], p->uv[1]},
                                     {p->uv[0], p->uv[3]},
                                     {p->uv[2], p->uv[3]}};
        for(const CadGpuPatch* q : patches) {
          if(q == p) {
            continue;
          }
          for(const auto& c : corners) {
            // strictly inside an edge of q along u or along v
            const bool on_u_edge = (c[1] == q->uv[1] || c[1] == q->uv[3]) &&
                                   c[0] > q->uv[0] && c[0] < q->uv[2];
            const bool on_v_edge = (c[0] == q->uv[0] || c[0] == q->uv[2]) &&
                                   c[1] > q->uv[1] && c[1] < q->uv[3];
            EXPECT_FALSE(on_u_edge || on_v_edge)
              << "corner (" << c[0] << ", " << c[1] << ")";
          }
        }
      }
    }
  }
}

/* a grid of quads per patch, its triangles counted in first_triangles, and
patches of a face in the same column (u range) or row (v range) with the
same count along it, so that their triangles share their edges' points */
TEST_F(CadScene, RayTracingGrids) {
  for(const std::string& file : Files()) {
    SCOPED_TRACE(file);
    const Model m = Read(file);
    const SceneCad cad = MakeSceneCad(m, m.Flatten(), 1.0);
    const size_t n = cad.patches.size();
    ASSERT_EQ(cad.grids.size(), 2 * n);
    ASSERT_EQ(cad.first_triangles.size(), n + 1);
    EXPECT_EQ(cad.first_triangles[0], 0);
    for(size_t i = 0; i < n; ++i) {
      const int nu = cad.grids[2 * i], nv = cad.grids[2 * i + 1];
      EXPECT_GE(nu, 1);
      EXPECT_GE(nv, 1);
      EXPECT_LE(nu, 32);
      EXPECT_LE(nv, 32);
      EXPECT_EQ(cad.first_triangles[i + 1] - cad.first_triangles[i],
                2 * nu * nv);
      for(size_t j = 0; j < n; ++j) {
        const CadGpuPatch& p = cad.patches[i];
        const CadGpuPatch& q = cad.patches[j];
        if(p.info[0] != q.info[0]) {
          continue;
        }
        if(p.uv[0] == q.uv[0] && p.uv[2] == q.uv[2]) {
          EXPECT_EQ(nu, cad.grids[2 * j]);
        }
        if(p.uv[1] == q.uv[1] && p.uv[3] == q.uv[3]) {
          EXPECT_EQ(nv, cad.grids[2 * j + 1]);
        }
      }
    }
  }
  // the faces of a box are flat: one quad per patch
  const Model box = Read("box_mm.step");
  const SceneCad cad = MakeSceneCad(box, box.Flatten(), 1.0);
  for(int32_t g : cad.grids) {
    EXPECT_EQ(g, 1);
  }
}

/* the quads of the grids are within the ray tracing tolerance (2.5e-4 of
the size of the model, here with a margin of 3 for the estimate of the
grid from a few points) of the surface: the middle of each quad against the
middle of its corners */
TEST_F(CadScene, RayTracingGridsFollowSurfaces) {
  for(const std::string& file : Files()) {
    SCOPED_TRACE(file);
    const Model m = Read(file);
    const std::vector<PlacedBody> bodies = m.Flatten();
    const SceneCad cad = MakeSceneCad(m, bodies, 1.0);
    const std::vector<int> faces = PackedFaces(m, bodies);
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for(const auto& vertex : m.vertices) {
      for(int k = 0; k < 3; ++k) {
        lo[k] = std::min(lo[k], vertex.point[k]);
        hi[k] = std::max(hi[k], vertex.point[k]);
      }
    }
    const double size = std::hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
    double worst = 0.0;
    for(size_t i = 0; i < cad.patches.size(); ++i) {
      const CadGpuPatch& p = cad.patches[i];
      const auto& s = m.surfaces[m.faces[faces[p.info[0]]].surface];
      const int nu = cad.grids[2 * i], nv = cad.grids[2 * i + 1];
      for(int a = 0; a < nu; ++a) {
        for(int b = 0; b < nv; ++b) {
          const double u0 = p.uv[0] + (p.uv[2] - p.uv[0]) * a / nu;
          const double u1 = p.uv[0] + (p.uv[2] - p.uv[0]) * (a + 1) / nu;
          const double v0 = p.uv[1] + (p.uv[3] - p.uv[1]) * b / nv;
          const double v1 = p.uv[1] + (p.uv[3] - p.uv[1]) * (b + 1) / nv;
          double corners[3] = {0.0, 0.0, 0.0};
          for(double u : {u0, u1}) {
            for(double v : {v0, v1}) {
              const auto q = shift::core::cad::SurfacePoint(m, s, u, v);
              for(int k = 0; k < 3; ++k) {
                corners[k] += 0.25 * q[k];
              }
            }
          }
          const auto c = shift::core::cad::SurfacePoint(m, s, 0.5 * (u0 + u1),
                                                        0.5 * (v0 + v1));
          worst = std::max(worst, std::hypot(c[0] - corners[0],
                                             c[1] - corners[1],
                                             c[2] - corners[2]));
        }
      }
    }
    EXPECT_LE(worst, 3.0 * 2.5e-4 * size);
  }
}

/* the trims of several geometries merged for the ray tracer test as those
of each geometry */
TEST_F(CadScene, MergedTrims) {
  std::vector<SceneCad> cads;
  for(const std::string& file : Files()) {
    const Model m = Read(file);
    cads.push_back(MakeSceneCad(m, m.Flatten(), 1.0));
  }
  const CadTrimTables tables = MergeCadTrims(cads);
  ASSERT_EQ(tables.face_base.size(), cads.size());
  size_t faces = 0;
  for(size_t c = 0; c < cads.size(); ++c) {
    EXPECT_EQ(tables.face_base[c], static_cast<int>(faces));
    faces += cads[c].faces.size();
  }
  ASSERT_EQ(tables.faces.size(), faces);
  for(size_t c = 0; c < cads.size(); ++c) {
    SCOPED_TRACE(Files()[c]);
    for(size_t f = 0; f < cads[c].faces.size(); ++f) {
      // over the face's patches, where the fragments are tested
      for(const CadGpuPatch& p : cads[c].patches) {
        if(p.info[0] != static_cast<int>(f)) {
          continue;
        }
        for(int a = 0; a < 3; ++a) {
          for(int b = 0; b < 3; ++b) {
            const float u = p.uv[0] + (p.uv[2] - p.uv[0]) * (a + 0.3f) / 3.f;
            const float v = p.uv[1] + (p.uv[3] - p.uv[1]) * (b + 0.6f) / 3.f;
            EXPECT_EQ(CadInside(tables, tables.face_base[c] + int(f), u, v),
                      CadInside(cads[c], int(f), u, v));
          }
        }
      }
    }
  }
}
