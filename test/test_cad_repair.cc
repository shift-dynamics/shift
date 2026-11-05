/* Evaluation of curves and surfaces (cad/evaluate.h), and the repair of
models read from STEP files (cad::Repair): edge ranges, pcurves checked and
computed, and trim loops in the parameters of surfaces, on the hand-written
files of examples/cad */

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/cad/brep.h"
#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"
#include "shift/core/cad/step_reader.h"

using shift::core::cad::Curve2;
using shift::core::cad::Curve3;
using shift::core::cad::CurvePoint;
using shift::core::cad::Distance;
using shift::core::cad::Face;
using shift::core::cad::Model;
using shift::core::cad::Pcurve;
using shift::core::cad::ProjectToCurve;
using shift::core::cad::ProjectToSurface;
using shift::core::cad::RepairReport;
using shift::core::cad::SignedArea;
using shift::core::cad::Surface;
using shift::core::cad::SurfacePoint;
using shift::core::cad::TrimCurve;
using shift::core::cad::TrimLoop;
using shift::core::cad::TrimPoint;
using shift::core::cad::TrimPolygon;
using shift::core::cad::Vec2;
using shift::core::cad::Vec3;

/* derivatives of a surface against central differences, and projection of
its points back to their parameters */
static void ExpectSurface(const Model& m, const Surface& s,
                          const std::vector<Vec2>& params) {
  const double h = 1e-6;
  for(const Vec2& uv : params) {
    Vec3 du, dv;
    Vec3 p = SurfacePoint(m, s, uv[0], uv[1], &du, &dv);
    Vec3 pu0 = SurfacePoint(m, s, uv[0] - h, uv[1]);
    Vec3 pu1 = SurfacePoint(m, s, uv[0] + h, uv[1]);
    Vec3 pv0 = SurfacePoint(m, s, uv[0], uv[1] - h);
    Vec3 pv1 = SurfacePoint(m, s, uv[0], uv[1] + h);
    for(int k = 0; k < 3; ++k) {
      EXPECT_NEAR(du[k], (pu1[k] - pu0[k]) / (2 * h), 1e-6)
        << "type " << static_cast<int>(s.type) << " du " << k;
      EXPECT_NEAR(dv[k], (pv1[k] - pv0[k]) / (2 * h), 1e-6)
        << "type " << static_cast<int>(s.type) << " dv " << k;
    }
    Vec2 guess{uv[0] + 0.01, uv[1] + 0.01};
    Vec2 q;
    ASSERT_TRUE(ProjectToSurface(m, s, p, q, &guess));
    EXPECT_NEAR(q[0], uv[0], 1e-8) << "type " << static_cast<int>(s.type);
    EXPECT_NEAR(q[1], uv[1], 1e-8) << "type " << static_cast<int>(s.type);
    // a point off the surface along its normal projects to the same place
    Vec3 n{du[1] * dv[2] - du[2] * dv[1], du[2] * dv[0] - du[0] * dv[2],
           du[0] * dv[1] - du[1] * dv[0]};
    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    Vec3 off{p[0] + 0.01 * n[0] / len, p[1] + 0.01 * n[1] / len,
             p[2] + 0.01 * n[2] / len};
    ASSERT_TRUE(ProjectToSurface(m, s, off, q, &guess));
    EXPECT_LT(Distance(SurfacePoint(m, s, q[0], q[1]), p), 1e-8)
      << "type " << static_cast<int>(s.type);
  }
}

TEST(CadEvaluate, SurfaceDerivativesAndProjection) {
  Model m;
  Surface s;
  s.position.origin = {0.1, -0.2, 0.3};
  // a turned frame: x = (0, 1, 0), y = (0, 0, 1), z = (1, 0, 0)
  s.position.x = {0.0, 1.0, 0.0};
  s.position.y = {0.0, 0.0, 1.0};
  s.position.z = {1.0, 0.0, 0.0};
  const std::vector<Vec2> angles{{0.3, 0.2}, {2.0, -0.4}, {4.5, 1.1}};

  s.type = Surface::Type::kPlane;
  ExpectSurface(m, s, {{0.3, -0.7}, {2.0, 1.5}});
  s.type = Surface::Type::kCylinder;
  s.radius = 0.5;
  ExpectSurface(m, s, angles);
  s.type = Surface::Type::kCone;
  s.semi_angle = 0.4;
  ExpectSurface(m, s, angles);
  s.type = Surface::Type::kSphere;
  ExpectSurface(m, s, angles);
  s.type = Surface::Type::kTorus;
  s.minor_radius = 0.2;
  ExpectSurface(m, s, angles);

  // a revolution of a quadratic B-spline profile, and an extrusion of it
  Curve3 profile;
  profile.type = Curve3::Type::kNurbs;
  profile.nurbs.degree = 2;
  profile.nurbs.knots = {0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
  profile.nurbs.points = {{0.4, 0.3, 0.0}, {0.6, 0.0, 0.5},
                          {0.5, 0.0, 0.9}, {0.7, 0.1, 1.2}};
  profile.nurbs.weights = {1.0, 0.8, 1.2, 1.0};
  m.curves.push_back(profile);
  Surface r;
  r.type = Surface::Type::kRevolution;
  r.curve = 0;
  ExpectSurface(m, r, {{0.3, 0.2}, {2.0, 0.6}, {4.5, 0.8}});
  Surface x;
  x.type = Surface::Type::kExtrusion;
  x.curve = 0;
  x.extrusion = {0.1, 0.9, 0.2};
  ExpectSurface(m, x, {{0.2, 0.3}, {0.6, -0.5}, {0.8, 1.0}});

  // a rational B-spline surface: a quarter cylinder
  Surface n;
  n.type = Surface::Type::kNurbs;
  auto& b = n.nurbs;
  b.u_degree = 2;
  b.v_degree = 1;
  b.u_count = 3;
  b.v_count = 2;
  b.u_knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  b.v_knots = {0.0, 0.0, 1.0, 1.0};
  b.points = {{1, 0, 0}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}, {0, 1, 0}, {0, 1, 1}};
  const double w = std::sqrt(0.5);
  b.weights = {1, 1, w, w, 1, 1};
  ExpectSurface(m, n, {{0.2, 0.3}, {0.5, 0.5}, {0.9, 0.7}});
  // every point is 1 from the axis
  for(double u : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    Vec3 p = SurfacePoint(m, n, u, 0.5);
    EXPECT_NEAR(std::hypot(p[0], p[1]), 1.0, 1e-14);
  }
}

TEST(CadEvaluate, CurveDerivativesAndProjection) {
  Curve3 c;
  c.type = Curve3::Type::kConic;
  c.origin = {1.0, 2.0, 3.0};
  c.a = {0.0, 2.0, 0.0}; // an ellipse
  c.b = {0.0, 0.0, 1.0};
  Curve3 n;
  n.type = Curve3::Type::kNurbs;
  n.nurbs.degree = 3;
  n.nurbs.knots = {0, 0, 0, 0, 0.3, 0.7, 1, 1, 1, 1};
  n.nurbs.points = {{0, 0, 0}, {1, 1, 0}, {2, -1, 1}, {3, 0, 0},
                    {4, 1, 2}, {5, 0, 0}};
  n.nurbs.weights = {1, 2, 0.5, 1, 1.5, 1};
  const double h = 1e-6;
  for(const Curve3* curve : {&c, &n}) {
    for(double t : {0.1, 0.35, 0.69, 0.95}) {
      Vec3 d;
      Vec3 p = CurvePoint<3>(*curve, t, &d);
      Vec3 p0 = CurvePoint<3>(*curve, t - h);
      Vec3 p1 = CurvePoint<3>(*curve, t + h);
      for(int k = 0; k < 3; ++k) {
        EXPECT_NEAR(d[k], (p1[k] - p0[k]) / (2 * h), 1e-6);
      }
      double s = 0.0;
      ASSERT_TRUE(ProjectToCurve(*curve, p, s));
      EXPECT_NEAR(s, t, 1e-9);
    }
  }
}

class CadRepair : public testing::Test {
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

  static int FaceOf(const Model& m, int step_id) {
    for(size_t i = 0; i < m.faces.size(); ++i) {
      if(m.faces[i].step_id == step_id) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  /* every trim loop of every face is a closed chain in (u, v), its edge
  pieces lie on their edges' curves, outer loops are counterclockwise and
  holes clockwise */
  static void ExpectValidTrims(const Model& m) {
    for(const Face& f : m.faces) {
      ASSERT_FALSE(f.trims.empty()) << "face #" << f.step_id;
      EXPECT_TRUE(f.trims[0].outer) << "face #" << f.step_id;
      const Surface& s = m.surfaces[f.surface];
      for(const TrimLoop& loop : f.trims) {
        const size_t n = loop.curves.size();
        for(size_t i = 0; i < n; ++i) {
          const TrimCurve& c = loop.curves[i];
          Vec2 end = TrimPoint(m, c, 1.0);
          Vec2 next = TrimPoint(m, loop.curves[(i + 1) % n], 0.0);
          EXPECT_LT(std::hypot(end[0] - next[0], end[1] - next[1]), 1e-9)
            << "face #" << f.step_id << " curve " << i;
          EXPECT_NE(c.kind, TrimCurve::Kind::kGap) << "face #" << f.step_id;
          if(c.kind != TrimCurve::Kind::kEdge) {
            continue;
          }
          const auto& e = m.edges[c.edge];
          const Pcurve& pc = e.pcurves[c.pcurve];
          EXPECT_EQ(pc.surface, f.surface);
          for(double frac : {0.0, 0.3, 0.5, 0.8, 1.0}) {
            const double sp = c.s0 + frac * (c.s1 - c.s0);
            const double g = (sp - pc.range[0]) / (pc.range[1] - pc.range[0]);
            const double t = e.range[0] + g * (e.range[1] - e.range[0]);
            Vec2 uv = TrimPoint(m, c, frac);
            EXPECT_LT(Distance(SurfacePoint(m, s, uv[0], uv[1]),
                               CurvePoint<3>(m.curves[e.curve], t)),
                      1e-6)
              << "face #" << f.step_id << " edge #" << e.step_id;
          }
        }
        const double area = SignedArea(TrimPolygon(m, loop, 256));
        EXPECT_EQ(area > 0.0, loop.outer) << "face #" << f.step_id;
      }
    }
  }

  static double Area(const Model& m, int face) {
    double a = 0.0;
    for(const TrimLoop& loop : m.faces[face].trims) {
      a += SignedArea(TrimPolygon(m, loop, 1024));
    }
    return a;
  }

  /* volume of a solid body by the divergence theorem, (1/3) of the integral
  of p . n over its faces, n outward: a face whose normal points inward
  subtracts its share instead of adding it. Over each face's trims in (u, v)
  by Green's theorem, the integral of F dv around the loops, F the integral
  of p . (dp/du x dp/dv) along u (Gauss-Legendre) */
  static double Volume(const Model& m, int body) {
    static const double x[5] = {-0.906179845938664, -0.538469310105683, 0.0,
                                0.538469310105683, 0.906179845938664};
    static const double w[5] = {0.236926885056189, 0.478628670499366,
                                0.568888888888889, 0.478628670499366,
                                0.236926885056189};
    double volume = 0.0;
    for(int shell : m.bodies[body].shells) {
      for(int fi : m.shells[shell].faces) {
        const Face& f = m.faces[fi];
        const Surface& s = m.surfaces[f.surface];
        auto integrand = [&](double u, double v) {
          Vec3 du, dv;
          Vec3 p = SurfacePoint(m, s, u, v, &du, &dv);
          return p[0] * (du[1] * dv[2] - du[2] * dv[1]) +
                 p[1] * (du[2] * dv[0] - du[0] * dv[2]) +
                 p[2] * (du[0] * dv[1] - du[1] * dv[0]);
        };
        auto along_u = [&](double u, double v) { // from u = 0, 4 pieces
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
        for(const TrimLoop& loop : f.trims) {
          std::vector<Vec2> poly = TrimPolygon(m, loop, 1024);
          for(size_t i = 0; i < poly.size(); ++i) {
            const Vec2& a = poly[i];
            const Vec2& b = poly[(i + 1) % poly.size()];
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

  static int Count(const Face& f, TrimCurve::Kind kind) {
    int n = 0;
    for(const TrimLoop& loop : f.trims) {
      for(const TrimCurve& c : loop.curves) {
        n += c.kind == kind ? 1 : 0;
      }
    }
    return n;
  }
};

TEST_F(CadRepair, BoxPcurvesOnPlanes) {
  Model m = Read("box_mm.step");
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.edges_failed, 0);
  EXPECT_EQ(r.vertex_gaps, 0);
  EXPECT_EQ(r.pcurves_checked, 0);
  EXPECT_EQ(r.pcurves_computed, 24);
  EXPECT_EQ(r.pcurves_exact, 24);
  EXPECT_EQ(r.faces_trimmed, 6);
  EXPECT_LT(r.max_pcurve_deviation, 1e-15);
  ExpectValidTrims(m);
  // on a plane (u, v) are lengths: the trims enclose the faces' areas
  std::vector<double> areas;
  for(size_t i = 0; i < m.faces.size(); ++i) {
    areas.push_back(Area(m, static_cast<int>(i)));
  }
  std::sort(areas.begin(), areas.end());
  const std::vector<double> expected{2e-4, 2e-4, 3e-4, 3e-4, 6e-4, 6e-4};
  for(size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(areas[i], expected[i], 1e-15);
  }
  for(const auto& e : m.edges) {
    EXPECT_TRUE(e.has_range);
    EXPECT_NEAR(std::fabs(e.range[1] - e.range[0]), 1.0, 1e-12)
      << "edge #" << e.step_id; // lines of VECTOR magnitude equal to length
  }
}

TEST_F(CadRepair, CylinderWithSeam) {
  Model m = Read("cylinder_inch.step");
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.edges_failed, 0);
  EXPECT_EQ(r.pcurves_checked, 5);
  EXPECT_EQ(r.pcurves_replaced, 0);
  EXPECT_EQ(r.pcurves_not_same_parameter, 0);
  EXPECT_EQ(r.pcurves_computed, 1); // the bottom circle on the bottom plane
  EXPECT_EQ(r.pcurves_exact, 1);
  EXPECT_EQ(r.seams_reassigned, 0);
  EXPECT_EQ(r.faces_trimmed, 3);
  ExpectValidTrims(m);
  const double radius = 0.0254, height = 0.0508;
  EXPECT_NEAR(Area(m, FaceOf(m, 50)), 2 * M_PI * height, 1e-12);
  EXPECT_NEAR(Area(m, FaceOf(m, 51)), M_PI * radius * radius, 1e-6);
  EXPECT_NEAR(Area(m, FaceOf(m, 52)), M_PI * radius * radius, 1e-6);
  // the circles run once around: their ranges span 2 pi
  for(const auto& e : m.edges) {
    if(e.start == e.end) {
      EXPECT_NEAR(std::fabs(e.range[1] - e.range[0]), 2 * M_PI, 1e-12);
    }
  }
  // the seam runs up at u = 2 pi (its first pcurve) and down at u = 0
  const Face& side = m.faces[FaceOf(m, 50)];
  int seams = 0;
  for(const TrimCurve& c : side.trims[0].curves) {
    if(c.kind == TrimCurve::Kind::kEdge && m.edges[c.edge].step_id == 106) {
      Vec2 a = TrimPoint(m, c, 0.0), b = TrimPoint(m, c, 1.0);
      EXPECT_NEAR(a[0], b[0], 1e-12);
      EXPECT_NEAR(a[0], b[1] > a[1] ? 2 * M_PI : 0.0, 1e-12);
      ++seams;
    }
  }
  EXPECT_EQ(seams, 2);
}

TEST_F(CadRepair, PcurveOffItsEdgeIsReplaced) {
  Model m = Read("cylinder_inch.step");
  // move the bottom circle's pcurve on the cylinder 1 mm up
  for(auto& e : m.edges) {
    if(e.step_id == 100) {
      ASSERT_EQ(e.pcurves.size(), 1u);
      e.pcurves[0].curve.origin[1] += 0.001;
    }
  }
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.pcurves_replaced, 1);
  EXPECT_EQ(r.pcurves_computed, 2);
  EXPECT_LT(r.max_pcurve_deviation, 1e-9);
  ExpectValidTrims(m);
  EXPECT_NEAR(Area(m, FaceOf(m, 50)), 2 * M_PI * 0.0508, 1e-12);
}

TEST_F(CadRepair, RationalPatchKeepsItsPcurves) {
  Model m = Read("rational_patch.step");
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.pcurves_checked, 4);
  EXPECT_EQ(r.pcurves_replaced, 0);
  EXPECT_EQ(r.pcurves_not_same_parameter, 0);
  EXPECT_EQ(r.pcurves_computed, 0);
  EXPECT_LT(r.max_pcurve_deviation, 1e-12);
  ExpectValidTrims(m);
  EXPECT_NEAR(Area(m, 0), 1.0, 1e-12); // the whole domain [0, 1] x [0, 1]
}

TEST_F(CadRepair, PcurveShortOfItsVertexIsReplaced) {
  Model m = Read("rational_patch.step");
  // the pcurve of B-C, a straight B-spline from (1, 0) to (1, 1), made to
  // stop 1% short of C
  for(auto& e : m.edges) {
    if(e.step_id == 111) {
      ASSERT_EQ(e.pcurves.size(), 1u);
      e.pcurves[0].curve.nurbs.points[1][1] = 0.99;
    }
  }
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.pcurves_checked, 4);
  EXPECT_EQ(r.pcurves_replaced, 1);
  EXPECT_EQ(r.gap_segments, 0);
  ExpectValidTrims(m);
  EXPECT_NEAR(Area(m, 0), 1.0, 1e-6);
}

TEST_F(CadRepair, RationalPatchPcurvesComputed) {
  Model m = Read("rational_patch.step");
  for(auto& e : m.edges) {
    e.pcurves.clear();
  }
  for(auto& f : m.faces) {
    for(auto& l : f.loops) {
      for(auto& c : l.coedges) {
        c.pcurve = -1;
      }
    }
  }
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.pcurves_computed, 4);
  EXPECT_EQ(r.pcurves_failed, 0);
  EXPECT_LE(r.max_pcurve_deviation, 1e-6);
  ExpectValidTrims(m);
  EXPECT_NEAR(Area(m, 0), 1.0, 1e-6);
}

TEST_F(CadRepair, ConeClosedAtItsApex) {
  Model m = Read("cone_apex.step");
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.edges_failed, 0);
  EXPECT_EQ(r.pcurves_computed, 2);
  EXPECT_EQ(r.pcurves_exact, 2); // a line in (u, v) on the cone, a circle
  EXPECT_EQ(r.faces_trimmed, 2);
  ExpectValidTrims(m);
  const Face& side = m.faces[FaceOf(m, 50)];
  EXPECT_EQ(Count(side, TrimCurve::Kind::kSeam), 2);
  EXPECT_EQ(Count(side, TrimCurve::Kind::kSingular), 1);
  // (u, v) of the side: a full turn by the 20 mm of the axis
  EXPECT_NEAR(Area(m, FaceOf(m, 50)), 2 * M_PI * 0.02, 1e-12);
  EXPECT_NEAR(Area(m, FaceOf(m, 51)), M_PI * 1e-4, 1e-8); // a 1024-gon
  // the singular segment is at the apex
  const Surface& cone = m.surfaces[side.surface];
  for(const TrimCurve& c : side.trims[0].curves) {
    if(c.kind == TrimCurve::Kind::kSingular) {
      Vec3 p = SurfacePoint(m, cone, c.a[0], c.a[1]);
      EXPECT_LT(Distance(p, {0.0, 0.0, 0.02}), 1e-12);
    }
  }
}

TEST_F(CadRepair, TubeAndDomeWithoutSeams) {
  Model m = Read("tube_dome.step");
  RepairReport r = shift::core::cad::Repair(m);
  EXPECT_EQ(r.pcurves_computed, 3);
  EXPECT_EQ(r.pcurves_exact, 3);
  EXPECT_EQ(r.faces_trimmed, 2);
  ExpectValidTrims(m);
  const Face& tube = m.faces[FaceOf(m, 40)];
  const Face& dome = m.faces[FaceOf(m, 41)];
  ASSERT_EQ(tube.trims.size(), 1u); // the two circles, joined by the cut
  EXPECT_EQ(Count(tube, TrimCurve::Kind::kSeam), 2);
  EXPECT_EQ(Count(dome, TrimCurve::Kind::kSeam), 2);
  EXPECT_EQ(Count(dome, TrimCurve::Kind::kSingular), 1);
  EXPECT_NEAR(Area(m, FaceOf(m, 40)), 2 * M_PI * 0.02, 1e-12);
  EXPECT_NEAR(Area(m, FaceOf(m, 41)), M_PI * M_PI, 1e-12); // up to the pole
}

/* the sprocket and track link written by scripts/make_track_step.py: lines
and circles only, with no pcurves in the file; every face is trimmed with
no gaps */
TEST_F(CadRepair, TrackSprocketAndLink) {
  // the volumes of the solids (m^3), from their profiles: a face facing
  // the wrong way would change them
  // the link plates, less their bores: the bushing's (20 mm) in the inner
  // plates, the pin's (19 mm) in the outer plates
  const double plate = 4.69945e-4;
  const double inner = plate - M_PI * 0.02 * 0.02 * 0.035;
  const double outer = plate - M_PI * 0.019 * 0.019 * 0.035;
  for(const auto& [name, faces, volumes] :
      {std::tuple<std::string, int, std::vector<double>>{"sprocket.step", 88,
                                                         {1.664550e-2}},
       std::tuple<std::string, int, std::vector<double>>{
         "track_link.step", 60,
         {1.77280e-4, 2.94870e-4, inner, inner, outer, outer, 2.664e-3}}}) {
    SCOPED_TRACE(name);
    Model m = Read(name);
    ASSERT_EQ(m.bodies.size(), volumes.size());
    RepairReport r = shift::core::cad::Repair(m);
    EXPECT_EQ(r.faces, faces);
    EXPECT_EQ(r.faces_trimmed, faces);
    EXPECT_EQ(r.edges_failed, 0);
    EXPECT_EQ(r.vertex_gaps, 0);
    EXPECT_EQ(r.pcurves_checked, 0);
    EXPECT_EQ(r.pcurves_failed, 0);
    EXPECT_EQ(r.pcurves_exact, r.pcurves_computed);
    EXPECT_EQ(r.gap_segments, 0);
    EXPECT_EQ(r.seam_segments, 0);
    EXPECT_LT(r.max_curve_to_surface, 1e-12);
    ExpectValidTrims(m);
    for(size_t b = 0; b < volumes.size(); ++b) {
      EXPECT_TRUE(m.bodies[b].solid) << m.bodies[b].name;
      EXPECT_NEAR(Volume(m, static_cast<int>(b)), volumes[b],
                  1e-5 * volumes[b])
        << m.bodies[b].name;
    }
  }
}
