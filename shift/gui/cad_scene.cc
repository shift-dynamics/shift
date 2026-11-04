#include "shift/gui/cad_scene.h"

#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace shift {
namespace gui {

namespace cad = shift::core::cad;

static constexpr double kQuarterTurn = 0.25 * M_PI; //!< largest patch angle
/*! largest distance of the triangles for ray tracing from the surface,
 * relative to the size of the model */
static constexpr double kTriangleTolerance = 2.5e-4;
static constexpr int kMaxGrid = 32; //!< quads of a patch along u or v

static void Copy4(float out[4], const cad::Vec3& v, double w) {
  out[0] = float(v[0]);
  out[1] = float(v[1]);
  out[2] = float(v[2]);
  out[3] = float(w);
}

static cad::Vec3 Rotate(const cad::Transform& t, const cad::Vec3& v) {
  const auto& r = t.rotation;
  return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
          r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
          r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

/* knot span s of t: knots[s] <= t < knots[s + 1], degree <= s < count */
static int SpanOf(const std::vector<double>& knots, int degree, int count,
                  double t) {
  int s = degree;
  while(s + 1 < count && knots[s + 1] <= t) {
    ++s;
  }
  return s;
}

/* winding number of a closed set of segments (u0, v0, u1, v1) about (u, v):
crossings of the ray towards +u, upward +1 and downward -1 */
static int Winding(const float* segments, const int* indices, int count,
                   float u, float v) {
  int winding = 0;
  for(int k = 0; k < count; ++k) {
    // a negative index ~i: segment i, drawn without an edge line
    const int i = indices ? (indices[k] < 0 ? ~indices[k] : indices[k]) : k;
    const float* s = segments + 4 * i;
    const bool a_below = s[1] <= v, b_below = s[3] <= v;
    if(a_below == b_below) {
      continue;
    }
    const float x = s[0] + (v - s[1]) * (s[2] - s[0]) / (s[3] - s[1]);
    if(x > u) {
      winding += s[3] > s[1] ? 1 : -1;
    }
  }
  return winding;
}

/* whether a segment meets a rectangle (Liang-Barsky clipping) */
static bool SegmentMeetsBox(const float* s, double u0, double v0, double u1,
                            double v1) {
  const double du = s[2] - s[0], dv = s[3] - s[1];
  double t0 = 0.0, t1 = 1.0;
  const double p[4] = {-du, du, -dv, dv};
  const double q[4] = {s[0] - u0, u1 - s[0], s[1] - v0, v1 - s[1]};
  for(int k = 0; k < 4; ++k) {
    if(p[k] == 0.0) {
      if(q[k] < 0.0) {
        return false;
      }
    } else {
      const double r = q[k] / p[k];
      if(p[k] < 0.0) {
        t0 = std::max(t0, r);
      } else {
        t1 = std::min(t1, r);
      }
      if(t0 > t1) {
        return false;
      }
    }
  }
  return true;
}

/*! packs the faces of bodies, see MakeSceneCad */
class CadPacker {
public:
  CadPacker(const cad::Model& model, double scale, double size,
            double tolerance, SceneCad& out)
    : model_(model), scale_(scale), size_(size), tolerance_(tolerance),
      out_(out) {}

  void AddBody(const cad::PlacedBody& placed);

private:
  int AddCurve(const cad::Curve3& c, const cad::Transform& t);
  void AddFace(int face, const cad::Transform& t, const cad::Color& color);
  /* points of a trim loop in (u, v), within tolerance on the surface, and
  whether the segment from each point to the next is on an edge of the
  model (drawn with an edge line), not a seam or singular point */
  void Sample(const cad::Surface& s, const cad::Face& face,
              const cad::TrimLoop& loop, std::vector<cad::Vec2>& points,
              std::vector<bool>& lines) const;
  void Refine(const cad::Surface& s, const cad::TrimCurve& c, double fa,
              const cad::Vec2& a, double fb, const cad::Vec2& b, int depth,
              std::vector<cad::Vec2>& points) const;
  /* parameters at which patches of a surface split along u (dim 0) or v */
  std::vector<double> Breaks(const cad::Surface& s, int dim, double lo,
                             double hi) const;
  /* knot span of a patch along dim, or -1 */
  int Span(const cad::Surface& s, int dim, double lo, double hi) const;
  /* 0: outside the trims, 1: inside, 2: crossed by them */
  int Classify(const std::vector<float>& segments, double u0, double v0,
               double u1, double v1) const;
  void AddPatches(const cad::Surface& s, const cad::Transform& t, int face,
                  const std::vector<float>& segments, double u0, double v0,
                  double u1, double v1, int u_span, int v_span, int n);
  void Grow(const cad::Surface& s, const cad::Transform& t, double u,
            double v);
  /* quads along u (dim 0) or v of a patch for its triangles within the
  ray tracing tolerance */
  int GridSize(const cad::Surface& s, int dim, const CadGpuPatch& p) const;
  /* grids of the patches of a face from first on, shared along its rows
  and columns */
  void SetGrids(const cad::Surface& s, size_t first);

  const cad::Model& model_;
  double scale_;
  double size_; // of the bodies packed (diagonal of their box)
  double tolerance_;
  SceneCad& out_;
};

int CadPacker::AddCurve(const cad::Curve3& c, const cad::Transform& t) {
  CadGpuCurve g{};
  Copy4(g.origin, t.Apply(c.origin), double(static_cast<int>(c.type)));
  if(c.type == cad::Curve3::Type::kLine) {
    Copy4(g.a, Rotate(t, c.direction), 0.0);
  } else if(c.type == cad::Curve3::Type::kConic) {
    Copy4(g.a, Rotate(t, c.a), 0.0);
    Copy4(g.b, Rotate(t, c.b), 0.0);
  } else {
    const auto& n = c.nurbs;
    g.nurbs[0] = n.degree;
    g.nurbs[1] = static_cast<int32_t>(n.points.size());
    g.nurbs[2] = static_cast<int32_t>(out_.knots.size());
    g.nurbs[3] = static_cast<int32_t>(out_.points.size() / 4);
    for(double k : n.knots) {
      out_.knots.push_back(float(k));
    }
    for(size_t i = 0; i < n.points.size(); ++i) {
      const cad::Vec3 p = t.Apply(n.points[i]);
      out_.points.insert(out_.points.end(),
                         {float(p[0]), float(p[1]), float(p[2]),
                          float(n.rational() ? n.weights[i] : 1.0)});
    }
  }
  out_.curves.push_back(g);
  return static_cast<int>(out_.curves.size()) - 1;
}

void CadPacker::Refine(const cad::Surface& s, const cad::TrimCurve& c,
                       double fa, const cad::Vec2& a, double fb,
                       const cad::Vec2& b, int depth,
                       std::vector<cad::Vec2>& points) const {
  const double fm = 0.5 * (fa + fb);
  const cad::Vec2 m = cad::TrimPoint(model_, c, fm);
  const cad::Vec2 chord{0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])};
  const double error =
    cad::Distance(cad::SurfacePoint(model_, s, chord[0], chord[1]),
                  cad::SurfacePoint(model_, s, m[0], m[1]));
  if(error > tolerance_ && depth < 14) {
    Refine(s, c, fa, a, fm, m, depth + 1, points);
    Refine(s, c, fm, m, fb, b, depth + 1, points);
  } else {
    points.push_back(a);
  }
}

void CadPacker::Sample(const cad::Surface& s, const cad::Face& face,
                       const cad::TrimLoop& loop,
                       std::vector<cad::Vec2>& points,
                       std::vector<bool>& lines) const {
  using Kind = cad::TrimCurve::Kind;
  for(const cad::TrimCurve& c : loop.curves) {
    // an edge bounding the face on both sides is a seam of a periodic
    // surface
    bool line = c.kind != Kind::kSingular && c.kind != Kind::kSeam;
    if(line && c.kind == Kind::kEdge) {
      int uses = 0;
      for(const cad::TrimLoop& other : face.trims) {
        for(const cad::TrimCurve& d : other.curves) {
          uses += d.kind == Kind::kEdge && d.edge == c.edge;
        }
      }
      line = uses < 2;
    }
    int n = 1;
    if(c.kind == cad::TrimCurve::Kind::kEdge &&
       model_.edges[c.edge].pcurves[c.pcurve].curve.type !=
         cad::Curve2::Type::kLine) {
      n = 4;
    }
    cad::Vec2 a = cad::TrimPoint(model_, c, 0.0);
    for(int i = 1; i <= n; ++i) {
      const double f = double(i) / n;
      const cad::Vec2 b = cad::TrimPoint(model_, c, f);
      Refine(s, c, double(i - 1) / n, a, f, b, 0, points);
      a = b;
    }
    lines.resize(points.size(), line);
  }
}

std::vector<double> CadPacker::Breaks(const cad::Surface& s, int dim,
                                      double lo, double hi) const {
  using Type = cad::Surface::Type;
  std::vector<double> b{lo};
  auto angle = [&]() {
    const int n = std::max(1, int(std::ceil((hi - lo) / kQuarterTurn - 1e-9)));
    for(int i = 1; i < n; ++i) {
      b.push_back(lo + (hi - lo) * i / n);
    }
  };
  auto knots = [&](const std::vector<double>& k) {
    const double margin = 1e-9 * (hi - lo);
    for(size_t i = 0; i < k.size(); ++i) {
      if(k[i] > lo + margin && k[i] < hi - margin &&
         (i == 0 || k[i] != k[i - 1])) {
        b.push_back(k[i]);
      }
    }
  };
  auto swept = [&]() {
    const cad::Curve3& c = model_.curves[s.curve];
    if(c.type == cad::Curve3::Type::kConic) {
      angle();
    } else if(c.type == cad::Curve3::Type::kNurbs) {
      knots(c.nurbs.knots);
    }
  };
  if(dim == 0) {
    switch(s.type) {
    case Type::kCylinder:
    case Type::kCone:
    case Type::kSphere:
    case Type::kTorus:
    case Type::kRevolution:
      angle();
      break;
    case Type::kNurbs:
      knots(s.nurbs.u_knots);
      break;
    case Type::kExtrusion:
      swept();
      break;
    case Type::kPlane:
      break;
    }
  } else {
    switch(s.type) {
    case Type::kSphere:
    case Type::kTorus:
      angle();
      break;
    case Type::kNurbs:
      knots(s.nurbs.v_knots);
      break;
    case Type::kRevolution:
      swept();
      break;
    default:
      break;
    }
  }
  b.push_back(hi);
  return b;
}

int CadPacker::Span(const cad::Surface& s, int dim, double lo,
                    double hi) const {
  const double t = 0.5 * (lo + hi);
  if(s.type == cad::Surface::Type::kNurbs) {
    const auto& n = s.nurbs;
    return dim == 0 ? SpanOf(n.u_knots, n.u_degree, n.u_count, t)
                    : SpanOf(n.v_knots, n.v_degree, n.v_count, t);
  }
  const bool swept = (dim == 0 && s.type == cad::Surface::Type::kExtrusion) ||
                     (dim == 1 && s.type == cad::Surface::Type::kRevolution);
  if(swept && model_.curves[s.curve].type == cad::Curve3::Type::kNurbs) {
    const auto& n = model_.curves[s.curve].nurbs;
    return SpanOf(n.knots, n.degree, int(n.points.size()), t);
  }
  return -1;
}

int CadPacker::Classify(const std::vector<float>& segments, double u0,
                        double v0, double u1, double v1) const {
  const int count = int(segments.size() / 4);
  for(int k = 0; k < count; ++k) {
    if(SegmentMeetsBox(&segments[4 * k], u0, v0, u1, v1)) {
      return 2;
    }
  }
  return Winding(segments.data(), nullptr, count, float(0.5 * (u0 + u1)),
                 float(0.5 * (v0 + v1))) != 0
           ? 1
           : 0;
}

void CadPacker::Grow(const cad::Surface& s, const cad::Transform& t, double u,
                     double v) {
  const cad::Vec3 p = t.Apply(cad::SurfacePoint(model_, s, u, v));
  for(int k = 0; k < 3; ++k) {
    out_.bounds_min[k] = std::min(out_.bounds_min[k], float(scale_ * p[k]));
    out_.bounds_max[k] = std::max(out_.bounds_max[k], float(scale_ * p[k]));
  }
}

void CadPacker::AddPatches(const cad::Surface& s, const cad::Transform& t,
                           int face, const std::vector<float>& segments,
                           double u0, double v0, double u1, double v1,
                           int u_span, int v_span, int n) {
  // n x n patches, all of them: neighbors of the same size share their
  // corners, while a large patch next to small ones would leave cracks at
  // the small ones' corners. Those wholly outside the trims are dropped.
  if(n > 1) {
    auto at = [n](double a, double b, int i) {
      return i == n ? b : a + (b - a) * i / n;
    };
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < n; ++j) {
        AddPatches(s, t, face, segments, at(u0, u1, i), at(v0, v1, j),
                   at(u0, u1, i + 1), at(v0, v1, j + 1), u_span, v_span, 1);
      }
    }
    return;
  }
  const int c = Classify(segments, u0, v0, u1, v1);
  if(c == 0) {
    return;
  }
  CadGpuPatch p{};
  p.uv[0] = float(u0);
  p.uv[1] = float(v0);
  p.uv[2] = float(u1);
  p.uv[3] = float(v1);
  p.info[0] = face;
  p.info[1] = c == 2 ? 1 : 0;
  p.info[2] = u_span;
  p.info[3] = v_span;
  out_.patches.push_back(p);
  for(int i = 0; i <= 2; ++i) {
    for(int j = 0; j <= 2; ++j) {
      Grow(s, t, u0 + 0.5 * i * (u1 - u0), v0 + 0.5 * j * (v1 - v0));
    }
  }
}

int CadPacker::GridSize(const cad::Surface& s, int dim,
                        const CadGpuPatch& p) const {
  using Type = cad::Surface::Type;
  // straight along the parameter
  if(s.type == Type::kPlane ||
     (dim == 1 && (s.type == Type::kCylinder || s.type == Type::kCone ||
                   s.type == Type::kExtrusion))) {
    return 1;
  }
  // the largest distance of the surface from the chords along three lines
  // of the patch, at a quarter, half and three quarters of the way; n
  // chords leave 1 / n^2 of it
  double deviation = 0.0;
  for(int line = 0; line <= 2; ++line) {
    auto at = [&](double f) {
      const double a = p.uv[dim] + (p.uv[dim + 2] - p.uv[dim]) * f;
      const double b = p.uv[1 - dim] +
                       (p.uv[3 - dim] - p.uv[1 - dim]) * 0.5 * line;
      return dim == 0 ? cad::SurfacePoint(model_, s, a, b)
                      : cad::SurfacePoint(model_, s, b, a);
    };
    const cad::Vec3 a = at(0.0), b = at(1.0);
    for(double f : {0.25, 0.5, 0.75}) {
      const cad::Vec3 chord{a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f,
                            a[2] + (b[2] - a[2]) * f};
      deviation = std::max(deviation, cad::Distance(at(f), chord));
    }
  }
  const double tolerance = kTriangleTolerance * size_;
  return std::clamp(int(std::ceil(std::sqrt(deviation / tolerance))), 1,
                    kMaxGrid);
}

void CadPacker::SetGrids(const cad::Surface& s, size_t first) {
  // patches with the same u range are a column, sharing the edges along u
  // of their neighbors above and below; those with the same v range a row
  std::map<std::pair<float, float>, int> columns, rows;
  const size_t count = out_.patches.size();
  for(size_t i = first; i < count; ++i) {
    const CadGpuPatch& p = out_.patches[i];
    int& nu = columns[{p.uv[0], p.uv[2]}];
    nu = std::max(nu, GridSize(s, 0, p));
    int& nv = rows[{p.uv[1], p.uv[3]}];
    nv = std::max(nv, GridSize(s, 1, p));
  }
  for(size_t i = first; i < count; ++i) {
    const CadGpuPatch& p = out_.patches[i];
    const int nu = columns[{p.uv[0], p.uv[2]}];
    const int nv = rows[{p.uv[1], p.uv[3]}];
    out_.grids.insert(out_.grids.end(), {nu, nv});
    out_.first_triangles.push_back(out_.first_triangles.back() + 2 * nu * nv);
  }
}

void CadPacker::AddFace(int fi, const cad::Transform& t,
                        const cad::Color& body_color) {
  const cad::Face& face = model_.faces[fi];
  if(face.trims.empty()) {
    ++out_.faces_skipped;
    return;
  }
  const cad::Surface& s = model_.surfaces[face.surface];

  // trim segments, and the box of the face in (u, v)
  std::vector<float> segments;
  std::vector<bool> segment_lines;
  double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
  cad::Vec3 box_lo{1e300, 1e300, 1e300}, box_hi{-1e300, -1e300, -1e300};
  for(const cad::TrimLoop& loop : face.trims) {
    std::vector<cad::Vec2> points;
    std::vector<bool> lines;
    Sample(s, face, loop, points, lines);
    const size_t n = points.size();
    for(size_t i = 0; i < n; ++i) {
      const cad::Vec2& a = points[i];
      const cad::Vec2& b = points[(i + 1) % n];
      const cad::Vec3 p = cad::SurfacePoint(model_, s, a[0], a[1]);
      for(int k = 0; k < 3; ++k) {
        box_lo[k] = std::min(box_lo[k], p[k]);
        box_hi[k] = std::max(box_hi[k], p[k]);
      }
      for(int k = 0; k < 2; ++k) {
        lo[k] = std::min(lo[k], a[k]);
        hi[k] = std::max(hi[k], a[k]);
      }
      if(a[0] != b[0] || a[1] != b[1]) {
        segments.insert(segments.end(), {float(a[0]), float(a[1]),
                                         float(b[0]), float(b[1])});
        segment_lines.push_back(lines[i]);
      }
    }
  }
  if(segments.empty() || !(hi[0] > lo[0]) || !(hi[1] > lo[1])) {
    ++out_.faces_skipped;
    return;
  }

  CadGpuFace g{};
  const cad::Frame& f = s.position;
  Copy4(g.origin, t.Apply(f.origin), double(static_cast<int>(s.type)));
  Copy4(g.axis_x, Rotate(t, f.x), s.radius);
  Copy4(g.axis_y, Rotate(t, f.y), s.minor_radius);
  Copy4(g.axis_z, Rotate(t, f.z), std::tan(s.semi_angle));
  Copy4(g.extrusion, Rotate(t, s.extrusion), face.same_sense ? 1.0 : -1.0);
  const cad::Color& color = face.color.set ? face.color : body_color;
  const cad::Vec3 rgb{color.r, color.g, color.b};
  Copy4(g.color, rgb, color.set ? 1.0 : 0.0);
  if(s.type == cad::Surface::Type::kNurbs) {
    const auto& n = s.nurbs;
    g.nurbs[0] = n.u_degree;
    g.nurbs[1] = n.v_degree;
    g.nurbs[2] = n.u_count;
    g.nurbs[3] = n.v_count;
    g.offsets[0] = static_cast<int32_t>(out_.knots.size());
    for(double k : n.u_knots) {
      out_.knots.push_back(float(k));
    }
    for(double k : n.v_knots) {
      out_.knots.push_back(float(k));
    }
    g.offsets[1] = static_cast<int32_t>(out_.points.size() / 4);
    for(size_t i = 0; i < n.points.size(); ++i) {
      const cad::Vec3 p = t.Apply(n.points[i]);
      out_.points.insert(out_.points.end(),
                         {float(p[0]), float(p[1]), float(p[2]),
                          float(n.rational() ? n.weights[i] : 1.0)});
    }
  }
  g.offsets[2] = -1;
  if(s.type == cad::Surface::Type::kRevolution ||
     s.type == cad::Surface::Type::kExtrusion) {
    g.offsets[2] = AddCurve(model_.curves[s.curve], t);
  }

  // bands of v, each with the segments that reach into it
  const int count = static_cast<int>(segments.size() / 4);
  const int bands = std::clamp(count / 4, 1, 128);
  const double height = (hi[1] - lo[1]) / bands;
  std::vector<std::vector<int32_t>> lists(bands);
  const int32_t first_segment = static_cast<int32_t>(out_.segments.size() / 4);
  for(int k = 0; k < count; ++k) {
    const double a = std::min(segments[4 * k + 1], segments[4 * k + 3]);
    const double b = std::max(segments[4 * k + 1], segments[4 * k + 3]);
    const int ba = std::clamp(int(std::floor((a - lo[1]) / height - 1e-3)),
                              0, bands - 1);
    const int bb = std::clamp(int(std::floor((b - lo[1]) / height + 1e-3)),
                              0, bands - 1);
    for(int band = ba; band <= bb; ++band) {
      lists[band].push_back(segment_lines[k] ? first_segment + k
                                             : ~(first_segment + k));
    }
  }
  g.trim_box[0] = float(lo[1]);
  g.trim_box[1] = float(height);
  g.trim[0] = static_cast<int32_t>(out_.trim_index.size());
  g.trim[1] = bands;
  out_.trim_index.resize(out_.trim_index.size() + 2 * size_t(bands));
  for(int band = 0; band < bands; ++band) {
    out_.trim_index[g.trim[0] + 2 * band] =
      static_cast<int32_t>(out_.trim_index.size());
    out_.trim_index[g.trim[0] + 2 * band + 1] =
      static_cast<int32_t>(lists[band].size());
    out_.trim_index.insert(out_.trim_index.end(), lists[band].begin(),
                           lists[band].end());
  }
  out_.segments.insert(out_.segments.end(), segments.begin(), segments.end());
  const int index = static_cast<int>(out_.faces.size());
  out_.faces.push_back(g);

  // patches: split at knots and every quarter turn, then each cell of a
  // face that is large in the model into n x n, so that smaller patches
  // leave out more of what the trims cut away and fewer fragments test the
  // trims
  const std::vector<double> bu = Breaks(s, 0, lo[0], hi[0]);
  const std::vector<double> bv = Breaks(s, 1, lo[1], hi[1]);
  const size_t cells = (bu.size() - 1) * (bv.size() - 1);
  const double extent = cad::Distance(box_lo, box_hi) / size_;
  const int by_cells = cells <= 4 ? 8 : cells <= 32 ? 4 : cells <= 256 ? 2 : 1;
  const int by_extent =
    extent > 0.2 ? 8 : extent > 0.05 ? 4 : extent > 0.01 ? 2 : 1;
  const int n = std::min(by_cells, by_extent);
  const size_t first_patch = out_.patches.size();
  for(size_t i = 0; i + 1 < bu.size(); ++i) {
    const int u_span = Span(s, 0, bu[i], bu[i + 1]);
    for(size_t j = 0; j + 1 < bv.size(); ++j) {
      AddPatches(s, t, index, segments, bu[i], bv[j], bu[i + 1], bv[j + 1],
                 u_span, Span(s, 1, bv[j], bv[j + 1]), n);
    }
  }
  SetGrids(s, first_patch);
}

void CadPacker::AddBody(const cad::PlacedBody& placed) {
  const cad::Body& body = model_.bodies[placed.body];
  for(int shell : body.shells) {
    for(int face : model_.shells[shell].faces) {
      AddFace(face, placed.transform, body.color);
    }
  }
}

SceneCad MakeSceneCad(const cad::Model& model,
                      const std::vector<cad::PlacedBody>& bodies,
                      double scale, double tolerance) {
  SceneCad out;
  out.scale = float(scale);
  out.first_triangles.push_back(0);
  out.bounds_min = QVector3D(1e30f, 1e30f, 1e30f);
  out.bounds_max = -out.bounds_min;
  // size of the bodies, from the box of their vertices
  cad::Vec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
  for(const auto& placed : bodies) {
    for(int shell : model.bodies[placed.body].shells) {
      for(int f : model.shells[shell].faces) {
        for(const auto& loop : model.faces[f].loops) {
          for(const auto& c : loop.coedges) {
            const cad::Vec3 p = placed.transform.Apply(
              model.vertices[model.edges[c.edge].start].point);
            for(int k = 0; k < 3; ++k) {
              lo[k] = std::min(lo[k], p[k]);
              hi[k] = std::max(hi[k], p[k]);
            }
          }
        }
      }
    }
  }
  const double size =
    std::max(hi[0] >= lo[0] ? cad::Distance(lo, hi) : 1.0, 1e-6);
  if(!(tolerance > 0.0)) {
    tolerance = 2e-5 * size;
  }
  CadPacker packer(model, scale, size, tolerance, out);
  for(const auto& placed : bodies) {
    packer.AddBody(placed);
  }
  if(out.patches.empty()) {
    out.bounds_min = out.bounds_max = QVector3D();
  }
  return out;
}

/* the inside test of the fragment shader on the records of a face */
static bool Inside(const CadGpuFace& f, const std::vector<float>& segments,
                   const std::vector<int32_t>& trim_index, float u, float v) {
  const int band = std::clamp(int(std::floor((v - f.trim_box[0]) /
                                             f.trim_box[1])),
                              0, f.trim[1] - 1);
  const int32_t first = trim_index[f.trim[0] + 2 * band];
  const int32_t count = trim_index[f.trim[0] + 2 * band + 1];
  return Winding(segments.data(), trim_index.data() + first, count, u, v) !=
         0;
}

bool CadInside(const SceneCad& cad, int face, float u, float v) {
  return Inside(cad.faces[face], cad.segments, cad.trim_index, u, v);
}

CadTrimTables MergeCadTrims(const std::vector<SceneCad>& cads) {
  CadTrimTables out;
  for(const SceneCad& cad : cads) {
    out.face_base.push_back(static_cast<int>(out.faces.size()));
    const int32_t segment_base =
      static_cast<int32_t>(out.segments.size() / 4);
    const int32_t index_base = static_cast<int32_t>(out.trim_index.size());
    for(CadGpuFace f : cad.faces) {
      f.trim[0] += index_base;
      out.faces.push_back(f);
    }
    out.segments.insert(out.segments.end(), cad.segments.begin(),
                        cad.segments.end());
    // then each band's header (first of its list, count) and list, moved
    // by the tables before it
    out.trim_index.insert(out.trim_index.end(), cad.trim_index.begin(),
                          cad.trim_index.end());
    for(const CadGpuFace& f : cad.faces) {
      for(int band = 0; band < f.trim[1]; ++band) {
        const int32_t h = index_base + f.trim[0] + 2 * band;
        const int32_t list = out.trim_index[h] + index_base;
        const int32_t count = out.trim_index[h + 1];
        out.trim_index[h] = list;
        for(int32_t k = 0; k < count; ++k) {
          int32_t& e = out.trim_index[list + k];
          e = e < 0 ? ~(~e + segment_base) : e + segment_base;
        }
      }
    }
  }
  return out;
}

bool CadInside(const CadTrimTables& tables, int face, float u, float v) {
  return Inside(tables.faces[face], tables.segments, tables.trim_index, u, v);
}

} // namespace gui
} // namespace shift
