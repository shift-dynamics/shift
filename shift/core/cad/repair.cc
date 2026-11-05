#include "shift/core/cad/repair.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

#include "shift/core/cad/evaluate.h"

namespace shift {
namespace core {
namespace cad {

static constexpr double kTwoPi = 2.0 * M_PI;

static double Lerp(double a, double b, double f) {
  return a + f * (b - a);
}

static Vec2 Lerp2(const Vec2& a, const Vec2& b, double f) {
  return {Lerp(a[0], b[0], f), Lerp(a[1], b[1], f)};
}

static Vec2 Add2(const Vec2& a, const Vec2& b) {
  return {a[0] + b[0], a[1] + b[1]};
}

static Vec2 Sub2(const Vec2& a, const Vec2& b) {
  return {a[0] - b[0], a[1] - b[1]};
}

static double Norm2(const Vec2& a) {
  return std::hypot(a[0], a[1]);
}

static double Dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static std::string Ref(const char* what, int step_id) {
  return std::string(what) + " #" + std::to_string(step_id);
}

/* a curve in (u, v) translated by shift */
static void Translate(Curve2& c, const Vec2& shift) {
  if(c.type == Curve2::Type::kNurbs) {
    for(auto& p : c.nurbs.points) {
      p = Add2(p, shift);
    }
  } else {
    c.origin = Add2(c.origin, shift);
  }
}

Vec2 TrimPoint(const Model& model, const TrimCurve& c, double f) {
  if(c.kind != TrimCurve::Kind::kEdge) {
    return Lerp2(c.a, c.b, f);
  }
  const Curve2& curve = model.edges[c.edge].pcurves[c.pcurve].curve;
  return Add2(CurvePoint<2>(curve, Lerp(c.s0, c.s1, f)), c.offset);
}

std::vector<Vec2> TrimPolygon(const Model& model, const TrimLoop& loop,
                              int segments) {
  std::vector<Vec2> polygon;
  for(const TrimCurve& c : loop.curves) {
    int n = 1;
    if(c.kind == TrimCurve::Kind::kEdge) {
      const Curve2& curve = model.edges[c.edge].pcurves[c.pcurve].curve;
      if(curve.type == Curve2::Type::kConic) {
        n = std::max(2, static_cast<int>(std::ceil(
                          segments * std::fabs(c.s1 - c.s0) / kTwoPi)));
      } else if(curve.type == Curve2::Type::kNurbs) {
        // pieces between distinct knots
        const auto& nurbs = curve.nurbs;
        int pieces = 0;
        for(size_t i = 0; i + 1 < nurbs.knots.size(); ++i) {
          pieces += nurbs.knots[i + 1] > nurbs.knots[i] ? 1 : 0;
        }
        n = nurbs.degree == 1 ? pieces
                              : pieces * std::max(1, segments / 8);
        n = std::max(n, 1);
      }
    }
    for(int i = 0; i < n; ++i) {
      polygon.push_back(TrimPoint(model, c, static_cast<double>(i) / n));
    }
  }
  return polygon;
}

double SignedArea(const std::vector<Vec2>& polygon) {
  double area = 0.0;
  const size_t n = polygon.size();
  for(size_t i = 0; i < n; ++i) {
    const Vec2& a = polygon[i];
    const Vec2& b = polygon[(i + 1) % n];
    area += a[0] * b[1] - b[0] * a[1];
  }
  return 0.5 * area;
}

std::string RepairReport::Summary() const {
  std::ostringstream s;
  s << "edges " << edges << " (" << edges_failed << " without range), "
    << "vertex gaps " << vertex_gaps << " (max " << max_vertex_gap << " m)\n"
    << "pcurves: read " << pcurves_checked << " (" << pcurves_not_same_parameter
    << " not same parameter, " << pcurves_replaced << " replaced), computed "
    << pcurves_computed << " (" << pcurves_exact << " exact or straight, "
    << pcurves_failed << " failed), seam twins " << seam_twins
    << ", seams reassigned " << seams_reassigned << ", max deviation "
    << max_pcurve_deviation << " m, max curve to surface "
    << max_curve_to_surface << " m\n"
    << "faces " << faces << " (" << faces_trimmed << " trimmed, "
    << domain_outer_loops << " bounded by their domain); segments: singular "
    << singular_segments << ", seam " << seam_segments << ", join "
    << join_segments << ", gap " << gap_segments << " (max gap " << max_gap
    << " m)";
  return s.str();
}

/* a trim loop being built: curves from start to start + wrap */
struct LoopUV {
  std::vector<TrimCurve> curves;
  Vec2 start{0.0, 0.0};
  Vec2 wrap{0.0, 0.0};
  double score = 0.0;
};

class Repairer {
public:
  Repairer(Model& model, const RepairOptions& options)
      : model_(model), options_(options) {}

  RepairReport Run();

private:
  struct Node {
    double t = 0.0;
    Vec2 uv{0.0, 0.0};
    Vec2 d{0.0, 0.0};
    bool singular = false;
    double off = 0.0; //!< distance of the curve from the surface
  };

  Model& model_;
  RepairOptions options_;
  RepairReport report_;
  /* pcurves that could not be used, per edge */
  std::vector<std::vector<bool>> bad_;

  void Warn(const std::string& s) {
    model_.warnings.push_back(s);
  }

  const Surface& surface(int s) const {
    return model_.surfaces[s];
  }

  Vec3 OnSurface(int s, const Vec2& uv, Vec3* du = nullptr,
                 Vec3* dv = nullptr) const {
    return SurfacePoint(model_, model_.surfaces[s], uv[0], uv[1], du, dv);
  }

  /* periods of the parameters of a surface, 0 if not periodic */
  Vec2 Period(int s) const {
    auto d = SurfaceDomain(model_, model_.surfaces[s]);
    return {d[0].periodic ? d[0].hi - d[0].lo : 0.0,
            d[1].periodic ? d[1].hi - d[1].lo : 0.0};
  }

  /* whole periods to add to p to bring it nearest to q */
  static Vec2 PeriodShift(const Vec2& period, const Vec2& p, const Vec2& q) {
    Vec2 shift{0.0, 0.0};
    for(int k = 0; k < 2; ++k) {
      if(period[k] > 0.0) {
        shift[k] = period[k] * std::round((q[k] - p[k]) / period[k]);
      }
    }
    return shift;
  }

  Vec3 EdgePoint(const Edge& e, double t, Vec3* d = nullptr) const;
  Vec3 PcurvePoint(const Pcurve& pc, double s, Vec3* d = nullptr) const;
  int Samples(const Edge& e) const;
  bool CurveRange(Edge& e);
  bool PcurveOnlyRange(Edge& e);
  bool ProjectToPcurve(const Pcurve& pc, const Vec3& p, double& s,
                       const std::vector<double>& guesses) const;
  double Deviation(const Edge& e, const Pcurve& pc, bool& same_parameter) const;
  bool Check(Edge& e, int index);
  bool Compute(Edge& e, int s, Pcurve& pc);
  bool ComputeOnPlane(const Edge& e, int s, Pcurve& pc) const;
  bool Project(int s, const Vec3& p, Vec2& uv, const Vec2* guess) const;
  bool Singular(int s, const Vec2& uv, bool& in_u) const;
  bool Tangent(int s, const Vec2& uv, const Vec3& d, Vec2& duv) const;
  Node MakeNode(const Edge& e, int s, double t, const Vec2* guess) const;
  static Vec2 Hermite(const Node& a, const Node& b, double f);
  double SegmentError(const Edge& e, int s, const Node& a,
                      const Node& b) const;
  bool AddTwin(Edge& e, int index, int s);
  void EnsurePcurves(Face& f);
  TrimCurve Segment(int s, const Vec2& a, const Vec2& b, double gap_tolerance);
  bool Chain(const Face& f, const Loop& loop, LoopUV& out);
  void Shift(LoopUV& loop, const Vec2& shift) const;
  void Reverse(LoopUV& loop) const;
  void Close(LoopUV& loop, const Vec2& from, const Vec2& to,
             TrimCurve::Kind kind);
  bool Trim(Face& f);
};

Vec3 Repairer::EdgePoint(const Edge& e, double t, Vec3* d) const {
  if(e.curve >= 0) {
    return CurvePoint<3>(model_.curves[e.curve], t, d);
  }
  return PcurvePoint(e.pcurves[0], t, d);
}

Vec3 Repairer::PcurvePoint(const Pcurve& pc, double s, Vec3* d) const {
  Vec2 dc;
  Vec2 c = CurvePoint<2>(pc.curve, s, &dc);
  Vec3 du, dv;
  Vec3 p = OnSurface(pc.surface, c, &du, &dv);
  if(d) {
    for(int k = 0; k < 3; ++k) {
      (*d)[k] = du[k] * dc[0] + dv[k] * dc[1];
    }
  }
  return p;
}

/* number of samples along an edge for checks and the start of fits */
int Repairer::Samples(const Edge& e) const {
  if(e.curve < 0) {
    return 16;
  }
  const Curve3& c = model_.curves[e.curve];
  const double span = std::fabs(e.range[1] - e.range[0]);
  if(c.type == Curve3::Type::kConic) {
    return std::max(4, static_cast<int>(std::ceil(16.0 * span / kTwoPi)));
  }
  if(c.type == Curve3::Type::kNurbs) {
    const auto& n = c.nurbs;
    const double lo = std::min(e.range[0], e.range[1]);
    const double hi = std::max(e.range[0], e.range[1]);
    int spans = 0;
    for(size_t i = n.degree; i + 1 < n.knots.size() - n.degree; ++i) {
      if(n.knots[i + 1] > n.knots[i] && n.knots[i + 1] > lo &&
         n.knots[i] < hi) {
        ++spans;
      }
    }
    return std::max(4, 2 * spans * std::max(1, n.degree));
  }
  return 4;
}

bool Repairer::CurveRange(Edge& e) {
  const Curve3& c = model_.curves[e.curve];
  const Vec3& ps = model_.vertices[e.start].point;
  const Vec3& pe = model_.vertices[e.end].point;
  const bool closed = e.start == e.end;
  const Interval domain = CurveDomain<3>(c);
  double ts = 0.0, te = 0.0;
  if(c.type == Curve3::Type::kNurbs && closed) {
    ts = e.same_sense ? domain.lo : domain.hi;
    te = e.same_sense ? domain.hi : domain.lo;
  } else {
    if(!ProjectToCurve(c, ps, ts)) {
      return false;
    }
    if(closed) {
      if(c.type != Curve3::Type::kConic) {
        Warn(Ref("closed edge along a line", e.step_id));
        return false;
      }
      te = e.same_sense ? ts + kTwoPi : ts - kTwoPi;
    } else {
      if(!ProjectToCurve(c, pe, te)) {
        return false;
      }
      if(c.type == Curve3::Type::kConic) {
        double d = te - ts;
        d -= kTwoPi * std::floor(d / kTwoPi);
        te = e.same_sense ? ts + d : ts - (kTwoPi - d);
        if(!e.same_sense && d == 0.0) {
          te = ts;
        }
      } else if(c.type == Curve3::Type::kNurbs &&
                (te > ts) != e.same_sense) {
        // a closed curve whose seam is at one of the vertices
        const double gap = Distance(CurvePoint<3>(c, domain.lo),
                                    CurvePoint<3>(c, domain.hi));
        if(gap <= options_.tolerance) {
          const double near = 1e-9 * (domain.hi - domain.lo);
          if(e.same_sense) {
            if(te - domain.lo <= near) {
              te = domain.hi;
            } else if(domain.hi - ts <= near) {
              ts = domain.lo;
            }
          } else {
            if(ts - domain.lo <= near) {
              ts = domain.hi;
            } else if(domain.hi - te <= near) {
              te = domain.lo;
            }
          }
        }
      }
      if(ts == te || (te > ts) != e.same_sense) {
        Warn(Ref("edge runs against the sense of its curve", e.step_id));
      }
    }
  }
  e.range = {ts, te};
  e.has_range = true;
  const double gap = std::max(Distance(CurvePoint<3>(c, ts), ps),
                              Distance(CurvePoint<3>(c, te), pe));
  e.tolerance = std::max(e.tolerance, gap);
  report_.max_vertex_gap = std::max(report_.max_vertex_gap, gap);
  if(gap > options_.tolerance) {
    ++report_.vertex_gaps;
    Warn(Ref("vertex off the curve of edge", e.step_id) + " by " +
         std::to_string(gap) + " m");
  }
  return true;
}

bool Repairer::ProjectToPcurve(const Pcurve& pc, const Vec3& p, double& s,
                               const std::vector<double>& guesses) const {
  const Interval domain = CurveDomain<2>(pc.curve);
  double best = 1e300;
  bool found = false;
  for(double guess : guesses) {
    double x = guess;
    for(int it = 0; it < 50; ++it) {
      Vec3 d;
      Vec3 q = PcurvePoint(pc, x, &d);
      const double dd = Dot(d, d);
      if(dd == 0.0) {
        break;
      }
      double next = x - Dot({q[0] - p[0], q[1] - p[1], q[2] - p[2]}, d) / dd;
      if(domain.bounded && !domain.periodic) {
        next = std::clamp(next, domain.lo, domain.hi);
      }
      const double step = std::fabs(next - x);
      x = next;
      if(step <= 1e-15 * (1.0 + std::fabs(x))) {
        break;
      }
    }
    const double dist = Distance(PcurvePoint(pc, x), p);
    if(dist < best) {
      best = dist;
      s = x;
      found = true;
    }
  }
  return found;
}

/* guesses for the parameter of a point of a pcurve: samples of its domain,
or for a line the projections of the point's (u, v), translated by periods */
static std::vector<double> PcurveGuesses(const Model& model, const Pcurve& pc,
                                         const Vec3& p) {
  std::vector<double> g;
  const Interval domain = CurveDomain<2>(pc.curve);
  if(domain.bounded) {
    const int n = 32;
    for(int i = 0; i <= n; ++i) {
      g.push_back(Lerp(domain.lo, domain.hi, static_cast<double>(i) / n));
    }
    return g;
  }
  Vec2 uv;
  if(!ProjectToSurface(model, model.surfaces[pc.surface], p, uv)) {
    return g;
  }
  const Vec2& d = pc.curve.direction;
  const double dd = d[0] * d[0] + d[1] * d[1];
  if(dd == 0.0) {
    return g;
  }
  auto sd = SurfaceDomain(model, model.surfaces[pc.surface]);
  for(int i = -1; i <= 1; ++i) {
    for(int j = -1; j <= 1; ++j) {
      if((i != 0 && !sd[0].periodic) || (j != 0 && !sd[1].periodic)) {
        continue;
      }
      Vec2 q{uv[0] + i * kTwoPi - pc.curve.origin[0],
             uv[1] + j * kTwoPi - pc.curve.origin[1]};
      g.push_back((q[0] * d[0] + q[1] * d[1]) / dd);
    }
  }
  return g;
}

bool Repairer::PcurveOnlyRange(Edge& e) {
  Pcurve& pc = e.pcurves[0];
  const Vec3& ps = model_.vertices[e.start].point;
  const Vec3& pe = model_.vertices[e.end].point;
  const Interval domain = CurveDomain<2>(pc.curve);
  double ss = 0.0, se = 0.0;
  if(e.start == e.end) {
    if(pc.curve.type == Curve2::Type::kNurbs) {
      ss = domain.lo;
      se = domain.hi;
    } else if(pc.curve.type == Curve2::Type::kConic &&
              ProjectToPcurve(pc, ps, ss, PcurveGuesses(model_, pc, ps))) {
      se = ss + kTwoPi;
    } else {
      Warn(Ref("closed edge without a curve", e.step_id));
      return false;
    }
    if(!e.same_sense) {
      std::swap(ss, se);
    }
  } else if(!ProjectToPcurve(pc, ps, ss, PcurveGuesses(model_, pc, ps)) ||
            !ProjectToPcurve(pc, pe, se, PcurveGuesses(model_, pc, pe))) {
    return false;
  }
  pc.range = {ss, se};
  pc.same_parameter = true;
  e.range = pc.range;
  e.has_range = true;
  const double gap = std::max(Distance(PcurvePoint(pc, ss), ps),
                              Distance(PcurvePoint(pc, se), pe));
  e.tolerance = std::max(e.tolerance, gap);
  report_.max_vertex_gap = std::max(report_.max_vertex_gap, gap);
  if(gap > options_.tolerance) {
    ++report_.vertex_gaps;
    Warn(Ref("vertex off the pcurve of edge", e.step_id) + " by " +
         std::to_string(gap) + " m");
  }
  return true;
}

/* largest distance between an edge and its pcurve on the surface: between
points at the same fraction of their ranges if that is within tolerance
(same_parameter), otherwise from points of the pcurve to the edge's curve */
double Repairer::Deviation(const Edge& e, const Pcurve& pc,
                           bool& same_parameter) const {
  const int n = 2 * Samples(e);
  double d1 = 0.0;
  for(int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    d1 = std::max(d1, Distance(EdgePoint(e, Lerp(e.range[0], e.range[1], f)),
                               PcurvePoint(pc, Lerp(pc.range[0], pc.range[1],
                                                    f))));
  }
  same_parameter = d1 <= options_.tolerance;
  if(same_parameter || e.curve < 0) {
    return d1;
  }
  const Curve3& c = model_.curves[e.curve];
  double d2 = 0.0;
  for(int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    Vec3 q = PcurvePoint(pc, Lerp(pc.range[0], pc.range[1], f));
    double t = Lerp(e.range[0], e.range[1], f);
    const double guess = t;
    if(!ProjectToCurve(c, q, t, &guess)) {
      return 1e300;
    }
    // only the edge's part of the curve counts (a pcurve may follow its
    // circle the wrong way round)
    t = std::clamp(t, std::min(e.range[0], e.range[1]),
                   std::max(e.range[0], e.range[1]));
    d2 = std::max(d2, Distance(CurvePoint<3>(c, t), q));
  }
  return d2;
}

/* checks a pcurve read from the file; false if it must be replaced */
bool Repairer::Check(Edge& e, int index) {
  Pcurve& pc = e.pcurves[index];
  const Vec3& ps = model_.vertices[e.start].point;
  const Vec3& pe = model_.vertices[e.end].point;
  const double near = options_.tolerance + e.tolerance;
  // most pcurves share the parameter of their edge's curve
  if(Distance(PcurvePoint(pc, e.range[0]), ps) <= near &&
     Distance(PcurvePoint(pc, e.range[1]), pe) <= near) {
    pc.range = e.range;
  } else if(e.start == e.end) {
    Warn(Ref("pcurve of closed edge", e.step_id) +
         " does not share its parameter, replaced");
    return false;
  } else {
    std::vector<double> gs = PcurveGuesses(model_, pc, ps);
    gs.push_back(e.range[0]);
    std::vector<double> ge = PcurveGuesses(model_, pc, pe);
    ge.push_back(e.range[1]);
    if(!ProjectToPcurve(pc, ps, pc.range[0], gs) ||
       !ProjectToPcurve(pc, pe, pc.range[1], ge)) {
      return false;
    }
    // a pcurve runs the same way as its edge's curve: on a closed pcurve, a
    // vertex at the point where it closes may be at either end of it
    if((pc.range[1] - pc.range[0]) * (e.range[1] - e.range[0]) <= 0.0) {
      const Interval d = CurveDomain<2>(pc.curve);
      const double width = d.hi - d.lo;
      const bool up = e.range[1] > e.range[0];
      if(d.periodic) {
        pc.range[1] += up ? width : -width;
        if((pc.range[1] - pc.range[0]) * (e.range[1] - e.range[0]) <= 0.0) {
          pc.range[1] += up ? width : -width;
        }
      } else if(d.bounded &&
                Distance(PcurvePoint(pc, d.lo), PcurvePoint(pc, d.hi)) <=
                  near) {
        const double at = 1e-9 * width;
        auto other = [&](double x) {
          return std::fabs(x - d.lo) <= at   ? d.hi
                 : std::fabs(x - d.hi) <= at ? d.lo
                                             : x;
        };
        const double r1 = other(pc.range[1]);
        if((r1 - pc.range[0]) * (e.range[1] - e.range[0]) > 0.0) {
          pc.range[1] = r1;
        } else {
          pc.range[0] = other(pc.range[0]);
        }
      }
    }
    // a pcurve that stops short of a vertex
    const double gap = std::max(Distance(PcurvePoint(pc, pc.range[0]), ps),
                                Distance(PcurvePoint(pc, pc.range[1]), pe));
    if(gap > near) {
      Warn(Ref("pcurve of edge", e.step_id) + " ends " + std::to_string(gap) +
           " m from its vertex, replaced");
      return false;
    }
  }
  bool same = false;
  const double deviation = Deviation(e, pc, same);
  pc.same_parameter = same;
  if(!same) {
    ++report_.pcurves_not_same_parameter;
  }
  if(deviation > options_.tolerance) {
    Warn(Ref("pcurve off the curve of edge", e.step_id) + " by " +
         std::to_string(deviation) + " m, replaced");
    return false;
  }
  e.tolerance = std::max(e.tolerance, deviation);
  report_.max_pcurve_deviation =
    std::max(report_.max_pcurve_deviation, deviation);
  return true;
}

bool Repairer::ComputeOnPlane(const Edge& e, int s, Pcurve& pc) const {
  if(e.curve < 0) {
    return false;
  }
  const Frame& f = surface(s).position;
  const Curve3& c = model_.curves[e.curve];
  auto point = [&](const Vec3& p) -> Vec2 {
    Vec3 r{p[0] - f.origin[0], p[1] - f.origin[1], p[2] - f.origin[2]};
    return {Dot(r, f.x), Dot(r, f.y)};
  };
  auto vector = [&](const Vec3& v) -> Vec2 {
    return {Dot(v, f.x), Dot(v, f.y)};
  };
  Curve2 out;
  switch(c.type) {
  case Curve3::Type::kLine:
    out.type = Curve2::Type::kLine;
    out.origin = point(c.origin);
    out.direction = vector(c.direction);
    break;
  case Curve3::Type::kConic:
    out.type = Curve2::Type::kConic;
    out.origin = point(c.origin);
    out.a = vector(c.a);
    out.b = vector(c.b);
    break;
  case Curve3::Type::kNurbs:
    out.type = Curve2::Type::kNurbs;
    out.nurbs.degree = c.nurbs.degree;
    out.nurbs.knots = c.nurbs.knots;
    out.nurbs.weights = c.nurbs.weights;
    for(const Vec3& p : c.nurbs.points) {
      out.nurbs.points.push_back(point(p));
    }
    break;
  }
  pc.surface = s;
  pc.curve = out;
  pc.range = e.range;
  return true;
}

bool Repairer::Project(int s, const Vec3& p, Vec2& uv,
                       const Vec2* guess) const {
  return ProjectToSurface(model_, model_.surfaces[s], p, uv, guess);
}

/* whether the surface is singular at uv, or so near a singular point that
running one parameter across its whole domain moves the point less than the
fit tolerance (in_u: u is that parameter) */
bool Repairer::Singular(int s, const Vec2& uv, bool& in_u) const {
  Vec3 du, dv;
  OnSurface(s, uv, &du, &dv);
  const auto domain = SurfaceDomain(model_, surface(s));
  const double a = std::sqrt(Dot(du, du)), c = std::sqrt(Dot(dv, dv));
  const double wu = domain[0].bounded ? domain[0].hi - domain[0].lo : 0.0;
  const double wv = domain[1].bounded ? domain[1].hi - domain[1].lo : 0.0;
  if(a <= 1e-10 * c || (wu > 0.0 && a * wu <= options_.fit_tolerance)) {
    in_u = true;
    return true;
  }
  if(c <= 1e-10 * a || (wv > 0.0 && c * wv <= options_.fit_tolerance)) {
    in_u = false;
    return true;
  }
  return false;
}

/* the rate of change of (u, v) along a curve on the surface with rate of
change d in 3D */
bool Repairer::Tangent(int s, const Vec2& uv, const Vec3& d, Vec2& duv) const {
  Vec3 du, dv;
  OnSurface(s, uv, &du, &dv);
  const double a = Dot(du, du), b = Dot(du, dv), c = Dot(dv, dv);
  const double det = a * c - b * b;
  if(det <= 1e-12 * a * c || a == 0.0 || c == 0.0) {
    return false;
  }
  const double g0 = Dot(du, d), g1 = Dot(dv, d);
  duv = {(c * g0 - b * g1) / det, (a * g1 - b * g0) / det};
  return true;
}

Repairer::Node Repairer::MakeNode(const Edge& e, int s, double t,
                                  const Vec2* guess) const {
  Node n;
  n.t = t;
  Vec3 d;
  Vec3 p = EdgePoint(e, t, &d);
  Project(s, p, n.uv, guess);
  n.off = Distance(OnSurface(s, n.uv), p);
  if(guess && n.off > options_.fit_tolerance) {
    // the projection from the previous point may have stopped at a local
    // minimum of the distance: search the whole surface
    Vec2 uv;
    if(Project(s, p, uv, nullptr)) {
      const double off = Distance(OnSurface(s, uv), p);
      if(off < 0.5 * n.off) {
        n.uv = Add2(uv, PeriodShift(Period(s), uv, *guess));
        n.off = off;
      }
    }
  }
  bool in_u = false;
  if(Singular(s, n.uv, in_u)) {
    // the parameter that is arbitrary here continues from the guess
    if(guess) {
      n.uv[in_u ? 0 : 1] = (*guess)[in_u ? 0 : 1];
    }
    n.singular = true;
  } else {
    n.singular = !Tangent(s, n.uv, d, n.d);
  }
  return n;
}

/* cubic Hermite interpolation of (u, v) between nodes; at a singular node
the chord gives the tangent */
Vec2 Repairer::Hermite(const Node& a, const Node& b, double f) {
  const double h = b.t - a.t;
  const Vec2 chord{(b.uv[0] - a.uv[0]) / h, (b.uv[1] - a.uv[1]) / h};
  const Vec2 ma = a.singular ? chord : a.d;
  const Vec2 mb = b.singular ? chord : b.d;
  const double f2 = f * f, f3 = f2 * f;
  const double h00 = 2 * f3 - 3 * f2 + 1, h10 = f3 - 2 * f2 + f;
  const double h01 = -2 * f3 + 3 * f2, h11 = f3 - f2;
  return {h00 * a.uv[0] + h10 * h * ma[0] + h01 * b.uv[0] + h11 * h * mb[0],
          h00 * a.uv[1] + h10 * h * ma[1] + h01 * b.uv[1] + h11 * h * mb[1]};
}

double Repairer::SegmentError(const Edge& e, int s, const Node& a,
                              const Node& b) const {
  // distance from the point of the surface nearest to the curve, so that a
  // curve slightly off its surface can still be fitted within tolerance,
  // and how much farther from the curve that point is than at the nodes
  const double allowed = std::max(a.off, b.off) + options_.fit_tolerance;
  double err = 0.0;
  for(double f : {0.25, 0.5, 0.75}) {
    const Vec2 h = Hermite(a, b, f);
    const Vec3 p = EdgePoint(e, Lerp(a.t, b.t, f));
    Vec2 foot;
    if(!Project(s, p, foot, &h)) {
      return 1e300;
    }
    const Vec3 q = OnSurface(s, foot);
    err = std::max({err, Distance(OnSurface(s, h), q),
                    Distance(q, p) - allowed});
  }
  return err;
}

bool Repairer::Compute(Edge& e, int s, Pcurve& pc) {
  pc = Pcurve();
  if(e.range[0] == e.range[1]) {
    return false;
  }
  pc.surface = s;
  pc.computed = true;
  pc.same_parameter = true;
  pc.range = e.range;
  if(surface(s).type == Surface::Type::kPlane && ComputeOnPlane(e, s, pc)) {
    bool same = false;
    const double deviation = Deviation(e, pc, same);
    if(deviation <= options_.fit_tolerance) {
      pc.computed = true;
      pc.same_parameter = true;
      e.tolerance = std::max(e.tolerance, deviation);
      report_.max_pcurve_deviation =
        std::max(report_.max_pcurve_deviation, deviation);
      ++report_.pcurves_exact;
      return true;
    }
    pc.curve = Curve2();
  }
  const int n = Samples(e);
  std::vector<Node> nodes;
  for(int i = 0; i <= n; ++i) {
    const double t =
      Lerp(e.range[0], e.range[1], static_cast<double>(i) / n);
    nodes.push_back(MakeNode(e, s, t, nodes.empty() ? nullptr
                                                    : &nodes.back().uv));
  }
  // at a singular point (u, v) is arbitrary in one parameter: take it from
  // the neighbouring node
  // (the first node has no guess: it takes the parameter from the first
  // node that is not singular)
  bool in_u = false;
  if(Singular(s, nodes[0].uv, in_u)) {
    for(size_t j = 1; j < nodes.size(); ++j) {
      if(!nodes[j].singular) {
        for(size_t i = 0; i < j; ++i) {
          nodes[i].uv[in_u ? 0 : 1] = nodes[j].uv[in_u ? 0 : 1];
        }
        break;
      }
    }
  }
  const double scale = 1.0 + std::max(Norm2(nodes.front().uv),
                                      Norm2(nodes.back().uv));
  // straight in (u, v)?
  {
    const Node& a = nodes.front();
    const Node& b = nodes.back();
    const double h = b.t - a.t;
    const Vec2 d{(b.uv[0] - a.uv[0]) / h, (b.uv[1] - a.uv[1]) / h};
    bool straight = true;
    for(size_t i = 1; i + 1 < nodes.size() && straight; ++i) {
      Vec2 q{a.uv[0] + (nodes[i].t - a.t) * d[0],
             a.uv[1] + (nodes[i].t - a.t) * d[1]};
      straight = Norm2(Sub2(q, nodes[i].uv)) <= 1e-9 * scale;
    }
    for(size_t i = 0; i + 1 < nodes.size() && straight; ++i) {
      const double t = 0.5 * (nodes[i].t + nodes[i + 1].t);
      Vec2 q{a.uv[0] + (t - a.t) * d[0], a.uv[1] + (t - a.t) * d[1]};
      straight = Distance(OnSurface(s, q), EdgePoint(e, t)) <=
                 options_.fit_tolerance;
    }
    if(straight) {
      pc.curve.type = Curve2::Type::kLine;
      pc.curve.origin = {a.uv[0] - a.t * d[0], a.uv[1] - a.t * d[1]};
      pc.curve.direction = d;
      bool same = false;
      const double deviation = Deviation(e, pc, same);
      e.tolerance = std::max(e.tolerance, deviation);
      report_.max_pcurve_deviation =
        std::max(report_.max_pcurve_deviation, deviation);
      ++report_.pcurves_exact;
      return true;
    }
  }
  // piecewise cubic, subdivided until within fit_tolerance
  double worst = 0.0;
  for(size_t i = 0; i + 1 < nodes.size();) {
    const double err = SegmentError(e, s, nodes[i], nodes[i + 1]);
    if(err <= options_.fit_tolerance ||
       static_cast<int>(nodes.size()) > options_.max_segments) {
      worst = std::max(worst, err);
      ++i;
      continue;
    }
    const double t = 0.5 * (nodes[i].t + nodes[i + 1].t);
    const Vec2 guess = Hermite(nodes[i], nodes[i + 1], 0.5);
    nodes.insert(nodes.begin() + i + 1, MakeNode(e, s, t, &guess));
  }
  if(worst > options_.fit_tolerance) {
    static const char* curve_types[] = {"line", "conic", "B-spline"};
    static const char* surface_types[] = {
      "plane", "cylinder", "cone", "sphere", "torus", "B-spline",
      "revolution", "extrusion"};
    Warn(Ref("pcurve of edge", e.step_id) + " (" +
         (e.curve >= 0
            ? curve_types[static_cast<int>(model_.curves[e.curve].type)]
            : "pcurve") +
         ") on " + Ref(surface_types[static_cast<int>(surface(s).type)],
                       surface(s).step_id) +
         " fitted only within " + std::to_string(worst) + " m, " +
         std::to_string(nodes.size() - 1) + " pieces");
  }
  // Bezier pieces of a degree 3 B-spline, with knots of multiplicity 3
  // between pieces
  NurbsCurve<2>& b = pc.curve.nurbs;
  pc.curve.type = Curve2::Type::kNurbs;
  b.degree = 3;
  // knots must increase: a reversed range is handled by the sign of h
  const bool increasing = e.range[1] > e.range[0];
  std::vector<Node> ordered = nodes;
  if(!increasing) {
    std::reverse(ordered.begin(), ordered.end());
  }
  b.knots.assign(4, ordered.front().t);
  b.points.push_back(ordered.front().uv);
  for(size_t i = 0; i + 1 < ordered.size(); ++i) {
    const Node& p = ordered[i];
    const Node& q = ordered[i + 1];
    const double h = q.t - p.t;
    const Vec2 chord{(q.uv[0] - p.uv[0]) / h, (q.uv[1] - p.uv[1]) / h};
    const Vec2 mp = p.singular ? chord : p.d;
    const Vec2 mq = q.singular ? chord : q.d;
    b.points.push_back({p.uv[0] + h * mp[0] / 3.0, p.uv[1] + h * mp[1] / 3.0});
    b.points.push_back({q.uv[0] - h * mq[0] / 3.0, q.uv[1] - h * mq[1] / 3.0});
    b.points.push_back(q.uv);
    for(int k = 0; k < (i + 2 < ordered.size() ? 3 : 4); ++k) {
      b.knots.push_back(q.t);
    }
  }
  double off = 0.0;
  for(const Node& node : nodes) {
    off = std::max(off, node.off);
  }
  e.tolerance = std::max({e.tolerance, worst, off});
  report_.max_pcurve_deviation = std::max(report_.max_pcurve_deviation, worst);
  report_.max_curve_to_surface = std::max(report_.max_curve_to_surface, off);
  if(off > options_.tolerance) {
    Warn(Ref("curve of edge", e.step_id) + " is up to " +
         std::to_string(off) + " m from " +
         Ref("its surface", surface(s).step_id));
  }
  return true;
}

/* the pcurve of the other side of a seam: index translated by a period, or
across the domain of a closed surface */
bool Repairer::AddTwin(Edge& e, int index, int s) {
  const Pcurve& pc = e.pcurves[index];
  const auto domain = SurfaceDomain(model_, surface(s));
  const int n = 8;
  Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
  for(int i = 0; i <= n; ++i) {
    Vec2 c = CurvePoint<2>(
      pc.curve, Lerp(pc.range[0], pc.range[1], static_cast<double>(i) / n));
    for(int k = 0; k < 2; ++k) {
      lo[k] = std::min(lo[k], c[k]);
      hi[k] = std::max(hi[k], c[k]);
    }
  }
  for(int k = 0; k < 2; ++k) {
    if(!domain[k].bounded) {
      continue;
    }
    const double width = domain[k].hi - domain[k].lo;
    if(hi[k] - lo[k] > 1e-9 * width) {
      continue;
    }
    const double x = 0.5 * (lo[k] + hi[k]);
    Vec2 shift{0.0, 0.0};
    if(domain[k].periodic) {
      double r = x - domain[k].lo;
      r -= width * std::floor(r / width);
      shift[k] = r < 0.5 * width ? width : -width;
    } else if(std::fabs(x - domain[k].lo) <= 1e-9 * width) {
      shift[k] = width;
    } else if(std::fabs(x - domain[k].hi) <= 1e-9 * width) {
      shift[k] = -width;
    } else {
      continue;
    }
    Pcurve twin = pc;
    Translate(twin.curve, shift);
    bool same = false;
    const double deviation = Deviation(e, twin, same);
    if(deviation > std::max(options_.fit_tolerance, e.tolerance) * 10.0) {
      continue;
    }
    twin.computed = true;
    e.pcurves.push_back(twin);
    bad_[&e - model_.edges.data()].push_back(false);
    ++report_.seam_twins;
    return true;
  }
  return false;
}

/* makes sure every edge of a face has a usable pcurve on its surface, and
seams one for each side */
void Repairer::EnsurePcurves(Face& f) {
  std::map<int, int> uses;
  for(const Loop& loop : f.loops) {
    for(const Coedge& c : loop.coedges) {
      ++uses[c.edge];
    }
  }
  for(auto [ei, count] : uses) {
    Edge& e = model_.edges[ei];
    if(!e.has_range) {
      continue;
    }
    std::vector<int> found;
    for(size_t i = 0; i < e.pcurves.size(); ++i) {
      if(e.pcurves[i].surface == f.surface && !bad_[ei][i]) {
        found.push_back(static_cast<int>(i));
      }
    }
    if(found.empty()) {
      Pcurve pc;
      ++report_.pcurves_computed;
      if(!Compute(e, f.surface, pc)) {
        ++report_.pcurves_failed;
        Warn(Ref("no pcurve for edge", e.step_id) + " on " +
             Ref("face", f.step_id));
        continue;
      }
      e.pcurves.push_back(pc);
      bad_[ei].push_back(false);
      found.push_back(static_cast<int>(e.pcurves.size()) - 1);
    }
    if(count >= 2 && found.size() == 1) {
      AddTwin(e, found[0], f.surface);
    }
  }
}

/* straight trim curve from a to b closing a gap in a loop */
TrimCurve Repairer::Segment(int s, const Vec2& a, const Vec2& b,
                            double gap_tolerance) {
  TrimCurve c;
  c.a = a;
  c.b = b;
  const Vec3 pa = OnSurface(s, a);
  const double d = std::max({Distance(pa, OnSurface(s, b)),
                             Distance(pa, OnSurface(s, Lerp2(a, b, 0.5)))});
  const double uv_gap = Norm2(Sub2(b, a));
  const double scale = 1.0 + std::max(Norm2(a), Norm2(b));
  if(d > gap_tolerance) {
    c.kind = TrimCurve::Kind::kGap;
    report_.max_gap = std::max(report_.max_gap, d);
  } else if(uv_gap > 1e-6 * scale) {
    c.kind = TrimCurve::Kind::kSingular;
  } else {
    c.kind = TrimCurve::Kind::kJoin;
  }
  return c;
}

/* the coedges of a loop as trim curves in (u, v), choosing for each edge
the pcurve, and the period offset, that continue from the previous one */
bool Repairer::Chain(const Face& f, const Loop& loop, LoopUV& out) {
  const int s = f.surface;
  const Vec2 period = Period(s);
  struct Use {
    int edge;
    bool forward;
    int file_pcurve;
    std::vector<int> candidates;
  };
  std::vector<Use> uses;
  for(const Coedge& c : loop.coedges) {
    Use u{c.edge, c.orientation == loop.orientation, c.pcurve, {}};
    const Edge& e = model_.edges[c.edge];
    if(!e.has_range) {
      Warn(Ref("edge", e.step_id) + " without range in " +
           Ref("face", f.step_id));
      return false;
    }
    for(size_t i = 0; i < e.pcurves.size(); ++i) {
      if(e.pcurves[i].surface == s && !bad_[c.edge][i]) {
        u.candidates.push_back(static_cast<int>(i));
      }
    }
    if(u.candidates.empty()) {
      return false;
    }
    uses.push_back(u);
  }
  if(!loop.orientation) {
    std::reverse(uses.begin(), uses.end());
  }
  auto curve_of = [&](const Use& u, int index) {
    const Pcurve& pc = model_.edges[u.edge].pcurves[index];
    TrimCurve c;
    c.edge = u.edge;
    c.pcurve = index;
    c.s0 = u.forward ? pc.range[0] : pc.range[1];
    c.s1 = u.forward ? pc.range[1] : pc.range[0];
    return c;
  };
  // try each pcurve of the first edge (both sides of a seam), and keep the
  // chain with the smallest gaps
  bool have = false;
  for(int first : uses[0].candidates) {
    LoopUV chain;
    TrimCurve c = curve_of(uses[0], first);
    chain.curves.push_back(c);
    chain.start = TrimPoint(model_, c, 0.0);
    Vec2 end = TrimPoint(model_, c, 1.0);
    for(size_t k = 1; k < uses.size(); ++k) {
      double best = 1e300;
      TrimCurve chosen;
      for(int index : uses[k].candidates) {
        TrimCurve d = curve_of(uses[k], index);
        Vec2 p = TrimPoint(model_, d, 0.0);
        d.offset = PeriodShift(period, p, end);
        const double gap = Norm2(Sub2(Add2(p, d.offset), end));
        // on a tie (both sides of a seam of a periodic surface) keep the
        // file's choice
        const double scale = 1e-12 * (1.0 + Norm2(end));
        if(gap < best - scale ||
           (gap <= best + scale && index == uses[k].file_pcurve)) {
          best = gap;
          chosen = d;
        }
      }
      chain.score += best;
      chain.curves.push_back(chosen);
      end = TrimPoint(model_, chosen, 1.0);
    }
    chain.wrap = PeriodShift(period, chain.start, end);
    chain.score += Norm2(Sub2(Add2(chain.start, chain.wrap), end));
    if(!have || chain.score < out.score) {
      out = chain;
      have = true;
    }
  }
  // seams: the pcurve chosen for each coedge
  for(size_t k = 0; k < uses.size(); ++k) {
    const TrimCurve& c = out.curves[k];
    const Use& u = uses[k];
    if(u.candidates.size() > 1 && u.file_pcurve >= 0 &&
       u.file_pcurve != c.pcurve &&
       !model_.edges[u.edge].pcurves[u.file_pcurve].computed) {
      ++report_.seams_reassigned;
    }
  }
  // close the gaps between consecutive curves
  double gap_tolerance = 10.0 * options_.tolerance;
  for(const Use& u : uses) {
    gap_tolerance = std::max(gap_tolerance, 2.0 * model_.edges[u.edge].tolerance);
  }
  std::vector<TrimCurve> closed;
  for(size_t k = 0; k < out.curves.size(); ++k) {
    closed.push_back(out.curves[k]);
    const Vec2 end = TrimPoint(model_, out.curves[k], 1.0);
    const Vec2 next = k + 1 < out.curves.size()
                        ? TrimPoint(model_, out.curves[k + 1], 0.0)
                        : Add2(out.start, out.wrap);
    const double scale = 1.0 + std::max(Norm2(end), Norm2(next));
    if(Norm2(Sub2(next, end)) > 1e-12 * scale) {
      closed.push_back(Segment(s, end, next, gap_tolerance));
      if(closed.back().kind == TrimCurve::Kind::kGap) {
        const int after =
          out.curves[k + 1 < out.curves.size() ? k + 1 : 0].edge;
        Warn("gap of " +
             std::to_string(Distance(OnSurface(s, end), OnSurface(s, next))) +
             " m in " + Ref("face", f.step_id) + " after " +
             Ref("edge", model_.edges[out.curves[k].edge].step_id) +
             ", before " + Ref("edge", model_.edges[after].step_id));
      }
    }
  }
  out.curves = closed;
  return true;
}

void Repairer::Shift(LoopUV& loop, const Vec2& shift) const {
  for(TrimCurve& c : loop.curves) {
    if(c.kind == TrimCurve::Kind::kEdge) {
      c.offset = Add2(c.offset, shift);
    } else {
      c.a = Add2(c.a, shift);
      c.b = Add2(c.b, shift);
    }
  }
  loop.start = Add2(loop.start, shift);
}

void Repairer::Reverse(LoopUV& loop) const {
  std::reverse(loop.curves.begin(), loop.curves.end());
  for(TrimCurve& c : loop.curves) {
    std::swap(c.s0, c.s1);
    std::swap(c.a, c.b);
  }
  loop.start = Add2(loop.start, loop.wrap);
  loop.wrap = {-loop.wrap[0], -loop.wrap[1]};
}

void Repairer::Close(LoopUV& loop, const Vec2& from, const Vec2& to,
                     TrimCurve::Kind kind) {
  TrimCurve c;
  c.kind = kind;
  c.a = from;
  c.b = to;
  loop.curves.push_back(c);
  if(kind == TrimCurve::Kind::kSeam) {
    ++report_.seam_segments;
  } else if(kind == TrimCurve::Kind::kSingular) {
    ++report_.singular_segments;
  }
}

static double LoopArea(const Model& model, const LoopUV& loop) {
  TrimLoop t;
  t.curves = loop.curves;
  return SignedArea(TrimPolygon(model, t));
}

static void Bounds(const Model& model, const LoopUV& loop, Vec2& lo,
                   Vec2& hi) {
  TrimLoop t;
  t.curves = loop.curves;
  lo = {1e300, 1e300};
  hi = {-1e300, -1e300};
  for(const Vec2& p : TrimPolygon(model, t, 8)) {
    for(int k = 0; k < 2; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  }
}

bool Repairer::Trim(Face& f) {
  const int s = f.surface;
  const Vec2 period = Period(s);
  const auto domain = SurfaceDomain(model_, surface(s));
  std::vector<LoopUV> wrapping, closed;
  std::vector<int> vertex_loops;
  for(const Loop& loop : f.loops) {
    if(loop.vertex >= 0) {
      vertex_loops.push_back(loop.vertex);
      continue;
    }
    LoopUV l;
    if(!Chain(f, loop, l)) {
      return false;
    }
    // the face lies to the left of its loops in (u, v) when its normal is
    // that of the surface
    if(!f.same_sense) {
      Reverse(l);
    }
    (l.wrap[0] != 0.0 || l.wrap[1] != 0.0 ? wrapping : closed).push_back(l);
  }
  for(const LoopUV& l : closed) {
    for(const TrimCurve& c : l.curves) {
      if(c.kind == TrimCurve::Kind::kSingular) {
        ++report_.singular_segments;
      } else if(c.kind == TrimCurve::Kind::kJoin) {
        ++report_.join_segments;
      } else if(c.kind == TrimCurve::Kind::kGap) {
        ++report_.gap_segments;
      }
    }
  }
  for(const LoopUV& l : wrapping) {
    for(const TrimCurve& c : l.curves) {
      if(c.kind == TrimCurve::Kind::kSingular) {
        ++report_.singular_segments;
      } else if(c.kind == TrimCurve::Kind::kJoin) {
        ++report_.join_segments;
      } else if(c.kind == TrimCurve::Kind::kGap) {
        ++report_.gap_segments;
      }
    }
  }
  // loops wrapping around a periodic surface are joined into one closed
  // loop
  if(wrapping.size() == 2) {
    LoopUV& a = wrapping[0];
    LoopUV b = wrapping[1];
    if(Norm2(Add2(a.wrap, b.wrap)) > 1e-9) {
      Warn(Ref("face", f.step_id) +
           " has two loops wrapping the same way around its surface");
      return false;
    }
    const int k = a.wrap[0] != 0.0 ? 0 : 1;
    const Vec2 a_end = Add2(a.start, a.wrap);
    Vec2 shift{0.0, 0.0};
    shift[k] = period[k] * std::round((a_end[k] - b.start[k]) / period[k]);
    Shift(b, shift);
    LoopUV joined;
    joined.curves = a.curves;
    Close(joined, a_end, b.start, TrimCurve::Kind::kSeam);
    joined.curves.insert(joined.curves.end(), b.curves.begin(),
                         b.curves.end());
    Close(joined, Add2(b.start, b.wrap), a.start, TrimCurve::Kind::kSeam);
    joined.start = a.start;
    closed.insert(closed.begin(), joined);
  } else if(wrapping.size() == 1) {
    LoopUV& a = wrapping[0];
    const int k = a.wrap[0] != 0.0 ? 0 : 1;
    const int other = 1 - k;
    const Vec2 a_end = Add2(a.start, a.wrap);
    double q = 0.0;
    bool found = false;
    if(vertex_loops.size() == 1) {
      // the apex of a cone, or the pole of a sphere, given as a vertex
      Vec2 uv;
      found = Project(s, model_.vertices[vertex_loops[0]].point, uv, &a.start);
      q = uv[other];
      vertex_loops.clear();
    } else if(vertex_loops.empty()) {
      // the face lies to the left of the loop: above it (greater v) if it
      // runs along increasing u, to the right of it (greater u) if it runs
      // along decreasing v
      const bool greater = (k == 0) == (a.wrap[k] > 0.0);
      const Surface& sf = surface(s);
      if(domain[other].bounded && !domain[other].periodic) {
        q = greater ? domain[other].hi : domain[other].lo;
        found = true;
      } else if(sf.type == Surface::Type::kCone && other == 1) {
        q = -sf.radius / std::tan(sf.semi_angle);
        found = greater == (q > a.start[1]);
      }
    }
    if(!found) {
      Warn(Ref("face", f.step_id) +
           " has a loop wrapping around its surface with nothing to close it");
      return false;
    }
    Vec2 p1 = a_end, p2 = a.start;
    p1[other] = q;
    p2[other] = q;
    const double singular =
      std::max(Distance(OnSurface(s, p1), OnSurface(s, p2)),
               Distance(OnSurface(s, p1), OnSurface(s, Lerp2(p1, p2, 0.5))));
    if(singular > 10.0 * options_.tolerance) {
      Warn(Ref("face", f.step_id) + " closed along a line that is not a "
                                    "singular point of its surface");
    }
    LoopUV joined;
    joined.curves = a.curves;
    Close(joined, a_end, p1, TrimCurve::Kind::kSeam);
    Close(joined, p1, p2, TrimCurve::Kind::kSingular);
    Close(joined, p2, a.start, TrimCurve::Kind::kSeam);
    joined.start = a.start;
    closed.insert(closed.begin(), joined);
  } else if(wrapping.size() > 2) {
    Warn(Ref("face", f.step_id) + " has more than two loops wrapping around "
                                  "its surface");
    return false;
  }
  if(!vertex_loops.empty() && !closed.empty()) {
    Warn(Ref("face", f.step_id) + " has a vertex loop that bounds nothing");
  }
  // outer loops run counterclockwise, holes clockwise
  std::vector<double> areas;
  bool any_outer = false;
  for(const LoopUV& l : closed) {
    areas.push_back(LoopArea(model_, l));
    any_outer = any_outer || areas.back() > 0.0;
  }
  if(!any_outer) {
    if(domain[0].bounded && domain[1].bounded) {
      // the face is the whole surface, less any holes
      LoopUV whole;
      const Vec2 p0{domain[0].lo, domain[1].lo}, p1{domain[0].hi, domain[1].lo};
      const Vec2 p2{domain[0].hi, domain[1].hi}, p3{domain[0].lo, domain[1].hi};
      Close(whole, p0, p1, TrimCurve::Kind::kSeam);
      Close(whole, p1, p2, TrimCurve::Kind::kSeam);
      Close(whole, p2, p3, TrimCurve::Kind::kSeam);
      Close(whole, p3, p0, TrimCurve::Kind::kSeam);
      whole.start = p0;
      closed.insert(closed.begin(), whole);
      areas.insert(areas.begin(), 1.0);
      ++report_.domain_outer_loops;
    } else if(!closed.empty()) {
      Warn(Ref("face", f.step_id) + " has no counterclockwise loop: loops "
                                    "reversed");
      for(size_t i = 0; i < closed.size(); ++i) {
        Reverse(closed[i]);
        areas[i] = -areas[i];
      }
    } else {
      Warn(Ref("face", f.step_id) + " has no loops on an unbounded surface");
      return false;
    }
  }
  // the outer loop with the largest area first, in the first period of the
  // domain; holes in the same period as it
  size_t outer = 0;
  for(size_t i = 1; i < closed.size(); ++i) {
    if(areas[i] > areas[outer]) {
      outer = i;
    }
  }
  std::swap(closed[0], closed[outer]);
  std::swap(areas[0], areas[outer]);
  Vec2 lo, hi;
  Bounds(model_, closed[0], lo, hi);
  Vec2 base{0.0, 0.0};
  for(int k = 0; k < 2; ++k) {
    if(period[k] > 0.0) {
      base[k] = -period[k] *
                std::floor((lo[k] - domain[k].lo) / period[k] + 1e-9);
    }
  }
  Shift(closed[0], base);
  lo = Add2(lo, base);
  hi = Add2(hi, base);
  const Vec2 centre{0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])};
  for(size_t i = 1; i < closed.size(); ++i) {
    Vec2 l, h;
    Bounds(model_, closed[i], l, h);
    Shift(closed[i],
          PeriodShift(period, {0.5 * (l[0] + h[0]), 0.5 * (l[1] + h[1])},
                      centre));
    if(areas[i] > 0.0) {
      Warn(Ref("face", f.step_id) + " has more than one outer loop");
    }
  }
  f.trims.clear();
  for(size_t i = 0; i < closed.size(); ++i) {
    TrimLoop t;
    t.curves = closed[i].curves;
    t.outer = areas[i] > 0.0;
    f.trims.push_back(t);
  }
  return true;
}

RepairReport Repairer::Run() {
  report_.edges = static_cast<int>(model_.edges.size());
  bad_.resize(model_.edges.size());
  for(size_t ei = 0; ei < model_.edges.size(); ++ei) {
    Edge& e = model_.edges[ei];
    bad_[ei].assign(e.pcurves.size(), false);
    e.tolerance = 0.0;
    e.has_range = false;
    const bool ok = e.curve >= 0 ? CurveRange(e)
                                 : !e.pcurves.empty() && PcurveOnlyRange(e);
    if(!ok) {
      ++report_.edges_failed;
      Warn(Ref("no parameter range for edge", e.step_id));
      continue;
    }
    for(size_t i = e.curve >= 0 ? 0 : 1; i < e.pcurves.size(); ++i) {
      ++report_.pcurves_checked;
      if(!Check(e, static_cast<int>(i))) {
        ++report_.pcurves_replaced;
        ++report_.pcurves_computed;
        Pcurve pc;
        if(Compute(e, e.pcurves[i].surface, pc)) {
          e.pcurves[i] = pc;
        } else {
          ++report_.pcurves_failed;
          bad_[ei][i] = true;
        }
      }
    }
  }
  report_.faces = static_cast<int>(model_.faces.size());
  for(Face& f : model_.faces) {
    EnsurePcurves(f);
    f.trims.clear();
    if(Trim(f)) {
      ++report_.faces_trimmed;
      // the pcurve used by each coedge
      for(Loop& loop : f.loops) {
        for(Coedge& c : loop.coedges) {
          for(const TrimLoop& t : f.trims) {
            for(const TrimCurve& tc : t.curves) {
              if(tc.kind == TrimCurve::Kind::kEdge && tc.edge == c.edge) {
                const Edge& e = model_.edges[c.edge];
                const bool forward = (c.orientation == loop.orientation) ==
                                     f.same_sense;
                const Pcurve& pc = e.pcurves[tc.pcurve];
                if((tc.s0 == pc.range[0]) == forward ||
                   pc.range[0] == pc.range[1]) {
                  c.pcurve = tc.pcurve;
                }
              }
            }
          }
        }
      }
    }
  }
  return report_;
}

RepairReport Repair(Model& model, const RepairOptions& options) {
  Repairer r(model, options);
  return r.Run();
}

} // namespace cad
} // namespace core
} // namespace shift
