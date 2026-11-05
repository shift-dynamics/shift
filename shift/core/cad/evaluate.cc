#include "shift/core/cad/evaluate.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace shift {
namespace core {
namespace cad {

static constexpr double kTwoPi = 2.0 * M_PI;

static double Dot3(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static Vec3 Sub3(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

static Vec3 Cross3(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

double Distance(const Vec3& a, const Vec3& b) {
  Vec3 d = Sub3(a, b);
  return std::sqrt(Dot3(d, d));
}

/* the value of a periodic parameter x (period 2 pi) nearest to guess */
static double Nearest(double x, double guess) {
  return x + kTwoPi * std::round((guess - x) / kTwoPi);
}

/* span of t in a full knot vector: the index s with knots[s] <= t <
knots[s + 1], from degree to count - 1 */
static int FindSpan(const std::vector<double>& knots, int degree, int count,
                    double t) {
  if(t >= knots[count]) {
    int s = count - 1;
    while(s > degree && knots[s] == knots[s + 1]) {
      --s;
    }
    return s;
  }
  if(t <= knots[degree]) {
    int s = degree;
    while(s < count - 1 && knots[s + 1] == knots[s]) {
      ++s;
    }
    return s;
  }
  int lo = degree;
  int hi = count;
  int mid = (lo + hi) / 2;
  while(t < knots[mid] || t >= knots[mid + 1]) {
    if(t < knots[mid]) {
      hi = mid;
    } else {
      lo = mid;
    }
    mid = (lo + hi) / 2;
  }
  return mid;
}

/* the degree + 1 basis functions non-zero at t in span, and their first
derivatives (Piegl and Tiller, The NURBS Book, A2.2 and A2.3) */
static void Basis(const std::vector<double>& knots, int degree, int span,
                  double t, std::vector<double>& n, std::vector<double>& dn) {
  const int p = degree;
  std::vector<std::vector<double>> ndu(p + 1, std::vector<double>(p + 1));
  std::vector<double> left(p + 1), right(p + 1);
  ndu[0][0] = 1.0;
  for(int j = 1; j <= p; ++j) {
    left[j] = t - knots[span + 1 - j];
    right[j] = knots[span + j] - t;
    double saved = 0.0;
    for(int r = 0; r < j; ++r) {
      ndu[j][r] = right[r + 1] + left[j - r];
      double temp = ndu[j][r] != 0.0 ? ndu[r][j - 1] / ndu[j][r] : 0.0;
      ndu[r][j] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    ndu[j][j] = saved;
  }
  n.assign(p + 1, 0.0);
  dn.assign(p + 1, 0.0);
  for(int r = 0; r <= p; ++r) {
    n[r] = ndu[r][p];
  }
  if(p == 0) {
    return;
  }
  // N'(i, p) = p N(i, p-1) / (u(i+p) - u(i)) - p N(i+1, p-1) / (u(i+p+1) -
  // u(i+1)), with i = span - p + r; the degree p - 1 functions non-zero in
  // the span are ndu[0..p-1][p-1], for i = span - p + 1 ...
  for(int r = 0; r <= p; ++r) {
    const int i = span - p + r;
    double d = 0.0;
    if(r >= 1) {
      double den = knots[i + p] - knots[i];
      if(den != 0.0) {
        d += ndu[r - 1][p - 1] / den;
      }
    }
    if(r <= p - 1) {
      double den = knots[i + p + 1] - knots[i + 1];
      if(den != 0.0) {
        d -= ndu[r][p - 1] / den;
      }
    }
    dn[r] = p * d;
  }
}

template <int D>
static std::array<double, D> NurbsPoint(const NurbsCurve<D>& c, double t,
                                        std::array<double, D>* derivative) {
  const int count = static_cast<int>(c.points.size());
  t = std::clamp(t, c.knots[c.degree], c.knots[count]);
  const int span = FindSpan(c.knots, c.degree, count, t);
  std::vector<double> n, dn;
  Basis(c.knots, c.degree, span, t, n, dn);
  std::array<double, D> a{}, da{};
  double w = 0.0, dw = 0.0;
  for(int r = 0; r <= c.degree; ++r) {
    const int i = span - c.degree + r;
    const double wi = c.rational() ? c.weights[i] : 1.0;
    for(int k = 0; k < D; ++k) {
      a[k] += n[r] * wi * c.points[i][k];
      da[k] += dn[r] * wi * c.points[i][k];
    }
    w += n[r] * wi;
    dw += dn[r] * wi;
  }
  std::array<double, D> p{};
  for(int k = 0; k < D; ++k) {
    p[k] = a[k] / w;
  }
  if(derivative) {
    for(int k = 0; k < D; ++k) {
      (*derivative)[k] = (da[k] - dw * p[k]) / w;
    }
  }
  return p;
}

template <int D>
std::array<double, D> CurvePoint(const Curve<D>& curve, double t,
                                 std::array<double, D>* derivative) {
  using Type = typename Curve<D>::Type;
  std::array<double, D> p{};
  switch(curve.type) {
  case Type::kLine:
    for(int k = 0; k < D; ++k) {
      p[k] = curve.origin[k] + t * curve.direction[k];
    }
    if(derivative) {
      *derivative = curve.direction;
    }
    return p;
  case Type::kConic: {
    const double c = std::cos(t), s = std::sin(t);
    for(int k = 0; k < D; ++k) {
      p[k] = curve.origin[k] + c * curve.a[k] + s * curve.b[k];
    }
    if(derivative) {
      for(int k = 0; k < D; ++k) {
        (*derivative)[k] = -s * curve.a[k] + c * curve.b[k];
      }
    }
    return p;
  }
  case Type::kNurbs:
    return NurbsPoint<D>(curve.nurbs, t, derivative);
  }
  return p;
}

template <int D>
Interval CurveDomain(const Curve<D>& curve) {
  using Type = typename Curve<D>::Type;
  Interval i;
  if(curve.type == Type::kConic) {
    i.lo = 0.0;
    i.hi = kTwoPi;
    i.bounded = true;
    i.periodic = true;
  } else if(curve.type == Type::kNurbs) {
    i.lo = curve.nurbs.knots[curve.nurbs.degree];
    i.hi = curve.nurbs.knots[curve.nurbs.points.size()];
    i.bounded = true;
  }
  return i;
}

template std::array<double, 2> CurvePoint<2>(const Curve<2>&, double,
                                             std::array<double, 2>*);
template std::array<double, 3> CurvePoint<3>(const Curve<3>&, double,
                                             std::array<double, 3>*);
template Interval CurveDomain<2>(const Curve<2>&);
template Interval CurveDomain<3>(const Curve<3>&);

static Vec3 NurbsSurfacePoint(const NurbsSurface& s, double u, double v,
                              Vec3* du, Vec3* dv) {
  u = std::clamp(u, s.u_knots[s.u_degree], s.u_knots[s.u_count]);
  v = std::clamp(v, s.v_knots[s.v_degree], s.v_knots[s.v_count]);
  const int su = FindSpan(s.u_knots, s.u_degree, s.u_count, u);
  const int sv = FindSpan(s.v_knots, s.v_degree, s.v_count, v);
  std::vector<double> nu, dnu, nv, dnv;
  Basis(s.u_knots, s.u_degree, su, u, nu, dnu);
  Basis(s.v_knots, s.v_degree, sv, v, nv, dnv);
  Vec3 a{}, au{}, av{};
  double w = 0.0, wu = 0.0, wv = 0.0;
  for(int r = 0; r <= s.u_degree; ++r) {
    const int i = su - s.u_degree + r;
    for(int q = 0; q <= s.v_degree; ++q) {
      const int j = sv - s.v_degree + q;
      const int k = i * s.v_count + j;
      const double wk = s.rational() ? s.weights[k] : 1.0;
      const double b = nu[r] * nv[q] * wk;
      const double bu = dnu[r] * nv[q] * wk;
      const double bv = nu[r] * dnv[q] * wk;
      for(int d = 0; d < 3; ++d) {
        a[d] += b * s.points[k][d];
        au[d] += bu * s.points[k][d];
        av[d] += bv * s.points[k][d];
      }
      w += b;
      wu += bu;
      wv += bv;
    }
  }
  Vec3 p{a[0] / w, a[1] / w, a[2] / w};
  if(du) {
    for(int d = 0; d < 3; ++d) {
      (*du)[d] = (au[d] - wu * p[d]) / w;
    }
  }
  if(dv) {
    for(int d = 0; d < 3; ++d) {
      (*dv)[d] = (av[d] - wv * p[d]) / w;
    }
  }
  return p;
}

/* a point given in a frame's coordinates, in its parent's */
static Vec3 FromFrame(const Frame& f, double x, double y, double z) {
  Vec3 p;
  for(int i = 0; i < 3; ++i) {
    p[i] = f.origin[i] + x * f.x[i] + y * f.y[i] + z * f.z[i];
  }
  return p;
}

static Vec3 VectorFromFrame(const Frame& f, double x, double y, double z) {
  Vec3 p;
  for(int i = 0; i < 3; ++i) {
    p[i] = x * f.x[i] + y * f.y[i] + z * f.z[i];
  }
  return p;
}

/* a vector r turned by the angle u about the unit axis z */
static Vec3 Turn(const Vec3& r, const Vec3& z, double u) {
  const double along = Dot3(r, z);
  Vec3 perp{r[0] - along * z[0], r[1] - along * z[1], r[2] - along * z[2]};
  Vec3 side = Cross3(z, perp);
  const double c = std::cos(u), s = std::sin(u);
  return {along * z[0] + c * perp[0] + s * side[0],
          along * z[1] + c * perp[1] + s * side[1],
          along * z[2] + c * perp[2] + s * side[2]};
}

Vec3 SurfacePoint(const Model& model, const Surface& s, double u, double v,
                  Vec3* du, Vec3* dv) {
  const Frame& f = s.position;
  const double cu = std::cos(u), su = std::sin(u);
  switch(s.type) {
  case Surface::Type::kPlane:
    if(du) {
      *du = f.x;
    }
    if(dv) {
      *dv = f.y;
    }
    return FromFrame(f, u, v, 0.0);
  case Surface::Type::kCylinder:
    if(du) {
      *du = VectorFromFrame(f, -s.radius * su, s.radius * cu, 0.0);
    }
    if(dv) {
      *dv = f.z;
    }
    return FromFrame(f, s.radius * cu, s.radius * su, v);
  case Surface::Type::kCone: {
    const double t = std::tan(s.semi_angle);
    const double r = s.radius + v * t;
    if(du) {
      *du = VectorFromFrame(f, -r * su, r * cu, 0.0);
    }
    if(dv) {
      *dv = VectorFromFrame(f, t * cu, t * su, 1.0);
    }
    return FromFrame(f, r * cu, r * su, v);
  }
  case Surface::Type::kSphere: {
    const double cv = std::cos(v), sv = std::sin(v);
    const double R = s.radius;
    if(du) {
      *du = VectorFromFrame(f, -R * cv * su, R * cv * cu, 0.0);
    }
    if(dv) {
      *dv = VectorFromFrame(f, -R * sv * cu, -R * sv * su, R * cv);
    }
    return FromFrame(f, R * cv * cu, R * cv * su, R * sv);
  }
  case Surface::Type::kTorus: {
    const double cv = std::cos(v), sv = std::sin(v);
    const double r = s.radius + s.minor_radius * cv;
    if(du) {
      *du = VectorFromFrame(f, -r * su, r * cu, 0.0);
    }
    if(dv) {
      *dv = VectorFromFrame(f, -s.minor_radius * sv * cu,
                            -s.minor_radius * sv * su, s.minor_radius * cv);
    }
    return FromFrame(f, r * cu, r * su, s.minor_radius * sv);
  }
  case Surface::Type::kNurbs:
    return NurbsSurfacePoint(s.nurbs, u, v, du, dv);
  case Surface::Type::kRevolution: {
    const Curve3& c = model.curves[s.curve];
    Vec3 dc;
    Vec3 q = CurvePoint<3>(c, v, &dc);
    Vec3 r = Sub3(q, f.origin);
    Vec3 p = Turn(r, f.z, u);
    if(du) {
      *du = Cross3(f.z, p);
    }
    if(dv) {
      *dv = Turn(dc, f.z, u);
    }
    return {f.origin[0] + p[0], f.origin[1] + p[1], f.origin[2] + p[2]};
  }
  case Surface::Type::kExtrusion: {
    const Curve3& c = model.curves[s.curve];
    Vec3 dc;
    Vec3 q = CurvePoint<3>(c, u, &dc);
    if(du) {
      *du = dc;
    }
    if(dv) {
      *dv = s.extrusion;
    }
    return {q[0] + v * s.extrusion[0], q[1] + v * s.extrusion[1],
            q[2] + v * s.extrusion[2]};
  }
  }
  return {0.0, 0.0, 0.0};
}

std::array<Interval, 2> SurfaceDomain(const Model& model, const Surface& s) {
  std::array<Interval, 2> d;
  Interval angle{0.0, kTwoPi, true, true};
  switch(s.type) {
  case Surface::Type::kPlane:
    break;
  case Surface::Type::kCylinder:
  case Surface::Type::kCone:
    d[0] = angle;
    break;
  case Surface::Type::kSphere:
    d[0] = angle;
    d[1] = {-M_PI / 2.0, M_PI / 2.0, true, false};
    break;
  case Surface::Type::kTorus:
    d[0] = angle;
    d[1] = angle;
    break;
  case Surface::Type::kNurbs:
    d[0] = {s.nurbs.u_knots[s.nurbs.u_degree], s.nurbs.u_knots[s.nurbs.u_count],
            true, false};
    d[1] = {s.nurbs.v_knots[s.nurbs.v_degree], s.nurbs.v_knots[s.nurbs.v_count],
            true, false};
    break;
  case Surface::Type::kRevolution:
    d[0] = angle;
    d[1] = CurveDomain<3>(model.curves[s.curve]);
    break;
  case Surface::Type::kExtrusion:
    d[0] = CurveDomain<3>(model.curves[s.curve]);
    break;
  }
  return d;
}

bool ProjectToCurve(const Curve3& curve, const Vec3& p, double& t,
                    const double* guess) {
  switch(curve.type) {
  case Curve3::Type::kLine: {
    const double dd = Dot3(curve.direction, curve.direction);
    if(dd == 0.0) {
      return false;
    }
    t = Dot3(Sub3(p, curve.origin), curve.direction) / dd;
    return true;
  }
  case Curve3::Type::kConic: {
    const Vec3 q = Sub3(p, curve.origin);
    const double aa = Dot3(curve.a, curve.a), bb = Dot3(curve.b, curve.b);
    if(aa == 0.0 || bb == 0.0) {
      return false;
    }
    t = std::atan2(Dot3(q, curve.b) / bb, Dot3(q, curve.a) / aa);
    if(std::fabs(aa - bb) > 1e-12 * aa) {
      // an ellipse: refine the angle
      for(int i = 0; i < 30; ++i) {
        Vec3 d;
        Vec3 c = CurvePoint<3>(curve, t, &d);
        Vec3 r = Sub3(c, p);
        Vec3 dd{-(c[0] - curve.origin[0]), -(c[1] - curve.origin[1]),
                -(c[2] - curve.origin[2])};
        double f = Dot3(r, d);
        double df = Dot3(d, d) + Dot3(r, dd);
        if(df == 0.0) {
          break;
        }
        double step = f / df;
        t -= step;
        if(std::fabs(step) < 1e-15) {
          break;
        }
      }
    }
    t = guess ? Nearest(t, *guess) : t - kTwoPi * std::floor(t / kTwoPi);
    return true;
  }
  case Curve3::Type::kNurbs: {
    const auto& n = curve.nurbs;
    const Interval d = CurveDomain<3>(curve);
    double best = d.lo;
    if(guess) {
      best = std::clamp(*guess, d.lo, d.hi);
    } else {
      const int samples =
        8 * (n.degree + 1) * std::max<int>(1, n.points.size() - n.degree);
      double best_d = 1e300;
      for(int i = 0; i <= samples; ++i) {
        double s = d.lo + (d.hi - d.lo) * i / samples;
        double dist = Distance(CurvePoint<3>(curve, s), p);
        if(dist < best_d) {
          best_d = dist;
          best = s;
        }
      }
    }
    t = best;
    for(int i = 0; i < 50; ++i) {
      Vec3 dc;
      Vec3 c = CurvePoint<3>(curve, t, &dc);
      double f = Dot3(Sub3(c, p), dc);
      double df = Dot3(dc, dc);
      if(df == 0.0) {
        break;
      }
      double next = std::clamp(t - f / df, d.lo, d.hi);
      double step = next - t;
      t = next;
      if(std::fabs(step) < 1e-15 * std::max(1.0, d.hi - d.lo)) {
        break;
      }
    }
    return true;
  }
  }
  return false;
}

/* Gauss-Newton on the distance from p, from uv, keeping bounded parameters
in their domains */
static bool NewtonProject(const Model& model, const Surface& s,
                          const std::array<Interval, 2>& domain,
                          const Vec3& p, Vec2& uv) {
  for(int it = 0; it < 60; ++it) {
    Vec3 du, dv;
    Vec3 q = SurfacePoint(model, s, uv[0], uv[1], &du, &dv);
    Vec3 r = Sub3(q, p);
    const double a = Dot3(du, du), b = Dot3(du, dv), c = Dot3(dv, dv);
    const double g0 = Dot3(du, r), g1 = Dot3(dv, r);
    const double det = a * c - b * b;
    double x, y;
    if(std::fabs(det) > 1e-300) {
      x = -(c * g0 - b * g1) / det;
      y = -(a * g1 - b * g0) / det;
    } else if(a > 0.0) {
      x = -g0 / a;
      y = 0.0;
    } else if(c > 0.0) {
      x = 0.0;
      y = -g1 / c;
    } else {
      return false;
    }
    Vec2 next{uv[0] + x, uv[1] + y};
    for(int k = 0; k < 2; ++k) {
      if(domain[k].bounded && !domain[k].periodic) {
        next[k] = std::clamp(next[k], domain[k].lo, domain[k].hi);
      }
    }
    const double step = std::hypot(next[0] - uv[0], next[1] - uv[1]);
    uv = next;
    if(step < 1e-14 * (1.0 + std::hypot(uv[0], uv[1]))) {
      break;
    }
  }
  return true;
}

bool ProjectToSurface(const Model& model, const Surface& s, const Vec3& p,
                      Vec2& uv, const Vec2* guess) {
  const Frame& f = s.position;
  const Vec3 r = Sub3(p, f.origin);
  const double x = Dot3(r, f.x), y = Dot3(r, f.y), z = Dot3(r, f.z);
  const double rho = std::hypot(x, y);
  auto angle = [&](double singular_scale) {
    if(rho <= 1e-12 * singular_scale && guess) {
      return (*guess)[0];
    }
    double u = std::atan2(y, x);
    return guess ? Nearest(u, (*guess)[0]) : u - kTwoPi * std::floor(u / kTwoPi);
  };
  switch(s.type) {
  case Surface::Type::kPlane:
    uv = {x, y};
    return true;
  case Surface::Type::kCylinder:
    uv = {angle(s.radius), z};
    return true;
  case Surface::Type::kCone: {
    const double t = std::tan(s.semi_angle);
    const double v = ((rho - s.radius) * t + z) / (1.0 + t * t);
    const double scale = std::max(std::fabs(s.radius), std::fabs(v * t));
    const double radius = s.radius + v * t;
    uv = {std::fabs(radius) <= 1e-12 * std::max(scale, 1e-300) && guess
            ? (*guess)[0]
            : angle(scale),
          v};
    return true;
  }
  case Surface::Type::kSphere:
    uv = {angle(s.radius), std::atan2(z, rho)};
    return true;
  case Surface::Type::kTorus: {
    double v = std::atan2(z, rho - s.radius);
    if(guess) {
      v = Nearest(v, (*guess)[1]);
    } else {
      v -= kTwoPi * std::floor(v / kTwoPi);
    }
    uv = {angle(s.radius), v};
    return true;
  }
  default:
    break;
  }
  // NURBS, revolution, extrusion: from the guess, or from the nearest points
  // of a grid (several, since a surface folding back near itself has local
  // minima of the distance)
  const auto domain = SurfaceDomain(model, s);
  if(guess) {
    uv = *guess;
    return NewtonProject(model, s, domain, p, uv);
  }
  Interval d0 = domain[0], d1 = domain[1];
  if(!d0.bounded) {
    d0 = {-1.0, 1.0, true, false};
  }
  if(!d1.bounded) {
    d1 = {-1.0, 1.0, true, false};
  }
  int nu = 24, nv = 24;
  if(s.type == Surface::Type::kNurbs) {
    nu = std::clamp(3 * s.nurbs.u_count, 16, 96);
    nv = std::clamp(3 * s.nurbs.v_count, 16, 96);
  }
  std::vector<std::pair<double, Vec2>> grid;
  grid.reserve((nu + 1) * (nv + 1));
  for(int i = 0; i <= nu; ++i) {
    for(int j = 0; j <= nv; ++j) {
      Vec2 g{d0.lo + (d0.hi - d0.lo) * i / nu, d1.lo + (d1.hi - d1.lo) * j / nv};
      grid.push_back({Distance(SurfacePoint(model, s, g[0], g[1]), p), g});
    }
  }
  const size_t starts = std::min<size_t>(6, grid.size());
  std::partial_sort(grid.begin(), grid.begin() + starts, grid.end(),
                    [](const auto& a, const auto& b) {
                      return a.first < b.first;
                    });
  double best = 1e300;
  bool found = false;
  for(size_t k = 0; k < starts; ++k) {
    Vec2 x = grid[k].second;
    if(!NewtonProject(model, s, domain, p, x)) {
      continue;
    }
    const double dist = Distance(SurfacePoint(model, s, x[0], x[1]), p);
    if(dist < best) {
      best = dist;
      uv = x;
      found = true;
    }
  }
  return found;
}

} // namespace cad
} // namespace core
} // namespace shift
