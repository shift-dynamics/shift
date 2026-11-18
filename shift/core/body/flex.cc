#include "shift/core/body/flex.h"

#include <algorithm>
#include <sstream>

#include "ksl/matrix.h"
#include "ksl/vec3.h"

#include "shift/core/exception.h"
#include "shift/core/log.h"

/* a A + s (a~ b~), with a~ b~ = b a^T - (a . b) I; at[c][r] is row r of
column c */
static void AddSkewSkew(double s, const ksl_vec3_t& a, const ksl_vec3_t& b,
                        ksl_mat3x3_t* A) {
  const double ab = ksl_dot_vv(&a, &b);
  for(int c = 0; c < 3; c++) {
    for(int r = 0; r < 3; r++) {
      A->at[c][r] += s * (b.at[r] * a.at[c] - (r == c ? ab : 0.0));
    }
  }
}

static void AddScaled(double s, const ksl_mat3x3_t& B, ksl_mat3x3_t* A) {
  for(int c = 0; c < 3; c++) {
    for(int r = 0; r < 3; r++) {
      A->at[c][r] += s * B.at[c][r];
    }
  }
}

static void AddScaledVector(double s, const ksl_vec3_t& b, ksl_vec3_t* a) {
  a->x += s * b.x;
  a->y += s * b.y;
  a->z += s * b.z;
}

void shift::core::body::Flex::SetModel(
  const std::vector<ksl_vec3_t>& positions, const std::vector<double>& masses,
  const std::vector<ksl_mat3x3_t>& rotary_inertias,
  unsigned int elastic_count, const std::vector<ksl_screw_t>& shapes,
  const std::vector<double>& stiffness, const std::vector<double>& damping) {
  const size_t n = elastic_count;
  if(positions.size() != masses.size() ||
     rotary_inertias.size() != masses.size() ||
     shapes.size() != masses.size() * n || stiffness.size() != n * n ||
     !(damping.empty() || damping.size() == n * n)) {
    throw shift::core::Exception(
      "The model of flexible body '" + get_name() +
      "' has arrays of inconsistent sizes");
  }
  elastic_count_ = elastic_count;
  positions_ = positions;
  masses_ = masses;
  rotary_inertias_ = rotary_inertias;
  shapes_ = shapes;
  stiffness_ = stiffness;
  damping_ = damping.empty() ? std::vector<double>(n * n, 0.0) : damping;
  initial_eta_.assign(n, 0.0);
  initial_eta_dot_.assign(n, 0.0);
}

void shift::core::body::Flex::SetNode(const std::string& frame_name,
                                      unsigned int point) {
  node_names_.push_back(frame_name);
  node_points_.push_back(point);
  node_sliding_.push_back(-1);
}

void shift::core::body::Flex::SetSlidingNode(
  const std::string& frame_name, const std::vector<unsigned int>& stations,
  double first, double spacing, const ksl_vec3_t& axis,
  const shift::core::body::SlidingBearing& bearing) {
  if(stations.empty() || (stations.size() > 1 && !(spacing > 0.0))) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': sliding node '" + frame_name +
                                 "' needs stations a positive spacing apart");
  }
  SlidingNode node;
  node.stations = stations;
  node.first = first;
  node.spacing = spacing;
  node.axis = axis;
  ksl_vec3_normalize(&node.axis);
  node.bearing = bearing;
  node.center = BearingCenter(bearing);
  node_names_.push_back(frame_name);
  node_points_.push_back(stations[0]);
  node_sliding_.push_back(static_cast<int>(sliding_.size()));
  sliding_.push_back(node);
}

std::vector<double>
shift::core::body::Flex::SplineCurvatures(const std::vector<double>& y,
                                          double h) {
  const size_t count = y.size();
  std::vector<double> m(count, 0.0);
  if(count < 3) {
    return m;
  }
  /* M_i-1 + 4 M_i + M_i+1 = 6 (y_i+1 - 2 y_i + y_i-1) / h^2 over the
  interior rows 1 .. N-2, by the Thomas algorithm */
  std::vector<double> diagonal(count), right(count);
  for(size_t i = 1; i + 1 < count; i++) {
    diagonal[i] = 4.0;
    right[i] = 6.0 * (y[i + 1] - 2.0 * y[i] + y[i - 1]) / (h * h);
    if(i > 1) {
      const double factor = 1.0 / diagonal[i - 1];
      diagonal[i] -= factor;
      right[i] -= factor * right[i - 1];
    }
  }
  for(size_t i = count - 2; i >= 1; i--) {
    m[i] = (right[i] - m[i + 1]) / diagonal[i];
  }
  return m;
}

double shift::core::body::Flex::BearingCenter(
  const shift::core::body::SlidingBearing& bearing) {
  double length = 0.0, moment = 0.0;
  for(const auto& interval : bearing.intervals) {
    length += interval[1] - interval[0];
    moment += 0.5 * (interval[1] * interval[1] - interval[0] * interval[0]);
  }
  return length > 0.0 ? moment / length : 0.0;
}

std::vector<double> shift::core::body::Flex::SlidingMap(
  size_t count, double first, double spacing, const ksl_vec3_t& axis,
  double s, const shift::core::body::SlidingBearing& bearing,
  int derivative) {
  const size_t n = 6 * count;
  std::vector<double> map(6 * n, 0.0);
  if(count == 0) {
    return map;
  }
  SlidingNode node;
  node.stations.assign(count, 0);
  node.first = first;
  node.spacing = spacing;
  node.axis = axis;
  ksl_vec3_normalize(&node.axis);
  node.bearing = bearing;
  node.center = BearingCenter(bearing);
  const Carried carried = Carry(node, s);
  const size_t k = size_t(std::clamp(derivative, 0, 2));
  std::vector<double> unit(count, 0.0);
  std::vector<ksl_screw_t> y(count), m(count);
  for(size_t i = 0; i < count; i++) {
    unit[i] = 1.0;
    const std::vector<double> curvature = SplineCurvatures(unit, spacing);
    unit[i] = 0.0;
    for(int c = 0; c < 6; c++) {
      for(size_t j = 0; j < count; j++) {
        y[j] = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        m[j] = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        m[j].at[c] = curvature[j];
      }
      y[i].at[c] = 1.0;
      ksl_screw_t out[3];
      CarryShape(node, carried, node.axis, y, m, out);
      for(int r = 0; r < 6; r++) {
        map[size_t(r) * n + 6 * i + size_t(c)] = out[k].at[r];
      }
    }
  }
  return map;
}

void shift::core::body::Flex::AddValue(const SlidingNode& node, double x,
                                       double scale, std::vector<double>* y,
                                       std::vector<double>* m) {
  const SplineWeights w = Weights(node, x);
  (*y)[w.i] += scale * w.c0;
  (*y)[w.i1] += scale * w.c1;
  (*m)[w.i] += scale * w.e0;
  (*m)[w.i1] += scale * w.e1;
}

void shift::core::body::Flex::AddSlope(const SlidingNode& node, double x,
                                       double scale, std::vector<double>* y,
                                       std::vector<double>* m) {
  if(node.stations.size() < 2) {
    return;
  }
  const SplineWeights w = Weights(node, x);
  (*y)[w.i] += scale * w.d0;
  (*y)[w.i1] += scale * w.d1;
  (*m)[w.i] += scale * w.f0;
  (*m)[w.i1] += scale * w.f1;
}

void shift::core::body::Flex::AddIntegral(const SlidingNode& node, double x,
                                          double scale,
                                          std::vector<double>* y,
                                          std::vector<double>* m) {
  const size_t count = node.stations.size();
  const double delta_0 = x - node.first;
  if(count < 2) {
    (*y)[0] += scale * delta_0;
    return;
  }
  const double h = node.spacing;
  const double last = node.first + h * double(count - 1);
  /* over a whole segment: h (Y_k + Y_k+1) / 2 - h^3 (M_k + M_k+1) / 24 */
  auto whole = [&](size_t k) {
    (*y)[k] += scale * 0.5 * h;
    (*y)[k + 1] += scale * 0.5 * h;
    (*m)[k] -= scale * h * h * h / 24.0;
    (*m)[k + 1] -= scale * h * h * h / 24.0;
  };
  if(x < node.first) {
    /* delta Y_0 + delta^2 / 2 Y'(x_0) */
    (*y)[0] += scale * delta_0;
    AddSlope(node, node.first, scale * 0.5 * delta_0 * delta_0, y, m);
    return;
  }
  if(x > last) {
    for(size_t k = 0; k + 1 < count; k++) {
      whole(k);
    }
    const double delta = x - last;
    (*y)[count - 1] += scale * delta;
    AddSlope(node, last, scale * 0.5 * delta * delta, y, m);
    return;
  }
  const size_t i = std::min(size_t(delta_0 / h), count - 2);
  for(size_t k = 0; k < i; k++) {
    whole(k);
  }
  /* h int_0^t of r Y_i + t Y_i+1 + h^2 / 6 ((r^3 - r) M_i + (t^3 - t)
  M_i+1), r = 1 - t */
  const double t = (delta_0 - h * double(i)) / h;
  const double r = 1.0 - t;
  (*y)[i] += scale * h * (t - 0.5 * t * t);
  (*y)[i + 1] += scale * h * 0.5 * t * t;
  (*m)[i] += scale * h * h * h / 6.0 *
             (0.5 * r * r - 0.25 * r * r * r * r - 0.25);
  (*m)[i + 1] += scale * h * h * h / 6.0 * (0.25 * t * t * t * t - 0.5 * t * t);
}

void shift::core::body::Flex::AddMoment(const SlidingNode& node, double x,
                                        double scale, std::vector<double>* y,
                                        std::vector<double>* m) {
  const size_t count = node.stations.size();
  const double x0 = node.first;
  if(count < 2) {
    (*y)[0] += scale * 0.5 * (x * x - x0 * x0);
    return;
  }
  const double h = node.spacing;
  const double last = x0 + h * double(count - 1);
  /* over segment k, x_k F_k + h^2 int_0^1 t Y dt: h^2 (Y_k / 6 + Y_k+1 / 3)
  - h^4 (7 M_k + 8 M_k+1) / 360 */
  auto whole = [&](size_t k) {
    const double xk = x0 + h * double(k);
    (*y)[k] += scale * (0.5 * xk * h + h * h / 6.0);
    (*y)[k + 1] += scale * (0.5 * xk * h + h * h / 3.0);
    (*m)[k] += scale * (-xk * h * h * h / 24.0 - 7.0 * h * h * h * h / 360.0);
    (*m)[k + 1] +=
      scale * (-xk * h * h * h / 24.0 - 8.0 * h * h * h * h / 360.0);
  };
  if(x < x0) {
    /* int_0^delta (x_0 + e) (Y_0 + e Y'(x_0)) de */
    const double delta = x - x0;
    (*y)[0] += scale * (x0 * delta + 0.5 * delta * delta);
    AddSlope(node, x0,
             scale * (0.5 * x0 * delta * delta + delta * delta * delta / 3.0),
             y, m);
    return;
  }
  if(x > last) {
    for(size_t k = 0; k + 1 < count; k++) {
      whole(k);
    }
    const double delta = x - last;
    (*y)[count - 1] += scale * (last * delta + 0.5 * delta * delta);
    AddSlope(node, last,
             scale * (0.5 * last * delta * delta + delta * delta * delta / 3.0),
             y, m);
    return;
  }
  const size_t i = std::min(size_t((x - x0) / h), count - 2);
  for(size_t k = 0; k < i; k++) {
    whole(k);
  }
  /* x_i h int_0^t Y dt + h^2 int_0^t t Y dt */
  const double xi = x0 + h * double(i);
  const double t = (x - xi) / h;
  const double r = 1.0 - t;
  const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
  const double h2 = h * h, h4 = h2 * h2;
  (*y)[i] += scale * (xi * h * (t - 0.5 * t2) + h2 * (0.5 * t2 - t3 / 3.0));
  (*y)[i + 1] += scale * (xi * h * 0.5 * t2 + h2 * t3 / 3.0);
  (*m)[i] += scale * (xi * h * h2 / 6.0 *
                        (0.5 * r * r - 0.25 * r * r * r * r - 0.25) +
                      h4 / 6.0 * (-2.0 * t3 / 3.0 + 0.75 * t4 - 0.2 * t5));
  (*m)[i + 1] += scale * (xi * h * h2 / 6.0 * (0.25 * t4 - 0.5 * t2) +
                          h4 / 6.0 * (0.2 * t5 - t3 / 3.0));
}

/* jets: a value and its first and second derivatives */
static void JetProduct(const double u[3], const double v[3], double out[3]) {
  out[0] = u[0] * v[0];
  out[1] = u[1] * v[0] + u[0] * v[1];
  out[2] = u[2] * v[0] + 2.0 * u[1] * v[1] + u[0] * v[2];
}

static void JetInverse(const double u[3], double out[3]) {
  out[0] = 1.0 / u[0];
  out[1] = -u[1] * out[0] * out[0];
  out[2] = 2.0 * u[1] * u[1] * out[0] * out[0] * out[0] -
           u[2] * out[0] * out[0];
}

static void JetScale(const ksl_vec3_t v[3], const double c[3],
                     ksl_vec3_t out[3]) {
  for(int i = 0; i < 3; i++) {
    out[0].at[i] = c[0] * v[0].at[i];
    out[1].at[i] = c[1] * v[0].at[i] + c[0] * v[1].at[i];
    out[2].at[i] =
      c[2] * v[0].at[i] + 2.0 * c[1] * v[1].at[i] + c[0] * v[2].at[i];
  }
}

shift::core::body::Flex::Carried
shift::core::body::Flex::Carry(const SlidingNode& node, double s) {
  const size_t count = node.stations.size();
  Carried carried;
  for(int k = 0; k < 3; k++) {
    carried.y[k].assign(count, 0.0);
    carried.m[k].assign(count, 0.0);
  }
  carried.x_c = s + node.center;
  /* over the part [low, high] of each interval within the bore: an end
  within the bore moves with s (its derivative is 1), one held at the
  bore's end does not (0) */
  bool contact = false;
  for(const auto& interval : node.bearing.intervals) {
    double low = s + interval[0], high = s + interval[1];
    double low_1 = 1.0, high_1 = 1.0;
    if(low < node.bearing.low) {
      low = node.bearing.low;
      low_1 = 0.0;
    }
    if(high > node.bearing.high) {
      high = node.bearing.high;
      high_1 = 0.0;
    }
    if(!(high > low)) {
      continue;
    }
    if(!contact) {
      contact = true;
      for(int k = 0; k < 3; k++) {
        carried.py[k].assign(count, 0.0);
        carried.pm[k].assign(count, 0.0);
      }
    }
    /* F = int Y, F' = [Y], F'' = [Y']; P = int x Y, P' = [x Y], P'' =
    [Y + x Y'] (the ends' derivatives are 0 or 1) */
    AddIntegral(node, high, 1.0, &carried.y[0], &carried.m[0]);
    AddIntegral(node, low, -1.0, &carried.y[0], &carried.m[0]);
    AddValue(node, high, high_1, &carried.y[1], &carried.m[1]);
    AddValue(node, low, -low_1, &carried.y[1], &carried.m[1]);
    AddSlope(node, high, high_1, &carried.y[2], &carried.m[2]);
    AddSlope(node, low, -low_1, &carried.y[2], &carried.m[2]);
    AddMoment(node, high, 1.0, &carried.py[0], &carried.pm[0]);
    AddMoment(node, low, -1.0, &carried.py[0], &carried.pm[0]);
    AddValue(node, high, high_1 * high, &carried.py[1], &carried.pm[1]);
    AddValue(node, low, -low_1 * low, &carried.py[1], &carried.pm[1]);
    AddValue(node, high, high_1, &carried.py[2], &carried.pm[2]);
    AddValue(node, low, -low_1, &carried.py[2], &carried.pm[2]);
    AddSlope(node, high, high_1 * high, &carried.py[2], &carried.pm[2]);
    AddSlope(node, low, -low_1 * low, &carried.py[2], &carried.pm[2]);
    /* the integrals of 1, x and x^2 */
    carried.length[0] += high - low;
    carried.length[1] += high_1 - low_1;
    carried.first[0] += 0.5 * (high * high - low * low);
    carried.first[1] += high * high_1 - low * low_1;
    carried.first[2] += high_1 - low_1;
    carried.second[0] += (high * high * high - low * low * low) / 3.0;
    carried.second[1] += high * high * high_1 - low * low * low_1;
    carried.second[2] += 2.0 * (high * high_1 - low * low_1);
  }
  carried.point = !contact;
  if(!contact) {
    AddValue(node, carried.x_c, 1.0, &carried.y[0], &carried.m[0]);
    AddSlope(node, carried.x_c, 1.0, &carried.y[1], &carried.m[1]);
    const SplineWeights w = Weights(node, carried.x_c);
    carried.m[2][w.i] += w.g0;
    carried.m[2][w.i1] += w.g1;
  }
  return carried;
}

void shift::core::body::Flex::CarryShape(const SlidingNode& node,
                                         const Carried& carried,
                                         const ksl_vec3_t& a,
                                         const std::vector<ksl_screw_t>& y,
                                         const std::vector<ksl_screw_t>& m,
                                         ksl_screw_t out[3]) {
  const size_t count = node.stations.size();
  ksl_screw_t F[3] = {}, P[3] = {};
  for(size_t i = 0; i < count; i++) {
    for(int k = 0; k < 3; k++) {
      const double wy = carried.y[k][i], wm = carried.m[k][i];
      if(wy != 0.0 || wm != 0.0) {
        for(int c = 0; c < 6; c++) {
          F[k].at[c] += wy * y[i].at[c] + wm * m[i].at[c];
        }
      }
      if(!carried.point) {
        const double py = carried.py[k][i], pm = carried.pm[k][i];
        if(py != 0.0 || pm != 0.0) {
          for(int c = 0; c < 6; c++) {
            P[k].at[c] += py * y[i].at[c] + pm * m[i].at[c];
          }
        }
      }
    }
  }
  if(carried.point) {
    for(int k = 0; k < 3; k++) {
      out[k] = F[k];
    }
    return;
  }
  /* the scalars: 1 / l, the middle xbar = int x / l, and D = I + c l with
  I = int x^2 - xbar int x, c = r^2 / 2 */
  const double c = 0.5 * node.bearing.radius * node.bearing.radius;
  double inverse_l[3], middle[3], moment[3], d[3], inverse_d[3];
  JetInverse(carried.length, inverse_l);
  JetProduct(carried.first, inverse_l, middle);
  JetProduct(carried.first, middle, moment);
  for(int k = 0; k < 3; k++) {
    d[k] = carried.second[k] - moment[k] + c * carried.length[k];
  }
  JetInverse(d, inverse_d);
  /* the mean translation int u / l; the rotation along the axis int
  theta_a / l, across it (a x (int x u - xbar int u) + c int theta_perp) / D
  */
  ksl_vec3_t u[3], theta_a[3], theta_perp[3], p[3], turn[3];
  for(int k = 0; k < 3; k++) {
    u[k] = F[k].lin;
    const double along = ksl_dot_vv(&F[k].ang, &a);
    ksl_product_av(along, &a, &theta_a[k]);
    ksl_subtract_vv(&F[k].ang, &theta_a[k], &theta_perp[k]);
    p[k] = P[k].lin;
  }
  ksl_vec3_t mean_u[3], mean_a[3], middle_u[3], across[3], theta[3];
  JetScale(u, inverse_l, mean_u);
  JetScale(theta_a, inverse_l, mean_a);
  JetScale(u, middle, middle_u);
  for(int k = 0; k < 3; k++) {
    ksl_vec3_t lever;
    ksl_subtract_vv(&p[k], &middle_u[k], &lever);
    ksl_cross_vv(&a, &lever, &turn[k]);
    ksl_axpy_vv(c, &theta_perp[k], &turn[k]);
  }
  JetScale(turn, inverse_d, across);
  for(int k = 0; k < 3; k++) {
    ksl_add_vv(&mean_a[k], &across[k], &theta[k]);
  }
  /* at the child's point: u + (x_c - xbar) theta x a */
  const double offset[3] = {carried.x_c - middle[0], 1.0 - middle[1],
                            -middle[2]};
  ksl_vec3_t theta_a_cross[3], shift_c[3];
  for(int k = 0; k < 3; k++) {
    ksl_cross_vv(&theta[k], &a, &theta_a_cross[k]);
  }
  JetScale(theta_a_cross, offset, shift_c);
  for(int k = 0; k < 3; k++) {
    ksl_add_vv(&mean_u[k], &shift_c[k], &out[k].lin);
    out[k].ang = theta[k];
  }
}

shift::core::body::Flex::SplineWeights shift::core::body::Flex::Weights(
  const SlidingNode& node, double s) {
  SplineWeights w;
  const size_t count = node.stations.size();
  if(count < 2) {
    return w;
  }
  const double h = node.spacing;
  const double last = node.first + h * double(count - 1);
  w.d0 = -1.0 / h;
  w.d1 = 1.0 / h;
  if(s < node.first) {
    /* Y_0 + delta Y'(x_0), Y'(x_0) = (Y_1 - Y_0) / h - h M_1 / 6 */
    const double delta = s - node.first;
    w.i = 0;
    w.i1 = 1;
    w.c0 = 1.0 - delta / h;
    w.c1 = delta / h;
    w.e1 = -delta * h / 6.0;
    w.f1 = -h / 6.0;
    return w;
  }
  if(s > last) {
    /* Y_N + delta Y'(x_N), Y'(x_N) = (Y_N - Y_N-1) / h + h M_N-1 / 6 */
    const double delta = s - last;
    w.i = count - 2;
    w.i1 = count - 1;
    w.c0 = -delta / h;
    w.c1 = 1.0 + delta / h;
    w.e0 = delta * h / 6.0;
    w.f0 = h / 6.0;
    return w;
  }
  w.i = std::min(size_t((s - node.first) / h), count - 2);
  w.i1 = w.i + 1;
  const double t = (s - node.first - h * double(w.i)) / h;
  const double r = 1.0 - t;
  w.c0 = r;
  w.c1 = t;
  w.e0 = h * h / 6.0 * (r * r * r - r);
  w.e1 = h * h / 6.0 * (t * t * t - t);
  w.f0 = -h / 6.0 * (3.0 * r * r - 1.0);
  w.f1 = h / 6.0 * (3.0 * t * t - 1.0);
  w.g0 = r;
  w.g1 = t;
  return w;
}

void shift::core::body::Flex::SetInitialElasticState(
  const std::vector<double>& eta, const std::vector<double>& eta_dot) {
  if(eta.size() != elastic_count_ || eta_dot.size() != elastic_count_) {
    throw shift::core::Exception(
      "The initial elastic state of flexible body '" + get_name() +
      "' must have " + std::to_string(elastic_count_) + " values each");
  }
  initial_eta_ = eta;
  initial_eta_dot_ = eta_dot;
}

void shift::core::body::Flex::Connect() {
  Base::Connect();
  /* a body built from a mesh has its model and nodes from BuildElastic */
  if(builds_model()) {
    return;
  }
  ResolveNodes();
}

void shift::core::body::Flex::ResolveNodes() {
  const size_t point_count = masses_.size();
  if(elastic_count_ == 0 || point_count == 0) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no model");
  }
  node_frames_.assign(node_names_.size(), nullptr);
  for(size_t k = 0; k < node_names_.size(); k++) {
    for(auto& frame : get_frames()) {
      if(frame->get_name() == node_names_[k]) {
        node_frames_[k] = frame.get();
      }
    }
    if(!node_frames_[k] || node_points_[k] >= point_count) {
      throw shift::core::Exception(
        "Flexible body '" + get_name() + "' has no frame '" + node_names_[k] +
        "' or no point " + std::to_string(node_points_[k]) + " for a node");
    }
  }
}

void shift::core::body::Flex::SetupElastic() {
  const size_t n = elastic_count_;
  const size_t point_count = masses_.size();

  /* the mass properties of the undeformed body, in the origin's
  coordinates, for the rigid-body uses of the body's inertia */
  double mass = 0.0;
  ksl_vec3_t first_moment = ksl_vec3(0.0, 0.0, 0.0);
  ksl_mat3x3_t J_o = {};
  for(size_t p = 0; p < point_count; p++) {
    mass += masses_[p];
    AddScaledVector(masses_[p], positions_[p], &first_moment);
    AddScaled(1.0, rotary_inertias_[p], &J_o);
    AddSkewSkew(-masses_[p], positions_[p], positions_[p], &J_o);
  }
  if(!(mass > 0.0)) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no mass");
  }
  ksl_vec3_t center;
  ksl_product_av(1.0 / mass, &first_moment, &center);
  /* centroidal: J_o = I_c - m c~ c~ */
  ksl_mat3x3_t I_c = J_o;
  AddSkewSkew(mass, center, center, &I_c);
  inertia_ = ksl_inertia(mass, center, I_c.m00, I_c.m11, I_c.m22, I_c.m01,
                         I_c.m12, I_c.m02);
  center_of_mass_wrt_origin_ = center;

  /* the model in the reference frame's coordinates */
  const ksl_SE3_t* D_or = get_reference_frame()->get_d_wrt_origin();
  positions_r_.resize(point_count);
  shapes_r_.resize(point_count * n);
  std::vector<ksl_mat3x3_t> rotary_r(point_count);
  for(size_t p = 0; p < point_count; p++) {
    ksl_product_dinvv(D_or, &positions_[p], &positions_r_[p]);
    for(size_t j = 0; j < n; j++) {
      const ksl_screw_t& phi = shapes_[p * n + j];
      ksl_screw_t& phi_r = shapes_r_[p * n + j];
      ksl_product_drinvv(&D_or->R, &phi.lin, &phi_r.lin);
      ksl_product_drinvv(&D_or->R, &phi.ang, &phi_r.ang);
    }
    ksl_mat3x3_t temp;
    ksl_product_drdr(&rotary_inertias_[p], &D_or->R, &temp);
    ksl_product_drinvdr(&D_or->R, &temp, &rotary_r[p]);
  }

  /* the sliding nodes' splines: M_0 = M_N-1 = 0 and, between,
  M_i-1 + 4 M_i + M_i+1 = 6 (Y_i+1 - 2 Y_i + Y_i-1) / h^2, by the Thomas
  algorithm for each of the 6 n_e components */
  for(SlidingNode& node : sliding_) {
    const size_t count = node.stations.size();
    for(unsigned int station : node.stations) {
      if(station >= point_count) {
        throw shift::core::Exception("Flexible body '" + get_name() +
                                     "': a sliding node has no point " +
                                     std::to_string(station));
      }
    }
    node.curvature.assign(count * n, ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    if(count < 3) {
      continue;
    }
    std::vector<double> y(count);
    for(size_t j = 0; j < n; j++) {
      for(int c = 0; c < 6; c++) {
        for(size_t i = 0; i < count; i++) {
          y[i] = shapes_r_[size_t(node.stations[i]) * n + j].at[c];
        }
        const std::vector<double> m = SplineCurvatures(y, node.spacing);
        for(size_t i = 0; i < count; i++) {
          node.curvature[i * n + j].at[c] = m[i];
        }
      }
    }
  }

  /* the invariants */
  mass_ = mass;
  invariant_1_ = {};
  invariant_4_ = ksl_vec3(0.0, 0.0, 0.0);
  invariant_5_.assign(n, ksl_vec3(0.0, 0.0, 0.0));
  invariant_2s_.assign(n, ksl_mat3x3_t{});
  invariant_3_.assign(n * n, ksl_mat3x3_t{});
  invariant_6_.assign(n, ksl_vec3(0.0, 0.0, 0.0));
  invariant_7_.assign(n * n, ksl_vec3(0.0, 0.0, 0.0));
  invariant_9_.assign(n * n, 0.0);
  for(size_t p = 0; p < point_count; p++) {
    const double m_p = masses_[p];
    const ksl_vec3_t& t = positions_r_[p];
    AddScaled(1.0, rotary_r[p], &invariant_1_);
    AddSkewSkew(-m_p, t, t, &invariant_1_);
    AddScaledVector(m_p, t, &invariant_4_);
    for(size_t j = 0; j < n; j++) {
      const ksl_vec3_t& phi_j = shapes_r_[p * n + j].lin;
      const ksl_vec3_t& psi_j = shapes_r_[p * n + j].ang;
      AddScaledVector(m_p, phi_j, &invariant_5_[j]);
      /* T^2_j + T^2T_j = -m (t~ phi~ + phi~ t~) */
      AddSkewSkew(-m_p, t, phi_j, &invariant_2s_[j]);
      AddSkewSkew(-m_p, phi_j, t, &invariant_2s_[j]);
      /* T^6_j = m t~ phi + I psi */
      ksl_vec3_t temp;
      ksl_cross_vv(&t, &phi_j, &temp);
      AddScaledVector(m_p, temp, &invariant_6_[j]);
      ksl_product_drv(&rotary_r[p], &psi_j, &temp);
      AddScaledVector(1.0, temp, &invariant_6_[j]);
      for(size_t i = 0; i < n; i++) {
        const ksl_vec3_t& phi_i = shapes_r_[p * n + i].lin;
        const ksl_vec3_t& psi_i = shapes_r_[p * n + i].ang;
        AddSkewSkew(-m_p, phi_i, phi_j, &invariant_3_[i * n + j]);
        ksl_cross_vv(&phi_i, &phi_j, &temp);
        AddScaledVector(m_p, temp, &invariant_7_[i * n + j]);
        ksl_vec3_t I_psi_j;
        ksl_product_drv(&rotary_r[p], &psi_j, &I_psi_j);
        invariant_9_[i * n + j] +=
          m_p * ksl_dot_vv(&phi_i, &phi_j) + ksl_dot_vv(&psi_i, &I_psi_j);
      }
    }
  }

  mass_be_.resize(n);
  bias_e_.resize(n);
  std::vector<double> eta_dot(n, 0.0);
  EvaluateElastic(initial_eta_.data(), eta_dot.data(),
                  ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  std::stringstream ss;
  ss << "Flexible body '" << get_name() << "': " << point_count
     << " points, mass " << mass << ", " << n << " elastic coordinates, "
     << node_frames_.size() << " nodes, reference frame '"
     << get_reference_frame()->get_name() << "'";
  SHIFT_DEBUG(ss.str());
}

void shift::core::body::Flex::EvaluateMassBlocks(const double* eta,
                                                 ksl_mat3x3_t* J,
                                                 ksl_vec3_t* s,
                                                 ksl_vec3_t* G) const {
  const size_t n = elastic_count_;
  *J = invariant_1_;
  *s = invariant_4_;
  for(size_t j = 0; j < n; j++) {
    AddScaledVector(eta[j], invariant_5_[j], s);
    AddScaled(eta[j], invariant_2s_[j], J);
    G[j] = invariant_6_[j];
    for(size_t i = 0; i < n; i++) {
      AddScaled(eta[i] * eta[j], invariant_3_[i * n + j], J);
      AddScaledVector(eta[i], invariant_7_[i * n + j], &G[j]);
    }
  }
}

void shift::core::body::Flex::EvaluateElastic(const double* eta,
                                              const double* eta_dot,
                                              const ksl_screw_t& v_r) {
  const size_t n = elastic_count_;
  ksl_mat3x3_t J;
  ksl_vec3_t s;
  std::vector<ksl_vec3_t> G(n);
  EvaluateMassBlocks(eta, &J, &s, G.data());

  /* the frame-frame block in the ten-term form, with centroidal moments:
  J = I_c - (1/m) s~ s~ */
  ksl_mat3x3_t I_c = J;
  AddSkewSkew(1.0 / mass_, s, s, &I_c);
  inertia_wrt_reference_.m = mass_;
  inertia_wrt_reference_.mt = s;
  inertia_wrt_reference_.Ixx = I_c.m00;
  inertia_wrt_reference_.Iyy = I_c.m11;
  inertia_wrt_reference_.Izz = I_c.m22;
  inertia_wrt_reference_.Ixy = I_c.m01;
  inertia_wrt_reference_.Iyz = I_c.m12;
  inertia_wrt_reference_.Izx = I_c.m02;

  for(size_t j = 0; j < n; j++) {
    mass_be_[j].lin = invariant_5_[j];
    mass_be_[j].ang = G[j];
  }

  const ksl_vec3_t& v = v_r.lin;
  const ksl_vec3_t& w = v_r.ang;

  /* s_dot = T^5 eta_dot, G eta_dot, J_dot w */
  ksl_vec3_t s_dot = ksl_vec3(0.0, 0.0, 0.0);
  ksl_vec3_t G_eta_dot = ksl_vec3(0.0, 0.0, 0.0);
  ksl_mat3x3_t J_dot = {};
  for(size_t j = 0; j < n; j++) {
    AddScaledVector(eta_dot[j], invariant_5_[j], &s_dot);
    AddScaledVector(eta_dot[j], G[j], &G_eta_dot);
    AddScaled(eta_dot[j], invariant_2s_[j], &J_dot);
    for(size_t i = 0; i < n; i++) {
      AddScaled(eta_dot[i] * eta[j] + eta[i] * eta_dot[j],
                invariant_3_[i * n + j], &J_dot);
    }
  }
  ksl_vec3_t J_dot_w;
  ksl_product_drv(&J_dot, &w, &J_dot_w);

  /* b = -{2 w x s_dot; w x G eta_dot + J_dot w} */
  ksl_vec3_t temp;
  ksl_cross_vv(&w, &s_dot, &temp);
  ksl_product_av(-2.0, &temp, &bias_b_.lin);
  ksl_cross_vv(&w, &G_eta_dot, &temp);
  ksl_add_vv(&temp, &J_dot_w, &bias_b_.ang);
  ksl_vec3_invert(&bias_b_.ang);

  for(size_t k = 0; k < n; k++) {
    double b = 0.0;
    /* 2 w . sum_j T^7_kj eta_dot_j */
    ksl_vec3_t T7_eta_dot = ksl_vec3(0.0, 0.0, 0.0);
    for(size_t j = 0; j < n; j++) {
      AddScaledVector(eta_dot[j], invariant_7_[k * n + j], &T7_eta_dot);
    }
    b += 2.0 * ksl_dot_vv(&w, &T7_eta_dot);
    /* -v . (T^5_k x w) */
    ksl_cross_vv(&invariant_5_[k], &w, &temp);
    b -= ksl_dot_vv(&v, &temp);
    /* 1/2 w^T (dJ/d eta_k) w = 1/2 w^T Js_k w + sum_j eta_j w^T T^3_kj w */
    ksl_mat3x3_t dJ = {};
    AddScaled(0.5, invariant_2s_[k], &dJ);
    for(size_t j = 0; j < n; j++) {
      AddScaled(eta[j], invariant_3_[k * n + j], &dJ);
    }
    ksl_product_drv(&dJ, &w, &temp);
    b += ksl_dot_vv(&w, &temp);
    /* elastic and damping forces */
    for(size_t j = 0; j < n; j++) {
      b -= stiffness_[k * n + j] * eta[j] + damping_[k * n + j] * eta_dot[j];
    }
    bias_e_[k] = b;
  }
}

double shift::core::body::Flex::KineticEnergy(const double* eta,
                                              const double* eta_dot,
                                              const ksl_screw_t& v_r) const {
  const size_t n = elastic_count_;
  ksl_mat3x3_t J;
  ksl_vec3_t s;
  std::vector<ksl_vec3_t> G(n);
  EvaluateMassBlocks(eta, &J, &s, G.data());
  const ksl_vec3_t& v = v_r.lin;
  const ksl_vec3_t& w = v_r.ang;

  /* 2T = m v.v - 2 v.(s x w) + 2 v.(T^5 eta_dot) + w^T J w
          + 2 w.(G eta_dot) + eta_dot^T T^9 eta_dot */
  ksl_vec3_t temp;
  double two_t = mass_ * ksl_dot_vv(&v, &v);
  ksl_cross_vv(&s, &w, &temp);
  two_t -= 2.0 * ksl_dot_vv(&v, &temp);
  ksl_product_drv(&J, &w, &temp);
  two_t += ksl_dot_vv(&w, &temp);
  for(size_t j = 0; j < n; j++) {
    two_t += 2.0 * eta_dot[j] *
             (ksl_dot_vv(&v, &invariant_5_[j]) + ksl_dot_vv(&w, &G[j]));
    for(size_t i = 0; i < n; i++) {
      two_t += eta_dot[i] * invariant_9_[i * n + j] * eta_dot[j];
    }
  }
  return 0.5 * two_t;
}

double shift::core::body::Flex::ElasticEnergy(const double* eta) const {
  const size_t n = elastic_count_;
  double e = 0.0;
  for(size_t i = 0; i < n; i++) {
    for(size_t j = 0; j < n; j++) {
      e += eta[i] * stiffness_[i * n + j] * eta[j];
    }
  }
  return 0.5 * e;
}

void shift::core::body::Flex::EvaluateSlidingNode(
  unsigned int k, double s, double s_dot, const double* eta,
  const double* eta_dot, ksl_SE3_t* d_rk, ksl_screw_t* h_k,
  ksl_screw_t* gamma_k, ksl_screw_t* h_s, ksl_screw_t* h_s_dot) const {
  if(!is_sliding_node(k)) {
    EvaluateNode(k, eta, eta_dot, d_rk, h_k, gamma_k);
    return;
  }
  const size_t n = elastic_count_;
  const SlidingNode& node = sliding_[size_t(node_sliding_[k])];
  const Carried carried = Carry(node, s);
  const bool rates = eta_dot != nullptr;
  const ksl_SE3_t* D_0 = node_frames_[k]->get_d_wrt_reference();
  ksl_vec3_t a;
  ksl_product_drv(&D_0->R, &node.axis, &a);
  /* the stations the child's deformation depends on */
  const size_t count = node.stations.size();
  std::vector<size_t> used;
  for(size_t i = 0; i < count; i++) {
    for(int d = 0; d < 3; d++) {
      if(carried.y[d][i] != 0.0 || carried.m[d][i] != 0.0 ||
         (!carried.point &&
          (carried.py[d][i] != 0.0 || carried.pm[d][i] != 0.0))) {
        used.push_back(i);
        break;
      }
    }
  }
  std::vector<ksl_screw_t> y_j(count, ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  std::vector<ksl_screw_t> m_j(count, ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  /* the deformation at the child's point and its derivatives in s: u,
  theta (and ', ''), and with the rates: rho_e, Theta_dot (Upsilon eta_dot)
  and U', Theta' (Upsilon' eta_dot) */
  ksl_vec3_t u = {}, theta = {}, u1 = {}, theta1 = {}, u2 = {}, theta2 = {};
  ksl_vec3_t rho_e = {}, Theta = {}, U1 = {}, Theta1 = {};
  for(size_t j = 0; j < n; j++) {
    for(size_t i : used) {
      y_j[i] = shapes_r_[size_t(node.stations[i]) * n + j];
      m_j[i] = node.curvature[i * n + j];
    }
    ksl_screw_t v[3];
    CarryShape(node, carried, a, y_j, m_j, v);
    const ksl_screw_t& y = v[0];
    const ksl_screw_t& d = v[1];
    const ksl_screw_t& dd = v[2];
    AddScaledVector(eta[j], y.lin, &u);
    AddScaledVector(eta[j], y.ang, &theta);
    AddScaledVector(eta[j], d.lin, &u1);
    AddScaledVector(eta[j], d.ang, &theta1);
    AddScaledVector(eta[j], dd.lin, &u2);
    AddScaledVector(eta[j], dd.ang, &theta2);
    if(rates) {
      AddScaledVector(eta_dot[j], y.lin, &rho_e);
      AddScaledVector(eta_dot[j], y.ang, &Theta);
      AddScaledVector(eta_dot[j], d.lin, &U1);
      AddScaledVector(eta_dot[j], d.ang, &Theta1);
    }
    if(h_k) {
      h_k[j] = y;
    }
  }

  /* the child's point p(s) = t_0 + (s + center) a, moved: rho = p + u,
  turned by theta */
  const double x = s + node.center;
  ksl_vec3_t rho = D_0->t;
  AddScaledVector(x, a, &rho);
  AddScaledVector(1.0, u, &rho);
  ksl_mat3x3_t R_theta;
  const double angle = ksl_vec3_l2norm(&theta);
  if(angle > 1e-300) {
    ksl_vec3_t direction;
    ksl_product_av(1.0 / angle, &theta, &direction);
    ksl_mat3x3_setFromAxisAngle(&R_theta, &direction, angle);
  } else {
    ksl_mat3x3_setIdentity(&R_theta);
  }
  /* the frame: offset -(s + center) a from the point, turned with it */
  ksl_vec3_t offset, offset_turned;
  ksl_product_av(-x, &a, &offset);
  ksl_product_drv(&R_theta, &offset, &offset_turned);
  ksl_product_drdr(&R_theta, &D_0->R, &d_rk->R);
  ksl_add_vv(&rho, &offset_turned, &d_rk->t);

  if(h_k) {
    for(size_t j = 0; j < n; j++) {
      ksl_vec3_t moment;
      ksl_cross_vv(&rho, &h_k[j].ang, &moment);
      AddScaledVector(1.0, moment, &h_k[j].lin);
    }
  }
  /* rho_dot = (a + u') s_dot + rho_e */
  ksl_vec3_t rho_dot = rho_e;
  AddScaledVector(s_dot, a, &rho_dot);
  AddScaledVector(s_dot, u1, &rho_dot);
  if(gamma_k) {
    /* {rho_dot x Theta_dot + s_dot (U' + rho x Theta'); s_dot Theta'} */
    ksl_vec3_t temp;
    ksl_cross_vv(&rho_dot, &Theta, &gamma_k->lin);
    AddScaledVector(s_dot, U1, &gamma_k->lin);
    ksl_cross_vv(&rho, &Theta1, &temp);
    AddScaledVector(s_dot, temp, &gamma_k->lin);
    ksl_product_av(s_dot, &Theta1, &gamma_k->ang);
  }
  if(h_s) {
    /* {a + u' + rho x theta'; theta'} */
    ksl_cross_vv(&rho, &theta1, &h_s->lin);
    AddScaledVector(1.0, a, &h_s->lin);
    AddScaledVector(1.0, u1, &h_s->lin);
    h_s->ang = theta1;
  }
  if(h_s_dot) {
    /* {s_dot u'' + U' + rho_dot x theta' + rho x (s_dot theta'' + Theta');
    s_dot theta'' + Theta'} */
    ksl_vec3_t angular = Theta1, temp;
    AddScaledVector(s_dot, theta2, &angular);
    ksl_cross_vv(&rho_dot, &theta1, &h_s_dot->lin);
    AddScaledVector(s_dot, u2, &h_s_dot->lin);
    AddScaledVector(1.0, U1, &h_s_dot->lin);
    ksl_cross_vv(&rho, &angular, &temp);
    AddScaledVector(1.0, temp, &h_s_dot->lin);
    h_s_dot->ang = angular;
  }
}

void shift::core::body::Flex::EvaluateNode(unsigned int k, const double* eta,
                                           const double* eta_dot,
                                           ksl_SE3_t* d_rk, ksl_screw_t* h_k,
                                           ksl_screw_t* gamma_k) const {
  const size_t n = elastic_count_;
  const unsigned int p = node_points_[k];
  const ksl_screw_t* phi = &shapes_r_[p * n];

  /* the point's deformed position and rotation vector */
  ksl_vec3_t rho = positions_r_[p];
  ksl_vec3_t theta = ksl_vec3(0.0, 0.0, 0.0);
  for(size_t j = 0; j < n; j++) {
    AddScaledVector(eta[j], phi[j].lin, &rho);
    AddScaledVector(eta[j], phi[j].ang, &theta);
  }

  /* the node frame is carried rigidly by the point: turned by theta, and
  its offset from the point turned with it */
  ksl_mat3x3_t R_theta;
  const double angle = ksl_vec3_l2norm(&theta);
  if(angle > 1e-300) {
    ksl_vec3_t axis;
    ksl_product_av(1.0 / angle, &theta, &axis);
    ksl_mat3x3_setFromAxisAngle(&R_theta, &axis, angle);
  } else {
    ksl_mat3x3_setIdentity(&R_theta);
  }
  const ksl_SE3_t* D_0 = node_frames_[k]->get_d_wrt_reference();
  ksl_vec3_t offset, offset_turned;
  ksl_subtract_vv(&D_0->t, &positions_r_[p], &offset);
  ksl_product_drv(&R_theta, &offset, &offset_turned);
  ksl_product_drdr(&R_theta, &D_0->R, &d_rk->R);
  ksl_add_vv(&rho, &offset_turned, &d_rk->t);

  /* in r, referred to its origin: h_j = {phi_j + rho x psi_j; psi_j} */
  if(h_k) {
    for(size_t j = 0; j < n; j++) {
      ksl_cross_vv(&rho, &phi[j].ang, &h_k[j].lin);
      AddScaledVector(1.0, phi[j].lin, &h_k[j].lin);
      h_k[j].ang = phi[j].ang;
    }
  }
  /* sum_j h_dot_j eta_dot_j = {rho_dot x theta_dot; 0} */
  if(gamma_k) {
    ksl_vec3_t rho_dot = ksl_vec3(0.0, 0.0, 0.0);
    ksl_vec3_t theta_dot = ksl_vec3(0.0, 0.0, 0.0);
    for(size_t j = 0; j < n; j++) {
      AddScaledVector(eta_dot[j], phi[j].lin, &rho_dot);
      AddScaledVector(eta_dot[j], phi[j].ang, &theta_dot);
    }
    ksl_cross_vv(&rho_dot, &theta_dot, &gamma_k->lin);
    gamma_k->ang = ksl_vec3(0.0, 0.0, 0.0);
  }
}
