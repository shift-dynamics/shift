#include "shift/core/controller/slide_equilibrium.h"

#include <algorithm>
#include <cmath>
#include <vector>

void shift::core::controller::SlideEquilibrium::TabulateDeflection(
  double most, double step) {
  deflections_.clear();
  resistances_.clear();
  deflection_step_ = 0.0;
  if(!(most > 0.0) || !(step > 0.0)) {
    return;
  }
  deflection_step_ = step;
  const int n = static_cast<int>(std::ceil(most / step)) + 1;
  for(int i = 0; i <= n; ++i) {
    double deflection, resistance;
    tire.RestingDeflection(i * step, &deflection, &resistance);
    deflections_.push_back(deflection);
    resistances_.push_back(resistance);
  }
}

void shift::core::controller::SlideEquilibrium::Deflection(
  double load, double* deflection, double* resistance) const {
  const double f = load / std::max(deflection_step_, 1e-300);
  if(deflections_.size() < 2 || !(f >= 0.0) ||
     f >= static_cast<double>(deflections_.size() - 1)) {
    tire.RestingDeflection(load, deflection, resistance);
    return;
  }
  const size_t i = static_cast<size_t>(f);
  const double w = f - static_cast<double>(i);
  *deflection = (1.0 - w) * deflections_[i] + w * deflections_[i + 1];
  *resistance = (1.0 - w) * resistances_[i] + w * resistances_[i + 1];
}

void shift::core::controller::SlideEquilibrium::Wheels(
  double vx, double vy, double r, double ax, double ay, double carrier,
  const double* difference, double delta, Totals* totals, State* state,
  double normal) const {
  /* the loads of the wheels: left front, right front, left rear, right
  rear, of gravity normal to the road */
  const double g = normal < 0.0 ? gravity : normal;
  const double front_axle =
    mass * (g * center_of_mass - ax * height) / wheelbase;
  const double rear_axle =
    mass * (g * (wheelbase - center_of_mass) + ax * height) / wheelbase;
  const double transfer = mass * ay * height / track;
  const double load[4] = {
    0.5 * front_axle + front_load_transfer * transfer,
    0.5 * front_axle - front_load_transfer * transfer,
    0.5 * rear_axle + (1.0 - front_load_transfer) * transfer,
    0.5 * rear_axle - (1.0 - front_load_transfer) * transfer};
  const double px[4] = {wheelbase, wheelbase, 0.0, 0.0};
  const double py[4] = {-0.5 * track, 0.5 * track, -0.5 * track, 0.5 * track};
  /* the spins: both carriers at one speed, each axle's wheels about it */
  const double radius = tire.unloaded_radius;
  double omega[4];
  for(int a = 0; a < 2; ++a) {
    omega[2 * a] = carrier + 0.5 * difference[a];
    omega[2 * a + 1] = carrier - 0.5 * difference[a];
  }
  /* each wheel's forces in its heading, turned into the frame, and its
  torque */
  *totals = Totals();
  for(int i = 0; i < 4; ++i) {
    const double steer =
      i < 2 ? delta - front_roll_steer * ay : -rear_roll_steer * ay;
    const double c = std::cos(steer), s = std::sin(steer);
    const double hx = vx - r * py[i];
    const double hy = vy + r * px[i];
    const double wx = hx * c + hy * s;
    const double wy = -hx * s + hy * c;
    /* the rolling radius and resistance as the tire has them at the load:
    its spring, or its sinkage on soft soil */
    double deflection, resistance;
    Deflection(load[i], &deflection, &resistance);
    const double rolling = radius - deflection;
    const double u = omega[i] * rolling;
    /* the greater of the tread's speed and the hub's, smoothly with
    slip_smoothing */
    const double au = std::fabs(u), aw = std::fabs(wx);
    const double greater =
      slip_smoothing > 0.0
        ? 0.5 * (au + aw + std::hypot(au - aw, slip_smoothing))
        : std::max(au, aw);
    double kappa = (u - wx) / std::max(greater, tire.low_speed);
    kappa = std::min(std::max(kappa, -1.0), 1.0);
    const double alpha =
      std::atan2(wy, std::max(std::fabs(wx), tire.low_speed));
    double tx = 0.0, ty = 0.0, aligning = 0.0;
    if(load[i] > 0.0) {
      tire.TreadForces(kappa, alpha, wx, wy, load[i], tire.is_soft(), &tx, &ty,
                       &aligning);
    }
    totals->torque[i] = tx * rolling;
    totals->omega[i] = omega[i];
    totals->drive_force += tx;
    if(i < 2) {
      totals->front_force += ty;
    } else {
      totals->rear_force += ty;
    }
    const double total = tx - resistance * (wx > 0.0 ? 1.0 : -1.0);
    const double fx = total * c - ty * s;
    const double fy = total * s + ty * c;
    totals->fx += fx;
    totals->fy += fy;
    totals->moment += (px[i] - center_of_mass) * fy - py[i] * fx + aligning;
    if(state) {
      state->slip[i] = kappa;
      state->slip_angle[i] = alpha;
      state->load[i] = load[i];
      state->force_x[i] = tx;
      state->force_y[i] = ty;
    }
  }
}

double shift::core::controller::SlideEquilibrium::Differential(
  const Totals& totals, int axle) const {
  const Axle& a = axle == 0 ? front : rear;
  const double tl = totals.torque[2 * axle], tr = totals.torque[2 * axle + 1];
  double bias = 0.0;
  if(a.slip_speed > 0.0) {
    bias = 2.0 * (a.bias_torque + a.bias_ratio * std::fabs(tl + tr)) *
           std::tanh((totals.omega[2 * axle] - totals.omega[2 * axle + 1]) /
                     a.slip_speed);
  }
  return (tl - tr + bias) / (mass * gravity * tire.unloaded_radius);
}

void shift::core::controller::SlideEquilibrium::Residuals(double rho,
                                                          double beta,
                                                          const double* x,
                                                          double* e,
                                                          State* state) const {
  const double v = x[0], delta = x[1], sigma = x[2];
  const double vx = v * std::cos(beta);
  const double vy = v * std::sin(beta);
  const double r = v / rho;
  /* the acceleration of the center of mass on the circle */
  const double ax = -r * (vy + center_of_mass * r);
  const double ay = r * vx;
  const double radius = tire.unloaded_radius;
  const double carrier = (1.0 + sigma) * vx / radius;
  const double difference[2] = {x[3] * vx / radius, x[4] * vx / radius};
  Totals t;
  Wheels(vx, vy, r, ax, ay, carrier, difference, delta, &t, state);
  const double weight = mass * gravity;
  e[0] = (mass * ax - t.fx) / weight;
  e[1] = (mass * ay - t.fy) / weight;
  e[2] = t.moment / (weight * wheelbase);
  e[3] = Differential(t, 0);
  e[4] = Differential(t, 1);
  if(state) {
    state->speed = v;
    state->steering = delta;
    state->spin = sigma;
    state->front_difference = x[3];
    state->rear_difference = x[4];
    state->yaw_rate = r;
    state->drive_torque = t.torque[0] + t.torque[1] + t.torque[2] + t.torque[3];
    state->drive_force = t.drive_force;
    state->front_force = t.front_force;
    state->rear_force = t.rear_force;
    double n = 0.0;
    for(int k = 0; k < kUnknowns; ++k) {
      n += e[k] * e[k];
    }
    state->residual = std::sqrt(n);
  }
}

bool shift::core::controller::SlideEquilibrium::Solve(double rho, double beta,
                                                      State* state) const {
  constexpr int n = kUnknowns;
  double x[n] = {state->speed, state->steering, state->spin,
                 state->front_difference, state->rear_difference};
  const double lower[n] = {1.0, -0.7, -0.5, -1.0, -1.0};
  const double upper[n] = {80.0, 0.7, 3.0, 1.0, 1.0};
  const double step[n] = {1e-5, 1e-7, 1e-7, 1e-7, 1e-7};
  auto size = [](const double* f) {
    double s = 0.0;
    for(int k = 0; k < n; ++k) {
      s += f[k] * f[k];
    }
    return std::sqrt(s);
  };
  double e[n];
  Residuals(rho, beta, x, e, nullptr);
  double norm = size(e);
  for(int iteration = 0; iteration < 80 && norm > 1e-11; ++iteration) {
    /* the Jacobian by differences, and the Newton step */
    double j[n * n];
    for(int k = 0; k < n; ++k) {
      double y[n];
      std::copy(x, x + n, y);
      y[k] += step[k];
      double f[n];
      Residuals(rho, beta, y, f, nullptr);
      for(int i = 0; i < n; ++i) {
        j[i * n + k] = (f[i] - e[i]) / step[k];
      }
    }
    double dx[n];
    for(int i = 0; i < n; ++i) {
      dx[i] = -e[i];
    }
    if(!SolveLinear(n, j, dx)) {
      return false;
    }
    /* a damped step, within the bounds, that lowers the residual */
    double scale = 1.0;
    bool lowered = false;
    for(int halving = 0; halving < 25; ++halving, scale *= 0.5) {
      double y[n];
      for(int k = 0; k < n; ++k) {
        y[k] = std::min(std::max(x[k] + scale * dx[k], lower[k]), upper[k]);
      }
      double f[n];
      Residuals(rho, beta, y, f, nullptr);
      const double s = size(f);
      if(s < norm) {
        std::copy(y, y + n, x);
        std::copy(f, f + n, e);
        norm = s;
        lowered = true;
        break;
      }
    }
    if(!lowered) {
      break;
    }
  }
  Residuals(rho, beta, x, e, state);
  return norm < 1e-8;
}

bool shift::core::controller::SlideEquilibrium::Find(double rho, double beta,
                                                     State* state) const {
  const double side = rho > 0.0 ? 1.0 : -1.0;
  const double grip = std::sqrt(0.7 * tire.friction_coefficient * gravity *
                                std::fabs(rho));
  const double speeds[] = {grip, 0.7 * grip, 1.3 * grip};
  const double steerings[] = {side * std::atan(wheelbase / std::fabs(rho)),
                              0.0, -side * 0.1, side * 0.2, -side * 0.25};
  const double spins[] = {0.1, 0.03, 0.3, 0.8};
  for(double sigma : spins) {
    for(double delta : steerings) {
      for(double v : speeds) {
        State s;
        s.speed = v;
        s.steering = delta;
        s.spin = sigma;
        if(Solve(rho, beta, &s)) {
          *state = s;
          return true;
        }
      }
    }
  }
  return false;
}

void shift::core::controller::SlideEquilibrium::Derivatives(
  const double* s, double delta, double torque, double* ds,
  double* differences, const double* slope) const {
  const double gx = slope ? slope[0] : 0.0, gy = slope ? slope[1] : 0.0;
  const double normal = slope ? slope[2] : -1.0;
  const double vx = s[0], vy = s[1], r = s[2], carrier = s[3];
  /* the loads of the accelerations of the center of mass, from the steady
  ones by a few passes */
  double ax = -r * (vy + center_of_mass * r);
  double ay = r * vx;
  double d[2] = {0.0, 0.0};
  if(differences) {
    d[0] = differences[0];
    d[1] = differences[1];
  }
  Totals t;
  for(int pass = 0; pass < 4; ++pass) {
    Balance(vx, vy, r, ax, ay, carrier, delta, d, &t, normal);
    const double r_dot = t.moment / yaw_inertia;
    /* the tires' forces carry the loads over; gravity in the road's plane
    adds to the motion */
    ax = t.fx / mass;
    ay = t.fy / mass;
    ds[0] = ax + gx + r * (vy + center_of_mass * r);
    ds[1] = ay + gy - r * vx - center_of_mass * r_dot;
    ds[2] = r_dot;
  }
  ds[3] = (torque - (t.torque[0] + t.torque[1] + t.torque[2] + t.torque[3])) /
          drive_inertia;
  if(differences) {
    differences[0] = d[0];
    differences[1] = d[1];
  }
}

void shift::core::controller::SlideEquilibrium::Balance(
  double vx, double vy, double r, double ax, double ay, double carrier,
  double delta, double* d, Totals* totals, double normal) const {
  /* the differences of the axles' wheels that balance their differentials,
  each by the secant method from the last */
  Totals& t = *totals;
  for(int a = 0; a < 2; ++a) {
    const double h = 1e-4 * std::max(std::fabs(carrier), 1.0);
    double d0 = d[a], d1 = d[a] + h;
    double f0, f1;
    {
      double y[2] = {d[0], d[1]};
      y[a] = d0;
      Wheels(vx, vy, r, ax, ay, carrier, y, delta, &t, nullptr, normal);
      f0 = Differential(t, a);
      y[a] = d1;
      Wheels(vx, vy, r, ax, ay, carrier, y, delta, &t, nullptr, normal);
      f1 = Differential(t, a);
    }
    for(int k = 0; k < 30 && std::fabs(f1) > 1e-12 && f1 != f0; ++k) {
      const double d2 = d1 - f1 * (d1 - d0) / (f1 - f0);
      double y[2] = {d[0], d[1]};
      y[a] = d2;
      Wheels(vx, vy, r, ax, ay, carrier, y, delta, &t, nullptr, normal);
      d0 = d1;
      f0 = f1;
      d1 = d2;
      f1 = Differential(t, a);
    }
    d[a] = d1;
  }
  Wheels(vx, vy, r, ax, ay, carrier, d, delta, &t, nullptr, normal);
}

void shift::core::controller::SlideEquilibrium::PathDerivatives(
  const double* s, double delta, double torque, double curvature, double* ds,
  double* differences, const double* slope) const {
  Derivatives(s, delta, torque, ds, differences, slope);
  const double vx = s[0], vy = s[1], v2 = vx * vx + vy * vy;
  const double v = std::sqrt(v2);
  /* the lateral error grows with the course from the path's direction, which
  turns at the yaw rate and the sideslip's rate less the path's */
  ds[4] = v * std::sin(s[5]);
  ds[5] = s[2] + (vx * ds[1] - vy * ds[0]) / v2 - v * curvature;
}

double shift::core::controller::SlideEquilibrium::FrontUse(
  const State& state) const {
  double most = 0.0;
  for(int i = 0; i < 2; ++i) {
    const double limit = tire.Friction(state.load[i]) * state.load[i];
    if(limit > 0.0) {
      most = std::max(most,
                      std::hypot(state.force_x[i], state.force_y[i]) / limit);
    }
  }
  return most;
}

void shift::core::controller::SlideEquilibrium::Linearize(
  double beta, const State& state, double* a, double* b) const {
  constexpr int n = kPathStates;
  const double vx = state.speed * std::cos(beta);
  const double curvature = state.yaw_rate / state.speed;
  const double s0[n] = {vx,
                        state.speed * std::sin(beta),
                        state.yaw_rate,
                        (1.0 + state.spin) * vx / tire.unloaded_radius,
                        0.0,
                        0.0};
  const double u0[kInputs] = {state.steering, state.drive_torque};
  const double step[n + kInputs] = {1e-4, 1e-4, 1e-5, 1e-3,
                                   1e-4, 1e-5, 1e-5, 1e-1};
  double differences[2] = {
    state.front_difference * vx / tire.unloaded_radius,
    state.rear_difference * vx / tire.unloaded_radius};
  for(int k = 0; k < n + kInputs; ++k) {
    double sp[n], sm[n];
    std::copy(s0, s0 + n, sp);
    std::copy(s0, s0 + n, sm);
    double up[kInputs] = {u0[0], u0[1]}, um[kInputs] = {u0[0], u0[1]};
    if(k < n) {
      sp[k] += step[k];
      sm[k] -= step[k];
    } else {
      up[k - n] += step[k];
      um[k - n] -= step[k];
    }
    double fp[n], fm[n];
    double dp[2] = {differences[0], differences[1]};
    double dm[2] = {differences[0], differences[1]};
    PathDerivatives(sp, up[0], up[1], curvature, fp, dp);
    PathDerivatives(sm, um[0], um[1], curvature, fm, dm);
    for(int i = 0; i < n; ++i) {
      const double g = (fp[i] - fm[i]) / (2.0 * step[k]);
      if(k < n) {
        a[i * n + k] = g;
      } else {
        b[i * kInputs + (k - n)] = g;
      }
    }
  }
}

void shift::core::controller::SlideEquilibrium::Discretize(
  const double* a, const double* b, double dt, double* ad, double* bd) {
  constexpr int n = kPathStates, m = kInputs;
  /* the system held over dt, by the series of the exponential of [A B; 0
  0] dt */
  constexpr int z = n + m;
  double e[z * z] = {}, term[z * z] = {}, next[z * z];
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      term[i * z + j] = a[i * n + j] * dt;
    }
    for(int j = 0; j < m; ++j) {
      term[i * z + n + j] = b[i * m + j] * dt;
    }
  }
  double power[z * z];
  std::copy(term, term + z * z, power);
  for(int i = 0; i < z; ++i) {
    e[i * z + i] = 1.0;
  }
  for(int order = 1; order < 30; ++order) {
    for(int i = 0; i < z * z; ++i) {
      e[i] += power[i];
    }
    for(int i = 0; i < z; ++i) {
      for(int j = 0; j < z; ++j) {
        double s = 0.0;
        for(int l = 0; l < z; ++l) {
          s += power[i * z + l] * term[l * z + j];
        }
        next[i * z + j] = s / (order + 1);
      }
    }
    std::copy(next, next + z * z, power);
  }
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      ad[i * n + j] = e[i * z + j];
    }
    for(int j = 0; j < m; ++j) {
      bd[i * m + j] = e[i * z + n + j];
    }
  }
}

bool shift::core::controller::SlideEquilibrium::DiscreteRegulator(
  int n, int m, const double* ad, const double* bd, const double* qd,
  const double* rd, double* k, double* coupling) {
  /* the discrete Riccati equation by the structure-preserving doubling
  algorithm: A_0 = A_d, G_0 = B_d R_d^-1 B_d', H_0 = Q_d, W = I + G H,
  A' = A W^-1 A, G' = G + A W^-1 G A', H' = H + A' H W^-1 A, H converging
  to P */
  auto multiply = [](int rows, int inner, int cols, const double* x,
                     const double* y, double* out) {
    for(int i = 0; i < rows; ++i) {
      for(int j = 0; j < cols; ++j) {
        double sum = 0.0;
        for(int l = 0; l < inner; ++l) {
          sum += x[i * inner + l] * y[l * cols + j];
        }
        out[i * cols + j] = sum;
      }
    }
  };
  auto transpose = [](int rows, int cols, const double* x, double* out) {
    for(int i = 0; i < rows; ++i) {
      for(int j = 0; j < cols; ++j) {
        out[j * rows + i] = x[i * cols + j];
      }
    }
  };
  /* x = w^-1 y for the size x size w and the size x cols y */
  auto divide = [](int size, int cols, const double* w, const double* y,
                   double* x) {
    std::vector<double> wc(size * size), col(size);
    for(int j = 0; j < cols; ++j) {
      std::copy(w, w + size * size, wc.begin());
      for(int i = 0; i < size; ++i) {
        col[i] = y[i * cols + j];
      }
      if(!SolveLinear(size, wc.data(), col.data())) {
        return false;
      }
      for(int i = 0; i < size; ++i) {
        x[i * cols + j] = col[i];
      }
    }
    return true;
  };
  const int nn = n * n;
  std::vector<double> identity(m * m, 0.0), rinv(m * m);
  for(int i = 0; i < m; ++i) {
    identity[i * m + i] = 1.0;
  }
  if(!divide(m, m, rd, identity.data(), rinv.data())) {
    return false;
  }
  std::vector<double> ak(ad, ad + nn), gk(nn), hk(qd, qd + nn);
  std::vector<double> bt(m * n), rb(m * n);
  transpose(n, m, bd, bt.data());
  multiply(m, m, n, rinv.data(), bt.data(), rb.data());
  multiply(n, m, n, bd, rb.data(), gk.data());
  std::vector<double> w(nn), wa(nn), wg(nn), at(nn), a1(nn), g1(nn), h1(nn),
    t1(nn);
  bool converged = false;
  for(int iteration = 0; iteration < 100 && !converged; ++iteration) {
    multiply(n, n, n, gk.data(), hk.data(), w.data());
    for(int i = 0; i < n; ++i) {
      w[i * n + i] += 1.0;
    }
    if(!divide(n, n, w.data(), ak.data(), wa.data()) ||
       !divide(n, n, w.data(), gk.data(), wg.data())) {
      return false;
    }
    transpose(n, n, ak.data(), at.data());
    multiply(n, n, n, ak.data(), wa.data(), a1.data());
    multiply(n, n, n, ak.data(), wg.data(), t1.data());
    multiply(n, n, n, t1.data(), at.data(), g1.data());
    multiply(n, n, n, hk.data(), wa.data(), t1.data());
    multiply(n, n, n, at.data(), t1.data(), h1.data());
    double change = 0.0, size = 0.0;
    for(int i = 0; i < nn; ++i) {
      g1[i] += gk[i];
      h1[i] += hk[i];
      change = std::max(change, std::fabs(h1[i] - hk[i]));
      size = std::max(size, std::fabs(h1[i]));
    }
    ak.swap(a1);
    gk.swap(g1);
    hk.swap(h1);
    if(!std::isfinite(change)) {
      return false;
    }
    converged = change <= 1e-13 * size;
  }
  if(!converged) {
    return false;
  }
  /* K = (R_d + B_d' P B_d)^-1 B_d' P A_d */
  std::vector<double> btp(m * n), g(m * m), h(m * n);
  multiply(m, n, n, bt.data(), hk.data(), btp.data());
  multiply(m, n, m, btp.data(), bd, g.data());
  multiply(m, n, n, btp.data(), ad, h.data());
  for(int i = 0; i < m * m; ++i) {
    g[i] += rd[i];
  }
  if(coupling) {
    *coupling = g[1 * m + 0] / g[1 * m + 1];
  }
  return divide(m, n, g.data(), h.data(), k);
}

bool shift::core::controller::SlideEquilibrium::Regulator(
  const double* a, const double* b, const double* q, const double* r,
  double dt, double* k, double* coupling) {
  constexpr int n = kPathStates, m = kInputs;
  double ad[n * n], bd[n * m], qd[n * n], rd[m * m];
  Discretize(a, b, dt, ad, bd);
  for(int i = 0; i < n * n; ++i) {
    qd[i] = q[i] * dt;
  }
  for(int i = 0; i < m * m; ++i) {
    rd[i] = r[i] * dt;
  }
  return DiscreteRegulator(n, m, ad, bd, qd, rd, k, coupling);
}

bool shift::core::controller::SlideEquilibrium::RateRegulator(
  const double* a, const double* b, const double* q, const double* r,
  const double* s, double dt, double* k, double* coupling) {
  constexpr int n = kPathStates, m = kInputs, na = n + m;
  double ad[n * n], bd[n * m];
  Discretize(a, b, dt, ad, bd);
  /* the state with the last inputs, x' = [x; u_-], the input their change:
  x+ = A_d x + B_d (u_- + du), u = u_- + du */
  double aa[na * na] = {}, ba[na * m] = {}, qa[na * na] = {}, ra[m * m];
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      aa[i * na + j] = ad[i * n + j];
      qa[i * na + j] = q[i * n + j] * dt;
    }
    for(int j = 0; j < m; ++j) {
      aa[i * na + n + j] = bd[i * m + j];
      ba[i * m + j] = bd[i * m + j];
    }
  }
  for(int i = 0; i < m; ++i) {
    aa[(n + i) * na + n + i] = 1.0;
    ba[(n + i) * m + i] = 1.0;
    for(int j = 0; j < m; ++j) {
      qa[(n + i) * na + n + j] = r[i * m + j] * dt;
      /* the rate's cost over the step, (du / dt)' S (du / dt) dt */
      ra[i * m + j] = s[i * m + j] / dt;
    }
  }
  return DiscreteRegulator(na, m, aa, ba, qa, ra, k, coupling);
}

bool shift::core::controller::SlideEquilibrium::SolveLinear(int n, double* a,
                                                            double* x) {
  /* Gaussian elimination with partial pivoting, a row major */
  for(int col = 0; col < n; ++col) {
    int pivot = col;
    for(int i = col + 1; i < n; ++i) {
      if(std::fabs(a[i * n + col]) > std::fabs(a[pivot * n + col])) {
        pivot = i;
      }
    }
    if(!(std::fabs(a[pivot * n + col]) > 1e-300)) {
      return false;
    }
    if(pivot != col) {
      for(int k = 0; k < n; ++k) {
        std::swap(a[col * n + k], a[pivot * n + k]);
      }
      std::swap(x[col], x[pivot]);
    }
    for(int i = col + 1; i < n; ++i) {
      const double f = a[i * n + col] / a[col * n + col];
      for(int k = col; k < n; ++k) {
        a[i * n + k] -= f * a[col * n + k];
      }
      x[i] -= f * x[col];
    }
  }
  for(int i = n - 1; i >= 0; --i) {
    double s = x[i];
    for(int k = i + 1; k < n; ++k) {
      s -= a[i * n + k] * x[k];
    }
    x[i] = s / a[i * n + i];
  }
  return true;
}
