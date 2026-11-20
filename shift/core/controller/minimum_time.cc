#include "shift/core/controller/minimum_time.h"

#include <algorithm>
#include <cmath>

/* the scales of the states and the inputs: the steps of the central
differences, and the regularization of the inputs' Hessian */
static constexpr double kStateScale[shift::core::controller::MinimumTime::
                                      kStates] = {10.0, 2.0, 0.5, 1.0,
                                                  0.2,  0.1, 30.0};
static constexpr double kInputScale[shift::core::controller::MinimumTime::
                                      kInputs] = {0.05, 2.0};
static constexpr double kDifference = 1.0e-4; //!< of the scales

double shift::core::controller::MinimumTime::TorqueLimit(double omega) const {
  if(limits_.empty()) {
    return 0.0;
  }
  const double f = std::max(omega, 0.0) / kLimitStep;
  const size_t i = std::min(static_cast<size_t>(f), limits_.size() - 2);
  const double w = std::min(f - static_cast<double>(i), 1.0);
  return (1.0 - w) * limits_[i] + w * limits_[i + 1];
}

void shift::core::controller::MinimumTime::Rates(int sample, const double* x,
                                                 const double* u, double* dx,
                                                 double* dt,
                                                 double* torque) const {
  const double kappa = curvatures_[sample];
  /* gravity on the road in the car's frame, turned from the road's by the
  heading from it, the course less the sideslip */
  const double* road = &slopes_[3 * sample];
  const double psi = x[4] - std::atan2(x[1], x[0]);
  const double c = std::cos(psi), sn = std::sin(psi);
  const double gravity[3] = {road[0] * c + road[1] * sn,
                             -road[0] * sn + road[1] * c, road[2]};
  /* the motion in time, with the carriers at their speed and no torque of
  the drive: the rate of the carriers' speed is then that of the tires'
  torque on them, which the drive gives */
  const double s[SlideEquilibrium::kPathStates] = {x[0], x[1], x[2],
                                                   x[6], x[3], x[4]};
  double ds[SlideEquilibrium::kPathStates];
  car.PathDerivatives(s, x[5], 0.0, kappa, ds, nullptr, gravity);
  const double v = std::hypot(x[0], x[1]);
  const double s_dot =
    std::max(v * std::cos(x[4]) / std::max(1.0 - kappa * x[3], 0.1), 0.1);
  dx[0] = ds[0] / s_dot;
  dx[1] = ds[1] / s_dot;
  dx[2] = ds[2] / s_dot;
  dx[3] = ds[4] / s_dot;
  dx[4] = ds[5] / s_dot;
  dx[5] = u[0];
  dx[6] = u[1];
  *dt = 1.0 / s_dot;
  *torque = -car.drive_inertia * ds[3];
}

void shift::core::controller::MinimumTime::Step(int k, const double* x,
                                                const double* u, double* next,
                                                double* time,
                                                double* c) const {
  const double h = step;
  double k1[kStates], k2[kStates], k3[kStates], k4[kStates], y[kStates];
  double g1, g2, g3, g4, tau, unused;
  Rates(2 * k, x, u, k1, &g1, &tau);
  for(int i = 0; i < kStates; ++i) {
    y[i] = x[i] + 0.5 * h * k1[i];
  }
  Rates(2 * k + 1, y, u, k2, &g2, &unused);
  for(int i = 0; i < kStates; ++i) {
    y[i] = x[i] + 0.5 * h * k2[i];
  }
  Rates(2 * k + 1, y, u, k3, &g3, &unused);
  for(int i = 0; i < kStates; ++i) {
    y[i] = x[i] + h * k3[i];
  }
  Rates(2 * k + 2, y, u, k4, &g4, &unused);
  for(int i = 0; i < kStates; ++i) {
    next[i] = x[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  *time = h / 6.0 * (g1 + 2.0 * g2 + 2.0 * g3 + g4);
  if(c == nullptr) {
    return;
  }
  /* the corners on the road, by the heading's angle to it */
  const double psi = x[4] - std::atan2(x[1], x[0]);
  const double sp = std::sin(psi), cp = std::cos(psi);
  int j = 0;
  for(int a = 0; a < 2; ++a) {
    for(int b = -1; b <= 1; b += 2) {
      const double p = x[3] + outline[a] * sp + b * outline[2] * cp;
      c[j] = p - half_width;
      c[j + 4] = -p - half_width;
      ++j;
    }
  }
  c[8] = x[5] - steering_limit;
  c[9] = -x[5] - steering_limit;
  c[10] = 1.0e-3 * (tau - TorqueLimit(x[6]));
  c[11] = 1.0e-3 * (-tau - brake_torque);
  c[12] = min_speed - x[0];
  c[13] = x[4] - max_course;
  c[14] = -x[4] - max_course;
  const double rate = u[0] / g1;
  c[15] = rate - steering_rate;
  c[16] = -rate - steering_rate;
  c[17] = -car.tire.unloaded_radius * x[6];
}

void shift::core::controller::MinimumTime::EndConstraints(const double* x,
                                                          double* c) const {
  const double psi = x[4] - std::atan2(x[1], x[0]);
  const double sp = std::sin(psi), cp = std::cos(psi);
  int j = 0;
  for(int a = 0; a < 2; ++a) {
    for(int b = -1; b <= 1; b += 2) {
      const double p = x[3] + outline[a] * sp + b * outline[2] * cp;
      c[j] = p - half_width;
      c[j + 4] = -p - half_width;
      ++j;
    }
  }
  c[8] = x[3] - end_lateral;
  c[9] = -x[3] - end_lateral;
  c[10] = psi - end_heading;
  c[11] = -psi - end_heading;
  c[12] = x[2] - end_yaw_rate;
  c[13] = -x[2] - end_yaw_rate;
}

double
shift::core::controller::MinimumTime::InputCost(const double* u) const {
  return step *
         (rate_weights[0] * u[0] * u[0] + rate_weights[1] * u[1] * u[1]);
}

void shift::core::controller::MinimumTime::Guess() {
  /* along the middle, steered to the road's curvature ahead and back to the
  middle, the carriers at the speed of guess_speed */
  const double radius = car.tire.unloaded_radius;
  states_.assign((n_ + 1) * kStates, 0.0);
  inputs_.assign(n_ * kInputs, 0.0);
  constraints_.assign(n_ * kConstraints + kEndConstraints, 0.0);
  step_times_.assign(n_, 0.0);
  double* x = states_.data();
  x[0] = start_speed;
  x[3] = start_lateral;
  x[6] = start_speed / radius;
  for(int k = 0; k < n_; ++k) {
    const double station = start + k * step;
    const double ahead = curvature ? curvature(station + 3.0) : 0.0;
    const double steer =
      std::clamp(std::atan(car.wheelbase * ahead) - 0.1 * x[3] - 1.0 * x[4],
                 -steering_limit, steering_limit);
    const double v = std::max(std::hypot(x[0], x[1]), min_speed);
    double* u = &inputs_[k * kInputs];
    const double most = steering_rate / v;
    u[0] = std::clamp((steer - x[5]) / 1.0, -most, most);
    const double target =
      guess_speed ? guess_speed(station) : std::max(start_speed, min_speed);
    u[1] = (target / radius - x[6]) / 2.0;
    Step(k, x, u, x + kStates, &step_times_[k], &constraints_[k * kConstraints]);
    x += kStates;
  }
  EndConstraints(x, &constraints_[n_ * kConstraints]);
}

void shift::core::controller::MinimumTime::Linearize() {
  constexpr int nz = kStates + kInputs;
  a_.assign(n_ * kStates * kStates, 0.0);
  b_.assign(n_ * kStates * kInputs, 0.0);
  time_x_.assign(n_ * kStates, 0.0);
  time_u_.assign(n_ * kInputs, 0.0);
  time_xx_.assign(n_ * kStates, 0.0);
  time_uu_.assign(n_ * kInputs, 0.0);
  c_x_.assign(n_ * kConstraints * kStates, 0.0);
  c_u_.assign(n_ * kConstraints * kInputs, 0.0);
#pragma omp parallel for schedule(dynamic, 8)
  for(int k = 0; k < n_; ++k) {
    double z[nz];
    std::copy_n(&states_[k * kStates], kStates, z);
    std::copy_n(&inputs_[k * kInputs], kInputs, z + kStates);
    const double t0 = step_times_[k];
    for(int i = 0; i < nz; ++i) {
      const double e =
        kDifference * (i < kStates ? kStateScale[i] : kInputScale[i - kStates]);
      double zp[nz], zm[nz];
      std::copy_n(z, nz, zp);
      std::copy_n(z, nz, zm);
      zp[i] += e;
      zm[i] -= e;
      double np[kStates], nm[kStates], tp, tm, cp[kConstraints],
        cm[kConstraints];
      Step(k, zp, zp + kStates, np, &tp, cp);
      Step(k, zm, zm + kStates, nm, &tm, cm);
      const double d = 0.5 / e;
      const double hessian = std::max((tp - 2.0 * t0 + tm) / (e * e), 0.0);
      if(i < kStates) {
        for(int r = 0; r < kStates; ++r) {
          a_[(k * kStates + r) * kStates + i] = (np[r] - nm[r]) * d;
        }
        time_x_[k * kStates + i] = (tp - tm) * d;
        time_xx_[k * kStates + i] = hessian;
        for(int r = 0; r < kConstraints; ++r) {
          c_x_[(k * kConstraints + r) * kStates + i] = (cp[r] - cm[r]) * d;
        }
      } else {
        const int j = i - kStates;
        for(int r = 0; r < kStates; ++r) {
          b_[(k * kStates + r) * kInputs + j] = (np[r] - nm[r]) * d;
        }
        time_u_[k * kInputs + j] = (tp - tm) * d;
        time_uu_[k * kInputs + j] = hessian;
        for(int r = 0; r < kConstraints; ++r) {
          c_u_[(k * kConstraints + r) * kInputs + j] = (cp[r] - cm[r]) * d;
        }
      }
    }
  }
  /* the end's constraints */
  end_c_x_.assign(kEndConstraints * kStates, 0.0);
  const double* x = &states_[n_ * kStates];
  for(int i = 0; i < kStates; ++i) {
    const double e = kDifference * kStateScale[i];
    double xp[kStates], xm[kStates], cp[kEndConstraints], cm[kEndConstraints];
    std::copy_n(x, kStates, xp);
    std::copy_n(x, kStates, xm);
    xp[i] += e;
    xm[i] -= e;
    EndConstraints(xp, cp);
    EndConstraints(xm, cm);
    for(int r = 0; r < kEndConstraints; ++r) {
      end_c_x_[r * kStates + i] = (cp[r] - cm[r]) * 0.5 / e;
    }
  }
}

/* the weight of a constraint in the augmented cost: the penalty while it is
violated or its multiplier holds it */
static double Active(double c, double lambda, double mu) {
  return c > 0.0 || lambda > 0.0 ? mu : 0.0;
}

bool shift::core::controller::MinimumTime::Backward(double rho,
                                                    double* expected_1,
                                                    double* expected_2) {
  constexpr int n = kStates, m = kInputs;
  double vx[n] = {}, vxx[n * n] = {};
  /* the end's augmented cost */
  const double* lambda_end = &multipliers_[n_ * kConstraints];
  const double* c_end = &constraints_[n_ * kConstraints];
  for(int j = 0; j < kEndConstraints; ++j) {
    const double w = Active(c_end[j], lambda_end[j], penalty_);
    const double g = lambda_end[j] + w * c_end[j];
    const double* row = &end_c_x_[j * n];
    for(int a = 0; a < n; ++a) {
      vx[a] += row[a] * g;
      for(int b = 0; b < n; ++b) {
        vxx[a * n + b] += w * row[a] * row[b];
      }
    }
  }
  gain_k_.assign(n_ * m, 0.0);
  gain_kk_.assign(n_ * m * n, 0.0);
  *expected_1 = *expected_2 = 0.0;
  for(int k = n_ - 1; k >= 0; --k) {
    const double* a = &a_[k * n * n];
    const double* b = &b_[k * n * m];
    const double* u = &inputs_[k * m];
    double qx[n], qu[m], qxx[n * n], quu[m * m], qux[m * n];
    for(int i = 0; i < n; ++i) {
      qx[i] = time_x_[k * n + i];
    }
    qu[0] = time_u_[k * m] + 2.0 * step * rate_weights[0] * u[0];
    qu[1] = time_u_[k * m + 1] + 2.0 * step * rate_weights[1] * u[1];
    std::fill_n(qxx, n * n, 0.0);
    std::fill_n(quu, m * m, 0.0);
    std::fill_n(qux, m * n, 0.0);
    for(int i = 0; i < n; ++i) {
      qxx[i * n + i] = time_xx_[k * n + i];
    }
    quu[0] = time_uu_[k * m] + 2.0 * step * rate_weights[0];
    quu[3] = time_uu_[k * m + 1] + 2.0 * step * rate_weights[1];
    /* the constraints' terms */
    for(int j = 0; j < kConstraints; ++j) {
      const double c = constraints_[k * kConstraints + j];
      const double lambda = multipliers_[k * kConstraints + j];
      const double w = Active(c, lambda, penalty_);
      const double g = lambda + w * c;
      const double* cx = &c_x_[(k * kConstraints + j) * n];
      const double* cu = &c_u_[(k * kConstraints + j) * m];
      for(int i = 0; i < n; ++i) {
        qx[i] += cx[i] * g;
      }
      for(int i = 0; i < m; ++i) {
        qu[i] += cu[i] * g;
      }
      if(w > 0.0) {
        for(int i = 0; i < n; ++i) {
          for(int l = 0; l < n; ++l) {
            qxx[i * n + l] += w * cx[i] * cx[l];
          }
        }
        for(int i = 0; i < m; ++i) {
          for(int l = 0; l < m; ++l) {
            quu[i * m + l] += w * cu[i] * cu[l];
          }
          for(int l = 0; l < n; ++l) {
            qux[i * n + l] += w * cu[i] * cx[l];
          }
        }
      }
    }
    /* through the step: A' Vx, A' Vxx A, B' Vxx B, B' Vxx A */
    double va[n * n], vb[n * m];
    for(int i = 0; i < n; ++i) {
      for(int l = 0; l < n; ++l) {
        double s = 0.0;
        for(int p = 0; p < n; ++p) {
          s += vxx[i * n + p] * a[p * n + l];
        }
        va[i * n + l] = s;
      }
      for(int l = 0; l < m; ++l) {
        double s = 0.0;
        for(int p = 0; p < n; ++p) {
          s += vxx[i * n + p] * b[p * m + l];
        }
        vb[i * m + l] = s;
      }
    }
    for(int i = 0; i < n; ++i) {
      for(int p = 0; p < n; ++p) {
        qx[i] += a[p * n + i] * vx[p];
      }
      for(int l = 0; l < n; ++l) {
        double s = 0.0;
        for(int p = 0; p < n; ++p) {
          s += a[p * n + i] * va[p * n + l];
        }
        qxx[i * n + l] += s;
      }
    }
    for(int i = 0; i < m; ++i) {
      for(int p = 0; p < n; ++p) {
        qu[i] += b[p * m + i] * vx[p];
      }
      for(int l = 0; l < m; ++l) {
        double s = 0.0;
        for(int p = 0; p < n; ++p) {
          s += b[p * m + i] * vb[p * m + l];
        }
        quu[i * m + l] += s;
      }
      for(int l = 0; l < n; ++l) {
        double s = 0.0;
        for(int p = 0; p < n; ++p) {
          s += b[p * m + i] * va[p * n + l];
        }
        qux[i * n + l] += s;
      }
    }
    /* the inputs' step, with the Hessian regularized in the inputs' scales */
    const double h00 = quu[0] + rho / (kInputScale[0] * kInputScale[0]);
    const double h11 = quu[3] + rho / (kInputScale[1] * kInputScale[1]);
    const double h01 = 0.5 * (quu[1] + quu[2]);
    const double det = h00 * h11 - h01 * h01;
    if(!(h00 > 0.0) || !(det > 0.0)) {
      return false;
    }
    const double inv[4] = {h11 / det, -h01 / det, -h01 / det, h00 / det};
    double* kff = &gain_k_[k * m];
    double* kfb = &gain_kk_[k * m * n];
    for(int i = 0; i < m; ++i) {
      kff[i] = -(inv[i * 2] * qu[0] + inv[i * 2 + 1] * qu[1]);
      for(int l = 0; l < n; ++l) {
        kfb[i * n + l] = -(inv[i * 2] * qux[l] + inv[i * 2 + 1] * qux[n + l]);
      }
    }
    const double qs[4] = {quu[0], h01, h01, quu[3]};
    *expected_1 += kff[0] * qu[0] + kff[1] * qu[1];
    *expected_2 += 0.5 * (kff[0] * (qs[0] * kff[0] + qs[1] * kff[1]) +
                          kff[1] * (qs[2] * kff[0] + qs[3] * kff[1]));
    /* the cost to go */
    for(int i = 0; i < n; ++i) {
      double s = qx[i];
      for(int p = 0; p < m; ++p) {
        double quk = 0.0;
        for(int q = 0; q < m; ++q) {
          quk += qs[p * m + q] * kff[q];
        }
        s += kfb[p * n + i] * (quk + qu[p]) + qux[p * n + i] * kff[p];
      }
      vx[i] = s;
    }
    for(int i = 0; i < n; ++i) {
      for(int l = 0; l < n; ++l) {
        double s = qxx[i * n + l];
        for(int p = 0; p < m; ++p) {
          double qk = 0.0;
          for(int q = 0; q < m; ++q) {
            qk += qs[p * m + q] * kfb[q * n + l];
          }
          s += kfb[p * n + i] * qk + kfb[p * n + i] * qux[p * n + l] +
               qux[p * n + i] * kfb[p * n + l];
        }
        vxx[i * n + l] = s;
      }
    }
    for(int i = 0; i < n; ++i) {
      for(int l = 0; l < i; ++l) {
        const double s = 0.5 * (vxx[i * n + l] + vxx[l * n + i]);
        vxx[i * n + l] = vxx[l * n + i] = s;
      }
    }
  }
  return true;
}

double shift::core::controller::MinimumTime::Forward(
  double alpha, std::vector<double>* xs, std::vector<double>* us,
  std::vector<double>* cs, std::vector<double>* ts) const {
  xs->assign(states_.size(), 0.0);
  us->assign(inputs_.size(), 0.0);
  cs->assign(constraints_.size(), 0.0);
  ts->assign(step_times_.size(), 0.0);
  std::copy_n(states_.data(), kStates, xs->data());
  for(int k = 0; k < n_; ++k) {
    const double* x = &(*xs)[k * kStates];
    const double* xbar = &states_[k * kStates];
    double* u = &(*us)[k * kInputs];
    for(int i = 0; i < kInputs; ++i) {
      double s = inputs_[k * kInputs + i] + alpha * gain_k_[k * kInputs + i];
      for(int l = 0; l < kStates; ++l) {
        s += gain_kk_[(k * kInputs + i) * kStates + l] * (x[l] - xbar[l]);
      }
      u[i] = s;
    }
    Step(k, x, u, &(*xs)[(k + 1) * kStates], &(*ts)[k],
         &(*cs)[k * kConstraints]);
    if(!std::isfinite((*xs)[(k + 1) * kStates]) || (*xs)[(k + 1) * kStates] <= 0.0) {
      return INFINITY;
    }
  }
  EndConstraints(&(*xs)[n_ * kStates], &(*cs)[n_ * kConstraints]);
  return Cost(*xs, *us, *cs, *ts);
}

double shift::core::controller::MinimumTime::Cost(
  const std::vector<double>& xs, const std::vector<double>& us,
  const std::vector<double>& cs, const std::vector<double>& ts) const {
  (void)xs;
  double total = 0.0;
  for(int k = 0; k < n_; ++k) {
    total += ts[k] + InputCost(&us[k * kInputs]);
  }
  for(size_t j = 0; j < cs.size(); ++j) {
    const double w = Active(cs[j], multipliers_[j], penalty_);
    total += multipliers_[j] * cs[j] + 0.5 * w * cs[j] * cs[j];
  }
  return std::isfinite(total) ? total : INFINITY;
}

double shift::core::controller::MinimumTime::Violation(
  const std::vector<double>& cs) const {
  double most = 0.0;
  for(const double c : cs) {
    most = std::max(most, c);
  }
  return most;
}

void shift::core::controller::MinimumTime::Shooting::Stage(
  int k, const double* x, const double* u, double* next, double* cost,
  double* c) const {
  double time;
  plan_.Step(k, x, u, next, &time, c);
  *cost = time + plan_.InputCost(u);
}

void shift::core::controller::MinimumTime::Shooting::End(const double* x,
                                                         double* cost,
                                                         double* c) const {
  *cost = 0.0;
  plan_.EndConstraints(x, c);
}

double
shift::core::controller::MinimumTime::Shooting::StateScale(int i) const {
  return kStateScale[i];
}

double
shift::core::controller::MinimumTime::Shooting::InputScale(int i) const {
  return kInputScale[i];
}

bool shift::core::controller::MinimumTime::Load(
  const std::vector<double>& states) {
  Prepare();
  if(states.size() != static_cast<size_t>((n_ + 1) * kStates)) {
    return false;
  }
  states_ = states;
  inputs_.assign(n_ * kInputs, 0.0);
  constraints_.assign(n_ * kConstraints + kEndConstraints, 0.0);
  step_times_.assign(n_, 0.0);
  for(int k = 0; k < n_; ++k) {
    inputs_[k * kInputs] =
      (states_[(k + 1) * kStates + 5] - states_[k * kStates + 5]) / step;
    inputs_[k * kInputs + 1] =
      (states_[(k + 1) * kStates + 6] - states_[k * kStates + 6]) / step;
    double next[kStates];
    Step(k, &states_[k * kStates], &inputs_[k * kInputs], next,
         &step_times_[k], &constraints_[k * kConstraints]);
  }
  EndConstraints(&states_[n_ * kStates], &constraints_[n_ * kConstraints]);
  violation_ = Violation(constraints_);
  iterations_ = 0;
  Finish();
  return true;
}

void shift::core::controller::MinimumTime::Prepare() {
  n_ = std::max(static_cast<int>(std::lround((end - start) / step)), 1);
  /* the samples of the curvature and of the torque's limit, for the
  parallel steps */
  curvatures_.assign(2 * n_ + 1, 0.0);
  slopes_.assign(3 * (2 * n_ + 1), 0.0);
  for(int i = 0; i <= 2 * n_; ++i) {
    curvatures_[i] = curvature ? curvature(start + 0.5 * step * i) : 0.0;
    double* g = &slopes_[3 * i];
    g[0] = g[1] = 0.0;
    g[2] = car.gravity;
    if(slope) {
      slope(start + 0.5 * step * i, g);
    }
  }
  limits_.clear();
  for(double w = 0.0; w <= 1000.0; w += kLimitStep) {
    limits_.push_back(torque_limit ? torque_limit(w) : INFINITY);
  }
  /* the tires' sinkage by load from samples, which on soft soil is most of
  the work of a step */
  car.TabulateDeflection(1.5 * car.mass * car.gravity, 2.0);
  /* the slip ratio smooth through no slip, for the Newton steps */
  car.slip_smoothing = 0.2;
}

bool shift::core::controller::MinimumTime::Solve() {
  Prepare();
  Guess();
  if(interior_point) {
    int first = 0;
    if(warm_start > 0) {
      SolveIlqr(warm_start);
      first = iterations_;
    }
    InteriorPoint solver;
    solver.max_iterations = max_iterations;
    solver.constraint_tolerance = tolerance;
    solver.verbose = verbose;
    solver.gauss_newton = gauss_newton;
    solver.project = project;
    const Shooting problem(*this);
    solver.Solve(problem, states_, inputs_);
    states_ = solver.get_states();
    inputs_ = solver.get_inputs();
    iterations_ = first + solver.get_iterations();
    /* the run's times and constraints, and its violation with the steps'
    defects */
    for(int k = 0; k < n_; ++k) {
      double next[kStates];
      Step(k, &states_[k * kStates], &inputs_[k * kInputs], next,
           &step_times_[k], &constraints_[k * kConstraints]);
    }
    EndConstraints(&states_[n_ * kStates], &constraints_[n_ * kConstraints]);
    violation_ = solver.get_violation();
  } else if(!SolveIlqr(max_iterations)) {
    violation_ = Violation(constraints_);
  }
  Finish();
  return violation_ < tolerance;
}

void shift::core::controller::MinimumTime::Finish() {
  /* the times and the torques at the stations */
  times_.assign(n_ + 1, 0.0);
  torques_.assign(n_ + 1, 0.0);
  for(int k = 0; k <= n_; ++k) {
    if(k > 0) {
      times_[k] = times_[k - 1] + step_times_[k - 1];
    }
    double dx[kStates], dt;
    const double zero[kInputs] = {0.0, 0.0};
    Rates(2 * k, &states_[k * kStates], zero, dx, &dt, &torques_[k]);
  }
}

bool shift::core::controller::MinimumTime::SolveIlqr(int iterations) {
  multipliers_.assign(n_ * kConstraints + kEndConstraints, 0.0);
  penalty_ = 10.0;
  std::vector<double> xs, us, cs, ts;
  double cost = Cost(states_, inputs_, constraints_, step_times_);
  double rho = 1.0e-4;
  int inner = 0;
  bool linear = false;
  violation_ = Violation(constraints_);
  for(iterations_ = 0; iterations_ < iterations; ++iterations_) {
    if(!linear) {
      Linearize();
      linear = true;
    }
    double e1, e2;
    bool accepted = false;
    double improvement = 0.0;
    if(Backward(rho, &e1, &e2)) {
      for(double alpha = 1.0; alpha > 1.0e-3; alpha *= 0.5) {
        const double next = Forward(alpha, &xs, &us, &cs, &ts);
        const double expected = -(alpha * e1 + alpha * alpha * e2);
        if(next < cost && (cost - next) > 1.0e-4 * expected) {
          improvement = cost - next;
          cost = next;
          states_.swap(xs);
          inputs_.swap(us);
          constraints_.swap(cs);
          step_times_.swap(ts);
          accepted = true;
          linear = false;
          break;
        }
      }
    }
    rho = accepted ? std::max(rho / 3.0, 1.0e-8) : rho * 10.0;
    ++inner;
    const bool settled = (accepted && improvement < 1.0e-6 * std::fabs(cost)) ||
                         rho > 1.0e6 || inner >= 40;
    if(!settled) {
      continue;
    }
    /* the multipliers and the penalty */
    violation_ = Violation(constraints_);
    if(violation_ < tolerance && (accepted || rho > 1.0e6)) {
      break;
    }
    for(size_t j = 0; j < multipliers_.size(); ++j) {
      multipliers_[j] =
        std::max(0.0, multipliers_[j] + penalty_ * constraints_[j]);
    }
    penalty_ = std::min(10.0 * penalty_, 1.0e8);
    cost = Cost(states_, inputs_, constraints_, step_times_);
    rho = 1.0e-4;
    inner = 0;
  }
  violation_ = Violation(constraints_);
  return violation_ < tolerance;
}

bool shift::core::controller::MinimumTime::TrackingGains(
  const double* scales, std::vector<double>* gains) {
  constexpr int n = kTrackStates, m = kTrackInputs;
  if(n_ < 1 || states_.size() != static_cast<size_t>((n_ + 1) * kStates)) {
    return false;
  }
  Linearize();
  double q[n], r[m];
  for(int i = 0; i < n; ++i) {
    q[i] = step / (scales[i] * scales[i]);
  }
  for(int i = 0; i < m; ++i) {
    r[i] = step / (scales[n + i] * scales[n + i]);
  }
  gains->assign((n_ + 1) * m * n, 0.0);
  double p[n * n] = {};
  for(int i = 0; i < n; ++i) {
    p[i * n + i] = q[i];
  }
  for(int k = n_ - 1; k >= 0; --k) {
    /* the step's linearization in the tracked states, its inputs the
    steering's change and the carriers' speed, held over the step */
    double a[n * n], b[n * m];
    const double* full = &a_[k * kStates * kStates];
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < n; ++j) {
        a[i * n + j] = full[i * kStates + j];
      }
      b[i * m] = b_[(k * kStates + i) * kInputs];
      b[i * m + 1] = full[i * kStates + 6];
    }
    double pa[n * n], pb[n * m];
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < n; ++j) {
        double s = 0.0;
        for(int l = 0; l < n; ++l) {
          s += p[i * n + l] * a[l * n + j];
        }
        pa[i * n + j] = s;
      }
      for(int j = 0; j < m; ++j) {
        double s = 0.0;
        for(int l = 0; l < n; ++l) {
          s += p[i * n + l] * b[l * m + j];
        }
        pb[i * m + j] = s;
      }
    }
    /* S = R + B' P B, K = S^-1 B' P A */
    double sm[m * m], bpa[m * n];
    for(int i = 0; i < m; ++i) {
      for(int j = 0; j < m; ++j) {
        double s = i == j ? r[i] : 0.0;
        for(int l = 0; l < n; ++l) {
          s += b[l * m + i] * pb[l * m + j];
        }
        sm[i * m + j] = s;
      }
      for(int j = 0; j < n; ++j) {
        double s = 0.0;
        for(int l = 0; l < n; ++l) {
          s += b[l * m + i] * pa[l * n + j];
        }
        bpa[i * n + j] = s;
      }
    }
    const double det = sm[0] * sm[3] - sm[1] * sm[2];
    if(!(det > 0.0) || !(sm[0] > 0.0)) {
      return false;
    }
    const double inv[4] = {sm[3] / det, -sm[1] / det, -sm[2] / det,
                           sm[0] / det};
    double* kk = &(*gains)[k * m * n];
    for(int i = 0; i < m; ++i) {
      for(int j = 0; j < n; ++j) {
        kk[i * n + j] = inv[i * 2] * bpa[j] + inv[i * 2 + 1] * bpa[n + j];
      }
    }
    /* P = Q + A' P A - (B' P A)' K */
    double next[n * n];
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < n; ++j) {
        double s = i == j ? q[i] : 0.0;
        for(int l = 0; l < n; ++l) {
          s += a[l * n + i] * pa[l * n + j];
        }
        for(int l = 0; l < m; ++l) {
          s -= bpa[l * n + i] * kk[l * n + j];
        }
        next[i * n + j] = s;
      }
    }
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < n; ++j) {
        p[i * n + j] = 0.5 * (next[i * n + j] + next[j * n + i]);
      }
    }
  }
  /* the last station's, the step's before it */
  std::copy_n(&(*gains)[(n_ - 1) * m * n], m * n, &(*gains)[n_ * m * n]);
  return true;
}
