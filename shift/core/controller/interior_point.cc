#include "shift/core/controller/interior_point.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

/* the constants of Wächter and Biegler (2006), their Table 1 */
static constexpr double kGammaTheta = 1.0e-5;
static constexpr double kGammaPhi = 1.0e-5;
static constexpr double kSwitching = 1.0;     //!< delta of the switching rule
static constexpr double kSwitchTheta = 1.1;   //!< s_theta
static constexpr double kSwitchPhi = 2.3;     //!< s_phi
static constexpr double kArmijo = 1.0e-4;     //!< eta_phi
static constexpr double kBarrierError = 10.0; //!< kappa_epsilon
static constexpr double kBarrierFactor = 0.2; //!< kappa_mu
static constexpr double kBarrierPower = 1.5;  //!< theta_mu
static constexpr double kSigma = 1.0e10;      //!< kappa_Sigma
static constexpr double kScaleMax = 100.0;    //!< s_max
static constexpr int kBacktracks = 30;
static constexpr int kAcceptable = 15; //!< IPOPT's acceptable_iter

/* small dense algebra, row major */
static void Multiply(int r, int k, int c, const double* a, const double* b,
                     double* out) {
  for(int i = 0; i < r; ++i) {
    for(int j = 0; j < c; ++j) {
      double s = 0.0;
      for(int l = 0; l < k; ++l) {
        s += a[i * k + l] * b[l * c + j];
      }
      out[i * c + j] = s;
    }
  }
}

/* a' b, a (k x r), b (k x c) */
static void MultiplyTransposed(int r, int k, int c, const double* a,
                               const double* b, double* out) {
  for(int i = 0; i < r; ++i) {
    for(int j = 0; j < c; ++j) {
      double s = 0.0;
      for(int l = 0; l < k; ++l) {
        s += a[l * r + i] * b[l * c + j];
      }
      out[i * c + j] = s;
    }
  }
}

/* the Cholesky factor of a (n x n) in its lower triangle; false if it is
not positive definite */
static bool Cholesky(int n, double* a) {
  for(int j = 0; j < n; ++j) {
    double d = a[j * n + j];
    for(int l = 0; l < j; ++l) {
      d -= a[j * n + l] * a[j * n + l];
    }
    if(!(d > 0.0)) {
      return false;
    }
    d = std::sqrt(d);
    a[j * n + j] = d;
    for(int i = j + 1; i < n; ++i) {
      double s = a[i * n + j];
      for(int l = 0; l < j; ++l) {
        s -= a[i * n + l] * a[j * n + l];
      }
      a[i * n + j] = s / d;
    }
  }
  return true;
}

/* solves l l' x = b in place for the factor of Cholesky */
static void CholeskySolve(int n, const double* l, double* b) {
  for(int i = 0; i < n; ++i) {
    double s = b[i];
    for(int j = 0; j < i; ++j) {
      s -= l[i * n + j] * b[j];
    }
    b[i] = s / l[i * n + i];
  }
  for(int i = n - 1; i >= 0; --i) {
    double s = b[i];
    for(int j = i + 1; j < n; ++j) {
      s -= l[j * n + i] * b[j];
    }
    b[i] = s / l[i * n + i];
  }
}

/* a (n x n, symmetric) with its eigenvalues below floor raised to it, by
the cyclic Jacobi method */
static void Project(int n, double* a, double floor) {
  std::vector<double> v(n * n, 0.0), d(a, a + n * n);
  for(int i = 0; i < n; ++i) {
    v[i * n + i] = 1.0;
  }
  for(int sweep = 0; sweep < 50; ++sweep) {
    double off = 0.0, diagonal = 0.0;
    for(int i = 0; i < n; ++i) {
      diagonal += d[i * n + i] * d[i * n + i];
      for(int j = i + 1; j < n; ++j) {
        off += d[i * n + j] * d[i * n + j];
      }
    }
    if(off <= 1.0e-24 * diagonal || off == 0.0) {
      break;
    }
    for(int p = 0; p < n; ++p) {
      for(int q = p + 1; q < n; ++q) {
        const double apq = d[p * n + q];
        if(apq == 0.0) {
          continue;
        }
        const double theta = (d[q * n + q] - d[p * n + p]) / (2.0 * apq);
        const double t = (theta >= 0.0 ? 1.0 : -1.0) /
                         (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for(int k = 0; k < n; ++k) {
          const double dkp = d[k * n + p], dkq = d[k * n + q];
          d[k * n + p] = c * dkp - s * dkq;
          d[k * n + q] = s * dkp + c * dkq;
        }
        for(int k = 0; k < n; ++k) {
          const double dpk = d[p * n + k], dqk = d[q * n + k];
          d[p * n + k] = c * dpk - s * dqk;
          d[q * n + k] = s * dpk + c * dqk;
        }
        for(int k = 0; k < n; ++k) {
          const double vkp = v[k * n + p], vkq = v[k * n + q];
          v[k * n + p] = c * vkp - s * vkq;
          v[k * n + q] = s * vkp + c * vkq;
        }
      }
    }
  }
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      double s = 0.0;
      for(int k = 0; k < n; ++k) {
        s += v[i * n + k] * std::max(d[k * n + k], floor) * v[j * n + k];
      }
      a[i * n + j] = s;
    }
  }
}

void shift::core::controller::InteriorPoint::Evaluate(
  const Problem& p, const std::vector<double>& x, const std::vector<double>& u,
  std::vector<double>* next, std::vector<double>* costs,
  std::vector<double>* c) const {
  next->assign(steps_ * n_, 0.0);
  costs->assign(steps_ + 1, 0.0);
  c->assign(steps_ * mc_ + me_, 0.0);
#pragma omp parallel for schedule(dynamic, 8)
  for(int k = 0; k < steps_; ++k) {
    p.Stage(k, x.data() + k * n_, u.data() + k * m_, next->data() + k * n_,
            costs->data() + k, c->data() + k * mc_);
  }
  p.End(x.data() + steps_ * n_, costs->data() + steps_,
        c->data() + steps_ * mc_);
}

void shift::core::controller::InteriorPoint::Linearize(const Problem& p) {
  const int n = n_, m = m_, nz = n + m;
  a_.assign(steps_ * n * n, 0.0);
  b_.assign(steps_ * n * m, 0.0);
  g_.assign(steps_ * nz + n, 0.0);
  cj_.assign(steps_ * mc_ * nz + me_ * n, 0.0);
  h_.assign(steps_ * nz * nz + n * n, 0.0);
#pragma omp parallel for schedule(dynamic, 4)
  for(int k = 0; k <= steps_; ++k) {
    const bool end = k == steps_;
    const int nv = end ? n : nz;
    const int nc = end ? me_ : mc_;
    std::vector<double> w(nv), wp(nv), next(n), c(nc);
    std::copy_n(x_.data() + (k * n), n, w.data());
    if(!end) {
      std::copy_n(u_.data() + (k * m), m, w.data() + n);
    }
    const double* lambda = end ? nullptr : lambda_.data() + ((k + 1) * n);
    const double* z = z_.data() + (k * mc_);
    double* gk = g_.data() + (k * nz);
    double* ck = cj_.data() + (k * mc_ * nz);
    double* hk = h_.data() + (k * nz * nz);
    /* the cost, the Lagrangian, the step and the constraints at w */
    auto eval = [&](const double* v, double* cost, double* lagrangian,
                    double* nx, double* cc) {
      double l;
      if(end) {
        p.End(v, &l, cc);
      } else {
        p.Stage(k, v, v + n, nx, &l, cc);
      }
      double s = l;
      if(!end) {
        for(int i = 0; i < n; ++i) {
          s += lambda[i] * nx[i];
        }
      }
      for(int i = 0; i < nc; ++i) {
        s += z[i] * cc[i];
      }
      *cost = l;
      *lagrangian = s;
    };
    double l0, f0;
    eval(w.data(), &l0, &f0, next.data(), c.data());
    std::vector<double> step(nv), plus(nv), lp(nv);
    std::vector<double> np(n), nm(n), cp(nc), cm(nc);
    for(int i = 0; i < nv; ++i) {
      step[i] =
        difference * (i < n ? p.StateScale(i) : p.InputScale(i - n));
      wp = w;
      wp[i] = w[i] + step[i];
      double lplus, fplus, lminus, fminus;
      eval(wp.data(), &lplus, &fplus, np.data(), cp.data());
      wp[i] = w[i] - step[i];
      eval(wp.data(), &lminus, &fminus, nm.data(), cm.data());
      const double d = 0.5 / step[i];
      gk[i] = (lplus - lminus) * d;
      if(!end) {
        for(int r = 0; r < n; ++r) {
          const double v = (np[r] - nm[r]) * d;
          if(i < n) {
            a_[(k * n + r) * n + i] = v;
          } else {
            b_[(k * n + r) * m + i - n] = v;
          }
        }
      }
      for(int r = 0; r < nc; ++r) {
        ck[r * nv + i] = (cp[r] - cm[r]) * d;
      }
      const double e2 = step[i] * step[i];
      hk[i * nv + i] = gauss_newton
                         ? std::max((lplus - 2.0 * l0 + lminus) / e2, 0.0)
                         : (fplus - 2.0 * f0 + fminus) / e2;
      plus[i] = fplus;
      lp[i] = lplus;
    }
    if(!gauss_newton) {
      for(int i = 0; i < nv; ++i) {
        for(int j = i + 1; j < nv; ++j) {
          wp = w;
          wp[i] += step[i];
          wp[j] += step[j];
          double lij, fij;
          eval(wp.data(), &lij, &fij, np.data(), cp.data());
          const double v = (fij - plus[i] - plus[j] + f0) / (step[i] * step[j]);
          hk[i * nv + j] = hk[j * nv + i] = v;
        }
      }
    }
    if(project && !gauss_newton) {
      /* in the scales, made positive, and back */
      for(int i = 0; i < nv; ++i) {
        for(int j = 0; j < nv; ++j) {
          hk[i * nv + j] *= step[i] * step[j] / (difference * difference);
        }
      }
      Project(nv, hk, min_curvature);
      for(int i = 0; i < nv; ++i) {
        for(int j = 0; j < nv; ++j) {
          hk[i * nv + j] *= difference * difference / (step[i] * step[j]);
        }
      }
    }
  }
}

bool shift::core::controller::InteriorPoint::Newton(double delta) {
  const int n = n_, m = m_, nz = n + m;
  std::vector<double> kk(steps_ * m * n), kf(steps_ * m);
  std::vector<double> pp((steps_ + 1) * n * n), pv((steps_ + 1) * n);
  std::vector<double> ht(nz * nz), qt(nz), r(std::max(mc_, me_)),
    sigma(std::max(mc_, me_));
  /* the stage's Hessian and gradient with the constraints' terms */
  auto reduce = [&](int k, int nv, int nc) {
    const double* hk = h_.data() + (k * nz * nz);
    const double* gk = g_.data() + (k * nz);
    const double* ck = cj_.data() + (k * mc_ * nz);
    const int o = k * mc_;
    for(int i = 0; i < nc; ++i) {
      sigma[i] = z_[o + i] / s_[o + i];
      r[i] = (mu_ + z_[o + i] * (c_[o + i] + s_[o + i])) / s_[o + i];
    }
    for(int i = 0; i < nv; ++i) {
      double q = gk[i];
      for(int l = 0; l < nc; ++l) {
        q += ck[l * nv + i] * r[l];
      }
      qt[i] = q;
      for(int j = 0; j < nv; ++j) {
        double h = hk[i * nv + j];
        for(int l = 0; l < nc; ++l) {
          h += ck[l * nv + i] * sigma[l] * ck[l * nv + j];
        }
        ht[i * nv + j] = h;
      }
      const double scale = i < n ? state_scales_[i] : input_scales_[i - n];
      ht[i * nv + i] += delta / (scale * scale);
    }
  };
  /* the end */
  reduce(steps_, n, me_);
  std::copy_n(ht.data(), n * n, pp.data() + (steps_ * n * n));
  std::copy_n(qt.data(), n, pv.data() + (steps_ * n));
  std::vector<double> pa(n * n), pb(n * m), pd(n), qxx(n * n), qux(m * n),
    quu(m * m), qx(n), qu(m), aux(n * n);
  for(int k = steps_ - 1; k >= 0; --k) {
    reduce(k, nz, mc_);
    const double* a = a_.data() + (k * n * n);
    const double* b = b_.data() + (k * n * m);
    const double* p = pp.data() + ((k + 1) * n * n);
    const double* pk = pv.data() + ((k + 1) * n);
    for(int i = 0; i < n; ++i) {
      double s = pk[i];
      for(int j = 0; j < n; ++j) {
        s += p[i * n + j] * (next_[k * n + j] - x_[(k + 1) * n + j]);
      }
      pd[i] = s;
    }
    Multiply(n, n, n, p, a, pa.data());
    Multiply(n, n, m, p, b, pb.data());
    MultiplyTransposed(n, n, n, a, pa.data(), qxx.data());
    MultiplyTransposed(m, n, n, b, pa.data(), qux.data());
    MultiplyTransposed(m, n, m, b, pb.data(), quu.data());
    MultiplyTransposed(n, n, 1, a, pd.data(), qx.data());
    MultiplyTransposed(m, n, 1, b, pd.data(), qu.data());
    for(int i = 0; i < n; ++i) {
      qx[i] += qt[i];
      for(int j = 0; j < n; ++j) {
        qxx[i * n + j] += ht[i * nz + j];
      }
    }
    for(int i = 0; i < m; ++i) {
      qu[i] += qt[n + i];
      for(int j = 0; j < n; ++j) {
        qux[i * n + j] += ht[(n + i) * nz + j];
      }
      for(int j = 0; j < m; ++j) {
        quu[i * m + j] += ht[(n + i) * nz + n + j];
      }
    }
    if(!Cholesky(m, quu.data())) {
      return false;
    }
    double* kfb = kk.data() + (k * m * n);
    double* kff = kf.data() + (k * m);
    std::vector<double> col(m);
    for(int j = 0; j < n; ++j) {
      for(int i = 0; i < m; ++i) {
        col[i] = -qux[i * n + j];
      }
      CholeskySolve(m, quu.data(), col.data());
      for(int i = 0; i < m; ++i) {
        kfb[i * n + j] = col[i];
      }
    }
    for(int i = 0; i < m; ++i) {
      kff[i] = -qu[i];
    }
    CholeskySolve(m, quu.data(), kff);
    /* P = Qxx + Qux' K, p = qx + Qux' k */
    double* pn = pp.data() + (k * n * n);
    double* pvn = pv.data() + (k * n);
    MultiplyTransposed(n, m, n, qux.data(), kfb, aux.data());
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < n; ++j) {
        pn[i * n + j] = qxx[i * n + j] + aux[i * n + j];
      }
      double s = qx[i];
      for(int l = 0; l < m; ++l) {
        s += qux[l * n + i] * kff[l];
      }
      pvn[i] = s;
    }
    for(int i = 0; i < n; ++i) {
      for(int j = 0; j < i; ++j) {
        const double v = 0.5 * (pn[i * n + j] + pn[j * n + i]);
        pn[i * n + j] = pn[j * n + i] = v;
      }
    }
  }
  /* forward: the primal step and the multipliers */
  dx_.assign((steps_ + 1) * n, 0.0);
  du_.assign(steps_ * m, 0.0);
  lambda_new_.assign((steps_ + 1) * n, 0.0);
  for(int k = 0; k < steps_; ++k) {
    const double* dx = dx_.data() + (k * n);
    double* du = du_.data() + (k * m);
    for(int i = 0; i < m; ++i) {
      double s = kf[k * m + i];
      for(int j = 0; j < n; ++j) {
        s += kk[(k * m + i) * n + j] * dx[j];
      }
      du[i] = s;
    }
    const double* a = a_.data() + (k * n * n);
    const double* b = b_.data() + (k * n * m);
    double* dn = dx_.data() + ((k + 1) * n);
    for(int i = 0; i < n; ++i) {
      double s = next_[k * n + i] - x_[(k + 1) * n + i];
      for(int j = 0; j < n; ++j) {
        s += a[i * n + j] * dx[j];
      }
      for(int j = 0; j < m; ++j) {
        s += b[i * m + j] * du[j];
      }
      dn[i] = s;
    }
  }
  for(int k = 1; k <= steps_; ++k) {
    for(int i = 0; i < n; ++i) {
      double s = pv[k * n + i];
      for(int j = 0; j < n; ++j) {
        s += pp[(k * n + i) * n + j] * dx_[k * n + j];
      }
      lambda_new_[k * n + i] = s;
    }
  }
  /* the slacks' and the constraints' multipliers' steps */
  ds_.assign(s_.size(), 0.0);
  dz_.assign(z_.size(), 0.0);
  for(int k = 0; k <= steps_; ++k) {
    const bool end = k == steps_;
    const int nv = end ? n : nz, nc = end ? me_ : mc_;
    const double* ck = cj_.data() + (k * mc_ * nz);
    for(int i = 0; i < nc; ++i) {
      const int o = k * mc_ + i;
      double cdw = 0.0;
      for(int j = 0; j < nv; ++j) {
        const double dw = j < n ? dx_[k * n + j] : du_[k * m + j - n];
        cdw += ck[i * nv + j] * dw;
      }
      const double rc = c_[o] + s_[o];
      ds_[o] = -rc - cdw;
      dz_[o] = (mu_ + z_[o] * rc + z_[o] * cdw) / s_[o] - z_[o];
    }
  }
  return true;
}

std::pair<double, double> shift::core::controller::InteriorPoint::Measure(
  const std::vector<double>& x, const std::vector<double>& next,
  const std::vector<double>& costs, const std::vector<double>& c,
  const std::vector<double>& s) const {
  double theta = 0.0, phi = 0.0;
  for(int k = 0; k < steps_; ++k) {
    for(int i = 0; i < n_; ++i) {
      theta += std::fabs(next[k * n_ + i] - x[(k + 1) * n_ + i]) /
               state_scales_[i];
    }
  }
  for(size_t i = 0; i < c.size(); ++i) {
    theta += std::fabs(c[i] + s[i]);
    phi -= mu_ * std::log(s[i]);
  }
  for(const double l : costs) {
    phi += l;
  }
  return {theta, phi};
}

double shift::core::controller::InteriorPoint::Error(double mu) const {
  const int n = n_, m = m_, nz = n + m;
  double stationarity = 0.0, feasibility = 0.0, complementarity = 0.0;
  double sum_lambda = 0.0, sum_z = 0.0;
  for(double v : lambda_) {
    sum_lambda += std::fabs(v);
  }
  for(double v : z_) {
    sum_z += std::fabs(v);
  }
  for(int k = 0; k <= steps_; ++k) {
    const bool end = k == steps_;
    const int nv = end ? n : nz, nc = end ? me_ : mc_;
    const double* ck = cj_.data() + (k * mc_ * nz);
    for(int i = k == 0 ? n : 0; i < nv; ++i) {
      double gl = g_[k * nz + i];
      if(i < n) {
        gl -= lambda_[k * n + i];
      }
      if(!end) {
        for(int r = 0; r < n; ++r) {
          const double j = i < n ? a_[(k * n + r) * n + i]
                                 : b_[(k * n + r) * m + i - n];
          gl += j * lambda_[(k + 1) * n + r];
        }
      }
      for(int r = 0; r < nc; ++r) {
        gl += ck[r * nv + i] * z_[k * mc_ + r];
      }
      const double scale = i < n ? state_scales_[i] : input_scales_[i - n];
      stationarity = std::max(stationarity, std::fabs(gl) * scale);
    }
    if(!end) {
      for(int i = 0; i < n; ++i) {
        feasibility = std::max(feasibility,
                               std::fabs(next_[k * n + i] -
                                         x_[(k + 1) * n + i]) /
                                 state_scales_[i]);
      }
    }
    for(int i = 0; i < nc; ++i) {
      const int o = k * mc_ + i;
      feasibility = std::max(feasibility, std::fabs(c_[o] + s_[o]));
      complementarity =
        std::max(complementarity, std::fabs(s_[o] * z_[o] - mu));
    }
  }
  const double count = static_cast<double>(lambda_.size() + z_.size());
  const double sd =
    std::max(kScaleMax, (sum_lambda + sum_z) / count) / kScaleMax;
  const double sc =
    std::max(kScaleMax, sum_z / std::max<double>(z_.size(), 1)) / kScaleMax;
  return std::max({stationarity / sd, feasibility, complementarity / sc});
}

bool shift::core::controller::InteriorPoint::Solve(const Problem& p,
                                                   std::vector<double> xs,
                                                   std::vector<double> us) {
  n_ = p.get_states();
  m_ = p.get_inputs();
  steps_ = p.get_steps();
  mc_ = p.get_constraints();
  me_ = p.get_end_constraints();
  state_scales_.resize(n_);
  input_scales_.resize(m_);
  for(int i = 0; i < n_; ++i) {
    state_scales_[i] = p.StateScale(i);
  }
  for(int i = 0; i < m_; ++i) {
    input_scales_[i] = p.InputScale(i);
  }
  x_ = std::move(xs);
  u_ = std::move(us);
  Evaluate(p, x_, u_, &next_, &costs_, &c_);
  mu_ = initial_barrier;
  s_.resize(c_.size());
  z_.resize(c_.size());
  for(size_t i = 0; i < c_.size(); ++i) {
    s_[i] = std::max(-c_[i], 1.0e-2 * std::max(1.0, std::fabs(c_[i])));
    z_[i] = mu_ / s_[i];
  }
  lambda_.assign((steps_ + 1) * n_, 0.0);
  std::vector<std::pair<double, double>> filter;
  const double theta_0 = Measure(x_, next_, costs_, c_, s_).first;
  const double theta_max = max_violation * std::max(1.0, theta_0);
  const double theta_min = 1.0e-4 * std::max(1.0, theta_0);
  filter.push_back({theta_max, -INFINITY});
  double delta_last = 0.0;
  double proximal = min_proximal;
  int acceptable = 0; //!< iterations in a row within acceptable_tolerance
  std::vector<double> xt, ut, st, nt, lt, ct;
  bool converged = false;
  for(iterations_ = 0; iterations_ < max_iterations; ++iterations_) {
    Linearize(p);
    error_ = Error(0.0);
    double worst = 0.0;
    for(size_t i = 0; i < c_.size(); ++i) {
      worst = std::max(worst, c_[i]);
    }
    for(int k = 0; k < steps_; ++k) {
      for(int i = 0; i < n_; ++i) {
        worst = std::max(worst,
                         std::fabs(next_[k * n_ + i] - x_[(k + 1) * n_ + i]));
      }
    }
    violation_ = worst;
    if(violation_ <= constraint_tolerance &&
       (error_ <= tolerance ||
        (error_ <= acceptable_tolerance && ++acceptable >= kAcceptable))) {
      converged = true;
      break;
    }
    if(!(violation_ <= constraint_tolerance &&
         error_ <= acceptable_tolerance)) {
      acceptable = 0;
    }
    /* the barrier, lowered while its problem is solved well enough */
    while(mu_ > 0.1 * tolerance && Error(mu_) <= kBarrierError * mu_) {
      mu_ = std::max(0.1 * tolerance,
                     std::min(kBarrierFactor * mu_, std::pow(mu_, kBarrierPower)));
      filter.assign(1, {theta_max, -INFINITY});
    }
    /* the Newton step, the Hessian corrected to the right inertia */
    double delta = proximal;
    if(!Newton(delta)) {
      delta = std::max(proximal, delta_last == 0.0
                                   ? 1.0e-4
                                   : std::max(1.0e-20, delta_last / 3.0));
      while(!Newton(delta)) {
        delta *= delta_last == 0.0 ? 100.0 : 8.0;
        if(delta > 1.0e40) {
          return false;
        }
      }
      delta_last = delta;
    }
    /* the fraction to the boundary */
    const double tau = std::max(0.99, 1.0 - mu_);
    double alpha_max = 1.0, alpha_z = 1.0;
    for(size_t i = 0; i < s_.size(); ++i) {
      if(ds_[i] < 0.0) {
        alpha_max = std::min(alpha_max, -tau * s_[i] / ds_[i]);
      }
      if(dz_[i] < 0.0) {
        alpha_z = std::min(alpha_z, -tau * z_[i] / dz_[i]);
      }
    }
    /* the barrier cost's derivative along the step */
    double slope = 0.0;
    const int nz = n_ + m_;
    for(int k = 0; k <= steps_; ++k) {
      for(int i = 0; i < n_; ++i) {
        slope += g_[k * nz + i] * dx_[k * n_ + i];
      }
      if(k < steps_) {
        for(int i = 0; i < m_; ++i) {
          slope += g_[k * nz + n_ + i] * du_[k * m_ + i];
        }
      }
    }
    for(size_t i = 0; i < s_.size(); ++i) {
      slope -= mu_ * ds_[i] / s_[i];
    }
    const auto [theta, phi] = Measure(x_, next_, costs_, c_, s_);
    /* the filter line search */
    double alpha = alpha_max;
    bool accepted = false;
    for(int t = 0; t < kBacktracks; ++t, alpha *= 0.5) {
      xt = x_;
      ut = u_;
      st = s_;
      for(size_t i = n_; i < xt.size(); ++i) {
        xt[i] += alpha * dx_[i];
      }
      for(size_t i = 0; i < ut.size(); ++i) {
        ut[i] += alpha * du_[i];
      }
      for(size_t i = 0; i < st.size(); ++i) {
        st[i] += alpha * ds_[i];
      }
      Evaluate(p, xt, ut, &nt, &lt, &ct);
      /* the slacks no smaller than the constraints leave (the slack reset
      of R. H. Byrd, J. Nocedal and R. A. Waltz, KNITRO, 2006), and no
      greater where that keeps the share tau of the step's */
      for(size_t i = 0; i < st.size(); ++i) {
        st[i] = std::max(-ct[i], (1.0 - tau) * st[i]);
      }
      const auto [theta_t, phi_t] = Measure(xt, nt, lt, ct, st);
      if(!std::isfinite(phi_t) || !std::isfinite(theta_t) ||
         theta_t > theta_max) {
        continue;
      }
      bool blocked = false;
      for(const auto& f : filter) {
        if(theta_t >= f.first && phi_t >= f.second) {
          blocked = true;
          break;
        }
      }
      if(blocked) {
        continue;
      }
      const bool switching =
        theta <= theta_min && slope < 0.0 &&
        alpha * std::pow(-slope, kSwitchPhi) >
          kSwitching * std::pow(theta, kSwitchTheta);
      if(switching) {
        if(phi_t <= phi + kArmijo * alpha * slope) {
          accepted = true;
        }
      } else if(theta_t <= (1.0 - kGammaTheta) * theta ||
                phi_t <= phi - kGammaPhi * theta) {
        filter.push_back(
          {(1.0 - kGammaTheta) * theta, phi - kGammaPhi * theta});
        accepted = true;
      }
      if(accepted) {
        break;
      }
    }
    /* the proximal term: less after a full step, more after a short one */
    if(accepted && alpha >= 0.99 * alpha_max) {
      proximal = std::max(proximal / 3.0, min_proximal);
    } else if(!accepted || alpha < 0.3 * alpha_max) {
      proximal = std::min(std::max(5.0 * proximal, 1.0e-4), max_proximal);
    }
    if(!accepted) {
      /* no restoration phase: the shortest step, the filter forgotten */
      filter.assign(1, {theta_max, -INFINITY});
      alpha *= 2.0;
      const auto [theta_t, phi_t] = Measure(xt, nt, lt, ct, st);
      if(!std::isfinite(theta_t) || !std::isfinite(phi_t)) {
        return false;
      }
    }
    if(verbose) {
      double cost = 0.0;
      for(const double l : lt) {
        cost += l;
      }
      double defects = 0.0;
      for(int k = 0; k < steps_; ++k) {
        for(int i = 0; i < n_; ++i) {
          defects += std::fabs(nt[k * n_ + i] - xt[(k + 1) * n_ + i]) /
                     state_scales_[i];
        }
      }
      std::fprintf(stderr,
                   "%4d cost %12.6f theta %9.2e (steps %9.2e) error %9.2e "
                   "mu %8.1e delta %8.1e alpha %8.1e z %8.1e%s\n",
                   iterations_, cost, theta, defects, error_, mu_, delta,
                   alpha, alpha_z, accepted ? "" : " (forced)");
    }
    x_.swap(xt);
    u_.swap(ut);
    s_.swap(st);
    next_.swap(nt);
    costs_.swap(lt);
    c_.swap(ct);
    for(size_t i = n_; i < lambda_.size(); ++i) {
      lambda_[i] += alpha * (lambda_new_[i] - lambda_[i]);
    }
    for(size_t i = 0; i < z_.size(); ++i) {
      const double z = z_[i] + alpha_z * dz_[i];
      z_[i] = std::clamp(z, mu_ / (kSigma * s_[i]), kSigma * mu_ / s_[i]);
    }
  }
  cost_ = 0.0;
  for(const double l : costs_) {
    cost_ += l;
  }
  return converged;
}
