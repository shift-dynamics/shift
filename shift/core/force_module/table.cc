#include "shift/core/force_module/table.h"

#include <algorithm>
#include <cmath>

#include "shift/core/exception.h"

shift::core::force_module::Table::Table(const std::vector<double>& x,
                                        const std::vector<double>& y,
                                        Extrapolation extrapolation)
  : x_(x)
  , y_(y)
  , extrapolation_(extrapolation) {
  const size_t n = x_.size();
  if(n == 0 || n != y_.size()) {
    throw shift::core::Exception("Table: needs as many y as x values, and "
                                 "at least one point");
  }
  for(size_t k = 1; k < n; k++) {
    if(!(x_[k] > x_[k - 1])) {
      throw shift::core::Exception("Table: x values must increase");
    }
  }
  m_.assign(n, 0.0);
  if(n == 1) {
    return;
  }
  /* the slopes of the intervals; at each inner point, the weighted harmonic
  mean of its two intervals' slopes, or zero where the data turn or are
  level (Fritsch and Butland's form of the monotone slopes) */
  std::vector<double> h(n - 1), d(n - 1);
  for(size_t k = 0; k + 1 < n; k++) {
    h[k] = x_[k + 1] - x_[k];
    d[k] = (y_[k + 1] - y_[k]) / h[k];
  }
  for(size_t k = 1; k + 1 < n; k++) {
    if(d[k - 1] * d[k] > 0.0) {
      const double w1 = 2.0 * h[k] + h[k - 1];
      const double w2 = h[k] + 2.0 * h[k - 1];
      m_[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k]);
    }
  }
  /* the end slopes: of the end interval for two points; otherwise from the
  three-point formula, limited so that the curve does not overshoot */
  if(n == 2) {
    m_[0] = m_[1] = d[0];
    return;
  }
  auto end_slope = [](double h0, double h1, double d0, double d1) {
    double m = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
    if(m * d0 <= 0.0) {
      m = 0.0;
    } else if(d0 * d1 <= 0.0 && std::fabs(m) > 3.0 * std::fabs(d0)) {
      m = 3.0 * d0;
    }
    return m;
  };
  m_[0] = end_slope(h[0], h[1], d[0], d[1]);
  m_[n - 1] = end_slope(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
}

shift::core::force_module::Table shift::core::force_module::Table::FromPairs(
  const std::vector<double>& pairs, Extrapolation extrapolation,
  const std::string& what) {
  if(pairs.empty() || pairs.size() % 2 != 0) {
    throw shift::core::Exception(what + ": a table needs pairs of numbers, "
                                 "[x0, y0, x1, y1, ...]");
  }
  std::vector<double> x, y;
  for(size_t k = 0; k < pairs.size(); k += 2) {
    x.push_back(pairs[k]);
    y.push_back(pairs[k + 1]);
  }
  try {
    return Table(x, y, extrapolation);
  } catch(const shift::core::Exception& e) {
    throw shift::core::Exception(what + ": " + e.what());
  }
}

double shift::core::force_module::Table::Evaluate(double x,
                                                  double* slope) const {
  const size_t n = x_.size();
  if(n == 0) {
    if(slope) {
      *slope = 0.0;
    }
    return 0.0;
  }
  if(x <= x_.front() || x >= x_.back()) {
    const size_t k = x <= x_.front() ? 0 : n - 1;
    if(extrapolation_ == Extrapolation::kHold) {
      if(slope) {
        *slope = 0.0;
      }
      return y_[k];
    }
    if(slope) {
      *slope = m_[k];
    }
    return y_[k] + m_[k] * (x - x_[k]);
  }
  const size_t k =
    std::upper_bound(x_.begin(), x_.end(), x) - x_.begin() - 1;
  const double h = x_[k + 1] - x_[k];
  const double s = (x - x_[k]) / h;
  const double s2 = s * s, s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  if(slope) {
    const double d00 = (6.0 * s2 - 6.0 * s) / h;
    const double d10 = 3.0 * s2 - 4.0 * s + 1.0;
    const double d01 = (-6.0 * s2 + 6.0 * s) / h;
    const double d11 = 3.0 * s2 - 2.0 * s;
    *slope = d00 * y_[k] + d10 * m_[k] + d01 * y_[k + 1] + d11 * m_[k + 1];
  }
  return h00 * y_[k] + h10 * h * m_[k] + h01 * y_[k + 1] +
         h11 * h * m_[k + 1];
}
