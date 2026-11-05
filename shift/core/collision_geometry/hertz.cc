#include "shift/core/collision_geometry/hertz.h"

#include <algorithm>
#include <cmath>

void shift::core::collision_geometry::CompleteEllipticIntegrals(
  double m, double complement, double& k, double& e, double& k_minus_e) {
  /* a_n, b_n to their common mean; c_n by c_{n+1} = c_n^2 / (4 a_{n+1}),
  free of the cancellation in (a_n - b_n) / 2. E = K (1 - S) with
  S = sum 2^(n-1) c_n^2 */
  double a = 1.0, b = std::sqrt(complement), c = std::sqrt(m);
  double power = 0.5, sum = 0.5 * m;
  for(int n = 0; n < 40; n++) {
    const double a_next = 0.5 * (a + b);
    b = std::sqrt(a * b);
    c = c * c / (4.0 * a_next);
    a = a_next;
    power *= 2.0;
    sum += power * c * c;
    if(c < 1e-17 * a) {
      break;
    }
  }
  k = M_PI / (2.0 * a);
  k_minus_e = k * sum;
  e = k - k_minus_e;
}

/* B / A of an ellipse of parameter m = e^2, with 1 - m given, and the
factor g of HertzEllipseFactor */
static double Ratio(double m, double complement, double& factor) {
  double k, e, k_minus_e;
  shift::core::collision_geometry::CompleteEllipticIntegrals(m, complement, k,
                                                             e, k_minus_e);
  /* (E / (1 - m) - K) / (K - E) = (m - S) / ((1 - m) S), S = (K - E) / K */
  const double s = k_minus_e / k;
  const double ratio = (m - s) / (complement * s);
  /* g = (2 pi / (3 K)) sqrt((K - E) / (K A m)) / (4/3 sqrt(1 / (A + B)))
  with A + B = A (1 + ratio) */
  factor = M_PI / (2.0 * k) * std::sqrt(s * (1.0 + ratio) / m);
  return ratio;
}

double shift::core::collision_geometry::HertzEllipseFactor(double ratio,
                                                          double* elongation) {
  if(elongation) {
    *elongation = 1.0;
  }
  if(!(ratio > 1.0 + 1e-12)) {
    return 1.0;
  }
  ratio = std::min(ratio, 1e6);
  /* B / A increases with m: bisection on log(1 - m), the axes' ratio
  (b / a)^2 */
  double lo = -40.0, hi = 0.0, factor = 1.0;
  for(int i = 0; i < 64; i++) {
    const double mid = 0.5 * (lo + hi);
    if(Ratio(-std::expm1(mid), std::exp(mid), factor) > ratio) {
      lo = mid; // too elongated
    } else {
      hi = mid;
    }
  }
  const double mid = 0.5 * (lo + hi);
  Ratio(-std::expm1(mid), std::exp(mid), factor);
  if(elongation) {
    *elongation = std::exp(-0.5 * mid); // a / b = 1 / sqrt(1 - m)
  }
  return factor;
}
