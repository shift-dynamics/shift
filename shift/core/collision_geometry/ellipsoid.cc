#include "shift/core/collision_geometry/ellipsoid.h"

#include <algorithm>
#include <cmath>

#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"

bool shift::core::collision_geometry::Ellipsoid::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("ellipsoid",
                     shift::core::collision_geometry::Ellipsoid::Create);

std::unique_ptr<shift::core::collision_geometry::Ellipsoid>
shift::core::collision_geometry::Ellipsoid::Create() {
  return std::make_unique<shift::core::collision_geometry::Ellipsoid>();
}

void shift::core::collision_geometry::Ellipsoid::DeclareParameters() {
  DeclareParameter("tolerance", &tolerance_,
                   "convergence tolerance of the minimum volume bounding "
                   "ellipsoid fit, defaults to 0.001",
                   false);
  DeclareParameter("semi_axes", &semi_axes_,
                   "semi axis lengths [a, b, c] along the x, y and z axes of "
                   "the pose, centered at its origin (the body origin by "
                   "default); if not given, the ellipsoid is fitted to the "
                   "body's mesh",
                   false);
}

bool shift::core::collision_geometry::Ellipsoid::HasDimensions() const {
  const auto& a = semi_axes_;
  if(a.x == 0.0 && a.y == 0.0 && a.z == 0.0) {
    return false;
  }
  if(!(a.x > 0.0 && a.y > 0.0 && a.z > 0.0)) {
    throw shift::core::Exception("Ellipsoid collision geometry '" + name_ +
                                 "' needs three positive semi axes");
  }
  return true;
}

void shift::core::collision_geometry::Ellipsoid::SetFromDimensions(
  const ksl_SE3_t& pose) {
  d_wrt_body_ = pose;
  bounding_radius_ = std::max({semi_axes_.x, semi_axes_.y, semi_axes_.z});
}


/*! \brief in place Cholesky factorization of a symmetric positive definite
 * 4x4 matrix in row major order, the lower triangle is overwritten with L */
static bool Cholesky4(double X[16]) {
  for(int j = 0; j < 4; j++) {
    double d = X[j * 4 + j];
    for(int k = 0; k < j; k++) {
      d -= X[j * 4 + k] * X[j * 4 + k];
    }
    if(!(d > 0.0)) {
      return false;
    }
    X[j * 4 + j] = std::sqrt(d);
    for(int i = j + 1; i < 4; i++) {
      double s = X[i * 4 + j];
      for(int k = 0; k < j; k++) {
        s -= X[i * 4 + k] * X[j * 4 + k];
      }
      X[i * 4 + j] = s / X[j * 4 + j];
    }
  }
  return true;
}


/*!
Khachiyan's algorithm finds the ellipse equation in center form

  (x - c)^T * A * (x - c) = 1

by solving the optimization problem

  minimize log(det(A^-1)) subject to (p_i - c)^T * A * (p_i - c) <= 1

The points p_i are lifted to homogeneous coordinates q_i = [p_i; 1] and a
weight u_i is iterated for each point; the center and shape follow as

  c = P * u,   A = (1/3) * inv(P * diag(u) * P^T - c * c^T)

The principal axes of A^-1 = 3 * (P * diag(u) * P^T - c * c^T) give the
orientation of the ellipsoid and the square roots of its eigenvalues the semi
axis lengths.
*/
void shift::core::collision_geometry::Ellipsoid::Fit(
  const std::vector<ksl_vec3_t>& vertices, const std::vector<unsigned int>&) {
  const size_t n = vertices.size();
  ksl_vec3_t& c = d_wrt_body_.t;
  ksl_mat3x3_t& R = d_wrt_body_.R;
  if(n == 1) {
    c = vertices[0];
    semi_axes_ = {{0.0, 0.0, 0.0}};
    bounding_radius_ = 0.0;
    return;
  }

  std::vector<double> u(n, 1.0 / n);
  std::vector<double> m(n, 0.0);
  bool converged = false;
  double error = 1.0;
  for(int count = 0; count < 10000 && error > tolerance_; count++) {
    /* X = Q * diag(u) * Q^T, Q = [P; 1] */
    double X[16] = {0.0};
    for(size_t k = 0; k < n; k++) {
      const double q[4] = {vertices[k].x, vertices[k].y, vertices[k].z, 1.0};
      for(int i = 0; i < 4; i++) {
        for(int j = 0; j <= i; j++) {
          X[i * 4 + j] += q[i] * u[k] * q[j];
        }
      }
    }
    if(!Cholesky4(X)) {
      break;
    }

    /* m = diag(Q^T * inv(X) * Q), with L * y = q_k, m_k = y^T * y */
    size_t j_max = 0;
    for(size_t k = 0; k < n; k++) {
      const double q[4] = {vertices[k].x, vertices[k].y, vertices[k].z, 1.0};
      double y[4];
      m[k] = 0.0;
      for(int i = 0; i < 4; i++) {
        double s = q[i];
        for(int l = 0; l < i; l++) {
          s -= X[i * 4 + l] * y[l];
        }
        y[i] = s / X[i * 4 + i];
        m[k] += y[i] * y[i];
      }
      if(m[k] > m[j_max]) {
        j_max = k;
      }
    }

    const double maximum = m[j_max];
    const double step_size = (maximum - 4.0) / (4.0 * (maximum - 1.0));
    error = 0.0;
    for(size_t k = 0; k < n; k++) {
      const double u_new =
        (1.0 - step_size) * u[k] + (k == j_max ? step_size : 0.0);
      error += (u_new - u[k]) * (u_new - u[k]);
      u[k] = u_new;
    }
    error = std::sqrt(error);
    converged = error <= tolerance_;
  }

  if(converged) {
    /* c = P * u */
    c = {{0.0, 0.0, 0.0}};
    for(size_t k = 0; k < n; k++) {
      for(int i = 0; i < 3; i++) {
        c.at[i] += vertices[k].at[i] * u[k];
      }
    }
    /* S = 3 * (P * diag(u) * P^T - c * c^T) = inv(A) */
    ksl_mat3x3_t S = {{{0, 0, 0, 0, 0, 0, 0, 0, 0}}};
    for(size_t k = 0; k < n; k++) {
      for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
          S.at[j][i] += vertices[k].at[i] * u[k] * vertices[k].at[j];
        }
      }
    }
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        S.at[j][i] = 3.0 * (S.at[j][i] - c.at[i] * c.at[j]);
      }
    }
    Jacobi(S, R);
    for(int i = 0; i < 3; i++) {
      semi_axes_.at[i] = std::sqrt(std::max(S.at[i][i], 0.0));
    }
  } else {
    /* degenerate, e.g. coplanar, points: bound the box spanned by the points
    along their principal axes; an ellipsoid with semi axes sqrt(3) times the
    box half extents contains the box */
    SHIFT_WARNING("Minimum bounding ellipsoid of collision geometry '" +
                  name_ +
                  "' did not converge, using the ellipsoid bounding the "
                  "principal axis box instead");
    ksl_mat3x3_t covariance = Covariance(vertices);
    Jacobi(covariance, R);
    ksl_vec3_t lo = {{HUGE_VAL, HUGE_VAL, HUGE_VAL}};
    ksl_vec3_t hi = {{-HUGE_VAL, -HUGE_VAL, -HUGE_VAL}};
    for(const auto& p : vertices) {
      for(int i = 0; i < 3; i++) {
        const double projection = ksl_dot_vv(&R.v[i], &p);
        lo.at[i] = std::min(lo.at[i], projection);
        hi.at[i] = std::max(hi.at[i], projection);
      }
    }
    c = {{0.0, 0.0, 0.0}};
    for(int i = 0; i < 3; i++) {
      const double mid = 0.5 * (lo.at[i] + hi.at[i]);
      semi_axes_.at[i] = std::sqrt(3.0) * 0.5 * (hi.at[i] - lo.at[i]);
      for(int k = 0; k < 3; k++) {
        c.at[k] += mid * R.v[i].at[k];
      }
    }
  }
  bounding_radius_ =
    std::max({semi_axes_.at[0], semi_axes_.at[1], semi_axes_.at[2]});
  SHIFT_INFO("Ellipsoid '" + name_ + "' semi axes: " +
              std::to_string(semi_axes_.at[0]) + ", " +
              std::to_string(semi_axes_.at[1]) + ", " +
              std::to_string(semi_axes_.at[2]));
}

void shift::core::collision_geometry::Ellipsoid::CopyFit(const Base& other) {
  const auto& ellipsoid = dynamic_cast<const Ellipsoid&>(other);
  semi_axes_ = ellipsoid.semi_axes_;
  d_wrt_body_ = ellipsoid.d_wrt_body_;
  bounding_radius_ = ellipsoid.bounding_radius_;
}

/*!
For a direction u in the ellipsoid frame and S = diag(a, b, c), the extreme
point is S^2 * u / |S * u|. This is the closed form of the parametric (theta,
phi) evaluation in the original Shift implementation.
*/
void shift::core::collision_geometry::Ellipsoid::Support(
  const ksl_vec3_t& direction, ksl_vec3_t& local, ksl_vec3_t& global,
  int&) const {
  const ksl_mat3x3_t& R = d_absolute_.R;
  double su[3];
  double norm2 = 0.0;
  for(int i = 0; i < 3; i++) {
    su[i] = semi_axes_.at[i] * ksl_dot_vv(&R.v[i], &direction);
    norm2 += su[i] * su[i];
  }
  const double k = norm2 > 0.0 ? 1.0 / std::sqrt(norm2) : 0.0;
  for(int i = 0; i < 3; i++) {
    local.at[i] = semi_axes_.at[i] * su[i] * k;
  }
  ksl_product_dv(&d_absolute_, &local, &global);
}
