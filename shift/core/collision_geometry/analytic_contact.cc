#include "shift/core/collision_geometry/analytic_contact.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"

static inline ksl_vec3_t Sub(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x - b.x, a.y - b.y, a.z - b.z}};
}

static inline ksl_vec3_t Add(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x + b.x, a.y + b.y, a.z + b.z}};
}

static inline ksl_vec3_t Scale(double k, const ksl_vec3_t& a) {
  return {{k * a.x, k * a.y, k * a.z}};
}

static inline double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*! coordinates of an inertial point in a frame, R^T * (p - t) */
static ksl_vec3_t ToLocal(const ksl_SE3_t& d, const ksl_vec3_t& p) {
  const ksl_vec3_t r = Sub(p, d.t);
  return {{Dot(d.R.v[0], r), Dot(d.R.v[1], r), Dot(d.R.v[2], r)}};
}

/*! R * v, a direction from a frame to the inertial frame */
static ksl_vec3_t Rotate(const ksl_SE3_t& d, const ksl_vec3_t& v) {
  return Add(Add(Scale(v.x, d.R.v[0]), Scale(v.y, d.R.v[1])),
             Scale(v.z, d.R.v[2]));
}

/*! fill the contact of a shape (first) and a sphere (second) from the
 * closest surface point q of the shape and its outward normal n, both in the
 * shape frame d, the distance of the sphere center from the surface
 * (negative inside) and the radius */
static void SetSphereContact(
  const ksl_SE3_t& d, const ksl_vec3_t& q, const ksl_vec3_t& n,
  double distance, const shift::core::collision_geometry::Sphere& s,
  shift::core::collision_geometry::AnalyticContact& c) {
  const double radius = s.get_radius();
  c.collision = distance < radius;
  if(!c.collision) {
    return;
  }
  c.depth = radius - distance;
  ksl_product_dv(&d, &q, &c.point_first);
  c.normal = Rotate(d, n);
  c.point_second = Sub(s.get_center(), Scale(radius, c.normal));
}

/*! closest point of triangle abc to p (Ericson, Real-Time Collision
 * Detection, 5.1.5) */
static ksl_vec3_t ClosestPointOnTriangle(const ksl_vec3_t& p,
                                         const ksl_vec3_t& a,
                                         const ksl_vec3_t& b,
                                         const ksl_vec3_t& c) {
  const ksl_vec3_t ab = Sub(b, a);
  const ksl_vec3_t ac = Sub(c, a);
  const ksl_vec3_t ap = Sub(p, a);
  const double d1 = Dot(ab, ap);
  const double d2 = Dot(ac, ap);
  if(d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }
  const ksl_vec3_t bp = Sub(p, b);
  const double d3 = Dot(ab, bp);
  const double d4 = Dot(ac, bp);
  if(d3 >= 0.0 && d4 <= d3) {
    return b;
  }
  const double vc = d1 * d4 - d3 * d2;
  if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return Add(a, Scale(d1 / (d1 - d3), ab));
  }
  const ksl_vec3_t cp = Sub(p, c);
  const double d5 = Dot(ab, cp);
  const double d6 = Dot(ac, cp);
  if(d6 >= 0.0 && d5 <= d6) {
    return c;
  }
  const double vb = d5 * d2 - d1 * d6;
  if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return Add(a, Scale(d2 / (d2 - d6), ac));
  }
  const double va = d3 * d6 - d5 * d4;
  if(va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
    return Add(b, Scale((d4 - d3) / ((d4 - d3) + (d5 - d6)), Sub(c, b)));
  }
  const double denominator = 1.0 / (va + vb + vc);
  return Add(a, Add(Scale(vb * denominator, ab), Scale(vc * denominator, ac)));
}

bool shift::core::collision_geometry::ConvexHullSphereContact(
  const ConvexHull& convex_hull, const Sphere& sphere,
  AnalyticContact& contact) {
  const ConvexHullData& hull = convex_hull.get_hull();
  if(hull.faces.empty() || hull.face_normals.size() * 3 != hull.faces.size()) {
    return false;
  }
  contact = AnalyticContact();
  const ksl_SE3_t& d = convex_hull.get_d_absolute();
  const ksl_vec3_t c = ToLocal(d, sphere.get_center());
  const double radius = sphere.get_radius();

  /* signed distances of the center in front of the face planes */
  size_t nearest = 0;
  double largest = -HUGE_VAL;
  for(size_t f = 0; f < hull.face_normals.size(); f++) {
    const double s = Dot(hull.face_normals[f], c) - hull.face_offsets[f];
    if(s >= radius) {
      return true; // a separating plane
    }
    if(s > largest) {
      largest = s;
      nearest = f;
    }
  }
  if(largest <= 0.0) {
    /* inside: the nearest boundary point is on the nearest face plane */
    const ksl_vec3_t& n = hull.face_normals[nearest];
    SetSphereContact(d, Sub(c, Scale(largest, n)), n, largest, sphere,
                     contact);
    return true;
  }
  /* outside: the closest point is on a face the center is in front of */
  double best = HUGE_VAL;
  ksl_vec3_t q = c;
  for(size_t f = 0; f < hull.face_normals.size(); f++) {
    if(Dot(hull.face_normals[f], c) - hull.face_offsets[f] <= 0.0) {
      continue;
    }
    const ksl_vec3_t p = ClosestPointOnTriangle(
      c, hull.vertices[hull.faces[3 * f]], hull.vertices[hull.faces[3 * f + 1]],
      hull.vertices[hull.faces[3 * f + 2]]);
    const ksl_vec3_t r = Sub(c, p);
    const double distance2 = Dot(r, r);
    if(distance2 < best) {
      best = distance2;
      q = p;
    }
  }
  const double distance = std::sqrt(best);
  /* distance >= largest > 0 */
  SetSphereContact(d, q, Scale(1.0 / distance, Sub(c, q)), distance, sphere,
                   contact);
  return true;
}

/* Eberly's bisection for the root s of
   sum_i (r_i z_i / (s + r_i))^2 - 1 = 0, r_last = 1, between s0 and s1 */
static double GetRoot(int count, const double* r, const double* z, double g) {
  double n[3];
  double length2 = 0.0;
  for(int i = 0; i < count; i++) {
    n[i] = r[i] * z[i];
    length2 += n[i] * n[i];
  }
  double s0 = z[count - 1] - 1.0;
  double s1 = g < 0.0 ? 0.0 : std::sqrt(length2) - 1.0;
  double s = 0.0;
  for(int iteration = 0; iteration < 2200; iteration++) {
    s = 0.5 * (s0 + s1);
    if(s == s0 || s == s1) {
      break;
    }
    double value = -1.0;
    for(int i = 0; i < count; i++) {
      const double ratio = n[i] / (s + r[i]);
      value += ratio * ratio;
    }
    if(value > 0.0) {
      s0 = s;
    } else if(value < 0.0) {
      s1 = s;
    } else {
      break;
    }
  }
  return s;
}

/* closest point x of the ellipse with semi axes e0 >= e1 > 0 to y, y >= 0 */
static void ClosestPointOnEllipse(double e0, double e1, double y0, double y1,
                                  double& x0, double& x1) {
  if(y1 > 0.0) {
    if(y0 > 0.0) {
      const double z[2] = {y0 / e0, y1 / e1};
      const double g = z[0] * z[0] + z[1] * z[1] - 1.0;
      if(g != 0.0) {
        const double r[2] = {(e0 / e1) * (e0 / e1), 1.0};
        const double s = GetRoot(2, r, z, g);
        x0 = r[0] * y0 / (s + r[0]);
        x1 = y1 / (s + 1.0);
      } else {
        x0 = y0;
        x1 = y1;
      }
    } else {
      x0 = 0.0;
      x1 = e1;
    }
  } else {
    const double numerator = e0 * y0;
    const double denominator = e0 * e0 - e1 * e1;
    if(numerator < denominator) {
      const double xde = numerator / denominator;
      x0 = e0 * xde;
      x1 = e1 * std::sqrt(1.0 - xde * xde);
    } else {
      x0 = e0;
      x1 = 0.0;
    }
  }
}

/* closest point x of the ellipsoid with semi axes e0 >= e1 >= e2 > 0 to y,
 * y >= 0 */
static void ClosestPointOnEllipsoid(const double e[3], const double y[3],
                                    double x[3]) {
  if(y[2] > 0.0) {
    if(y[1] > 0.0) {
      if(y[0] > 0.0) {
        const double z[3] = {y[0] / e[0], y[1] / e[1], y[2] / e[2]};
        const double g = z[0] * z[0] + z[1] * z[1] + z[2] * z[2] - 1.0;
        if(g != 0.0) {
          const double r[3] = {(e[0] / e[2]) * (e[0] / e[2]),
                               (e[1] / e[2]) * (e[1] / e[2]), 1.0};
          const double s = GetRoot(3, r, z, g);
          x[0] = r[0] * y[0] / (s + r[0]);
          x[1] = r[1] * y[1] / (s + r[1]);
          x[2] = y[2] / (s + 1.0);
        } else {
          x[0] = y[0];
          x[1] = y[1];
          x[2] = y[2];
        }
      } else {
        x[0] = 0.0;
        ClosestPointOnEllipse(e[1], e[2], y[1], y[2], x[1], x[2]);
      }
    } else {
      x[1] = 0.0;
      if(y[0] > 0.0) {
        ClosestPointOnEllipse(e[0], e[2], y[0], y[2], x[0], x[2]);
      } else {
        x[0] = 0.0;
        x[2] = e[2];
      }
    }
  } else {
    const double denominator0 = e[0] * e[0] - e[2] * e[2];
    const double denominator1 = e[1] * e[1] - e[2] * e[2];
    const double numerator0 = e[0] * y[0];
    const double numerator1 = e[1] * y[1];
    if(numerator0 < denominator0 && numerator1 < denominator1) {
      const double xde0 = numerator0 / denominator0;
      const double xde1 = numerator1 / denominator1;
      const double discriminant = 1.0 - xde0 * xde0 - xde1 * xde1;
      if(discriminant > 0.0) {
        x[0] = e[0] * xde0;
        x[1] = e[1] * xde1;
        x[2] = e[2] * std::sqrt(discriminant);
        return;
      }
    }
    x[2] = 0.0;
    ClosestPointOnEllipse(e[0], e[1], y[0], y[1], x[0], x[1]);
  }
}

/*! the point q of an ellipsoid's surface closest to the inertial point p,
 * the outward normal n there, both in the ellipsoid frame, and the signed
 * distance of p from the surface, negative inside */
void shift::core::collision_geometry::ClosestOnEllipsoid(
  const Ellipsoid& ellipsoid, const ksl_vec3_t& p, ksl_vec3_t& q,
  ksl_vec3_t& n, double& distance) {
  const ksl_vec3_t& axes = ellipsoid.get_semi_axes();
  const ksl_vec3_t c = ToLocal(ellipsoid.get_d_absolute(), p);

  /* reduce to the first octant with the semi axes in decreasing order */
  int order[3] = {0, 1, 2};
  std::sort(order, order + 3,
            [&](int i, int j) { return axes.at[i] > axes.at[j]; });
  double e[3], y[3], x[3];
  for(int k = 0; k < 3; k++) {
    e[k] = axes.at[order[k]];
    y[k] = std::fabs(c.at[order[k]]);
  }
  ClosestPointOnEllipsoid(e, y, x);
  double inside = 0.0;
  for(int k = 0; k < 3; k++) {
    const int i = order[k];
    q.at[i] = std::copysign(x[k], c.at[i]);
    inside += (c.at[i] / axes.at[i]) * (c.at[i] / axes.at[i]);
  }
  /* the outward normal at q is along the gradient of sum (x_i / a_i)^2 */
  n = {{q.x / (axes.x * axes.x), q.y / (axes.y * axes.y),
        q.z / (axes.z * axes.z)}};
  n = Scale(1.0 / std::sqrt(Dot(n, n)), n);
  const ksl_vec3_t r = Sub(c, q);
  distance = std::sqrt(Dot(r, r));
  if(inside < 1.0) {
    distance = -distance;
  }
}

bool shift::core::collision_geometry::EllipsoidSphereContact(
  const Ellipsoid& ellipsoid, const Sphere& sphere, AnalyticContact& contact) {
  const ksl_vec3_t& axes = ellipsoid.get_semi_axes();
  if(std::min({axes.x, axes.y, axes.z}) <= 0.0) {
    return false;
  }
  contact = AnalyticContact();
  ksl_vec3_t q, n;
  double distance;
  ClosestOnEllipsoid(ellipsoid, sphere.get_center(), q, n, distance);
  SetSphereContact(ellipsoid.get_d_absolute(), q, n, distance, sphere,
                   contact);
  return true;
}

/* symmetric 3 x 3 matrices, row major */
struct Sym3 {
  double m[3][3];
};

/*! the shape matrix G = R diag(a^2) R^T of an ellipsoid, inertial frame */
static Sym3 ShapeMatrix(const shift::core::collision_geometry::Ellipsoid& e) {
  Sym3 g = {};
  const ksl_mat3x3_t& R = e.get_d_absolute().R;
  const ksl_vec3_t& a = e.get_semi_axes();
  for(int k = 0; k < 3; k++) {
    const double a2 = a.at[k] * a.at[k];
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        g.m[i][j] += a2 * R.v[k].at[i] * R.v[k].at[j];
      }
    }
  }
  return g;
}

static ksl_vec3_t Multiply(const Sym3& g, const ksl_vec3_t& v) {
  ksl_vec3_t out;
  for(int i = 0; i < 3; i++) {
    out.at[i] = g.m[i][0] * v.x + g.m[i][1] * v.y + g.m[i][2] * v.z;
  }
  return out;
}

/*! solve M x = r for a symmetric positive definite M, by its cofactors */
static ksl_vec3_t Solve(const Sym3& M, const ksl_vec3_t& r) {
  const auto& m = M.m;
  double c[3][3];
  c[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
  c[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
  c[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];
  c[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
  c[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
  c[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
  c[2][0] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
  c[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
  c[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  const double det = m[0][0] * c[0][0] + m[0][1] * c[1][0] + m[0][2] * c[2][0];
  ksl_vec3_t x;
  for(int i = 0; i < 3; i++) {
    x.at[i] = (c[i][0] * r.x + c[i][1] * r.y + c[i][2] * r.z) / det;
  }
  return x;
}

/*! the translation along a unit direction n separating ellipsoids with shape
matrices ga and gb and centers r apart, h_a(n) + h_b(n) - n . r */
static double Separation(const Sym3& ga, const Sym3& gb, const ksl_vec3_t& r,
                         const ksl_vec3_t& n) {
  return std::sqrt(Dot(n, Multiply(ga, n))) +
         std::sqrt(Dot(n, Multiply(gb, n))) - Dot(n, r);
}

/*! minimize Separation on the unit sphere by Newton's method from n, which
is replaced by the local minimum; returns the separation there */
static double MinimizeSeparation(const Sym3& ga, const Sym3& gb,
                                 const ksl_vec3_t& r, double scale,
                                 ksl_vec3_t& n) {
  auto distance = [&](const ksl_vec3_t& u) {
    return Separation(ga, gb, r, u);
  };
  double depth = distance(n);
  for(int iteration = 0; iteration < 50; iteration++) {
    const ksl_vec3_t gan = Multiply(ga, n);
    const ksl_vec3_t gbn = Multiply(gb, n);
    const double ha = std::sqrt(Dot(n, gan));
    const double hb = std::sqrt(Dot(n, gbn));
    /* gradient: the support offsets minus r, i.e. point_a - point_b; b may
    be a point, gb = 0 */
    const ksl_vec3_t gradient =
      Sub(Add(Scale(1.0 / ha, gan), hb > 0.0 ? Scale(1.0 / hb, gbn) : gbn),
          r);
    const double radial = Dot(gradient, n); // = d(n), degree 1 homogeneous
    const ksl_vec3_t tangent = Sub(gradient, Scale(radial, n));
    if(std::sqrt(Dot(tangent, tangent)) <= 1e-14 * scale) {
      break;
    }
    /* Hessian on the sphere, P (H_a + H_b) P - d(n) P, in a tangent basis
    (e1, e2); H = G / h - (G n)(G n)^T / h^3 */
    const ksl_vec3_t axis =
      std::fabs(n.x) < 0.6 ? ksl_vec3_t{{1, 0, 0}} : ksl_vec3_t{{0, 1, 0}};
    ksl_vec3_t e1 = Sub(axis, Scale(Dot(axis, n), n));
    e1 = Scale(1.0 / std::sqrt(Dot(e1, e1)), e1);
    const ksl_vec3_t e2 = {{n.y * e1.z - n.z * e1.y, n.z * e1.x - n.x * e1.z,
                            n.x * e1.y - n.y * e1.x}};
    const ksl_vec3_t* e[2] = {&e1, &e2};
    double H[2][2], g[2];
    for(int i = 0; i < 2; i++) {
      g[i] = Dot(tangent, *e[i]);
      for(int j = 0; j < 2; j++) {
        H[i][j] =
          Dot(*e[i], Multiply(ga, *e[j])) / ha -
          Dot(*e[i], gan) * Dot(*e[j], gan) / (ha * ha * ha) +
          (hb > 0.0 ? Dot(*e[i], Multiply(gb, *e[j])) / hb -
                        Dot(*e[i], gbn) * Dot(*e[j], gbn) / (hb * hb * hb)
                    : 0.0) -
          (i == j ? radial : 0.0);
      }
    }
    double step[2];
    const double det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
    if(det > 0.0 && H[0][0] > 0.0) {
      step[0] = -(H[1][1] * g[0] - H[0][1] * g[1]) / det;
      step[1] = -(H[0][0] * g[1] - H[1][0] * g[0]) / det;
    } else { // not convex here: a gradient step of the length of the scale
      const double k = scale / std::sqrt(g[0] * g[0] + g[1] * g[1]);
      step[0] = -k * g[0] * 0.1;
      step[1] = -k * g[1] * 0.1;
    }
    /* backtrack until the distance decreases. It is flat at the minimum, so
    near it a good step changes it by no more than rounding: only the
    gradient above can still tell, and the distance must not stop Newton */
    const double rounding = 8.0 * std::numeric_limits<double>::epsilon() * scale;
    bool improved = false;
    for(int halving = 0; halving < 30 && !improved; halving++) {
      ksl_vec3_t trial = Add(n, Add(Scale(step[0], e1), Scale(step[1], e2)));
      trial = Scale(1.0 / std::sqrt(Dot(trial, trial)), trial);
      const double trial_depth = distance(trial);
      if(trial_depth <= depth + rounding) {
        n = trial;
        depth = trial_depth;
        improved = true;
      }
      step[0] *= 0.5;
      step[1] *= 0.5;
    }
    if(!improved) {
      break;
    }
  }
  return depth;
}

bool shift::core::collision_geometry::EllipsoidEllipsoidContact(
  const Ellipsoid& a, const Ellipsoid& b, AnalyticContact& contact) {
  const ksl_vec3_t& axes_a = a.get_semi_axes();
  const ksl_vec3_t& axes_b = b.get_semi_axes();
  if(std::min({axes_a.x, axes_a.y, axes_a.z, axes_b.x, axes_b.y, axes_b.z}) <=
     0.0) {
    return false;
  }
  const ksl_vec3_t& ca = a.get_d_absolute().t;
  const ksl_vec3_t& cb = b.get_d_absolute().t;
  const ksl_vec3_t r = Sub(cb, ca);
  const double scale = std::max({axes_a.x, axes_a.y, axes_a.z}) +
                       std::max({axes_b.x, axes_b.y, axes_b.z});
  if(std::sqrt(Dot(r, r)) <= 1e-12 * scale) {
    return false; // no direction to separate along
  }
  contact = AnalyticContact();
  const Sym3 ga = ShapeMatrix(a);
  const Sym3 gb = ShapeMatrix(b);

  /* F'(l) = (1 - 2 l) r^T s - l (1 - l) s^T (G_b - G_a) s, s = M(l)^-1 r */
  auto derivative = [&](double l, double& f, ksl_vec3_t& s) {
    Sym3 M;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        M.m[i][j] = (1.0 - l) * ga.m[i][j] + l * gb.m[i][j];
      }
    }
    s = Solve(M, r);
    const double rs = Dot(r, s);
    f = l * (1.0 - l) * rs;
    return (1.0 - 2.0 * l) * rs -
           l * (1.0 - l) * (Dot(s, Multiply(gb, s)) - Dot(s, Multiply(ga, s)));
  };
  /* F'(0) > 0 > F'(1): regula falsi with the Illinois modification, falling
  back to bisection when it stalls */
  double l0 = 0.0, l1 = 1.0, f, f_max = 0.0;
  ksl_vec3_t s, s_max = r;
  double d0 = derivative(l0, f, s);
  double d1 = derivative(l1, f, s);
  double l = 0.5;
  int side = 0;
  for(int iteration = 0; iteration < 100; iteration++) {
    l = (d0 - d1) != 0.0 ? (l0 * d1 - l1 * d0) / (d1 - d0) : 0.5 * (l0 + l1);
    if(!(l > l0 && l < l1)) {
      l = 0.5 * (l0 + l1);
    }
    const double d = derivative(l, f, s);
    f_max = f;
    s_max = s;
    if(d == 0.0 || l1 - l0 <= 1e-15) {
      break;
    }
    if(d > 0.0) {
      l0 = l;
      d0 = d;
      if(side == 1) {
        d1 *= 0.5;
      }
      side = 1;
    } else {
      l1 = l;
      d1 = d;
      if(side == -1) {
        d0 *= 0.5;
      }
      side = -1;
    }
  }
  if(f_max >= 1.0) {
    return true; // separated or touching
  }

  /* minimize d(n) = h_a(n) + h_b(n) - n . r on the unit sphere. It is the
  distance from r to the boundary of the Minkowski difference, and for deep
  overlaps has several local minima, so Newton's method starts from the
  Perram-Wertheim normal, which is the global minimum for shallow contact,
  from the principal axes of both ellipsoids, and from their opposites */
  ksl_vec3_t starts[8];
  starts[0] = Scale(1.0 / std::sqrt(Dot(s_max, s_max)), s_max);
  for(int k = 0; k < 3; k++) {
    starts[1 + k] = a.get_d_absolute().R.v[k];
    starts[4 + k] = b.get_d_absolute().R.v[k];
  }
  ksl_vec3_t n = starts[0];
  double depth = HUGE_VAL;
  for(int i = 0; i < 7; i++) {
    for(double sign : {1.0, -1.0}) {
      ksl_vec3_t u = Scale(sign, starts[i]);
      const double d = MinimizeSeparation(ga, gb, r, scale, u);
      if(d < depth) {
        depth = d;
        n = u;
      }
    }
  }
  if(depth <= 0.0) {
    return true; // touching to rounding
  }
  const double ha = std::sqrt(Dot(n, Multiply(ga, n)));
  const double hb = std::sqrt(Dot(n, Multiply(gb, n)));
  contact.collision = true;
  contact.depth = depth;
  contact.normal = n;
  contact.point_first = Add(ca, Scale(1.0 / ha, Multiply(ga, n)));
  contact.point_second = Sub(cb, Scale(1.0 / hb, Multiply(gb, n)));
  return true;
}

void shift::core::collision_geometry::PlaneContact(const Plane& plane,
                                                   const Base& other,
                                                   AnalyticContact& contact) {
  contact = AnalyticContact();
  const ksl_vec3_t n = plane.get_normal();
  const double offset = Dot(n, plane.get_point());
  if(auto other_plane = dynamic_cast<const Plane*>(&other)) {
    /* two half spaces overlap without bound unless they face each other;
    facing, they overlap in a slab between the planes */
    const ksl_vec3_t m = other_plane->get_normal();
    if(Dot(n, m) > -1.0 + 1e-12) {
      return;
    }
    const double other_offset = Dot(m, other_plane->get_point());
    contact.depth = offset + other_offset; // n . x <= offset, n . x >= -o
    contact.collision = contact.depth > 0.0;
    if(contact.collision) {
      contact.normal = n;
      /* along the normal through the other plane's point */
      const ksl_vec3_t& p = other_plane->get_point();
      contact.point_second = Sub(p, Scale(Dot(n, p) + other_offset, n));
      contact.point_first = Add(contact.point_second, Scale(contact.depth, n));
    }
    return;
  }
  if(auto sphere = dynamic_cast<const Sphere*>(&other)) {
    const ksl_vec3_t& c = sphere->get_center();
    contact.depth = offset - (Dot(n, c) - sphere->get_radius());
    contact.point_second = Sub(c, Scale(sphere->get_radius(), n));
  } else {
    /* the point deepest in the half space is the support point against the
    normal: closed form for an ellipsoid, the vertex search for a hull */
    ksl_vec3_t local;
    int hint = 0;
    other.Support(Scale(-1.0, n), local, contact.point_second, hint);
    contact.depth = offset - Dot(n, contact.point_second);
  }
  contact.collision = contact.depth > 0.0;
  contact.normal = n;
  contact.point_first = Add(contact.point_second, Scale(contact.depth, n));
}

static inline ksl_vec3_t Cross(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
           a.x * b.y - a.y * b.x}};
}

/*! faces whose normals have 1 - cos^2 below this, an angle of about 1e-6,
 * are coplanar: the edge between them is a diagonal of a polygon */
static constexpr double kCoplanar = 1e-12;

/*! the faces (triangles) of a hull with an outward normal that have the
 * edge v1 v2 */
static int EdgeFaces(const shift::core::collision_geometry::ConvexHullData& h,
                     unsigned int v1, unsigned int v2, int faces[2]) {
  int count = 0;
  for(auto f : h.vertex_faces[v1]) {
    const unsigned int* t = &h.faces[3 * f];
    if((t[0] == v2 || t[1] == v2 || t[2] == v2) &&
       Dot(h.face_normals[f], h.face_normals[f]) > 0.0) {
      if(count < 2) {
        faces[count] = static_cast<int>(f);
      }
      count++;
    }
  }
  return count;
}

bool shift::core::collision_geometry::ConvexHullEllipsoidContact(
  const ConvexHull& hull, const Ellipsoid& ellipsoid, const ksl_vec3_t& start,
  bool nearest, AnalyticContact& contact) {
  const ksl_vec3_t& axes = ellipsoid.get_semi_axes();
  const auto& data = hull.get_hull();
  if(std::min({axes.x, axes.y, axes.z}) <= 0.0 || data.faces.empty() ||
     data.vertex_faces.size() != data.vertices.size() ||
     data.neighbors.size() != data.vertices.size() ||
     data.face_normals.size() * 3 != data.faces.size()) {
    return false;
  }
  contact = AnalyticContact();
  const ksl_SE3_t& pose = hull.get_d_absolute();
  std::vector<ksl_vec3_t> w(data.vertices.size());
  double size = 0.0;
  for(size_t i = 0; i < w.size(); i++) {
    ksl_product_dv(&pose, &data.vertices[i], &w[i]);
    const ksl_vec3_t r = Sub(w[i], pose.t);
    size = std::max(size, Dot(r, r));
  }
  const double scale = std::sqrt(size) + std::max({axes.x, axes.y, axes.z});
  const double tolerance = 1e-12 * scale;
  const ksl_vec3_t& c = ellipsoid.get_d_absolute().t;
  const Sym3 G = ShapeMatrix(ellipsoid);
  auto normal_of = [&](int f) {
    return Rotate(pose, data.face_normals[f]);
  };
  /* the support point q of the ellipsoid against n, and h_b(-n) + n . c */
  auto against = [&](const ksl_vec3_t& n, ksl_vec3_t& q) {
    const ksl_vec3_t g = Multiply(G, n);
    const double h = std::sqrt(Dot(n, g));
    q = Sub(c, Scale(1.0 / h, g));
    return h;
  };

  /* the least d is set by a feature of the hull, a vertex, an edge or a
  face (a triangle: the polygons of coplanar triangles are only nearly
  planar), each with its own smooth problem; the feature solves its problem
  inside its normal cone, with the support point q of the ellipsoid against
  n projecting onto it. start, EPA's normal, is close to the least d, so the
  feature is made of vertices near the extreme along start; each candidate
  is solved and checked, and the passing one of least d kept, or the one
  nearest start */
  const double slack = 1e-9;
  struct Solution {
    bool found = false;
    double depth = HUGE_VAL;
    double closeness = -HUGE_VAL;
    ksl_vec3_t n, q, x;
  };
  const ksl_vec3_t n0 = Scale(1.0 / std::sqrt(Dot(start, start)), start);
  Solution best;
  auto keep = [&](double depth, const ksl_vec3_t& n, const ksl_vec3_t& q,
                  const ksl_vec3_t& x) {
    const double closeness = Dot(n, n0);
    if(!best.found ||
       (nearest ? closeness > best.closeness : depth < best.depth)) {
      best.closeness = closeness;
      best.found = true;
      best.depth = depth;
      best.n = n;
      best.q = q;
      best.x = x;
    }
  };
  const Sym3 point = {};

  /* a vertex: d_v(n) = h_b(n) - n . (c - v), the separation of the
  ellipsoid and the point v (see MinimizeSeparation), from start; v must be
  extreme along n */
  auto vertex = [&](unsigned int v) {
    ksl_vec3_t n = n0, q;
    MinimizeSeparation(G, point, Sub(c, w[v]), scale, n);
    const double depth = Dot(n, Sub(w[v], c)) + against(n, q);
    for(unsigned int u : data.neighbors[v]) {
      if(Dot(Sub(w[u], w[v]), n) > tolerance) {
        return;
      }
    }
    keep(depth, n, q, w[v]);
  };

  /* an edge: d(n) = n . (v1 - c) + h_b(n) over its normal cone, the arc
  from n1 to n2, n = cos t n1 + sin t s with s the unit part of n2 normal to
  n1, least at roots of its derivative inside (a least end is a face's
  solution); q must project onto the edge */
  auto edge = [&](unsigned int v1, unsigned int v2) {
    int faces[2];
    if(EdgeFaces(data, v1, v2, faces) != 2) {
      return;
    }
    const ksl_vec3_t n1 = normal_of(faces[0]);
    const ksl_vec3_t n2 = normal_of(faces[1]);
    const double k12 = Dot(n1, n2);
    if(1.0 - k12 * k12 <= kCoplanar) {
      return; // a diagonal of coplanar faces, solved as a face
    }
    const ksl_vec3_t s = Scale(1.0 / std::sqrt(1.0 - k12 * k12),
                               Sub(n2, Scale(k12, n1)));
    const double end = std::acos(std::max(-1.0, std::min(1.0, k12)));
    const ksl_vec3_t a = Sub(w[v1], c);
    auto at = [&](double t) {
      return Add(Scale(std::cos(t), n1), Scale(std::sin(t), s));
    };
    auto slope = [&](double t) {
      const ksl_vec3_t m = at(t);
      const ksl_vec3_t dm = Add(Scale(-std::sin(t), n1), Scale(std::cos(t), s));
      const ksl_vec3_t g = Multiply(G, m);
      return Dot(dm, a) + Dot(dm, g) / std::sqrt(Dot(m, g));
    };
    /* d may dip inside the arc although it rises from both ends: each
    local minimum inside, where the slope of samples changes from - to +,
    by bisection */
    const ksl_vec3_t e = Sub(w[v2], w[v1]);
    const int samples = 16;
    double previous = slope(0.0);
    for(int k = 1; k <= samples; k++) {
      double low = end * (k - 1) / samples, high = end * k / samples;
      const double current = slope(high);
      const bool dip = previous < 0.0 && current > 0.0;
      previous = current;
      if(!dip) {
        continue;
      }
      for(int i = 0; i < 200; i++) {
        const double middle = 0.5 * (low + high);
        if(middle <= low || middle >= high) {
          break;
        }
        (slope(middle) < 0.0 ? low : high) = middle;
      }
      const ksl_vec3_t n = at(0.5 * (low + high));
      ksl_vec3_t q;
      const double depth = Dot(n, a) + against(n, q);
      const double along = Dot(Sub(q, w[v1]), e) / Dot(e, e);
      if(along >= -slack && along <= 1.0 + slack) {
        keep(depth, n, q, Add(w[v1], Scale(along, e)));
      }
    }
  };

  /* a face: n is its normal, and q must project into it (or into a
  coplanar face across one of its edges) */
  auto face = [&](unsigned int f) {
    const unsigned int* t = &data.faces[3 * f];
    const ksl_vec3_t n = normal_of(static_cast<int>(f));
    ksl_vec3_t q;
    const double depth = Dot(n, Sub(w[t[0]], c)) + against(n, q);
    for(int k = 0; k < 3; k++) {
      const ksl_vec3_t& p = w[t[k]];
      ksl_vec3_t side = Cross(Sub(w[t[(k + 1) % 3]], p), n);
      const double l = std::sqrt(Dot(side, side));
      if(!(l > 0.0)) {
        continue;
      }
      if(Dot(side, Sub(w[t[(k + 2) % 3]], p)) > 0.0) {
        side = Scale(-1.0, side); // away from the third vertex
      }
      if(Dot(side, Sub(q, p)) / l > tolerance) {
        return;
      }
    }
    keep(depth, n, q, Add(q, Scale(depth, n)));
  };

  double extreme = -HUGE_VAL;
  for(const auto& p : w) {
    extreme = std::max(extreme, Dot(p, n0));
  }
  for(double margin = 1e-4 * scale; margin < 2.0 * scale && !best.found;
      margin *= 10.0) {
    std::vector<char> near(w.size(), 0);
    for(size_t i = 0; i < w.size(); i++) {
      near[i] = Dot(w[i], n0) >= extreme - margin;
    }
    for(size_t f = 0; 3 * f + 2 < data.faces.size(); f++) {
      const unsigned int* t = &data.faces[3 * f];
      if(near[t[0]] && near[t[1]] && near[t[2]] &&
         Dot(data.face_normals[f], data.face_normals[f]) > 0.0) {
        face(static_cast<unsigned int>(f));
      }
    }
    for(size_t v = 0; v < w.size(); v++) {
      if(!near[v]) {
        continue;
      }
      vertex(static_cast<unsigned int>(v));
      for(unsigned int u : data.neighbors[v]) {
        if(u > v && near[u]) {
          edge(static_cast<unsigned int>(v), u);
        }
      }
    }
  }
  if(!best.found) {
    return false;
  }
  contact.collision = best.depth > 0.0;
  if(contact.collision) {
    contact.depth = best.depth;
    contact.normal = best.n;
    contact.point_first = best.x;
    contact.point_second = best.q;
  }
  return true;
}

/*! whether point p of the plane of triangle t (vertices w) projects into
 * it, within tolerance, for normal n of the plane */
static bool InTriangle(const std::vector<ksl_vec3_t>& w, const unsigned int* t,
                       const ksl_vec3_t& n, const ksl_vec3_t& p,
                       double tolerance) {
  for(int k = 0; k < 3; k++) {
    const ksl_vec3_t& a = w[t[k]];
    ksl_vec3_t side = Cross(Sub(w[t[(k + 1) % 3]], a), n);
    const double l = std::sqrt(Dot(side, side));
    if(!(l > 0.0)) {
      continue;
    }
    if(Dot(side, Sub(w[t[(k + 2) % 3]], a)) > 0.0) {
      side = Scale(-1.0, side); // away from the third vertex
    }
    if(Dot(side, Sub(p, a)) / l > tolerance) {
      return false;
    }
  }
  return true;
}

/*! whether n is in the normal cone of edge v1 v2 of a hull, between the
 * normals of its two faces; false for a diagonal of coplanar faces */
static bool InEdgeCone(const shift::core::collision_geometry::ConvexHull& hull,
                       unsigned int v1, unsigned int v2, const ksl_vec3_t& n) {
  int faces[2];
  const auto& data = hull.get_hull();
  if(EdgeFaces(data, v1, v2, faces) != 2) {
    return false;
  }
  const ksl_vec3_t n1 = Rotate(hull.get_d_absolute(), data.face_normals[faces[0]]);
  const ksl_vec3_t n2 = Rotate(hull.get_d_absolute(), data.face_normals[faces[1]]);
  const double k12 = Dot(n1, n2);
  const double det = 1.0 - k12 * k12;
  if(det <= kCoplanar) {
    return false;
  }
  const double b1 = Dot(n1, n), b2 = Dot(n2, n);
  return (b1 - k12 * b2) / det >= -1e-12 && (b2 - k12 * b1) / det >= -1e-12;
}

/*! the inertial vertices of a hull, transformed when first used */
struct LazyVertices {
  const shift::core::collision_geometry::ConvexHull& hull;
  std::vector<ksl_vec3_t> w;
  std::vector<char> done;
  explicit LazyVertices(const shift::core::collision_geometry::ConvexHull& h)
    : hull(h)
    , w(h.get_hull().vertices.size())
    , done(h.get_hull().vertices.size(), 0) {}
  const ksl_vec3_t& operator()(unsigned int i) {
    if(!done[i]) {
      ksl_product_dv(&hull.get_d_absolute(), &hull.get_hull().vertices[i],
                     &w[i]);
      done[i] = 1;
    }
    return w[i];
  }
};

/*! the vertices of a hull within margin of the extreme along d: they are
 * connected by its edges (a superlevel set of a linear function on a convex
 * polytope), so they are found by a search from the support vertex */
static void NearVertices(LazyVertices& w, const ksl_vec3_t& d, double margin,
                         int& hint, std::vector<unsigned int>& near,
                         std::vector<char>& is_near) {
  ksl_vec3_t local, global;
  w.hull.Support(d, local, global, hint);
  const auto& neighbors = w.hull.get_hull().neighbors;
  const double top = Dot(w(static_cast<unsigned int>(hint)), d);
  is_near.assign(neighbors.size(), 0);
  near.assign(1, static_cast<unsigned int>(hint));
  is_near[hint] = 1;
  for(size_t k = 0; k < near.size(); k++) {
    for(unsigned int u : neighbors[near[k]]) {
      if(!is_near[u] && Dot(w(u), d) >= top - margin) {
        is_near[u] = 1;
        near.push_back(u);
      }
    }
  }
}

bool shift::core::collision_geometry::ConvexHullPairContact(
  const ConvexHull& a, const ConvexHull& b, const ksl_vec3_t& start,
  int& hint_a, int& hint_b, AnalyticContact& contact) {
  for(const ConvexHull* hull : {&a, &b}) {
    const auto& data = hull->get_hull();
    if(data.faces.empty() || data.vertex_faces.size() != data.vertices.size() ||
       data.neighbors.size() != data.vertices.size() ||
       data.face_normals.size() * 3 != data.faces.size()) {
      return false;
    }
  }
  contact = AnalyticContact();
  const auto& da = a.get_hull();
  const auto& db = b.get_hull();
  LazyVertices wa(a), wb(b);
  const double size =
    std::max(a.get_bounding_radius() + b.get_bounding_radius(), 1e-300);
  const double tolerance = 1e-12 * size;
  const double slack = 1e-9;
  const ksl_vec3_t n0 = Scale(1.0 / std::sqrt(Dot(start, start)), start);
  const ksl_vec3_t m0 = Scale(-1.0, n0);

  struct Solution {
    bool found = false;
    double closeness = -HUGE_VAL;
    double depth = 0.0;
    ksl_vec3_t n, x, y;
  };
  Solution best;
  auto keep = [&](const ksl_vec3_t& n, double depth, const ksl_vec3_t& x,
                  const ksl_vec3_t& y) {
    const double closeness = Dot(n, n0);
    if(!best.found || closeness > best.closeness) {
      best = {true, closeness, depth, n, x, y};
    }
  };

  /* a face of one hull and the vertices of the other extreme against its
  normal nf (outward of the first): they must project into the face. For a
  face of a the normal of the contact is nf, from a to b; for one of b it is
  -nf */
  std::vector<unsigned int> support;
  std::vector<char> in_support;
  auto face = [&](bool first_is_a, unsigned int f) {
    const auto& data = first_is_a ? da : db;
    LazyVertices& wf = first_is_a ? wa : wb;
    LazyVertices& wo = first_is_a ? wb : wa;
    const unsigned int* t = &data.faces[3 * f];
    const ksl_vec3_t nf =
      Rotate(wf.hull.get_d_absolute(), data.face_normals[f]);
    if(!(Dot(nf, nf) > 0.0)) {
      return;
    }
    const ksl_vec3_t against = Scale(-1.0, nf);
    int hint = first_is_a ? hint_b : hint_a;
    NearVertices(wo, against, tolerance, hint, support, in_support);
    const double low = Dot(wo(support[0]), against);
    const double depth = Dot(nf, wf(t[0])) + low;
    std::vector<ksl_vec3_t> triangle = {wf(t[0]), wf(t[1]), wf(t[2])};
    const unsigned int local[3] = {0, 1, 2};
    for(unsigned int i : support) {
      const ksl_vec3_t& v = wo(i);
      const ksl_vec3_t p = Add(v, Scale(depth, nf)); // onto the face plane
      if(!InTriangle(triangle, local, nf, p, tolerance)) {
        continue;
      }
      if(first_is_a) {
        keep(nf, depth, p, v);
      } else {
        keep(against, depth, v, p);
      }
    }
  };

  /* an edge of a and an edge of b: n normal to both, in the normal cone of
  a's edge, with -n in that of b's, and the closest points of their lines on
  both segments */
  auto edges = [&](unsigned int a1, unsigned int a2, unsigned int b1,
                   unsigned int b2) {
    const ksl_vec3_t ea = Sub(wa(a2), wa(a1));
    const ksl_vec3_t eb = Sub(wb(b2), wb(b1));
    ksl_vec3_t n = Cross(ea, eb);
    const double l = std::sqrt(Dot(n, n));
    if(!(l > 1e-9 * std::sqrt(Dot(ea, ea) * Dot(eb, eb)))) {
      return; // parallel edges: a face's solution
    }
    n = Scale(1.0 / l, n);
    if(!InEdgeCone(a, a1, a2, n)) {
      n = Scale(-1.0, n);
      if(!InEdgeCone(a, a1, a2, n)) {
        return;
      }
    }
    if(!InEdgeCone(b, b1, b2, Scale(-1.0, n))) {
      return;
    }
    /* x = a1 + s ea, y = b1 + t eb with x - y along n */
    const ksl_vec3_t r = Sub(wa(a1), wb(b1));
    const double aa = Dot(ea, ea), ab = Dot(ea, eb), bb = Dot(eb, eb);
    const double ar = Dot(ea, r), br = Dot(eb, r);
    const double det = aa * bb - ab * ab;
    const double s = (ab * br - bb * ar) / det;
    const double t = (aa * br - ab * ar) / det;
    if(s < -slack || s > 1.0 + slack || t < -slack || t > 1.0 + slack) {
      return;
    }
    const ksl_vec3_t x = Add(wa(a1), Scale(s, ea));
    const ksl_vec3_t y = Add(wb(b1), Scale(t, eb));
    keep(n, Dot(n, Sub(x, y)), x, y);
  };

  std::vector<unsigned int> near_a, near_b;
  std::vector<char> is_a, is_b;
  /* the minimum near start, else none (a new contact: EPA's) */
  for(double margin = 1e-4 * size; margin < 2e-2 * size && !best.found;
      margin *= 10.0) {
    NearVertices(wa, n0, margin, hint_a, near_a, is_a);
    NearVertices(wb, m0, margin, hint_b, near_b, is_b);
    /* the faces of near vertices, each once from its least vertex */
    for(int first = 0; first < 2; first++) {
      const auto& data = first == 0 ? da : db;
      const auto& near = first == 0 ? near_a : near_b;
      const auto& is_near = first == 0 ? is_a : is_b;
      for(unsigned int v : near) {
        for(auto f : data.vertex_faces[v]) {
          const unsigned int* t = &data.faces[3 * f];
          if(is_near[t[0]] && is_near[t[1]] && is_near[t[2]] &&
             v == std::min({t[0], t[1], t[2]})) {
            face(first == 0, f);
          }
        }
      }
    }
    for(unsigned int a1 : near_a) {
      for(unsigned int a2 : da.neighbors[a1]) {
        if(a2 <= a1 || !is_a[a2]) {
          continue;
        }
        for(unsigned int b1 : near_b) {
          for(unsigned int b2 : db.neighbors[b1]) {
            if(b2 > b1 && is_b[b2]) {
              edges(a1, a2, b1, b2);
            }
          }
        }
      }
    }
  }
  if(!best.found) {
    return false;
  }
  contact.collision = best.depth > 0.0;
  if(contact.collision) {
    contact.depth = best.depth;
    contact.normal = best.n;
    contact.point_first = best.x;
    contact.point_second = best.y;
  }
  return true;
}

bool shift::core::collision_geometry::CylinderSphereContact(
  const Cylinder& cylinder, const Sphere& sphere, AnalyticContact& contact) {
  const double r = cylinder.get_radius();
  const double h = cylinder.get_half_length();
  if(!(r > 0.0 && h > 0.0)) {
    return false;
  }
  contact = AnalyticContact();
  const ksl_SE3_t& d = cylinder.get_d_absolute();
  const ksl_vec3_t c = ToLocal(d, sphere.get_center());
  /* the unit direction across the axis toward the center, any on the axis */
  const double radial = std::hypot(c.x, c.y);
  const ksl_vec3_t u = radial > 0.0
                         ? ksl_vec3_t{{c.x / radial, c.y / radial, 0.0}}
                         : ksl_vec3_t{{1.0, 0.0, 0.0}};
  const double end_sign = c.z >= 0.0 ? 1.0 : -1.0;
  const double side = radial - r;          // > 0 outside the side
  const double end = std::fabs(c.z) - h;   // > 0 beyond an end
  ksl_vec3_t q, n;
  double distance;
  if(side > 0.0 && end > 0.0) {
    /* beyond a rim */
    q = {{r * u.x, r * u.y, end_sign * h}};
    const ksl_vec3_t g = Sub(c, q);
    distance = std::sqrt(Dot(g, g));
    n = Scale(1.0 / distance, g);
  } else if(end > 0.0 || (side <= 0.0 && end > side)) {
    /* over an end, or inside and nearer the end than the side */
    q = {{c.x, c.y, end_sign * h}};
    n = {{0.0, 0.0, end_sign}};
    distance = end;
  } else {
    q = {{r * u.x, r * u.y, c.z}};
    n = u;
    distance = side;
  }
  SetSphereContact(d, q, n, distance, sphere, contact);
  return true;
}

/*! the polygon of a hull across its edge v1 v2 from polygon p, or -1 */
static int AcrossEdge(const shift::core::collision_geometry::ConvexHullData& h,
                      unsigned int p, unsigned int v1, unsigned int v2) {
  for(unsigned int q : h.vertex_polygons[v1]) {
    if(q == p) {
      continue;
    }
    const auto& polygon = h.polygons[q];
    const size_t m = polygon.size();
    for(size_t k = 0; k < m; k++) {
      const unsigned int i = polygon[k], j = polygon[(k + 1) % m];
      if((i == v1 && j == v2) || (i == v2 && j == v1)) {
        return static_cast<int>(q);
      }
    }
  }
  return -1;
}

/*! whether n is in the normal cone of an edge between faces of unit normals
 * n1 and n2 (n a combination of them with weights >= 0) */
static bool InCone(const ksl_vec3_t& n1, const ksl_vec3_t& n2,
                   const ksl_vec3_t& n) {
  const double k12 = Dot(n1, n2);
  const double det = 1.0 - k12 * k12;
  if(det <= kCoplanar) {
    return false;
  }
  const double b1 = Dot(n1, n), b2 = Dot(n2, n);
  return (b1 - k12 * b2) / det >= -1e-12 && (b2 - k12 * b1) / det >= -1e-12;
}

bool shift::core::collision_geometry::CylinderHullContact(
  const ConvexHull& hull, const Cylinder& cylinder, const ksl_vec3_t& start,
  bool nearest, int& hint, AnalyticContact& contact) {
  const auto& data = hull.get_hull();
  const double r = cylinder.get_radius();
  const double h = cylinder.get_half_length();
  const double start_length = std::sqrt(Dot(start, start));
  if(!(r > 0.0 && h > 0.0) || !(start_length > 0.0) || data.polygons.empty() ||
     data.polygon_normals.size() != data.polygons.size() ||
     data.polygon_offsets.size() != data.polygons.size() ||
     data.vertex_polygons.size() != data.vertices.size() ||
     data.neighbors.size() != data.vertices.size()) {
    return false;
  }
  contact = AnalyticContact();
  LazyVertices w(hull);
  const ksl_SE3_t& pose = hull.get_d_absolute();
  const ksl_vec3_t& c = cylinder.get_d_absolute().t;
  const ksl_vec3_t& a = cylinder.get_axis();
  const double size =
    std::max(hull.get_bounding_radius() + std::hypot(r, h), 1e-300);
  const double tolerance = 1e-12 * size;
  const double slack = 1e-9;
  /* an end, or a side line, is parallel to a face where the rim's point
  deepest against the face's normal is this close to it, in angle */
  const double parallel = 1e-9;
  const ksl_vec3_t n0 = Scale(1.0 / start_length, start);

  /* the cylinder's extent against n from its center, h |g| + r sqrt(1 - g^2) */
  auto reach = [&](const ksl_vec3_t& n) {
    const double g = Dot(n, a);
    return h * std::fabs(g) + r * std::sqrt(std::max(1.0 - g * g, 0.0));
  };
  /* the point of the cylinder deepest against n, on a rim (where n is
  neither along nor across the axis) */
  auto deepest = [&](const ksl_vec3_t& n) {
    const double g = Dot(n, a);
    ksl_vec3_t y = Sub(c, Scale(g >= 0.0 ? h : -h, a));
    const ksl_vec3_t radial = Sub(n, Scale(g, a));
    const double l = std::sqrt(Dot(radial, radial));
    if(l > 0.0) {
      y = Sub(y, Scale(r / l, radial));
    }
    return y;
  };

  struct Solution {
    bool found = false;
    double depth = HUGE_VAL;
    double closeness = -HUGE_VAL;
    ksl_vec3_t n, x, y;
  };
  Solution best;
  auto keep = [&](const ksl_vec3_t& n, double depth, const ksl_vec3_t& x,
                  const ksl_vec3_t& y) {
    const double closeness = Dot(n, n0);
    if(!best.found ||
       (nearest ? closeness > best.closeness : depth < best.depth)) {
      best.found = true;
      best.depth = depth;
      best.closeness = closeness;
      best.n = n;
      best.x = x;
      best.y = y;
    }
  };

  auto polygon_normal = [&](unsigned int p) {
    return Rotate(pose, data.polygon_normals[p]);
  };
  /* the unit outward normal, in its plane, of edge k of polygon p of
  normal np, and the edge's first vertex */
  auto edge_out = [&](unsigned int p, size_t k, const ksl_vec3_t& np,
                      ksl_vec3_t& from) {
    const auto& polygon = data.polygons[p];
    from = w(polygon[k]);
    const ksl_vec3_t e = Sub(w(polygon[(k + 1) % polygon.size()]), from);
    const ksl_vec3_t out = Cross(e, np); // counterclockwise about np
    const double l = std::sqrt(Dot(out, out));
    return l > 0.0 ? Scale(1.0 / l, out) : out;
  };
  /* whether x projects along np into polygon p */
  auto in_polygon = [&](unsigned int p, const ksl_vec3_t& np,
                        const ksl_vec3_t& x) {
    for(size_t k = 0; k < data.polygons[p].size(); k++) {
      ksl_vec3_t from;
      const ksl_vec3_t out = edge_out(p, k, np, from);
      if(Dot(out, Sub(x, from)) > tolerance) {
        return false;
      }
    }
    return true;
  };

  /* a face of the hull and the cylinder's point deepest against its normal */
  auto face = [&](unsigned int p) {
    const ksl_vec3_t np = polygon_normal(p);
    const double offset = data.polygon_offsets[p] + Dot(np, pose.t);
    const double depth = offset - Dot(np, c) + reach(np);
    const double g = Dot(np, a);
    const double across = std::sqrt(std::max(1.0 - g * g, 0.0));
    const auto& polygon = data.polygons[p];
    ksl_vec3_t y;
    if(across <= parallel) {
      /* an end on the face: the point of its disc nearest the polygon */
      const ksl_vec3_t center = Sub(c, Scale(g >= 0.0 ? h : -h, a));
      const ksl_vec3_t foot =
        Sub(center, Scale(Dot(np, center) - offset, np)); // on the plane
      ksl_vec3_t nearest_point = foot;
      if(!in_polygon(p, np, foot)) {
        double least = HUGE_VAL;
        for(size_t k = 0; k < polygon.size(); k++) {
          const ksl_vec3_t p1 = w(polygon[k]);
          const ksl_vec3_t e = Sub(w(polygon[(k + 1) % polygon.size()]), p1);
          const double t =
            std::clamp(Dot(Sub(foot, p1), e) / Dot(e, e), 0.0, 1.0);
          const ksl_vec3_t q = Add(p1, Scale(t, e));
          const ksl_vec3_t gap = Sub(q, foot);
          const double l2 = Dot(gap, gap);
          if(l2 < least) {
            least = l2;
            nearest_point = q;
          }
        }
        if(std::sqrt(least) > r + tolerance) {
          return;
        }
      }
      y = Add(center, Sub(nearest_point, foot));
    } else if(std::fabs(g) <= parallel) {
      /* a side line on the face: the middle of its part over the polygon */
      const ksl_vec3_t radial = Scale(1.0 / across, Sub(np, Scale(g, a)));
      const ksl_vec3_t line = Sub(c, Scale(r, radial));
      double s1 = -h, s2 = h;
      for(size_t k = 0; k < polygon.size(); k++) {
        ksl_vec3_t from;
        const ksl_vec3_t out = edge_out(p, k, np, from);
        const double at_0 = Dot(out, Sub(line, from)) - tolerance;
        const double slope = Dot(out, a);
        if(slope > 0.0) {
          s2 = std::min(s2, -at_0 / slope);
        } else if(slope < 0.0) {
          s1 = std::max(s1, -at_0 / slope);
        } else if(at_0 > 0.0) {
          return;
        }
      }
      if(s1 > s2) {
        return;
      }
      y = Add(line, Scale(0.5 * (s1 + s2), a));
    } else {
      y = deepest(np);
      if(!in_polygon(p, np, y)) {
        return;
      }
    }
    keep(np, depth, Add(y, Scale(depth, np)), y);
  };

  /* an end of the cylinder, of outward normal sign * a, and the hull's
  vertex or edge extreme against it */
  std::vector<unsigned int> support;
  std::vector<char> in_support;
  auto end = [&](double sign) {
    const ksl_vec3_t n = Scale(-sign, a);
    int end_hint = hint;
    NearVertices(w, n, tolerance, end_hint, support, in_support);
    if(support.size() > 2) {
      return; // a face on the end: the face's solution
    }
    const ksl_vec3_t center = Add(c, Scale(sign * h, a));
    /* the radial offset of a point from the axis */
    auto radial = [&](const ksl_vec3_t& p) {
      const ksl_vec3_t q = Sub(p, center);
      return Sub(q, Scale(Dot(q, a), a));
    };
    ksl_vec3_t x;
    const ksl_vec3_t v1 = w(support[0]);
    if(support.size() == 1) {
      const ksl_vec3_t q = radial(v1);
      if(Dot(q, q) > (r + tolerance) * (r + tolerance)) {
        return;
      }
      x = v1;
    } else {
      /* an edge on the end: the middle of its part over the disc,
      |q1 + t e|^2 <= r^2 */
      const ksl_vec3_t q1 = radial(v1);
      const ksl_vec3_t e = Sub(radial(w(support[1])), q1);
      const double ee = Dot(e, e), qe = Dot(q1, e);
      const double qq = Dot(q1, q1) - (r + tolerance) * (r + tolerance);
      double t1 = 0.0, t2 = 1.0;
      if(ee > 0.0) {
        const double discriminant = qe * qe - ee * qq;
        if(discriminant < 0.0) {
          return;
        }
        const double root = std::sqrt(discriminant);
        t1 = std::max(t1, (-qe - root) / ee);
        t2 = std::min(t2, (-qe + root) / ee);
      } else if(qq > 0.0) {
        return;
      }
      if(t1 > t2) {
        return;
      }
      x = Add(v1, Scale(0.5 * (t1 + t2), Sub(w(support[1]), v1)));
    }
    const double depth = Dot(n, Sub(x, c)) + h;
    keep(n, depth, x, Sub(x, Scale(depth, n)));
  };

  /* the side of the cylinder and a vertex of the hull */
  auto side_vertex = [&](unsigned int v) {
    const ksl_vec3_t rel = Sub(w(v), c);
    const double s = Dot(rel, a);
    if(std::fabs(s) > h * (1.0 + slack)) {
      return;
    }
    const ksl_vec3_t radial = Sub(rel, Scale(s, a));
    const double rho = std::sqrt(Dot(radial, radial));
    if(!(rho > tolerance)) {
      return;
    }
    const ksl_vec3_t n = Scale(-1.0 / rho, radial);
    for(unsigned int u : data.neighbors[v]) {
      if(Dot(Sub(w(u), w(v)), n) > tolerance) {
        return;
      }
    }
    const double depth = r - rho;
    keep(n, depth, w(v), Sub(w(v), Scale(depth, n)));
  };

  /* the side of the cylinder and edge v1 v2 between polygons of normals n1
  and n2: n normal to the axis and the edge */
  auto side_edge = [&](const ksl_vec3_t& p1, const ksl_vec3_t& p2,
                       const ksl_vec3_t& n1, const ksl_vec3_t& n2) {
    const ksl_vec3_t e = Sub(p2, p1);
    ksl_vec3_t n = Cross(a, e);
    const double l = std::sqrt(Dot(n, n));
    if(!(l > 1e-9 * std::sqrt(Dot(e, e)))) {
      /* along the axis: n from the edge's line to the axis, at the middle of
      the part of the edge beside the side */
      const ksl_vec3_t q = Sub(p1, c);
      const ksl_vec3_t radial = Sub(q, Scale(Dot(q, a), a));
      const double rho = std::sqrt(Dot(radial, radial));
      if(!(rho > tolerance)) {
        return;
      }
      n = Scale(-1.0 / rho, radial);
      if(!InCone(n1, n2, n)) {
        return;
      }
      const double s1 = Dot(q, a), s2 = Dot(Sub(p2, c), a);
      const double low = std::max(std::min(s1, s2), -h);
      const double high = std::min(std::max(s1, s2), h);
      if(low > high || s1 == s2) {
        return;
      }
      const ksl_vec3_t x =
        Add(p1, Scale((0.5 * (low + high) - s1) / (s2 - s1), e));
      const double depth = r - rho;
      keep(n, depth, x, Sub(x, Scale(depth, n)));
      return;
    }
    n = Scale(1.0 / l, n);
    if(!InCone(n1, n2, n)) {
      n = Scale(-1.0, n);
      if(!InCone(n1, n2, n)) {
        return;
      }
    }
    /* x = p1 + t e on the edge, c + s a on the axis, x - (c + s a) along n */
    const ksl_vec3_t q = Sub(p1, c);
    const double ae = Dot(a, e), ee = Dot(e, e);
    const double aq = Dot(a, q), eq = Dot(e, q);
    const double det = ee - ae * ae;
    if(!(det > 0.0)) {
      return;
    }
    const double t = (ae * aq - eq) / det;
    const double s_axis = aq + ae * t;
    if(t < -slack || t > 1.0 + slack || std::fabs(s_axis) > h * (1.0 + slack)) {
      return;
    }
    const ksl_vec3_t x = Add(p1, Scale(t, e));
    const double depth = Dot(n, Sub(x, c)) + r;
    keep(n, depth, x, Sub(x, Scale(depth, n)));
  };

  /* a rim of the cylinder and edge p1 p2 between polygons of normals n1 and
  n2: the least d(n) = n . (p1 - c) + h |g| + r sqrt(1 - g^2) over the arc of
  normals from n1 to n2, by the root of its derivative */
  auto rim_edge = [&](const ksl_vec3_t& p1, const ksl_vec3_t& p2,
                      const ksl_vec3_t& n1, const ksl_vec3_t& n2) {
    const double k12 = Dot(n1, n2);
    if(1.0 - k12 * k12 <= kCoplanar) {
      return;
    }
    const ksl_vec3_t s = Scale(1.0 / std::sqrt(1.0 - k12 * k12),
                               Sub(n2, Scale(k12, n1)));
    const double arc = std::acos(std::max(-1.0, std::min(1.0, k12)));
    const ksl_vec3_t q = Sub(p1, c);
    auto at = [&](double t) {
      return Add(Scale(std::cos(t), n1), Scale(std::sin(t), s));
    };
    /* the derivative, smooth where n is neither along nor across the axis
    (those are the solutions of an end and of the side) */
    auto slope = [&](double t) {
      const ksl_vec3_t m = at(t);
      const ksl_vec3_t dm = Add(Scale(-std::sin(t), n1), Scale(std::cos(t), s));
      const double g = Dot(m, a), dg = Dot(dm, a);
      const double across = std::sqrt(std::max(1.0 - g * g, 0.0));
      return Dot(dm, q) + h * (g >= 0.0 ? dg : -dg) -
             (across > 0.0 ? r * g * dg / across : 0.0);
    };
    const ksl_vec3_t e = Sub(p2, p1);
    const int samples = 16;
    double previous = slope(0.0);
    for(int k = 1; k <= samples; k++) {
      double low = arc * (k - 1) / samples, high = arc * k / samples;
      const double current = slope(high);
      const bool dip = previous < 0.0 && current > 0.0;
      previous = current;
      if(!dip) {
        continue;
      }
      for(int i = 0; i < 200; i++) {
        const double middle = 0.5 * (low + high);
        if(middle <= low || middle >= high) {
          break;
        }
        (slope(middle) < 0.0 ? low : high) = middle;
      }
      const ksl_vec3_t n = at(0.5 * (low + high));
      const double g = Dot(n, a);
      if(std::fabs(g) <= parallel ||
         std::sqrt(std::max(1.0 - g * g, 0.0)) <= parallel) {
        continue; // a crease of d: the side's or an end's solution
      }
      const ksl_vec3_t y = deepest(n);
      const double along = Dot(Sub(y, p1), e) / Dot(e, e);
      if(along < -slack || along > 1.0 + slack) {
        continue;
      }
      const double depth = Dot(n, q) + reach(n);
      keep(n, depth, Add(p1, Scale(along, e)), Sub(Add(p1, Scale(along, e)),
                                                  Scale(depth, n)));
    }
  };

  std::vector<unsigned int> near;
  std::vector<char> is_near;
  std::vector<char> seen(data.polygons.size(), 0);
  for(double margin = 1e-4 * size; margin < 2.0 * size && !best.found;
      margin *= 10.0) {
    NearVertices(w, n0, margin, hint, near, is_near);
    std::fill(seen.begin(), seen.end(), 0);
    for(unsigned int v : near) {
      side_vertex(v);
      for(unsigned int p : data.vertex_polygons[v]) {
        if(seen[p]) {
          continue;
        }
        seen[p] = 1;
        face(p);
        /* its edges with a near vertex, each once */
        const auto& polygon = data.polygons[p];
        const ksl_vec3_t np = polygon_normal(p);
        for(size_t k = 0; k < polygon.size(); k++) {
          const unsigned int i = polygon[k];
          const unsigned int j = polygon[(k + 1) % polygon.size()];
          if(!is_near[i] && !is_near[j]) {
            continue;
          }
          /* both polygons of the edge have its near vertex: the edge is
          solved from the one of the lesser index */
          const int other = AcrossEdge(data, p, i, j);
          if(other < 0 || static_cast<unsigned int>(other) < p) {
            continue;
          }
          const ksl_vec3_t nq = polygon_normal(static_cast<unsigned int>(other));
          side_edge(w(i), w(j), np, nq);
          rim_edge(w(i), w(j), np, nq);
        }
      }
    }
    /* an end whose extreme vertex of the hull is near */
    for(double sign : {1.0, -1.0}) {
      ksl_vec3_t local, global;
      int end_hint = hint;
      hull.Support(Scale(-sign, a), local, global, end_hint);
      if(end_hint >= 0 && static_cast<size_t>(end_hint) < is_near.size() &&
         is_near[end_hint]) {
        end(sign);
      }
    }
  }
  if(!best.found) {
    return false;
  }
  contact.collision = best.depth > 0.0;
  if(contact.collision) {
    contact.depth = best.depth;
    contact.normal = best.n;
    contact.point_first = best.x;
    contact.point_second = best.y;
  }
  return true;
}
