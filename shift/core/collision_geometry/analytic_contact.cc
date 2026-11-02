#include "shift/core/collision_geometry/analytic_contact.h"

#include <algorithm>
#include <cmath>

#include "shift/core/collision_geometry/convex_hull.h"
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

bool shift::core::collision_geometry::EllipsoidSphereContact(
  const Ellipsoid& ellipsoid, const Sphere& sphere, AnalyticContact& contact) {
  const ksl_vec3_t& axes = ellipsoid.get_semi_axes();
  if(std::min({axes.x, axes.y, axes.z}) <= 0.0) {
    return false;
  }
  contact = AnalyticContact();
  const ksl_SE3_t& d = ellipsoid.get_d_absolute();
  const ksl_vec3_t c = ToLocal(d, sphere.get_center());

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
  ksl_vec3_t q;
  double inside = 0.0;
  for(int k = 0; k < 3; k++) {
    const int i = order[k];
    q.at[i] = std::copysign(x[k], c.at[i]);
    inside += (c.at[i] / axes.at[i]) * (c.at[i] / axes.at[i]);
  }
  /* the outward normal at q is along the gradient of sum (x_i / a_i)^2 */
  ksl_vec3_t n = {{q.x / (axes.x * axes.x), q.y / (axes.y * axes.y),
                   q.z / (axes.z * axes.z)}};
  n = Scale(1.0 / std::sqrt(Dot(n, n)), n);
  const ksl_vec3_t r = Sub(c, q);
  const double distance = std::sqrt(Dot(r, r));
  SetSphereContact(d, q, n, inside < 1.0 ? -distance : distance, sphere,
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
    /* gradient: the support offsets minus r, i.e. point_a - point_b */
    const ksl_vec3_t gradient =
      Sub(Add(Scale(1.0 / ha, gan), Scale(1.0 / hb, gbn)), r);
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
          Dot(*e[i], Multiply(gb, *e[j])) / hb -
          Dot(*e[i], gbn) * Dot(*e[j], gbn) / (hb * hb * hb) -
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
    /* backtrack until the distance decreases */
    bool improved = false;
    for(int halving = 0; halving < 30 && !improved; halving++) {
      ksl_vec3_t trial = Add(n, Add(Scale(step[0], e1), Scale(step[1], e2)));
      trial = Scale(1.0 / std::sqrt(Dot(trial, trial)), trial);
      const double trial_depth = distance(trial);
      if(trial_depth <= depth) {
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
