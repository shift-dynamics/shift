/* ray casts against collision geometry (Base::RayCast): planes, spheres and
boxes in closed form, CAD geometry on its exact faces, other convex geometry
by GJK on its support function (RayCastConvex) */

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "ksl/vec3.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/box.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"

static inline ksl_vec3_t Add(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x + b.x, a.y + b.y, a.z + b.z}};
}

static inline ksl_vec3_t Sub(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return {{a.x - b.x, a.y - b.y, a.z - b.z}};
}

static inline ksl_vec3_t Scale(double s, const ksl_vec3_t& a) {
  return {{s * a.x, s * a.y, s * a.z}};
}

static inline double Dot(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline ksl_vec3_t Negative(const ksl_vec3_t& a) {
  return {{-a.x, -a.y, -a.z}};
}

/* R^T v: v in the frame of d */
static inline ksl_vec3_t ToFrame(const ksl_SE3_t& d, const ksl_vec3_t& v) {
  return {{Dot(d.R.v0, v), Dot(d.R.v1, v), Dot(d.R.v2, v)}};
}

/* R v: v of the frame of d in the inertial frame */
static inline ksl_vec3_t FromFrame(const ksl_SE3_t& d, const ksl_vec3_t& v) {
  ksl_vec3_t r;
  for(int k = 0; k < 3; k++) {
    r.at[k] = d.R.v0.at[k] * v.x + d.R.v1.at[k] * v.y + d.R.v2.at[k] * v.z;
  }
  return r;
}

/* the entry and exit of the ray o + t d through the box [lo, hi] (frame of
the box): false if it misses */
static bool SlabInterval(const ksl_vec3_t& o, const ksl_vec3_t& d,
                         const ksl_vec3_t& lo, const ksl_vec3_t& hi,
                         double& t_in, double& t_out, int& axis_in) {
  t_in = -HUGE_VAL;
  t_out = HUGE_VAL;
  axis_in = -1;
  for(int k = 0; k < 3; k++) {
    if(std::fabs(d.at[k]) < 1e-300) {
      if(o.at[k] < lo.at[k] || o.at[k] > hi.at[k]) {
        return false;
      }
      continue;
    }
    double t1 = (lo.at[k] - o.at[k]) / d.at[k];
    double t2 = (hi.at[k] - o.at[k]) / d.at[k];
    if(t1 > t2) {
      std::swap(t1, t2);
    }
    if(t1 > t_in) {
      t_in = t1;
      axis_in = k;
    }
    t_out = std::min(t_out, t2);
    if(t_in > t_out) {
      return false;
    }
  }
  return true;
}

bool shift::core::collision_geometry::Base::RayCast(
  const ksl_vec3_t& origin, const ksl_vec3_t& direction, double max_distance,
  double& distance, ksl_vec3_t& normal) const {
  return RayCastConvex(*this, origin, direction, max_distance, distance,
                       normal);
}

/*
The point of least norm of the convex hull of the points w (at most 5), and
the points of the face it lies in, by trying each subset: the point of least
norm of a subset's affine hull is a candidate if it is inside the subset's
hull (all its barycentric coordinates positive), and the least of the
candidates is the point of the hull. keep is set to the subset.
*/
static ksl_vec3_t ClosestToOrigin(const std::vector<ksl_vec3_t>& w,
                                  std::vector<int>& keep) {
  const int n = static_cast<int>(w.size());
  double best = HUGE_VAL;
  ksl_vec3_t closest = w[0];
  keep.assign(1, 0);
  for(int mask = 1; mask < (1 << n); mask++) {
    int index[5], m = 0;
    for(int k = 0; k < n; k++) {
      if(mask & (1 << k)) {
        index[m++] = k;
      }
    }
    if(m > 4) {
      continue;
    }
    /* p = w0 + sum_a mu_a (w_a - w0), with G mu = -b */
    const ksl_vec3_t& w0 = w[index[0]];
    ksl_vec3_t e[3];
    for(int a = 1; a < m; a++) {
      e[a - 1] = Sub(w[index[a]], w0);
    }
    const int q = m - 1;
    double mu[3] = {0.0, 0.0, 0.0};
    if(q > 0) {
      double g[3][3], b[3];
      double scale = 0.0;
      for(int a = 0; a < q; a++) {
        for(int c = 0; c < q; c++) {
          g[a][c] = Dot(e[a], e[c]);
        }
        b[a] = -Dot(e[a], w0);
        scale = std::max(scale, g[a][a]);
      }
      /* Gaussian elimination with partial pivoting; skip degenerate subsets */
      bool singular = false;
      for(int c = 0; c < q && !singular; c++) {
        int pivot = c;
        for(int r = c + 1; r < q; r++) {
          if(std::fabs(g[r][c]) > std::fabs(g[pivot][c])) {
            pivot = r;
          }
        }
        if(std::fabs(g[pivot][c]) <= 1e-12 * scale) {
          singular = true;
          break;
        }
        std::swap(g[c], g[pivot]);
        std::swap(b[c], b[pivot]);
        for(int r = c + 1; r < q; r++) {
          const double f = g[r][c] / g[c][c];
          for(int k = c; k < q; k++) {
            g[r][k] -= f * g[c][k];
          }
          b[r] -= f * b[c];
        }
      }
      if(singular) {
        continue;
      }
      for(int c = q - 1; c >= 0; c--) {
        double sum = b[c];
        for(int k = c + 1; k < q; k++) {
          sum -= g[c][k] * mu[k];
        }
        mu[c] = sum / g[c][c];
      }
      double mu0 = 1.0;
      bool inside = true;
      for(int a = 0; a < q; a++) {
        mu0 -= mu[a];
        inside = inside && mu[a] > 0.0;
      }
      if(!inside || !(mu0 > 0.0)) {
        continue;
      }
    }
    ksl_vec3_t p = w0;
    for(int a = 0; a < q; a++) {
      p = Add(p, Scale(mu[a], e[a]));
    }
    const double norm = Dot(p, p);
    /* the smallest subset wins a tie, as it has the fewest points */
    if(norm < best * (1.0 - 1e-12)) {
      best = norm;
      closest = p;
      keep.assign(index, index + m);
    }
  }
  return closest;
}

bool shift::core::collision_geometry::RayCastConvex(
  const Base& geometry, const ksl_vec3_t& origin, const ksl_vec3_t& direction,
  double max_distance, double& distance, ksl_vec3_t& normal) {
  constexpr int kMaxIterations = 100;
  const double scale = std::max(geometry.get_bounding_radius(), 1e-12);
  const double tolerance = 1e-9 * scale;
  int hint = 0;
  ksl_vec3_t local, p;
  /* any point of the geometry starts the search */
  geometry.Support(Negative(direction), local, p, hint);
  double lambda = 0.0;
  ksl_vec3_t x = origin;
  ksl_vec3_t n = {{0.0, 0.0, 0.0}};
  ksl_vec3_t v = Sub(x, p);
  std::vector<ksl_vec3_t> points; // of the geometry, the simplex is x - points
  std::vector<ksl_vec3_t> w;
  std::vector<int> keep;
  int iteration = 0;
  for(; iteration < kMaxIterations; iteration++) {
    if(Dot(v, v) <= tolerance * tolerance) {
      break;
    }
    geometry.Support(v, local, p, hint);
    const ksl_vec3_t to_x = Sub(x, p);
    const double vw = Dot(v, to_x);
    if(vw > 0.0) {
      /* x is outside the half space of the support plane: move it along
      the ray to the plane */
      const double vr = Dot(v, direction);
      if(vr >= 0.0) {
        return false; // the ray runs away from the geometry
      }
      lambda -= vw / vr;
      if(lambda > max_distance) {
        return false;
      }
      x = Add(origin, Scale(lambda, direction));
      n = v;
    }
    /* add p to the simplex, unless it is already there */
    bool known = false;
    for(const auto& q : points) {
      const ksl_vec3_t d = Sub(q, p);
      known = known || Dot(d, d) <= tolerance * tolerance;
    }
    if(!known) {
      points.push_back(p);
    } else if(vw <= 0.0) {
      /* no progress: x is as close as the support function can tell */
      break;
    }
    w.clear();
    for(const auto& q : points) {
      w.push_back(Sub(x, q));
    }
    v = ClosestToOrigin(w, keep);
    std::vector<ksl_vec3_t> kept;
    for(int k : keep) {
      kept.push_back(points[k]);
    }
    points.swap(kept);
  }
  /* stopped without reaching the geometry: the support function cannot
  bring x closer, or the iterations ran out */
  if(Dot(v, v) > 1e-12 * scale * scale) {
    return false;
  }
  distance = lambda;
  const double length = std::sqrt(Dot(n, n));
  normal = length > 0.0 ? Scale(1.0 / length, n) : Negative(direction);
  return true;
}

bool shift::core::collision_geometry::Sphere::RayCast(
  const ksl_vec3_t& origin, const ksl_vec3_t& direction, double max_distance,
  double& distance, ksl_vec3_t& normal) const {
  const ksl_vec3_t m = Sub(origin, get_center());
  const double c = Dot(m, m) - radius_ * radius_;
  if(c <= 0.0) {
    distance = 0.0;
    normal = Negative(direction);
    return true;
  }
  const double b = Dot(m, direction);
  const double discriminant = b * b - c;
  if(b >= 0.0 || discriminant < 0.0) {
    return false;
  }
  const double t = -b - std::sqrt(discriminant);
  if(t > max_distance) {
    return false;
  }
  distance = t;
  normal = Scale(1.0 / radius_, Add(m, Scale(t, direction)));
  return true;
}

bool shift::core::collision_geometry::Box::RayCast(
  const ksl_vec3_t& origin, const ksl_vec3_t& direction, double max_distance,
  double& distance, ksl_vec3_t& normal) const {
  const ksl_SE3_t& d = get_d_absolute();
  const ksl_vec3_t o = ToFrame(d, Sub(origin, d.t));
  const ksl_vec3_t r = ToFrame(d, direction);
  const ksl_vec3_t& h = half_extents_;
  if(std::fabs(o.x) <= h.x && std::fabs(o.y) <= h.y && std::fabs(o.z) <= h.z) {
    distance = 0.0;
    normal = Negative(direction);
    return true;
  }
  double t_in, t_out;
  int axis;
  if(!SlabInterval(o, r, Negative(h), h, t_in, t_out, axis) || t_out < 0.0 ||
     t_in < 0.0 || t_in > max_distance || axis < 0) {
    return false;
  }
  distance = t_in;
  ksl_vec3_t local = {{0.0, 0.0, 0.0}};
  local.at[axis] = r.at[axis] > 0.0 ? -1.0 : 1.0;
  normal = FromFrame(d, local);
  return true;
}

bool shift::core::collision_geometry::Plane::RayCast(
  const ksl_vec3_t& origin, const ksl_vec3_t& direction, double max_distance,
  double& distance, ksl_vec3_t& normal) const {
  const ksl_vec3_t n = get_normal();
  const double height = Dot(n, Sub(origin, get_point()));
  if(height <= 0.0) {
    distance = 0.0;
    normal = Negative(direction);
    return true;
  }
  const double rate = Dot(n, direction);
  if(rate >= 0.0) {
    return false;
  }
  const double t = -height / rate;
  if(t > max_distance) {
    return false;
  }
  distance = t;
  normal = n;
  return true;
}

/*
The faces whose boxes the ray (frame of the geometry) crosses within
max_distance, by the box tree of the shape.
*/
static void FacesAlongRay(
  const shift::core::collision_geometry::CadContactShape& shape,
  const ksl_vec3_t& o, const ksl_vec3_t& r, double max_distance,
  std::vector<int>& faces) {
  if(shape.nodes.empty()) {
    return;
  }
  std::vector<int> stack = {0};
  while(!stack.empty()) {
    const auto& node = shape.nodes[stack.back()];
    stack.pop_back();
    double t_in, t_out;
    int axis;
    if(!SlabInterval(o, r, node.box.lo, node.box.hi, t_in, t_out, axis) ||
       t_out < 0.0 || t_in > max_distance) {
      continue;
    }
    if(node.left < 0) {
      for(int k = node.first; k < node.first + node.count; k++) {
        faces.push_back(shape.order[k]);
      }
    } else {
      stack.push_back(node.left);
      stack.push_back(node.right);
    }
  }
}

bool shift::core::collision_geometry::Cad::RayCast(
  const ksl_vec3_t& origin, const ksl_vec3_t& direction, double max_distance,
  double& distance, ksl_vec3_t& normal) const {
  constexpr int kStarts = 4;        // samples of a face Newton starts from
  constexpr int kMaxIterations = 30;
  if(!shape_) {
    return false;
  }
  const CadContactShape& shape = *shape_;
  const ksl_SE3_t& d = get_d_absolute();
  const ksl_vec3_t o = ToFrame(d, Sub(origin, d.t));
  const ksl_vec3_t r = ToFrame(d, direction);
  const double tolerance = 1e-10 * std::max(shape.size, 1e-12);
  std::vector<int> faces;
  FacesAlongRay(shape, o, r, max_distance, faces);
  double best = HUGE_VAL;
  ksl_vec3_t best_normal = {{0.0, 0.0, 0.0}};
  for(int face : faces) {
    const CadContactFace& f = shape.faces[face];
    /* the samples nearest the line of the ray */
    std::vector<std::pair<double, int>> near;
    for(int k = 0; k < static_cast<int>(f.sample_points.size()); k++) {
      const ksl_vec3_t m = Sub(f.sample_points[k], o);
      const ksl_vec3_t off = Sub(m, Scale(Dot(m, r), r));
      near.push_back({Dot(off, off), k});
    }
    const int starts = std::min(kStarts, static_cast<int>(near.size()));
    std::partial_sort(near.begin(), near.begin() + starts, near.end());
    for(int s = 0; s < starts; s++) {
      const int k = near[s].second;
      cad::Vec2 uv = f.sample_uv[k];
      double t = Dot(Sub(f.sample_points[k], o), r);
      bool converged = false;
      for(int iteration = 0; iteration < kMaxIterations; iteration++) {
        /* F = P(u, v) - (o + t r) = 0, J = [P_u, P_v, -r] */
        ksl_vec3_t du, dv;
        const ksl_vec3_t p = shape.Point(face, uv, &du, &dv);
        const ksl_vec3_t residual = Sub(p, Add(o, Scale(t, r)));
        if(Dot(residual, residual) <= tolerance * tolerance) {
          converged = true;
          break;
        }
        const ksl_vec3_t c0 = du, c1 = dv, c2 = Negative(r);
        /* Cramer's rule for J delta = -residual */
        auto det3 = [](const ksl_vec3_t& a, const ksl_vec3_t& b,
                       const ksl_vec3_t& c) {
          return a.x * (b.y * c.z - b.z * c.y) - b.x * (a.y * c.z - a.z * c.y) +
                 c.x * (a.y * b.z - a.z * b.y);
        };
        const double det = det3(c0, c1, c2);
        if(std::fabs(det) < 1e-300) {
          break; // the ray runs along the surface
        }
        const ksl_vec3_t rhs = Negative(residual);
        uv[0] += det3(rhs, c1, c2) / det;
        uv[1] += det3(c0, rhs, c2) / det;
        t += det3(c0, c1, rhs) / det;
        if(!std::isfinite(uv[0]) || !std::isfinite(uv[1]) ||
           !std::isfinite(t)) {
          break;
        }
      }
      if(!converged || t < -tolerance || t > max_distance || t >= best ||
         !shape.Inside(face, uv)) {
        continue;
      }
      best = std::max(t, 0.0);
      best_normal = shape.Normal(face, uv);
    }
  }
  if(best == HUGE_VAL) {
    return false;
  }
  if(Dot(best_normal, r) > 0.0) {
    /* the first point leaves the solid: the ray starts inside it */
    distance = 0.0;
    normal = Negative(direction);
    return true;
  }
  distance = best;
  normal = FromFrame(d, best_normal);
  return true;
}

/* ------------------------------------------------------------------------ */
/* grids of parallel rays */

void shift::core::collision_geometry::Base::RayCastGrid(const RayGrid& grid,
                                                        double* distances) const {
  for(long j = 0; j < grid.count_j; j++) {
    for(long i = 0; i < grid.count_i; i++) {
      const long k = i + j * grid.count_i;
      distances[k] = HUGE_VAL;
      const double offset = grid.offsets[k];
      if(std::isnan(offset)) {
        continue;
      }
      const ksl_vec3_t origin =
        Sub(Add(grid.corner, Add(Scale(double(i), grid.step_i),
                                 Scale(double(j), grid.step_j))),
            Scale(offset, grid.direction));
      double t;
      ksl_vec3_t normal;
      if(RayCast(origin, grid.direction, grid.reach, t, normal)) {
        distances[k] = t;
      }
    }
  }
}

/* the largest distance of a face's surface from the chords of its mesh, at
the middles of the cells' sides along u, along v, and of their diagonals */
static void MeshDeviation(const shift::core::collision_geometry::CadContactShape& shape,
                          int face,
                          const shift::core::collision_geometry::CadFaceMesh& mesh,
                          double& along_u, double& along_v, double& across) {
  along_u = along_v = across = 0.0;
  const int row = mesh.nu + 1;
  auto deviation = [&](int a, int b) {
    const shift::core::cad::Vec2 uv{0.5 * (mesh.uv[a][0] + mesh.uv[b][0]),
                                    0.5 * (mesh.uv[a][1] + mesh.uv[b][1])};
    const ksl_vec3_t p = shape.Point(face, uv);
    const ksl_vec3_t chord =
      Scale(0.5, Add(mesh.points[a], mesh.points[b]));
    const ksl_vec3_t off = Sub(p, chord);
    return std::sqrt(Dot(off, off));
  };
  for(int v = 0; v <= mesh.nv; v++) {
    for(int u = 0; u <= mesh.nu; u++) {
      const int k = u + v * row;
      if(u < mesh.nu) {
        along_u = std::max(along_u, deviation(k, k + 1));
      }
      if(v < mesh.nv) {
        along_v = std::max(along_v, deviation(k, k + row));
      }
      if(u < mesh.nu && v < mesh.nv) {
        across = std::max(across, deviation(k, k + row + 1));
      }
    }
  }
}

/* each face as a grid in (u, v) over the bounds of its trims, halved along u
or v until the surface is within 0.1% of the shape's size of the chords */
static std::vector<shift::core::collision_geometry::CadFaceMesh>
MeshFaces(const shift::core::collision_geometry::CadContactShape& shape) {
  constexpr int kMaxCells = 64;
  const double tolerance = 1e-3 * shape.size;
  std::vector<shift::core::collision_geometry::CadFaceMesh> meshes(
    shape.faces.size());
  for(size_t face = 0; face < shape.faces.size(); face++) {
    const auto& f = shape.faces[face];
    auto& mesh = meshes[face];
    for(;;) {
      mesh.uv.clear();
      mesh.points.clear();
      for(int v = 0; v <= mesh.nv; v++) {
        for(int u = 0; u <= mesh.nu; u++) {
          const shift::core::cad::Vec2 uv{
            f.lo[0] + (f.hi[0] - f.lo[0]) * u / mesh.nu,
            f.lo[1] + (f.hi[1] - f.lo[1]) * v / mesh.nv};
          mesh.uv.push_back(uv);
          mesh.points.push_back(shape.Point(static_cast<int>(face), uv));
        }
      }
      double along_u, along_v, across;
      MeshDeviation(shape, static_cast<int>(face), mesh, along_u, along_v,
                    across);
      mesh.error = std::max({along_u, along_v, across});
      bool refined = false;
      if(along_u > tolerance && mesh.nu < kMaxCells) {
        mesh.nu *= 2;
        refined = true;
      }
      if(along_v > tolerance && mesh.nv < kMaxCells) {
        mesh.nv *= 2;
        refined = true;
      }
      if(!refined && across > tolerance && mesh.nu < kMaxCells &&
         mesh.nv < kMaxCells) {
        mesh.nu *= 2;
        mesh.nv *= 2;
        refined = true;
      }
      if(!refined) {
        break;
      }
    }
  }
  return meshes;
}

void shift::core::collision_geometry::Cad::RayCastGrid(const RayGrid& grid,
                                                       double* distances) const {
  constexpr int kMaxIterations = 10;
  const long count = grid.count_i * grid.count_j;
  std::fill(distances, distances + count, HUGE_VAL);
  if(!shape_ || count <= 0) {
    return;
  }
  const CadContactShape& shape = *shape_;
  if(!face_meshes_) {
    face_meshes_ =
      std::make_shared<const std::vector<CadFaceMesh>>(MeshFaces(shape));
  }
  const std::vector<CadFaceMesh>& meshes = *face_meshes_;

  /* the grid in the frame of the geometry, and the coordinates (i, j, s) of
  a point p = c + i a + j b + s r there */
  const ksl_SE3_t& d = get_d_absolute();
  const ksl_vec3_t c = ToFrame(d, Sub(grid.corner, d.t));
  const ksl_vec3_t a = ToFrame(d, grid.step_i);
  const ksl_vec3_t b = ToFrame(d, grid.step_j);
  const ksl_vec3_t r = ToFrame(d, grid.direction);
  ksl_vec3_t b_r, r_a, a_b;
  ksl_cross_vv(&b, &r, &b_r);
  ksl_cross_vv(&r, &a, &r_a);
  ksl_cross_vv(&a, &b, &a_b);
  const double det = Dot(a, b_r);
  if(!(std::fabs(det) > 0.0)) {
    return;
  }
  struct GridPoint {
    double i, j, s;
  };
  auto to_grid = [&](const ksl_vec3_t& p) {
    const ksl_vec3_t q = Sub(p, c);
    return GridPoint{Dot(q, b_r) / det, Dot(q, r_a) / det, Dot(q, a_b) / det};
  };
  double offset_lo = HUGE_VAL, offset_hi = -HUGE_VAL;
  for(long k = 0; k < count; k++) {
    if(!std::isnan(grid.offsets[k])) {
      offset_lo = std::min(offset_lo, grid.offsets[k]);
      offset_hi = std::max(offset_hi, grid.offsets[k]);
    }
  }
  if(offset_lo > offset_hi) {
    return;
  }
  const double tolerance = 1e-10 * std::max(shape.size, 1e-12);
  const double tie = 10.0 * tolerance; // crossings at one point
  /* the length of a step of the grid, for the triangles' padding */
  const double step = std::sqrt(std::min(Dot(a, a), Dot(b, b)));
  std::vector<double> best(count, HUGE_VAL);
  std::vector<char> leaving(count, 0);
  std::vector<GridPoint> projected;

  for(size_t face = 0; face < shape.faces.size(); face++) {
    const CadContactFace& f = shape.faces[face];
    const CadFaceMesh& mesh = meshes[face];
    /* the face's box on the grid: skip it if it is off the grid or beyond
    the reach of the rays */
    double lo[3] = {HUGE_VAL, HUGE_VAL, HUGE_VAL};
    double hi[3] = {-HUGE_VAL, -HUGE_VAL, -HUGE_VAL};
    for(int corner = 0; corner < 8; corner++) {
      const ksl_vec3_t p = {{(corner & 1) ? f.box.hi.x : f.box.lo.x,
                             (corner & 2) ? f.box.hi.y : f.box.lo.y,
                             (corner & 4) ? f.box.hi.z : f.box.lo.z}};
      const GridPoint g = to_grid(p);
      const double v[3] = {g.i, g.j, g.s};
      for(int n = 0; n < 3; n++) {
        lo[n] = std::min(lo[n], v[n]);
        hi[n] = std::max(hi[n], v[n]);
      }
    }
    const double box_margin = 2.0 * mesh.error + tolerance;
    if(hi[0] < -1e-9 || lo[0] > grid.count_i - 1 + 1e-9 || hi[1] < -1e-9 ||
       lo[1] > grid.count_j - 1 + 1e-9 ||
       hi[2] + offset_hi < -box_margin ||
       lo[2] + offset_lo > grid.reach + box_margin) {
      continue;
    }
    projected.resize(mesh.points.size());
    for(size_t n = 0; n < mesh.points.size(); n++) {
      projected[n] = to_grid(mesh.points[n]);
    }
    const int row = mesh.nu + 1;
    for(int v = 0; v < mesh.nv; v++) {
      for(int u = 0; u < mesh.nu; u++) {
        const int k00 = u + v * row, k10 = k00 + 1, k01 = k00 + row,
                  k11 = k01 + 1;
        const int triangles[2][3] = {{k00, k10, k11}, {k00, k11, k01}};
        for(const auto& tri : triangles) {
          const GridPoint& p0 = projected[tri[0]];
          const GridPoint& p1 = projected[tri[1]];
          const GridPoint& p2 = projected[tri[2]];
          const double area =
            (p1.i - p0.i) * (p2.j - p0.j) - (p2.i - p0.i) * (p1.j - p0.j);
          if(!(std::fabs(area) > 1e-12)) {
            continue; // seen edge on: the rays run along it
          }
          /* the error of the triangle's estimate of t, the surface being
          within the mesh's error of its plane */
          ksl_vec3_t e1 = Sub(mesh.points[tri[1]], mesh.points[tri[0]]);
          ksl_vec3_t e2 = Sub(mesh.points[tri[2]], mesh.points[tri[0]]);
          ksl_vec3_t n;
          ksl_cross_vv(&e1, &e2, &n);
          const double length = std::sqrt(Dot(n, n));
          const double cosine =
            length > 0.0 ? std::fabs(Dot(n, r)) / length : 0.0;
          const double margin =
            2.0 * mesh.error / std::max(cosine, 0.05) + tolerance;
          /* the surface reaches up to the mesh's error beyond the chords,
          so the triangle covers the nodes within that of its sides (in steps
          of the grid), where the true surface may be: near the outline of a
          cylinder seen from the side */
          const double pad = 2.0 * mesh.error / step + 1e-9;
          const double side[3] = {std::hypot(p2.i - p1.i, p2.j - p1.j),
                                  std::hypot(p0.i - p2.i, p0.j - p2.j),
                                  std::hypot(p1.i - p0.i, p1.j - p0.j)};
          const long i0 = std::max(
            0L, static_cast<long>(std::ceil(
                  std::min({p0.i, p1.i, p2.i}) - pad)));
          const long i1 = std::min(
            grid.count_i - 1, static_cast<long>(std::floor(
                                std::max({p0.i, p1.i, p2.i}) + pad)));
          const long j0 = std::max(
            0L, static_cast<long>(std::ceil(
                  std::min({p0.j, p1.j, p2.j}) - pad)));
          const long j1 = std::min(
            grid.count_j - 1, static_cast<long>(std::floor(
                                std::max({p0.j, p1.j, p2.j}) + pad)));
          for(long j = j0; j <= j1; j++) {
            for(long i = i0; i <= i1; i++) {
              /* barycentric weights of node (i, j) in the triangle */
              const double w1 =
                ((i - p0.i) * (p2.j - p0.j) - (p2.i - p0.i) * (j - p0.j)) /
                area;
              const double w2 =
                ((p1.i - p0.i) * (j - p0.j) - (i - p0.i) * (p1.j - p0.j)) /
                area;
              const double w0 = 1.0 - w1 - w2;
              /* w times the altitude over the opposite side is the distance
              inside that side, |area| / side the altitude */
              const double alt = std::fabs(area);
              if(w0 * alt < -pad * side[0] || w1 * alt < -pad * side[1] ||
                 w2 * alt < -pad * side[2]) {
                continue;
              }
              const long k = i + j * grid.count_i;
              const double offset = grid.offsets[k];
              if(std::isnan(offset)) {
                continue;
              }
              double t = w0 * p0.s + w1 * p1.s + w2 * p2.s + offset;
              if(t < -margin || t > grid.reach + margin ||
                 t > best[k] + margin) {
                continue;
              }
              cad::Vec2 uv{w0 * mesh.uv[tri[0]][0] + w1 * mesh.uv[tri[1]][0] +
                             w2 * mesh.uv[tri[2]][0],
                           w0 * mesh.uv[tri[0]][1] + w1 * mesh.uv[tri[1]][1] +
                             w2 * mesh.uv[tri[2]][1]};
              const ksl_vec3_t o =
                Sub(Add(c, Add(Scale(double(i), a), Scale(double(j), b))),
                    Scale(offset, r));
              /* Newton's method for P(u, v) = o + t r, as in RayCast */
              bool converged = false;
              for(int iteration = 0; iteration < kMaxIterations; iteration++) {
                ksl_vec3_t du, dv;
                const ksl_vec3_t p =
                  shape.Point(static_cast<int>(face), uv, &du, &dv);
                const ksl_vec3_t residual = Sub(p, Add(o, Scale(t, r)));
                if(Dot(residual, residual) <= tolerance * tolerance) {
                  converged = true;
                  break;
                }
                const ksl_vec3_t c2 = Negative(r);
                auto det3 = [](const ksl_vec3_t& x, const ksl_vec3_t& y,
                               const ksl_vec3_t& z) {
                  return x.x * (y.y * z.z - y.z * z.y) -
                         y.x * (x.y * z.z - x.z * z.y) +
                         z.x * (x.y * y.z - x.z * y.y);
                };
                const double jacobian = det3(du, dv, c2);
                if(std::fabs(jacobian) < 1e-300) {
                  break;
                }
                const ksl_vec3_t rhs = Negative(residual);
                uv[0] += det3(rhs, dv, c2) / jacobian;
                uv[1] += det3(du, rhs, c2) / jacobian;
                t += det3(du, dv, rhs) / jacobian;
                if(!std::isfinite(uv[0]) || !std::isfinite(uv[1]) ||
                   !std::isfinite(t)) {
                  break;
                }
              }
              if(!converged || t < -tolerance || t > grid.reach ||
                 t > std::max(best[k], 0.0) + tie ||
                 !shape.Inside(static_cast<int>(face), uv)) {
                continue;
              }
              const bool leaves =
                Dot(shape.Normal(static_cast<int>(face), uv), r) > 0.0;
              if(t < best[k] - tie) {
                best[k] = std::max(t, 0.0);
                leaving[k] = leaves;
              } else if(leaves) {
                /* where a face it leaves meets one it enters (two bodies
                touching), the ray was inside before the point */
                best[k] = std::min(best[k], std::max(t, 0.0));
                leaving[k] = 1;
              }
            }
          }
        }
      }
    }
  }
  for(long k = 0; k < count; k++) {
    if(best[k] != HUGE_VAL) {
      /* the first point leaves the solid: the ray starts inside it */
      distances[k] = leaving[k] ? 0.0 : best[k];
    }
  }
}
