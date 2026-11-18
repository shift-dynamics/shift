#include "shift/core/geometry/simplify.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <unordered_map>

/* a quadric error form, a symmetric 4x4 matrix of the planes it sums, each
weighted, with the sum of the weights: the weighted mean squared distance of
a point x to the planes is (x^T A x + 2 b.x + c) / w */
struct SimplifyQuadricForm {
  double a00 = 0.0, a11 = 0.0, a22 = 0.0, a01 = 0.0, a12 = 0.0, a02 = 0.0;
  double b0 = 0.0, b1 = 0.0, b2 = 0.0, c = 0.0, w = 0.0;
};

/* adds the plane n.x + d = 0, n of unit length, with weight */
static void SimplifyAddPlane(SimplifyQuadricForm& q, double nx, double ny,
                             double nz, double d, double weight) {
  q.a00 += weight * nx * nx;
  q.a11 += weight * ny * ny;
  q.a22 += weight * nz * nz;
  q.a01 += weight * nx * ny;
  q.a12 += weight * ny * nz;
  q.a02 += weight * nx * nz;
  q.b0 += weight * nx * d;
  q.b1 += weight * ny * d;
  q.b2 += weight * nz * d;
  q.c += weight * d * d;
  q.w += weight;
}

static void SimplifyAddQuadric(SimplifyQuadricForm& q,
                               const SimplifyQuadricForm& r) {
  q.a00 += r.a00;
  q.a11 += r.a11;
  q.a22 += r.a22;
  q.a01 += r.a01;
  q.a12 += r.a12;
  q.a02 += r.a02;
  q.b0 += r.b0;
  q.b1 += r.b1;
  q.b2 += r.b2;
  q.c += r.c;
  q.w += r.w;
}

/* the weighted sum of the squared distances of (x, y, z) to the planes */
static double SimplifyEvaluate(const SimplifyQuadricForm& q, double x,
                               double y, double z) {
  const double r = q.a00 * x * x + q.a11 * y * y + q.a22 * z * z +
                   2.0 * (q.a01 * x * y + q.a12 * y * z + q.a02 * x * z) +
                   2.0 * (q.b0 * x + q.b1 * y + q.b2 * z) + q.c;
  return std::max(r, 0.0);
}

/* the key of the undirected edge between a and b */
static uint64_t SimplifyEdgeKey(uint32_t a, uint32_t b) {
  if(a > b) {
    std::swap(a, b);
  }
  return (uint64_t(a) << 32) | b;
}

/* positions scaled into the unit box (as doubles): the largest side of the
bounding box becomes 1; returns that side */
static double SimplifyNormalize(const std::vector<float>& positions,
                                std::vector<double>& normalized) {
  const size_t n = positions.size() / 3;
  double lo[3] = {std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max()};
  double hi[3] = {std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest()};
  for(size_t i = 0; i < n; i++) {
    for(int k = 0; k < 3; k++) {
      lo[k] = std::min(lo[k], double(positions[3 * i + k]));
      hi[k] = std::max(hi[k], double(positions[3 * i + k]));
    }
  }
  double extent = 0.0;
  for(int k = 0; k < 3; k++) {
    extent = std::max(extent, hi[k] - lo[k]);
  }
  if(n == 0 || extent <= 0.0) {
    extent = 1.0;
  }
  normalized.resize(3 * n);
  for(size_t i = 0; i < n; i++) {
    for(int k = 0; k < 3; k++) {
      normalized[3 * i + k] = (positions[3 * i + k] - lo[k]) / extent;
    }
  }
  return extent;
}

/* the first vertex at the same position as each vertex: vertices equal in
position (the wedges of a seam) share it */
static std::vector<uint32_t> SimplifyPositionRemap(
  const std::vector<float>& positions) {
  const size_t n = positions.size() / 3;
  std::vector<uint32_t> order(n);
  std::iota(order.begin(), order.end(), 0u);
  auto less = [&](uint32_t a, uint32_t b) {
    for(int k = 0; k < 3; k++) {
      if(positions[3 * a + k] != positions[3 * b + k]) {
        return positions[3 * a + k] < positions[3 * b + k];
      }
    }
    return a < b;
  };
  std::sort(order.begin(), order.end(), less);
  std::vector<uint32_t> remap(n);
  for(size_t i = 0; i < n; i++) {
    const uint32_t v = order[i];
    if(i > 0) {
      const uint32_t u = order[i - 1];
      if(positions[3 * u] == positions[3 * v] &&
         positions[3 * u + 1] == positions[3 * v + 1] &&
         positions[3 * u + 2] == positions[3 * v + 2]) {
        remap[v] = remap[u];
        continue;
      }
    }
    remap[v] = v;
  }
  return remap;
}

/* the unnormalized normal of the triangle a, b, c (twice its area) */
static void SimplifyTriangleNormal(const double* a, const double* b,
                                   const double* c, double* n) {
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  n[0] = u[1] * v[2] - u[2] * v[1];
  n[1] = u[2] * v[0] - u[0] * v[2];
  n[2] = u[0] * v[1] - u[1] * v[0];
}

/* the kinds of vertices: what collapses a vertex may make */
enum class SimplifyKind : uint8_t {
  kManifold, // inside a closed sheet: onto any neighbor
  kBorder,   // on an open border: along the border
  kSeam,     // one of two wedges on a seam: along the seam, both wedges
  kLocked    // anything else: stays
};

std::vector<uint32_t> shift::core::geometry::SimplifyQuadric(
  const std::vector<float>& positions, const std::vector<float>& attributes,
  size_t attribute_count, const std::vector<uint32_t>& faces,
  const SimplifyOptions& options, double* error) {
  const size_t n = positions.size() / 3;
  std::vector<uint32_t> result = faces;
  double largest_error = 0.0;
  if(error) {
    *error = 0.0;
  }
  if(n == 0 || faces.size() / 3 <= options.target_face_count) {
    return result;
  }
  std::vector<double> p;
  SimplifyNormalize(positions, p);
  const std::vector<uint32_t> remap = SimplifyPositionRemap(positions);
  // the unit normal each face had in the input: a face may turn by a
  // little at each collapse, but never away from where it faced
  std::vector<double> original_normal(result.size(), 0.0);
  for(size_t f = 0; f + 2 < result.size(); f += 3) {
    SimplifyTriangleNormal(&p[3 * result[f]], &p[3 * result[f + 1]],
                           &p[3 * result[f + 2]], &original_normal[f]);
    const double length = std::sqrt(
      original_normal[f] * original_normal[f] +
      original_normal[f + 1] * original_normal[f + 1] +
      original_normal[f + 2] * original_normal[f + 2]);
    for(int k = 0; k < 3; k++) {
      original_normal[f + k] = length > 0.0 ? original_normal[f + k] / length
                                            : 0.0;
    }
  }
  const bool use_attributes =
    attribute_count > 0 && attributes.size() >= n * attribute_count &&
    options.attribute_weights.size() >= attribute_count;

  // the quadrics of the positions: the planes of their faces weighted by
  // area, and planes across the open borders, upright on the faces along
  // them, which hold the outline
  std::vector<SimplifyQuadricForm> quadric(n);
  {
    std::unordered_map<uint64_t, int> edge_faces;
    for(size_t f = 0; f + 2 < result.size(); f += 3) {
      for(int e = 0; e < 3; e++) {
        const uint32_t a = remap[result[f + e]];
        const uint32_t b = remap[result[f + (e + 1) % 3]];
        if(a != b) {
          edge_faces[SimplifyEdgeKey(a, b)]++;
        }
      }
    }
    for(size_t f = 0; f + 2 < result.size(); f += 3) {
      const uint32_t v[3] = {remap[result[f]], remap[result[f + 1]],
                             remap[result[f + 2]]};
      double normal[3];
      SimplifyTriangleNormal(&p[3 * v[0]], &p[3 * v[1]], &p[3 * v[2]],
                             normal);
      const double length = std::sqrt(normal[0] * normal[0] +
                                       normal[1] * normal[1] +
                                       normal[2] * normal[2]);
      if(length <= 0.0) {
        continue;
      }
      const double area = 0.5 * length;
      const double nx = normal[0] / length;
      const double ny = normal[1] / length;
      const double nz = normal[2] / length;
      const double d =
        -(nx * p[3 * v[0]] + ny * p[3 * v[0] + 1] + nz * p[3 * v[0] + 2]);
      for(int k = 0; k < 3; k++) {
        SimplifyAddPlane(quadric[v[k]], nx, ny, nz, d, area);
      }
      for(int e = 0; e < 3; e++) {
        const uint32_t a = v[e];
        const uint32_t b = v[(e + 1) % 3];
        if(a == b || edge_faces[SimplifyEdgeKey(a, b)] != 1) {
          continue;
        }
        // the plane through the edge, upright on the face
        double edge[3] = {p[3 * b] - p[3 * a], p[3 * b + 1] - p[3 * a + 1],
                          p[3 * b + 2] - p[3 * a + 2]};
        const double edge_length = std::sqrt(
          edge[0] * edge[0] + edge[1] * edge[1] + edge[2] * edge[2]);
        if(edge_length <= 0.0) {
          continue;
        }
        double m[3] = {edge[1] * nz - edge[2] * ny, edge[2] * nx - edge[0] * nz,
                       edge[0] * ny - edge[1] * nx};
        const double m_length =
          std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        for(double& x : m) {
          x /= m_length;
        }
        const double md =
          -(m[0] * p[3 * a] + m[1] * p[3 * a + 1] + m[2] * p[3 * a + 2]);
        const double weight = 10.0 * edge_length * edge_length;
        SimplifyAddPlane(quadric[a], m[0], m[1], m[2], md, weight);
        SimplifyAddPlane(quadric[b], m[0], m[1], m[2], md, weight);
      }
    }
  }

  const double max_cost = options.max_error * options.max_error;
  for(;;) {
    const size_t face_count = result.size() / 3;
    if(face_count <= options.target_face_count) {
      break;
    }
    // the edges of the positions and of the vertices, with their faces
    std::unordered_map<uint64_t, int> position_edges;
    std::unordered_map<uint64_t, int> vertex_edges;
    for(size_t f = 0; f < result.size(); f += 3) {
      for(int e = 0; e < 3; e++) {
        const uint32_t a = result[f + e];
        const uint32_t b = result[f + (e + 1) % 3];
        position_edges[SimplifyEdgeKey(remap[a], remap[b])]++;
        vertex_edges[SimplifyEdgeKey(a, b)]++;
      }
    }
    // the faces of each vertex
    std::vector<uint32_t> first(n + 1, 0);
    for(uint32_t v : result) {
      first[v + 1]++;
    }
    for(size_t i = 0; i < n; i++) {
      first[i + 1] += first[i];
    }
    std::vector<uint32_t> vertex_faces(result.size());
    {
      std::vector<uint32_t> fill(first.begin(), first.end() - 1);
      for(size_t i = 0; i < result.size(); i++) {
        vertex_faces[fill[result[i]]++] = uint32_t(i / 3);
      }
    }
    // the faces of each position (of all its wedges)
    std::vector<uint32_t> position_first(n + 1, 0);
    for(uint32_t v : result) {
      position_first[remap[v] + 1]++;
    }
    for(size_t i = 0; i < n; i++) {
      position_first[i + 1] += position_first[i];
    }
    std::vector<uint32_t> position_faces(result.size());
    {
      std::vector<uint32_t> fill(position_first.begin(),
                                 position_first.end() - 1);
      for(size_t i = 0; i < result.size(); i++) {
        position_faces[fill[remap[result[i]]]++] = uint32_t(i / 3);
      }
    }
    // the wedges in use of each position, its open and non-manifold edges
    std::vector<uint32_t> wedges(n, 0), wedge(n, 0), sibling(n, 0);
    std::vector<uint8_t> border_edges(n, 0), complex(n, 0);
    std::vector<uint8_t> open_edges(n, 0);
    for(size_t v = 0; v < n; v++) {
      if(first[v + 1] > first[v]) {
        const uint32_t q = remap[v];
        if(wedges[q] == 0) {
          wedge[q] = uint32_t(v);
        } else {
          sibling[v] = wedge[q];
          sibling[wedge[q]] = uint32_t(v);
        }
        wedges[q]++;
      }
    }
    for(const auto& [key, count] : position_edges) {
      const uint32_t a = uint32_t(key >> 32);
      const uint32_t b = uint32_t(key & 0xffffffffu);
      if(a == b) {
        continue;
      }
      if(count == 1) {
        border_edges[a] = uint8_t(std::min(border_edges[a] + 1, 255));
        border_edges[b] = uint8_t(std::min(border_edges[b] + 1, 255));
      } else if(count > 2) {
        complex[a] = complex[b] = 1;
      }
    }
    for(const auto& [key, count] : vertex_edges) {
      if(count == 1) {
        const uint32_t a = uint32_t(key >> 32);
        const uint32_t b = uint32_t(key & 0xffffffffu);
        open_edges[a] = uint8_t(std::min(open_edges[a] + 1, 255));
        open_edges[b] = uint8_t(std::min(open_edges[b] + 1, 255));
      }
    }
    std::vector<SimplifyKind> kind(n, SimplifyKind::kLocked);
    for(size_t v = 0; v < n; v++) {
      const uint32_t q = remap[v];
      if(first[v + 1] == first[v] || complex[q]) {
        continue;
      }
      if(wedges[q] == 1 && border_edges[q] == 0) {
        kind[v] = SimplifyKind::kManifold;
      } else if(wedges[q] == 1 && border_edges[q] == 2 &&
                !options.lock_border) {
        kind[v] = SimplifyKind::kBorder;
      } else if(wedges[q] == 2 && border_edges[q] == 0 &&
                open_edges[v] == 2 && open_edges[sibling[v]] == 2) {
        kind[v] = SimplifyKind::kSeam;
      }
    }

    // the cost of moving vertex v onto t (and its sibling v2 onto t2)
    auto attribute_cost = [&](uint32_t v, uint32_t t) {
      if(!use_attributes) {
        return 0.0;
      }
      double sum = 0.0;
      for(size_t k = 0; k < attribute_count; k++) {
        const double d =
          options.attribute_weights[k] *
          (attributes[v * attribute_count + k] -
           attributes[t * attribute_count + k]);
        sum += d * d;
      }
      return sum;
    };
    auto collapse_cost = [&](uint32_t v, uint32_t t, uint32_t v2,
                             uint32_t t2) {
      SimplifyQuadricForm q = quadric[remap[v]];
      SimplifyAddQuadric(q, quadric[remap[t]]);
      const double w = std::max(q.w, 1e-30);
      const uint32_t pt = remap[t];
      double cost =
        SimplifyEvaluate(q, p[3 * pt], p[3 * pt + 1], p[3 * pt + 2]) / w;
      const double share = quadric[remap[v]].w / w;
      cost += share * attribute_cost(v, t);
      if(v2 != v) {
        cost += share * attribute_cost(v2, t2);
      }
      return cost;
    };

    struct Candidate {
      double cost;
      uint32_t v, t, v2, t2;
    };
    std::vector<Candidate> candidates;
    for(uint32_t v = 0; v < n; v++) {
      if(kind[v] == SimplifyKind::kLocked) {
        continue;
      }
      uint32_t v2 = v;
      if(kind[v] == SimplifyKind::kSeam) {
        v2 = sibling[v];
        if(v2 < v) {
          continue; // the pair is the smaller wedge's
        }
      }
      Candidate best{std::numeric_limits<double>::max(), v, v, v, v};
      for(uint32_t i = first[v]; i < first[v + 1]; i++) {
        const uint32_t f = vertex_faces[i];
        for(int e = 0; e < 3; e++) {
          const uint32_t t = result[3 * f + e];
          if(remap[t] == remap[v]) {
            continue;
          }
          uint32_t t2 = t;
          if(kind[v] == SimplifyKind::kBorder) {
            if(position_edges[SimplifyEdgeKey(remap[v], remap[t])] != 1) {
              continue;
            }
          } else if(kind[v] == SimplifyKind::kSeam) {
            if(vertex_edges[SimplifyEdgeKey(v, t)] != 1) {
              continue;
            }
            // the wedge of t's position along the sibling's open edge
            bool found = false;
            for(uint32_t j = first[v2]; j < first[v2 + 1] && !found; j++) {
              const uint32_t g = vertex_faces[j];
              for(int k = 0; k < 3; k++) {
                const uint32_t u = result[3 * g + k];
                if(u != t && remap[u] == remap[t] &&
                   vertex_edges[SimplifyEdgeKey(v2, u)] == 1) {
                  t2 = u;
                  found = true;
                  break;
                }
              }
            }
            if(!found) {
              continue;
            }
          }
          const double cost = collapse_cost(v, t, v2, t2);
          if(cost < best.cost) {
            best = {cost, v, t, v2, t2};
          }
        }
      }
      if(best.t != v) {
        candidates.push_back(best);
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                return a.cost < b.cost ||
                       (a.cost == b.cost && a.v < b.v);
              });

    // whether moving v onto t turns over a face of v that stays
    auto flips = [&](uint32_t v, uint32_t t) {
      const uint32_t pv = remap[v];
      const uint32_t pt = remap[t];
      for(uint32_t i = first[v]; i < first[v + 1]; i++) {
        const uint32_t f = vertex_faces[i];
        const uint32_t q[3] = {remap[result[3 * f]], remap[result[3 * f + 1]],
                               remap[result[3 * f + 2]]};
        if(q[0] == pt || q[1] == pt || q[2] == pt) {
          continue; // goes away
        }
        const double* a[3];
        const double* b[3];
        for(int k = 0; k < 3; k++) {
          a[k] = &p[3 * q[k]];
          b[k] = q[k] == pv ? &p[3 * pt] : a[k];
        }
        double before[3], after[3];
        SimplifyTriangleNormal(a[0], a[1], a[2], before);
        SimplifyTriangleNormal(b[0], b[1], b[2], after);
        const double dot = before[0] * after[0] + before[1] * after[1] +
                           before[2] * after[2];
        const double lengths =
          std::sqrt((before[0] * before[0] + before[1] * before[1] +
                     before[2] * before[2]) *
                    (after[0] * after[0] + after[1] * after[1] +
                     after[2] * after[2]));
        if(dot <= 0.25 * lengths) {
          return true;
        }
        const double* o = &original_normal[3 * f];
        const double after_length = std::sqrt(
          after[0] * after[0] + after[1] * after[1] + after[2] * after[2]);
        if(o[0] * after[0] + o[1] * after[1] + o[2] * after[2] <=
           0.2 * after_length) {
          return true;
        }
      }
      return false;
    };

    // the link condition: the positions next to both pv and pt must be
    // those opposite the edge in its faces, or the collapse folds the
    // surface onto itself (two faces back to back)
    auto keeps_link = [&](uint32_t pv, uint32_t pt) {
      std::vector<uint32_t> around_v, around_t, opposite;
      for(uint32_t i = position_first[pv]; i < position_first[pv + 1]; i++) {
        const uint32_t f = position_faces[i];
        bool has_t = false;
        for(int k = 0; k < 3; k++) {
          has_t = has_t || remap[result[3 * f + k]] == pt;
        }
        for(int k = 0; k < 3; k++) {
          const uint32_t q = remap[result[3 * f + k]];
          if(q != pv && q != pt) {
            (has_t ? opposite : around_v).push_back(q);
          }
        }
      }
      for(uint32_t i = position_first[pt]; i < position_first[pt + 1]; i++) {
        const uint32_t f = position_faces[i];
        for(int k = 0; k < 3; k++) {
          const uint32_t q = remap[result[3 * f + k]];
          if(q != pv && q != pt) {
            around_t.push_back(q);
          }
        }
      }
      std::sort(around_t.begin(), around_t.end());
      std::sort(opposite.begin(), opposite.end());
      for(uint32_t q : around_v) {
        if(std::binary_search(around_t.begin(), around_t.end(), q) &&
           !std::binary_search(opposite.begin(), opposite.end(), q)) {
          return false;
        }
      }
      return true;
    };

    // the cheapest collapses that do not touch each other's neighborhoods,
    // as many as the target needs
    const size_t needed = face_count - options.target_face_count;
    size_t removed = 0;
    std::vector<uint8_t> touched(n, 0);
    std::vector<uint32_t> collapse_to(n);
    std::iota(collapse_to.begin(), collapse_to.end(), 0u);
    for(const Candidate& c : candidates) {
      if(c.cost > max_cost || removed >= needed) {
        break;
      }
      const uint32_t pv = remap[c.v];
      const uint32_t pt = remap[c.t];
      if(touched[pv] || touched[pt]) {
        continue;
      }
      if(!keeps_link(pv, pt) || flips(c.v, c.t) ||
         (c.v2 != c.v && flips(c.v2, c.t2))) {
        continue;
      }
      collapse_to[c.v] = c.t;
      collapse_to[c.v2] = c.t2;
      SimplifyAddQuadric(quadric[pt], quadric[pv]);
      for(uint32_t v : {c.v, c.v2}) {
        for(uint32_t i = first[v]; i < first[v + 1]; i++) {
          const uint32_t f = vertex_faces[i];
          for(int k = 0; k < 3; k++) {
            touched[remap[result[3 * f + k]]] = 1;
          }
        }
      }
      removed += kind[c.v] == SimplifyKind::kBorder ? 1 : 2;
      largest_error = std::max(largest_error, c.cost);
    }
    if(removed == 0) {
      break;
    }
    std::vector<uint32_t> next;
    std::vector<double> next_normal;
    next.reserve(result.size());
    next_normal.reserve(result.size());
    for(size_t f = 0; f < result.size(); f += 3) {
      const uint32_t a = collapse_to[result[f]];
      const uint32_t b = collapse_to[result[f + 1]];
      const uint32_t c = collapse_to[result[f + 2]];
      if(remap[a] == remap[b] || remap[b] == remap[c] ||
         remap[a] == remap[c]) {
        continue;
      }
      next.push_back(a);
      next.push_back(b);
      next.push_back(c);
      next_normal.insert(next_normal.end(), original_normal.begin() + f,
                         original_normal.begin() + f + 3);
    }
    result.swap(next);
    original_normal.swap(next_normal);
  }
  if(error) {
    *error = std::sqrt(largest_error);
  }
  return result;
}

/* the faces of positions clustered in a grid of resolution cells along the
largest side: each vertex replaced by the representative of its cell, the
collapsed and repeated faces dropped */
static std::vector<uint32_t> SimplifyCluster(const std::vector<double>& p,
                                             const std::vector<uint32_t>& faces,
                                             int resolution) {
  const size_t n = p.size() / 3;
  const uint64_t r = uint64_t(resolution);
  std::vector<uint64_t> cell(n);
  for(size_t i = 0; i < n; i++) {
    uint64_t index[3];
    for(int k = 0; k < 3; k++) {
      index[k] = std::min<uint64_t>(r - 1,
                                    uint64_t(std::max(0.0, p[3 * i + k]) *
                                             double(r)));
    }
    cell[i] = (index[0] * r + index[1]) * r + index[2];
  }
  // the representative of a cell: its vertex nearest the mean of its
  // vertices in use
  std::vector<uint8_t> used(n, 0);
  for(uint32_t v : faces) {
    used[v] = 1;
  }
  std::unordered_map<uint64_t, std::array<double, 4>> mean;
  for(size_t i = 0; i < n; i++) {
    if(used[i]) {
      auto& m = mean[cell[i]];
      m[0] += p[3 * i];
      m[1] += p[3 * i + 1];
      m[2] += p[3 * i + 2];
      m[3] += 1.0;
    }
  }
  std::unordered_map<uint64_t, std::pair<double, uint32_t>> nearest;
  for(size_t i = 0; i < n; i++) {
    if(!used[i]) {
      continue;
    }
    const auto& m = mean[cell[i]];
    double d = 0.0;
    for(int k = 0; k < 3; k++) {
      const double x = p[3 * i + k] - m[k] / m[3];
      d += x * x;
    }
    auto [it, inserted] = nearest.try_emplace(cell[i], d, uint32_t(i));
    if(!inserted && d < it->second.first) {
      it->second = {d, uint32_t(i)};
    }
  }
  std::vector<uint32_t> result;
  std::vector<std::array<uint32_t, 3>> seen;
  for(size_t f = 0; f + 2 < faces.size(); f += 3) {
    std::array<uint32_t, 3> t;
    for(int k = 0; k < 3; k++) {
      t[k] = nearest[cell[faces[f + k]]].second;
    }
    if(cell[t[0]] == cell[t[1]] || cell[t[1]] == cell[t[2]] ||
       cell[t[0]] == cell[t[2]]) {
      continue;
    }
    // the same triangle once, in either winding
    std::array<uint32_t, 3> key = t;
    const auto low = std::min_element(key.begin(), key.end());
    std::rotate(key.begin(), low, key.end());
    seen.push_back(key);
  }
  std::sort(seen.begin(), seen.end());
  seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
  result.reserve(3 * seen.size());
  for(const auto& t : seen) {
    result.insert(result.end(), t.begin(), t.end());
  }
  return result;
}

std::vector<uint32_t> shift::core::geometry::SimplifyGrid(
  const std::vector<float>& positions, const std::vector<uint32_t>& faces,
  size_t target_face_count, double* error) {
  if(error) {
    *error = 0.0;
  }
  if(faces.size() / 3 <= target_face_count) {
    return faces;
  }
  std::vector<double> p;
  SimplifyNormalize(positions, p);
  // the finest grid with few enough faces: the count grows with the
  // resolution, nearly monotonically
  int low = 1;
  int high = 1024;
  std::vector<uint32_t> best = SimplifyCluster(p, faces, low);
  if(best.size() / 3 > target_face_count) {
    best.clear();
  }
  int best_resolution = low;
  while(low < high) {
    const int middle = (low + high + 1) / 2;
    std::vector<uint32_t> trial = SimplifyCluster(p, faces, middle);
    if(trial.size() / 3 <= target_face_count) {
      best.swap(trial);
      best_resolution = middle;
      low = middle;
    } else {
      high = middle - 1;
    }
  }
  if(error) {
    *error = std::sqrt(3.0) / double(best_resolution);
  }
  return best;
}

/* the root of a set of the union-find forest parent, with path halving */
static uint32_t SimplifyFind(std::vector<uint32_t>& parent, uint32_t a) {
  while(parent[a] != a) {
    parent[a] = parent[parent[a]];
    a = parent[a];
  }
  return a;
}

std::vector<uint32_t> shift::core::geometry::PruneComponents(
  const std::vector<float>& positions, const std::vector<uint32_t>& faces,
  double keep_fraction, uint32_t seed, std::vector<float>* scaled_positions,
  double* error) {
  const size_t n = positions.size() / 3;
  *scaled_positions = positions;
  if(error) {
    *error = 0.0;
  }
  if(keep_fraction >= 1.0 || faces.empty()) {
    return faces;
  }
  // the parts: vertices joined by faces, and by sharing a position
  const std::vector<uint32_t> remap = SimplifyPositionRemap(positions);
  std::vector<uint32_t> parent(n);
  std::iota(parent.begin(), parent.end(), 0u);
  auto join = [&](uint32_t a, uint32_t b) {
    a = SimplifyFind(parent, a);
    b = SimplifyFind(parent, b);
    if(a != b) {
      parent[std::max(a, b)] = std::min(a, b);
    }
  };
  for(size_t v = 0; v < n; v++) {
    join(uint32_t(v), remap[v]);
  }
  for(size_t f = 0; f + 2 < faces.size(); f += 3) {
    join(faces[f], faces[f + 1]);
    join(faces[f], faces[f + 2]);
  }
  // the faces, area and area-weighted centroid of each part
  std::vector<uint32_t> part_of(n);
  std::unordered_map<uint32_t, uint32_t> part_index;
  for(size_t v = 0; v < n; v++) {
    const uint32_t root = SimplifyFind(parent, uint32_t(v));
    auto [it, inserted] =
      part_index.try_emplace(root, uint32_t(part_index.size()));
    part_of[v] = it->second;
  }
  const size_t parts = part_index.size();
  std::vector<size_t> part_faces(parts, 0);
  std::vector<double> part_area(parts, 0.0);
  std::vector<std::array<double, 3>> centroid(parts, {0.0, 0.0, 0.0});
  double total_area = 0.0;
  for(size_t f = 0; f + 2 < faces.size(); f += 3) {
    double a[3], b[3], c[3], normal[3];
    for(int k = 0; k < 3; k++) {
      a[k] = positions[3 * faces[f] + k];
      b[k] = positions[3 * faces[f + 1] + k];
      c[k] = positions[3 * faces[f + 2] + k];
    }
    SimplifyTriangleNormal(a, b, c, normal);
    const double area = 0.5 * std::sqrt(normal[0] * normal[0] +
                                        normal[1] * normal[1] +
                                        normal[2] * normal[2]);
    const uint32_t part = part_of[faces[f]];
    part_faces[part]++;
    part_area[part] += area;
    total_area += area;
    for(int k = 0; k < 3; k++) {
      centroid[part][k] += area * (a[k] + b[k] + c[k]) / 3.0;
    }
  }
  // the parts in a random order of the seed (Fisher-Yates with the same
  // generator everywhere), kept until they have enough faces
  std::vector<uint32_t> order(parts);
  std::iota(order.begin(), order.end(), 0u);
  std::mt19937 generator(seed);
  for(size_t i = parts; i > 1; i--) {
    const size_t j = size_t(generator() % i);
    std::swap(order[i - 1], order[j]);
  }
  const size_t wanted = size_t(
    std::ceil(std::max(keep_fraction, 0.0) * double(faces.size() / 3)));
  std::vector<uint8_t> kept(parts, 0);
  size_t kept_faces = 0;
  double kept_area = 0.0;
  for(uint32_t part : order) {
    if(kept_faces >= std::max<size_t>(wanted, 1)) {
      break;
    }
    if(part_faces[part] == 0) {
      continue;
    }
    kept[part] = 1;
    kept_faces += part_faces[part];
    kept_area += part_area[part];
  }
  // enlarged so that the kept parts cover the area of all of them
  const double scale =
    kept_area > 0.0 ? std::clamp(std::sqrt(total_area / kept_area), 1.0, 4.0)
                    : 1.0;
  std::vector<double> radius(parts, 0.0);
  for(size_t v = 0; v < n; v++) {
    const uint32_t part = part_of[v];
    if(!kept[part] || part_area[part] <= 0.0) {
      continue;
    }
    double r2 = 0.0;
    for(int k = 0; k < 3; k++) {
      const double c = centroid[part][k] / part_area[part];
      r2 += (positions[3 * v + k] - c) * (positions[3 * v + k] - c);
      (*scaled_positions)[3 * v + k] =
        float(c + scale * (positions[3 * v + k] - c));
    }
    radius[part] = std::max(radius[part], std::sqrt(r2));
  }
  if(error && kept_area > 0.0) {
    std::vector<double> normalized;
    const double extent = SimplifyNormalize(positions, normalized);
    double mean_radius = 0.0;
    for(size_t part = 0; part < parts; part++) {
      if(kept[part]) {
        mean_radius += part_area[part] * radius[part];
      }
    }
    mean_radius /= kept_area;
    *error = (scale - 1.0) * mean_radius / extent;
  }
  std::vector<uint32_t> result;
  result.reserve(3 * kept_faces);
  for(size_t f = 0; f + 2 < faces.size(); f += 3) {
    if(kept[part_of[faces[f]]]) {
      result.insert(result.end(), faces.begin() + f, faces.begin() + f + 3);
    }
  }
  return result;
}

std::vector<uint32_t> shift::core::geometry::CompactVertices(
  std::vector<uint32_t>& faces, size_t vertex_count) {
  std::vector<uint32_t> new_index(vertex_count,
                                  std::numeric_limits<uint32_t>::max());
  std::vector<uint32_t> old_index;
  for(uint32_t& v : faces) {
    if(new_index[v] == std::numeric_limits<uint32_t>::max()) {
      new_index[v] = uint32_t(old_index.size());
      old_index.push_back(v);
    }
    v = new_index[v];
  }
  return old_index;
}
