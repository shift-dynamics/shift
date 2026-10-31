#include "shift/core/collision_geometry/fitting.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

extern "C" {
#include <libqhull_r/qhull_ra.h>
}

#include "shift/core/log.h"

/*!
code derived from the reentrant Qhull template at https://github.com/qhull/qhull

Only one convex hull is generated to encompass a body. Later, this could be
modified to fit a number of conformal convex hulls with adjustable level of
detail using the v-hacd algorithm or similar.
*/
bool shift::core::collision_geometry::ComputeConvexHull(
  const std::vector<ksl_vec3_t>& points, ConvexHullData& hull) {
  hull = ConvexHullData();
  if(points.size() < 4) {
    return false;
  }

  std::vector<coordT> coordinates(3 * points.size());
  for(size_t i = 0; i < points.size(); i++) {
    for(int k = 0; k < 3; k++) {
      coordinates[3 * i + k] = points[i].at[k];
    }
  }

  qhT qh_qh;
  qhT* qh = &qh_qh;
  QHULL_LIB_CHECK
  qh_zero(qh, stderr);
  char flags[] = "qhull Qt";
  int exitcode = qh_new_qhull(qh, 3, static_cast<int>(points.size()),
                              coordinates.data(), False, flags, nullptr,
                              stderr);

  if(!exitcode) {
    /* compress qhull's point ids to contiguous vertex indices */
    std::vector<int> index_of_point(points.size(), -1);
    vertexT* vertex;
    FORALLvertices {
      const int id = qh_pointid(qh, vertex->point);
      index_of_point[id] = static_cast<int>(hull.vertices.size());
      hull.vertices.push_back(points[id]);
    }
    facetT* facet;
    vertexT** vertexp;
    FORALLfacets {
      FOREACHvertex_(facet->vertices) {
        hull.faces.push_back(
          static_cast<unsigned int>(index_of_point[qh_pointid(qh, vertex->point)]));
      }
    }
  }

  int curlong, totlong;
  qh_freeqhull(qh, !qh_ALL);
  qh_memfreeshort(qh, &curlong, &totlong);
  if(curlong || totlong) {
    SHIFT_WARNING("qhull did not free " + std::to_string(totlong) +
                  " bytes of long memory (" + std::to_string(curlong) +
                  " pieces)");
  }

  if(exitcode || hull.faces.size() % 3 != 0) {
    hull = ConvexHullData();
    return false;
  }
  ComputeVertexNeighbors(hull);
  return true;
}

void shift::core::collision_geometry::ComputeVertexNeighbors(
  ConvexHullData& hull) {
  hull.neighbors.assign(hull.vertices.size(), {});
  for(size_t f = 0; f + 2 < hull.faces.size(); f += 3) {
    for(int j = 0; j < 3; j++) {
      auto& list = hull.neighbors[hull.faces[f + j]];
      for(int k = 0; k < 3; k++) {
        const unsigned int vertex_k = hull.faces[f + k];
        if(j != k && std::find(list.begin(), list.end(), vertex_k) == list.end()) {
          list.push_back(vertex_k);
        }
      }
    }
  }
}

ksl_mat3x3_t shift::core::collision_geometry::Covariance(
  const std::vector<ksl_vec3_t>& points) {
  ksl_mat3x3_t covariance = {{{0, 0, 0, 0, 0, 0, 0, 0, 0}}};
  if(points.empty()) {
    return covariance;
  }
  const double oon = 1.0 / points.size();
  ksl_vec3_t c = {{0, 0, 0}};
  for(const auto& p : points) {
    for(int k = 0; k < 3; k++) {
      c.at[k] += p.at[k] * oon;
    }
  }
  for(const auto& p : points) {
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        covariance.at[j][i] += (p.at[i] - c.at[i]) * (p.at[j] - c.at[j]) * oon;
      }
    }
  }
  return covariance;
}

/*!
cyclic Jacobi method, rotating away the largest off diagonal element on each
iteration until the off diagonal norm stops decreasing (Ericson, "Real-Time
Collision Detection", 2005, section 4.3.4)
*/
void shift::core::collision_geometry::Jacobi(ksl_mat3x3_t& A, ksl_mat3x3_t& V) {
  /* at[column][row] */
  auto a = [&](int row, int column) -> double& { return A.at[column][row]; };
  auto v = [&](int row, int column) -> double& { return V.at[column][row]; };
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      v(i, j) = (i == j) ? 1.0 : 0.0;
    }
  }

  const int kMaxIterations = 50;
  double previous_off = DBL_MAX;
  for(int n = 0; n < kMaxIterations; n++) {
    /* find largest off diagonal element */
    int p = 0;
    int q = 1;
    for(int i = 0; i < 3; i++) {
      for(int j = i + 1; j < 3; j++) {
        if(std::fabs(a(i, j)) > std::fabs(a(p, q))) {
          p = i;
          q = j;
        }
      }
    }
    if(a(p, q) == 0.0) {
      break;
    }

    /* 2x2 symmetric Schur decomposition */
    const double r = (a(q, q) - a(p, p)) / (2.0 * a(p, q));
    const double t = (r >= 0.0) ? 1.0 / (r + std::sqrt(1.0 + r * r))
                                : -1.0 / (-r + std::sqrt(1.0 + r * r));
    const double c = 1.0 / std::sqrt(1.0 + t * t);
    const double s = t * c;

    /* A <- J^T A J, V <- V J, with J the rotation in the p, q plane */
    for(int k = 0; k < 3; k++) {
      const double akp = a(k, p);
      const double akq = a(k, q);
      a(k, p) = c * akp - s * akq;
      a(k, q) = s * akp + c * akq;
    }
    for(int k = 0; k < 3; k++) {
      const double apk = a(p, k);
      const double aqk = a(q, k);
      a(p, k) = c * apk - s * aqk;
      a(q, k) = s * apk + c * aqk;
    }
    for(int k = 0; k < 3; k++) {
      const double vkp = v(k, p);
      const double vkq = v(k, q);
      v(k, p) = c * vkp - s * vkq;
      v(k, q) = s * vkp + c * vkq;
    }

    double off = 0.0;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(i != j) {
          off += a(i, j) * a(i, j);
        }
      }
    }
    if(n > 2 && off >= previous_off) {
      break;
    }
    previous_off = off;
  }

  /* make V a proper rotation */
  const double det = v(0, 0) * (v(1, 1) * v(2, 2) - v(1, 2) * v(2, 1)) -
                     v(0, 1) * (v(1, 0) * v(2, 2) - v(1, 2) * v(2, 0)) +
                     v(0, 2) * (v(1, 0) * v(2, 1) - v(1, 1) * v(2, 0));
  if(det < 0.0) {
    for(int k = 0; k < 3; k++) {
      v(k, 2) = -v(k, 2);
    }
  }
}
