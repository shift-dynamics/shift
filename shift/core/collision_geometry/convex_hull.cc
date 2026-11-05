#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/fitting.h"

#include <cmath>

#include "shift/core/factory.h"
#include "shift/core/exception.h"

bool shift::core::collision_geometry::ConvexHull::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("convex_hull",
                     shift::core::collision_geometry::ConvexHull::Create);

std::unique_ptr<shift::core::collision_geometry::ConvexHull>
shift::core::collision_geometry::ConvexHull::Create() {
  return std::make_unique<shift::core::collision_geometry::ConvexHull>();
}

void shift::core::collision_geometry::ConvexHull::DeclareParameters() {
  DeclareParameter("half_extents", &half_extents_,
                   "half extents [x, y, z] of a box along the axes of the "
                   "pose, centered at its origin (the body origin by "
                   "default); if not given, the hull is that of the body's "
                   "mesh",
                   false);
}

bool shift::core::collision_geometry::ConvexHull::HasDimensions() const {
  const auto& h = half_extents_;
  if(h.x == 0.0 && h.y == 0.0 && h.z == 0.0) {
    return false;
  }
  if(!(h.x > 0.0 && h.y > 0.0 && h.z > 0.0)) {
    throw shift::core::Exception("Convex hull collision geometry '" + name_ +
                                 "' needs three positive half extents");
  }
  return true;
}

void shift::core::collision_geometry::ConvexHull::SetFromDimensions(
  const ksl_SE3_t& pose) {
  std::vector<ksl_vec3_t> corners;
  for(int i = 0; i < 8; i++) {
    corners.push_back({{(i & 1 ? 1 : -1) * half_extents_.x,
                        (i & 2 ? 1 : -1) * half_extents_.y,
                        (i & 4 ? 1 : -1) * half_extents_.z}});
  }
  ConvexHullData hull;
  ComputeConvexHull(corners, hull);
  SetHull(hull.vertices, hull.faces);
  d_wrt_body_ = pose;
}

void shift::core::collision_geometry::ConvexHull::Fit(
  const std::vector<ksl_vec3_t>& vertices,
  const std::vector<unsigned int>& faces) {
  SetHull(vertices, faces);
}

void shift::core::collision_geometry::ConvexHull::SetHull(
  const std::vector<ksl_vec3_t>& vertices,
  const std::vector<unsigned int>& faces) {
  auto hull = std::make_shared<ConvexHullData>();
  hull->vertices = vertices;
  hull->faces = faces;
  ComputeVertexNeighbors(*hull);
    ComputeFacePlanes(*hull);
  ComputeFacePolygons(*hull);
  bounding_radius_ = 0.0;
  for(const auto& v : vertices) {
    bounding_radius_ = std::max(bounding_radius_, ksl_vec3_l2norm(&v));
  }
  hull_ = hull;
}

void shift::core::collision_geometry::ConvexHull::CopyFit(const Base& other) {
  const auto& convex_hull = dynamic_cast<const ConvexHull&>(other);
  hull_ = convex_hull.hull_;
  d_wrt_body_ = convex_hull.d_wrt_body_;
  bounding_radius_ = convex_hull.bounding_radius_;
}

/*!
Hill climbing: starting from the vertex at hint, move to the neighbor with the
greatest projection along the search direction until no neighbor improves on
the current vertex. On a convex polytope this local maximum is the global one.
Without adjacency (a degenerate hull) every vertex is searched.
*/
void shift::core::collision_geometry::ConvexHull::Support(
  const ksl_vec3_t& direction, ksl_vec3_t& local, ksl_vec3_t& global,
  int& hint) const {
  const auto& vertices = hull_->vertices;
  const auto& neighbors = hull_->neighbors;
  if(vertices.empty()) {
    local = {{0.0, 0.0, 0.0}};
    ksl_product_dv(&d_absolute_, &local, &global);
    return;
  }

  /* search direction in the hull frame, u = R^T * direction */
  ksl_vec3_t u;
  for(int i = 0; i < 3; i++) {
    u.at[i] = ksl_dot_vv(&d_absolute_.R.v[i], &direction);
  }

  size_t best = (hint >= 0 && static_cast<size_t>(hint) < vertices.size())
                  ? static_cast<size_t>(hint)
                  : 0;
  double best_projection = ksl_dot_vv(&u, &vertices[best]);
  if(neighbors.size() == vertices.size() && !hull_->faces.empty()) {
    bool improved = true;
    while(improved) {
      improved = false;
      for(unsigned int j : neighbors[best]) {
        const double projection = ksl_dot_vv(&u, &vertices[j]);
        if(projection > best_projection) {
          best_projection = projection;
          best = j;
          improved = true;
        }
      }
    }
  } else {
    for(size_t j = 0; j < vertices.size(); j++) {
      const double projection = ksl_dot_vv(&u, &vertices[j]);
      if(projection > best_projection) {
        best_projection = projection;
        best = j;
      }
    }
  }
  hint = static_cast<int>(best);
  local = vertices[best];
  ksl_product_dv(&d_absolute_, &local, &global);
}
