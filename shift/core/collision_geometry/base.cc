#include "shift/core/collision_geometry/base.h"

#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/collision_geometry/sphere.h"
#include "shift/core/context.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/log.h"

bool shift::core::collision_geometry::Modules::is_initialized() {
  if(!shift::core::collision_geometry::ConvexHull::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Ellipsoid::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Sphere::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::collision_geometry::Base::DeclareParameters() {
  DeclareParameter("name", &name_,
                   "User defined descriptive collision geometry name.", false);
  DeclareParameter(
    "active", &active_,
    "Indicates whether collision geometry is active in simulation: if the "
    "value is set to false, it is not fitted to the body's mesh and is "
    "excluded from contact detection.",
    false);
}

void shift::core::collision_geometry::Base::ActivateObjects(
  bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
};

/*!
Collects the vertices of every active mesh geometry of the parent body, which
are expressed in the body origin frame, computes their convex hull and fits the
collision geometry to the hull vertices. If the hull is degenerate, e.g. for a
flat mesh, the collision geometry is fitted to all of the mesh vertices.
*/
void shift::core::collision_geometry::Base::Load() {
  auto body = get_parent_body();
  if(!body) {
    throw shift::core::Exception("Collision geometry '" + name_ +
                                 "' is not attached to a body");
  }
  if(name_.empty()) {
    name_ = body->get_name();
  }

  std::vector<shift::core::geometry::Mesh*> meshes;
  std::string key = type_ + ":";
  for(auto& geometry : body->get_geometry()) {
    auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
    if(mesh && mesh->is_active()) {
      meshes.push_back(mesh);
      for(const auto& filename : mesh->get_filenames()) {
        key += filename + ";";
      }
    }
  }

  /* collision geometry of the same type fitted to the same mesh files is
  fitted once and copied, e.g. for many instances of one part */
  if(!meshes.empty()) {
    if(auto prototype = context_->FindCollisionGeometryPrototype(key)) {
      CopyFit(*prototype);
      loaded_ = true;
      return;
    }
  }

  std::vector<ksl_vec3_t> points;
  for(auto mesh : meshes) {
    for(const auto& data : mesh->get_mesh_data()) {
      for(size_t i = 0; i + 2 < data.vertices.size(); i += 3) {
        points.push_back({{data.vertices[i], data.vertices[i + 1],
                           data.vertices[i + 2]}});
      }
    }
  }
  if(points.empty()) {
    SHIFT_WARNING("Collision geometry '" + name_ + "' of body '" +
                  body->get_name() +
                  "' has no mesh geometry to fit to; it is ignored in "
                  "contact detection");
    return;
  }

  ConvexHullData hull;
  if(ComputeConvexHull(points, hull)) {
    Fit(hull.vertices, hull.faces);
  } else {
    SHIFT_WARNING("Convex hull of the mesh of body '" + body->get_name() +
                  "' is degenerate, fitting collision geometry '" + name_ +
                  "' to all mesh vertices");
    Fit(points, {});
  }
  loaded_ = true;
  context_->AddCollisionGeometryPrototype(key, this);
}

void shift::core::collision_geometry::Base::Connect() {}

void shift::core::collision_geometry::Base::UpdatePose(
  const ksl_SE3_t& body_origin_pose) {
  ksl_product_dd(&body_origin_pose, &d_wrt_body_, &d_absolute_);
}
