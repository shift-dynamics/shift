#include "shift/core/collision_geometry/base.h"

#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/box.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/log.h"

bool shift::core::collision_geometry::Modules::is_initialized() {
  if(!shift::core::collision_geometry::ConvexHull::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Box::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Capsule::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Cylinder::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Ellipsoid::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Sphere::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Plane::is_registered()) {
    return false;
  }
  if(!shift::core::collision_geometry::Cad::is_registered()) {
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
  pose_["name"].set_required(false);
  DeclareParameter(
    "pose", &pose_,
    "displacement of the collision geometry with respect to the body origin, "
    "for geometry given by its dimensions (e.g. a sphere radius); defaults "
    "to identity. Not allowed for geometry fitted to a mesh, which the fit "
    "places.",
    false);
      DeclareParameter(
    "contact", &contact_,
    "whether the geometry takes part in contact detection; defaults to true",
    false);
  DeclareParameter(
    "material", &material_,
    "name of the contact material, one of the materials of the contact force "
    "module; defaults to its default material",
    false);
  DeclareParameter(
    "fit_to", &fit_to_,
    "names of the body's mesh geometry to fit to, when the dimensions are "
    "not given; defaults to all active meshes of the body. A named mesh may "
    "be inactive, to fit to a collision mesh that is not drawn.",
    false);
}

static bool IsIdentity(const ksl_SE3_t& d) {
  for(int i = 0; i < 4; i++) {
    for(int k = 0; k < 3; k++) {
      if(d.v[i].at[k] != (i == k ? 1.0 : 0.0)) {
        return false;
      }
    }
  }
  return true;
}

void shift::core::collision_geometry::Base::ActivateObjects(
  bool parent_is_active) {
  if(context_) {
    context_->ActivateObject(this, active_ && parent_is_active);
  }
};

/*!
Geometry given by its dimensions is set from them, placed by its pose.
Otherwise, collects the vertices of the mesh geometry named in fit_to, or of
every active mesh geometry of the parent body, which are expressed in the body
origin frame, computes their convex hull and fits the
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
  const std::string description =
    "Collision geometry '" + name_ + "' of body '" + body->get_name() + "'";
  const ksl_SE3_t& pose = *pose_.get_d_wrt_origin();
  if(HasDimensions()) {
    if(!fit_to_.empty()) {
      throw shift::core::Exception(
        description + " gives its dimensions and fit_to; give one or the "
                      "other");
    }
    SetFromDimensions(pose);
    loaded_ = true;
    return;
  }
  if(!IsIdentity(pose)) {
    throw shift::core::Exception(
      description + " is fitted to a mesh, which places it, but gives a "
                    "pose; a pose needs the dimensions of the geometry");
  }

  std::vector<shift::core::geometry::Mesh*> meshes;
  if(fit_to_.empty()) {
    for(auto& geometry : body->get_geometry()) {
      auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(mesh && mesh->is_active()) {
        meshes.push_back(mesh);
      }
    }
  } else {
    for(const auto& name : fit_to_) {
      shift::core::geometry::Mesh* mesh = nullptr;
      for(auto& geometry : body->get_geometry()) {
        if(geometry->get_name() == name) {
          mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
          if(!mesh) {
            throw shift::core::Exception(description + " fits to geometry '" +
                                         name + "', which is not a mesh");
          }
        }
      }
      if(!mesh) {
        throw shift::core::Exception(description + " fits to geometry '" +
                                     name + "', which the body does not have");
      }
      /* inactive geometry is not loaded with the model; a collision mesh
      that is not drawn is loaded here */
      if(!mesh->is_active() && mesh->get_mesh_data().empty()) {
        mesh->Load();
      }
      meshes.push_back(mesh);
    }
  }
  std::string key = type_ + ":";
  for(auto mesh : meshes) {
    for(const auto& filename : mesh->get_filenames()) {
      key += filename + ";";
    }
    key += "|";
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
    SHIFT_WARNING(description +
                  " has no mesh geometry to fit to and no dimensions; it is "
                  "ignored in contact detection");
    return;
  }

  FitToPoints(points);
  context_->AddCollisionGeometryPrototype(key, this);
}

void shift::core::collision_geometry::Base::FitToPoints(
  const std::vector<ksl_vec3_t>& points) {
  ConvexHullData hull;
  if(ComputeConvexHull(points, hull)) {
    Fit(hull.vertices, hull.faces);
  } else {
    SHIFT_WARNING("Convex hull of the points of collision geometry '" +
                  name_ + "' is degenerate, fitting it to all points");
    Fit(points, {});
  }
  loaded_ = true;
}

void shift::core::collision_geometry::Base::Connect() {}

void shift::core::collision_geometry::Base::UpdatePose(
  const ksl_SE3_t& body_origin_pose) {
  ksl_product_dd(&body_origin_pose, &d_wrt_body_, &d_absolute_);
}
