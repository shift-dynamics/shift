#include <sstream>

#include "shift/core/body/base.h"
#include "shift/core/body/fixed.h"
#include "shift/core/body/free.h"
#include "shift/core/body/rigid.h"
#include "shift/core/body/virtual.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/light/base.h"
#include "shift/core/exception.h"

#include "ksl/matrix.h"
#include "ksl/vec3.h"
#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/geometry/mass_properties.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/log.h"

#include <assert.h>
#include <iostream>

bool shift::core::body::Modules::is_initialized() {
  if(!shift::core::body::Fixed::is_registered()) {
    return false;
  }
  if(!shift::core::body::Free::is_registered()) {
    return false;
  }
  if(!shift::core::body::Rigid::is_registered()) {
    return false;
  }
  if(!shift::core::body::Virtual::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::body::Base::DeclareParameters() {

  DeclareParameter("name", &name_,
               "User defined descriptive body name; must be unique within a "
               "subassembly (required)",
               true);

  DeclareParameter(
    "active", &active_,
    "Indicates whether body is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model "
    "(optional, default=true).",
    false);

  DeclareParameter(
    "center_of_mass", &center_of_mass_wrt_origin_,
    "Specifies a vector from the body origin to the body center of mass "
    "expressed in the body-fixed coordinate system. (optional, "
    "default={0,0,0})",
    false);

  DeclareParameter(
    "density", &density_,
    "Density of the body (mass per volume): if given, the mass, center of "
    "mass and inertia are computed from the solid bounded by the body's "
    "meshes (their convex hull where a mesh is not closed), and "
    "center_of_mass and inertia are not used. (optional)",
    false);

  DeclareParameter(
    "inertia", &inertia_,
    "Specify body mass and centroidal moment of inertia "
    "expressed in the body-fixed coordinate system. Note that if a zero or "
    "very low mass is specified, it may cause problems for some solvers. "
    "(optional, default={mass=1,Ixx=1,Iyy=1,Izz=1,Ixy=0,Iyz=0,Izx=0})",
    false);

  DeclareParameter("frames", &frames_,
               "Specify a number of body-fixed reference frames. Each frame "
               "in a body must have a unique name identified and 'origin' is "
               "a reserved system name that may not be used.",
               false);

  DeclareParameter("lights", &lights_,
               "Specify a number of body-fixed lights, used to "
               "illuminate scene for visualization.",
               false);

  DeclareParameter("geometry", &geometry_,
               "Specify a number of body-fixed geometry, used for "
               "visualization or contact detection. Each geometry "
               "in a body must have a unique name.",
               false);

  DeclareParameter("collision_geometry", &collision_geometry_,
               "Specify a number of body-fixed collision geometries, used for "
               "contact detection. Each collision geometry "
               "in a body must have a unique name.",
               false);
}

void shift::core::body::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, active_ && parent_is_active);

  for(auto& geometry : geometry_.get_data()) {
    geometry->ActivateObjects(active_ && parent_is_active);
  }
  for(auto& collision_geometry : collision_geometry_.get_data()) {
    collision_geometry->ActivateObjects(active_ && parent_is_active);
  }  for(auto& light : lights_.get_data()) {
    light->ActivateObjects(active_ && parent_is_active);
  }
}

void shift::core::body::Base::set_parent_joint(shift::core::joint::Base* parent) {
  parent_joint_ = parent;
  auto reference_frame = parent_joint_->get_system_child_frame();
  for(auto& frame : frames_.get_data()) {
    frame->set_is_reference(frame.get() == reference_frame);
  }
  SetupDWrtReference();
}

void shift::core::body::Base::Connect() {}

shift::core::Frame* shift::core::body::Base::get_origin() {
  for(auto& frame : frames_.get_data()) {
    if(frame->get_name() == "origin") {
      return frame.get();
    }
  }
  return nullptr;
}

shift::core::Frame* shift::core::body::Base::get_reference_frame() {
  for(auto& frame : frames_.get_data()) {
    if(frame->is_reference()) {
      return frame.get();
    }
  }
  throw shift::core::Exception("No reference frame found for body" + get_name());
  return nullptr;
}

void shift::core::body::Base::SetupDWrtReference() {
  Frame* reference = get_reference_frame();
  ksl_SE3_t d_reference;
  ksl_SE3_setIdentity(&d_reference);
  reference->set_d_wrt_reference(d_reference);
  const ksl_SE3_t* reference_wrt_origin = reference->get_d_wrt_origin();
  for(auto& frame : frames_.get_data()) {
    if(!frame->is_reference()) {
      const ksl_SE3_t* get_d_wrt_origin = frame->get_d_wrt_origin();
      ksl_SE3_t d_wrt_reference;
      ksl_product_dinvd(reference_wrt_origin, get_d_wrt_origin, &d_wrt_reference);
      frame->set_d_wrt_reference(d_wrt_reference);
    }
  }
}

void shift::core::body::Base::SetupDWrtReference(ksl_SE3_t d_reference) {
  Frame* reference = get_reference_frame();
  reference->set_d_wrt_reference(d_reference);
  const ksl_SE3_t* reference_wrt_origin = reference->get_d_wrt_origin();
  for(auto& frame : frames_.get_data()) {
    if(!frame->is_reference()) {
      const ksl_SE3_t* get_d_wrt_origin = frame->get_d_wrt_origin();
      ksl_SE3_t temp;
      ksl_product_dinvd(reference_wrt_origin, get_d_wrt_origin, &temp);
      ksl_SE3_t d_wrt_reference;
      ksl_product_dd(&d_reference, &temp, &d_wrt_reference);
      frame->set_d_wrt_reference(d_wrt_reference);
    }
  }
}

void shift::core::body::Base::set_reference_frame(shift::core::Frame* reference_frame) {
  for(auto& frame : frames_.get_data()) {
    frame->set_reference_frame(reference_frame);
  }
  SetupDWrtReference();
};

void shift::core::body::Base::SetupReferenceFrame() {
  auto reference = get_reference_frame();
  for(auto& frame : frames_.get_data()) {
    if(frame.get() == reference) {
      frame->set_reference_frame(nullptr);
    } else {
      frame->set_reference_frame(reference);
    }
  }
  SetupDWrtReference();
};

const ksl_vec3_t shift::core::body::Base::get_center_of_mass_wrt_reference() {
  if(get_reference_frame() != get_origin()) {
    // D_or // transformation from origin to reference frame
    // D_ro // transformation from reference frame to origin
    // t_om_o // vector from origin (o) to center of mass frame (m),
    // expressed in origin coordinates
    // t_rm_r // vector from reference frame to center of mass, expressed in
    // reference frame coordinates

    // t_rm_r = D_ro_ro * t_om_o
    ksl_vec3_t t_om_o = center_of_mass_wrt_origin_;
    const ksl_SE3_t* D_or = get_reference_frame()->get_d_wrt_origin();
    ksl_SE3_t D_ro;
    ksl_SE3_inverted(D_or, &D_ro);
    ksl_vec3_t t_rm_r;
    ksl_product_dv(&D_ro, &t_om_o, &t_rm_r);
    return t_rm_r;
  }
  return center_of_mass_wrt_origin_;
}

void shift::core::body::Base::ClearInTreeFlags() {
  for(auto& frame : frames_.get_data()) {
    frame->set_index(-1);
    frame->set_in_tree(false);
  }
}

unsigned int shift::core::body::Base::get_frame_in_tree_count() {
  unsigned int count = 0;
    for(auto& frame : frames_.get_data()) {
    if(frame->is_in_tree()) {
      count++;
    }
  }
  return count;
}

void shift::core::body::Base::Load() {}

/* the properties of unit density of the solid bounded by meshes, and
whether they are closed surfaces (otherwise those of the convex hull) */
static void UnitMassProperties(
  const std::string& name,
  const std::vector<const shift::core::geometry::Mesh*>& meshes,
  shift::core::geometry::MassProperties& properties, bool& closed) {
  std::vector<ksl_vec3_t> vertices;
  std::vector<unsigned int> faces;
  closed = true;
  for(auto mesh : meshes) {
    for(const auto& data : mesh->get_mesh_data()) {
      std::vector<ksl_vec3_t> points;
      for(size_t i = 0; i + 2 < data.vertices.size(); i += 3) {
        points.push_back(
          {{data.vertices[i], data.vertices[i + 1], data.vertices[i + 2]}});
      }
      closed = closed && shift::core::geometry::IsClosedSurface(points,
                                                                 data.faces);
      const unsigned int offset = static_cast<unsigned int>(vertices.size());
      vertices.insert(vertices.end(), points.begin(), points.end());
      for(auto f : data.faces) {
        faces.push_back(offset + f);
      }
    }
  }
  if(vertices.empty()) {
    throw shift::core::Exception("Body '" + name +
                                 "' gives a density but has no meshes");
  }
  if(!closed) {
    /* holes, e.g. the base of a scanned model, are capped if they are simple
     * loops */
    std::vector<ksl_vec3_t> capped_vertices = vertices;
    std::vector<unsigned int> capped_faces = faces;
    const int holes =
      shift::core::geometry::CloseHoles(capped_vertices, capped_faces);
    if(holes > 0 && shift::core::geometry::IsClosedSurface(capped_vertices,
                                                           capped_faces)) {
      SHIFT_INFO("Body '" + name + "': closed " + std::to_string(holes) +
                 " holes of its meshes to compute its mass properties");
      vertices.swap(capped_vertices);
      faces.swap(capped_faces);
      closed = true;
    }
  }
  if(!closed) {
    /* the convex hull is closed; orient its triangles outward */
    shift::core::collision_geometry::ConvexHullData hull;
    if(!shift::core::collision_geometry::ComputeConvexHull(vertices, hull)) {
      throw shift::core::Exception("Body '" + name +
                                   "': the convex hull of its meshes is "
                                   "degenerate, its volume is zero");
    }
    shift::core::collision_geometry::ComputeFacePlanes(hull);
    for(size_t f = 0; f + 2 < hull.faces.size(); f += 3) {
      const ksl_vec3_t& p0 = hull.vertices[hull.faces[f]];
      const ksl_vec3_t& p1 = hull.vertices[hull.faces[f + 1]];
      const ksl_vec3_t& p2 = hull.vertices[hull.faces[f + 2]];
      ksl_vec3_t e1, e2, n;
      ksl_subtract_vv(&p1, &p0, &e1);
      ksl_subtract_vv(&p2, &p0, &e2);
      ksl_cross_vv(&e1, &e2, &n);
      if(ksl_dot_vv(&n, &hull.face_normals[f / 3]) < 0.0) {
        std::swap(hull.faces[f + 1], hull.faces[f + 2]);
      }
    }
    SHIFT_WARNING("Body '" + name +
                  "': its meshes are not closed surfaces, its mass "
                  "properties are those of their convex hull");
    vertices = hull.vertices;
    faces = hull.faces;
  }
  if(!shift::core::geometry::ComputeMassProperties(vertices, faces,
                                                   properties)) {
    throw shift::core::Exception("Body '" + name +
                                 "': the volume of its meshes is zero");
  }
}

void shift::core::body::Base::ComputeMassFromGeometry(MassCache& cache) {
  if(!(density_ > 0.0)) {
    return;
  }
  std::vector<const shift::core::geometry::Mesh*> meshes;
  std::vector<const shift::core::geometry::MeshAsset*> key;
  for(auto& geometry : geometry_.get_data()) {
    auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
    if(mesh && mesh->is_active()) {
      meshes.push_back(mesh);
      key.push_back(mesh->get_asset());
    }
  }
  auto found = cache.find(key);
  if(found == cache.end()) {
    std::pair<shift::core::geometry::MassProperties, bool> entry;
    UnitMassProperties(name_, meshes, entry.first, entry.second);
    found = cache.emplace(key, entry).first;
  }
  const shift::core::geometry::MassProperties& properties =
    found->second.first;
  const bool closed = found->second.second;
  const double m = density_ * properties.volume;
  inertia_ = ksl_inertia(m, {{0.0, 0.0, 0.0}}, density_ * properties.Ixx,
                         density_ * properties.Iyy, density_ * properties.Izz,
                         density_ * properties.Ixy, density_ * properties.Iyz,
                         density_ * properties.Izx);
  inertia_.mt = {{0.0, 0.0, 0.0}};
  center_of_mass_wrt_origin_ = properties.centroid;
  std::stringstream ss;
  ss.precision(10);
  ss << "Body '" << name_ << "' from density " << density_ << ": volume "
     << properties.volume << ", mass " << m << ", center of mass ["
     << properties.centroid.x << ", " << properties.centroid.y << ", "
     << properties.centroid.z << "], inertia [" << inertia_.Ixx << ", "
     << inertia_.Iyy << ", " << inertia_.Izz << ", " << inertia_.Ixy << ", "
     << inertia_.Iyz << ", " << inertia_.Izx << "]"
     << (closed ? "" : " (convex hull)");
  SHIFT_INFO(ss.str());
}
