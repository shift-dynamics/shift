#include "shift/core/body/Base.h"
#include "shift/core/body/Fixed.h"
#include "shift/core/body/Free.h"
#include "shift/core/body/Rigid.h"
#include "shift/core/body/Virtual.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/light/Base.h"
#include "shift/core/Exception.h"

#include "ksl/matrix.h"

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

void shift::core::body::Base::set_reference_frame(shift::core::Frame* referenceFrame) {
  for(auto& frame : frames_.get_data()) {
    frame->set_reference_frame(referenceFrame);
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
