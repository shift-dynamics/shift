#include "shift/core/body/Base.h"
#include "shift/core/body/Fixed.h"
#include "shift/core/body/Free.h"
#include "shift/core/body/Rigid.h"
#include "shift/core/body/Virtual.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/light/Base.h"

#include "ksl/matrix.h"

#include <assert.h>
#include <iostream>

bool shift::core::body::Modules::initialized() {
  if(!shift::core::body::Fixed::isRegistered()) {
    return false;
  }
  if(!shift::core::body::Free::isRegistered()) {
    return false;
  }
  if(!shift::core::body::Rigid::isRegistered()) {
    return false;
  }
  if(!shift::core::body::Virtual::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::body::Base::addParameters() {

  addParameter("name", &name_,
               "User defined descriptive body name; must be unique within a "
               "subassembly (required)",
               true);

  addParameter(
    "active", &active_,
    "Indicates whether body is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model "
    "(optional, default=true).",
    false);

  addParameter(
    "center_of_mass", &center_of_mass_wrt_origin_,
    "Specifies a vector from the body origin to the body center of mass "
    "expressed in the body-fixed coordinate system. (optional, "
    "default={0,0,0})",
    false);

  addParameter(
    "inertia", &inertia_,
    "Specify body mass and centroidal moment of inertia "
    "expressed in the body-fixed coordinate system. Note that if a zero or "
    "very low mass is specified, it may cause problems for some solvers. "
    "(optional, default={mass=1,Ixx=1,Iyy=1,Izz=1,Ixy=0,Iyz=0,Izx=0})",
    false);

  addParameter("frames", &frames_,
               "Specify a number of body-fixed reference frames. Each frame "
               "in a body must have a unique name identified and 'origin' is "
               "a reserved system name that may not be used.",
               false);

  addParameter("lights", &lights_,
               "Specify a number of body-fixed lights, used to "
               "illuminate scene for visualization.",
               false);

  addParameter("geometry", &geometry_,
               "Specify a number of body-fixed geometry, used for "
               "visualization or contact detection. Each geometry "
               "in a body must have a unique name.",
               false);

  addParameter("collision_geometry", &collision_geometry_,
               "Specify a number of body-fixed collision geometries, used for "
               "contact detection. Each collision geometry "
               "in a body must have a unique name.",
               false);
}

void shift::core::body::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, active_ && parent_is_active);

  for(auto& geometry : geometry_.data()) {
    geometry->setActiveObjects(active_ && parent_is_active);
  }
  for(auto& collision_geometry : collision_geometry_.data()) {
    collision_geometry->setActiveObjects(active_ && parent_is_active);
  }
}

void shift::core::body::Base::setParentJoint(shift::core::joint::Base* parent) {
  parent_joint_ = parent;
  auto referenceFrame = parent_joint_->systemChildFrame();
  for(auto& frame : frames_.data()) {
    frame->setIsReference(frame.get() == referenceFrame);
  }
  setDWRTReference();
}

void shift::core::body::Base::connect() {}

shift::core::Frame* shift::core::body::Base::origin() {
  for(auto& frame : frames_.data()) {
    if(frame->name() == "origin") {
      return frame.get();
    }
  }
  return nullptr;
}

shift::core::Frame* shift::core::body::Base::referenceFrame() {
  for(auto& frame : frames_.data()) {
    if(frame->isReference()) {
      return frame.get();
    }
  }
  std::cerr << "no reference frame found on body: " << name() << std::endl;
  assert(0);
  return nullptr;
}

void shift::core::body::Base::setDWRTReference() {
  Frame* reference = referenceFrame();
  ksl_SE3_t dReference;
  ksl_SE3_setIdentity(&dReference);
  reference->setDWRTReference(dReference);
  const ksl_SE3_t* referenceWRTOrigin = reference->dWRTOrigin();
  for(auto& frame : frames_.data()) {
    if(!frame->isReference()) {
      const ksl_SE3_t* dWRTOrigin = frame->dWRTOrigin();
      ksl_SE3_t dWRTReference;
      ksl_product_dinvd(referenceWRTOrigin, dWRTOrigin, &dWRTReference);
      frame->setDWRTReference(dWRTReference);
    }
  }
}

void shift::core::body::Base::setDWRTReference(ksl_SE3_t dReference) {
  Frame* reference = referenceFrame();
  reference->setDWRTReference(dReference);
  const ksl_SE3_t* referenceWRTOrigin = reference->dWRTOrigin();
  for(auto& frame : frames_.data()) {
    if(!frame->isReference()) {
      const ksl_SE3_t* dWRTOrigin = frame->dWRTOrigin();
      ksl_SE3_t temp;
      ksl_product_dinvd(referenceWRTOrigin, dWRTOrigin, &temp);
      ksl_SE3_t dWRTReference;
      ksl_product_dd(&dReference, &temp, &dWRTReference);
      frame->setDWRTReference(dWRTReference);
    }
  }
}

void shift::core::body::Base::setReferenceFrame(shift::core::Frame* referenceFrame) {
  for(auto& frame : frames_.data()) {
    frame->setReferenceFrame(referenceFrame);
  }
  setDWRTReference();
};

void shift::core::body::Base::setReferenceFrame() {
  auto reference = referenceFrame();
  for(auto& frame : frames_.data()) {
    if(frame.get() == reference) {
      frame->setReferenceFrame(nullptr);
    } else {
      frame->setReferenceFrame(reference);
    }
  }
  setDWRTReference();
};

const ksl_vec3_t shift::core::body::Base::centerOfMassWRTReference() {
  if(referenceFrame() != origin()) {
    // D_or // transformation from origin to reference frame
    // D_ro // transformation from reference frame to origin
    // t_om_o // vector from origin (o) to center of mass frame (m),
    // expressed in origin coordinates
    // t_rm_r // vector from reference frame to center of mass, expressed in
    // reference frame coordinates

    // t_rm_r = D_ro_ro * t_om_o
    ksl_vec3_t t_om_o = center_of_mass_wrt_origin_;
    const ksl_SE3_t* D_or = referenceFrame()->dWRTOrigin();
    ksl_SE3_t D_ro;
    ksl_SE3_inverted(D_or, &D_ro);
    ksl_vec3_t t_rm_r;
    ksl_product_dv(&D_ro, &t_om_o, &t_rm_r);
    return t_rm_r;
  }
  return center_of_mass_wrt_origin_;
}

void shift::core::body::Base::clearInTreeFlags() {
  for(auto& frame : frames_.data()) {
    frame->setIndex(-1);
    frame->setInTree(false);
  }
}

unsigned int shift::core::body::Base::frameInTreeCount() {
  unsigned int count = 0;
  for(auto& frame : frames_.data()) {
    if(frame->inTree()) {
      count++;
    }
  }
  return count;
}

void shift::core::body::Base::load() {}
