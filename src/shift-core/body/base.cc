#include "shift-core/body/base.h"
#include "shift-core/body/fixed.h"
#include "shift-core/body/free.h"
#include "shift-core/body/rigid.h"
#include "shift-core/body/virtual.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"
#include "shift-core/light/base.h"

#include "ksl/matrix.h"

#include <assert.h>
#include <iostream>

bool Shift::Body::Modules::initialized() {
  if(!Shift::Body::Fixed::isRegistered()) {
    return false;
  }
  if(!Shift::Body::Free::isRegistered()) {
    return false;
  }
  if(!Shift::Body::Rigid::isRegistered()) {
    return false;
  }
  if(!Shift::Body::Virtual::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Body::Base::addParameters() {

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
}

void Shift::Body::Base::appendFrame(Frame* frame) {
  frames_.push_back(frame);
}

void Shift::Body::Base::setFullyQualifiedNames(
  std::vector<std::string>& baseName) {
  setFullyQualifiedName(baseName);
  std::vector<std::string> bodyName = fullyQualifiedName();
  for(auto& i : frames_.data()) {
    i->setFullyQualifiedName(bodyName);
  }
  for(auto& i : lights_.data()) {
    i->setFullyQualifiedName(bodyName);
  }
  for(auto& i : geometry_.data()) {
    i->setFullyQualifiedName(bodyName);
  }
  for(auto& i : collision_geometry_.data()) {
    i->setFullyQualifiedName(bodyName);
  }
}

void Shift::Body::Base::setActiveObjects(bool parentIsActive) {
  if(active_ && context_ && parentIsActive) {
    for(auto& i : lights_.data()) {
      context_->setActive(i.get());
    }
    for(auto& i : geometry_.data()) {
      context_->setActive(i.get());
    }
    for(auto& i : collision_geometry_.data()) {
      context_->setActive(i.get());
    }
  } else {
    if(context_) {
      for(auto& i : lights_.data()) {
        context_->removeActive(i.get());
      }
      for(auto& i : geometry_.data()) {
        context_->removeActive(i.get());
      }
      for(auto& i : collision_geometry_.data()) {
        context_->removeActive(i.get());
      }
    }
  }
};

Shift::Frame* Shift::Body::Base::getFrame(std::string name) {
  for(auto& i : frames_.data()) {
    if(i->name() == name) {
      return i.get();
    }
  }
  return nullptr;
}

void Shift::Body::Base::setParentJoint(Shift::Joint::Base* parent) {
  parent_joint_ = parent;
  auto referenceFrame = parent_joint_->systemChildFrame();
  for(auto& frame : frames_.data()) {
    frame->setIsReference(frame.get() == referenceFrame);
  }
  setDWRTReference();
}

void Shift::Body::Base::load() {}

void Shift::Body::Base::connect() {}

Shift::Frame* Shift::Body::Base::origin() {
  return getFrame("origin");
}

Shift::Frame* Shift::Body::Base::referenceFrame() {
  for(auto& frame : frames_.data()) {
    if(frame->isReference()) {
      return frame.get();
    }
  }
  std::cerr << "no reference frame found on body: " << name() << std::endl;
  assert(0);
  return nullptr;
}

void Shift::Body::Base::setDWRTReference() {
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

void Shift::Body::Base::setDWRTReference(ksl_SE3_t dReference) {
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

void Shift::Body::Base::setReferenceFrame(Shift::Frame* referenceFrame) {
  for(auto& frame : frames_.data()) {
    frame->setReferenceFrame(referenceFrame);
  }
  setDWRTReference();
};

void Shift::Body::Base::setReferenceFrame() {
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

const ksl_vec3_t Shift::Body::Base::centerOfMassWRTReference() {
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

void Shift::Body::Base::clearInTreeFlags() {
  for(auto& frame : frames_.data()) {
    frame->setIndex(-1);
    frame->setInTree(false);
  }
}

unsigned int Shift::Body::Base::frameInTreeCount() {
  unsigned int count = 0;
  for(auto& frame : frames_.data()) {
    if(frame->inTree()) {
      count++;
    }
  }
  return count;
}
