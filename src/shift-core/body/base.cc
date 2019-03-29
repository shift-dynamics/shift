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

void Shift::Body::Base::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(true);
  name.setDescription("User defined descriptive body name.");
  addSerialEntry(name);

  SerialEntry active("active", SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether body is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.");
  addSerialEntry(active);
}

void Shift::Body::Base::readJSON(Json::Value& value) {

  /* iterate over keys */
  Shift::Serialize::readJSON(value);

  if(value.isMember("frames") && value["frames"].isArray()) {
    for(unsigned int i = 0; i < value["frames"].size(); i++) {
      Frame frame;
      frame.readJSON(value["frames"][i]);
      appendFrame(frame);
    }
  }

  if(value.isMember("lights") && value["lights"].isArray()) {
    for(unsigned int i = 0; i < value["lights"].size(); i++) {
      std::string lightType("point");
      if(value["lights"][i].isMember("type") &&
         value["lights"][i]["type"].isString()) {
        lightType.assign(value["lights"][i]["type"].asString());
      }
      auto light = Shift::Factory<Shift::Light::Base>::get()->create(lightType);
      light->setContext(context_);
      light->readJSON(value["lights"][i]);
      lights_.push_back(std::move(light));
    }
  }

  if(value.isMember("geometry") && value["geometry"].isArray()) {
    for(unsigned int i = 0; i < value["geometry"].size(); i++) {
      std::string geometryType;
      if(value["geometry"][i].isMember("type") &&
         value["geometry"][i]["type"].isString()) {
        geometryType.assign(value["geometry"][i]["type"].asString());
        auto geometry =
          Shift::Factory<Shift::Geometry::Base>::get()->create(geometryType);
        geometry->setContext(context_);
        geometry->readJSON(value["geometry"][i]);
        geometry_.push_back(std::move(geometry));
      }
    }
  }

  if(value.isMember("collision_geometry") &&
     value["collision_geometry"].isArray()) {
    for(unsigned int i = 0; i < value["collision_geometry"].size(); i++) {
      std::string geometryType;
      if(value["collision_geometry"][i].isMember("type") &&
         value["collision_geometry"][i]["type"].isString()) {
        geometryType.assign(value["collision_geometry"][i]["type"].asString());

        auto geometry =
          Shift::Factory<Shift::CollisionGeometry::Base>::get()->create(
            geometryType);
        geometry->setContext(context_);
        geometry->readJSON(value["collision_geometry"][i]);
        collisionGeometry_.push_back(std::move(geometry));
      }
    }
  }
}

void Shift::Body::Base::appendFrame(Frame& frame) {
  frames_.push_back(frame);
}

void Shift::Body::Base::appendLight(
  std::unique_ptr<Shift::Light::Base>& light) {
  lights_.push_back(std::move(light));
}

void Shift::Body::Base::appendGeometry(
  std::unique_ptr<Shift::Geometry::Base>& geometry) {
  geometry_.push_back(std::move(geometry));
}

void Shift::Body::Base::appendCollisionGeometry(
  std::unique_ptr<Shift::CollisionGeometry::Base>& collisionGeometry) {
  collisionGeometry_.push_back(std::move(collisionGeometry));
}

void Shift::Body::Base::print() {
  std::cout << "body: " << name_ << std::endl;
  std::cout << "full name: ";
  for(unsigned int i = 0; i < fullyQualifiedName_.size(); i++) {
    std::cout << fullyQualifiedName_[i];
    if(i != fullyQualifiedName_.size() - 1) {
      std::cout << "::";
    }
  }
  std::cout << std::endl;
  std::cout << "frames: " << std::endl;
  for(auto& i : frames_) {
    i.print();
  }
  if(lights_.size() > 0) {
    std::cout << "lights: " << std::endl;
    for(auto& i : lights_) {
      i->print();
    }
  }
}

void Shift::Body::Base::setFullyQualifiedNames(
  std::vector<std::string>& baseName) {
  setFullyQualifiedName(baseName);
  std::vector<std::string> bodyName = fullyQualifiedName();
  for(auto& i : frames_) {
    i.setFullyQualifiedName(bodyName);
  }
  for(auto& i : lights_) {
    i->setFullyQualifiedName(bodyName);
  }
  for(auto& i : geometry_) {
    i->setFullyQualifiedName(bodyName);
  }
  for(auto& i : collisionGeometry_) {
    i->setFullyQualifiedName(bodyName);
  }
}

void Shift::Body::Base::setActiveObjects(bool parentIsActive) {
  if(active_ && context_ && parentIsActive) {
    for(auto& i : lights_) {
      context_->setActive(i.get());
    }
    for(auto& i : geometry_) {
      context_->setActive(i.get());
    }
    for(auto& i : collisionGeometry_) {
      context_->setActive(i.get());
    }
  } else {
    if(context_) {
      for(auto& i : lights_) {
        context_->removeActive(i.get());
      }
      for(auto& i : geometry_) {
        context_->removeActive(i.get());
      }
      for(auto& i : collisionGeometry_) {
        context_->removeActive(i.get());
      }
    }
  }
};

Shift::Frame* Shift::Body::Base::getFrame(std::string name) {
  for(auto& i : frames_) {
    if(i.name() == name) {
      return &i;
    }
  }
  return NULL;
}

void Shift::Body::Base::setParentJoint(Shift::Joint::Base* parent) {
  parentJoint_ = parent;
  auto referenceFrame = parentJoint_->systemChildFrame();
  for(auto& frame : frames_) {
    frame.setIsReference(&frame == referenceFrame);
  }
  setDWRTReference();
}

void Shift::Body::Base::load() {}

void Shift::Body::Base::connect() {}

Shift::Frame* Shift::Body::Base::origin() {
  return getFrame("origin");
}

Shift::Frame* Shift::Body::Base::referenceFrame() {
  for(auto& frame : frames_) {
    if(frame.isReference()) {
      return &frame;
    }
  }
  std::cerr << "no reference frame found on body: " << name() << std::endl;
  assert(0);
  return NULL;
}

void Shift::Body::Base::setDWRTReference() {
  Frame* reference = referenceFrame();
  ksl_SE3_t dReference;
  ksl_SE3_setIdentity(&dReference);
  reference->setDWRTReference(dReference);
  const ksl_SE3_t* referenceWRTOrigin = reference->DWRTOrigin();
  for(auto& frame : frames_) {
    if(!frame.isReference()) {
      const ksl_SE3_t* dWRTOrigin = frame.DWRTOrigin();
      ksl_SE3_t dWRTReference;
      ksl_product_dinvd(referenceWRTOrigin, dWRTOrigin, &dWRTReference);
      frame.setDWRTReference(dWRTReference);
    }
  }
}

void Shift::Body::Base::setDWRTReference(ksl_SE3_t dReference) {
  Frame* reference = referenceFrame();
  reference->setDWRTReference(dReference);
  const ksl_SE3_t* referenceWRTOrigin = reference->DWRTOrigin();
  for(auto& frame : frames_) {
    if(!frame.isReference()) {
      const ksl_SE3_t* dWRTOrigin = frame.DWRTOrigin();
      ksl_SE3_t temp;
      ksl_product_dinvd(referenceWRTOrigin, dWRTOrigin, &temp);
      ksl_SE3_t dWRTReference;
      ksl_product_dd(&dReference, &temp, &dWRTReference);
      frame.setDWRTReference(dWRTReference);
    }
  }
}

void Shift::Body::Base::setReferenceFrame(Shift::Frame* referenceFrame) {
  for(auto& frame : frames_) {
    frame.setReferenceFrame(referenceFrame);
  }
};

void Shift::Body::Base::setReferenceFrame() {
  auto reference = referenceFrame();
  for(auto& frame : frames_) {
    if(&frame == reference) {
      frame.setReferenceFrame(NULL);
    } else {
      frame.setReferenceFrame(reference);
    }
  }
};

void Shift::Body::Base::clearInTreeFlags() {
  for(auto& frame : frames_) {
    frame.setIndex(-1);
    frame.setInTree(false);
  }
}

unsigned int Shift::Body::Base::frameInTreeCount() {
  unsigned int count = 0;
  for(auto& frame : frames_) {
    if(frame.inTree()) {
      count++;
    }
  }
  return count;
}
