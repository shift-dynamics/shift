#include "shift-core/bodies/body.h"
#include "shift-core/bodies/fixed.h"
#include "shift-core/bodies/free.h"
#include "shift-core/bodies/rigid.h"
#include "shift-core/factory.h"
#include "shift-core/lights/light.h"

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
  return true;
}

void Shift::Body::BodyBase::addKeys() {
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

void Shift::Body::BodyBase::readJSON(Json::Value& value) {

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
      Shift::Light::LightBase* light =
        Shift::Factory<Shift::Light::LightBase>::get()->create(lightType);
      light->readJSON(value["lights"][i]);
      appendLight(light);
    }
  }

  if(value.isMember("geometry") && value["geometry"].isArray()) {
    for(unsigned int i = 0; i < value["geometry"].size(); i++) {
      std::string geometryType;
      if(value["geometry"][i].isMember("type") &&
         value["geometry"][i]["type"].isString()) {
        geometryType.assign(value["geometry"][i]["type"].asString());
        Shift::Geometry::GeometryBase* geometry =
          Shift::Factory<Shift::Geometry::GeometryBase>::get()->create(
            geometryType);
        geometry->readJSON(value["geometry"][i]);
        appendGeometry(geometry);
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

        Shift::CollisionGeometry::CollisionGeometryBase* geometry =
          Shift::Factory<Shift::CollisionGeometry::CollisionGeometryBase>::get()
            ->create(geometryType);
        geometry->readJSON(value["collision_geometry"][i]);
        appendCollisionGeometry(geometry);
      }
    }
  }
}

void Shift::Body::BodyBase::appendFrame(Frame& frame) {
  frames_.push_back(frame);
}

void Shift::Body::BodyBase::appendLight(Shift::Light::LightBase* light) {
  lights_.push_back(light);
}

void Shift::Body::BodyBase::appendGeometry(
  Shift::Geometry::GeometryBase* geometry) {
  geometry_.push_back(geometry);
}

void Shift::Body::BodyBase::appendCollisionGeometry(
  Shift::CollisionGeometry::CollisionGeometryBase* collisionGeometry) {
  collisionGeometry_.push_back(collisionGeometry);
}

void Shift::Body::BodyBase::print() {
  std::cout << "body: " << name_ << std::endl;
  std::cout << "frames: " << std::endl;
  for(unsigned int i = 0; i < frames_.size(); i++) {
    frames_[i].print();
  }
  if(lights_.size() > 0) {
    std::cout << "lights: " << std::endl;
    for(unsigned int i = 0; i < lights_.size(); i++) {
      lights_[i]->print();
    }
  }
}
