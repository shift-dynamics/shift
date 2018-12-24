#include <fstream>
#include <iostream>

#include <json/json.h>

#include "shift-core/behaviors/behavior.h"
#include "shift-core/bodies/body.h"
#include "shift-core/collisiongeometry/collisiongeometry.h"
#include "shift-core/constraints/constraint.h"
#include "shift-core/context.h"
#include "shift-core/forcemodules/forcemodule.h"
#include "shift-core/geometry/geometry.h"
#include "shift-core/integrators/integrator.h"
#include "shift-core/joints/joint.h"
#include "shift-core/lights/light.h"
#include "shift-core/loggers/logger.h"
#include "shift-core/sensors/sensor.h"
#include "shift-core/solvers/solver.h"
#include "shift-core/terrain/terrain.h"
#include "shift-core/topology.h"

void Shift::Context::readJSON(const std::string& filename) {
  std::ifstream file(filename);
  file >> root_;
  if(root_.isMember("model_parameters")) {
    modelParameters_.readJSON(root_["model_parameters"]);
  }
  fixed_.readJSON(root_, true);
  setFullyQualifiedNames();
  connect();
  loadData();
}

void Shift::Context::writeJSON(std::ostream& ostream) {
  Json::StreamWriterBuilder builder;
  builder.settings_["indentation"] = indentation_;
  builder.settings_["dropNullPlaceholders"] = false;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  if(writer != NULL) {
    writer->write(root_, &ostream);
    ostream << std::endl;
  } else {
    std::cerr << "no writer!" << std::endl;
  }
}

void Shift::Context::writeJSON() {
  writeJSON(filename_);
}

void Shift::Context::writeJSON(const std::string& filename) {
  std::ofstream file(filename);
  Json::StreamWriterBuilder builder;
  builder.settings_["indentation"] = indentation_;
  builder.settings_["dropNullPlaceholders"] = false;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  if(writer != NULL) {
    writer->write(root_, &file);
    file << std::endl;
  } else {
    std::cerr << "no writer!" << std::endl;
  }
}

void Shift::Context::initialize() {

  try {
    if(!Shift::Behavior::Modules::initialized()) {
      throw "Behavior modules did not properly initialize";
    }
    if(!Shift::Body::Modules::initialized()) {
      throw "Body modules did not properly initialize";
    }
    if(!Shift::CollisionGeometry::Modules::initialized()) {
      throw "Collision geometry modules did not properly initialize";
    }
    if(!Shift::Constraint::Modules::initialized()) {
      throw "Constraint modules did not properly initialize";
    }
    if(!Shift::ForceModule::Modules::initialized()) {
      throw "Force modules did not properly initialize";
    }
    if(!Shift::Geometry::Modules::initialized()) {
      throw "Geometry modules did not properly initialize";
    }
    if(!Shift::Integrator::Modules::initialized()) {
      throw "Integrator modules did not properly initialize";
    }
    if(!Shift::Joint::Modules::initialized()) {
      throw "Joint modules did not properly initialize";
    }
    if(!Shift::Light::Modules::initialized()) {
      throw "Light modules did not properly initialize";
    }
    if(!Shift::Logger::Modules::initialized()) {
      throw "Logger modules did not properly initialize";
    }
    if(!Shift::Sensor::Modules::initialized()) {
      throw "Sensor modules did not properly initialize";
    }
    if(!Shift::Solver::Modules::initialized()) {
      throw "Solver modules did not properly initialize";
    }
    if(!Shift::Terrain::Modules::initialized()) {
      throw "Terrain modules did not properly initialize";
    }
  } catch(const char* msg) {
    std::cerr << "Error in loading modules: " << msg << std::endl;
    exit(-1);
  }
}

void Shift::Context::print() {
  modelParameters_.print();
  fixed_.print();
}

void Shift::Context::setFullyQualifiedNames() {
  std::vector<std::string> rootName;
  fixed_.setFullyQualifiedNames(rootName);
}

void Shift::Context::setActiveObjects() {
  fixed_.setActiveObjects(true);
}

void Shift::Context::appendActive(Shift::Behavior::BehaviorBase* behavior) {
  if(behavior->isActive()) {
    for(auto i : activeBehaviors_) {
      if(i == behavior) {
        return;
      }
    }
    activeBehaviors_.push_back(behavior);
  }
}

void Shift::Context::appendActive(Shift::Body::BodyBase* body) {

  if(body && body->isActive()) {
    for(auto i : activeBodies_) {
      std::cout << (*i).getName() << std::endl;
      if(i == body) {
        return;
      }
    }
    activeBodies_.push_back(body);
  }
}

void Shift::Context::appendActive(
  Shift::CollisionGeometry::CollisionGeometryBase* geometry) {
  if(geometry->isActive()) {
    for(auto i : activeCollisionGeometry_) {
      if(i == geometry) {
        return;
      }
    }
    activeCollisionGeometry_.push_back(geometry);
  }
}

void Shift::Context::appendActive(
  Shift::Constraint::ConstraintBase* constraint) {
  if(constraint->isActive()) {
    for(auto i : activeConstraints_) {
      if(i == constraint) {
        return;
      }
    }
    activeConstraints_.push_back(constraint);
  }
}

void Shift::Context::appendActive(Shift::ForceModule::ForceModuleBase* module) {
  if(module->isActive()) {
    for(auto i : activeForceModules_) {
      if(i == module) {
        return;
      }
    }
    activeForceModules_.push_back(module);
  }
}

void Shift::Context::appendActive(Shift::Geometry::GeometryBase* geometry) {
  if(geometry->isActive()) {
    for(auto i : activeGeometry_) {
      if(i == geometry) {
        return;
      }
    }
    activeGeometry_.push_back(geometry);
  }
}

void Shift::Context::appendActive(Shift::Joint::JointBase* joint) {
  if(joint->isActive()) {
    for(auto i : activeJoints_) {
      if(i == joint) {
        return;
      }
    }
    activeJoints_.push_back(joint);
  }
}

void Shift::Context::appendActive(Shift::Light::LightBase* light) {
  if(light->isActive()) {
    for(auto i : activeLights_) {
      if(i == light) {
        return;
      }
    }
    activeLights_.push_back(light);
  }
}

void Shift::Context::appendActive(Shift::Sensor::SensorBase* sensor) {
  if(sensor->isActive()) {
    for(auto i : activeSensors_) {
      if(i == sensor) {
        return;
      }
    }
    activeSensors_.push_back(sensor);
  }
}

void Shift::Context::appendActive(Shift::Terrain::TerrainBase* terrain) {
  if(terrain->isActive()) {
    for(auto i : activeTerrain_) {
      if(i == terrain) {
        return;
      }
    }
    activeTerrain_.push_back(terrain);
  }
}

void Shift::Context::removeActive(Shift::Behavior::BehaviorBase* behavior) {
  for(unsigned int i = 0; i < activeBehaviors_.size(); i++) {
    if(activeBehaviors_[i] == behavior) {
      activeBehaviors_.erase(activeBehaviors_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Body::BodyBase* body) {
  for(unsigned int i = 0; i < activeBodies_.size(); i++) {
    if(activeBodies_[i] == body) {
      activeBodies_.erase(activeBodies_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(
  Shift::CollisionGeometry::CollisionGeometryBase* geometry) {
  for(unsigned int i = 0; i < activeCollisionGeometry_.size(); i++) {
    if(activeCollisionGeometry_[i] == geometry) {
      activeCollisionGeometry_.erase(activeCollisionGeometry_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(
  Shift::Constraint::ConstraintBase* constraint) {
  for(unsigned int i = 0; i < activeConstraints_.size(); i++) {
    if(activeConstraints_[i] == constraint) {
      activeConstraints_.erase(activeConstraints_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::ForceModule::ForceModuleBase* module) {
  for(unsigned int i = 0; i < activeForceModules_.size(); i++) {
    if(activeForceModules_[i] == module) {
      activeForceModules_.erase(activeForceModules_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Geometry::GeometryBase* geometry) {
  for(unsigned int i = 0; i < activeGeometry_.size(); i++) {
    if(activeGeometry_[i] == geometry) {
      activeGeometry_.erase(activeGeometry_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Joint::JointBase* joint) {
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {
    if(activeJoints_[i] == joint) {
      activeJoints_.erase(activeJoints_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Light::LightBase* light) {
  for(unsigned int i = 0; i < activeLights_.size(); i++) {
    if(activeLights_[i] == light) {
      activeLights_.erase(activeLights_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Sensor::SensorBase* sensor) {
  for(unsigned int i = 0; i < activeSensors_.size(); i++) {
    if(activeSensors_[i] == sensor) {
      activeSensors_.erase(activeSensors_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Terrain::TerrainBase* terrain) {
  for(unsigned int i = 0; i < activeTerrain_.size(); i++) {
    if(activeTerrain_[i] == terrain) {
      activeTerrain_.erase(activeTerrain_.begin() + i);
      return;
    }
  }
}

void Shift::Context::setActive(Shift::Behavior::BehaviorBase* behavior) {
  if(behavior->isActive()) {
    appendActive(behavior);
  } else {
    removeActive(behavior);
  }
}

void Shift::Context::setActive(Shift::Body::BodyBase* body) {
  if(body->isActive()) {
    appendActive(body);
  } else {
    removeActive(body);
  }
}

void Shift::Context::setActive(
  Shift::CollisionGeometry::CollisionGeometryBase* geometry) {
  if(geometry->isActive()) {
    appendActive(geometry);
  } else {
    removeActive(geometry);
  }
}

void Shift::Context::setActive(Shift::Constraint::ConstraintBase* constraint) {
  if(constraint->isActive()) {
    appendActive(constraint);
  } else {
    removeActive(constraint);
  }
}

void Shift::Context::setActive(Shift::ForceModule::ForceModuleBase* module) {
  if(module->isActive()) {
    appendActive(module);
  } else {
    removeActive(module);
  }
}

void Shift::Context::setActive(Shift::Geometry::GeometryBase* geometry) {
  if(geometry->isActive()) {
    appendActive(geometry);
  } else {
    removeActive(geometry);
  }
}

void Shift::Context::setActive(Shift::Joint::JointBase* joint) {
  if(joint->isActive()) {
    appendActive(joint);
  } else {
    removeActive(joint);
  }
}

void Shift::Context::setActive(Shift::Light::LightBase* light) {
  if(light->isActive()) {
    appendActive(light);
  } else {
    removeActive(light);
  }
}

void Shift::Context::setActive(Shift::Sensor::SensorBase* sensor) {
  if(sensor->isActive()) {
    appendActive(sensor);
  } else {
    removeActive(sensor);
  }
}

void Shift::Context::setActive(Shift::Terrain::TerrainBase* terrain) {
  if(terrain->isActive()) {
    appendActive(terrain);
  } else {
    removeActive(terrain);
  }
}

void Shift::Context::clearActiveBehavior() {
  activeBehaviors_.clear();
}

void Shift::Context::clearActiveBodies() {
  activeBodies_.clear();
}

void Shift::Context::clearActiveCollisionGeometry() {
  activeCollisionGeometry_.clear();
}

void Shift::Context::clearActiveConstraints() {
  activeConstraints_.clear();
}

void Shift::Context::clearActiveForceModules() {
  activeForceModules_.clear();
}

void Shift::Context::clearActiveGeometry() {
  activeGeometry_.clear();
}

void Shift::Context::clearActiveJoints() {
  activeJoints_.clear();
}

void Shift::Context::clearActiveLights() {
  activeLights_.clear();
}

void Shift::Context::clearActiveSensors() {
  activeSensors_.clear();
}

void Shift::Context::clearActiveTerrain() {
  activeTerrain_.clear();
}

void Shift::Context::clearActiveArrays() {
  clearActiveBehavior();
  clearActiveBodies();
  clearActiveCollisionGeometry();
  clearActiveConstraints();
  clearActiveForceModules();
  clearActiveGeometry();
  clearActiveJoints();
  clearActiveLights();
  clearActiveSensors();
  clearActiveTerrain();
}

Shift::Behavior::BehaviorBase*
Shift::Context::getBehavior(std::vector<std::string>& name) {
  for(auto& i : activeBehaviors_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Body::BodyBase* Shift::Context::getBody(std::vector<std::string>& name) {
  for(auto& i : activeBodies_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::CollisionGeometry::CollisionGeometryBase*
Shift::Context::getCollisionGeometry(std::vector<std::string>& name) {
  for(auto& i : activeCollisionGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Constraint::ConstraintBase*
Shift::Context::getConstraint(std::vector<std::string>& name) {
  for(auto& i : activeConstraints_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::ForceModule::ForceModuleBase*
Shift::Context::getForceModule(std::vector<std::string>& name) {
  for(auto& i : activeForceModules_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Geometry::GeometryBase*
Shift::Context::getGeometry(std::vector<std::string>& name) {
  for(auto& i : activeGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Joint::JointBase*
Shift::Context::getJoint(std::vector<std::string>& name) {
  for(auto& i : activeJoints_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Light::LightBase*
Shift::Context::getLight(std::vector<std::string>& name) {
  for(auto& i : activeLights_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Sensor::SensorBase*
Shift::Context::getSensor(std::vector<std::string>& name) {
  for(auto& i : activeSensors_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Terrain::TerrainBase*
Shift::Context::getTerrain(std::vector<std::string>& name) {
  for(auto& i : activeTerrain_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

void Shift::Context::connect() {
  for(auto& i : activeBehaviors_) {
    i->connect();
  }
  for(auto& i : activeBodies_) {
    i->connect();
  }
  for(auto& i : activeCollisionGeometry_) {
    i->connect();
  }
  for(auto& i : activeConstraints_) {
    i->connect();
  }
  for(auto& i : activeForceModules_) {
    i->connect();
  }
  for(auto& i : activeGeometry_) {
    i->connect();
  }
  for(auto& i : activeJoints_) {
    i->connect();
  }
  for(auto& i : activeLights_) {
    i->connect();
  }
  for(auto& i : activeSensors_) {
    i->connect();
  }
  for(auto& i : activeTerrain_) {
    i->connect();
  }
}

void Shift::Context::loadData() {
  for(auto& i : activeBehaviors_) {
    i->load();
  }
  for(auto& i : activeBodies_) {
    i->load();
  }
  for(auto& i : activeCollisionGeometry_) {
    i->load();
  }
  for(auto& i : activeConstraints_) {
    i->load();
  }
  for(auto& i : activeForceModules_) {
    i->load();
  }
  for(auto& i : activeGeometry_) {
    i->load();
  }
  for(auto& i : activeJoints_) {
    i->load();
  }
  for(auto& i : activeLights_) {
    i->load();
  }
  for(auto& i : activeSensors_) {
    i->load();
  }
  for(auto& i : activeTerrain_) {
    i->load();
  }
}
