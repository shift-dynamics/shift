#include <assert.h>
#include <fstream>
#include <iostream>
#include <list>
#include <queue>
#include <string.h>

#include <json/json.h>

#include "shift-core/behavior/base.h"
#include "shift-core/body/base.h"
#include "shift-core/collisiongeometry/base.h"
#include "shift-core/constraint/base.h"
#include "shift-core/constraint/chord.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"
#include "shift-core/forcemodule/base.h"
#include "shift-core/geometry/base.h"
#include "shift-core/integrator/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/light/base.h"
#include "shift-core/logger/base.h"
#include "shift-core/sensor/base.h"
#include "shift-core/solver/base.h"
#include "shift-core/terrain/base.h"
#include "shift-core/topology.h"

void Shift::Context::readJSON(const std::string& filename) {
  std::ifstream file(filename);
  file >> root_;
  if(root_.isMember("model_parameters")) {
    modelParameters_.readJSON(root_["model_parameters"]);
  }
  fixed_.setContext(this);
  fixed_.readJSON(root_, true);
  setFullyQualifiedNames();
  setActiveObjects();
  connect();

  /* set topology */

  /* account for frames added for chord constraints and frames attached to fixed
   * body
   */
  // setFramesInTree();
  // mallocRunTimeArrays();
  // mallocDHVAArrays();
  levelOrderSort();
  processVirtualBodies();
  processPCL();
  processLoopMembers();
  detectSubstructures();
  processPCL();
  // setReferenceFrames();
  // setDVAIndex();
  // copyDVA();
  // compressRigidJoints();
  // setParentJointPointers();
  // countDHVA();
  // setDHVAPointers();
  // setFixedFrameGravityVector();
  // updateDH();
  // updateDHVA();
  // setDReferenceFrames();
  // initializeJointForces();
  // initializeGLReferenceFrames();
  // flattenForceModules();

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

void Shift::Context::appendActive(Shift::Behavior::Base* behavior) {
  if(behavior->isActive()) {
    for(auto i : activeBehaviors_) {
      if(i == behavior) {
        return;
      }
    }
    activeBehaviors_.push_back(behavior);
  }
}

void Shift::Context::appendActive(Shift::Body::Base* body) {
  if(body && body->isActive()) {
    for(auto& i : activeBodies_) {
      if(i == body) {
        return;
      }
    }
    activeBodies_.push_back(body);
  }
}

void Shift::Context::appendActive(Shift::CollisionGeometry::Base* geometry) {
  if(geometry->isActive()) {
    for(auto i : activeCollisionGeometry_) {
      if(i == geometry) {
        return;
      }
    }
    activeCollisionGeometry_.push_back(geometry);
  }
}

void Shift::Context::appendActive(Shift::Constraint::Base* constraint) {
  if(constraint->isActive()) {
    for(auto i : activeConstraints_) {
      if(i == constraint) {
        return;
      }
    }
    activeConstraints_.push_back(constraint);
  }
}

void Shift::Context::appendActive(Shift::ForceModule::Base* module) {
  if(module->isActive()) {
    for(auto i : activeForceModules_) {
      if(i == module) {
        return;
      }
    }
    activeForceModules_.push_back(module);
  }
}

void Shift::Context::appendActive(Shift::Geometry::Base* geometry) {
  if(geometry->isActive()) {
    for(auto i : activeGeometry_) {
      if(i == geometry) {
        return;
      }
    }
    activeGeometry_.push_back(geometry);
  }
}

void Shift::Context::appendActive(Shift::Joint::Base* joint) {
  if(joint->isActive()) {
    for(auto i : activeJoints_) {
      if(i == joint) {
        return;
      }
    }
    activeJoints_.push_back(joint);
  }
}

void Shift::Context::appendActive(Shift::Light::Base* light) {
  if(light->isActive()) {
    for(auto i : activeLights_) {
      if(i == light) {
        return;
      }
    }
    activeLights_.push_back(light);
  }
}

void Shift::Context::appendActive(Shift::Sensor::Base* sensor) {
  if(sensor->isActive()) {
    for(auto i : activeSensors_) {
      if(i == sensor) {
        return;
      }
    }
    activeSensors_.push_back(sensor);
  }
}

void Shift::Context::appendActive(Shift::Terrain::Base* terrain) {
  if(terrain->isActive()) {
    for(auto i : activeTerrain_) {
      if(i == terrain) {
        return;
      }
    }
    activeTerrain_.push_back(terrain);
  }
}

void Shift::Context::removeActive(Shift::Behavior::Base* behavior) {
  for(unsigned int i = 0; i < activeBehaviors_.size(); i++) {
    if(activeBehaviors_[i] == behavior) {
      activeBehaviors_.erase(activeBehaviors_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Body::Base* body) {
  for(unsigned int i = 0; i < activeBodies_.size(); i++) {
    if(activeBodies_[i] == body) {
      activeBodies_.erase(activeBodies_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::CollisionGeometry::Base* geometry) {
  for(unsigned int i = 0; i < activeCollisionGeometry_.size(); i++) {
    if(activeCollisionGeometry_[i] == geometry) {
      activeCollisionGeometry_.erase(activeCollisionGeometry_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Constraint::Base* constraint) {
  for(unsigned int i = 0; i < activeConstraints_.size(); i++) {
    if(activeConstraints_[i] == constraint) {
      activeConstraints_.erase(activeConstraints_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::ForceModule::Base* module) {
  for(unsigned int i = 0; i < activeForceModules_.size(); i++) {
    if(activeForceModules_[i] == module) {
      activeForceModules_.erase(activeForceModules_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Geometry::Base* geometry) {
  for(unsigned int i = 0; i < activeGeometry_.size(); i++) {
    if(activeGeometry_[i] == geometry) {
      activeGeometry_.erase(activeGeometry_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Joint::Base* joint) {
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {
    if(activeJoints_[i] == joint) {
      activeJoints_.erase(activeJoints_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Light::Base* light) {
  for(unsigned int i = 0; i < activeLights_.size(); i++) {
    if(activeLights_[i] == light) {
      activeLights_.erase(activeLights_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Sensor::Base* sensor) {
  for(unsigned int i = 0; i < activeSensors_.size(); i++) {
    if(activeSensors_[i] == sensor) {
      activeSensors_.erase(activeSensors_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Terrain::Base* terrain) {
  for(unsigned int i = 0; i < activeTerrain_.size(); i++) {
    if(activeTerrain_[i] == terrain) {
      activeTerrain_.erase(activeTerrain_.begin() + i);
      return;
    }
  }
}

void Shift::Context::setActive(Shift::Behavior::Base* behavior) {
  if(behavior && behavior->isActive()) {
    appendActive(behavior);
  } else {
    removeActive(behavior);
  }
}

void Shift::Context::setActive(Shift::Body::Base* body) {
  if(body && body->isActive()) {
    appendActive(body);
  } else {
    removeActive(body);
  }
}

void Shift::Context::setActive(Shift::CollisionGeometry::Base* geometry) {
  if(geometry && geometry->isActive()) {
    appendActive(geometry);
  } else {
    removeActive(geometry);
  }
}

void Shift::Context::setActive(Shift::Constraint::Base* constraint) {
  if(constraint && constraint->isActive()) {
    appendActive(constraint);
  } else {
    removeActive(constraint);
  }
}

void Shift::Context::setActive(Shift::ForceModule::Base* module) {
  if(module && module->isActive()) {
    appendActive(module);
  } else {
    removeActive(module);
  }
}

void Shift::Context::setActive(Shift::Geometry::Base* geometry) {
  if(geometry && geometry->isActive()) {
    appendActive(geometry);
  } else {
    removeActive(geometry);
  }
}

void Shift::Context::setActive(Shift::Joint::Base* joint) {
  if(joint && joint->isActive()) {
    appendActive(joint);
  } else {
    removeActive(joint);
  }
}

void Shift::Context::setActive(Shift::Light::Base* light) {
  if(light && light->isActive()) {
    appendActive(light);
  } else {
    removeActive(light);
  }
}

void Shift::Context::setActive(Shift::Sensor::Base* sensor) {
  if(sensor && sensor->isActive()) {
    appendActive(sensor);
  } else {
    removeActive(sensor);
  }
}

void Shift::Context::setActive(Shift::Terrain::Base* terrain) {
  if(terrain && terrain->isActive()) {
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

Shift::Behavior::Base*
Shift::Context::getBehavior(std::vector<std::string>& name) {
  for(auto& i : activeBehaviors_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Body::Base* Shift::Context::getBody(std::vector<std::string>& name) {
  for(auto& i : activeBodies_) {

    if(i && Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                     name)) {
      return i;
    }
  }
  return NULL;
}

Shift::CollisionGeometry::Base*
Shift::Context::getCollisionGeometry(std::vector<std::string>& name) {
  for(auto& i : activeCollisionGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {

      return i;
    }
  }
  return NULL;
}

Shift::Constraint::Base*
Shift::Context::getConstraint(std::vector<std::string>& name) {
  for(auto& i : activeConstraints_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::ForceModule::Base*
Shift::Context::getForceModule(std::vector<std::string>& name) {
  for(auto& i : activeForceModules_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Geometry::Base*
Shift::Context::getGeometry(std::vector<std::string>& name) {
  for(auto& i : activeGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Joint::Base* Shift::Context::getJoint(std::vector<std::string>& name) {
  for(auto& i : activeJoints_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Light::Base* Shift::Context::getLight(std::vector<std::string>& name) {
  for(auto& i : activeLights_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Sensor::Base* Shift::Context::getSensor(std::vector<std::string>& name) {
  for(auto& i : activeSensors_) {
    if(Shift::Topology::matchFullyQualifiedName(i->getFullyQualifiedName(),
                                                name)) {
      return i;
    }
  }
  return NULL;
}

Shift::Terrain::Base*
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

Shift::Body::Base* Shift::Context::getFixedBody() {
  return fixed_.getBody(0);
}

void Shift::Context::levelOrderSort() {

  std::queue<Shift::Joint::Base*> queue;
  std::vector<Shift::Joint::Base*> joints;
  std::vector<Shift::Body::Base*> bodies;

  /* find joints attached to fixed body */
  auto it = activeJoints_.begin();
  while(it != activeJoints_.end()) {
    assert((*it)->getParentBody() != NULL);
    assert((*it)->getChildBody() != NULL);
    if((*it)->getParentBody() == getFixedBody()) {
      (*it)->setSense(1);
      (*it)->setLevel(1);
      queue.push(*it);
      it = activeJoints_.erase(it);
    } else if((*it)->getChildBody() == getFixedBody()) {
      (*it)->setSense(-1);
      (*it)->setLevel(1);
      queue.push(*it);
      it = activeJoints_.erase(it);
    } else {
      it++;
    }
  }

  while(!queue.empty()) {
    auto joint = queue.front();
    queue.pop();
    Shift::Body::Base* body = NULL;
    if(joint->getSense() == 1) {
      body = joint->getChildBody();
    } else if(joint->getSense() == -1) {
      body = joint->getParentBody();
    }
    joints.push_back(joint);
    assert(body);

    /* find any joints attached to joint's child body */
    it = activeJoints_.begin();
    while(it != activeJoints_.end()) {
      if((*it)->getParentBody() == body) {
        (*it)->setSense(1);
        (*it)->setLevel(joint->getLevel() + 1);
        queue.push(*it);
        it = activeJoints_.erase(it);
      } else if((*it)->getChildBody() == body) {
        (*it)->setSense(-1);
        (*it)->setLevel(joint->getLevel() + 1);
        queue.push(*it);
        it = activeJoints_.erase(it);
      } else {
        it++;
      }
    }
  }

  activeJoints_ = joints;
}

void Shift::Context::processVirtualBodies() {

  std::vector<Shift::Body::Base*> bodies(activeJoints_.size());

  std::cout << activeJoints_.size() << std::endl;
  std::cout << activeBodies_.size() << std::endl;
  for(auto& body : activeBodies_) {
    std::cout << "body: " << body->getName() << std::endl;
  }
  // assert(activeJoints_.size() >= activeBodies_.size());

  std::cout << activeJoints_.size() - activeBodies_.size() << std::endl;

  chordConstraints_.clear();
  int j = 0;
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {

    bool found = false;
    auto body = activeJoints_[i]->getSystemChildBody();
    for(auto it = activeBodies_.begin(); it < activeBodies_.end(); it++) {
      if(body == (*it)) {
        activeBodies_.erase(it);
        bodies[i] = body;
        found = true;
        break;
      }
    }

    if(!found) {
      auto constraint =
        Shift::Factory<Shift::Constraint::Base>::get()->create("chord");

      Shift::Constraint::Chord* chordConstraint =
        dynamic_cast<Shift::Constraint::Chord*>(constraint.get());

      auto virtualBody =
        Shift::Factory<Shift::Body::Base>::get()->create("virtual");
      virtualBody->setName("_virtualBody[" + std::to_string(j++) + "]");

      chordConstraint->setParentBody(virtualBody.get());
      chordConstraint->setParentFrame(virtualBody->getOrigin());

      auto chordJoint = activeJoints_[i];
      chordConstraint->setChildBody(chordJoint->getSystemChildBody());
      chordConstraint->setChildFrame(chordJoint->getSystemChildFrame());
      chordConstraint->setOriginalParentBody(chordJoint->getSystemParentBody());
      chordConstraint->setOriginalParentFrame(
        chordJoint->getSystemParentFrame());

      /* add virtual bodies and chord constraints to active lists */
      bodies[i] = virtualBody.get();

      activeConstraints_.push_back(constraint.get());
      chordConstraints_.push_back(chordConstraint);

      /* transfer ownership of smart pointers for virtual bodies and chord
       * constraints to `Context` data structure */
      bodies_.push_back(std::move(virtualBody));
      constraints_.push_back(std::move(constraint));
    }
  }

  activeBodies_ = bodies;
}

void Shift::Context::processPCL() {
  for(unsigned int i = 0; i < activeBodies_.size(); i++) {
    activeBodies_[i]->setIndex(i);
  }
  pcl_.clear();
  pcl_.resize(activeJoints_.size());
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {
    auto joint = activeJoints_[i];
    auto parentBody = joint->getSystemParentBody();
    if(parentBody) {
      pcl_[i] = parentBody->getIndex();
    } else {
      std::cerr << "Error: failed to find parent body of joint: "
                << joint->getName() << std::endl;
      assert(1);
    }
  }

  // std::cout << "pcl: ";
  // for(auto i : pcl_) {
  //   std::cout << i << " ";
  // }
  // std::cout << std::endl;

  // std::cout << "parent list: " << std::endl;
  // for(unsigned int i = 0; i < pcl_.size(); i++) {
  //   int parentIndex = pcl_[i];
  //
  //   std::cout << "body[" << i << "]: " << activeBodies_[i]->getName()
  //             << ", parent: ";
  //   if(parentIndex < 0) {
  //     std::cout << "fixed";
  //   } else {
  //     std::cout << activeBodies_[parentIndex]->getName();
  //   }
  //   std::cout << ", " << activeJoints_[i]->getName() << std::endl;
  // }

  // std::cout << std::endl << "# body / joint list: " << std::endl;
  // for(unsigned int i = 0; i < activeBodies_.size(); i++) {
  //   std::cout << "[" << i << "]" << activeJoints_[i]->getName();
  //   if(activeJoints_[i]->getSense() > 0) {
  //     std::cout << " +";
  //   } else {
  //     std::cout << " -";
  //   }
  //   std::cout << " " << activeBodies_[i]->getName() << std::endl;
  // }
}

void Shift::Context::processLoopMembers() {
  // std::cout << "processLoopMembers()" << std::endl;

  std::vector<std::vector<int>> loopMembers(chordConstraints_.size());
  std::vector<std::vector<int>> negativeLoopMembers(chordConstraints_.size());
  std::vector<std::vector<int>> positiveLoopMembers(chordConstraints_.size());

  // std::cout << "chordConstraints_.size(): " << chordConstraints_.size()
  //           << std::endl;

  for(unsigned int i = 0; i < chordConstraints_.size(); i++) {
    // std::cout << "processing chord constraint: " << i << std::endl;
    loopMembers[i].clear();
    negativeLoopMembers[i].clear();
    positiveLoopMembers[i].clear();

    // if(chordConstraints_[i] != NULL) {
    //   std::cout << "chordConstraint: " << i << " found" << std::endl;
    // }

    if(!chordConstraints_[i]->getParentBody()) {
      std::cerr << "no parent body for chordConstraint: " << i << std::endl;
      assert(0);
    }

    if(!chordConstraints_[i]->getChildBody()) {
      std::cerr << "no child body for chordConstraint: " << i << std::endl;
      assert(0);
    }

    int positive = chordConstraints_[i]->getParentBody()->getIndex();
    int negative = chordConstraints_[i]->getChildBody()->getIndex();

    loopMembers[i].push_back(negative);
    negativeLoopMembers[i].push_back(negative);
    loopMembers[i].push_back(positive);
    positiveLoopMembers[i].push_back(positive);

    while(activeJoints_[positive]->getLevel() >
          activeJoints_[negative]->getLevel()) {
      positive = pcl_[positive];
      loopMembers[i].push_back(positive);
      positiveLoopMembers[i].push_back(positive);
    }
    while(activeJoints_[negative]->getLevel() >
          activeJoints_[positive]->getLevel()) {
      negative = pcl_[negative];
      loopMembers[i].push_back(negative);
      negativeLoopMembers[i].push_back(negative);
    }
    while(positive != negative) {
      negative = pcl_[negative];
      positive = pcl_[positive];
      loopMembers[i].push_back(positive);
      positiveLoopMembers[i].push_back(positive);
      loopMembers[i].push_back(negative);
      negativeLoopMembers[i].push_back(negative);
    }
    loopMembers[i].pop_back();
    negativeLoopMembers[i].pop_back();
    positiveLoopMembers[i].pop_back();
  }

  /* sort loop members in monotonically increasing level order */
  for(unsigned int i = 0; i < chordConstraints_.size(); i++) {
    std::sort(loopMembers[i].begin(), loopMembers[i].end(), levelOrder(this));
  }
  loopMembers_ = loopMembers;

  /* sort positive and negative loop member lists to follow master loop member
   * list ordering */
  negativeLoopMembers_.clear();
  positiveLoopMembers_.clear();
  negativeLoopMembers_.resize(chordConstraints_.size());
  positiveLoopMembers_.resize(chordConstraints_.size());
  for(unsigned int i = 0; i < chordConstraints_.size(); i++) {
    negativeLoopMembers_[i].clear();
    for(auto j : loopMembers_[i]) {
      if(std::find(negativeLoopMembers[i].begin(), negativeLoopMembers[i].end(),
                   j) != negativeLoopMembers[i].end()) {
        negativeLoopMembers_[i].push_back(j);
      }
      if(std::find(positiveLoopMembers[i].begin(), positiveLoopMembers[i].end(),
                   j) != positiveLoopMembers[i].end()) {
        positiveLoopMembers_[i].push_back(j);
      }
    }
  }
}

void Shift::Context::detectSubstructures() {
  // std::cout << "detectSubstructures()" << std::endl;
  if(chordConstraints_.size() > 0) {

    /* detect substructures */

    /* contains chord constraint indices */
    std::vector<std::vector<int>> substructures;

    /* create stack of all chordConstraint indices */
    std::list<int> chordConstraintIndices;
    for(unsigned int i = 0; i < chordConstraints_.size(); i++) {
      chordConstraintIndices.push_back(i);
    }

    // std::cout << "chord constraint indices: ";
    // for(auto i : chordConstraintIndices) {
    //   std::cout << i << " ";
    // }
    // std::cout << std::endl;

    /* p_01 is used to permute chord loops here */
    std::vector<int> p_01(chordConstraints_.size());
    std::vector<int> p_10(chordConstraints_.size());
    int currentLoopIndex = 0;

    while(!chordConstraintIndices.empty()) {

      // std::cout << std::endl
      //           << "creating substructure: " << substructures.size()
      //           << std::endl;
      std::vector<int> substructure;
      int chordConstraintIndex = chordConstraintIndices.front();
      chordConstraintIndices.pop_front();

      substructure.push_back(chordConstraintIndex);
      p_01[currentLoopIndex++] = chordConstraintIndex;

      /* append first substructure to list of substructures */
      auto it = chordConstraintIndices.begin();
      int mergeCount = 1;
      while(mergeCount != 0) {
        mergeCount = 0;
        while(it != chordConstraintIndices.end()) {
          /* base node of substructure */
          int baseNode = loopMembers_[substructure[0]][0];
          // std::cout << "baseNode: " << baseNode << std::endl;
          // std::cout << "loopHead: " << loopMembers_[(*it)][0] << std::endl;
          if(Shift::Topology::inPath(pcl_, loopMembers_[(*it)][0], baseNode)) {
            // std::cout << "  in path!" << std::endl;
            for(auto loopIndex : substructure) {
              for(auto loopMember = loopMembers_[loopIndex].begin() + 1;
                  loopMember < loopMembers_[loopIndex].end(); ++loopMember) {
                for(auto mergeCandidateMember = loopMembers_[(*it)].begin() + 1;
                    mergeCandidateMember < loopMembers_[(*it)].end();
                    ++mergeCandidateMember) {
                  if(*loopMember == *mergeCandidateMember) {
                    // std::cout << "  appending " << *it << " to substructure "
                    // << substructures.size() << std::endl;
                    substructure.push_back(*it);
                    p_01[currentLoopIndex++] = (*it);
                    mergeCount++;
                    it = chordConstraintIndices.erase(it);
                    goto endLoopMerge;
                  }
                }
              }
            }
          }
          it++;
        endLoopMerge:
          (void)0;
        }
      }
      substructures.push_back(substructure);
      // std::cout << "substructure count: " << substructures.size() <<
      // std::endl;
    }

    /* sort loops of each substructure in monotonically increasing level order
     */
    for(auto& substructure : substructures) {
      std::sort(substructure.begin(), substructure.end(),
                substructureLoopLevelOrder(this, loopMembers_));
    }

    // int index = 0;
    // for(auto& substructure : substructures) {
    //   std::cout << "[" << index++ << "]: ";
    //   for(auto i : substructure) {
    //     std::cout << i << " ";
    //   }
    //   std::cout << std::endl;
    // }

    /* sort substructures in monotonically level increasing order */
    std::sort(substructures.begin(), substructures.end(),
              substructureLevelOrder(this, loopMembers_));

    std::vector<std::vector<int>> substructureMembers(substructures.size());
    for(unsigned int i = 0; i < substructures.size(); i++) {
      for(auto loopIndex : substructures[i]) {
        for(auto loopMember : loopMembers_[loopIndex]) {
          substructureMembers[i].push_back(loopMember);
        }
      }

      /* remove duplicate entries */
      std::sort(substructureMembers[i].begin(), substructureMembers[i].end());
      substructureMembers[i].erase(
        unique(substructureMembers[i].begin(), substructureMembers[i].end()),
        substructureMembers[i].end());

      /* sort substructure members in level increasing order */
      std::sort(substructureMembers[i].begin(), substructureMembers[i].end(),
                levelOrder(this));
    }

    // std::cout << "substructures:" << std::endl;
    // unsigned int i = 0;
    // for(auto& members : substructureMembers) {
    //   std::cout << "  [" << i++ << "]: ";
    //   for(auto& member : members) {
    //     std::cout << member << " ";
    //   }
    //   std::cout << std::endl;
    // }

    /* reorder loops */

    /* reorder loop members to follow substructure ordering */
    std::vector<std::vector<int>> loopMembers;
    std::vector<std::vector<int>> negativeLoopMembers;
    std::vector<std::vector<int>> positiveLoopMembers;
    std::vector<Shift::Constraint::Chord*> chordConstraints;

    for(auto substructure : substructures) {
      for(auto loop : substructure) {
        loopMembers.push_back(loopMembers_[loop]);
        negativeLoopMembers.push_back(negativeLoopMembers_[loop]);
        positiveLoopMembers.push_back(positiveLoopMembers_[loop]);
        chordConstraints.push_back(chordConstraints_[loop]);
      }
    }

    loopMembers_ = loopMembers;
    negativeLoopMembers_ = negativeLoopMembers;
    positiveLoopMembers_ = positiveLoopMembers;
    chordConstraints_ = chordConstraints;

    // std::cout << "loop members after sorting:" << std::endl;
    // for(unsigned int i = 0; i < loopMembers.size(); i++) {
    //   std::cout << "loop members [" << i << "]: " << std::endl;
    //   for(auto member : loopMembers[i]) {
    //     std::cout << "  " << activeBodies_[member]->getName() << std::endl;
    //   }
    //   std::cout << "negative loop members [" << i << "]: " << std::endl;
    //   for(auto member : negativeLoopMembers[i]) {
    //     std::cout << "  " << activeBodies_[member]->getName() << std::endl;
    //   }
    //   std::cout << "positive loop members [" << i << "]: " << std::endl;
    //   for(auto member : positiveLoopMembers[i]) {
    //     std::cout << "  " << activeBodies_[member]->getName() << std::endl;
    //   }
    //   std::cout << std::endl;
    // }

    /* substructure body joint reorder */

    /* sort substructures in monotonically increasing level order by base node
     */

    /* reorder bodies and joints according to substructure assignments */

    /* create a short list of indices that are either not in any
       substructure or are the head of a substructure; the short list
       may contain the fixed body */
    std::list<int> shortList;
    for(int i = -1; i < (int)activeJoints_.size(); i++) {
      if(Shift::Topology::isSubstructureHead(i, substructureMembers) ||
         !Shift::Topology::isSubstructureMember(i, substructureMembers)) {
        shortList.push_back(i);
      }
    }

    // std::cout << "short list:" << std::endl;
    // for(auto member : shortList) {
    //   if(member == -1) {
    //     std::cout << "[" << member << "]: "
    //               << "fixed" << std::endl;
    //   } else {
    //     std::cout << "[" << member << "]: " <<
    //     activeBodies_[member]->getName()
    //               << std::endl;
    //   }
    // }

    /* expand short list */
    std::list<int> expandedList;
    unsigned int j = 0;

    while(!shortList.empty()) {
      int value = shortList.front();
      shortList.pop_front();

      if(std::find(expandedList.begin(), expandedList.end(), value) ==
         expandedList.end()) {
        expandedList.push_back(value);
      }

      /*
        if `value` is head of a substructure, append substructure members only
        if they are not already in the list,

        check to make sure that j is less than total number of substructures

        multiple substructures may be found at the same level, so
        increment through all substructures to find all of them
      */
      while(j < substructureMembers.size() &&
            value == substructureMembers[j][0]) {
        for(unsigned int k = 1; k < substructureMembers[j].size(); k++) {

          /* append substructure members only if they are not already in
          the list */
          if(std::find(expandedList.begin(), expandedList.end(),
                       substructureMembers[j][k]) == expandedList.end()) {
            expandedList.push_back(substructureMembers[j][k]);
          }
        }
        j++;
      }
    }

    std::vector<int> expandedArray{std::begin(expandedList),
                                   std::end(expandedList)};

    /* verify that all joints were added to the expanded array */
    bool found = true;
    for(unsigned int i = 0; i < activeJoints_.size(); i++) {
      found = false;
      for(auto j : expandedArray) {
        if((int)i == j) {
          found = true;
        }
      }
      if(!found) {
        std::cerr << " not found! " << activeBodies_[i]->getName() << std::endl;
      }
    }
    assert(found);

    /* reorder bodies, joints and levels */
    std::vector<Shift::Joint::Base*> activeJoints(activeJoints_.size());
    std::vector<Shift::Body::Base*> activeBodies(activeBodies_.size());

    /* initialize permutation arrays to permute bodies and joints from order
     * originally loaded (p0) to order dictated by kinematic substructuring
     * algorithm */
    p_01.clear();
    p_01.resize(activeJoints_.size());
    p_10.clear();
    p_10.resize(activeJoints_.size());

    j = 0;
    for(unsigned int i = 0; i < expandedArray.size(); i++) {
      if(expandedArray[i] != -1) {
        p_01[j] = expandedArray[i];
        activeJoints[j] = activeJoints_[expandedArray[i]];
        activeBodies[j] = activeBodies_[expandedArray[i]];
        activeBodies[j]->setIndex(j);
        j++;
      }
    }
    activeJoints_ = activeJoints;
    activeBodies_ = activeBodies;

    for(unsigned int i = 0; i < activeJoints.size(); i++) {
      p_10[p_01[i]] = i;
    }

    for(auto& loop : loopMembers_) {
      for(auto& i : loop) {
        i = p_10[i];
      }
    }

    for(auto& loop : negativeLoopMembers_) {
      for(auto& i : loop) {
        i = p_10[i];
      }
    }

    for(auto& loop : positiveLoopMembers_) {
      for(auto& i : loop) {
        i = p_10[i];
      }
    }
  }
}
