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
#include "shift-core/joint/rigid.h"
#include "shift-core/light/base.h"
#include "shift-core/logger/base.h"
#include "shift-core/sensor/base.h"
#include "shift-core/solver/base.h"
#include "shift-core/terrain/base.h"
#include "shift-core/topology.h"

#include "ksl/print.h"
#include "ksl/screw.h"
#include "ksl/trig.h"

void Shift::Context::readJSON(const std::string& filename) {
  std::ifstream file(filename);
  file >> root_;
  if(root_.isMember("model_parameters")) {
    modelParameters_.readJSON(root_["model_parameters"]);
  }
  fixed_.setContext(this);
  fixed_.readJSON(root_, true);
  setFullyQualifiedNames();

  /* process topology */
  setActiveObjects();
  connect();
  levelOrderSort();
  processVirtualBodies();
  processPCL();
  processLoopMembers();
  detectSubstructures();
  processPCL();
  processFrames();
  allocateRunTimeArrays();

  setFixedFrameGravityVector();
  setToInitialConditions();

  updateD();
  updateDH();
  updateDHVA();

  std::cout << "dqCount after updateDH: " << dqCount_ << std::endl;
  for(unsigned int i = 0; i < dqCount_; i++) {
    std::cout << "[" << i << "]: ";
    ksl_screw_print(stdout, dhva_.h + i);
  }

  if(substructureIndices_.size() > 0) {
    jacobian_.initialize();
  }

  // setDReferenceFrames();
  // initializeGLReferenceFrames();

  loadData();

  // initializeJointForces();
  // flattenForceModules();
}

void Shift::Context::processFrames() {

  /* process active frames */
  activeFrames_.clear();
  auto fixed = fixedBody();
  for(auto& frame : fixed->frames()) {
    activeFrames_.push_back((Shift::Frame*)&frame);
  }
  for(auto& body : activeArticulatedBodies_) {
    for(auto& frame : body->frames()) {
      if(frame.isActive()) {
        activeFrames_.push_back((Shift::Frame*)&frame);
      }
    }
  }

  /* identify frames in spanning tree */
  framesInTree_.clear();
  fixedBody()->clearInTreeFlags();
  for(auto& body : activeArticulatedBodies_) {
    body->clearInTreeFlags();
  }
  for(auto& frame : fixed->frames()) {
    framesInTree_.push_back((Shift::Frame*)&frame);
  }

  /* identify frames attached to non-rigid, active joints */
  for(auto& joint : activeJoints_) {
    /* parent and child frames of a joint are only added to spanning tree if
     * joint has nonzero degrees of freedom. Rigid joints
     * are optimized out of the spanning tree */
    if(joint->qCount()) {
      auto parentFrame = joint->systemParentFrame();
      assert(parentFrame);
      if(std::find(framesInTree_.begin(), framesInTree_.end(), parentFrame) ==
         framesInTree_.end()) {
        // std::cout << "pushing frame: "
        //           << joint->systemParentBody()->name() << " : "
        //           << parentFrame->name() << std::endl;
        framesInTree_.push_back(parentFrame);
      }
      auto childFrame = joint->systemChildFrame();
      if(std::find(framesInTree_.begin(), framesInTree_.end(), childFrame) ==
         framesInTree_.end()) {
        // std::cout << "pushing frame: " <<
        // joint->systemChildBody()->name()
        //           << " : " << childFrame->name() << std::endl;
        framesInTree_.push_back(childFrame);
      }
    }
  }

  /* get frames associated with chord constraints */
  for(auto& constraint : chordConstraints_) {
    if(std::find(framesInTree_.begin(), framesInTree_.end(),
                 constraint->parentFrame()) == framesInTree_.end()) {
      assert(constraint->parentFrame());
      framesInTree_.push_back(constraint->parentFrame());
    }
  }

  unsigned int i = 0;
  for(auto& frame : framesInTree_) {
    frame->setIndex(i++);
    frame->setInTree(true);
  }

  std::cout << "setting reference frame on fixed body" << std::endl;
  fixedBody()->setReferenceFrame();
  for(auto& joint : activeJoints_) {
    auto body = joint->systemChildBody();
    body->setParentJoint(joint);
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
    if(body->isArticulated()) {
      for(auto& i : activeArticulatedBodies_) {
        if(i == body) {
          return;
        }
      }
      activeArticulatedBodies_.push_back(body);
    } else {
      for(auto& i : activeFreeBodies_) {
        if(i == body) {
          return;
        }
      }
      activeFreeBodies_.push_back(body);
    }
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
  if(body->isArticulated()) {
    for(unsigned int i = 0; i < activeArticulatedBodies_.size(); i++) {
      if(activeArticulatedBodies_[i] == body) {
        activeArticulatedBodies_.erase(activeArticulatedBodies_.begin() + i);
        return;
      }
    }
  } else {
    for(unsigned int i = 0; i < activeFreeBodies_.size(); i++) {
      if(activeFreeBodies_[i] == body) {
        activeFreeBodies_.erase(activeFreeBodies_.begin() + i);
        return;
      }
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

void Shift::Context::clearActiveArticulatedBodies() {
  activeArticulatedBodies_.clear();
}

void Shift::Context::clearActiveFreeBodies() {
  activeFreeBodies_.clear();
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
  clearActiveArticulatedBodies();
  clearActiveFreeBodies();
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
  for(auto& behavior : activeBehaviors_) {
    if(Shift::Topology::matchFullyQualifiedName(behavior->fullyQualifiedName(),
                                                name)) {
      return behavior;
    }
  }
  return NULL;
}

Shift::Body::Base* Shift::Context::getBody(std::vector<std::string>& name) {
  for(auto& body : activeArticulatedBodies_) {

    if(body && Shift::Topology::matchFullyQualifiedName(
                 body->fullyQualifiedName(), name)) {
      return body;
    }
  }
  return NULL;
}

Shift::CollisionGeometry::Base*
Shift::Context::getCollisionGeometry(std::vector<std::string>& name) {
  for(auto& geometry : activeCollisionGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(geometry->fullyQualifiedName(),
                                                name)) {

      return geometry;
    }
  }
  return NULL;
}

Shift::Constraint::Base*
Shift::Context::getConstraint(std::vector<std::string>& name) {
  for(auto& constraint : activeConstraints_) {
    if(Shift::Topology::matchFullyQualifiedName(
         constraint->fullyQualifiedName(), name)) {
      return constraint;
    }
  }
  return NULL;
}

Shift::ForceModule::Base*
Shift::Context::getForceModule(std::vector<std::string>& name) {
  for(auto& module : activeForceModules_) {
    if(Shift::Topology::matchFullyQualifiedName(module->fullyQualifiedName(),
                                                name)) {
      return module;
    }
  }
  return NULL;
}

Shift::Geometry::Base*
Shift::Context::getGeometry(std::vector<std::string>& name) {
  for(auto& geometry : activeGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(geometry->fullyQualifiedName(),
                                                name)) {
      return geometry;
    }
  }
  return NULL;
}

Shift::Joint::Base* Shift::Context::getJoint(std::vector<std::string>& name) {
  for(auto& joint : activeJoints_) {
    if(Shift::Topology::matchFullyQualifiedName(joint->fullyQualifiedName(),
                                                name)) {
      return joint;
    }
  }
  return NULL;
}

Shift::Light::Base* Shift::Context::getLight(std::vector<std::string>& name) {
  for(auto& light : activeLights_) {
    if(Shift::Topology::matchFullyQualifiedName(light->fullyQualifiedName(),
                                                name)) {
      return light;
    }
  }
  return NULL;
}

Shift::Sensor::Base* Shift::Context::getSensor(std::vector<std::string>& name) {
  for(auto& sensor : activeSensors_) {
    if(Shift::Topology::matchFullyQualifiedName(sensor->fullyQualifiedName(),
                                                name)) {
      return sensor;
    }
  }
  return NULL;
}

Shift::Terrain::Base*
Shift::Context::getTerrain(std::vector<std::string>& name) {
  for(auto& terrain : activeTerrain_) {
    if(Shift::Topology::matchFullyQualifiedName(terrain->fullyQualifiedName(),
                                                name)) {
      return terrain;
    }
  }
  return NULL;
}

void Shift::Context::connect() {
  for(auto& behavior : activeBehaviors_) {
    behavior->connect();
  }
  for(auto& body : activeArticulatedBodies_) {
    body->connect();
  }
  for(auto& geometry : activeCollisionGeometry_) {
    geometry->connect();
  }
  for(auto& constraint : activeConstraints_) {
    constraint->connect();
  }
  for(auto& module : activeForceModules_) {
    module->connect();
  }
  for(auto& geometry : activeGeometry_) {
    geometry->connect();
  }
  for(auto& joint : activeJoints_) {
    joint->connect();
  }
  for(auto& light : activeLights_) {
    light->connect();
  }
  for(auto& sensor : activeSensors_) {
    sensor->connect();
  }
  for(auto& terrain : activeTerrain_) {
    terrain->connect();
  }
}

void Shift::Context::loadData() {
  for(auto& behavior : activeBehaviors_) {
    behavior->load();
  }
  for(auto& body : activeArticulatedBodies_) {
    body->load();
  }
  for(auto& geometry : activeCollisionGeometry_) {
    geometry->load();
  }
  for(auto& constraint : activeConstraints_) {
    constraint->load();
  }
  for(auto& module : activeForceModules_) {
    module->load();
  }
  for(auto& geometry : activeGeometry_) {
    geometry->load();
  }
  for(auto& joint : activeJoints_) {
    joint->load();
  }
  for(auto& light : activeLights_) {
    light->load();
  }
  for(auto& sensor : activeSensors_) {
    sensor->load();
  }
  for(auto& terrain : activeTerrain_) {
    terrain->load();
  }
}

Shift::Body::Base* Shift::Context::fixedBody() {
  return fixed_.getBody(0);
}

void Shift::Context::levelOrderSort() {

  std::queue<Shift::Joint::Base*> queue;
  std::vector<Shift::Joint::Base*> joints;
  std::vector<Shift::Body::Base*> bodies;

  /* find joints attached to fixed body */
  auto it = activeJoints_.begin();
  while(it != activeJoints_.end()) {
    assert((*it)->parentBody() != NULL);
    assert((*it)->childBody() != NULL);
    if((*it)->parentBody() == fixedBody()) {
      (*it)->setSense(1);
      (*it)->setLevel(1);
      queue.push(*it);
      it = activeJoints_.erase(it);
    } else if((*it)->childBody() == fixedBody()) {
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
    if(joint->sense() == 1) {
      body = joint->childBody();
    } else if(joint->sense() == -1) {
      body = joint->parentBody();
    }
    joints.push_back(joint);
    assert(body);

    /* find any joints attached to joint's child body */
    it = activeJoints_.begin();
    while(it != activeJoints_.end()) {
      if((*it)->parentBody() == body) {
        (*it)->setSense(1);
        (*it)->setLevel(joint->getLevel() + 1);
        queue.push(*it);
        it = activeJoints_.erase(it);
      } else if((*it)->childBody() == body) {
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

  // clear all system child body and frame pointers
  for(auto& joint : activeJoints_) {
    joint->setSystemChildBody(NULL);
    joint->setSystemChildFrame(NULL);
  }

  chordConstraints_.clear();
  int j = 0;
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {

    bool found = false;
    auto body = activeJoints_[i]->systemChildBody();
    for(auto it = activeArticulatedBodies_.begin();
        it < activeArticulatedBodies_.end(); it++) {
      if(body == (*it)) {
        activeArticulatedBodies_.erase(it);
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
      chordConstraint->setParentFrame(virtualBody->origin());

      auto chordJoint = activeJoints_[i];
      chordConstraint->setChildBody(chordJoint->systemChildBody());
      chordConstraint->setChildFrame(chordJoint->systemChildFrame());
      chordConstraint->setOriginalParentBody(chordJoint->systemParentBody());
      chordConstraint->setOriginalParentFrame(chordJoint->systemParentFrame());

      /* add virtual bodies and chord constraints to active lists */
      bodies[i] = virtualBody.get();

      activeConstraints_.push_back(constraint.get());
      chordConstraints_.push_back(chordConstraint);

      activeJoints_[i]->setSystemChildBody(virtualBody.get());
      activeJoints_[i]->setSystemChildFrame(virtualBody.get()->origin());

      /* transfer ownership of smart pointers for virtual bodies and chord
       * constraints to `Context` data structure */
      bodies_.push_back(std::move(virtualBody));
      constraints_.push_back(std::move(constraint));
    }
  }

  activeArticulatedBodies_ = bodies;
}

void Shift::Context::processPCL() {
  for(unsigned int i = 0; i < activeArticulatedBodies_.size(); i++) {
    activeArticulatedBodies_[i]->setIndex(i);
  }
  pcl_.clear();
  pcl_.resize(activeJoints_.size());
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {
    auto joint = activeJoints_[i];
    auto parentBody = joint->systemParentBody();
    if(parentBody) {
      pcl_[i] = parentBody->index();
    } else {
      std::cerr << "Error: failed to find parent body of joint: "
                << joint->name() << std::endl;
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
  //   std::cout << "body[" << i << "]: " << activeBodies_[i]->name()
  //             << ", parent: ";
  //   if(parentIndex < 0) {
  //     std::cout << "fixed";
  //   } else {
  //     std::cout << activeBodies_[parentIndex]->name();
  //   }
  //   std::cout << ", " << activeJoints_[i]->name() << std::endl;
  // }

  // std::cout << std::endl << "# body / joint list: " << std::endl;
  // for(unsigned int i = 0; i < activeBodies_.size(); i++) {
  //   std::cout << "[" << i << "]" << activeJoints_[i]->name();
  //   if(activeJoints_[i]->sense() > 0) {
  //     std::cout << " +";
  //   } else {
  //     std::cout << " -";
  //   }
  //   std::cout << " " << activeBodies_[i]->name() << std::endl;
  // }
}

void Shift::Context::processLoopMembers() {
  // std::cout << "processLoopMembers()" << std::endl;

  std::vector<std::vector<int>> loopMembers(chordConstraints_.size());
  std::vector<std::vector<int>> negativeLoopMembers(chordConstraints_.size());
  std::vector<std::vector<int>> positiveLoopMembers(chordConstraints_.size());

  for(unsigned int i = 0; i < chordConstraints_.size(); i++) {
    loopMembers[i].clear();
    negativeLoopMembers[i].clear();
    positiveLoopMembers[i].clear();

    if(!chordConstraints_[i]->parentBody()) {
      std::cerr << "no parent body for chordConstraint: " << i << std::endl;
      assert(0);
    }

    if(!chordConstraints_[i]->childBody()) {
      std::cerr << "no child body for chordConstraint: " << i << std::endl;
      assert(0);
    }

    int positive = chordConstraints_[i]->parentBody()->index();
    int negative = chordConstraints_[i]->childBody()->index();

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

    int k = 0;
    for(auto substructure : substructures) {
      std::cout << "substructure [" << k++ << "]: ";
      for(auto index : substructure) {
        std::cout << index << " ";
      }
      std::cout << std::endl;
    }
    substructureIndices_ = substructures;

    substructureMembers_.clear();
    substructureMembers_.resize(substructures.size());
    for(unsigned int i = 0; i < substructures.size(); i++) {
      for(auto loopIndex : substructures[i]) {
        for(auto loopMember : loopMembers_[loopIndex]) {
          substructureMembers_[i].push_back(loopMember);
        }
      }

      /* remove duplicate entries */
      std::sort(substructureMembers_[i].begin(), substructureMembers_[i].end());
      substructureMembers_[i].erase(
        unique(substructureMembers_[i].begin(), substructureMembers_[i].end()),
        substructureMembers_[i].end());

      /* sort substructure members in level increasing order */
      std::sort(substructureMembers_[i].begin(), substructureMembers_[i].end(),
                levelOrder(this));
    }

    std::cout << "substructure members:" << std::endl;
    unsigned int i = 0;
    for(auto& members : substructureMembers_) {
      std::cout << "[" << i++ << "]: " << std::endl;
      for(auto& member : members) {
        std::cout << "  " << activeJoints_[member]->name() << std::endl;
      }
      std::cout << std::endl;
    }

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

    std::cout << "loop members:" << std::endl;
    for(unsigned int i = 0; i < loopMembers.size(); i++) {
      std::cout << "loop members [" << i << "]: " << std::endl;
      for(auto member : loopMembers[i]) {
        std::cout << "  " << activeJoints_[member]->name() << std::endl;
      }
      std::cout << "negative loop members [" << i << "]: " << std::endl;
      for(auto member : negativeLoopMembers[i]) {
        std::cout << "  " << activeJoints_[member]->name() << std::endl;
      }
      std::cout << "positive loop members [" << i << "]: " << std::endl;
      for(auto member : positiveLoopMembers[i]) {
        std::cout << "  " << activeJoints_[member]->name() << std::endl;
      }
      std::cout << std::endl;
    }

    /* substructure body joint reorder */

    /* sort substructures in monotonically increasing level order by base node
     */

    /* reorder bodies and joints according to substructure assignments */

    /* create a short list of indices that are either not in any
       substructure or are the head of a substructure; the short list
       may contain the fixed body */
    std::list<int> shortList;
    for(int i = -1; i < (int)activeJoints_.size(); i++) {
      if(Shift::Topology::isSubstructureHead(i, substructureMembers_) ||
         !Shift::Topology::isSubstructureMember(i, substructureMembers_)) {
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
    //     activeBodies_[member]->name()
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
      while(j < substructureMembers_.size() &&
            value == substructureMembers_[j][0]) {
        for(unsigned int k = 1; k < substructureMembers_[j].size(); k++) {

          /* append substructure members only if they are not already in
          the list */
          if(std::find(expandedList.begin(), expandedList.end(),
                       substructureMembers_[j][k]) == expandedList.end()) {
            expandedList.push_back(substructureMembers_[j][k]);
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
        std::cerr << " not found! " << activeArticulatedBodies_[i]->name()
                  << std::endl;
      }
    }
    assert(found);

    /* reorder bodies, joints and levels */
    std::vector<Shift::Joint::Base*> activeJoints(activeJoints_.size());
    std::vector<Shift::Body::Base*> activeBodies(
      activeArticulatedBodies_.size());

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
        activeBodies[j] = activeArticulatedBodies_[expandedArray[i]];
        activeBodies[j]->setIndex(j);
        j++;
      }
    }
    activeJoints_ = activeJoints;
    activeArticulatedBodies_ = activeBodies;

    Shift::Topology::setInversePermutation(p_01, p_10);

    i = 0;
    for(auto& members : substructureMembers_) {
      std::cout << "  [" << i++ << "]: ";
      for(auto& member : members) {
        std::cout << member << " ";
      }
      std::cout << std::endl;
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

    for(auto& substructure : substructureMembers_) {
      for(auto& i : substructure) {
        i = p_10[i];
      }
    }
  }

  std::cout << std::endl << "substructures after sort:" << std::endl;
  unsigned int i = 0;
  for(auto& members : substructureMembers_) {
    std::cout << "  [" << i++ << "]: ";
    for(auto& member : members) {
      std::cout << member << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "substructure members:" << std::endl;
  i = 0;
  for(auto& members : substructureMembers_) {
    std::cout << "[" << i++ << "]: " << std::endl;
    for(auto& member : members) {
      std::cout << "  " << activeJoints_[member]->name() << std::endl;
    }
    std::cout << std::endl;
  }

  std::cout << "loop members:" << std::endl;
  for(unsigned int i = 0; i < loopMembers_.size(); i++) {
    std::cout << "loop members [" << i << "]: " << std::endl;
    for(auto member : loopMembers_[i]) {
      std::cout << "  " << activeJoints_[member]->name() << std::endl;
    }
    std::cout << "negative loop members [" << i << "]: " << std::endl;
    for(auto member : negativeLoopMembers_[i]) {
      std::cout << "  " << activeJoints_[member]->name() << std::endl;
    }
    std::cout << "positive loop members [" << i << "]: " << std::endl;
    for(auto member : positiveLoopMembers_[i]) {
      std::cout << "  " << activeJoints_[member]->name() << std::endl;
    }
    std::cout << std::endl;
  }
}

unsigned int Shift::Context::qCount() {
  return qCount_;
}

unsigned int Shift::Context::dqCount() {
  return dqCount_;
}

/*!
\brief compress any frames on child body attached to parent body by
series of one or more rigid joints
*/
void Shift::Context::compressRigidJoints(
  int i, // current body Index and parent joint index
  unsigned int& operationCount) {

  ksl_SE3_t dWRTReference;
  ksl_SE3_setIdentity(&dWRTReference);

  int j = i;
  auto parentJoint = activeJoints_[i];
  // bool rigidConnectionToFixedFrame = false;

  while(!parentJoint->qCount()) { // rigid joint

    std::cout << "rigid joint found at index: [" << j << "]" << std::endl;

    // dWRTReference = dJoint * dWRTReference
    ksl_SE3_t temp;
    ksl_SE3_t transform = parentJoint->transform();

    ksl_product_dd(&transform, &dWRTReference, &temp);
    ksl_SE3_copy(&temp, &dWRTReference);

    // dWRTReference = dWRTReference_parent * dWRTReference
    // std::cout << parentJoint->systemParentFrame()->name() << std::endl;
    // ksl_SE3_print(stdout, parentJoint->systemParentFrame()->dWRTReference());

    ksl_product_dd(parentJoint->systemParentFrame()->dWRTReference(),
                   &dWRTReference, &temp);
    ksl_SE3_copy(&temp, &dWRTReference);

    j = pcl_[j];
    if(j == -1) {
      // rigidConnectionToFixedFrame = true;
      break;
    }
    parentJoint = activeJoints_[j];
  }

  auto joint = activeJoints_[i];
  if(!joint->qCount()) {
    std::cout << "processing joint: " << joint->name() << std::endl;
    auto parentBody = joint->systemParentBody();
    auto childBody = activeArticulatedBodies_[i];

    ksl_SE3_print(stdout, &dWRTReference, "dWRTReference_parent: ");

    for(auto& frame : childBody->frames()) {
      std::cout << childBody->name() << ": " << frame.name() << std::endl;

      // dWRTReference = dWRTReference * dWRTReference_child
      ksl_SE3_t temp;
      ksl_product_dd(&dWRTReference, frame.dWRTReference(), &temp);

      ksl_SE3_print(stdout, &dWRTReference, "dWRTReference_frame: ");

      // frame.setSystemDWRTReference(temp);
      if(frame.inTree() && parentBody != fixedBody()) {
        std::cout << "intree && parentBody!= fixedBody: processing frame: "
                  << childBody->name() << ": " << frame.name() << std::endl;
        ksl_SE3_copy(&temp, dWRTReference_ + frame.index());

        dhvaOffsets_[operationCount].parent.d =
          parentJoint->parentBody()->referenceFrame()->index();
        dhvaOffsets_[operationCount].child.d = frame.index();

        d_fptr_[operationCount] = Shift::Joint::rigid_dh;
        dh_fptr_[operationCount] = Shift::Joint::rigid_dh;
        dhva_fptr_[operationCount] = Shift::Joint::rigid_dhva;
        operationCount++;
      } else if(frame.inTree()) {
        std::cout << "processing frame: " << childBody->name() << ": "
                  << frame.name() << std::endl;

        ksl_SE3_print(stdout, frame.dWRTReference());
        ksl_SE3_copy(&temp, dWRTReference_ + frame.index());
        ksl_product_dd(dhva_.d +
                         parentJoint->parentBody()->referenceFrame()->index(),
                       &temp, dhva_.d + frame.index());
        ksl_SE3_print(stdout, dhva_.d + frame.index());
      }
    }
  }
}

void Shift::Context::allocateRunTimeArrays() {
  unsigned int dcCount = 0;
  qCount_ = 0;
  dqCount_ = 0;
  unsigned int dataCount = 0;
  nonRigidJointCount_ = 0;
  for(auto& joint : activeJoints_) {
    dcCount += joint->dcCount();
    qCount_ += joint->qCount();
    dqCount_ += joint->dqCount();
    dataCount += joint->dataCount();
    if(joint->qCount() > 0) {
      nonRigidJointCount_++;
    }
  }

  dhva_ = Shift::Topology::DHVA(framesInTree_.size(), dcCount, dqCount_,
                                nonRigidJointCount_);

  std::cout << "qCount: " << qCount_ << std::endl;
  std::cout << "dqCount: " << dqCount_ << std::endl;
  std::cout << "dcCount: " << dcCount << std::endl;

  std::cout << "dWRTReference_.size(): " << framesInTree_.size() << std::endl;
  if(dWRTReference_) {
    free(dWRTReference_);
    dWRTReference_ = NULL;
  }
  dWRTReference_ = ksl_SE3_alloc(framesInTree_.size());

  dReferenceFrames_.clear();
  dReferenceFrames_.resize(nonRigidJointCount_);

  dcq_.clear();
  dcq_.resize(dcCount);

  if(jointData_) {
    delete jointData_;
    jointData_ = NULL;
  }
  jointData_ = new double[dataCount];

  if(q_) {
    delete[] q_;
    q_ = NULL;
    dq_ = NULL;
  }
  q_ = new double[qCount_ + dqCount_]{};
  dq_ = q_ + qCount_;

  if(deltaq_) {
    delete[] deltaq_;
  }
  deltaq_ = new double[dqCount_];

  if(ddq_) {
    delete ddq_;
    ddq_ = NULL;
  }
  ddq_ = new double[dqCount_];
  memset(ddq_, 0, dqCount_ * sizeof(double));

  unsigned int i = 0;
  for(auto& body : activeArticulatedBodies_) {
    if(body->parentJoint() && body->parentJoint()->qCount() > 0) {
      body->setVAIndex(i++);
    } else {
      body->setVAIndex(-1);
    }
  }

  // get operations count
  unsigned int operationCount = 0;
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {
    int j = i;
    auto joint = activeJoints_[j];
    auto parentJoint = joint;
    bool rigidConnectionToFixedFrame = false;
    while(!parentJoint->qCount()) { // rigid joint
      j = pcl_[j];
      if(j == -1) {
        std::cout << "rigid connection to fixed frame found: ";
        std::cout << joint->name() << std::endl;
        rigidConnectionToFixedFrame = true;
        break;
      }
      parentJoint = activeJoints_[j];
    }

    if(!rigidConnectionToFixedFrame) {
      auto body = joint->systemChildBody();
      operationCount += body->frameInTreeCount();
      std::cout << body->name() << ": " << body->frameInTreeCount()
                << std::endl;
    }
  }

  std::cout << "operationCount: " << operationCount << std::endl;

  dhvaOffsets_.clear();
  dhvaOffsets_.resize(operationCount);

  d_fptr_.clear();
  d_fptr_.resize(operationCount);

  dh_fptr_.clear();
  dh_fptr_.resize(operationCount);

  dhva_fptr_.clear();
  dhva_fptr_.resize(operationCount);

  int qOffset = 0;
  int dqOffset = 0;
  int dcOffset = 0;
  int dataOffset = 0;
  for(auto& joint : activeJoints_) {
    joint->setqIndex(qOffset);
    joint->setdqIndex(dqOffset);
    joint->setdcIndex(dcOffset);
    joint->setDataIndex(dataOffset);
    qOffset += joint->qCount();
    dqOffset += joint->dqCount();
    dcOffset += joint->dcCount();
    dataOffset += joint->dataCount();
  }

  /* one time update of frames on fixed body */
  std::cout << "updating frames on fixed body" << std::endl;
  for(auto& frame : fixedBody()->frames()) {
    if(!frame.isReference()) {
      assert(frame.parentBody());
      std::cout << "[" << frame.index() << "]: " << frame.parentBody()->name()
                << ":" << frame.name() << std::endl;
      ksl_SE3_copy(frame.dWRTReference(), dhva_.d + frame.index());
      ksl_SE3_copy(frame.dWRTReference(), dWRTReference_ + frame.index());
      ksl_SE3_print(stdout, dhva_.d + frame.index());
    }
  }

  operationCount = 0;
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {
    auto& joint = activeJoints_[i];
    auto parentBody = joint->systemParentBody();
    auto parentFrame = joint->systemParentFrame();
    auto childBody = activeArticulatedBodies_[i];
    auto childFrame = childBody->referenceFrame();

    if(joint->qCount()) {
      if(joint->dcCount()) {
        dcq_[joint->dcIndex()] = joint->qIndex();
        dhvaOffsets_[operationCount].dc = joint->dcIndex();
      } else {
        dhvaOffsets_[operationCount].q = joint->qIndex();
      }
      dhvaOffsets_[operationCount].parent.d = parentFrame->index();
      dhvaOffsets_[operationCount].child.d = childFrame->index();
      dhvaOffsets_[operationCount].parent.v = parentBody->VAIndex();
      dhvaOffsets_[operationCount].child.v = childBody->VAIndex();
      dhvaOffsets_[operationCount].dq = joint->dqIndex();
      dhvaOffsets_[operationCount].data = joint->dataIndex();
      d_fptr_[operationCount] = joint->Dfptr();
      dh_fptr_[operationCount] = joint->DHfptr();
      dhva_fptr_[operationCount] = joint->DHVAfptr();
      operationCount++;

      auto referenceFrame = childBody->referenceFrame();
      std::cout << "referenceFrame: " << referenceFrame->parentBody()->name()
                << ":" << referenceFrame->name() << std::endl;
      for(auto& frame : childBody->frames()) {
        /*!
          @TODO add logic to detect if transformation has zero entries
          and use appropriate sparse function evaluation
          for now, all function pointers are assigned as general SE3
          displacements
        */
        if(!frame.isReference() && frame.inTree()) {
          d_fptr_[operationCount] = Shift::Joint::rigid_dh;
          dh_fptr_[operationCount] = Shift::Joint::rigid_dh;
          dhva_fptr_[operationCount] = Shift::Joint::rigid_dhva;
          dhvaOffsets_[operationCount].parent.d = referenceFrame->index();
          dhvaOffsets_[operationCount].child.d = frame.index();
          std::cout << "  processing child frame[" << frame.index()
                    << "]: " << frame.name() << std::endl;
          ksl_SE3_print(stdout, frame.dWRTReference(), "  dWRTReference: ");
          ksl_SE3_copy(frame.dWRTReference(), dWRTReference_ + frame.index());
          operationCount++;
        }
      }
    } else {
      compressRigidJoints(i, operationCount);
    }
  }

  std::cout << "operationCount: " << operationCount << std::endl;
  jointDOF_.resize(activeJoints_.size());
  modelParameters_.initialize();
}

void Shift::Context::updateDC(Shift::Topology::DHVA& dhva,
                              const double* __restrict__ q) {
  for(unsigned int i = 0; i < dcq_.size(); i++) {
    ksl_dc(q[dcq_[i]], dhva.dc + (i * 2));
  }
}

void Shift::Context::updateD() {
  updateD(dhva_, q_);
}

void Shift::Context::updateD(Shift::Topology::DHVA& dhva,
                             const double* __restrict__ q) {
  updateDC(dhva, q);
  for(unsigned int i = 0; i < d_fptr_.size(); i++) {
    (d_fptr_[i])(*this, dhva, dhvaOffsets_[i], q);
  }
}

void Shift::Context::updateDH() {
  updateDH(dhva_, q_);
}

void Shift::Context::updateDH(Shift::Topology::DHVA& dhva,
                              const double* __restrict__ q) {
  updateDC(dhva, q);
  std::cout << "updateDH()" << std::endl;
  for(unsigned int i = 0; i < dh_fptr_.size(); i++) {
    std::cout << "  [" << i << "] ";
    (dh_fptr_[i])(*this, dhva, dhvaOffsets_[i], q);
  }
  std::cout << std::endl;
  for(auto& frame : framesInTree_) {
    std::cout << "[" << frame->index() << "]: " << frame->parentBody()->name()
              << ": " << frame->name() << std::endl;
    ksl_SE3_print(stdout, dWRTReference_ + frame->index(),
                  "    dWRTReference: ");
    ksl_SE3_print(stdout, dhva_.d + frame->index(), "        dAbsolute: ");

    std::cout << std::endl;
  }

  // for(unsigned int i = 0; i < framesInTree_.size(); i++) {
  //   std::string name = "[" + std::to_string(i) + "]: ";
  //   ksl_SE3_print(stdout, dhva_.d + i, name.c_str());
  // }
}

void Shift::Context::updateDHVA() {
  updateDHVA(dhva_, q_, dq_);
}

void Shift::Context::updateDHVA(Shift::Topology::DHVA& dhva,
                                const double* __restrict__ q,
                                const double* __restrict__ dq) {
  updateDC(dhva, q);
  for(unsigned int i = 0; i < dhva_fptr_.size(); i++) {
    (dhva_fptr_[i])(*this, dhva, dhvaOffsets_[i], q, dq);
  }
}

void Shift::Context::setFixedFrameGravityVector() {
  assert(dhva_.aq);
  ksl_vec3_inverted(modelParameters_.gravity(), &dhva_.aq[-1].lin);
}

void Shift::Context::setToInitialConditions() {
  std::cout << "setting initial conditions:" << std::endl;
  assert(q_);
  assert(dq_);
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  for(auto& joint : activeJoints_) {
    if(joint->qCount()) {
      std::cout << joint->name() << " " << std::flush;
      assert(joint->q0());
      for(unsigned int i = 0; i < joint->qCount(); i++) {
        std::cout << joint->q0()[i] << " ";
      }
      std::cout << std::endl;
      memcpy(q_ + qOffset, joint->q0(), joint->qCount() * sizeof(double));
    }
    if(joint->dqCount()) {
      for(unsigned int i = 0; i < joint->dqCount(); i++) {
        std::cout << joint->dq0()[i] << " ";
      }
      std::cout << std::endl;
      memcpy(dq_ + dqOffset, joint->dq0(), joint->dqCount() * sizeof(double));
    }
    qOffset += joint->qCount();
    dqOffset += joint->dqCount();
  }
}

void Shift::Context::setInitialConditionsToCurrentConditions() {
  assert(q_);
  assert(dq_);
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  for(auto& joint : activeJoints_) {
    joint->setInitialDisplacement(q_ + qOffset);
    joint->setInitialVelocity(dq_ + dqOffset);
    qOffset += joint->qCount();
    dqOffset += joint->dqCount();
  }
}
