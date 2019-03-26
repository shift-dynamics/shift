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
  auto fixed = getFixedBody();
  for(auto& frame : fixed->getFrames()) {
    activeFrames_.push_back((Shift::Frame*)&frame);
  }
  for(auto& body : activeArticulatedBodies_) {
    for(auto& frame : body->getFrames()) {
      if(frame.isActive()) {
        activeFrames_.push_back((Shift::Frame*)&frame);
      }
    }
  }

  /* identify frames in spanning tree */
  framesInTree_.clear();
  getFixedBody()->clearInTreeFlags();
  for(auto& body : activeArticulatedBodies_) {
    body->clearInTreeFlags();
  }
  for(auto& frame : fixed->getFrames()) {
    framesInTree_.push_back((Shift::Frame*)&frame);
  }

  /* identify frames attached to non-rigid, active joints */
  for(auto& joint : activeJoints_) {
    /* parent and child frames of a joint are only added to spanning tree if
     * joint has nonzero degrees of freedom. Rigid joints
     * are optimized out of the spanning tree */
    if(joint->getqCount() > 0) {
      auto parentFrame = joint->getSystemParentFrame();
      assert(parentFrame);
      if(std::find(framesInTree_.begin(), framesInTree_.end(), parentFrame) ==
         framesInTree_.end()) {
        // std::cout << "pushing frame: "
        //           << joint->getSystemParentBody()->getName() << " : "
        //           << parentFrame->getName() << std::endl;
        framesInTree_.push_back(parentFrame);
      }
      auto childFrame = joint->getSystemChildFrame();
      if(std::find(framesInTree_.begin(), framesInTree_.end(), childFrame) ==
         framesInTree_.end()) {
        // std::cout << "pushing frame: " <<
        // joint->getSystemChildBody()->getName()
        //           << " : " << childFrame->getName() << std::endl;
        framesInTree_.push_back(childFrame);
      }
    }
  }

  /* get frames associated with chord constraints */
  for(auto& constraint : chordConstraints_) {
    if(std::find(framesInTree_.begin(), framesInTree_.end(),
                 constraint->getParentFrame()) == framesInTree_.end()) {
      assert(constraint->getParentFrame());
      framesInTree_.push_back(constraint->getParentFrame());
    }
  }

  unsigned int i = 0;
  for(auto& frame : framesInTree_) {
    frame->setIndex(i++);
    frame->setInTree(true);
  }

  getFixedBody()->setReferenceFrame();

  /* set reference frames */
  for(auto& joint : activeJoints_) {
    auto body = joint->getSystemChildBody();
    body->setParentJoint(joint);
  }

  /* compress rigid joints */
  std::cout << "compressing rigid joints" << std::endl;
  for(int i = (int)activeJoints_.size() - 1; i >= 0; i--) {
    auto joint = activeJoints_[i];
    assert(joint->getSystemChildBody());
    auto childBody = joint->getSystemChildBody();
    if(joint->getqCount() == 0) {
      // A0 : upstream parent body, parent frame
      // B0 : parent body, parent frame
      // B1 : parent Body, child frame
      // C0 : child body, parent frame
      // C1 : child body, child frame

      ksl_SE3_t d_B0_C0;
      ksl_SE3_setIdentity(&d_B0_C0);
      int j = i;
      Shift::Frame* referenceFrame = childBody->getReferenceFrame();
      std::cout << "processing frame: " << referenceFrame->getName()
                << std::endl;
      while(j > -1 && joint && joint->getqCount() == 0) {
        if(!joint->getSystemParentBody()) {
          std::cerr << "parent body not found for joint " << joint->getName()
                    << "!" << std::endl;
          assert(0);
        }
        std::cout << "  parent body: "
                  << joint->getSystemParentBody()->getName() << std::endl;
        referenceFrame = joint->getSystemParentBody()->getReferenceFrame();
        std::cout << "  reference frame: " << referenceFrame->getName()
                  << std::endl;
        if(!referenceFrame) {
          std::cerr << "reference frame not found on body: "
                    << childBody->getName() << "!" << std::endl;
          assert(0);
        }

        ksl_SE3_t d_A1_B0 = joint->getTransform();
        ksl_SE3_print(stdout, &d_A1_B0, "  d_A1_B0: ");
        assert(joint->getSystemParentFrame());
        std::cout << "  parent frame: "
                  << joint->getSystemParentFrame()->getName() << std::endl;
        const ksl_SE3_t* d_A0_A1 =
          joint->getSystemParentFrame()->getDWRTReference();
        ksl_SE3_print(stdout, d_A0_A1, "  d_A0_A1: ");

        ksl_SE3_t d_A0_B0;
        ksl_product_dd(d_A0_A1, &d_A1_B0, &d_A0_B0);

        ksl_SE3_t temp;
        ksl_product_dd(&d_A0_B0, &d_B0_C0, &temp);
        ksl_SE3_print(stdout, &temp, "  temp: ");

        ksl_SE3_copy(&temp, &d_B0_C0);
        ksl_SE3_print(stdout, &d_B0_C0, "  d_B0_C0: ");

        ksl_SE3_print(stdout, &temp);

        std::cout << "temp: " << std::endl;
        ksl_SE3_print(stdout, &d_B0_C0);

        j = pcl_[j];
        if(j < 0) {
          joint = NULL;
        } else {
          joint = activeJoints_[j];
        }
      }
      childBody->setDWRTReference(d_B0_C0);
      childBody->setReferenceFrame(referenceFrame);
    } else {
      childBody->setReferenceFrame();
    }
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
    if(body->getIsArticulated()) {
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
  if(body->getIsArticulated()) {
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
    if(Shift::Topology::matchFullyQualifiedName(
         behavior->getFullyQualifiedName(), name)) {
      return behavior;
    }
  }
  return NULL;
}

Shift::Body::Base* Shift::Context::getBody(std::vector<std::string>& name) {
  for(auto& body : activeArticulatedBodies_) {

    if(body && Shift::Topology::matchFullyQualifiedName(
                 body->getFullyQualifiedName(), name)) {
      return body;
    }
  }
  return NULL;
}

Shift::CollisionGeometry::Base*
Shift::Context::getCollisionGeometry(std::vector<std::string>& name) {
  for(auto& geometry : activeCollisionGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(
         geometry->getFullyQualifiedName(), name)) {

      return geometry;
    }
  }
  return NULL;
}

Shift::Constraint::Base*
Shift::Context::getConstraint(std::vector<std::string>& name) {
  for(auto& constraint : activeConstraints_) {
    if(Shift::Topology::matchFullyQualifiedName(
         constraint->getFullyQualifiedName(), name)) {
      return constraint;
    }
  }
  return NULL;
}

Shift::ForceModule::Base*
Shift::Context::getForceModule(std::vector<std::string>& name) {
  for(auto& module : activeForceModules_) {
    if(Shift::Topology::matchFullyQualifiedName(module->getFullyQualifiedName(),
                                                name)) {
      return module;
    }
  }
  return NULL;
}

Shift::Geometry::Base*
Shift::Context::getGeometry(std::vector<std::string>& name) {
  for(auto& geometry : activeGeometry_) {
    if(Shift::Topology::matchFullyQualifiedName(
         geometry->getFullyQualifiedName(), name)) {
      return geometry;
    }
  }
  return NULL;
}

Shift::Joint::Base* Shift::Context::getJoint(std::vector<std::string>& name) {
  for(auto& joint : activeJoints_) {
    if(Shift::Topology::matchFullyQualifiedName(joint->getFullyQualifiedName(),
                                                name)) {
      return joint;
    }
  }
  return NULL;
}

Shift::Light::Base* Shift::Context::getLight(std::vector<std::string>& name) {
  for(auto& light : activeLights_) {
    if(Shift::Topology::matchFullyQualifiedName(light->getFullyQualifiedName(),
                                                name)) {
      return light;
    }
  }
  return NULL;
}

Shift::Sensor::Base* Shift::Context::getSensor(std::vector<std::string>& name) {
  for(auto& sensor : activeSensors_) {
    if(Shift::Topology::matchFullyQualifiedName(sensor->getFullyQualifiedName(),
                                                name)) {
      return sensor;
    }
  }
  return NULL;
}

Shift::Terrain::Base*
Shift::Context::getTerrain(std::vector<std::string>& name) {
  for(auto& terrain : activeTerrain_) {
    if(Shift::Topology::matchFullyQualifiedName(
         terrain->getFullyQualifiedName(), name)) {
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

  // clear all system child body and frame pointers
  for(auto& joint : activeJoints_) {
    joint->setSystemChildBody(NULL);
    joint->setSystemChildFrame(NULL);
  }

  chordConstraints_.clear();
  int j = 0;
  for(unsigned int i = 0; i < activeJoints_.size(); i++) {

    bool found = false;
    auto body = activeJoints_[i]->getSystemChildBody();
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

      activeJoints_[i]->setSystemChildBody(virtualBody.get());
      activeJoints_[i]->setSystemChildFrame(virtualBody.get()->getOrigin());

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

  for(unsigned int i = 0; i < chordConstraints_.size(); i++) {
    loopMembers[i].clear();
    negativeLoopMembers[i].clear();
    positiveLoopMembers[i].clear();

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
      std::cout << "  [" << i++ << "]: ";
      for(auto& member : members) {
        std::cout << member << " ";
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
        std::cerr << " not found! " << activeArticulatedBodies_[i]->getName()
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

  std::cout << "substructures:" << std::endl;
  unsigned int i = 0;
  for(auto& members : substructureMembers_) {
    std::cout << "  [" << i++ << "]: ";
    for(auto& member : members) {
      std::cout << member << " ";
    }
    std::cout << std::endl;
  }
}

unsigned int Shift::Context::getqCount() {
  return qCount_;
}

unsigned int Shift::Context::getdqCount() {
  return dqCount_;
}

void Shift::Context::allocateRunTimeArrays() {
  unsigned int dcCount = 0;
  qCount_ = 0;
  dqCount_ = 0;
  unsigned int dataCount = 0;
  nonRigidJointCount_ = 0;
  for(auto& joint : activeJoints_) {
    dcCount += joint->getdcCount();
    qCount_ += joint->getqCount();
    dqCount_ += joint->getdqCount();
    dataCount += joint->getDataCount();
    if(joint->getqCount() > 0) {
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
    if(body->getParentJoint() && body->getParentJoint()->getqCount() > 0) {
      body->setVAIndex(i++);
    } else {
      body->setVAIndex(-1);
    }
  }

  // get operations count
  // int m = 0;
  // int n = 0;
  unsigned int operationCount = 0;
  for(auto& body : activeArticulatedBodies_) {
    // std::cout << "(" << m++ << "):" << body->getName();
    // std::cout << std::flush;
    // if(body->getParentJoint()) {
    //   std::cout << ": " << body->getParentJoint()->getName();
    // }
    // std::cout << std::endl;
    // for(auto& frame : body->getFrames()) {
    //   if(frame.getInTree()) {
    //     std::cout << "  (" << n++ << "): " << frame.getName() << std::endl;
    //   }
    // }
    // std::cout << "  " << body->getFrameInTreeCount() << std::endl;
    operationCount += body->getFrameInTreeCount();
  }

  dhvaOffsets_.clear();
  dhvaOffsets_.resize(operationCount);

  d_fptr_.clear();
  d_fptr_.resize(operationCount);

  dh_fptr_.clear();
  dh_fptr_.resize(operationCount);

  dhva_fptr_.clear();
  dhva_fptr_.resize(operationCount);

  int k = 0;
  i = 0;
  int j = 0;
  int dcOffset = 0;

  std::cout << "frame indices: " << std::endl;
  for(auto& frame : framesInTree_) {
    std::cout << frame->getIndex() << std::endl;
  }
  Shift::Topology::Offsets offsets = Shift::Topology::Offsets();
  for(auto& joint : activeJoints_) {
    std::cout << "(" << k++ << "/" << activeJoints_.size()
              << "): " << joint->getName() << std::endl;
    if(joint->getqCount()) {
      assert(i < operationCount);
      if(joint->getdcCount()) {
        dcq_[j++] = offsets.q;
        dhvaOffsets_[i].dc = dcOffset;
        dcOffset += joint->getdcCount();
      } else {
        dhvaOffsets_[i].q = offsets.q;
      }
      dhvaOffsets_[i].dq = offsets.dq;
      dhvaOffsets_[i].data = offsets.data;
      dhvaOffsets_[i].parent.d = joint->getSystemParentFrame()->getIndex();
      assert(dhvaOffsets_[i].parent.d >= 0);
      dhvaOffsets_[i].parent.v = joint->getSystemParentBody()->getVAIndex();
      dhvaOffsets_[i].child.d = joint->getSystemChildFrame()->getIndex();
      assert(dhvaOffsets_[i].child.d >= 0);
      dhvaOffsets_[i].child.v = joint->getSystemChildBody()->getVAIndex();

      assert(joint->getSystemChildFrame()->getInTree());
      // std::cout << "  (" << i << "/" << operationCount << ") "
      //           << joint->getSystemChildFrame()->getName() << ": ";
      // std::cout << " parent.v: " << dhvaOffsets_[i].parent.v;
      // std::cout << " parent.d: " << dhvaOffsets_[i].parent.d;
      // std::cout << " child.v: " << dhvaOffsets_[i].child.v;
      // std::cout << " child.d: " << dhvaOffsets_[i].child.d << std::endl;

      offsets.q += joint->getqCount();
      offsets.dq += joint->getdqCount();
      offsets.data += joint->getDataCount();
      std::cout << "[" << i << "]:" << std::endl;
      ksl_SE3_copy(joint->getSystemChildFrame()->getDWRTReference(),
                   dWRTReference_ + i);
      ksl_SE3_print(stdout, dWRTReference_ + i);

      d_fptr_[i] = joint->getDfptr();
      dh_fptr_[i] = joint->getDHfptr();
      dhva_fptr_[i] = joint->getDHVAfptr();
      i++;
    }

    auto childBody = joint->getSystemChildBody();
    for(auto& frame : childBody->getFrames()) {
      if(frame.getInTree() && !frame.getIsReference()) {
        assert(frame.getInTree());
        assert(i < operationCount);
        dhvaOffsets_[i].q = 0;
        dhvaOffsets_[i].dq = 0;
        dhvaOffsets_[i].data = 0;
        if(frame.getReferenceFrame()) {
          dhvaOffsets_[i].parent.d = frame.getReferenceFrame()->getIndex();
        } else {
          std::cerr << "no reference frame found for body: "
                    << childBody->getName() << " frame: " << frame.getName()
                    << "!" << std::endl;
          assert(0);
        }
        assert(dhvaOffsets_[i].parent.d >= 0);
        dhvaOffsets_[i].parent.v = 0;
        dhvaOffsets_[i].child.d = frame.getIndex();
        assert(dhvaOffsets_[i].child.d >= 0);
        dhvaOffsets_[i].child.v = 0;

        std::cout << "  (" << i << "/" << operationCount << ") "
                  << frame.getName() << ": ";
        std::cout << " parent.v: " << dhvaOffsets_[i].parent.v;
        std::cout << " parent.d: " << dhvaOffsets_[i].parent.d;
        std::cout << " child.v: " << dhvaOffsets_[i].child.v;
        std::cout << " child.d: " << dhvaOffsets_[i].child.d << std::endl;

        std::cout << frame.getName() << std::endl;
        std::cout << "dWRTReference: " << std::endl;
        // ksl_SE3_print(stdout, frame.getDWRTReference());

        std::cout << "copying...: " << std::endl;

        assert(frame.getDWRTReference());
        ksl_SE3_print(stdout, frame.getDWRTReference());

        ksl_SE3_copy(frame.getDWRTReference(),
                     dWRTReference_ + dhvaOffsets_[i].parent.d);

        std::cout << "[" << i << "]:" << std::endl;
        ksl_SE3_print(stdout, dWRTReference_ + dhvaOffsets_[i].parent.d);

        d_fptr_[i] = Shift::Joint::rigid_dh;
        dh_fptr_[i] = Shift::Joint::rigid_dh;
        dhva_fptr_[i] = Shift::Joint::rigid_dhva;
        i++;
      }
    }
  }

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
  for(unsigned int i = 0; i < framesInTree_.size(); i++) {
    std::string name = "[" + std::to_string(i) + "]: ";
    ksl_SE3_print(stdout, dhva_.d + i, name.c_str());
  }
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
  ksl_vec3_inverted(modelParameters_.getGravity(), &dhva_.aq[-1].lin);
}

void Shift::Context::setToInitialConditions() {
  std::cout << "setting initial conditions:" << std::endl;
  assert(q_);
  assert(dq_);
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  for(auto& joint : activeJoints_) {
    if(joint->getqCount()) {
      std::cout << joint->getName() << " " << std::flush;
      assert(joint->getq0());
      for(unsigned int i = 0; i < joint->getqCount(); i++) {
        std::cout << joint->getq0()[i] << " ";
      }
      std::cout << std::endl;
      memcpy(q_ + qOffset, joint->getq0(), joint->getqCount() * sizeof(double));
    }
    if(joint->getdqCount()) {
      for(unsigned int i = 0; i < joint->getdqCount(); i++) {
        std::cout << joint->getdq0()[i] << " ";
      }
      std::cout << std::endl;
      memcpy(dq_ + dqOffset, joint->getdq0(),
             joint->getdqCount() * sizeof(double));
    }
    qOffset += joint->getqCount();
    dqOffset += joint->getdqCount();
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
    qOffset += joint->getqCount();
    dqOffset += joint->getdqCount();
  }
}
