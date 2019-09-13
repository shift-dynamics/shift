#include <assert.h>
#include <fstream>
#include <iostream>
#include <list>
#include <queue>
#include <string.h>

#include <graphviz/cgraph.h>
#include <graphviz/gvc.h>

#ifdef SHIFT_WITH_BLAS_
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#endif

#include "shift-core/behavior/base.h"
#include "shift-core/body/base.h"
#include "shift-core/body/rigid.h"
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

void Shift::Context::addParameters() {

  addParameter("model_parameters", &model_parameters_,
               "Parameters associated with configuring a Shift simulation.",
               true);

  addParameter(
    "assemblies", &assemblies_,
    "an array of Shift assemblies, where each assembly contains a list of "
    "bodies, joints, force modules, behaviors, etc.; Each assembly at a given "
    "level must have a unique name identifier",
    true);
}

void Shift::Context::loadJSON(const std::string filename) {
  std::ifstream file(filename);
  file >> root_;
  try {
    readJSON(root_);
  } catch(const char* error) {
    std::cerr << "catching error in context.cc" << std::endl;
    std::cerr << error << std::endl;
  };
  setParent(this);
  postLoad(filename);
}

void Shift::Context::postLoad(const std::string) {

  /* process topology */
  setActiveObjects(true);

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
  setLookup();

  if(substructure_indices_.size() > 0) {
    jacobian_.initialize();
  }
  initializeMobility();
  initializeInertia();

  // setDReferenceFrames();
  // initializeGLReferenceFrames();

  loadData();

  // std::string graph_filename =
  //   Shift::Topology::getBasename(Shift::Topology::getFilename(filename)) +
  //   ".png";
  // digraph(graph_filename);
  //
  // graph_filename =
  //   Shift::Topology::getBasename(Shift::Topology::getFilename(filename)) +
  //   "_frames.png";
  // digraphFrames(graph_filename);

  // initializeJointForces();
  // flattenForceModules();
}

void Shift::Context::processFrames() {

  /* process active frames */
  active_frames_.clear();
  auto fixed = fixedBody();
  for(auto& frame : fixed->frames()) {
    assert(frame.get());
    // frame->print();
    active_frames_.push_back(frame.get());
  }
  for(auto& body : active_articulated_bodies_) {
    for(auto& frame : body->frames()) {
      if(frame->active()) {
        active_frames_.push_back(frame.get());
      }
    }
  }

  /* identify frames in spanning tree */
  frames_in_tree_.clear();
  fixedBody()->clearInTreeFlags();
  for(auto& body : active_articulated_bodies_) {
    body->clearInTreeFlags();
  }
  for(auto& frame : fixed->frames()) {
    frames_in_tree_.push_back(frame.get());
  }

  /* identify frames attached to non-rigid, active joints */
  for(auto& joint : active_joints_) {
    /* parent and child frames of a joint are only added to spanning tree if
     * joint has nonzero degrees of freedom. Rigid joints
     * are optimized out of the spanning tree */
    if(joint->qCount()) {
      auto parentFrame = joint->systemParentFrame();
      assert(parentFrame);
      if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(),
                   parentFrame) == frames_in_tree_.end()) {
        frames_in_tree_.push_back(parentFrame);
      }
      auto childFrame = joint->systemChildFrame();
      if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(),
                   childFrame) == frames_in_tree_.end()) {
        frames_in_tree_.push_back(childFrame);
      }
    }
  }

  /* get frames associated with chord constraints */
  for(auto& constraint : chord_constraints_) {
    if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(),
                 constraint->parentFrame()) == frames_in_tree_.end()) {
      assert(constraint->parentFrame());
      frames_in_tree_.push_back(constraint->parentFrame());
    }
  }

  // std::cout << "frames_in_tree_.size(): " << frames_in_tree_.size()
  //           << std::endl;
  unsigned int i = 0;
  for(auto& frame : frames_in_tree_) {
    assert(frame);
    // (*frame)["name"].print();
    // std::cout << "[" << i << "]: " << frame->name() << std::endl;
    frame->setIndex(i++);
    frame->setInTree(true);
  }

  // std::cout << "setting reference frame on fixed body" << std::endl;
  fixedBody()->setReferenceFrame();
  for(auto& joint : active_joints_) {
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
  if(writer != nullptr) {
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

void Shift::Context::setActiveObjects(bool) {
  model_parameters_.setActiveObjects(true);
  assemblies_.setActiveObjects(true);
}

void Shift::Context::appendActive(Shift::Integrator::Base* integrator) {
  if(integrator->active()) {
    for(auto i : active_integrators_) {
      if(i == integrator) {
        return;
      }
    }
    active_integrators_.push_back(integrator);
  }
}

void Shift::Context::appendActive(Shift::Logger::Base* logger) {
  if(logger->active()) {
    for(auto i : active_loggers_) {
      if(i == logger) {
        return;
      }
    }
    active_loggers_.push_back(logger);
  }
}

void Shift::Context::appendActive(Shift::Solver::Base* solver) {
  if(solver->active()) {
    for(auto i : active_solvers_) {
      if(i == solver) {
        return;
      }
    }
    active_solvers_.push_back(solver);
  }
}

void Shift::Context::appendActive(Shift::Behavior::Base* behavior) {
  if(behavior->active()) {
    for(auto i : active_behaviors_) {
      if(i == behavior) {
        return;
      }
    }
    active_behaviors_.push_back(behavior);
  }
}

void Shift::Context::appendActive(Shift::Body::Base* body) {
  if(body && body->active()) {
    if(body->isArticulated()) {
      for(auto& i : active_articulated_bodies_) {
        if(i == body) {
          return;
        }
      }
      active_articulated_bodies_.push_back(body);
    } else {
      for(auto& i : active_free_bodies_) {
        if(i == body) {
          return;
        }
      }
      active_free_bodies_.push_back(body);
    }
  }
}

void Shift::Context::appendActive(Shift::CollisionGeometry::Base* geometry) {
  if(geometry->active()) {
    for(auto i : active_collision_geometry_) {
      if(i == geometry) {
        return;
      }
    }
    active_collision_geometry_.push_back(geometry);
  }
}

void Shift::Context::appendActive(Shift::Constraint::Base* constraint) {
  if(constraint->active()) {
    for(auto i : active_constraints_) {
      if(i == constraint) {
        return;
      }
    }
    active_constraints_.push_back(constraint);
  }
}

void Shift::Context::appendActive(Shift::ForceModule::Base* module) {
  if(module->active()) {
    for(auto i : active_force_modules_) {
      if(i == module) {
        return;
      }
    }
    active_force_modules_.push_back(module);
  }
}

void Shift::Context::appendActive(Shift::Geometry::Base* geometry) {
  if(geometry->active()) {
    for(auto i : active_geometry_) {
      if(i == geometry) {
        return;
      }
    }
    active_geometry_.push_back(geometry);
  }
}

void Shift::Context::appendActive(Shift::Joint::Base* joint) {
  if(joint->active()) {
    for(auto i : active_joints_) {
      if(i == joint) {
        return;
      }
    }
    active_joints_.push_back(joint);
  }
}

void Shift::Context::appendActive(Shift::Light::Base* light) {
  if(light->active()) {
    for(auto i : active_lights_) {
      if(i == light) {
        return;
      }
    }
    active_lights_.push_back(light);
  }
}

void Shift::Context::appendActive(Shift::Sensor::Base* sensor) {
  if(sensor->active()) {
    for(auto i : active_sensors_) {
      if(i == sensor) {
        return;
      }
    }
    active_sensors_.push_back(sensor);
  }
}

void Shift::Context::appendActive(Shift::Terrain::Base* terrain) {
  if(terrain->active()) {
    for(auto i : active_terrain_) {
      if(i == terrain) {
        return;
      }
    }
    active_terrain_.push_back(terrain);
  }
}

void Shift::Context::removeActive(Shift::Logger::Base* logger) {
  for(unsigned int i = 0; i < active_loggers_.size(); i++) {
    if(active_loggers_[i] == logger) {
      active_loggers_.erase(active_loggers_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Integrator::Base* integrator) {
  for(unsigned int i = 0; i < active_integrators_.size(); i++) {
    if(active_integrators_[i] == integrator) {
      active_integrators_.erase(active_integrators_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Solver::Base* solver) {
  for(unsigned int i = 0; i < active_solvers_.size(); i++) {
    if(active_solvers_[i] == solver) {
      active_solvers_.erase(active_solvers_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Behavior::Base* behavior) {
  for(unsigned int i = 0; i < active_behaviors_.size(); i++) {
    if(active_behaviors_[i] == behavior) {
      active_behaviors_.erase(active_behaviors_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Body::Base* body) {
  if(body->isArticulated()) {
    for(unsigned int i = 0; i < active_articulated_bodies_.size(); i++) {
      if(active_articulated_bodies_[i] == body) {
        active_articulated_bodies_.erase(active_articulated_bodies_.begin() +
                                         i);
        return;
      }
    }
  } else {
    for(unsigned int i = 0; i < active_free_bodies_.size(); i++) {
      if(active_free_bodies_[i] == body) {
        active_free_bodies_.erase(active_free_bodies_.begin() + i);
        return;
      }
    }
  }
}

void Shift::Context::removeActive(Shift::CollisionGeometry::Base* geometry) {
  for(unsigned int i = 0; i < active_collision_geometry_.size(); i++) {
    if(active_collision_geometry_[i] == geometry) {
      active_collision_geometry_.erase(active_collision_geometry_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Constraint::Base* constraint) {
  for(unsigned int i = 0; i < active_constraints_.size(); i++) {
    if(active_constraints_[i] == constraint) {
      active_constraints_.erase(active_constraints_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::ForceModule::Base* module) {
  for(unsigned int i = 0; i < active_force_modules_.size(); i++) {
    if(active_force_modules_[i] == module) {
      active_force_modules_.erase(active_force_modules_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Geometry::Base* geometry) {
  for(unsigned int i = 0; i < active_geometry_.size(); i++) {
    if(active_geometry_[i] == geometry) {
      active_geometry_.erase(active_geometry_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Joint::Base* joint) {
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    if(active_joints_[i] == joint) {
      active_joints_.erase(active_joints_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Light::Base* light) {
  for(unsigned int i = 0; i < active_lights_.size(); i++) {
    if(active_lights_[i] == light) {
      active_lights_.erase(active_lights_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Sensor::Base* sensor) {
  for(unsigned int i = 0; i < active_sensors_.size(); i++) {
    if(active_sensors_[i] == sensor) {
      active_sensors_.erase(active_sensors_.begin() + i);
      return;
    }
  }
}

void Shift::Context::removeActive(Shift::Terrain::Base* terrain) {
  for(unsigned int i = 0; i < active_terrain_.size(); i++) {
    if(active_terrain_[i] == terrain) {
      active_terrain_.erase(active_terrain_.begin() + i);
      return;
    }
  }
}

void Shift::Context::setActive(Shift::Behavior::Base* behavior, bool active) {
  if(behavior) {
    active ? appendActive(behavior) : removeActive(behavior);
  }
}

void Shift::Context::setActive(Shift::Body::Base* body, bool active) {
  if(body) {
    active ? appendActive(body) : removeActive(body);
  }
}

void Shift::Context::setActive(Shift::CollisionGeometry::Base* geometry,
                               bool active) {
  if(geometry) {
    active ? appendActive(geometry) : removeActive(geometry);
  }
}

void Shift::Context::setActive(Shift::Constraint::Base* constraint,
                               bool active) {
  if(constraint) {
    active ? appendActive(constraint) : removeActive(constraint);
  }
}

void Shift::Context::setActive(Shift::ForceModule::Base* module, bool active) {
  if(module) {
    active ? appendActive(module) : removeActive(module);
  }
}

void Shift::Context::setActive(Shift::Geometry::Base* geometry, bool active) {
  if(geometry) {
    active ? appendActive(geometry) : removeActive(geometry);
  }
}

void Shift::Context::setActive(Shift::Joint::Base* joint, bool active) {
  if(joint) {
    active ? appendActive(joint) : removeActive(joint);
  }
}

void Shift::Context::setActive(Shift::Light::Base* light, bool active) {
  if(light) {
    active ? appendActive(light) : removeActive(light);
  }
}

void Shift::Context::setActive(Shift::Sensor::Base* sensor, bool active) {
  if(sensor) {
    active ? appendActive(sensor) : removeActive(sensor);
  }
}

void Shift::Context::setActive(Shift::Terrain::Base* terrain, bool active) {
  if(terrain) {
    active ? appendActive(terrain) : removeActive(terrain);
  }
}

void Shift::Context::setActive(Shift::Logger::Base* logger, bool active) {
  if(logger) {
    active ? appendActive(logger) : removeActive(logger);
  }
}

void Shift::Context::setActive(Shift::Integrator::Base* integrator,
                               bool active) {
  if(integrator) {
    active ? appendActive(integrator) : removeActive(integrator);
  }
}

void Shift::Context::setActive(Shift::Solver::Base* solver, bool active) {
  if(solver) {
    active ? appendActive(solver) : removeActive(solver);
  }
}

void Shift::Context::clearActiveBehavior() {
  active_behaviors_.clear();
}

void Shift::Context::clearActiveArticulatedBodies() {
  active_articulated_bodies_.clear();
}

void Shift::Context::clearActiveFreeBodies() {
  active_free_bodies_.clear();
}

void Shift::Context::clearActiveCollisionGeometry() {
  active_collision_geometry_.clear();
}

void Shift::Context::clearActiveConstraints() {
  active_constraints_.clear();
}

void Shift::Context::clearActiveForceModules() {
  active_force_modules_.clear();
}

void Shift::Context::clearActiveGeometry() {
  active_geometry_.clear();
}

void Shift::Context::clearActiveJoints() {
  active_joints_.clear();
}

void Shift::Context::clearActiveLights() {
  active_lights_.clear();
}

void Shift::Context::clearActiveSensors() {
  active_sensors_.clear();
}

void Shift::Context::clearActiveTerrain() {
  active_terrain_.clear();
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

void Shift::Context::connect() {
  for(auto& behavior : active_behaviors_) {
    behavior->connect();
  }
  for(auto& body : active_articulated_bodies_) {
    body->connect();
  }
  for(auto& geometry : active_collision_geometry_) {
    geometry->connect();
  }
  for(auto& constraint : active_constraints_) {
    constraint->connect();
  }
  for(auto& module : active_force_modules_) {
    module->connect();
  }
  for(auto& geometry : active_geometry_) {
    geometry->connect();
  }
  for(auto& joint : active_joints_) {
    joint->connect();
  }
  for(auto& light : active_lights_) {
    light->connect();
  }
  for(auto& sensor : active_sensors_) {
    sensor->connect();
  }
  for(auto& terrain : active_terrain_) {
    terrain->connect();
  }
}

void Shift::Context::loadData() {
  for(auto& behavior : active_behaviors_) {
    behavior->load();
  }
  for(auto& body : active_articulated_bodies_) {
    body->load();
  }
  for(auto& geometry : active_collision_geometry_) {
    geometry->load();
  }
  for(auto& constraint : active_constraints_) {
    constraint->load();
  }
  for(auto& module : active_force_modules_) {
    module->load();
  }
  for(auto& geometry : active_geometry_) {
    geometry->load();
  }
  for(auto& joint : active_joints_) {
    joint->load();
  }
  for(auto& light : active_lights_) {
    light->load();
  }
  for(auto& sensor : active_sensors_) {
    sensor->load();
  }
  for(auto& terrain : active_terrain_) {
    terrain->load();
  }
}

void Shift::Context::levelOrderSort() {

  std::queue<Shift::Joint::Base*> queue;
  std::vector<Shift::Joint::Base*> joints;
  std::vector<Shift::Body::Base*> bodies;

  /* find joints attached to fixed body */
  auto it = active_joints_.begin();
  while(it != active_joints_.end()) {
    assert((*it)->parentBody() != nullptr);
    assert((*it)->childBody() != nullptr);
    if((*it)->parentBody() == fixedBody()) {
      (*it)->setSense(1);
      (*it)->setLevel(1);
      queue.push(*it);
      it = active_joints_.erase(it);
    } else if((*it)->childBody() == fixedBody()) {
      (*it)->setSense(-1);
      (*it)->setLevel(1);
      queue.push(*it);
      it = active_joints_.erase(it);
    } else {
      it++;
    }
  }

  while(!queue.empty()) {
    auto joint = queue.front();
    queue.pop();
    Shift::Body::Base* body = nullptr;
    if(joint->sense() == 1) {
      body = joint->childBody();
    } else if(joint->sense() == -1) {
      body = joint->parentBody();
    }
    joints.push_back(joint);
    assert(body);

    /* find any joints attached to joint's child body */
    it = active_joints_.begin();
    while(it != active_joints_.end()) {
      if((*it)->parentBody() == body) {
        (*it)->setSense(1);
        (*it)->setLevel(joint->getLevel() + 1);
        queue.push(*it);
        it = active_joints_.erase(it);
      } else if((*it)->childBody() == body) {
        (*it)->setSense(-1);
        (*it)->setLevel(joint->getLevel() + 1);
        queue.push(*it);
        it = active_joints_.erase(it);
      } else {
        it++;
      }
    }
  }

  active_joints_ = joints;
}

void Shift::Context::processVirtualBodies() {

  // std::cout << "Processing virtual bodies" << std::endl;
  // std::cout << "active_articulated_bodies_.size(): "
  //           << active_articulated_bodies_.size() << std::endl;
  // std::cout << "active_bodies_.size(): " << active_bodies_.size() <<
  // std::endl;

  std::vector<Shift::Body::Base*> bodies(active_joints_.size());

  // clear system child body and frame pointers
  for(auto& joint : active_joints_) {
    joint->setSystemChildBody(nullptr);
    joint->setSystemChildFrame(nullptr);
  }

  chord_constraints_.clear();
  int j = 0;
  for(unsigned int i = 0; i < active_joints_.size(); i++) {

    bool found = false;
    auto body = active_joints_[i]->systemChildBody();
    for(auto it = active_articulated_bodies_.begin();
        it < active_articulated_bodies_.end(); it++) {
      if(body == (*it)) {
        active_articulated_bodies_.erase(it);
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

      auto chordJoint = active_joints_[i];
      chordConstraint->setChildBody(chordJoint->systemChildBody());
      chordConstraint->setChildFrame(chordJoint->systemChildFrame());
      chordConstraint->setOriginalParentBody(chordJoint->systemParentBody());
      chordConstraint->setOriginalParentFrame(chordJoint->systemParentFrame());

      /* add virtual bodies and chord constraints to active lists */
      bodies[i] = virtualBody.get();

      active_constraints_.push_back(constraint.get());
      chord_constraints_.push_back(chordConstraint);

      active_joints_[i]->setSystemChildBody(virtualBody.get());
      active_joints_[i]->setSystemChildFrame(virtualBody.get()->origin());

      /* transfer ownership of smart pointers for virtual bodies and chord
       * constraints to `Context` data structure */
      bodies_.push_back(std::move(virtualBody));
      constraints_.push_back(std::move(constraint));
    }
  }

  active_articulated_bodies_ = bodies;
}

void Shift::Context::processPCL() {
  for(unsigned int i = 0; i < active_articulated_bodies_.size(); i++) {
    active_articulated_bodies_[i]->setIndex(i);
  }
  pcl_.clear();
  pcl_.resize(active_joints_.size());
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    auto joint = active_joints_[i];
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
  //   std::cout << ", " << active_joints_[i]->name() << std::endl;
  // }

  // std::cout << std::endl << "# body / joint list: " << std::endl;
  // for(unsigned int i = 0; i < activeBodies_.size(); i++) {
  //   std::cout << "[" << i << "]" << active_joints_[i]->name();
  //   if(active_joints_[i]->sense() > 0) {
  //     std::cout << " +";
  //   } else {
  //     std::cout << " -";
  //   }
  //   std::cout << " " << activeBodies_[i]->name() << std::endl;
  // }
}

void Shift::Context::processLoopMembers() {
  std::vector<std::vector<int>> loopMembers(chord_constraints_.size());
  std::vector<std::vector<int>> negativeLoopMembers(chord_constraints_.size());
  std::vector<std::vector<int>> positiveLoopMembers(chord_constraints_.size());
  for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
    loopMembers[i].clear();
    negativeLoopMembers[i].clear();
    positiveLoopMembers[i].clear();

    if(!chord_constraints_[i]->parentBody()) {
      std::cerr << "no parent body for chordConstraint: " << i << std::endl;
      assert(0);
    }

    if(!chord_constraints_[i]->childBody()) {
      std::cerr << "no child body for chordConstraint: " << i << std::endl;
      assert(0);
    }

    int positive = chord_constraints_[i]->parentBody()->index();
    int negative = chord_constraints_[i]->childBody()->index();

    // std::cout << "parent body: " <<
    // chord_constraints_[i]->parentBody()->name()
    //           << ": " << positive << std::endl;
    // std::cout << "child body: " << chord_constraints_[i]->childBody()->name()
    //           << ": " << negative << std::endl;

    loopMembers[i].push_back(negative);
    negativeLoopMembers[i].push_back(negative);
    loopMembers[i].push_back(positive);
    positiveLoopMembers[i].push_back(positive);

    while(active_joints_[positive]->getLevel() >
          active_joints_[negative]->getLevel()) {
      positive = pcl_[positive];
      loopMembers[i].push_back(positive);
      positiveLoopMembers[i].push_back(positive);
    }
    while(active_joints_[negative]->getLevel() >
          active_joints_[positive]->getLevel()) {
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
  for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
    std::sort(loopMembers[i].begin(), loopMembers[i].end(), levelOrder(this));
  }
  loop_members_ = loopMembers;

  /* sort positive and negative loop member lists to follow master loop member
   * list ordering */
  negative_loop_members_.clear();
  positive_loop_members_.clear();
  negative_loop_members_.resize(chord_constraints_.size());
  positive_loop_members_.resize(chord_constraints_.size());
  for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
    negative_loop_members_[i].clear();
    for(auto j : loop_members_[i]) {
      if(std::find(negativeLoopMembers[i].begin(), negativeLoopMembers[i].end(),
                   j) != negativeLoopMembers[i].end()) {
        negative_loop_members_[i].push_back(j);
      }
      if(std::find(positiveLoopMembers[i].begin(), positiveLoopMembers[i].end(),
                   j) != positiveLoopMembers[i].end()) {
        positive_loop_members_[i].push_back(j);
      }
    }
  }
}

void Shift::Context::detectSubstructures() {
  // std::cout << "detectSubstructures()" << std::endl;
  if(chord_constraints_.size() > 0) {

    /* detect substructures */

    /* contains chord constraint indices */
    std::vector<std::vector<int>> substructures;

    /* create stack of all chordConstraint indices */
    std::list<int> chordConstraintIndices;
    for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
      chordConstraintIndices.push_back(i);
    }

    /* p_01 is used to permute chord loops here */
    std::vector<int> p_01(chord_constraints_.size());
    std::vector<int> p_10(chord_constraints_.size());
    int currentLoopIndex = 0;

    while(!chordConstraintIndices.empty()) {
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
          int baseNode = loop_members_[substructure[0]][0];
          if(Shift::Topology::inPath(pcl_, loop_members_[(*it)][0], baseNode)) {
            for(auto loopIndex : substructure) {
              for(auto loopMember = loop_members_[loopIndex].begin() + 1;
                  loopMember < loop_members_[loopIndex].end(); ++loopMember) {
                for(auto mergeCandidateMember =
                      loop_members_[(*it)].begin() + 1;
                    mergeCandidateMember < loop_members_[(*it)].end();
                    ++mergeCandidateMember) {
                  if(*loopMember == *mergeCandidateMember) {
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
    }

    /* sort loops of each substructure in monotonically increasing level order
     */
    for(auto& substructure : substructures) {
      std::sort(substructure.begin(), substructure.end(),
                substructureLoopLevelOrder(this, loop_members_));
    }

    /* sort substructures in monotonically level increasing order */
    std::sort(substructures.begin(), substructures.end(),
              substructureLevelOrder(this, loop_members_));

    // int k = 0;
    // for(auto substructure : substructures) {
    //   std::cout << "substructure [" << k++ << "]: ";
    //   for(auto index : substructure) {
    //     std::cout << index << " ";
    //   }
    //   std::cout << std::endl;
    // }
    substructure_indices_ = substructures;

    substructure_members_.clear();
    substructure_members_.resize(substructures.size());
    for(unsigned int i = 0; i < substructures.size(); i++) {
      for(auto loopIndex : substructures[i]) {
        for(auto loopMember : loop_members_[loopIndex]) {
          substructure_members_[i].push_back(loopMember);
        }
      }

      /* remove duplicate entries */
      std::sort(substructure_members_[i].begin(),
                substructure_members_[i].end());
      substructure_members_[i].erase(unique(substructure_members_[i].begin(),
                                            substructure_members_[i].end()),
                                     substructure_members_[i].end());

      /* sort substructure members in level increasing order */
      std::sort(substructure_members_[i].begin(),
                substructure_members_[i].end(), levelOrder(this));
    }

    // std::cout << "substructure members:" << std::endl;
    // unsigned int i = 0;
    // for(auto& members : substructure_members_) {
    //   std::cout << "[" << i++ << "]: " << std::endl;
    //   for(auto& member : members) {
    //     std::cout << "  " << active_joints_[member]->name() << std::endl;
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
        loopMembers.push_back(loop_members_[loop]);
        negativeLoopMembers.push_back(negative_loop_members_[loop]);
        positiveLoopMembers.push_back(positive_loop_members_[loop]);
        chordConstraints.push_back(chord_constraints_[loop]);
      }
    }

    loop_members_ = loopMembers;
    negative_loop_members_ = negativeLoopMembers;
    positive_loop_members_ = positiveLoopMembers;
    chord_constraints_ = chordConstraints;

    // std::cout << "loop members:" << std::endl;
    // for(unsigned int i = 0; i < loopMembers.size(); i++) {
    //   std::cout << "loop members [" << i << "]: " << std::endl;
    //   for(auto member : loopMembers[i]) {
    //     std::cout << "  " << active_joints_[member]->name() << std::endl;
    //   }
    //   std::cout << "negative loop members [" << i << "]: " << std::endl;
    //   for(auto member : negativeLoopMembers[i]) {
    //     std::cout << "  " << active_joints_[member]->name() << std::endl;
    //   }
    //   std::cout << "positive loop members [" << i << "]: " << std::endl;
    //   for(auto member : positiveLoopMembers[i]) {
    //     std::cout << "  " << active_joints_[member]->name() << std::endl;
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
    for(int i = -1; i < (int)active_joints_.size(); i++) {
      if(Shift::Topology::isSubstructureHead(i, substructure_members_) ||
         !Shift::Topology::isSubstructureMember(i, substructure_members_)) {
        shortList.push_back(i);
      }
    }

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
      while(j < substructure_members_.size() &&
            value == substructure_members_[j][0]) {
        for(unsigned int k = 1; k < substructure_members_[j].size(); k++) {
          /* append substructure members only if they are not already in
          the list */
          if(std::find(expandedList.begin(), expandedList.end(),
                       substructure_members_[j][k]) == expandedList.end()) {
            expandedList.push_back(substructure_members_[j][k]);
          }
        }
        j++;
      }
    }

    std::vector<int> expandedArray{std::begin(expandedList),
                                   std::end(expandedList)};

    /* verify that all joints were added to the expanded array */
    bool found = true;
    for(unsigned int i = 0; i < active_joints_.size(); i++) {
      found = false;
      for(auto j : expandedArray) {
        if((int)i == j) {
          found = true;
        }
      }
      if(!found) {
        std::cerr << " not found! " << active_articulated_bodies_[i]->name()
                  << std::endl;
      }
    }
    assert(found);

    /* reorder bodies, joints and levels */
    std::vector<Shift::Joint::Base*> activeJoints(active_joints_.size());
    std::vector<Shift::Body::Base*> activeBodies(
      active_articulated_bodies_.size());

    /* initialize permutation arrays to permute bodies and joints from order
     * originally loaded (p0) to order dictated by kinematic substructuring
     * algorithm */
    p_01.clear();
    p_01.resize(active_joints_.size());
    p_10.clear();
    p_10.resize(active_joints_.size());

    j = 0;
    for(unsigned int i = 0; i < expandedArray.size(); i++) {
      if(expandedArray[i] != -1) {
        p_01[j] = expandedArray[i];
        activeJoints[j] = active_joints_[expandedArray[i]];
        activeBodies[j] = active_articulated_bodies_[expandedArray[i]];
        activeBodies[j]->setIndex(j);
        j++;
      }
    }
    active_joints_ = activeJoints;
    active_articulated_bodies_ = activeBodies;

    Shift::Topology::setInversePermutation(p_01, p_10);

    // i = 0;
    // for(auto& members : substructure_members_) {
    //   std::cout << "  [" << i++ << "]: ";
    //   for(auto& member : members) {
    //     std::cout << member << " ";
    //   }
    //   std::cout << std::endl;
    // }

    for(auto& loop : loop_members_) {
      for(auto& i : loop) {
        i = p_10[i];
      }
    }

    for(auto& loop : negative_loop_members_) {
      for(auto& i : loop) {
        i = p_10[i];
      }
    }

    for(auto& loop : positive_loop_members_) {
      for(auto& i : loop) {
        i = p_10[i];
      }
    }

    for(auto& substructure : substructure_members_) {
      for(auto& i : substructure) {
        i = p_10[i];
      }
    }
  }

  // std::cout << std::endl << "substructures after sort:" << std::endl;
  // unsigned int i = 0;
  // for(auto& members : substructure_members_) {
  //   std::cout << "  [" << i++ << "]: ";
  //   for(auto& member : members) {
  //     std::cout << member << " ";
  //   }
  //   std::cout << std::endl;
  // }
  //
  // std::cout << "substructure members:" << std::endl;
  // i = 0;
  // for(auto& members : substructure_members_) {
  //   std::cout << "[" << i++ << "]: " << std::endl;
  //   for(auto& member : members) {
  //     std::cout << "  " << active_joints_[member]->name() << std::endl;
  //   }
  //   std::cout << std::endl;
  // }
  //
  // std::cout << "loop members:" << std::endl;
  // for(unsigned int i = 0; i < loop_members_.size(); i++) {
  //   std::cout << "loop members [" << i << "]: " << std::endl;
  //   for(auto member : loop_members_[i]) {
  //     std::cout << "  " << active_joints_[member]->name() << std::endl;
  //   }
  //   std::cout << "negative loop members [" << i << "]: " << std::endl;
  //   for(auto member : negative_loop_members_[i]) {
  //     std::cout << "  " << active_joints_[member]->name() << std::endl;
  //   }
  //   std::cout << "positive loop members [" << i << "]: " << std::endl;
  //   for(auto member : positive_loop_members_[i]) {
  //     std::cout << "  " << active_joints_[member]->name() << std::endl;
  //   }
  //   std::cout << std::endl;
  // }
}

unsigned int Shift::Context::qCount() {
  return q_count_;
}

unsigned int Shift::Context::dqCount() {
  return dq_count_;
}

/*!
\brief compress any frames on child body attached to parent body by
series of one or more rigid joints
*/
void Shift::Context::compressRigidJoints(
  int i, // current body Index and parent joint index
  unsigned int& operationCount) {

  ksl_SE3_t D;
  ksl_SE3_setIdentity(&D);

  int j = i;
  auto parentJoint = active_joints_[i];

  // D_p_01: transformation from proximal frame 0 to distal frame 1 on parent
  // body p
  // D_pc: relative transformation across rigid joint
  // D_c_01: transformation from proximal frame 0 to distal frame 1 on child
  // body c

  while(!parentJoint->qCount()) { // rigid joint
    // D = D_p1_c0 * D
    ksl_SE3_t temp;
    ksl_SE3_t D_p1_c0 = parentJoint->transform();
    // ksl_SE3_print(stdout, &D_p1_c0, "  D_p1_c0: ");
    ksl_product_dd(&D_p1_c0, &D, &temp);
    ksl_SE3_copy(&temp, &D);

    // D = D_p0_p1 * D_p1_c0
    // D <- D_p0_c0
    ksl_product_dd(parentJoint->systemParentFrame()->dWRTReference(), &D,
                   &temp);
    ksl_SE3_copy(&temp, &D);

    j = pcl_[j];
    if(j == -1) {
      break;
    }
    parentJoint = active_joints_[j];
  }

  auto joint = active_joints_[i];
  if(!joint->qCount()) {
    // std::cout << "processing joint: " << joint->name() << std::endl;
    auto parentBody = joint->systemParentBody();
    auto childBody = active_articulated_bodies_[i];

    // ksl_SE3_print(stdout, &D, "  D_p0_c0: ");

    for(auto& frame : childBody->frames()) {
      // std::cout << childBody->name() << ": " << frame.name() << std::endl;

      // dWRTReference = dWRTReference * dWRTReference_child
      ksl_SE3_t D_p0_c1;
      ksl_product_dd(&D, frame->dWRTReference(), &D_p0_c1);

      // ksl_SE3_print(stdout, &D_p0_c1, "  D_p0_c1: ");

      if(frame->inTree() && parentBody != fixedBody()) {
        // std::cout << "intree && parentBody!= fixedBody: processing frame: "
        //           << childBody->name() << ": " << frame.name() << std::endl;

        ksl_SE3_copy(&D_p0_c1, d_wrt_reference_ + frame->index());

        dhva_offsets_[operationCount].parent.d =
          parentJoint->systemParentBody()->referenceFrame()->index();
        dhva_offsets_[operationCount].child.d = frame->index();
        dhva_offsets_[operationCount].q = frame->index();

        d_fptr_[operationCount] = Shift::Joint::rigid_dh;
        dh_fptr_[operationCount] = Shift::Joint::rigid_dh;
        dhva_fptr_[operationCount] = Shift::Joint::rigid_dhva;
        operationCount++;
      } else if(frame->inTree()) {
        // std::cout << "processing frame: " << childBody->name() << ": "
        //           << frame.name() << std::endl;

        // ksl_SE3_print(stdout, frame->dWRTReference(), "d_wrt_reference: ");
        ksl_SE3_copy(&D_p0_c1, d_wrt_reference_ + frame->index());

        // ksl_SE3_print(
        //   stdout,
        //   dhva_.d +
        //   parentJoint->systemParentBody()->referenceFrame()->index(), "
        //   d_parent: ");

        ksl_product_dd(
          dhva_.d + parentJoint->systemParentBody()->referenceFrame()->index(),
          &D_p0_c1, dhva_.d + frame->index());
        // ksl_SE3_print(stdout, dhva_.d + frame->index(), "  dAbsolute: ");
      }
    }
  }
}

void Shift::Context::allocateRunTimeArrays() {
  unsigned int dc_count = 0;
  q_count_ = 0;
  dq_count_ = 0;
  unsigned int data_count = 0;
  non_rigid_joint_count_ = 0;
  for(auto& joint : active_joints_) {
    dc_count += joint->dcCount();
    q_count_ += joint->qCount();
    dq_count_ += joint->dqCount();
    data_count += joint->dataCount();
    if(joint->qCount() > 0) {
      non_rigid_joint_count_++;
    }
  }

  dhva_ = Shift::Topology::DHVA(frames_in_tree_.size(), dc_count, dq_count_,
                                non_rigid_joint_count_);

  // std::cout << "qCount: " << qCount_ << std::endl;
  // std::cout << "dqCount: " << dqCount_ << std::endl;
  // std::cout << "dcCount: " << dcCount << std::endl;

  // std::cout << "dWRTReference_.size(): " << frames_in_tree_.size() <<
  // std::endl;
  if(d_wrt_reference_) {
    free(d_wrt_reference_);
    d_wrt_reference_ = nullptr;
  }
  d_wrt_reference_ = ksl_SE3_alloc(frames_in_tree_.size());

  d_reference_frames_.clear();
  d_reference_frames_.resize(non_rigid_joint_count_);

  dcq_.clear();
  dcq_.resize(dc_count);

  if(joint_data_) {
    delete joint_data_;
    joint_data_ = nullptr;
  }
  joint_data_ = new double[data_count];

  if(q_) {
    delete[] q_;
    q_ = nullptr;
    dq_ = nullptr;
  }
  q_ = new double[q_count_ + dq_count_]{};
  dq_ = q_ + q_count_;

  if(deltaq_) {
    delete[] deltaq_;
  }
  deltaq_ = new double[dq_count_];

  if(ddq_) {
    delete ddq_;
    ddq_ = nullptr;
  }
  ddq_ = new double[dq_count_];
  memset(ddq_, 0, dq_count_ * sizeof(double));

  unsigned int i = 0;
  for(auto& body : active_articulated_bodies_) {
    if(body->parentJoint() && body->parentJoint()->qCount() > 0) {
      body->setvaIndex(i++);
    } else {
      body->setvaIndex(-1);
    }
  }

  // get operations count
  unsigned int operationCount = 0;
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    int j = i;
    auto joint = active_joints_[j];
    auto parentJoint = joint;
    bool rigidConnectionToFixedFrame = false;
    while(!parentJoint->qCount()) { // rigid joint
      j = pcl_[j];
      if(j == -1) {
        // std::cout << "rigid connection to fixed frame found: ";
        // std::cout << joint->name() << std::endl;
        rigidConnectionToFixedFrame = true;
        break;
      }
      parentJoint = active_joints_[j];
    }

    if(!rigidConnectionToFixedFrame) {
      auto body = joint->systemChildBody();
      operationCount += body->frameInTreeCount();
      // std::cout << body->name() << ": " << body->frameInTreeCount()
      //           << std::endl;
    }
  }

  // std::cout << "operationCount: " << operationCount << std::endl;

  dhva_offsets_.clear();
  dhva_offsets_.resize(operationCount);
  for(auto& offset : dhva_offsets_) {
    offset.q = -1;
    offset.dq = -1;
    offset.data = -1;
    offset.parent.d = -1;
    offset.parent.v = -1;
    offset.child.d = -1;
    offset.child.v = -1;
  }

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
  for(auto& joint : active_joints_) {
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
  // std::cout << "updating frames on fixed body" << std::endl;
  for(auto& frame : fixedBody()->frames()) {
    frame->setParentBody(fixedBody());
    if(!frame->isReference()) {
      // std::cout << frame->name() << std::endl;
      assert(frame->parentBody());
      // std::cout << "[" << frame.index() << "]: " <<
      // frame.parentBody()->name()
      //           << ":" << frame.name() << std::endl;
      ksl_SE3_copy(frame->dWRTReference(), dhva_.d + frame->index());
      ksl_SE3_copy(frame->dWRTReference(), d_wrt_reference_ + frame->index());
      // ksl_SE3_print(stdout, dhva_.d + frame.index());
    }
  }

  operationCount = 0;
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    auto& joint = active_joints_[i];
    auto parentBody = joint->systemParentBody();
    auto parentFrame = joint->systemParentFrame();
    auto childBody = active_articulated_bodies_[i];
    auto childFrame = childBody->referenceFrame();

    if(joint->qCount()) {
      if(joint->dcCount()) {
        dcq_[joint->dcIndex()] = joint->qIndex();
        dhva_offsets_[operationCount].dc = joint->dcIndex() * 2;
      } else {
        dhva_offsets_[operationCount].q = joint->qIndex();
      }

      dhva_offsets_[operationCount].parent.d = parentFrame->index();
      dhva_offsets_[operationCount].child.d = childFrame->index();

      dhva_offsets_[operationCount].parent.v = parentBody->vaIndex();
      dhva_offsets_[operationCount].child.v = childBody->vaIndex();
      dhva_offsets_[operationCount].dq = joint->dqIndex();
      dhva_offsets_[operationCount].data = joint->dataIndex();

      d_fptr_[operationCount] = joint->Dfptr();
      dh_fptr_[operationCount] = joint->DHfptr();
      dhva_fptr_[operationCount] = joint->DHVAfptr();
      operationCount++;

      auto referenceFrame = childBody->referenceFrame();
      // std::cout << "referenceFrame: " << referenceFrame->parentBody()->name()
      //           << ":" << referenceFrame->name() << std::endl;
      for(auto& frame : childBody->frames()) {
        /*!
          @TODO add logic to detect if transformation has zero entries
          and use appropriate sparse function evaluation
          for now, all function pointers are assigned as general SE3
          displacements
        */
        if(!frame->isReference() && frame->inTree()) {
          d_fptr_[operationCount] = Shift::Joint::rigid_dh;
          dh_fptr_[operationCount] = Shift::Joint::rigid_dh;
          dhva_fptr_[operationCount] = Shift::Joint::rigid_dhva;
          dhva_offsets_[operationCount].parent.d = referenceFrame->index();
          dhva_offsets_[operationCount].child.d = frame->index();
          dhva_offsets_[operationCount].q = frame->index();
          // std::cout << "  processing child frame[" << frame.index()
          //           << "]: " << frame.name() << std::endl;
          // ksl_SE3_print(stdout, frame.dWRTReference(), "  dWRTReference: ");
          ksl_SE3_copy(frame->dWRTReference(),
                       d_wrt_reference_ + frame->index());
          operationCount++;
        }
      }
    } else {
      compressRigidJoints(i, operationCount);
    }
  }

  // std::cout << "operationCount: " << operationCount << std::endl;
  joint_dof_.resize(active_joints_.size());
  model_parameters_.initialize();
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
    (d_fptr_[i])(*this, dhva, dhva_offsets_[i], q);
  }
}

void Shift::Context::updateDH() {
  updateDH(dhva_, q_);
}

void Shift::Context::updateDH(Shift::Topology::DHVA& dhva,
                              const double* __restrict__ q) {
  updateDC(dhva, q);
  // std::cout << "updateDH()" << std::endl;
  for(unsigned int i = 0; i < dh_fptr_.size(); i++) {
    // auto parentFrame = frames_in_tree_[dhva_offsets_[i].parent.d];
    // auto childFrame = frames_in_tree_[dhva_offsets_[i].child.d];
    // std::cout << "  [" << i
    //           << "] dParent: " << parentFrame->parentBody()->name() << ": "
    //           << parentFrame->name() << std::endl;
    // std::cout << "        dChild:  " << childFrame->parentBody()->name() <<
    // ": "
    //           << childFrame->name() << std::endl;
    (dh_fptr_[i])(*this, dhva, dhva_offsets_[i], q);
  }
  // std::cout << std::endl;
  // FILE* f = fopen("frames.dat", "w");
  // for(auto& frame : frames_in_tree_) {
  //   fprintf(f, "[%d]: [%s, %s]\n", frame->index(),
  //           frame->parentBody()->name().c_str(), frame->name().c_str());
  //   // ksl_SE3_print(f, dWRTReference_ + frame->index(), "    dWRTReference:
  //   "); ksl_SE3_print(f, dhva_.d + frame->index(), "  "); fprintf(f, "\n");
  // }
  // fclose(f);
}

void Shift::Context::updateDHVA() {
  updateDHVA(dhva_, q_, dq_);
}

void Shift::Context::updateDHVA(Shift::Topology::DHVA& dhva,
                                const double* __restrict__ q,
                                const double* __restrict__ dq) {
  updateDC(dhva, q);
  for(unsigned int i = 0; i < dhva_fptr_.size(); i++) {
    (dhva_fptr_[i])(*this, dhva, dhva_offsets_[i], q, dq);
  }
}

void Shift::Context::setFixedFrameGravityVector() {
  assert(dhva_.aq);
  ksl_vec3_inverted(model_parameters_.gravity(), &dhva_.aq[-1].lin);
}

void Shift::Context::setToInitialConditions() {
  // std::cout << "setting initial conditions:" << std::endl;
  assert(q_);
  assert(dq_);
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  for(auto& joint : active_joints_) {
    if(joint->qCount()) {
      // std::cout << joint->name() << " " << std::flush;
      assert(joint->q0());
      // for(unsigned int i = 0; i < joint->qCount(); i++) {
      //   std::cout << joint->q0()[i] << " ";
      // }
      // std::cout << std::endl;
      memcpy(q_ + qOffset, joint->q0(), joint->qCount() * sizeof(double));
    }
    if(joint->dqCount()) {
      // for(unsigned int i = 0; i < joint->dqCount(); i++) {
      //   std::cout << joint->dq0()[i] << " ";
      // }
      // std::cout << std::endl;

      memcpy(dq_ + dqOffset, joint->dq0(), joint->dqCount() * sizeof(double));
    }
    qOffset += joint->qCount();
    dqOffset += joint->dqCount();
  }
}

void Shift::Context::initializeInertia() {

  /*
    create a temporary inertia array, equal in length to activeJoints
    after this list is generated, it will be compressed to an array
    equal to length of nonRigidJointCount
  */
  ksl_inertia_t* inertia_temp = ksl_inertia_alloc(active_joints_.size());

  /* compute each inertia matrix with respect to reference frame on
     each body */
  for(int i = (int)active_joints_.size() - 1; i > -1; i--) {
    auto& body = active_articulated_bodies_[i];
    if(body->hasMass()) {
      inertia_temp[i] = body->inertiaWRTReference();
    }
  }

  /* for bodies connected by rigid joints, merge inertias into parent body
  inertia */
  for(int i = (int)active_joints_.size() - 1; i > -1; i--) {
    /* only perform merge if parent joint type is rigid and
       the parent body is not the fixed body */
    if(active_joints_[i]->qCount() == 0 && pcl_[i] > -1) {
      auto childBody = active_articulated_bodies_[i];
      auto parentBody = active_articulated_bodies_[pcl_[i]];
      auto parentFrame = active_joints_[i]->systemParentFrame();
      ksl_vec3_t t_ic_i = parentBody->centerOfMassWRTReference();
      ksl_vec3_t t_jc_j = childBody->centerOfMassWRTReference();
      ksl_inertia_t parentInertia = parentBody->inertiaWRTReference();
      ksl_inertia_t childInertia = childBody->inertiaWRTReference();

      ksl_SE3_t D_ij;
      ksl_SE3_t temp = active_joints_[i]->transform();
      if(parentFrame->isReference()) {
        ksl_SE3_copy(&temp, &D_ij);
      } else {
        ksl_product_dd(parentFrame->dWRTReference(), &temp, &D_ij);
      }
      // i = parent, j = child;
      ksl_inertia_merge(&parentInertia, &t_ic_i, &childInertia, &t_jc_j, &D_ij);

      inertia_temp[pcl_[i]] = parentInertia;
    }
  }

  /* compress expanded inertia list into compressed list */
  if(inertia_) {
    free(inertia_);
  }
  inertia_ = ksl_inertia_alloc(non_rigid_joint_count_);
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    int j = joint_lookup_[i];
    if(j != -1) {
      memcpy(inertia_ + j, inertia_temp + i, sizeof(ksl_inertia_t));
    }
  }

#if SHIFT_VERBOSE_
  printf("inertia:\n");
  for(int i = 0; i < nonRigidJointCount; i++) {
    printf("  [%i] ", i);
    s2_inertia_print(stdout, inertia + i);
  }
#endif

  if(inertiaP_) {
    free(inertiaP_);
  }
  inertiaP_ = ksl_inertia_alloc(non_rigid_joint_count_ + 1);
  free(inertia_temp);
}

void Shift::Context::setInitialConditionsToCurrentConditions() {
  assert(q_);
  assert(dq_);
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  for(auto& joint : active_joints_) {
    joint->setInitialDisplacement(q_ + qOffset);
    joint->setInitialVelocity(dq_ + dqOffset);
    qOffset += joint->qCount();
    dqOffset += joint->dqCount();
  }
}

void Shift::Context::setLookup() {
  joint_lookup_.clear();
  joint_lookup_.resize(activeJointCount());

  dq_lookup_.clear();
  dq_lookup_.resize(activeJointCount());

  q_lookup_.clear();
  q_lookup_.resize(activeJointCount());

  dq_lookup_compressed_.clear();
  dq_lookup_compressed_.resize(nonRigidJointCount());

  int jointCount = 0;
  int dqCount = 0;
  int qCount = 0;
  unsigned int i = 0;
  for(auto& joint : activeJoints()) {
    if(joint->qCount() > 0) {
      joint_lookup_[i] = jointCount;
      dq_lookup_[i] = dqCount;
      q_lookup_[i] = qCount;
      jointCount++;
    } else {
      joint_lookup_[i] = -1;
      dq_lookup_[i] = -1;
      q_lookup_[i] = -1;
    }
    i++;
    dqCount += joint->qCount();
    qCount += joint->qCount();
  }

  for(unsigned int j = 0; j < activeJointCount(); j++) {
    if(joint_lookup_[j] > -1) {
      dq_lookup_compressed_[joint_lookup_[j]] = dq_lookup_[j];
    }
  }
}

/*!
@brief Initialize quantities necessary to compute mobility numbers

This function evaluates the variable mobility numbers in
permuted order. It first forms an n by n-r matrix from the
submatrix J[0:r-1][r:n-1] stacked above an (n-r) by (n-r)
identity matrix. Then it calls GramSchmidt() to
orthonormalize the columns and return them in the same
matrix. Finally it returns the square of row norms of
this matrix as mobility numbers in the mn[] array.
*/
void Shift::Context::initializeMobility() {
  mobility_ = 0;
  ksl_array_free(mobility_numbers_);
  if(dq_count_ > 0) {
    mobility_numbers_ = (double*)malloc(dq_count_ * sizeof(double));
    for(unsigned int i = 0; i < dq_count_; i++) {
      mobility_numbers_[i] = 1.0;
    }
  } else {
    mobility_numbers_ = nullptr;
  }
  jacobian_.initializeMobility();
}

/*!
  @brief compute global instantaneous mobility, to be called after
  mobility has been computed within each substructure
*/
double Shift::Context::globalMobility() {
#if SHIFT_WITH_BLAS_
  mobility_ = cblas_dasum(dqCount_, mobilityNumbers_, 1);
#else
  mobility_ = 0;
  for(unsigned int i = 0; i < dq_count_; i++) {
    mobility_ += mobility_numbers_[i];
  }
#endif
  return mobility_;
}

/*!
@brief Generate a directed graph using graphviz api library

@param context a pointer to a Shift::Context data structure.

*/
int Shift::Context::digraph(std::string filename) {

  std::string base_filename = Shift::Topology::getFilename(filename);
  std::cout << "base_filename: " << base_filename << std::endl;
  if(base_filename.length() == 0) {
    std::cerr << "Error: could not resolve filename for: " << filename
              << std::endl;
    return -1;
  };

  std::string extension = Shift::Topology::getExtension(base_filename);
  if(extension.length() == 0) {
    std::cerr << "Error: coud not resolve extension for: " << filename
              << std::endl;
  }

  std::string gv_ext = Shift::Topology::getGraphvizExtension(extension);
  if(gv_ext.length() == 0) {
    std::cerr << "Error: requested extension is not supported by Shift. "
                 "Supported extensions include: \"gif\", \"jpeg\", \"jpg\", "
                 "\"png\", \"svg\", \"pdf\" and \"dot\""
              << std::endl;
    return -1;
  }

  char name[1000];
  GVC_t* gvc = gvContext();
  Agraph_t* g = agopen(const_cast<char*>("g"), Agdirected, nullptr);

  agattr(g, AGNODE, const_cast<char*>("shape"), const_cast<char*>(""));
  agattr(g, AGEDGE, const_cast<char*>("dir"), const_cast<char*>("arrowtail"));
  agattr(g, AGEDGE, const_cast<char*>("style"), const_cast<char*>(""));
  agattr(g, AGEDGE, const_cast<char*>("label"), const_cast<char*>(""));
  agattr(g, AGNODE, const_cast<char*>("penwidth"), const_cast<char*>(""));

  /* set nodes (bodies) */
  Agnode_t** nodes = (Agnode_t**)malloc(activeJointCount() * sizeof(Agnode_t*));
  for(unsigned int i = 0; i < activeJointCount(); i++) {
    auto bodyName = activeArticulatedBodies()[i]->name();
    sprintf(name, "[%d] %s", i, const_cast<char*>(bodyName.c_str()));
    nodes[i] = agnode(g, name, TRUE);
    if(activeArticulatedBodies()[i]->isVirtual()) {
      agset(nodes[i], const_cast<char*>("shape"), const_cast<char*>("octagon"));
    }
  }

  Agnode_t* fixed = agnode(g, (char*)"[-1] fixed", TRUE);
  Agedge_t* edge;
  for(unsigned int i = 0; i < activeJointCount(); i++) {
    std::string jointName = activeJoints()[i]->name();
    if(pcl()[i] == -1) {
      edge =
        agedge(g, fixed, nodes[i], const_cast<char*>(jointName.c_str()), TRUE);
    } else {
      edge = agedge(g, nodes[pcl()[i]], nodes[i],
                    const_cast<char*>(jointName.c_str()), TRUE);
    }
    sprintf(name, "[%d] %s (%s)", i, jointName.c_str(),
            activeJoints()[i]->type().c_str());
    agset(edge, const_cast<char*>("label"), name);
  }

  for(unsigned int i = 0; i < chordConstraintCount(); i++) {
    int parentIndex = chordConstraints()[i]->parentBody()->index();
    std::string jointName = chordConstraints()[i]->name();
    if(parentIndex == -1) {
      edge =
        agedge(g, fixed, nodes[chordConstraints()[i]->childBody()->index()],
               const_cast<char*>(jointName.c_str()), TRUE);
    } else {
      // s2_stackstr_toStr(context->jointPointers[i]->name);
      edge = agedge(g, nodes[parentIndex],
                    nodes[chordConstraints()[i]->childBody()->index()],
                    const_cast<char*>(jointName.c_str()), TRUE);
    }
    char label[100];
    sprintf(label, "[%d]", i);
    agset(edge, const_cast<char*>("label"), label);
    agset(edge, const_cast<char*>("style"), const_cast<char*>("dashed"));
    agset(edge, const_cast<char*>("dir"), const_cast<char*>("none"));
  }

  gvLayout(gvc, g, "dot");
  gvRenderFilename(gvc, g, const_cast<char*>(gv_ext.c_str()), filename.c_str());

  if(gv_ext != "dot") {
    std::string basename = Shift::Topology::getBasename(filename);
    if(basename.length() == 0) {
      gvFreeLayout(gvc, g);
      agclose(g);
      gvFreeContext(gvc);
      free(nodes);
      // fclose(DGF);
      return -1;
    }
    basename += ".dot";
    gvRenderFilename(gvc, g, "dot", const_cast<char*>(basename.c_str()));
  }

  // cleanup
  gvFreeLayout(gvc, g);
  agclose(g);
  gvFreeContext(gvc);
  free(nodes);
  // fclose(DGF);
  return 0;

  // agclose and gvFreeContext appear to not entirely free memory, it
  // up to 1MB of memory not properly freed at program termination.
  // @TODO find fix for this or just write to DOT file and call graphviz using
  // external system call
}

bool findReferenceFrame(const Shift::Context& context, int dIndex,
                        int* referenceFrameIndex) {
  auto& bodies = context.activeArticulatedBodies();
  for(auto& body : bodies) {
    int referenceFrame = body->referenceFrame()->index();
    for(auto& frame : body->frames()) {
      if(frame->index() == dIndex) {
        *referenceFrameIndex = referenceFrame;
      }
    }
  }
  return false;
}

/*!
@brief Generate a directed graph using graphviz api library

The graphviz library leaks memory

*/
int Shift::Context::digraphFrames(std::string filename) {

  std::string base_filename = Shift::Topology::getFilename(filename);
  if(base_filename.length() == 0) {
    std::cerr << "Error: could not resolve filename for: " << filename
              << std::endl;
    return -1;
  };

  std::string extension = Shift::Topology::getExtension(base_filename);
  if(extension.length() == 0) {
    std::cerr << "Error: coud not resolve extension for: " << filename
              << std::endl;
  }

  std::string gv_ext = Shift::Topology::getGraphvizExtension(extension);
  if(gv_ext.length() == 0) {
    std::cerr << "Error: requested extension is not supported by Shift. "
                 "Supported extensions include: \"gif\", \"jpeg\", \"jpg\", "
                 "\"png\", \"svg\", \"pdf\" and \"dot\""
              << std::endl;
    return -1;
  }

  // FILE* DGF;
  // if((DGF = fopen(filename.c_str(), "w")) == nullptr) {
  //   std::cerr << "Cannot open " << filename << ". No file was generated."
  //             << std::endl;
  //   return (-1);
  // }

  char name[1200];
  GVC_t* gvc = gvContext();

  /* root graph */
  Agraph_t* g = agopen(const_cast<char*>("g"), Agdirected, nullptr);
  agattr(g, AGNODE, const_cast<char*>("shape"), const_cast<char*>(""));
  agattr(g, AGEDGE, const_cast<char*>("dir"), const_cast<char*>("arrowtail"));
  agattr(g, AGEDGE, const_cast<char*>("style"), const_cast<char*>(""));
  agattr(g, AGEDGE, const_cast<char*>("label"), const_cast<char*>(""));
  agattr(g, AGNODE, const_cast<char*>("penwidth"), const_cast<char*>("1"));

  int jointCounter = 0;
  for(unsigned int i = 0; i < dhvaOffsets().size(); i++) {

    int parentFrameIndex = dhvaOffsets()[i].parent.d;
    int childFrameIndex = dhvaOffsets()[i].child.d;

    ksl_SE3_t* dParent = dhva_.d + parentFrameIndex;
    ksl_SE3_t* dChild = dhva_.d + childFrameIndex;

    int parentReferenceFrameIndex = -1;
    findReferenceFrame(*this, parentFrameIndex, &parentReferenceFrameIndex);
    bool parentIsReference = (parentFrameIndex == parentReferenceFrameIndex);

    int childReferenceFrameIndex = -1;
    findReferenceFrame(*this, childFrameIndex, &childReferenceFrameIndex);
    bool childIsReference = (childFrameIndex == childReferenceFrameIndex);

    assert(framesInTree()[parentFrameIndex]->parentBody());
    std::string parentBodyName =
      framesInTree()[parentFrameIndex]->parentBody()->name();

    char parentFrameName[1200];
    sprintf(parentFrameName,
            "[%d] {%d} %s\n"
            "[% 2.2f % 2.2f % 2.2f % 0.2f]\n"
            "[% 2.2f % 2.2f % 2.2f % 0.2f]\n"
            "[% 2.2f % 2.2f % 2.2f % 0.2f]\n"
            "[ 0.00  0.00  0.00  1.00]",
            parentFrameIndex, parentReferenceFrameIndex,
            framesInTree()[parentFrameIndex]->name().c_str(), dParent->R.m00,
            dParent->R.m01, dParent->R.m02, dParent->t.x, dParent->R.m10,
            dParent->R.m11, dParent->R.m12, dParent->t.y, dParent->R.m20,
            dParent->R.m21, dParent->R.m22, dParent->t.z);

    assert(framesInTree()[childFrameIndex]->parentBody());
    std::string childBodyName =
      framesInTree()[childFrameIndex]->parentBody()->name();

    char childFrameName[1200];
    sprintf(childFrameName,
            "[%d] {%d} %s\n"
            "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
            "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
            "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
            "[ 0.00  0.00  0.00  1.00]",
            childFrameIndex, childReferenceFrameIndex,
            framesInTree()[childFrameIndex]->name().c_str(), dChild->R.m00,
            dChild->R.m01, dChild->R.m02, dChild->t.x, dChild->R.m10,
            dChild->R.m11, dChild->R.m12, dChild->t.y, dChild->R.m20,
            dChild->R.m21, dChild->R.m22, dChild->t.z);

    std::string jointName = Shift::Topology::getJointType(dh_fptr_[i]);

    if(jointName == "PRISMATIC_TX") {
      sprintf(name, "[%d] {%d} Tx(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TX_INVERSE") {
      sprintf(name, "[%d] {%d} Txinverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TY") {
      sprintf(name, "[%d] {%d} Ty(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TY_INVERSE") {
      sprintf(name, "[%d] {%d} Tyinverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TZ") {
      sprintf(name, "[%d] {%d} Tz(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TZ_INVERSE") {
      sprintf(name, "[%d] {%d} Tzinverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TA") {
      sprintf(name, "[%d] {%d} Ta(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "PRISMATIC_TA_INVERSE") {
      sprintf(name, "[%d] {%d} Tainverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "REVOLUTE_RX") {
      sprintf(name, "[%d] {%d} Rx(%f, %f)\n", i, jointCounter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(jointName == "REVOLUTE_RX_INVERSE") {
      sprintf(name, "[%d] {%d} Rxinverse(%f, %f)\n", i, jointCounter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(jointName == "REVOLUTE_RY") {
      sprintf(name, "[%d] {%d} Ry(%f, %f)\n", i, jointCounter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(jointName == "REVOLUTE_RY_INVERSE") {
      sprintf(name, "[%d] {%d} Ryinverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "REVOLUTE_RZ") {
      sprintf(name, "[%d] {%d} Rz(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "REVOLUTE_RZ_INVERSE") {
      sprintf(name, "[%d] {%d} Rzinverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "REVOLUTE_RA") {
      sprintf(name, "[%d] {%d} Ra(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "REVOLUTE_RA_INVERSE") {
      sprintf(name, "[%d] {%d} Rainverse(%f)\n", i, jointCounter++,
              q_[dhva_offsets_[i].q]);

    } else if(jointName == "SPHERICAL" || jointName == "SPHERICAL_INVERSE") {
      ksl_quaternion_t* q = (ksl_quaternion_t*)&(q_[dhva_offsets_[i].q]);
      sprintf(name, "[%d] {%d} q: [% 2.2f, % 2.2f, % 2.2f, % 2.2f]", i,
              jointCounter++, q->x, q->y, q->z, q->w);

    } else if(jointName == "FLOAT" || jointName == "FLOAT_INVERSE") {
      ksl_vec3_t* t = (ksl_vec3_t*)&(q_[dhva_offsets_[i].q]);
      ksl_quaternion_t* q = (ksl_quaternion_t*)&(q_[dhva_offsets_[i].q + 3]);
      sprintf(name,
              "[%d] {%d} t: [% 2.2f, % 2.2f, % 2.2f]\n"
              "     q: [% 2.2f, % 2.2f, % 2.2f, % 2.2f]",
              i, jointCounter++, t->x, t->y, t->z, q->x, q->y, q->z, q->w);
    } else if(jointName == "RIGID" || jointName == "RIGID_INVERSE") {

      ksl_SE3_t* dPrimitiveDisp = dhva_.d + dhva_offsets_[i].child.d;
      sprintf(name,
              "[%d] Rigid\n"
              "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
              "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
              "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
              "[ 0.00  0.00  0.00  1.00]",
              i, dPrimitiveDisp->R.m00, dPrimitiveDisp->R.m01,
              dPrimitiveDisp->R.m02, dPrimitiveDisp->t.x, dPrimitiveDisp->R.m10,
              dPrimitiveDisp->R.m11, dPrimitiveDisp->R.m12, dPrimitiveDisp->t.y,
              dPrimitiveDisp->R.m20, dPrimitiveDisp->R.m21,
              dPrimitiveDisp->R.m22, dPrimitiveDisp->t.z);
    } else {
      sprintf(name, "joint type not found / unknown");
    }

    char parentName[400];
    sprintf(parentName, "cluster_%s", parentBodyName.c_str());
    Agraph_t* parentBodyGraph = agsubg(g, parentName, TRUE);
    agset(parentBodyGraph, const_cast<char*>("label"),
          const_cast<char*>(parentBodyName.c_str()));
    agset(parentBodyGraph, const_cast<char*>("fillcolor"),
          const_cast<char*>("lightgrey"));
    agset(parentBodyGraph, const_cast<char*>("style"),
          const_cast<char*>("filled"));

    char childName[400];
    sprintf(childName, "cluster_%s", childBodyName.c_str());
    Agraph_t* childBodyGraph = agsubg(g, childName, TRUE);
    agset(childBodyGraph, const_cast<char*>("fillcolor"),
          const_cast<char*>("lightgrey"));
    agset(childBodyGraph, const_cast<char*>("style"),
          const_cast<char*>("filled"));
    agset(childBodyGraph, const_cast<char*>("label"),
          const_cast<char*>(childBodyName.c_str()));

    Agnode_t* parentNode = agnode(parentBodyGraph, parentFrameName, TRUE);
    if(parentIsReference) {
      agset(parentNode, const_cast<char*>("penwidth"), const_cast<char*>("3"));
    }

    Agnode_t* childNode = agnode(childBodyGraph, childFrameName, TRUE);
    if(childIsReference) {
      agset(childNode, const_cast<char*>("penwidth"), const_cast<char*>("3"));
    }

    Agedge_t* edge = agedge(g, parentNode, childNode, name, TRUE);
    agset(edge, const_cast<char*>("label"), name);
  }

  gvLayout(gvc, g, "dot");

  gvRenderFilename(gvc, g, const_cast<char*>(gv_ext.c_str()), filename.c_str());

  if(gv_ext != "dot") {
    std::string basename = Shift::Topology::getBasename(filename);
    if(basename.length() == 0) {
      gvFreeLayout(gvc, g);
      agclose(g);
      gvFreeContext(gvc);
      // fclose(DGF);
      std::cerr << "Could not generate .dot file." << std::endl;
      return -1;
    }
    basename += ".dot";
    gvRenderFilename(gvc, g, "dot", basename.c_str());
  }

  gvFreeLayout(gvc, g);
  agclose(g);
  gvFreeContext(gvc);

  // agclose and gvFreeContext appear to not entirely free memory,
  // up to 1MB of memory not properly freed at program termination.
  // fclose(DGF);
  return 0;
}

/*!
@brief computes total gravitational potential energy for system

@return gravitational potential energy
*/
double Shift::Context::gravitationalPotentialEnergy() {
  return ksl_dot_vv(&inertiaP_->mt, model_parameters_.gravity());
}

/*!
@brief compute kinetic energy for system

*/
double Shift::Context::kineticEnergy() {
  assert(p_);
  assert(dhva_.v);
#if SHIFT_WITH_BLAS_
  return cblas_ddot(6 * nonRigidJointCount_, (double*)(p_), 1,
                    (double*)(dhva_.v), 1) /
         2.0;
#else
  double sum = 0.0;
  for(unsigned int i = 0; i < non_rigid_joint_count_; i++) {
    sum += ksl_dot_cs(p_ + i, dhva_.v + i);
  }
  return sum;
#endif
}
