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

#include "shift/core/Context.h"
#include "shift/core/Exception.h"
#include "shift/core/Factory.h"
#include "shift/core/Topology.h"
#include "shift/core/behavior/Base.h"
#include "shift/core/body/Base.h"
#include "shift/core/body/Rigid.h"
#include "shift/core/collisiongeometry/Base.h"
#include "shift/core/constraint/Base.h"
#include "shift/core/constraint/Chord.h"
#include "shift/core/forcemodule/Base.h"
#include "shift/core/geometry/Base.h"
#include "shift/core/integrator/Base.h"
#include "shift/core/joint/Base.h"
#include "shift/core/joint/Rigid.h"
#include "shift/core/light/Base.h"
#include "shift/core/logger/Base.h"
#include "shift/core/sensor/Base.h"
#include "shift/core/solver/Base.h"
#include "shift/core/terrain/Base.h"

#include "shift/core/log.h"

#include "ksl/print.h"
#include "ksl/screw.h"
#include "ksl/trig.h"

void shift::core::Context::DeclareParameters() {

  DeclareParameter("model_parameters", &model_parameters_,
                   "Parameters associated with configuring a Shift simulation.",
                   true);

  DeclareParameter(
    "assemblies", &assemblies_,
    "an array of Shift assemblies, where each assembly contains a list of "
    "bodies, joints, force modules, behaviors, etc.; Each assembly at a given "
    "level must have a unique name identifier",
    true);
}

void shift::core::Context::LoadJSON(const std::string filename) {
  filename_ = filename;
  std::ifstream file(filename);
  file >> root_;
  ReadJSON(root_);
  set_parent(this);
  PostLoad(filename);
}

void shift::core::Context::PostLoad(const std::string) {

  /* process topology */
  ActivateObjects(true);
  Connect();
  LevelOrderSort();
  ProcessVirtualBodies();
  ProcessPCL();
  ProcessLoopMembers();
  DetectSubstructures();
  ProcessPCL();
  ProcessFrames();
  AllocateRunTimeArrays();
  SetupFixedFrameGravityVector();
  SetToInitialConditions();
  UpdateD();
  UpdateDH();
  UpdateDHVA();
  SetupLookup();
  jacobian_.Initialize();
  InitializeMobility();
  InitializeInertia();
  SetupQuaternionMap();

  // SetupDReferenceFrames();
  // InitializeGLReferenceFrames();

  LoadData();

  // InitializeJointForces();
  // FlattenForceModules();
  for(auto& logger : active_loggers_) {
    logger->Initialize();
  }
}

void shift::core::Context::ProcessFrames() {

  /* process active frames */
  active_frames_.clear();
  auto fixed = get_fixed_body();
  for(auto& frame : fixed->get_frames()) {
    assert(frame.get());
    active_frames_.push_back(frame.get());
  }
  for(auto& body : active_articulated_bodies_) {
    for(auto& frame : body->get_frames()) {
      if(frame->is_active()) {
        active_frames_.push_back(frame.get());
      }
    }
  }

  /* identify frames in spanning tree */
  frames_in_tree_.clear();
  get_fixed_body()->ClearInTreeFlags();
  for(auto& body : active_articulated_bodies_) {
    body->ClearInTreeFlags();
  }
  for(auto& frame : fixed->get_frames()) {
    frames_in_tree_.push_back(frame.get());
  }

  /* identify frames attached to non-rigid, active joints */
  for(auto& joint : active_joints_) {
    /* parent and child frames of a joint are only added to spanning tree if
     * joint has nonzero degrees of freedom. Rigid joints
     * are optimized out of the spanning tree */
    if(joint->get_q_count()) {
      auto parent_frame = joint->get_system_parent_frame();
      assert(parent_frame);
      if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(),
                   parent_frame) == frames_in_tree_.end()) {
        frames_in_tree_.push_back(parent_frame);
      }
      auto child_frame = joint->get_system_child_frame();
      assert(child_frame);
      if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(),
                   child_frame) == frames_in_tree_.end()) {
        frames_in_tree_.push_back(child_frame);
      }
    }
  }

  /* identify frames attached to chord constraints */
  for(auto& constraint : chord_constraints_) {
    auto parent_frame = constraint->get_parent_frame();
    if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(),
                 parent_frame) == frames_in_tree_.end()) {
      frames_in_tree_.push_back(parent_frame);
    }

    auto child_frame = constraint->get_child_frame();
    if(std::find(frames_in_tree_.begin(), frames_in_tree_.end(), child_frame) ==
       frames_in_tree_.end()) {
      frames_in_tree_.push_back(child_frame);
    }
  }

  unsigned int i = 0;
  for(auto& frame : frames_in_tree_) {
    assert(frame);
    frame->set_index(i++);
    frame->set_in_tree(true);
  }

  get_fixed_body()->SetupReferenceFrame();
  for(auto& joint : active_joints_) {
    auto body = joint->get_system_child_body();
    body->set_parent_joint(joint);
  }

  for(auto& body : active_articulated_bodies_) {
    for(auto& frame : body->get_frames()) {
      frame->set_parent_body(body);
      assert(frame->get_parent_body());
    }
  }
}

void shift::core::Context::WriteJSON() {
  WriteJSON(filename_);
}

void shift::core::Context::WriteJSON(const std::string& filename) {
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

void shift::core::Context::Initialize() {

  shift::core::log::SetupLogFile();

  if(!shift::core::behavior::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Behavior modules did not properly initialize");
  }
  if(!shift::core::body::Modules::is_initialized()) {
    throw shift::core::Exception("Body modules did not properly initialize");
  }
  if(!shift::core::collisiongeometry::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Collision geometry modules did not properly initialize");
  }
  if(!shift::core::constraint::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Constraint modules did not properly initialize");
  }
  if(!shift::core::forcemodule::Modules::is_initialized()) {
    throw shift::core::Exception("Force modules did not properly initialize");
  }
  if(!shift::core::geometry::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Geometry modules did not properly initialize");
  }
  if(!shift::core::integrator::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Integrator modules did not properly initialize");
  }
  if(!shift::core::joint::Modules::is_initialized()) {
    throw shift::core::Exception("Joint modules did not properly initialize");
  }
  if(!shift::core::light::Modules::is_initialized()) {
    throw shift::core::Exception("Light modules did not properly initialize");
  }
  if(!shift::core::logger::Modules::is_initialized()) {
    throw shift::core::Exception("Logger modules did not properly initialize");
  }
  if(!shift::core::sensor::Modules::is_initialized()) {
    throw shift::core::Exception("Sensor modules did not properly initialize");
  }
  if(!shift::core::solver::Modules::is_initialized()) {
    throw shift::core::Exception("Solver modules did not properly initialize");
  }
  if(!shift::core::terrain::Modules::is_initialized()) {
    throw shift::core::Exception("Terrain modules did not properly initialize");
  }
}

void shift::core::Context::ActivateObjects(bool) {
  model_parameters_.ActivateObjects(true);
  assemblies_.ActivateObjects(true);
}

void shift::core::Context::AppendActiveObject(
  shift::core::integrator::Base* integrator) {
  if(integrator->is_active()) {
    for(auto i : active_integrators_) {
      if(i == integrator) {
        return;
      }
    }
    active_integrators_.push_back(integrator);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::logger::Base* logger) {
  if(logger->is_active()) {
    for(auto i : active_loggers_) {
      if(i == logger) {
        return;
      }
    }
    active_loggers_.push_back(logger);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::solver::Base* solver) {
  if(solver->is_active()) {
    for(auto i : active_solvers_) {
      if(i == solver) {
        return;
      }
    }
    active_solvers_.push_back(solver);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::behavior::Base* behavior) {
  if(behavior->is_active()) {
    for(auto i : active_behaviors_) {
      if(i == behavior) {
        return;
      }
    }
    active_behaviors_.push_back(behavior);
  }
}

void shift::core::Context::AppendActiveObject(shift::core::body::Base* body) {
  if(body && body->is_active()) {
    if(body->is_articulated()) {
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

    //    for(auto& geometry : (*body)["geometry"]) {
    //
    //    }
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::collisiongeometry::Base* geometry) {
  if(geometry->is_active()) {
    for(auto i : active_collision_geometry_) {
      if(i == geometry) {
        return;
      }
    }
    active_collision_geometry_.push_back(geometry);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::constraint::Base* constraint) {
  if(constraint->is_active()) {
    for(auto i : active_constraints_) {
      if(i == constraint) {
        return;
      }
    }
    active_constraints_.push_back(constraint);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::forcemodule::Base* module) {
  if(module->is_active()) {
    for(auto i : active_force_modules_) {
      if(i == module) {
        return;
      }
    }
    active_force_modules_.push_back(module);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::geometry::Base* geometry) {
  if(geometry->is_active()) {
    for(auto i : active_geometry_) {
      if(i == geometry) {
        return;
      }
    }
    active_geometry_.push_back(geometry);
  }
}

void shift::core::Context::AppendActiveObject(shift::core::joint::Base* joint) {
  if(joint->is_active()) {
    for(auto i : active_joints_) {
      if(i == joint) {
        return;
      }
    }
    active_joints_.push_back(joint);
  }
}

void shift::core::Context::AppendActiveObject(shift::core::light::Base* light) {
  if(light->is_active()) {
    for(auto i : active_lights_) {
      if(i == light) {
        return;
      }
    }
    active_lights_.push_back(light);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::sensor::Base* sensor) {
  if(sensor->is_active()) {
    for(auto i : active_sensors_) {
      if(i == sensor) {
        return;
      }
    }
    active_sensors_.push_back(sensor);
  }
}

void shift::core::Context::AppendActiveObject(
  shift::core::terrain::Base* terrain) {
  if(terrain->is_active()) {
    for(auto i : active_terrain_) {
      if(i == terrain) {
        return;
      }
    }
    active_terrain_.push_back(terrain);
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::logger::Base* logger) {
  for(unsigned int i = 0; i < active_loggers_.size(); i++) {
    if(active_loggers_[i] == logger) {
      active_loggers_.erase(active_loggers_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::integrator::Base* integrator) {
  for(unsigned int i = 0; i < active_integrators_.size(); i++) {
    if(active_integrators_[i] == integrator) {
      active_integrators_.erase(active_integrators_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::solver::Base* solver) {
  for(unsigned int i = 0; i < active_solvers_.size(); i++) {
    if(active_solvers_[i] == solver) {
      active_solvers_.erase(active_solvers_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::behavior::Base* behavior) {
  for(unsigned int i = 0; i < active_behaviors_.size(); i++) {
    if(active_behaviors_[i] == behavior) {
      active_behaviors_.erase(active_behaviors_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(shift::core::body::Base* body) {
  if(body->is_articulated()) {
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

void shift::core::Context::RemoveActiveObject(
  shift::core::collisiongeometry::Base* geometry) {
  for(unsigned int i = 0; i < active_collision_geometry_.size(); i++) {
    if(active_collision_geometry_[i] == geometry) {
      active_collision_geometry_.erase(active_collision_geometry_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::constraint::Base* constraint) {
  for(unsigned int i = 0; i < active_constraints_.size(); i++) {
    if(active_constraints_[i] == constraint) {
      active_constraints_.erase(active_constraints_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::forcemodule::Base* module) {
  for(unsigned int i = 0; i < active_force_modules_.size(); i++) {
    if(active_force_modules_[i] == module) {
      active_force_modules_.erase(active_force_modules_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::geometry::Base* geometry) {
  for(unsigned int i = 0; i < active_geometry_.size(); i++) {
    if(active_geometry_[i] == geometry) {
      active_geometry_.erase(active_geometry_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(shift::core::joint::Base* joint) {
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    if(active_joints_[i] == joint) {
      active_joints_.erase(active_joints_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(shift::core::light::Base* light) {
  for(unsigned int i = 0; i < active_lights_.size(); i++) {
    if(active_lights_[i] == light) {
      active_lights_.erase(active_lights_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::sensor::Base* sensor) {
  for(unsigned int i = 0; i < active_sensors_.size(); i++) {
    if(active_sensors_[i] == sensor) {
      active_sensors_.erase(active_sensors_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::RemoveActiveObject(
  shift::core::terrain::Base* terrain) {
  for(unsigned int i = 0; i < active_terrain_.size(); i++) {
    if(active_terrain_[i] == terrain) {
      active_terrain_.erase(active_terrain_.begin() + i);
      return;
    }
  }
}

void shift::core::Context::ActivateObject(shift::core::behavior::Base* behavior,
                                          bool active) {
  if(behavior) {
    active ? AppendActiveObject(behavior) : RemoveActiveObject(behavior);
  }
}

void shift::core::Context::ActivateObject(shift::core::body::Base* body,
                                          bool active) {
  if(body) {
    active ? AppendActiveObject(body) : RemoveActiveObject(body);
  }
}

void shift::core::Context::ActivateObject(
  shift::core::collisiongeometry::Base* geometry, bool active) {
  if(geometry) {
    active ? AppendActiveObject(geometry) : RemoveActiveObject(geometry);
  }
}

void shift::core::Context::ActivateObject(
  shift::core::constraint::Base* constraint, bool active) {
  if(constraint) {
    active ? AppendActiveObject(constraint) : RemoveActiveObject(constraint);
  }
}

void shift::core::Context::ActivateObject(
  shift::core::forcemodule::Base* module, bool active) {
  if(module) {
    active ? AppendActiveObject(module) : RemoveActiveObject(module);
  }
}

void shift::core::Context::ActivateObject(shift::core::geometry::Base* geometry,
                                          bool active) {
  if(geometry) {
    active ? AppendActiveObject(geometry) : RemoveActiveObject(geometry);
  }
}

void shift::core::Context::ActivateObject(shift::core::joint::Base* joint,
                                          bool active) {
  if(joint) {
    active ? AppendActiveObject(joint) : RemoveActiveObject(joint);
  }
}

void shift::core::Context::ActivateObject(shift::core::light::Base* light,
                                          bool active) {
  if(light) {
    active ? AppendActiveObject(light) : RemoveActiveObject(light);
  }
}

void shift::core::Context::ActivateObject(shift::core::sensor::Base* sensor,
                                          bool active) {
  if(sensor) {
    active ? AppendActiveObject(sensor) : RemoveActiveObject(sensor);
  }
}

void shift::core::Context::ActivateObject(shift::core::terrain::Base* terrain,
                                          bool active) {
  if(terrain) {
    active ? AppendActiveObject(terrain) : RemoveActiveObject(terrain);
  }
}

void shift::core::Context::ActivateObject(shift::core::logger::Base* logger,
                                          bool active) {
  if(logger) {
    active ? AppendActiveObject(logger) : RemoveActiveObject(logger);
  }
}

void shift::core::Context::ActivateObject(
  shift::core::integrator::Base* integrator, bool active) {
  if(integrator) {
    active ? AppendActiveObject(integrator) : RemoveActiveObject(integrator);
  }
}

void shift::core::Context::ActivateObject(shift::core::solver::Base* solver,
                                          bool active) {
  if(solver) {
    active ? AppendActiveObject(solver) : RemoveActiveObject(solver);
  }
}

void shift::core::Context::ClearActiveBehavior() {
  active_behaviors_.clear();
}

void shift::core::Context::ClearActiveArticulatedBodies() {
  active_articulated_bodies_.clear();
}

void shift::core::Context::ClearActiveFreeBodies() {
  active_free_bodies_.clear();
}

void shift::core::Context::ClearActiveCollisionGeometry() {
  active_collision_geometry_.clear();
}

void shift::core::Context::ClearActiveConstraints() {
  active_constraints_.clear();
}

void shift::core::Context::ClearActiveforcemodules() {
  active_force_modules_.clear();
}

void shift::core::Context::ClearActiveGeometry() {
  active_geometry_.clear();
}

void shift::core::Context::ClearActiveJoints() {
  active_joints_.clear();
}

void shift::core::Context::ClearActiveLights() {
  active_lights_.clear();
}

void shift::core::Context::ClearActiveSensors() {
  active_sensors_.clear();
}

void shift::core::Context::ClearActiveTerrain() {
  active_terrain_.clear();
}

void shift::core::Context::ClearActiveArrays() {
  ClearActiveBehavior();
  ClearActiveArticulatedBodies();
  ClearActiveFreeBodies();
  ClearActiveCollisionGeometry();
  ClearActiveConstraints();
  ClearActiveforcemodules();
  ClearActiveGeometry();
  ClearActiveJoints();
  ClearActiveLights();
  ClearActiveSensors();
  ClearActiveTerrain();
}

void shift::core::Context::Connect() {
  for(auto& behavior : active_behaviors_) {
    behavior->Connect();
  }
  for(auto& body : active_articulated_bodies_) {
    body->Connect();
  }
  for(auto& geometry : active_collision_geometry_) {
    geometry->Connect();
  }
  for(auto& constraint : active_constraints_) {
    constraint->Connect();
  }
  for(auto& module : active_force_modules_) {
    module->Connect();
  }
  for(auto& geometry : active_geometry_) {
    geometry->Connect();
  }
  for(auto& joint : active_joints_) {
    joint->Connect();
  }
  for(auto& light : active_lights_) {
    light->Connect();
  }
  for(auto& sensor : active_sensors_) {
    sensor->Connect();
  }
  for(auto& terrain : active_terrain_) {
    terrain->Connect();
  }
  for(auto& logger : active_loggers_) {
    logger->Connect();
  }
  if(active_integrators_.size() == 0) {
    throw shift::core::Exception(
      "No integrators were specified in the model input file");
  }
  if(active_solvers_.size() == 0) {
    throw shift::core::Exception(
      "No solvers were specified in the model input file");
  }
  for(auto& integrator : active_integrators_) {
    integrator->Connect();
  }
  for(auto& solver : active_solvers_) {
    solver->Connect();
  }
}

void shift::core::Context::LoadData() {
  for(auto& behavior : active_behaviors_) {
    behavior->Load();
  }
  for(auto& body : active_articulated_bodies_) {
    body->Load();
  }
  for(auto& collision_geometry : active_collision_geometry_) {
    collision_geometry->Load();
  }
  for(auto& constraint : active_constraints_) {
    constraint->Load();
  }
  for(auto& force_module : active_force_modules_) {
    force_module->Load();
  }
  for(auto& geometry : active_geometry_) {
    geometry->Load();
  }
  for(auto& joint : active_joints_) {
    joint->Load();
  }
  for(auto& light : active_lights_) {
    light->Load();
  }
  for(auto& sensor : active_sensors_) {
    sensor->Load();
  }
  for(auto& terrain : active_terrain_) {
    terrain->Load();
  }
}

void shift::core::Context::LevelOrderSort() {

  std::queue<shift::core::joint::Base*> queue;
  std::vector<shift::core::joint::Base*> joints;
  std::vector<shift::core::body::Base*> bodies;

  /* find joints attached to fixed body */
  auto it = active_joints_.begin();
  while(it != active_joints_.end()) {
    assert((*it)->get_parent_body() != nullptr);
    assert((*it)->get_child_body() != nullptr);
    if((*it)->get_parent_body() == get_fixed_body()) {
      (*it)->set_sense(1);
      (*it)->set_level(1);
      queue.push(*it);
      it = active_joints_.erase(it);
    } else if((*it)->get_child_body() == get_fixed_body()) {
      (*it)->set_sense(-1);
      (*it)->set_level(1);
      queue.push(*it);
      it = active_joints_.erase(it);
    } else {
      it++;
    }
  }

  while(!queue.empty()) {
    auto joint = queue.front();
    queue.pop();
    shift::core::body::Base* body = nullptr;
    if(joint->get_sense() == 1) {
      body = joint->get_child_body();
    } else if(joint->get_sense() == -1) {
      body = joint->get_parent_body();
    }
    joints.push_back(joint);
    assert(body);

    /* find any joints attached to joint's child body */
    it = active_joints_.begin();
    while(it != active_joints_.end()) {
      if((*it)->get_parent_body() == body) {
        (*it)->set_sense(1);
        (*it)->set_level(joint->get_level() + 1);
        queue.push(*it);
        it = active_joints_.erase(it);
      } else if((*it)->get_child_body() == body) {
        (*it)->set_sense(-1);
        (*it)->set_level(joint->get_level() + 1);
        queue.push(*it);
        it = active_joints_.erase(it);
      } else {
        it++;
      }
    }
  }

  active_joints_ = joints;
}

void shift::core::Context::ProcessVirtualBodies() {

  std::vector<shift::core::body::Base*> bodies(active_joints_.size());

  // clear system child body and frame pointers
  for(auto& joint : active_joints_) {
    joint->set_system_child_body(nullptr);
    joint->set_system_child_frame(nullptr);
  }

  chord_constraints_.clear();
  int j = 0;
  for(unsigned int i = 0; i < active_joints_.size(); i++) {

    bool found = false;
    auto body = active_joints_[i]->get_system_child_body();
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
        shift::core::Factory<shift::core::constraint::Base>::get()->Create(
          "chord");

      shift::core::constraint::Chord* chord_constraint =
        dynamic_cast<shift::core::constraint::Chord*>(constraint.get());

      auto virtual_body =
        shift::core::Factory<shift::core::body::Base>::get()->Create("virtual");
      virtual_body->set_name("_virtual_body[" + std::to_string(j++) + "]");

      chord_constraint->set_parent_body(virtual_body.get());
      chord_constraint->set_parent_frame(virtual_body->get_origin());

      auto chord_joint = active_joints_[i];
      chord_constraint->set_child_body(chord_joint->get_system_child_body());
      chord_constraint->set_child_frame(chord_joint->get_system_child_frame());
      chord_constraint->set_original_parent_body(
        chord_joint->get_system_parent_body());
      chord_constraint->set_original_parent_frame(
        chord_joint->get_system_parent_frame());

      /* add virtual bodies and chord constraints to active lists */
      bodies[i] = virtual_body.get();

      active_constraints_.push_back(constraint.get());
      chord_constraints_.push_back(chord_constraint);

      active_joints_[i]->set_system_child_body(virtual_body.get());
      active_joints_[i]->set_system_child_frame(
        virtual_body.get()->get_origin());

      /* transfer ownership of smart pointers for virtual bodies and chord
       * constraints to `Context` data structure */
      bodies_.push_back(std::move(virtual_body));
      constraints_.push_back(std::move(constraint));
    }
  }

  active_articulated_bodies_ = bodies;
}

void shift::core::Context::ProcessPCL() {
  for(unsigned int i = 0; i < active_articulated_bodies_.size(); i++) {
    active_articulated_bodies_[i]->set_index(i);
  }
  pcl_.clear();
  pcl_.resize(active_joints_.size());
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    auto joint = active_joints_[i];
    auto parent_body = joint->get_system_parent_body();
    if(parent_body) {
      pcl_[i] = parent_body->get_index();
    } else {
      throw shift::core::Exception("Failed to find parent body of joint: " +
                                   joint->get_name());
    }
  }


#if SHIFT_LOG_LEVEL > 4
  {
    std::stringstream ss;
    ss << "pcl: ";
    for(auto i : pcl_) {
      ss << i << " ";
    }
    SHIFT_TRACE(ss.str());
  }
#endif
}

void shift::core::Context::ProcessLoopMembers() {
  std::vector<std::vector<int>> loop_members(chord_constraints_.size());
  std::vector<std::vector<int>> negative_loop_members(
    chord_constraints_.size());
  std::vector<std::vector<int>> positive_loop_members(
    chord_constraints_.size());
  for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
    loop_members[i].clear();
    negative_loop_members[i].clear();
    positive_loop_members[i].clear();

    if(!chord_constraints_[i]->get_parent_body()) {
      throw shift::core::Exception("no parent body for chord constraint: " +
                                   std::to_string(i));
    }

    if(!chord_constraints_[i]->get_child_body()) {
      throw shift::core::Exception("no child body for chord constraint: " +
                                   std::to_string(i));
    }

    int positive = chord_constraints_[i]->get_parent_body()->get_index();
    int negative = chord_constraints_[i]->get_child_body()->get_index();

    SHIFT_TRACE("Procesing chord constranint " + std::to_string(i) + 
      " with parent body: '" +
      chord_constraints_[i]->get_parent_body()->get_name() +
      "' and child body: '" +
      chord_constraints_[i]->get_child_body()->get_name() + "'");

    loop_members[i].push_back(negative);
    negative_loop_members[i].push_back(negative);
    loop_members[i].push_back(positive);
    positive_loop_members[i].push_back(positive);

    while(active_joints_[positive]->get_level() >
          active_joints_[negative]->get_level()) {
      positive = pcl_[positive];
      loop_members[i].push_back(positive);
      positive_loop_members[i].push_back(positive);
    }
    while(active_joints_[negative]->get_level() >
          active_joints_[positive]->get_level()) {
      negative = pcl_[negative];
      loop_members[i].push_back(negative);
      negative_loop_members[i].push_back(negative);
    }
    while(positive != negative) {
      negative = pcl_[negative];
      positive = pcl_[positive];
      loop_members[i].push_back(positive);
      positive_loop_members[i].push_back(positive);
      loop_members[i].push_back(negative);
      negative_loop_members[i].push_back(negative);
    }
    loop_members[i].pop_back();
    negative_loop_members[i].pop_back();
    positive_loop_members[i].pop_back();
  }

  /* sort loop members in monotonically increasing level order */
  for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
    std::sort(loop_members[i].begin(), loop_members[i].end(), LevelOrder(this));
  }
  loop_members_ = loop_members;

  /* sort positive and negative loop member lists to follow master loop member
   * list ordering */
  negative_loop_members_.clear();
  positive_loop_members_.clear();
  negative_loop_members_.resize(chord_constraints_.size());
  positive_loop_members_.resize(chord_constraints_.size());
  for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
    negative_loop_members_[i].clear();
    for(auto j : loop_members_[i]) {
      if(std::find(negative_loop_members[i].begin(),
                   negative_loop_members[i].end(),
                   j) != negative_loop_members[i].end()) {
        negative_loop_members_[i].push_back(j);
      }
      if(std::find(positive_loop_members[i].begin(),
                   positive_loop_members[i].end(),
                   j) != positive_loop_members[i].end()) {
        positive_loop_members_[i].push_back(j);
      }
    }
  }
}

void shift::core::Context::DetectSubstructures() {

  SHIFT_TRACE("Merging chord loops into substructures");

  if(chord_constraints_.size() > 0) {

    /* contains chord constraint indices */
    std::vector<std::vector<int>> substructures;

    /* create stack of all chord_constraint indices */
    std::list<int> chord_constraint_indices;
    for(unsigned int i = 0; i < chord_constraints_.size(); i++) {
      chord_constraint_indices.push_back(i);
    }

    /* p_01 is used to permute chord loops here */
    std::vector<int> p_01(chord_constraints_.size());
    std::vector<int> p_10(chord_constraints_.size());
    int current_loop_index = 0;

    while(!chord_constraint_indices.empty()) {
      std::vector<int> substructure;
      int chord_constraint_index = chord_constraint_indices.front();
      chord_constraint_indices.pop_front();

      substructure.push_back(chord_constraint_index);
      p_01[current_loop_index++] = chord_constraint_index;

      /* append first substructure to list of substructures */
      auto it = chord_constraint_indices.begin();
      int merge_count = 1;
      while(merge_count != 0) {
        merge_count = 0;
        while(it != chord_constraint_indices.end()) {
          /* base node of substructure */
          int base_node = loop_members_[substructure[0]][0];
          if(shift::core::topology::InPath(pcl_, loop_members_[(*it)][0],
                                           base_node)) {
            for(auto loop_index : substructure) {
              for(auto loop_member = loop_members_[loop_index].begin() + 1;
                  loop_member < loop_members_[loop_index].end(); ++loop_member) {
                for(auto merge_candidate_member =
                      loop_members_[(*it)].begin() + 1;
                    merge_candidate_member < loop_members_[(*it)].end();
                    ++merge_candidate_member) {
                  if(*loop_member == *merge_candidate_member) {
                    substructure.push_back(*it);
                    p_01[current_loop_index++] = (*it);
                    merge_count++;
                    it = chord_constraint_indices.erase(it);
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

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "loop members: \n";
      int k = 0;
      for(auto& loop : loop_members_) {
        ss << "  [" << k++ << "]: ";
        for(auto& member : loop) {
          if(member < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[member]->get_name()[0] << " ";
          }
        } 
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "Substructure chord loop members:\n";
      int k = 0;
      for(auto sub : substructures) {
        ss << "  substructure [" << k++ << "]: ";
        for(auto i : sub) {
          ss << i << " ";
        }
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    // sort loops of each substructure in monotonically increasing level order
    for(auto& substructure : substructures) {
      std::sort(substructure.begin(), substructure.end(),
                SubstructureLoopLevelOrder(this, loop_members_));
    }

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "Substructure members after sorting loops in monotonically " <<
        "level increasing order:\n"; 
      int k = 0;
      for(auto sub : substructures) {
        ss << "  substructure [" << k++ << "]: ";
        for(auto i : sub) {
          ss << i << " ";
        }
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    // sort substructures in monotonically level increasing order
    std::sort(substructures.begin(), substructures.end(),
              SubstructureLevelOrder(this, loop_members_));

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "Substructures after sorting substructures in monotonically " <<
        "level increasing order:\n";
      int k = 0;
      for(auto& sub : substructures) {
        ss << "  substructure [" << k++ << "]: ";
        for(auto& i : sub) {
          ss << i << " ";
        }
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    substructure_indices_ = substructures;

    substructure_members_.clear();
    substructure_members_.resize(substructures.size());
    for(unsigned int i = 0; i < substructures.size(); i++) {
      for(auto loop_index : substructures[i]) {
        for(auto loop_member : loop_members_[loop_index]) {
          substructure_members_[i].push_back(loop_member);
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
                substructure_members_[i].end(), LevelOrder(this));
    }

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "Substructure members:\n";
      int k = 0;
      for(auto& sub : substructure_members_) {
        ss << "  [" << k++ << "]: ";
        for(auto& i : sub) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        } 
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    /* reorder loops */

    /* reorder loop members to follow substructure ordering */
    std::vector<std::vector<int>> loop_members;
    std::vector<std::vector<int>> negative_loop_members;
    std::vector<std::vector<int>> positive_loop_members;
    std::vector<shift::core::constraint::Chord*> chord_constraints;

    for(auto substructure : substructures) {
      for(auto loop : substructure) {
        loop_members.push_back(loop_members_[loop]);
        negative_loop_members.push_back(negative_loop_members_[loop]);
        positive_loop_members.push_back(positive_loop_members_[loop]);
        chord_constraints.push_back(chord_constraints_[loop]);
      }
    }

    loop_members_ = loop_members;
    negative_loop_members_ = negative_loop_members;
    positive_loop_members_ = positive_loop_members;
    chord_constraints_ = chord_constraints;

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "Loop members:\n";
      ss << "  all loop members:\n";
      int k = 0;
      for(auto& loop : loop_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : loop) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        } 
        ss << std::endl;
      }

      ss << "  negative loop members:\n";
      k = 0;
      for(auto& loop : negative_loop_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : loop) {
          ss << active_joints_[i]->get_name()[0] << " ";
        } 
        ss << std::endl;
      }

      ss << "  positive loop members:\n";
      k = 0;
      for(auto& loop : positive_loop_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : loop) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        } 
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    /* substructure body joint reorder */
    // sort substructures in monotonically increasing level order by base node
    // reorder bodies and joints according to substructure assignments
    // create a short list of indices that are either not in any
    // substructure or are the head of a substructure; the short list
    // may contain the fixed body
    std::list<int> short_list;
    for(int i = -1; i < (int)active_joints_.size(); i++) {
      if(shift::core::topology::IsSubstructureHead(i, substructure_members_) ||
         !shift::core::topology::IsSubstructureMember(i,
                                                      substructure_members_)) {
        short_list.push_back(i);
      }
    }

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "short list: ";
      for(auto& i : short_list) {
        if(i < 0) {
          ss << "fixed ";
        } else {
          ss << active_joints_[i]->get_name()[0] << " "; 
        }
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    /* expand short list */
    std::list<int> expanded_list;
    unsigned int j = 0;

    while(!short_list.empty()) {
      int value = short_list.front();
      short_list.pop_front();

      if(std::find(expanded_list.begin(), expanded_list.end(), value) ==
         expanded_list.end()) {
        expanded_list.push_back(value);
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
          if(std::find(expanded_list.begin(), expanded_list.end(),
                       substructure_members_[j][k]) == expanded_list.end()) {
            expanded_list.push_back(substructure_members_[j][k]);
          }
        }
        j++;
      }
    }

    std::vector<int> expanded_array{std::begin(expanded_list),
                                    std::end(expanded_list)};

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "expanded array: ";
      for(auto& i : expanded_array) {
        if(i < 0) {
          ss << "fixed ";
        } else {
          ss << active_joints_[i]->get_name()[0] << " "; 
        }
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    /* verify that all joints were added to the expanded array */
    bool found = true;
    for(unsigned int i = 0; i < active_joints_.size(); i++) {
      found = false;
      for(auto j : expanded_array) {
        if((int)i == j) {
          found = true;
        }
      }
      if(!found) {
        SHIFT_FATAL(active_articulated_bodies_[i]->get_name() + " not found!");
      }
    }
    if(!found) {
      throw shift::core::Exception("Articulated body not found");
    }

    /* reorder bodies, joints and levels */
    std::vector<shift::core::joint::Base*> active_joints(active_joints_.size());
    std::vector<shift::core::body::Base*> active_bodies(
      active_articulated_bodies_.size());

    /* initialize permutation arrays to permute bodies and joints from order
     * originally loaded (p0) to order dictated by kinematic substructuring
     * algorithm, previous p_01 was used to permute chord constraints, here 
     * we resize the vector to allow resuing it to permute bodies and joints */
    p_01.clear();
    p_01.resize(active_joints_.size());
    p_10.clear();
    p_10.resize(active_joints_.size());

    j = 0;
    for(auto& i : expanded_array) {
      if(i != -1) {
        p_01[j] = i;
        active_joints[j] = active_joints_[i];
        active_bodies[j] = active_articulated_bodies_[i];
        active_bodies[j]->set_index(j);
        j++;
      }
    }
    active_joints_ = active_joints;
    active_articulated_bodies_ = active_bodies;

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "reordered joints: ";
      for(auto& joint : active_joints_) {
        ss << joint->get_name()[0] << " ";
      }
      SHIFT_TRACE(ss.str());
    }
#endif
    
    shift::core::topology::SetInversePermutation(p_01, p_10);

    // convert loop members from permutation order 0 to 1
    for(auto& loop : loop_members_) {
      for(auto& i : loop) {
        if(i > -1) {
          i = p_10[i];
        }
      }
    }

    for(auto& loop : negative_loop_members_) {
      for(auto& i : loop) {
        if(i > -1) {
          i = p_10[i];
        }
      }
    }

    for(auto& loop : positive_loop_members_) {
      for(auto& i : loop) {
        if(i > -1) {
          i = p_10[i];
        }
      }
    }

    for(auto& substructure : substructure_members_) {
      for(auto& i : substructure) {
        if(i > -1) {
          i = p_10[i];
        }
      }
    }

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss; 
      ss << "active joints after sorting: ";
      for(auto& joint : active_joints_) {
        ss << joint->get_name()[0] << " ";
      }
      SHIFT_TRACE(ss.str());
    }
#endif

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "substructures:\n";
      ss << "  all loops:\n";
      int k = 0;
      for(auto& loop : loop_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : loop) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        }
        ss << std::endl;
      }

      ss << "  negative loops:\n";
      k = 0;
      for(auto& loop : negative_loop_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : loop) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        }
        ss << std::endl;
      }

      ss << "  positive loops:\n"; 
      k = 0;
      for(auto& loop : positive_loop_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : loop) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        }
        ss << std::endl;
      }

      ss << "  substructures:\n";
      k = 0;
      for(auto& substructure : substructure_members_) {
        ss << "    [" << k++ << "]: ";
        for(auto& i : substructure) {
          if(i < 0) {
            ss << "fixed ";
          } else {
            ss << active_joints_[i]->get_name()[0] << " ";
          }
        }
        ss << std::endl;
      }
      SHIFT_TRACE(ss.str());
    }
#endif

  }
}

unsigned int shift::core::Context::get_q_count() {
  return q_count_;
}

unsigned int shift::core::Context::get_dq_count() {
  return dq_count_;
}

/*!
\brief compress any frames on child body attached to parent body by
series of one or more rigid joints
*/
void shift::core::Context::CompressRigidJoints(
  int i, // current body Index and parent joint index
  unsigned int& operation_count) {

  ksl_SE3_t D;
  ksl_SE3_setIdentity(&D);

  int j = i;
  auto parent_joint = active_joints_[i];

  // D_p_01: transformation from proximal frame 0 to distal frame 1 on parent
  // body p
  // D_pc: relative transformation across rigid joint
  // D_c_01: transformation from proximal frame 0 to distal frame 1 on child
  // body c

  while(!parent_joint->get_q_count()) { // rigid joint
    // D = D_p1_c0 * D
    ksl_SE3_t temp;
    ksl_SE3_t D_p1_c0 = parent_joint->get_transform();
    // ksl_SE3_print(stdout, &D_p1_c0, "  D_p1_c0: ");
    ksl_product_dd(&D_p1_c0, &D, &temp);
    ksl_SE3_copy(&temp, &D);

    // D = D_p0_p1 * D_p1_c0
    // D <- D_p0_c0
    ksl_product_dd(
      parent_joint->get_system_parent_frame()->get_d_wrt_reference(), &D,
      &temp);
    ksl_SE3_copy(&temp, &D);

    j = pcl_[j];
    if(j == -1) {
      break;
    }
    parent_joint = active_joints_[j];
  }

  auto joint = active_joints_[i];
  if(!joint->get_q_count()) {
    // std::cout << "processing joint: " << joint->get_name() << std::endl;
    auto parent_body = joint->get_system_parent_body();
    auto child_body = active_articulated_bodies_[i];

    // ksl_SE3_print(stdout, &D, "  D_p0_c0: ");

    for(auto& frame : child_body->get_frames()) {
      // std::cout << child_body->get_name() << ": " << frame.name() <<
      // std::endl;

      // d_wrt_reference = d_wrt_reference * d_wrt_reference_child
      ksl_SE3_t D_p0_c1;
      ksl_product_dd(&D, frame->get_d_wrt_reference(), &D_p0_c1);

      // ksl_SE3_print(stdout, &D_p0_c1, "  D_p0_c1: ");

      if(frame->is_in_tree() && parent_body != get_fixed_body()) {
        // std::cout << "intree && parent_body!= fixedBody: processing frame: "
        //           << child_body->get_name() << ": " << frame.name() <<
        //           std::endl;

        ksl_SE3_copy(&D_p0_c1, &d_wrt_reference_[frame->get_index()]);

        dhva_offsets_[operation_count].parent.d =
          parent_joint->get_system_parent_body()
            ->get_reference_frame()
            ->get_index();
        dhva_offsets_[operation_count].child.d = frame->get_index();
        dhva_offsets_[operation_count].q = frame->get_index();

        d_fptr_[operation_count] = shift::core::joint::rigid_dh;
        dh_fptr_[operation_count] = shift::core::joint::rigid_dh;
        dhva_fptr_[operation_count] = shift::core::joint::rigid_dhva;
        operation_count++;
      } else if(frame->is_in_tree()) {
        // std::cout << "processing frame: " << child_body->get_name() << ": "
        //           << frame.name() << std::endl;

        // ksl_SE3_print(stdout, frame->get_d_wrt_reference(), "d_wrt_reference:
        // ");
        ksl_SE3_copy(&D_p0_c1, &d_wrt_reference_[frame->get_index()]);

        // ksl_SE3_print(
        //   stdout,
        //   dhva_.d +
        //   parent_joint->get_system_parent_body()->get_reference_frame()->get_index(),
        //   " d_parent: ");

        ksl_product_dd(dhva_.d + parent_joint->get_system_parent_body()
                                   ->get_reference_frame()
                                   ->get_index(),
                       &D_p0_c1, dhva_.d + frame->get_index());
        // ksl_SE3_print(stdout, dhva_.d + frame->get_index(), "  d_absolute: ");
      }
    }
  }
}

void shift::core::Context::AllocateRunTimeArrays() {
  unsigned int dc_count = 0;
  q_count_ = 0;
  dq_count_ = 0;
  unsigned int data_count = 0;
  non_rigid_joint_count_ = 0;
  for(auto& joint : active_joints_) {
    dc_count += joint->get_dc_count();
    q_count_ += joint->get_q_count();
    dq_count_ += joint->get_dq_count();
    data_count += joint->get_data_count();
    if(joint->get_q_count() > 0) {
      non_rigid_joint_count_++;
    }
  }

  dhva_ = shift::core::topology::DHVA(frames_in_tree_.size(), dc_count,
                                      dq_count_, non_rigid_joint_count_);

  // std::cout << "q_count: " << q_count_ << std::endl;
  // std::cout << "dq_count: " << dq_count_ << std::endl;
  // std::cout << "dc_count: " << dc_count << std::endl;

  // std::cout << "d_wrt_reference_.size(): " << frames_in_tree_.size() <<
  // std::endl;

  d_wrt_reference_.resize(frames_in_tree_.size());
  std::fill(d_wrt_reference_.begin(), d_wrt_reference_.end(),
            ksl_SE3_identity());

  d_reference_frames_.clear();
  d_reference_frames_.resize(non_rigid_joint_count_);

  dcq_.resize(dc_count);
  std::fill(dcq_.begin(), dcq_.end(), 0);

  joint_data_.resize(data_count);
  std::fill(joint_data_.begin(), joint_data_.end(), 0);

  q_copy_.resize(q_count_);
  std::fill(q_copy_.begin(), q_copy_.end(), 0);

  q_.resize(q_count_ + dq_count_);
  std::fill(q_.begin(), q_.end(), 0);

  dq_ = q_.data() + q_count_;

  delta_q_.resize(dq_count_);
  std::fill(delta_q_.begin(), delta_q_.end(), 0);

  ddq_.resize(dq_count_);
  std::fill(ddq_.begin(), ddq_.end(), 0);

  f_.resize(dq_count_);
  std::fill(f_.begin(), f_.end(), 0);

  unsigned int i = 0;
  for(auto& body : active_articulated_bodies_) {
    if(body->get_parent_joint() &&
       body->get_parent_joint()->get_q_count() > 0) {
      body->set_va_index(i++);
    } else {
      body->set_va_index(-1);
    }
  }

  // get operations count
  unsigned int operation_count = 0;
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    int j = i;
    auto joint = active_joints_[j];
    auto parent_joint = joint;
    bool rigid_connection_to_fixed_frame = false;
    while(!parent_joint->get_q_count()) { // rigid joint
      j = pcl_[j];
      if(j == -1) {
        // std::cout << "rigid connection to fixed frame found: ";
        // std::cout << joint->get_name() << std::endl;
        rigid_connection_to_fixed_frame = true;
        break;
      }
      parent_joint = active_joints_[j];
    }

    if(!rigid_connection_to_fixed_frame) {
      auto body = joint->get_system_child_body();
      operation_count += body->get_frame_in_tree_count();
    }
  }

  dhva_offsets_.clear();
  dhva_offsets_.resize(operation_count);
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
  d_fptr_.resize(operation_count);

  dh_fptr_.clear();
  dh_fptr_.resize(operation_count);

  dhva_fptr_.clear();
  dhva_fptr_.resize(operation_count);

  int q_offset = 0;
  int dq_offset = 0;
  int dc_offset = 0;
  int data_offset = 0;
  for(auto& joint : active_joints_) {
    joint->set_q_index(q_offset);
    joint->set_dq_index(dq_offset);
    joint->set_dc_index(dc_offset);
    joint->set_data_index(data_offset);
    q_offset += joint->get_q_count();
    dq_offset += joint->get_dq_count();
    dc_offset += joint->get_dc_count();
    data_offset += joint->get_data_count();
  }

  /* one time update of frames on fixed body */
  // std::cout << "updating frames on fixed body" << std::endl;
  for(auto& frame : get_fixed_body()->get_frames()) {
    frame->set_parent_body(get_fixed_body());
    assert(frame->get_parent_body());
    // std::cout << frame->get_parent_body()->get_name() << std::endl;
    if(!frame->is_reference()) {
      // std::cout << frame->get_name() << std::endl;
      assert(frame->get_parent_body());
      // std::cout << "[" << frame.index() << "]: " <<
      // frame.parent_body()->get_name()
      //           << ":" << frame.name() << std::endl;
      ksl_SE3_copy(frame->get_d_wrt_reference(), dhva_.d + frame->get_index());
      ksl_SE3_copy(frame->get_d_wrt_reference(),
                   &d_wrt_reference_[frame->get_index()]);
      // ksl_SE3_print(stdout, dhva_.d + frame.index());
    }
  }

  operation_count = 0;
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    auto& joint = active_joints_[i];
    auto parent_body = joint->get_system_parent_body();
    auto parent_frame = joint->get_system_parent_frame();
    auto child_body = active_articulated_bodies_[i];
    auto child_frame = child_body->get_reference_frame();

    if(joint->get_q_count()) {
      if(joint->get_dc_count()) {
        dcq_[joint->get_dc_index()] = joint->get_q_index();
        dhva_offsets_[operation_count].dc = joint->get_dc_index() * 2;
      } else {
        dhva_offsets_[operation_count].q = joint->get_q_index();
      }

      dhva_offsets_[operation_count].parent.d = parent_frame->get_index();
      dhva_offsets_[operation_count].child.d = child_frame->get_index();

      dhva_offsets_[operation_count].parent.v = parent_body->get_va_index();
      dhva_offsets_[operation_count].child.v = child_body->get_va_index();
      dhva_offsets_[operation_count].dq = joint->get_dq_index();
      dhva_offsets_[operation_count].data = joint->get_data_index();

      d_fptr_[operation_count] = joint->Dfptr();
      dh_fptr_[operation_count] = joint->DHfptr();
      dhva_fptr_[operation_count] = joint->DHVAfptr();
      operation_count++;

      auto reference_frame = child_body->get_reference_frame();
      // std::cout << "reference_frame: " <<
      // reference_frame->get_parent_body()->get_name()
      //           << ":" << reference_frame->get_name() << std::endl;
      for(auto& frame : child_body->get_frames()) {
        /*!
          @TODO add logic to detect if transformation has zero entries
          and use appropriate sparse function evaluation
          for now, all function pointers are assigned as general SE3
          displacements
        */
        if(!frame->is_reference() && frame->is_in_tree()) {
          d_fptr_[operation_count] = shift::core::joint::rigid_dh;
          dh_fptr_[operation_count] = shift::core::joint::rigid_dh;
          dhva_fptr_[operation_count] = shift::core::joint::rigid_dhva;
          dhva_offsets_[operation_count].parent.d = reference_frame->get_index();
          dhva_offsets_[operation_count].child.d = frame->get_index();
          dhva_offsets_[operation_count].q = frame->get_index();
          // std::cout << "  processing child frame[" << frame.index()
          //           << "]: " << frame.name() << std::endl;
          // ksl_SE3_print(stdout, frame.get_d_wrt_reference(), " d_wrt_reference:
          // ");
          ksl_SE3_copy(frame->get_d_wrt_reference(),
                       &d_wrt_reference_[frame->get_index()]);
          operation_count++;
        }
      }
    } else {
      CompressRigidJoints(i, operation_count);
    }
  }

  // std::cout << "operation_count: " << operation_count << std::endl;
  joint_dof_.resize(active_joints_.size());

  // std::cout << "d_reference_frames: ";
  size_t j = 0;
  for(size_t i = 0; i < active_joints_.size(); i++) {
    if(active_joints_[i]->get_dq_count() > 0) {
      d_reference_frames_[j++] =
        active_articulated_bodies_[i]->get_reference_frame()->get_index();
      // std::cout <<
      // active_articulated_bodies_[i]->get_reference_frame()->get_index()
      //           << " ";
    }
  }
  // std::cout << std::endl;
}

void shift::core::Context::UpdateDC(shift::core::topology::DHVA& dhva,
                                    const double* __restrict__ q) {
  for(unsigned int i = 0; i < dcq_.size(); i++) {
    ksl_dc(q[dcq_[i]], dhva.dc + (i * 2));
  }
}

void shift::core::Context::UpdateD() {
  UpdateD(dhva_, q_.data());
}

void shift::core::Context::UpdateD(const double* q) {
  UpdateD(dhva_, q);
}

void shift::core::Context::UpdateD(shift::core::topology::DHVA& dhva,
                                   const double* __restrict__ q) {
  UpdateDC(dhva, q);
  for(unsigned int i = 0; i < d_fptr_.size(); i++) {
    (d_fptr_[i])(*this, dhva, dhva_offsets_[i], q);
  }
}

void shift::core::Context::UpdateDH() {
  UpdateDH(dhva_, q_.data());
}

void shift::core::Context::UpdateDH(const double* q) {
  UpdateDH(dhva_, q);
}

void shift::core::Context::UpdateDH(shift::core::topology::DHVA& dhva,
                                    const double* __restrict__ q) {
  UpdateDC(dhva, q);
  // std::cout << "UpdateDH()" << std::endl;
  for(unsigned int i = 0; i < dh_fptr_.size(); i++) {
    // auto parent_frame = frames_in_tree_[dhva_offsets_[i].parent.d];
    // auto child_frame = frames_in_tree_[dhva_offsets_[i].child.d];
    // std::cout << "  [" << i
    //           << "] d_parent: " << parent_frame->get_parent_body()->get_name()
    //           << ": "
    //           << parent_frame->get_name() << std::endl;
    // std::cout << "        d_child:  " <<
    // child_frame->get_parent_body()->get_name() <<
    // ": "
    //           << child_frame->get_name() << std::endl;
    (dh_fptr_[i])(*this, dhva, dhva_offsets_[i], q);
  }
  // std::cout << std::endl;
  // FILE* f = fopen("frames.dat", "w");
  // for(auto& frame : frames_in_tree_) {
  //   fprintf(f, "[%d]: [%s, %s]\n", frame->get_index(),
  //           frame->get_parent_body()->get_name().c_str(),
  //           frame->get_name().c_str());
  //   // ksl_SE3_print(f, d_wrt_reference_ + frame->get_index(), " d_wrt_reference:
  //   "); ksl_SE3_print(f, dhva_.d + frame->get_index(), "  "); fprintf(f,
  //   "\n");
  // }
  // fclose(f);
}

void shift::core::Context::UpdateDHVA() {
  UpdateDHVA(dhva_, q_.data(), dq_);
}

void shift::core::Context::UpdateDHVA(const double* q, const double* dq) {
  UpdateDHVA(dhva_, q, dq);
}

void shift::core::Context::UpdateDHVA(shift::core::topology::DHVA& dhva,
                                      const double* __restrict__ q,
                                      const double* __restrict__ dq) {
  UpdateDC(dhva, q);
  for(unsigned int i = 0; i < dhva_fptr_.size(); i++) {
    (dhva_fptr_[i])(*this, dhva, dhva_offsets_[i], q, dq);
  }
}

void shift::core::Context::SetupFixedFrameGravityVector() {
  assert(dhva_.aq);
  ksl_vec3_inverted(model_parameters_.get_gravity(), &dhva_.aq[-1].lin);
}

void shift::core::Context::SetToInitialConditions() {
  // std::cout << "setting initial conditions:" << std::endl;
  assert(dq_);
  unsigned int q_offset = 0;
  unsigned int dq_offset = 0;
  for(auto& joint : active_joints_) {
    if(joint->get_q_count()) {
      // std::cout << joint->get_name() << " " << std::flush;
      assert(joint->get_q0());
      // for(unsigned int i = 0; i < joint->get_q_count(); i++) {
      //   std::cout << joint->get_q0()[i] << " ";
      // }
      // std::cout << std::endl;
      memcpy(&q_[q_offset], joint->get_q0(),
             joint->get_q_count() * sizeof(double));
    }
    if(joint->get_dq_count()) {
      // for(unsigned int i = 0; i < joint->get_dq_count(); i++) {
      //   std::cout << joint->get_dq0()[i] << " ";
      // }
      // std::cout << std::endl;

      memcpy(&dq_[dq_offset], joint->get_dq0(),
             joint->get_dq_count() * sizeof(double));
    }
    q_offset += joint->get_q_count();
    dq_offset += joint->get_dq_count();
  }
}

void shift::core::Context::InitializeInertia() {

  /*
    create a temporary inertia array, equal in length to active_joints
    after this list is generated, it will be compressed to an array
    equal to length of non_rigid_joint_count
  */
  std::vector<ksl_inertia_t> inertia_temp = {};
  inertia_temp.resize(active_joints_.size());
  std::fill(inertia_temp.begin(), inertia_temp.end(),
            ksl_inertia(0, {{0, 0, 0}}, 0, 0, 0, 0, 0, 0));

  /* compute each inertia matrix with respect to reference frame on
     each body */
  for(int i = (int)active_joints_.size() - 1; i > -1; i--) {
    auto& body = active_articulated_bodies_[i];
    if(body->has_mass()) {
      inertia_temp[i] = body->get_inertia_wrt_reference();
    }
  }

  /* for bodies connected by rigid joints, merge inertias into parent body
  inertia */
  for(int i = (int)active_joints_.size() - 1; i > -1; i--) {
    /* only perform merge if parent joint type is rigid and
       the parent body is not the fixed body */
    if(active_joints_[i]->get_q_count() == 0 && pcl_[i] > -1) {
      auto child_body = active_articulated_bodies_[i];
      auto parent_body = active_articulated_bodies_[pcl_[i]];
      auto parent_frame = active_joints_[i]->get_system_parent_frame();
      ksl_vec3_t t_ic_i = parent_body->get_center_of_mass_wrt_reference();
      ksl_vec3_t t_jc_j = child_body->get_center_of_mass_wrt_reference();
      ksl_inertia_t parent_inertia = parent_body->get_inertia_wrt_reference();
      ksl_inertia_t child_inertia = child_body->get_inertia_wrt_reference();

      ksl_SE3_t D_ij;
      ksl_SE3_t temp = active_joints_[i]->get_transform();
      if(parent_frame->is_reference()) {
        ksl_SE3_copy(&temp, &D_ij);
      } else {
        ksl_product_dd(parent_frame->get_d_wrt_reference(), &temp, &D_ij);
      }
      // i = parent, j = child;
      ksl_inertia_merge(&parent_inertia, &t_ic_i, &child_inertia, &t_jc_j,
                        &D_ij);

      inertia_temp[pcl_[i]] = parent_inertia;
    }
  }

  /* compress expanded inertia list into compressed list */
  inertia_.resize(non_rigid_joint_count_);
  for(unsigned int i = 0; i < active_joints_.size(); i++) {
    int j = joint_lookup_[i];
    if(j != -1) {
      memcpy(&inertia_[j], &inertia_temp[i], sizeof(ksl_inertia_t));
    }
  }
}

void shift::core::Context::SetInitialConditionsToCurrentConditions() {
  assert(dq_);
  unsigned int q_offset = 0;
  unsigned int dq_offset = 0;
  for(auto& joint : active_joints_) {
    joint->set_initial_displacement(q_.data() + q_offset);
    joint->set_initial_velocity(dq_ + dq_offset);
    q_offset += joint->get_q_count();
    dq_offset += joint->get_dq_count();
  }
}

void shift::core::Context::SetupLookup() {
  joint_lookup_.clear();
  joint_lookup_.resize(get_active_joint_count());

  dq_lookup_.clear();
  dq_lookup_.resize(get_active_joint_count());

  q_lookup_.clear();
  q_lookup_.resize(get_active_joint_count());

  dq_lookup_compressed_.clear();
  dq_lookup_compressed_.resize(get_non_rigid_joint_count());

  int joint_count = 0;
  int dq_count = 0;
  int q_count = 0;
  unsigned int i = 0;
  for(auto& joint : get_active_joints()) {
    if(joint->get_q_count() > 0) {
      joint_lookup_[i] = joint_count;
      dq_lookup_[i] = dq_count;
      q_lookup_[i] = q_count;
      joint_count++;
    } else {
      joint_lookup_[i] = -1;
      dq_lookup_[i] = -1;
      q_lookup_[i] = -1;
    }
    i++;
    dq_count += joint->get_q_count();
    q_count += joint->get_q_count();
  }

  for(unsigned int j = 0; j < active_joints_.size(); j++) {
    if(joint_lookup_[j] > -1) {
      dq_lookup_compressed_[joint_lookup_[j]] = dq_lookup_[j];
    }
  }

  for(size_t i = 0; i < active_joints_.size(); i++) {
    joint_dof_[i] = active_joints_[i]->get_dq_count();
  }

  joint_dof_compressed_.resize(non_rigid_joint_count_);
  for(size_t i = 0; i < active_joints_.size(); i++) {
    int compressed_joint_index = joint_lookup_[i];
    if(compressed_joint_index > -1) {
      joint_dof_compressed_[compressed_joint_index] = joint_dof_[i];
    }
  }

  pcldq_.resize(dq_count_);
  for(size_t i = 0; i < active_joints_.size(); i++) {
    int parent_index = pcl_[i];
    int variable_index = dq_lookup_[i];
    if(variable_index != -1) {
      if(parent_index != -1) {
        int joint_dof_offset = fmax(joint_dof_[parent_index] - 1, 0);
        pcldq_[variable_index] = dq_lookup_[parent_index] + joint_dof_offset;
      } else {
        pcldq_[variable_index] = -1;
      }
    }
    int dof = joint_dof_[i];
    if(dof > 1) {
      for(int j = 1; j < dof; j++) {
        pcldq_[variable_index + j] = variable_index + j - 1;
      }
    }
  }

  has_mass_.resize(non_rigid_joint_count_);
  std::fill(has_mass_.begin(), has_mass_.end(), 0);
  has_mass_dq_.resize(dq_count_);
  std::fill(has_mass_dq_.begin(), has_mass_dq_.end(), 0);
  for(size_t i = 0; i < active_articulated_bodies_.size(); i++) {
    auto& body = active_articulated_bodies_[i];
    auto& joint = active_joints_[i];
    int joint_index = joint_lookup_[i];
    int dq_index = dq_lookup_[i];
    if(joint_index > -1) {
      if(body->has_mass()) {
        has_mass_[joint_index] = true;
        for(size_t j = 0; j < joint->get_dq_count(); j++) {
          has_mass_dq_[dq_index + j] = true;
        }
      }
    }
  }

  /* update has mass dq to set true for any variable that has mass
  projected onto it */
  for(int k = dq_count_ - 1; k > -1; k--) {
    int j = pcldq_[k];
    if(has_mass_dq_[k] && j > -1) {
      has_mass_dq_[j] = true;
    }
  }

  for(size_t i = 0; i < active_articulated_bodies_.size(); i++) {
    int compressed_joint_index = joint_lookup_[i];
    if(compressed_joint_index > -1) {
      joint_dof_compressed_[compressed_joint_index] = joint_dof_[i];
    }
  }

  is_rotation_.resize(dq_count_);
  std::fill(is_rotation_.begin(), is_rotation_.end(), 0);
  for(size_t i = 0; i < active_joints_.size(); i++) {
    auto& joint = active_joints_[i];
    int base_index = dq_lookup_[i];
    for(auto& rotation_index : joint->get_rotation_indices()) {
      is_rotation_[base_index + rotation_index] = true;
    }
  }

  pcl_compressed_.resize(non_rigid_joint_count_);
  std::fill(pcl_compressed_.begin(), pcl_compressed_.end(), 0);

  for(size_t i = 0; i < active_joints_.size(); i++) {
    auto& joint = active_joints_[i];
    if(joint->get_dq_count() > 0) {
      int parent = pcl_[i];
      while(true) {
        if(parent == -1) {
          break;
        }
        if(active_joints_[parent]->get_dq_count() > 0) {
          break;
        }
        parent = pcl_[parent];
      }
      if(parent == -1) {
        pcl_compressed_[joint_lookup_[i]] = -1;
      } else {
        pcl_compressed_[joint_lookup_[i]] = joint_lookup_[parent];
      }
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
void shift::core::Context::InitializeMobility() {
  mobility_ = 0;
  if(dq_count_ > 0) {
    mobility_numbers_.resize(dq_count_);
    std::fill(mobility_numbers_.begin(), mobility_numbers_.end(), 1.0);
  } else {
    mobility_numbers_.clear();
  }
  jacobian_.InitializeMobility();
}

/*!
  @brief compute global instantaneous mobility, to be called after
  mobility has been computed within each substructure
*/
double shift::core::Context::GlobalMobility() {
#if SHIFT_WITH_BLAS_
  mobility_ = cblas_dasum(dq_count_, mobility_numbers_.data(), 1);
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

@param context a pointer to a shift::core::Context data structure.

*/
int shift::core::Context::Digraph(std::string filename) {

  std::string base_filename = shift::core::topology::GetFilename(filename);
  if(base_filename.length() == 0) {
    std::cerr << "Error: could not resolve filename for: " << filename
              << std::endl;
    return -1;
  };

  std::string extension = shift::core::topology::GetExtension(base_filename);
  if(extension.length() == 0) {
    std::cerr << "Error: coud not resolve extension for: " << filename
              << std::endl;
  }

  std::string gv_ext =
    shift::core::topology::GetValidGraphvizExtension(extension);
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
  Agnode_t** nodes =
    (Agnode_t**)malloc(get_active_joint_count() * sizeof(Agnode_t*));
  for(unsigned int i = 0; i < get_active_joint_count(); i++) {
    auto body_name = get_active_articulated_bodies()[i]->get_name();
    sprintf(name, "[%d] %s", i, const_cast<char*>(body_name.c_str()));
    nodes[i] = agnode(g, name, TRUE);
    if(get_active_articulated_bodies()[i]->is_virtual()) {
      agset(nodes[i], const_cast<char*>("shape"), const_cast<char*>("octagon"));
    }
  }

  Agnode_t* fixed = agnode(g, (char*)"[-1] fixed", TRUE);
  Agedge_t* edge;
  for(unsigned int i = 0; i < get_active_joint_count(); i++) {
    std::string joint_name = get_active_joints()[i]->get_name();
    if(get_pcl()[i] == -1) {
      edge =
        agedge(g, fixed, nodes[i], const_cast<char*>(joint_name.c_str()), TRUE);
    } else {
      edge = agedge(g, nodes[get_pcl()[i]], nodes[i],
                    const_cast<char*>(joint_name.c_str()), TRUE);
    }
    sprintf(name, "[%d] %s (%s)", i, joint_name.c_str(),
            get_active_joints()[i]->get_type().c_str());
    agset(edge, const_cast<char*>("label"), name);
  }

  for(unsigned int i = 0; i < get_chord_constraint_count(); i++) {
    int parent_index =
      get_chord_constraints()[i]->get_parent_body()->get_index();
    std::string joint_name = get_chord_constraints()[i]->get_name();
    if(parent_index == -1) {
      edge =
        agedge(g, fixed,
               nodes[get_chord_constraints()[i]->get_child_body()->get_index()],
               const_cast<char*>(joint_name.c_str()), TRUE);
    } else {
      edge =
        agedge(g, nodes[parent_index],
               nodes[get_chord_constraints()[i]->get_child_body()->get_index()],
               const_cast<char*>(joint_name.c_str()), TRUE);
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
    std::string basename = shift::core::topology::GetBasename(filename);
    if(basename.length() == 0) {
      gvFreeLayout(gvc, g);
      agclose(g);
      gvFreeContext(gvc);
      free(nodes);
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
  return 0;

  // agclose and gvFreeContext do not free their memory;
  // up to 1MB is not properly freed at program termination.

  // @TODO find fix for memory leak or write to DOT file and call graphviz
  // using external system call
}

int shift::core::Context::Digraph() {
  std::string base_filename = shift::core::topology::GetFilename(filename_);
  std::string graph_filename =
    shift::core::topology::RemoveExtension(base_filename) + ".png";
  if(base_filename.length() == 0) {
    std::cerr << "Error: could not resolve filename for: " << filename_
              << std::endl;
    return -1;
  };
  return Digraph(graph_filename);
}

bool FindReferenceFrame(const shift::core::Context& context, int d_index,
                        int* reference_frame_index) {
  auto& bodies = context.get_active_articulated_bodies();
  for(auto& body : bodies) {
    int reference_frame = body->get_reference_frame()->get_index();
    for(auto& frame : body->get_frames()) {
      if(frame->get_index() == d_index) {
        *reference_frame_index = reference_frame;
      }
    }
  }
  return false;
}

/*!
@brief Generate a directed graph using graphviz api library

The graphviz library leaks memory:

agclose and gvFreeContext appear do not entirely free memory,
up to 1MB of memory is not properly freed at program termination.

@TODO find fix for this or just write to DOT file and call graphviz using
external system call
*/
int shift::core::Context::DigraphFrames(std::string filename) {

  std::string base_filename = shift::core::topology::GetFilename(filename);
  if(base_filename.length() == 0) {
    std::cerr << "Error: could not resolve filename for: " << filename
              << std::endl;
    return -1;
  };

  std::string extension = shift::core::topology::GetExtension(base_filename);
  if(extension.length() == 0) {
    std::cerr << "Error: coud not resolve extension for: " << filename
              << std::endl;
  }

  std::string gv_ext =
    shift::core::topology::GetValidGraphvizExtension(extension);
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

  int joint_counter = 0;
  for(unsigned int i = 0; i < get_dhva_offsets().size(); i++) {

    int parent_frame_index = get_dhva_offsets()[i].parent.d;
    int child_frame_index = get_dhva_offsets()[i].child.d;

    ksl_SE3_t* d_parent = dhva_.d + parent_frame_index;
    ksl_SE3_t* d_child = dhva_.d + child_frame_index;

    int parent_reference_frame_index = -1;
    FindReferenceFrame(*this, parent_frame_index,
                       &parent_reference_frame_index);
    bool parent_is_reference =
      (parent_frame_index == parent_reference_frame_index);

    int child_reference_frame_index = -1;
    FindReferenceFrame(*this, child_frame_index, &child_reference_frame_index);
    bool childIsReference = (child_frame_index == child_reference_frame_index);

    assert(get_frames_in_tree()[parent_frame_index]->get_parent_body());
    std::string parent_body_name =
      get_frames_in_tree()[parent_frame_index]->get_parent_body()->get_name();

    char parent_frame_name[1200];
    sprintf(parent_frame_name,
            "[%d] {%d} %s: %s\n"
            "[% 2.10f % 2.10f % 2.10f % 0.10f]\n"
            "[% 2.10f % 2.10f % 2.10f % 0.10f]\n"
            "[% 2.10f % 2.10f % 2.10f % 0.10f]\n"
            "[ 0.00  0.00  0.00  1.00]",
            parent_frame_index, parent_reference_frame_index,
            get_frames_in_tree()[parent_frame_index]
              ->get_parent_body()
              ->get_name()
              .c_str(),
            get_frames_in_tree()[parent_frame_index]->get_name().c_str(),
            d_parent->R.m00, d_parent->R.m01, d_parent->R.m02, d_parent->t.x,
            d_parent->R.m10, d_parent->R.m11, d_parent->R.m12, d_parent->t.y,
            d_parent->R.m20, d_parent->R.m21, d_parent->R.m22, d_parent->t.z);

    // std::cout << child_frame_index << std::endl;
    // std::cout << get_frames_in_tree()[child_frame_index]->get_name() <<
    // std::endl; std::cout <<
    // get_frames_in_tree()[child_frame_index]->get_parent_body() << std::endl;
    assert(get_frames_in_tree()[child_frame_index]->get_parent_body());
    std::string child_body_name =
      get_frames_in_tree()[child_frame_index]->get_parent_body()->get_name();

    char child_frame_name[1200];
    sprintf(child_frame_name,
            "[%d] {%d} %s: %s\n"
            "[% 2.10f % 2.10f % 2.10f % 2.10f]\n"
            "[% 2.10f % 2.10f % 2.10f % 2.10f]\n"
            "[% 2.10f % 2.10f % 2.10f % 2.10f]\n"
            "[ 0.00  0.00  0.00  1.00]",
            child_frame_index, child_reference_frame_index,
            get_frames_in_tree()[child_frame_index]
              ->get_parent_body()
              ->get_name()
              .c_str(),
            get_frames_in_tree()[child_frame_index]->get_name().c_str(),
            d_child->R.m00, d_child->R.m01, d_child->R.m02, d_child->t.x,
            d_child->R.m10, d_child->R.m11, d_child->R.m12, d_child->t.y,
            d_child->R.m20, d_child->R.m21, d_child->R.m22, d_child->t.z);

    std::string joint_name = shift::core::topology::GetJointType(dh_fptr_[i]);

    if(joint_name == "PRISMATIC_TX") {
      sprintf(name, "[%d] {%d} Tx(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TX_INVERSE") {
      sprintf(name, "[%d] {%d} Txinverse(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TY") {
      sprintf(name, "[%d] {%d} Ty(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TY_INVERSE") {
      sprintf(name, "[%d] {%d} Tyinverse(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TZ") {
      sprintf(name, "[%d] {%d} Tz(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TZ_INVERSE") {
      sprintf(name, "[%d] {%d} Tzinverse(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TA") {
      sprintf(name, "[%d] {%d} Ta(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "PRISMATIC_TA_INVERSE") {
      sprintf(name, "[%d] {%d} Tainverse(% 0.10f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "REVOLUTE_RX") {
      sprintf(name, "[%d] {%d} Rx(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "REVOLUTE_RX_INVERSE") {
      sprintf(name, "[%d] {%d} Rxinverse(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "REVOLUTE_RY") {
      sprintf(name, "[%d] {%d} Ry(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "REVOLUTE_RY_INVERSE") {
      sprintf(name, "[%d] {%d} Ryinverse(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "REVOLUTE_RZ") {
      sprintf(name, "[%d] {%d} Rz(%f)\n", i, joint_counter++,
              q_[dhva_offsets_[i].q]);

    } else if(joint_name == "REVOLUTE_RZ_INVERSE") {
      sprintf(name, "[%d] {%d} Rzinverse(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "REVOLUTE_RA") {
      sprintf(name, "[%d] {%d} Ra(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "REVOLUTE_RA_INVERSE") {
      sprintf(name, "[%d] {%d} Rainverse(%0.10f, %0.10f)\n", i, joint_counter++,
              dhva_.dc[dhva_offsets_[i].dc], dhva_.dc[dhva_offsets_[i].dc + 1]);

    } else if(joint_name == "SPHERICAL" || joint_name == "SPHERICAL_INVERSE") {
      ksl_quaternion_t* q = (ksl_quaternion_t*)&(q_[dhva_offsets_[i].q]);
      sprintf(name, "[%d] {%d} q: [% 2.2f, % 2.2f, % 2.2f, % 2.2f]", i,
              joint_counter++, q->x, q->y, q->z, q->w);

    } else if(joint_name == "FLOAT" || joint_name == "FLOAT_INVERSE") {
      ksl_vec3_t* t = (ksl_vec3_t*)&(q_[dhva_offsets_[i].q]);
      ksl_quaternion_t* q = (ksl_quaternion_t*)&(q_[dhva_offsets_[i].q + 3]);
      sprintf(name,
              "[%d] {%d} t: [% 2.10f, % 2.10f, % 2.10f]\n"
              "     q: [% 2.10f, % 2.10f, % 2.10f, % 2.10f]",
              i, joint_counter++, t->x, t->y, t->z, q->x, q->y, q->z, q->w);
    } else if(joint_name == "RIGID" || joint_name == "RIGID_INVERSE") {
      ksl_SE3_t& d_primitive_disp = d_wrt_reference_[dhva_offsets_[i].child.d];
      sprintf(name,
              "[%d] Rigid\n"
              "[% 2.10f % 2.10f % 2.10f % 2.10f]\n"
              "[% 2.10f % 2.10f % 2.10f % 2.10f]\n"
              "[% 2.10f % 2.10f % 2.10f % 2.10f]\n"
              "[ 0.00  0.00  0.00  1.00]",
              i, d_primitive_disp.R.m00, d_primitive_disp.R.m01,
              d_primitive_disp.R.m02, d_primitive_disp.t.x,
              d_primitive_disp.R.m10, d_primitive_disp.R.m11,
              d_primitive_disp.R.m12, d_primitive_disp.t.y,
              d_primitive_disp.R.m20, d_primitive_disp.R.m21,
              d_primitive_disp.R.m22, d_primitive_disp.t.z);
    } else {
      sprintf(name, "joint type not found / unknown");
    }

    char parent_name[400];
    sprintf(parent_name, "cluster_%s", parent_body_name.c_str());
    Agraph_t* parent_body_graph = agsubg(g, parent_name, TRUE);
    agset(parent_body_graph, const_cast<char*>("label"),
          const_cast<char*>(parent_body_name.c_str()));
    agset(parent_body_graph, const_cast<char*>("fillcolor"),
          const_cast<char*>("lightgrey"));
    agset(parent_body_graph, const_cast<char*>("style"),
          const_cast<char*>("filled"));

    char childName[400];
    sprintf(childName, "cluster_%s", child_body_name.c_str());
    Agraph_t* child_body_graph = agsubg(g, childName, TRUE);
    agset(child_body_graph, const_cast<char*>("fillcolor"),
          const_cast<char*>("lightgrey"));
    agset(child_body_graph, const_cast<char*>("style"),
          const_cast<char*>("filled"));
    agset(child_body_graph, const_cast<char*>("label"),
          const_cast<char*>(child_body_name.c_str()));

    Agnode_t* parent_node = agnode(parent_body_graph, parent_frame_name, TRUE);
    if(parent_is_reference) {
      agset(parent_node, const_cast<char*>("penwidth"), const_cast<char*>("3"));
    }

    Agnode_t* child_node = agnode(child_body_graph, child_frame_name, TRUE);
    if(childIsReference) {
      agset(child_node, const_cast<char*>("penwidth"), const_cast<char*>("3"));
    }

    Agedge_t* edge = agedge(g, parent_node, child_node, name, TRUE);
    agset(edge, const_cast<char*>("label"), name);
  }

  gvLayout(gvc, g, "dot");

  gvRenderFilename(gvc, g, const_cast<char*>(gv_ext.c_str()), filename.c_str());

  if(gv_ext != "dot") {
    std::string basename = shift::core::topology::GetBasename(filename);
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

int shift::core::Context::DigraphFrames() {
  std::string base_filename = shift::core::topology::GetFilename(filename_);
  std::string graph_filename =
    shift::core::topology::RemoveExtension(base_filename) + ".png";
  if(base_filename.length() == 0) {
    std::cerr << "Error: could not resolve filename for: " << filename_
              << std::endl;
    return -1;
  };
  return DigraphFrames(graph_filename);
}

/*!
@brief computes total gravitational potential energy for system

@return gravitational potential energy
*/
double shift::core::Context::GravitationalPotentialEnergy() {
  // return ksl_dot_vv(&projected_inertia_[0].mt,
  // model_parameters_.gravity());
  return 0;
}

/*!
@brief compute kinetic energy for system

*/
double shift::core::Context::KineticEnergy() {
  assert(dhva_.v);
#if SHIFT_WITH_BLAS_
  return cblas_ddot(6 * non_rigid_joint_count_, (p_.data()), 1,
                    (double*)(dhva_.v), 1) /
         2.0;
#else
  double sum = 0.0;
  for(unsigned int i = 0; i < non_rigid_joint_count_; i++) {
    sum += ksl_dot_cs(&p_[i], dhva_.v + i);
  }
  return sum;
#endif
}

unsigned int shift::core::Context::get_force_module_equation_count() {
  unsigned int equation_count = 0;
  for(auto& force_module : active_force_modules_) {
    equation_count += force_module->get_equation_count();
  }
  return equation_count;
}

void shift::core::Context::EvaluateConstraints() {
  constraint_error_ = jacobian_.EvaluateConstraints();
}

void shift::core::Context::SetForces(const double t,
                                     const std::vector<double>& y) {
  (void)t;
  std::fill(f_.begin(), f_.end(), 0);
  for(auto& joint : active_joints_) {
    joint->Service(y);
  }
  // std::cout << "f_: ";
  // for(auto& force : f_) {
  //   std::cout << force << " ";
  // }
  // std::cout << std::endl;
}

void shift::core::Context::ConvertdqTody(const std::vector<double>& y,
                                         std::vector<double>& dy) {
  const double* q = y.data();
  const double* dq = q + q_count_;

  /* convert angular velocities to time derivative of quaternions */
  for(auto& index : quaternion_indices_) {
    ksl_omega_to_dquaternion((ksl_vec3_t*)(&dq[index.dq]),
                             (ksl_quaternion_t*)(&q[index.q]),
                             (ksl_quaternion_t*)(&dy[index.q]));
  }

  for(auto& index : non_quaternion_indices_) {
    // std::cout << "index.q: " << index.q << "/" << q_count_ << " ";
    // std::cout << "index.dq: " << index.dq << "/" << dq_count_ << std::endl;
    dy[index.q] = dq[index.dq];
  }

  memcpy(dy.data() + q_count_, ddq_.data(), dq_count_ * sizeof(double));
}

void shift::core::Context::SetupQuaternionMap() {
  quaternion_indices_.clear();
  non_quaternion_indices_.clear();
  unsigned int q_index = 0;
  unsigned int dq_index = 0;
  for(auto& joint : active_joints_) {
    if(joint->get_type() == "spherical") {
      quaternion_indices_.push_back({q_index, dq_index});
      q_index += 4;
      dq_index += 3;
    } else if(joint->get_type() == "float") {
      quaternion_indices_.push_back({q_index, dq_index});
      q_index += 4;
      dq_index += 3;
      non_quaternion_indices_.push_back({q_index++, dq_index++});
      non_quaternion_indices_.push_back({q_index++, dq_index++});
      non_quaternion_indices_.push_back({q_index++, dq_index++});
    } else {
      assert(joint->get_q_count() == joint->get_dq_count());
      for(unsigned int i = 0; i < joint->get_q_count(); i++) {
        non_quaternion_indices_.push_back({q_index++, dq_index++});
      }
    }
  }

  for(auto& force_module : active_force_modules_) {
    for(unsigned int i = 0; i < force_module->get_equation_count(); i++) {
      non_quaternion_indices_.push_back({q_index++, dq_index++});
    }
  }
}

void shift::core::Context::Simulate() {
  if(active_integrators_.size() == 0 || active_solvers_.size() == 0) {
    throw shift::core::Exception(
      "Error: no active integrators or solvers have been created");
  }
  assert(active_integrators_.size());
  assert(active_solvers_.size());
  auto& integrator = active_integrators_[0];
  integrator->Integrate();
}

void shift::core::Context::get_current_time(double& time, double& delta_time) {
  if(active_integrators_.size() == 0) {
    time = 0.0;
    delta_time = 0.0;
    return;
  } else {
    auto& integrator = active_integrators_[0];
    integrator->get_current_time(time, delta_time);
  }
}

void shift::core::Context::WriteStates(const double time,
                                       const std::vector<double>& y) {
  for(auto& logger : active_loggers_) {
    logger->WriteStates(time, y);
  }
}
