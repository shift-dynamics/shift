#include "shift/core/context.h"

#include <assert.h>
#include <string.h>

#ifdef SHIFT_WITH_BLAS_
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#endif

#include <graphviz/cgraph.h>
#include <graphviz/gvc.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <omp.h>
#include <fstream>
#include <iostream>
#include <list>
#include <queue>
#include <sstream>

#include "ksl/matrix.h"
#include "ksl/quaternion.h"
#include "shift/core/parameter/yaml.h"

#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/topology.h"
#include "shift/core/controller/base.h"
#include "shift/core/body/base.h"
#include "shift/core/body/free.h"
#include "shift/core/body/rigid.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/constraint/base.h"
#include "shift/core/constraint/chord.h"
#include "shift/core/force_module/base.h"
#include "shift/core/geometry/base.h"
#include "shift/core/integrator/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/joint/rigid.h"
#include "shift/core/light/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/sensor/base.h"
#include "shift/core/solver/base.h"
#include "shift/core/gpu/device.h"
#include "shift/core/terrain/base.h"

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
    "bodies, joints, force modules, controllers, etc.; Each assembly at a given "
    "level must have a unique name identifier",
    true);
}

void shift::core::Context::LoadJSON(const std::string filename) {
  filename_ = filename;
  files_read_.clear();
  file_names_.clear();
  std::error_code error;
  if(!std::filesystem::exists(filename, error)) {
    throw shift::core::Exception("The file '" + filename +
                                 "' does not exist");
  }
  if(!std::filesystem::is_regular_file(filename, error)) {
    throw shift::core::Exception("'" + filename + "' is not a file");
  }
  std::ifstream file(filename);
  if(!file) {
    throw shift::core::Exception("The file '" + filename +
                                 "' could not be opened: " +
                                 std::strerror(errno));
  }
  std::string extension = std::filesystem::path(filename).extension().string();
  for(char& c : extension) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if(extension == ".yaml" || extension == ".yml") {
    /* the same structure as JSON, in YAML */
    std::stringstream text;
    text << file.rdbuf();
    root_ = shift::core::parameter::YamlToJson(text.str());
    if(!root_.isObject()) {
      throw shift::core::Exception("The file '" + filename +
                                   "' must hold a mapping at its top level");
    }
  } else {
    file >> root_;
  }
  ReadJSON(root_);
  AddDefaultFloor();
  set_parent(this);
  /* collision geometry given at the top level belongs to the fixed body */
  for(auto& geometry : collision_geometry_.get_data()) {
    geometry->set_parent_body(this);
  }
  PostLoad(filename);
}

void shift::core::Context::PostLoad(const std::string) {

  CheckUniqueNames();

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
  SetupFreeBodies();
  AllocateRunTimeArrays();
  SetupFixedFrameGravityVector();
  SetToInitialConditions();
  UpdateD();
  UpdateDH();
  UpdateDHVA();

  SetupLookup();
  DetectTrees();
  jacobian_.Initialize();
  InitializeMobility();
  InitializeInertia();
  SetupQuaternionMap();

  // SetupDReferenceFrames();
  // InitializeGLReferenceFrames();

  LoadData();
  SetupContactDetection();

  // InitializeJointForces();
  // FlattenForceModules();
  ActivateLoggers();
}

void shift::core::Context::ActivateLoggers() {
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

  if(!shift::core::controller::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Controller modules did not properly initialize");
  }
  if(!shift::core::body::Modules::is_initialized()) {
    throw shift::core::Exception("Body modules did not properly initialize");
  }
  if(!shift::core::collision_geometry::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Collision geometry modules did not properly initialize");
  }
  if(!shift::core::constraint::Modules::is_initialized()) {
    throw shift::core::Exception(
      "Constraint modules did not properly initialize");
  }
  if(!shift::core::force_module::Modules::is_initialized()) {
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

std::shared_ptr<shift::core::gpu::Device> shift::core::Context::GetGpuDevice() {
  if(!gpu_device_) {
    gpu_device_ = std::make_shared<shift::core::gpu::Device>();
  }
  return gpu_device_;
}

void shift::core::Context::ActivateObjects(bool) {
  model_parameters_.ActivateObjects(true);
  collision_geometry_.ActivateObjects(true);
  assemblies_.ActivateObjects(true);
  for(auto& light : get_lights()) {
    light->ActivateObjects(true);
  }
  for(auto& terrain : get_terrain()) {
    terrain->ActivateObjects(true);
  }
}

void shift::core::Context::AddDefaultFloor() {
  for(auto& geometry : collision_geometry_.get_data()) {
    if(dynamic_cast<collision_geometry::Plane*>(geometry.get())) {
      return;
    }
  }
  auto floor = std::make_unique<collision_geometry::Plane>("floor");
  floor->set_contact_enabled(false);
  collision_geometry_.PushBack(std::move(floor));
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
  shift::core::controller::Base* controller) {
  if(controller->is_active()) {
    for(auto i : active_controllers_) {
      if(i == controller) {
        return;
      }
    }
    /* controllers that run first, in the order they come, go before the
    others, which keep theirs */
    if(controller->runs_first()) {
      auto position = active_controllers_.begin();
      while(position != active_controllers_.end() && (*position)->runs_first()) {
        ++position;
      }
      active_controllers_.insert(position, controller);
    } else {
      active_controllers_.push_back(controller);
    }
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
  shift::core::collision_geometry::Base* geometry) {
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
  shift::core::force_module::Base* module) {
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
  shift::core::controller::Base* controller) {
  for(unsigned int i = 0; i < active_controllers_.size(); i++) {
    if(active_controllers_[i] == controller) {
      active_controllers_.erase(active_controllers_.begin() + i);
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
  shift::core::collision_geometry::Base* geometry) {
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
  shift::core::force_module::Base* module) {
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

void shift::core::Context::ActivateObject(shift::core::controller::Base* controller,
                                          bool active) {
  if(controller) {
    active ? AppendActiveObject(controller) : RemoveActiveObject(controller);
  }
}

void shift::core::Context::ActivateObject(shift::core::body::Base* body,
                                          bool active) {
  if(body) {
    active ? AppendActiveObject(body) : RemoveActiveObject(body);
  }
}

void shift::core::Context::ActivateObject(
  shift::core::collision_geometry::Base* geometry, bool active) {
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
  shift::core::force_module::Base* module, bool active) {
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

void shift::core::Context::ClearActiveControllers() {
  active_controllers_.clear();
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

void shift::core::Context::ClearActiveForceModules() {
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
  ClearActiveControllers();
  ClearActiveArticulatedBodies();
  ClearActiveFreeBodies();
  ClearActiveCollisionGeometry();
  ClearActiveConstraints();
  ClearActiveForceModules();
  ClearActiveGeometry();
  ClearActiveJoints();
  ClearActiveLights();
  ClearActiveSensors();
  ClearActiveTerrain();
}

void shift::core::Context::Connect() {
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
    for(auto* body : {joint->get_parent_body(), joint->get_child_body()}) {
      if(body && body != get_fixed_body() &&
         std::find(active_articulated_bodies_.begin(),
                   active_articulated_bodies_.end(),
                   body) == active_articulated_bodies_.end()) {
        throw shift::core::Exception(
          "Joint " + joint->get_name() + " connects inactive body " +
          body->get_name() + "; set \"active\": false on the joint too");
      }
    }
  }
  for(auto& light : active_lights_) {
    light->Connect();
  }
  for(auto& sensor : active_sensors_) {
    sensor->Connect();
  }
  /* after the sensors: a PID controller's derivative term needs to know
  whether its sensor provides a rate, which the sensor finds in Connect */
  for(auto& controller : active_controllers_) {
    controller->Connect();
  }
  for(auto& terrain : active_terrain_) {
    terrain->ConnectAll();
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

/* with CAD geometry in the model, the exact shapes of the other geometry, once
it is all loaded, so that contact threads only read them. Geometry fitted to the
same mesh shares its hull, and so its shape */
void shift::core::Context::LoadExactShapes() {
  bool cad = false;
  for(auto geometry : active_collision_geometry_) {
    cad = cad || dynamic_cast<collision_geometry::Cad*>(geometry);
  }
  if(!cad) {
    return;
  }
  std::map<const void*, std::shared_ptr<const collision_geometry::CadContactShape>>
    shapes;
  for(auto geometry : active_collision_geometry_) {
    const void* key = geometry->ExactShapeKey();
    auto found = shapes.find(key);
    if(found == shapes.end()) {
      found = shapes.insert({key, geometry->MakeExactShape()}).first;
    }
    geometry->SetExactShape(found->second);
  }
}

void shift::core::Context::LoadData() {
  for(auto& controller : active_controllers_) {
    controller->Load();
  }
  for(auto& body : active_articulated_bodies_) {
    body->Load();
  }
  for(auto& constraint : active_constraints_) {
    constraint->Load();
  }
  for(auto& force_module : active_force_modules_) {
    force_module->Load();
  }
  mesh_assets_.clear();
  for(auto& geometry : active_geometry_) {
    geometry->Load();
  }
  /* mass properties from the meshes, now that they are loaded */
  bool from_density = false;
  body::Base::MassCache mass_cache;
  for(auto list : {&active_articulated_bodies_, &active_free_bodies_}) {
    for(auto body : *list) {
      if(body->get_density() > 0.0) {
        body->ComputeMassFromGeometry(mass_cache);
        from_density = true;
      }
    }
  }
  if(from_density) {
    InitializeInertia();
    for(auto body : free_bodies_) {
      body->SetupInertia();
    }
  }
  mesh_assets_.clear();
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
    terrain->LoadAll();
  }
  /* collision geometry is fitted to the mesh geometry loaded above */
  collision_geometry_prototypes_.clear();
  for(auto& collision_geometry : active_collision_geometry_) {
    collision_geometry->Load();
  }
  LoadExactShapes();
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

    SHIFT_TRACE("Procesing chord constraint " + std::to_string(i) +
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
      /* merge every remaining loop that shares a joint with the
      substructure; repeat until a pass merges nothing, since a merged loop
      can overlap loops already passed over */
      int merge_count = 1;
      while(merge_count != 0) {
        merge_count = 0;
        auto it = chord_constraint_indices.begin();
        while(it != chord_constraint_indices.end()) {
          /* base node of substructure */
          /* loops that share a joint have base nodes on one path to the
          fixed body, either one above the other */
          int base_node = loop_members_[substructure[0]][0];
          int candidate_base_node = loop_members_[(*it)][0];
          if(shift::core::topology::InPath(pcl_, candidate_base_node,
                                           base_node) ||
             shift::core::topology::InPath(pcl_, base_node,
                                           candidate_base_node)) {
            for(auto loop_index : substructure) {
              for(auto loop_member = loop_members_[loop_index].begin() + 1;
                  loop_member < loop_members_[loop_index].end();
                  ++loop_member) {
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
      ss << "Substructure members after sorting loops in monotonically "
         << "level increasing order:\n";
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
      ss << "Substructures after sorting substructures in monotonically "
         << "level increasing order:\n";
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
  /* parent body of the topmost rigid joint of the chain: D is the
  displacement with respect to its reference frame */
  auto reference_body = parent_joint->get_system_parent_body();

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
    reference_body = parent_joint->get_system_parent_body();

    j = pcl_[j];
    if(j == -1) {
      break;
    }
    parent_joint = active_joints_[j];
  }

  auto joint = active_joints_[i];
  if(!joint->get_q_count()) {
    // std::cout << "processing joint: " << joint->get_name() << std::endl;
    auto child_body = active_articulated_bodies_[i];
    /* its frames that are not in the tree, e.g. the origin of a body with
    no joints below it, are placed from this (get_body_origin_pose) */
    rigid_placements_[child_body] = {
      reference_body->get_reference_frame()->get_index(), D};

    // ksl_SE3_print(stdout, &D, "  D_p0_c0: ");

    for(auto& frame : child_body->get_frames()) {
      // std::cout << child_body->get_name() << ": " << frame.name() <<
      // std::endl;

      // d_wrt_reference = d_wrt_reference * d_wrt_reference_child
      ksl_SE3_t D_p0_c1;
      ksl_product_dd(&D, frame->get_d_wrt_reference(), &D_p0_c1);

      // ksl_SE3_print(stdout, &D_p0_c1, "  D_p0_c1: ");

      if(frame->is_in_tree() && reference_body != get_fixed_body()) {
        // std::cout << "intree && parent_body!= fixedBody: processing frame: "
        //           << child_body->get_name() << ": " << frame.name() <<
        //           std::endl;

        ksl_SE3_copy(&D_p0_c1, &d_wrt_reference_[frame->get_index()]);

        dhva_offsets_[operation_count].parent.d =
          reference_body->get_reference_frame()->get_index();
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

        ksl_product_dd(
          &dhva_.d[reference_body->get_reference_frame()->get_index()],
          &D_p0_c1, &dhva_.d[frame->get_index()]);
        // ksl_SE3_print(stdout, dhva_.d + frame->get_index(), "  d_absolute:
        // ");
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
  dhva_.free.Resize(free_bodies_.size());

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

  q_.resize(q_count_ + dq_count_ + free_state_count_);
  std::fill(q_.begin(), q_.end(), 0);

  dq_ = q_.data() + q_count_;

  delta_q_.resize(dq_count_);
  std::fill(delta_q_.begin(), delta_q_.end(), 0);

  ddq_.resize(dq_count_);
  std::fill(ddq_.begin(), ddq_.end(), 0);

  f_.resize(dq_count_);
  std::fill(f_.begin(), f_.end(), 0);

  /* bodies are in level increasing order, so the parent of a body attached
  by a rigid joint has its index already: the body moves with its parent and
  shares its velocities (-1 is the fixed body) */
  unsigned int i = 0;
  for(auto& body : active_articulated_bodies_) {
    auto joint = body->get_parent_joint();
    if(joint && joint->get_q_count() > 0) {
      body->set_va_index(i++);
    } else if(joint && joint->get_system_parent_body()) {
      body->set_va_index(joint->get_system_parent_body()->get_va_index());
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
    /* constant data of the joint, e.g. an axis that is not x, y or z, read
    by its kernels at offsets.data */
    if(const double* data = joint->get_data()) {
      std::copy(data, data + joint->get_data_count(),
                joint_data_.begin() + data_offset);
    }
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
      ksl_SE3_copy(frame->get_d_wrt_reference(), &dhva_.d[frame->get_index()]);
      ksl_SE3_copy(frame->get_d_wrt_reference(),
                   &d_wrt_reference_[frame->get_index()]);
      // ksl_SE3_print(stdout, dhva_.d + frame.index());
    }
  }

  operation_count = 0;
  rigid_placements_.clear();
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
          dhva_offsets_[operation_count].parent.d =
            reference_frame->get_index();
          dhva_offsets_[operation_count].child.d = frame->get_index();
          dhva_offsets_[operation_count].q = frame->get_index();
          // std::cout << "  processing child frame[" << frame.index()
          //           << "]: " << frame.name() << std::endl;
          // ksl_SE3_print(stdout, frame.get_d_wrt_reference(), "
          // d_wrt_reference:
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
    ksl_dc(q[dcq_[i]], &dhva.dc[i * 2]);
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
    //           << "] d_parent: " <<
    //           parent_frame->get_parent_body()->get_name()
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
  //   // ksl_SE3_print(f, d_wrt_reference_ + frame->get_index(), "
  //   d_wrt_reference:
  //   "); ksl_SE3_print(f, dhva_.d + frame->get_index(), "  "); fprintf(f,
  //   "\n");
  // }
  // fclose(f);
}

void shift::core::Context::UpdateDHVA() {
  UpdateDHVA(dhva_, q_.data(), dq_);
  UpdateFreeDV(dhva_, q_.data() + get_free_state_offset());
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

/* the generalized coordinates of a free body, see body::Free::kStateCount */
static void FreeCoordinates(const ksl_SE3_t& d, const ksl_screw_t& v,
                            double* q) {
  ksl_vec3_copy(&d.t, reinterpret_cast<ksl_vec3_t*>(q));
  ksl_mat3x3_toQuaternion(&d.R, reinterpret_cast<ksl_quaternion_t*>(q + 3));
  for(int i = 0; i < 6; i++) {
    q[7 + i] = v.at[i];
  }
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
  double* free_q = q_.data() + get_free_state_offset();
  for(auto body : free_bodies_) {
    FreeCoordinates(body->get_initial_d(), body->get_initial_v(),
                    free_q + body::Free::kStateCount * body->get_free_index());
  }
  UpdateFreeDV(dhva_, free_q);
}

void shift::core::Context::CheckUniqueNames() {
  shift::core::body::Fixed::CheckUniqueNames();
  CheckObjectNames(assemblies_.get_data(), "Assembly");
  for(auto& assembly : assemblies_.get_data()) {
    assembly->CheckUniqueNames();
  }
}

void shift::core::Context::SetupFreeBodies() {
  free_bodies_.clear();
  for(auto body : active_free_bodies_) {
    if(auto free = dynamic_cast<body::Free*>(body)) {
      free->set_free_index(static_cast<int>(free_bodies_.size()));
      if(!(free->get_density() > 0.0)) {
        free->SetupInertia(); // otherwise in LoadData, from the meshes
      }
      free_bodies_.push_back(free);
    }
  }
  for(auto joint : active_joints_) {
    for(auto body : {joint->get_parent_body(), joint->get_child_body(),
                     joint->get_system_parent_body(),
                     joint->get_system_child_body()}) {
      if(dynamic_cast<body::Free*>(body)) {
        throw shift::core::Exception(
          "Joint '" + joint->get_name() + "' connects to free body '" +
          body->get_name() + "'; a free body may not have joints");
      }
    }
  }
  free_state_count_ =
    static_cast<unsigned int>(free_bodies_.size()) * body::Free::kStateCount;
  free_loads_.assign(free_bodies_.size(), ksl_coscrew(0, 0, 0, 0, 0, 0));
}

/* free bodies below this count are evaluated by one thread, where starting
threads costs more than it saves */
static constexpr int kFreeBodyGrain = 64;

static int FreeBodyThreads(int count) {
  return std::max(1, std::min(omp_get_max_threads(), count / kFreeBodyGrain));
}

void shift::core::Context::UpdateFreeDV(topology::DHVA& dhva,
                                        const double* free_q) const {
  const int count = static_cast<int>(free_bodies_.size());
  dhva.free.d.resize(count);
  dhva.free.v.resize(count);
  dhva.free.a.resize(count);
#pragma omp parallel for schedule(static) num_threads(FreeBodyThreads(count))
  for(int k = 0; k < count; k++) {
    const double* q = free_q + body::Free::kStateCount * k;
    ksl_SE3_t& d = dhva.free.d[k];
    ksl_vec3_copy(reinterpret_cast<const ksl_vec3_t*>(q), &d.t);
    ksl_quaternion_t quaternion = {{q[3], q[4], q[5], q[6]}};
    ksl_quaternion_toMat3x3(&quaternion, &d.R); // normalizes the copy
    ksl_screw_t& v = dhva.free.v[k];
    for(int i = 0; i < 6; i++) {
      v.at[i] = q[7 + i];
    }
  }
}

void shift::core::Context::EvaluateFreeBodies(const double* y, double* dy) {
  const int count = static_cast<int>(free_bodies_.size());
  const size_t offset = get_free_state_offset();
  const ksl_vec3_t& gravity = *model_parameters_.get_gravity();
  topology::DVA& free = dhva_.free;
#pragma omp parallel for schedule(static) num_threads(FreeBodyThreads(count))
  for(int k = 0; k < count; k++) {
    free_bodies_[k]->Evaluate(free.d[k], free.v[k], free_loads_[k], gravity,
                              free.a[k]);
    /* the rates of the generalized coordinates: the velocity of the origin
    v.lin + v.ang x t, the quaternion rate and a */
    const size_t i = offset + body::Free::kStateCount * k;
    const ksl_screw_t& v = free.v[k];
    ksl_vec3_t omega_t;
    ksl_cross_vv(&v.ang, &free.d[k].t, &omega_t);
    ksl_add_vv(&v.lin, &omega_t, reinterpret_cast<ksl_vec3_t*>(dy + i));
    ksl_omega_to_dquaternion(&v.ang,
                             reinterpret_cast<const ksl_quaternion_t*>(y + i + 3),
                             reinterpret_cast<ksl_quaternion_t*>(dy + i + 3));
    for(int j = 0; j < 6; j++) {
      dy[i + 7 + j] = free.a[k].at[j];
    }
  }
}

void shift::core::Context::InitializeInertia() {

  /* create a temporary inertia array, equal in length to active_joints
  after this list is generated, it will be compressed to an array
  equal to length of non_rigid_joint_count */
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
  UpdateFreeDV(dhva_, q_.data() + get_free_state_offset());
  for(auto body : free_bodies_) {
    const int k = body->get_free_index();
    body->set_initial_dv(dhva_.free.d[k], dhva_.free.v[k]);
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
    dq_count += joint->get_dq_count();
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
    /* the parent variable is the last one of the nearest joint above that is
    not rigid; a rigid joint has no variables */
    int parent_index = pcl_[i];
    while(parent_index != -1 && joint_dof_[parent_index] == 0) {
      parent_index = pcl_[parent_index];
    }
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

int shift::core::Context::CompressedIndex(int i) const {
  while(i > -1 && joint_lookup_[i] < 0) {
    i = pcl_[i];
  }
  return i < 0 ? -1 : joint_lookup_[i];
}

void shift::core::Context::DetectTrees() {
  /* a chord constraint joins the virtual body at the end of its cut joint to
  the body the joint attached to; the loop it closes runs through the trees
  of both (one tree, or two trees and the fixed body) */
  std::vector<int> loop_bodies;
  for(auto chord : chord_constraints_) {
    for(auto body : {chord->get_parent_body(), chord->get_child_body()}) {
      if(body != nullptr && body != get_fixed_body()) {
        loop_bodies.push_back(CompressedIndex(body->get_index()));
      }
    }
  }
  trees_ = topology::DetectTrees(pcl_compressed_, joint_dof_compressed_,
                                 joint_lookup_, loop_bodies);

  for(auto& tree : trees_) {
    std::stringstream ss;
    ss << "Tree with root joint '" << active_joints_[tree.root_joint]->get_name()
       << "': " << topology::Tree::KindName(tree.kind) << ", "
       << tree.bodies.size() << " bodies, " << tree.dq_count
       << " degrees of freedom";
    SHIFT_DEBUG(ss.str());
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

double shift::core::Context::ComputeMobility() {
  UpdateDH();
  for(auto& substructure : jacobian_.get_substructures()) {
    substructure.SetupPO1(dhva_); // full factor and B matrix
    substructure.ComputeMobility();
  }
  return GlobalMobility();
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
    nodes[i] = agnode(g, name, 1);
    if(get_active_articulated_bodies()[i]->is_virtual()) {
      agset(nodes[i], const_cast<char*>("shape"), const_cast<char*>("octagon"));
    }
  }

  Agnode_t* fixed = agnode(g, (char*)"[-1] fixed", 1);
  Agedge_t* edge;
  for(unsigned int i = 0; i < get_active_joint_count(); i++) {
    std::string joint_name = get_active_joints()[i]->get_name();
    if(get_pcl()[i] == -1) {
      edge =
        agedge(g, fixed, nodes[i], const_cast<char*>(joint_name.c_str()), 1);
    } else {
      edge = agedge(g, nodes[get_pcl()[i]], nodes[i],
                    const_cast<char*>(joint_name.c_str()), 1);
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
               const_cast<char*>(joint_name.c_str()), 1);
    } else {
      edge =
        agedge(g, nodes[parent_index],
               nodes[get_chord_constraints()[i]->get_child_body()->get_index()],
               const_cast<char*>(joint_name.c_str()), 1);
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

    ksl_SE3_t* d_parent = &dhva_.d[parent_frame_index];
    ksl_SE3_t* d_child = &dhva_.d[child_frame_index];

    int parent_reference_frame_index = -1;
    FindReferenceFrame(*this, parent_frame_index,
                       &parent_reference_frame_index);
    bool parent_is_reference =
      (parent_frame_index == parent_reference_frame_index);

    int child_reference_frame_index = -1;
    FindReferenceFrame(*this, child_frame_index, &child_reference_frame_index);
    bool child_is_reference =
      (child_frame_index == child_reference_frame_index);

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
    Agraph_t* parent_body_graph = agsubg(g, parent_name, 1);
    agset(parent_body_graph, const_cast<char*>("label"),
          const_cast<char*>(parent_body_name.c_str()));
    agset(parent_body_graph, const_cast<char*>("fillcolor"),
          const_cast<char*>("lightgrey"));
    agset(parent_body_graph, const_cast<char*>("style"),
          const_cast<char*>("filled"));

    char child_name[400];
    sprintf(child_name, "cluster_%s", child_body_name.c_str());
    Agraph_t* child_body_graph = agsubg(g, child_name, 1);
    agset(child_body_graph, const_cast<char*>("fillcolor"),
          const_cast<char*>("lightgrey"));
    agset(child_body_graph, const_cast<char*>("style"),
          const_cast<char*>("filled"));
    agset(child_body_graph, const_cast<char*>("label"),
          const_cast<char*>(child_body_name.c_str()));

    Agnode_t* parent_node = agnode(parent_body_graph, parent_frame_name, 1);
    if(parent_is_reference) {
      agset(parent_node, const_cast<char*>("penwidth"), const_cast<char*>("3"));
    }

    Agnode_t* child_node = agnode(child_body_graph, child_frame_name, 1);
    if(child_is_reference) {
      agset(child_node, const_cast<char*>("penwidth"), const_cast<char*>("3"));
    }

    Agedge_t* edge = agedge(g, parent_node, child_node, name, 1);
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
  double pe = 0.0;
  for(auto& solver : active_solvers_) {
    pe += solver->GravitationalPotentialEnergy(dhva_);
  }
  return pe;
}

/*!
@brief compute kinetic energy for system

*/
double shift::core::Context::KineticEnergy() {
  assert(dhva_.v);
  double ke = 0.0;
  for(auto& solver : active_solvers_) {
    ke += solver->KineticEnergy(dhva_);
  }
  return ke;
}

unsigned int shift::core::Context::get_auxiliary_state_count() {
  unsigned int equation_count = 0;
  for(auto& force_module : active_force_modules_) {
    equation_count += force_module->get_equation_count();
  }
  for(auto& controller : active_controllers_) {
    equation_count += controller->get_equation_count();
  }
  return equation_count;
}

void shift::core::Context::InitializeAuxiliaryStates(double* y_auxiliary) {
  size_t offset = get_state_count();
  size_t k = 0;
  for(auto& force_module : active_force_modules_) {
    force_module->set_state_offset(offset + k);
    for(double value : force_module->get_q0()) {
      y_auxiliary[k++] = value;
    }
  }
  for(auto& controller : active_controllers_) {
    controller->set_state_offset(offset + k);
    for(double value : controller->get_q0()) {
      y_auxiliary[k++] = value;
    }
  }
  assert(k == get_auxiliary_state_count());
}

shift::core::body::Base*
shift::core::Context::FindActiveBody(const std::string& name,
                                     const Assembly* assembly) {
  if(name == "fixed") {
    return get_fixed_body();
  }
  std::vector<body::Base*> matches;
  for(auto list : {&active_articulated_bodies_, &active_free_bodies_}) {
    for(auto body : *list) {
      if(body->get_name() == name) {
        if(assembly && body->get_parent_assembly() == assembly) {
          return body;
        }
        matches.push_back(body);
      }
    }
  }
  for(auto list : {&active_articulated_bodies_, &active_free_bodies_}) {
    for(auto body : *list) {
      if(body->FullyQualifiedName(body->get_name()) == name) {
        return body;
      }
    }
  }
  if(matches.size() > 1) {
    std::string candidates;
    for(auto body : matches) {
      candidates += (candidates.empty() ? "'" : ", '") +
                    body->FullyQualifiedName(body->get_name()) + "'";
    }
    throw shift::core::Exception("Body name '" + name + "' is ambiguous: use " +
                                 "the fully qualified name, one of " +
                                 candidates);
  }
  return matches.empty() ? nullptr : matches[0];
}

int shift::core::Context::GetBodyLoadIndex(const body::Base* body) const {
  if(auto free = dynamic_cast<const body::Free*>(body)) {
    return free->get_free_index() < 0
             ? -1
             : static_cast<int>(non_rigid_joint_count_) + free->get_free_index();
  }
  for(size_t i = 0; i < active_articulated_bodies_.size(); i++) {
    if(active_articulated_bodies_[i] != body) {
      continue;
    }
    /* bodies attached by rigid joints move with their nearest non-rigid
    ancestor */
    int j = static_cast<int>(i);
    while(j > -1 && joint_lookup_[j] < 0) {
      j = pcl_[j];
    }
    return j > -1 ? joint_lookup_[j] : -1;
  }
  return -1;
}

ksl_screw_t shift::core::Context::GetBodyVelocity(int load_index) const {
  if(load_index < 0) {
    return ksl_screw(0, 0, 0, 0, 0, 0);
  }
  if(load_index >= static_cast<int>(non_rigid_joint_count_)) {
    return dhva_.free.v[load_index - non_rigid_joint_count_];
  }
  return dhva_.v[load_index];
}

void shift::core::Context::AddBodyLoad(int load_index,
                                       const ksl_coscrew_t& load) {
  if(load_index < 0) {
    return;
  }
  ksl_coscrew_t& sum =
    load_index >= static_cast<int>(non_rigid_joint_count_)
      ? free_loads_[load_index - non_rigid_joint_count_]
      : body_loads_[load_index];
  for(int i = 0; i < 6; i++) {
    sum.at[i] += load.at[i];
  }
}

ksl_coscrew_t shift::core::Context::GetBodyLoad(int load_index) const {
  if(load_index < 0) {
    return ksl_coscrew(0, 0, 0, 0, 0, 0);
  }
  const size_t index = static_cast<size_t>(load_index);
  if(index >= non_rigid_joint_count_) {
    const size_t free = index - non_rigid_joint_count_;
    return free < free_loads_.size() ? free_loads_[free]
                                     : ksl_coscrew(0, 0, 0, 0, 0, 0);
  }
  return index < body_loads_.size() ? body_loads_[index]
                                    : ksl_coscrew(0, 0, 0, 0, 0, 0);
}

std::string shift::core::Context::FindFile(const std::string& name) {
  const auto mapped = file_map_.find(name);
  const std::string file =
    mapped != file_map_.end()
      ? mapped->second
      : shift::core::utility::FindFile(
          name, filename_,
          (*this)["model_parameters"]["include_directories"]
            .get<std::vector<std::string>>());
  if(!file.empty()) {
    RecordFile(file);
    std::error_code error;
    const auto absolute = std::filesystem::absolute(file, error);
    if(!error && !shift::core::utility::IsResourcePath(file)) {
      file_names_[name] = absolute.lexically_normal().string();
    }
  }
  return file;
}

void shift::core::Context::RecordFile(const std::string& path) {
  if(shift::core::utility::IsResourcePath(path)) {
    return; // built into the viewer
  }
  std::error_code error;
  std::filesystem::path absolute = std::filesystem::absolute(path, error);
  if(error) {
    return;
  }
  const std::string file = absolute.lexically_normal().string();
  if(std::find(files_read_.begin(), files_read_.end(), file) ==
     files_read_.end()) {
    files_read_.push_back(file);
  }
}

void shift::core::Context::EvaluateConstraints() {
  rms_constraint_error_ = jacobian_.EvaluateConstraints();
}

void shift::core::Context::SetForces(const double t,
                                     const std::vector<double>& y,
                                     std::vector<double>& dy) {
  force_time_ = t;
  std::fill(f_.begin(), f_.end(), 0);
  body_loads_.resize(non_rigid_joint_count_);
  std::fill(body_loads_.begin(), body_loads_.end(),
            ksl_coscrew(0, 0, 0, 0, 0, 0));
  std::fill(free_loads_.begin(), free_loads_.end(),
            ksl_coscrew(0, 0, 0, 0, 0, 0));
  UpdateFreeDV(dhva_, y.data() + get_free_state_offset());
  /* contact detection is performed before all other force modules */
  if(model_parameters_.is_contact_detection_enabled()) {
    DetectContacts();
  }
  for(auto& joint : active_joints_) {
    joint->Service(y);
  }
  /* signal chain: sensors measure the state, controllers read the sensors
  and drive joints, force modules (e.g. tires) apply forces */
  for(auto& sensor : active_sensors_) {
    sensor->Update(y.data());
  }
  for(auto& controller : active_controllers_) {
    controller->Service(y.data(), dy.data());
  }
  for(auto& force_module : active_force_modules_) {
    force_module->Service(y.data(), dy.data());
  }
  /* the free bodies move under the loads applied above */
  EvaluateFreeBodies(y.data(), dy.data());
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

  /* convert angular velocities to time derivative of quaternions;
  ksl_omega_to_dquaternion expects the angular velocity in the parent frame,
  joints with sense >= 0 express it in the child frame (see ksl_hsrx) and
  inverse joints in the original parent frame (see ksl_hsrxinv) */
  for(auto& index : quaternion_indices_) {
    ksl_quaternion_t* quaternion = (ksl_quaternion_t*)(&q[index.q]);
    const ksl_vec3_t* omega = (const ksl_vec3_t*)(&dq[index.dq]);
    const bool has_translation = index.translation_q > -1;
    const ksl_vec3_t* v =
      has_translation ? (const ksl_vec3_t*)(&dq[index.translation_dq]) : nullptr;
    ksl_vec3_t* dt =
      has_translation ? (ksl_vec3_t*)(&dy[index.translation_q]) : nullptr;

    if(index.child_frame) {
      ksl_mat3x3_t R;
      ksl_quaternion_toMat3x3(quaternion, &R);
      ksl_vec3_t omega_parent;
      ksl_product_drv(&R, omega, &omega_parent);
      ksl_omega_to_dquaternion(&omega_parent, quaternion,
                               (ksl_quaternion_t*)(&dy[index.q]));
      if(has_translation) {
        ksl_product_drv(&R, v, dt);
      }
    } else {
      ksl_omega_to_dquaternion(omega, quaternion,
                               (ksl_quaternion_t*)(&dy[index.q]));
      if(has_translation) {
        ksl_vec3_copy(v, dt);
      }
    }
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
      QuaternionIndex index;
      index.q = q_index;
      index.dq = dq_index;
      index.child_frame = joint->get_sense() >= 0;
      quaternion_indices_.push_back(index);
      q_index += 4;
      dq_index += 3;
    } else if(joint->get_type() == "float") {
      /* q = [t, quaternion], dq = [v, omega], see joint::float_d */
      QuaternionIndex index;
      index.translation_q = q_index;
      index.translation_dq = dq_index;
      index.child_frame = joint->get_sense() >= 0;
      index.q = q_index + 3;
      index.dq = dq_index + 3;
      quaternion_indices_.push_back(index);
      q_index += 7;
      dq_index += 6;
    } else {
      assert(joint->get_q_count() == joint->get_dq_count());
      for(unsigned int i = 0; i < joint->get_q_count(); i++) {
        non_quaternion_indices_.push_back({q_index++, dq_index++});
      }
    }
  }
  /* the internal states of force modules and controllers follow q and dq
  (see InitializeAuxiliaryStates); the modules set their rates themselves */
}

void shift::core::Context::Simulate() {

  // for(auto& joint : active_joints_) {
  //   std::cout << joint->get_name() << ": " << joint->get_dq_index() << std::endl;
  // }
  if(active_integrators_.size() == 0 || active_solvers_.size() == 0) {
    throw shift::core::Exception(
      "Error: no active integrators or solvers have been created");
  }
  assert(active_integrators_.size());
  assert(active_solvers_.size());
  auto& integrator = active_integrators_[0];
  SHIFT_INFO("Starting simulation");
  const auto wall_start = std::chrono::steady_clock::now();
  contact_detection_seconds_ = 0.0;
  contact_detection_calls_ = 0;
  broad_phase_calls_ = 0;

  /* report simulated time over elapsed wall-clock time, also when the
  integrator stops early, so the report covers the time actually simulated */
  auto report_real_time_factor = [&]() {
    const double wall_elapsed = std::chrono::duration<double>(
                                  std::chrono::steady_clock::now() - wall_start)
                                  .count();
    double sim_time = 0.0;
    double delta_time = 0.0;
    integrator->get_current_time(sim_time, delta_time);
    const double real_time_factor =
      wall_elapsed > 0.0 ? sim_time / wall_elapsed : 0.0;
    SHIFT_INFO("Simulated time: " + std::to_string(sim_time) +
               " s, wall-clock time: " + std::to_string(wall_elapsed) +
               " s, real-time factor: " + std::to_string(real_time_factor));
    if(contact_detection_calls_ > 0) {
      SHIFT_INFO("Contact detection: " +
                 std::to_string(contact_detection_seconds_) +
                 " s wall-clock in " +
                 std::to_string(contact_detection_calls_) + " calls, " +
                 std::to_string(broad_phase_calls_) + " broad phases");
    }
  };

  HoldInitialStates();
  try {
    integrator->Integrate();
  } catch(...) {
    report_real_time_factor();
    throw;
  }
  report_real_time_factor();
}

/* solve the n x n system A x = b in place by Gaussian elimination with
partial pivoting; A is row major */
static void SolveDense(std::vector<double>& A, std::vector<double>& b,
                       size_t n) {
  for(size_t c = 0; c < n; c++) {
    size_t pivot = c;
    for(size_t r = c + 1; r < n; r++) {
      if(std::fabs(A[r * n + c]) > std::fabs(A[pivot * n + c])) {
        pivot = r;
      }
    }
    for(size_t k = 0; k < n; k++) {
      std::swap(A[c * n + k], A[pivot * n + k]);
    }
    std::swap(b[c], b[pivot]);
    for(size_t r = c + 1; r < n; r++) {
      const double factor = A[r * n + c] / A[c * n + c];
      for(size_t k = c; k < n; k++) {
        A[r * n + k] -= factor * A[c * n + k];
      }
      b[r] -= factor * b[c];
    }
  }
  for(size_t c = n; c-- > 0;) {
    for(size_t k = c + 1; k < n; k++) {
      b[c] -= A[c * n + k] * b[k];
    }
    b[c] /= A[c * n + c];
  }
}

void shift::core::Context::HoldInitialStates() {
  std::vector<force_module::Base*> modules;
  for(auto& module : active_force_modules_) {
    if(module->holds()) {
      modules.push_back(module);
    }
  }
  if(modules.empty()) {
    return;
  }
  const size_t n = modules.size();
  /* the integrator and solver start as they will in Integrate: the solver
  corrects the initial coordinates to satisfy the constraints, and reads the
  integrator's integral of the constraint error */
  active_integrators_[0]->Initialize();
  auto& solver = active_solvers_[0];
  solver->Initialize();
  std::vector<double> y(get_state_count() + get_auxiliary_state_count(), 0.0);
  memcpy(y.data(), get_q().data(), get_state_count() * sizeof(double));
  InitializeAuxiliaryStates(y.data() + get_state_count());
  std::vector<double> dy(y.size(), 0.0);

  /* the accelerations of the held joints are linear in the forces on them
  at a given state: a = a0 + S f, column j of S from a unit force on joint
  j alone */
  auto accelerations = [&](const std::vector<double>& forces) {
    for(size_t j = 0; j < n; j++) {
      modules[j]->set_hold_force(true, forces[j]);
    }
    if(solver->Solve(0.0, y, dy)) {
      throw shift::core::Exception("Holding the initial states: the "
                                   "equations of motion could not be "
                                   "solved");
    }
    std::vector<double> a(n);
    for(size_t k = 0; k < n; k++) {
      a[k] = dy[get_q_count() + modules[k]->get_held_joint()->get_dq_index()];
    }
    return a;
  };
  const std::vector<double> a0 = accelerations(std::vector<double>(n, 0.0));
  std::vector<double> S(n * n);
  for(size_t j = 0; j < n; j++) {
    std::vector<double> unit(n, 0.0);
    unit[j] = 1.0;
    const std::vector<double> a = accelerations(unit);
    for(size_t k = 0; k < n; k++) {
      S[k * n + j] = a[k] - a0[k];
    }
  }

  /* the least forces with S f = -a0: f = S^T z, (S S^T + e I) z = -a0; the
  small e lets modules that share a load, such as two cylinders side by
  side, share it equally */
  std::vector<double> A(n * n, 0.0);
  double trace = 0.0;
  for(size_t r = 0; r < n; r++) {
    for(size_t c = 0; c < n; c++) {
      for(size_t k = 0; k < n; k++) {
        A[r * n + c] += S[r * n + k] * S[c * n + k];
      }
    }
    trace += A[r * n + r];
  }
  for(size_t r = 0; r < n; r++) {
    A[r * n + r] += 1e-12 * trace / n;
  }
  std::vector<double> z(a0);
  for(auto& value : z) {
    value = -value;
  }
  SolveDense(A, z, n);
  std::vector<double> forces(n, 0.0);
  for(size_t j = 0; j < n; j++) {
    for(size_t k = 0; k < n; k++) {
      forces[j] += S[k * n + j] * z[k];
    }
  }

  /* check: the held joints' accelerations with these forces */
  const std::vector<double> a = accelerations(forces);
  double residual = 0.0, scale = 0.0;
  for(size_t k = 0; k < n; k++) {
    residual = std::max(residual, std::fabs(a[k]));
    scale = std::max(scale, std::fabs(a0[k]));
  }
  if(residual > 1e-6 * scale + 1e-12) {
    SHIFT_WARNING("Holding the initial states: the held joints keep an "
                  "acceleration of " + std::to_string(residual) +
                  " (from " + std::to_string(scale) +
                  "); they may not all be able to hold the load");
  }
  for(size_t j = 0; j < n; j++) {
    modules[j]->set_hold_force(false, 0.0);
    modules[j]->Hold(forces[j], y.data());
    SHIFT_INFO("Force module " + modules[j]->get_name() + " holds joint " +
               modules[j]->get_held_joint()->get_name() + " with " +
               std::to_string(forces[j]));
  }
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
  /* the force modules' outputs (pressures, forces) of this state, then the
  sensors', which may report the forces */
  for(auto& force_module : active_force_modules_) {
    force_module->UpdateOutputs(time, y.data());
  }
  for(auto& sensor : active_sensors_) {
    sensor->UpdateOutputs(time, y.data());
  }
  /* the changes of the terrains' surfaces since the previous output */
  for(auto& terrain : active_terrain_) {
    terrain->UpdateOutputAll();
  }
  for(auto& logger : active_loggers_) {
    logger->WriteStates(time, y);
  }
  if(state_observer_) {
    state_observer_(time, y);
  }
}

ksl_SE3_t
shift::core::Context::get_body_origin_pose(shift::core::body::Base* body) {
  return get_body_origin_pose(body, dhva_);
}

ksl_SE3_t shift::core::Context::get_body_origin_pose(
  shift::core::body::Base* body, const shift::core::topology::DHVA& dhva) {
  if(auto free = dynamic_cast<const body::Free*>(body)) {
    const int k = free->get_free_index();
    return k >= 0 && k < static_cast<int>(dhva.free.get_count())
             ? dhva.free.d[k]
             : ksl_SE3_identity();
  }
  // frames in the tree are evaluated directly, including transforms of rigid
  // joints compressed into them by CompressRigidJoints
  const shift::core::Frame* origin = body->get_origin();
  if(origin->is_in_tree() && origin->get_index() >= 0) {
    return dhva.d[origin->get_index()];
  }
  // otherwise recover the origin from any other frame of the body in the tree
  for(auto& frame : body->get_frames()) {
    if(frame->is_in_tree() && frame->get_index() >= 0) {
      ksl_SE3_t d_origin;
      ksl_product_ddinv(&dhva.d[frame->get_index()], frame->get_d_wrt_origin(),
                        &d_origin);
      return d_origin;
    }
  }
  /* a body attached by rigid joints: placed by the chain's displacement
  from the body above it, which its own reference frame, not in the tree,
  does not hold */
  ksl_SE3_t d_reference = ksl_SE3_identity();
  auto placement = rigid_placements_.find(body);
  if(placement != rigid_placements_.end()) {
    const int above = placement->second.first;
    const ksl_SE3_t d_above =
      above >= 0 ? dhva.d[above] : ksl_SE3_identity();
    ksl_product_dd(&d_above, &placement->second.second, &d_reference);
  } else {
    const shift::core::Frame* reference = body->get_reference_frame();
    if(reference->get_index() >= 0) {
      d_reference = dhva.d[reference->get_index()];
    }
  }
  ksl_SE3_t d_origin;
  ksl_product_dd(&d_reference, body->get_origin()->get_d_wrt_reference(),
                 &d_origin);
  return d_origin;
}

void shift::core::Context::RegisterLogValue(
  shift::core::parameter::Value* value) {
  for(auto& logger : active_loggers_) {
    logger->RegisterLogValue(value);
  }
}

void shift::core::Context::UnregisterLogValue(
  shift::core::parameter::Value* value) {
  for(auto& logger : active_loggers_) {
    logger->UnregisterLogValue(value);
  }
}

shift::core::collision_geometry::Base*
shift::core::Context::FindCollisionGeometryPrototype(
  const std::string& key) const {
  auto it = collision_geometry_prototypes_.find(key);
  return it == collision_geometry_prototypes_.end() ? nullptr : it->second;
}

void shift::core::Context::AddCollisionGeometryPrototype(
  const std::string& key, shift::core::collision_geometry::Base* geometry) {
  collision_geometry_prototypes_.emplace(key, geometry);
}

std::shared_ptr<const shift::core::geometry::MeshAsset>
shift::core::Context::FindMeshAsset(const std::string& key) const {
  auto it = mesh_assets_.find(key);
  return it == mesh_assets_.end() ? nullptr : it->second;
}

void shift::core::Context::AddMeshAsset(
  const std::string& key,
  std::shared_ptr<const shift::core::geometry::MeshAsset> asset) {
  mesh_assets_.emplace(key, std::move(asset));
}

void shift::core::Context::SetupContactDetection() {
  contact_geometry_.clear();
  contact_pairs_.clear();
  contacts_.clear();
  contact_candidates_.clear();
  contact_candidates_valid_ = false;
  for(auto geometry : active_collision_geometry_) {
    if(geometry->is_loaded()) {
      contact_geometry_.push_back(geometry);
    }
  }
  const size_t n = contact_geometry_.size();
  contact_group_.resize(n);
  contact_planes_.resize(n);
  broad_phase_centers_.resize(n);
  contact_margins_.resize(n);
  sweep_order_.clear();
  for(size_t i = 0; i < n; i++) {
    body::Base* body = contact_geometry_[i]->get_parent_body();
    contact_group_[i] = body->is_articulated() ? GetBodyLoadIndex(body)
                                               : -2 - static_cast<int>(i);
    contact_planes_[i] =
      dynamic_cast<const collision_geometry::Plane*>(contact_geometry_[i]);
    if(!contact_planes_[i]) {
      sweep_order_.push_back(static_cast<int>(i));
    }
  }
  if(!contact_geometry_.empty()) {
    SHIFT_INFO("Contact detection " +
               std::string(model_parameters_.is_contact_detection_enabled()
                             ? "enabled"
                             : "disabled") +
               " for " + std::to_string(contact_geometry_.size()) +
               " collision geometries");
  }
}

void shift::core::Context::ExcludeContact(const body::Base* a,
                                          const body::Base* b) {
  excluded_contacts_.insert(std::minmax(a, b));
  contact_candidates_valid_ = false;
}

/* the poses of the geometries are shared by threads with at least this many
each: a pose takes well under a microsecond, and starting threads costs more
than it saves for less */
static constexpr int kPoseGrain = 512;

/* the tests of the candidate pairs are shared by threads if the last round of
them took this long (s). The number of pairs says little: most of them end at
the bounding spheres, or are a plane and a sphere, while a mesh pair in contact
takes tens of microseconds. */
static constexpr double kParallelTestSeconds = 100e-6;

static int ThreadCount(int work) {
  return std::max(1, std::min(omp_get_max_threads(), work / kPoseGrain));
}

/* the margin of a bounding sphere in the broad phase, as a fraction of its
radius: a larger margin repeats the broad phase less often, with more
candidates to test */
static constexpr double kBroadPhaseMargin = 0.25;

static double DistanceSquared(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

bool shift::core::Context::is_contact_allowed(int i, int j) const {
  const auto a = contact_geometry_[i];
  const auto b = contact_geometry_[j];
  if(contact_group_[i] == contact_group_[j] ||
     a->get_parent_body() == b->get_parent_body()) {
    return false;
  }
  return excluded_contacts_.empty() ||
         !excluded_contacts_.count(
           std::minmax<const body::Base*>(a->get_parent_body(),
                                          b->get_parent_body()));
}

void shift::core::Context::UpdateContactCandidates() {
  broad_phase_calls_++;
  const size_t n = contact_geometry_.size();
  for(size_t i = 0; i < n; i++) {
    const auto g = contact_geometry_[i];
    broad_phase_centers_[i] = g->get_d_absolute().t;
    contact_margins_[i] =
      contact_planes_[i] ? 0.0 : kBroadPhaseMargin * g->get_bounding_radius();
  }
  for(auto& entry : contact_pairs_) {
    entry.second.in_scope = false;
  }
  contact_candidates_.clear();
  auto add = [&](int i, int j) {
    const uint64_t lo = static_cast<uint64_t>(std::min(i, j));
    const uint64_t hi = static_cast<uint64_t>(std::max(i, j));
    auto [it, inserted] = contact_pairs_.try_emplace((hi << 32) | lo);
    auto& pair = it->second;
    if(inserted) {
      pair.a = contact_geometry_[lo];
      pair.b = contact_geometry_[hi];
    }
    pair.in_scope = true;
    ContactCandidate candidate;
    candidate.pair = &pair;
    candidate.a = static_cast<int>(lo);
    candidate.b = static_cast<int>(hi);
    contact_candidates_.push_back(candidate);
  };

  /* planes against all bounded geometry */
  for(size_t p = 0; p < n; p++) {
    if(!contact_planes_[p]) {
      continue;
    }
    for(int i : sweep_order_) {
      if(is_contact_allowed(static_cast<int>(p), i)) {
        add(static_cast<int>(p), i);
      }
    }
  }

  /* sweep and prune of the enlarged bounding spheres along the axis with the
  largest spread of their centers; the order changes little between calls,
  so insertion sort is nearly linear */
  const size_t count = sweep_order_.size();
  if(count >= 2) {
    double sum[3] = {0, 0, 0}, sum2[3] = {0, 0, 0};
    for(int i : sweep_order_) {
      for(int k = 0; k < 3; k++) {
        const double c = broad_phase_centers_[i].at[k];
        sum[k] += c;
        sum2[k] += c * c;
      }
    }
    int axis = 0;
    double spread = -1.0;
    for(int k = 0; k < 3; k++) {
      const double variance = sum2[k] / count - std::pow(sum[k] / count, 2);
      if(variance > spread) {
        spread = variance;
        axis = k;
      }
    }
    sweep_axis_ = axis;
    auto radius = [&](int i) {
      return contact_geometry_[i]->get_bounding_radius() + contact_margins_[i];
    };
    auto lower = [&](int i) {
      return broad_phase_centers_[i].at[axis] - radius(i);
    };
    for(size_t i = 1; i < count; i++) {
      const int index = sweep_order_[i];
      const double key = lower(index);
      size_t j = i;
      while(j > 0 && lower(sweep_order_[j - 1]) > key) {
        sweep_order_[j] = sweep_order_[j - 1];
        j--;
      }
      sweep_order_[j] = index;
    }
    for(size_t i = 0; i < count; i++) {
      const int ia = sweep_order_[i];
      const ksl_vec3_t& ca = broad_phase_centers_[ia];
      const double ra = radius(ia);
      const double upper = ca.at[axis] + ra;
      for(size_t j = i + 1; j < count && lower(sweep_order_[j]) <= upper; j++) {
        const int ib = sweep_order_[j];
        const double r = ra + radius(ib);
        if(DistanceSquared(ca, broad_phase_centers_[ib]) <= r * r &&
           is_contact_allowed(ia, ib)) {
          add(ia, ib);
        }
      }
    }
  }

  for(auto it = contact_pairs_.begin(); it != contact_pairs_.end();) {
    if(!it->second.in_scope) {
      if(it->second.collision) {
        SHIFT_DEBUG("Collision geometry '" + it->second.a->get_name() +
                    "' and '" + it->second.b->get_name() + "' separated");
      }
      it = contact_pairs_.erase(it);
    } else {
      ++it;
    }
  }
  contact_candidates_valid_ = true;
}

/* the narrow phase of a candidate pair: a cheap test of the bounding spheres
(or of the bounding sphere against a plane), then TestContact */
static void TestCandidate(shift::core::collision_geometry::ContactPair& pair,
                          const shift::core::collision_geometry::Plane* plane_a,
                          const shift::core::collision_geometry::Plane* plane_b) {
  const auto a = pair.a;
  const auto b = pair.b;
  bool near = a->is_contact_enabled() && b->is_contact_enabled();
  if(near && (plane_a || plane_b)) {
    const auto plane = plane_a ? plane_a : plane_b;
    const auto other = plane_a ? b : a;
    const ksl_vec3_t n = plane->get_normal();
    const ksl_vec3_t& q = plane->get_point();
    const ksl_vec3_t& c = other->get_d_absolute().t;
    const double height =
      n.x * (c.x - q.x) + n.y * (c.y - q.y) + n.z * (c.z - q.z);
    near = height <= other->get_bounding_radius();
  } else if(near) {
    const double r = a->get_bounding_radius() + b->get_bounding_radius();
    near = DistanceSquared(a->get_d_absolute().t, b->get_d_absolute().t) <=
           r * r;
  }
  if(near) {
    shift::core::collision_geometry::TestContact(pair);
  } else {
    pair.collision = false;
  }
}

void shift::core::Context::AcceptStep(const std::vector<double>&) {
  /* the contact normals of the accepted state, the last evaluation of the
  step */
  for(auto& entry : contact_pairs_) {
    auto& pair = entry.second;
    pair.accepted_normal =
      pair.collision ? pair.normal : ksl_vec3_t{{0.0, 0.0, 0.0}};
  }
  /* the changes of deformable terrain in that evaluation */
  for(auto terrain : active_terrain_) {
    terrain->AcceptStepAll();
  }
}

void shift::core::Context::DetectContacts() {
  const auto start = std::chrono::steady_clock::now();
  contacts_.clear();
  const int n = static_cast<int>(contact_geometry_.size());
  if(n < 2) {
    return;
  }
#pragma omp parallel for schedule(static) num_threads(ThreadCount(n))
  for(int i = 0; i < n; i++) {
    const auto geometry = contact_geometry_[i];
    geometry->UpdatePose(get_body_origin_pose(geometry->get_parent_body()));
  }

  /* repeat the broad phase once a geometry leaves its margin */
  bool moved = !contact_candidates_valid_;
  for(int i = 0; i < n && !moved; i++) {
    const double margin = contact_margins_[i];
    moved = !contact_planes_[i] &&
            DistanceSquared(contact_geometry_[i]->get_d_absolute().t,
                            broad_phase_centers_[i]) > margin * margin;
  }
  if(moved) {
    UpdateContactCandidates();
  }

  /* a pair at a time: the pairs that take long are few, and neighbors in the
  list, so larger chunks leave them all to one thread. The pairs do not depend
  on each other, so the threads do not change the results. */
  const int m = static_cast<int>(contact_candidates_.size());
  const int threads =
    contact_test_seconds_ > kParallelTestSeconds ? omp_get_max_threads() : 1;
  const auto tests_start = std::chrono::steady_clock::now();
#pragma omp parallel for schedule(dynamic, 1) num_threads(threads)
  for(int k = 0; k < m; k++) {
    ContactCandidate& candidate = contact_candidates_[k];
    candidate.was_in_contact = candidate.pair->collision;
    TestCandidate(*candidate.pair, contact_planes_[candidate.a],
                  contact_planes_[candidate.b]);
  }
  contact_test_seconds_ =
    threads * std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            tests_start)
                .count();

  /* in the order of the candidates, the same for any number of threads */
  for(const auto& candidate : contact_candidates_) {
    auto& pair = *candidate.pair;
    if(pair.collision != candidate.was_in_contact) {
      SHIFT_DEBUG("Collision geometry '" + pair.a->get_name() + "' and '" +
                  pair.b->get_name() + "' " +
                  (pair.collision ? "in contact" : "separated"));
    }
    if(pair.collision) {
      contacts_.push_back(&pair);
    }
  }
  contact_detection_calls_++;
  contact_detection_seconds_ +=
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}
