#include <fstream>
#include <iostream>

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

void Shift::Context::readJSON(const std::string filename) {
  std::ifstream file(filename);
  file >> root_;
  if(root_.isMember("model_parameters")) {
    modelParameters_.readJSON(root_["model_parameters"]);
  }
  fixed_.readJSON(root_, true);
}

void Shift::Context::writeJSON() {}

void Shift::Context::writeJSON(const std::string filename) {}

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
