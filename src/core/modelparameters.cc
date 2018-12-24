#include <iostream>

#include "shift-core/factory.h"
#include "shift-core/modelparameters.h"

#include "ksl/print.h"
#include "ksl/vec3.h"

void Shift::ModelParameters::appendSolver(
  std::unique_ptr<Shift::Solver::SolverBase>& solver) {
  solvers_.push_back(std::move(solver));
}

void Shift::ModelParameters::appendIntegrator(
  std::unique_ptr<Shift::Integrator::IntegratorBase>& integrator) {
  integrators_.push_back(std::move(integrator));
}

void Shift::ModelParameters::appendLogger(
  std::unique_ptr<Shift::Logger::LoggerBase>& logger) {
  loggers_.push_back(std::move(logger));
}

void Shift::ModelParameters::addKeys() {
  SerialEntry version("version", SerialType::STRING, &version_);
  version.setRequired(false);
  version.setDescription("Shift version.");
  addSerialEntry(version);

  SerialEntry gravity("gravity", SerialType::KSL_VEC3, &gravity_);
  gravity.setRequired(false);
  gravity.setDescription("Gravity, expressed in the inertial reference frame. "
                         "Default: [0.0, 0.0, 0.0].");
  addSerialEntry(gravity);

  SerialEntry include_directories(
    "include_directories", SerialType::STRING_ARRAY, &include_directories_);
  include_directories.setRequired(false);
  include_directories.setDescription(
    "External include directories, these directories will be searched when "
    "searching for external files.");
  addSerialEntry(include_directories);

  SerialEntry contact_detection_enabled("enable_contact_detection",
                                        SerialType::BOOLEAN,
                                        &contact_detection_enabled_);
  contact_detection_enabled.setRequired(false);
  contact_detection_enabled.setDescription(
    "Indicate whether contact detection should be performed during "
    "simulation.");
  addSerialEntry(contact_detection_enabled);
}

void Shift::ModelParameters::readJSON(Json::Value& value) {
  Serialize::readJSON(value);

  if(value.isMember("units")) {
    units_.readJSON(value["units"]);
  }

  if(value.isMember("solver")) {
    if(value["solver"].isArray()) {
      for(unsigned int i = 0; i < value["solver"].size(); i++) {

        if(value["solver"][i].isMember("type") &&
           value["solver"][i]["type"].isString()) {
          std::string solverType = value["solver"][i]["type"].asString();
          auto solver =
            Shift::Factory<Shift::Solver::SolverBase>::get()->create(
              solverType);
          try {
            solver->readJSON(value["solver"][i]);
          } catch(const char* msg) {
            std::cerr << msg << std::endl;
          }
          appendSolver(solver);
        }
      }
    } else if(value["solver"].isObject()) {
      if(value["solver"].isMember("type") &&
         value["solver"]["type"].isString()) {
        std::string solverType = value["solver"]["type"].asString();
        auto solver =
          Shift::Factory<Shift::Solver::SolverBase>::get()->create(solverType);
        try {
          solver->readJSON(value["solver"]);
        } catch(const char* msg) {
          std::cerr << msg << std::endl;
        }
        appendSolver(solver);
      }
    }
  } else {
    auto solver =
      Shift::Factory<Shift::Solver::SolverBase>::get()->create("rnca");
    appendSolver(solver);
  }

  if(value.isMember("integrator")) {
    if(value["integrator"].isArray()) {
      for(unsigned int i = 0; i < value["solver"].size(); i++) {
        if(value["integrator"][i].isMember("type") &&
           value["integrator"][i]["type"].isString()) {
          std::string integratorType =
            value["integrator"][i]["type"].asString();
          auto integrator =
            Shift::Factory<Shift::Integrator::IntegratorBase>::get()->create(
              integratorType);
          integrator->readJSON(value["integrator"][i]);
          appendIntegrator(integrator);
        }
      }
    } else if(value["integrator"].isObject()) {
      if(value["integrator"].isMember("type") &&
         value["integrator"]["type"].isString()) {
        std::string integratorType = value["integrator"]["type"].asString();
        auto integrator =
          Shift::Factory<Shift::Integrator::IntegratorBase>::get()->create(
            integratorType);
        integrator->readJSON(value["integrator"]);
        appendIntegrator(integrator);
      }
    }
  } else {
    auto integrator =
      Shift::Factory<Shift::Integrator::IntegratorBase>::get()->create(
        "ode113");
    appendIntegrator(integrator);
  }

  if(value.isMember("logger")) {
    if(value["logger"].isArray()) {
      for(unsigned int i = 0; i < value["logger"].size(); i++) {
        if(value["logger"][i].isMember("type") &&
           value["logger"][i]["type"].isString()) {
          std::string loggerType = value["logger"][i]["type"].asString();
          auto logger =
            Shift::Factory<Shift::Logger::LoggerBase>::get()->create(
              loggerType);
          logger->readJSON(value["logger"][i]);
          appendLogger(logger);
        }
      }
    } else if(value["logger"].isObject()) {
      if(value["logger"].isMember("type") &&
         value["logger"]["type"].isString()) {
        std::string loggerType = value["logger"]["type"].asString();
        auto logger =
          Shift::Factory<Shift::Logger::LoggerBase>::get()->create(loggerType);
        logger->readJSON(value["logger"]);
        appendLogger(logger);
      }
    }
  }
}

void Shift::ModelParameters::print() {
  std::cout << "model_parameters: " << std::endl;
  std::cout << "  version: " << version_ << std::endl;
  ksl_vec3_print(stdout, &gravity_, "  gravity: ");
  std::cout << "  include_directories: [";
  for(unsigned int i = 0; i < include_directories_.size(); i++) {
    std::cout << include_directories_[i];
    if(i != include_directories_.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
  units_.print();
}
