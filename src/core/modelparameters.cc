#include <iostream>

#include "shift/core/factory.h"
#include "shift/core/modelparameters.h"

#include "ksl/print.h"
#include "ksl/vec3.h"

void shift::core::ModelParameters::appendSolver(
  std::unique_ptr<shift::core::solver::Base>& solver) {
  solvers_.push_back(std::move(solver));
}

void shift::core::ModelParameters::appendIntegrator(
  std::unique_ptr<shift::core::integrator::Base>& integrator) {
  integrators_.push_back(std::move(integrator));
}

void shift::core::ModelParameters::appendLogger(
  std::unique_ptr<shift::core::logger::Base>& logger) {
  loggers_.push_back(std::move(logger));
}

void shift::core::ModelParameters::addParameters() {

  addParameter("version", &version_, "Shift version string.", false);

  addParameter("gravity", &gravity_,
               "Gravity vector, expressed in the inertial reference frame; "
               "default: [0.0, 0.0, 0.0]",
               false);

  addParameter(
    "include_directories", &include_directories_,
    "External include directories, these directories will be searched when "
    "searching for external files.",
    false);

  addParameter("enable_contact_detection", &contact_detection_enabled_,
               "Globally enable contact detection for Shift simulation", false);

  addParameter(
    "epsilon_rank_determination", &epsilon_rank_determination_,
    "Epsilon used to determine rank of Jacobian matrix using Gaussian "
    "Elimination with Complete Pivoting (GECP).",
    false);

  addParameter("units", &units_, "Specify units for simulation", false);

  addParameter("solvers", &solvers_, "", false);

  addParameter("integrators", &integrators_, "", false);

  addParameter("loggers", &loggers_, "", false);
}

void shift::core::ModelParameters::initialize() {}
