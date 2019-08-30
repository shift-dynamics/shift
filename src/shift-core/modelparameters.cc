#include <iostream>

#include "shift-core/factory.h"
#include "shift-core/modelparameters.h"

#include "ksl/print.h"
#include "ksl/vec3.h"

void Shift::ModelParameters::appendSolver(
  std::unique_ptr<Shift::Solver::Base>&) {
  // solvers_.push_back(std::move(solver));
}

void Shift::ModelParameters::appendIntegrator(
  std::unique_ptr<Shift::Integrator::Base>&) {
  // integrators_.push_back(std::move(integrator));
}

void Shift::ModelParameters::appendLogger(
  std::unique_ptr<Shift::Logger::Base>&) {
  // loggers_.push_back(std::move(logger));
}

void Shift::ModelParameters::addParameters() {
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

  addParameter(
    "infinity_norm_multiplier", &infinity_norm_muliplier_,
    "The infinity norm of each Jacobian matrix's *B* matrix is used as a "
    "heuristic to determine when to reapply Gaussian elimination with complete "
    "row and column pivoting on as-needed basis. GECP will be reapplied when "
    "the infinity norm reaches this multiple of the initial starting value -- "
    "this setting defaults to 2.0.",
    false);

  addParameter("units", &units_, "Specify units for simulation", false);

  addParameter("solver", &solvers_, "", false);

  addParameter("integrator", &integrators_, "", false);

  addParameter("logger", &loggers_, "", false);
}

void Shift::ModelParameters::initialize() {
  // for(auto& solver : solvers_) {
  //   solver->setContext(context_);
  //   solver->initialize();
  // }
  // for(auto& integrator : integrators_) {
  //   integrator->setContext(context_);
  //   integrator->initialize();
  // }
  // for(auto& logger : loggers_) {
  //   logger->setContext(context_);
  //   logger->initialize();
  // }
}
