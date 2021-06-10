#include <iostream>

#include "shift/core/Factory.h"
#include "shift/core/ModelParameters.h"

#include "ksl/print.h"
#include "ksl/vec3.h"

void shift::core::ModelParameters::AppendSolver(
  std::unique_ptr<shift::core::solver::Base>& solver) {
  solvers_.PushBack(std::move(solver));
}

void shift::core::ModelParameters::AppendIntegrator(
  std::unique_ptr<shift::core::integrator::Base>& integrator) {
  integrators_.PushBack(std::move(integrator));
}

void shift::core::ModelParameters::AppendLogger(
  std::unique_ptr<shift::core::logger::Base>& logger) {
  loggers_.PushBack(std::move(logger));
}

void shift::core::ModelParameters::DeclareParameters() {

  DeclareParameter("version", &version_, "Requires Shift version string.", false);

  DeclareParameter("gravity", &gravity_,
               "Gravity vector, expressed in the inertial reference frame; "
               "default: [0.0, 0.0, 0.0]",
               false);

  DeclareParameter(
    "include_directories", &include_directories_,
    "External include directories, these directories will be searched when "
    "searching for external files.",
    false);

  DeclareParameter("enable_contact_detection", &contact_detection_enabled_,
               "Globally enable contact detection for Shift simulation", false);

  DeclareParameter(
    "epsilon_rank_determination", &epsilon_rank_determination_,
    "Epsilon used to determine rank of Jacobian matrix using Gaussian "
    "Elimination with Complete Pivoting (GECP).",
    false);

  DeclareParameter(
    "print_constraint_error", &print_constraint_error_,
    "On initial model loading, print constraint error resolution during "
    "Newton-Raphson iteration",
    false);

  DeclareParameter("units", &units_, "Specify units for simulation", false);

  DeclareParameter("solvers", &solvers_, "", false);

  DeclareParameter("integrators", &integrators_, "", false);

  DeclareParameter("loggers", &loggers_, "", false);

}

void shift::core::ModelParameters::Initialize() {}
