#ifndef _SHIFT_MODELPARAMETERS_H_
#define _SHIFT_MODELPARAMETERS_H_

#include <json/json.h>
#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift-core/base.h"
#include "shift-core/integrators/integrator.h"
#include "shift-core/loggers/logger.h"
#include "shift-core/serialize.h"
#include "shift-core/solvers/solver.h"
#include "shift-core/units.h"

namespace Shift {

/*!
\brief Shift Assembly datastructure
*/
class ModelParameters
  : public Base
  , public Serialize {

public:
  ModelParameters(std::string name)
    : Base(name) {
    gravity_ = ksl_vec3(0.0, 0.0, 0.0);
    addKeys();
  }
  ModelParameters()
    : Base("") {
    gravity_ = ksl_vec3(0.0, 0.0, 0.0);
    addKeys();
  }
  ~ModelParameters() {}

  /*! \brief retrieve gravity vector */
  const ksl_vec3_t& getGravity();

  /*! \brief set gravity vector */
  void setGravity(ksl_vec3_t);

  /*! \brief set gravity vector */
  void setGravity(double, double, double);

  void enableContactDetection(bool);

  void readJSON(Json::Value&);

  void writeJSON(Json::Value&);

  void print();

  void appendSolver(Solver::SolverBase*);
  void appendIntegrator(Integrator::IntegratorBase*);
  void appendLogger(Logger::LoggerBase*);

private:
  void addKeys();

  std::string version_;
  std::vector<std::string> include_directories_;
  ksl_vec3_t gravity_;

  bool contact_detection_enabled_ = false;
  ksl_vec3_t viewport_background_color_ = {{1.0, 1.0, 1.0}};

  std::vector<Solver::SolverBase*> solvers_;
  std::vector<Integrator::IntegratorBase*> integrators_;
  std::vector<Logger::LoggerBase*> loggers_;
  Units units_;
};

} // namespace Shift

#endif
